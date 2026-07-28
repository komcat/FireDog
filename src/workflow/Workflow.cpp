// Workflow.cpp
#include "workflow/Workflow.h"

#include "hardware/HardwareManager.h"
#include "hardware/HardwareConfig.h"
#include "hardware/io/EziIoDevice.h"
#include "hardware/io/WagoIoDevice.h"
#include "hardware/motion/PIHexapod.h"
#include "hardware/motion/ACSController.h"
#include "core/Topics.h"
#include "core/Log.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

namespace firedog {
namespace {
double NowSec()
{
    return std::chrono::duration<double>(
               std::chrono::steady_clock::now().time_since_epoch()).count();
}
// Presses closer together than this accumulate onto one target; a longer gap
// starts a fresh burst re-seeded from the live position.
constexpr double kJogCoalesceSec = 0.4;

// Velocity guardrail, enforced for EVERY caller (UI, recipes, programs) — the
// UI's clamp is convenience, this is the hard limit per controller type.
constexpr double kMinVelMMs    = 0.001;
constexpr double kMaxVelPIMMs  = 10.0;   // hexapods
constexpr double kMaxVelACSMMs = 40.0;   // gantry
} // namespace

Workflow::Workflow(HardwareManager& hw) : hw_(hw) {}

Workflow::~Workflow()
{
    // The safety watch thread calls back into this object (StopAllMotion) —
    // make sure it is fully stopped before any member is torn down.
    if (safetyWatch_) safetyWatch_->Stop();
}

void Workflow::Start(const HardwareConfig& hw)
{
    StartMotion(hw);
    StartIO(hw);
    StartPneumatics(hw);   // after StartIO — resolves pins on the IO fleet
    StartIoWatch(hw);      // after StartIO — same pin resolution
    StartSafetyWatch(hw);  // after StartIO — first-priority safety signals
    StartCamera(hw);
    StartKeithley(hw);
}

void Workflow::StartMotion(const HardwareConfig& hw)
{
    const std::vector<MotionConfig> devs = hw.EnabledMotionDevices();
    if (devs.empty()) {
        FD_WARN("workflow", "no enabled motion devices in config");
        return;
    }

    // Build each driver (PI hexapod or ACS gantry) and hand ownership to layer 1,
    // then connect them concurrently — each device on its own thread.
    for (const MotionConfig& d : devs) {
        std::unique_ptr<IMotionController> dev;
        if (d.controllerType == "PI") {
            dev = std::make_unique<PIHexapod>(d);
        } else if (d.controllerType == "ACS") {
            dev = std::make_unique<ACSController>(d);
        } else {
            FD_WARN("workflow", "unknown controller_type '%s' for '%s' — skipping",
                    d.controllerType.c_str(), d.name.c_str());
            continue;
        }

        const size_t idx = hw_.MotionCount();   // index this device will occupy
        hw_.AddMotion(std::move(dev));
        hw_.SetMotionState(idx, ConnState::Connecting);
        FD_INFO("workflow", "connecting %s '%s' @ %s:%d",
                d.controllerType.c_str(), d.name.c_str(), d.ipAddress.c_str(), d.port);

        const std::string name = d.name;
        const std::string type = d.controllerType;
        motionConnect_.push_back(std::async(std::launch::async, [this, idx, name, type] {
            IMotionController* m = hw_.MotionDevice(idx);
            const bool ok = m && m->Connect();
            hw_.SetMotionState(idx, ok ? ConnState::Connected : ConnState::Failed);
            if (ok) FD_INFO("workflow", "%s '%s' online", type.c_str(), name.c_str());
            else    FD_ERROR("workflow", "%s '%s' failed: %s", type.c_str(), name.c_str(),
                             m ? m->LastError().c_str() : "no driver");
        }));
    }
}

void Workflow::StartIO(const HardwareConfig& hw)
{
    const std::vector<IoDeviceConfig> devs = hw.EnabledIoDevices();
    if (devs.empty()) {
        FD_INFO("workflow", "no enabled IO devices in config");
        return;
    }

    // Build each driver (Ezi-IO board or WAGO coupler) and hand ownership to
    // layer 1, then connect + start polling concurrently, one thread each.
    std::vector<int> eziIds;
    for (const IoDeviceConfig& d : devs) {
        std::unique_ptr<IIoDevice> dev;
        if (d.type == "eziio") {
            // FAS_* calls address boards by device_id — duplicates would alias.
            if (std::find(eziIds.begin(), eziIds.end(), d.deviceId) != eziIds.end()) {
                FD_WARN("workflow", "duplicate eziio device_id %d for '%s' — skipping",
                        d.deviceId, d.name.c_str());
                continue;
            }
            eziIds.push_back(d.deviceId);
            EziIoDevice::Config cfg;
            cfg.name        = d.name;
            cfg.ipAddress   = d.ipAddress;
            cfg.deviceId    = d.deviceId;
            cfg.inputCount  = d.inputCount;
            cfg.outputCount = d.outputCount;
            cfg.inputNames  = d.inputNames;
            cfg.outputNames = d.outputNames;
            dev = std::make_unique<EziIoDevice>(cfg);
        } else if (d.type == "wago") {
            WagoIoDevice::Config cfg;
            cfg.name        = d.name;
            cfg.ipAddress   = d.ipAddress;
            cfg.port        = d.port;
            cfg.inputCount  = d.inputCount;
            cfg.outputCount = d.outputCount;
            cfg.simulated   = d.simulated;
            cfg.inputNames  = d.inputNames;
            cfg.outputNames = d.outputNames;
            dev = std::make_unique<WagoIoDevice>(cfg);
        } else {
            FD_WARN("workflow", "unknown io type '%s' for '%s' — skipping",
                    d.type.c_str(), d.name.c_str());
            continue;
        }

        const size_t idx = hw_.IoCount();   // index this device will occupy
        hw_.AddIo(std::move(dev));
        hw_.SetIoState(idx, ConnState::Connecting);
        FD_INFO("workflow", "connecting IO %s '%s' @ %s",
                d.type.c_str(), d.name.c_str(), d.ipAddress.c_str());

        const std::string name     = d.name;
        const std::string type     = d.type;
        const int         interval = d.pollIntervalMs;
        ioConnect_.push_back(std::async(std::launch::async, [this, idx, name, type, interval] {
            IIoDevice* io = hw_.IoDevice(idx);
            bool ok = io && io->Connect();
            if (ok) ok = io->StartPolling(interval, topics::IoInputs(name), topics::IoOutputs(name));
            hw_.SetIoState(idx, ok ? ConnState::Connected : ConnState::Failed);
            if (ok) FD_INFO("workflow", "IO %s '%s' online", type.c_str(), name.c_str());
            else    FD_ERROR("workflow", "IO %s '%s' failed: %s", type.c_str(), name.c_str(),
                             io ? io->LastError().c_str() : "no driver");
        }));
    }
}

void Workflow::StartPneumatics(const HardwareConfig& hw)
{
    const std::vector<PneumaticSlideConfig> slides = hw.EnabledPneumaticSlides();
    if (slides.empty()) return;

    auto findIo = [this](const std::string& name) -> IIoDevice* {
        for (size_t i = 0; i < hw_.IoCount(); ++i) {
            if (hw_.IoName(i) == name) return hw_.IoDevice(i);
        }
        return nullptr;
    };

    // The IO device objects exist as soon as StartIO returns (only their
    // connects are async), so name/pin resolution here is synchronous.
    for (const PneumaticSlideConfig& c : slides) {
        PneumaticSlide::Wiring w;
        w.name         = c.name;
        w.timeoutMs    = static_cast<uint32_t>(c.timeoutMs > 0 ? c.timeoutMs : 10000);
        w.outputDev    = findIo(c.outputDevice);
        w.extendedDev  = findIo(c.extendedDevice);
        w.retractedDev = findIo(c.retractedDevice);
        w.outputPin    = w.outputDev    ? w.outputDev->OutputPinByName(c.outputPin)      : -1;
        w.extendedPin  = w.extendedDev  ? w.extendedDev->InputPinByName(c.extendedPin)   : -1;
        w.retractedPin = w.retractedDev ? w.retractedDev->InputPinByName(c.retractedPin) : -1;

        if (!w.outputDev || w.outputPin < 0) {
            FD_WARN("workflow", "pneumatic '%s': output pin '%s' not found on '%s' — skipping",
                    c.name.c_str(), c.outputPin.c_str(), c.outputDevice.c_str());
            continue;
        }
        if (!w.extendedDev || w.extendedPin < 0 || !w.retractedDev || w.retractedPin < 0) {
            FD_WARN("workflow", "pneumatic '%s': sensor pin '%s'@'%s' or '%s'@'%s' not found — skipping",
                    c.name.c_str(), c.extendedPin.c_str(), c.extendedDevice.c_str(),
                    c.retractedPin.c_str(), c.retractedDevice.c_str());
            continue;
        }

        FD_INFO("workflow", "pneumatic '%s': out %s/%d, ext %s/%d, ret %s/%d, timeout %u ms",
                c.name.c_str(), c.outputDevice.c_str(), w.outputPin,
                c.extendedDevice.c_str(), w.extendedPin,
                c.retractedDevice.c_str(), w.retractedPin, w.timeoutMs);
        hw_.AddSlide(std::make_unique<PneumaticSlide>(std::move(w)));
    }
}

void Workflow::StartIoWatch(const HardwareConfig& hw)
{
    ApplyIoWatch(hw.ioWatch);
}

bool Workflow::ApplyIoWatch(const IoWatchConfig& cfg)
{
    hw_.SetIoWatch(nullptr);   // stop + drop any running watch first
    if (!cfg.enabled || cfg.bindings.empty()) {
        FD_INFO("workflow", "io_watch disabled");
        return true;
    }

    auto findIo = [this](const std::string& name) -> IIoDevice* {
        for (size_t i = 0; i < hw_.IoCount(); ++i) {
            if (hw_.IoName(i) == name) return hw_.IoDevice(i);
        }
        return nullptr;
    };

    std::vector<IoWatch::Binding> bindings;
    for (const IoWatchBindingConfig& c : cfg.bindings) {
        if (!c.enabled) continue;
        IoWatch::Binding b;
        b.name       = c.name;
        b.trigger    = IoWatch::ParseEdge(c.trigger);
        b.behavior   = IoWatch::ParseBehavior(c.behavior);
        b.debounceMs = c.debounceMs > 0 ? c.debounceMs : 100;
        b.inputDev   = findIo(c.inputDevice);
        b.outputDev  = findIo(c.outputDevice);
        b.inputPin   = b.inputDev  ? b.inputDev->InputPinByName(c.inputPin)    : -1;
        b.outputPin  = b.outputDev ? b.outputDev->OutputPinByName(c.outputPin) : -1;

        if (!b.inputDev || b.inputPin < 0 || !b.outputDev || b.outputPin < 0) {
            FD_WARN("workflow", "io_watch '%s': unresolved pin ('%s'@'%s' -> '%s'@'%s') — skipping",
                    c.name.c_str(), c.inputPin.c_str(), c.inputDevice.c_str(),
                    c.outputPin.c_str(), c.outputDevice.c_str());
            continue;
        }
        FD_INFO("workflow", "io_watch '%s': %s %s/%d -> %s %s/%d (debounce %d ms)",
                c.name.c_str(), IoWatch::EdgeName(b.trigger),
                c.inputDevice.c_str(), b.inputPin, IoWatch::BehaviorName(b.behavior),
                c.outputDevice.c_str(), b.outputPin, b.debounceMs);
        bindings.push_back(std::move(b));
    }
    if (bindings.empty()) {
        FD_WARN("workflow", "io_watch enabled but no usable bindings");
        return false;
    }

    auto watch = std::make_unique<IoWatch>(std::move(bindings), cfg.pollingIntervalMs);
    watch->Start();
    hw_.SetIoWatch(std::move(watch));
    return true;
}

void Workflow::StartSafetyWatch(const HardwareConfig& hw)
{
    if (!hw.safetyWatch.enabled || hw.safetyWatch.watchedInputs.empty()) return;

    auto findIo = [this](const std::string& name) -> IIoDevice* {
        for (size_t i = 0; i < hw_.IoCount(); ++i) {
            if (hw_.IoName(i) == name) return hw_.IoDevice(i);
        }
        return nullptr;
    };

    std::vector<SafetyWatch::Signal> signals;
    for (const SafetySignalConfig& c : hw.safetyWatch.watchedInputs) {
        if (!c.enabled) continue;
        SafetyWatch::Signal s;
        s.device       = c.device;
        s.pinName      = c.pinName;
        s.trigger      = IoWatch::ParseEdge(c.trigger);
        s.alertTitle   = c.alertTitle;
        s.alertMessage = c.alertMessage;
        s.critical     = c.critical;
        s.dev = findIo(c.device);
        s.pin = s.dev ? s.dev->InputPinByName(c.pinName) : -1;
        if (s.dev && s.pin < 0 && c.pinNumber >= 0 && c.pinNumber < s.dev->InputCount()) {
            FD_WARN("workflow", "safety '%s'@'%s': name unresolved, using pin_number %d",
                    c.pinName.c_str(), c.device.c_str(), c.pinNumber);
            s.pin = c.pinNumber;
        }
        if (!s.dev || s.pin < 0) {
            FD_ERROR("workflow", "safety signal '%s'@'%s' UNRESOLVED — NOT monitored",
                     c.pinName.c_str(), c.device.c_str());
            continue;
        }
        FD_INFO("workflow", "safety signal %s/%d (%s, %s): \"%s\"",
                c.device.c_str(), s.pin, IoWatch::EdgeName(s.trigger),
                c.critical ? "CRITICAL" : "non-critical", c.alertTitle.c_str());
        signals.push_back(std::move(s));
    }
    if (signals.empty()) {
        FD_ERROR("workflow", "safety_watch enabled but NO signals wired");
        return;
    }

    safetyWatch_ = std::make_unique<SafetyWatch>(
        std::move(signals), hw.safetyWatch.pollingIntervalMs,
        [this](const SafetyWatch::Alert& a) {
            // Watch thread: critical alerts hard-stop every motion device NOW.
            if (a.critical) StopAllMotion();
        });
    safetyWatch_->Start();
}

void Workflow::StopAllMotion()
{
    for (size_t i = 0; i < hw_.MotionCount(); ++i) {
        IMotionController* m = hw_.MotionDevice(i);
        if (m) {
            FD_ERROR("safety", "STOP %s", m->Name().c_str());
            m->StopAll();
        }
    }
}

void Workflow::StartCamera(const HardwareConfig& hw)
{
    const CameraConfig* cam = hw.FirstEnabledCamera();
    if (!cam) {
        FD_WARN("workflow", "no enabled camera in config");
        hw_.SetCameraState(ConnState::Disconnected);
        return;
    }

    const std::string name = cam->displayName.empty() ? cam->id : cam->displayName;
    hw_.SetCameraName(name);
    hw_.SetCameraState(ConnState::Connecting);

    const std::string ip = cam->ipAddress;
    const bool byIp = (cam->connectionType == "ip") && !ip.empty();
    FD_INFO("workflow", "connecting camera '%s' (%s)", name.c_str(),
            byIp ? ip.c_str() : "first available");

    camConnect_ = std::async(std::launch::async, [this, ip, byIp, name] {
        PylonCamera& camera = hw_.Camera();
        const bool ok = (byIp ? camera.ConnectByIP(ip) : camera.ConnectFirst())
                        && camera.StartGrabbing();
        hw_.SetCameraState(ok ? ConnState::Connected : ConnState::Failed);
        if (ok) FD_INFO("workflow", "camera '%s' grabbing", name.c_str());
        else    FD_ERROR("workflow", "camera '%s' failed to start", name.c_str());
    });
}

void Workflow::StartKeithley(const HardwareConfig& hw)
{
    const KeithleyConfig* k = hw.FirstEnabledKeithley();
    if (!k) {
        FD_WARN("workflow", "no enabled Keithley in config");
        hw_.SetKeithleyState(ConnState::Disconnected);
        return;
    }

    hw_.SetKeithleyName(k->name);
    hw_.SetKeithleyState(ConnState::Connecting);
    FD_INFO("workflow", "connecting Keithley '%s' (%s)", k->name.c_str(), k->visaResource.c_str());

    Keithley6482::Config cfg;
    cfg.name            = k->name;
    cfg.visaResource    = k->visaResource;
    cfg.channelCount    = k->channelCount;
    cfg.enabledChannels = k->enabledChannels;
    cfg.nplc            = k->nplc;
    cfg.autoRange       = k->autoRange;
    cfg.fixedRange      = k->fixedRange;
    hw_.SetKeithley(std::make_unique<Keithley6482>(cfg));

    const int interval = k->pollIntervalMs;
    keiConnect_ = std::async(std::launch::async, [this, interval] {
        Keithley6482* kei = hw_.Keithley();
        bool ok = kei && kei->Connect();
        if (ok) ok = kei->SetVoltageBias(1.0);   // ch1 bias = 1 V (alignment policy)
        if (ok) ok = kei->StartPolling(interval, topics::kKeithleyCh1, topics::kKeithleyCh2);
        hw_.SetKeithleyState(ok ? ConnState::Connected : ConnState::Failed);
        if (!ok) FD_ERROR("workflow", "Keithley bring-up failed: %s",
                          kei ? kei->LastError().c_str() : "no driver");
    });
}

void Workflow::Update(float dt)
{
    clock_ += dt;
    // Placeholder measurement: a noisy sinusoid standing in for the
    // photocurrent read during alignment. Replace with a real device read.
    sample_ = std::sin(clock_ * 2.0f) * 0.5f + 0.5f
            + 0.05f * std::sin(clock_ * 37.0f);

    // Tick the pneumatic state machines every frame (reads the IO drivers'
    // caches only) so movement completion and timeouts fire regardless of
    // which page is visible.
    for (size_t i = 0; i < hw_.SlideCount(); ++i) {
        hw_.Slide(i)->Update();
    }
}

const char* Workflow::StatusText() const
{
    return hw_.AllConnected() ? "ready" : "connecting hardware...";
}

bool Workflow::MoveGantryRelativeMM(double dxMM, double dyMM)
{
    if (SafetyBlocked()) {
        FD_WARN("safety", "move blocked — acknowledge the safety alert first");
        return false;
    }
    // Locate gantry-main (fall back to the first ACS controller).
    IMotionController* g = nullptr;
    for (size_t i = 0; i < hw_.MotionCount(); ++i) {
        IMotionController* m = hw_.MotionDevice(i);
        if (m && m->Type() == "ACS" && m->Name() == "gantry-main") { g = m; break; }
    }
    if (!g) {
        for (size_t i = 0; i < hw_.MotionCount(); ++i) {
            IMotionController* m = hw_.MotionDevice(i);
            if (m && m->Type() == "ACS") { g = m; break; }
        }
    }
    if (!g) { FD_WARN("workflow", "click-to-move: no ACS gantry available"); return false; }

    // Relative move = current position + requested offset, X and Y together.
    const std::map<std::string, double> pos = g->Positions();
    const auto itx = pos.find("X");
    const auto ity = pos.find("Y");
    const double x = (itx != pos.end()) ? itx->second : 0.0;
    const double y = (ity != pos.end()) ? ity->second : 0.0;

    const std::vector<std::pair<std::string, double>> tgt = {
        {"X", x + dxMM}, {"Y", y + dyMM}
    };
    FD_INFO("workflow", "%s move dXY=(%+.4f, %+.4f) mm -> XY=(%.4f, %.4f)",
            g->Name().c_str(), dxMM, dyMM, x + dxMM, y + dyMM);
    return g->MoveAbsolute(tgt);
}

void Workflow::SetTransforms(std::map<std::string, AxisTransform> t)
{
    transforms_ = std::move(t);
}

bool Workflow::SetJogVelocity(std::size_t devIdx, double mmPerSec)
{
    if (devIdx >= hw_.MotionCount()) return false;
    IMotionController* m = hw_.MotionDevice(devIdx);
    if (!m) return false;

    const double maxVel = (m->Type() == "ACS") ? kMaxVelACSMMs : kMaxVelPIMMs;
    double v = mmPerSec;
    if (v < kMinVelMMs) v = kMinVelMMs;
    if (v > maxVel) {
        FD_WARN("workflow", "%s velocity %.3f mm/s over limit — clamped to %.1f",
                m->Name().c_str(), mmPerSec, maxVel);
        v = maxVel;
    }
    return m->SetVelocity(v);
}

bool Workflow::JogGlobal(std::size_t devIdx, float gx, float gy, float gz, double stepMM)
{
    if (devIdx >= hw_.MotionCount()) return false;
    IMotionController* m = hw_.MotionDevice(devIdx);
    if (!m) return false;

    // Map the global jog direction to a signed local axis via the device transform.
    AxisTransform t;   // identity if none configured
    const auto it = transforms_.find(m->Name());
    if (it != transforms_.end()) t = it->second;
    const double g[3] = {gx, gy, gz};
    double loc[3];
    for (int i = 0; i < 3; ++i)
        loc[i] = t.m[i][0] * g[0] + t.m[i][1] * g[1] + t.m[i][2] * g[2];
    int ax = 0;
    for (int i = 1; i < 3; ++i) if (std::fabs(loc[i]) > std::fabs(loc[ax])) ax = i;

    const std::vector<std::string> axes = m->Axes();
    if (ax >= static_cast<int>(axes.size())) return false;
    const double delta = stepMM * (loc[ax] >= 0.0 ? 1.0 : -1.0);
    return JogLocalAxis(devIdx, axes[ax], delta);
}

bool Workflow::JogLocalAxis(std::size_t devIdx, const std::string& axis, double deltaMM)
{
    if (SafetyBlocked()) {
        FD_WARN("safety", "jog blocked — acknowledge the safety alert first");
        return false;
    }
    if (devIdx >= hw_.MotionCount()) return false;
    IMotionController* m = hw_.MotionDevice(devIdx);
    if (!m) return false;

    // Accumulate an evolving target so a new press OVERRIDES (extends) the
    // previous target rather than queuing a move read from lagging feedback.
    // After an idle gap, re-seed from the live position (a fresh burst).
    const double      now = NowSec();
    const std::string key = std::to_string(devIdx) + "/" + axis;
    JogTrack&         jt  = jogTracks_[key];
    if (now - jt.lastT > kJogCoalesceSec) {
        const std::map<std::string, double> pos = m->Positions();
        const auto it = pos.find(axis);
        jt.target = (it != pos.end()) ? it->second : 0.0;
    }
    jt.target += deltaMM;
    jt.lastT   = now;

    FD_INFO("jog", "%s: %s %+.4f mm -> %.4f", m->Name().c_str(), axis.c_str(), deltaMM, jt.target);
    return m->MoveAbsolute(axis, jt.target);   // driver retargets (PI_MOV / acsc_ToPoint)
}

bool Workflow::StopMotion(std::size_t devIdx)
{
    if (devIdx >= hw_.MotionCount()) return false;
    IMotionController* m = hw_.MotionDevice(devIdx);
    if (!m) return false;
    FD_WARN("workflow", "STOP %s", m->Name().c_str());
    return m->StopAll();
}

bool Workflow::MoveToPosition(std::size_t devIdx,
                              const std::vector<std::pair<std::string, double>>& targets)
{
    if (SafetyBlocked()) {
        FD_WARN("safety", "move blocked — acknowledge the safety alert first");
        return false;
    }
    if (devIdx >= hw_.MotionCount() || targets.empty()) return false;
    IMotionController* m = hw_.MotionDevice(devIdx);
    return m && m->MoveAbsolute(targets);   // coordinated (PI_MOV / acsc_ToPointM+GoM)
}

bool Workflow::SetIoOutput(std::size_t devIdx, int pin, bool on)
{
    if (devIdx >= hw_.IoCount()) return false;
    if (hw_.IoState(devIdx) != ConnState::Connected) return false;
    IIoDevice* d = hw_.IoDevice(devIdx);
    if (!d) return false;

    const std::string pinName = d->OutputPinName(pin);
    FD_INFO("io", "%s: output %d%s%s%s -> %s", d->Name().c_str(), pin,
            pinName.empty() ? "" : " (", pinName.c_str(), pinName.empty() ? "" : ")",
            on ? "ON" : "OFF");
    return d->SetOutput(pin, on);
}

bool Workflow::SetIoOutputByName(const std::string& device, const std::string& pinName, bool on)
{
    for (size_t i = 0; i < hw_.IoCount(); ++i) {
        if (hw_.IoName(i) != device) continue;
        IIoDevice* d = hw_.IoDevice(i);
        const int pin = d ? d->OutputPinByName(pinName) : -1;
        if (pin < 0) {
            FD_WARN("io", "%s: unknown output pin '%s'", device.c_str(), pinName.c_str());
            return false;
        }
        return SetIoOutput(i, pin, on);
    }
    FD_WARN("io", "unknown IO device '%s'", device.c_str());
    return false;
}

bool Workflow::ClearIoLatch(std::size_t devIdx, uint64_t mask)
{
    if (devIdx >= hw_.IoCount()) return false;
    if (hw_.IoState(devIdx) != ConnState::Connected) return false;
    IIoDevice* d = hw_.IoDevice(devIdx);
    return d && d->ClearLatch(mask);
}

bool Workflow::ExtendSlide(std::size_t idx)
{
    if (idx >= hw_.SlideCount()) return false;
    return hw_.Slide(idx)->Extend();
}

bool Workflow::RetractSlide(std::size_t idx)
{
    if (idx >= hw_.SlideCount()) return false;
    return hw_.Slide(idx)->Retract();
}

bool Workflow::StopSlide(std::size_t idx)
{
    if (idx >= hw_.SlideCount()) return false;
    return hw_.Slide(idx)->Stop();
}

} // namespace firedog
