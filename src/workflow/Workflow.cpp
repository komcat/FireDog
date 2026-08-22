// Workflow.cpp
#include "workflow/Workflow.h"

#include "hardware/HardwareManager.h"
#include "hardware/HardwareConfig.h"
#include "hardware/instruments/Keithley2602B.h"
#include "hardware/instruments/Keithley6482.h"
#include "hardware/io/EziIoDevice.h"
#include "hardware/io/WagoIoDevice.h"
#include "hardware/motion/PIHexapod.h"
#include "hardware/motion/ACSController.h"
#include "core/DataBus.h"
#include "core/Topics.h"
#include "core/Log.h"
#include "vision/FocusMetric.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <memory>
#include <string>
#include <system_error>

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

// A device that has been registered with layer 1 and still needs its connect
// thread. Shared by StartMotion and StartIO, which both build the whole fleet
// before launching any of them — see the note in StartMotion.
struct Pending {
    size_t      idx;        // slot this device occupies in the fleet
    std::string name;
    std::string type;
    int         interval;   // IO poll interval in ms; unused for motion
};

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
    StartCld(hw);
    StartPsu(hw);
    StartNeedleCal();
}

// The needle lever arm is read from disk, not from the hardware config: it is
// measured on the machine and rewritten by a calibration run, so it follows the
// xxx.local.json -> xxx.json rule rather than living in hardware.json.
void Workflow::StartNeedleCal()
{
    std::string path;
    const NeedleCal c = LoadNeedleCal(&path);
    {
        const std::lock_guard<std::mutex> lock(needleMu_);
        needle_ = c;
    }
    // Say which of the two files answered. "The dispenser is 1.4 mm out" and
    // "the machine is running on the shipped seed" look identical at the needle,
    // and only the log can tell them apart.
    if (c.valid)
        FD_INFO("needle", "needle offset (%.4f, %.4f, %.4f) mm from %s, calibrated %s",
                c.dx, c.dy, c.dz, path.c_str(),
                c.calibratedAt.empty() ? "(undated)" : c.calibratedAt.c_str());
    else
        FD_WARN("needle", "no needle calibration (%s) - dispensing will refuse to "
                          "run until the needle calibration procedure has been done",
                path.empty() ? "no file" : path.c_str());
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
    //
    // TWO passes, and the split is load-bearing: EVERY device is registered
    // before ANY connect thread starts. AddMotion pushes into a vector, and a
    // push_back that reallocates while a worker is reading motion_[idx] hands
    // that worker a moved-from slot — which surfaces as a null dereference deep
    // inside HardwareManager::MotionDevice. It needs two configured devices and
    // a lost race, so as one interleaved loop it crashed startup now and then
    // and looked like a driver bug.
    std::vector<Pending> pending;
    for (const MotionConfig& d : devs) {
        std::unique_ptr<IMotionController> dev;
        if (d.controllerType == "PI") {
            dev = std::make_unique<PIHexapod>(d);
        } else if (d.controllerType == "ACS") {
            dev = std::make_unique<ACSController>(d);
        } else {
            FD_WARN("workflow", "unknown controller_type '%s' for '%s' - skipping",
                    d.controllerType.c_str(), d.name.c_str());
            continue;
        }

        const size_t idx = hw_.MotionCount();   // index this device will occupy
        hw_.AddMotion(std::move(dev));
        hw_.SetMotionState(idx, ConnState::Connecting);
        FD_INFO("workflow", "connecting %s '%s' @ %s:%d",
                d.controllerType.c_str(), d.name.c_str(), d.ipAddress.c_str(), d.port);
        pending.push_back({idx, d.name, d.controllerType, 0});
    }

    // The fleet is now fixed for the life of the process — nothing below adds
    // to it, so indexing it off-thread is safe.
    for (const Pending& p : pending) {
        const size_t      idx  = p.idx;
        const std::string name = p.name;
        const std::string type = p.type;
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
    // layer 1, then connect + start polling concurrently, one thread each —
    // in two passes, like StartMotion.
    std::vector<Pending> pending;
    std::vector<int> eziIds;
    for (const IoDeviceConfig& d : devs) {
        std::unique_ptr<IIoDevice> dev;
        if (d.type == "eziio") {
            // FAS_* calls address boards by device_id — duplicates would alias.
            if (std::find(eziIds.begin(), eziIds.end(), d.deviceId) != eziIds.end()) {
                FD_WARN("workflow", "duplicate eziio device_id %d for '%s' - skipping",
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
            FD_WARN("workflow", "unknown io type '%s' for '%s' - skipping",
                    d.type.c_str(), d.name.c_str());
            continue;
        }

        const size_t idx = hw_.IoCount();   // index this device will occupy
        hw_.AddIo(std::move(dev));
        hw_.SetIoState(idx, ConnState::Connecting);
        FD_INFO("workflow", "connecting IO %s '%s' @ %s",
                d.type.c_str(), d.name.c_str(), d.ipAddress.c_str());
        pending.push_back({idx, d.name, d.type, d.pollIntervalMs});
    }

    // Second pass, for the same reason StartMotion has one: AddIo grows io_,
    // and a worker indexing that vector while it reallocates reads a
    // moved-from slot.
    for (const Pending& p : pending) {
        const size_t      idx      = p.idx;
        const std::string name     = p.name;
        const std::string type     = p.type;
        const int         interval = p.interval;
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
            FD_WARN("workflow", "pneumatic '%s': output pin '%s' not found on '%s' - skipping",
                    c.name.c_str(), c.outputPin.c_str(), c.outputDevice.c_str());
            continue;
        }
        if (!w.extendedDev || w.extendedPin < 0 || !w.retractedDev || w.retractedPin < 0) {
            FD_WARN("workflow", "pneumatic '%s': sensor pin '%s'@'%s' or '%s'@'%s' not found - skipping",
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
            FD_WARN("workflow", "io_watch '%s': unresolved pin ('%s'@'%s' -> '%s'@'%s') - skipping",
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
            FD_ERROR("workflow", "safety signal '%s'@'%s' UNRESOLVED - NOT monitored",
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
    // May run on the safety-watch thread: request path abandon via the atomic;
    // TickPath honors it on the next frame.
    pathCancelReq_.store(true);
    for (size_t i = 0; i < hw_.MotionCount(); ++i) {
        IMotionController* m = hw_.MotionDevice(i);
        if (m) {
            FD_ERROR("safety", "STOP %s", m->Name().c_str());
            m->StopAll();
        }
    }
}

void Workflow::QueueSavedPosition(const std::string& device, const SavedPos& p)
{
    const std::lock_guard<std::mutex> lock(teachMu_);
    teachQueue_.emplace_back(device, p);
}

std::vector<std::pair<std::string, SavedPos>> Workflow::TakeSavedPositions()
{
    const std::lock_guard<std::mutex> lock(teachMu_);
    return std::exchange(teachQueue_, {});
}

void Workflow::SetCapturedView(CapturedView v)
{
    const std::lock_guard<std::mutex> lock(shotMu_);
    shot_        = std::move(v);   // latest wins; the previous frame is dropped
    shotPending_ = true;
}

bool Workflow::TakeCapturedView(CapturedView* out)
{
    const std::lock_guard<std::mutex> lock(shotMu_);
    if (!shotPending_ || !out) return false;
    *out         = std::move(shot_);
    shot_        = CapturedView{};
    shotPending_ = false;
    return true;
}

void Workflow::QueueAlignResult(const std::string& uid, const AlignResult& r)
{
    const std::lock_guard<std::mutex> lock(alignMu_);
    alignQueue_.emplace_back(uid, r);
}

std::vector<std::pair<std::string, AlignResult>> Workflow::TakeAlignResults()
{
    const std::lock_guard<std::mutex> lock(alignMu_);
    return std::exchange(alignQueue_, {});
}

void Workflow::QueueVisionReliability(const std::string& uid, const VisionReliability& r)
{
    const std::lock_guard<std::mutex> lock(visionRelMu_);
    visionRelQueue_.emplace_back(uid, r);
}

std::vector<std::pair<std::string, VisionReliability>> Workflow::TakeVisionReliability()
{
    const std::lock_guard<std::mutex> lock(visionRelMu_);
    return std::exchange(visionRelQueue_, {});
}

void Workflow::SetLiveFrame(const std::string& alignUid, const AlignResult& r)
{
    const std::lock_guard<std::mutex> lock(frameMu_);
    liveFrames_[alignUid] = r;
}

bool Workflow::LiveFrame(const std::string& alignUid, AlignResult* out) const
{
    const std::lock_guard<std::mutex> lock(frameMu_);
    const auto it = liveFrames_.find(alignUid);
    if (it == liveFrames_.end() || !it->second.valid) return false;
    if (out) *out = it->second;
    return true;
}

void Workflow::ClearLiveFrames(const char* reason)
{
    std::size_t n = 0;
    {
        const std::lock_guard<std::mutex> lock(frameMu_);
        n = liveFrames_.size();
        liveFrames_.clear();
    }
    // Logged even at zero would be noise; logged when it MATTERS is the point —
    // "why did move_local start failing" has to be answerable from the log.
    if (n > 0)
        FD_INFO("align", "live taught frame(s) cleared: %s (%zu dropped) - "
                         "re-run the alignment before any move_local",
                reason ? reason : "reset", n);
}

std::size_t Workflow::LiveFrameCount() const
{
    const std::lock_guard<std::mutex> lock(frameMu_);
    return liveFrames_.size();
}

void Workflow::QueueAlignPoint(const std::string& uid, int station, const AlignPoint& p)
{
    const std::lock_guard<std::mutex> lock(alignPtMu_);
    alignPtQueue_.push_back(AlignPointMark{uid, station, p});
}

std::vector<Workflow::AlignPointMark> Workflow::TakeAlignPoints()
{
    const std::lock_guard<std::mutex> lock(alignPtMu_);
    return std::exchange(alignPtQueue_, {});
}

NeedleCal Workflow::Needle() const
{
    const std::lock_guard<std::mutex> lock(needleMu_);
    return needle_;
}

void Workflow::SetNeedleCal(const NeedleCal& c)
{
    const std::lock_guard<std::mutex> lock(needleMu_);
    needle_ = c;
}

void Workflow::QueueNeedleCal(const NeedleCal& c)
{
    const std::lock_guard<std::mutex> lock(needleMu_);
    needleQueue_.push_back(c);
}

std::vector<NeedleCal> Workflow::TakeNeedleCals()
{
    const std::lock_guard<std::mutex> lock(needleMu_);
    return std::exchange(needleQueue_, {});
}

bool Workflow::GantryXY(double& x, double& y) const
{
    const std::size_t gi = GantryIndex();
    if (gi >= hw_.MotionCount()) return false;
    IMotionController* m = hw_.MotionDevice(gi);
    if (!m || !m->IsConnected()) return false;
    const std::map<std::string, double> pos = m->Positions();
    const auto ix = pos.find("X"), iy = pos.find("Y");
    if (ix == pos.end() || iy == pos.end()) return false;
    x = ix->second;
    y = iy->second;
    return true;
}

bool Workflow::DevicePose(const std::string& name, std::map<std::string, double>& out) const
{
    for (std::size_t i = 0; i < hw_.MotionCount(); ++i) {
        if (hw_.MotionName(i) != name) continue;
        IMotionController* m = hw_.MotionDevice(i);
        if (!m || !m->IsConnected()) return false;
        out = m->Positions();
        return true;
    }
    return false;
}

void Workflow::StartCamera(const HardwareConfig& hw)
{
    const CameraConfig* cam = hw.FirstEnabledCamera();
    if (!cam) {
        FD_WARN("workflow", "no enabled camera in config");
        hw_.SetCameraState(ConnState::Disconnected);
        return;
    }

    camCfg_      = *cam;   // kept so the Camera page can reconnect later
    camCfgValid_ = true;
    hw_.SetCameraName(cam->displayName.empty() ? cam->id : cam->displayName);
    LaunchCameraConnect();
}

void Workflow::LaunchCameraConnect()
{
    hw_.SetCameraState(ConnState::Connecting);

    const std::string name = hw_.CameraName();
    const std::string ip   = camCfg_.ipAddress;
    const bool   byIp      = (camCfg_.connectionType == "ip") && !ip.empty();
    const double expoUs    = camCfg_.exposureUs;   // 0 = leave camera default
    FD_INFO("workflow", "connecting camera '%s' (%s)", name.c_str(),
            byIp ? ip.c_str() : "first available");

    camConnect_ = std::async(std::launch::async, [this, ip, byIp, name, expoUs] {
        PylonCamera& camera = hw_.Camera();
        const bool ok = (byIp ? camera.ConnectByIP(ip) : camera.ConnectFirst())
                        && camera.StartGrabbing();
        if (ok && expoUs > 0.0) camera.SetExposureUs(expoUs);
        hw_.SetCameraState(ok ? ConnState::Connected : ConnState::Failed);
        if (ok) FD_INFO("workflow", "camera '%s' grabbing", name.c_str());
        else    FD_ERROR("workflow", "camera '%s' failed to start", name.c_str());
    });
}

bool Workflow::ConnectCamera()
{
    if (!camCfgValid_) {
        FD_WARN("workflow", "camera connect: no camera configured");
        return false;
    }
    if (hw_.CameraState() == ConnState::Connecting) return false;   // in flight
    if (hw_.Camera().IsConnected()) return true;                    // already up
    LaunchCameraConnect();
    return true;
}

bool Workflow::DisconnectCamera()
{
    if (hw_.CameraState() == ConnState::Connecting) return false;   // let it finish
    // Quick: StopGrabbing joins the grab thread (<=50 ms timeout) + Close.
    hw_.Camera().Disconnect();
    hw_.SetCameraState(ConnState::Disconnected);
    FD_INFO("workflow", "camera '%s' disconnected", hw_.CameraName().c_str());
    return true;
}

bool Workflow::SetCameraExposure(double us)
{
    if (hw_.CameraState() != ConnState::Connected) return false;
    const bool ok = hw_.Camera().SetExposureUs(us);
    if (ok) FD_INFO("camera", "exposure set to %.0f us", us);
    else    FD_WARN("camera", "exposure set failed: %s",
                    hw_.Camera().LastError().c_str());
    return ok;
}

void Workflow::StartKeithley(const HardwareConfig& hw)
{
    const KeithleyConfig* k = hw.FirstEnabledKeithley();
    if (!k) {
        FD_WARN("workflow", "no enabled Keithley in config");
        hw_.SetKeithleyState(ConnState::Disconnected);
        return;
    }

    keiCfg_      = *k;   // kept so the Keithley page can reconnect later
    keiCfgValid_ = true;
    hw_.SetKeithleyName(k->name);
    LaunchKeithleyConnect();
}

void Workflow::LaunchKeithleyConnect()
{
    hw_.SetKeithleyState(ConnState::Connecting);
    FD_INFO("workflow", "connecting Keithley '%s' (%s)",
            keiCfg_.name.c_str(), keiCfg_.visaResource.c_str());

    SmuConfig cfg;
    cfg.name            = keiCfg_.name;
    cfg.type            = keiCfg_.type;
    cfg.visaResource    = keiCfg_.visaResource;
    cfg.channelCount    = keiCfg_.channelCount;
    cfg.enabledChannels = keiCfg_.enabledChannels;
    cfg.nplc            = keiCfg_.nplc;
    cfg.autoRange       = keiCfg_.autoRange;
    cfg.fixedRange      = keiCfg_.fixedRange;
    // The ONE place a concrete SMU is chosen. Everything above layer 1 works
    // through ISmu, so a third meter is a new driver plus a branch here.
    if (cfg.type.find("2602") != std::string::npos)
        hw_.SetKeithley(std::make_unique<Keithley2602B>(cfg));
    else
        hw_.SetKeithley(std::make_unique<Keithley6482>(cfg));

    const int    interval = keiCfg_.pollIntervalMs;
    const double bias     = keiCfg_.biasVoltage;   // ch1 alignment bias, from config
    keiConnect_ = std::async(std::launch::async, [this, interval, bias] {
        ISmu* kei = hw_.Keithley();
        bool ok = kei && kei->Connect();
        // Bias is verified against the instrument (:OUTP?); a refusal is
        // logged but doesn't abort bring-up — measurement works unbiased.
        if (ok && !kei->SetVoltageBias(bias))
            FD_WARN("workflow", "Keithley bias not confirmed (see keithley log) - polling anyway");
        if (ok) ok = kei->StartPolling(interval, topics::kKeithleyCh1, topics::kKeithleyCh2);
        hw_.SetKeithleyState(ok ? ConnState::Connected : ConnState::Failed);
        if (!ok) FD_ERROR("workflow", "Keithley bring-up failed: %s",
                          kei ? kei->LastError().c_str() : "no driver");
    });
}

bool Workflow::ConnectKeithley()
{
    if (!keiCfgValid_) {
        FD_WARN("workflow", "Keithley connect: no instrument configured");
        return false;
    }
    if (hw_.KeithleyState() == ConnState::Connecting) return false;   // in flight
    ISmu* kei = hw_.Keithley();
    if (kei && kei->IsConnected()) return true;                       // already up
    LaunchKeithleyConnect();   // rebuilds the driver from keiCfg_
    return true;
}

bool Workflow::DisconnectKeithley()
{
    if (hw_.KeithleyState() == ConnState::Connecting) return false;   // let it finish
    ISmu* kei = hw_.Keithley();
    if (!kei) return false;
    kei->Disconnect();   // stops polling and drops the output (both drivers)
    hw_.SetKeithleyState(ConnState::Disconnected);
    FD_INFO("workflow", "Keithley '%s' disconnected", hw_.KeithleyName().c_str());
    return true;
}

// The runtime-settings commands all require a live connection.
ISmu* Workflow::ConnectedKeithley() const
{
    if (hw_.KeithleyState() != ConnState::Connected) return nullptr;
    return hw_.Keithley();
}

bool Workflow::SetKeithleyBias(double volts)
{
    ISmu* kei = ConnectedKeithley();
    return kei && kei->SetVoltageBias(volts);
}

bool Workflow::KeithleyOutputOff()
{
    ISmu* kei = ConnectedKeithley();
    return kei && kei->OutputOff();
}

bool Workflow::SetKeithleyNplc(double nplc)
{
    ISmu* kei = ConnectedKeithley();
    return kei && kei->SetNplc(nplc);
}

bool Workflow::SetKeithleyRangeAuto()
{
    ISmu* kei = ConnectedKeithley();
    return kei && kei->SetRangeAuto();
}

bool Workflow::SetKeithleyRangeFixed(double amps)
{
    ISmu* kei = ConnectedKeithley();
    return kei && kei->SetRangeFixed(amps);
}

bool Workflow::SetKeithleyPollInterval(int ms)
{
    ISmu* kei = ConnectedKeithley();
    if (!kei) return false;
    kei->StopPolling();
    const bool ok = kei->StartPolling(ms, topics::kKeithleyCh1, topics::kKeithleyCh2);
    if (ok) keiCfg_.pollIntervalMs = ms;   // so a later reconnect keeps it
    return ok;
}

void Workflow::StartCld(const HardwareConfig& hw)
{
    const Cld101xConfig* c = hw.FirstEnabledCld();
    if (!c) {
        FD_WARN("workflow", "no enabled CLD101x in config");
        hw_.SetCldState(ConnState::Disconnected);
        return;
    }

    cldCfg_      = *c;   // kept so the CLD page can reconnect later
    cldCfgValid_ = true;
    hw_.SetCldName(c->name);
    LaunchCldConnect();
}

void Workflow::LaunchCldConnect()
{
    hw_.SetCldState(ConnState::Connecting);
    FD_INFO("workflow", "connecting CLD101x '%s' (%s)",
            cldCfg_.name.c_str(), cldCfg_.visaResource.c_str());

    CLD101x::Config cfg;
    cfg.name                  = cldCfg_.name;
    cfg.visaResource          = cldCfg_.visaResource;
    cfg.defaultTemperature    = cldCfg_.defaultTemperature;
    cfg.defaultLaserCurrentMA = cldCfg_.defaultLaserCurrentMA;
    hw_.SetCld(std::make_unique<CLD101x>(cfg));

    // Connect + poll only. No setpoints and no output switching at startup —
    // the device keeps whatever state it is in (bobcat behavior); enabling the
    // laser is always an explicit UI action.
    const int interval = cldCfg_.pollIntervalMs;
    cldConnect_ = std::async(std::launch::async, [this, interval] {
        CLD101x* cld = hw_.Cld();
        bool ok = cld && cld->Connect();
        if (ok) ok = cld->StartPolling(interval, topics::kCldTemp, topics::kCldCurrent);
        hw_.SetCldState(ok ? ConnState::Connected : ConnState::Failed);
        if (!ok) FD_ERROR("workflow", "CLD101x bring-up failed: %s",
                          cld ? cld->LastError().c_str() : "no driver");
    });
}

bool Workflow::ConnectCld()
{
    if (!cldCfgValid_) {
        FD_WARN("workflow", "CLD connect: no CLD101x configured");
        return false;
    }
    if (hw_.CldState() == ConnState::Connecting) return false;   // in flight
    CLD101x* cld = hw_.Cld();
    if (cld && cld->IsConnected()) return true;                  // already up
    LaunchCldConnect();   // rebuilds the driver from cldCfg_
    return true;
}

bool Workflow::DisconnectCld()
{
    if (hw_.CldState() == ConnState::Connecting) return false;   // let it finish
    CLD101x* cld = hw_.Cld();
    if (!cld) return false;
    cld->Disconnect();   // stops polling; device outputs keep their state
    hw_.SetCldState(ConnState::Disconnected);
    FD_INFO("workflow", "CLD101x '%s' disconnected", hw_.CldName().c_str());
    return true;
}

CLD101x* Workflow::ConnectedCld() const
{
    if (hw_.CldState() != ConnState::Connected) return nullptr;
    return hw_.Cld();
}

bool Workflow::SetCldTemperature(double celsius)
{
    CLD101x* cld = ConnectedCld();
    return cld && cld->SetTemperature(celsius);
}

bool Workflow::SetCldLaserCurrentMA(double mA)
{
    CLD101x* cld = ConnectedCld();
    return cld && cld->SetLaserCurrentMA(mA);
}

bool Workflow::CldTecOn()    { CLD101x* c = ConnectedCld(); return c && c->TecOn(); }
bool Workflow::CldTecOff()   { CLD101x* c = ConnectedCld(); return c && c->TecOff(); }
bool Workflow::CldLaserOn()  { CLD101x* c = ConnectedCld(); return c && c->LaserOn(); }
bool Workflow::CldLaserOff() { CLD101x* c = ConnectedCld(); return c && c->LaserOff(); }

void Workflow::StartPsu(const HardwareConfig& hw)
{
    const PsuConfig* p = hw.FirstEnabledPsu();
    if (!p) {
        FD_WARN("workflow", "no enabled PSU in config");
        hw_.SetPsuState(ConnState::Disconnected);
        return;
    }

    psuCfg_      = *p;   // kept so the PSU page can reconnect later
    psuCfgValid_ = true;
    hw_.SetPsuName(p->name);
    LaunchPsuConnect();
}

void Workflow::LaunchPsuConnect()
{
    hw_.SetPsuState(ConnState::Connecting);
    FD_INFO("workflow", "connecting PSU '%s' (%s)",
            psuCfg_.name.c_str(), psuCfg_.visaResource.c_str());

    UDP3305S::Config cfg;
    cfg.name           = psuCfg_.name;
    cfg.visaResource   = psuCfg_.visaResource;
    cfg.channels       = psuCfg_.channels;
    cfg.pollIntervalMs = psuCfg_.pollIntervalMs;
    hw_.SetPsu(std::make_unique<UDP3305S>(cfg));

    // Connect + poll only. Nothing is written to the outputs at startup —
    // enabling an output is always an explicit UI action.
    const int interval = psuCfg_.pollIntervalMs;
    psuConnect_ = std::async(std::launch::async, [this, interval] {
        UDP3305S* psu = hw_.Psu();
        bool ok = psu && psu->Connect();
        if (ok) ok = psu->StartPolling(interval);
        hw_.SetPsuState(ok ? ConnState::Connected : ConnState::Failed);
        if (!ok) FD_ERROR("workflow", "PSU bring-up failed: %s",
                          psu ? psu->LastError().c_str() : "no driver");
    });
}

bool Workflow::ConnectPsu()
{
    if (!psuCfgValid_) {
        FD_WARN("workflow", "PSU connect: no PSU configured");
        return false;
    }
    if (hw_.PsuState() == ConnState::Connecting) return false;   // in flight
    UDP3305S* psu = hw_.Psu();
    if (psu && psu->IsConnected()) return true;                  // already up
    LaunchPsuConnect();   // rebuilds the driver from psuCfg_
    return true;
}

bool Workflow::DisconnectPsu()
{
    if (hw_.PsuState() == ConnState::Connecting) return false;   // let it finish
    UDP3305S* psu = hw_.Psu();
    if (!psu) return false;
    psu->Disconnect();   // stops polling; forces every output OFF (bobcat habit)
    hw_.SetPsuState(ConnState::Disconnected);
    FD_INFO("workflow", "PSU '%s' disconnected", hw_.PsuName().c_str());
    return true;
}

UDP3305S* Workflow::ConnectedPsu() const
{
    if (hw_.PsuState() != ConnState::Connected) return nullptr;
    return hw_.Psu();
}

bool Workflow::SetPsuVI(int channel, double volts, double amps)
{
    UDP3305S* psu = ConnectedPsu();
    return psu && psu->Apply(channel, volts, amps);
}

bool Workflow::PsuOutputOn(int channel)  { UDP3305S* p = ConnectedPsu(); return p && p->OutputOn(channel); }
bool Workflow::PsuOutputOff(int channel) { UDP3305S* p = ConnectedPsu(); return p && p->OutputOff(channel); }

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

    // Live focus score, only while something is watching it (the Jog window's
    // Camera tab). Gated on the frame counter so a static feed costs nothing,
    // and scored over the ROI, which keeps it well under a millisecond — this
    // is compute, not a blocking call, so it is legal on the UI thread.
    if (focusStream_.load()) {
        const unsigned long long seq = hw_.Camera().FrameSeq();
        if (seq != focusSeqSeen_) {
            focusSeqSeen_ = seq;
            MeasureFocus(focusRoiPx_.load());
        }
    }

    TickPath(dt);   // mapped-path executor (no-op unless a path is running)
}

// ---- Mapped-path executor ---------------------------------------------------
// Runs on the UI thread from Update(). Arrival = every targeted axis within
// kPathTolMM of its target; each leg gets kPathLegTimeoutSec before the path
// aborts (device stalled / interlocked / velocity too low).
namespace {
constexpr double kPathTolMM         = 0.01;
constexpr float  kPathLegTimeoutSec = 120.0f;
} // namespace

bool Workflow::StartPathMove(std::size_t devIdx, std::vector<PathWaypoint> waypoints)
{
    if (pathExec_.active) {
        FD_WARN("path", "a path is already running - cancel it first");
        return false;
    }
    if (SafetyBlocked()) {
        FD_WARN("safety", "path blocked - acknowledge the safety alert first");
        return false;
    }
    if (devIdx >= hw_.MotionCount() || waypoints.empty()) return false;
    IMotionController* m = hw_.MotionDevice(devIdx);
    if (!m || !m->IsConnected()) return false;

    if (!m->MoveAbsolute(waypoints.front().targets)) {
        FD_WARN("path", "path start failed: move to '%s' rejected",
                waypoints.front().label.c_str());
        return false;
    }
    pathCancelReq_.store(false);
    pathExec_.active    = true;
    pathExec_.devIdx    = devIdx;
    pathExec_.waypoints = std::move(waypoints);
    pathExec_.cur       = 0;
    pathExec_.legTime   = 0.0f;
    FD_INFO("path", "%s: path started, %zu node(s), first '%s'", m->Name().c_str(),
            pathExec_.waypoints.size(), pathExec_.waypoints.front().label.c_str());
    return true;
}

void Workflow::CancelPathMove()
{
    if (!pathExec_.active) return;
    pathExec_.active = false;
    FD_WARN("path", "path cancelled at node %zu/%zu",
            pathExec_.cur + 1, pathExec_.waypoints.size());
    StopMotion(pathExec_.devIdx);
}

Workflow::PathProgress Workflow::PathStatus() const
{
    PathProgress p;
    p.active = pathExec_.active;
    if (p.active) {
        p.devIdx  = pathExec_.devIdx;
        p.current = pathExec_.cur;
        p.total   = pathExec_.waypoints.size();
        p.label   = pathExec_.waypoints[pathExec_.cur].label;
    }
    return p;
}

void Workflow::TickPath(float dt)
{
    if (!pathExec_.active) return;
    if (pathCancelReq_.exchange(false)) {   // e.g. safety STOP from the watch thread
        pathExec_.active = false;
        FD_WARN("path", "path aborted - motion was stopped");
        return;
    }
    if (SafetyBlocked()) {
        pathExec_.active = false;
        FD_WARN("path", "path aborted - safety alert active");
        return;
    }
    IMotionController* m = (pathExec_.devIdx < hw_.MotionCount())
                             ? hw_.MotionDevice(pathExec_.devIdx) : nullptr;
    if (!m || !m->IsConnected()) {
        pathExec_.active = false;
        FD_WARN("path", "path aborted - device unavailable");
        return;
    }
    m->RequestFastPoll();   // keep the arrival check on fresh positions

    const PathWaypoint& wp  = pathExec_.waypoints[pathExec_.cur];
    const std::map<std::string, double> pos = m->Positions();
    bool arrived = true;
    for (const auto& [axis, tgt] : wp.targets) {
        const auto it = pos.find(axis);
        if (it == pos.end() || std::fabs(it->second - tgt) > kPathTolMM) {
            arrived = false;
            break;
        }
    }

    if (arrived) {
        FD_INFO("path", "%s: reached '%s' (%zu/%zu)", m->Name().c_str(),
                wp.label.c_str(), pathExec_.cur + 1, pathExec_.waypoints.size());
        ++pathExec_.cur;
        pathExec_.legTime = 0.0f;
        if (pathExec_.cur >= pathExec_.waypoints.size()) {
            pathExec_.active = false;
            FD_INFO("path", "%s: path complete", m->Name().c_str());
            return;
        }
        const PathWaypoint& next = pathExec_.waypoints[pathExec_.cur];
        if (!m->MoveAbsolute(next.targets)) {
            pathExec_.active = false;
            FD_WARN("path", "path aborted - move to '%s' rejected", next.label.c_str());
        }
        return;
    }

    pathExec_.legTime += dt;
    if (pathExec_.legTime > kPathLegTimeoutSec) {
        pathExec_.active = false;
        FD_ERROR("path", "path aborted - timeout driving to '%s'", wp.label.c_str());
        StopMotion(pathExec_.devIdx);
    }
}

const char* Workflow::StatusText() const
{
    return hw_.AllConnected() ? "ready" : "connecting hardware...";
}

// ---- Vision: focus measure --------------------------------------------------

unsigned long long Workflow::CameraFrameSeq() const
{
    return hw_.Camera().FrameSeq();
}

Workflow::FocusSample Workflow::MeasureFocus(int roiPx)
{
    FocusSample s;
    s.roiPx = roiPx > 0 ? roiPx : 0;

    // One frame copy + one score per call, under the lock: the auto-focus
    // worker and the UI readout may both land here, and they share the scratch
    // buffer. The copy is the expensive part (a few MB memcpy), not the score.
    {
        std::lock_guard<std::mutex> lock(focusMu_);
        int w = 0, h = 0;
        unsigned long long seq = 0;
        if (!hw_.Camera().GetLatestFrameSeq(focusBuf_, w, h, seq) || w <= 0 || h <= 0)
            return s;   // ok stays false — no frame yet

        const FocusRoi roi = (roiPx > 0) ? CenteredRoi(w, h, roiPx, roiPx)
                                         : CenteredRoi(w, h, 0, 0);
        s.ok       = true;
        s.width    = w;
        s.height   = h;
        s.frameSeq = seq;
        s.score    = FocusScoreRgb(focusBuf_.data(), w, h, roi);
        s.meanLuma = MeanLumaRgb(focusBuf_.data(), w, h, roi);
        lastFocus_ = s;
    }

    DataBus::Instance().Stream(topics::kCameraFocus).Publish(NowSec(), s.score);
    return s;
}

void Workflow::SetFocusStreaming(bool on, int roiPx)
{
    if (roiPx > 0) focusRoiPx_.store(roiPx);
    focusStream_.store(on);
}

Workflow::FocusSample Workflow::LatestFocus() const
{
    std::lock_guard<std::mutex> lock(focusMu_);
    return lastFocus_;
}

void Workflow::CameraPixelToMm(double& x, double& y) const
{
    x = camCfg_.pixelToMmX;
    y = camCfg_.pixelToMmY;
}

void Workflow::SetCameraPixelToMm(double x, double y)
{
    if (x > 0.0) camCfg_.pixelToMmX = x;
    if (y > 0.0) camCfg_.pixelToMmY = y;
}

// ---- Vision: feature detection ----------------------------------------------

void Workflow::InvalidateDetector(const std::string& uid)
{
    std::lock_guard<std::mutex> lock(detectMu_);
    if (uid.empty()) { detectors_.clear(); detectorStamp_.clear(); return; }
    detectors_.erase(uid);
    detectorStamp_.erase(uid);
}

VisionDetection Workflow::LatestDetection() const
{
    std::lock_guard<std::mutex> lock(detectMu_);
    return lastDetection_;
}

bool Workflow::Detect(const VisionModel& m, const std::string& modelDir,
                      VisionDetection* out)
{
    VisionDetection local;
    VisionDetection* det = out ? out : &local;
    *det = VisionDetection{};
    det->method = m.method;

    {
        std::lock_guard<std::mutex> lock(detectMu_);

        // Rebuild whenever the model's serialization differs from the one the
        // cached detector was built with. Comparing the whole document rather
        // than a "dirty" flag means no edit path can forget to invalidate:
        // HALCON bakes the angle range and the pyramid depth into the model, so
        // a stale detector silently ignores half the parameter form.
        const std::string stamp = VisionModelToJson(m).dump();
        auto it = detectors_.find(m.uid);
        if (it == detectors_.end() || detectorStamp_[m.uid] != stamp) {
            auto d = std::make_unique<HalconDetector>();
            std::string err;
            if (!d->Load(m, modelDir, &err)) {
                det->ok    = false;
                det->error = err;
                lastDetection_ = *det;
                return false;
            }
            detectorStamp_[m.uid] = stamp;
            it = detectors_.insert_or_assign(m.uid, std::move(d)).first;
        }

        int w = 0, h = 0;
        unsigned long long seq = 0;
        if (!hw_.Camera().GetLatestFrameSeq(detectBuf_, w, h, seq) || w <= 0 || h <= 0) {
            det->ok    = false;
            det->error = "no camera frame";
            lastDetection_ = *det;
            return false;
        }

        it->second->Detect(detectBuf_.data(), w, h, det);
        det->frameSeq = seq;   // Detect resets the struct; stamp the provenance after

        // Pixels -> millimetres from the image centre, using the live camera
        // calibration. Doing it HERE, once, is what lets an auto station run
        // with no UI in the loop.
        if (det->ok) {
            det->dxMM = (det->xPx - 0.5 * w) * camCfg_.pixelToMmX;
            det->dyMM = (det->yPx - 0.5 * h) * camCfg_.pixelToMmY;
        }
        lastDetection_ = *det;
    }

    // Published outside the lock: a subscriber must never be able to deadlock
    // against a detect. Misses go on the stream too — a reliability run counts
    // them, so suppressing them would flatter every model.
    const double t = NowSec();
    DataBus& bus = DataBus::Instance();
    bus.Stream(topics::kVisionOk).Publish(t, det->ok ? 1.0 : 0.0);
    if (det->ok) {
        bus.Stream(topics::kVisionX).Publish(t, det->xPx);
        bus.Stream(topics::kVisionY).Publish(t, det->yPx);
        bus.Stream(topics::kVisionScore).Publish(t, det->score);
        bus.Stream(topics::kVisionAngle).Publish(t, det->angleDeg);
    }
    return det->ok;
}

// The teach page's filtered-image preview. No detector is built and none is
// cached: the chain is stateless, and the operator is usually tuning it before
// there is a template to build a model from.
//
// Under detectMu_ like Detect, because both borrow detectBuf_ — a preview and a
// detect racing for that one buffer is exactly the kind of bug that would show
// up as an occasional wrong picture and never reproduce.
bool Workflow::PreprocessPreview(const VisionModel& m, std::vector<unsigned char>* gray,
                                 FocusRoi* used, std::string* err)
{
    if (gray) gray->clear();
    if (used) *used = FocusRoi{};

    std::lock_guard<std::mutex> lock(detectMu_);

    int fw = 0, fh = 0;
    unsigned long long seq = 0;
    if (!hw_.Camera().GetLatestFrameSeq(detectBuf_, fw, fh, seq) || fw <= 0 || fh <= 0) {
        if (err) *err = "no camera frame";
        return false;
    }
    // `used` is the CLAMPED ROI, origin included: the caller draws this plane
    // back over the live image, and an origin it had to guess would put the
    // overlay somewhere the pixels never were.
    return HalconDetector::Preprocess(m, detectBuf_.data(), fw, fh, gray, used, err);
}

bool Workflow::TeachDetector(const VisionModel& m, const std::string& modelDir,
                             const VisionRoi& patch, int* tmplW, int* tmplH,
                             std::string* err)
{
    std::lock_guard<std::mutex> lock(detectMu_);

    int w = 0, h = 0;
    unsigned long long seq = 0;
    if (!hw_.Camera().GetLatestFrameSeq(detectBuf_, w, h, seq) || w <= 0 || h <= 0) {
        if (err) *err = "no camera frame to teach from";
        return false;
    }

    // The model directory is created on demand — a product that never taught a
    // template has no vision/ folder, and the first teach is when it needs one.
    std::error_code ec;
    std::filesystem::create_directories(modelDir, ec);

    auto d = std::make_unique<HalconDetector>();
    const std::string pgm = modelDir + "/" + m.uid + ".pgm";
    if (!d->Teach(m, detectBuf_.data(), w, h, patch, pgm, tmplW, tmplH, err))
        return false;

    // The caller is about to write tmplW/tmplH and the filename back into the
    // model, which changes its stamp — so drop the cache entry rather than
    // adopting this detector under a stamp that is already stale.
    detectors_.erase(m.uid);
    detectorStamp_.erase(m.uid);
    return true;
}

std::size_t Workflow::GantryIndex() const
{
    // Locate gantry-main (fall back to the first ACS controller).
    for (size_t i = 0; i < hw_.MotionCount(); ++i) {
        IMotionController* m = hw_.MotionDevice(i);
        if (m && m->Type() == "ACS" && m->Name() == "gantry-main") return i;
    }
    for (size_t i = 0; i < hw_.MotionCount(); ++i) {
        IMotionController* m = hw_.MotionDevice(i);
        if (m && m->Type() == "ACS") return i;
    }
    return hw_.MotionCount();
}

bool Workflow::JogGantryZ(double deltaMM)
{
    const std::size_t gi = GantryIndex();
    if (gi >= hw_.MotionCount()) {
        FD_WARN("workflow", "gantry Z jog: no ACS gantry available");
        return false;
    }
    // Same path as the Jog window: global direction -> local axis via transform
    // (JogGlobal/JogLocalAxis also carry the safety-block check).
    const float gz = (deltaMM >= 0.0) ? 1.0f : -1.0f;
    return JogGlobal(gi, 0.0f, 0.0f, gz, std::fabs(deltaMM));
}

bool Workflow::MoveGantryRelativeMM(double dxMM, double dyMM)
{
    if (SafetyBlocked()) {
        FD_WARN("safety", "move blocked - acknowledge the safety alert first");
        return false;
    }
    const std::size_t gi = GantryIndex();
    IMotionController* g = (gi < hw_.MotionCount()) ? hw_.MotionDevice(gi) : nullptr;
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
        FD_WARN("workflow", "%s velocity %.3f mm/s over limit - clamped to %.1f",
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
    if (std::fabs(loc[ax]) < 1e-9) return false;   // unmapped (zero column)

    const std::vector<std::string> axes = m->Axes();
    if (ax >= static_cast<int>(axes.size())) return false;
    const double delta = stepMM * (loc[ax] >= 0.0 ? 1.0 : -1.0);
    return JogLocalAxis(devIdx, axes[ax], delta);
}

bool Workflow::GlobalDeltaToAxes(std::size_t devIdx, double gxMM, double gyMM,
                                 double gzMM,
                                 std::vector<std::pair<std::string, double>>* out) const
{
    if (out) out->clear();
    if (devIdx >= hw_.MotionCount()) return false;
    IMotionController* m = hw_.MotionDevice(devIdx);
    if (!m) return false;

    AxisTransform t;   // identity if none configured
    const auto it = transforms_.find(m->Name());
    if (it != transforms_.end()) t = it->second;

    const double g[3] = {gxMM, gyMM, gzMM};
    const std::vector<std::string> axes = m->Axes();
    for (int i = 0; i < 3; ++i) {
        if (i >= static_cast<int>(axes.size())) break;
        const double d = t.m[i][0] * g[0] + t.m[i][1] * g[1] + t.m[i][2] * g[2];
        if (out) out->emplace_back(axes[static_cast<std::size_t>(i)], d);
    }
    return true;
}

bool Workflow::GlobalDeltaTargets(std::size_t devIdx, double gxMM, double gyMM,
                                  double gzMM,
                                  std::vector<std::pair<std::string, double>>* targets) const
{
    if (targets) targets->clear();
    if (!targets || devIdx >= hw_.MotionCount()) return false;
    IMotionController* m = hw_.MotionDevice(devIdx);
    if (!m) return false;
    if (!m->IsConnected()) {
        FD_WARN("workflow", "relative move: '%s' not connected", m->Name().c_str());
        return false;
    }

    std::vector<std::pair<std::string, double>> deltas;
    if (!GlobalDeltaToAxes(devIdx, gxMM, gyMM, gzMM, &deltas)) return false;

    // Relative = live pose + the mapped delta, all axes in one command. An axis
    // the device does not report is skipped rather than assumed to be at zero:
    // a target built from a missing feedback value is a move to somewhere
    // nobody asked for.
    const std::map<std::string, double> pos = m->Positions();
    std::string detail;
    for (const auto& [axis, d] : deltas) {
        const auto it = pos.find(axis);
        if (it == pos.end()) continue;
        targets->emplace_back(axis, it->second + d);
        char buf[64];
        std::snprintf(buf, sizeof(buf), "%s%s %+.4f -> %.4f", detail.empty() ? "" : "  ",
                      axis.c_str(), d, it->second + d);
        detail += buf;
    }
    if (targets->empty()) {
        FD_WARN("workflow", "%s: global delta maps to no axis the device reports",
                m->Name().c_str());
        return false;
    }

    FD_INFO("workflow", "%s relative move dGlobal=(%+.4f, %+.4f, %+.4f) mm | %s",
            m->Name().c_str(), gxMM, gyMM, gzMM, detail.c_str());
    return true;
}

bool Workflow::MoveRelativeGlobal(std::size_t devIdx, double gxMM, double gyMM,
                                  double gzMM,
                                  std::vector<std::pair<std::string, double>>* issued)
{
    if (issued) issued->clear();
    if (SafetyBlocked()) {
        FD_WARN("safety", "move blocked - acknowledge the safety alert first");
        return false;
    }
    std::vector<std::pair<std::string, double>> targets;
    if (!GlobalDeltaTargets(devIdx, gxMM, gyMM, gzMM, &targets)) return false;

    IMotionController* m = hw_.MotionDevice(devIdx);
    if (!m) return false;
    if (issued) *issued = targets;
    return m->MoveAbsolute(targets);
}

bool Workflow::JogGlobalRot(std::size_t devIdx, float gu, float gv, float gw, double step)
{
    if (devIdx >= hw_.MotionCount()) return false;
    IMotionController* m = hw_.MotionDevice(devIdx);
    if (!m) return false;

    // Map the global rotation direction to a signed local rotational axis via
    // the transform's rotation block (same scheme as JogGlobal's translation).
    AxisTransform t;   // identity if none configured
    const auto it = transforms_.find(m->Name());
    if (it != transforms_.end()) t = it->second;
    const double g[3] = {gu, gv, gw};
    double loc[3];
    for (int i = 0; i < 3; ++i)
        loc[i] = t.r[i][0] * g[0] + t.r[i][1] * g[1] + t.r[i][2] * g[2];
    int ax = 0;
    for (int i = 1; i < 3; ++i) if (std::fabs(loc[i]) > std::fabs(loc[ax])) ax = i;
    if (std::fabs(loc[ax]) < 1e-9) return false;   // unmapped (zero column)

    // Local rotational axes are the ones beyond XYZ (hexapod U/V/W).
    const std::vector<std::string> axes = m->Axes();
    if (3 + ax >= static_cast<int>(axes.size())) return false;
    const double delta = step * (loc[ax] >= 0.0 ? 1.0 : -1.0);
    return JogLocalAxis(devIdx, axes[3 + ax], delta);
}

bool Workflow::JogLocalAxis(std::size_t devIdx, const std::string& axis, double deltaMM)
{
    if (SafetyBlocked()) {
        FD_WARN("safety", "jog blocked - acknowledge the safety alert first");
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
    // A manual stop on the pathing device abandons the running path (UI thread;
    // the executor's own Cancel/timeout deactivate before calling here).
    if (pathExec_.active && pathExec_.devIdx == devIdx) {
        pathExec_.active = false;
        FD_WARN("path", "path abandoned - device stopped");
    }
    IMotionController* m = hw_.MotionDevice(devIdx);
    if (!m) return false;
    FD_WARN("workflow", "STOP %s", m->Name().c_str());
    return m->StopAll();
}

bool Workflow::MoveToPosition(std::size_t devIdx,
                              const std::vector<std::pair<std::string, double>>& targets)
{
    if (SafetyBlocked()) {
        FD_WARN("safety", "move blocked - acknowledge the safety alert first");
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
