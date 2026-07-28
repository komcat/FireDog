// Workflow.cpp
#include "workflow/Workflow.h"

#include "hardware/HardwareManager.h"
#include "hardware/HardwareConfig.h"
#include "hardware/motion/PIHexapod.h"
#include "hardware/motion/ACSController.h"
#include "core/Topics.h"
#include "core/Log.h"

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

void Workflow::Start(const HardwareConfig& hw)
{
    StartMotion(hw);
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
}

const char* Workflow::StatusText() const
{
    return hw_.AllConnected() ? "ready" : "connecting hardware...";
}

bool Workflow::MoveGantryRelativeMM(double dxMM, double dyMM)
{
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
    if (devIdx >= hw_.MotionCount() || targets.empty()) return false;
    IMotionController* m = hw_.MotionDevice(devIdx);
    return m && m->MoveAbsolute(targets);   // coordinated (PI_MOV / acsc_ToPointM+GoM)
}

} // namespace firedog
