// RigSession.cpp
#include "RigSession.h"

#include "RigConfigEdit.h"

#include <chrono>
#include <thread>

namespace firedog::rig {
namespace {
constexpr double kTickSec = 0.05;   // 50 ms — the rig's poll granularity
} // namespace

const char* ConnStateName(ConnState s)
{
    switch (s) {
        case ConnState::Disconnected: return "Disconnected";
        case ConnState::Connecting:   return "Connecting";
        case ConnState::Connected:    return "Connected";
        case ConnState::Failed:       return "Failed";
    }
    return "?";
}

HardwareConfig FilterToDevice(const HardwareConfig& full, const DeviceRef& only)
{
    HardwareConfig f = full;

    for (auto& m : f.motionDevices) m.enabled = false;
    for (auto& d : f.ioDevices)     d.enabled = false;
    for (auto& c : f.cameras)       c.enabled = false;
    for (auto& k : f.keithleys)     k.enabled = false;
    for (auto& c : f.clds)          c.enabled = false;
    for (auto& p : f.psus)          p.enabled = false;
    for (auto& l : f.lights)        l.enabled = false;

    switch (only.kind) {
        case DeviceKind::Motion:   f.motionDevices[only.index].enabled = true; break;
        case DeviceKind::Io:       f.ioDevices[only.index].enabled     = true; break;
        case DeviceKind::Camera:   f.cameras[only.index].enabled       = true; break;
        case DeviceKind::Keithley: f.keithleys[only.index].enabled     = true; break;
        case DeviceKind::Cld:      f.clds[only.index].enabled          = true; break;
        case DeviceKind::Psu:      f.psus[only.index].enabled          = true; break;
        case DeviceKind::Light:    f.lights[only.index].enabled        = true; break;
    }

    // Slides and both watches reference pins by name across the whole IO fleet;
    // with one device up they would only log resolution failures.
    for (auto& s : f.pneumaticSlides) s.enabled = false;
    f.ioWatch.enabled     = false;
    f.safetyWatch.enabled = false;

    return f;
}

RigSession::RigSession(const HardwareConfig& cfg, bool connectSimFleet) : cfg_(cfg)
{
    if (connectSimFleet) hw_.ConnectAllAsync();
    wf_.Start(cfg_);
}

void RigSession::Settle(double seconds)
{
    const auto until = std::chrono::steady_clock::now()
                     + std::chrono::milliseconds(static_cast<long long>(seconds * 1000.0));
    while (std::chrono::steady_clock::now() < until) {
        wf_.Update(static_cast<float>(kTickSec));
        std::this_thread::sleep_for(std::chrono::milliseconds(
            static_cast<long long>(kTickSec * 1000.0)));
    }
}

bool RigSession::AnyConnecting() const
{
    for (const auto& s : hw_.Snapshot()) {
        if (s.state == ConnState::Connecting) return true;
    }
    return false;
}

bool RigSession::WaitForConnects(double timeoutSec)
{
    const auto deadline = std::chrono::steady_clock::now()
                        + std::chrono::milliseconds(static_cast<long long>(timeoutSec * 1000.0));
    while (AnyConnecting()) {
        if (std::chrono::steady_clock::now() >= deadline) return false;
        wf_.Update(static_cast<float>(kTickSec));
        std::this_thread::sleep_for(std::chrono::milliseconds(
            static_cast<long long>(kTickSec * 1000.0)));
    }
    return true;
}

bool RigSession::AllExpectedConnected() const
{
    // The fleets contain exactly the enabled devices Workflow built, so every
    // slot in them was expected to come up.
    for (std::size_t i = 0; i < hw_.MotionCount(); ++i) {
        if (hw_.MotionState(i) != ConnState::Connected) return false;
    }
    for (std::size_t i = 0; i < hw_.IoCount(); ++i) {
        if (hw_.IoState(i) != ConnState::Connected) return false;
    }
    // The other direct-SDK instruments are singletons: only the first enabled entry of
    // each kind is ever started, so that is what we hold them to.
    if (cfg_.FirstEnabledCamera()   && hw_.CameraState()   != ConnState::Connected) return false;
    if (cfg_.FirstEnabledKeithley() && hw_.KeithleyState() != ConnState::Connected) return false;
    if (cfg_.FirstEnabledCld()      && hw_.CldState()      != ConnState::Connected) return false;
    for (std::size_t i = 0; i < hw_.PsuCount(); ++i) {   // every enabled PSU is built now
        if (hw_.PsuState(i) != ConnState::Connected) return false;
    }
    if (cfg_.FirstEnabledLight()    && hw_.LightState()    != ConnState::Connected) return false;
    return true;
}

} // namespace firedog::rig
