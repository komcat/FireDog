// PneumaticSlide.cpp
//
// State machine ported from bobcat_engine's PneumaticSlide.cpp (Extend :29,
// Stop :87, UpdatePositionInternal :139, DeterminePosition :302,
// WaitForPosition :355), reading through the IIoDevice cached Status()
// instead of EziIOManager name lookups.
//
// One deliberate change vs bobcat: Timeout is *sticky*. bobcat recomputed the
// position from the sensors on the tick after a timeout, so TIMEOUT flashed
// for one poll and collapsed to UNKNOWN. Here it persists until a new command
// is issued or a sensor definitively reports Extended/Retracted.
#include "hardware/pneumatic/PneumaticSlide.h"

#include "core/Log.h"

#include <algorithm>
#include <thread>
#include <utility>

namespace firedog {

const char* SlidePositionName(SlidePosition p)
{
    switch (p) {
        case SlidePosition::Extended:      return "Extended";
        case SlidePosition::Retracted:     return "Retracted";
        case SlidePosition::Extending:     return "Extending";
        case SlidePosition::Retracting:    return "Retracting";
        case SlidePosition::HardwareError: return "HardwareError";
        case SlidePosition::Timeout:       return "Timeout";
        default:                           return "Unknown";
    }
}

namespace {
// Read one cached pin bit; ok=false when the device has no data yet.
bool ReadInputBit(const IIoDevice* dev, int pin, bool& ok)
{
    if (!dev || pin < 0) { ok = false; return false; }
    const IIoDevice::IoStatus s = dev->Status();
    ok = s.valid;
    return ((s.inputs >> pin) & 1ull) != 0;
}
} // namespace

PneumaticSlide::PneumaticSlide(Wiring w) : w_(std::move(w)) {}

bool PneumaticSlide::IsIoConnected() const
{
    return w_.outputDev && w_.outputDev->IsConnected()
        && w_.extendedDev && w_.extendedDev->IsConnected()
        && w_.retractedDev && w_.retractedDev->IsConnected();
}

bool PneumaticSlide::SetOutput(bool on)
{
    if (!w_.outputDev || w_.outputPin < 0) { lastError_ = "output not wired"; return false; }
    if (!w_.outputDev->SetOutput(w_.outputPin, on)) {
        lastError_ = w_.outputDev->LastError();
        FD_WARN("pneumatic", "%s: output write failed: %s", w_.name.c_str(), lastError_.c_str());
        return false;
    }
    return true;
}

uint32_t PneumaticSlide::ElapsedMs() const
{
    return static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - moveStart_).count());
}

bool PneumaticSlide::Extend()
{
    std::lock_guard<std::mutex> lock(mu_);
    UpdateInternal();
    if (position_ == SlidePosition::Extended) return true;   // already there
    if (!SetOutput(true)) return false;
    target_     = SlidePosition::Extended;
    timedOut_   = false;
    moveStart_  = std::chrono::steady_clock::now();
    lastMoveMs_ = 0;
    position_   = SlidePosition::Extending;
    FD_INFO("pneumatic", "%s: extend", w_.name.c_str());
    return true;
}

bool PneumaticSlide::Retract()
{
    std::lock_guard<std::mutex> lock(mu_);
    UpdateInternal();
    if (position_ == SlidePosition::Retracted) return true;   // already there
    if (!SetOutput(false)) return false;
    target_     = SlidePosition::Retracted;
    timedOut_   = false;
    moveStart_  = std::chrono::steady_clock::now();
    lastMoveMs_ = 0;
    position_   = SlidePosition::Retracting;
    FD_INFO("pneumatic", "%s: retract", w_.name.c_str());
    return true;
}

bool PneumaticSlide::Stop()
{
    std::lock_guard<std::mutex> lock(mu_);
    target_   = SlidePosition::Unknown;
    timedOut_ = false;
    FD_WARN("pneumatic", "%s: STOP (output off)", w_.name.c_str());
    return SetOutput(false);   // de-energize — air-return safe state
}

void PneumaticSlide::Update()
{
    std::lock_guard<std::mutex> lock(mu_);
    UpdateInternal();
}

SlidePosition PneumaticSlide::Determine(bool extended, bool retracted) const
{
    if (extended && retracted) return SlidePosition::HardwareError;
    if (extended)              return SlidePosition::Extended;
    if (retracted)             return SlidePosition::Retracted;
    // Neither sensor: in transit toward the target, else position unknown.
    if (target_ == SlidePosition::Extended)  return SlidePosition::Extending;
    if (target_ == SlidePosition::Retracted) return SlidePosition::Retracting;
    return timedOut_ ? SlidePosition::Timeout : SlidePosition::Unknown;
}

SlidePosition PneumaticSlide::UpdateInternal()
{
    bool okE = false, okR = false;
    const bool extended  = ReadInputBit(w_.extendedDev, w_.extendedPin, okE);
    const bool retracted = ReadInputBit(w_.retractedDev, w_.retractedPin, okR);
    if (!okE || !okR) {
        // IO caches empty (still connecting) — no basis for a position yet.
        position_ = SlidePosition::Unknown;
        return position_;
    }

    SlidePosition newPos = Determine(extended, retracted);
    if (newPos == SlidePosition::Extended || newPos == SlidePosition::Retracted) {
        timedOut_ = false;   // a definite sensor clears a stale timeout
    }

    const bool moving = target_ == SlidePosition::Extended
                     || target_ == SlidePosition::Retracted;
    if (moving && ElapsedMs() > w_.timeoutMs) {
        newPos      = SlidePosition::Timeout;
        timedOut_   = true;
        lastMoveMs_ = ElapsedMs();
        target_     = SlidePosition::Unknown;
        lastError_  = "movement timeout";
        FD_WARN("pneumatic", "%s: movement timeout after %u ms", w_.name.c_str(), lastMoveMs_);
    }

    if (newPos != position_) {
        FD_INFO("pneumatic", "%s: %s -> %s", w_.name.c_str(),
                SlidePositionName(position_), SlidePositionName(newPos));
        position_ = newPos;
    }

    // Arrived at the target: stamp the move time and stop the clock.
    if (position_ == target_ &&
        (target_ == SlidePosition::Extended || target_ == SlidePosition::Retracted)) {
        lastMoveMs_ = ElapsedMs();
        target_     = SlidePosition::Unknown;
        FD_INFO("pneumatic", "%s: %s in %u ms", w_.name.c_str(),
                SlidePositionName(position_), lastMoveMs_);
    }
    return position_;
}

PneumaticSlide::Status PneumaticSlide::GetStatus() const
{
    std::lock_guard<std::mutex> lock(mu_);
    Status s;
    bool okE = false, okR = false;
    s.extendedOn  = ReadInputBit(w_.extendedDev, w_.extendedPin, okE);
    s.retractedOn = ReadInputBit(w_.retractedDev, w_.retractedPin, okR);
    s.ioValid     = okE && okR;
    if (w_.outputDev && w_.outputPin >= 0) {
        s.outputOn = ((w_.outputDev->Status().outputs >> w_.outputPin) & 1ull) != 0;
    }
    s.position  = position_;
    s.moving    = target_ == SlidePosition::Extended || target_ == SlidePosition::Retracted;
    s.elapsedMs = s.moving ? ElapsedMs() : lastMoveMs_;
    return s;
}

SlidePosition PneumaticSlide::Position() const
{
    std::lock_guard<std::mutex> lock(mu_);
    return position_;
}

bool PneumaticSlide::WaitForPosition(SlidePosition target, int pollMs)
{
    if (pollMs <= 0) pollMs = 50;
    // Bounded like bobcat: 2x the timeout in poll steps, at least 200 laps.
    const int maxIterations =
        std::max(200, static_cast<int>(w_.timeoutMs / static_cast<uint32_t>(pollMs)) * 2);
    for (int i = 0; i < maxIterations; ++i) {
        Update();
        const SlidePosition p = Position();
        if (p == target) return true;
        if (p == SlidePosition::Timeout) { return false; }
        if (p == SlidePosition::HardwareError) {
            std::lock_guard<std::mutex> lock(mu_);
            lastError_ = "both sensors active";
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(pollMs));
    }
    std::lock_guard<std::mutex> lock(mu_);
    lastError_ = "wait exhausted";
    return false;
}

bool PneumaticSlide::ExtendAndWait(int pollMs)
{
    if (!Extend()) return false;
    if (Position() == SlidePosition::Extended) return true;
    return WaitForPosition(SlidePosition::Extended, pollMs);
}

bool PneumaticSlide::RetractAndWait(int pollMs)
{
    if (!Retract()) return false;
    if (Position() == SlidePosition::Retracted) return true;
    return WaitForPosition(SlidePosition::Retracted, pollMs);
}

std::string PneumaticSlide::LastError() const
{
    std::lock_guard<std::mutex> lock(mu_);
    return lastError_;
}

} // namespace firedog
