// Workflow.h : layer 2 — orchestrates the hardware collaboration. It owns the
// startup sequence (connect + bias + poll) and, later, the alignment procedures.
// Device *objects* live in HardwareManager (layer 1); Workflow only drives them.
#pragma once

#include "hardware/HardwareConfig.h"   // AxisTransform, HardwareConfig

#include <cstddef>
#include <cstdint>
#include <future>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "hardware/io/SafetyWatch.h"

namespace firedog {

class HardwareManager;

class Workflow {
public:
    explicit Workflow(HardwareManager& hw);
    ~Workflow();   // stops the safety watch before hw_'s devices can die

    // Bring the direct-SDK instruments online from config: async connect, set
    // the alignment bias, start the poll/publish. Call once from the UI at startup.
    void Start(const HardwareConfig& hw);

    void  Update(float dt);
    float LatestSample() const { return sample_; }
    const char* StatusText() const;

    // Move gantry-main (ACS) by a relative XY offset in mm, both axes together.
    // Called by the UI's click-to-move on the camera image. Returns false if no
    // ACS gantry is available. The UI never touches the device directly.
    bool MoveGantryRelativeMM(double dxMM, double dyMM);

    // ---- Jog (layer 2 owns the kinematics + command) ------------------------
    void SetTransforms(std::map<std::string, AxisTransform> t);
    bool SetJogVelocity(std::size_t devIdx, double mmPerSec);
    // Jog by `stepMM` along a GLOBAL direction (gx,gy,gz unit vector); the device
    // transform maps it to a signed local axis relative move.
    bool JogGlobal(std::size_t devIdx, float gx, float gy, float gz, double stepMM);
    // Jog a LOCAL axis directly (e.g. rotational U/V/W) by a signed delta.
    bool JogLocalAxis(std::size_t devIdx, const std::string& axis, double deltaMM);
    // Stop all motion on one device (jog panel Stop button).
    bool StopMotion(std::size_t devIdx);

    // Move to an absolute per-axis target set (a saved/taught position) in one
    // coordinated command. Called by the saved-positions Move button.
    bool MoveToPosition(std::size_t devIdx,
                        const std::vector<std::pair<std::string, double>>& targets);

    // ---- Digital IO commands (IO page + future procedures) ------------------
    // devIdx indexes the HardwareManager IO fleet. Writes require Connected.
    bool SetIoOutput(std::size_t devIdx, int pin, bool on);
    bool SetIoOutputByName(const std::string& device, const std::string& pinName, bool on);
    bool ClearIoLatch(std::size_t devIdx, uint64_t mask);

    // ---- Pneumatic slide commands (Pneumatics page; non-blocking) -----------
    // idx indexes the HardwareManager slide fleet.
    bool ExtendSlide(std::size_t idx);
    bool RetractSlide(std::size_t idx);
    bool StopSlide(std::size_t idx);

    // ---- IO watch (editor window) -------------------------------------------
    // Replace the running watch with cfg (stops the old one first). Also used
    // by startup. Returns false if enabled but nothing could be wired.
    bool ApplyIoWatch(const IoWatchConfig& cfg);

    // ---- Safety (first-priority critical signals, e.g. E-Stop) --------------
    // A critical alert stops all motion immediately (from the watch thread)
    // and blocks new motion commands until acknowledged.
    bool SafetyAlertActive() const { return safetyWatch_ && safetyWatch_->AlertActive(); }
    SafetyWatch::Alert SafetyAlert() const
    {
        return safetyWatch_ ? safetyWatch_->ActiveAlert() : SafetyWatch::Alert{};
    }
    void AcknowledgeSafety() { if (safetyWatch_) safetyWatch_->Acknowledge(); }
    void StopAllMotion();   // hard stop on every motion device

private:
    void StartCamera(const HardwareConfig& hw);
    void StartKeithley(const HardwareConfig& hw);
    void StartMotion(const HardwareConfig& hw);
    void StartIO(const HardwareConfig& hw);
    void StartPneumatics(const HardwareConfig& hw);   // wires slides to IO pins
    void StartIoWatch(const HardwareConfig& hw);      // wires edge->action bindings
    void StartSafetyWatch(const HardwareConfig& hw);  // wires safety signals

    // Motion is inhibited while a CRITICAL safety alert is unacknowledged.
    bool SafetyBlocked() const
    {
        return safetyWatch_ && safetyWatch_->AlertActive() && safetyWatch_->AlertCritical();
    }

    HardwareManager& hw_;

    // Connect futures are held here (not in the UI). Destroyed before hw_ in App,
    // so their blocking dtor still sees a live HardwareManager.
    std::future<void>              camConnect_;
    std::future<void>              keiConnect_;
    std::vector<std::future<void>> motionConnect_;   // one per motion device
    std::vector<std::future<void>> ioConnect_;       // one per IO device

    std::map<std::string, AxisTransform> transforms_;   // global->local, per device

    // Per-(device,axis) jog target: rapid presses accumulate onto one evolving
    // target so the latest press overrides the previous instead of queuing; the
    // target re-seeds from the live position after an idle gap (new burst).
    struct JogTrack { double target = 0.0; double lastT = -1.0e9; };
    std::map<std::string, JogTrack> jogTracks_;

    float clock_  = 0.0f;
    float sample_ = 0.0f;

    // Owned by Workflow (not HardwareManager): its callback stops motion via
    // this object, so it must die first. Declared last so it is destroyed
    // before every other member; ~Workflow also stops it explicitly.
    std::unique_ptr<SafetyWatch> safetyWatch_;
};

} // namespace firedog
