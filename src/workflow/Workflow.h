// Workflow.h : layer 2 — orchestrates the hardware collaboration. It owns the
// startup sequence (connect + bias + poll) and, later, the alignment procedures.
// Device *objects* live in HardwareManager (layer 1); Workflow only drives them.
#pragma once

#include "hardware/HardwareConfig.h"   // AxisTransform, HardwareConfig

#include <cstddef>
#include <future>
#include <map>
#include <string>
#include <vector>

namespace firedog {

class HardwareManager;

class Workflow {
public:
    explicit Workflow(HardwareManager& hw);

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

private:
    void StartCamera(const HardwareConfig& hw);
    void StartKeithley(const HardwareConfig& hw);
    void StartMotion(const HardwareConfig& hw);

    HardwareManager& hw_;

    // Connect futures are held here (not in the UI). Destroyed before hw_ in App,
    // so their blocking dtor still sees a live HardwareManager.
    std::future<void>              camConnect_;
    std::future<void>              keiConnect_;
    std::vector<std::future<void>> motionConnect_;   // one per motion device

    std::map<std::string, AxisTransform> transforms_;   // global->local, per device

    float clock_  = 0.0f;
    float sample_ = 0.0f;
};

} // namespace firedog
