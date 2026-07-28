// HardwareManager.h : layer 1 — the single owner of every instrument. It holds
// both the IDevice-based simulated fleet AND the two direct-SDK drivers (camera,
// Keithley), and connects them concurrently so startup never blocks the UI.
//
// Ownership lives here; the connect/bias/poll *sequence* is driven by Workflow
// (layer 2). The UI (layer 3) only reads Snapshot() and the accessors below —
// it never constructs or connects a device.
#pragma once

#include "hardware/IDevice.h"
#include "hardware/camera/PylonCamera.h"
#include "hardware/instruments/CLD101x.h"
#include "hardware/instruments/Keithley6482.h"
#include "hardware/io/IIoDevice.h"
#include "hardware/io/IoWatch.h"
#include "hardware/motion/IMotionController.h"
#include "hardware/pneumatic/PneumaticSlide.h"

#include <atomic>
#include <future>
#include <memory>
#include <string>
#include <vector>

namespace firedog {

class HardwareManager {
public:
    HardwareManager();
    ~HardwareManager();

    void ConnectAllAsync();          // fire off every simulated Connect() on its own thread
    bool AllConnected() const;

    struct DeviceStatus { std::string name; ConnState state; };
    std::vector<DeviceStatus> Snapshot() const;   // thread-safe copy for the UI (incl. camera + Keithley)

    // --- Direct-SDK instruments (no IDevice adapter — deliberately). ----------
    // Owned here; Workflow drives their connect/bias/poll and reports state back.
    PylonCamera&  Camera()   { return camera_; }
    Keithley6482* Keithley() { return keithley_.get(); }   // null until Workflow builds it
    void SetKeithley(std::unique_ptr<Keithley6482> k) { keithley_ = std::move(k); }
    CLD101x*      Cld()      { return cld_.get(); }        // null until Workflow builds it
    void SetCld(std::unique_ptr<CLD101x> c) { cld_ = std::move(c); }

    // --- Motion fleet (PI hexapods + ACS gantry). Multiple, heterogeneous. ----
    // Workflow builds each from config, hands it here, then drives async connect.
    // Each slot carries its own ConnState atomic (the async-connect hand-off).
    void   AddMotion(std::unique_ptr<IMotionController> dev);
    size_t MotionCount() const { return motion_.size(); }
    IMotionController* MotionDevice(size_t i) const { return motion_[i]->dev.get(); }
    ConnState MotionState(size_t i) const { return motion_[i]->state.load(); }
    void      SetMotionState(size_t i, ConnState s) { motion_[i]->state.store(s); }
    const std::string& MotionName(size_t i) const { return motion_[i]->name; }

    // --- IO fleet (Ezi-IO boards + WAGO couplers). Multiple, heterogeneous. --
    // Same ownership pattern as the motion fleet: Workflow builds each from
    // config, hands it here, then drives async connect + polling.
    void   AddIo(std::unique_ptr<IIoDevice> dev);
    size_t IoCount() const { return io_.size(); }
    IIoDevice* IoDevice(size_t i) const { return io_[i]->dev.get(); }
    ConnState IoState(size_t i) const { return io_[i]->state.load(); }
    void      SetIoState(size_t i, ConnState s) { io_[i]->state.store(s); }
    const std::string& IoName(size_t i) const { return io_[i]->name; }

    // --- Pneumatic slides (synthetic devices over the IO fleet). -------------
    // No ConnState: a slide's "connection" is its underlying IO devices
    // (PneumaticSlide::IsIoConnected), which are already in Snapshot().
    void AddSlide(std::unique_ptr<PneumaticSlide> s) { slides_.push_back(std::move(s)); }
    size_t SlideCount() const { return slides_.size(); }
    PneumaticSlide* Slide(size_t i) const { return slides_[i].get(); }

    // --- IO watch (background input->output override; may be null). ----------
    void SetIoWatch(std::unique_ptr<IoWatch> w) { ioWatch_ = std::move(w); }
    IoWatch* Watch() const { return ioWatch_.get(); }

    // Connection state is the atomic hand-off point between the async connect
    // (Workflow) and the UI read. Names are set once on the main thread before
    // the render loop, so a plain std::string is race-free here.
    ConnState CameraState()   const { return camState_.load(); }
    ConnState KeithleyState() const { return keiState_.load(); }
    ConnState CldState()      const { return cldState_.load(); }
    void SetCameraState(ConnState s)   { camState_.store(s); }
    void SetKeithleyState(ConnState s) { keiState_.store(s); }
    void SetCldState(ConnState s)      { cldState_.store(s); }

    const std::string& CameraName()   const { return camName_; }
    const std::string& KeithleyName() const { return keiName_; }
    const std::string& CldName()      const { return cldName_; }
    void SetCameraName(std::string n)   { camName_ = std::move(n); }
    void SetKeithleyName(std::string n) { keiName_ = std::move(n); }
    void SetCldName(std::string n)      { cldName_ = std::move(n); }

private:
    // A motion device + its async-connect state. Held via unique_ptr so the
    // atomic stays put across vector growth.
    struct MotionSlot {
        std::unique_ptr<IMotionController> dev;
        std::atomic<ConnState>             state{ConnState::Disconnected};
        std::string                        name;
    };

    // An IO device + its async-connect state (same shape as MotionSlot).
    struct IoSlot {
        std::unique_ptr<IIoDevice> dev;
        std::atomic<ConnState>     state{ConnState::Disconnected};
        std::string                name;
    };

    std::vector<std::unique_ptr<IDevice>>        devices_;
    std::vector<std::unique_ptr<MotionSlot>>     motion_;
    std::vector<std::unique_ptr<IoSlot>>         io_;
    std::vector<std::unique_ptr<PneumaticSlide>> slides_;   // reference io_ devices
    std::unique_ptr<IoWatch>                     ioWatch_;  // references io_ devices;
                                                            // declared after io_ so its
                                                            // thread stops before they die
    std::vector<std::future<void>>               pending_;

    // Direct-SDK instruments, owned alongside the IDevice fleet. Their own
    // destructors stop the grab/poll threads.
    PylonCamera                   camera_;
    std::unique_ptr<Keithley6482> keithley_;
    std::unique_ptr<CLD101x>      cld_;
    std::atomic<ConnState>        camState_{ConnState::Disconnected};
    std::atomic<ConnState>        keiState_{ConnState::Disconnected};
    std::atomic<ConnState>        cldState_{ConnState::Disconnected};
    std::string                   camName_ = "Camera";
    std::string                   keiName_ = "Keithley";
    std::string                   cldName_ = "CLD101x";
};

} // namespace firedog
