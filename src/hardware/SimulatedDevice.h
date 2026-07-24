// SimulatedDevice.h : layer 1 — stand-in for a real instrument until the
// vendor SDK adapter (PI GCS2, ACSC, VISA) is wired in.
#pragma once

#include "hardware/IDevice.h"

#include <atomic>
#include <string>

namespace firedog {

class SimulatedDevice : public IDevice {
public:
    SimulatedDevice(std::string name, int connectMs);

    const std::string& Name() const override { return name_; }
    bool  Connect()    override;   // blocks ~connectMs, then succeeds
    void  Disconnect() override;
    ConnState State() const override { return state_.load(); }

private:
    std::string            name_;
    int                    connectMs_;
    std::atomic<ConnState> state_{ConnState::Disconnected};
};

} // namespace firedog
