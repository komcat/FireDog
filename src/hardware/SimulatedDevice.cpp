// SimulatedDevice.cpp
#include "hardware/SimulatedDevice.h"

#include <chrono>
#include <thread>
#include <utility>

namespace firedog {

SimulatedDevice::SimulatedDevice(std::string name, int connectMs)
    : name_(std::move(name)), connectMs_(connectMs) {}

bool SimulatedDevice::Connect()
{
    state_.store(ConnState::Connecting);
    std::this_thread::sleep_for(std::chrono::milliseconds(connectMs_));
    state_.store(ConnState::Connected);
    return true;
}

void SimulatedDevice::Disconnect()
{
    state_.store(ConnState::Disconnected);
}

} // namespace firedog
