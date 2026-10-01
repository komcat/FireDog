// HardwareManager.cpp
#include "hardware/HardwareManager.h"

#include "hardware/SimulatedDevice.h"
#include "core/Log.h"

namespace firedog {

HardwareManager::HardwareManager()
{
    // Remaining simulated instruments from readme.txt. PI/ACS are now real
    // layer-1 drivers (added by Workflow from config via AddMotion); camera,
    // Keithley, CLD101x, and the UDP3305S PSU are direct-SDK members. Replace
    // each SimulatedDevice below with its SDK adapter as it comes online.
    devices_.emplace_back(std::make_unique<SimulatedDevice>("SMU", 450));
}

void HardwareManager::AddMotion(std::unique_ptr<IMotionController> dev)
{
    auto slot  = std::make_unique<MotionSlot>();
    slot->name = dev->Name();
    slot->dev  = std::move(dev);
    motion_.push_back(std::move(slot));
}

void HardwareManager::AddIo(std::unique_ptr<IIoDevice> dev)
{
    auto slot  = std::make_unique<IoSlot>();
    slot->name = dev->Name();
    slot->dev  = std::move(dev);
    io_.push_back(std::move(slot));
}

std::size_t HardwareManager::AddPsuSlot(std::string name)
{
    auto slot  = std::make_unique<PsuSlot>();
    slot->name = std::move(name);
    psu_.push_back(std::move(slot));
    return psu_.size() - 1;
}

int HardwareManager::PsuIndex(const std::string& name) const
{
    if (psu_.empty()) return -1;
    if (name.empty()) return 0;
    for (std::size_t i = 0; i < psu_.size(); ++i)
        if (psu_[i]->name == name) return static_cast<int>(i);
    return -1;
}

HardwareManager::~HardwareManager()
{
    for (auto& f : pending_) {
        if (f.valid()) f.wait();
    }
}

void HardwareManager::ConnectAllAsync()
{
    FD_INFO("hw", "connecting %zu simulated devices", devices_.size());
    pending_.clear();
    for (auto& d : devices_) {
        IDevice* dev = d.get();
        pending_.push_back(std::async(std::launch::async, [dev] {
            const bool ok = dev->Connect();
            if (ok) FD_INFO("hw", "%s connected", dev->Name().c_str());
            else    FD_WARN("hw", "%s failed to connect", dev->Name().c_str());
        }));
    }
}

bool HardwareManager::AllConnected() const
{
    for (const auto& d : devices_) {
        if (d->State() != ConnState::Connected) return false;
    }
    return true;
}

std::vector<HardwareManager::DeviceStatus> HardwareManager::Snapshot() const
{
    std::vector<DeviceStatus> out;
    out.reserve(motion_.size() + io_.size() + devices_.size() + 4);
    // Motion fleet first (PI hexapods + ACS gantry), then the IO fleet, then
    // the simulated instruments, then the direct-SDK instruments.
    for (const auto& m : motion_) {
        out.push_back({m->name, m->state.load()});
    }
    for (const auto& i : io_) {
        out.push_back({i->name, i->state.load()});
    }
    for (const auto& d : devices_) {
        out.push_back({d->Name(), d->State()});
    }
    out.push_back({camName_, camState_.load()});
    out.push_back({keiName_, keiState_.load()});
    out.push_back({cldName_, cldState_.load()});
    for (const auto& p : psu_) {
        out.push_back({p->name, p->state.load()});
    }
    out.push_back({lightName_, lightState_.load()});
    return out;
}

} // namespace firedog
