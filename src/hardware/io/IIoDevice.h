// IIoDevice.h : layer-1 interface for digital-IO devices (EZI-IO, WAGO).
// Same role as IMotionController for the motion fleet: Workflow builds the
// concrete driver from config and hands it to HardwareManager (the owner);
// the UI reads Status() and issues commands through Workflow only.
//
// Pin addressing is normalized: bit i == pin i for BOTH inputs and outputs.
// Any vendor bit layout (EZI-IO packs outputs at bits 8+/16+) is translated
// inside the driver — a shifted value must never cross this interface.
#pragma once

#include <cstdint>
#include <map>
#include <string>

namespace firedog {

class IIoDevice {
public:
    virtual ~IIoDevice() = default;

    // Blocking; called from Workflow's per-device std::async connect.
    virtual bool Connect() = 0;
    virtual void Disconnect() = 0;
    virtual bool IsConnected() const = 0;

    virtual const std::string& Name() const = 0;
    virtual const std::string& Type() const = 0;   // "EZIIO" | "WAGO"
    virtual int InputCount() const  = 0;
    virtual int OutputCount() const = 0;

    // Cached snapshot, refreshed by the poll thread. `latch` semantics differ:
    // EZI-IO latches input edges in hardware; WAGO synthesizes rising edges in
    // software per poll cycle, so it misses pulses shorter than the interval.
    struct IoStatus {
        uint64_t inputs  = 0;
        uint64_t latch   = 0;
        uint64_t outputs = 0;
        bool     valid   = false;   // false until the first successful read
    };
    virtual IoStatus Status() const = 0;   // thread-safe copy

    virtual bool SetOutput(int pin, bool on) = 0;
    virtual bool SetOutputs(uint64_t setMask, uint64_t clearMask) = 0;
    virtual bool ClearLatch(uint64_t mask) = 0;

    // Poll inputs+outputs into the cache every intervalMs (<=0 -> 100 ms) and
    // publish the bitmasks (as double) to the DataBus topics when they change.
    // An empty topic string disables that publication.
    virtual bool StartPolling(int intervalMs, const std::string& topicIn,
                              const std::string& topicOut) = 0;
    virtual void StopPolling() = 0;

    // Pin names from config. "" if unnamed; -1 if the name is unknown.
    virtual std::string InputPinName(int pin) const  = 0;
    virtual std::string OutputPinName(int pin) const = 0;
    virtual int InputPinByName(const std::string& name) const  = 0;
    virtual int OutputPinByName(const std::string& name) const = 0;

    virtual std::string LastError() const = 0;
};

// Bidirectional pin-name lookup built from the config map. Plain data helper
// shared by the drivers' Impls (not part of the device interface).
struct IoPinNames {
    std::map<int, std::string> byPin;
    std::map<std::string, int> byName;

    void Load(const std::map<int, std::string>& names)
    {
        byPin = names;
        byName.clear();
        for (const auto& [pin, name] : names) byName[name] = pin;
    }
    std::string NameOf(int pin) const
    {
        const auto it = byPin.find(pin);
        return it != byPin.end() ? it->second : std::string();
    }
    int PinOf(const std::string& name) const
    {
        const auto it = byName.find(name);
        return it != byName.end() ? it->second : -1;
    }
};

} // namespace firedog
