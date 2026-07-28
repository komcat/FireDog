// WagoIoDevice.h : WAGO 750-series digital-IO coupler over Modbus TCP
// (hand-rolled frames on a raw winsock socket — no vendor SDK). Ported from
// bobcat_engine's WagoIODevice, dropping the Qt layers and the legacy
// EZI-IO-compatible bit remapping (here bit i == pin i, per IIoDevice).
//
// Always compiled (winsock is always available); set "simulated": true in
// config to run an in-memory simulation for hardware-less UI testing.
#pragma once

#include "hardware/io/IIoDevice.h"

#include <cstdint>
#include <map>
#include <memory>
#include <string>

namespace firedog {

class WagoIoDevice : public IIoDevice {
public:
    struct Config {
        std::string name = "Wago";
        std::string ipAddress;             // dotted, e.g. "192.168.3.13"
        int         port        = 502;     // Modbus TCP
        int         inputCount  = 8;
        int         outputCount = 8;
        bool        simulated   = false;   // in-memory sim instead of sockets
        std::map<int, std::string> inputNames;    // pin -> display name
        std::map<int, std::string> outputNames;
    };

    explicit WagoIoDevice(Config cfg);
    ~WagoIoDevice() override;
    WagoIoDevice(const WagoIoDevice&) = delete;
    WagoIoDevice& operator=(const WagoIoDevice&) = delete;

    bool Connect() override;
    void Disconnect() override;
    bool IsConnected() const override;

    const std::string& Name() const override;
    const std::string& Type() const override;
    int InputCount() const override;
    int OutputCount() const override;

    IoStatus Status() const override;

    bool SetOutput(int pin, bool on) override;
    bool SetOutputs(uint64_t setMask, uint64_t clearMask) override;
    bool ClearLatch(uint64_t mask) override;

    bool StartPolling(int intervalMs, const std::string& topicIn,
                      const std::string& topicOut) override;
    void StopPolling() override;

    std::string InputPinName(int pin) const override;
    std::string OutputPinName(int pin) const override;
    int InputPinByName(const std::string& name) const override;
    int OutputPinByName(const std::string& name) const override;

    std::string LastError() const override;

private:
    struct Impl;
    std::unique_ptr<Impl> p_;   // PIMPL — keeps winsock headers out of the header
};

} // namespace firedog
