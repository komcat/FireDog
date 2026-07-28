// EziIoDevice.h : FASTECH Ezi-IO Plus-E digital-IO module over the
// EziMOTIONPlusE SDK (TCP, addressed by board id). Ported from
// bobcat_engine's EziIODevice, dropping the Qt layers.
//
// PIMPL keeps the FASTECH/Windows headers out of this header. Without
// HAS_EZIIO the .cpp compiles a simulated stub with the same API, so the
// build (and the IO page) works with no SDK and no hardware.
#pragma once

#include "hardware/io/IIoDevice.h"

#include <cstdint>
#include <map>
#include <memory>
#include <string>

namespace firedog {

class EziIoDevice : public IIoDevice {
public:
    struct Config {
        std::string name = "EziIO";
        std::string ipAddress;               // dotted, e.g. "192.168.0.3"
        int         deviceId    = 0;         // FAS board id — unique per board
        int         inputCount  = 8;
        int         outputCount = 8;
        std::map<int, std::string> inputNames;    // pin -> display name
        std::map<int, std::string> outputNames;
    };

    explicit EziIoDevice(Config cfg);
    ~EziIoDevice() override;
    EziIoDevice(const EziIoDevice&) = delete;
    EziIoDevice& operator=(const EziIoDevice&) = delete;

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
    std::unique_ptr<Impl> p_;   // PIMPL — keeps FAS_EziMOTIONPlusE.h out of the header
};

} // namespace firedog
