// Keithley6482.h : layer 1 — direct VISA driver for the Keithley 6482
// dual-channel picoammeter + voltage source (no ISMUDevice/adapter/Qt).
//
// SCPI sequence ported from bobcat_engine's Keithley6482Adapter. PIMPL keeps
// <visa.h> out of this header. A background poll thread reads both channels
// and publishes each to a DataBus topic. Without HAS_VISA it compiles a
// simulated dual-channel stub so the pub-sub + chart pipeline still runs.
//
// One of the ISmu implementations — see ISmu.h for the shared contract and
// Keithley2602B.h for the LAN/TSP sibling.
#pragma once

#include "hardware/instruments/ISmu.h"

#include <memory>
#include <string>
#include <vector>

namespace firedog {

class Keithley6482 final : public ISmu {
public:
    using Config = SmuConfig;

    explicit Keithley6482(Config cfg);
    ~Keithley6482() override;

    Keithley6482(const Keithley6482&) = delete;
    Keithley6482& operator=(const Keithley6482&) = delete;

    bool Connect() override;   // open VISA + *IDN? + init (reset/NPLC/range/dual read)
    void Disconnect() override;
    bool IsConnected() const override;

    bool SetVoltageBias(double volts) override;   // :SOUR:VOLT + :OUTP ON
    bool OutputOff() override;

    // ---- Runtime measurement settings (safe against the poll loop: SCPI goes
    // out under the same VISA mutex the :READ? transactions hold) ----
    bool SetNplc(double nplc) override;        // both channels; governs the sample rate
    bool SetRangeAuto() override;              // :SENSn:CURR:RANG:AUTO ON
    bool SetRangeFixed(double amps) override;  // fixed range, e.g. 2e-6 for 2 uA

    // ---- Cheap state reads for the UI (never blocks on VISA) ----
    double BiasVolts() const override;         // last applied bias (0 until set)
    bool   OutputOn() const override;
    Config GetConfig() const override;         // reflects runtime NPLC/range changes

    // Poll both channels every intervalMs and publish to the two DataBus topics.
    bool StartPolling(int intervalMs, const std::string& topicCh1,
                                      const std::string& topicCh2) override;
    void StopPolling() override;
    bool IsPolling() const override;

    std::string DeviceId() const override;
    std::string LastError() const override;
    std::string ModelName() const override { return "Keithley 6482"; }

private:
    struct Impl;
    std::unique_ptr<Impl> p_;
};

} // namespace firedog
