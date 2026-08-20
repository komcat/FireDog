// Keithley2602B.h : layer 1 — direct VISA driver for the Keithley 2602B
// dual-channel SourceMeter SMU, over LAN.
//
// The other ISmu (see ISmu.h). It answers the same questions as the 6482 —
// two channels of current on the DataBus, a switchable bias, NPLC and range —
// but almost nothing about the wire is the same:
//
//   * TSP, not SCPI. The 2600B series has no SCPI mode: you send Lua that runs
//     ON the instrument ("smua.measure.nplc = 0.1"), and you read a value back
//     only because you asked it to print() one. There is no ":READ?".
//   * LAN, not GPIB/USB. "TCPIP0::<ip>::INSTR" (VXI-11) or
//     "TCPIP0::<ip>::5025::SOCKET" (raw). Both work — the driver enables the
//     '\n' termchar, which is what a SOCKET session needs to end a read.
//   * It is a SOURCE-measure unit. A 6482 ammeter measures with its V source
//     off; an SMU measures nothing at all until its output is ON. So
//     SetVoltageBias enables EVERY enabled channel — ch1 at the bias voltage,
//     ch2 at 0 V — and OutputOff drops them all. A channel with its output off
//     reads ~0 A, which looks exactly like a dark photodiode.
//
// PIMPL keeps <visa.h> out of this header; without HAS_VISA it compiles the
// same simulated dual-channel stub shape as the 6482.
//
// NPLC on the 2600B is 0.001..25 (the 6482 allows more) — SetNplc clamps into
// that window rather than letting the instrument reject the line silently.
#pragma once

#include "hardware/instruments/ISmu.h"

#include <memory>
#include <string>
#include <vector>

namespace firedog {

class Keithley2602B final : public ISmu {
public:
    using Config = SmuConfig;

    // Instrument limits (2600B series). Public so the UI can clamp its edits
    // to the same window the driver enforces.
    static constexpr double kMinNplc = 0.001;
    static constexpr double kMaxNplc = 25.0;

    explicit Keithley2602B(Config cfg);
    ~Keithley2602B() override;

    Keithley2602B(const Keithley2602B&) = delete;
    Keithley2602B& operator=(const Keithley2602B&) = delete;

    bool Connect() override;   // open VISA + *IDN? + smuX.reset() + measurement setup
    void Disconnect() override;
    bool IsConnected() const override;

    // Bias ch1, source 0 V on ch2, and switch every enabled channel's output ON
    // (an SMU cannot measure with the output off). Verified via smuX.source.output.
    bool SetVoltageBias(double volts) override;
    bool OutputOff() override;

    // ---- Runtime measurement settings (safe against the poll loop: the TSP
    // goes out under the same VISA mutex the measure transaction holds) ----
    bool SetNplc(double nplc) override;        // clamped to [kMinNplc, kMaxNplc]
    bool SetRangeAuto() override;              // smuX.measure.autorangei = AUTORANGE_ON
    bool SetRangeFixed(double amps) override;  // smuX.measure.rangei

    // ---- Cheap state reads for the UI (never blocks on VISA) ----
    double BiasVolts() const override;
    bool   OutputOn() const override;          // true only if EVERY enabled channel is on
    Config GetConfig() const override;

    bool StartPolling(int intervalMs, const std::string& topicCh1,
                                      const std::string& topicCh2) override;
    void StopPolling() override;
    bool IsPolling() const override;

    std::string DeviceId() const override;
    std::string LastError() const override;
    std::string ModelName() const override { return "Keithley 2602B"; }

private:
    struct Impl;
    std::unique_ptr<Impl> p_;
};

} // namespace firedog
