// Keithley6482.h : layer 1 — direct VISA driver for the Keithley 6482
// dual-channel picoammeter + voltage source (no ISMUDevice/adapter/Qt).
//
// SCPI sequence ported from bobcat_engine's Keithley6482Adapter. PIMPL keeps
// <visa.h> out of this header. A background poll thread reads both channels
// and publishes each to a DataBus topic. Without HAS_VISA it compiles a
// simulated dual-channel stub so the pub-sub + chart pipeline still runs.
#pragma once

#include <memory>
#include <string>
#include <vector>

namespace firedog {

class Keithley6482 {
public:
    struct Config {
        std::string      name = "Keithley_6482";
        std::string      visaResource = "GPIB0::1::INSTR";
        int              channelCount = 2;
        std::vector<int> enabledChannels{1, 2};
        double           nplc = 5.0;
        bool             autoRange = true;
        double           fixedRange = 0.0;
    };

    explicit Keithley6482(Config cfg);
    ~Keithley6482();

    Keithley6482(const Keithley6482&) = delete;
    Keithley6482& operator=(const Keithley6482&) = delete;

    bool Connect();        // open VISA + *IDN? + init (reset/NPLC/range/dual read)
    void Disconnect();
    bool IsConnected() const;

    bool SetVoltageBias(double volts);   // :SOUR:VOLT + :OUTP ON
    bool OutputOff();

    // Poll both channels every intervalMs and publish to the two DataBus topics.
    bool StartPolling(int intervalMs, const std::string& topicCh1, const std::string& topicCh2);
    void StopPolling();
    bool IsPolling() const;

    std::string DeviceId() const;
    std::string LastError() const;

private:
    struct Impl;
    std::unique_ptr<Impl> p_;
};

} // namespace firedog
