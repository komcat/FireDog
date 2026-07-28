// UDP3305S.h : layer 1 — direct NI-VISA driver for the UNI-T UDP3305S(-E)
// 3-channel programmable DC power supply (no adapter/manager/Qt layers).
//
// SCPI ported from bobcat_engine's UnitTUDP3305S (the battle-tested source):
// USB resource "USB0::0x0483::0x5740::<serial>::INSTR" (UNI-T enumerates with
// the STMicro VID 0x0483), '\n' termchar, 5 s I/O timeout, 200 ms USB settle
// after open. Hardware-verified commands (docs/UDP3305S/test_channels.py):
//   APPLy CH<n>,<volt>,<curr>     program V+I in one shot
//   SOURce<n>:VOLTage? / :CURRent?   read programmed setpoints
//   OUTPut CH<n>,ON|OFF / OUTPut? CH<n>
//   MEASure:VOLTage? CH<n> / MEASure:CURRent? CH<n>
// Output switches are read-back verified (OUTPut? CH<n>) like the Keithley/CLD.
// PIMPL keeps <visa.h> out of this header; without HAS_VISA it compiles a
// simulated stub (measured V settles toward the setpoint when the output is on).
//
// Channel specs (datasheet): CH1/CH2 0..32 V / 0..5 A, CH3 0..6 V / 0..3 A.
//
// Nothing is written to the outputs at CONNECT (device state persists, like
// the CLD101x). DISCONNECT forces every output OFF first — that is bobcat's
// deliberate PSU safety habit and differs from the CLD on purpose.
#pragma once

#include <memory>
#include <string>

namespace firedog {

class UDP3305S {
public:
    static constexpr int kMaxChannels = 3;

    struct Config {
        std::string name = "PSU3";
        std::string visaResource;      // e.g. "USB0::0x0483::0x5740::<serial>::INSTR"
        int         channels = 3;      // active channels (1..kMaxChannels)
        int         pollIntervalMs = 500;
    };

    explicit UDP3305S(Config cfg);
    ~UDP3305S();

    UDP3305S(const UDP3305S&) = delete;
    UDP3305S& operator=(const UDP3305S&) = delete;

    // Open VISA + *IDN? + read each channel's live output state and setpoints.
    // Nothing is written to the outputs at connect.
    bool Connect();
    // Forces every output OFF (bobcat safety habit), then closes the session.
    void Disconnect();
    bool IsConnected() const;

    // Channels are 1-based (1..Channels()).
    bool OutputOn(int channel);                          // verified via OUTPut? read-back
    bool OutputOff(int channel);
    bool Apply(int channel, double volts, double amps);  // APPLy CH<n>,<v>,<i> + setpoint read-back

    struct ChannelStatus {
        bool   outputOn = false;
        double setV  = 0.0;   // programmed setpoints (read back from the device)
        double setI  = 0.0;
        double measV = 0.0;   // measured (poll thread; meaningful iff Status::valid)
        double measI = 0.0;
    };
    struct Status {
        ChannelStatus ch[kMaxChannels];
        bool          valid = false;   // false until the first successful poll
    };
    // One consistent snapshot for the UI (cheap; never blocks on VISA).
    Status GetStatus() const;

    // Poll measured V/I (+ output state, so front-panel changes surface) every
    // intervalMs (<=0 -> 500 ms) and publish to topics::PsuVoltage/PsuCurrent.
    bool StartPolling(int intervalMs);
    void StopPolling();
    bool IsPolling() const;

    int         Channels() const;
    std::string DeviceId() const;
    std::string LastError() const;

private:
    struct Impl;
    std::unique_ptr<Impl> p_;
};

} // namespace firedog
