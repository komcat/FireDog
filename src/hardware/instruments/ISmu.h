// ISmu.h : layer 1 — the source-measure-unit interface.
//
// The alignment pipeline does not care WHICH instrument produces the photo-
// current: it wants two channels of amps on the DataBus, a bias output it can
// switch, and NPLC/range control. Two very different instruments answer that:
//
//   Keithley 6482  — dual picoammeter + V source, SCPI, GPIB/USB.
//   Keithley 2602B — dual-channel SMU, TSP (Lua on the instrument), LAN.
//
// so this is the same "fleet behind an interface" shape as IMotionController /
// IIoDevice: Workflow builds ONE from config's `type`, HardwareManager owns it
// as a unique_ptr<ISmu>, and every caller above stays type-agnostic. Adding a
// third meter is a new .cpp plus one line in LaunchKeithleyConnect.
//
// Threading contract every implementation must honour: the poll thread and the
// runtime setters share one VISA mutex, so a setter is safe while measuring;
// the cheap UI reads (BiasVolts/OutputOn/GetConfig/IsPolling/IsConnected) must
// NEVER block on that mutex — a per-frame read cannot wait behind an in-flight
// measurement.
#pragma once

#include <string>
#include <vector>

namespace firedog {

// Shared config for every SMU. `type` selects the driver; the rest is the
// measurement setup, identical in meaning across instruments (nplc is line
// cycles per integration, fixedRange is full-scale amps).
struct SmuConfig {
    std::string      name = "Keithley_6482";
    std::string      type = "keithley_6482";
    std::string      visaResource = "GPIB0::1::INSTR";
    int              channelCount = 2;
    std::vector<int> enabledChannels{1, 2};
    double           nplc = 5.0;
    bool             autoRange = true;
    double           fixedRange = 0.0;
};

class ISmu {
public:
    using Config = SmuConfig;

    virtual ~ISmu() = default;

    virtual bool Connect() = 0;        // open VISA + identify + init
    virtual void Disconnect() = 0;     // stops polling, drops the output
    virtual bool IsConnected() const = 0;

    // Apply the bias voltage to channel 1 and enable the output. Implementations
    // verify the result against the instrument rather than assuming the write
    // landed, and return false when the instrument disagrees.
    virtual bool SetVoltageBias(double volts) = 0;
    virtual bool OutputOff() = 0;

    // ---- Runtime measurement settings (safe against the poll loop) ----
    virtual bool SetNplc(double nplc) = 0;        // governs the sample rate
    virtual bool SetRangeAuto() = 0;
    virtual bool SetRangeFixed(double amps) = 0;

    // ---- Cheap state reads for the UI (never block on the instrument) ----
    virtual double BiasVolts() const = 0;
    virtual bool   OutputOn() const = 0;
    virtual Config GetConfig() const = 0;         // reflects runtime edits

    // Poll both channels and publish each to its DataBus topic. ms <= 0 means
    // "as fast as the instrument allows" — NPLC, not this number, sets the rate.
    virtual bool StartPolling(int ms, const std::string& topicCh1,
                                      const std::string& topicCh2) = 0;
    virtual void StopPolling() = 0;
    virtual bool IsPolling() const = 0;

    virtual std::string DeviceId() const = 0;     // *IDN? reply, empty until connected
    virtual std::string LastError() const = 0;
    // Human-readable model for the UI header, e.g. "Keithley 6482". Fixed per
    // driver — this is what the page shows before anything is connected.
    virtual std::string ModelName() const = 0;
};

} // namespace firedog
