// CLD101x.h : layer 1 — direct NI-VISA driver for the Thorlabs CLD101x
// compact laser-diode + TEC controller (no adapter/manager/Qt layers).
//
// SCPI ported from bobcat_engine's CLD101xDirectAdapter (the battle-tested
// source): USB resource "USB0::0x1313::0x804F::<serial>::INSTR", lowercase
// Thorlabs SCPI (source1:current / source2:temperature / output1|2:state /
// sense2|3 ... data?). PIMPL keeps <visa.h> out of this header. Without
// HAS_VISA it compiles a simulated stub (temperature settles toward the
// setpoint; laser current follows the setpoint when on).
//
// Units follow bobcat's convention: laser current in mA at this API (converted
// to amps on the wire), temperature in degrees C.
#pragma once

#include <memory>
#include <string>

namespace firedog {

class CLD101x {
public:
    struct Config {
        std::string name = "laser-main";
        std::string visaResource = "USB0::0x1313::0x804F::M00930341::INSTR";
        double defaultTemperature    = 25.0;   // UI seed values — NOT pushed to
        double defaultLaserCurrentMA = 50.0;   // the device at connect
    };

    explicit CLD101x(Config cfg);
    ~CLD101x();

    CLD101x(const CLD101x&) = delete;
    CLD101x& operator=(const CLD101x&) = delete;

    // Open VISA + *CLS + *IDN? + read the device's live states/setpoints.
    // Nothing is written to the outputs at connect (bobcat behavior).
    bool Connect();
    // Close the session. Deliberately does NOT switch the laser or TEC off —
    // matches bobcat: device state survives an app restart.
    void Disconnect();
    bool IsConnected() const;

    // ---- TEC (output2 / source2 / sense2) ----
    bool TecOn();                        // verified via output2:state? read-back
    bool TecOff();
    bool SetTemperature(double celsius); // source2:temperature:spoint

    // ---- Laser (output1 / source1 / sense3) ----
    bool LaserOn();                      // verified via output1:state? read-back
    bool LaserOff();
    bool SetLaserCurrentMA(double mA);   // source1:current:level:amplitude (A on wire)

    // One consistent snapshot for the UI (cheap; refreshed by the poll thread
    // and the verified on/off commands — never blocks on VISA).
    struct Status {
        bool   laserOn    = false;
        bool   tecOn      = false;
        double tempC      = 0.0;   // actual temperature (meaningful iff tempValid)
        bool   tempValid  = false; // false when the sensor returns the SCPI NaN
                                   // sentinel (9.9e37) — e.g. thermistor open
        double tempSetC   = 0.0;   // setpoint
        double laserSetMA = 0.0;   // setpoint
        double laserActMA = 0.0;   // measured (0 while laser off)
        bool   valid      = false; // false until the first successful poll
    };
    Status GetStatus() const;

    // Poll temperature + laser current every intervalMs (<=0 -> 500 ms) and
    // publish them to the DataBus topics (deg C / mA). Empty topic = skip.
    bool StartPolling(int intervalMs, const std::string& topicTemp,
                      const std::string& topicCurrent);
    void StopPolling();
    bool IsPolling() const;

    std::string DeviceId() const;
    std::string LastError() const;

private:
    struct Impl;
    std::unique_ptr<Impl> p_;
};

} // namespace firedog
