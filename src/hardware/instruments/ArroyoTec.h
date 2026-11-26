// ArroyoTec.h : layer 1 - Arroyo Instruments TECSource temperature controller
// (5240 today; the 5300/5400 series speak the same set) over a serial port.
// Commands per Arroyo's Computer Interfacing Manual, every one of them checked
// against the instrument on this machine (2026-11-25, a 5240 v1.56 on COM8):
//
//   *IDN? *CLS ERRSTR? LOCAL
//   TEC:T <c>      set point (refused by the unit outside TLO..THI)
//   TEC:OUT 0|1    output        TEC:OUT?  read back
//   TEC:T? TEC:SET:T? TEC:ITE? TEC:V? TEC:COND? TEC:MODE? TEC:TOL?
//   TEC:LIM:ITE? TEC:LIM:THI? TEC:LIM:TLO?
//
// Four facts that are easy to get wrong:
//  - The USB connection is a virtual COM port (FTDI) and its baud is FIXED at
//    38400, no flow control. RS-232 takes the menu's rate (factory 9600).
//  - A command the unit does not know gets NO reply at all - only an E-123 in
//    its error queue. So a write is checked by reading ERRSTR? afterwards, and
//    a query that times out has dropped the stream out of step.
//  - There is no "stable?" query. Stability is TEC:COND? bit 9 (out of
//    tolerance, judged against TEC:TOL - 0.1 C for 5 s by default) clear while
//    bit 10 (output on) is set: exactly the front panel's Stable LED.
//  - A shorted or open sensor reads -273.15 C. That is a fault flag (COND bits
//    5 / 6), never a temperature: tempValid goes false and nothing is published.
//
// Talking to the unit puts it in REMOTE (front panel locked). Disconnect sends
// LOCAL, so the knob works again, and leaves the OUTPUT as it is - a TEC
// holding a part at temperature must survive an application restart (the
// CLD101x habit, not the PSU's).
//
// A dropped link (USB pulled, a timeout) closes the port; the poll thread
// reopens it (the PD3 pattern), and Status::linkUp says which side you are on.
// Win32 serial behind the PIMPL, so there is no SDK gate; Config::simulated
// selects an in-memory controller that answers the same strings.
#pragma once

#include <memory>
#include <string>

namespace firedog {

class ArroyoTec {
public:
    struct Config {
        std::string name = "TEC1";
        std::string port = "COM1";
        int         baud = 38400;
        int         pollIntervalMs = 500;
        bool        simulated = false;
    };

    // TEC:COND? bits (Computer Interfacing Manual, "TEC Condition Status Register").
    enum CondBit : unsigned {
        kCondCurrentLimit = 1u << 0,
        kCondVoltageLimit = 1u << 1,
        kCondRLimit       = 1u << 2,
        kCondTHighLimit   = 1u << 3,
        kCondTLowLimit    = 1u << 4,
        kCondSensorShort  = 1u << 5,
        kCondSensorOpen   = 1u << 6,
        kCondTecOpen      = 1u << 7,
        kCondOutOfTol     = 1u << 9,
        kCondOutputOn     = 1u << 10,
        kCondRunaway      = 1u << 12,
    };
    // The set condition bits as words ("sensor shorted, out of tolerance"), "" for none.
    static std::string CondText(unsigned cond);

    explicit ArroyoTec(Config cfg);
    ~ArroyoTec();

    ArroyoTec(const ArroyoTec&) = delete;
    ArroyoTec& operator=(const ArroyoTec&) = delete;

    // Open the port, *CLS, *IDN? (must answer "Arroyo ..."), then read the
    // set point, output, mode, limits and tolerance. Nothing is written to the
    // output at connect.
    bool Connect();
    // Stops polling, sends LOCAL, closes the port. The output is left as it is.
    void Disconnect();
    bool IsConnected() const;

    // ---- Blocking setters (worker threads: recipes) -------------------------
    // Refused before sending outside the unit's TLO..THI (read at connect);
    // checked with ERRSTR? and read back with TEC:SET:T?.
    bool SetTemperature(double celsius);
    // TEC:OUT then TEC:OUT? read back. An ON the unit refuses or drops at once
    // (a bad sensor, the interlock) fails with the unit's error text.
    bool SetOutput(bool on);

    // ---- Non-blocking setters (UI thread) -----------------------------------
    // Queued; the poll thread sends them within a few milliseconds. The latest
    // request of each kind wins. The UI never does serial I/O.
    void RequestTemperature(double celsius);
    void RequestOutput(bool on);

    struct Status {
        bool        valid     = false;   // false until the first successful poll
        bool        linkUp    = false;   // port open right now
        unsigned long long pollSeq = 0;  // +1 per completed poll (a wait's freshness check)
        bool        outputOn  = false;
        double      tempC     = 0.0;     // meaningful iff tempValid
        bool        tempValid = false;   // false on a shorted / open sensor
        double      setC      = 0.0;
        double      currentA  = 0.0;
        double      voltageV  = 0.0;
        unsigned    cond      = 0;       // TEC:COND? as read
        bool        stable    = false;   // output on AND in tolerance (the Stable LED)
        std::string mode;                // "T" | "R" | "ITE"
        double      limItA    = 0.0;     // current limit
        double      limTHiC   = 0.0;
        double      limTLoC   = 0.0;
        double      tolC      = 0.0;     // tolerance window
        double      tolS      = 0.0;     // ... held for this long
        std::string lastUnitError;       // the unit's last reported error ("" = none since connect)
    };
    // Cached; never touches the port.
    Status GetStatus() const;

    // A fresh read NOW under the port lock - not the poll cache, which can be a
    // whole interval old. What tec_startup verifies against. `unitErrors` is
    // what ERRSTR? held ("" = none; reading it empties the unit's queue), so a
    // trip comes with its reason. False when the link is down or a query failed.
    struct Reading {
        double      tempC     = 0.0;
        bool        tempValid = false;
        double      currentA  = 0.0;
        double      voltageV  = 0.0;
        unsigned    cond      = 0;
        std::string unitErrors;
    };
    bool Measure(Reading& out);

    // Read-back every intervalMs (<=0 -> 500 ms), publish topics::Tec*(name), drain
    // the Request* queue, reopen a dropped link, log the unit's errors.
    bool StartPolling(int intervalMs);
    void StopPolling();
    bool IsPolling() const;

    // One synchronous read-back. For the rig.
    bool PollOnce();

    std::string DeviceId() const;
    std::string LastError() const;
    const Config& GetConfig() const;

private:
    struct Impl;
    std::unique_ptr<Impl> p_;
};

} // namespace firedog
