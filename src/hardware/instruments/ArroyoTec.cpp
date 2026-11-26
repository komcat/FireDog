// ArroyoTec.cpp
//
// Plain Win32 serial I/O (no SDK). One line out ("<cmd>\r\n"), one line back for
// a query; a setting command has no reply, so it is followed by ERRSTR? to learn
// whether the unit took it. See the header for the command set and the facts
// that shaped this file.
#include "hardware/instruments/ArroyoTec.h"

#include "core/DataBus.h"
#include "core/Log.h"
#include "core/Topics.h"

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>

#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace firedog {
namespace {

constexpr int         kReplyTimeoutMs = 600;   // the unit answers in ~15 ms
constexpr DWORD       kWriteTimeoutMs = 500;
constexpr std::size_t kMaxReplyLength = 512;
constexpr int         kOutputSettleMs = 150;   // a bad sensor trips the output this fast
// The unit holds the set point to 0.01 C on a 5240 (0.001 C on a 5400); a read
// back further off than that is not the number that was sent.
constexpr double      kSetpointReadbackTol = 0.011;

enum class Xfer { Ok, Link };

std::string Trim(std::string s)
{
    while (!s.empty() && std::isspace(static_cast<unsigned char>(s.back()))) s.pop_back();
    std::size_t i = 0;
    while (i < s.size() && std::isspace(static_cast<unsigned char>(s[i]))) ++i;
    return s.substr(i);
}

std::string Upper(std::string s)
{
    for (char& c : s) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    return s;
}

bool ParseDouble(const std::string& s, double& v)
{
    const std::string t = Trim(s);
    if (t.empty()) return false;
    char* end = nullptr;
    v = std::strtod(t.c_str(), &end);
    return end != t.c_str() && *end == '\0';
}

std::string Fmt(const char* f, double a)
{
    char b[64];
    std::snprintf(b, sizeof(b), f, a);
    return b;
}

std::string WinError(DWORD e)
{
    switch (e) {
        case ERROR_FILE_NOT_FOUND: return "no such port";
        case ERROR_ACCESS_DENIED:  return "port in use by another program";
        default:                   return "Windows error " + std::to_string(e);
    }
}

struct UnitError {
    int         code = 0;
    std::string text;
};

// ERRSTR? -> "0" or `402,"Sensor open, output turned off",123,"Path not found"`.
std::vector<UnitError> ParseErrors(const std::string& r)
{
    std::vector<UnitError> out;
    std::size_t i = 0;
    while (i < r.size()) {
        while (i < r.size() && (r[i] == ',' || r[i] == ' ')) ++i;
        if (i >= r.size()) break;
        char* end = nullptr;
        const long code = std::strtol(r.c_str() + i, &end, 10);
        if (end == r.c_str() + i) break;   // not a number: stop, keep what parsed
        i = static_cast<std::size_t>(end - r.c_str());
        UnitError e;
        e.code = static_cast<int>(code);
        while (i < r.size() && (r[i] == ',' || r[i] == ' ')) ++i;
        if (i < r.size() && r[i] == '"') {
            const std::size_t close = r.find('"', i + 1);
            e.text = r.substr(i + 1, close == std::string::npos ? std::string::npos : close - i - 1);
            i = close == std::string::npos ? r.size() : close + 1;
        }
        if (e.code != 0) out.push_back(e);
    }
    return out;
}

std::string ErrorsText(const std::vector<UnitError>& v)
{
    std::string s;
    for (const UnitError& e : v) {
        if (!s.empty()) s += "; ";
        char b[16];
        std::snprintf(b, sizeof(b), "E-%03d", e.code);
        s += b;
        if (!e.text.empty()) s += " " + e.text;
    }
    return s;
}

} // namespace

std::string ArroyoTec::CondText(unsigned cond)
{
    static const struct { unsigned bit; const char* text; } kBits[] = {
        {kCondCurrentLimit, "current limit"},
        {kCondVoltageLimit, "voltage limit"},
        {kCondRLimit,       "sensor (R) limit"},
        {kCondTHighLimit,   "high temperature limit"},
        {kCondTLowLimit,    "low temperature limit"},
        {kCondSensorShort,  "sensor shorted"},
        {kCondSensorOpen,   "sensor open"},
        {kCondTecOpen,      "TEC module open"},
        {kCondOutOfTol,     "out of tolerance"},
        {kCondOutputOn,     "output on"},
        {kCondRunaway,      "thermal run-away"},
    };
    std::string s;
    for (const auto& b : kBits) {
        if (!(cond & b.bit)) continue;
        if (!s.empty()) s += ", ";
        s += b.text;
    }
    return s;
}

struct ArroyoTec::Impl {
    Config cfg;

    std::atomic<bool> connected{false};   // a session the caller opened (survives a link drop)
    std::atomic<bool> polling{false};
    std::thread       pollThread;
    int               intervalMs = 500;

    std::mutex ioMu;                         // port lifecycle + every transaction
    HANDLE     port = INVALID_HANDLE_VALUE;  // ioMu
    bool       linkFailing = false;          // ioMu - log a dropped link once per streak

    // The in-memory controller (ioMu). Answers the same strings the unit does.
    struct Sim {
        double tempC = 23.0, setC = 25.0, iteA = 0.0, vteV = 0.0;
        bool   out = false;
        double tolC = 0.10, tolS = 5.0, limI = 3.0, thi = 125.0, tlo = -99.0;
        std::chrono::steady_clock::time_point last{}, inTolSince{};
        bool   inTol = false;
        std::vector<UnitError> errs;
    } sim;

    mutable std::mutex statusMu;
    Status             status;
    std::string        lastError;
    std::string        deviceId;

    std::mutex              reqMu;   // UI requests -> poll thread
    std::condition_variable reqCv;
    std::optional<double>   reqTemp;
    std::optional<bool>     reqOut;

    DataStream* sTemp = nullptr;
    DataStream* sCurr = nullptr;
    DataStream* sVolt = nullptr;

    explicit Impl(Config c) : cfg(std::move(c)) {}
    ~Impl() { StopPolling(); Disconnect(); }

    void SetError(const std::string& e)
    {
        std::lock_guard<std::mutex> l(statusMu);
        lastError = e;
    }
    std::string Error() const
    {
        std::lock_guard<std::mutex> l(statusMu);
        return lastError;
    }
    Status GetStatus() const
    {
        std::lock_guard<std::mutex> l(statusMu);
        return status;
    }

    // ---- port layer (ioMu held) ---------------------------------------------

    bool LinkUp() const { return cfg.simulated ? connected.load() : port != INVALID_HANDLE_VALUE; }

    void SetLinkStatus()
    {
        std::lock_guard<std::mutex> l(statusMu);
        status.linkUp = LinkUp();
    }

    void ClosePort()
    {
        if (port == INVALID_HANDLE_VALUE) return;
        CloseHandle(port);
        port = INVALID_HANDLE_VALUE;
    }

    bool OpenPort()
    {
        ClosePort();
        const std::string path = "\\\\.\\" + cfg.port;   // COM10+ needs the device namespace
        HANDLE h = CreateFileA(path.c_str(), GENERIC_READ | GENERIC_WRITE, 0, nullptr,
                               OPEN_EXISTING, 0, nullptr);
        if (h == INVALID_HANDLE_VALUE) {
            SetError("cannot open " + cfg.port + ": " + WinError(GetLastError()));
            return false;
        }

        DCB dcb{};
        dcb.DCBlength = sizeof(dcb);
        if (!GetCommState(h, &dcb)) {
            SetError("cannot read the settings of " + cfg.port + ": " + WinError(GetLastError()));
            CloseHandle(h);
            return false;
        }
        // 8N1, NO flow control - the manual: with flow control on, the unit
        // never gets to send its replies.
        dcb.BaudRate        = static_cast<DWORD>(cfg.baud);
        dcb.ByteSize        = 8;
        dcb.Parity          = NOPARITY;
        dcb.StopBits        = ONESTOPBIT;
        dcb.fBinary         = TRUE;
        dcb.fParity         = FALSE;
        dcb.fOutxCtsFlow    = FALSE;
        dcb.fOutxDsrFlow    = FALSE;
        dcb.fDtrControl     = DTR_CONTROL_ENABLE;
        dcb.fDsrSensitivity = FALSE;
        dcb.fOutX           = FALSE;
        dcb.fInX            = FALSE;
        dcb.fNull           = FALSE;
        dcb.fRtsControl     = RTS_CONTROL_ENABLE;
        dcb.fAbortOnError   = FALSE;
        if (!SetCommState(h, &dcb)) {
            SetError("cannot set " + cfg.port + " to " + std::to_string(cfg.baud) + " 8N1: " +
                     WinError(GetLastError()));
            CloseHandle(h);
            return false;
        }

        // A read returns as soon as bytes are there, else after 50 ms; ReadLine
        // loops on that up to kReplyTimeoutMs.
        COMMTIMEOUTS t{};
        t.ReadIntervalTimeout         = MAXDWORD;
        t.ReadTotalTimeoutMultiplier  = MAXDWORD;
        t.ReadTotalTimeoutConstant    = 50;
        t.WriteTotalTimeoutMultiplier = 0;
        t.WriteTotalTimeoutConstant   = kWriteTimeoutMs;
        SetCommTimeouts(h, &t);
        PurgeComm(h, PURGE_RXCLEAR | PURGE_TXCLEAR);
        port = h;
        return true;
    }

    // One reply line. CR or LF ends it (the unit's Msg Term may be CR/LF, CR or
    // LF); an empty line is skipped, so a trailing LF never reads as a reply.
    bool ReadLine(std::string& out)
    {
        out.clear();
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(kReplyTimeoutMs);
        char buf[64];
        while (std::chrono::steady_clock::now() < deadline) {
            DWORD n = 0;
            if (!ReadFile(port, buf, sizeof(buf), &n, nullptr)) return false;
            for (DWORD i = 0; i < n; ++i) {
                const char c = buf[i];
                if (c == '\r' || c == '\n') {
                    if (!out.empty()) return true;
                    continue;
                }
                out.push_back(c);
                if (out.size() > kMaxReplyLength) return false;
            }
        }
        return false;
    }

    // Sends one command; with `reply` reads one line back. A Link failure has
    // already closed the port.
    Xfer Transact(const std::string& cmd, std::string* reply)
    {
        if (cfg.simulated) {
            if (!connected.load()) { SetError("not connected"); return Xfer::Link; }
            const std::string r = SimReply(cmd);
            if (!reply) return Xfer::Ok;
            if (r.empty()) return LinkFail("'" + cmd + "': no reply");
            *reply = r;
            return Xfer::Ok;
        }
        if (port == INVALID_HANDLE_VALUE) { SetError("not connected"); return Xfer::Link; }

        PurgeComm(port, PURGE_RXCLEAR);   // a late byte must not read as this reply
        const std::string line = cmd + "\r\n";
        DWORD written = 0;
        if (!WriteFile(port, line.data(), static_cast<DWORD>(line.size()), &written, nullptr) ||
            written != line.size())
            return LinkFail("'" + cmd + "': write failed (" + WinError(GetLastError()) + ")");
        if (!reply) return Xfer::Ok;
        if (!ReadLine(*reply))
            return LinkFail("'" + cmd + "': no reply (timeout, or the link is gone)");
        *reply = Trim(*reply);
        return Xfer::Ok;
    }

    // A query that got no answer means the stream is out of step: drop it.
    Xfer LinkFail(const std::string& what)
    {
        SetError(what);
        if (!linkFailing) {
            linkFailing = true;
            FD_ERROR("tec", "%s: %s - closing the port", cfg.name.c_str(), what.c_str());
        }
        ClosePort();
        SetLinkStatus();
        return Xfer::Link;
    }

    bool Query(const std::string& cmd, std::string& r) { return Transact(cmd, &r) == Xfer::Ok; }

    bool QueryDouble(const std::string& cmd, double& v)
    {
        std::string r;
        if (!Query(cmd, r)) return false;
        if (!ParseDouble(r, v)) {
            LinkFail("'" + cmd + "': unexpected reply '" + r + "'");
            return false;
        }
        return true;
    }

    // ERRSTR? - reading it also empties the unit's queue. Every error is logged
    // here, once; the last one is kept for the page.
    bool ReadErrors(std::vector<UnitError>& out)
    {
        std::string r;
        if (!Query("ERRSTR?", r)) return false;
        out = ParseErrors(r);
        if (!out.empty()) {
            const std::string text = ErrorsText(out);
            FD_WARN("tec", "%s: controller reports %s", cfg.name.c_str(), text.c_str());
            std::lock_guard<std::mutex> l(statusMu);
            status.lastUnitError = text;
        }
        return true;
    }

    // A setting command has no reply; the error queue says whether it was
    // taken. A code below 400 (E-1xx parse, E-2xx range) is a refusal of THIS
    // command. A 4xx and up is something the unit did meanwhile (an output
    // trip) - logged by ReadErrors, not blamed on the write.
    bool WriteChecked(const std::string& cmd, std::string& why)
    {
        if (Transact(cmd, nullptr) != Xfer::Ok) { why = Error(); return false; }
        std::vector<UnitError> errs;
        if (!ReadErrors(errs)) { why = Error(); return false; }
        std::vector<UnitError> refusals;
        for (const UnitError& e : errs)
            if (e.code > 0 && e.code < 400) refusals.push_back(e);
        if (!refusals.empty()) {
            why = "'" + cmd + "' refused by the controller: " + ErrorsText(refusals);
            return false;
        }
        return true;
    }

    // ---- the in-memory controller (ioMu held) -------------------------------

    void SimStep()
    {
        const auto now = std::chrono::steady_clock::now();
        if (sim.last == std::chrono::steady_clock::time_point{}) sim.last = now;
        const double dt = std::chrono::duration<double>(now - sim.last).count();
        sim.last = now;
        if (sim.out) {
            sim.tempC += (sim.setC - sim.tempC) * std::min(1.0, dt * 0.4);
            sim.iteA = std::clamp((sim.setC - sim.tempC) * 0.8, -sim.limI, sim.limI);
        } else {
            sim.tempC += (23.0 - sim.tempC) * std::min(1.0, dt * 0.05);
            sim.iteA = 0.0;
        }
        sim.vteV = sim.iteA * 1.8;
        const bool in = sim.out && std::fabs(sim.tempC - sim.setC) <= sim.tolC;
        if (in && !sim.inTol) sim.inTolSince = now;
        sim.inTol = in;
    }

    std::string SimReply(const std::string& raw)
    {
        SimStep();
        const std::string cmd = Upper(Trim(raw));
        const auto num = [](double v, const char* f) { return Fmt(f, v); };
        const auto arg = [&cmd](double& v) {
            const std::size_t sp = cmd.find(' ');
            return sp != std::string::npos && ParseDouble(cmd.substr(sp + 1), v);
        };
        double v = 0.0;

        if (cmd == "*CLS")        { sim.errs.clear(); return ""; }
        if (cmd == "LOCAL")       return "";
        if (cmd == "*IDN?")       return "Arroyo 5240 SIMULATED v1.56";
        if (cmd == "ERRSTR?") {
            if (sim.errs.empty()) return "0";
            std::string s;
            for (const UnitError& e : sim.errs) {
                if (!s.empty()) s += ",";
                s += std::to_string(e.code) + ",\"" + e.text + "\"";
            }
            sim.errs.clear();
            return s;
        }
        if (cmd == "TEC:T?")       return num(sim.tempC, "%.3f");
        if (cmd == "TEC:SET:T?")   return num(sim.setC, "%.2f");
        if (cmd == "TEC:ITE?")     return num(sim.iteA, "%.2f");
        if (cmd == "TEC:V?")       return num(sim.vteV, "%.2f");
        if (cmd == "TEC:OUT?")     return sim.out ? "1" : "0";
        if (cmd == "TEC:MODE?")    return "T";
        if (cmd == "TEC:LIM:ITE?") return num(sim.limI, "%.2f");
        if (cmd == "TEC:LIM:THI?") return num(sim.thi, "%.1f");
        if (cmd == "TEC:LIM:TLO?") return num(sim.tlo, "%.1f");
        if (cmd == "TEC:TOL?")     return num(sim.tolC, "%.2f") + "," + num(sim.tolS, "%.1f");
        if (cmd == "TEC:COND?") {
            const bool stable = sim.inTol &&
                std::chrono::duration<double>(std::chrono::steady_clock::now() - sim.inTolSince).count()
                    >= sim.tolS;
            unsigned c = 0;
            if (sim.out) c |= kCondOutputOn;
            if (!stable) c |= kCondOutOfTol;
            if (std::fabs(sim.iteA) >= sim.limI) c |= kCondCurrentLimit;
            return std::to_string(c);
        }
        if (cmd.rfind("TEC:T ", 0) == 0 && arg(v)) {
            if (v < sim.tlo || v > sim.thi) sim.errs.push_back({201, "Out of range"});
            else { sim.setC = v; sim.inTol = false; }
            return "";
        }
        if (cmd.rfind("TEC:OUT ", 0) == 0 && arg(v)) {
            sim.out   = v != 0.0;
            sim.inTol = false;
            return "";
        }
        // Like the unit: an unknown command gets no reply, only a queued E-123.
        sim.errs.push_back({123, "Path not found"});
        return "";
    }

    // ---- reads (ioMu held) --------------------------------------------------

    // What cannot change while the unit is in remote (its menu is locked):
    // mode, limits, tolerance. Read at connect and after a reopened link.
    bool ReadSettingsLocked()
    {
        std::string mode, tol;
        double limI = 0.0, thi = 0.0, tlo = 0.0;
        if (!Query("TEC:MODE?", mode) || !QueryDouble("TEC:LIM:ITE?", limI) ||
            !QueryDouble("TEC:LIM:THI?", thi) || !QueryDouble("TEC:LIM:TLO?", tlo) ||
            !Query("TEC:TOL?", tol))
            return false;
        double tolC = 0.0, tolS = 0.0;
        const std::size_t comma = tol.find(',');
        if (comma != std::string::npos) {
            ParseDouble(tol.substr(0, comma), tolC);
            ParseDouble(tol.substr(comma + 1), tolS);
        }
        std::lock_guard<std::mutex> l(statusMu);
        status.mode    = mode;
        status.limItA  = limI;
        status.limTHiC = thi;
        status.limTLoC = tlo;
        status.tolC    = tolC;
        status.tolS    = tolS;
        return true;
    }

    Xfer PollLocked()
    {
        double t = 0.0, set = 0.0, ite = 0.0, vte = 0.0;
        std::string cond;
        if (!QueryDouble("TEC:T?", t) || !QueryDouble("TEC:SET:T?", set) ||
            !QueryDouble("TEC:ITE?", ite) || !QueryDouble("TEC:V?", vte) ||
            !Query("TEC:COND?", cond))
            return Xfer::Link;
        double condD = 0.0;
        if (!ParseDouble(cond, condD) || condD < 0.0)
            return LinkFail("'TEC:COND?': unexpected reply '" + cond + "'");
        std::vector<UnitError> errs;
        if (!ReadErrors(errs)) return Xfer::Link;

        const unsigned c = static_cast<unsigned>(condD);
        std::lock_guard<std::mutex> l(statusMu);
        status.cond      = c;
        status.outputOn  = (c & kCondOutputOn) != 0;
        status.tempC     = t;
        status.tempValid = !(c & (kCondSensorShort | kCondSensorOpen)) && t > -273.0;
        status.setC      = set;
        status.currentA  = ite;
        status.voltageV  = vte;
        status.stable    = status.outputOn && !(c & kCondOutOfTol);
        status.valid     = true;
        status.linkUp    = LinkUp();
        ++status.pollSeq;
        return Xfer::Ok;
    }

    // Reopen a dropped link (real hardware only). Called by the poll thread.
    bool EnsureLink()
    {
        if (cfg.simulated || port != INVALID_HANDLE_VALUE) return true;
        if (!OpenPort()) return false;
        std::string id;
        if (!Query("*IDN?", id)) return false;
        if (id.rfind("Arroyo", 0) != 0) {
            LinkFail(cfg.port + " now answers '" + id + "' - not an Arroyo controller");
            return false;
        }
        if (!ReadSettingsLocked()) return false;
        FD_INFO("tec", "%s: link on %s reopened", cfg.name.c_str(), cfg.port.c_str());
        linkFailing = false;
        SetLinkStatus();
        return true;
    }

    bool Ready()
    {
        if (!connected.load()) { SetError("not connected"); return false; }
        if (!LinkUp()) { SetError("link down - reconnecting"); return false; }
        return true;
    }

    // ---- device API ---------------------------------------------------------

    bool Connect()
    {
        std::lock_guard<std::mutex> lock(ioMu);
        if (connected.load()) return true;
        if (!cfg.simulated && !OpenPort()) {
            FD_ERROR("tec", "%s: %s", cfg.name.c_str(), Error().c_str());
            return false;
        }
        connected.store(true);
        linkFailing = true;   // a failure below is reported once, by Connect
        SetError("");

        // *CLS empties the error queue, so every error read from here on is new.
        std::string id;
        bool ok = Transact("*CLS", nullptr) == Xfer::Ok && Query("*IDN?", id);
        if (!ok) {
            SetError("no answer to *IDN? on " + cfg.port + " at " + std::to_string(cfg.baud) +
                     " baud - wrong port, or the controller is off?");
        } else if (id.rfind("Arroyo", 0) != 0) {
            SetError(cfg.port + " answered '" + id + "' - not an Arroyo controller");
            ok = false;
        }
        if (ok) {
            std::lock_guard<std::mutex> l(statusMu);
            deviceId = id;
        }
        // Read the real state - nothing is written to the output.
        ok = ok && ReadSettingsLocked() && PollLocked() == Xfer::Ok;
        if (!ok) {
            ClosePort();
            connected.store(false);
            SetLinkStatus();
            FD_ERROR("tec", "%s: connect failed: %s", cfg.name.c_str(), Error().c_str());
            return false;
        }
        linkFailing = false;

        const Status s = GetStatus();
        FD_INFO("tec", "%s connected on %s%s: %s (output %s, set %.2f C, mode %s, limits %.1f..%.1f C, %.2f A)",
                cfg.name.c_str(), cfg.port.c_str(), cfg.simulated ? " (simulated)" : "", id.c_str(),
                s.outputOn ? "ON" : "off", s.setC, s.mode.c_str(), s.limTLoC, s.limTHiC, s.limItA);
        if (s.mode != "T")
            FD_WARN("tec", "%s: controller is in %s mode, not T - a temperature set point does "
                    "nothing until it is switched to T mode on its front panel", cfg.name.c_str(),
                    s.mode.c_str());
        const unsigned faults = s.cond & (kCondSensorShort | kCondSensorOpen | kCondTecOpen | kCondRunaway);
        if (faults)
            FD_WARN("tec", "%s: %s - the output will not stay on", cfg.name.c_str(),
                    CondText(faults).c_str());
        return true;
    }

    void Disconnect()
    {
        StopPolling();
        std::lock_guard<std::mutex> lock(ioMu);
        if (!connected.load()) return;
        // Hand the front panel back. The output keeps its state.
        if (LinkUp()) Transact("LOCAL", nullptr);
        connected.store(false);
        ClosePort();
        SetLinkStatus();
        FD_INFO("tec", "%s disconnected (output left as it is)", cfg.name.c_str());
    }

    bool SetTemperature(double c)
    {
        std::lock_guard<std::mutex> lock(ioMu);
        if (!Ready()) return false;
        const Status s = GetStatus();
        if (s.limTHiC > s.limTLoC && (c < s.limTLoC || c > s.limTHiC)) {
            char b[160];
            std::snprintf(b, sizeof(b), "%.3f C is outside the controller's limits %.1f..%.1f C",
                          c, s.limTLoC, s.limTHiC);
            SetError(b);
            FD_WARN("tec", "%s: %s", cfg.name.c_str(), b);
            return false;
        }

        char cmd[48];
        std::snprintf(cmd, sizeof(cmd), "TEC:T %.3f", c);
        std::string why;
        if (!WriteChecked(cmd, why)) {
            SetError(why);
            FD_WARN("tec", "%s: %s", cfg.name.c_str(), why.c_str());
            return false;
        }
        double back = 0.0;
        if (!QueryDouble("TEC:SET:T?", back)) return false;
        {
            std::lock_guard<std::mutex> l(statusMu);
            status.setC = back;
        }
        if (std::fabs(back - c) > kSetpointReadbackTol) {
            char b[160];
            std::snprintf(b, sizeof(b), "set point reads back %.3f C, %.3f C was sent", back, c);
            SetError(b);
            FD_WARN("tec", "%s: %s", cfg.name.c_str(), b);
            return false;
        }
        FD_INFO("tec", "%s: set point %.3f C", cfg.name.c_str(), back);
        return true;
    }

    bool SetOutput(bool on)
    {
        std::lock_guard<std::mutex> lock(ioMu);
        if (!Ready()) return false;
        std::string why;
        if (!WriteChecked(on ? "TEC:OUT 1" : "TEC:OUT 0", why)) {
            SetError(why);
            FD_WARN("tec", "%s: %s", cfg.name.c_str(), why.c_str());
            return false;
        }
        // A bad sensor or an open interlock drops an ON at once: look again
        // after a moment, and take the unit's reason with the answer.
        if (on) std::this_thread::sleep_for(std::chrono::milliseconds(kOutputSettleMs));
        std::string r;
        if (!Query("TEC:OUT?", r)) return false;
        const bool actual = r == "1";
        std::vector<UnitError> errs;
        if (!ReadErrors(errs)) return false;
        {
            std::lock_guard<std::mutex> l(statusMu);
            status.outputOn = actual;
            if (!actual) status.stable = false;
        }
        if (actual != on) {
            std::string e = std::string("output ") + (on ? "ON" : "OFF") + " requested but it reads " +
                            (actual ? "ON" : "OFF");
            if (!errs.empty()) e += ": " + ErrorsText(errs);
            else if (on) {
                const unsigned faults = GetStatus().cond &
                    (kCondSensorShort | kCondSensorOpen | kCondTecOpen | kCondRunaway);
                if (faults) e += " (" + CondText(faults) + ")";
            }
            SetError(e);
            FD_WARN("tec", "%s: %s", cfg.name.c_str(), e.c_str());
            return false;
        }
        FD_INFO("tec", "%s: output %s (verified)", cfg.name.c_str(), on ? "ON" : "OFF");
        return true;
    }

    void RequestTemperature(double c)
    {
        {
            std::lock_guard<std::mutex> l(reqMu);
            reqTemp = c;
        }
        reqCv.notify_one();
    }
    void RequestOutput(bool on)
    {
        {
            std::lock_guard<std::mutex> l(reqMu);
            reqOut = on;
        }
        reqCv.notify_one();
    }

    bool Measure(Reading& r)
    {
        std::lock_guard<std::mutex> lock(ioMu);
        if (!Ready()) return false;
        double t = 0.0, i = 0.0, v = 0.0, condD = 0.0;
        std::string cond;
        if (!QueryDouble("TEC:T?", t) || !QueryDouble("TEC:ITE?", i) ||
            !QueryDouble("TEC:V?", v) || !Query("TEC:COND?", cond))
            return false;
        if (!ParseDouble(cond, condD) || condD < 0.0) {
            LinkFail("'TEC:COND?': unexpected reply '" + cond + "'");
            return false;
        }
        std::vector<UnitError> errs;
        if (!ReadErrors(errs)) return false;
        r.cond       = static_cast<unsigned>(condD);
        r.tempC      = t;
        r.tempValid  = !(r.cond & (kCondSensorShort | kCondSensorOpen)) && t > -273.0;
        r.currentA   = i;
        r.voltageV   = v;
        r.unitErrors = ErrorsText(errs);
        return true;
    }

    bool PollOnce()
    {
        std::lock_guard<std::mutex> lock(ioMu);
        if (!connected.load()) return false;
        return EnsureLink() && PollLocked() == Xfer::Ok;
    }

    bool StartPolling(int ms)
    {
        if (!connected.load()) { SetError("not connected"); return false; }
        if (polling.load()) return true;
        intervalMs = ms > 0 ? ms : 500;
        sTemp = &DataBus::Instance().Stream(topics::TecTemp(cfg.name));
        sCurr = &DataBus::Instance().Stream(topics::TecCurrent(cfg.name));
        sVolt = &DataBus::Instance().Stream(topics::TecVoltage(cfg.name));
        // A temperature settles over minutes, not the default seconds: keep
        // ten minutes so the TEC page can show the approach.
        sTemp->SetWindowSeconds(600.0);
        polling.store(true);
        pollThread = std::thread(&Impl::PollLoop, this);
        FD_INFO("tec", "%s polling every %d ms", cfg.name.c_str(), intervalMs);
        return true;
    }

    void StopPolling()
    {
        if (polling.exchange(false)) {
            reqCv.notify_all();
            if (pollThread.joinable()) pollThread.join();
        }
    }

    void PollLoop()
    {
        const auto start = std::chrono::steady_clock::now();
        auto nextPoll = start;
        while (polling.load()) {
            // UI requests first: they are what the operator is waiting to see.
            // The set point before the output, so an ON lands on the new one.
            std::optional<double> temp;
            std::optional<bool>   out;
            {
                std::lock_guard<std::mutex> l(reqMu);
                temp.swap(reqTemp);
                out.swap(reqOut);
            }
            if (temp) SetTemperature(*temp);   // failures are logged inside
            if (out)  SetOutput(*out);

            if (std::chrono::steady_clock::now() >= nextPoll) {
                bool ok;
                {
                    std::lock_guard<std::mutex> lock(ioMu);
                    ok = EnsureLink() && PollLocked() == Xfer::Ok;
                    if (!ok) SetLinkStatus();
                }
                if (ok) {
                    const Status s = GetStatus();
                    const double t = std::chrono::duration<double>(
                        std::chrono::steady_clock::now() - start).count();
                    if (s.tempValid) sTemp->Publish(t, s.tempC);
                    sCurr->Publish(t, s.currentA);
                    sVolt->Publish(t, s.voltageV);
                }
                nextPoll = std::chrono::steady_clock::now() + std::chrono::milliseconds(intervalMs);
            }

            std::unique_lock<std::mutex> l(reqMu);
            reqCv.wait_until(l, nextPoll, [this] {
                return !polling.load() || reqTemp.has_value() || reqOut.has_value();
            });
        }
    }
};

// ---- Forwarding shell ----
ArroyoTec::ArroyoTec(Config cfg) : p_(std::make_unique<Impl>(std::move(cfg))) {}
ArroyoTec::~ArroyoTec() = default;

bool ArroyoTec::Connect()           { return p_->Connect(); }
void ArroyoTec::Disconnect()        { p_->Disconnect(); }
bool ArroyoTec::IsConnected() const { return p_->connected.load(); }

bool ArroyoTec::SetTemperature(double c) { return p_->SetTemperature(c); }
bool ArroyoTec::SetOutput(bool on)       { return p_->SetOutput(on); }
void ArroyoTec::RequestTemperature(double c) { p_->RequestTemperature(c); }
void ArroyoTec::RequestOutput(bool on)       { p_->RequestOutput(on); }

ArroyoTec::Status ArroyoTec::GetStatus() const { return p_->GetStatus(); }
bool ArroyoTec::Measure(Reading& out)          { return p_->Measure(out); }

bool ArroyoTec::StartPolling(int intervalMs) { return p_->StartPolling(intervalMs); }
void ArroyoTec::StopPolling()                { p_->StopPolling(); }
bool ArroyoTec::IsPolling() const            { return p_->polling.load(); }
bool ArroyoTec::PollOnce()                   { return p_->PollOnce(); }

std::string ArroyoTec::DeviceId() const
{
    std::lock_guard<std::mutex> l(p_->statusMu);
    return p_->deviceId;
}
std::string ArroyoTec::LastError() const { return p_->Error(); }
const ArroyoTec::Config& ArroyoTec::GetConfig() const { return p_->cfg; }

} // namespace firedog
