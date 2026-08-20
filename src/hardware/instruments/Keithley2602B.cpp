// Keithley2602B.cpp
//
// Direct VISA usage, TSP command set. There is no bobcat_engine adapter to port
// from — the 2600B series is the first TSP instrument in this codebase — so the
// sequence below is written against the 2600B reference manual and kept
// deliberately close in SHAPE to Keithley6482.cpp: one VISA mutex shared by the
// poll loop and the runtime setters, cheap atomics for the UI, one transaction
// per dual-channel sample.
//
// TSP in one paragraph: every line you send is Lua executed on the instrument.
// Assignments are silent ("smua.measure.nplc = 0.1"); you get bytes back only
// when the chunk calls print(). So a "query" here is print(<expr>), and a
// dual-channel read is one print() of a formatted pair — one round trip, like
// the 6482's ":READ?", not two.
#include "hardware/instruments/Keithley2602B.h"

#include "core/DataBus.h"
#include "core/Log.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <thread>
#include <utility>

namespace firedog {
namespace {
bool HasChannel(const std::vector<int>& ch, int n)
{
    return std::find(ch.begin(), ch.end(), n) != ch.end();
}
// Parse "1.2e-6,5.6e-7" -> (ch1, ch2). Missing values stay as sentinel.
void ParseDual(const std::string& resp, double& ch1, double& ch2)
{
    std::stringstream ss(resp);
    std::string tok;
    int i = 0;
    while (std::getline(ss, tok, ',')) {
        try {
            const double v = std::stod(tok);
            if (i == 0) ch1 = v; else if (i == 1) ch2 = v;
        } catch (...) {}
        ++i;
    }
}
constexpr double kBad = -999.0;

// The instrument rejects an out-of-window NPLC by raising a queue error and
// keeping the old value — a silent no-op from the UI's point of view. Clamp
// here so what the page shows is what the instrument is actually doing.
double ClampNplc(double n)
{
    if (n < Keithley2602B::kMinNplc) return Keithley2602B::kMinNplc;
    if (n > Keithley2602B::kMaxNplc) return Keithley2602B::kMaxNplc;
    return n;
}
// TSP is Lua: a number written at ostringstream's default 6 significant digits
// loses a 1e-9 range. Everything numeric goes out at full precision.
std::string Num(double v)
{
    std::ostringstream o;
    o << std::setprecision(10) << v;
    return o.str();
}
} // namespace
} // namespace firedog

#ifdef HAS_VISA
// ------------------------------- Real VISA driver -------------------------------
#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#endif
#include <visa.h>

namespace firedog {

struct Keithley2602B::Impl {
    Config            cfg;
    ViSession         rm   = VI_NULL;
    ViSession         inst = VI_NULL;
    std::atomic<bool> connected{false};
    std::atomic<bool> polling{false};
    std::thread       pollThread;
    std::mutex        visaMutex;          // guards all VISA I/O
    std::string       deviceId;
    std::string       lastError;
    int               intervalMs = 0;
    std::string       readCmd;            // built once in StartPolling
    DataStream*       s1 = nullptr;
    DataStream*       s2 = nullptr;

    // Which physical SMU backs each logical channel. Fixed at construction —
    // enabledChannels is config, never edited at runtime.
    bool useA = true;   // channel 1 -> smua
    bool useB = true;   // channel 2 -> smub

    // UI-facing state, kept off visaMutex so per-frame reads never wait behind
    // an in-flight measurement.
    mutable std::mutex  cfgMutex;         // guards cfg (runtime NPLC/range edits)
    std::atomic<double> biasVolts{0.0};
    std::atomic<bool>   outputOn{false};

    explicit Impl(Config c) : cfg(std::move(c))
    {
        useA = HasChannel(cfg.enabledChannels, 1);
        useB = HasChannel(cfg.enabledChannels, 2);
        cfg.nplc = ClampNplc(cfg.nplc);
    }
    ~Impl() { StopPolling(); Disconnect(); }

    Config GetConfig() const
    {
        std::lock_guard<std::mutex> lock(cfgMutex);
        return cfg;
    }

    bool WriteInternal(const std::string& cmd)   // assumes visaMutex held
    {
        if (inst == VI_NULL) return false;
        const std::string full = cmd + "\n";
        ViUInt32 written = 0;
        return viWrite(inst, (ViBuf)full.c_str(),
                       static_cast<ViUInt32>(full.size()), &written) >= VI_SUCCESS;
    }
    bool QueryInternal(const std::string& cmd, std::string& out)   // assumes held
    {
        if (!WriteInternal(cmd)) return false;
        char buf[1024];
        ViUInt32 read = 0;
        if (viRead(inst, (ViBuf)buf, sizeof(buf) - 1, &read) < VI_SUCCESS) return false;
        out.assign(buf, read);
        while (!out.empty() && (out.back() == '\n' || out.back() == '\r' ||
                                out.back() == ' '  || out.back() == '\t')) {
            out.pop_back();
        }
        return true;
    }
    bool QueryDouble(const std::string& cmd, double& v)   // assumes held
    {
        std::string resp;
        if (!QueryInternal(cmd, resp)) return false;
        try { v = std::stod(resp); } catch (...) { return false; }
        return true;
    }

    // "smua" / "smub" for the enabled channels, in order. Every per-channel
    // setter walks this, so a one-channel config never touches the other SMU.
    std::vector<std::string> Smus() const
    {
        std::vector<std::string> v;
        if (useA) v.push_back("smua");
        if (useB) v.push_back("smub");
        return v;
    }

    bool Connect()
    {
        std::lock_guard<std::mutex> lock(visaMutex);
        if (connected.load()) return true;

        if (viOpenDefaultRM(&rm) < VI_SUCCESS) {
            lastError = "viOpenDefaultRM failed";
            FD_ERROR("keithley", "%s", lastError.c_str());
            return false;
        }
        if (viOpen(rm, cfg.visaResource.c_str(), VI_NULL, 5000, &inst) < VI_SUCCESS) {
            lastError = "viOpen failed for " + cfg.visaResource;
            FD_ERROR("keithley", "%s", lastError.c_str());
            viClose(rm); rm = VI_NULL;
            return false;
        }
        viSetAttribute(inst, VI_ATTR_TMO_VALUE, 5000);
        // A raw-socket session ("::5025::SOCKET") has no END bit to end a read;
        // the '\n' termchar is what terminates it. Harmless on VXI-11 INSTR.
        viSetAttribute(inst, VI_ATTR_TERMCHAR_EN, VI_TRUE);
        viSetAttribute(inst, VI_ATTR_TERMCHAR, '\n');
        viClear(inst);   // drop whatever a previous session left in the buffers

        // Both of these BEFORE the first query: with prompts on, the instrument
        // interleaves "TSP>" tokens into the output stream and every read comes
        // back one reply out of step. showerrors would likewise push unsolicited
        // error text between a print() and its value.
        WriteInternal("localnode.prompts = 0");
        WriteInternal("localnode.showerrors = 0");

        std::string id;
        if (!QueryInternal("*IDN?", id)) {
            lastError = "*IDN? failed";
            FD_ERROR("keithley", "%s", lastError.c_str());
            viClose(inst); viClose(rm); inst = rm = VI_NULL;
            return false;
        }
        deviceId = id;
        connected.store(true);

        // --- Measurement setup. smuX.reset() puts the channel at its defaults
        //     (DC volts source, 0 V, 100 mA compliance, output OFF); the lines
        //     after it are the ones that matter for a fast photocurrent stream.
        WriteInternal("errorqueue.clear()");
        WriteInternal("format.data = format.ASCII");
        WriteInternal("format.asciiprecision = 8");

        for (const std::string& s : Smus()) {
            WriteInternal(s + ".reset()");
            WriteInternal(s + ".source.func = " + s + ".OUTPUT_DCVOLTS");
            WriteInternal(s + ".source.levelv = 0");
            // NPLC dominates the sample rate exactly as on the 6482: one NPLC
            // is one mains cycle of integration, per channel.
            WriteInternal(s + ".measure.nplc = " + Num(cfg.nplc));
            // Speed: no auto-zero refresh, no averaging filter — the same two
            // knobs the 6482 turns off for a continuous stream.
            WriteInternal(s + ".measure.autozero = " + s + ".AUTOZERO_OFF");
            WriteInternal(s + ".measure.filter.enable = " + s + ".FILTER_OFF");
            if (cfg.autoRange) {
                WriteInternal(s + ".measure.autorangei = " + s + ".AUTORANGE_ON");
            } else if (cfg.fixedRange > 0.0) {
                WriteInternal(s + ".measure.rangei = " + Num(cfg.fixedRange));
            }
            WriteInternal("display." + s + ".measure.func = display.MEASURE_DCAMPS");
        }
        DrainErrorsInternal();

        FD_INFO("keithley", "connected: %s", deviceId.c_str());
        return true;
    }

    void Disconnect()
    {
        StopPolling();
        std::lock_guard<std::mutex> lock(visaMutex);
        if (!connected.load()) return;
        for (const std::string& s : Smus())
            WriteInternal(s + ".source.output = " + s + ".OUTPUT_OFF");
        if (inst != VI_NULL) { viClose(inst); inst = VI_NULL; }
        if (rm   != VI_NULL) { viClose(rm);   rm   = VI_NULL; }
        connected.store(false);
        outputOn.store(false);
    }

    // Read back the ACTUAL output state of every enabled channel and drain the
    // instrument's error queue into the log, so a silently rejected TSP line is
    // visible. Assumes visaMutex is held.
    void VerifyOutputInternal()
    {
        bool all = true;
        for (const std::string& s : Smus()) {
            double v = 0.0;
            if (QueryDouble("print(" + s + ".source.output)", v)) all = all && (v != 0.0);
            else                                                 all = false;
        }
        outputOn.store(all);
        DrainErrorsInternal();
    }

    void DrainErrorsInternal()   // assumes visaMutex held
    {
        double count = 0.0;
        if (!QueryDouble("print(errorqueue.count)", count)) return;
        int n = static_cast<int>(count);
        if (n > 10) n = 10;   // a flood is one bug repeated; the log needs a few
        for (int i = 0; i < n; ++i) {
            std::string line;
            if (!QueryInternal("print(errorqueue.next())", line)) break;
            lastError = line;
            FD_WARN("keithley", "instrument error: %s", line.c_str());
        }
    }

    bool SetVoltageBias(double volts)
    {
        std::lock_guard<std::mutex> lock(visaMutex);
        if (!connected.load()) { lastError = "not connected"; return false; }

        bool ok = true;
        if (useA) ok = ok && WriteInternal("smua.source.levelv = " + Num(volts));
        // Channel 2 sources 0 V and measures through it. An SMU is not an
        // ammeter with its output off — it reads ~0 A, which looks exactly like
        // a dark photodiode, so ch2 goes ON at 0 V rather than staying off.
        if (useB) ok = ok && WriteInternal("smub.source.levelv = 0");
        for (const std::string& s : Smus())
            ok = ok && WriteInternal(s + ".source.output = " + s + ".OUTPUT_ON");
        if (ok) biasVolts.store(volts);
        VerifyOutputInternal();   // UI shows the instrument's truth, not our hope
        if (ok && outputOn.load())
            FD_INFO("keithley", "voltage bias %.3f V, output ON (verified)", volts);
        else
            FD_WARN("keithley", "bias write %s but output reads %s",
                    ok ? "ok" : "FAILED", outputOn.load() ? "ON" : "OFF");
        return ok && outputOn.load();
    }

    bool OutputOff()
    {
        std::lock_guard<std::mutex> lock(visaMutex);
        if (!connected.load()) return false;
        bool ok = true;
        for (const std::string& s : Smus())
            ok = ok && WriteInternal(s + ".source.output = " + s + ".OUTPUT_OFF");
        VerifyOutputInternal();
        return ok && !outputOn.load();
    }

    bool SetNplc(double nplc)
    {
        std::lock_guard<std::mutex> lock(visaMutex);
        if (!connected.load()) { lastError = "not connected"; return false; }
        const double clamped = ClampNplc(nplc);
        if (clamped != nplc)
            FD_WARN("keithley", "NPLC %.3g out of the 2602B window [%.3g, %.3g] - using %.3g",
                    nplc, kMinNplc, kMaxNplc, clamped);
        bool ok = true;
        for (const std::string& s : Smus())
            ok = ok && WriteInternal(s + ".measure.nplc = " + Num(clamped));
        if (ok) {
            std::lock_guard<std::mutex> cl(cfgMutex);
            cfg.nplc = clamped;   // store what the instrument got, not what was asked
            FD_INFO("keithley", "NPLC set to %.3g", clamped);
        }
        DrainErrorsInternal();
        return ok;
    }

    bool SetRangeAuto()
    {
        std::lock_guard<std::mutex> lock(visaMutex);
        if (!connected.load()) { lastError = "not connected"; return false; }
        bool ok = true;
        for (const std::string& s : Smus())
            ok = ok && WriteInternal(s + ".measure.autorangei = " + s + ".AUTORANGE_ON");
        if (ok) {
            std::lock_guard<std::mutex> cl(cfgMutex);
            cfg.autoRange = true;
            FD_INFO("keithley", "auto range ON");
        }
        DrainErrorsInternal();
        return ok;
    }

    bool SetRangeFixed(double amps)
    {
        std::lock_guard<std::mutex> lock(visaMutex);
        if (!connected.load()) { lastError = "not connected"; return false; }
        const std::vector<std::string> smus = Smus();
        bool ok = true;
        for (const std::string& s : smus)
            ok = ok && WriteInternal(s + ".measure.rangei = " + Num(amps));
        // The UI's range table holds the 6482's decades (2 nA .. 20 mA); the
        // 2602B has different ones and its floor is 100 nA. It does not reject
        // an in-between value — it selects the smallest range that CONTAINS it —
        // so read back what was actually chosen instead of trusting the request.
        double actual = amps;
        if (!smus.empty()) QueryDouble("print(" + smus.front() + ".measure.rangei)", actual);
        if (ok) {
            std::lock_guard<std::mutex> cl(cfgMutex);
            cfg.autoRange  = false;
            cfg.fixedRange = actual;
            FD_INFO("keithley", "fixed range %.1e A requested, instrument selected %.1e A",
                    amps, actual);
        }
        DrainErrorsInternal();
        return ok;
    }

    bool StartPolling(int ms, const std::string& t1, const std::string& t2)
    {
        if (!connected.load()) { lastError = "not connected"; return false; }
        if (polling.load()) return true;
        intervalMs = ms > 0 ? ms : 0;   // <= 0: continuous — NPLC paces it
        s1 = useA ? &DataBus::Instance().Stream(t1) : nullptr;
        s2 = useB ? &DataBus::Instance().Stream(t2) : nullptr;
        // ONE transaction returns both channels. A disabled channel is padded
        // with a literal 0 rather than dropped, so the reply always has two
        // fields and the parser stays the same shape as the 6482's.
        readCmd = std::string("print(string.format(\"%.6e,%.6e\", ") +
                  (useA ? "smua.measure.i()" : "0") + ", " +
                  (useB ? "smub.measure.i()" : "0") + "))";
        polling.store(true);
        pollThread = std::thread(&Impl::PollLoop, this);
        if (intervalMs > 0) FD_INFO("keithley", "polling started (%d ms rate cap, NPLC %.3g)",
                                    intervalMs, cfg.nplc);
        else                FD_INFO("keithley", "polling started (continuous, NPLC %.3g)", cfg.nplc);
        return true;
    }

    void StopPolling()
    {
        if (polling.exchange(false)) {
            if (pollThread.joinable()) pollThread.join();
        }
    }

    void PollLoop()
    {
        const auto start = std::chrono::steady_clock::now();
        while (polling.load()) {
            const auto t0 = std::chrono::steady_clock::now();
            double ch1 = kBad, ch2 = kBad;
            bool ok = false;
            {
                std::lock_guard<std::mutex> lock(visaMutex);
                if (inst != VI_NULL) {
                    std::string resp;
                    if (QueryInternal(readCmd, resp)) { ParseDual(resp, ch1, ch2); ok = true; }
                }
            }
            if (ok) {
                const double t = std::chrono::duration<double>(
                    std::chrono::steady_clock::now() - start).count();
                if (s1 && ch1 != kBad) s1->Publish(t, ch1);
                if (s2 && ch2 != kBad) s2->Publish(t, ch2);
            }
            // Continuous by default (integration + LAN latency pace the loop).
            // Even then, yield 1 ms so a waiting UI command (bias/NPLC/range)
            // can grab visaMutex between reads instead of starving.
            if (intervalMs > 0) {
                const auto elapsed = std::chrono::steady_clock::now() - t0;
                const auto period  = std::chrono::milliseconds(intervalMs);
                if (elapsed < period) std::this_thread::sleep_for(period - elapsed);
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }
    }
};

} // namespace firedog

#else
// ------------------------------- Simulated stub -------------------------------
namespace firedog {

struct Keithley2602B::Impl {
    Config            cfg;
    std::atomic<bool> connected{false};
    std::atomic<bool> polling{false};
    std::thread       pollThread;
    int               intervalMs = 0;
    DataStream*       s1 = nullptr;
    DataStream*       s2 = nullptr;
    std::string       deviceId = "Keithley 2602B (simulated)";
    std::string       lastError;

    // UI-facing state (same surface as the real driver).
    mutable std::mutex  cfgMutex;
    std::atomic<double> biasVolts{0.0};
    std::atomic<bool>   outputOn{false};

    explicit Impl(Config c) : cfg(std::move(c)) { cfg.nplc = ClampNplc(cfg.nplc); }
    ~Impl() { StopPolling(); }

    Config GetConfig() const
    {
        std::lock_guard<std::mutex> lock(cfgMutex);
        return cfg;
    }

    bool Connect()    { connected.store(true); FD_INFO("keithley", "connected: %s", deviceId.c_str()); return true; }
    void Disconnect() { StopPolling(); connected.store(false); outputOn.store(false); }
    bool SetVoltageBias(double v)
    {
        if (!connected.load()) { lastError = "not connected"; return false; }
        biasVolts.store(v);
        outputOn.store(true);
        FD_INFO("keithley", "voltage bias %.3f V (simulated)", v);
        return true;
    }
    bool OutputOff()
    {
        if (!connected.load()) return false;
        outputOn.store(false);
        return true;
    }

    bool SetNplc(double nplc)
    {
        if (!connected.load()) { lastError = "not connected"; return false; }
        const double clamped = ClampNplc(nplc);
        std::lock_guard<std::mutex> lock(cfgMutex);
        cfg.nplc = clamped;   // the sim loop derives its rate from this
        FD_INFO("keithley", "NPLC set to %.3g (simulated)", clamped);
        return true;
    }
    bool SetRangeAuto()
    {
        if (!connected.load()) return false;
        std::lock_guard<std::mutex> lock(cfgMutex);
        cfg.autoRange = true;
        return true;
    }
    bool SetRangeFixed(double amps)
    {
        if (!connected.load()) return false;
        std::lock_guard<std::mutex> lock(cfgMutex);
        cfg.autoRange  = false;
        cfg.fixedRange = amps;
        return true;
    }

    bool StartPolling(int ms, const std::string& t1, const std::string& t2)
    {
        if (!connected.load()) return false;
        if (polling.load()) return true;
        intervalMs = ms > 0 ? ms : 0;
        s1 = HasChannel(cfg.enabledChannels, 1) ? &DataBus::Instance().Stream(t1) : nullptr;
        s2 = HasChannel(cfg.enabledChannels, 2) ? &DataBus::Instance().Stream(t2) : nullptr;
        polling.store(true);
        pollThread = std::thread(&Impl::Loop, this);
        FD_INFO("keithley", "polling started (simulated)");
        return true;
    }
    void StopPolling()
    {
        if (polling.exchange(false)) {
            if (pollThread.joinable()) pollThread.join();
        }
    }
    void Loop()
    {
        const auto start = std::chrono::steady_clock::now();
        while (polling.load()) {
            const double t = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - start).count();
            // Same synthetic photocurrents as the 6482 stub, so swapping the two
            // drivers in simulation changes nothing on the chart.
            const double ch1 = 1.0e-3 * (0.5 + 0.5 * std::sin(t * 1.5));
            const double ch2 = 8.0e-4 * (0.5 + 0.5 * std::sin(t * 1.5 + 2.0))
                             + 5.0e-5 * std::sin(t * 20.0);
            if (s1) s1->Publish(t, ch1);
            if (s2) s2->Publish(t, ch2);
            int ms = intervalMs;
            if (ms <= 0) {
                double nplc;
                {
                    std::lock_guard<std::mutex> lock(cfgMutex);
                    nplc = cfg.nplc;
                }
                ms = static_cast<int>(2.0 * nplc * 16.7 + 9.0);   // NPLC 0.1 -> ~80 Hz
                if (ms < 1)    ms = 1;
                if (ms > 1000) ms = 1000;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(ms));
        }
    }
};

} // namespace firedog

#endif // HAS_VISA

// ---- Forwarding shell (identical for both Impls) ----
namespace firedog {

Keithley2602B::Keithley2602B(Config cfg) : p_(std::make_unique<Impl>(std::move(cfg))) {}
Keithley2602B::~Keithley2602B() = default;

bool Keithley2602B::Connect()           { return p_->Connect(); }
void Keithley2602B::Disconnect()        { p_->Disconnect(); }
bool Keithley2602B::IsConnected() const { return p_->connected.load(); }
bool Keithley2602B::SetVoltageBias(double v) { return p_->SetVoltageBias(v); }
bool Keithley2602B::OutputOff()         { return p_->OutputOff(); }
bool Keithley2602B::SetNplc(double n)   { return p_->SetNplc(n); }
bool Keithley2602B::SetRangeAuto()      { return p_->SetRangeAuto(); }
bool Keithley2602B::SetRangeFixed(double a) { return p_->SetRangeFixed(a); }
double Keithley2602B::BiasVolts() const { return p_->biasVolts.load(); }
bool   Keithley2602B::OutputOn() const  { return p_->outputOn.load(); }
Keithley2602B::Config Keithley2602B::GetConfig() const { return p_->GetConfig(); }
bool Keithley2602B::StartPolling(int ms, const std::string& t1, const std::string& t2)
{
    return p_->StartPolling(ms, t1, t2);
}
void Keithley2602B::StopPolling()       { p_->StopPolling(); }
bool Keithley2602B::IsPolling() const   { return p_->polling.load(); }
std::string Keithley2602B::DeviceId() const  { return p_->deviceId; }
std::string Keithley2602B::LastError() const { return p_->lastError; }

} // namespace firedog
