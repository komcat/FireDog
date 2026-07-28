// Keithley6482.cpp
//
// Direct VISA usage. The connect/init/read SCPI calls are ported from
// bobcat_engine's Keithley6482Adapter (the battle-tested source), condensed
// to a single class with a std::thread poll loop publishing to the DataBus.
#include "hardware/instruments/Keithley6482.h"

#include "core/DataBus.h"
#include "core/Log.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
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

struct Keithley6482::Impl {
    Config            cfg;
    ViSession         rm   = VI_NULL;
    ViSession         inst = VI_NULL;
    std::atomic<bool> connected{false};
    std::atomic<bool> polling{false};
    std::thread       pollThread;
    std::mutex        visaMutex;          // guards all VISA I/O
    std::string       deviceId;
    std::string       lastError;
    int               intervalMs = 100;
    DataStream*       s1 = nullptr;
    DataStream*       s2 = nullptr;

    // UI-facing state, kept off visaMutex so per-frame reads never wait behind
    // an in-flight :READ? transaction.
    mutable std::mutex  cfgMutex;         // guards cfg (runtime NPLC/range edits)
    std::atomic<double> biasVolts{0.0};
    std::atomic<bool>   outputOn{false};

    explicit Impl(Config c) : cfg(std::move(c)) {}
    ~Impl() { StopPolling(); Disconnect(); }

    Config GetConfig() const
    {
        std::lock_guard<std::mutex> lock(cfgMutex);
        return cfg;
    }

    bool WriteInternal(const std::string& cmd)   // assumes visaMutex held
    {
        if (inst == VI_NULL) return false;
        return viPrintf(inst, "%s\n", cmd.c_str()) >= VI_SUCCESS;
    }
    bool QueryInternal(const std::string& cmd, std::string& out)   // assumes held
    {
        if (inst == VI_NULL) return false;
        if (viPrintf(inst, "%s\n", cmd.c_str()) < VI_SUCCESS) return false;
        char buf[512];
        if (viScanf(inst, "%t", buf) < VI_SUCCESS) return false;
        out = buf;
        // trim trailing whitespace/newline
        while (!out.empty() && (out.back() == '\n' || out.back() == '\r' ||
                                out.back() == ' '  || out.back() == '\t')) {
            out.pop_back();
        }
        return true;
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

        std::string id;
        if (!QueryInternal("*IDN?", id)) {
            lastError = "*IDN? failed";
            FD_ERROR("keithley", "%s", lastError.c_str());
            viClose(inst); viClose(rm); inst = rm = VI_NULL;
            return false;
        }
        deviceId = id;
        connected.store(true);

        // --- Fast continuous dual-channel acquisition (ported from
        //     bobcat_engine's FAST/ULTRA_FAST modes) ---
        WriteInternal("*RST");
        WriteInternal("*CLS");
        WriteInternal(":SYST:ZCH OFF");
        WriteInternal(":FORM:ELEM CURR1,CURR2");   // both channels per read

        // NPLC dominates the sample rate: 1 NPLC ~= 1/mains seconds, per channel.
        // NPLC 0.1 -> ~80 Hz; NPLC 1 -> ~30 Hz; NPLC 5 -> ~12 Hz.
        {
            std::ostringstream c1, c2;
            c1 << ":SENS1:CURR:NPLC " << cfg.nplc;
            c2 << ":SENS2:CURR:NPLC " << cfg.nplc;
            WriteInternal(c1.str());
            WriteInternal(c2.str());
        }

        if (cfg.autoRange) {
            WriteInternal(":SENS1:CURR:RANG:AUTO ON");
            WriteInternal(":SENS2:CURR:RANG:AUTO ON");
        } else if (cfg.fixedRange > 0.0) {
            std::ostringstream r1, r2;
            r1 << ":SENS1:CURR:RANG " << std::scientific << cfg.fixedRange;
            r2 << ":SENS2:CURR:RANG " << std::scientific << cfg.fixedRange;
            WriteInternal(r1.str());
            WriteInternal(r2.str());
        }

        // Speed: no averaging, no auto-zero. The poll loop reads with :READ?
        // (INIT + FETCh in one transaction), exactly like the proven Keithley
        // 6482 driver. Do NOT set :INIT:CONT ON — with continuous init the
        // instrument raises -213 "Init ignored" on every :READ? and the stream
        // stalls (this was the 0 Hz bug).
        WriteInternal(":SENS1:CURR:AVER:STAT OFF");
        WriteInternal(":SENS2:CURR:AVER:STAT OFF");
        WriteInternal(":SYST:AZER OFF");
        // Front-panel display refresh throttles reads; turn it off for a mild
        // throughput boost (restored in Disconnect). Auto-range stays ON, so
        // range-hunting remains the dominant per-read cost.
        WriteInternal(":DISP:ENAB OFF");
        WriteInternal("*CLS");

        FD_INFO("keithley", "connected: %s", deviceId.c_str());
        return true;
    }

    void Disconnect()
    {
        StopPolling();
        std::lock_guard<std::mutex> lock(visaMutex);
        if (!connected.load()) return;
        WriteInternal(":OUTP OFF");
        WriteInternal(":SYST:ZCH ON");
        WriteInternal(":DISP:ENAB ON");   // restore the front-panel display
        if (inst != VI_NULL) { viClose(inst); inst = VI_NULL; }
        if (rm   != VI_NULL) { viClose(rm);   rm   = VI_NULL; }
        connected.store(false);
        outputOn.store(false);
    }

    // Read back the ACTUAL output state (":OUTP?" -> "1"/"0", like bobcat's
    // IsOutputOn) and drain the instrument's SCPI error queue into the log so
    // a silently rejected command is visible. Assumes visaMutex is held.
    void VerifyOutputInternal()
    {
        std::string resp;
        if (QueryInternal(":OUTP?", resp))
            outputOn.store(resp == "1" || resp == "ON");
        for (int i = 0; i < 5; ++i) {   // queue holds several; "0,..." = empty
            std::string err;
            if (!QueryInternal(":SYST:ERR?", err) || err.rfind("0,", 0) == 0) break;
            lastError = err;
            FD_WARN("keithley", "instrument error: %s", err.c_str());
        }
    }

    bool SetVoltageBias(double volts)
    {
        std::lock_guard<std::mutex> lock(visaMutex);
        if (!connected.load()) { lastError = "not connected"; return false; }
        std::ostringstream c; c << ":SOUR:VOLT " << volts;
        const bool ok = WriteInternal(c.str()) && WriteInternal(":OUTP ON");
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
        const bool ok = WriteInternal(":OUTP OFF");
        VerifyOutputInternal();
        return ok && !outputOn.load();
    }

    bool SetNplc(double nplc)
    {
        std::lock_guard<std::mutex> lock(visaMutex);
        if (!connected.load()) { lastError = "not connected"; return false; }
        std::ostringstream c1, c2;
        c1 << ":SENS1:CURR:NPLC " << nplc;
        c2 << ":SENS2:CURR:NPLC " << nplc;
        const bool ok = WriteInternal(c1.str()) && WriteInternal(c2.str());
        if (ok) {
            std::lock_guard<std::mutex> cl(cfgMutex);
            cfg.nplc = nplc;
            FD_INFO("keithley", "NPLC set to %.3g", nplc);
        }
        return ok;
    }

    bool SetRangeAuto()
    {
        std::lock_guard<std::mutex> lock(visaMutex);
        if (!connected.load()) { lastError = "not connected"; return false; }
        const bool ok = WriteInternal(":SENS1:CURR:RANG:AUTO ON") &&
                        WriteInternal(":SENS2:CURR:RANG:AUTO ON");
        if (ok) {
            std::lock_guard<std::mutex> cl(cfgMutex);
            cfg.autoRange = true;
            FD_INFO("keithley", "auto range ON");
        }
        return ok;
    }

    bool SetRangeFixed(double amps)
    {
        std::lock_guard<std::mutex> lock(visaMutex);
        if (!connected.load()) { lastError = "not connected"; return false; }
        std::ostringstream r1, r2;
        r1 << ":SENS1:CURR:RANG " << std::scientific << amps;
        r2 << ":SENS2:CURR:RANG " << std::scientific << amps;
        const bool ok = WriteInternal(r1.str()) && WriteInternal(r2.str());
        if (ok) {
            std::lock_guard<std::mutex> cl(cfgMutex);
            cfg.autoRange  = false;
            cfg.fixedRange = amps;
            FD_INFO("keithley", "fixed range %.1e A", amps);
        }
        return ok;
    }

    bool StartPolling(int ms, const std::string& t1, const std::string& t2)
    {
        if (!connected.load()) { lastError = "not connected"; return false; }
        if (polling.load()) return true;
        intervalMs = ms > 0 ? ms : 0;   // <= 0: continuous — NPLC/GPIB paces it
        s1 = HasChannel(cfg.enabledChannels, 1) ? &DataBus::Instance().Stream(t1) : nullptr;
        s2 = HasChannel(cfg.enabledChannels, 2) ? &DataBus::Instance().Stream(t2) : nullptr;
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
                    // :READ? triggers one dual-channel measurement and returns
                    // "ch1,ch2" (the proven 6482 read path — never :FETC?).
                    if (QueryInternal(":READ?", resp)) { ParseDual(resp, ch1, ch2); ok = true; }
                }
            }
            if (ok) {
                const double t = std::chrono::duration<double>(
                    std::chrono::steady_clock::now() - start).count();
                if (s1 && ch1 != kBad) s1->Publish(t, ch1);
                if (s2 && ch2 != kBad) s2->Publish(t, ch2);
            }
            // Continuous by default (GPIB transaction latency paces the loop).
            // intervalMs > 0 rate-limits to that minimum period. Even when
            // continuous, yield 1 ms so a waiting UI command (bias/NPLC/range)
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

struct Keithley6482::Impl {
    Config            cfg;
    std::atomic<bool> connected{false};
    std::atomic<bool> polling{false};
    std::thread       pollThread;
    int               intervalMs = 100;
    DataStream*       s1 = nullptr;
    DataStream*       s2 = nullptr;
    std::string       deviceId = "Keithley 6482 (simulated)";
    std::string       lastError;

    // UI-facing state (same surface as the real driver).
    mutable std::mutex  cfgMutex;
    std::atomic<double> biasVolts{0.0};
    std::atomic<bool>   outputOn{false};

    explicit Impl(Config c) : cfg(std::move(c)) {}
    ~Impl() { StopPolling(); }

    Config GetConfig() const
    {
        std::lock_guard<std::mutex> lock(cfgMutex);
        return cfg;
    }

    bool Connect()        { connected.store(true); FD_INFO("keithley", "connected: %s", deviceId.c_str()); return true; }
    void Disconnect()     { StopPolling(); connected.store(false); outputOn.store(false); }
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
        std::lock_guard<std::mutex> lock(cfgMutex);
        cfg.nplc = nplc;   // the sim loop derives its rate from this
        FD_INFO("keithley", "NPLC set to %.3g (simulated)", nplc);
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
        intervalMs = ms > 0 ? ms : 100;
        s1 = HasChannel(cfg.enabledChannels, 1) ? &DataBus::Instance().Stream(t1) : nullptr;
        s2 = HasChannel(cfg.enabledChannels, 2) ? &DataBus::Instance().Stream(t2) : nullptr;
        polling.store(true);
        pollThread = std::thread(&Impl::Loop, this);
        FD_INFO("keithley", "polling started (simulated ~80 Hz)");
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
            // Simulated picoamp-scale currents (amps).
            const double ch1 = 1.0e-3 * (0.5 + 0.5 * std::sin(t * 1.5));
            const double ch2 = 8.0e-4 * (0.5 + 0.5 * std::sin(t * 1.5 + 2.0))
                             + 5.0e-5 * std::sin(t * 20.0);
            if (s1) s1->Publish(t, ch1);
            if (s2) s2->Publish(t, ch2);
            // 0 means "as fast as the instrument allows": approximate the real
            // NPLC-governed rate (2 channels x NPLC/60 s + overhead) so the
            // NPLC control visibly changes the measured Hz in simulation.
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

Keithley6482::Keithley6482(Config cfg) : p_(std::make_unique<Impl>(std::move(cfg))) {}
Keithley6482::~Keithley6482() = default;

bool Keithley6482::Connect()          { return p_->Connect(); }
void Keithley6482::Disconnect()       { p_->Disconnect(); }
bool Keithley6482::IsConnected() const { return p_->connected.load(); }
bool Keithley6482::SetVoltageBias(double v) { return p_->SetVoltageBias(v); }
bool Keithley6482::OutputOff()        { return p_->OutputOff(); }
bool Keithley6482::SetNplc(double n)  { return p_->SetNplc(n); }
bool Keithley6482::SetRangeAuto()     { return p_->SetRangeAuto(); }
bool Keithley6482::SetRangeFixed(double a) { return p_->SetRangeFixed(a); }
double Keithley6482::BiasVolts() const { return p_->biasVolts.load(); }
bool   Keithley6482::OutputOn() const  { return p_->outputOn.load(); }
Keithley6482::Config Keithley6482::GetConfig() const { return p_->GetConfig(); }
bool Keithley6482::StartPolling(int ms, const std::string& t1, const std::string& t2)
{
    return p_->StartPolling(ms, t1, t2);
}
void Keithley6482::StopPolling()      { p_->StopPolling(); }
bool Keithley6482::IsPolling() const  { return p_->polling.load(); }
std::string Keithley6482::DeviceId() const  { return p_->deviceId; }
std::string Keithley6482::LastError() const { return p_->lastError; }

} // namespace firedog
