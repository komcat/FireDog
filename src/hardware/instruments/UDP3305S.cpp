// UDP3305S.cpp
//
// Direct VISA usage. Open sequence and SCPI strings are ported from
// bobcat_engine's UnitTUDP3305S (viWrite/viRead with '\n' termchar, 5 s I/O
// timeout, 200 ms USB settle after open). Output switches read the state back
// (OUTPut? CH<n>) so the UI shows the instrument's truth — the lesson learned
// on the Keithley page. Setpoints are programmed with the hardware-verified
// APPLy form and read back via SOURce<n>:VOLTage?/:CURRent?.
#include "hardware/instruments/UDP3305S.h"

#include "core/DataBus.h"
#include "core/Log.h"
#include "core/Topics.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <mutex>
#include <sstream>
#include <thread>
#include <utility>

namespace firedog {
namespace {
// The UDP3305S answers OUTPut? with "ON"/"OFF" (some firmware "1"/"0").
bool ParseOn(const std::string& resp)
{
    return resp.find('1') != std::string::npos || resp.find("ON") != std::string::npos;
}
int ClampChannels(int n)
{
    return std::clamp(n, 1, UDP3305S::kMaxChannels);
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

struct UDP3305S::Impl {
    Config            cfg;
    ViSession         rm   = VI_NULL;
    ViSession         inst = VI_NULL;
    std::atomic<bool> connected{false};
    std::atomic<bool> polling{false};
    std::thread       pollThread;
    std::mutex        visaMutex;    // guards all VISA I/O (poll + setters)
    std::string       deviceId;
    std::string       lastError;
    int               intervalMs = 500;
    DataStream*       sV[kMaxChannels] = {nullptr, nullptr, nullptr};
    DataStream*       sI[kMaxChannels] = {nullptr, nullptr, nullptr};
    DataStream*       sVL[kMaxChannels] = {nullptr, nullptr, nullptr};   // legacy "psu/ch<n>/v"
    DataStream*       sIL[kMaxChannels] = {nullptr, nullptr, nullptr};

    // UI-facing status cache, off visaMutex so per-frame reads never wait
    // behind an in-flight query (the Keithley cfgMutex pattern).
    mutable std::mutex statusMutex;
    Status             status;

    explicit Impl(Config c) : cfg(std::move(c)) { cfg.channels = ClampChannels(cfg.channels); }
    ~Impl() { StopPolling(); Disconnect(); }

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

    void SetStatus(const Status& s)
    {
        std::lock_guard<std::mutex> lock(statusMutex);
        status = s;
    }
    Status GetStatus() const
    {
        std::lock_guard<std::mutex> lock(statusMutex);
        return status;
    }

    bool Connect()
    {
        std::lock_guard<std::mutex> lock(visaMutex);
        if (connected.load()) return true;

        if (viOpenDefaultRM(&rm) < VI_SUCCESS) {
            lastError = "viOpenDefaultRM failed";
            FD_ERROR("udp3305s", "%s", lastError.c_str());
            return false;
        }
        if (viOpen(rm, cfg.visaResource.c_str(), VI_NULL, 5000, &inst) < VI_SUCCESS) {
            lastError = "viOpen failed for " + cfg.visaResource;
            FD_ERROR("udp3305s", "%s", lastError.c_str());
            viClose(rm); rm = VI_NULL;
            return false;
        }
        viSetAttribute(inst, VI_ATTR_TMO_VALUE, 5000);
        viSetAttribute(inst, VI_ATTR_TERMCHAR_EN, VI_TRUE);
        viSetAttribute(inst, VI_ATTR_TERMCHAR, '\n');
        viFlush(inst, VI_READ_BUF_DISCARD | VI_WRITE_BUF_DISCARD);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));   // USB settle

        std::string id;
        if (!QueryInternal("*IDN?", id)) {
            lastError = "*IDN? failed";
            FD_ERROR("udp3305s", "%s", lastError.c_str());
            viClose(inst); viClose(rm); inst = rm = VI_NULL;
            return false;
        }
        deviceId = id;
        connected.store(true);

        // Read each channel's LIVE state — nothing is written to the outputs.
        Status s;
        for (int ch = 1; ch <= cfg.channels; ++ch) {
            ChannelStatus& c = s.ch[ch - 1];
            std::string resp;
            if (QueryInternal("OUTPut? CH" + std::to_string(ch), resp)) c.outputOn = ParseOn(resp);
            double v = 0.0;
            if (QueryDouble("SOURce" + std::to_string(ch) + ":VOLTage?", v)) c.setV = v;
            if (QueryDouble("SOURce" + std::to_string(ch) + ":CURRent?", v)) c.setI = v;
        }
        s.valid = false;   // measured values arrive with the first poll
        SetStatus(s);

        FD_INFO("udp3305s", "connected: %s (CH1 %s, CH2 %s, CH3 %s)", deviceId.c_str(),
                s.ch[0].outputOn ? "ON" : "off", s.ch[1].outputOn ? "ON" : "off",
                s.ch[2].outputOn ? "ON" : "off");
        return true;
    }

    void Disconnect()
    {
        StopPolling();
        std::lock_guard<std::mutex> lock(visaMutex);
        if (!connected.load()) return;
        // bobcat safety habit: force every output OFF before tearing down
        // (best-effort — deliberately different from the CLD, which persists).
        for (int ch = 1; ch <= cfg.channels; ++ch) {
            (void)WriteInternal("OUTPut CH" + std::to_string(ch) + ",OFF");
        }
        {
            std::lock_guard<std::mutex> sl(statusMutex);
            for (int ch = 0; ch < kMaxChannels; ++ch) status.ch[ch].outputOn = false;
        }
        FD_INFO("udp3305s", "all outputs forced OFF at disconnect");
        if (inst != VI_NULL) { viClose(inst); inst = VI_NULL; }
        if (rm   != VI_NULL) { viClose(rm);   rm   = VI_NULL; }
        connected.store(false);
    }

    bool ValidChannel(int ch)
    {
        if (ch < 1 || ch > cfg.channels) {
            lastError = "invalid channel " + std::to_string(ch);
            return false;
        }
        return true;
    }

    // Switch an output and read the state back; updates the status cache.
    bool SetOutput(int ch, bool on)
    {
        if (!ValidChannel(ch)) return false;
        std::lock_guard<std::mutex> lock(visaMutex);
        if (!connected.load()) { lastError = "not connected"; return false; }
        const std::string chStr = "CH" + std::to_string(ch);
        const bool wrote = WriteInternal("OUTPut " + chStr + (on ? ",ON" : ",OFF"));
        bool actual = on;
        std::string resp;
        if (QueryInternal("OUTPut? " + chStr, resp)) actual = ParseOn(resp);
        {
            std::lock_guard<std::mutex> sl(statusMutex);
            status.ch[ch - 1].outputOn = actual;
        }
        if (wrote && actual == on)
            FD_INFO("udp3305s", "%s output %s (verified)", chStr.c_str(), on ? "ON" : "OFF");
        else
            FD_WARN("udp3305s", "%s output %s requested but reads %s", chStr.c_str(),
                    on ? "ON" : "OFF", actual ? "ON" : "OFF");
        return wrote && actual == on;
    }

    // Program V+I in one shot (the hardware-verified APPLy form), then read the
    // setpoints back into the cache so the UI shows the instrument's truth.
    bool Apply(int ch, double volts, double amps)
    {
        if (!ValidChannel(ch)) return false;
        std::lock_guard<std::mutex> lock(visaMutex);
        if (!connected.load()) { lastError = "not connected"; return false; }
        std::ostringstream cmd;
        cmd << "APPLy CH" << ch << "," << volts << "," << amps;
        const bool ok = WriteInternal(cmd.str());
        double backV = volts, backI = amps;
        const std::string n = std::to_string(ch);
        (void)QueryDouble("SOURce" + n + ":VOLTage?", backV);
        (void)QueryDouble("SOURce" + n + ":CURRent?", backI);
        {
            std::lock_guard<std::mutex> sl(statusMutex);
            status.ch[ch - 1].setV = backV;
            status.ch[ch - 1].setI = backI;
        }
        if (ok) FD_INFO("udp3305s", "CH%d setpoints %.3f V / %.3f A", ch, backV, backI);
        return ok;
    }

    bool Measure(int ch, double& v, double& i, bool& on)
    {
        if (!ValidChannel(ch)) return false;
        std::lock_guard<std::mutex> lock(visaMutex);
        if (!connected.load() || inst == VI_NULL) { lastError = "not connected"; return false; }
        const std::string chStr = "CH" + std::to_string(ch);
        std::string resp;
        if (!QueryDouble("MEASure:VOLTage? " + chStr, v) ||
            !QueryDouble("MEASure:CURRent? " + chStr, i) ||
            !QueryInternal("OUTPut? " + chStr, resp)) {
            lastError = chStr + " measurement failed";
            return false;
        }
        on = ParseOn(resp);
        std::lock_guard<std::mutex> sl(statusMutex);
        status.ch[ch - 1].measV    = v;
        status.ch[ch - 1].measI    = i;
        status.ch[ch - 1].outputOn = on;
        return true;
    }

    bool StartPolling(int ms)
    {
        if (!connected.load()) { lastError = "not connected"; return false; }
        if (polling.load()) return true;
        intervalMs = ms > 0 ? ms : 500;
        for (int ch = 1; ch <= cfg.channels; ++ch) {
            sV[ch - 1] = &DataBus::Instance().Stream(topics::PsuVoltage(cfg.name, ch));
            sI[ch - 1] = &DataBus::Instance().Stream(topics::PsuCurrent(cfg.name, ch));
            if (cfg.legacyTopics) {
                sVL[ch - 1] = &DataBus::Instance().Stream(topics::PsuVoltage(ch));
                sIL[ch - 1] = &DataBus::Instance().Stream(topics::PsuCurrent(ch));
            }
        }
        polling.store(true);
        pollThread = std::thread(&Impl::PollLoop, this);
        FD_INFO("udp3305s", "polling started (%d ms)", intervalMs);
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
            Status s = GetStatus();
            bool ok = false;
            {
                std::lock_guard<std::mutex> lock(visaMutex);
                if (inst != VI_NULL) {
                    for (int ch = 1; ch <= cfg.channels; ++ch) {
                        ChannelStatus& c = s.ch[ch - 1];
                        const std::string chStr = "CH" + std::to_string(ch);
                        double v = 0.0;
                        if (QueryDouble("MEASure:VOLTage? " + chStr, v)) { c.measV = v; ok = true; }
                        if (QueryDouble("MEASure:CURRent? " + chStr, v)) { c.measI = v; }
                        // Also track the output state so front-panel changes surface.
                        std::string resp;
                        if (QueryInternal("OUTPut? " + chStr, resp)) c.outputOn = ParseOn(resp);
                    }
                }
            }
            if (ok) {
                s.valid = true;
                SetStatus(s);
                const double t = std::chrono::duration<double>(
                    std::chrono::steady_clock::now() - start).count();
                for (int ch = 0; ch < cfg.channels; ++ch) {
                    if (sV[ch])  sV[ch]->Publish(t, s.ch[ch].measV);
                    if (sI[ch])  sI[ch]->Publish(t, s.ch[ch].measI);
                    if (sVL[ch]) sVL[ch]->Publish(t, s.ch[ch].measV);
                    if (sIL[ch]) sIL[ch]->Publish(t, s.ch[ch].measI);
                }
            }
            const auto elapsed = std::chrono::steady_clock::now() - t0;
            const auto period  = std::chrono::milliseconds(intervalMs);
            if (elapsed < period) std::this_thread::sleep_for(period - elapsed);
        }
    }
};

} // namespace firedog

#else
// ------------------------------- Simulated stub -------------------------------
namespace firedog {

struct UDP3305S::Impl {
    Config            cfg;
    std::atomic<bool> connected{false};
    std::atomic<bool> polling{false};
    std::thread       pollThread;
    int               intervalMs = 500;
    DataStream*       sV[kMaxChannels] = {nullptr, nullptr, nullptr};
    DataStream*       sI[kMaxChannels] = {nullptr, nullptr, nullptr};
    DataStream*       sVL[kMaxChannels] = {nullptr, nullptr, nullptr};   // legacy "psu/ch<n>/v"
    DataStream*       sIL[kMaxChannels] = {nullptr, nullptr, nullptr};
    std::string       deviceId = "UNI-T UDP3305S (simulated)";
    std::string       lastError;

    mutable std::mutex statusMutex;
    Status             status;

    explicit Impl(Config c) : cfg(std::move(c))
    {
        cfg.channels = ClampChannels(cfg.channels);
        status.ch[0].setV = 5.0; status.ch[0].setI = 1.0;
        status.ch[1].setV = 5.0; status.ch[1].setI = 1.0;
        status.ch[2].setV = 3.3; status.ch[2].setI = 1.0;
    }
    ~Impl() { StopPolling(); }

    void SetStatus(const Status& s) { std::lock_guard<std::mutex> l(statusMutex); status = s; }
    Status GetStatus() const        { std::lock_guard<std::mutex> l(statusMutex); return status; }

    bool Connect()
    {
        connected.store(true);
        FD_INFO("udp3305s", "connected: %s", deviceId.c_str());
        return true;
    }
    void Disconnect()
    {
        StopPolling();
        std::lock_guard<std::mutex> l(statusMutex);
        for (int ch = 0; ch < kMaxChannels; ++ch) status.ch[ch].outputOn = false;
        connected.store(false);
    }

    bool SetOutput(int ch, bool on)
    {
        if (!connected.load() || ch < 1 || ch > cfg.channels) return false;
        std::lock_guard<std::mutex> l(statusMutex);
        status.ch[ch - 1].outputOn = on;
        FD_INFO("udp3305s", "CH%d output %s (simulated)", ch, on ? "ON" : "OFF");
        return true;
    }
    bool Apply(int ch, double volts, double amps)
    {
        if (!connected.load() || ch < 1 || ch > cfg.channels) return false;
        std::lock_guard<std::mutex> l(statusMutex);
        status.ch[ch - 1].setV = volts;
        status.ch[ch - 1].setI = amps;
        return true;
    }

    // The simulated load: a fixed resistor. The supply holds the voltage
    // setpoint until that would need more than the current limit, then holds
    // the limit (CC) and the voltage sags to I*R - so CV and CC both show up.
    static constexpr double kSimLoadOhm = 100.0;
    static void SimSettle(ChannelStatus& c)
    {
        const double v = c.outputOn ? std::min(c.setV, c.setI * kSimLoadOhm) : 0.0;
        c.measV = v;
        c.measI = c.outputOn ? v / kSimLoadOhm : 0.0;
    }

    bool Measure(int ch, double& v, double& i, bool& on)
    {
        if (!connected.load() || ch < 1 || ch > cfg.channels) return false;
        std::lock_guard<std::mutex> l(statusMutex);
        ChannelStatus& c = status.ch[ch - 1];
        SimSettle(c);
        v = c.measV; i = c.measI; on = c.outputOn;
        return true;
    }

    bool StartPolling(int ms)
    {
        if (!connected.load()) return false;
        if (polling.load()) return true;
        intervalMs = ms > 0 ? ms : 500;
        for (int ch = 1; ch <= cfg.channels; ++ch) {
            sV[ch - 1] = &DataBus::Instance().Stream(topics::PsuVoltage(cfg.name, ch));
            sI[ch - 1] = &DataBus::Instance().Stream(topics::PsuCurrent(cfg.name, ch));
            if (cfg.legacyTopics) {
                sVL[ch - 1] = &DataBus::Instance().Stream(topics::PsuVoltage(ch));
                sIL[ch - 1] = &DataBus::Instance().Stream(topics::PsuCurrent(ch));
            }
        }
        polling.store(true);
        pollThread = std::thread(&Impl::Loop, this);
        FD_INFO("udp3305s", "polling started (simulated, %d ms)", intervalMs);
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
            Status s = GetStatus();
            for (int ch = 0; ch < cfg.channels; ++ch) {
                ChannelStatus& c = s.ch[ch];
                SimSettle(c);   // CV until the limit, then CC (see SimSettle)
                if (c.outputOn) c.measI += 0.0005 * std::sin(t * 4.0 + ch);
                if (c.measI < 0.0) c.measI = 0.0;
            }
            s.valid = true;
            SetStatus(s);
            for (int ch = 0; ch < cfg.channels; ++ch) {
                if (sV[ch])  sV[ch]->Publish(t, s.ch[ch].measV);
                if (sI[ch])  sI[ch]->Publish(t, s.ch[ch].measI);
                if (sVL[ch]) sVL[ch]->Publish(t, s.ch[ch].measV);
                if (sIL[ch]) sIL[ch]->Publish(t, s.ch[ch].measI);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(intervalMs));
        }
    }
};

} // namespace firedog

#endif // HAS_VISA

// ---- Forwarding shell (identical for both Impls) ----
namespace firedog {

UDP3305S::UDP3305S(Config cfg) : p_(std::make_unique<Impl>(std::move(cfg))) {}
UDP3305S::~UDP3305S() = default;

bool UDP3305S::Connect()           { return p_->Connect(); }
void UDP3305S::Disconnect()        { p_->Disconnect(); }
bool UDP3305S::IsConnected() const { return p_->connected.load(); }

bool UDP3305S::OutputOn(int channel)  { return p_->SetOutput(channel, true); }
bool UDP3305S::OutputOff(int channel) { return p_->SetOutput(channel, false); }
bool UDP3305S::Apply(int channel, double volts, double amps)
{
    return p_->Apply(channel, volts, amps);
}

UDP3305S::Status UDP3305S::GetStatus() const { return p_->GetStatus(); }
bool UDP3305S::Measure(int channel, double& volts, double& amps, bool& outputOn)
{
    return p_->Measure(channel, volts, amps, outputOn);
}

bool UDP3305S::StartPolling(int intervalMs) { return p_->StartPolling(intervalMs); }
void UDP3305S::StopPolling()                { p_->StopPolling(); }
bool UDP3305S::IsPolling() const            { return p_->polling.load(); }

int         UDP3305S::Channels() const  { return p_->cfg.channels; }
std::string UDP3305S::DeviceId() const  { return p_->deviceId; }
std::string UDP3305S::LastError() const { return p_->lastError; }

} // namespace firedog
