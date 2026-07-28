// CLD101x.cpp
//
// Direct VISA usage. Open sequence, SCPI strings, and unit conversions are
// ported from bobcat_engine's CLD101xDirectAdapter (viWrite/viRead with '\n'
// termchar, 2 s I/O timeout, 200 ms USB settle after open). On/off commands
// read the state back (output1|2:state?) so the UI shows the instrument's
// truth — the lesson learned on the Keithley page.
#include "hardware/instruments/CLD101x.h"

#include "core/DataBus.h"
#include "core/Log.h"

#include <atomic>
#include <chrono>
#include <cmath>
#include <mutex>
#include <sstream>
#include <thread>
#include <utility>

namespace firedog {
namespace {
// bobcat parses "1" or "ON" anywhere in the response as ON.
bool ParseOn(const std::string& resp)
{
    return resp.find('1') != std::string::npos || resp.find("ON") != std::string::npos;
}
// IEEE-488.2/SCPI "not a number" sentinel (+/-9.9e37) — returned e.g. for a
// disconnected thermistor. Anything that big is a fault flag, not a reading.
bool ScpiValueValid(double v)
{
    return std::fabs(v) < 9.0e36;
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

struct CLD101x::Impl {
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
    DataStream*       sTemp = nullptr;
    DataStream*       sCurr = nullptr;

    // UI-facing status cache, off visaMutex so per-frame reads never wait
    // behind an in-flight query (the Keithley cfgMutex pattern).
    mutable std::mutex statusMutex;
    Status             status;

    explicit Impl(Config c) : cfg(std::move(c)) {}
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
            FD_ERROR("cld101x", "%s", lastError.c_str());
            return false;
        }
        if (viOpen(rm, cfg.visaResource.c_str(), VI_NULL, 5000, &inst) < VI_SUCCESS) {
            lastError = "viOpen failed for " + cfg.visaResource;
            FD_ERROR("cld101x", "%s", lastError.c_str());
            viClose(rm); rm = VI_NULL;
            return false;
        }
        viSetAttribute(inst, VI_ATTR_TMO_VALUE, 2000);
        viSetAttribute(inst, VI_ATTR_TERMCHAR_EN, VI_TRUE);
        viSetAttribute(inst, VI_ATTR_TERMCHAR, '\n');
        viFlush(inst, VI_READ_BUF_DISCARD | VI_WRITE_BUF_DISCARD);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));   // USB settle

        WriteInternal("*CLS");
        std::string id;
        if (!QueryInternal("*IDN?", id)) {
            lastError = "*IDN? failed";
            FD_ERROR("cld101x", "%s", lastError.c_str());
            viClose(inst); viClose(rm); inst = rm = VI_NULL;
            return false;
        }
        deviceId = id;
        connected.store(true);

        // Read the device's LIVE state — nothing is written to the outputs.
        Status s;
        std::string resp;
        if (QueryInternal("output1:state?", resp)) s.laserOn = ParseOn(resp);
        if (QueryInternal("output2:state?", resp)) s.tecOn   = ParseOn(resp);
        double v = 0.0;
        if (QueryDouble("source2:temperature:spoint?", v) && ScpiValueValid(v))
            s.tempSetC = v;
        if (QueryDouble("source1:current:level:amplitude?", v) && ScpiValueValid(v))
            s.laserSetMA = v * 1000.0;
        s.valid = false;   // actuals arrive with the first poll
        SetStatus(s);

        FD_INFO("cld101x", "connected: %s (laser %s, TEC %s)", deviceId.c_str(),
                s.laserOn ? "ON" : "off", s.tecOn ? "ON" : "off");
        return true;
    }

    void Disconnect()
    {
        StopPolling();
        std::lock_guard<std::mutex> lock(visaMutex);
        if (!connected.load()) return;
        // bobcat behavior: no implicit laser/TEC off — device state persists.
        if (inst != VI_NULL) { viClose(inst); inst = VI_NULL; }
        if (rm   != VI_NULL) { viClose(rm);   rm   = VI_NULL; }
        connected.store(false);
    }

    // Switch an output and read the state back; updates the status cache.
    bool SetOutput(int channel, bool on)
    {
        std::lock_guard<std::mutex> lock(visaMutex);
        if (!connected.load()) { lastError = "not connected"; return false; }
        const std::string base = (channel == 1) ? "output1:state" : "output2:state";
        const bool wrote = WriteInternal(base + (on ? " on" : " off"));
        bool actual = on;
        std::string resp;
        if (QueryInternal(base + "?", resp)) actual = ParseOn(resp);
        {
            std::lock_guard<std::mutex> sl(statusMutex);
            if (channel == 1) status.laserOn = actual;
            else              status.tecOn   = actual;
        }
        const char* what = (channel == 1) ? "laser" : "TEC";
        if (wrote && actual == on)
            FD_INFO("cld101x", "%s output %s (verified)", what, on ? "ON" : "OFF");
        else
            FD_WARN("cld101x", "%s output %s requested but reads %s", what,
                    on ? "ON" : "OFF", actual ? "ON" : "OFF");
        return wrote && actual == on;
    }

    bool SetTemperature(double c)
    {
        std::lock_guard<std::mutex> lock(visaMutex);
        if (!connected.load()) { lastError = "not connected"; return false; }
        std::ostringstream cmd;
        cmd << "source2:temperature:spoint " << c;
        const bool ok = WriteInternal(cmd.str());
        double back = c;
        if (QueryDouble("source2:temperature:spoint?", back) && ScpiValueValid(back)) {
            std::lock_guard<std::mutex> sl(statusMutex);
            status.tempSetC = back;
        }
        if (ok) FD_INFO("cld101x", "temperature setpoint %.2f C", back);
        return ok;
    }

    bool SetLaserCurrentMA(double mA)
    {
        std::lock_guard<std::mutex> lock(visaMutex);
        if (!connected.load()) { lastError = "not connected"; return false; }
        std::ostringstream cmd;
        cmd << "source1:current:level:amplitude " << (mA / 1000.0);   // amps on the wire
        const bool ok = WriteInternal(cmd.str());
        double backA = mA / 1000.0;
        if (QueryDouble("source1:current:level:amplitude?", backA) && ScpiValueValid(backA)) {
            std::lock_guard<std::mutex> sl(statusMutex);
            status.laserSetMA = backA * 1000.0;
        }
        if (ok) FD_INFO("cld101x", "laser current setpoint %.1f mA", backA * 1000.0);
        return ok;
    }

    bool StartPolling(int ms, const std::string& tTemp, const std::string& tCurr)
    {
        if (!connected.load()) { lastError = "not connected"; return false; }
        if (polling.load()) return true;
        intervalMs = ms > 0 ? ms : 500;
        sTemp = tTemp.empty() ? nullptr : &DataBus::Instance().Stream(tTemp);
        sCurr = tCurr.empty() ? nullptr : &DataBus::Instance().Stream(tCurr);
        polling.store(true);
        pollThread = std::thread(&Impl::PollLoop, this);
        FD_INFO("cld101x", "polling started (%d ms)", intervalMs);
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
                    double v = 0.0;
                    if (QueryDouble("SENSe2:temperature:data?", v)) {
                        // 9.9e37 = SCPI NaN (thermistor open / no reading).
                        s.tempValid = ScpiValueValid(v);
                        if (s.tempValid) s.tempC = v;
                        ok = true;
                    }
                    // bobcat guard: the measurement is meaningless with the
                    // laser off — report 0 without touching the instrument.
                    if (s.laserOn) {
                        if (QueryDouble("sense3:current:dc:data?", v) && ScpiValueValid(v))
                            s.laserActMA = v * 1000.0;
                    } else {
                        s.laserActMA = 0.0;
                    }
                }
            }
            if (ok) {
                s.valid = true;
                SetStatus(s);
                const double t = std::chrono::duration<double>(
                    std::chrono::steady_clock::now() - start).count();
                if (sTemp && s.tempValid) sTemp->Publish(t, s.tempC);
                if (sCurr) sCurr->Publish(t, s.laserActMA);
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

struct CLD101x::Impl {
    Config            cfg;
    std::atomic<bool> connected{false};
    std::atomic<bool> polling{false};
    std::thread       pollThread;
    int               intervalMs = 500;
    DataStream*       sTemp = nullptr;
    DataStream*       sCurr = nullptr;
    std::string       deviceId = "Thorlabs CLD101x (simulated)";
    std::string       lastError;

    mutable std::mutex statusMutex;
    Status             status;

    explicit Impl(Config c) : cfg(std::move(c))
    {
        status.tempC      = 23.0;                       // ambient start
        status.tempSetC   = cfg.defaultTemperature;
        status.laserSetMA = cfg.defaultLaserCurrentMA;
    }
    ~Impl() { StopPolling(); }

    void SetStatus(const Status& s) { std::lock_guard<std::mutex> l(statusMutex); status = s; }
    Status GetStatus() const        { std::lock_guard<std::mutex> l(statusMutex); return status; }

    bool Connect()
    {
        connected.store(true);
        FD_INFO("cld101x", "connected: %s", deviceId.c_str());
        return true;
    }
    void Disconnect() { StopPolling(); connected.store(false); }

    bool SetOutput(int channel, bool on)
    {
        if (!connected.load()) return false;
        std::lock_guard<std::mutex> l(statusMutex);
        if (channel == 1) status.laserOn = on;
        else              status.tecOn   = on;
        FD_INFO("cld101x", "%s output %s (simulated)",
                channel == 1 ? "laser" : "TEC", on ? "ON" : "OFF");
        return true;
    }
    bool SetTemperature(double c)
    {
        if (!connected.load()) return false;
        std::lock_guard<std::mutex> l(statusMutex);
        status.tempSetC = c;
        return true;
    }
    bool SetLaserCurrentMA(double mA)
    {
        if (!connected.load()) return false;
        std::lock_guard<std::mutex> l(statusMutex);
        status.laserSetMA = mA;
        return true;
    }

    bool StartPolling(int ms, const std::string& tTemp, const std::string& tCurr)
    {
        if (!connected.load()) return false;
        if (polling.load()) return true;
        intervalMs = ms > 0 ? ms : 500;
        sTemp = tTemp.empty() ? nullptr : &DataBus::Instance().Stream(tTemp);
        sCurr = tCurr.empty() ? nullptr : &DataBus::Instance().Stream(tCurr);
        polling.store(true);
        pollThread = std::thread(&Impl::Loop, this);
        FD_INFO("cld101x", "polling started (simulated, %d ms)", intervalMs);
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
            // First-order settle toward the setpoint when the TEC is on;
            // drift back toward 23 C ambient when off. Laser follows setpoint.
            const double target = s.tecOn ? s.tempSetC : 23.0;
            s.tempC += (target - s.tempC) * 0.10 + 0.02 * std::sin(t * 3.0);
            s.tempValid  = true;
            s.laserActMA = s.laserOn ? s.laserSetMA + 0.3 * std::sin(t * 5.0) : 0.0;
            s.valid = true;
            SetStatus(s);
            if (sTemp) sTemp->Publish(t, s.tempC);
            if (sCurr) sCurr->Publish(t, s.laserActMA);
            std::this_thread::sleep_for(std::chrono::milliseconds(intervalMs));
        }
    }
};

} // namespace firedog

#endif // HAS_VISA

// ---- Forwarding shell (identical for both Impls) ----
namespace firedog {

CLD101x::CLD101x(Config cfg) : p_(std::make_unique<Impl>(std::move(cfg))) {}
CLD101x::~CLD101x() = default;

bool CLD101x::Connect()           { return p_->Connect(); }
void CLD101x::Disconnect()        { p_->Disconnect(); }
bool CLD101x::IsConnected() const { return p_->connected.load(); }

bool CLD101x::TecOn()   { return p_->SetOutput(2, true); }
bool CLD101x::TecOff()  { return p_->SetOutput(2, false); }
bool CLD101x::LaserOn() { return p_->SetOutput(1, true); }
bool CLD101x::LaserOff(){ return p_->SetOutput(1, false); }

bool CLD101x::SetTemperature(double c)     { return p_->SetTemperature(c); }
bool CLD101x::SetLaserCurrentMA(double mA) { return p_->SetLaserCurrentMA(mA); }

CLD101x::Status CLD101x::GetStatus() const { return p_->GetStatus(); }

bool CLD101x::StartPolling(int ms, const std::string& tTemp, const std::string& tCurr)
{
    return p_->StartPolling(ms, tTemp, tCurr);
}
void CLD101x::StopPolling()     { p_->StopPolling(); }
bool CLD101x::IsPolling() const { return p_->polling.load(); }

std::string CLD101x::DeviceId() const  { return p_->deviceId; }
std::string CLD101x::LastError() const { return p_->lastError; }

} // namespace firedog
