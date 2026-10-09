// ACSController.cpp
//
// Direct ACS SPiiPlus (ACSC) usage. The open/enable/query/move calls are ported
// from the production ACS adapter (komcat/bobcat_engine + BobDog ACSController),
// condensed to a single class with a background position-poll thread. Without
// HAS_ACS it compiles a simulated stub so gantry status/positions still populate.
#include "hardware/motion/ACSController.h"

#include "core/Log.h"
#include "core/DataBus.h"
#include "core/Topics.h"

#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <sstream>
#include <thread>
#include <utility>

namespace firedog {
namespace {
std::vector<std::string> SplitAxes(const std::string& s)
{
    std::vector<std::string> out;
    std::istringstream ss(s);
    std::string tok;
    while (ss >> tok) out.push_back(tok);
    if (out.empty()) out = {"X", "Y", "Z"};
    return out;
}

// Stay in fast-poll mode this long after each RequestFastPoll() (steady-clock ns).
constexpr std::int64_t kFastWindowNs = 500'000'000;   // 500 ms

std::int64_t NowNs()
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::steady_clock::now().time_since_epoch()).count();
}
} // namespace
} // namespace firedog

#ifdef HAS_ACS
// --------------------------------- Real ACSC driver ---------------------------------
#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <winsock2.h>
#include <ws2tcpip.h>
#ifndef _WINSOCKAPI_
#define _WINSOCKAPI_
#endif
#endif
#include "ACSC.h"

namespace firedog {

struct ACSController::Impl {
    MotionConfig             cfg;
    std::string              type = "ACS";
    std::vector<std::string> axes;
    HANDLE                   handle = ACSC_INVALID;
    std::atomic<bool>        connected{false};
    std::atomic<bool>        polling{false};
    std::thread              pollThread;
    mutable std::mutex       mtx;
    std::map<std::string, double> positions;
    std::map<std::string, bool>   enabled;   // ACSC_MST_ENABLE per axis, polled
    std::string              lastError;
    std::vector<DataStream*> streams;       // one DataBus topic per axis
    std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
    std::mutex               pollMtx;       // guards fastUntilNs + the poll CV
    std::condition_variable  pollCv;
    std::int64_t             fastUntilNs = 0;

    explicit Impl(MotionConfig c) : cfg(std::move(c))
    {
        axes = SplitAxes(cfg.axes);
        for (const auto& a : axes) positions[a] = 0.0;
    }
    ~Impl() { Disconnect(); }

    bool Connect()
    {
        if (connected.load()) return true;
        char ip[64];
        strncpy_s(ip, sizeof(ip), cfg.ipAddress.c_str(), _TRUNCATE);
        handle = acsc_OpenCommEthernet(ip, cfg.port);
        if (handle == ACSC_INVALID) {
            const int err = acsc_GetLastError();
            lastError = "acsc_OpenCommEthernet failed (" + std::to_string(err) + ")";
            FD_ERROR("acs", "%s: %s", cfg.name.c_str(), lastError.c_str());
            return false;
        }
        connected.store(true);
        FD_INFO("acs", "%s connected @ %s:%d", cfg.name.c_str(), cfg.ipAddress.c_str(), cfg.port);

        // Enable each axis (index = position in the configured axis list).
        for (int i = 0; i < static_cast<int>(axes.size()); ++i) {
            if (!acsc_Enable(handle, i, NULL))
                FD_WARN("acs", "%s enable axis %s failed (%d)", cfg.name.c_str(),
                        axes[i].c_str(), acsc_GetLastError());
        }

        streams.resize(axes.size());
        for (size_t i = 0; i < axes.size(); ++i)
            streams[i] = &DataBus::Instance().Stream(topics::MotionAxis(cfg.name, axes[i]));

        polling.store(true);
        pollThread = std::thread(&Impl::PollLoop, this);
        return true;
    }

    void Disconnect()
    {
        bool was;
        { std::lock_guard<std::mutex> lk(pollMtx); was = polling.exchange(false); }
        if (was) {
            pollCv.notify_one();
            if (pollThread.joinable()) pollThread.join();
        }
        if (connected.exchange(false)) {
            std::lock_guard<std::mutex> lock(mtx);
            acsc_KillAll(handle, NULL);
            acsc_CloseComm(handle);
            handle = ACSC_INVALID;
            enabled.clear();
        }
    }

    void PollLoop()
    {
        std::vector<double> vals(axes.size(), 0.0);
        while (polling.load()) {
            {
                std::lock_guard<std::mutex> lock(mtx);
                for (int i = 0; i < static_cast<int>(axes.size()); ++i) {
                    double pos = 0.0;
                    if (acsc_GetFPosition(handle, i, &pos, NULL)) positions[axes[i]] = pos;
                    vals[i] = positions[axes[i]];
                    int state = 0;
                    if (acsc_GetMotorState(handle, i, &state, NULL))
                        enabled[axes[i]] = (state & ACSC_MST_ENABLE) != 0;
                }
            }
            // Broadcast each axis to its DataBus topic (outside the lock).
            const double t = std::chrono::duration<double>(
                                 std::chrono::steady_clock::now() - start).count();
            for (size_t i = 0; i < streams.size(); ++i)
                if (streams[i]) streams[i]->Publish(t, vals[i]);

            // Adaptive cadence: 10 Hz while a consumer is asking, else 1 Hz idle.
            std::unique_lock<std::mutex> lk(pollMtx);
            if (!polling.load()) break;
            const auto period = (NowNs() < fastUntilNs) ? std::chrono::milliseconds(100)
                                                        : std::chrono::milliseconds(1000);
            pollCv.wait_for(lk, period);
        }
    }

    void RequestFastPoll()
    {
        std::lock_guard<std::mutex> lk(pollMtx);
        const std::int64_t now = NowNs();
        const bool wasIdle = fastUntilNs <= now;
        fastUntilNs = now + kFastWindowNs;
        if (wasIdle) pollCv.notify_one();
    }

    std::map<std::string, double> Positions() const
    {
        std::lock_guard<std::mutex> lock(mtx);
        return positions;
    }

    int AxisIndex(const std::string& axis) const
    {
        for (int i = 0; i < static_cast<int>(axes.size()); ++i)
            if (axes[i] == axis) return i;
        return -1;
    }

    bool MoveAbsolute(const std::string& axis, double position)
    {
        std::lock_guard<std::mutex> lock(mtx);
        if (!connected.load()) { lastError = "not connected"; return false; }
        const int idx = AxisIndex(axis);
        if (idx < 0) { lastError = "invalid axis: " + axis; return false; }
        // Immediate single-axis PTP (not multipoint): issuing a new point while
        // moving re-targets the running move, so jog presses override the
        // previous instead of queuing points. flags=0 => start immediately.
        if (!acsc_ToPoint(handle, 0, idx, position, NULL)) {
            const int err = acsc_GetLastError();
            lastError = "acsc_ToPoint failed (" + std::to_string(err) + ")";
            FD_ERROR("acs", "%s move %s failed: %s", cfg.name.c_str(), axis.c_str(), lastError.c_str());
            return false;
        }
        return true;
    }

    bool MoveAbsolute(const std::vector<std::pair<std::string, double>>& targets)
    {
        if (targets.empty()) return true;
        std::lock_guard<std::mutex> lock(mtx);
        if (!connected.load()) { lastError = "not connected"; return false; }
        std::vector<int>    idxArr;
        std::vector<double> pts;
        idxArr.reserve(targets.size() + 1);
        pts.reserve(targets.size());
        for (const auto& kv : targets) {
            const int i = AxisIndex(kv.first);
            if (i < 0) { lastError = "invalid axis: " + kv.first; return false; }
            idxArr.push_back(i);
            pts.push_back(kv.second);
        }
        idxArr.push_back(-1);   // ACSC axis arrays are -1 terminated
        // One ToPointM/GoM over all axes -> coordinated simultaneous motion.
        if (!acsc_ToPointM(handle, ACSC_AMF_WAIT, idxArr.data(), pts.data(), NULL) ||
            !acsc_GoM(handle, idxArr.data(), NULL)) {
            const int err = acsc_GetLastError();
            lastError = "acsc multi-move failed (" + std::to_string(err) + ")";
            FD_ERROR("acs", "%s multi-move failed: %s", cfg.name.c_str(), lastError.c_str());
            return false;
        }
        return true;
    }

    bool SetVelocity(double v)
    {
        std::lock_guard<std::mutex> lock(mtx);
        if (!connected.load()) { lastError = "not connected"; return false; }
        bool ok = true;
        for (int i = 0; i < static_cast<int>(axes.size()); ++i)
            if (!acsc_SetVelocity(handle, i, v, NULL)) ok = false;
        if (!ok) {
            lastError = "acsc_SetVelocity failed (" + std::to_string(acsc_GetLastError()) + ")";
            FD_ERROR("acs", "%s set velocity failed: %s", cfg.name.c_str(), lastError.c_str());
        } else {
            FD_INFO("acs", "%s velocity (all axes) = %.3f mm/s", cfg.name.c_str(), v);
        }
        return ok;
    }

    bool StopAll()
    {
        std::lock_guard<std::mutex> lock(mtx);
        return connected.load() && acsc_KillAll(handle, NULL);
    }

    std::map<std::string, bool> AxisEnabled() const
    {
        std::lock_guard<std::mutex> lock(mtx);
        return enabled;
    }

    // The reply only says the controller accepted the command; the ENABLE bit
    // the poll reads next is what the indicator shows.
    bool SetAxisEnabled(const std::string& axis, bool on)
    {
        std::lock_guard<std::mutex> lock(mtx);
        if (!connected.load()) { lastError = "not connected"; return false; }
        const int idx = AxisIndex(axis);
        if (idx < 0) { lastError = "invalid axis: " + axis; return false; }
        const int ok = on ? acsc_Enable(handle, idx, NULL) : acsc_Disable(handle, idx, NULL);
        if (!ok) {
            lastError = std::string(on ? "acsc_Enable" : "acsc_Disable") + " failed (" +
                        std::to_string(acsc_GetLastError()) + ")";
            FD_ERROR("acs", "%s axis %s: %s", cfg.name.c_str(), axis.c_str(), lastError.c_str());
            return false;
        }
        FD_INFO("acs", "%s axis %s %s", cfg.name.c_str(), axis.c_str(), on ? "enabled" : "disabled");
        return true;
    }
};

} // namespace firedog

#else
// --------------------------------- Simulated stub ---------------------------------
namespace firedog {

struct ACSController::Impl {
    MotionConfig             cfg;
    std::string              type = "ACS";
    std::vector<std::string> axes;
    std::atomic<bool>        connected{false};
    std::atomic<bool>        polling{false};
    std::thread              pollThread;
    mutable std::mutex       mtx;
    std::map<std::string, double> targets;
    std::map<std::string, double> positions;   // current (simulated) position
    std::map<std::string, bool>   enabled;     // servo state; empty while disconnected
    std::vector<DataStream*> streams;          // one DataBus topic per axis
    std::string              lastError;
    std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
    std::mutex               pollMtx;          // guards fastUntilNs + the poll CV
    std::condition_variable  pollCv;
    std::int64_t             fastUntilNs = 0;

    explicit Impl(MotionConfig c) : cfg(std::move(c))
    {
        axes = SplitAxes(cfg.axes);
        for (const auto& a : axes) { targets[a] = 0.0; positions[a] = 0.0; }
    }
    ~Impl() { Disconnect(); }

    bool Connect()
    {
        if (connected.load()) return true;
        connected.store(true);
        FD_INFO("acs", "%s connected (simulated) @ %s:%d", cfg.name.c_str(),
                cfg.ipAddress.c_str(), cfg.port);
        {
            // The real Connect enables every axis.
            std::lock_guard<std::mutex> lock(mtx);
            for (const auto& a : axes) enabled[a] = true;
        }
        streams.resize(axes.size());
        for (size_t i = 0; i < axes.size(); ++i)
            streams[i] = &DataBus::Instance().Stream(topics::MotionAxis(cfg.name, axes[i]));
        polling.store(true);
        pollThread = std::thread(&Impl::PollLoop, this);
        return true;
    }
    void Disconnect()
    {
        bool was;
        { std::lock_guard<std::mutex> lk(pollMtx); was = polling.exchange(false); }
        if (was) { pollCv.notify_one(); if (pollThread.joinable()) pollThread.join(); }
        connected.store(false);
        std::lock_guard<std::mutex> lock(mtx);
        enabled.clear();
    }

    void PollLoop()
    {
        while (polling.load()) {
            const double t = std::chrono::duration<double>(
                                 std::chrono::steady_clock::now() - start).count();
            std::vector<double> vals(axes.size(), 0.0);
            {
                std::lock_guard<std::mutex> lock(mtx);
                int k = 0;
                for (const auto& a : axes) {
                    const double v = targets.at(a) + 0.003 * std::sin(t * 0.5 + k * 2.1);
                    positions[a] = v;
                    vals[k] = v;
                    ++k;
                }
            }
            for (size_t i = 0; i < streams.size(); ++i)
                if (streams[i]) streams[i]->Publish(t, vals[i]);

            // Adaptive cadence: ~20 Hz while asked, else 2 Hz idle.
            std::unique_lock<std::mutex> lk(pollMtx);
            if (!polling.load()) break;
            const auto period = (NowNs() < fastUntilNs) ? std::chrono::milliseconds(50)
                                                        : std::chrono::milliseconds(500);
            pollCv.wait_for(lk, period);
        }
    }

    void RequestFastPoll()
    {
        std::lock_guard<std::mutex> lk(pollMtx);
        const std::int64_t now = NowNs();
        const bool wasIdle = fastUntilNs <= now;
        fastUntilNs = now + kFastWindowNs;
        if (wasIdle) pollCv.notify_one();
    }

    std::map<std::string, double> Positions() const
    {
        std::lock_guard<std::mutex> lock(mtx);
        return positions;
    }

    bool MoveAbsolute(const std::string& axis, double position)
    {
        std::lock_guard<std::mutex> lock(mtx);
        if (!connected.load()) { lastError = "not connected"; return false; }
        if (!Enabled(axis)) return false;
        targets[axis] = position;
        return true;
    }
    bool MoveAbsolute(const std::vector<std::pair<std::string, double>>& t)
    {
        std::lock_guard<std::mutex> lock(mtx);
        if (!connected.load()) { lastError = "not connected"; return false; }
        for (const auto& kv : t)
            if (!Enabled(kv.first)) return false;
        for (const auto& kv : t) targets[kv.first] = kv.second;
        return true;
    }
    // Caller holds mtx. The controller refuses to move a disabled motor, and so
    // does the stub, so a sim run shows the same failure the machine would.
    bool Enabled(const std::string& axis)
    {
        const auto it = enabled.find(axis);
        if (it == enabled.end()) { lastError = "invalid axis: " + axis; return false; }
        if (!it->second) {
            lastError = "axis " + axis + " is disabled";
            FD_ERROR("acs", "%s move refused: %s (simulated)", cfg.name.c_str(), lastError.c_str());
            return false;
        }
        return true;
    }
    std::map<std::string, bool> AxisEnabled() const
    {
        std::lock_guard<std::mutex> lock(mtx);
        return enabled;
    }
    bool SetAxisEnabled(const std::string& axis, bool on)
    {
        std::lock_guard<std::mutex> lock(mtx);
        if (!connected.load()) { lastError = "not connected"; return false; }
        const auto it = enabled.find(axis);
        if (it == enabled.end()) { lastError = "invalid axis: " + axis; return false; }
        it->second = on;
        FD_INFO("acs", "%s axis %s %s (simulated)", cfg.name.c_str(), axis.c_str(),
                on ? "enabled" : "disabled");
        return true;
    }
    bool SetVelocity(double v)
    {
        FD_INFO("acs", "%s velocity (all axes) = %.3f mm/s (simulated)", cfg.name.c_str(), v);
        return connected.load();
    }
    bool StopAll() { return connected.load(); }
};

} // namespace firedog

#endif // HAS_ACS

// ---- Forwarding shell (identical for both Impls) ----
namespace firedog {

ACSController::ACSController(MotionConfig cfg) : p_(std::make_unique<Impl>(std::move(cfg))) {}
ACSController::~ACSController() = default;

bool ACSController::Connect()            { return p_->Connect(); }
void ACSController::Disconnect()         { p_->Disconnect(); }
bool ACSController::IsConnected() const  { return p_->connected.load(); }
const std::string& ACSController::Name() const { return p_->cfg.name; }
const std::string& ACSController::Type() const { return p_->type; }
std::vector<std::string> ACSController::Axes() const { return p_->axes; }
std::map<std::string, double> ACSController::Positions() const { return p_->Positions(); }
bool ACSController::MoveAbsolute(const std::string& a, double p) { return p_->MoveAbsolute(a, p); }
bool ACSController::MoveAbsolute(const std::vector<std::pair<std::string, double>>& t) { return p_->MoveAbsolute(t); }
bool ACSController::SetVelocity(double v) { return p_->SetVelocity(v); }
bool ACSController::StopAll()            { return p_->StopAll(); }
std::map<std::string, bool> ACSController::AxisEnabled() const { return p_->AxisEnabled(); }
bool ACSController::SetAxisEnabled(const std::string& a, bool on) { return p_->SetAxisEnabled(a, on); }
void ACSController::RequestFastPoll()    { p_->RequestFastPoll(); }
std::string ACSController::LastError() const { return p_->lastError; }

} // namespace firedog
