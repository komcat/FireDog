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

#include <algorithm>
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
    std::map<int, bool>           bufRunning;   // ACSC_PST_RUN per started buffer, polled
    MotionProfile            profile;       // cached VEL/ACC/DEC/JERK/KDEC (mtx)
    std::atomic<bool>        profileReadReq{false};   // poll thread re-reads the profile
    std::vector<AxisProfile> defaults;      // first complete read after connect (mtx)
    bool                     dynamicOn = false;       // controller holds a dynamic profile (mtx)
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

    // The last error as "<code>: <controller's own text>". A bare number from
    // the 3000s is a controller code no header here describes, so ask the
    // controller what it means. Falls back to the number alone when the lookup
    // fails (no connection yet, or a library-side code it does not know).
    std::string LastErrorText() const
    {
        const int code = acsc_GetLastError();
        std::string out = std::to_string(code);
        if (handle == ACSC_INVALID) return out;
        char buf[256] = {};
        int  got = 0;
        if (acsc_GetErrorString(handle, code, buf, static_cast<int>(sizeof(buf)) - 1, &got) &&
            got > 0) {
            buf[std::min(got, static_cast<int>(sizeof(buf)) - 1)] = '\0';
            out += ": ";
            out += buf;
        }
        return out;
    }

    bool Connect()
    {
        if (connected.load()) return true;
        char ip[64];
        strncpy_s(ip, sizeof(ip), cfg.ipAddress.c_str(), _TRUNCATE);
        handle = acsc_OpenCommEthernet(ip, cfg.port);
        if (handle == ACSC_INVALID) {
            lastError = "acsc_OpenCommEthernet failed (" + LastErrorText() + ")";
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

        profileReadReq.store(true);   // the poll thread's first pass reads it
        polling.store(true);
        pollThread = std::thread(&Impl::PollLoop, this);
        return true;
    }

    // Caller holds mtx. One exchange per value per axis; an axis whose read
    // fails is reported invalid rather than half-filled, and the profile is
    // logged once per read so the log says what the controller was set to.
    void ReadProfileLocked()
    {
        MotionProfile pr;
        pr.seq = profile.seq + 1;
        for (int i = 0; i < static_cast<int>(axes.size()); ++i) {
            AxisProfile a;
            a.axis  = axes[i];
            a.valid = acsc_GetVelocity(handle, i, &a.vel, NULL) &&
                      acsc_GetAcceleration(handle, i, &a.acc, NULL) &&
                      acsc_GetDeceleration(handle, i, &a.dec, NULL) &&
                      acsc_GetJerk(handle, i, &a.jerk, NULL) &&
                      acsc_GetKillDeceleration(handle, i, &a.kdec, NULL);
            if (!a.valid && pr.error.empty())
                pr.error = "axis " + axes[i] + ": " + LastErrorText();
            pr.axes.push_back(a);
        }
        pr.valid = true;
        // The DEFAULT profile is the first complete read after connect, taken
        // before FireDog has written any ACC/DEC/JERK - so it is what the
        // controller was configured with, and what every ordinary move restores.
        const bool complete = std::all_of(pr.axes.begin(), pr.axes.end(),
                                          [](const AxisProfile& a) { return a.valid; });
        if (defaults.empty() && !dynamicOn && complete && !pr.axes.empty()) {
            defaults = pr.axes;
            FD_INFO("acs", "%s default profile captured", cfg.name.c_str());
        }
        pr.defaults = defaults;
        pr.dynamic  = dynamicOn;
        profile     = std::move(pr);
        if (!profile.error.empty())
            FD_WARN("acs", "%s profile read: %s", cfg.name.c_str(), profile.error.c_str());
        for (const AxisProfile& a : profile.axes)
            if (a.valid)
                FD_INFO("acs", "%s %s: VEL %g ACC %g DEC %g JERK %g KDEC %g", cfg.name.c_str(),
                        a.axis.c_str(), a.vel, a.acc, a.dec, a.jerk, a.kdec);
    }

    // Caller holds mtx. ACC, DEC, JERK of one axis; KDEC is never written.
    bool WriteAxisProfileLocked(int i, double acc, double dec, double jerk)
    {
        return acsc_SetAcceleration(handle, i, acc, NULL) &&
               acsc_SetDeceleration(handle, i, dec, NULL) &&
               acsc_SetJerk(handle, i, jerk, NULL);
    }

    // Caller holds mtx. Puts the default profile back if a dynamic one is on
    // the controller; a no-op (no exchange) otherwise. A failed restore leaves
    // dynamicOn set, so the next move tries again rather than running on a
    // profile nobody asked for.
    bool RestoreDefaultsLocked()
    {
        if (!dynamicOn) return true;
        for (int i = 0; i < static_cast<int>(defaults.size()) &&
                        i < static_cast<int>(axes.size()); ++i) {
            const AxisProfile& d = defaults[static_cast<std::size_t>(i)];
            if (!WriteAxisProfileLocked(i, d.acc, d.dec, d.jerk)) {
                lastError = "restoring the default profile on " + axes[i] + " failed (" +
                            LastErrorText() + ")";
                FD_ERROR("acs", "%s: %s", cfg.name.c_str(), lastError.c_str());
                return false;
            }
        }
        dynamicOn = false;
        profileReadReq.store(true);
        FD_INFO("acs", "%s default profile restored", cfg.name.c_str());
        return true;
    }

    // Caller holds mtx. ACC/DEC/JERK of every axis from that axis's own VEL,
    // read off the controller now (not the cache: a velocity set a moment ago
    // must be the one the profile is derived from).
    bool ApplyDynamicLocked(const DynamicProfile& dyn)
    {
        if (defaults.empty()) {
            lastError = "the default profile has not been read yet - no dynamic profile "
                        "until there is one to restore";
            return false;
        }
        if (!(dyn.accPerVel > 0.0) || !(dyn.jerkPerAcc > 0.0)) {
            lastError = "dynamic profile ratios must be > 0";
            return false;
        }
        for (int i = 0; i < static_cast<int>(axes.size()); ++i) {
            double vel = 0.0;
            if (!acsc_GetVelocity(handle, i, &vel, NULL) || !(vel > 0.0)) {
                lastError = "reading VEL of " + axes[i] + " failed (" + LastErrorText() + ")";
                RestoreDefaultsLocked();
                return false;
            }
            const double acc  = vel * dyn.accPerVel;
            const double jerk = acc * dyn.jerkPerAcc;
            dynamicOn = true;   // from the first write on, something must be restored
            if (!WriteAxisProfileLocked(i, acc, acc, jerk)) {
                const std::string why = "writing the dynamic profile on " + axes[i] +
                                        " failed (" + LastErrorText() + ")";
                RestoreDefaultsLocked();
                lastError = why;
                FD_ERROR("acs", "%s: %s", cfg.name.c_str(), lastError.c_str());
                return false;
            }
            FD_INFO("acs", "%s %s dynamic profile: VEL %g ACC %g DEC %g JERK %g",
                    cfg.name.c_str(), axes[i].c_str(), vel, acc, acc, jerk);
        }
        profileReadReq.store(true);
        return true;
    }

    bool RestoreDefaultProfile()
    {
        std::lock_guard<std::mutex> lock(mtx);
        if (!connected.load()) return true;   // nothing to restore on
        return RestoreDefaultsLocked();
    }

    MotionProfile Profile() const
    {
        std::lock_guard<std::mutex> lock(mtx);
        return profile;
    }

    void RequestProfileRead()
    {
        profileReadReq.store(true);
        std::lock_guard<std::mutex> lk(pollMtx);
        pollCv.notify_one();   // answer within one exchange, not the idle period
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
            RestoreDefaultsLocked();   // leave the controller as it was found
            acsc_KillAll(handle, NULL);
            acsc_CloseComm(handle);
            handle = ACSC_INVALID;
            enabled.clear();
            bufRunning.clear();
            const unsigned seq = profile.seq + 1;
            profile     = MotionProfile{};
            profile.seq = seq;
            defaults.clear();
            dynamicOn = false;
        }
    }

    void PollLoop()
    {
        std::vector<double> vals(axes.size(), 0.0);
        while (polling.load()) {
            {
                std::lock_guard<std::mutex> lock(mtx);
                // Only the buffers this session started, BEFORE the positions:
                // a "finished" seen here comes with positions read after the
                // program ended. Read and stored under the lock RunBuffer takes,
                // so a stale poll never overwrites its optimistic "running".
                if (profileReadReq.exchange(false)) ReadProfileLocked();
                for (auto& [buf, running] : bufRunning) {
                    int pst = 0;
                    if (acsc_GetProgramState(handle, buf, &pst, NULL))
                        running = (pst & ACSC_PST_RUN) != 0;
                }
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
        if (!RestoreDefaultsLocked()) return false;   // an ordinary move never runs dynamic
        // Immediate single-axis PTP (not multipoint): issuing a new point while
        // moving re-targets the running move, so jog presses override the
        // previous instead of queuing points. flags=0 => start immediately.
        if (!acsc_ToPoint(handle, 0, idx, position, NULL)) {
            lastError = "acsc_ToPoint failed (" + LastErrorText() + ")";
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
        if (!RestoreDefaultsLocked()) return false;   // an ordinary move never runs dynamic
        return MoveCoordLocked(targets);
    }

    // The one opt-in: profile first, then the same coordinated move. The axes
    // are validated BEFORE the profile is written, so a bad target never
    // leaves a dynamic profile behind for a move that was refused.
    bool MoveAbsoluteDynamic(const std::vector<std::pair<std::string, double>>& targets,
                             const DynamicProfile& dyn)
    {
        if (targets.empty()) return true;
        std::lock_guard<std::mutex> lock(mtx);
        if (!connected.load()) { lastError = "not connected"; return false; }
        for (const auto& kv : targets)
            if (AxisIndex(kv.first) < 0) { lastError = "invalid axis: " + kv.first; return false; }
        if (!ApplyDynamicLocked(dyn)) {
            FD_ERROR("acs", "%s dynamic move refused: %s", cfg.name.c_str(), lastError.c_str());
            return false;
        }
        return MoveCoordLocked(targets);
    }

    // Caller holds mtx and has checked the connection.
    bool MoveCoordLocked(const std::vector<std::pair<std::string, double>>& targets)
    {
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
            lastError = "acsc multi-move failed (" + LastErrorText() + ")";
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
            lastError = "acsc_SetVelocity failed (" + LastErrorText() + ")";
            FD_ERROR("acs", "%s set velocity failed: %s", cfg.name.c_str(), lastError.c_str());
        } else {
            FD_INFO("acs", "%s velocity (all axes) = %.3f mm/s", cfg.name.c_str(), v);
        }
        profileReadReq.store(true);   // the cached VEL is stale now, either way
        return ok;
    }

    // Axis 0 speaks for the device: SetVelocity writes every axis the same.
    bool ReadVelocity(double* v)
    {
        std::lock_guard<std::mutex> lock(mtx);
        if (!connected.load()) { lastError = "not connected"; return false; }
        if (!acsc_GetVelocity(handle, 0, v, NULL)) {
            lastError = "acsc_GetVelocity failed (" + LastErrorText() + ")";
            FD_ERROR("acs", "%s read velocity failed: %s", cfg.name.c_str(), lastError.c_str());
            return false;
        }
        return true;
    }

    // True when no configured axis is still executing a motion. Asked of the
    // controller each time, not the poll cache: a position-tolerance arrival
    // lands while the profile's tail is still running, and the controller
    // refuses a coordinated move on an axis in motion (error 3065) - so the
    // settle wait must see the motion end, not a second-old snapshot of it.
    bool IsSettled() const
    {
        std::lock_guard<std::mutex> lock(mtx);
        if (!connected.load()) return true;    // nothing can be moving
        for (int i = 0; i < static_cast<int>(axes.size()); ++i) {
            int state = 0;
            if (!acsc_GetMotorState(handle, i, &state, NULL))
                return true;                   // can't tell - don't wedge the caller
            if (state & ACSC_MST_MOVE) return false;
        }
        return true;
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
                        LastErrorText() + ")";
            FD_ERROR("acs", "%s axis %s: %s", cfg.name.c_str(), axis.c_str(), lastError.c_str());
            return false;
        }
        FD_INFO("acs", "%s axis %s %s", cfg.name.c_str(), axis.c_str(), on ? "enabled" : "disabled");
        return true;
    }

    // `START <buffer>, <label>`; an empty label (NULL) runs from the first
    // line. The controller refuses a buffer that is not compiled or a label it
    // does not have; its error code lands in lastError.
    bool RunBuffer(int buffer, const std::string& label)
    {
        std::lock_guard<std::mutex> lock(mtx);
        if (!connected.load()) { lastError = "not connected"; return false; }
        std::string lbl = label;   // the SDK takes a non-const char*
        if (!acsc_RunBuffer(handle, buffer, lbl.empty() ? NULL : lbl.data(), NULL)) {
            lastError = "acsc_RunBuffer(" + std::to_string(buffer) +
                        (label.empty() ? "" : ", " + label) + ") failed (" +
                        LastErrorText() + ")";
            FD_ERROR("acs", "%s: %s", cfg.name.c_str(), lastError.c_str());
            return false;
        }
        bufRunning[buffer] = true;   // accepted; the poll takes it from here
        FD_INFO("acs", "%s buffer %d started%s%s", cfg.name.c_str(), buffer,
                label.empty() ? "" : " at ", label.c_str());
        return true;
    }

    bool BufferRunning(int buffer) const
    {
        std::lock_guard<std::mutex> lock(mtx);
        const auto it = bufRunning.find(buffer);
        return it != bufRunning.end() && it->second;
    }

    bool StopBuffer(int buffer)
    {
        std::lock_guard<std::mutex> lock(mtx);
        if (!connected.load()) { lastError = "not connected"; return false; }
        if (!acsc_StopBuffer(handle, buffer, NULL)) {
            lastError = "acsc_StopBuffer(" + std::to_string(buffer) + ") failed (" +
                        LastErrorText() + ")";
            FD_ERROR("acs", "%s: %s", cfg.name.c_str(), lastError.c_str());
            return false;
        }
        bufRunning[buffer] = false;
        FD_INFO("acs", "%s buffer %d stopped", cfg.name.c_str(), buffer);
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
        bufUntil.clear();
        SimRestoreLocked();
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
        SimRestoreLocked();   // an ordinary move never runs dynamic
        targets[axis] = position;
        return true;
    }
    bool MoveAbsolute(const std::vector<std::pair<std::string, double>>& t)
    {
        std::lock_guard<std::mutex> lock(mtx);
        if (!connected.load()) { lastError = "not connected"; return false; }
        for (const auto& kv : t)
            if (!Enabled(kv.first)) return false;
        SimRestoreLocked();   // an ordinary move never runs dynamic
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
        if (!connected.load()) return false;
        simVelocity.store(v);
        return true;
    }
    bool ReadVelocity(double* v)
    {
        if (!connected.load()) { lastError = "not connected"; return false; }
        *v = simVelocity.load();
        return true;
    }
    std::atomic<double> simVelocity{40.0};   // what the "controller" is set to
    // The simulated profile: VEL follows SetVelocity, ACC/DEC/JERK are the
    // fixed defaults below until a dynamic move derives them from VEL, and
    // every ordinary move puts them back - the same contract as the real one.
    static constexpr double kSimAcc = 500.0, kSimJerk = 1000.0, kSimKdec = 5000.0;
    double   simAcc = kSimAcc, simJerk = kSimJerk;   // under mtx
    bool     simDynamic = false;                     // under mtx
    unsigned profileSeq = 1;                         // under mtx

    MotionProfile Profile() const
    {
        std::lock_guard<std::mutex> lock(mtx);
        MotionProfile pr;
        if (!connected.load()) return pr;
        pr.valid   = true;
        pr.seq     = profileSeq;
        pr.dynamic = simDynamic;
        for (const auto& a : axes) {
            pr.axes.push_back({a, simVelocity.load(), simAcc, simAcc, simJerk, kSimKdec, true});
            pr.defaults.push_back({a, simVelocity.load(), kSimAcc, kSimAcc, kSimJerk, kSimKdec, true});
        }
        return pr;
    }
    void RequestProfileRead()
    {
        std::lock_guard<std::mutex> lock(mtx);
        ++profileSeq;
    }
    // Caller holds mtx.
    void SimRestoreLocked()
    {
        if (!simDynamic) return;
        simAcc = kSimAcc;
        simJerk = kSimJerk;
        simDynamic = false;
        ++profileSeq;
        FD_INFO("acs", "%s default profile restored (simulated)", cfg.name.c_str());
    }
    bool RestoreDefaultProfile()
    {
        std::lock_guard<std::mutex> lock(mtx);
        SimRestoreLocked();
        return true;
    }
    bool MoveAbsoluteDynamic(const std::vector<std::pair<std::string, double>>& t,
                             const DynamicProfile& dyn)
    {
        std::lock_guard<std::mutex> lock(mtx);
        if (!connected.load()) { lastError = "not connected"; return false; }
        if (!(dyn.accPerVel > 0.0) || !(dyn.jerkPerAcc > 0.0)) {
            lastError = "dynamic profile ratios must be > 0";
            return false;
        }
        for (const auto& kv : t)
            if (!Enabled(kv.first)) return false;
        simAcc     = simVelocity.load() * dyn.accPerVel;
        simJerk    = simAcc * dyn.jerkPerAcc;
        simDynamic = true;
        ++profileSeq;
        FD_INFO("acs", "%s dynamic profile: VEL %g ACC %g DEC %g JERK %g (simulated)",
                cfg.name.c_str(), simVelocity.load(), simAcc, simAcc, simJerk);
        for (const auto& kv : t) targets[kv.first] = kv.second;
        return true;
    }
    bool IsSettled() const { return true; }   // sim snaps to target - never "moving"
    bool StopAll() { return connected.load(); }
    // A simulated program runs for kSimBufferRun, so Stop / the follow-up move
    // can be exercised without a controller.
    static constexpr std::chrono::seconds kSimBufferRun{3};
    std::map<int, std::chrono::steady_clock::time_point> bufUntil;   // under mtx

    bool RunBuffer(int buffer, const std::string& label)
    {
        std::lock_guard<std::mutex> lock(mtx);
        if (!connected.load()) { lastError = "not connected"; return false; }
        bufUntil[buffer] = std::chrono::steady_clock::now() + kSimBufferRun;
        FD_INFO("acs", "%s buffer %d started%s%s (simulated)", cfg.name.c_str(), buffer,
                label.empty() ? "" : " at ", label.c_str());
        return true;
    }
    bool StopBuffer(int buffer)
    {
        std::lock_guard<std::mutex> lock(mtx);
        if (!connected.load()) { lastError = "not connected"; return false; }
        bufUntil.erase(buffer);
        FD_INFO("acs", "%s buffer %d stopped (simulated)", cfg.name.c_str(), buffer);
        return true;
    }
    bool BufferRunning(int buffer) const
    {
        std::lock_guard<std::mutex> lock(mtx);
        const auto it = bufUntil.find(buffer);
        return it != bufUntil.end() && std::chrono::steady_clock::now() < it->second;
    }
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
bool ACSController::ReadVelocity(double* v) { return p_->ReadVelocity(v); }
bool ACSController::IsSettled() const    { return p_->IsSettled(); }
bool ACSController::StopAll()            { return p_->StopAll(); }
std::map<std::string, bool> ACSController::AxisEnabled() const { return p_->AxisEnabled(); }
bool ACSController::SetAxisEnabled(const std::string& a, bool on) { return p_->SetAxisEnabled(a, on); }
bool ACSController::RunBuffer(int b, const std::string& l) { return p_->RunBuffer(b, l); }
bool ACSController::StopBuffer(int b)    { return p_->StopBuffer(b); }
bool ACSController::BufferRunning(int b) const { return p_->BufferRunning(b); }
MotionProfile ACSController::Profile() const { return p_->Profile(); }
void ACSController::RequestProfileRead() { p_->RequestProfileRead(); }
bool ACSController::MoveAbsoluteDynamic(const std::vector<std::pair<std::string, double>>& t,
                                        const DynamicProfile& d) { return p_->MoveAbsoluteDynamic(t, d); }
bool ACSController::RestoreDefaultProfile() { return p_->RestoreDefaultProfile(); }
void ACSController::RequestFastPoll()    { p_->RequestFastPoll(); }
std::string ACSController::LastError() const { return p_->lastError; }

} // namespace firedog
