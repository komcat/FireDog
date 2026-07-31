// PIHexapod.cpp
//
// Direct PI GCS2 usage. The connect/init/query/move calls are ported from the
// production PI adapter (komcat/bobcat_engine + BobDog PIController), condensed
// to a single class with a background position-poll thread. Without HAS_PI it
// compiles a simulated stub so the motion status/positions still populate.
#include "hardware/motion/PIHexapod.h"

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
// Split "X Y Z U V W" into {"X","Y","Z","U","V","W"}.
std::vector<std::string> SplitAxes(const std::string& s)
{
    std::vector<std::string> out;
    std::istringstream ss(s);
    std::string tok;
    while (ss >> tok) out.push_back(tok);
    if (out.empty()) out = {"X", "Y", "Z", "U", "V", "W"};
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

#ifdef HAS_PI
// --------------------------------- Real GCS2 driver ---------------------------------
#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <Windows.h>
#endif
#include "PI_GCS2_DLL.h"

namespace firedog {

struct PIHexapod::Impl {
    MotionConfig             cfg;
    std::string              type = "PI";
    std::vector<std::string> axes;
    std::string              axesStr;       // "X Y Z U V W" for PI_qPOS/PI_MOV
    int                      id = -1;       // PI controller id (<0 = invalid)
    std::atomic<bool>        connected{false};
    std::atomic<bool>        polling{false};
    std::thread              pollThread;
    mutable std::mutex       mtx;           // guards positions + SDK I/O
    std::map<std::string, double> positions;
    std::string              lastError;
    std::vector<DataStream*> streams;       // one DataBus topic per axis
    std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
    std::mutex               pollMtx;       // guards fastUntilNs + the poll CV
    std::condition_variable  pollCv;
    std::int64_t             fastUntilNs = 0;   // poll fast until this steady-ns deadline

    explicit Impl(MotionConfig c) : cfg(std::move(c))
    {
        axes    = SplitAxes(cfg.axes);
        axesStr = cfg.axes.empty() ? "X Y Z U V W" : cfg.axes;
        for (const auto& a : axes) positions[a] = 0.0;
    }
    ~Impl() { Disconnect(); }

    bool Connect()
    {
        if (connected.load()) return true;
        id = PI_ConnectTCPIP(cfg.ipAddress.c_str(), cfg.port);
        if (id < 0) {
            const int err = PI_GetInitError();
            lastError = "PI_ConnectTCPIP failed (" + std::to_string(err) + ")";
            FD_ERROR("pi", "%s: %s", cfg.name.c_str(), lastError.c_str());
            return false;
        }
        PI_INI(id, NULL);
        connected.store(true);
        FD_INFO("pi", "%s connected @ %s:%d (id %d)", cfg.name.c_str(),
                cfg.ipAddress.c_str(), cfg.port, id);

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
            pollCv.notify_one();   // wake the poll loop out of wait_for
            if (pollThread.joinable()) pollThread.join();
        }
        if (connected.exchange(false)) {
            std::lock_guard<std::mutex> lock(mtx);
            PI_STP(id);
            PI_CloseConnection(id);
            id = -1;
        }
    }

    void PollLoop()
    {
        std::vector<double> buf(axes.size(), 0.0);
        while (polling.load()) {
            {
                std::lock_guard<std::mutex> lock(mtx);
                if (id >= 0 && PI_qPOS(id, axesStr.c_str(), buf.data())) {
                    for (size_t i = 0; i < axes.size(); ++i) positions[axes[i]] = buf[i];
                }
            }
            // Broadcast each axis to its DataBus topic (outside the lock so a
            // subscriber callback can't deadlock against Positions()).
            const double t = std::chrono::duration<double>(
                                 std::chrono::steady_clock::now() - start).count();
            for (size_t i = 0; i < streams.size(); ++i)
                if (streams[i]) streams[i]->Publish(t, buf[i]);

            // Adaptive cadence: 20 Hz while a consumer is asking, else 1 Hz idle.
            std::unique_lock<std::mutex> lk(pollMtx);
            if (!polling.load()) break;
            const auto period = (NowNs() < fastUntilNs) ? std::chrono::milliseconds(50)
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
        if (wasIdle) pollCv.notify_one();   // wake an idle loop; leave a fast one alone
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
        double vals[1] = {position};
        if (!PI_MOV(id, axis.c_str(), vals)) {
            int err = 0; PI_qERR(id, &err);
            lastError = "PI_MOV failed (" + std::to_string(err) + ")";
            FD_ERROR("pi", "%s move %s failed: %s", cfg.name.c_str(), axis.c_str(), lastError.c_str());
            return false;
        }
        return true;
    }

    bool MoveAbsolute(const std::vector<std::pair<std::string, double>>& targets)
    {
        if (targets.empty()) return true;
        std::lock_guard<std::mutex> lock(mtx);
        if (!connected.load()) { lastError = "not connected"; return false; }
        std::string ax;
        std::vector<double> vals;
        vals.reserve(targets.size());
        for (const auto& kv : targets) {
            if (!ax.empty()) ax += ' ';
            ax += kv.first;
            vals.push_back(kv.second);
        }
        if (!PI_MOV(id, ax.c_str(), vals.data())) {   // one command -> axes move together
            int err = 0; PI_qERR(id, &err);
            lastError = "PI_MOV(multi) failed (" + std::to_string(err) + ")";
            FD_ERROR("pi", "%s multi-move failed: %s", cfg.name.c_str(), lastError.c_str());
            return false;
        }
        return true;
    }

    bool SetVelocity(double v)
    {
        // The C-887 rejects VLS while the platform is still executing a motion
        // profile (GCS error 1005 CONTROLLER_BUSY), and a position-tolerance
        // arrival check can lead the controller's own motion-complete state by
        // a moment. Retry briefly on busy instead of failing the caller.
        constexpr int kBusyErr = 1005, kStepMs = 100, kMaxWaitMs = 2000;
        for (int waited = 0;; waited += kStepMs) {
            {
                std::lock_guard<std::mutex> lock(mtx);
                if (!connected.load()) { lastError = "not connected"; return false; }
                if (PI_VLS(id, v)) {   // system (vectorial) velocity
                    FD_INFO("pi", "%s system velocity = %.3f mm/s", cfg.name.c_str(), v);
                    return true;
                }
                int err = 0; PI_qERR(id, &err);
                lastError = "PI_VLS failed (" + std::to_string(err) + ")";
                if (err != kBusyErr || waited >= kMaxWaitMs) {
                    FD_ERROR("pi", "%s set velocity failed: %s", cfg.name.c_str(),
                             lastError.c_str());
                    return false;
                }
            }   // drop the SDK mutex while waiting — the poll thread needs it
            std::this_thread::sleep_for(std::chrono::milliseconds(kStepMs));
        }
    }

    // True when no configured axis is still executing a motion profile. The
    // commands' arrival check is position-tolerance based, so it can complete
    // while the controller still reports moving — callers wait on this before
    // issuing anything the controller rejects mid-motion (VLS).
    bool IsSettled()
    {
        std::lock_guard<std::mutex> lock(mtx);
        if (!connected.load()) return true;    // nothing can be moving
        BOOL moving[16] = {};
        if (!PI_IsMoving(id, axesStr.c_str(), moving))
            return true;                       // can't tell — don't wedge the caller
        for (std::size_t i = 0; i < axes.size() && i < 16; ++i)
            if (moving[i]) return false;
        return true;
    }

    bool StopAll()
    {
        std::lock_guard<std::mutex> lock(mtx);
        return connected.load() && PI_STP(id);
    }
};

} // namespace firedog

#else
// --------------------------------- Simulated stub ---------------------------------
namespace firedog {

struct PIHexapod::Impl {
    MotionConfig             cfg;
    std::string              type = "PI";
    std::vector<std::string> axes;
    std::atomic<bool>        connected{false};
    std::atomic<bool>        polling{false};
    std::thread              pollThread;
    mutable std::mutex       mtx;
    std::map<std::string, double> targets;     // last commanded position per axis
    std::map<std::string, double> positions;   // current (simulated) position
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
        FD_INFO("pi", "%s connected (simulated) @ %s:%d", cfg.name.c_str(),
                cfg.ipAddress.c_str(), cfg.port);
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
    }

    // Settle toward the commanded target plus a tiny live jitter, store it, and
    // publish each axis — so both Positions() and DataBus subscribers see motion.
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
                    const double v = targets.at(a) + 0.002 * std::sin(t * 0.7 + k * 1.3);
                    positions[a] = v;
                    vals[k] = v;
                    ++k;
                }
            }
            for (size_t i = 0; i < streams.size(); ++i)
                if (streams[i]) streams[i]->Publish(t, vals[i]);

            // Adaptive cadence: ~50 Hz while asked, else 2 Hz idle.
            std::unique_lock<std::mutex> lk(pollMtx);
            if (!polling.load()) break;
            const auto period = (NowNs() < fastUntilNs) ? std::chrono::milliseconds(20)
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
        targets[axis] = position;
        return true;
    }
    bool MoveAbsolute(const std::vector<std::pair<std::string, double>>& t)
    {
        std::lock_guard<std::mutex> lock(mtx);
        if (!connected.load()) { lastError = "not connected"; return false; }
        for (const auto& kv : t) targets[kv.first] = kv.second;
        return true;
    }
    bool SetVelocity(double v)
    {
        FD_INFO("pi", "%s system velocity = %.3f mm/s (simulated)", cfg.name.c_str(), v);
        return connected.load();
    }
    bool IsSettled() { return true; }   // sim snaps to target — never "moving"
    bool StopAll() { return connected.load(); }
};

} // namespace firedog

#endif // HAS_PI

// ---- Forwarding shell (identical for both Impls) ----
namespace firedog {

PIHexapod::PIHexapod(MotionConfig cfg) : p_(std::make_unique<Impl>(std::move(cfg))) {}
PIHexapod::~PIHexapod() = default;

bool PIHexapod::Connect()               { return p_->Connect(); }
void PIHexapod::Disconnect()            { p_->Disconnect(); }
bool PIHexapod::IsConnected() const     { return p_->connected.load(); }
const std::string& PIHexapod::Name() const { return p_->cfg.name; }
const std::string& PIHexapod::Type() const { return p_->type; }
std::vector<std::string> PIHexapod::Axes() const { return p_->axes; }
std::map<std::string, double> PIHexapod::Positions() const { return p_->Positions(); }
bool PIHexapod::MoveAbsolute(const std::string& a, double p) { return p_->MoveAbsolute(a, p); }
bool PIHexapod::MoveAbsolute(const std::vector<std::pair<std::string, double>>& t) { return p_->MoveAbsolute(t); }
bool PIHexapod::SetVelocity(double v)   { return p_->SetVelocity(v); }
bool PIHexapod::IsSettled() const       { return p_->IsSettled(); }
bool PIHexapod::StopAll()               { return p_->StopAll(); }
void PIHexapod::RequestFastPoll()       { p_->RequestFastPoll(); }
std::string PIHexapod::LastError() const { return p_->lastError; }

} // namespace firedog
