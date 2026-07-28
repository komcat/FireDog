// EziIoDevice.cpp
//
// Direct FASTECH EziMOTIONPlusE usage, ported from bobcat_engine's
// EziIODevice (the battle-tested source), condensed to one class with a
// std::thread poll loop publishing bitmask changes to the DataBus.
//
// Vendor bit layout (the porting trap): FAS_GetOutput/FAS_SetOutput pack the
// outputs ABOVE the inputs — output pin p lives at bit 8+p on 8-output boards
// and bit 16+p on 16-output boards. This file shifts exactly once at the FAS
// boundary; everywhere else (cache, interface, DataBus) bit i == pin i.
// Inputs and the input latch are not shifted.
#include "hardware/io/EziIoDevice.h"

#include "core/DataBus.h"
#include "core/Log.h"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <utility>

#ifdef HAS_EZIIO
// ------------------------------- Real FASTECH driver -------------------------------
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <FAS_EziMOTIONPlusE.h>

namespace firedog {
namespace {
// Parse "a.b.c.d" into 4 bytes; false if malformed.
bool ParseIp(const std::string& ip, unsigned char out[4])
{
    std::istringstream iss(ip);
    std::string tok;
    int i = 0;
    while (std::getline(iss, tok, '.') && i < 4) {
        try {
            const int v = std::stoi(tok);
            if (v < 0 || v > 255) return false;
            out[i++] = static_cast<unsigned char>(v);
        } catch (...) {
            return false;
        }
    }
    return i == 4;
}
std::once_flag g_autoReconnectOnce;   // FAS_SetAutoReconnect is process-global
} // namespace

struct EziIoDevice::Impl {
    Config             cfg;
    std::string        type = "EZIIO";
    std::atomic<bool>  connected{false};
    std::atomic<bool>  polling{false};
    std::thread        pollThread;
    std::mutex         ioMu;       // guards all FAS_* calls
    mutable std::mutex cacheMu;    // guards cache (read by the UI each frame)
    IoStatus           cache;
    IoPinNames         inNames, outNames;
    std::string        lastError;
    int                intervalMs = 100;
    int                outShift   = 8;
    uint64_t           outMask    = 0xFF;   // pin-indexed output width mask
    DataStream*        sIn  = nullptr;
    DataStream*        sOut = nullptr;
    double             pubIn = -1.0, pubOut = -1.0;   // last published (publish on change)
    bool               readFailing = false;           // log once per failure streak

    explicit Impl(Config c) : cfg(std::move(c))
    {
        inNames.Load(cfg.inputNames);
        outNames.Load(cfg.outputNames);
        outShift = (cfg.outputCount <= 8) ? 8 : 16;
        outMask  = (cfg.outputCount >= 64) ? ~0ull : ((1ull << cfg.outputCount) - 1);
    }
    ~Impl() { StopPolling(); Disconnect(); }

    bool Connect()
    {
        if (connected.load()) return true;
        unsigned char b[4] = {};
        if (!ParseIp(cfg.ipAddress, b)) {
            lastError = "bad ip '" + cfg.ipAddress + "'";
            FD_ERROR("eziio", "%s: %s", cfg.name.c_str(), lastError.c_str());
            return false;
        }
        std::call_once(g_autoReconnectOnce, [] { PE::FAS_SetAutoReconnect(TRUE); });

        std::lock_guard<std::mutex> lock(ioMu);
        if (!PE::FAS_ConnectTCP(b[0], b[1], b[2], b[3], cfg.deviceId)) {
            lastError = "FAS_ConnectTCP failed";
            FD_ERROR("eziio", "%s @ %s (board %d): %s", cfg.name.c_str(),
                     cfg.ipAddress.c_str(), cfg.deviceId, lastError.c_str());
            return false;
        }
        connected.store(true);
        FD_INFO("eziio", "%s @ %s (board %d) connected", cfg.name.c_str(),
                cfg.ipAddress.c_str(), cfg.deviceId);
        return true;
    }

    void Disconnect()
    {
        StopPolling();
        if (!connected.exchange(false)) return;
        std::lock_guard<std::mutex> lock(ioMu);
        PE::FAS_Close(cfg.deviceId);
    }

    bool SetOutputs(uint64_t setMask, uint64_t clearMask)
    {
        if (!connected.load()) { lastError = "not connected"; return false; }
        setMask   &= outMask;
        clearMask &= outMask;
        int rc;
        {
            std::lock_guard<std::mutex> lock(ioMu);
            rc = PE::FAS_SetOutput(cfg.deviceId,
                                   static_cast<unsigned long>(setMask << outShift),
                                   static_cast<unsigned long>(clearMask << outShift));
        }
        if (rc != FMM_OK) {
            lastError = "FAS_SetOutput rc=" + std::to_string(rc);
            FD_WARN("eziio", "%s: %s", cfg.name.c_str(), lastError.c_str());
            return false;
        }
        // Optimistic: patch the cache now, the next poll (<=intervalMs) confirms.
        std::lock_guard<std::mutex> lock(cacheMu);
        cache.outputs = (cache.outputs | setMask) & ~clearMask;
        return true;
    }

    bool SetOutput(int pin, bool on)
    {
        if (pin < 0 || pin >= cfg.outputCount) { lastError = "bad output pin"; return false; }
        const uint64_t m = 1ull << pin;
        return on ? SetOutputs(m, 0) : SetOutputs(0, m);
    }

    bool ClearLatch(uint64_t mask)
    {
        if (!connected.load()) { lastError = "not connected"; return false; }
        int rc;
        {
            std::lock_guard<std::mutex> lock(ioMu);
            rc = PE::FAS_ClearLatch(cfg.deviceId, static_cast<unsigned long>(mask));
        }
        if (rc != FMM_OK) {
            lastError = "FAS_ClearLatch rc=" + std::to_string(rc);
            return false;
        }
        std::lock_guard<std::mutex> lock(cacheMu);
        cache.latch &= ~mask;
        return true;
    }

    bool StartPolling(int ms, const std::string& tIn, const std::string& tOut)
    {
        if (!connected.load()) { lastError = "not connected"; return false; }
        if (polling.load()) return true;
        intervalMs = ms > 0 ? ms : 100;
        sIn  = tIn.empty()  ? nullptr : &DataBus::Instance().Stream(tIn);
        sOut = tOut.empty() ? nullptr : &DataBus::Instance().Stream(tOut);
        polling.store(true);
        pollThread = std::thread(&Impl::PollLoop, this);
        FD_INFO("eziio", "%s polling every %d ms", cfg.name.c_str(), intervalMs);
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
            unsigned long rawIn = 0, rawLatch = 0, rawOut = 0, outSt = 0;
            bool ok;
            {
                std::lock_guard<std::mutex> lock(ioMu);
                ok = PE::FAS_GetInput(cfg.deviceId, &rawIn, &rawLatch) == FMM_OK
                  && PE::FAS_GetOutput(cfg.deviceId, &rawOut, &outSt) == FMM_OK;
            }
            // Transient failures are expected (SDK auto-reconnect); keep the
            // loop alive and only log the streak edges.
            if (!ok && !readFailing) {
                readFailing = true;
                FD_WARN("eziio", "%s poll read failed (auto-reconnect pending)", cfg.name.c_str());
            } else if (ok && readFailing) {
                readFailing = false;
                FD_INFO("eziio", "%s poll read recovered", cfg.name.c_str());
            }
            if (ok) {
                double vIn, vOut;
                {
                    std::lock_guard<std::mutex> lock(cacheMu);
                    cache.inputs  = rawIn;
                    cache.latch   = rawLatch;
                    cache.outputs = (static_cast<uint64_t>(rawOut) >> outShift) & outMask;
                    cache.valid   = true;
                    vIn  = static_cast<double>(cache.inputs);
                    vOut = static_cast<double>(cache.outputs);
                }
                const double t = std::chrono::duration<double>(
                    std::chrono::steady_clock::now() - start).count();
                if (sIn  && vIn  != pubIn)  { sIn->Publish(t, vIn);   pubIn  = vIn;  }
                if (sOut && vOut != pubOut) { sOut->Publish(t, vOut); pubOut = vOut; }
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
#include "hardware/io/IoSim.h"

namespace firedog {

struct EziIoDevice::Impl {
    Config             cfg;
    std::string        type = "EZIIO";
    std::atomic<bool>  connected{false};
    std::atomic<bool>  polling{false};
    std::thread        pollThread;
    mutable std::mutex cacheMu;
    IoStatus           cache;
    IoPinNames         inNames, outNames;
    std::string        lastError;
    int                intervalMs = 100;
    uint64_t           outMask    = 0xFF;
    uint64_t           prevIn     = 0;
    DataStream*        sIn  = nullptr;
    DataStream*        sOut = nullptr;
    double             pubIn = -1.0, pubOut = -1.0;

    explicit Impl(Config c) : cfg(std::move(c))
    {
        inNames.Load(cfg.inputNames);
        outNames.Load(cfg.outputNames);
        outMask = (cfg.outputCount >= 64) ? ~0ull : ((1ull << cfg.outputCount) - 1);
    }
    ~Impl() { StopPolling(); }

    bool Connect()
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        connected.store(true);
        FD_INFO("eziio", "%s connected (simulated)", cfg.name.c_str());
        return true;
    }
    void Disconnect() { StopPolling(); connected.store(false); }

    bool SetOutputs(uint64_t setMask, uint64_t clearMask)
    {
        if (!connected.load()) { lastError = "not connected"; return false; }
        std::lock_guard<std::mutex> lock(cacheMu);
        cache.outputs = (cache.outputs | (setMask & outMask)) & ~(clearMask & outMask);
        return true;
    }
    bool SetOutput(int pin, bool on)
    {
        if (pin < 0 || pin >= cfg.outputCount) { lastError = "bad output pin"; return false; }
        const uint64_t m = 1ull << pin;
        return on ? SetOutputs(m, 0) : SetOutputs(0, m);
    }
    bool ClearLatch(uint64_t mask)
    {
        std::lock_guard<std::mutex> lock(cacheMu);
        cache.latch &= ~mask;
        return true;
    }

    bool StartPolling(int ms, const std::string& tIn, const std::string& tOut)
    {
        if (!connected.load()) { lastError = "not connected"; return false; }
        if (polling.load()) return true;
        intervalMs = ms > 0 ? ms : 100;
        sIn  = tIn.empty()  ? nullptr : &DataBus::Instance().Stream(tIn);
        sOut = tOut.empty() ? nullptr : &DataBus::Instance().Stream(tOut);
        polling.store(true);
        pollThread = std::thread(&Impl::PollLoop, this);
        FD_INFO("eziio", "%s polling every %d ms (simulated)", cfg.name.c_str(), intervalMs);
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
            const double t = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - start).count();
            const uint64_t in     = SimIoInputs(t, cfg.inputCount);
            const uint64_t rising = (in ^ prevIn) & in;
            prevIn = in;
            double vIn, vOut;
            {
                std::lock_guard<std::mutex> lock(cacheMu);
                cache.inputs = in;
                cache.latch |= rising;
                cache.valid  = true;
                vIn  = static_cast<double>(cache.inputs);
                vOut = static_cast<double>(cache.outputs);
            }
            if (sIn  && vIn  != pubIn)  { sIn->Publish(t, vIn);   pubIn  = vIn;  }
            if (sOut && vOut != pubOut) { sOut->Publish(t, vOut); pubOut = vOut; }
            std::this_thread::sleep_for(std::chrono::milliseconds(intervalMs));
        }
    }
};

} // namespace firedog

#endif // HAS_EZIIO

// ---- Forwarding shell (identical for both Impls) ----
namespace firedog {

EziIoDevice::EziIoDevice(Config cfg) : p_(std::make_unique<Impl>(std::move(cfg))) {}
EziIoDevice::~EziIoDevice() = default;

bool EziIoDevice::Connect()           { return p_->Connect(); }
void EziIoDevice::Disconnect()        { p_->Disconnect(); }
bool EziIoDevice::IsConnected() const { return p_->connected.load(); }

const std::string& EziIoDevice::Name() const { return p_->cfg.name; }
const std::string& EziIoDevice::Type() const { return p_->type; }
int EziIoDevice::InputCount() const   { return p_->cfg.inputCount; }
int EziIoDevice::OutputCount() const  { return p_->cfg.outputCount; }

IIoDevice::IoStatus EziIoDevice::Status() const
{
    std::lock_guard<std::mutex> lock(p_->cacheMu);
    return p_->cache;
}

bool EziIoDevice::SetOutput(int pin, bool on) { return p_->SetOutput(pin, on); }
bool EziIoDevice::SetOutputs(uint64_t setMask, uint64_t clearMask)
{
    return p_->SetOutputs(setMask, clearMask);
}
bool EziIoDevice::ClearLatch(uint64_t mask) { return p_->ClearLatch(mask); }

bool EziIoDevice::StartPolling(int intervalMs, const std::string& topicIn,
                               const std::string& topicOut)
{
    return p_->StartPolling(intervalMs, topicIn, topicOut);
}
void EziIoDevice::StopPolling() { p_->StopPolling(); }

std::string EziIoDevice::InputPinName(int pin) const  { return p_->inNames.NameOf(pin); }
std::string EziIoDevice::OutputPinName(int pin) const { return p_->outNames.NameOf(pin); }
int EziIoDevice::InputPinByName(const std::string& name) const
{
    return p_->inNames.PinOf(name);
}
int EziIoDevice::OutputPinByName(const std::string& name) const
{
    return p_->outNames.PinOf(name);
}

std::string EziIoDevice::LastError() const { return p_->lastError; }

} // namespace firedog
