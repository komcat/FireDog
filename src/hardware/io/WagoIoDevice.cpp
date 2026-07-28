// WagoIoDevice.cpp
//
// Hand-rolled Modbus TCP client (ported from bobcat_engine's WagoIODevice):
// MBAP frames over one winsock socket, each request/response serialized under
// a socket mutex with transaction-id verification and recv-until-complete.
//
// WAGO 750 process-image addressing is asymmetric: digital inputs are read at
// address 0 (FC 0x02), outputs are WRITTEN at address 0 (FC 0x05/0x0F) but
// READ BACK at address 0x0200 (FC 0x01). Bits are pin-indexed end to end —
// bobcat's EZI-IO-compatible remap (WagoToUiBits) is intentionally dropped.
//
// The input "latch" is synthesized in software: rising edges accumulate per
// poll cycle, so pulses shorter than the poll interval can be missed.
#include "hardware/io/WagoIoDevice.h"

#include "core/DataBus.h"
#include "core/Log.h"
#include "hardware/io/IoSim.h"

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <winsock2.h>
#include <ws2tcpip.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace firedog {
namespace {

constexpr uint16_t kOutputWriteAddr = 0x0000;   // FC 0x05/0x0F target
constexpr uint16_t kOutputReadAddr  = 0x0200;   // output read-back (PII area)
constexpr uint16_t kInputReadAddr   = 0x0000;
constexpr int      kTimeoutMs       = 3000;

constexpr uint8_t kFcReadCoils      = 0x01;
constexpr uint8_t kFcReadDiscrete   = 0x02;
constexpr uint8_t kFcWriteSingle    = 0x05;
constexpr uint8_t kFcWriteMultiple  = 0x0F;

std::once_flag g_wsaOnce;   // WSAStartup once per process; never per-device cleanup

void MbapHeader(std::vector<uint8_t>& f, uint16_t transId, uint8_t pduLen)
{
    f.push_back(static_cast<uint8_t>(transId >> 8));
    f.push_back(static_cast<uint8_t>(transId & 0xFF));
    f.push_back(0x00);                    // protocol id
    f.push_back(0x00);
    f.push_back(0x00);                    // length = unit id + PDU
    f.push_back(static_cast<uint8_t>(1 + pduLen));
    f.push_back(0x01);                    // unit id
}

std::vector<uint8_t> BuildReadReq(uint16_t transId, uint8_t fc, uint16_t addr, uint16_t count)
{
    std::vector<uint8_t> f;
    MbapHeader(f, transId, 5);
    f.push_back(fc);
    f.push_back(static_cast<uint8_t>(addr >> 8));
    f.push_back(static_cast<uint8_t>(addr & 0xFF));
    f.push_back(static_cast<uint8_t>(count >> 8));
    f.push_back(static_cast<uint8_t>(count & 0xFF));
    return f;
}

std::vector<uint8_t> BuildWriteSingleCoilReq(uint16_t transId, uint16_t addr, bool on)
{
    std::vector<uint8_t> f;
    MbapHeader(f, transId, 5);
    f.push_back(kFcWriteSingle);
    f.push_back(static_cast<uint8_t>(addr >> 8));
    f.push_back(static_cast<uint8_t>(addr & 0xFF));
    f.push_back(on ? 0xFF : 0x00);        // 0xFF00 = ON, 0x0000 = OFF
    f.push_back(0x00);
    return f;
}

std::vector<uint8_t> BuildWriteMultipleCoilsReq(uint16_t transId, uint16_t addr,
                                                int count, uint64_t bits)
{
    const uint8_t nBytes = static_cast<uint8_t>((count + 7) / 8);
    std::vector<uint8_t> f;
    MbapHeader(f, transId, static_cast<uint8_t>(6 + nBytes));
    f.push_back(kFcWriteMultiple);
    f.push_back(static_cast<uint8_t>(addr >> 8));
    f.push_back(static_cast<uint8_t>(addr & 0xFF));
    f.push_back(static_cast<uint8_t>(count >> 8));
    f.push_back(static_cast<uint8_t>(count & 0xFF));
    f.push_back(nBytes);
    for (int i = 0; i < nBytes; ++i) {
        f.push_back(static_cast<uint8_t>((bits >> (i * 8)) & 0xFF));
    }
    return f;
}

} // namespace

struct WagoIoDevice::Impl {
    Config             cfg;
    std::string        type = "WAGO";
    std::atomic<bool>  connected{false};
    std::atomic<bool>  polling{false};
    std::thread        pollThread;
    std::mutex         sockMu;     // serializes socket lifecycle + each transaction
    mutable std::mutex cacheMu;    // guards cache (read by the UI each frame)
    IoStatus           cache;
    IoPinNames         inNames, outNames;
    std::string        lastError;
    int                intervalMs = 100;
    uint64_t           outMask    = 0xFF;   // output width mask
    uint64_t           outState   = 0;      // last known raw outputs (sockMu)
    uint64_t           prevIn     = 0;      // for the software latch (poll thread)
    uint16_t           transId    = 0;      // Modbus transaction id (sockMu)
    SOCKET             sock       = INVALID_SOCKET;
    DataStream*        sIn  = nullptr;
    DataStream*        sOut = nullptr;
    double             pubIn = -1.0, pubOut = -1.0;   // publish on change
    bool               readFailing = false;           // log once per failure streak

    explicit Impl(Config c) : cfg(std::move(c))
    {
        inNames.Load(cfg.inputNames);
        outNames.Load(cfg.outputNames);
        outMask = (cfg.outputCount >= 64) ? ~0ull : ((1ull << cfg.outputCount) - 1);
    }
    ~Impl() { StopPolling(); Disconnect(); }

    // ---- socket layer (sockMu held unless noted) ----------------------------

    void DropSocket()
    {
        if (sock != INVALID_SOCKET) {
            closesocket(sock);
            sock = INVALID_SOCKET;
        }
    }

    bool OpenSocket()
    {
        DropSocket();
        sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        if (sock == INVALID_SOCKET) { lastError = "socket() failed"; return false; }

        const DWORD timeout = kTimeoutMs;
        setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO,
                   reinterpret_cast<const char*>(&timeout), sizeof(timeout));
        setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO,
                   reinterpret_cast<const char*>(&timeout), sizeof(timeout));

        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port   = htons(static_cast<u_short>(cfg.port));
        if (inet_pton(AF_INET, cfg.ipAddress.c_str(), &addr.sin_addr) <= 0) {
            lastError = "bad ip '" + cfg.ipAddress + "'";
            DropSocket();
            return false;
        }
        if (connect(sock, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) == SOCKET_ERROR) {
            lastError = "connect to " + cfg.ipAddress + ":" + std::to_string(cfg.port) + " failed";
            DropSocket();
            return false;
        }
        return true;
    }

    bool SendAll(const std::vector<uint8_t>& buf)
    {
        size_t off = 0;
        while (off < buf.size()) {
            const int n = send(sock, reinterpret_cast<const char*>(buf.data() + off),
                               static_cast<int>(buf.size() - off), 0);
            if (n <= 0) return false;
            off += static_cast<size_t>(n);
        }
        return true;
    }

    bool RecvAll(uint8_t* buf, size_t len)
    {
        size_t off = 0;
        while (off < len) {   // TCP may fragment — read until complete
            const int n = recv(sock, reinterpret_cast<char*>(buf + off),
                               static_cast<int>(len - off), 0);
            if (n <= 0) return false;
            off += static_cast<size_t>(n);
        }
        return true;
    }

    // One Modbus request/response. Returns the PDU (function code onward).
    // The MBAP length field sizes the read, so exception responses (which are
    // shorter than the expected reply) never stall the recv.
    bool Transact(const std::vector<uint8_t>& req, std::vector<uint8_t>& pdu)
    {
        if (sock == INVALID_SOCKET) { lastError = "no socket"; return false; }
        if (!SendAll(req)) { lastError = "send failed"; DropSocket(); return false; }

        uint8_t hdr[7];
        if (!RecvAll(hdr, sizeof(hdr))) { lastError = "recv header failed"; DropSocket(); return false; }
        if (hdr[0] != req[0] || hdr[1] != req[1]) {
            lastError = "transaction id mismatch";
            DropSocket();   // stream is desynced — start over
            return false;
        }
        const int len = (hdr[4] << 8) | hdr[5];   // unit id + PDU
        if (len < 2 || len > 260) { lastError = "bad frame length"; DropSocket(); return false; }

        uint8_t unitId = 0;
        pdu.resize(static_cast<size_t>(len - 1));
        if (!RecvAll(&unitId, 1) || !RecvAll(pdu.data(), pdu.size())) {
            lastError = "recv body failed";
            DropSocket();
            return false;
        }
        if (pdu[0] & 0x80) {
            lastError = "modbus exception " + std::to_string(pdu.size() > 1 ? pdu[1] : 0)
                      + " (fc " + std::to_string(pdu[0] & 0x7F) + ")";
            return false;
        }
        return true;
    }

    bool ReadBits(uint8_t fc, uint16_t addr, int count, uint64_t& bits)
    {
        std::vector<uint8_t> pdu;
        if (!Transact(BuildReadReq(++transId, fc, addr, static_cast<uint16_t>(count)), pdu)) {
            return false;
        }
        if (pdu.size() < 2 || pdu[0] != fc) { lastError = "bad read reply"; return false; }
        const int nBytes = pdu[1];
        bits = 0;
        for (int i = 0; i < nBytes && i < 8 && 2 + i < static_cast<int>(pdu.size()); ++i) {
            bits |= static_cast<uint64_t>(pdu[2 + i]) << (i * 8);
        }
        if (count < 64) bits &= (1ull << count) - 1;
        return true;
    }

    // ---- device API ---------------------------------------------------------

    bool Connect()
    {
        if (connected.load()) return true;
        if (cfg.simulated) {
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            connected.store(true);
            FD_INFO("wago", "%s connected (simulated)", cfg.name.c_str());
            return true;
        }

        std::call_once(g_wsaOnce, [] {
            WSADATA d;
            WSAStartup(MAKEWORD(2, 2), &d);
        });

        std::lock_guard<std::mutex> lock(sockMu);
        if (!OpenSocket()) {
            FD_ERROR("wago", "%s: %s", cfg.name.c_str(), lastError.c_str());
            return false;
        }
        // Seed the output state from the device so read-modify-write starts true.
        uint64_t out = 0;
        if (ReadBits(kFcReadCoils, kOutputReadAddr, cfg.outputCount, out)) {
            outState = out;
            std::lock_guard<std::mutex> cl(cacheMu);
            cache.outputs = out;
        }
        connected.store(true);
        FD_INFO("wago", "%s @ %s:%d connected", cfg.name.c_str(),
                cfg.ipAddress.c_str(), cfg.port);
        return true;
    }

    void Disconnect()
    {
        StopPolling();
        if (!connected.exchange(false)) return;
        std::lock_guard<std::mutex> lock(sockMu);
        DropSocket();
    }

    bool SetOutputs(uint64_t setMask, uint64_t clearMask)
    {
        if (!connected.load()) { lastError = "not connected"; return false; }
        setMask   &= outMask;
        clearMask &= outMask;
        if (cfg.simulated) {
            std::lock_guard<std::mutex> lock(cacheMu);
            cache.outputs = (cache.outputs | setMask) & ~clearMask;
            return true;
        }

        uint64_t newState;
        {
            std::lock_guard<std::mutex> lock(sockMu);
            newState = (outState | setMask) & ~clearMask;   // RMW on the cached word
            std::vector<uint8_t> pdu;
            if (!Transact(BuildWriteMultipleCoilsReq(++transId, kOutputWriteAddr,
                                                     cfg.outputCount, newState), pdu)) {
                FD_WARN("wago", "%s write outputs failed: %s", cfg.name.c_str(), lastError.c_str());
                return false;
            }
            outState = newState;
        }
        std::lock_guard<std::mutex> lock(cacheMu);
        cache.outputs = newState;
        return true;
    }

    bool SetOutput(int pin, bool on)
    {
        if (pin < 0 || pin >= cfg.outputCount) { lastError = "bad output pin"; return false; }
        if (!connected.load()) { lastError = "not connected"; return false; }
        const uint64_t m = 1ull << pin;
        if (cfg.simulated) {
            std::lock_guard<std::mutex> lock(cacheMu);
            cache.outputs = on ? (cache.outputs | m) : (cache.outputs & ~m);
            return true;
        }

        uint64_t newState;
        {
            std::lock_guard<std::mutex> lock(sockMu);
            std::vector<uint8_t> pdu;
            if (!Transact(BuildWriteSingleCoilReq(++transId,
                              static_cast<uint16_t>(kOutputWriteAddr + pin), on), pdu)) {
                FD_WARN("wago", "%s write pin %d failed: %s", cfg.name.c_str(), pin, lastError.c_str());
                return false;
            }
            outState = on ? (outState | m) : (outState & ~m);
            newState = outState;
        }
        std::lock_guard<std::mutex> lock(cacheMu);
        cache.outputs = newState;
        return true;
    }

    bool ClearLatch(uint64_t mask)
    {
        // Software latch — clearing is purely a cache operation.
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
        FD_INFO("wago", "%s polling every %d ms%s", cfg.name.c_str(), intervalMs,
                cfg.simulated ? " (simulated)" : "");
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
            const double t = std::chrono::duration<double>(t0 - start).count();

            uint64_t in = 0, out = 0;
            bool ok;
            if (cfg.simulated) {
                in = SimIoInputs(t, cfg.inputCount);
                {
                    std::lock_guard<std::mutex> lock(cacheMu);
                    out = cache.outputs;
                }
                ok = true;
            } else {
                std::lock_guard<std::mutex> lock(sockMu);
                ok = (sock != INVALID_SOCKET || OpenSocket())   // reconnect a dropped link
                  && ReadBits(kFcReadDiscrete, kInputReadAddr, cfg.inputCount, in)
                  && ReadBits(kFcReadCoils, kOutputReadAddr, cfg.outputCount, out);
                if (ok) outState = out;
            }

            if (!ok && !readFailing) {
                readFailing = true;
                FD_WARN("wago", "%s poll failed: %s", cfg.name.c_str(), lastError.c_str());
            } else if (ok && readFailing) {
                readFailing = false;
                FD_INFO("wago", "%s poll recovered", cfg.name.c_str());
            }

            if (ok) {
                const uint64_t rising = (in ^ prevIn) & in;
                prevIn = in;
                double vIn, vOut;
                {
                    std::lock_guard<std::mutex> lock(cacheMu);
                    cache.inputs  = in;
                    cache.latch  |= rising;
                    cache.outputs = out;
                    cache.valid   = true;
                    vIn  = static_cast<double>(cache.inputs);
                    vOut = static_cast<double>(cache.outputs);
                }
                if (sIn  && vIn  != pubIn)  { sIn->Publish(t, vIn);   pubIn  = vIn;  }
                if (sOut && vOut != pubOut) { sOut->Publish(t, vOut); pubOut = vOut; }
            }

            const auto elapsed = std::chrono::steady_clock::now() - t0;
            const auto period  = std::chrono::milliseconds(intervalMs);
            if (elapsed < period) std::this_thread::sleep_for(period - elapsed);
        }
    }
};

// ---- Forwarding shell ----
WagoIoDevice::WagoIoDevice(Config cfg) : p_(std::make_unique<Impl>(std::move(cfg))) {}
WagoIoDevice::~WagoIoDevice() = default;

bool WagoIoDevice::Connect()           { return p_->Connect(); }
void WagoIoDevice::Disconnect()        { p_->Disconnect(); }
bool WagoIoDevice::IsConnected() const { return p_->connected.load(); }

const std::string& WagoIoDevice::Name() const { return p_->cfg.name; }
const std::string& WagoIoDevice::Type() const { return p_->type; }
int WagoIoDevice::InputCount() const   { return p_->cfg.inputCount; }
int WagoIoDevice::OutputCount() const  { return p_->cfg.outputCount; }

IIoDevice::IoStatus WagoIoDevice::Status() const
{
    std::lock_guard<std::mutex> lock(p_->cacheMu);
    return p_->cache;
}

bool WagoIoDevice::SetOutput(int pin, bool on) { return p_->SetOutput(pin, on); }
bool WagoIoDevice::SetOutputs(uint64_t setMask, uint64_t clearMask)
{
    return p_->SetOutputs(setMask, clearMask);
}
bool WagoIoDevice::ClearLatch(uint64_t mask) { return p_->ClearLatch(mask); }

bool WagoIoDevice::StartPolling(int intervalMs, const std::string& topicIn,
                                const std::string& topicOut)
{
    return p_->StartPolling(intervalMs, topicIn, topicOut);
}
void WagoIoDevice::StopPolling() { p_->StopPolling(); }

std::string WagoIoDevice::InputPinName(int pin) const  { return p_->inNames.NameOf(pin); }
std::string WagoIoDevice::OutputPinName(int pin) const { return p_->outNames.NameOf(pin); }
int WagoIoDevice::InputPinByName(const std::string& name) const
{
    return p_->inNames.PinOf(name);
}
int WagoIoDevice::OutputPinByName(const std::string& name) const
{
    return p_->outNames.PinOf(name);
}

std::string WagoIoDevice::LastError() const { return p_->lastError; }

} // namespace firedog
