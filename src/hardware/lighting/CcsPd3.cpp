// CcsPd3.cpp
//
// Hand-rolled TCP client for the CCS PD3 (protocol ported from Horizon's
// CcsPd3Adapter). One winsock socket, every transaction serialised under ioMu;
// the UI-facing status cache sits behind its own statusMu so a per-frame read
// never waits behind an in-flight command's 2 s I/O timeout.
//
// The simulated build talks the SAME protocol to an in-memory controller
// (SimReply): frames, checksums and reply parsing are exercised either way, and
// only the bytes' destination differs.
#include "hardware/lighting/CcsPd3.h"

#include "core/DataBus.h"
#include "core/Log.h"
#include "core/Topics.h"

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <winsock2.h>
#include <ws2tcpip.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cctype>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace firedog {
namespace {

constexpr int kIoTimeoutMs      = 2000;   // the PD3 drops a frame not completed within 4 s
constexpr int kConnectTimeoutMs = 3000;
constexpr int kDrainTimeoutMs   = 500;
constexpr int kMaxReplyLength   = 64;
constexpr int kMaxDrainReads    = 16;

std::once_flag g_wsaOnce;   // WSAStartup once per process; never per-device cleanup

// Low byte of the ASCII sum, as 2 uppercase hex digits.
std::string Checksum(const std::string& text)
{
    unsigned sum = 0;
    for (unsigned char c : text) sum += c;
    char buf[3];
    std::snprintf(buf, sizeof(buf), "%02X", sum & 0xFFu);
    return buf;
}

// Front-panel channel (L1 = 1) -> wire code ("00").
std::string Wire(int channel)
{
    char buf[3];
    std::snprintf(buf, sizeof(buf), "%02d", channel - 1);
    return buf;
}

// The outcome of one exchange. Rejected keeps the link; Link drops it.
enum class Xfer { Ok, Rejected, Link };

} // namespace

int CcsPd3::PercentToNative(double percent)
{
    if (!std::isfinite(percent)) percent = 0.0;
    percent = std::clamp(percent, 0.0, 100.0);
    return static_cast<int>(std::lround(percent / 100.0 * kFullScale));
}

double CcsPd3::NativeToPercent(int native)
{
    return std::round(std::clamp(native, 0, kFullScale) * 1000.0 / kFullScale) / 10.0;
}

struct CcsPd3::Impl {
    Config               cfg;
    std::vector<Channel> chans;   // enabled, valid, unique — config order

    std::atomic<bool> connected{false};   // a session the caller opened (survives a link drop)
    std::atomic<bool> polling{false};
    std::thread       pollThread;
    int               intervalMs = 1000;

    std::mutex ioMu;                    // socket lifecycle + every transaction
    SOCKET     sock = INVALID_SOCKET;   // ioMu
    bool       linkFailing = false;     // ioMu — log a dropped link once per streak

    // Simulated controller state (ioMu), indexed by front-panel channel.
    int  simF[kMaxChannels + 1] = {};
    bool simL[kMaxChannels + 1] = {};

    mutable std::mutex statusMu;
    Status             status;
    std::string        lastError;

    std::mutex              reqMu;      // UI requests -> poll thread
    std::condition_variable reqCv;
    std::map<int, int>      requests;   // channel -> latest requested value

    DataStream* sPct[kMaxChannels + 1] = {};
    DataStream* sOn[kMaxChannels + 1]  = {};

    explicit Impl(Config c) : cfg(std::move(c))
    {
        bool seen[kMaxChannels + 1] = {};
        for (const Channel& ch : cfg.channels) {
            if (!ch.enabled) continue;
            if (ch.channel < 1 || ch.channel > kMaxChannels || seen[ch.channel]) {
                FD_WARN("pd3", "%s: channel %d ('%s') ignored - must be a unique L1..L%d number",
                        cfg.name.c_str(), ch.channel, ch.name.c_str(), kMaxChannels);
                continue;
            }
            seen[ch.channel] = true;
            chans.push_back(ch);
        }
        for (const Channel& ch : chans) status.ch.push_back({ch.channel, 0, false, "00"});
    }
    ~Impl() { StopPolling(); Disconnect(); }

    bool Has(int channel) const
    {
        for (const Channel& c : chans) if (c.channel == channel) return true;
        return false;
    }

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

    // ---- socket layer (ioMu held) -------------------------------------------

    bool LinkUp() const { return cfg.simulated ? connected.load() : sock != INVALID_SOCKET; }

    void SetLinkStatus()
    {
        std::lock_guard<std::mutex> l(statusMu);
        status.linkUp = LinkUp();
    }

    // Graceful: half-close, then drain (bounded) until the PD3 closes its side,
    // so its single client slot is released at once. Abortive is for a link
    // that is already broken — there is nothing left to say goodbye on.
    void CloseSocket(bool graceful)
    {
        if (sock == INVALID_SOCKET) return;
        if (graceful) {
            shutdown(sock, SD_SEND);
            const DWORD t = kDrainTimeoutMs;
            setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, reinterpret_cast<const char*>(&t), sizeof(t));
            char buf[64];
            for (int i = 0; i < kMaxDrainReads; ++i) {
                if (recv(sock, buf, sizeof(buf), 0) <= 0) break;   // 0 = peer closed
            }
        }
        closesocket(sock);
        sock = INVALID_SOCKET;
    }

    bool OpenSocket()
    {
        CloseSocket(false);
        std::call_once(g_wsaOnce, [] {
            WSADATA d;
            WSAStartup(MAKEWORD(2, 2), &d);
        });

        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port   = htons(static_cast<u_short>(cfg.port));
        if (inet_pton(AF_INET, cfg.ipAddress.c_str(), &addr.sin_addr) <= 0) {
            SetError("bad ip '" + cfg.ipAddress + "'");
            return false;
        }

        SOCKET s = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        if (s == INVALID_SOCKET) { SetError("socket() failed"); return false; }

        // Bounded connect: non-blocking connect + select. A blocking connect to
        // an absent host waits for the OS's ~20 s SYN timeout.
        u_long nb = 1;
        ioctlsocket(s, FIONBIO, &nb);
        const int rc = connect(s, reinterpret_cast<sockaddr*>(&addr), sizeof(addr));
        bool ok = (rc == 0);
        if (!ok && WSAGetLastError() == WSAEWOULDBLOCK) {
            fd_set wr, ex;
            FD_ZERO(&wr); FD_SET(s, &wr);
            FD_ZERO(&ex); FD_SET(s, &ex);
            timeval tv{kConnectTimeoutMs / 1000, (kConnectTimeoutMs % 1000) * 1000};
            ok = select(0, nullptr, &wr, &ex, &tv) > 0 && FD_ISSET(s, &wr);
        }
        if (!ok) {
            closesocket(s);
            SetError("connect to " + cfg.ipAddress + ":" + std::to_string(cfg.port) +
                     " failed (the PD3 accepts only one TCP client - is another program connected?)");
            return false;
        }
        nb = 0;
        ioctlsocket(s, FIONBIO, &nb);

        const DWORD t = kIoTimeoutMs;
        setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, reinterpret_cast<const char*>(&t), sizeof(t));
        setsockopt(s, SOL_SOCKET, SO_SNDTIMEO, reinterpret_cast<const char*>(&t), sizeof(t));
        BOOL noDelay = TRUE;
        setsockopt(s, IPPROTO_TCP, TCP_NODELAY, reinterpret_cast<const char*>(&noDelay), sizeof(noDelay));
        sock = s;
        return true;
    }

    bool SendAll(const std::string& buf)
    {
        size_t off = 0;
        while (off < buf.size()) {
            const int n = send(sock, buf.data() + off, static_cast<int>(buf.size() - off), 0);
            if (n <= 0) return false;
            off += static_cast<size_t>(n);
        }
        return true;
    }

    // One reply through its LF (CR dropped), bounded by kMaxReplyLength; the
    // socket's receive timeout bounds the wait.
    bool RecvLine(std::string& out)
    {
        out.clear();
        for (int i = 0; i < kMaxReplyLength; ++i) {
            char c = 0;
            if (recv(sock, &c, 1, 0) != 1) return false;
            if (c == '\n') return true;
            if (c != '\r') out.push_back(c);
        }
        return false;
    }

    // The in-memory controller: answers a frame the way the PD3 does.
    std::string SimReply(const std::string& body)
    {
        const std::string wire = body.substr(1, 2);
        const int ch = std::atoi(wire.c_str()) + 1;
        const std::string cmd = body.substr(3);
        std::string data;
        bool ok = true;
        if (cmd == "C") {
            data = "00";
        } else if (ch < 1 || ch > kMaxChannels) {
            ok = false; data = "03";
        } else if (cmd.size() == 4 && cmd[0] == 'F') {
            const int v = std::atoi(cmd.c_str() + 1);
            if (v < 0 || v > kFullScale) { ok = false; data = "03"; }
            else simF[ch] = v;
        } else if (cmd == "L0" || cmd == "L1") {
            simL[ch] = (cmd == "L1");
        } else if (cmd == "M") {
            char buf[16];
            std::snprintf(buf, sizeof(buf), "F%03d.S00.L%d", simF[ch], simL[ch] ? 1 : 0);
            data = buf;
        } else {
            ok = false; data = "01";
        }
        const std::string head = "@" + wire + (ok ? "O" : "N") + data;
        return head + Checksum(head);
    }

    // Sends one frame; on Ok `data` is the reply's data field. A Link failure
    // has already dropped the socket.
    Xfer Transact(const std::string& wire, const std::string& command, std::string& data)
    {
        const std::string body = "@" + wire + command;
        std::string reply;
        if (cfg.simulated) {
            if (!connected.load()) return Xfer::Link;
            reply = SimReply(body);
        } else {
            if (sock == INVALID_SOCKET) { SetError("not connected"); return Xfer::Link; }
            if (!SendAll(body + Checksum(body) + "\r\n") || !RecvLine(reply)) {
                return LinkFail("'" + body + "': no reply (timeout or connection closed)");
            }
        }

        if (reply.size() < 6 || reply[0] != '@')
            return LinkFail("'" + body + "': unexpected reply '" + reply + "'");
        if (Checksum(reply.substr(0, reply.size() - 2)) != reply.substr(reply.size() - 2))
            return LinkFail("'" + body + "': reply checksum mismatch in '" + reply + "'");
        if (reply.compare(1, 2, wire) != 0)
            return LinkFail("'" + body + "': reply for another channel '" + reply + "'");

        data = reply.substr(4, reply.size() - 6);
        if (reply[3] == 'O') return Xfer::Ok;
        if (reply[3] == 'N') {
            const char* why = data == "01" ? "command error"
                            : data == "02" ? "checksum error"
                            : data == "03" ? "value out of range" : "rejected";
            SetError("'" + body + "' rejected: " + why + " (N" + data + ")");
            FD_WARN("pd3", "%s: %s", cfg.name.c_str(), Error().c_str());
            return Xfer::Rejected;
        }
        return LinkFail("'" + body + "': unexpected reply '" + reply + "'");
    }

    // A reply that cannot be trusted means the stream is out of step: drop it.
    Xfer LinkFail(const std::string& what)
    {
        SetError(what);
        if (!linkFailing) {
            linkFailing = true;
            FD_ERROR("pd3", "%s: %s - closing the link", cfg.name.c_str(), what.c_str());
        }
        CloseSocket(false);
        SetLinkStatus();
        return Xfer::Link;
    }

    // Reopen a dropped link (real hardware only). Called by the poll thread.
    bool EnsureLink()
    {
        if (cfg.simulated || sock != INVALID_SOCKET) return true;
        if (!OpenSocket()) return false;
        FD_INFO("pd3", "%s: link to %s:%d reopened", cfg.name.c_str(),
                cfg.ipAddress.c_str(), cfg.port);
        linkFailing = false;
        SetLinkStatus();
        return true;
    }

    // ---- commands (ioMu held) -----------------------------------------------

    // 0 drops the output first (F000 still glows); otherwise level, then on.
    Xfer ApplyLocked(int channel, int value)
    {
        value = std::clamp(value, 0, kFullScale);
        const std::string w = Wire(channel);
        std::string d;
        char f[8];
        std::snprintf(f, sizeof(f), "F%03d", value);
        Xfer r;
        if (value == 0) {
            r = Transact(w, "L0", d);
            if (r == Xfer::Ok) r = Transact(w, f, d);
        } else {
            r = Transact(w, f, d);
            if (r == Xfer::Ok) r = Transact(w, "L1", d);
        }
        if (r == Xfer::Ok) {
            std::lock_guard<std::mutex> l(statusMu);
            for (ChannelStatus& s : status.ch) {
                if (s.channel != channel) continue;
                s.intensity = value;
                s.on        = value != 0;
            }
        }
        return r;
    }

    // Unit error check, then an M read-back of every configured channel.
    Xfer PollLocked()
    {
        std::string d;
        Xfer r = Transact("00", "C", d);
        if (r != Xfer::Ok) return r;
        const bool unitError = (d == "11");

        std::vector<ChannelStatus> next;
        std::string warn = unitError ? "controller reports overcurrent / fan-stop error" : "";
        for (const Channel& c : chans) {
            r = Transact(Wire(c.channel), "M", d);
            if (r != Xfer::Ok) return r;
            // "F075.S04.L0"
            if (d.size() != 11 || d[0] != 'F' || d[4] != '.' || d[5] != 'S' ||
                d[8] != '.' || d[9] != 'L' || (d[10] != '0' && d[10] != '1') ||
                !std::isdigit(static_cast<unsigned char>(d[1])) ||
                !std::isdigit(static_cast<unsigned char>(d[2])) ||
                !std::isdigit(static_cast<unsigned char>(d[3]))) {
                return LinkFail("unexpected status reply '" + d + "' for L" +
                                std::to_string(c.channel));
            }
            ChannelStatus s;
            s.channel   = c.channel;
            s.intensity = std::atoi(d.substr(1, 3).c_str());
            s.mode      = d.substr(6, 2);
            s.on        = d[10] == '1';
            if (s.mode != "00") {
                if (!warn.empty()) warn += "; ";
                warn += "L" + std::to_string(c.channel) + " (" + c.name + ") is in strobe mode S" +
                        s.mode + " - dark without a trigger";
            }
            next.push_back(s);
        }

        std::lock_guard<std::mutex> l(statusMu);
        status.ch        = std::move(next);
        status.valid     = true;
        status.unitError = unitError;
        status.warning   = warn;
        status.linkUp    = LinkUp();
        return Xfer::Ok;
    }

    // ---- device API ---------------------------------------------------------

    bool Connect()
    {
        std::lock_guard<std::mutex> lock(ioMu);
        if (connected.load()) return true;
        if (cfg.simulated) {
            connected.store(true);
        } else {
            if (!OpenSocket()) {
                FD_ERROR("pd3", "%s: %s", cfg.name.c_str(), Error().c_str());
                return false;
            }
            connected.store(true);
        }
        linkFailing = false;
        SetError("");
        // Read the real state - nothing is written at connect. A failure here
        // closes the link again and the connect fails.
        if (PollLocked() != Xfer::Ok) {
            CloseSocket(false);
            connected.store(false);
            SetLinkStatus();
            FD_ERROR("pd3", "%s: first read-back failed: %s", cfg.name.c_str(), Error().c_str());
            return false;
        }
        FD_INFO("pd3", "%s connected to %s:%d%s (%zu channel(s))", cfg.name.c_str(),
                cfg.ipAddress.c_str(), cfg.port, cfg.simulated ? " (simulated)" : "", chans.size());
        return true;
    }

    void Disconnect()
    {
        StopPolling();
        std::lock_guard<std::mutex> lock(ioMu);
        if (!connected.exchange(false)) return;
        CloseSocket(true);
        SetLinkStatus();
        FD_INFO("pd3", "%s disconnected", cfg.name.c_str());
    }

    bool Set(const std::vector<std::pair<int, int>>& values)
    {
        for (const auto& cv : values) {
            if (!Has(cv.first)) {
                SetError("L" + std::to_string(cv.first) + " is not a configured channel");
                return false;
            }
        }
        std::lock_guard<std::mutex> lock(ioMu);
        if (!connected.load()) { SetError("not connected"); return false; }
        if (!cfg.simulated && sock == INVALID_SOCKET) { SetError("link down - reconnecting"); return false; }
        // Best effort: every channel is attempted (a call-out must not leave a
        // light half-switched because one channel refused), the result is the AND.
        bool ok = true;
        for (const auto& [ch, v] : values) {
            const Xfer r = ApplyLocked(ch, v);
            if (r != Xfer::Ok) ok = false;
            if (r == Xfer::Link) break;   // nothing more will get through
        }
        return ok;
    }

    bool SetOutput(int channel, bool on)
    {
        if (!Has(channel)) { SetError("L" + std::to_string(channel) + " is not a configured channel"); return false; }
        std::lock_guard<std::mutex> lock(ioMu);
        if (!connected.load()) { SetError("not connected"); return false; }
        std::string d;
        if (Transact(Wire(channel), on ? "L1" : "L0", d) != Xfer::Ok) return false;
        std::lock_guard<std::mutex> l(statusMu);
        for (ChannelStatus& s : status.ch) if (s.channel == channel) s.on = on;
        return true;
    }

    void Request(int channel, int value)
    {
        if (!Has(channel)) return;
        {
            std::lock_guard<std::mutex> l(reqMu);
            requests[channel] = std::clamp(value, 0, kFullScale);
        }
        reqCv.notify_one();
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
        intervalMs = ms > 0 ? ms : 1000;
        for (const Channel& c : chans) {
            sPct[c.channel] = &DataBus::Instance().Stream(topics::LightPct(c.channel));
            sOn[c.channel]  = &DataBus::Instance().Stream(topics::LightOn(c.channel));
        }
        polling.store(true);
        pollThread = std::thread(&Impl::PollLoop, this);
        FD_INFO("pd3", "%s polling every %d ms", cfg.name.c_str(), intervalMs);
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
            std::map<int, int> todo;
            {
                std::lock_guard<std::mutex> l(reqMu);
                todo.swap(requests);
            }
            if (!todo.empty()) {
                std::vector<std::pair<int, int>> v(todo.begin(), todo.end());
                if (!Set(v)) FD_WARN("pd3", "%s: set failed: %s", cfg.name.c_str(), Error().c_str());
            }

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
                    for (const ChannelStatus& c : s.ch) {
                        if (sPct[c.channel]) sPct[c.channel]->Publish(t, NativeToPercent(c.intensity));
                        if (sOn[c.channel])  sOn[c.channel]->Publish(t, c.on ? 1.0 : 0.0);
                    }
                }
                nextPoll = std::chrono::steady_clock::now() + std::chrono::milliseconds(intervalMs);
            }

            std::unique_lock<std::mutex> l(reqMu);
            reqCv.wait_until(l, nextPoll, [this] { return !polling.load() || !requests.empty(); });
        }
    }

    Status GetStatus() const
    {
        std::lock_guard<std::mutex> l(statusMu);
        return status;
    }
};

// ---- Forwarding shell ----
CcsPd3::CcsPd3(Config cfg) : p_(std::make_unique<Impl>(std::move(cfg))) {}
CcsPd3::~CcsPd3() = default;

bool CcsPd3::Connect()           { return p_->Connect(); }
void CcsPd3::Disconnect()        { p_->Disconnect(); }
bool CcsPd3::IsConnected() const { return p_->connected.load(); }

bool CcsPd3::SetIntensity(int channel, int value) { return p_->Set({{channel, value}}); }
bool CcsPd3::SetIntensities(const std::vector<std::pair<int, int>>& v) { return p_->Set(v); }
bool CcsPd3::SetOutput(int channel, bool on)      { return p_->SetOutput(channel, on); }
void CcsPd3::Request(int channel, int value)      { p_->Request(channel, value); }

CcsPd3::Status CcsPd3::GetStatus() const { return p_->GetStatus(); }

bool CcsPd3::StartPolling(int intervalMs) { return p_->StartPolling(intervalMs); }
void CcsPd3::StopPolling()                { p_->StopPolling(); }
bool CcsPd3::IsPolling() const            { return p_->polling.load(); }
bool CcsPd3::PollOnce()                   { return p_->PollOnce(); }

const std::vector<CcsPd3::Channel>& CcsPd3::Channels() const { return p_->chans; }
const CcsPd3::Channel* CcsPd3::FindChannel(int channel) const
{
    for (const Channel& c : p_->chans) if (c.channel == channel) return &c;
    return nullptr;
}
std::string CcsPd3::DeviceId() const
{
    return "CCS PD3 @ " + p_->cfg.ipAddress + ":" + std::to_string(p_->cfg.port) +
           (p_->cfg.simulated ? " (simulated)" : "");
}
std::string CcsPd3::LastError() const { return p_->Error(); }

} // namespace firedog
