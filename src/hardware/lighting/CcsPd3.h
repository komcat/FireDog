// CcsPd3.h : layer 1 — CCS PD3 (PD3-10024-8-EI) LED lighting controller over
// TCP, per the CCS instruction guide KZ03995 §8. Ported from Horizon's
// CcsPd3Adapter (the hardware-verified source).
//
// Frame: '@' + wire channel (2 digits) + command + data + checksum (2 hex) +
// CR LF. The checksum is the low byte of the ASCII sum from '@' through the
// data. Replies are "@ccO<data>ss" (accepted) or "@ccN<ee>ss" (rejected: 01
// command, 02 checksum, 03 range). Commands used: F000..F255 intensity, L0/L1
// output off/on, M read "F075.S04.L0", C unit error check ("11" = overcurrent
// or fan stop).
//
// Channels are numbered as on the FRONT PANEL (L1..L8 = 1..8); the wire code is
// 0-based (L1 = "00"). That translation happens here and nowhere else.
//
// Three facts that are easy to get wrong and expensive when you do:
//  - The PD3 accepts ONE TCP client. The link is held open and closed
//    GRACEFULLY (half-close, then drain until the peer closes) — an abortive
//    close can leave the controller's only slot busy until it is power-cycled.
//  - F000 is NOT dark (~0.4 % glow on 24 V light units), so intensity 0 means
//    "output off": L0 then F000. Any other value is Fnnn then L1.
//  - A channel in strobe mode (S != 00) stays dark without a trigger; the poll
//    reports it as a warning rather than leaving the operator to guess.
//
// A rejected command (NG) keeps the link; any I/O error, timeout or malformed
// reply drops it, because the stream can no longer be trusted to be in step.
// A dropped link is reopened by the poll thread (the WAGO pattern), so a pulled
// cable recovers on its own; `Status::linkUp` says which side of that you are on.
//
// Raw Winsock behind the PIMPL, so there is no SDK gate. `Config::simulated`
// selects an in-memory stub with the same API.
#pragma once

#include <memory>
#include <string>
#include <vector>

namespace firedog {

class CcsPd3 {
public:
    static constexpr int kMaxChannels = 8;    // L1..L8
    static constexpr int kFullScale   = 255;  // F255

    struct Channel {
        int         channel = 1;     // front-panel number, 1..8
        std::string name;            // "Red"
        std::string color;           // "red" | "green" | "blue" | "collimate" | ""
        std::string group;           // logical light ("main_camera_light"); "" = a light by its name
        std::string camera;          // camera it illuminates; "" = any
        bool        enabled = true;
    };

    struct Config {
        std::string          name = "LIGHT1";
        std::string          ipAddress;
        int                  port = 40001;
        int                  pollIntervalMs = 1000;
        bool                 simulated = false;
        std::vector<Channel> channels;   // invalid / duplicate numbers are dropped
    };

    // Percent of full scale <-> controller-native units. The one conversion:
    // every percent a caller types goes through PercentToNative.
    static int    PercentToNative(double percent);   // clamped to 0..100 first
    static double NativeToPercent(int native);

    explicit CcsPd3(Config cfg);
    ~CcsPd3();

    CcsPd3(const CcsPd3&) = delete;
    CcsPd3& operator=(const CcsPd3&) = delete;

    // Open the socket, then read every configured channel's real state (ON/OFF
    // is not retained across a PD3 power cycle). Nothing is written at connect.
    bool Connect();
    // Stops polling, then closes GRACEFULLY. The outputs are left as they are.
    void Disconnect();
    bool IsConnected() const;

    // ---- Blocking setters (worker threads: recipes, alignment) --------------
    // Channel is the front-panel number. 0..kFullScale; 0 = output off.
    bool SetIntensity(int channel, int value);
    // Several channels in ONE locked sequence — a colour call-out must not be
    // interleaved with a poll that would show half of it.
    bool SetIntensities(const std::vector<std::pair<int, int>>& channelValues);
    // Output on/off without touching the intensity setpoint.
    bool SetOutput(int channel, bool on);

    // ---- Non-blocking setter (UI thread) ------------------------------------
    // Queues the value; the poll thread sends it within a few milliseconds and
    // the latest request per channel wins. The UI never does socket I/O.
    void Request(int channel, int value);

    struct ChannelStatus {
        int         channel   = 0;
        int         intensity = 0;      // 0..255 as read back
        bool        on        = false;
        std::string mode;               // "00" = continuous; anything else is strobe
    };
    struct Status {
        std::vector<ChannelStatus> ch;   // one per configured channel, config order
        bool        valid     = false;   // false until the first successful read-back
        bool        linkUp    = false;   // socket open right now
        bool        unitError = false;   // C -> "11": overcurrent or fan stop
        std::string warning;             // unit error / strobe channels, "" when clean
    };
    // Cached; never touches the socket.
    Status GetStatus() const;

    // Read-back every intervalMs (<=0 -> 1000 ms) and publish topics::LightPct/
    // LightOn per channel. Also drains Request() and reopens a dropped link.
    bool StartPolling(int intervalMs);
    void StopPolling();
    bool IsPolling() const;

    // One synchronous read-back (unit check + M per channel). For the rig.
    bool PollOnce();

    const std::vector<Channel>& Channels() const;
    const Channel*              FindChannel(int channel) const;
    std::string                 DeviceId() const;
    std::string                 LastError() const;

private:
    struct Impl;
    std::unique_ptr<Impl> p_;
};

} // namespace firedog
