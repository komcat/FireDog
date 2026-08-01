// PylonCamera.h : layer 1 — direct Basler Pylon SDK camera (no wrapper stack).
//
// Talks to the Pylon SDK directly. PIMPL keeps all Pylon/Windows headers out
// of this header, so UI code can include it without macro clashes. When the
// Pylon SDK is not found at build time (no HAS_PYLON), it falls back to an
// animated test-pattern stub so the feed pipeline still runs.
#pragma once

#include <memory>
#include <string>
#include <vector>

namespace firedog {

class PylonCamera {
public:
    PylonCamera();
    ~PylonCamera();

    PylonCamera(const PylonCamera&) = delete;
    PylonCamera& operator=(const PylonCamera&) = delete;

    bool ConnectByIP(const std::string& ipAddress);   // GigE, by IP
    bool ConnectFirst();                               // first available device
    void Disconnect();

    bool StartGrabbing();
    void StopGrabbing();

    bool IsConnected() const;
    bool IsGrabbing() const;

    // Copy the latest RGB8 frame into `rgb` (size w*h*3). Returns false if no
    // frame is available yet. `isNew` is true if this frame is fresh since the
    // last call.
    //
    // `isNew` is a ONE-SHOT flag: the first caller of a frame consumes it. With
    // two consumers (the UI texture upload and the focus sampler) that races,
    // so anything that must reason about frame freshness uses the sequence
    // number below instead.
    bool GetLatestFrame(std::vector<unsigned char>& rgb, int& width, int& height, bool& isNew);

    // Same copy, but reports the frame's monotonic sequence number instead of a
    // consumable flag. `seq` only advances when a genuinely new frame arrived,
    // so any number of consumers can gate on "seq > seen" independently — this
    // is what the auto-focus sweep uses to prove a sample is post-move (the
    // camera analogue of the DataBus publish counter).
    bool GetLatestFrameSeq(std::vector<unsigned char>& rgb, int& width, int& height,
                           unsigned long long& seq);
    unsigned long long FrameSeq() const;   // cheap poll, no copy

    // One consistent snapshot of camera identity + live stats for the UI.
    struct Info {
        std::string model;
        std::string serial;
        std::string ip;
        int    width  = 0;      // latest frame size (0 until a frame arrives)
        int    height = 0;
        double fps    = 0.0;    // measured grab rate
        bool   grabbing = false;
    };
    Info GetInfo() const;

    // Exposure in microseconds (SFNC "ExposureTime", legacy GigE
    // "ExposureTimeAbs"; ExposureAuto is forced Off at connect). Get returns
    // <= 0 when not connected/supported; Set clips to the camera's range.
    double ExposureUs() const;
    bool   SetExposureUs(double us);
    void   ExposureRange(double& minUs, double& maxUs) const;   // 0,0 if unknown

    std::string ModelName() const;
    std::string LastError() const;

private:
    struct Impl;
    std::unique_ptr<Impl> p_;
};

} // namespace firedog
