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
    bool GetLatestFrame(std::vector<unsigned char>& rgb, int& width, int& height, bool& isNew);

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
