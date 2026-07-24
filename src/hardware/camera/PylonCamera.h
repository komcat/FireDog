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

    std::string ModelName() const;
    std::string LastError() const;

private:
    struct Impl;
    std::unique_ptr<Impl> p_;
};

} // namespace firedog
