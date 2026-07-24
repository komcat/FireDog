// PylonCamera.cpp
//
// Direct Pylon SDK usage. The connect/configure/grab/convert calls are ported
// from bobcat_engine's pylon_camera.cpp (the battle-tested source), condensed
// to a single class for one GigE camera.
#include "hardware/camera/PylonCamera.h"

#ifdef HAS_PYLON
// ---- Windows macro guards must precede any Pylon (winsock) includes ----
#ifdef _WIN32
#define _WINSOCKAPI_
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#endif
#include <pylon/PylonIncludes.h>

#include <atomic>
#include <chrono>
#include <iostream>
#include <mutex>
#include <thread>

namespace firedog {

struct PylonCamera::Impl {
    Pylon::CInstantCamera        camera;
    Pylon::CImageFormatConverter converter;

    std::thread          grabThread;
    std::atomic<bool>    running{false};
    std::atomic<bool>    connected{false};

    std::mutex                 frameMutex;
    std::vector<unsigned char> latestRGB;
    int                        width = 0;
    int                        height = 0;
    std::atomic<bool>          newFrame{false};

    std::string model;
    std::string lastError;

    Impl()
    {
        Pylon::PylonInitialize();
        converter.OutputPixelFormat  = Pylon::PixelType_RGB8packed;
        converter.OutputBitAlignment = Pylon::OutputBitAlignment_MsbAligned;
    }

    ~Impl()
    {
        StopGrabbing();
        try {
            if (camera.IsOpen()) camera.Close();
            if (camera.IsPylonDeviceAttached()) camera.DetachDevice();
        } catch (...) {}
        Pylon::PylonTerminate();
    }

    bool IsConnected() const { return connected.load(); }
    bool IsGrabbing()  const { return running.load(); }
    std::string Model() const { return model; }
    std::string LastError() const { return lastError; }

    // Enumerate GigE devices and match one by its IP address.
    bool FindByIP(const std::string& ip, Pylon::CDeviceInfo& out)
    {
        try {
            Pylon::CTlFactory& tl = Pylon::CTlFactory::GetInstance();
            Pylon::DeviceInfoList_t devices;
            if (tl.EnumerateDevices(devices) == 0) return false;

            for (size_t i = 0; i < devices.size(); ++i) {
                const auto& dev = devices[i];
                std::string cls(dev.GetDeviceClass().c_str());
                if (cls.find("GigE") == std::string::npos &&
                    cls.find("Gige") == std::string::npos) {
                    continue;
                }

                std::string camIP;
                try {
                    Pylon::String_t s = dev.GetIpAddress();
                    if (!s.empty()) camIP = std::string(s.c_str());
                } catch (...) {}

                if (camIP.empty()) {
                    try {
                        std::string full(dev.GetFullName().c_str());
                        size_t hash = full.rfind('#');
                        if (hash != std::string::npos) {
                            std::string after = full.substr(hash + 1);
                            size_t colon = after.find(':');
                            if (colon != std::string::npos) camIP = after.substr(0, colon);
                        }
                    } catch (...) {}
                }

                if (!camIP.empty() && camIP == ip) { out = dev; return true; }
            }
        }
        catch (const Pylon::GenericException& e) {
            lastError = e.GetDescription();
        }
        return false;
    }

    void ConfigureGigE()
    {
        try {
            if (camera.GetTLNodeMap().GetNode("HeartbeatTimeout")) {
                Pylon::CIntegerParameter(camera.GetTLNodeMap(), "HeartbeatTimeout")
                    .TrySetValue(1000, Pylon::IntegerValueCorrection_Nearest);
            }
            if (camera.GetTLNodeMap().GetNode("GevSCPSPacketSize")) {
                Pylon::CIntegerParameter packet(camera.GetTLNodeMap(), "GevSCPSPacketSize");
                packet.SetValue(packet.GetMax());
            }
        }
        catch (const Pylon::GenericException& e) {
            std::cerr << "[PylonCamera] GigE config warning: " << e.GetDescription() << std::endl;
        }
    }

    bool Open(const Pylon::CDeviceInfo& info)
    {
        try {
            camera.Attach(Pylon::CTlFactory::GetInstance().CreateDevice(info));
            camera.Open();

            std::string cls(camera.GetDeviceInfo().GetDeviceClass().c_str());
            if (cls.find("GigE") != std::string::npos) ConfigureGigE();

            if (camera.GetNodeMap().GetNode("TriggerMode")) {
                Pylon::CEnumParameter(camera.GetNodeMap(), "TriggerMode").SetValue("Off");
            }

            model = std::string(camera.GetDeviceInfo().GetModelName().c_str());
            connected.store(true);
            std::cout << "[PylonCamera] connected: " << model << std::endl;
            return true;
        }
        catch (const Pylon::GenericException& e) {
            lastError = e.GetDescription();
            std::cerr << "[PylonCamera] open failed: " << lastError << std::endl;
            return false;
        }
    }

    bool ConnectByIP(const std::string& ip)
    {
        Pylon::CDeviceInfo info;
        if (!FindByIP(ip, info)) {
            lastError = "Camera with IP " + ip + " not found";
            std::cerr << "[PylonCamera] " << lastError << std::endl;
            return false;
        }
        return Open(info);
    }

    bool ConnectFirst()
    {
        try {
            Pylon::DeviceInfoList_t devices;
            if (Pylon::CTlFactory::GetInstance().EnumerateDevices(devices) == 0) {
                lastError = "No camera found";
                return false;
            }
            return Open(devices[0]);
        }
        catch (const Pylon::GenericException& e) {
            lastError = e.GetDescription();
            return false;
        }
    }

    bool StartGrabbing()
    {
        if (!connected.load()) { lastError = "Not connected"; return false; }
        try {
            camera.MaxNumBuffer = 5;
            camera.StartGrabbing(Pylon::GrabStrategy_LatestImageOnly,
                                 Pylon::GrabLoop_ProvidedByUser);
            running.store(true);
            grabThread = std::thread(&Impl::GrabLoop, this);
            return true;
        }
        catch (const Pylon::GenericException& e) {
            lastError = e.GetDescription();
            return false;
        }
    }

    void StopGrabbing()
    {
        if (running.exchange(false)) {
            if (grabThread.joinable()) grabThread.join();
        }
        try {
            if (camera.IsGrabbing()) camera.StopGrabbing();
        } catch (...) {}
    }

    void Disconnect()
    {
        StopGrabbing();
        try {
            if (camera.IsOpen()) camera.Close();
        } catch (...) {}
        connected.store(false);
    }

    void GrabLoop()
    {
        while (running.load()) {
            try {
                Pylon::CGrabResultPtr result;
                // Short timeout + Return so StopGrabbing() is responsive.
                if (camera.RetrieveResult(50, result, Pylon::TimeoutHandling_Return) &&
                    result && result->GrabSucceeded()) {
                    const int w = static_cast<int>(result->GetWidth());
                    const int h = static_cast<int>(result->GetHeight());

                    Pylon::CPylonImage src, dst;
                    src.AttachGrabResultBuffer(result);
                    converter.Convert(dst, src);

                    const auto*  buf = static_cast<const unsigned char*>(dst.GetBuffer());
                    const size_t sz  = static_cast<size_t>(w) * h * 3;

                    {
                        std::lock_guard<std::mutex> lock(frameMutex);
                        latestRGB.assign(buf, buf + sz);
                        width  = w;
                        height = h;
                    }
                    newFrame.store(true);
                }
                else {
                    std::this_thread::sleep_for(std::chrono::milliseconds(2));
                }
            }
            catch (const Pylon::GenericException& e) {
                std::cerr << "[PylonCamera] grab error: " << e.GetDescription() << std::endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
        }
    }

    bool GetLatestFrame(std::vector<unsigned char>& out, int& w, int& h, bool& isNew)
    {
        std::lock_guard<std::mutex> lock(frameMutex);
        if (latestRGB.empty()) return false;
        out   = latestRGB;
        w     = width;
        h     = height;
        isNew = newFrame.exchange(false);
        return true;
    }
};

} // namespace firedog

#else // ---------------- No Pylon SDK: animated test-pattern stub ----------------

#include <atomic>

namespace firedog {

struct PylonCamera::Impl {
    std::atomic<bool> connected{false};
    std::atomic<bool> grabbing{false};
    int         phase = 0;
    std::string lastError;

    bool IsConnected() const { return connected.load(); }
    bool IsGrabbing()  const { return grabbing.load(); }
    std::string Model() const { return "Test Pattern (no Pylon SDK)"; }
    std::string LastError() const { return lastError; }

    bool ConnectByIP(const std::string&) { connected.store(true); return true; }
    bool ConnectFirst()                  { connected.store(true); return true; }
    void Disconnect()                    { grabbing.store(false); connected.store(false); }

    bool StartGrabbing() { if (!connected.load()) return false; grabbing.store(true); return true; }
    void StopGrabbing()  { grabbing.store(false); }

    bool GetLatestFrame(std::vector<unsigned char>& out, int& w, int& h, bool& isNew)
    {
        if (!grabbing.load()) return false;
        const int W = 640, H = 480;
        w = W; h = H;
        out.resize(static_cast<size_t>(W) * H * 3);
        const int ph = phase++;
        for (int y = 0; y < H; ++y) {
            for (int x = 0; x < W; ++x) {
                const size_t i = (static_cast<size_t>(y) * W + x) * 3;
                out[i + 0] = static_cast<unsigned char>((x + ph) & 0xFF);
                out[i + 1] = static_cast<unsigned char>((y + ph) & 0xFF);
                out[i + 2] = static_cast<unsigned char>((x + y) & 0xFF);
            }
        }
        isNew = true;
        return true;
    }
};

} // namespace firedog

#endif // HAS_PYLON

// ---- Thin forwarding shell (identical for both Impls) ----
namespace firedog {

PylonCamera::PylonCamera()  : p_(std::make_unique<Impl>()) {}
PylonCamera::~PylonCamera() = default;

bool PylonCamera::ConnectByIP(const std::string& ip) { return p_->ConnectByIP(ip); }
bool PylonCamera::ConnectFirst()                     { return p_->ConnectFirst(); }
void PylonCamera::Disconnect()                       { p_->Disconnect(); }
bool PylonCamera::StartGrabbing()                    { return p_->StartGrabbing(); }
void PylonCamera::StopGrabbing()                     { p_->StopGrabbing(); }
bool PylonCamera::IsConnected() const                { return p_->IsConnected(); }
bool PylonCamera::IsGrabbing()  const                { return p_->IsGrabbing(); }
std::string PylonCamera::ModelName() const           { return p_->Model(); }
std::string PylonCamera::LastError() const           { return p_->LastError(); }

bool PylonCamera::GetLatestFrame(std::vector<unsigned char>& rgb, int& w, int& h, bool& isNew)
{
    return p_->GetLatestFrame(rgb, w, h, isNew);
}

} // namespace firedog
