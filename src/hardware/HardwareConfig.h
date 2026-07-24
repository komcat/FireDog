// HardwareConfig.h : consolidated hardware config (config/hardware.json).
// Cameras first; motion/instruments sections come as those modules are ported.
#pragma once

#include <map>
#include <string>
#include <vector>

namespace firedog {

// Global->local axis transform for a motion device (3x3 rotation/permutation).
// local = M * global. Loaded from config/motion_transforms.json, keyed by device
// id/name. Defaults to identity.
struct AxisTransform {
    double m[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
};

// Tolerant loader — returns an empty map if the file is missing/invalid.
std::map<std::string, AxisTransform> LoadMotionTransforms(const std::string& path);

struct CameraConfig {
    std::string id;
    std::string cameraType    = "PYLON";   // PYLON | IDS
    std::string connectionType = "ip";     // ip | serial | index | auto
    std::string ipAddress;
    std::string serialNumber;
    std::string displayName;
    bool        enabled     = true;
    bool        autoConnect = true;
    int         deviceIndex = 0;
    double      pixelToMmX  = 0.00248;   // camera-pixel -> mm calibration (X)
    double      pixelToMmY  = 0.00248;   // camera-pixel -> mm calibration (Y)
};

struct KeithleyConfig {
    std::string      name = "Keithley_6482";
    std::string      type = "keithley_6482";
    std::string      visaResource = "GPIB0::1::INSTR";
    bool             enabled = true;
    int              channelCount = 2;
    std::vector<int> enabledChannels{1, 2};
    double           nplc = 5.0;
    bool             autoRange = true;
    double           fixedRange = 0.0;
    int              pollIntervalMs = 100;
};

// One motion controller entry (from "motion_devices"): a PI hexapod or an ACS
// gantry, connected over TCP/IP. Shared shape for both controller kinds.
struct MotionConfig {
    std::string id;
    std::string name;
    std::string controllerType;      // "PI" | "ACS"
    std::string axes;                // space-separated, e.g. "X Y Z U V W"
    std::string ipAddress;
    int         port     = 0;
    int         deviceId = 0;
    bool        enabled  = true;
};

struct HardwareConfig {
    std::vector<CameraConfig>   cameras;
    std::vector<KeithleyConfig> keithleys;      // from "instruments" (type keithley_*)
    std::vector<MotionConfig>   motionDevices;  // from "motion_devices" (PI / ACS)

    // Tolerant loader — returns an empty config if the file is missing/invalid.
    static HardwareConfig Load(const std::string& path);

    const CameraConfig*   FirstEnabledCamera() const;
    const KeithleyConfig* FirstEnabledKeithley() const;
    std::vector<MotionConfig> EnabledMotionDevices() const;   // enabled only
};

} // namespace firedog
