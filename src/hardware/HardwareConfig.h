// HardwareConfig.h : consolidated hardware config (config/hardware.json).
// Cameras first; motion/instruments sections come as those modules are ported.
#pragma once

#include <map>
#include <string>
#include <vector>

namespace firedog {

// Global->local axis transform for a motion device. local = M * global, in
// two independent 3x3 signed-permutation blocks: `m` for translation (global
// X,Y,Z) and `r` for rotation (global U=thetaX, V=thetaY, W=thetaZ -> local
// rotational axes, indexed 0..2 == the device's axes beyond XYZ). Loaded from
// config/motion_transforms.json, keyed by device name. Defaults to identity.
// Column j of a block encodes where global axis j goes: entry (i,j) = sign.
struct AxisTransform {
    double m[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    double r[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
};

// Tolerant loader — returns an empty map if the file is missing/invalid.
std::map<std::string, AxisTransform> LoadMotionTransforms(const std::string& path);

// Rewrite the whole transforms file from the map (every entry, including
// devices not currently in the fleet). Used by the Transforms editor's Save.
bool SaveMotionTransforms(const std::string& path,
                          const std::map<std::string, AxisTransform>& transforms);

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
    double      exposureUs  = 0.0;       // applied after connect; 0 = camera default
};

// The current-measuring instrument (from "instruments", any type containing
// "keithley"). `type` picks the DRIVER, and that is the only difference the
// rest of the program sees — both are ISmu:
//   "keithley_6482"  SCPI picoammeter, GPIB/USB ("GPIB0::1::INSTR")
//   "keithley_2602b" TSP SourceMeter over LAN ("TCPIP0::<ip>::INSTR", or
//                    "TCPIP0::<ip>::5025::SOCKET" if VXI-11 does not enumerate)
// Exactly one entry should be `enabled` — FirstEnabledKeithley takes the first.
struct KeithleyConfig {
    std::string      name = "Keithley_6482";
    std::string      type = "keithley_6482";   // selects the ISmu driver
    std::string      visaResource = "GPIB0::1::INSTR";
    bool             enabled = true;
    int              channelCount = 2;
    std::vector<int> enabledChannels{1, 2};
    double           nplc = 5.0;
    bool             autoRange = true;
    double           fixedRange = 0.0;
    int              pollIntervalMs = 100;
    double           biasVoltage = 1.0;   // ch1 bias applied after connect
};

// Thorlabs CLD101x laser-diode + TEC controller (from "instruments", type
// containing "cld"). Defaults seed the UI only — never pushed at connect.
struct Cld101xConfig {
    std::string name = "laser-main";
    std::string type = "cld101x";
    std::string visaResource;
    bool        enabled = true;
    double      defaultTemperature    = 25.0;   // deg C
    double      defaultLaserCurrentMA = 50.0;   // mA
    int         pollIntervalMs        = 500;
};

// UNI-T UDP3305S 3-channel DC power supply (from "instruments", type
// containing "udp3305"). Setpoints seed from the device at connect — the
// config carries only identity + polling.
struct PsuConfig {
    std::string name = "PSU3";
    std::string type = "unit_t_udp3305s";
    std::string visaResource;
    bool        enabled = true;
    int         channels = 3;
    int         pollIntervalMs = 500;
};

// CCS PD3 LED lighting controller (from "instruments", type containing "pd3"),
// raw TCP on the LAN. Channels are numbered as on the FRONT PANEL (L1..L8 =
// 1..8). `color` drives the colour call-outs (red/green/blue mix to white,
// "collimate" is its own); `group` names the logical light several channels
// make together — an ungrouped channel is a light by its own name.
struct LightChannelConfig {
    int         channel = 1;
    std::string name;
    std::string color;
    std::string group;
    std::string camera;
    bool        enabled = true;
};

struct LightConfig {
    std::string name = "LIGHT1";
    std::string type = "ccs_pd3";
    std::string ipAddress;
    int         port = 40001;
    bool        enabled = true;
    bool        simulated = false;   // in-memory controller, no socket
    int         pollIntervalMs = 1000;
    std::vector<LightChannelConfig> channels;
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

// One digital-IO node (from "io_devices"): a FASTECH Ezi-IO board or a WAGO
// Modbus TCP coupler. Shared shape for both device kinds.
struct IoDeviceConfig {
    std::string id;
    std::string name;
    std::string type;                // "eziio" | "wago"
    std::string ipAddress;
    int         port           = 502;    // wago Modbus port; eziio ignores it
    int         deviceId       = 0;      // eziio FAS board id — unique per board
    int         inputCount     = 8;
    int         outputCount    = 8;
    int         pollIntervalMs = 100;
    bool        enabled        = true;
    bool        simulated      = false;  // wago only: in-memory sim, no sockets
    std::map<int, std::string> inputNames;    // pin -> display name (from "pins")
    std::map<int, std::string> outputNames;
};

// One pneumatic slide (from "pneumatic_slides"): a synthetic device built from
// IO pins — one output (ON = extend), two position sensors. Pins by NAME,
// resolved against the io_devices pin maps at startup.
struct PneumaticSlideConfig {
    std::string name;
    std::string outputDevice,    outputPin;
    std::string extendedDevice,  extendedPin;
    std::string retractedDevice, retractedPin;
    int  timeoutMs = 10000;
    bool enabled   = true;
};

// One IO-watch binding (from "io_watch"): an input edge that fires an action
// on an output pin. Pins by NAME, resolved at startup like pneumatic slides.
struct IoWatchBindingConfig {
    std::string name;
    std::string inputDevice,  inputPin;
    std::string outputDevice, outputPin;
    std::string trigger  = "rising";    // rising | falling | both
    std::string behavior = "toggle";    // toggle | set_on | set_off | momentary
    int  debounceMs = 100;
    bool enabled    = true;
};

struct IoWatchConfig {
    bool enabled           = false;
    int  pollingIntervalMs = 50;
    std::vector<IoWatchBindingConfig> bindings;
};

// One watched safety input (from "safety_watch"): an edge that latches an
// alert; critical signals also stop all motion (wired in Workflow).
struct SafetySignalConfig {
    std::string device;
    std::string pinName;                // preferred reference
    int         pinNumber = 0;          // fallback if the name doesn't resolve
    std::string trigger = "rising";     // rising | falling | both
    std::string alertTitle   = "Safety Alert";
    std::string alertMessage = "A safety condition was triggered.";
    bool        critical = false;
    bool        enabled  = true;
};

struct SafetyWatchConfig {
    bool enabled           = false;
    int  pollingIntervalMs = 100;
    std::vector<SafetySignalConfig> watchedInputs;
};

struct HardwareConfig {
    std::vector<CameraConfig>   cameras;
    std::vector<KeithleyConfig> keithleys;      // from "instruments" (type keithley_*)
    std::vector<Cld101xConfig>  clds;           // from "instruments" (type cld*)
    std::vector<PsuConfig>      psus;           // from "instruments" (type *udp3305*); every enabled one is built
    std::vector<LightConfig>    lights;         // from "instruments" (type *pd3*)
    std::vector<MotionConfig>   motionDevices;  // from "motion_devices" (PI / ACS)
    std::vector<IoDeviceConfig> ioDevices;      // from "io_devices" (eziio / wago)
    std::vector<PneumaticSlideConfig> pneumaticSlides;  // from "pneumatic_slides"
    IoWatchConfig     ioWatch;                          // from "io_watch"
    SafetyWatchConfig safetyWatch;                      // from "safety_watch"

    // Tolerant loader — returns an empty config if the file is missing/invalid.
    static HardwareConfig Load(const std::string& path);

    const CameraConfig*   FirstEnabledCamera() const;
    const KeithleyConfig* FirstEnabledKeithley() const;
    const Cld101xConfig*  FirstEnabledCld() const;
    std::vector<PsuConfig> EnabledPsus() const;                  // enabled only, config order
    const LightConfig*    FirstEnabledLight() const;
    std::vector<MotionConfig>   EnabledMotionDevices() const;   // enabled only
    std::vector<IoDeviceConfig> EnabledIoDevices() const;       // enabled only
    std::vector<PneumaticSlideConfig> EnabledPneumaticSlides() const;

    // Write ONLY the "io_watch" section back into the config file (read-
    // modify-write; all other sections are preserved). Used by the IO Watch
    // editor window. The exe-side file is the machine's own copy — a build
    // never overwrites it (cmake/StageConfig.cmake stages missing files only).
    static bool SaveIoWatch(const std::string& path, const IoWatchConfig& cfg);

    // Write ONLY one camera's calibration + exposure back into the config file
    // (read-modify-write like SaveIoWatch; same rebuild caveat). Matches the
    // cameras[] entry by id; exposureUs <= 0 leaves that field untouched.
    static bool SaveCameraSettings(const std::string& path, const std::string& id,
                                   double pixelToMmX, double pixelToMmY,
                                   double exposureUs);

    // Write ONLY one Keithley's measurement settings back into the config file
    // (read-modify-write; same rebuild caveat). Matches instruments[] by name.
    static bool SaveKeithleySettings(const std::string& path, const std::string& name,
                                     double nplc, bool autoRange, double fixedRange,
                                     int pollIntervalMs, double biasVoltage);

    // Write ONLY one CLD101x's default setpoints back into the config file
    // (read-modify-write; same rebuild caveat). Matches instruments[] by name.
    static bool SaveCldSettings(const std::string& path, const std::string& name,
                                double defaultTemperature, double defaultLaserCurrentMA);

    // ---- Whole-entry writers (used by the FireDogRig console) ---------------
    // Same read-modify-write contract as the setting-specific savers above: an
    // unparsable file is never clobbered, entries are matched by id (devices) or
    // name (instruments), and every key these do NOT own is preserved — notably
    // io_devices[].pins, which no rig command edits. Same rebuild caveat.
    static bool SaveMotionDevice(const std::string& path, const MotionConfig& cfg);
    static bool SaveIoDevice(const std::string& path, const IoDeviceConfig& cfg);
    static bool SaveCameraDevice(const std::string& path, const CameraConfig& cfg);
    static bool SaveInstrument(const std::string& path, const KeithleyConfig& cfg);
    static bool SaveInstrument(const std::string& path, const Cld101xConfig& cfg);
    static bool SaveInstrument(const std::string& path, const PsuConfig& cfg);
    // Identity + polling only; the "channels" array is preserved (no rig
    // command edits the channel map, exactly like io_devices[].pins).
    static bool SaveInstrument(const std::string& path, const LightConfig& cfg);
};

} // namespace firedog
