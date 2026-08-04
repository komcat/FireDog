// RigReport.cpp
//
// This is a console TU, not a UI one, so including <windows.h> here does not
// touch the "Widgets.cpp is the only UI TU that includes windows.h" rule.
#include "RigReport.h"

#include "RigConfigEdit.h"

#include "hardware/HardwareConfig.h"

#include <cstddef>
#include <ctime>
#include <exception>
#include <filesystem>
#include <fstream>
#include <map>
#include <ostream>
#include <set>
#include <sstream>
#include <string>
#include <system_error>
#include <vector>

// Last, and narrowed: windows.h's min/max macros would otherwise break the
// standard headers above.
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>

namespace firedog::rig {

const char* const kDefaultReportFile = "firedog-config-report.txt";

namespace {

// Wide enough for the longest label below ("default temperature"), so every
// value column in the file starts in the same place.
constexpr std::size_t kLabelW = 22;

constexpr const char* kRule =
    "==============================================================================";
constexpr const char* kThin =
    "------------------------------------------------------------------------------";

// Column padding that never lets two columns collide — always at least two
// trailing spaces, even when the value overruns the column (the same rule
// RigConfigEdit uses, so the connection map lines up with `list`).
std::string Pad(const std::string& s, std::size_t w)
{
    return s + std::string(s.size() + 2 <= w ? w - s.size() : 2, ' ');
}

std::string Yes(bool b)     { return b ? "yes" : "no"; }
std::string State(bool on)  { return on ? "ENABLED" : "disabled"; }

// Compact numeric formatting — 0.00248 stays 0.00248, no trailing ".000000".
std::string Num(double v)
{
    std::ostringstream os;
    os << v;
    return os.str();
}

std::string OrNotSet(const std::string& s) { return s.empty() ? "(not set)" : s; }

std::string ToUpper(std::string s)
{
    for (char& ch : s) {
        if (ch >= 'a' && ch <= 'z') ch = static_cast<char>(ch - 'a' + 'A');
    }
    return s;
}

// A VISA resource names its own transport: GPIB0::1::INSTR is a GPIB card,
// TCPIP0::192.168.0.60::inst0::INSTR is the LAN, USB0::0x1313::... is USB.
// Which one is in use decides what has to be installed besides NI-VISA.
enum class Transport { Gpib, Tcpip, Usb, Serial, Unknown };

Transport TransportOf(const std::string& resource)
{
    const std::string r = ToUpper(resource);
    if (r.rfind("GPIB",  0) == 0) return Transport::Gpib;
    if (r.rfind("TCPIP", 0) == 0) return Transport::Tcpip;
    if (r.rfind("USB",   0) == 0) return Transport::Usb;
    if (r.rfind("ASRL",  0) == 0) return Transport::Serial;
    return Transport::Unknown;
}

const char* TransportName(Transport t)
{
    switch (t) {
        case Transport::Gpib:    return "GPIB (IEEE-488 card or USB-GPIB adapter)";
        case Transport::Tcpip:   return "LAN (ethernet)";
        case Transport::Usb:     return "USB";
        case Transport::Serial:  return "serial / COM port";
        case Transport::Unknown: return "unrecognised - check the resource string";
    }
    return "?";
}

// "TCPIP0::192.168.0.60::inst0::INSTR" -> "192.168.0.60"; empty for the rest.
std::string TcpipAddressOf(const std::string& resource)
{
    if (TransportOf(resource) != Transport::Tcpip) return {};
    const std::size_t a = resource.find("::");
    if (a == std::string::npos) return {};
    const std::size_t b = resource.find("::", a + 2);
    return resource.substr(a + 2, b == std::string::npos ? std::string::npos : b - a - 2);
}

// "      label                : value"
void Field(std::ostream& os, const std::string& label, const std::string& value)
{
    os << "      " << Pad(label, kLabelW) << " : " << value << "\n";
}

// A continuation line under the previous Field's value column.
void Cont(std::ostream& os, const std::string& value)
{
    os << "      " << std::string(kLabelW, ' ') << "   " << value << "\n";
}

void Heading(std::ostream& os, int n, const std::string& title)
{
    os << "\n" << kRule << "\n"
       << "  " << n << ". " << title << "\n"
       << kRule << "\n\n";
}

// "gantry-main   \"ACS gantry\"" style device header line.
void DeviceHeader(std::ostream& os, const std::string& id, const std::string& name,
                  const std::string& kind, bool enabled)
{
    os << "  " << Pad(id, 16);
    if (!name.empty() && name != id) os << Pad("\"" + name + "\"", 20);
    else                             os << Pad("", 20);
    os << Pad(kind, 18) << State(enabled) << "\n";
}

std::string Timestamp()
{
    const std::time_t now = std::time(nullptr);
    std::tm parts{};
    if (localtime_s(&parts, &now) != 0) return "(unknown)";
    char buf[32] = {};
    if (std::strftime(buf, sizeof buf, "%Y-%m-%d %H:%M:%S", &parts) == 0) return "(unknown)";
    return buf;
}

std::string HostName()
{
    char  buf[MAX_COMPUTERNAME_LENGTH + 1] = {};
    DWORD n = static_cast<DWORD>(sizeof buf);
    if (GetComputerNameA(buf, &n) == 0) return "(unknown)";
    return buf;
}

std::string Absolute(const std::string& path)
{
    try { return std::filesystem::absolute(path).string(); }
    catch (const std::exception&) { return path; }
}

// Whitespace-separated axis letters of a motion device ("X Y Z U V W").
std::vector<std::string> AxisTokens(const std::string& axes)
{
    std::vector<std::string> out;
    std::istringstream is(axes);
    std::string t;
    while (is >> t) out.push_back(t);
    return out;
}

// One 3x3 signed-permutation block read the way an operator thinks about it:
// where each GLOBAL axis ends up on this device. Entry (i,j) = sign means local
// axis i receives global axis j, so a column names the destination.
std::string BlockText(const double b[3][3], const char* const global[3],
                      const std::vector<std::string>& localNames)
{
    bool identity = true;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            if (b[i][j] != (i == j ? 1.0 : 0.0)) identity = false;
        }
    }
    if (identity) return "identity  (this device's axes already follow the global ones)";

    std::string out;
    for (int j = 0; j < 3; ++j) {
        int row  = -1;
        int hits = 0;
        for (int i = 0; i < 3; ++i) {
            if (b[i][j] != 0.0) { row = i; ++hits; }
        }

        std::string one;
        if (hits == 0) {
            one = std::string(global[j]) + " -> (not mapped)";
        } else if (hits > 1) {
            one = std::string(global[j]) + " -> (mixed - see motion_transforms.json)";
        } else {
            const std::size_t r = static_cast<std::size_t>(row);
            const std::string ln = r < localNames.size() ? localNames[r] : std::to_string(row);
            one = std::string(global[j]) + " -> " + (b[row][j] < 0.0 ? "-" : "+") + ln;
        }
        if (!out.empty()) out += "   ";
        out += one;
    }
    return out;
}

// ---- Sections --------------------------------------------------------------

void WriteHeader(std::ostream& os, const ConfigStore& store,
                 const std::string& transformsPath, std::size_t transformCount,
                 bool transformsFound)
{
    os << kRule << "\n"
       << "  FIREDOG - MACHINE CONFIGURATION REPORT\n"
       << kRule << "\n"
       << "  generated      : " << Timestamp() << "\n"
       << "  generated by   : FireDogRig  (report command)\n"
       << "  generated on   : " << HostName() << "\n"
       << "  hardware file  : " << Absolute(store.Path()) << "\n"
       << "  transforms file: " << transformsPath
       << (transformsFound ? "   (" + std::to_string(transformCount) + " entries)"
                           : "   (NOT FOUND - every device uses an identity transform)")
       << "\n"
       << kThin << "\n"
       << "  This file is a read-only report for handover. Editing it changes nothing:\n"
       << "  the machine reads config/hardware.json next to FireDog.exe. Regenerate\n"
       << "  this report at any time with:   FireDogRig --report [<file>]\n";

    if (store.AnyDirty()) {
        os << kThin << "\n"
           << "  WARNING: " << store.DirtyDevices().size()
           << " device(s) have edits made in the rig that are NOT saved to the\n"
           << "  config file yet. They are marked (*) below and this report shows the\n"
           << "  EDITED values. Run `save all` in the rig to make the file agree.\n";
    }

    os << kThin << "\n"
       << "  CONTENTS\n"
       << "     1. Summary\n"
       << "     2. Connection map (every address in one place)\n"
       << "     3. Motion controllers\n"
       << "     4. Digital IO devices and pin names\n"
       << "     5. Pneumatic slides\n"
       << "     6. IO watch (input -> output bindings)\n"
       << "     7. Safety watch\n"
       << "     8. Cameras\n"
       << "     9. Instruments\n"
       << "    10. Notes for whoever configures the machine\n";
}

void WriteSummary(std::ostream& os, const HardwareConfig& c)
{
    auto count = [](std::size_t total, std::size_t enabled) {
        return std::to_string(total) + " configured, " + std::to_string(enabled) + " enabled";
    };
    auto enabledOf = [](const auto& v) {
        std::size_t n = 0;
        for (const auto& e : v) {
            if (e.enabled) ++n;
        }
        return n;
    };

    Field(os, "motion controllers", count(c.motionDevices.size(), enabledOf(c.motionDevices)));
    Field(os, "digital IO devices", count(c.ioDevices.size(),     enabledOf(c.ioDevices)));
    Field(os, "pneumatic slides",   count(c.pneumaticSlides.size(), enabledOf(c.pneumaticSlides)));
    Field(os, "cameras",            count(c.cameras.size(),       enabledOf(c.cameras)));
    Field(os, "Keithley SMUs",      count(c.keithleys.size(),     enabledOf(c.keithleys)));
    Field(os, "laser / TEC",        count(c.clds.size(),          enabledOf(c.clds)));
    Field(os, "power supplies",     count(c.psus.size(),          enabledOf(c.psus)));
    os << "\n";
    Field(os, "io watch", c.ioWatch.enabled
              ? "ENABLED, " + std::to_string(c.ioWatch.bindings.size()) + " binding(s)"
              : "disabled (" + std::to_string(c.ioWatch.bindings.size()) + " binding(s) defined)");
    Field(os, "safety watch", c.safetyWatch.enabled
              ? "ENABLED, " + std::to_string(c.safetyWatch.watchedInputs.size()) + " watched input(s)"
              : "disabled (" + std::to_string(c.safetyWatch.watchedInputs.size())
                + " watched input(s) defined)");
    os << "\n"
       << "  A device with enabled = no is skipped entirely at startup: it is never\n"
       << "  contacted, and nothing that references it will work.\n";
}

void WriteConnectionMap(std::ostream& os, const ConfigStore& store)
{
    os << "  Everything the PC has to reach. A wrong address here is the single most\n"
       << "  common bring-up failure - check these against the switch and the labels.\n\n"
       << "    " << Pad("DEVICE", 18) << Pad("KIND", 18) << Pad("ADDRESS", 44) << "STATE\n"
       << "  " << kThin << "\n";
    for (const DeviceRef& d : store.Devices()) {
        os << store.Summary(d) << "\n";
    }
    if (store.AnyDirty()) os << "\n    * = edited in the rig, not yet saved to the config file\n";
}

void WriteMotion(std::ostream& os, const HardwareConfig& c,
                 const std::map<std::string, AxisTransform>& transforms)
{
    if (c.motionDevices.empty()) { os << "  (none configured)\n"; return; }

    static const char* const kGlobalXyz[3] = {"X", "Y", "Z"};
    static const char* const kGlobalUvw[3] = {"U", "V", "W"};

    std::set<std::string> used;

    for (const MotionConfig& m : c.motionDevices) {
        DeviceHeader(os, m.id, m.name, m.controllerType + " controller", m.enabled);

        const std::vector<std::string> ax = AxisTokens(m.axes);
        Field(os, "controller type", m.controllerType);
        Field(os, "axes", OrNotSet(m.axes) + "   (" + std::to_string(ax.size()) + " axes)");
        Field(os, "address", OrNotSet(m.ipAddress) + " : " + std::to_string(m.port));
        Field(os, "device id", std::to_string(m.deviceId));

        // Transforms are keyed by the device NAME (that is what Workflow looks
        // up); accept the id as a fallback so a report never claims "identity"
        // for an entry that is really there.
        auto it = transforms.find(m.name);
        if (it == transforms.end()) it = transforms.find(m.id);

        if (it == transforms.end()) {
            Field(os, "axis transform", "no entry in motion_transforms.json - identity assumed");
        } else {
            used.insert(it->first);
            const std::ptrdiff_t take = static_cast<std::ptrdiff_t>(ax.size() < 3 ? ax.size() : 3);
            std::vector<std::string> trans(ax.begin(), ax.begin() + take);
            std::vector<std::string> rot;
            if (ax.size() > 3) rot.assign(ax.begin() + 3, ax.end());
            if (trans.empty()) trans = {"X", "Y", "Z"};
            if (rot.empty())   rot   = {"U", "V", "W"};

            Field(os, "axis transform", "translation:  "
                  + BlockText(it->second.m, kGlobalXyz, trans));
            Cont(os, "rotation:     " + (ax.size() > 3
                      ? BlockText(it->second.r, kGlobalUvw, rot)
                      : std::string("n/a - this controller has no rotational axes")));
            Cont(os, "(read as: a global jog in that direction moves this axis)");
        }
        os << "\n";
    }

    if (transforms.size() > used.size()) {
        os << "  motion_transforms.json also carries entries for devices that are not in\n"
              "  this hardware config (kept for other machines, ignored here):\n     ";
        bool first = true;
        for (const auto& entry : transforms) {
            if (used.count(entry.first) != 0) continue;
            if (!first) os << ", ";
            os << entry.first;
            first = false;
        }
        os << "\n";
    }
}

void WritePinTable(std::ostream& os, const char* what, int count,
                   const std::map<int, std::string>& names)
{
    os << "      " << what << "  (" << count << " pins)\n";
    if (count <= 0 && names.empty()) {
        os << "         (none)\n";
        return;
    }
    for (int pin = 0; pin < count; ++pin) {
        const auto it = names.find(pin);
        os << "         pin " << Pad(std::to_string(pin), 4)
           << (it == names.end() ? "-  (no name in the config)" : it->second) << "\n";
    }
    // A name pointing past the declared pin count never resolves at run time —
    // it is worth seeing in the handover rather than at first use.
    for (const auto& [pin, name] : names) {
        if (pin >= count) {
            os << "         pin " << Pad(std::to_string(pin), 4) << name
               << "   << WARNING: beyond the declared pin count\n";
        }
    }
}

void WriteIo(std::ostream& os, const HardwareConfig& c)
{
    if (c.ioDevices.empty()) { os << "  (none configured)\n"; return; }

    os << "  Pin numbering is normalized by the driver: bit i == pin i, for inputs and\n"
       << "  outputs alike. The names below are what recipes, slides and the watches\n"
       << "  refer to - renaming a pin here renames it everywhere.\n\n";

    for (const IoDeviceConfig& d : c.ioDevices) {
        DeviceHeader(os, d.id, d.name, d.type + " IO", d.enabled);
        Field(os, "type", d.type + (d.type == "eziio" ? "   (FASTECH Ezi-IO)"
                                                      : "   (WAGO Modbus TCP)"));
        if (d.type == "eziio") {
            Field(os, "address", OrNotSet(d.ipAddress));
            Field(os, "board id", std::to_string(d.deviceId) + "   (must be unique per board)");
        } else {
            Field(os, "address", OrNotSet(d.ipAddress) + " : " + std::to_string(d.port));
            Field(os, "simulated", Yes(d.simulated) + (d.simulated
                      ? "   (in-memory only - NO sockets, NO real IO)" : ""));
        }
        Field(os, "poll interval", std::to_string(d.pollIntervalMs) + " ms");
        os << "\n";
        WritePinTable(os, "Inputs ", d.inputCount,  d.inputNames);
        os << "\n";
        WritePinTable(os, "Outputs", d.outputCount, d.outputNames);
        os << "\n";
    }
}

void WriteSlides(std::ostream& os, const HardwareConfig& c)
{
    if (c.pneumaticSlides.empty()) { os << "  (none configured)\n"; return; }

    os << "  A slide is not a device on the network: it is one output pin plus two\n"
       << "  sensor pins, on the IO devices above, referenced BY NAME.\n\n";

    for (const PneumaticSlideConfig& s : c.pneumaticSlides) {
        DeviceHeader(os, s.name, "", "pneumatic slide", s.enabled);
        Field(os, "output (extend)",   s.outputDevice    + " . " + s.outputPin);
        Field(os, "extended sensor",   s.extendedDevice  + " . " + s.extendedPin);
        Field(os, "retracted sensor",  s.retractedDevice + " . " + s.retractedPin);
        Field(os, "timeout",           std::to_string(s.timeoutMs) + " ms");
        os << "\n";
    }
    os << "  Output ON extends, OFF retracts; the move is only reported complete when\n"
       << "  the matching sensor reads true within the timeout.\n";
}

void WriteIoWatch(std::ostream& os, const HardwareConfig& c)
{
    Field(os, "io watch", State(c.ioWatch.enabled));
    Field(os, "polling interval", std::to_string(c.ioWatch.pollingIntervalMs) + " ms");
    os << "\n";

    if (c.ioWatch.bindings.empty()) { os << "  (no bindings defined)\n"; return; }

    os << "  An input edge drives an output, with no software above it involved.\n\n"
       << "    " << Pad("BINDING", 26) << Pad("INPUT", 26) << Pad("OUTPUT", 26)
       << Pad("TRIGGER", 9) << Pad("BEHAVIOR", 11) << "DEBOUNCE   STATE\n"
       << "  " << kThin << "\n";
    for (const IoWatchBindingConfig& b : c.ioWatch.bindings) {
        os << "    " << Pad(b.name, 26)
           << Pad(b.inputDevice  + "." + b.inputPin,  26)
           << Pad(b.outputDevice + "." + b.outputPin, 26)
           << Pad(b.trigger, 9) << Pad(b.behavior, 11)
           << Pad(std::to_string(b.debounceMs) + " ms", 11)
           << State(b.enabled) << "\n";
    }
}

void WriteSafety(std::ostream& os, const HardwareConfig& c)
{
    Field(os, "safety watch", State(c.safetyWatch.enabled));
    Field(os, "polling interval", std::to_string(c.safetyWatch.pollingIntervalMs) + " ms");
    os << "\n";

    if (c.safetyWatch.watchedInputs.empty()) { os << "  (no watched inputs defined)\n"; return; }

    for (const SafetySignalConfig& s : c.safetyWatch.watchedInputs) {
        os << "  " << Pad(s.device + " . " + OrNotSet(s.pinName), 34)
           << (s.critical ? "CRITICAL" : "alert   ") << "   " << State(s.enabled) << "\n";
        Field(os, "pin", OrNotSet(s.pinName) + "   (falls back to pin number "
                  + std::to_string(s.pinNumber) + " if the name does not resolve)");
        Field(os, "trigger", s.trigger);
        Field(os, "on trigger", s.critical
                  ? "STOP ALL MOTION and raise the alert below"
                  : "raise the alert below (motion is not stopped)");
        Field(os, "alert title", s.alertTitle);
        Field(os, "alert message", s.alertMessage);
        os << "\n";
    }
}

void WriteCameras(std::ostream& os, const HardwareConfig& c)
{
    if (c.cameras.empty()) { os << "  (none configured)\n"; return; }

    for (const CameraConfig& cam : c.cameras) {
        DeviceHeader(os, cam.id, cam.displayName, cam.cameraType + " camera", cam.enabled);
        Field(os, "camera type", cam.cameraType);
        // connection_type decides WHICH of the three identifiers below is used;
        // the other two are carried in the file and ignored.
        std::string how = "the first camera found";
        if      (cam.connectionType == "ip")     how = "found by ip address";
        else if (cam.connectionType == "serial") how = "found by serial number";
        else if (cam.connectionType == "index")  how = "found by device index";
        Field(os, "connection type", cam.connectionType + "   (" + how + ")");
        Field(os, "ip address", OrNotSet(cam.ipAddress));
        Field(os, "serial number", OrNotSet(cam.serialNumber));
        Field(os, "device index", std::to_string(cam.deviceIndex));
        Field(os, "exposure", cam.exposureUs > 0.0
                  ? Num(cam.exposureUs) + " us   (applied after connect)"
                  : "camera default   (nothing applied after connect)");
        Field(os, "calibration", Num(cam.pixelToMmX) + " mm/pixel in X, "
                  + Num(cam.pixelToMmY) + " mm/pixel in Y");
        Field(os, "auto connect", Yes(cam.autoConnect));
        os << "\n";
    }
    os << "  The calibration is what turns a pixel offset into a move, so it belongs to\n"
       << "  the lens and working distance actually fitted - re-measure it if either\n"
       << "  changes.\n";
}

void WriteInstruments(std::ostream& os, const HardwareConfig& c)
{
    if (c.keithleys.empty() && c.clds.empty() && c.psus.empty()) {
        os << "  (none configured)\n";
        return;
    }

    os << "  Every instrument is reached over VISA, so NI-VISA has to be installed on\n"
       << "  this PC first - section 10, note 4 has the download link - and each\n"
       << "  resource string below must match what NI-MAX lists, exactly. A USB\n"
       << "  resource embeds the unit's serial number, so swapping the instrument for\n"
       << "  another one changes the string.\n\n";

    for (const KeithleyConfig& k : c.keithleys) {
        DeviceHeader(os, k.name, "", k.type, k.enabled);
        Field(os, "VISA resource", OrNotSet(k.visaResource));
        Field(os, "connected by", TransportName(TransportOf(k.visaResource)));
        Field(os, "channels", std::to_string(k.channelCount));
        std::string ch;
        for (const int n : k.enabledChannels) {
            if (!ch.empty()) ch += ", ";
            ch += std::to_string(n);
        }
        Field(os, "channels in use", ch.empty() ? "(none)" : ch);
        Field(os, "measurement NPLC", Num(k.nplc) + "   (this sets the sample rate, not the poll interval)");
        Field(os, "ranging", k.autoRange ? "auto" : "fixed at " + Num(k.fixedRange) + " A");
        Field(os, "bias voltage", Num(k.biasVoltage) + " V   (applied to channel 1 after connect)");
        Field(os, "poll interval", std::to_string(k.pollIntervalMs) + " ms"
                  + (k.pollIntervalMs == 0 ? "   (free-run - read as fast as the NPLC allows)" : ""));
        os << "\n";
    }

    for (const Cld101xConfig& l : c.clds) {
        DeviceHeader(os, l.name, "", l.type + " laser / TEC", l.enabled);
        Field(os, "VISA resource", OrNotSet(l.visaResource));
        Field(os, "connected by", TransportName(TransportOf(l.visaResource)));
        Field(os, "default temperature", Num(l.defaultTemperature) + " degC");
        Field(os, "default current", Num(l.defaultLaserCurrentMA) + " mA");
        Field(os, "poll interval", std::to_string(l.pollIntervalMs) + " ms");
        os << "\n"
           << "      the two defaults seed the operator's screen only - neither the laser\n"
           << "      nor the TEC is switched on or written at connect.\n\n";
    }

    for (const PsuConfig& p : c.psus) {
        DeviceHeader(os, p.name, "", p.type + " power supply", p.enabled);
        Field(os, "VISA resource", OrNotSet(p.visaResource));
        Field(os, "connected by", TransportName(TransportOf(p.visaResource)));
        Field(os, "channels", std::to_string(p.channels));
        Field(os, "poll interval", std::to_string(p.pollIntervalMs) + " ms");
        os << "      setpoints are read back from the supply at connect - the config\n"
           << "      carries no voltages or currents.\n\n";
    }
}

void WriteNotes(std::ostream& os, const HardwareConfig& c)
{
    // Distinct subnets actually referenced, so the note about NICs is concrete.
    std::set<std::string> subnets;
    auto addSubnet = [&subnets](const std::string& ip) {
        if (ip.empty()) return;
        const std::size_t last = ip.rfind('.');
        if (last != std::string::npos) subnets.insert(ip.substr(0, last) + ".x");
    };
    for (const MotionConfig& m : c.motionDevices)  { if (m.enabled) addSubnet(m.ipAddress); }
    for (const IoDeviceConfig& d : c.ioDevices)    { if (d.enabled) addSubnet(d.ipAddress); }
    for (const CameraConfig& cam : c.cameras)      { if (cam.enabled) addSubnet(cam.ipAddress); }
    // An instrument on LAN is on the network too — its ip is inside the VISA
    // resource string, which is the one place it is easy to miss.
    for (const KeithleyConfig& k : c.keithleys) { if (k.enabled) addSubnet(TcpipAddressOf(k.visaResource)); }
    for (const Cld101xConfig& l : c.clds)       { if (l.enabled) addSubnet(TcpipAddressOf(l.visaResource)); }
    for (const PsuConfig& p : c.psus)           { if (p.enabled) addSubnet(TcpipAddressOf(p.visaResource)); }

    os << "  1. Where the file lives\n"
       << "     The machine reads config/hardware.json in the folder that holds\n"
       << "     FireDog.exe. A rebuild recopies the repository's config/ over that\n"
       << "     copy, so a durable change must go into the repository file too.\n\n"
       << "  2. How to change a value\n"
       << "     Either edit the JSON directly, or use the rig without the GUI:\n"
       << "         FireDogRig --set hex-left.ip_address=192.168.0.11 --save\n"
       << "         FireDogRig --probe all       (connect and read every device back)\n"
       << "     Pin names, slides, io_watch and safety_watch are edited in the JSON or\n"
       << "     in the client - the rig deliberately does not touch them.\n\n"
       << "  3. Networking\n";
    if (subnets.empty()) {
        os << "     No enabled device has an IP address in this config.\n\n";
    } else {
        os << "     Enabled devices sit on: ";
        bool first = true;
        for (const std::string& s : subnets) {
            if (!first) os << ", ";
            os << s;
            first = false;
        }
        os << "\n"
           << "     The PC needs an adapter on each of those, and every address must be\n"
           << "     static. A camera on a different subnet from its adapter is found but\n"
           << "     never opens.\n\n";
    }
    // What has to be installed beyond NI-VISA depends on the transports this
    // config actually uses, so read them off the resource strings rather than
    // telling a LAN-only machine to install a GPIB driver.
    bool anyGpib = false, anyTcpip = false, anyUsb = false, anySerial = false;
    auto classify = [&](const std::string& res) {
        switch (TransportOf(res)) {
            case Transport::Gpib:   anyGpib   = true; break;
            case Transport::Tcpip:  anyTcpip  = true; break;
            case Transport::Usb:    anyUsb    = true; break;
            case Transport::Serial: anySerial = true; break;
            case Transport::Unknown: break;
        }
    };
    for (const KeithleyConfig& k : c.keithleys) classify(k.visaResource);
    for (const Cld101xConfig& l : c.clds)       classify(l.visaResource);
    for (const PsuConfig& p : c.psus)           classify(p.visaResource);

    os << "  4. Instruments - NI-VISA must be installed on this PC\n";
    if (c.keithleys.empty() && c.clds.empty() && c.psus.empty()) {
        os << "     No VISA instrument is configured on this machine, so NI-VISA is not\n"
              "     needed unless one is added later.\n\n";
    } else {
        os << "     FireDog opens every instrument in section 9 through the VISA library\n"
              "     (viOpen on the resource string). That is true whatever the cable is:\n"
              "     GPIB, LAN and USB are three transports of the SAME library, chosen by\n"
              "     the resource string. So NI-VISA is required even on a LAN-only\n"
              "     machine - what LAN saves you is the GPIB driver and its adapter, not\n"
              "     NI-VISA itself. Without it no instrument opens at all.\n"
              "\n"
              "     Download (free, no licence needed):\n"
              "         https://www.ni.com/en/support/downloads/drivers/download.ni-visa.html\n"
              "     If that link has moved: ni.com -> Support -> Downloads -> Drivers ->\n"
              "     NI-VISA. Take the latest Windows release and install the 64-bit\n"
              "     runtime; FireDog is a 64-bit application.\n"
              "     Expected install location: C:\\Program Files\\IVI Foundation\\VISA\\Win64\n"
              "\n"
              "     The installer also puts in NI-MAX (Measurement & Automation Explorer),\n"
              "     which is how you check the wiring: every resource string in section 9\n"
              "     must appear there, spelled exactly the same. \"Open VISA Test Panel\"\n"
              "     -> *IDN? is the fastest proof that an instrument answers.\n"
              "\n";

        os << "     This machine's instruments use:\n";
        if (anyGpib) {
            os << "       - GPIB (GPIB0::...). This ALSO needs NI-488.2 installed, for the\n"
                  "         GPIB controller card or USB-GPIB adapter itself, and each\n"
                  "         instrument's front-panel GPIB address must match the number in\n"
                  "         its resource string.\n";
        }
        if (anyTcpip) {
            os << "       - LAN (TCPIP0::...). No GPIB hardware and no NI-488.2 needed.\n"
                  "         Give the instrument a STATIC ip on a subnet the PC has an\n"
                  "         adapter on (see note 3), then add it in NI-MAX under Network\n"
                  "         Devices -> Add Network Device -> VISA TCP/IP Resource. The\n"
                  "         instrument's own LAN page must agree with the ip in the\n"
                  "         resource string, and a ::5025::SOCKET style string talks raw\n"
                  "         SCPI where ::inst0::INSTR uses VXI-11/HiSLIP - they are not\n"
                  "         interchangeable.\n";
        }
        if (anyUsb) {
            os << "       - USB (USB0::...). The string carries the unit's serial number,\n"
                  "         so a replacement instrument needs the string in the config\n"
                  "         updated - it will not be found otherwise.\n";
        }
        if (anySerial) {
            os << "       - serial (ASRL...). The COM port number is assigned by Windows\n"
                  "         and can move when the adapter is plugged into another socket.\n";
        }
        if (!anyGpib) {
            os << "       - no GPIB instrument is configured, so NI-488.2 and a GPIB\n"
                  "         adapter are NOT needed on this machine.\n";
        }
        os << "\n";
    }
    os << "  5. Order of bring-up\n"
       << "     Addresses first, then `FireDogRig --probe all`: it runs the client's own\n"
       << "     connect sequence and reports what each device answers. If the rig\n"
       << "     connects, the client will connect. The rig never moves an axis, writes\n"
       << "     an output or switches a supply, so it is safe against a live machine.\n";
}

} // namespace

bool WriteConfigReport(const ConfigStore& store, const std::string& outPath, std::string& err)
{
    const HardwareConfig& c = store.Config();

    // The transforms live beside the hardware file, so an explicit --config
    // pointed at the repository copy reports that copy's transforms.
    std::string transformsPath = "config/motion_transforms.json";
    try {
        const std::filesystem::path dir = std::filesystem::path(store.Path()).parent_path();
        transformsPath = (dir / "motion_transforms.json").string();
    } catch (const std::exception&) {
        // Keep the default; a bad config path is already reported elsewhere.
    }
    std::error_code ec;
    const bool transformsFound = std::filesystem::exists(transformsPath, ec) && !ec;
    const std::map<std::string, AxisTransform> transforms = LoadMotionTransforms(transformsPath);

    // Text mode on purpose: CRLF, so the file opens correctly in Notepad.
    std::ofstream os(outPath);
    if (!os) {
        err = "could not write '" + Absolute(outPath) + "' (is it open in another program?)";
        return false;
    }

    WriteHeader(os, store, Absolute(transformsPath), transforms.size(), transformsFound);

    Heading(os,  1, "SUMMARY");                       WriteSummary(os, c);
    Heading(os,  2, "CONNECTION MAP");                WriteConnectionMap(os, store);
    Heading(os,  3, "MOTION CONTROLLERS");            WriteMotion(os, c, transforms);
    Heading(os,  4, "DIGITAL IO DEVICES AND PINS");   WriteIo(os, c);
    Heading(os,  5, "PNEUMATIC SLIDES");              WriteSlides(os, c);
    Heading(os,  6, "IO WATCH");                      WriteIoWatch(os, c);
    Heading(os,  7, "SAFETY WATCH");                  WriteSafety(os, c);
    Heading(os,  8, "CAMERAS");                       WriteCameras(os, c);
    Heading(os,  9, "INSTRUMENTS");                   WriteInstruments(os, c);
    Heading(os, 10, "NOTES FOR WHOEVER CONFIGURES THE MACHINE");  WriteNotes(os, c);

    os << "\n" << kRule << "\n"
       << "  end of report - " << Absolute(store.Path()) << "\n"
       << kRule << "\n";

    if (!os) {
        err = "writing '" + Absolute(outPath) + "' failed part-way (disk full?)";
        return false;
    }
    return true;
}

} // namespace firedog::rig
