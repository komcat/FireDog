// HardwareConfig.cpp
#include "hardware/HardwareConfig.h"

#include "core/Paths.h"

#include <nlohmann/json.hpp>

#include <fstream>
#include <iostream>

namespace firedog {

HardwareConfig HardwareConfig::Load(const std::string& path)
{
    HardwareConfig hc;

    std::ifstream f(path);
    if (!f) {
        std::cerr << "[config] hardware config not found: " << path << std::endl;
        return hc;
    }

    try {
        nlohmann::json j;
        f >> j;

        if (j.contains("cameras")) {
            for (const auto& c : j["cameras"]) {
                CameraConfig cc;
                cc.id             = c.value("id", std::string());
                cc.cameraType     = c.value("camera_type", std::string("PYLON"));
                cc.connectionType = c.value("connection_type", std::string("ip"));
                cc.ipAddress      = c.value("ip_address", std::string());
                cc.serialNumber   = c.value("serial_number", std::string());
                cc.displayName    = c.value("display_name", cc.id);
                cc.enabled        = c.value("enabled", true);
                cc.autoConnect    = c.value("auto_connect", true);
                cc.deviceIndex    = c.value("device_index", 0);
                cc.pixelToMmX     = c.value("pixel_to_mm_x", 0.00248);
                cc.pixelToMmY     = c.value("pixel_to_mm_y", 0.00248);
                cc.exposureUs     = c.value("exposure_us", 0.0);
                hc.cameras.push_back(cc);
            }
        }

        if (j.contains("motion_devices")) {
            for (const auto& m : j["motion_devices"]) {
                MotionConfig mc;
                mc.id             = m.value("id", std::string());
                mc.name           = m.value("name", mc.id);
                mc.controllerType = m.value("controller_type", std::string());
                mc.axes           = m.value("axes", std::string());
                mc.ipAddress      = m.value("ip_address", std::string());
                mc.port           = m.value("port", 0);
                mc.deviceId       = m.value("device_id", 0);
                mc.enabled        = m.value("enabled", true);
                hc.motionDevices.push_back(mc);
            }
        }

        if (j.contains("io_devices")) {
            for (const auto& d : j["io_devices"]) {
                IoDeviceConfig ic;
                ic.id             = d.value("id", std::string());
                ic.name           = d.value("name", ic.id);
                ic.type           = d.value("type", std::string());
                ic.ipAddress      = d.value("ip_address", std::string());
                ic.port           = d.value("port", 502);
                ic.deviceId       = d.value("device_id", 0);
                ic.inputCount     = d.value("input_count", 8);
                ic.outputCount    = d.value("output_count", 8);
                ic.pollIntervalMs = d.value("poll_interval_ms", 100);
                ic.enabled        = d.value("enabled", true);
                ic.simulated      = d.value("simulated", false);
                if (d.contains("pins")) {
                    const auto& p = d["pins"];
                    if (p.contains("inputs")) {
                        for (const auto& e : p["inputs"]) {
                            const int pin = e.value("pin", -1);
                            if (pin >= 0) ic.inputNames[pin] = e.value("name", std::string());
                        }
                    }
                    if (p.contains("outputs")) {
                        for (const auto& e : p["outputs"]) {
                            const int pin = e.value("pin", -1);
                            if (pin >= 0) ic.outputNames[pin] = e.value("name", std::string());
                        }
                    }
                }
                hc.ioDevices.push_back(ic);
            }
        }

        if (j.contains("pneumatic_slides")) {
            // Helper: read {"device": "...", "pin": "..."} into two strings.
            auto pinRef = [](const nlohmann::json& e, const char* key,
                             std::string& dev, std::string& pin) {
                if (!e.contains(key)) return;
                dev = e[key].value("device", std::string());
                pin = e[key].value("pin", std::string());
            };
            for (const auto& s : j["pneumatic_slides"]) {
                PneumaticSlideConfig pc;
                pc.name      = s.value("name", std::string());
                pc.timeoutMs = s.value("timeout_ms", 10000);
                pc.enabled   = s.value("enabled", true);
                pinRef(s, "output",          pc.outputDevice,    pc.outputPin);
                pinRef(s, "extended_input",  pc.extendedDevice,  pc.extendedPin);
                pinRef(s, "retracted_input", pc.retractedDevice, pc.retractedPin);
                hc.pneumaticSlides.push_back(pc);
            }
        }

        if (j.contains("io_watch")) {
            const auto& w = j["io_watch"];
            hc.ioWatch.enabled           = w.value("enabled", false);
            hc.ioWatch.pollingIntervalMs = w.value("polling_interval_ms", 50);
            if (w.contains("bindings")) {
                for (const auto& b : w["bindings"]) {
                    IoWatchBindingConfig bc;
                    bc.name       = b.value("name", std::string());
                    bc.trigger    = b.value("trigger", std::string("rising"));
                    bc.behavior   = b.value("behavior", std::string("toggle"));
                    bc.debounceMs = b.value("debounce_ms", 100);
                    bc.enabled    = b.value("enabled", true);
                    if (b.contains("input")) {
                        bc.inputDevice = b["input"].value("device", std::string());
                        bc.inputPin    = b["input"].value("pin_name", std::string());
                    }
                    if (b.contains("output")) {
                        bc.outputDevice = b["output"].value("device", std::string());
                        bc.outputPin    = b["output"].value("pin_name", std::string());
                    }
                    if (b.contains("indicator")) {
                        bc.indicatorDevice = b["indicator"].value("device", std::string());
                        bc.indicatorPin    = b["indicator"].value("pin_name", std::string());
                    }
                    hc.ioWatch.bindings.push_back(bc);
                }
            }
        }

        if (j.contains("safety_watch")) {
            const auto& w = j["safety_watch"];
            hc.safetyWatch.enabled           = w.value("enabled", false);
            hc.safetyWatch.pollingIntervalMs = w.value("polling_interval_ms", 100);
            if (w.contains("watched_inputs")) {
                for (const auto& s : w["watched_inputs"]) {
                    SafetySignalConfig sc;
                    sc.device       = s.value("device", std::string());
                    sc.pinName      = s.value("pin_name", std::string());
                    sc.pinNumber    = s.value("pin_number", 0);
                    sc.trigger      = s.value("trigger", std::string("rising"));
                    sc.alertTitle   = s.value("alert_title", std::string("Safety Alert"));
                    sc.alertMessage = s.value("alert_message",
                                              std::string("A safety condition was triggered."));
                    sc.critical     = s.value("critical", false);
                    sc.enabled      = s.value("enabled", true);
                    hc.safetyWatch.watchedInputs.push_back(sc);
                }
            }
        }

        if (j.contains("instruments")) {
            for (const auto& inst : j["instruments"]) {
                const std::string type = inst.value("type", std::string());

                if (type.find("cld") != std::string::npos) {
                    Cld101xConfig cc;
                    cc.name         = inst.value("name", std::string("laser-main"));
                    cc.type         = type;
                    cc.visaResource = inst.value("visa_resource", std::string());
                    cc.enabled      = inst.value("enabled", true);
                    cc.defaultTemperature    = inst.value("default_temperature", 25.0);
                    cc.defaultLaserCurrentMA = inst.value("default_laser_current", 50.0);
                    cc.pollIntervalMs        = inst.value("poll_interval_ms", 500);
                    hc.clds.push_back(cc);
                    continue;
                }
                if (type.find("udp3305") != std::string::npos) {
                    PsuConfig pc;
                    pc.name         = inst.value("name", std::string("PSU3"));
                    pc.type         = type;
                    pc.visaResource = inst.value("visa_resource", std::string());
                    pc.enabled      = inst.value("enabled", true);
                    pc.channels     = inst.value("channels", 3);
                    pc.pollIntervalMs = inst.value("poll_interval_ms", 500);
                    hc.psus.push_back(pc);
                    continue;
                }
                if (type.find("pd3") != std::string::npos) {
                    LightConfig lc;
                    lc.name           = inst.value("name", std::string("LIGHT1"));
                    lc.type           = type;
                    lc.ipAddress      = inst.value("ip_address", std::string());
                    lc.port           = inst.value("port", 40001);
                    lc.enabled        = inst.value("enabled", true);
                    lc.simulated      = inst.value("simulated", false);
                    lc.pollIntervalMs = inst.value("poll_interval_ms", 1000);
                    if (inst.contains("channels") && inst["channels"].is_array()) {
                        for (const auto& c : inst["channels"]) {
                            LightChannelConfig ch;
                            ch.channel = c.value("channel", 0);
                            ch.name    = c.value("name", "L" + std::to_string(ch.channel));
                            ch.color   = c.value("color", std::string());
                            ch.group   = c.value("group", std::string());
                            ch.camera  = c.value("camera", std::string());
                            ch.enabled = c.value("enabled", true);
                            lc.channels.push_back(ch);
                        }
                    }
                    hc.lights.push_back(lc);
                    continue;
                }
                if (type.find("keithley") == std::string::npos) continue;

                KeithleyConfig kc;
                kc.name         = inst.value("name", std::string("Keithley_6482"));
                kc.type         = type;
                kc.visaResource = inst.value("visa_resource", std::string("GPIB0::1::INSTR"));
                kc.enabled      = inst.value("enabled", true);
                kc.channelCount = inst.value("channel_count", 2);
                if (inst.contains("enabled_channels")) {
                    kc.enabledChannels = inst["enabled_channels"].get<std::vector<int>>();
                }
                kc.nplc           = inst.value("measurement_nplc", 5.0);
                kc.autoRange      = inst.value("auto_range", true);
                kc.fixedRange     = inst.value("fixed_range", 0.0);
                kc.pollIntervalMs = inst.value("poll_interval_ms", 100);
                kc.biasVoltage    = inst.value("bias_voltage", 1.0);
                hc.keithleys.push_back(kc);
            }
        }
    }
    catch (const std::exception& e) {
        std::cerr << "[config] parse error: " << e.what() << std::endl;
    }

    return hc;
}

const CameraConfig* HardwareConfig::FirstEnabledCamera() const
{
    for (const auto& c : cameras) {
        if (c.enabled) return &c;
    }
    return nullptr;
}

const KeithleyConfig* HardwareConfig::FirstEnabledKeithley() const
{
    for (const auto& k : keithleys) {
        if (k.enabled) return &k;
    }
    return nullptr;
}

const Cld101xConfig* HardwareConfig::FirstEnabledCld() const
{
    for (const auto& c : clds) {
        if (c.enabled) return &c;
    }
    return nullptr;
}

std::vector<PsuConfig> HardwareConfig::EnabledPsus() const
{
    std::vector<PsuConfig> out;
    for (const auto& p : psus) {
        if (p.enabled) out.push_back(p);
    }
    return out;
}

const LightConfig* HardwareConfig::FirstEnabledLight() const
{
    for (const auto& l : lights) {
        if (l.enabled) return &l;
    }
    return nullptr;
}

std::vector<MotionConfig> HardwareConfig::EnabledMotionDevices() const
{
    std::vector<MotionConfig> out;
    for (const auto& m : motionDevices) {
        if (m.enabled) out.push_back(m);
    }
    return out;
}

std::vector<IoDeviceConfig> HardwareConfig::EnabledIoDevices() const
{
    std::vector<IoDeviceConfig> out;
    for (const auto& d : ioDevices) {
        if (d.enabled) out.push_back(d);
    }
    return out;
}

std::vector<PneumaticSlideConfig> HardwareConfig::EnabledPneumaticSlides() const
{
    std::vector<PneumaticSlideConfig> out;
    for (const auto& p : pneumaticSlides) {
        if (p.enabled) out.push_back(p);
    }
    return out;
}

bool HardwareConfig::SaveCameraSettings(const std::string& path, const std::string& id,
                                        double pixelToMmX, double pixelToMmY,
                                        double exposureUs)
{
    nlohmann::json j;
    {
        std::ifstream f(path);
        if (f) {
            try { f >> j; }
            catch (const std::exception& e) {
                std::cerr << "[config] cannot rewrite " << path << ": " << e.what() << std::endl;
                return false;   // don't clobber a file we couldn't parse
            }
        }
    }

    bool found = false;
    if (j.contains("cameras")) {
        for (auto& c : j["cameras"]) {
            if (c.value("id", std::string()) != id) continue;
            c["pixel_to_mm_x"] = pixelToMmX;
            c["pixel_to_mm_y"] = pixelToMmY;
            if (exposureUs > 0.0) c["exposure_us"] = exposureUs;
            found = true;
            break;
        }
    }
    if (!found) {
        std::cerr << "[config] camera id '" << id << "' not found in " << path << std::endl;
        return false;
    }

    std::ofstream out(path);
    if (!out) {
        std::cerr << "[config] cannot write " << path << std::endl;
        return false;
    }
    out << j.dump(2) << "\n";
    return true;
}

bool HardwareConfig::SaveKeithleySettings(const std::string& path, const std::string& name,
                                          double nplc, bool autoRange, double fixedRange,
                                          int pollIntervalMs, double biasVoltage)
{
    nlohmann::json j;
    {
        std::ifstream f(path);
        if (f) {
            try { f >> j; }
            catch (const std::exception& e) {
                std::cerr << "[config] cannot rewrite " << path << ": " << e.what() << std::endl;
                return false;   // don't clobber a file we couldn't parse
            }
        }
    }

    bool found = false;
    if (j.contains("instruments")) {
        for (auto& inst : j["instruments"]) {
            if (inst.value("name", std::string()) != name) continue;
            inst["measurement_nplc"] = nplc;
            inst["auto_range"]       = autoRange;
            inst["fixed_range"]      = fixedRange;
            inst["poll_interval_ms"] = pollIntervalMs;
            inst["bias_voltage"]     = biasVoltage;
            found = true;
            break;
        }
    }
    if (!found) {
        std::cerr << "[config] instrument '" << name << "' not found in " << path << std::endl;
        return false;
    }

    std::ofstream out(path);
    if (!out) {
        std::cerr << "[config] cannot write " << path << std::endl;
        return false;
    }
    out << j.dump(2) << "\n";
    return true;
}

bool HardwareConfig::SaveCldSettings(const std::string& path, const std::string& name,
                                     double defaultTemperature, double defaultLaserCurrentMA)
{
    nlohmann::json j;
    {
        std::ifstream f(path);
        if (f) {
            try { f >> j; }
            catch (const std::exception& e) {
                std::cerr << "[config] cannot rewrite " << path << ": " << e.what() << std::endl;
                return false;   // don't clobber a file we couldn't parse
            }
        }
    }

    bool found = false;
    if (j.contains("instruments")) {
        for (auto& inst : j["instruments"]) {
            if (inst.value("name", std::string()) != name) continue;
            inst["default_temperature"]   = defaultTemperature;
            inst["default_laser_current"] = defaultLaserCurrentMA;
            found = true;
            break;
        }
    }
    if (!found) {
        std::cerr << "[config] instrument '" << name << "' not found in " << path << std::endl;
        return false;
    }

    std::ofstream out(path);
    if (!out) {
        std::cerr << "[config] cannot write " << path << std::endl;
        return false;
    }
    out << j.dump(2) << "\n";
    return true;
}

bool HardwareConfig::SaveIoWatch(const std::string& path, const IoWatchConfig& cfg)
{
    nlohmann::json j;
    {
        std::ifstream f(path);
        if (f) {
            try { f >> j; }
            catch (const std::exception& e) {
                std::cerr << "[config] cannot rewrite " << path << ": " << e.what() << std::endl;
                return false;   // don't clobber a file we couldn't parse
            }
        }
    }

    nlohmann::json w;
    w["enabled"]             = cfg.enabled;
    w["polling_interval_ms"] = cfg.pollingIntervalMs;
    w["bindings"]            = nlohmann::json::array();
    for (const auto& b : cfg.bindings) {
        nlohmann::json e;
        e["name"]        = b.name;
        e["input"]       = {{"device", b.inputDevice}, {"pin_name", b.inputPin}};
        e["output"]      = {{"device", b.outputDevice}, {"pin_name", b.outputPin}};
        if (!b.indicatorDevice.empty() || !b.indicatorPin.empty())
            e["indicator"] = {{"device", b.indicatorDevice}, {"pin_name", b.indicatorPin}};
        e["trigger"]     = b.trigger;
        e["behavior"]    = b.behavior;
        e["debounce_ms"] = b.debounceMs;
        e["enabled"]     = b.enabled;
        w["bindings"].push_back(e);
    }
    j["io_watch"] = w;

    std::ofstream out(path);
    if (!out) {
        std::cerr << "[config] cannot write " << path << std::endl;
        return false;
    }
    out << j.dump(2) << "\n";
    return true;
}

// ---- Whole-entry writers (FireDogRig console) -------------------------------
// The read-modify-write halves of the savers above, factored out because six
// more entry writers follow the identical shape.
namespace {

// Load the document for rewriting. False means "do not write" — either the file
// exists but doesn't parse (clobbering it would lose the whole machine's config)
// or it isn't there at all (an entry can only be updated, never invented).
bool ReadConfigDoc(const std::string& path, nlohmann::json& j)
{
    std::ifstream f(path);
    if (!f) {
        std::cerr << "[config] hardware config not found: " << path << std::endl;
        return false;
    }
    try { f >> j; }
    catch (const std::exception& e) {
        std::cerr << "[config] cannot rewrite " << path << ": " << e.what() << std::endl;
        return false;   // don't clobber a file we couldn't parse
    }
    return true;
}

bool WriteConfigDoc(const std::string& path, const nlohmann::json& j)
{
    std::ofstream out(path);
    if (!out) {
        std::cerr << "[config] cannot write " << path << std::endl;
        return false;
    }
    out << j.dump(2) << "\n";
    return true;
}

// Find the entry of `section` whose `key` equals `value`. Null if absent.
nlohmann::json* FindEntry(nlohmann::json& j, const char* section,
                          const char* key, const std::string& value)
{
    if (!j.contains(section) || !j[section].is_array()) return nullptr;
    for (auto& e : j[section]) {
        if (e.value(key, std::string()) == value) return &e;
    }
    return nullptr;
}

void ReportMissing(const char* section, const std::string& value, const std::string& path)
{
    std::cerr << "[config] " << section << " entry '" << value << "' not found in "
              << path << std::endl;
}

} // namespace

bool HardwareConfig::SaveMotionDevice(const std::string& path, const MotionConfig& cfg)
{
    nlohmann::json j;
    if (!ReadConfigDoc(path, j)) return false;

    nlohmann::json* m = FindEntry(j, "motion_devices", "id", cfg.id);
    if (!m) {
        ReportMissing("motion_devices", cfg.id, path);
        return false;
    }

    (*m)["name"]            = cfg.name;
    (*m)["controller_type"] = cfg.controllerType;
    (*m)["axes"]            = cfg.axes;
    (*m)["ip_address"]      = cfg.ipAddress;
    (*m)["port"]            = cfg.port;
    (*m)["device_id"]       = cfg.deviceId;
    (*m)["enabled"]         = cfg.enabled;
    return WriteConfigDoc(path, j);
}

bool HardwareConfig::SaveIoDevice(const std::string& path, const IoDeviceConfig& cfg)
{
    nlohmann::json j;
    if (!ReadConfigDoc(path, j)) return false;

    nlohmann::json* d = FindEntry(j, "io_devices", "id", cfg.id);
    if (!d) {
        ReportMissing("io_devices", cfg.id, path);
        return false;
    }

    // "pins" is deliberately untouched — pin name maps are edited in the file or
    // the GUI, never by the rig, and rewriting them here would reorder the map.
    (*d)["name"]             = cfg.name;
    (*d)["type"]             = cfg.type;
    (*d)["ip_address"]       = cfg.ipAddress;
    (*d)["port"]             = cfg.port;
    (*d)["device_id"]        = cfg.deviceId;
    (*d)["input_count"]      = cfg.inputCount;
    (*d)["output_count"]     = cfg.outputCount;
    (*d)["poll_interval_ms"] = cfg.pollIntervalMs;
    (*d)["enabled"]          = cfg.enabled;
    (*d)["simulated"]        = cfg.simulated;
    return WriteConfigDoc(path, j);
}

bool HardwareConfig::SaveCameraDevice(const std::string& path, const CameraConfig& cfg)
{
    nlohmann::json j;
    if (!ReadConfigDoc(path, j)) return false;

    nlohmann::json* c = FindEntry(j, "cameras", "id", cfg.id);
    if (!c) {
        ReportMissing("cameras", cfg.id, path);
        return false;
    }

    // Unlike SaveCameraSettings (calibration only, from the Camera page), this
    // writes the whole entry: the rig edits identity and calibration together.
    (*c)["camera_type"]     = cfg.cameraType;
    (*c)["connection_type"] = cfg.connectionType;
    (*c)["ip_address"]      = cfg.ipAddress;
    (*c)["serial_number"]   = cfg.serialNumber;
    (*c)["display_name"]    = cfg.displayName;
    (*c)["device_index"]    = cfg.deviceIndex;
    (*c)["enabled"]         = cfg.enabled;
    (*c)["auto_connect"]    = cfg.autoConnect;
    (*c)["pixel_to_mm_x"]   = cfg.pixelToMmX;
    (*c)["pixel_to_mm_y"]   = cfg.pixelToMmY;
    (*c)["exposure_us"]     = cfg.exposureUs;
    return WriteConfigDoc(path, j);
}

bool HardwareConfig::SaveInstrument(const std::string& path, const KeithleyConfig& cfg)
{
    nlohmann::json j;
    if (!ReadConfigDoc(path, j)) return false;

    nlohmann::json* inst = FindEntry(j, "instruments", "name", cfg.name);
    if (!inst) {
        ReportMissing("instruments", cfg.name, path);
        return false;
    }

    (*inst)["visa_resource"]    = cfg.visaResource;
    (*inst)["enabled"]          = cfg.enabled;
    (*inst)["channel_count"]    = cfg.channelCount;
    (*inst)["enabled_channels"] = cfg.enabledChannels;
    (*inst)["measurement_nplc"] = cfg.nplc;
    (*inst)["auto_range"]       = cfg.autoRange;
    (*inst)["fixed_range"]      = cfg.fixedRange;
    (*inst)["poll_interval_ms"] = cfg.pollIntervalMs;
    (*inst)["bias_voltage"]     = cfg.biasVoltage;
    return WriteConfigDoc(path, j);
}

bool HardwareConfig::SaveInstrument(const std::string& path, const Cld101xConfig& cfg)
{
    nlohmann::json j;
    if (!ReadConfigDoc(path, j)) return false;

    nlohmann::json* inst = FindEntry(j, "instruments", "name", cfg.name);
    if (!inst) {
        ReportMissing("instruments", cfg.name, path);
        return false;
    }

    (*inst)["visa_resource"]         = cfg.visaResource;
    (*inst)["enabled"]               = cfg.enabled;
    (*inst)["default_temperature"]   = cfg.defaultTemperature;
    (*inst)["default_laser_current"] = cfg.defaultLaserCurrentMA;
    (*inst)["poll_interval_ms"]      = cfg.pollIntervalMs;
    return WriteConfigDoc(path, j);
}

bool HardwareConfig::SaveInstrument(const std::string& path, const PsuConfig& cfg)
{
    nlohmann::json j;
    if (!ReadConfigDoc(path, j)) return false;

    nlohmann::json* inst = FindEntry(j, "instruments", "name", cfg.name);
    if (!inst) {
        ReportMissing("instruments", cfg.name, path);
        return false;
    }

    (*inst)["visa_resource"]    = cfg.visaResource;
    (*inst)["enabled"]          = cfg.enabled;
    (*inst)["channels"]         = cfg.channels;
    (*inst)["poll_interval_ms"] = cfg.pollIntervalMs;
    return WriteConfigDoc(path, j);
}

bool HardwareConfig::SaveInstrument(const std::string& path, const LightConfig& cfg)
{
    nlohmann::json j;
    if (!ReadConfigDoc(path, j)) return false;

    nlohmann::json* inst = FindEntry(j, "instruments", "name", cfg.name);
    if (!inst) {
        ReportMissing("instruments", cfg.name, path);
        return false;
    }

    (*inst)["ip_address"]       = cfg.ipAddress;
    (*inst)["port"]             = cfg.port;
    (*inst)["enabled"]          = cfg.enabled;
    (*inst)["simulated"]        = cfg.simulated;
    (*inst)["poll_interval_ms"] = cfg.pollIntervalMs;
    return WriteConfigDoc(path, j);
}

std::string HardwareConfigPath()   { return MachinePath("config/hardware.json"); }
std::string MotionTransformsPath() { return MachinePath("config/motion_transforms.json"); }

std::map<std::string, AxisTransform> LoadMotionTransforms(const std::string& path)
{
    std::map<std::string, AxisTransform> out;
    std::ifstream f(path);
    if (!f) {
        std::cerr << "[config] motion transforms not found: " << path << std::endl;
        return out;
    }
    try {
        nlohmann::json j;
        f >> j;
        static const char* mKeys[3][3] = {
            {"M11", "M12", "M13"}, {"M21", "M22", "M23"}, {"M31", "M32", "M33"}};
        static const char* rKeys[3][3] = {
            {"R11", "R12", "R13"}, {"R21", "R22", "R23"}, {"R31", "R32", "R33"}};
        for (const auto& e : j) {
            const std::string id = e.value("DeviceId", std::string());
            if (id.empty() || !e.contains("Matrix")) continue;
            const auto& mm = e["Matrix"];
            AxisTransform t;
            for (int r = 0; r < 3; ++r)
                for (int c = 0; c < 3; ++c)
                    t.m[r][c] = mm.value(mKeys[r][c], (r == c) ? 1.0 : 0.0);
            if (e.contains("Rotation")) {   // optional; identity when absent
                const auto& rr = e["Rotation"];
                for (int r = 0; r < 3; ++r)
                    for (int c = 0; c < 3; ++c)
                        t.r[r][c] = rr.value(rKeys[r][c], (r == c) ? 1.0 : 0.0);
            }
            out[id] = t;
        }
    } catch (const std::exception& e) {
        std::cerr << "[config] transforms parse error: " << e.what() << std::endl;
    }
    return out;
}

bool SaveMotionTransforms(const std::string& path,
                          const std::map<std::string, AxisTransform>& transforms)
{
    static const char* mKeys[3][3] = {
        {"M11", "M12", "M13"}, {"M21", "M22", "M23"}, {"M31", "M32", "M33"}};
    static const char* rKeys[3][3] = {
        {"R11", "R12", "R13"}, {"R21", "R22", "R23"}, {"R31", "R32", "R33"}};

    nlohmann::json j = nlohmann::json::array();
    for (const auto& [id, t] : transforms) {
        nlohmann::json e;
        e["DeviceId"] = id;
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c) {
                e["Matrix"][mKeys[r][c]]   = t.m[r][c];
                e["Rotation"][rKeys[r][c]] = t.r[r][c];
            }
        j.push_back(e);
    }

    std::ofstream out(path);
    if (!out) {
        std::cerr << "[config] cannot write " << path << std::endl;
        return false;
    }
    out << j.dump(2) << "\n";
    return true;
}

} // namespace firedog
