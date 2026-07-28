// HardwareConfig.cpp
#include "hardware/HardwareConfig.h"

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
        static const char* keys[3][3] = {
            {"M11", "M12", "M13"}, {"M21", "M22", "M23"}, {"M31", "M32", "M33"}};
        for (const auto& e : j) {
            const std::string id = e.value("DeviceId", std::string());
            if (id.empty() || !e.contains("Matrix")) continue;
            const auto& mm = e["Matrix"];
            AxisTransform t;
            for (int r = 0; r < 3; ++r)
                for (int c = 0; c < 3; ++c)
                    t.m[r][c] = mm.value(keys[r][c], (r == c) ? 1.0 : 0.0);
            out[id] = t;
        }
    } catch (const std::exception& e) {
        std::cerr << "[config] transforms parse error: " << e.what() << std::endl;
    }
    return out;
}

} // namespace firedog
