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
