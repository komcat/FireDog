// RigConfigEdit.cpp
#include "RigConfigEdit.h"

#include <algorithm>
#include <cctype>
#include <cstddef>
#include <exception>
#include <initializer_list>
#include <sstream>

namespace firedog::rig {
namespace {

std::string ToLower(std::string s)
{
    std::transform(s.begin(), s.end(), s.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return s;
}

// Compact numeric formatting for `show` — 0.00248 stays 0.00248, 50000 stays
// 50000, no trailing ".000000".
std::string Num(double v)
{
    std::ostringstream os;
    os << v;
    return os.str();
}

std::string Yes(bool b) { return b ? "true" : "false"; }

// Column padding that never lets two columns collide: always at least two
// trailing spaces, even when the value is wider than the column (a USB VISA
// resource is 42 characters and overruns anything reasonable).
std::string Pad(const std::string& s, std::size_t width)
{
    return s + std::string(s.size() + 2 <= width ? width - s.size() : 2, ' ');
}

bool ParseBool(const std::string& s, bool& out)
{
    const std::string v = ToLower(s);
    if (v == "1" || v == "true"  || v == "on"  || v == "yes") { out = true;  return true; }
    if (v == "0" || v == "false" || v == "off" || v == "no")  { out = false; return true; }
    return false;
}

bool ParseInt(const std::string& s, int& out)
{
    try {
        std::size_t used = 0;
        const int v = std::stoi(s, &used);
        if (used != s.size()) return false;
        out = v;
        return true;
    } catch (const std::exception&) {
        return false;
    }
}

bool ParseDouble(const std::string& s, double& out)
{
    try {
        std::size_t used = 0;
        const double v = std::stod(s, &used);
        if (used != s.size()) return false;
        out = v;
        return true;
    } catch (const std::exception&) {
        return false;
    }
}

// Match one of the allowed literals case-insensitively, writing the canonical
// spelling to `out`. False if the value is none of them.
bool ParseEnum(const std::string& s, std::initializer_list<const char*> allowed,
               std::string& out)
{
    const std::string v = ToLower(s);
    for (const char* a : allowed) {
        if (v == ToLower(a)) { out = a; return true; }
    }
    return false;
}

std::string EnumList(std::initializer_list<const char*> allowed)
{
    std::string s;
    for (const char* a : allowed) {
        if (!s.empty()) s += " | ";
        s += a;
    }
    return s;
}

} // namespace

const char* KindName(DeviceKind k)
{
    switch (k) {
        case DeviceKind::Motion:   return "motion";
        case DeviceKind::Io:       return "io";
        case DeviceKind::Camera:   return "camera";
        case DeviceKind::Keithley: return "keithley";
        case DeviceKind::Cld:      return "cld";
        case DeviceKind::Psu:      return "psu";
    }
    return "?";
}

bool ConfigStore::Load(const std::string& path)
{
    path_ = path;
    cfg_  = HardwareConfig::Load(path);
    dirty_.clear();
    RebuildIndex();
    return !devices_.empty();
}

void ConfigStore::RebuildIndex()
{
    devices_.clear();
    for (std::size_t i = 0; i < cfg_.motionDevices.size(); ++i) {
        const MotionConfig& m = cfg_.motionDevices[i];
        devices_.push_back({DeviceKind::Motion, i, m.id.empty() ? m.name : m.id});
    }
    for (std::size_t i = 0; i < cfg_.ioDevices.size(); ++i) {
        const IoDeviceConfig& d = cfg_.ioDevices[i];
        devices_.push_back({DeviceKind::Io, i, d.id.empty() ? d.name : d.id});
    }
    for (std::size_t i = 0; i < cfg_.cameras.size(); ++i) {
        devices_.push_back({DeviceKind::Camera, i, cfg_.cameras[i].id});
    }
    for (std::size_t i = 0; i < cfg_.keithleys.size(); ++i) {
        devices_.push_back({DeviceKind::Keithley, i, cfg_.keithleys[i].name});
    }
    for (std::size_t i = 0; i < cfg_.clds.size(); ++i) {
        devices_.push_back({DeviceKind::Cld, i, cfg_.clds[i].name});
    }
    for (std::size_t i = 0; i < cfg_.psus.size(); ++i) {
        devices_.push_back({DeviceKind::Psu, i, cfg_.psus[i].name});
    }
}

std::optional<DeviceRef> ConfigStore::Find(const std::string& id) const
{
    for (const DeviceRef& d : devices_) {
        if (d.id == id) return d;
    }
    // Motion and IO entries carry both an id and a display name; accept either.
    for (const DeviceRef& d : devices_) {
        if (d.kind == DeviceKind::Motion && cfg_.motionDevices[d.index].name == id) return d;
        if (d.kind == DeviceKind::Io     && cfg_.ioDevices[d.index].name     == id) return d;
        if (d.kind == DeviceKind::Camera && cfg_.cameras[d.index].displayName == id) return d;
    }
    const std::string want = ToLower(id);
    for (const DeviceRef& d : devices_) {
        if (ToLower(d.id) == want) return d;
    }
    return std::nullopt;
}

std::string ConfigStore::KeyOf(const DeviceRef& d) const
{
    return std::string(KindName(d.kind)) + ":" + std::to_string(d.index);
}

bool ConfigStore::Dirty(const DeviceRef& d) const
{
    return dirty_.count(KeyOf(d)) != 0;
}

std::vector<DeviceRef> ConfigStore::DirtyDevices() const
{
    std::vector<DeviceRef> out;
    for (const DeviceRef& d : devices_) {
        if (Dirty(d)) out.push_back(d);
    }
    return out;
}

std::string ConfigStore::Summary(const DeviceRef& d) const
{
    std::ostringstream os;
    std::string kind;
    std::string addr;
    bool enabled = true;

    switch (d.kind) {
        case DeviceKind::Motion: {
            const MotionConfig& m = cfg_.motionDevices[d.index];
            kind    = m.controllerType + " motion";
            addr    = m.ipAddress + ":" + std::to_string(m.port);
            enabled = m.enabled;
            break;
        }
        case DeviceKind::Io: {
            const IoDeviceConfig& i = cfg_.ioDevices[d.index];
            kind    = i.type + " io";
            addr    = i.type == "eziio" ? i.ipAddress + " (board " + std::to_string(i.deviceId) + ")"
                                        : i.ipAddress + ":" + std::to_string(i.port);
            enabled = i.enabled;
            break;
        }
        case DeviceKind::Camera: {
            const CameraConfig& c = cfg_.cameras[d.index];
            kind    = c.cameraType + " camera";
            addr    = c.connectionType == "ip" ? c.ipAddress : c.connectionType;
            enabled = c.enabled;
            break;
        }
        case DeviceKind::Keithley: {
            const KeithleyConfig& k = cfg_.keithleys[d.index];
            kind = k.type; addr = k.visaResource; enabled = k.enabled;
            break;
        }
        case DeviceKind::Cld: {
            const Cld101xConfig& c = cfg_.clds[d.index];
            kind = c.type; addr = c.visaResource; enabled = c.enabled;
            break;
        }
        case DeviceKind::Psu: {
            const PsuConfig& p = cfg_.psus[d.index];
            kind = p.type; addr = p.visaResource; enabled = p.enabled;
            break;
        }
    }

    os << "  " << (Dirty(d) ? "*" : " ") << " "
       << Pad(d.id, 18) << Pad(kind, 18) << Pad(addr, 44)
       << (enabled ? "enabled" : "disabled");
    return os.str();
}

std::vector<std::pair<std::string, std::string>> ConfigStore::Fields(const DeviceRef& d) const
{
    std::vector<std::pair<std::string, std::string>> f;
    switch (d.kind) {
        case DeviceKind::Motion: {
            const MotionConfig& m = cfg_.motionDevices[d.index];
            f.push_back({"name",            m.name});
            f.push_back({"controller_type", m.controllerType});
            f.push_back({"axes",            m.axes});
            f.push_back({"ip_address",      m.ipAddress});
            f.push_back({"port",            std::to_string(m.port)});
            f.push_back({"device_id",       std::to_string(m.deviceId)});
            f.push_back({"enabled",         Yes(m.enabled)});
            break;
        }
        case DeviceKind::Io: {
            const IoDeviceConfig& i = cfg_.ioDevices[d.index];
            f.push_back({"name",             i.name});
            f.push_back({"type",             i.type});
            f.push_back({"ip_address",       i.ipAddress});
            f.push_back({"port",             std::to_string(i.port)});
            f.push_back({"device_id",        std::to_string(i.deviceId)});
            f.push_back({"input_count",      std::to_string(i.inputCount)});
            f.push_back({"output_count",     std::to_string(i.outputCount)});
            f.push_back({"poll_interval_ms", std::to_string(i.pollIntervalMs)});
            f.push_back({"enabled",          Yes(i.enabled)});
            f.push_back({"simulated",        Yes(i.simulated)});
            break;
        }
        case DeviceKind::Camera: {
            const CameraConfig& c = cfg_.cameras[d.index];
            f.push_back({"camera_type",     c.cameraType});
            f.push_back({"connection_type", c.connectionType});
            f.push_back({"ip_address",      c.ipAddress});
            f.push_back({"serial_number",   c.serialNumber});
            f.push_back({"display_name",    c.displayName});
            f.push_back({"device_index",    std::to_string(c.deviceIndex)});
            f.push_back({"exposure_us",     Num(c.exposureUs)});
            f.push_back({"pixel_to_mm_x",   Num(c.pixelToMmX)});
            f.push_back({"pixel_to_mm_y",   Num(c.pixelToMmY)});
            f.push_back({"enabled",         Yes(c.enabled)});
            f.push_back({"auto_connect",    Yes(c.autoConnect)});
            break;
        }
        case DeviceKind::Keithley: {
            const KeithleyConfig& k = cfg_.keithleys[d.index];
            f.push_back({"visa_resource",    k.visaResource});
            f.push_back({"channel_count",    std::to_string(k.channelCount)});
            f.push_back({"measurement_nplc", Num(k.nplc)});
            f.push_back({"auto_range",       Yes(k.autoRange)});
            f.push_back({"fixed_range",      Num(k.fixedRange)});
            f.push_back({"poll_interval_ms", std::to_string(k.pollIntervalMs)});
            f.push_back({"bias_voltage",     Num(k.biasVoltage)});
            f.push_back({"enabled",          Yes(k.enabled)});
            break;
        }
        case DeviceKind::Cld: {
            const Cld101xConfig& c = cfg_.clds[d.index];
            f.push_back({"visa_resource",         c.visaResource});
            f.push_back({"default_temperature",   Num(c.defaultTemperature)});
            f.push_back({"default_laser_current", Num(c.defaultLaserCurrentMA)});
            f.push_back({"poll_interval_ms",      std::to_string(c.pollIntervalMs)});
            f.push_back({"enabled",               Yes(c.enabled)});
            break;
        }
        case DeviceKind::Psu: {
            const PsuConfig& p = cfg_.psus[d.index];
            f.push_back({"visa_resource",    p.visaResource});
            f.push_back({"channels",         std::to_string(p.channels)});
            f.push_back({"poll_interval_ms", std::to_string(p.pollIntervalMs)});
            f.push_back({"enabled",          Yes(p.enabled)});
            break;
        }
    }
    return f;
}

bool ConfigStore::Set(const DeviceRef& d, const std::string& field,
                      const std::string& value, std::string& err)
{
    // Shared parse-or-explain helpers. Each returns false with `err` filled.
    bool   b = false;
    int    i = 0;
    double x = 0.0;
    auto asBool = [&](void) {
        if (ParseBool(value, b)) return true;
        err = "expected true/false for '" + field + "'";
        return false;
    };
    auto asInt = [&](void) {
        if (ParseInt(value, i)) return true;
        err = "expected an integer for '" + field + "'";
        return false;
    };
    auto asDouble = [&](void) {
        if (ParseDouble(value, x)) return true;
        err = "expected a number for '" + field + "'";
        return false;
    };
    auto unknown = [&](void) {
        err = "unknown field '" + field + "' for " + std::string(KindName(d.kind))
            + " device '" + d.id + "' (try: show " + d.id + ")";
        return false;
    };

    bool ok = false;
    switch (d.kind) {
        case DeviceKind::Motion: {
            MotionConfig& m = cfg_.motionDevices[d.index];
            if      (field == "name")       { m.name = value; ok = true; }
            else if (field == "axes")       { m.axes = value; ok = true; }
            else if (field == "ip_address") { m.ipAddress = value; ok = true; }
            else if (field == "controller_type") {
                // A typo here does not fail loudly at connect — Workflow skips
                // the device with a warning — so reject it at the edit instead.
                std::string t;
                if (!ParseEnum(value, {"PI", "ACS"}, t)) {
                    err = "controller_type must be one of: " + EnumList({"PI", "ACS"});
                    return false;
                }
                m.controllerType = t; ok = true;
            }
            else if (field == "port")      { if (!asInt())  return false; m.port = i; ok = true; }
            else if (field == "device_id") { if (!asInt())  return false; m.deviceId = i; ok = true; }
            else if (field == "enabled")   { if (!asBool()) return false; m.enabled = b; ok = true; }
            else return unknown();
            break;
        }
        case DeviceKind::Io: {
            IoDeviceConfig& io = cfg_.ioDevices[d.index];
            if      (field == "name")       { io.name = value; ok = true; }
            else if (field == "ip_address") { io.ipAddress = value; ok = true; }
            else if (field == "type") {
                std::string t;
                if (!ParseEnum(value, {"eziio", "wago"}, t)) {
                    err = "type must be one of: " + EnumList({"eziio", "wago"});
                    return false;
                }
                io.type = t; ok = true;
            }
            else if (field == "port")      { if (!asInt()) return false; io.port = i; ok = true; }
            else if (field == "device_id") { if (!asInt()) return false; io.deviceId = i; ok = true; }
            else if (field == "input_count" || field == "output_count") {
                if (!asInt()) return false;
                if (i < 0 || i > 64) { err = "pin count must be 0..64"; return false; }
                if (field == "input_count") io.inputCount = i; else io.outputCount = i;
                ok = true;
            }
            else if (field == "poll_interval_ms") { if (!asInt()) return false; io.pollIntervalMs = i; ok = true; }
            else if (field == "enabled")   { if (!asBool()) return false; io.enabled = b; ok = true; }
            else if (field == "simulated") { if (!asBool()) return false; io.simulated = b; ok = true; }
            else return unknown();
            break;
        }
        case DeviceKind::Camera: {
            CameraConfig& c = cfg_.cameras[d.index];
            if      (field == "ip_address")    { c.ipAddress = value; ok = true; }
            else if (field == "serial_number") { c.serialNumber = value; ok = true; }
            else if (field == "display_name")  { c.displayName = value; ok = true; }
            else if (field == "camera_type")   { c.cameraType = value; ok = true; }
            else if (field == "connection_type") {
                std::string t;
                if (!ParseEnum(value, {"ip", "serial", "index", "auto"}, t)) {
                    err = "connection_type must be one of: "
                        + EnumList({"ip", "serial", "index", "auto"});
                    return false;
                }
                c.connectionType = t; ok = true;
            }
            else if (field == "device_index")  { if (!asInt())    return false; c.deviceIndex = i; ok = true; }
            else if (field == "exposure_us")   { if (!asDouble()) return false; c.exposureUs = x; ok = true; }
            else if (field == "pixel_to_mm_x") { if (!asDouble()) return false; c.pixelToMmX = x; ok = true; }
            else if (field == "pixel_to_mm_y") { if (!asDouble()) return false; c.pixelToMmY = x; ok = true; }
            else if (field == "enabled")       { if (!asBool())   return false; c.enabled = b; ok = true; }
            else if (field == "auto_connect")  { if (!asBool())   return false; c.autoConnect = b; ok = true; }
            else return unknown();
            break;
        }
        case DeviceKind::Keithley: {
            KeithleyConfig& k = cfg_.keithleys[d.index];
            if      (field == "visa_resource")    { k.visaResource = value; ok = true; }
            else if (field == "channel_count")    { if (!asInt())    return false; k.channelCount = i; ok = true; }
            else if (field == "poll_interval_ms") { if (!asInt())    return false; k.pollIntervalMs = i; ok = true; }
            else if (field == "measurement_nplc") { if (!asDouble()) return false; k.nplc = x; ok = true; }
            else if (field == "fixed_range")      { if (!asDouble()) return false; k.fixedRange = x; ok = true; }
            else if (field == "bias_voltage")     { if (!asDouble()) return false; k.biasVoltage = x; ok = true; }
            else if (field == "auto_range")       { if (!asBool())   return false; k.autoRange = b; ok = true; }
            else if (field == "enabled")          { if (!asBool())   return false; k.enabled = b; ok = true; }
            else return unknown();
            break;
        }
        case DeviceKind::Cld: {
            Cld101xConfig& c = cfg_.clds[d.index];
            if      (field == "visa_resource")         { c.visaResource = value; ok = true; }
            else if (field == "poll_interval_ms")      { if (!asInt())    return false; c.pollIntervalMs = i; ok = true; }
            else if (field == "default_temperature")   { if (!asDouble()) return false; c.defaultTemperature = x; ok = true; }
            else if (field == "default_laser_current") { if (!asDouble()) return false; c.defaultLaserCurrentMA = x; ok = true; }
            else if (field == "enabled")               { if (!asBool())   return false; c.enabled = b; ok = true; }
            else return unknown();
            break;
        }
        case DeviceKind::Psu: {
            PsuConfig& p = cfg_.psus[d.index];
            if      (field == "visa_resource")    { p.visaResource = value; ok = true; }
            else if (field == "poll_interval_ms") { if (!asInt())  return false; p.pollIntervalMs = i; ok = true; }
            else if (field == "channels") {
                if (!asInt()) return false;
                if (i < 1 || i > 3) { err = "channels must be 1..3"; return false; }
                p.channels = i; ok = true;
            }
            else if (field == "enabled") { if (!asBool()) return false; p.enabled = b; ok = true; }
            else return unknown();
            break;
        }
    }

    if (ok) dirty_.insert(KeyOf(d));
    return ok;
}

bool ConfigStore::Save(const DeviceRef& d, std::string& err)
{
    bool ok = false;
    switch (d.kind) {
        case DeviceKind::Motion:
            ok = HardwareConfig::SaveMotionDevice(path_, cfg_.motionDevices[d.index]);
            break;
        case DeviceKind::Io:
            ok = HardwareConfig::SaveIoDevice(path_, cfg_.ioDevices[d.index]);
            break;
        case DeviceKind::Camera:
            ok = HardwareConfig::SaveCameraDevice(path_, cfg_.cameras[d.index]);
            break;
        case DeviceKind::Keithley:
            ok = HardwareConfig::SaveInstrument(path_, cfg_.keithleys[d.index]);
            break;
        case DeviceKind::Cld:
            ok = HardwareConfig::SaveInstrument(path_, cfg_.clds[d.index]);
            break;
        case DeviceKind::Psu:
            ok = HardwareConfig::SaveInstrument(path_, cfg_.psus[d.index]);
            break;
    }

    if (ok) dirty_.erase(KeyOf(d));
    else    err = "could not write '" + d.id + "' into " + path_ + " (see the message above)";
    return ok;
}

} // namespace firedog::rig
