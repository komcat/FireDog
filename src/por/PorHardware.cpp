// PorHardware.cpp : see PorHardware.h.
#include "por/PorHardware.h"

#include "command/CommandRegistry.h"
#include "command/CommandSpec.h"
#include "core/LightCallout.h"   // ChannelInLight

#include <cstdio>
#include <map>
#include <set>
#include <utility>

namespace firedog {

namespace {

PorStatus Max(PorStatus a, PorStatus b) { return static_cast<int>(a) >= static_cast<int>(b) ? a : b; }

void Finish(PorRow& r)
{
    for (const PorParam& p : r.params) r.status = Max(r.status, p.status);
    r.worst = r.status;
    for (const PorRow& c : r.children) r.worst = Max(r.worst, c.worst);
}

// ---- What the master's deck uses ------------------------------------------------

struct Used {
    std::set<std::string> motion, io, psu, slide, light;   // light = the call-outs named
    std::set<std::pair<std::string, std::string>> pins;   // (io device, pin name)
    bool camera = false, keithley = false, cld = false;
};

void CollectSteps(const std::vector<RecipeStep>& steps, const PorSide& m,
                  const CommandRegistry& reg, Used& u)
{
    for (const RecipeStep& s : steps) {
        if (const CommandSpec* spec = reg.Find(s.type)) {
            switch (spec->needs) {
            case CommandSpec::Needs::Camera:   u.camera   = true; break;
            case CommandSpec::Needs::Keithley: u.keithley = true; break;
            case CommandSpec::Needs::Cld:      u.cld      = true; break;
            default: break;
            }
            const nlohmann::json merged = EffectiveParams(*spec, s.params);
            const std::string device = merged.value("device", std::string());
            const std::string ioDev  = merged.value("io_device", device);
            for (const ParamSpec& p : spec->params) {
                if (!merged.contains(p.name) || !merged[p.name].is_string()) continue;
                const std::string v = merged[p.name].get<std::string>();
                switch (p.type) {
                case ParamType::DeviceMotion:
                case ParamType::DeviceHexapod: if (!v.empty()) u.motion.insert(v); break;
                case ParamType::DeviceIo:      if (!v.empty()) u.io.insert(v); break;
                case ParamType::DevicePsu:
                    // "" = the first configured supply, exactly as a run picks it.
                    if (!v.empty()) u.psu.insert(v);
                    else if (!m.hw.psus.empty()) u.psu.insert(m.hw.psus.front());
                    break;
                case ParamType::Slide:     if (!v.empty()) u.slide.insert(v); break;
                case ParamType::LightName: if (!v.empty()) u.light.insert(v); break;
                case ParamType::PinName:
                    if (!v.empty()) { u.pins.insert({ioDev, v}); u.io.insert(ioDev); }
                    break;
                default: break;
                }
            }
        }
        CollectSteps(s.children, m, reg, u);
        CollectSteps(s.elseChildren, m, reg, u);
    }
}

Used CollectUsed(const PorSide& m, const CommandRegistry& reg)
{
    Used u;
    for (const auto& [uid, why] : WorkingRecipes(m.programs, m.recipes))
        if (const Recipe* r = RecipeByUid(m.recipes, uid)) CollectSteps(r->steps, m, reg, u);
    // A slide the deck drives drives its own pins.
    for (const PneumaticSlideConfig& sl : m.hwConfig.pneumaticSlides) {
        if (!u.slide.count(sl.name)) continue;
        u.pins.insert({sl.outputDevice, sl.outputPin});
        u.pins.insert({sl.extendedDevice, sl.extendedPin});
        u.pins.insert({sl.retractedDevice, sl.retractedPin});
    }
    return u;
}

// ---- Fields --------------------------------------------------------------------

std::string Str(const std::string& s) { return s; }
std::string Str(int v)                { return std::to_string(v); }
std::string Str(bool v)               { return v ? "on" : "off"; }
std::string Str(double v)
{
    char b[32];
    std::snprintf(b, sizeof(b), "%g", v);
    return b;
}
std::string Str(const std::vector<int>& v)
{
    std::string t;
    for (int x : v) t += (t.empty() ? "" : ", ") + std::to_string(x);
    return t;
}

std::string MatrixText(const double (&a)[3][3])
{
    std::string t = "[";
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) t += (j ? " " : "") + Str(a[i][j]);
        t += i < 2 ? "; " : "]";
    }
    return t;
}

void Fs(PorRow& r, const char* name, bool hm, bool ht, const std::string& a,
        const std::string& b, PorParamClass cls = PorParamClass::Design)
{
    PorParam p;
    p.name      = name;
    p.cls       = cls;
    p.hasMaster = hm;
    p.hasTarget = ht;
    if (hm) p.master = a;
    if (ht) p.target = b;
    if (hm && ht && a != b) {
        if (cls == PorParamClass::Machine)       p.note = "this machine's own";
        else if (cls == PorParamClass::Measured) p.status = PorStatus::Measured;
        else                                     p.status = PorStatus::Changed;
    }
    r.params.push_back(std::move(p));
}

template <class T, class V>
void F(PorRow& r, const T* m, const T* t, const char* name, V T::*f,
       PorParamClass cls = PorParamClass::Design)
{
    Fs(r, name, m != nullptr, t != nullptr, m ? Str(m->*f) : std::string(),
       t ? Str(t->*f) : std::string(), cls);
}

// A device row's own presence. A one-sided device the deck USES is Missing.
PorRow Dev(PorRowKind kind, const std::string& name, bool hm, bool ht, bool used)
{
    PorRow r;
    r.kind = kind;
    r.used = used;
    if (hm) r.master = name;
    if (ht) r.target = name;
    if (hm && !ht)      r.status = used ? PorStatus::Missing : PorStatus::OnlyMaster;
    else if (!hm && ht) r.status = PorStatus::OnlyTarget;
    return r;
}

// Pair two lists by a key (a name, a channel number); a key used twice on a
// side is Ambiguous and left unpaired. Master order, then the target's rest.
template <class T, class KeyFn, class RowFn>
std::vector<PorRow> PairBy(const std::vector<T>& a, const std::vector<T>& b, KeyFn key, RowFn row)
{
    std::map<std::string, int> ca, cb;
    for (const T& x : a) ++ca[key(x)];
    for (const T& x : b) ++cb[key(x)];
    const auto amb = [&](const std::string& k) {
        return (ca.count(k) && ca.at(k) > 1) || (cb.count(k) && cb.at(k) > 1);
    };
    std::vector<PorRow> out;
    std::set<std::size_t> used;
    for (const T& x : a) {
        const std::string k = key(x);
        if (amb(k)) {
            PorRow r = row(&x, nullptr);
            r.status = PorStatus::Ambiguous;
            Finish(r);
            out.push_back(std::move(r));
            continue;
        }
        const T* match = nullptr;
        for (std::size_t j = 0; j < b.size(); ++j)
            if (key(b[j]) == k) { match = &b[j]; used.insert(j); break; }
        out.push_back(row(&x, match));
    }
    for (std::size_t j = 0; j < b.size(); ++j) {
        if (used.count(j)) continue;
        PorRow r = row(nullptr, &b[j]);
        if (amb(key(b[j]))) { r.status = PorStatus::Ambiguous; Finish(r); }
        out.push_back(std::move(r));
    }
    return out;
}

PorRow Section(const char* title, std::vector<PorRow> children)
{
    PorRow r;
    r.kind     = PorRowKind::Section;
    r.master   = title;
    r.target   = title;
    r.children = std::move(children);
    Finish(r);
    return r;
}

// ---- IO pins: where a name sits, pin by pin ---------------------------------------

void Pins(PorRow& dev, const std::string& devName, const IoDeviceConfig* m,
          const IoDeviceConfig* t, bool input, const Used& u)
{
    static const std::map<int, std::string> kNone;
    const std::map<int, std::string>& mm = m ? (input ? m->inputNames : m->outputNames) : kNone;
    const std::map<int, std::string>& tm = t ? (input ? t->inputNames : t->outputNames) : kNone;
    std::map<std::string, int> mRev, tRev;   // name -> pin
    for (const auto& [pin, name] : mm) mRev[name] = pin;
    for (const auto& [pin, name] : tm) tRev[name] = pin;
    std::set<int> pins;
    for (const auto& kv : mm) pins.insert(kv.first);
    for (const auto& kv : tm) pins.insert(kv.first);

    const std::string dir = input ? "IN " : "OUT ";
    for (int pin : pins) {
        const auto mi = mm.find(pin), ti = tm.find(pin);
        const std::string mn = mi != mm.end() ? mi->second : std::string();
        const std::string tn = ti != tm.end() ? ti->second : std::string();
        const bool hm = !mn.empty(), ht = !tn.empty();
        const std::string label = dir + std::to_string(pin);

        PorRow r;
        r.kind = PorRowKind::Item;
        r.used = hm && u.pins.count({devName, mn}) > 0;
        if (hm) r.master = label + "  " + mn;
        if (ht) r.target = label + "  " + tn;
        Fs(r, "name", hm, ht, mn, tn);
        r.params.back().status = PorStatus::Same;   // judged below, with the wiring

        if (hm && ht && mn == tn) {
            // the same name on the same pin: nothing to say
        } else if (hm) {
            const auto moved = tRev.find(mn);
            if (moved != tRev.end()) {
                r.status = PorStatus::Changed;
                r.params.back().note = "on the target '" + mn + "' is " + dir + std::to_string(moved->second) +
                                       " - wired differently";
            } else {
                r.status = r.used ? PorStatus::Missing : (ht ? PorStatus::Changed : PorStatus::OnlyMaster);
                r.params.back().note = "'" + mn + "' is not on the target" +
                                       std::string(r.used ? " and the deck uses it" : "");
            }
        } else {
            const auto there = mRev.find(tn);
            r.status = PorStatus::OnlyTarget;
            if (there != mRev.end()) {
                r.status = PorStatus::Changed;
                r.params.back().note = "on the master '" + tn + "' is " + dir + std::to_string(there->second);
            }
        }
        Finish(r);
        dev.children.push_back(std::move(r));
    }
}

// ---- Sections ---------------------------------------------------------------------

PorRow MotionRow(const MotionConfig* m, const MotionConfig* t, const Used& u)
{
    const std::string name = m ? m->name : t->name;
    PorRow r = Dev(PorRowKind::Device, name, m != nullptr, t != nullptr, u.motion.count(name) > 0);
    F(r, m, t, "controller", &MotionConfig::controllerType);
    F(r, m, t, "axes", &MotionConfig::axes);
    F(r, m, t, "enabled", &MotionConfig::enabled);
    F(r, m, t, "id", &MotionConfig::id, PorParamClass::Machine);
    F(r, m, t, "ip", &MotionConfig::ipAddress, PorParamClass::Machine);
    F(r, m, t, "port", &MotionConfig::port, PorParamClass::Machine);
    F(r, m, t, "device_id", &MotionConfig::deviceId, PorParamClass::Machine);
    Finish(r);
    return r;
}

using XfEntry = std::pair<std::string, AxisTransform>;

PorRow TransformRow(const XfEntry* m, const XfEntry* t, const Used& u)
{
    const std::string name = m ? m->first : t->first;
    PorRow r = Dev(PorRowKind::Device, name, m != nullptr, t != nullptr, u.motion.count(name) > 0);
    Fs(r, "matrix", m != nullptr, t != nullptr, m ? MatrixText(m->second.m) : "", t ? MatrixText(t->second.m) : "");
    Fs(r, "rotation", m != nullptr, t != nullptr, m ? MatrixText(m->second.r) : "", t ? MatrixText(t->second.r) : "");
    Finish(r);
    return r;
}

PorRow IoRow(const IoDeviceConfig* m, const IoDeviceConfig* t, const Used& u)
{
    const std::string name = m ? m->name : t->name;
    PorRow r = Dev(PorRowKind::Device, name, m != nullptr, t != nullptr, u.io.count(name) > 0);
    F(r, m, t, "type", &IoDeviceConfig::type);
    F(r, m, t, "inputs", &IoDeviceConfig::inputCount);
    F(r, m, t, "outputs", &IoDeviceConfig::outputCount);
    F(r, m, t, "enabled", &IoDeviceConfig::enabled);
    F(r, m, t, "poll_ms", &IoDeviceConfig::pollIntervalMs);
    F(r, m, t, "simulated", &IoDeviceConfig::simulated);
    F(r, m, t, "id", &IoDeviceConfig::id, PorParamClass::Machine);
    F(r, m, t, "ip", &IoDeviceConfig::ipAddress, PorParamClass::Machine);
    F(r, m, t, "port", &IoDeviceConfig::port, PorParamClass::Machine);
    F(r, m, t, "device_id", &IoDeviceConfig::deviceId, PorParamClass::Machine);
    Pins(r, name, m, t, true, u);
    Pins(r, name, m, t, false, u);
    Finish(r);
    return r;
}

PorRow SlideRow(const PneumaticSlideConfig* m, const PneumaticSlideConfig* t, const Used& u)
{
    const std::string name = m ? m->name : t->name;
    PorRow r = Dev(PorRowKind::Device, name, m != nullptr, t != nullptr, u.slide.count(name) > 0);
    F(r, m, t, "output_device", &PneumaticSlideConfig::outputDevice);
    F(r, m, t, "output_pin", &PneumaticSlideConfig::outputPin);
    F(r, m, t, "extended_device", &PneumaticSlideConfig::extendedDevice);
    F(r, m, t, "extended_pin", &PneumaticSlideConfig::extendedPin);
    F(r, m, t, "retracted_device", &PneumaticSlideConfig::retractedDevice);
    F(r, m, t, "retracted_pin", &PneumaticSlideConfig::retractedPin);
    F(r, m, t, "timeout_ms", &PneumaticSlideConfig::timeoutMs);
    F(r, m, t, "enabled", &PneumaticSlideConfig::enabled);
    Finish(r);
    return r;
}

std::string CameraKey(const CameraConfig& c) { return c.displayName.empty() ? c.id : c.displayName; }

PorRow CameraRow(const CameraConfig* m, const CameraConfig* t, const Used& u)
{
    const std::string name = CameraKey(m ? *m : *t);
    PorRow r = Dev(PorRowKind::Device, name, m != nullptr, t != nullptr, u.camera && m && m->enabled);
    F(r, m, t, "type", &CameraConfig::cameraType);
    F(r, m, t, "connection", &CameraConfig::connectionType);
    F(r, m, t, "enabled", &CameraConfig::enabled);
    F(r, m, t, "auto_connect", &CameraConfig::autoConnect);
    F(r, m, t, "exposure_us", &CameraConfig::exposureUs);
    F(r, m, t, "pixel_to_mm_x", &CameraConfig::pixelToMmX, PorParamClass::Measured);
    F(r, m, t, "pixel_to_mm_y", &CameraConfig::pixelToMmY, PorParamClass::Measured);
    F(r, m, t, "id", &CameraConfig::id, PorParamClass::Machine);
    F(r, m, t, "ip", &CameraConfig::ipAddress, PorParamClass::Machine);
    F(r, m, t, "serial", &CameraConfig::serialNumber, PorParamClass::Machine);
    F(r, m, t, "device_index", &CameraConfig::deviceIndex, PorParamClass::Machine);
    Finish(r);
    return r;
}

PorRow KeithleyRow(const KeithleyConfig* m, const KeithleyConfig* t, const Used& u)
{
    const std::string name = m ? m->name : t->name;
    PorRow r = Dev(PorRowKind::Device, name, m != nullptr, t != nullptr, u.keithley && m && m->enabled);
    F(r, m, t, "type", &KeithleyConfig::type);
    F(r, m, t, "enabled", &KeithleyConfig::enabled);
    F(r, m, t, "channels", &KeithleyConfig::channelCount);
    F(r, m, t, "enabled_channels", &KeithleyConfig::enabledChannels);
    F(r, m, t, "nplc", &KeithleyConfig::nplc);
    F(r, m, t, "auto_range", &KeithleyConfig::autoRange);
    F(r, m, t, "fixed_range", &KeithleyConfig::fixedRange);
    F(r, m, t, "poll_ms", &KeithleyConfig::pollIntervalMs);
    F(r, m, t, "bias_v", &KeithleyConfig::biasVoltage);
    F(r, m, t, "visa", &KeithleyConfig::visaResource, PorParamClass::Machine);
    Finish(r);
    return r;
}

PorRow CldRow(const Cld101xConfig* m, const Cld101xConfig* t, const Used& u)
{
    const std::string name = m ? m->name : t->name;
    PorRow r = Dev(PorRowKind::Device, name, m != nullptr, t != nullptr, u.cld && m && m->enabled);
    F(r, m, t, "type", &Cld101xConfig::type);
    F(r, m, t, "enabled", &Cld101xConfig::enabled);
    F(r, m, t, "default_temp_c", &Cld101xConfig::defaultTemperature);
    F(r, m, t, "default_current_ma", &Cld101xConfig::defaultLaserCurrentMA);
    F(r, m, t, "poll_ms", &Cld101xConfig::pollIntervalMs);
    F(r, m, t, "visa", &Cld101xConfig::visaResource, PorParamClass::Machine);
    Finish(r);
    return r;
}

PorRow PsuRow(const PsuConfig* m, const PsuConfig* t, const Used& u)
{
    const std::string name = m ? m->name : t->name;
    PorRow r = Dev(PorRowKind::Device, name, m != nullptr, t != nullptr, u.psu.count(name) > 0);
    F(r, m, t, "type", &PsuConfig::type);
    F(r, m, t, "enabled", &PsuConfig::enabled);
    F(r, m, t, "channels", &PsuConfig::channels);
    F(r, m, t, "poll_ms", &PsuConfig::pollIntervalMs);
    F(r, m, t, "visa", &PsuConfig::visaResource, PorParamClass::Machine);
    Finish(r);
    return r;
}

PorRow ChannelRow(const LightChannelConfig* m, const LightChannelConfig* t, const Used& u)
{
    const LightChannelConfig& any = m ? *m : *t;
    bool used = false;
    if (m)
        for (const std::string& callout : u.light)
            used = used || ChannelInLight(m->group, m->name, callout);
    PorRow r = Dev(PorRowKind::Item, "L" + std::to_string(any.channel), m != nullptr, t != nullptr, used);
    if (m) r.master = "L" + std::to_string(m->channel) + "  " + m->name;
    if (t) r.target = "L" + std::to_string(t->channel) + "  " + t->name;
    F(r, m, t, "name", &LightChannelConfig::name);
    F(r, m, t, "group", &LightChannelConfig::group);
    F(r, m, t, "color", &LightChannelConfig::color);
    F(r, m, t, "camera", &LightChannelConfig::camera);
    F(r, m, t, "enabled", &LightChannelConfig::enabled);
    Finish(r);
    return r;
}

PorRow LightRow(const LightConfig* m, const LightConfig* t, const Used& u)
{
    const std::string name = m ? m->name : t->name;
    PorRow r = Dev(PorRowKind::Device, name, m != nullptr, t != nullptr, !u.light.empty() && m && m->enabled);
    F(r, m, t, "type", &LightConfig::type);
    F(r, m, t, "enabled", &LightConfig::enabled);
    F(r, m, t, "simulated", &LightConfig::simulated);
    F(r, m, t, "poll_ms", &LightConfig::pollIntervalMs);
    F(r, m, t, "ip", &LightConfig::ipAddress, PorParamClass::Machine);
    F(r, m, t, "port", &LightConfig::port, PorParamClass::Machine);
    static const std::vector<LightChannelConfig> kNone;
    r.children = PairBy(m ? m->channels : kNone, t ? t->channels : kNone,
                        [](const LightChannelConfig& c) { return std::to_string(c.channel); },
                        [&u](const LightChannelConfig* a, const LightChannelConfig* b) {
                            return ChannelRow(a, b, u);
                        });
    Finish(r);
    return r;
}

PorRow WatchBindingRow(const IoWatchBindingConfig* m, const IoWatchBindingConfig* t)
{
    const std::string name = m ? m->name : t->name;
    PorRow r = Dev(PorRowKind::Item, name, m != nullptr, t != nullptr, false);
    F(r, m, t, "input_device", &IoWatchBindingConfig::inputDevice);
    F(r, m, t, "input_pin", &IoWatchBindingConfig::inputPin);
    F(r, m, t, "output_device", &IoWatchBindingConfig::outputDevice);
    F(r, m, t, "output_pin", &IoWatchBindingConfig::outputPin);
    F(r, m, t, "indicator_device", &IoWatchBindingConfig::indicatorDevice);
    F(r, m, t, "indicator_pin", &IoWatchBindingConfig::indicatorPin);
    F(r, m, t, "trigger", &IoWatchBindingConfig::trigger);
    F(r, m, t, "behavior", &IoWatchBindingConfig::behavior);
    F(r, m, t, "debounce_ms", &IoWatchBindingConfig::debounceMs);
    F(r, m, t, "enabled", &IoWatchBindingConfig::enabled);
    Finish(r);
    return r;
}

std::string SignalKey(const SafetySignalConfig& s)
{
    return s.device + " / " + (s.pinName.empty() ? "#" + std::to_string(s.pinNumber) : s.pinName);
}

PorRow SignalRow(const SafetySignalConfig* m, const SafetySignalConfig* t)
{
    PorRow r = Dev(PorRowKind::Item, SignalKey(m ? *m : *t), m != nullptr, t != nullptr, false);
    F(r, m, t, "pin_number", &SafetySignalConfig::pinNumber);
    F(r, m, t, "trigger", &SafetySignalConfig::trigger);
    F(r, m, t, "alert_title", &SafetySignalConfig::alertTitle);
    F(r, m, t, "alert_message", &SafetySignalConfig::alertMessage);
    F(r, m, t, "critical", &SafetySignalConfig::critical);
    F(r, m, t, "enabled", &SafetySignalConfig::enabled);
    Finish(r);
    return r;
}

} // namespace

PorHardwareReport CompareHardware(const PorSide& master, const PorSide& target,
                                  const CommandRegistry& reg)
{
    PorHardwareReport rep;
    if (!master.hasHardware) rep.notes.push_back("the master has no hardware.json");
    if (!target.hasHardware) rep.notes.push_back("the target has no hardware.json");
    const Used u = CollectUsed(master, reg);
    const HardwareConfig& m = master.hwConfig;
    const HardwareConfig& t = target.hwConfig;
    const auto byName = [](const auto& x) { return x.name; };

    rep.sections.push_back(Section("Motion devices", PairBy(m.motionDevices, t.motionDevices, byName,
        [&u](const MotionConfig* a, const MotionConfig* b) { return MotionRow(a, b, u); })));

    if (master.hasTransforms || target.hasTransforms) {
        const std::vector<XfEntry> xm(master.transforms.begin(), master.transforms.end());
        const std::vector<XfEntry> xt(target.transforms.begin(), target.transforms.end());
        rep.sections.push_back(Section("Motion transforms", PairBy(xm, xt,
            [](const XfEntry& e) { return e.first; },
            [&u](const XfEntry* a, const XfEntry* b) { return TransformRow(a, b, u); })));
    } else {
        rep.notes.push_back("neither side has motion_transforms.json");
    }

    rep.sections.push_back(Section("IO devices", PairBy(m.ioDevices, t.ioDevices, byName,
        [&u](const IoDeviceConfig* a, const IoDeviceConfig* b) { return IoRow(a, b, u); })));
    rep.sections.push_back(Section("Pneumatic slides", PairBy(m.pneumaticSlides, t.pneumaticSlides, byName,
        [&u](const PneumaticSlideConfig* a, const PneumaticSlideConfig* b) { return SlideRow(a, b, u); })));
    rep.sections.push_back(Section("Cameras", PairBy(m.cameras, t.cameras, CameraKey,
        [&u](const CameraConfig* a, const CameraConfig* b) { return CameraRow(a, b, u); })));
    rep.sections.push_back(Section("Keithley", PairBy(m.keithleys, t.keithleys, byName,
        [&u](const KeithleyConfig* a, const KeithleyConfig* b) { return KeithleyRow(a, b, u); })));
    rep.sections.push_back(Section("CLD101x", PairBy(m.clds, t.clds, byName,
        [&u](const Cld101xConfig* a, const Cld101xConfig* b) { return CldRow(a, b, u); })));
    rep.sections.push_back(Section("PSUs", PairBy(m.psus, t.psus, byName,
        [&u](const PsuConfig* a, const PsuConfig* b) { return PsuRow(a, b, u); })));
    rep.sections.push_back(Section("Lights", PairBy(m.lights, t.lights, byName,
        [&u](const LightConfig* a, const LightConfig* b) { return LightRow(a, b, u); })));

    {
        PorRow w = Dev(PorRowKind::Device, "io_watch", true, true, false);
        const IoWatchConfig* a = &m.ioWatch;
        const IoWatchConfig* b = &t.ioWatch;
        F(w, a, b, "enabled", &IoWatchConfig::enabled);
        F(w, a, b, "poll_ms", &IoWatchConfig::pollingIntervalMs);
        w.children = PairBy(m.ioWatch.bindings, t.ioWatch.bindings, byName,
                            [](const IoWatchBindingConfig* x, const IoWatchBindingConfig* y) {
                                return WatchBindingRow(x, y);
                            });
        Finish(w);
        std::vector<PorRow> one;
        one.push_back(std::move(w));
        rep.sections.push_back(Section("IO watch", std::move(one)));
    }
    {
        PorRow w = Dev(PorRowKind::Device, "safety_watch", true, true, false);
        const SafetyWatchConfig* a = &m.safetyWatch;
        const SafetyWatchConfig* b = &t.safetyWatch;
        F(w, a, b, "enabled", &SafetyWatchConfig::enabled);
        F(w, a, b, "poll_ms", &SafetyWatchConfig::pollingIntervalMs);
        w.children = PairBy(m.safetyWatch.watchedInputs, t.safetyWatch.watchedInputs, SignalKey,
                            [](const SafetySignalConfig* x, const SafetySignalConfig* y) {
                                return SignalRow(x, y);
                            });
        Finish(w);
        std::vector<PorRow> one;
        one.push_back(std::move(w));
        rep.sections.push_back(Section("Safety watch", std::move(one)));
    }
    return rep;
}

} // namespace firedog
