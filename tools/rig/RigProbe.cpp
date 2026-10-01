// RigProbe.cpp
#include "RigProbe.h"

#include "RigSession.h"

#include "core/DataBus.h"
#include "core/Topics.h"
#include "hardware/HardwareManager.h"
#include "vision/HalconEngine.h"

#include <cstddef>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

namespace firedog::rig {
namespace {

// Long enough for a poll thread to land at least one sample on every topic
// (the slowest configured interval is 500 ms for the CLD/PSU).
constexpr double kReadbackSettleSec = 0.8;

void Head(const std::string& title)
{
    const std::size_t pad = title.size() < 60u ? 60u - title.size() : 0u;
    std::cout << "\n-- " << title << " " << std::string(pad, '-') << "\n";
}

void Field(const std::string& name, const std::string& value)
{
    std::cout << "   " << std::left << std::setw(18) << name << std::right << value << "\n";
}

std::string Fixed(double v, int places)
{
    std::ostringstream os;
    os << std::fixed << std::setprecision(places) << v;
    return os.str();
}

std::string Sci(double v)
{
    std::ostringstream os;
    os << std::scientific << std::setprecision(4) << v;
    return os.str();
}

std::string Yes(bool b) { return b ? "yes" : "no"; }

// Latest published value of a topic, or "(no samples)" if the poll thread has
// not produced one yet — which is itself the useful answer.
std::string Latest(const std::string& topic, const char* unit, bool scientific)
{
    DataStream& s = DataBus::Instance().Stream(topic);
    if (s.Count() == 0) return "(no samples)";
    const double v = s.LatestValue();
    return (scientific ? Sci(v) : Fixed(v, 3)) + " " + unit;
}

// One line per pin: "  3  Dispenser_Head_Up      0".
void PrintPins(const IIoDevice& dev, uint64_t bits, int count, bool inputs)
{
    for (int p = 0; p < count; ++p) {
        const bool on = ((bits >> p) & 1ull) != 0;
        std::string name = inputs ? dev.InputPinName(p) : dev.OutputPinName(p);
        if (name.empty()) name = "(unnamed)";
        std::cout << "     " << std::setw(3) << p << "  "
                  << std::left << std::setw(24) << name << std::right
                  << (on ? "1" : "0") << "\n";
    }
}

void ProbeMotion(RigSession& s)
{
    HardwareManager& hw = s.Hw();
    for (std::size_t i = 0; i < hw.MotionCount(); ++i) {
        IMotionController* m = hw.MotionDevice(i);
        Head("motion  " + hw.MotionName(i));
        Field("state", ConnStateName(hw.MotionState(i)));
        if (!m) { Field("driver", "(none)"); continue; }

        Field("type", m->Type());
        std::string axes;
        for (const std::string& a : m->Axes()) axes += (axes.empty() ? "" : " ") + a;
        Field("axes", axes);

        if (hw.MotionState(i) != ConnState::Connected) {
            const std::string err = m->LastError();
            Field("last_error", err.empty() ? "(none)" : err);
            continue;
        }

        for (const auto& [axis, pos] : m->Positions()) {
            Field("pos " + axis, Fixed(pos, 4) + " mm");
        }
        Field("settled",   Yes(m->IsSettled()));
        Field("on_target", Yes(m->IsOnTarget()));
    }
}

void ProbeIo(RigSession& s)
{
    HardwareManager& hw = s.Hw();
    for (std::size_t i = 0; i < hw.IoCount(); ++i) {
        IIoDevice* d = hw.IoDevice(i);
        Head("io  " + hw.IoName(i));
        Field("state", ConnStateName(hw.IoState(i)));
        if (!d) { Field("driver", "(none)"); continue; }

        Field("type", d->Type());
        if (hw.IoState(i) != ConnState::Connected) {
            const std::string err = d->LastError();
            Field("last_error", err.empty() ? "(none)" : err);
            continue;
        }

        const IIoDevice::IoStatus st = d->Status();
        Field("poll_data", st.valid ? "valid" : "(no successful read yet)");
        if (!st.valid) continue;

        std::cout << "   inputs (" << d->InputCount() << ")\n";
        PrintPins(*d, st.inputs, d->InputCount(), true);
        std::cout << "   outputs (" << d->OutputCount() << ")\n";
        PrintPins(*d, st.outputs, d->OutputCount(), false);
        if (st.latch != 0) {
            std::ostringstream os;
            os << "0x" << std::hex << st.latch;
            Field("latched", os.str());
        }
    }
}

void ProbeSlides(RigSession& s)
{
    HardwareManager& hw = s.Hw();
    if (hw.SlideCount() == 0) return;
    Head("pneumatic slides");
    std::cout << "   (synthetic - a slide's connection is its IO devices)\n";
    for (std::size_t i = 0; i < hw.SlideCount(); ++i) {
        PneumaticSlide* sl = hw.Slide(i);
        if (!sl) continue;
        const PneumaticSlide::Status st = sl->GetStatus();
        std::cout << "   " << std::left << std::setw(20) << sl->Name()
                  << std::setw(14) << SlidePositionName(st.position) << std::right
                  << "out=" << (st.outputOn ? "1" : "0")
                  << "  ext=" << (st.extendedOn ? "1" : "0")
                  << "  ret=" << (st.retractedOn ? "1" : "0")
                  << "  io=" << (st.ioValid ? "ok" : "stale") << "\n";
    }
}

void ProbeCamera(RigSession& s)
{
    HardwareManager& hw = s.Hw();
    if (!s.Config().FirstEnabledCamera()) return;

    Head("camera  " + hw.CameraName());
    Field("state", ConnStateName(hw.CameraState()));
    PylonCamera& cam = hw.Camera();
    if (hw.CameraState() != ConnState::Connected) {
        const std::string err = cam.LastError();
        Field("last_error", err.empty() ? "(none)" : err);
        return;
    }

    const PylonCamera::Info info = cam.GetInfo();
    Field("model",  info.model);
    Field("serial", info.serial);
    Field("ip",     info.ip);
    Field("frame",  std::to_string(info.width) + " x " + std::to_string(info.height));
    Field("fps",    Fixed(info.fps, 1));
    Field("grabbing", Yes(info.grabbing));
    Field("exposure_us", Fixed(cam.ExposureUs(), 1));
}

void ProbeKeithley(RigSession& s)
{
    HardwareManager& hw = s.Hw();
    if (!s.Config().FirstEnabledKeithley()) return;

    Head("keithley  " + hw.KeithleyName());
    Field("state", ConnStateName(hw.KeithleyState()));
    ISmu* k = hw.Keithley();
    if (!k) { Field("driver", "(none)"); return; }
    Field("model", k->ModelName());   // which ISmu config picked
    if (hw.KeithleyState() != ConnState::Connected) {
        const std::string err = k->LastError();
        Field("last_error", err.empty() ? "(none)" : err);
        return;
    }

    const SmuConfig cfg = k->GetConfig();
    Field("idn",   k->DeviceId());
    Field("visa",  cfg.visaResource);
    Field("nplc",  Fixed(cfg.nplc, 3));
    Field("range", cfg.autoRange ? std::string("auto") : Sci(cfg.fixedRange) + " A");
    Field("bias",  Fixed(k->BiasVolts(), 3) + " V (output " + (k->OutputOn() ? "on)" : "off)"));
    Field("polling", Yes(k->IsPolling()));
    Field("ch1", Latest(topics::kKeithleyCh1, "A", true));
    Field("ch2", Latest(topics::kKeithleyCh2, "A", true));
}

void ProbeCld(RigSession& s)
{
    HardwareManager& hw = s.Hw();
    if (!s.Config().FirstEnabledCld()) return;

    Head("cld101x  " + hw.CldName());
    Field("state", ConnStateName(hw.CldState()));
    CLD101x* c = hw.Cld();
    if (!c) { Field("driver", "(none)"); return; }
    if (hw.CldState() != ConnState::Connected) {
        const std::string err = c->LastError();
        Field("last_error", err.empty() ? "(none)" : err);
        return;
    }

    const CLD101x::Status st = c->GetStatus();
    Field("idn", c->DeviceId());
    Field("poll_data", st.valid ? "valid" : "(no successful poll yet)");
    Field("tec",   st.tecOn ? "ON" : "off");
    Field("laser", st.laserOn ? "ON" : "off");
    Field("temp", st.tempValid ? Fixed(st.tempC, 3) + " C"
                               : std::string("(sensor invalid)"));
    Field("temp_setpoint",  Fixed(st.tempSetC, 3) + " C");
    Field("laser_setpoint", Fixed(st.laserSetMA, 3) + " mA");
    Field("laser_actual",   Fixed(st.laserActMA, 3) + " mA");
    Field("stream temp",    Latest(topics::kCldTemp, "C", false));
    Field("stream current", Latest(topics::kCldCurrent, "mA", false));
}

void ProbePsu(RigSession& s)
{
    HardwareManager& hw = s.Hw();
    if (!s.Config().FirstEnabledPsu()) return;

    Head("psu  " + hw.PsuName());
    Field("state", ConnStateName(hw.PsuState()));
    UDP3305S* p = hw.Psu();
    if (!p) { Field("driver", "(none)"); return; }
    if (hw.PsuState() != ConnState::Connected) {
        const std::string err = p->LastError();
        Field("last_error", err.empty() ? "(none)" : err);
        return;
    }

    const UDP3305S::Status st = p->GetStatus();
    Field("idn", p->DeviceId());
    Field("poll_data", st.valid ? "valid" : "(no successful poll yet)");
    for (int ch = 1; ch <= p->Channels() && ch <= UDP3305S::kMaxChannels; ++ch) {
        const UDP3305S::ChannelStatus& c = st.ch[ch - 1];
        std::cout << "   CH" << ch << "  output=" << (c.outputOn ? "ON " : "off")
                  << "  set " << Fixed(c.setV, 3) << " V / " << Fixed(c.setI, 3) << " A"
                  << "  meas " << Fixed(c.measV, 3) << " V / " << Fixed(c.measI, 3) << " A\n";
        std::cout << "        stream " << Latest(topics::PsuVoltage(ch), "V", false)
                  << " / "             << Latest(topics::PsuCurrent(ch), "A", false) << "\n";
    }
}

// Read-back only. The driver's connect and poll send C and M (status queries)
// and nothing else; no F/L command is ever issued from here, so probing never
// changes what the lights are doing.
void ProbeLight(RigSession& s)
{
    HardwareManager& hw = s.Hw();
    if (!s.Config().FirstEnabledLight()) return;

    Head("light  " + hw.LightName());
    Field("state", ConnStateName(hw.LightState()));
    CcsPd3* l = hw.Light();
    if (!l) { Field("driver", "(none)"); return; }
    Field("device", l->DeviceId());
    if (hw.LightState() != ConnState::Connected) {
        const std::string err = l->LastError();
        Field("last_error", err.empty() ? "(none)" : err);
        return;
    }

    const CcsPd3::Status st = l->GetStatus();
    Field("link", st.linkUp ? "up" : "DOWN (reconnecting)");
    Field("poll_data", st.valid ? "valid" : "(no successful read yet)");
    Field("unit", st.unitError ? "ERROR: overcurrent / fan stop" : "normal");
    if (!st.warning.empty()) Field("warning", st.warning);
    for (const CcsPd3::ChannelStatus& c : st.ch) {
        const CcsPd3::Channel* cfg = l->FindChannel(c.channel);
        std::cout << "   L" << c.channel << "  " << std::left << std::setw(12)
                  << (cfg ? cfg->name : std::string("?")) << std::right
                  << (c.on ? "ON " : "off") << "  F" << std::setw(3) << std::setfill('0')
                  << c.intensity << std::setfill(' ') << "  ("
                  << Fixed(CcsPd3::NativeToPercent(c.intensity), 1) << " %)"
                  << "  mode S" << c.mode << "\n";
    }
}

} // namespace

void PrintStatus(const RigSession& s)
{
    const std::vector<HardwareManager::DeviceStatus> snap = s.Hw().Snapshot();
    std::cout << "\n";
    for (const auto& d : snap) {
        std::cout << "   " << std::left << std::setw(24) << d.name << std::right
                  << ConnStateName(d.state) << "\n";
    }
    std::cout << "\n   " << (s.AllExpectedConnected()
                             ? "all configured devices connected"
                             : "NOT every configured device is connected") << "\n";
}

namespace {

// HALCON is machine state rather than a configured device, so it is reported
// unconditionally: "does this PC have a valid vision licence?" is a bring-up
// question, and the rig is where bring-up questions get answered. Read-only,
// like every other probe here — it only asks HalconEngine what it found.
void ProbeVision()
{
    const HalconStatus& h = HalconEngine::Instance().Status();
    Head("vision  HALCON");
    Field("available",   Yes(h.Available()));
    Field("compiled_in", Yes(h.compiledIn));
    Field("runtime",     Yes(h.runtimeLoaded));
    Field("licensed",    Yes(h.licensed));
    Field("version",     h.version.empty() ? "(unknown)" : h.version);
    Field("root",        h.root.empty() ? "(unset)" : h.root);
    Field("license_dir", h.licenseDir.empty() ? "(unknown)" : h.licenseDir);
    if (h.licenseFiles.empty()) {
        Field("license_files", "(none)");
    } else {
        std::string joined;
        for (const std::string& f : h.licenseFiles) {
            if (!joined.empty()) joined += ", ";
            joined += f;
        }
        Field("license_files", joined);
    }
    if (!h.error.empty()) Field("error", h.error);
}

} // namespace

void PrintProbe(RigSession& s)
{
    // Motion drivers poll slowly unless someone is asking; ask, then let every
    // poll thread (motion, IO, instruments) produce at least one sample.
    HardwareManager& hw = s.Hw();
    for (std::size_t i = 0; i < hw.MotionCount(); ++i) {
        if (IMotionController* m = hw.MotionDevice(i)) m->RequestFastPoll();
    }
    s.Settle(kReadbackSettleSec);

    ProbeMotion(s);
    ProbeIo(s);
    ProbeSlides(s);
    ProbeCamera(s);
    ProbeKeithley(s);
    ProbeCld(s);
    ProbePsu(s);
    ProbeLight(s);
    ProbeVision();
    std::cout << "\n";
}

} // namespace firedog::rig
