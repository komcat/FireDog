// CommandsInstruments.cpp : Keithley / CLD101x / Arroyo TEC / PSU / camera commands.
//
// Thin wrappers over the Workflow instrument methods (which are read-back
// verified and thread-safe against the drivers' poll loops). wait_for_
// temperature watches the CLD's published DataBus stream.
#include "command/Commands.h"

#include "command/CommandRegistry.h"
#include "command/ExecutionContext.h"
#include "core/DataBus.h"
#include "core/LightCallout.h"
#include "core/Log.h"
#include "core/Topics.h"
#include "hardware/instruments/ArroyoTec.h"
#include "hardware/instruments/UDP3305S.h"
#include "workflow/Workflow.h"

#include <cmath>
#include <cstdio>
#include <initializer_list>
#include <string>

namespace firedog {

namespace {

CommandResult BoolResult(bool ok, const char* what)
{
    return ok ? CommandResult::Success()
              : CommandResult::Fail(std::string(what) + " rejected (not connected?)");
}

// Latest published sample of `topic` into ctx[key]; fails when the stream has
// no data yet (instrument connected but not polled, or polling stopped).
CommandResult ReadTopicToCtx(ExecutionContext& ctx, const std::string& topic,
                             const std::string& key)
{
    DataStream& s = DataBus::Instance().Stream(topic);
    if (s.Count() == 0)
        return CommandResult::Fail("no data on topic '" + topic + "'");
    ctx.CtxSet(key, s.LatestValue());
    return CommandResult::Success();
}

CommandResult ExecWaitForTemperature(ExecutionContext& ctx, const ParamValues& p)
{
    const double target = p.GetDouble("target_c");
    const double tol    = p.GetDouble("tol_c");
    DataStream&  s      = DataBus::Instance().Stream(topics::kCldTemp);
    const auto r = ctx.WaitUntil(
        [&] { return s.Count() > 0 && std::fabs(s.LatestValue() - target) <= tol; },
        p.GetInt("timeout_ms"), 100);
    if (r == ExecutionContext::WaitResult::Ok) return CommandResult::Success();
    return CommandResult::Fail(r == ExecutionContext::WaitResult::Stopped
                                   ? "stopped" : "timeout waiting for temperature");
}

// The faults that keep a TEC output from staying on, as words; "" for none.
std::string TecFaults(const ArroyoTec::Status& s)
{
    std::string f = ArroyoTec::CondText(s.cond & (ArroyoTec::kCondSensorShort | ArroyoTec::kCondSensorOpen |
                                                  ArroyoTec::kCondTecOpen | ArroyoTec::kCondRunaway |
                                                  ArroyoTec::kCondTHighLimit | ArroyoTec::kCondTLowLimit));
    if (!s.lastUnitError.empty()) f += (f.empty() ? "" : "; ") + ("last controller error " + s.lastUnitError);
    return f;
}

// tec_startup: set point + output ON + a quick check that the TEC is really
// driving, as ONE step - with the operator in the loop when it is not.
//
// One attempt: read the start temperature (a fresh Measure, not the poll cache)
// -> TEC:T -> TEC:OUT 1 (read back; an ON the unit refuses or drops comes with
// its reason) -> wait verify_ms -> read again and judge:
//   - the output is still ON, and no sensor / module / run-away fault;
//   - not in VOLTAGE limit: an open TEC cable has no current path, so the
//     drive runs up against its compliance voltage;
//   - not in CURRENT limit, when fail_on_current_limit (default on). Note: a
//     big step (25 -> 60 C) can legitimately drive at the limit for a while;
//     untick it for such a set point;
//   - when the start was NOT already within the controller's tolerance of the
//     set point: |I| >= min_current_a (no current = nothing connected), and
//     the temperature moved at least min_temp_change_c from the start (0 =
//     skip). Already at temperature, neither is expected, so neither is asked.
// A failed attempt switches the output back OFF (off_on_fail) - an unverified
// TEC is worse than an off one - and asks the operator in the prompt window:
// Retry (the whole attempt again) or Cancel, for at most prompt_timeout_ms
// (default 30 min). Cancel, the timeout or Stop fail the step.
//
// Writes <prefix>_ok always, and <prefix>_start_c / _temp_c / _current_a /
// _voltage_v from the last attempt that got as far as measuring.
CommandResult ExecTecStartup(ExecutionContext& ctx, const ParamValues& p)
{
    Workflow&         wf       = ctx.Wf();
    const std::string dev      = p.GetString("device");   // "" = the first TEC
    const double      setC     = p.GetDouble("celsius");
    const int         verifyMs = p.GetInt("verify_ms");
    const double      minI     = p.GetDouble("min_current_a");
    const double      minDT    = p.GetDouble("min_temp_change_c");
    const bool        iLimFail = p.GetBool("fail_on_current_limit");
    const bool        offOnFail = p.GetBool("off_on_fail");
    const int         promptMs = p.GetInt("prompt_timeout_ms");
    const std::string prefix   = p.GetString("ctx_prefix");
    ctx.CtxSet(prefix + "_ok", false);

    enum class Try { Ok, Failed, Stopped };
    char buf[320];
    const auto attempt = [&](std::string& why) -> Try {
        ArroyoTec* tec = wf.ConnectedTec(dev);
        if (!tec) { why = wf.TecLabel(dev) + " is not connected (Hardware > Arroyo TEC > Connect)"; return Try::Failed; }

        ArroyoTec::Reading r0;
        if (!tec->Measure(r0)) { why = "the TEC does not answer: " + tec->LastError(); return Try::Failed; }
        if (!r0.tempValid) {
            why = "no valid temperature before switching on (" +
                  ArroyoTec::CondText(r0.cond & (ArroyoTec::kCondSensorShort | ArroyoTec::kCondSensorOpen)) +
                  ") - check the thermistor cable";
            return Try::Failed;
        }
        ctx.CtxSet(prefix + "_start_c", r0.tempC);

        if (!wf.SetTecTemperature(dev, setC, &why)) { why = "set point refused: " + why; return Try::Failed; }
        bool switchedOn = false;
        const auto fail = [&](const std::string& w) {
            why = w;
            if (offOnFail && switchedOn) wf.SetTecOutput(dev, false);
            return Try::Failed;
        };
        if (!wf.SetTecOutput(dev, true, &why)) return fail("output did not switch ON: " + why);
        switchedOn = true;

        if (!ctx.Sleep(verifyMs)) {
            if (offOnFail) wf.SetTecOutput(dev, false);
            return Try::Stopped;
        }

        ArroyoTec::Reading r1;
        if (!tec->Measure(r1)) return fail("the TEC stopped answering: " + tec->LastError());
        ctx.CtxSet(prefix + "_temp_c", r1.tempC);
        ctx.CtxSet(prefix + "_current_a", r1.currentA);
        ctx.CtxSet(prefix + "_voltage_v", r1.voltageV);
        const std::string unit = r1.unitErrors.empty() ? "" : " - controller: " + r1.unitErrors;

        if (!(r1.cond & ArroyoTec::kCondOutputOn))
            return fail("the output dropped OFF within " + std::to_string(verifyMs) + " ms" + unit);
        const unsigned faults = r1.cond & (ArroyoTec::kCondSensorShort | ArroyoTec::kCondSensorOpen |
                                           ArroyoTec::kCondTecOpen | ArroyoTec::kCondRunaway);
        if (faults) return fail(ArroyoTec::CondText(faults) + unit);
        if (r1.cond & ArroyoTec::kCondVoltageLimit) {
            std::snprintf(buf, sizeof(buf), "drive at its VOLTAGE limit (%.2f V, %.2f A) - TEC cable not "
                          "connected or the module open?", r1.voltageV, r1.currentA);
            return fail(buf + unit);
        }
        if (iLimFail && (r1.cond & ArroyoTec::kCondCurrentLimit)) {
            std::snprintf(buf, sizeof(buf), "drive at its CURRENT limit (%.2f A, %.2f V) - check the TEC "
                          "cable (a big temperature step can also do this: untick fail_on_current_limit)",
                          r1.currentA, r1.voltageV);
            return fail(buf + unit);
        }
        const double tolC = tec->GetStatus().tolC > 0.0 ? tec->GetStatus().tolC : 0.1;
        if (std::fabs(r0.tempC - setC) > tolC) {
            if (std::fabs(r1.currentA) < minI) {
                std::snprintf(buf, sizeof(buf), "no TEC current (%.3f A, want at least %.3f A) - TEC cable "
                              "not connected?", r1.currentA, minI);
                return fail(buf + unit);
            }
            if (minDT > 0.0 && std::fabs(r1.tempC - r0.tempC) < minDT) {
                std::snprintf(buf, sizeof(buf), "temperature did not move (%.3f -> %.3f C in %d ms, want "
                              "%.3f C) - is the sensor on the TEC it is driving?",
                              r0.tempC, r1.tempC, verifyMs, minDT);
                return fail(buf + unit);
            }
        }
        std::snprintf(buf, sizeof(buf), "set %.2f C, ON: %.3f -> %.3f C, %.3f A, %.2f V after %d ms",
                      setC, r0.tempC, r1.tempC, r1.currentA, r1.voltageV, verifyMs);
        FD_INFO("tec", "%s startup ok: %s", wf.TecLabel(dev).c_str(), buf);
        return Try::Ok;
    };

    for (int n = 1;; ++n) {
        std::string why;
        const Try t = attempt(why);
        if (t == Try::Ok) {
            ctx.CtxSet(prefix + "_ok", true);
            return CommandResult::Success();
        }
        if (t == Try::Stopped) return CommandResult::Fail("stopped");

        FD_WARN("tec", "%s startup attempt %d failed: %s", wf.TecLabel(dev).c_str(), n, why.c_str());
        const std::string msg = wf.TecLabel(dev) + " startup failed (attempt " + std::to_string(n) +
                                "):\n\n" + why +
                                (offOnFail ? "\n\nThe TEC output has been switched OFF." : "") +
                                "\n\nFix the hardware (TEC and sensor cables, controller), then press "
                                "Retry - or Cancel to stop this step.";
        const int choice = ctx.AwaitChoice(msg, {"Retry", "Cancel"}, PromptLevel::Error, promptMs);
        if (choice == 0) continue;
        if (choice == ExecutionContext::kChoiceTimedOut)
            return CommandResult::Fail("tec_startup: nobody answered within " +
                                       std::to_string(promptMs / 60000) + " min - " + why);
        if (choice == 1) return CommandResult::Fail("tec_startup: cancelled by the operator - " + why);
        return CommandResult::Fail("stopped");
    }
}

// tec_wait_stable: until the CONTROLLER says stable - output on and in its own
// tolerance window (TEC:TOL, e.g. 0.1 C held 5 s). That is the front panel's
// Stable LED, so the step and the instrument cannot disagree about "settled".
// Only a poll that began after the wait started counts: the cached status may
// still describe the set point before a tec_set_temperature one step earlier.
// An output that is (or drops) OFF fails at once - waiting the whole timeout
// for a tripped controller only hides the trip.
CommandResult ExecTecWaitStable(ExecutionContext& ctx, const ParamValues& p)
{
    const std::string dev = p.GetString("device");   // "" = the first TEC
    ArroyoTec* tec = ctx.Wf().ConnectedTec(dev);
    if (!tec) return CommandResult::Fail("tec_wait_stable: " + ctx.Wf().TecLabel(dev) + " is not connected");
    const unsigned long long seq0 = tec->GetStatus().pollSeq;
    std::string why;
    ArroyoTec::Status last;
    const auto r = ctx.WaitUntil([&] {
        ArroyoTec* t = ctx.Wf().ConnectedTec(dev);
        if (!t) { why = ctx.Wf().TecLabel(dev) + " disconnected"; return true; }
        last = t->GetStatus();
        if (last.pollSeq < seq0 + 2) return false;
        if (!last.outputOn) {
            const std::string f = TecFaults(last);
            why = "the TEC output is OFF" + (f.empty() ? std::string() : " (" + f + ")");
            return true;
        }
        return last.stable;
    }, p.GetInt("timeout_ms"), 100);

    if (r == ExecutionContext::WaitResult::Stopped) return CommandResult::Fail("stopped");
    if (!why.empty()) return CommandResult::Fail("tec_wait_stable: " + why);
    char buf[200];
    if (r == ExecutionContext::WaitResult::Timeout) {
        if (last.tempValid)
            std::snprintf(buf, sizeof(buf), "tec_wait_stable: not stable after %d ms (%.3f C, set %.3f C, window +/-%.2f C for %.1f s)",
                          p.GetInt("timeout_ms"), last.tempC, last.setC, last.tolC, last.tolS);
        else
            std::snprintf(buf, sizeof(buf), "tec_wait_stable: not stable after %d ms (no valid temperature)",
                          p.GetInt("timeout_ms"));
        return CommandResult::Fail(buf);
    }
    FD_INFO("tec", "stable at %.3f C (set %.3f C)", last.tempC, last.setC);
    return CommandResult::Success();
}

// psu_startup: the verified power-up (Horizon's psu.powerUp, with a CV/CC mode).
//
// The UDP3305S has no mode switch - the LOAD decides which quantity it
// regulates. So `mode` says which number is the TARGET and which reading is
// checked:
//   CV  volts = setpoint, amps = current LIMIT  -> verify the measured voltage
//   CC  amps  = setpoint, volts = voltage LIMIT -> verify the measured current
// Both program the same APPLy V,I. A CV start whose current sits at the limit
// did not reach its voltage (the load pulled it into CC); a CC start whose
// current falls short hit its voltage limit (the load cannot draw that much).
//
// Sequence: connection verified by a LIVE read (not the poll cache) -> V/I ->
// output ON (read-back verified) -> settle -> verify, re-reading every 100 ms
// up to verify_ms: one early read is not enough (Horizon saw 0.000 V / 0.300 A
// 150 ms after a good ON). Any failure after ON switches the output back OFF
// unless off_on_fail is cleared - a half-started supply is the one state worse
// than either outcome.
CommandResult ExecPsuStartup(ExecutionContext& ctx, const ParamValues& p)
{
    Workflow&         wf     = ctx.Wf();
    const std::string dev    = p.GetString("device");   // "" = the first PSU
    const int         ch     = p.GetInt("channel");
    const bool        cc     = p.GetString("mode") == "CC";
    const double      volts  = p.GetDouble("volts");
    const double      amps   = p.GetDouble("amps");
    const double      tolV   = p.GetDouble("tol_v");
    const double      tolA   = p.GetDouble("tol_a");
    const std::string prefix = p.GetString("ctx_prefix");
    const char*       mode   = cc ? "CC" : "CV";
    char buf[320];

    ctx.CtxSet(prefix + "_ok", false);

    // Datasheet limits: CH1/CH2 0..32 V / 0..5 A, CH3 0..6 V / 0..3 A. Refused
    // before anything is sent.
    const std::string who = dev.empty() ? std::string("the first PSU") : "PSU '" + dev + "'";
    if (ch > wf.PsuChannels(dev))
        return CommandResult::Fail("psu_startup: CH" + std::to_string(ch) + " is not configured on " + who);
    const double maxV = (ch == 3) ? 6.0 : 32.0;
    const double maxA = (ch == 3) ? 3.0 : 5.0;
    if (volts > maxV || amps > maxA) {
        std::snprintf(buf, sizeof(buf), "psu_startup: CH%d takes at most %.0f V / %.0f A", ch, maxV, maxA);
        return CommandResult::Fail(buf);
    }

    // 1. Connection, verified by a live answer.
    double v = 0.0, i = 0.0;
    bool   on = false;
    if (!wf.MeasurePsu(dev, ch, v, i, on))
        return CommandResult::Fail("psu_startup: " + who + " does not answer (connected?)");

    // 2. Mode -> which number is the target; one APPLy either way.
    if (!wf.SetPsuVI(dev, ch, volts, amps))
        return CommandResult::Fail("psu_startup: V/I setpoint rejected by " + who);
    std::snprintf(buf, sizeof(buf), "%s CH%d %s: %.3f V %s / %.3f A %s", who.c_str(), ch, mode,
                  volts, cc ? "limit" : "set", amps, cc ? "set" : "limit");
    FD_INFO("psu", "startup %s", buf);

    // 3. Output ON (the driver reads OUTPut? back).
    const bool offOnFail = p.GetBool("off_on_fail");
    const auto failOff = [&](const std::string& why) {
        if (offOnFail) wf.PsuOutputOff(dev, ch);
        FD_WARN("psu", "startup %s CH%d %s failed: %s%s", who.c_str(), ch, mode, why.c_str(),
                offOnFail ? " - output switched OFF" : "");
        return CommandResult::Fail("psu_startup: " + why);
    };
    if (!wf.PsuOutputOn(dev, ch)) return failOff("output did not switch ON");

    // 4. Settle.
    if (!ctx.Sleep(p.GetInt("settle_ms"))) return failOff("stopped");

    // 5. Verify, re-reading until it passes or verify_ms runs out.
    bool measured = false;
    const auto pass = [&] {
        measured = wf.MeasurePsu(dev, ch, v, i, on);
        if (!measured || !on) return false;
        if (cc) return std::fabs(i - amps) <= tolA;
        return std::fabs(v - volts) <= tolV && !(amps > 0.0 && i >= 0.98 * amps);
    };
    const auto r = ctx.WaitUntil(pass, p.GetInt("verify_ms"), 100);
    if (r == ExecutionContext::WaitResult::Stopped) return failOff("stopped");

    ctx.CtxSet(prefix + "_v", v);
    ctx.CtxSet(prefix + "_i", i);
    const bool limited = (amps > 0.0 && i >= 0.98 * amps);
    // What it IS doing, as measured - not the mode that was asked for.
    ctx.CtxSet(prefix + "_mode", std::string(!on ? "OFF" : limited ? "CC" : "CV"));

    if (r != ExecutionContext::WaitResult::Ok) {
        if (!measured) return failOff("the PSU stopped answering during verify");
        if (!on)       return failOff("the output reads OFF");
        if (cc)
            std::snprintf(buf, sizeof(buf),
                          "CH%d CC: read %.4f A at %.3f V, want %.4f +/- %.4f A%s", ch, i, v,
                          amps, tolA,
                          v >= volts - tolV ? " - at the voltage limit: the load cannot draw "
                                              "that current (it is in CV)" : "");
        else
            std::snprintf(buf, sizeof(buf),
                          "CH%d CV: read %.3f V at %.4f A, want %.3f +/- %.3f V%s", ch, v, i,
                          volts, tolV,
                          limited ? " - at the current limit: the load pulled it into CC" : "");
        return failOff(buf);
    }

    ctx.CtxSet(prefix + "_ok", true);
    FD_INFO("psu", "startup %s CH%d %s verified: %.3f V / %.4f A", who.c_str(), ch, mode, v, i);
    return CommandResult::Success();
}

// Opt the named params into per-VARIANT values (ParamSpec::variant): a step
// can then carry a value per product model, applied at run start
// (ApplyStepVariants). Only what is a property of the PART - a supply's volts
// for this model - never which device or channel it drives.
CommandSpec WithVariantParams(CommandSpec spec, std::initializer_list<const char*> names)
{
    for (ParamSpec& ps : spec.params)
        for (const char* n : names)
            if (ps.name == n) ps.variant = true;
    return spec;
}

} // namespace

void RegisterInstrumentCommands(CommandRegistry& reg, Workflow& wf, HardwareManager& hw)
{
    (void)hw;
    using PT    = ParamType;
    using Needs = CommandSpec::Needs;
    Workflow* w = &wf;

    // ---- Keithley 6482 ------------------------------------------------------
    reg.Register({"keithley_bias", "Instruments", "Set the Keithley ch1 bias voltage",
                  {
                      {"volts", PT::Double, true, {}, -30.0, 30.0, {}, "bias voltage (V)"},
                  },
                  Needs::Keithley,
                  [w](ExecutionContext&, const ParamValues& p) {
                      return BoolResult(w->SetKeithleyBias(p.GetDouble("volts")),
                                        "keithley bias");
                  }});
    reg.Register({"keithley_output_off", "Instruments", "Switch the Keithley output OFF",
                  {}, Needs::Keithley,
                  [w](ExecutionContext&, const ParamValues&) {
                      return BoolResult(w->KeithleyOutputOff(), "keithley output off");
                  }});
    reg.Register({"keithley_nplc", "Instruments", "Set the Keithley integration time (NPLC)",
                  {
                      {"nplc", PT::Double, true, {}, 0.01, 10.0, {}, "integration NPLC"},
                  },
                  Needs::Keithley,
                  [w](ExecutionContext&, const ParamValues& p) {
                      return BoolResult(w->SetKeithleyNplc(p.GetDouble("nplc")),
                                        "keithley nplc");
                  }});
    reg.Register({"keithley_range_auto", "Instruments", "Keithley current range: auto",
                  {}, Needs::Keithley,
                  [w](ExecutionContext&, const ParamValues&) {
                      return BoolResult(w->SetKeithleyRangeAuto(), "keithley range");
                  }});
    reg.Register({"keithley_range_fixed", "Instruments", "Keithley current range: fixed",
                  {
                      {"amps", PT::Double, true, {}, 0, 0, {}, "range full-scale (A)"},
                  },
                  Needs::Keithley,
                  [w](ExecutionContext&, const ParamValues& p) {
                      return BoolResult(w->SetKeithleyRangeFixed(p.GetDouble("amps")),
                                        "keithley range");
                  }});
    reg.Register({"keithley_read", "Instruments",
                  "Read a Keithley channel current (A) into the run context",
                  {
                      {"channel", PT::Int, false, 1, 1, 2, {}, "channel 1..2"},
                      {"ctx", PT::String, true, {}, 0, 0, {}, "destination ctx key"},
                  },
                  Needs::Keithley,
                  [](ExecutionContext& c, const ParamValues& p) {
                      return ReadTopicToCtx(c,
                                            p.GetInt("channel") == 2 ? topics::kKeithleyCh2
                                                                     : topics::kKeithleyCh1,
                                            p.GetString("ctx"));
                  }});

    // ---- CLD101x laser / TEC ------------------------------------------------
    reg.Register({"cld_tec", "Instruments", "Switch the CLD101x TEC on/off",
                  {
                      {"on", PT::Bool, true, {}, 0, 0, {}, "true = ON"},
                  },
                  Needs::Cld,
                  [w](ExecutionContext&, const ParamValues& p) {
                      return BoolResult(p.GetBool("on") ? w->CldTecOn() : w->CldTecOff(),
                                        "cld tec");
                  }});
    reg.Register({"cld_laser", "Instruments", "Switch the CLD101x laser on/off",
                  {
                      {"on", PT::Bool, true, {}, 0, 0, {}, "true = ON"},
                  },
                  Needs::Cld,
                  [w](ExecutionContext&, const ParamValues& p) {
                      return BoolResult(p.GetBool("on") ? w->CldLaserOn() : w->CldLaserOff(),
                                        "cld laser");
                  }});
    reg.Register({"cld_set_temperature", "Instruments", "Set the CLD101x TEC setpoint",
                  {
                      {"celsius", PT::Double, true, {}, 10.0, 40.0, {}, "TEC setpoint (deg C)"},
                  },
                  Needs::Cld,
                  [w](ExecutionContext&, const ParamValues& p) {
                      return BoolResult(w->SetCldTemperature(p.GetDouble("celsius")),
                                        "cld temperature");
                  }});
    reg.Register({"cld_set_laser_current", "Instruments", "Set the CLD101x laser current",
                  {
                      {"ma", PT::Double, true, {}, 0.0, 1500.0, {}, "laser current (mA)"},
                  },
                  Needs::Cld,
                  [w](ExecutionContext&, const ParamValues& p) {
                      return BoolResult(w->SetCldLaserCurrentMA(p.GetDouble("ma")),
                                        "cld laser current");
                  }});
    reg.Register({"cld_read_temperature", "Instruments",
                  "Read the CLD101x temperature (deg C) into the run context",
                  {
                      {"ctx", PT::String, true, {}, 0, 0, {}, "destination ctx key"},
                  },
                  Needs::Cld,
                  [](ExecutionContext& c, const ParamValues& p) {
                      return ReadTopicToCtx(c, topics::kCldTemp, p.GetString("ctx"));
                  }});
    reg.Register({"cld_read_laser_current", "Instruments",
                  "Read the CLD101x laser current (mA) into the run context",
                  {
                      {"ctx", PT::String, true, {}, 0, 0, {}, "destination ctx key"},
                  },
                  Needs::Cld,
                  [](ExecutionContext& c, const ParamValues& p) {
                      return ReadTopicToCtx(c, topics::kCldCurrent, p.GetString("ctx"));
                  }});
    reg.Register({"wait_for_temperature", "Instruments",
                  "Wait until the CLD101x temperature settles at a target",
                  {
                      {"target_c", PT::Double, true, {}, 10.0, 40.0, {}, "target (deg C)"},
                      {"tol_c", PT::Double, false, 0.5, 0.01, 10.0, {}, "tolerance"},
                      {"timeout_ms", PT::Int, false, 60000, 100, 600000, {}, "wait timeout"},
                  },
                  Needs::Cld, ExecWaitForTemperature});

    // ---- Arroyo TECSource -----------------------------------------------------
    // The controller's own limits (TLO..THI, read at connect) refuse a set
    // point before it is sent; the -99..250 bounds here are only the unit's range.
    // None of these is a teach mover, a parallel mover or dry-run allowed.
    // Every tec_* step names its TEC with `device` (its name in hardware.json),
    // listed first; empty = the FIRST configured TEC - the PSU fleet's rule.
    const auto tecDeviceParam = [] {
        return ParamSpec{"device", ParamType::DeviceTec, false, "", 0, 0, {},
                         "TEC controller (its name in hardware.json); empty = the first one"};
    };
    {
        CommandSpec s{"tec_startup", "Instruments",
                      "TEC startup: set point, output ON, check it is driving; on a failure "
                      "the operator fixes the hardware and Retries (or Cancels)",
                      {
                          tecDeviceParam(),
                          {"celsius", PT::Double, false, 25.0, -99.0, 250.0, {}, "set point (deg C)"},
                          {"verify_ms", PT::Int, false, 500, 100, 60000, {},
                           "how long after ON the drive is checked"},
                          {"min_current_a", PT::Double, false, 0.05, 0.0, 30.0, {},
                           "at least this much TEC current (A) when not already at the set point"},
                          {"min_temp_change_c", PT::Double, false, 0.01, 0.0, 10.0, {},
                           "the temperature must move this much from the start (deg C); 0 = skip"},
                          {"fail_on_current_limit", PT::Bool, false, true, 0, 0, {},
                           "a drive at its current limit fails (untick for a big temperature step)"},
                          {"off_on_fail", PT::Bool, false, true, 0, 0, {},
                           "switch the output OFF when an attempt fails"},
                          {"prompt_timeout_ms", PT::Int, false, 1800000, 10000, 86400000, {},
                           "how long the Retry / Cancel prompt waits (default 30 min)"},
                          {"ctx_prefix", PT::String, false, "tec", 0, 0, {}, "ctx key prefix"},
                      },
                      Needs::Tec, ExecTecStartup};
        s.outputs = [](const nlohmann::json& p) {
            const std::string px = p.value("ctx_prefix", std::string("tec"));
            CommandOutputs o;
            o.keys.push_back({px + "_ok", "bool", "true when the TEC started and was seen driving"});
            o.keys.push_back({px + "_start_c", "number", "deg C before switching on"});
            o.keys.push_back({px + "_temp_c", "number", "deg C verify_ms after ON"});
            o.keys.push_back({px + "_current_a", "number", "TEC current (A) verify_ms after ON"});
            o.keys.push_back({px + "_voltage_v", "number", "TEC voltage (V) verify_ms after ON"});
            return o;
        };
        reg.Register(std::move(s));
    }
    reg.Register({"tec_set_temperature", "Instruments", "Set the Arroyo TEC temperature set point",
                  {
                      tecDeviceParam(),
                      {"celsius", PT::Double, true, {}, -99.0, 250.0, {}, "set point (deg C)"},
                  },
                  Needs::Tec,
                  [w](ExecutionContext&, const ParamValues& p) {
                      std::string why;
                      if (w->SetTecTemperature(p.GetString("device"), p.GetDouble("celsius"), &why))
                          return CommandResult::Success();
                      return CommandResult::Fail("tec_set_temperature: " + why);
                  }});
    reg.Register({"tec_output", "Instruments", "Switch the Arroyo TEC output on/off",
                  {
                      tecDeviceParam(),
                      {"on", PT::Bool, true, {}, 0, 0, {}, "true = ON"},
                  },
                  Needs::Tec,
                  [w](ExecutionContext&, const ParamValues& p) {
                      std::string why;
                      if (w->SetTecOutput(p.GetString("device"), p.GetBool("on"), &why))
                          return CommandResult::Success();
                      return CommandResult::Fail("tec_output: " + why);
                  }});
    reg.Register({"tec_wait_stable", "Instruments",
                  "Wait until the Arroyo TEC reports stable (its own tolerance window)",
                  {
                      tecDeviceParam(),
                      {"timeout_ms", PT::Int, false, 300000, 1000, 3600000, {}, "wait timeout"},
                  },
                  Needs::Tec, ExecTecWaitStable});
    reg.Register({"tec_read_temperature", "Instruments",
                  "Read the Arroyo TEC temperature (deg C) into the run context",
                  {
                      tecDeviceParam(),
                      {"ctx", PT::String, true, {}, 0, 0, {}, "destination ctx key"},
                  },
                  Needs::Tec,
                  [w](ExecutionContext& c, const ParamValues& p) {
                      // From the driver's status, not the topic: a shorted
                      // sensor publishes nothing, and the topic's latest value
                      // would be an old reading passed off as a new one.
                      const std::string dev = p.GetString("device");
                      ArroyoTec* t = w->ConnectedTec(dev);
                      if (!t) return CommandResult::Fail("tec_read_temperature: " + w->TecLabel(dev) +
                                                         " is not connected");
                      const ArroyoTec::Status s = t->GetStatus();
                      if (!s.valid) return CommandResult::Fail("tec_read_temperature: no reading yet");
                      if (!s.tempValid) {
                          const std::string f = ArroyoTec::CondText(s.cond & (ArroyoTec::kCondSensorShort |
                                                                               ArroyoTec::kCondSensorOpen));
                          return CommandResult::Fail("tec_read_temperature: no valid temperature" +
                                                     (f.empty() ? std::string() : " (" + f + ")"));
                      }
                      c.CtxSet(p.GetString("ctx"), s.tempC);
                      return CommandResult::Success();
                  }});

    // ---- UDP3305S PSU fleet --------------------------------------------------
    // Every psu_* step names its supply with `device` (the PSU's name in
    // hardware.json). Empty = the FIRST configured PSU, which is what every step
    // saved before there were several meant, so those keep working unchanged.
    // Listed first so the editor shows WHICH supply before what it does to it.
    const auto psuDeviceParam = [] {
        return ParamSpec{"device", ParamType::DevicePsu, false, "", 0, 0, {},
                         "power supply (its name in hardware.json); empty = the first one"};
    };
    reg.Register(WithVariantParams({"psu_set_vi", "Instruments", "Set a PSU channel's V/I setpoints",
                  {
                      psuDeviceParam(),
                      {"channel", PT::Int, true, {}, 1, 3, {}, "channel 1..3"},
                      {"volts", PT::Double, true, {}, 0.0, 32.0, {}, "voltage setpoint"},
                      {"amps", PT::Double, true, {}, 0.0, 5.0, {}, "current limit"},
                  },
                  Needs::Psu,
                  [w](ExecutionContext&, const ParamValues& p) {
                      return BoolResult(w->SetPsuVI(p.GetString("device"), p.GetInt("channel"),
                                                    p.GetDouble("volts"),
                                                    p.GetDouble("amps")),
                                        "psu setpoint");
                  }}, {"volts", "amps"}));
    reg.Register({"psu_output", "Instruments", "Switch a PSU channel output on/off",
                  {
                      psuDeviceParam(),
                      {"channel", PT::Int, true, {}, 1, 3, {}, "channel 1..3"},
                      {"on", PT::Bool, true, {}, 0, 0, {}, "true = ON"},
                  },
                  Needs::Psu,
                  [w](ExecutionContext&, const ParamValues& p) {
                      const std::string dev = p.GetString("device");
                      const int ch = p.GetInt("channel");
                      return BoolResult(p.GetBool("on") ? w->PsuOutputOn(dev, ch)
                                                        : w->PsuOutputOff(dev, ch),
                                        "psu output");
                  }});

    reg.Register({"psu_read", "Instruments",
                  "Read a PSU channel's measured voltage (V), current (A) or mode (CV/CC) into the run context",
                  {
                      psuDeviceParam(),
                      {"channel", PT::Int, true, {}, 1, 3, {}, "channel 1..3"},
                      {"what", PT::Enum, false, "voltage", 0, 0, {"voltage", "current", "mode"},
                       "which measurement; mode = \"CV\" or \"CC\" (what the load is making it do)"},
                      {"ctx", PT::String, true, {}, 0, 0, {}, "destination ctx key"},
                  },
                  Needs::Psu,
                  [w](ExecutionContext& c, const ParamValues& p) {
                      const std::string dev = p.GetString("device");
                      const int ch = p.GetInt("channel");
                      const std::string what = p.GetString("what");
                      if (what == "mode") {
                          // A fresh read, not the poll cache: a mode read right
                          // after a step must describe the output as it is now.
                          double v = 0.0, i = 0.0;
                          bool on = false;
                          if (!w->MeasurePsu(dev, ch, v, i, on))
                              return CommandResult::Fail("psu_read: the PSU does not answer");
                          UDP3305S::ChannelStatus s;
                          s.outputOn = on;
                          s.measI    = i;
                          s.setI     = w->PsuSetCurrent(dev, ch);
                          c.CtxSet(p.GetString("ctx"),
                                   std::string(!on ? "OFF" : UDP3305S::IsCurrentLimited(s) ? "CC" : "CV"));
                          return CommandResult::Success();
                      }
                      // The named supply's own stream; "" = the first, whose
                      // device-less stream is the same samples.
                      const bool cur = (what == "current");
                      const std::string topic =
                          dev.empty() ? (cur ? topics::PsuCurrent(ch) : topics::PsuVoltage(ch))
                                      : (cur ? topics::PsuCurrent(dev, ch) : topics::PsuVoltage(dev, ch));
                      return ReadTopicToCtx(c, topic, p.GetString("ctx"));
                  }});
    reg.Register(WithVariantParams({"psu_startup", "Instruments",
                  "Verified PSU start-up: check link, set CV/CC target + limit, output ON, settle, verify",
                  {
                      psuDeviceParam(),
                      {"channel", PT::Int, false, 1, 1, 3, {}, "channel 1..3"},
                      {"mode", PT::Enum, false, "CV", 0, 0, {"CV", "CC"},
                       "CV: hold the voltage (current is the limit), verify V. "
                       "CC: hold the current (voltage is the limit), verify I"},
                      {"volts", PT::Double, false, 3.3, 0.0, 32.0, {},
                       "CV: voltage setpoint. CC: voltage limit (CH3 max 6 V)"},
                      {"amps", PT::Double, false, 0.5, 0.0, 5.0, {},
                       "CV: current limit. CC: current setpoint (CH3 max 3 A)"},
                      {"settle_ms", PT::Int, false, 300, 0, 10000, {},
                       "wait after output ON before the first verify read"},
                      {"verify_ms", PT::Int, false, 2000, 100, 10000, {},
                       "keep re-reading (every 100 ms) for this long before failing"},
                      {"tol_v", PT::Double, false, 0.1, 0.001, 5.0, {}, "CV: allowed voltage error (V)"},
                      {"tol_a", PT::Double, false, 0.02, 0.0001, 1.0, {}, "CC: allowed current error (A)"},
                      {"off_on_fail", PT::Bool, false, true, 0, 0, {},
                       "switch the output back OFF if the start-up does not verify"},
                      {"ctx_prefix", PT::String, false, "psu", 0, 0, {},
                       "results: <prefix>_ok, _v, _i, _mode (CV/CC as measured)"},
                  },
                  Needs::Psu, ExecPsuStartup}, {"volts", "amps", "tol_v", "tol_a"}));

    // ---- CCS PD3 lighting ---------------------------------------------------
    // Percent of full scale everywhere; 0 % is OUTPUT OFF (F000 still glows).
    // A light is a channel group from hardware.json ("main_camera_light"), or an
    // ungrouped channel's name. light_color and light_off FAIL when the light
    // or the call-out matches no channel: a step that lit nothing must not
    // read as one that worked.
    std::vector<std::string> colorOptions = LightColorNames();
    reg.Register({"light_set", "Instruments", "Set one lighting channel's intensity (0 % = off)",
                  {
                      {"channel", PT::Int, true, {}, 1, 8, {}, "front-panel channel L1..L8"},
                      {"percent", PT::Double, true, {}, 0.0, 100.0, {}, "intensity (%); 0 = off"},
                  },
                  Needs::Light,
                  [w](ExecutionContext&, const ParamValues& p) {
                      return BoolResult(w->SetLightChannel(p.GetInt("channel"),
                                                           p.GetDouble("percent")),
                                        "light set");
                  }});
    reg.Register({"light_color", "Instruments",
                  "Light a color call-out on one light; its other channels go dark",
                  {
                      {"light", PT::LightName, false, "main_camera_light", 0, 0, {},
                       "light: a channel group, or an ungrouped channel's name"},
                      {"color", PT::Enum, true, {}, 0, 0, colorOptions,
                       "white = red+green+blue; white_collimate adds the collimated light"},
                      {"percent", PT::Double, true, {}, 0.0, 100.0, {}, "intensity (%)"},
                  },
                  Needs::Light,
                  [w](ExecutionContext&, const ParamValues& p) {
                      std::string why;
                      if (w->SetLightCallout(p.GetString("light"), p.GetString("color"),
                                             p.GetDouble("percent"), &why))
                          return CommandResult::Success();
                      return CommandResult::Fail("light_color: " + why);
                  }});
    reg.Register({"light_off", "Instruments", "Switch a light (or every channel) off",
                  {
                      {"light", PT::LightName, false, "", 0, 0, {},
                       "light to switch off; empty = every configured channel"},
                  },
                  Needs::Light,
                  [w](ExecutionContext&, const ParamValues& p) {
                      std::string why;
                      if (w->LightOff(p.GetString("light"), &why)) return CommandResult::Success();
                      return CommandResult::Fail("light_off: " + why);
                  }});
    reg.Register({"light_read", "Instruments",
                  "Read a lighting channel's intensity (%, 0 when off) into the run context",
                  {
                      {"channel", PT::Int, true, {}, 1, 8, {}, "front-panel channel L1..L8"},
                      {"ctx", PT::String, true, {}, 0, 0, {}, "destination ctx key"},
                  },
                  Needs::Light,
                  [](ExecutionContext& c, const ParamValues& p) {
                      const int ch = p.GetInt("channel");
                      DataStream& on = DataBus::Instance().Stream(topics::LightOn(ch));
                      if (on.Count() > 0 && on.LatestValue() < 0.5) {
                          c.CtxSet(p.GetString("ctx"), 0.0);   // an OFF channel reads 0 %
                          return CommandResult::Success();
                      }
                      return ReadTopicToCtx(c, topics::LightPct(ch), p.GetString("ctx"));
                  }});

    // ---- Camera -------------------------------------------------------------
    reg.Register({"camera_exposure", "Instruments", "Set the camera exposure time",
                  {
                      {"us", PT::Double, true, {}, 1.0, 10000000.0, {}, "exposure (us)"},
                  },
                  Needs::Camera,
                  [w](ExecutionContext&, const ParamValues& p) {
                      return BoolResult(w->SetCameraExposure(p.GetDouble("us")),
                                        "camera exposure");
                  }});
}

} // namespace firedog
