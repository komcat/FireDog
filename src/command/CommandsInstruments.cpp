// CommandsInstruments.cpp : Keithley / CLD101x / PSU / camera commands.
//
// Thin wrappers over the Workflow instrument methods (which are read-back
// verified and thread-safe against the drivers' poll loops). wait_for_
// temperature watches the CLD's published DataBus stream.
#include "command/Commands.h"

#include "command/CommandRegistry.h"
#include "command/ExecutionContext.h"
#include "core/DataBus.h"
#include "core/Topics.h"
#include "workflow/Workflow.h"

#include <cmath>

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

    // ---- UDP3305S PSU -------------------------------------------------------
    reg.Register({"psu_set_vi", "Instruments", "Set a PSU channel's V/I setpoints",
                  {
                      {"channel", PT::Int, true, {}, 1, 3, {}, "channel 1..3"},
                      {"volts", PT::Double, true, {}, 0.0, 32.0, {}, "voltage setpoint"},
                      {"amps", PT::Double, true, {}, 0.0, 5.0, {}, "current limit"},
                  },
                  Needs::Psu,
                  [w](ExecutionContext&, const ParamValues& p) {
                      return BoolResult(w->SetPsuVI(p.GetInt("channel"),
                                                    p.GetDouble("volts"),
                                                    p.GetDouble("amps")),
                                        "psu setpoint");
                  }});
    reg.Register({"psu_output", "Instruments", "Switch a PSU channel output on/off",
                  {
                      {"channel", PT::Int, true, {}, 1, 3, {}, "channel 1..3"},
                      {"on", PT::Bool, true, {}, 0, 0, {}, "true = ON"},
                  },
                  Needs::Psu,
                  [w](ExecutionContext&, const ParamValues& p) {
                      const int ch = p.GetInt("channel");
                      return BoolResult(p.GetBool("on") ? w->PsuOutputOn(ch)
                                                        : w->PsuOutputOff(ch),
                                        "psu output");
                  }});

    reg.Register({"psu_read", "Instruments",
                  "Read a PSU channel's measured voltage (V) or current (A) into the run context",
                  {
                      {"channel", PT::Int, true, {}, 1, 3, {}, "channel 1..3"},
                      {"what", PT::Enum, false, "voltage", 0, 0, {"voltage", "current"},
                       "which measurement"},
                      {"ctx", PT::String, true, {}, 0, 0, {}, "destination ctx key"},
                  },
                  Needs::Psu,
                  [](ExecutionContext& c, const ParamValues& p) {
                      const int ch = p.GetInt("channel");
                      const std::string topic = p.GetString("what") == "current"
                                                    ? topics::PsuCurrent(ch)
                                                    : topics::PsuVoltage(ch);
                      return ReadTopicToCtx(c, topic, p.GetString("ctx"));
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
