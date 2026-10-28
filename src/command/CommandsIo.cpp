// CommandsIo.cpp : digital-IO commands (set/pulse/toggle/wait/clear-latch).
#include "command/Commands.h"

#include "command/CommandRegistry.h"
#include "command/ExecutionContext.h"
#include "hardware/HardwareManager.h"
#include "workflow/Workflow.h"

namespace firedog {

namespace {

IIoDevice* ResolveIo(ExecutionContext& ctx, const std::string& device, int* idxOut,
                     std::string* err)
{
    const int idx = ctx.IoIndexByName(device);
    if (idx < 0) {
        *err = "unknown io device '" + device + "'";
        return nullptr;
    }
    IIoDevice* d = ctx.Hw().IoDevice(static_cast<std::size_t>(idx));
    if (!d || !d->IsConnected()) {
        *err = "io device '" + device + "' is not connected";
        return nullptr;
    }
    if (idxOut) *idxOut = idx;
    return d;
}

CommandResult ExecSetOutput(ExecutionContext& ctx, const ParamValues& p)
{
    return ctx.Wf().SetIoOutputByName(p.GetString("device"), p.GetString("pin"),
                                      p.GetBool("on"))
               ? CommandResult::Success()
               : CommandResult::Fail("set output rejected (device/pin/connection?)");
}

CommandResult ExecPulseOutput(ExecutionContext& ctx, const ParamValues& p)
{
    const std::string device = p.GetString("device");
    const std::string pin    = p.GetString("pin");
    if (!ctx.Wf().SetIoOutputByName(device, pin, true))
        return CommandResult::Fail("pulse ON rejected (device/pin/connection?)");
    const bool full = ctx.Sleep(p.GetInt("ms"));
    // The authored intent is a PULSE — turn it off even on early stop.
    const bool offOk = ctx.Wf().SetIoOutputByName(device, pin, false);
    if (!full) return CommandResult::Fail("stopped");
    return offOk ? CommandResult::Success()
                 : CommandResult::Fail("pulse OFF rejected");
}

CommandResult ExecToggleOutput(ExecutionContext& ctx, const ParamValues& p)
{
    const std::string device = p.GetString("device");
    const std::string pin    = p.GetString("pin");
    const bool start         = p.GetBool("start_on");
    if (!ctx.Wf().SetIoOutputByName(device, pin, !start))
        return CommandResult::Fail("toggle rejected (device/pin/connection?)");
    const bool full = ctx.Sleep(p.GetInt("dwell_ms"));
    // Like pulse_output: the pin goes back to its starting state even on stop.
    const bool backOk = ctx.Wf().SetIoOutputByName(device, pin, start);
    if (!full) return CommandResult::Fail("stopped");
    return backOk ? CommandResult::Success()
                  : CommandResult::Fail("toggle back rejected");
}

CommandResult ExecWaitInput(ExecutionContext& ctx, const ParamValues& p)
{
    std::string err;
    IIoDevice* d = ResolveIo(ctx, p.GetString("device"), nullptr, &err);
    if (!d) return CommandResult::Fail(err);
    const std::string pinName = p.GetString("pin");
    const int pin = d->InputPinByName(pinName);
    if (pin < 0) return CommandResult::Fail("unknown input pin '" + pinName + "'");
    const bool want = p.GetBool("state");

    const auto r = ctx.WaitUntil(
        [&] {
            const IIoDevice::IoStatus st = d->Status();
            if (!st.valid) return false;
            const bool on = (st.inputs >> pin) & 1u;
            return on == want;
        },
        p.GetInt("timeout_ms"), p.GetInt("poll_ms"));
    if (r == ExecutionContext::WaitResult::Ok) return CommandResult::Success();
    return CommandResult::Fail(r == ExecutionContext::WaitResult::Stopped
                                   ? "stopped"
                                   : "timeout waiting for '" + pinName + "'");
}

CommandResult ExecClearLatch(ExecutionContext& ctx, const ParamValues& p)
{
    std::string err;
    int idx = -1;
    IIoDevice* d = ResolveIo(ctx, p.GetString("device"), &idx, &err);
    if (!d) return CommandResult::Fail(err);
    uint64_t mask = ~0ull;   // no pin named = clear everything
    const std::string pinName = p.GetString("pin");
    if (!pinName.empty()) {
        const int pin = d->InputPinByName(pinName);
        if (pin < 0) return CommandResult::Fail("unknown input pin '" + pinName + "'");
        mask = 1ull << pin;
    }
    return ctx.Wf().ClearIoLatch(static_cast<std::size_t>(idx), mask)
               ? CommandResult::Success()
               : CommandResult::Fail("clear latch rejected");
}

} // namespace

void RegisterIoCommands(CommandRegistry& reg, Workflow& wf, HardwareManager& hw)
{
    (void)wf;
    (void)hw;
    using PT = ParamType;

    reg.Register({"set_output", "IO", "Set a digital output by pin name",
                  {
                      {"device", PT::DeviceIo, true, {}, 0, 0, {}, "io device name"},
                      {"pin", PT::PinName, true, {}, 0, 0, {}, "output pin name",
                       PinDir::Output},
                      {"on", PT::Bool, true, {}, 0, 0, {}, "true = ON"},
                      PreDelayParam(),
                      PostDelayParam(),
                  },
                  CommandSpec::Needs::None, ExecSetOutput});

    reg.Register({"pulse_output", "IO",
                  "Set an output ON for a duration, then OFF (OFF even if stopped)",
                  {
                      {"device", PT::DeviceIo, true, {}, 0, 0, {}, "io device name"},
                      {"pin", PT::PinName, true, {}, 0, 0, {}, "output pin name",
                       PinDir::Output},
                      {"ms", PT::Int, false, 100, 1, 60000, {}, "pulse width"},
                      PreDelayParam(),
                      PostDelayParam(),
                  },
                  CommandSpec::Needs::None, ExecPulseOutput});

    reg.Register({"toggle_output", "IO",
                  "Switch an output away from its starting state, dwell, then switch "
                  "it back (back even if stopped)",
                  {
                      {"device", PT::DeviceIo, true, {}, 0, 0, {}, "io device name"},
                      {"pin", PT::PinName, true, {}, 0, 0, {}, "output pin name",
                       PinDir::Output},
                      {"start_on", PT::Bool, false, true, 0, 0, {},
                       "starting state: true = ON (ON->OFF->ON), false = OFF (OFF->ON->OFF)"},
                      {"dwell_ms", PT::Int, false, 100, 1, 60000, {},
                       "time held in the toggled state"},
                      PreDelayParam(),
                      PostDelayParam(),
                  },
                  CommandSpec::Needs::None, ExecToggleOutput});

    reg.Register({"wait_input", "IO", "Wait for a digital input to reach a state",
                  {
                      {"device", PT::DeviceIo, true, {}, 0, 0, {}, "io device name"},
                      {"pin", PT::PinName, true, {}, 0, 0, {}, "input pin name",
                       PinDir::Input},
                      {"state", PT::Bool, false, true, 0, 0, {}, "wanted state"},
                      {"timeout_ms", PT::Int, false, 10000, 10, 600000, {}, "wait timeout"},
                      {"poll_ms", PT::Int, false, 20, 1, 5000, {}, "poll interval"},
                  },
                  CommandSpec::Needs::None, ExecWaitInput});

    reg.Register({"clear_latch", "IO",
                  "Clear latched input edges (one pin by name, or all)",
                  {
                      {"device", PT::DeviceIo, true, {}, 0, 0, {}, "io device name"},
                      {"pin", PT::PinName, false, "", 0, 0, {}, "input pin name; empty = all",
                       PinDir::Input},
                  },
                  CommandSpec::Needs::None, ExecClearLatch});
}

} // namespace firedog
