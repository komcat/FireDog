// Commands.h : registration of the built-in (v1) command vocabulary.
//
// Each domain file registers its commands into the shared registry; the
// execute lambdas capture Workflow& / HardwareManager& (both outlive the
// registry — all three are owned by App, hardware first). Call once from App
// after Workflow::Start; the registry is read-only afterwards.
#pragma once

namespace firedog {

class CommandRegistry;
class Workflow;
class HardwareManager;

void RegisterMotionCommands(CommandRegistry& reg, Workflow& wf, HardwareManager& hw);
void RegisterIoCommands(CommandRegistry& reg, Workflow& wf, HardwareManager& hw);
void RegisterInstrumentCommands(CommandRegistry& reg, Workflow& wf, HardwareManager& hw);
void RegisterSystemCommands(CommandRegistry& reg, Workflow& wf, HardwareManager& hw);
void RegisterDutCommands(CommandRegistry& reg, Workflow& wf, HardwareManager& hw);
void RegisterVisionCommands(CommandRegistry& reg, Workflow& wf, HardwareManager& hw);
void RegisterAlignCommands(CommandRegistry& reg, Workflow& wf, HardwareManager& hw);
void RegisterNeedleCommands(CommandRegistry& reg, Workflow& wf, HardwareManager& hw);
void RegisterDeviceRegCommands(CommandRegistry& reg, Workflow& wf, HardwareManager& hw);
void RegisterGripCommands(CommandRegistry& reg, Workflow& wf, HardwareManager& hw);

inline void RegisterBuiltinCommands(CommandRegistry& reg, Workflow& wf,
                                    HardwareManager& hw)
{
    RegisterMotionCommands(reg, wf, hw);
    RegisterIoCommands(reg, wf, hw);
    RegisterInstrumentCommands(reg, wf, hw);
    RegisterSystemCommands(reg, wf, hw);
    RegisterDutCommands(reg, wf, hw);
    RegisterVisionCommands(reg, wf, hw);
    RegisterAlignCommands(reg, wf, hw);
    RegisterNeedleCommands(reg, wf, hw);
    RegisterDeviceRegCommands(reg, wf, hw);
    RegisterGripCommands(reg, wf, hw);
}

} // namespace firedog
