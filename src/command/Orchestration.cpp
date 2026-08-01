#include "command/Orchestration.h"

#include "core/Log.h"

namespace firedog {

// The software big-red-button (Jog window STOP). Order matters: abort every
// orchestration owner FIRST so no engine issues the next move after the stop,
// abandon any path leg, then hard-stop every motion device. NOT an E-stop
// substitute: laser / PSU / IO outputs are untouched (killing those on the
// hardware input is the SafetyWatch's job).
void Orchestration::StopAll()
{
    programs.Stop();            // marks the queue dead + stops its engine run
    recipe.RequestStop();       // direct recipe-editor runs
    lua.RequestStop();          // scripts (XCAN / raster / Scripts page)
    workflow.CancelPathMove();  // path executor: stop device + abandon legs
    workflow.StopAllMotion();   // hard stop on every motion device
    FD_WARN("ui", "STOP pressed: all motion stopped, orchestration aborted");
}

} // namespace firedog
