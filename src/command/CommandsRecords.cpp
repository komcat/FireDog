// CommandsRecords.cpp : the product's position history.
//
//   record_position   append the device's CURRENT pose to the product's history
//                     under a name (core/PositionRecords.h)
//
// The step decides nothing and moves nothing: it is placed right after whatever
// left the device at a pose worth keeping (an xcan peak, a climb, a manual
// adjust), and the NAME says which device and which step that was. Every unit
// run that passes it appends one line, and the Position Records window shows
// where those poses cluster - the known place to start the next search.
//
// Without an open DUT it SKIPS (logged, success) unless record_without_dut:
// a test run with no part must neither fail nor pollute the history. It is not
// in the dry-run allowlist, so a dry or teach run never writes one either.
// docs/position-records-design.md.
#include "command/Commands.h"

#include "command/CommandRegistry.h"
#include "command/ExecutionContext.h"
#include "command/MotionWaits.h"   // ResolveMotion
#include "core/DutSession.h"
#include "core/Log.h"
#include "core/PositionRecords.h"
#include "core/Product.h"          // NowStamp
#include "hardware/motion/IMotionController.h"

#include <chrono>
#include <cstdio>
#include <map>
#include <string>

namespace firedog {

namespace {

using PT = ParamType;

CommandResult ExecRecordPosition(ExecutionContext& ctx, const ParamValues& p)
{
    const std::string device = p.GetString("device");
    const std::string name   = p.GetString("name");
    if (!PositionRecordNameOk(name))
        return CommandResult::Fail("record name '" + name +
                                   "' must be 1-64 of A-Z a-z 0-9 - _ .");

    const DutSession&      dut = DutSession::Instance();
    const DutSession::Info di  = dut.GetInfo();
    const std::string      dir = PositionRecordsDir(dut.ProductDataDir());
    if (!di.productSet || dir.empty())
        return CommandResult::Fail("no product loaded - nowhere to keep the history");
    if (!di.open && !p.GetBool("record_without_dut")) {
        FD_INFO("records", "'%s' not recorded: no DUT open (record_without_dut is off)",
                name.c_str());
        return CommandResult::Success();
    }

    std::string err;
    int idx = -1;
    IMotionController* m = ResolveMotion(ctx, device, &idx, &err);
    if (!m) return CommandResult::Fail(err);

    // read_position's freshness rule: the idle poll cache can be a second old.
    m->RequestFastPoll();
    if (!ctx.Sleep(120)) return CommandResult::Fail("stopped");
    const std::map<std::string, double> pose = m->Positions();
    if (pose.empty())
        return CommandResult::Fail("no live positions for '" + device + "'");

    PositionRecord r;
    r.t = std::chrono::duration_cast<std::chrono::seconds>(
              std::chrono::system_clock::now().time_since_epoch()).count();
    r.ts         = NowStamp();
    r.name       = name;
    r.device     = device;
    r.pose       = pose;
    r.product    = di.product;
    r.productUid = dut.ProductUid();
    r.serial     = di.open ? di.serial : std::string();
    const nlohmann::json op = dut.OperatorInfo();
    if (op.is_object()) {
        r.operatorId   = op.value("id", std::string());
        r.operatorName = op.value("name", std::string());
    }

    // A gap nobody sees is worse than a failed step: the history was asked for.
    if (!AppendPositionRecord(dir, r, &err))
        return CommandResult::Fail("position record '" + name + "' not saved: " + err);

    std::string axes;
    for (const auto& [axis, v] : pose) {
        char b[48];
        std::snprintf(b, sizeof(b), " %s=%.4f", axis.c_str(), v);
        axes += b;
    }
    FD_INFO("records", "'%s' recorded for %s (%s):%s", name.c_str(),
            r.serial.empty() ? "no DUT" : r.serial.c_str(), device.c_str(), axes.c_str());
    return CommandResult::Success();
}

} // namespace

void RegisterRecordCommands(CommandRegistry& reg, Workflow& wf, HardwareManager& hw)
{
    (void)wf;   // everything goes through ExecutionContext at run time
    (void)hw;

    reg.Register({"record_position", "DUT",
                  "Save the device's CURRENT pose (all axes) into the product's position "
                  "history under a name - one record per unit run; view them in "
                  "View > Position Records",
                  {
                      {"device", PT::DeviceMotion, true, {}, 0, 0, {},
                       "motion device whose pose is recorded"},
                      {"name", PT::String, true, {}, 0, 0, {},
                       "history name - says which device and step, e.g. lens1_peak "
                       "(A-Z a-z 0-9 - _ .); one file per name"},
                      {"record_without_dut", PT::Bool, false, false, 0, 0, {},
                       "record even when no DUT is open (default: skip, so a test run "
                       "without a part does not pollute the history)"},
                  },
                  CommandSpec::Needs::None, ExecRecordPosition});
}

} // namespace firedog
