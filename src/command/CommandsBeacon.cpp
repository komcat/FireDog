// CommandsBeacon.cpp : temporary camera-pose markers.
//
//   plant_beacon         drive the CAMERA to a saved position, settle, and record
//                        where it measurably stands as a named beacon
//   plant_beacon_here    the same record, no move: wherever the camera stands now
//                        (the steps before it put it there)
//   remove_beacon        cancel one beacon (or all)
//   move_tool_to_beacon  move_tool_to_camera, with the beacon as "the camera"
//
// A beacon lets a recipe look somewhere, move the camera on (to inspect, to
// focus elsewhere), and still send a registered tool back to where it looked.
// The rule is that the camera MOVES to the plant location before planting: the
// step drives there itself through move_to_position and stores the MEASURED
// pose afterwards, so a beacon can never sit anywhere the camera did not go.
//
// Lifetime (core/Beacon.h): kBeaconLifeMin minutes, remove_beacon, product
// load, app close. Program start and DUT open keep it.
//
// Consuming a beacon is exactly move_tool_to_camera's APPROACH - the same
// PrepareRegMove refusals and ApproachRefPoint drive (command/RegApproach.h),
// with the beacon's device required to be the registration's reference. A
// typed rotation and one measured tip still only get the tool close.
// docs/beacon-design.md.
#include "command/Commands.h"

#include "command/CommandRegistry.h"
#include "command/ExecutionContext.h"
#include "command/MotionWaits.h"   // kMoveTolMM, the velocity / approach params
#include "command/RegApproach.h"   // PrepareRegMove / ApproachRefPoint
#include "core/Beacon.h"
#include "core/DeviceReg.h"
#include "core/Log.h"
#include "core/Product.h"          // NowStamp
#include "workflow/Workflow.h"

#include <chrono>
#include <map>
#include <string>
#include <utility>

namespace firedog {

namespace {

using PT = ParamType;

// Settle, read where the camera's device MEASURABLY stands, store the beacon.
// The one writer both plant commands share.
CommandResult PlantAtMeasuredPose(ExecutionContext& ctx, const ParamValues& p,
                                  const std::string& posUid)
{
    const std::string name   = p.GetString("name");
    const std::string device = p.GetString("device");

    const int settle = p.GetInt("settle_ms");
    if (settle > 0 && !ctx.Sleep(settle)) return CommandResult::Fail("stopped");

    // Where it MEASURABLY stands - never the commanded target.
    std::map<std::string, double> pose;
    if (!ctx.Wf().DevicePose(device, pose))
        return CommandResult::Fail("'" + device + "' is not connected");
    const auto ix = pose.find("X"), iy = pose.find("Y"), iz = pose.find("Z");
    if (ix == pose.end() || iy == pose.end() || iz == pose.end())
        return CommandResult::Fail("'" + device + "' does not report X/Y/Z");

    Beacon b;
    b.name        = name;
    b.device      = device;
    b.x           = ix->second;
    b.y           = iy->second;
    b.z           = iz->second;
    b.positionUid = posUid;
    b.at          = NowStamp();
    b.plantedAt   = std::chrono::steady_clock::now();
    ctx.Wf().SetBeacon(b);

    // Display mirrors for this recipe; a move never reads them.
    ctx.CtxSet(name + "_x", b.x);
    ctx.CtxSet(name + "_y", b.y);
    ctx.CtxSet(name + "_z", b.z);

    FD_INFO("beacon", "beacon '%s' planted on '%s' at (%.4f, %.4f, %.4f)%s - good for "
                      "%d min",
            name.c_str(), device.c_str(), b.x, b.y, b.z,
            posUid.empty() ? " where it stood" : "", kBeaconLifeMin);
    return CommandResult::Success();
}

// ---- plant_beacon -----------------------------------------------------------

CommandResult ExecPlantBeacon(ExecutionContext& ctx, const ParamValues& p,
                              const CommandRegistry& reg)
{
    const std::string posUid = p.GetString("position");
    if (p.GetString("name").empty()) return CommandResult::Fail("beacon name is empty");

    // The move FIRST, through move_to_position itself: its refusals, its
    // two-speed approach and its waits. Every param this step shares with that
    // command is handed on (the delays excepted - they belong to this step).
    const CommandSpec* mv = reg.Find("move_to_position");
    if (!mv) return CommandResult::Fail("move_to_position is not registered");
    nlohmann::json args = nlohmann::json::object();
    for (const ParamSpec& ps : mv->params) {
        if (ps.name == "pre_delay_ms" || ps.name == "post_delay_ms") continue;
        if (p.Has(ps.name.c_str())) args[ps.name] = p.Raw().at(ps.name);
    }
    const CommandResult moved = RunRegistered(ctx, reg, "move_to_position", args);
    if (!moved.ok) return moved;

    return PlantAtMeasuredPose(ctx, p, posUid);
}

// ---- plant_beacon_here ------------------------------------------------------
// The camera is already where it should be (a move, a jog, a detection's
// centre_after before this step): no move, just settle and record.
CommandResult ExecPlantBeaconHere(ExecutionContext& ctx, const ParamValues& p)
{
    if (p.GetString("name").empty()) return CommandResult::Fail("beacon name is empty");
    return PlantAtMeasuredPose(ctx, p, "");
}

// ---- remove_beacon ----------------------------------------------------------

CommandResult ExecRemoveBeacon(ExecutionContext& ctx, const ParamValues& p)
{
    if (p.GetBool("all")) {
        ctx.Wf().ClearBeacons("remove_beacon all");
        return CommandResult::Success();
    }
    const std::string name = p.GetString("name");
    if (ctx.Wf().EraseBeacon(name)) {
        FD_INFO("beacon", "beacon '%s' removed", name.c_str());
        return CommandResult::Success();
    }
    if (p.GetBool("must_exist"))
        return CommandResult::Fail("no beacon '" + name + "' to remove");
    FD_INFO("beacon", "remove_beacon: no beacon '%s' (nothing to do)", name.c_str());
    return CommandResult::Success();
}

// ---- move_tool_to_beacon ----------------------------------------------------
// move_tool_to_camera (CommandsDeviceReg.cpp) with the camera point read from
// the beacon instead of the live pose. The beacon's device is passed as the
// expected reference, so a registration taught against another device is
// refused rather than sent to a coordinate in a frame it never shared.

CommandResult ExecMoveToolToBeacon(ExecutionContext& ctx, const ParamValues& p)
{
    const std::string device = p.GetString("device");
    const std::string name   = p.GetString("name");

    Beacon      b;
    std::string why;
    if (!ctx.Wf().BeaconFor(name, &b, &why)) return CommandResult::Fail(why);

    DeviceReg          r;
    IMotionController* m   = nullptr;
    int                idx = -1;
    const CommandResult ready = PrepareRegMove(ctx, p, device, b.device, &r, &m, &idx, true);
    if (!ready.ok) return ready;

    const double  zoff = p.GetDouble("z_offset_mm");
    const RegVec3 target{b.x, b.y, b.z + zoff};
    FD_INFO("beacon", "'%s' to beacon '%s': (%.4f, %.4f, %.4f) on '%s', z_offset %+.4f, "
                      "%.1f min left",
            device.c_str(), name.c_str(), b.x, b.y, b.z, b.device.c_str(), zoff,
            BeaconMinutesLeft(b));
    return ApproachRefPoint(ctx, p, device, r, m, idx, target);
}

} // namespace

void RegisterBeaconCommands(CommandRegistry& reg, Workflow& wf, HardwareManager& hw)
{
    (void)wf;   // every action goes through ExecutionContext::Wf() at run time
    (void)hw;
    CommandRegistry* rp = &reg;

    CommandSpec plant{"plant_beacon", "Registration",
                      "Drive the CAMERA to a saved position, settle, and mark where it "
                      "stands as a named beacon (good for 30 min) - a registered tool "
                      "can be sent there later with move_tool_to_beacon",
                      {
                          {"name", PT::String, false, "beacon1", 0, 0, {},
                           "the beacon's name; planting an existing name replaces it"},
                          {"device", PT::DeviceMotion, false, "gantry-main", 0, 0, {},
                           "the CAMERA's motion device - the beacon is in its axis space"},
                          {"position", PT::PositionUid, true, {}, 0, 0, {},
                           "where the camera goes before planting (saved position uid)"},
                          {"settle_ms", PT::Int, false, 200, 0, 10000, {},
                           "wait after arrival before reading the pose"},
                          {"timeout_ms", PT::Int, false, 60000, 100, 600000, {},
                           "arrival timeout"},
                          {"tol_mm", PT::Double, false, kMoveTolMM, 0.0001, 1.0, {},
                           "arrival tolerance"},
                          VelocityParam(),
                          ApproachVelocityParam(),
                          ApproachMmParam(),
                          ApproachPctParam(),
                      },
                      CommandSpec::Needs::None,
                      [rp](ExecutionContext& c, const ParamValues& p) {
                          return ExecPlantBeacon(c, p, *rp);
                      }};
    plant.outputs = [](const nlohmann::json& p) {
        const std::string n   = p.value("name", std::string("beacon1"));
        const std::string dev = p.value("device", std::string("gantry-main"));
        CommandOutputs o;
        o.keys.push_back({n + "_x", "mm", "beacon X, '" + dev + "' axis mm (display only)"});
        o.keys.push_back({n + "_y", "mm", "beacon Y, '" + dev + "' axis mm (display only)"});
        o.keys.push_back({n + "_z", "mm", "beacon Z, '" + dev + "' axis mm (display only)"});
        return o;
    };
    CommandSpec here{"plant_beacon_here", "Registration",
                     "Mark where the CAMERA stands now as a named beacon (good for 30 "
                     "min) - no move: the steps before this one put it there",
                     {
                         {"name", PT::String, false, "beacon1", 0, 0, {},
                          "the beacon's name; planting an existing name replaces it"},
                         {"device", PT::DeviceMotion, false, "gantry-main", 0, 0, {},
                          "the CAMERA's motion device - the beacon is in its axis space"},
                         {"settle_ms", PT::Int, false, 200, 0, 10000, {},
                          "wait before reading the pose"},
                     },
                     CommandSpec::Needs::None, ExecPlantBeaconHere};
    here.outputs = plant.outputs;   // same keys: same name, same device
    reg.Register(std::move(plant));
    reg.Register(std::move(here));

    reg.Register({"remove_beacon", "Registration",
                  "Cancel a beacon planted by plant_beacon (or every beacon)",
                  {
                      {"name", PT::String, false, "beacon1", 0, 0, {},
                       "the beacon to remove"},
                      {"all", PT::Bool, false, false, 0, 0, {},
                       "remove every beacon (name is ignored)"},
                      {"must_exist", PT::Bool, false, false, 0, 0, {},
                       "fail when there is no such beacon (default: nothing to do)"},
                  },
                  CommandSpec::Needs::None, ExecRemoveBeacon});

    reg.Register({"move_tool_to_beacon", "Registration",
                  "APPROACH a beacon with a registered tool - move_tool_to_camera with "
                  "the beacon as the camera point; z_offset_mm stands it off",
                  {
                      {"device", PT::DeviceMotion, true, {}, 0, 0, {},
                       "the registered device whose TIP is being sent to the beacon"},
                      {"name", PT::String, false, "beacon1", 0, 0, {},
                       "the beacon (planted by plant_beacon, under 30 min old)"},
                      {"z_offset_mm", PT::Double, false, -2.0, -100.0, 100.0, {},
                       "offset from the beacon's Z, which is the camera Z that FOCUSED "
                       "there - so 0 sends the tip onto that surface. NEGATIVE stands "
                       "the tool off it (higher) on this machine, as for "
                       "move_tool_to_camera"},
                      {"axes", PT::String, false, "XYZ", 0, 0, {},
                       "which axes to command; the others are left where they are"},
                      {"max_mm", PT::Double, false, 10.0, 0.0, 500.0, {},
                       "refuse a move longer than this; 0 disables the ceiling"},
                      {"allow_unverified", PT::Bool, false, false, 0, 0, {},
                       "move through a registration whose check has not been run"},
                      {"timeout_ms", PT::Int, false, 60000, 100, 600000, {}, "arrival timeout"},
                      {"tol_mm", PT::Double, false, kMoveTolMM, 0.0001, 1.0, {},
                       "arrival tolerance"},
                      ToolVelocityParam(),
                      ApproachVelocityParam(),
                      ApproachMmParam(),
                      ApproachPctParam(),
                  },
                  CommandSpec::Needs::None, ExecMoveToolToBeacon});
}

} // namespace firedog
