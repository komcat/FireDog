// CommandsTray.cpp : the waffle-tray command family.
//
// A tray is a grid of positions taught from three corners (core/WaffleTray.h,
// docs/waffle-tray-design.md). These are the CONSUMING end — the teaching is
// the Waffle Tray page, for the reason it is with registrations: every corner
// needs an operator jogging onto a cavity by eye, and a second way to create a
// tray is a second answer to where its cells are.
//
//   move_to_cell   drive the tray's device to cell (row, col) — typed, or read
//                  from ctx keys so a loop index or a tray_next result picks it
//   tray_next      take the next enabled cell off the tray's session cursor
//                  and put its row/col into ctx; ok=false when it is used up
//   tray_reset     release every consumed cell of a tray
//
// move_to_cell is NOT a teach mover and NOT a parallel mover, and both are
// deliberate. A teach writes the live pose into the position the step drove
// to, by uid; a cell is derived from three corners and has no row to write
// into — re-teaching a corner on the page is how a tray moves. A parallel
// block deals its lanes by the VALUE of the device param, and this step's
// device is inside the tray record: two trays on one controller would become
// two lanes on one device. The validator refuses it inside `parallel`.
#include "command/Commands.h"

#include "command/CommandRegistry.h"
#include "command/ExecutionContext.h"
#include "command/MotionWaits.h"   // ResolveMotion, TargetsOf, MoveWithApproach, params
#include "core/Log.h"
#include "core/WaffleTray.h"
#include "hardware/motion/IMotionController.h"
#include "workflow/Workflow.h"

#include <cmath>
#include <cstdio>
#include <map>
#include <string>
#include <vector>

namespace firedog {

namespace {

using PT = ParamType;

std::string CellName(const WaffleTray& t, int r, int c)
{
    char b[96];
    std::snprintf(b, sizeof(b), "cell (%d,%d) of '%s'", r, c, t.name.c_str());
    return b;
}

// The tray a step names, usable or not. `*why` is the refusal.
const WaffleTray* TrayFor(ExecutionContext& ctx, const ParamValues& p, std::string* why)
{
    const std::string ref = p.GetString("tray");
    const WaffleTray* t = ctx.FindTray(ref);
    if (!t) {
        *why = "waffle tray '" + ref + "' not found";
        return nullptr;
    }
    const std::string v = TrayValidate(*t);
    if (!v.empty()) {
        *why = "tray '" + t->name + "': " + v + " - teach it on the Waffle Tray page";
        return nullptr;
    }
    return t;
}

// A row or column index: the typed value, unless `keyParam` names a ctx key —
// then that key's value, which must be there and be a whole number. A missing
// key FAILS rather than falling back to the typed value: a loop that forgot
// to set its index would otherwise drive every pass to the same cell.
bool ResolveIndex(ExecutionContext& ctx, const ParamValues& p, const char* valueParam,
                  const char* keyParam, int* out, std::string* err)
{
    const std::string key = p.GetString(keyParam);
    if (key.empty()) {
        *out = p.GetInt(valueParam);
        return true;
    }
    bool found = false;
    const double v = CtxNumber(ctx, key, &found);
    if (!found) {
        *err = std::string(keyParam) + ": ctx key '" + key + "' is not set (or not a number)";
        return false;
    }
    if (std::floor(v) != v || v < 1.0 || v > 1.0e6) {
        char b[160];
        std::snprintf(b, sizeof(b), "%s: ctx key '%s' is %g - not a cell index", keyParam,
                      key.c_str(), v);
        *err = b;
        return false;
    }
    *out = static_cast<int>(v);
    return true;
}

// ---- move_to_cell ------------------------------------------------------------

CommandResult ExecMoveToCell(ExecutionContext& ctx, const ParamValues& p)
{
    std::string err;
    const WaffleTray* t = TrayFor(ctx, p, &err);
    if (!t) return CommandResult::Fail(err);

    int r = 0, c = 0;
    if (!ResolveIndex(ctx, p, "row", "row_key", &r, &err)) return CommandResult::Fail(err);
    if (!ResolveIndex(ctx, p, "col", "col_key", &c, &err)) return CommandResult::Fail(err);

    const std::string prefix = p.GetString("ctx_prefix");
    ctx.CtxSet(prefix + "_row", static_cast<double>(r));
    ctx.CtxSet(prefix + "_col", static_cast<double>(c));
    ctx.CtxSet(prefix + "_moved", false);

    if (!TrayCellInRange(*t, r, c)) {
        char b[160];
        std::snprintf(b, sizeof(b), "(%d,%d) is outside '%s' (%d x %d)", r, c,
                      t->name.c_str(), t->rows, t->cols);
        return CommandResult::Fail(b);
    }

    int idx = -1;
    IMotionController* m = ResolveMotion(ctx, t->device, &idx, &err);
    if (!m) return CommandResult::Fail(err);

    if (!TrayCellEnabled(*t, r, c)) {
        if (p.GetBool("skip_disabled")) {
            FD_INFO("tray", "%s is disabled - skipped (%s_moved = false)",
                    CellName(*t, r, c).c_str(), prefix.c_str());
            return CommandResult::Success();
        }
        return CommandResult::Fail(CellName(*t, r, c) + " is disabled");
    }

    std::map<std::string, double> pose;
    if (!TrayCellPose(*t, r, c, &pose))   // TrayFor validated; cannot happen
        return CommandResult::Fail(CellName(*t, r, c) + ": no pose");

    // The stand-off, on one of the tray's own axes. Both halves or neither:
    // an axis with no distance is a typed intention that does nothing, and a
    // distance with no axis is a number that has nowhere to go.
    const std::string offAxis = p.GetString("offset_axis");
    const double      offMm   = p.GetDouble("offset_mm");
    if (offAxis.empty() != (offMm == 0.0))
        return CommandResult::Fail("offset_axis and offset_mm go together - set both or neither");
    if (!offAxis.empty()) {
        const auto it = pose.find(offAxis);
        if (it == pose.end())
            return CommandResult::Fail("the tray's poses carry no '" + offAxis + "' axis");
        it->second += offMm;
    }

    SavedPos sp;
    std::snprintf(sp.name, sizeof(sp.name), "%s (%d,%d)", t->name.c_str(), r, c);
    sp.vals = std::move(pose);
    const auto targets = TargetsOf(sp, m);
    if (targets.empty())
        return CommandResult::Fail("tray '" + t->name + "' shares no axes with " + t->device);

    const CommandResult pre = ApplyPreMove(ctx, idx, p);
    if (!pre.ok) return pre;
    const CommandResult mv = MoveWithApproach(ctx, m, idx, targets, p, CellName(*t, r, c));
    if (!mv.ok) return mv;
    ctx.CtxSet(prefix + "_moved", true);
    return CommandResult::Success();
}

// ---- tray_next ----------------------------------------------------------------

CommandResult ExecTrayNext(ExecutionContext& ctx, const ParamValues& p)
{
    std::string err;
    const WaffleTray* t = TrayFor(ctx, p, &err);
    if (!t) return CommandResult::Fail(err);

    TrayOrder order = TrayOrder::RowMajor;
    if (!TrayOrderFromName(p.GetString("order"), &order))
        return CommandResult::Fail("unknown order '" + p.GetString("order") + "'");

    const std::string prefix = p.GetString("ctx_prefix");
    const std::vector<TrayCell> cells = TrayEnabledCells(*t, order);

    TrayCell    cell{0, 0};
    std::size_t index = 0, remaining = 0;
    const bool ok = ctx.Wf().TrayNext(t->uid, cells, p.GetBool("wrap"), &cell, &index,
                                      &remaining);
    ctx.CtxSet(prefix + "_ok", ok);
    if (!ok) {
        ctx.CtxSet(prefix + "_remaining", 0.0);
        char b[160];
        std::snprintf(b, sizeof(b), "tray '%s': every enabled cell has been taken (%zu of %zu)",
                      t->name.c_str(), cells.size(), cells.size());
        if (p.GetBool("require")) return CommandResult::Fail(b);
        FD_INFO("tray", "%s - %s_ok = false", b, prefix.c_str());
        return CommandResult::Success();
    }
    ctx.CtxSet(prefix + "_row",       static_cast<double>(cell.first));
    ctx.CtxSet(prefix + "_col",       static_cast<double>(cell.second));
    ctx.CtxSet(prefix + "_index",     static_cast<double>(index));
    ctx.CtxSet(prefix + "_remaining", static_cast<double>(remaining));
    FD_INFO("tray", "tray '%s': next is (%d,%d), %zu of %zu, %zu left", t->name.c_str(),
            cell.first, cell.second, index, cells.size(), remaining);
    return CommandResult::Success();
}

// ---- tray_reset ---------------------------------------------------------------

CommandResult ExecTrayReset(ExecutionContext& ctx, const ParamValues& p)
{
    // Existence only: resetting a tray that cannot yet yield a pose is fine.
    const std::string ref = p.GetString("tray");
    const WaffleTray* t = ctx.FindTray(ref);
    if (!t) return CommandResult::Fail("waffle tray '" + ref + "' not found");
    ctx.Wf().ClearTrayCursor(t->uid, "tray_reset");
    return CommandResult::Success();
}

} // namespace

void RegisterTrayCommands(CommandRegistry& reg, Workflow& /*wf*/, HardwareManager& /*hw*/)
{
    reg.Register({"move_to_cell", "Tray",
                  "Drive a waffle tray's device to cell (row, col): the pose is derived "
                  "from the tray's three taught corners",
                  {
                      {"tray", PT::TrayUid, true, {}, 0, 0, {},
                       "the waffle tray; its device is what moves"},
                      {"row", PT::Int, false, 1, 1, kTrayMaxRows, {}, "row, 1-based"},
                      {"col", PT::Int, false, 1, 1, kTrayMaxCols, {}, "column, 1-based"},
                      {"row_key", PT::String, false, "", 0, 0, {},
                       "read the row from this ctx key instead (a loop's index_ctx, or "
                       "tray_next's <prefix>_row); fails when the key is not set"},
                      {"col_key", PT::String, false, "", 0, 0, {},
                       "read the column from this ctx key instead (tray_next's <prefix>_col)"},
                      {"skip_disabled", PT::Bool, false, false, 0, 0, {},
                       "a disabled cell is skipped (<prefix>_moved = false) instead of "
                       "failing the step"},
                      {"offset_axis", PT::String, false, "", 0, 0, {},
                       "stand off the cell along this axis of the tray's device (e.g. Z); "
                       "goes with offset_mm"},
                      {"offset_mm", PT::Double, false, 0.0, -500.0, 500.0, {},
                       "the stand-off, signed, in the axis's own direction"},
                      {"ctx_prefix", PT::String, false, "cell", 0, 0, {},
                       "writes <prefix>_row, <prefix>_col and <prefix>_moved"},
                      {"timeout_ms", PT::Int, false, 60000, 100, 600000, {}, "arrival timeout"},
                      {"tol_mm", PT::Double, false, kMoveTolMM, 0.0001, 1.0, {}, "arrival tolerance"},
                      VelocityParam(),
                      ExposureParam(),
                      ApproachVelocityParam(),
                      ApproachMmParam(),
                      ApproachPctParam(),
                      PreDelayParam(),
                      PostDelayParam(),
                  },
                  CommandSpec::Needs::None, ExecMoveToCell});

    reg.Register({"tray_next", "Tray",
                  "Take the next enabled cell of a waffle tray off its session cursor "
                  "into ctx (<prefix>_row/_col); <prefix>_ok = false once every cell is "
                  "taken",
                  {
                      {"tray", PT::TrayUid, true, {}, 0, 0, {}, "the waffle tray"},
                      {"ctx_prefix", PT::String, false, "tray", 0, 0, {},
                       "writes <prefix>_row, _col, _index (1-based in the walk), "
                       "_remaining (after this one) and _ok"},
                      {"order", PT::Enum, false, "row_major", 0, 0,
                       {"row_major", "col_major", "serpentine"},
                       "the walk over the enabled cells; serpentine reverses every "
                       "second row"},
                      {"wrap", PT::Bool, false, false, 0, 0, {},
                       "when every cell is taken, release them all and start over "
                       "(a tray reloaded without a tray_reset)"},
                      {"require", PT::Bool, false, false, 0, 0, {},
                       "fail the step when the tray is used up, instead of reporting "
                       "<prefix>_ok = false"},
                  },
                  CommandSpec::Needs::None, ExecTrayNext});

    reg.Register({"tray_reset", "Tray",
                  "Release every consumed cell of a waffle tray (a fresh tray was loaded)",
                  {
                      {"tray", PT::TrayUid, true, {}, 0, 0, {}, "the waffle tray"},
                  },
                  CommandSpec::Needs::None, ExecTrayReset});
}

} // namespace firedog
