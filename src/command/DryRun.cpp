// DryRun.cpp — see DryRun.h for what the transform does and why.
#include "command/DryRun.h"

#include "command/CommandRegistry.h"

#include <array>
#include <string_view>

namespace firedog {

namespace {

// THE list. Motion that drives (or reads) only its own controller, every IO
// command, both pneumatic slides, and the timing/ctx plumbing a motion
// sequence leans on. Deliberately NOT here:
//   save_position             writes the product document
//   xcan / raster_scan / climb_raster / xcan_to_target / xcan_two_lens
//                             scans: they read the meter they optimise on
//   move_local / move_to_nominal / align_* / grip_* / needle / dispense
//                             need a live frame, a measurement or a fluid
//   move_to_point / *_camera  registration approaches: a part-relative move
//   tray_next / tray_*        consume cells in the product document
//   prompt / prompt_input / manual_adjust / wait_until / run_script
//                             an operator, a reading, or arbitrary Lua
constexpr std::array<std::string_view, 25> kDryRunCommands = {
    // Motion
    "set_velocity", "read_velocity", "read_position", "check_position", "move_axis_to",
    "move_to_position", "move_to_position_offset", "move_to_position_axes",
    "move_to_position_approach", "move_path", "move_relative",
    "move_lift_off", "move_relative_global", "stop_motion",
    // Tray: a move to a cell; the cursor itself is not advanced
    "move_to_cell", "move_to_active_cell",
    // IO
    "set_output", "pulse_output", "wait_input", "clear_latch",
    // Pneumatic
    "extend_slide", "retract_slide",
    // System plumbing
    "delay", "log", "ctx_set",
};

std::size_t Walk(std::vector<RecipeStep>& steps, const CommandRegistry& reg)
{
    std::size_t kept = 0;
    for (RecipeStep& s : steps) {
        if (!s.enabled) continue;
        if (s.type == "group" || s.type == "sub_recipe" || s.type == "parallel" ||
            s.type == "loop") {
            kept += Walk(s.children, reg);
        } else if (s.type == "if") {
            kept += Walk(s.children, reg);
            kept += Walk(s.elseChildren, reg);
        } else if (s.type == "stop_recipe") {
            // runs as written: it only fires on a branch the dry run took
        } else if (IsDryRunCommand(s.type) && reg.Find(s.type)) {
            ++kept;
        } else {
            s.enabled = false;
        }
    }
    return kept;
}

} // namespace

bool IsDryRunCommand(const std::string& name)
{
    for (const std::string_view c : kDryRunCommands)
        if (c == name) return true;
    return false;
}

std::size_t MakeDryRunRecipe(Recipe& r, const CommandRegistry& reg)
{
    return Walk(r.steps, reg);
}

} // namespace firedog
