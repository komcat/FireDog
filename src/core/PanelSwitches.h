// PanelSwitches.h : the operator's panel switches - physical inputs that press
// on-screen buttons, so the operator does not have to walk back to the screen.
// An OFF -> ON edge of the input is the press.
//
//   confirm   the run prompt's OK / Continue        (any page; a prompt on screen)
//   cancel    the run prompt's Cancel               (any page; a prompt on screen)
//   run       the Run page's Execute                (Run page only, nothing busy)
//   stop      stop the program run                  (any page, while a program runs)
//
// <machine root>/config/panel_switches.local.json. There is no shipped default
// file: these values ARE the default, written out on the first start so the
// machine has a file to edit (the tower light's pattern). Which panel input is
// which is a fact about the station's wiring, so it is machine state.
//
// Every switch answers exactly what its button answers, and only a button that
// is ON SCREEN this frame: confirm/cancel are consumed by
// App::DrawRecipePromptWindow, run by the Run page's strip. A choice prompt
// (custom buttons) is not answered by them, nor is Manual Adjust or the
// alignment pick.
#pragma once

#include <string>

namespace firedog {

struct PanelSwitchConfig {
    bool        enabled = true;
    std::string device  = "IOSwitch";    // the IO device every input is on
    // Input pin NAMES; "" = that switch is not wired.
    std::string confirm = "switch_8";    // prompt OK / Continue
    std::string cancel  = "switch_9";    // prompt Cancel
    std::string run     = "switch_11";   // Run page Execute
    std::string stop    = "switch_12";   // stop the program run
    // A second rising edge on the same input within this long is bounce, not a
    // press - or one press would answer two back-to-back prompts.
    int debounceMs = 300;
    // A prompt must have been on screen this long before a switch answers it,
    // so a press meant for the previous prompt cannot answer one nobody read.
    int minShownMs = 300;
};

PanelSwitchConfig LoadPanelSwitchConfig();

} // namespace firedog
