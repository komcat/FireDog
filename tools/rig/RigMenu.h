// RigMenu.h : the numbered-menu front-end — pick 1/2/3, press Enter.
//
// Deliberately hand-rolled rather than built on a TUI library (FTXUI, PDCurses):
// those own the screen and redraw it, which would fight the thing that makes
// this rig useful — the driver's FD_* log lines streaming as they happen. The
// menu is reprinted after each action instead of clearing, so the whole session
// (menu, prompts, connect logs, read-back) stays in the terminal's scrollback.
//
// Every selection turns into the same command string RigConsole::Execute takes,
// so the menu, the typed prompt and the one-shot flags share one code path.
#pragma once

namespace firedog::rig {

class RigConsole;

// Runs until the operator quits or stdin hits EOF. Always returns 0.
int RunMenu(RigConsole& console);

} // namespace firedog::rig
