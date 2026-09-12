// Theme.h : centralized ImGui styling — a small set of selectable themes.
// Apply(index) rebuilds the whole style from a fresh baseline (safe to call
// any frame — the Display header's theme combo hot-switches it); the chosen
// index persists in UiSettings. Put all global style/color tweaks here rather
// than scattering them across panels.
#pragma once

#include "imgui.h"

namespace firedog {

// Teach mode's colour, shared by the three surfaces that show it (the Run
// strip's toggle and badge, the Recipe Editor strip's badge, the Manual Adjust
// window's Save row). Orange rather than the run strip's green or the arm's
// amber: a teach run moves the machine but proves nothing about a part, and
// it must not look like either.
inline constexpr ImVec4 kTeachOrange    = ImVec4(0.90f, 0.45f, 0.05f, 1.0f);
inline constexpr ImVec4 kTeachOrangeHov = ImVec4(1.00f, 0.55f, 0.12f, 1.0f);
inline constexpr ImVec4 kTeachOrangeAct = ImVec4(0.75f, 0.36f, 0.03f, 1.0f);

struct Theme {
    static int         Count();              // number of selectable themes
    static const char* Name(int index);      // display name (out-of-range -> 0)
    static void        Apply(int index = 0); // apply to ImGui + ImPlot (clamped)
};

} // namespace firedog
