// Theme.h : centralized ImGui styling — a small set of selectable themes.
// Apply(index) rebuilds the whole style from a fresh baseline (safe to call
// any frame — the Display header's theme combo hot-switches it); the chosen
// index persists in UiSettings. Put all global style/color tweaks here rather
// than scattering them across panels.
#pragma once

namespace firedog {

struct Theme {
    static int         Count();              // number of selectable themes
    static const char* Name(int index);      // display name (out-of-range -> 0)
    static void        Apply(int index = 0); // apply to ImGui + ImPlot (clamped)
};

} // namespace firedog
