// UiSettings.h : persisted UI preferences (font choice + size).
//
// Small JSON file written next to the exe, mirroring the project's config
// convention (a gitignored `*.local.json` for user-specific runtime prefs).
// Tolerant load: a missing or malformed file yields defaults, never throws.
#pragma once

#include <string>

namespace firedog {

struct UiSettings {
    int   fontIndex  = 0;       // index into App's kFonts table
    float fontSize   = 26.0f;   // UI font size in px
    int   themeIndex = 0;       // index into Theme's theme list

    // Main-window panel splitter sizes in px (0 = unset -> seed from proportion).
    // Shared across all pages (Overview / PI / ACS).
    float leftW  = 0.0f;
    float rightW = 0.0f;
    float lowerH = 0.0f;

    // Vision teach window: width of its parameter column in px (0 = default).
    // Its own field rather than sharing rightW: that window is not part of the
    // main five-region layout, and its column has to hold a whole parameter
    // form, so the two sizes have nothing useful to say to each other.
    float visionParamW = 0.0f;

    // Aux window toggles. NOTE: clickToMove restored true means a click drives
    // the real gantry immediately on startup.
    bool showChart   = true;
    bool clickToMove = false;

    // Aux window: overlay the last vision detection + the alignment E1/E2 marks.
    // Safe to default ON — unlike clickToMove it is a readout, not an action.
    bool showVisionMarks = true;

    // Keithley chart style: true = scope look (bare canvas, overlay legend +
    // live value), false = classic axes/ticks plot.
    bool chartScope  = true;

    // Last-loaded product name (folder under products/); empty = none.
    // Auto-reloaded at startup.
    std::string lastProduct;

    static UiSettings Load(const std::string& path);
    bool Save(const std::string& path) const;   // best-effort; false if unwritable
};

} // namespace firedog
