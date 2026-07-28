// UiSettings.h : persisted UI preferences (font choice + size).
//
// Small JSON file written next to the exe, mirroring the project's config
// convention (a gitignored `*.local.json` for user-specific runtime prefs).
// Tolerant load: a missing or malformed file yields defaults, never throws.
#pragma once

#include <string>

namespace firedog {

struct UiSettings {
    int   fontIndex = 0;        // index into App's kFonts table
    float fontSize  = 26.0f;    // UI font size in px

    // Main-window panel splitter sizes in px (0 = unset -> seed from proportion).
    // Shared across all pages (Overview / PI / ACS).
    float leftW  = 0.0f;
    float rightW = 0.0f;
    float lowerH = 0.0f;

    // Aux window toggles. NOTE: clickToMove restored true means a click drives
    // the real gantry immediately on startup.
    bool showChart   = true;
    bool clickToMove = false;

    // Last-loaded product name (folder under products/); empty = none.
    // Auto-reloaded at startup.
    std::string lastProduct;

    static UiSettings Load(const std::string& path);
    bool Save(const std::string& path) const;   // best-effort; false if unwritable
};

} // namespace firedog
