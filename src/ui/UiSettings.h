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

    // Vision teach page: widths of its middle (parameter) and right (result)
    // columns in px (0 = default). Their own fields rather than sharing
    // leftW/rightW: that page is a takeover which replaces the main five-region
    // layout, so the two sets of sizes have nothing useful to say to each other.
    float visionParamW  = 0.0f;
    float visionResultW = 0.0f;

    // Recipe editor: width of its inspector column in px (0 = default). Same
    // reasoning — a takeover page with its own layout.
    float edInspectW = 0.0f;

    // Run page: widths of its deck (left) and DUT (right) columns in px
    // (0 = default). Same reasoning again — a takeover page with its own layout.
    float runDeckW = 0.0f;
    float runDutW  = 0.0f;
    // Program Organizer: width of its recipe-library column in px (0 = default).
    // Its own field again — the organizer is a WINDOW, so it does not share a
    // layout with any page.
    float orgLibW = 0.0f;

    // Run page: height of the Scan band in px (0 = default). Its own field
    // rather than lowerH: the band carries the scan parameter forms, and how
    // much room those need has nothing to do with the log strip's height.
    float runScanH = 0.0f;

    // Aux window toggles. NOTE: clickToMove restored true means a click drives
    // the real gantry immediately on startup.
    bool showChart   = true;
    bool clickToMove = false;

    // Aux window: overlay the last vision detection + the alignment E1/E2 marks.
    // Safe to default ON — unlike clickToMove it is a readout, not an action.
    bool showVisionMarks = true;

    // Aux window: name each stage's live pose against the taught positions.
    // A readout too, so ON by default for the same reason.
    bool showPosLabels = true;
    // ...and whether `hidden` (system) positions are candidates for that name.
    // ON by default: the plate says where the stage IS, and dropping the true
    // nearest would name a neighbour instead. Off is for when the `algn_...`
    // names are noise the operator cannot act on.
    bool posLabelsHidden = true;

    // Keithley chart style: true = scope look (bare canvas, overlay legend +
    // live value), false = classic axes/ticks plot.
    bool chartScope  = true;

    // Run page deck: the last-used program (by UID, so a rename keeps it) and
    // which of its slots was showing. The deck shows exactly ONE slot, so this
    // is what the operator comes back to — re-applied on every product load
    // that still contains the uid, otherwise the first program / first slot.
    std::string runProgUid;
    int         runSlot = 0;

    // Last-loaded product name (folder under products/); empty = none.
    // Auto-reloaded at startup.
    std::string lastProduct;

    static UiSettings Load(const std::string& path);
    bool Save(const std::string& path) const;   // best-effort; false if unwritable
};

} // namespace firedog
