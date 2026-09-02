// UiSettings.h : persisted UI preferences (font choice + size).
//
// Small JSON file written next to the exe, mirroring the project's config
// convention (a gitignored `*.local.json` for user-specific runtime prefs).
// Tolerant load: a missing or malformed file yields defaults, never throws.
#pragma once

#include <map>
#include <string>
#include <vector>

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

    // Device Registration page: width of its right-hand control column in px
    // (0 = default). Same reasoning as the vision teach page — a takeover with
    // its own layout.
    float regCtlW = 0.0f;

    // Gripped Object page: width of its right-hand control column in px
    // (0 = default). Same reasoning as the Registration page beside it.
    float gripCtlW = 0.0f;

    // Run page: height of the Scan band in px (0 = default). Its own field
    // rather than lowerH: the band carries the scan parameter forms, and how
    // much room those need has nothing to do with the log strip's height.
    float runScanH = 0.0f;
    float runShotH = 0.0f;   // Run page image box height (px)

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

    // Nominal teach page: the tool footprint box, in mm. A viewing aid, kept
    // because re-typing the gripper's width at every teach is how one stops
    // being used. 0 = the page's own default.
    float nomBoxW = 0.0f;
    float nomBoxH = 0.0f;

    // Aux window: draw each registered device's tool tip on the image. A
    // readout, so ON by default like the two above.
    bool showGripperMarks = true;
    // Aux window: is the control strip expanded? It covers part of the
    // picture, so which state the operator left it in is worth keeping.
    bool auxPanelOpen = true;

    // Keithley chart style: true = scope look (bare canvas, overlay legend +
    // live value), false = classic axes/ticks plot.
    bool chartScope  = true;

    // Run page deck: the last-used program (by UID, so a rename keeps it) and
    // which of its slots was showing. The deck shows exactly ONE slot, so this
    // is what the operator comes back to — re-applied on every product load
    // that still contains the uid, otherwise the first program / first slot.
    std::string runProgUid;
    int         runSlot = 0;

    // Jog window "Pos" tab: the 3x3 quick-move pad, device name -> nine saved
    // position UIDs ("" = unassigned slot). Keyed by DEVICE NAME, not fleet
    // index, so reordering hardware.json does not shuffle an operator's pad.
    //
    // Here rather than in the product document for the same reason runProgUid
    // is: the pad is a station's own shortcut layout, and a uid that the loaded
    // product does not contain simply fails to resolve — the button greys out
    // instead of driving somewhere the operator did not mean.
    std::map<std::string, std::vector<std::string>> posPad;

    // Main OS window placement: the client rect it was last left at, in
    // virtual-screen px, plus whether it was maximized. Zero width/height
    // means "never saved" -> the built-in default size, placed by Windows.
    //
    // The rect stored while maximized is the RESTORED one (GlHost keeps the
    // last un-maximized geometry), so un-maximizing after a restart lands
    // where the operator left it rather than filling the screen forever.
    int  winX = 0, winY = 0;
    int  winW = 0, winH = 0;
    bool winMaximized = false;

    // Last-loaded product name (folder under products/); empty = none.
    // Auto-reloaded at startup.
    std::string lastProduct;

    static UiSettings Load(const std::string& path);
    bool Save(const std::string& path) const;   // best-effort; false if unwritable
};

} // namespace firedog
