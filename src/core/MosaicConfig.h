// MosaicConfig.h : the large-area image capture ("mosaic") record — where the
// areas are, where the pictures go, and how the camera's pixels map onto the
// gantry. Design: docs/mosaic-design.md.
//
// MACHINE state, not product state. A mosaic is a survey of whatever sits on
// the machine, taken by the camera on the gantry; it belongs to no product and
// to no DUT, so the areas live in config/mosaic.local.json (falling back to the
// shipped config/mosaic.json) next to the needle calibration, and the pictures
// go to a folder the operator picks once (default C:\mosaic).
//
// One writer: the UI thread. The Mosaic page edits the record and saves it on
// the spot; a capture run (mosaic_capture, on an engine worker) only READS the
// file at Start, and every editing surface is disabled while orch_.Busy(), so
// the two never overlap.
//
// A corner is a gantry AXIS reading (X, Y, Z) - the camera's own device, so the
// numbers cross no frame. Three corners describe a parallelogram (origin, row
// end, far corner: the waffle tray's convention, the fourth is derived); four
// describe any convex quadrilateral, taught in order around its edge.
//
// No ImGui, no Workflow: shared by the command layer and the page.
#pragma once

#include <string>
#include <vector>

#include <nlohmann/json.hpp>

namespace firedog {

struct MosaicCorner {
    bool        have = false;
    double      x = 0.0, y = 0.0, z = 0.0;   // gantry axis mm
    std::string taughtAt;                    // NowStamp() of the teach
};

// Where each tile's Z comes from.
//   Fixed - one Z for the whole scan (zFixedMm). The default: a flat part
//           under a camera with a few hundred µm of depth of field.
//   Plane - a plane through the corners' own Zs, so teaching every corner
//           in focus is what focuses the whole area. Costs nothing per tile,
//           and is what a part sitting slightly tilted needs.
enum class MosaicZMode { Fixed, Plane };

inline constexpr int kMosaicMaxCorners = 4;

struct MosaicArea {
    std::string  uid;
    std::string  name = "area";
    int          nCorners = 3;                 // 3 = parallelogram, 4 = quadrilateral
    MosaicCorner corners[kMosaicMaxCorners];
    MosaicZMode  zMode = MosaicZMode::Fixed;
    double       zFixedMm = 0.0;
    double       overlapPct = 15.0;            // between neighbouring tiles
    int          settleMs = 0;                 // extra wait after the move settles
    int          freshFrames = 2;              // frames to wait for after arrival
    double       velocityMmS = 0.0;            // travel velocity; 0 = keep current
};

// The camera's pixel -> gantry map: g = g_centre + M * (u - W/2, v - H/2), with
// M in mm per pixel. Uncalibrated, M is diag(pixel_to_mm_x, pixel_to_mm_y) from
// hardware.json - the Aux window's click-to-move convention (image +x = gantry
// +X, image +y = gantry +Y) with no rotation. A calibration measures all four
// terms, rotation and skew included; `calibrated` says which one a mosaic used.
struct MosaicCameraCal {
    bool        calibrated = false;
    double      m[4] = {0.0, 0.0, 0.0, 0.0};   // row-major 2x2: [m0 m1; m2 m3]
    std::string calibratedAt;
    std::string note;                          // how it was measured
};

struct MosaicConfig {
    std::string             saveDir;   // "" = <data root>/mosaic (filled in by the loader)
    std::vector<MosaicArea> areas;
    MosaicCameraCal         cal;
};

// Bounds the record accepts. A mistyped overlap must be refused before the
// machine moves, like a tray's row count.
inline constexpr double kMosaicMinOverlapPct = 5.0;
inline constexpr double kMosaicMaxOverlapPct = 50.0;

// ---- lookup ------------------------------------------------------------------
// Uid first, then exact name — the convention every record follows.
const MosaicArea* FindMosaicArea(const std::vector<MosaicArea>& list, const std::string& uidOrName);
MosaicArea*       FindMosaicArea(std::vector<MosaicArea>& list, const std::string& uidOrName);

const char* MosaicZModeName(MosaicZMode m);                    // "fixed" / "plane"
bool        MosaicZModeFromName(const std::string& s, MosaicZMode* out);

// "C1 origin", "C2 row end", ... for the page and the log.
const char* MosaicCornerLabel(int nCorners, int i);

// ---- persistence -------------------------------------------------------------
nlohmann::json  MosaicAreaToJson(const MosaicArea& a);
MosaicArea      MosaicAreaFromJson(const nlohmann::json& j);
nlohmann::json  MosaicConfigToJson(const MosaicConfig& c);
MosaicConfig    MosaicConfigFromJson(const nlohmann::json& j);

std::string  MosaicConfigLocalPath();   // <machine root>/config/mosaic.local.json
// config/mosaic.local.json, falling back to config/mosaic.json, falling back to
// the defaults. `pathUsed` receives the file actually read ("" = none).
MosaicConfig LoadMosaicConfig(std::string* pathUsed = nullptr);
bool         SaveMosaicConfig(const MosaicConfig& c);   // always the local file

} // namespace firedog
