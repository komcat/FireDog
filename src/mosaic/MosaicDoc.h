// MosaicDoc.h : one captured mosaic on disk — its folder layout, its capture
// record and its annotations. Design: docs/mosaic-design.md.
//
//   <save_dir>/<area>_<YYYYMMDD_HHMMSS>/
//       mosaic.json        the CAPTURE record: area, camera map, every tile's
//                          commanded and measured pose and canvas position.
//                          Written only by mosaic_capture (rewritten after each
//                          tile, so a stopped or crashed run leaves a usable
//                          partial record).
//       annotations.json   markers and crop labels. Written only by the Mosaic
//                          page (UI thread).
//       placement.json     refined tile positions (seam refinement). Written
//                          only by MosaicWorker; optional - without it every
//                          tile sits where its measured pose put it.
//       tiles/r03_c07.png      the full frame of grid cell (3,7)
//       tiles/r03_c07_q.png    its quarter-size preview
//       export/...             stitched exports and label crops
//
// One writer per file is the whole concurrency story: the run never touches
// the annotations and the page never touches the capture record, so neither
// needs a lock or a merge.
//
// Annotations are stored in CANVAS pixels only. A gantry coordinate is derived
// through the record's own camera map (MosaicCanvasToGantry) every time it is
// shown — storing both would be two answers to where a marker is.
#pragma once

#include "core/MosaicConfig.h"
#include "vision/MosaicLayout.h"

#include <string>
#include <vector>

#include <nlohmann/json.hpp>

namespace firedog {

struct MosaicTileRec {
    std::string file;                 // relative to the mosaic folder
    std::string quarter;              // ditto, the 1/4 preview
    int         row = 0, col = 0;
    double      cmdX = 0.0, cmdY = 0.0, cmdZ = 0.0;     // where the gantry was sent
    double      measX = 0.0, measY = 0.0, measZ = 0.0;  // where it reported it was
    double      x = 0.0, y = 0.0;     // frame top-left on the canvas, px (from meas)
    int         w = 0, h = 0;
    double      rx = 0.0, ry = 0.0;   // refined top-left (placement.json), when refined
};

struct MosaicRecord {
    int         format = 1;
    std::string status = "capturing";   // capturing | complete | partial
    std::string why;                    // why partial
    std::string created, finished;
    std::string device;                 // the gantry the camera rides on
    MosaicArea  area;                   // a snapshot, as captured
    bool        calibrated = false;     // geom.m came from a camera calibration
    MosaicGeom  geom;
    int         imgW = 0, imgH = 0;
    int         rows = 0, cols = 0;
    double      stepX = 0.0, stepY = 0.0;
    int         canvasW = 0, canvasH = 0;   // the PLAN's canvas
    double      quad[4][2] = {};            // the area outline, canvas px
    int         planned = 0;                // tiles in the plan
    std::vector<MosaicTileRec> tiles;       // as filed, in capture order
    // Seam refinement, loaded from placement.json when its tile count matches.
    bool        refined = false;
    std::string refinedNote;                // what the refinement reported
};

struct MosaicMarker {
    std::string uid, name;
    std::string tag;                  // free text, for filtering ("" = none)
    double      cx = 0.0, cy = 0.0;   // canvas px
    std::string at;                   // NowStamp() when placed
};

struct MosaicLabel {
    std::string uid, name;
    std::string tag;                  // free text; also the crop's export subfolder
    double      x0 = 0.0, y0 = 0.0, x1 = 0.0, y1 = 0.0;   // canvas px, x0<x1, y0<y1
    std::string at;
};

struct MosaicAnnotations {
    std::vector<MosaicMarker> markers;
    std::vector<MosaicLabel>  labels;
};

inline constexpr const char* kMosaicRecordFile      = "mosaic.json";
inline constexpr const char* kMosaicAnnotationsFile = "annotations.json";
inline constexpr const char* kMosaicPlacementFile   = "placement.json";

nlohmann::json MosaicRecordToJson(const MosaicRecord& r);
bool           MosaicRecordFromJson(const nlohmann::json& j, MosaicRecord* out, std::string* why);

// Written through a temporary file and a rename, so a reader never sees half a
// record however the writer is interrupted.
bool SaveMosaicRecord(const std::string& dir, const MosaicRecord& r);
bool LoadMosaicRecord(const std::string& dir, MosaicRecord* out, std::string* why);

bool LoadMosaicAnnotations(const std::string& dir, MosaicAnnotations* out);   // absent = empty, true
bool SaveMosaicAnnotations(const std::string& dir, const MosaicAnnotations& a);

// Refined positions: SaveMosaicPlacement writes one per tile (index-aligned);
// LoadMosaicPlacement fills rx/ry and sets `refined` when the file matches the
// record's tiles, and clears `refined` otherwise.
bool SaveMosaicPlacement(const std::string& dir, const std::vector<double>& x,
                         const std::vector<double>& y, const std::string& note);
void LoadMosaicPlacement(const std::string& dir, MosaicRecord* r);

// Where tile `t` is drawn and composed: refined when asked and available.
inline double MosaicTileX(const MosaicRecord& r, const MosaicTileRec& t, bool useRefined)
{
    return useRefined && r.refined ? t.rx : t.x;
}
inline double MosaicTileY(const MosaicRecord& r, const MosaicTileRec& t, bool useRefined)
{
    return useRefined && r.refined ? t.ry : t.y;
}

// The canvas rectangle every filed tile covers (measured poses can overhang
// the plan's canvas by a few pixels). False when there are no tiles.
bool MosaicRecordExtent(const MosaicRecord& r, double* x0, double* y0, double* x1, double* y1,
                        bool useRefined = false);
std::vector<MosaicTileRef> MosaicTileRefs(const MosaicRecord& r, bool useRefined = false);

// Folders under `saveDir` holding a mosaic.json, newest first (by the capture
// stamp every folder name ends in). Just the folder names.
std::vector<std::string> ListMosaicFolders(const std::string& saveDir);

// A name made safe to be a file name: anything but [A-Za-z0-9-_.] becomes '_'.
std::string MosaicSafeName(const std::string& s);

} // namespace firedog
