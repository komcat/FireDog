// RasterMap.h : what a raster scan measured, laid out for the Aux window's
// heatmap - every sampled node as an OFFSET from the start pose, plus the grid
// it was planned on. Plain data.
//
// raster_scan.lua opens it when the scan STARTS (fd.raster_map{live = true},
// the planned grid, no points), adds every node as it is measured
// (fd.raster_point), and closes it at the end (fd.raster_map{live = false},
// the summary). Workflow holds the one map with a change counter
// (SetRasterMap / AddRasterPoint / RasterMapSince) and App::Update copies it
// when the counter moves: a script runs on an engine worker and must never
// touch the UI. One map, not a queue - the window shows the LAST scan, live
// while it runs, then for Preferences > Graphic view > Raster heatmap's life
// seconds.
//
// Display only: nothing moves by it and nothing reads it back.
#pragma once

#include <string>
#include <vector>

namespace firedog {

struct RasterMapPoint {
    double sUm = 0.0;   // slow-axis offset from the start, um (signed: dir included)
    double fUm = 0.0;   // fast-axis offset from the start, um
    double v   = 0.0;   // the feedback value, topic units
};

struct RasterMap {
    std::string device, topic;
    std::string slowAxis, fastAxis;   // the hexapod's LOCAL axes
    int    slowDir    = -1;           // which way rows stepped from the start
    double slowSpanUm = 0.0, slowStepUm = 0.0;   // one-sided
    double fastSpanUm = 0.0, fastStepUm = 0.0;
    // The fast grid's first and last node, um from the start (centre mode
    // -n..+n steps, corner 0..n or -n..0). Both 0 = an older script: +/- half.
    double fastLoUm = 0.0, fastHiUm = 0.0;
    double baseline   = 0.0;
    double best = 0.0, bestSUm = 0.0, bestFUm = 0.0;
    double threshold  = 0.0;          // topic units; 0 = none
    bool   exitHit    = false;
    bool   live       = false;        // the scan is still running
    std::vector<RasterMapPoint> points;   // in scan order
    std::string at;                       // when it finished
};

} // namespace firedog
