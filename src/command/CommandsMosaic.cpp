// CommandsMosaic.cpp : large-area image capture. Design: docs/mosaic-design.md.
//
//   mosaic_capture   walk the camera's gantry over a taught area, one frame per
//                    grid cell, and file every frame with the pose it was
//                    taken at - the stitched picture is DERIVED from that
//                    record (the Mosaic page, its exports), never baked here.
//   mosaic_calibrate measure the camera's pixel -> gantry map, rotation and
//                    all: step the gantry +-X and +-Y over a textured spot and
//                    see how far the picture moves (vision/ImageShift). The
//                    result is queued as a PROPOSAL: the Mosaic page shows it
//                    beside the map in use and an Admin applies or discards it.
//
// The area, its Z mode, overlap and settle all live in the area record
// (core/MosaicConfig, machine state, edited on the Mosaic page), and the step
// names it: one source of truth for what "the area" means, so a recipe and the
// page's Capture button cannot capture two different things under one name.
//
// Not a teach mover (there is no position to write into) and not a parallel
// mover (it owns the camera for the whole walk).
//
// Why it is a command rather than an engine of its own: the page launches it as
// a one-step recipe, exactly as the Jog window's Autofocus does, so it inherits
// orch_.Busy(), Stop, the safety abort and the worker for free - and a recipe
// or a script can survey a part too.
#include "command/Commands.h"

#include "command/CommandRegistry.h"
#include "command/ExecutionContext.h"
#include "command/MotionWaits.h"
#include "core/Log.h"
#include "core/MosaicConfig.h"
#include "core/Product.h"   // NowStamp
#include "hardware/HardwareManager.h"
#include "hardware/motion/IMotionController.h"
#include "mosaic/MosaicDoc.h"
#include "vision/FocusMetric.h"   // RgbToGray8
#include "vision/ImageShift.h"
#include "vision/MosaicLayout.h"
#include "workflow/Workflow.h"

#pragma warning(push)
#pragma warning(disable : 4244 4456 4996)
#include <stb_image_write.h>   // implementation lives in CommandsDut.cpp
#pragma warning(pop)

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <map>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

namespace firedog {

namespace {

namespace fs = std::filesystem;
using PT = ParamType;

// The preview every tile carries to the page, and files beside itself.
constexpr int kQuarter = 4;

// Local wall-clock "YYYYMMDD_HHMMSS": the folder name an operator sorts by.
std::string FolderStamp()
{
    const std::time_t t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::tm tm{};
    localtime_s(&tm, &t);
    char b[32];
    std::snprintf(b, sizeof(b), "%04d%02d%02d_%02d%02d%02d", tm.tm_year + 1900, tm.tm_mon + 1,
                  tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
    return b;
}

// stb's PNG compression level is a process-wide knob. Fast for the capture
// (a 5 MP tile at the default level costs most of a second, which is most of
// the walk), put back on the way out for everybody else.
struct PngLevelScope {
    int saved;
    explicit PngLevelScope(int level) : saved(stbi_write_png_compression_level)
    {
        stbi_write_png_compression_level = level;
    }
    ~PngLevelScope() { stbi_write_png_compression_level = saved; }
    PngLevelScope(const PngLevelScope&) = delete;
    PngLevelScope& operator=(const PngLevelScope&) = delete;
};

// Wait out `settleMs`, then for `fresh` frames that are wholly post-move (the
// auto-focus rule and reason), then copy the newest. False only on a stop; a
// stalled feed warns once (via *warned) and hands over the latest frame.
bool GrabFresh(ExecutionContext& ctx, PylonCamera& cam, int settleMs, int fresh, int timeoutMs,
               bool* warned, std::vector<unsigned char>* rgb, int* w, int* h)
{
    if (settleMs > 0 && !ctx.Sleep(settleMs)) return false;
    if (fresh > 0) {
        const unsigned long long seq0 = ctx.Wf().CameraFrameSeq();
        const auto r = ctx.WaitUntil(
            [&] {
                return ctx.Wf().CameraFrameSeq() >= seq0 + static_cast<unsigned long long>(fresh);
            },
            timeoutMs, 10);
        if (r == ExecutionContext::WaitResult::Stopped) return false;
        if (r == ExecutionContext::WaitResult::Timeout && !*warned) {
            *warned = true;
            FD_WARN("mosaic", "no new camera frame within %d ms - using the latest one "
                              "(feed stalled?)", timeoutMs);
        }
    }
    unsigned long long seq = 0;
    if (!cam.GetLatestFrameSeq(*rgb, *w, *h, seq)) *w = *h = 0;
    return true;
}

CommandResult ExecMosaicCapture(ExecutionContext& ctx, const ParamValues& p)
{
    const std::string prefix = p.GetString("ctx_prefix");
    ctx.CtxSet(prefix + "_ok", false);

    // ---- The area --------------------------------------------------------------
    const MosaicConfig cfg = LoadMosaicConfig();
    const MosaicArea* area = FindMosaicArea(cfg.areas, p.GetString("area"));
    if (!area)
        return CommandResult::Fail("mosaic area '" + p.GetString("area") +
                                   "' not found - define it on the Mosaic page");

    // ---- The gantry the camera rides on ---------------------------------------
    const std::string device = ctx.Wf().GantryName();
    if (device.empty()) return CommandResult::Fail("no gantry to carry the camera");
    std::string err;
    int idx = -1;
    IMotionController* m = ResolveMotion(ctx, device, &idx, &err);
    if (!m) return CommandResult::Fail(err);
    const std::vector<std::string> axes = m->Axes();
    const auto hasAxis = [&](const char* a) {
        return std::find(axes.begin(), axes.end(), a) != axes.end();
    };
    if (!hasAxis("X") || !hasAxis("Y"))
        return CommandResult::Fail("'" + device + "' does not report X and Y");
    const bool haveZ = hasAxis("Z");

    // ---- The camera, and the size of one frame ---------------------------------
    PylonCamera& cam = ctx.Hw().Camera();
    if (!cam.IsConnected() || !cam.IsGrabbing())
        return CommandResult::Fail("the camera is not connected and grabbing");
    std::vector<unsigned char> rgb;
    int imgW = 0, imgH = 0;
    unsigned long long seq = 0;
    if (!cam.GetLatestFrameSeq(rgb, imgW, imgH, seq) || imgW <= 0 || imgH <= 0)
        return CommandResult::Fail("no camera frame available");

    double px2mmX = 0.0, px2mmY = 0.0;
    ctx.Wf().CameraPixelToMm(px2mmX, px2mmY);
    double mm[4];
    MosaicEffectiveM(cfg.cal, px2mmX, px2mmY, mm);

    // ---- The plan: refused here, before anything moves --------------------------
    const MosaicPlan plan = PlanMosaic(*area, mm, imgW, imgH);
    if (!plan.ok) return CommandResult::Fail("area '" + area->name + "': " + plan.why);

    // ---- The folder, and room on the disk for it --------------------------------
    std::error_code ec;
    fs::create_directories(cfg.saveDir, ec);
    if (ec || !fs::is_directory(cfg.saveDir, ec))
        return CommandResult::Fail("cannot create the mosaic folder '" + cfg.saveDir + "'");
    {
        // A fast PNG of a camera frame keeps well over half its raw size.
        const double perTile = imgW * static_cast<double>(imgH) * 3.0 * (0.8 + 1.0 / 16.0);
        const double need    = perTile * static_cast<double>(plan.tiles.size()) * 1.1;
        const fs::space_info si = fs::space(cfg.saveDir, ec);
        if (!ec && static_cast<double>(si.available) < need) {
            char b[200];
            std::snprintf(b, sizeof(b), "'%s' has %.1f GB free; %zu tiles need about %.1f GB",
                          cfg.saveDir.c_str(), si.available / 1e9, plan.tiles.size(), need / 1e9);
            return CommandResult::Fail(b);
        }
    }
    const std::string dir =
        (fs::path(cfg.saveDir) / (MosaicSafeName(area->name) + "_" + FolderStamp())).string();
    fs::create_directories(fs::path(dir) / "tiles", ec);
    if (ec) return CommandResult::Fail("cannot create '" + dir + "': " + ec.message());

    MosaicRecord rec;
    rec.created    = NowStamp();
    rec.device     = device;
    rec.area       = *area;
    rec.calibrated = cfg.cal.calibrated;
    rec.geom       = plan.geom;
    rec.imgW       = imgW;
    rec.imgH       = imgH;
    rec.rows       = plan.rows;
    rec.cols       = plan.cols;
    rec.stepX      = plan.stepX;
    rec.stepY      = plan.stepY;
    rec.canvasW    = plan.canvasW;
    rec.canvasH    = plan.canvasH;
    rec.planned    = static_cast<int>(plan.tiles.size());
    for (int k = 0; k < 4; ++k) {
        rec.quad[k][0] = plan.quad[k][0];
        rec.quad[k][1] = plan.quad[k][1];
    }
    if (!SaveMosaicRecord(dir, rec))
        return CommandResult::Fail("cannot write the mosaic record in '" + dir + "'");

    ctx.CtxSet(prefix + "_dir", dir);
    ctx.CtxSet(prefix + "_tiles", 0.0);
    {
        Workflow::MosaicDoneEvent ev;
        ev.dir     = dir;
        ev.started = true;
        ctx.Wf().QueueMosaicDone(ev);
    }
    FD_INFO("mosaic", "'%s': %zu tiles (%d x %d grid, %.0f %% overlap, Z %s) on %s -> %s",
            area->name.c_str(), plan.tiles.size(), plan.rows, plan.cols, area->overlapPct,
            MosaicZModeName(area->zMode), device.c_str(), dir.c_str());

    // Every exit from here on files the record as it stands and says so.
    const auto finish = [&](bool ok, const std::string& why) {
        rec.status   = ok ? "complete" : "partial";
        rec.why      = why;
        rec.finished = NowStamp();
        SaveMosaicRecord(dir, rec);
        Workflow::MosaicDoneEvent ev;
        ev.dir   = dir;
        ev.ok    = ok;
        ev.tiles = static_cast<int>(rec.tiles.size());
        ev.why   = why;
        ctx.Wf().QueueMosaicDone(ev);
        ctx.CtxSet(prefix + "_tiles", static_cast<double>(rec.tiles.size()));
        ctx.CtxSet(prefix + "_ok", ok);
        if (ok)
            FD_INFO("mosaic", "'%s': %zu tiles filed in %s", area->name.c_str(), rec.tiles.size(),
                    dir.c_str());
        else
            FD_WARN("mosaic", "'%s' stopped after %zu of %zu tiles: %s", area->name.c_str(),
                    rec.tiles.size(), plan.tiles.size(), why.c_str());
        return ok ? CommandResult::Success() : CommandResult::Fail(why);
    };

    // The area's travel velocity, unless the step names one of its own.
    nlohmann::json raw = p.Raw();
    if (p.GetDouble("velocity_mm_s") <= 0.0 && area->velocityMmS > 0.0)
        raw["velocity_mm_s"] = area->velocityMmS;
    const ParamValues pv(std::move(raw));
    if (const CommandResult pre = ApplyPreMove(ctx, idx, pv); !pre.ok)
        return finish(false, pre.error);

    const PngLevelScope pngLevel(1);
    const int frameTimeoutMs = p.GetInt("frame_timeout_ms");
    bool warnedStale = false;
    std::vector<unsigned char> thumb;

    for (std::size_t i = 0; i < plan.tiles.size(); ++i) {
        if (ctx.ShouldStop()) return finish(false, "stopped");
        const MosaicPlanTile& t = plan.tiles[i];

        std::vector<std::pair<std::string, double>> targets = {{"X", t.gx}, {"Y", t.gy}};
        if (haveZ) targets.emplace_back("Z", t.gz);
        char what[64];
        std::snprintf(what, sizeof(what), "mosaic tile (%d,%d)", t.row + 1, t.col + 1);
        const CommandResult mv = MoveWithApproach(ctx, m, idx, targets, pv, what);
        if (!mv.ok) return finish(false, mv.error);
        m->RequestFastPoll();   // the measured pose below is read off fresh positions

        int fw = 0, fh = 0;
        if (!GrabFresh(ctx, cam, area->settleMs, area->freshFrames, frameTimeoutMs, &warnedStale,
                       &rgb, &fw, &fh))
            return finish(false, "stopped");
        if (fw != imgW || fh != imgH)
            return finish(false, "the camera frame changed size or stopped mid-walk");

        // Placed by where the axes SAY they are, not where they were sent.
        const std::map<std::string, double> pos = m->Positions();
        const auto at = [&](const char* a, double dflt) {
            const auto it = pos.find(a);
            return it != pos.end() ? it->second : dflt;
        };
        MosaicTileRec tr;
        tr.row   = t.row;
        tr.col   = t.col;
        tr.cmdX  = t.gx;
        tr.cmdY  = t.gy;
        tr.cmdZ  = haveZ ? t.gz : 0.0;
        tr.measX = at("X", t.gx);
        tr.measY = at("Y", t.gy);
        tr.measZ = at("Z", tr.cmdZ);
        double cx = 0.0, cy = 0.0;
        MosaicGantryToCanvas(plan.geom, tr.measX, tr.measY, &cx, &cy);
        tr.x = cx - imgW * 0.5;
        tr.y = cy - imgH * 0.5;
        tr.w = imgW;
        tr.h = imgH;

        char name[48];
        std::snprintf(name, sizeof(name), "r%02d_c%02d", t.row + 1, t.col + 1);
        tr.file    = std::string("tiles/") + name + ".png";
        tr.quarter = std::string("tiles/") + name + "_q.png";
        const std::string full = (fs::path(dir) / tr.file).string();
        if (!stbi_write_png(full.c_str(), imgW, imgH, 3, rgb.data(), imgW * 3))
            return finish(false, "png write failed: " + full);
        int tw = 0, th = 0;
        DownsampleRgb(rgb.data(), imgW, imgH, kQuarter, &thumb, &tw, &th);
        const std::string quarter = (fs::path(dir) / tr.quarter).string();
        if (!stbi_write_png(quarter.c_str(), tw, th, 3, thumb.data(), tw * 3))
            return finish(false, "png write failed: " + quarter);

        rec.tiles.push_back(tr);
        SaveMosaicRecord(dir, rec);   // a stop or a crash still leaves a record

        Workflow::MosaicTileEvent ev;
        ev.dir   = dir;
        ev.index = static_cast<int>(i);
        ev.total = static_cast<int>(plan.tiles.size());
        ev.row   = tr.row;
        ev.col   = tr.col;
        ev.x     = tr.x;
        ev.y     = tr.y;
        ev.w     = imgW;
        ev.h     = imgH;
        ev.thumb = thumb;
        ev.tw    = tw;
        ev.th    = th;
        ctx.Wf().QueueMosaicTile(std::move(ev));
        ctx.CtxSet(prefix + "_tiles", static_cast<double>(rec.tiles.size()));
    }
    return finish(true, {});
}

// ---- mosaic_calibrate ------------------------------------------------------------------
//
// With M the camera's mm-per-pixel map, a gantry move D shifts the picture by
// d = -M^-1 D. Moving +s and -s along X and diffing the two shifts measures the
// first column of M^-1 (the base frame's own error cancels in the difference,
// and so does any backlash that is the same both ways); Y gives the second.
// Invert, and M carries scale, rotation and skew - everything the two
// hardware.json scalars cannot say.
CommandResult ExecMosaicCalibrate(ExecutionContext& ctx, const ParamValues& p)
{
    const std::string prefix = p.GetString("ctx_prefix");
    ctx.CtxSet(prefix + "_ok", false);

    const std::string device = ctx.Wf().GantryName();
    if (device.empty()) return CommandResult::Fail("no gantry to carry the camera");
    std::string err;
    int idx = -1;
    IMotionController* m = ResolveMotion(ctx, device, &idx, &err);
    if (!m) return CommandResult::Fail(err);
    PylonCamera& cam = ctx.Hw().Camera();
    if (!cam.IsConnected() || !cam.IsGrabbing())
        return CommandResult::Fail("the camera is not connected and grabbing");

    m->RequestFastPoll();
    if (!ctx.Sleep(120)) return CommandResult::Fail("stopped");
    const std::map<std::string, double> pos = m->Positions();
    if (!pos.count("X") || !pos.count("Y"))
        return CommandResult::Fail("'" + device + "' does not report X and Y");
    const double bx = pos.at("X"), by = pos.at("Y");

    const double step     = p.GetDouble("step_mm");
    const double minScore = p.GetDouble("min_score");
    const int    settleMs = p.GetInt("settle_ms");
    const int    fresh    = p.GetInt("fresh_frames");
    const int    frameTo  = p.GetInt("frame_timeout_ms");
    bool warned = false;

    std::vector<unsigned char> rgb, gA, gP, gM;
    int w = 0, h = 0;
    if (!GrabFresh(ctx, cam, settleMs, fresh, frameTo, &warned, &rgb, &w, &h))
        return CommandResult::Fail("stopped");
    if (w <= 0 || h <= 0) return CommandResult::Fail("no camera frame available");

    // The step must move the picture well inside the search: a quarter of the
    // frame, with room to spare for a scale that is off.
    double px2mmX = 0.0, px2mmY = 0.0;
    ctx.Wf().CameraPixelToMm(px2mmX, px2mmY);
    const double px2mm = std::min(std::fabs(px2mmX), std::fabs(px2mmY));
    if (!(px2mm > 0.0)) return CommandResult::Fail("pixel_to_mm is zero - set it on the Camera page");
    const double expectPx = step / px2mm;
    const int    limitPx  = std::min(w, h) / 4;
    if (expectPx * 1.3 > limitPx) {
        char b[200];
        std::snprintf(b, sizeof(b), "a %.3f mm step moves the picture ~%.0f px; keep it under "
                      "%d px (step <= %.3f mm)", step, expectPx, limitPx, limitPx / 1.3 * px2mm);
        return CommandResult::Fail(b);
    }
    const int maxShift = static_cast<int>(expectPx * 1.5) + 16;

    if (const CommandResult pre = ApplyPreMove(ctx, idx, p); !pre.ok) return pre;
    const auto moveTo = [&](double x, double y, const char* what) {
        return MoveWithApproach(ctx, m, idx, {{"X", x}, {"Y", y}}, p, what);
    };
    const auto grayNow = [&](std::vector<unsigned char>* g, std::string* why) {
        int fw = 0, fh = 0;
        if (!GrabFresh(ctx, cam, settleMs, fresh, frameTo, &warned, &rgb, &fw, &fh)) {
            *why = "stopped";
            return false;
        }
        if (fw != w || fh != h) {
            *why = "the camera frame changed size";
            return false;
        }
        RgbToGray8(rgb.data(), w, h, FocusRoi{}, *g);
        return true;
    };
    // Every exit from here on puts the camera back where it started.
    const auto fail = [&](const std::string& why) {
        moveTo(bx, by, "the calibration start");
        return CommandResult::Fail(why);
    };

    RgbToGray8(rgb.data(), w, h, FocusRoi{}, gA);
    double col[2][2] = {};   // col[axis] = px the picture moves per mm along that axis
    double worst = 1.0;
    const char* axisName[2] = {"X", "Y"};
    for (int axis = 0; axis < 2; ++axis) {
        const double ax = axis == 0 ? step : 0.0, ay = axis == 1 ? step : 0.0;
        std::string why;
        CommandResult mv = moveTo(bx + ax, by + ay, "the + calibration step");
        if (!mv.ok) return fail(mv.error);
        if (!grayNow(&gP, &why)) return fail(why);
        mv = moveTo(bx - ax, by - ay, "the - calibration step");
        if (!mv.ok) return fail(mv.error);
        if (!grayNow(&gM, &why)) return fail(why);

        const ShiftResult dp = EstimateFrameShift(gA.data(), gP.data(), w, h, maxShift);
        const ShiftResult dm = EstimateFrameShift(gA.data(), gM.data(), w, h, maxShift);
        if (!dp.ok || !dm.ok || dp.score < minScore || dm.score < minScore) {
            char b[240];
            std::snprintf(b, sizeof(b), "the picture could not be matched across the %s step "
                          "(NCC %.2f / %.2f, need %.2f) - put something textured under the "
                          "camera, in focus", axisName[axis], dp.score, dm.score, minScore);
            return fail(b);
        }
        worst = std::min(worst, std::min(dp.score, dm.score));
        col[axis][0] = -(dp.dx - dm.dx) / (2.0 * step);
        col[axis][1] = -(dp.dy - dm.dy) / (2.0 * step);
        FD_INFO("mosaic", "calibrate %s: +%.3f mm -> (%.2f, %.2f) px, -%.3f mm -> (%.2f, %.2f) px",
                axisName[axis], step, dp.dx, dp.dy, step, dm.dx, dm.dy);
    }
    if (const CommandResult back = moveTo(bx, by, "the calibration start"); !back.ok) return back;

    // px per mm, columns = gantry X, gantry Y  ->  invert for mm per px.
    const double a = col[0][0], b = col[1][0], c = col[0][1], d = col[1][1];
    const double det = a * d - b * c;
    if (!(std::fabs(det) > 1e-9)) return CommandResult::Fail("the measured map is singular");
    Workflow::MosaicCalEvent ev;
    ev.m[0] =  d / det;
    ev.m[1] = -b / det;
    ev.m[2] = -c / det;
    ev.m[3] =  a / det;
    const double sx  = std::hypot(ev.m[0], ev.m[2]), sy = std::hypot(ev.m[1], ev.m[3]);
    const double rot = std::atan2(ev.m[2], ev.m[0]) * 180.0 / 3.14159265358979323846;
    // A scale far from the typed one is a repeating pattern matched one period
    // off, not a camera that changed lenses.
    const double rx = sx / std::fabs(px2mmX), ry = sy / std::fabs(px2mmY);
    if (rx < 0.7 || rx > 1.4 || ry < 0.7 || ry > 1.4) {
        char b2[240];
        std::snprintf(b2, sizeof(b2), "measured %.6f x %.6f mm/px is too far from pixel_to_mm "
                      "%.6f x %.6f - a repeating pattern under the camera? Try another spot or "
                      "a smaller step", sx, sy, px2mmX, px2mmY);
        return CommandResult::Fail(b2);
    }
    char note[240];
    std::snprintf(note, sizeof(note), "step %.3f mm, NCC >= %.2f: %.6f x %.6f mm/px, rotation "
                  "%.3f deg", step, worst, sx, sy, rot);
    ev.note = note;
    ctx.Wf().QueueMosaicCal(ev);
    ctx.CtxSet(prefix + "_ok", true);
    ctx.CtxSet(prefix + "_rotation_deg", rot);
    ctx.CtxSet(prefix + "_scale_x", sx);
    ctx.CtxSet(prefix + "_scale_y", sy);
    FD_INFO("mosaic", "camera map measured: %s", note);
    return CommandResult::Success();
}

} // namespace

void RegisterMosaicCommands(CommandRegistry& reg, Workflow& /*wf*/, HardwareManager& /*hw*/)
{
    reg.Register({"mosaic_capture", "Vision",
                  "Photograph a taught mosaic area frame by frame with the gantry camera and "
                  "file every frame with its pose, for the Mosaic page to stitch",
                  {
                      {"area", PT::String, true, "", 0, 0, {},
                       "the mosaic area (name or uid) - its corners, Z mode, overlap and "
                       "settle are defined on the Mosaic page"},
                      {"frame_timeout_ms", PT::Int, false, 2000, 100, 60000, {},
                       "how long to wait for fresh camera frames at each tile"},
                      {"ctx_prefix", PT::String, false, "mosaic", 0, 0, {},
                       "writes <prefix>_dir (the folder), <prefix>_tiles and <prefix>_ok"},
                      {"timeout_ms", PT::Int, false, 60000, 100, 600000, {}, "arrival timeout per tile"},
                      {"tol_mm", PT::Double, false, kMoveTolMM, 0.0001, 1.0, {}, "arrival tolerance"},
                      VelocityParam(),
                      ExposureParam(),
                      ApproachVelocityParam(),
                      ApproachMmParam(),
                      ApproachPctParam(),
                      PreDelayParam(),
                      PostDelayParam(),
                  },
                  CommandSpec::Needs::Camera, ExecMosaicCapture});

    reg.Register({"mosaic_calibrate", "Vision",
                  "Measure the gantry camera's pixel -> gantry map (scale, rotation, skew) by "
                  "stepping +-X and +-Y over a textured spot; the result is proposed on the "
                  "Mosaic page for review before it is used",
                  {
                      {"step_mm", PT::Double, false, 0.5, 0.01, 5.0, {},
                       "how far each step moves the gantry; the picture must move less than a "
                       "quarter of the frame"},
                      {"min_score", PT::Double, false, 0.5, 0.1, 0.99, {},
                       "the weakest picture match (NCC) accepted as a measurement"},
                      {"settle_ms", PT::Int, false, 100, 0, 10000, {}, "wait after each move"},
                      {"fresh_frames", PT::Int, false, 2, 0, 10, {},
                       "frames to wait for after each move"},
                      {"frame_timeout_ms", PT::Int, false, 2000, 100, 60000, {},
                       "how long to wait for them"},
                      {"ctx_prefix", PT::String, false, "mosaic_cal", 0, 0, {},
                       "writes <prefix>_ok, _rotation_deg, _scale_x, _scale_y"},
                      {"timeout_ms", PT::Int, false, 30000, 100, 600000, {}, "arrival timeout"},
                      {"tol_mm", PT::Double, false, kMoveTolMM, 0.0001, 1.0, {}, "arrival tolerance"},
                      VelocityParam(),
                      ExposureParam(),
                      ApproachVelocityParam(),
                      ApproachMmParam(),
                      ApproachPctParam(),
                  },
                  CommandSpec::Needs::Camera, ExecMosaicCalibrate});
}

} // namespace firedog
