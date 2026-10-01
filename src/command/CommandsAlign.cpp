// CommandsAlign.cpp : the Alignment Procedure command family — teach a local
// coordinate system (O1) from two operator-marked features, then use it.
//
// align_station drives to a station, applies its exposure/lighting and gets the
// marked point; align_solve turns the two points into a frame and hands it back
// to the UI thread to persist; align_apply and move_local consume the frame.
// align_run is the typed one-step wrapper over alignment_procedure.lua, the
// same shape as `xcan` — the script IS the procedure, so it stays editable.
//
// Why a MANUAL pick answer arrives in millimetres: the operator's click happens
// in the UI, which is where the on-screen pixel mapping already lives, so it
// converts and the worker just adds the offset to the gantry pose. An AUTO
// station has no UI in the loop, so Workflow::Detect does the same conversion
// on the worker side with the same camera calibration — the two routes fill the
// same PickResult and everything downstream is identical.
// See docs/alignment-procedure-design.md.
#include "command/Commands.h"

#include "command/CommandRegistry.h"
#include "command/ExecutionContext.h"
#include "command/LuaEngine.h"      // RunLuaFile (align_run)
#include "command/MotionWaits.h"    // WaitArrive / WaitSettle / ResolveMotion
#include "core/AlignFrame.h"
#include "core/DutSession.h"         // ImagesDir / InsertImage — the attempt crops
#include "core/Log.h"
#include "hardware/HardwareManager.h"
#include "hardware/motion/IMotionController.h"
#include "vision/HalconDetector.h"   // Available() — the licence gate for auto mode
#include "workflow/Workflow.h"

#include <stb_image_write.h>         // implementation lives in CommandsDut.cpp

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace firedog {

namespace {

// Illumination for a station or a detection tier (core/AlignLighting.h). An
// empty kind is a no-op; Workflow::ApplyLighting is the one implementation.
CommandResult ApplyLighting(ExecutionContext& ctx, const AlignLighting& l)
{
    std::string why;
    if (ctx.Wf().ApplyLighting(l, &why)) return CommandResult::Success();
    return CommandResult::Fail(why);
}

// ---- detection attempt crops ------------------------------------------------
// Every auto attempt at a station files a picture of what it looked at: a crop
// of HALF the frame (50% of each side), centred on the point it found - or, on a
// miss, on its search ROI, else the image centre - with a cross on the hit. A
// miss is the case that matters most: "it did not find E2" is only answerable by
// looking, and the frame is gone by the time anybody asks.
//
// With a DUT open the crop goes into that part's record (images/ + an image
// entry, exactly like dut_capture_image). Without one - the Alignment page, a
// Frame check sweep - it goes to logs/align_attempts/, which is pruned to the
// newest kAttemptKeep files so a repeatability sweep cannot fill the disk.
// A failure to file is logged and never fails the station: the picture is
// evidence about the detection, not part of it.
constexpr std::size_t kAttemptKeep = 300;

std::string AttemptStamp()
{
    using namespace std::chrono;
    const auto   now = system_clock::now();
    const time_t t   = system_clock::to_time_t(now);
    const int    ms  = static_cast<int>(
        duration_cast<milliseconds>(now.time_since_epoch()).count() % 1000);
    std::tm tm{};
    gmtime_s(&tm, &t);
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%04d%02d%02d_%02d%02d%02d_%03d",
                  tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour,
                  tm.tm_min, tm.tm_sec, ms);
    return buf;
}

void SaveDetectAttemptCrop(ExecutionContext& ctx, const AlignProc& proc, int station,
                           int tier, const VisionModel& model, const VisionDetection& det)
{
    std::vector<unsigned char> rgb;
    int w = 0, h = 0;
    unsigned long long seq = 0;
    if (!ctx.Hw().Camera().GetLatestFrameSeq(rgb, w, h, seq) || w <= 0 || h <= 0 ||
        rgb.size() < static_cast<std::size_t>(w) * static_cast<std::size_t>(h) * 3) {
        FD_WARN("align", "E%d detection %d: no frame to file", station, tier);
        return;
    }

    // Centre: the hit, else the ROI, else the image centre.
    double cx = 0.5 * w, cy = 0.5 * h;
    if (det.ok) {
        cx = det.xPx;
        cy = det.yPx;
    } else if (model.roi.enabled && model.roi.w > 0 && model.roi.h > 0) {
        cx = model.roi.x + 0.5 * model.roi.w;
        cy = model.roi.y + 0.5 * model.roi.h;
    }
    const int cw = (std::max)(1, w / 2), ch = (std::max)(1, h / 2);
    int x0 = static_cast<int>(cx) - cw / 2, y0 = static_cast<int>(cy) - ch / 2;
    x0 = (std::max)(0, (std::min)(x0, w - cw));
    y0 = (std::max)(0, (std::min)(y0, h - ch));

    std::vector<unsigned char> crop(static_cast<std::size_t>(cw) * ch * 3);
    for (int y = 0; y < ch; ++y)
        std::memcpy(&crop[static_cast<std::size_t>(y) * cw * 3],
                    &rgb[(static_cast<std::size_t>(y0 + y) * w + x0) * 3],
                    static_cast<std::size_t>(cw) * 3);
    // A cross on the hit (green), so the picture says WHICH point it reported.
    if (det.ok) {
        const int hx = static_cast<int>(det.xPx) - x0, hy = static_cast<int>(det.yPx) - y0;
        const int arm = (std::max)(6, cw / 40);
        const auto px = [&](int x, int y) {
            if (x < 0 || y < 0 || x >= cw || y >= ch) return;
            unsigned char* p = &crop[(static_cast<std::size_t>(y) * cw + x) * 3];
            p[0] = 40; p[1] = 255; p[2] = 60;
        };
        for (int d = -arm; d <= arm; ++d) { px(hx + d, hy); px(hx, hy + d); }
    }

    std::string safe = proc.name;
    for (char& c : safe) {
        const bool ok = (c >= '0' && c <= '9') || (c >= 'a' && c <= 'z') ||
                        (c >= 'A' && c <= 'Z') || c == '-' || c == '_';
        if (!ok) c = '_';
    }
    char tail[96];
    std::snprintf(tail, sizeof(tail), "_E%d_det%d_%s_", station, tier, det.ok ? "hit" : "miss");
    const std::string file = "align_" + safe + tail + AttemptStamp() + ".png";

    const std::string dutDir = DutSession::Instance().ImagesDir();
    const std::string dir    = dutDir.empty() ? std::string("logs/align_attempts") : dutDir;
    std::error_code ec;
    std::filesystem::create_directories(dir, ec);
    const std::string path = dir + "/" + file;
    if (!stbi_write_png(path.c_str(), cw, ch, 3, crop.data(), cw * 3)) {
        FD_WARN("align", "could not write detection crop %s", path.c_str());
        return;
    }
    if (!dutDir.empty()) {
        char key[48];
        std::snprintf(key, sizeof(key), "align_E%d_det%d", station, tier);
        std::string err;
        if (!DutSession::Instance().InsertImage(key, "images/" + file, &err))
            FD_WARN("align", "detection crop not recorded: %s", err.c_str());
        return;
    }

    // The fallback folder is pruned: newest kAttemptKeep stay.
    std::vector<std::filesystem::directory_entry> files;
    for (const auto& de : std::filesystem::directory_iterator(dir, ec))
        if (de.is_regular_file(ec)) files.push_back(de);
    if (files.size() > kAttemptKeep) {
        // Names END in the UTC stamp (YYYYMMDD_HHMMSS_mmm, 19 chars), so sort
        // on that: the procedure name in front would otherwise group by name.
        std::sort(files.begin(), files.end(), [](const auto& l, const auto& r) {
            const std::string a = l.path().stem().string(), b = r.path().stem().string();
            const std::string sa = a.size() >= 19 ? a.substr(a.size() - 19) : a;
            const std::string sb = b.size() >= 19 ? b.substr(b.size() - 19) : b;
            return sa < sb;
        });
        for (std::size_t i = 0; i + kAttemptKeep < files.size(); ++i)
            std::filesystem::remove(files[i].path(), ec);
    }
}

// ctx key for one component of a captured point: "<prefix>_e1_wx" etc.
std::string PointKey(const std::string& prefix, int station, const char* field)
{
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%s_e%d_%s", prefix.c_str(), station, field);
    return buf;
}

// The world point of an already-captured station, straight out of ctx. False
// when that station has not been captured in this run under this prefix.
bool CtxWorldPoint(ExecutionContext& ctx, const std::string& prefix, int station,
                   double* wx, double* wy)
{
    bool fx = false, fy = false;
    const double x = CtxNumber(ctx, PointKey(prefix, station, "wx"), &fx);
    const double y = CtxNumber(ctx, PointKey(prefix, station, "wy"), &fy);
    if (!fx || !fy) return false;
    if (wx) *wx = x;
    if (wy) *wy = y;
    return true;
}

// The guard band for THIS pick, when there is one to show: E2 of a procedure
// whose band is on, with E1 already captured. Everything else leaves it empty
// and the window draws nothing.
//
// It is filled HERE rather than in the UI because only the worker knows E1 — it
// was captured at the other station, one move ago, and the pick window sees
// nothing but the current field of view.
void FillPickBand(ExecutionContext& ctx, const AlignProc& proc, int station,
                  const std::string& prefix, ExecutionContext::PickBand* band)
{
    if (!band || station != 2 || !proc.guard.enabled) return;
    double e1x = 0.0, e1y = 0.0;
    if (!CtxWorldPoint(ctx, prefix, 1, &e1x, &e1y)) return;
    band->have     = true;
    band->cx       = e1x;
    band->cy       = e1y;
    band->spanRef  = proc.guard.spanMm;
    band->spanTol  = proc.guard.spanTolMm;
    band->thetaRef = proc.guard.thetaDeg;
    band->thetaTol = proc.guard.thetaTolDeg;
}

// Turn a captured pick into a world point: where the device is, plus where in
// the frame the feature was marked. Read the pose AFTER the capture — nothing
// moved, but this is the pose the pixel offset is relative to.
//
// Shared by align_station and pick_point. One copy on purpose: a second one
// that read the pose in a different order would be wrong in a way no test
// would show, only a part placed 20 um out.
CommandResult CaptureWorldPoint(ExecutionContext& ctx, IMotionController* m,
                                const ExecutionContext::PickResult& pick,
                                AlignPoint* e)
{
    m->RequestFastPoll();
    if (!ctx.Sleep(120)) return CommandResult::Fail("stopped");
    const std::map<std::string, double> live = m->Positions();
    const auto itx = live.find("X"), ity = live.find("Y");
    if (itx == live.end() || ity == live.end())
        return CommandResult::Fail("device '" + m->Name() + "' does not report X/Y");

    e->gx = itx->second;
    e->gy = ity->second;
    // Z too, when the device has one: the focus height this point was marked
    // at. Axis space, straight off the device, like gx/gy.
    const auto itz = live.find("Z");
    e->hasZ = itz != live.end();
    e->gz   = e->hasZ ? itz->second : 0.0;
    e->px = pick.px;
    e->py = pick.py;
    e->wx = e->gx + pick.dxMM;
    e->wy = e->gy + pick.dyMM;
    return CommandResult::Success();
}

// ---- align_station -------------------------------------------------------

CommandResult ExecAlignStation(ExecutionContext& ctx, const ParamValues& p)
{
    const AlignProc* proc = ctx.FindAlign(p.GetString("alignment"));
    if (!proc) return CommandResult::Fail("alignment procedure not found: '" +
                                          p.GetString("alignment") + "'");
    const int station = p.GetInt("station");
    if (station != 1 && station != 2) return CommandResult::Fail("station must be 1 or 2");
    const AlignStation& st = (station == 1) ? proc->s1 : proc->s2;
    if (st.posUid.empty())
        return CommandResult::Fail("station has no saved position assigned");

    std::string err;
    int idx = -1;
    IMotionController* m = ResolveMotion(ctx, proc->device, &idx, &err);
    if (!m) return CommandResult::Fail(err);

    const SavedPos* pos = ctx.FindPosition(proc->device, st.posUid);
    if (!pos) return CommandResult::Fail("station position uid not found");
    const auto targets = TargetsOf(*pos, m);
    if (targets.empty()) return CommandResult::Fail("station position shares no axes with device");

    // The tiers this station will try, in order, each resolved BEFORE moving so
    // a missing model costs no travel. A tier that cannot run (no model, no
    // licence) is not fatal on its own: it is recorded and the next tier gets
    // its turn, exactly as if it had run and missed.
    struct Tier {
        int                tier  = 1;
        const VisionModel* model = nullptr;
        std::string        err;          // why it cannot run, "" = runnable
    };
    std::vector<Tier> tiers;
    if (st.mode == "auto") {
        std::string licence;
        const bool halconOk = HalconDetector::Available(&licence);
        for (int t = 1; t <= kAlignDetectTiers; ++t) {
            if (!AlignTierOn(st, t)) continue;
            Tier tr;
            tr.tier  = t;
            tr.model = ctx.FindVisionModel(AlignTierDetector(st, t));
            if (!tr.model)
                tr.err = "detection " + std::to_string(t) + ": no detection model taught";
            else if (!halconOk)
                tr.err = "detection " + std::to_string(t) + ": cannot run '" +
                         tr.model->name + "': " + licence;
            tiers.push_back(std::move(tr));
        }
    }
    // The exposure for a tier: its model's own, else the station's (what every
    // procedure taught before tiers used). 0 = leave the camera alone.
    const auto tierExposure = [&](const Tier& tr) {
        return (tr.model && tr.model->exposureUs > 0.0) ? tr.model->exposureUs
                                                        : st.exposureUs;
    };

    // Exposure BEFORE the move, so the sensor settles while the axes travel:
    // the first tier's on an auto station, the manual pick's otherwise.
    const double expo0 = (!tiers.empty()) ? tierExposure(tiers.front()) : st.exposureUs;
    double expoNow = 0.0;   // what was last applied, so a tier at the same value skips it
    if (expo0 > 0.0) {
        if (!ctx.Wf().SetCameraExposure(expo0))
            return CommandResult::Fail("camera exposure change rejected (camera connected?)");
        expoNow = expo0;
    }
    // Lighting belongs to the DETECTION: each tier's model carries the light
    // it was taught under, and there is no station-level fallback - a model
    // set to None leaves the lighting exactly as it is, and so does a manual
    // station. The first tier's is applied BEFORE the move, so it has settled
    // by the time the axes have.
    static const AlignLighting kNoLight;
    const auto tierLighting = [&](const Tier& tr) -> const AlignLighting& {
        return tr.model ? tr.model->lighting : kNoLight;
    };
    AlignLighting lightNow = (!tiers.empty()) ? tierLighting(tiers.front()) : kNoLight;
    {
        const CommandResult light = ApplyLighting(ctx, lightNow);
        if (!light.ok) return light;
    }

    if (!ctx.Wf().MoveToPosition(static_cast<std::size_t>(idx), targets))
        return CommandResult::Fail("move to station " + std::to_string(station) + " rejected");
    const auto arrived = WaitArrive(ctx, m, targets, p.GetDouble("tol_mm"),
                                    p.GetInt("timeout_ms"));
    if (arrived != ExecutionContext::WaitResult::Ok) {
        ctx.Wf().StopMotion(static_cast<std::size_t>(idx));   // defined state on any abort
        return CommandResult::Fail(arrived == ExecutionContext::WaitResult::Stopped
                                       ? "stopped"
                                       : "timeout driving to station " + std::to_string(station));
    }
    const CommandResult settled = WaitSettle(ctx, m, idx);
    if (!settled.ok) return settled;

    // Where the E point comes from. Both routes fill the SAME PickResult — the
    // pixel and the millimetre offset from the image centre — so everything
    // downstream is identical whether a detector or an operator produced it.
    ExecutionContext::PickResult pick;
    bool manual = (st.mode != "auto");
    // The world point, and whether it is already in hand. An auto capture has to
    // be turned into one EARLY, because the guard band is a statement about
    // where E2 sits relative to E1 and a pixel offset cannot answer it. Kept so
    // the happy path captures the pose exactly once.
    AlignPoint e;
    bool       haveE  = false;
    const std::string prefix = p.GetString("ctx_prefix");
    std::string        autoErr;              // every tier's reason, joined
    const VisionModel* lastModel = nullptr;  // shown to the operator on a fallback
    int                foundTier = 0;

    // Prove the frame is post-change before believing it: a frame may have been
    // integrating while the stage travelled, or at the previous exposure. Same
    // gate auto_focus uses. False = stopped.
    const auto waitFresh = [&]() -> bool {
        const int fresh = p.GetInt("fresh_frames");
        if (fresh <= 0) return true;
        const unsigned long long seq0 = ctx.Wf().CameraFrameSeq();
        const auto fr = ctx.WaitUntil(
            [&] {
                return ctx.Wf().CameraFrameSeq() >=
                       seq0 + static_cast<unsigned long long>(fresh);
            },
            2000, 10);
        if (fr == ExecutionContext::WaitResult::Stopped) return false;
        if (fr == ExecutionContext::WaitResult::Timeout)
            FD_WARN("align", "'%s' station %d: no new frame within 2000 ms - "
                             "detecting on the latest frame (feed stalled?)",
                    proc->name.c_str(), station);
        return true;
    };
    const auto addErr = [&](const std::string& why) {
        autoErr += (autoErr.empty() ? "" : "; ") + why;
    };

    for (std::size_t ti = 0; ti < tiers.size(); ++ti) {
        const Tier& tr = tiers[ti];
        if (!tr.err.empty()) {
            addErr(tr.err);
            continue;
        }
        lastModel = tr.model;
        // Each tier at its own exposure, then frames taken at it.
        const double ex = tierExposure(tr);
        if (ex > 0.0 && ex != expoNow) {
            if (!ctx.Wf().SetCameraExposure(ex))
                return CommandResult::Fail("camera exposure change rejected (camera connected?)");
            expoNow = ex;
        }
        const AlignLighting& lt = tierLighting(tr);
        if (lt.Set() && !(lt == lightNow)) {
            const CommandResult light = ApplyLighting(ctx, lt);
            if (!light.ok) return light;
            lightNow = lt;
        }
        if (!waitFresh()) return CommandResult::Fail("stopped");

        VisionDetection det;
        ctx.Wf().Detect(*tr.model, ctx.VisionDir(), &det);
        SaveDetectAttemptCrop(ctx, *proc, station, tr.tier, *tr.model, det);
        if (!det.ok) {
            addErr("detection " + std::to_string(tr.tier) + " '" + tr.model->name +
                   "': " + det.error);
            FD_INFO("align", "'%s' E%d: detection %d missed (%s)%s", proc->name.c_str(),
                    station, tr.tier, det.error.c_str(),
                    ti + 1 < tiers.size() ? " - trying the next" : "");
            continue;
        }
        ExecutionContext::PickResult cand;
        cand.px   = det.xPx;
        cand.py   = det.yPx;
        cand.dxMM = det.dxMM;
        cand.dyMM = det.dyMM;
        FD_INFO("align", "'%s' E%d: detection %d '%s' found at (%.2f, %.2f) px, score %.3f",
                proc->name.c_str(), station, tr.tier, tr.model->name.c_str(),
                det.xPx, det.yPx, det.score);

        // Is the point the detector found where E2 is ALLOWED to be? A detector
        // that locks onto the wrong instance of a repeated feature succeeds with
        // a high score and a frame wrong by the pitch between them. Out of band
        // counts as a MISS for this tier, so the next tier (or the operator)
        // gets its turn.
        AlignPoint cp;
        const CommandResult cap = CaptureWorldPoint(ctx, m, cand, &cp);
        if (!cap.ok) return cap;
        double e1x = 0.0, e1y = 0.0;
        if (proc->guard.enabled && station == 2 &&
            CtxWorldPoint(ctx, prefix, 1, &e1x, &e1y)) {
            AlignPoint e1;
            e1.wx = e1x;
            e1.wy = e1y;
            double span = 0.0, theta = 0.0;
            AlignSpanTheta(e1, cp, &span, &theta);
            if (!AlignGuardCheck(proc->guard, span, theta)) {
                char buf[256];
                std::snprintf(buf, sizeof(buf),
                              "detection %d '%s' found a point OUT OF BAND: span %.4f mm "
                              "(%.4f +/- %.4f), theta %.3f deg (%.3f +/- %.3f)",
                              tr.tier, tr.model->name.c_str(), span, proc->guard.spanMm,
                              proc->guard.spanTolMm, theta, proc->guard.thetaDeg,
                              proc->guard.thetaTolDeg);
                addErr(buf);
                continue;
            }
        }
        pick      = cand;
        e         = cp;
        haveE     = true;
        foundTier = tr.tier;
        break;
    }
    // Which tier found it: 0 = none (manual, or every tier missed). A later
    // tier doing the work is worth a warning - it is how a drifting part or a
    // dimming light shows up before tier 1 stops working altogether.
    if (!manual) {
        ctx.CtxSet(prefix + "_e" + std::to_string(station) + "_tier",
                   static_cast<double>(foundTier));
        if (foundTier > 1)
            FD_WARN("align", "'%s' E%d: found by detection %d (earlier detection missed)",
                    proc->name.c_str(), station, foundTier);
    }

    // Every tier missed (or none could run). Falling back is opt-in BECAUSE a
    // silent fallback would let a procedure claim to be automatic while a human
    // placed every point.
    if (!manual && foundTier == 0) {
        if (autoErr.empty()) autoErr = "no detection could run";
        if (!st.autoFallbackManual)
            return CommandResult::Fail(autoErr +
                " (set auto_fallback_manual or switch the station to manual)");
        FD_WARN("align", "'%s' station %d: %s - asking the operator",
                proc->name.c_str(), station, autoErr.c_str());
        manual = true;
        // The operator marks at the MANUAL exposure when one is set - a
        // detection tier's exposure may be tuned for a filter, not for eyes.
        if (st.exposureUs > 0.0 && st.exposureUs != expoNow)
            ctx.Wf().SetCameraExposure(st.exposureUs);
        // The lighting stays as the last tier left it: there is no station
        // light to go back to.
    }

    if (manual) {
        ExecutionContext::PickRequest req;
        req.alignName = proc->name;
        req.station   = station;
        // Only set on the FALLBACK path (autoErr non-empty): a station that was
        // manual by design has no taught point to show, and showing one would
        // claim a precision the procedure never asked for.
        req.autoError   = autoErr;
        if (!autoErr.empty() && lastModel) req.detectorUid = lastModel->uid;
        req.message   = ctx.Substitute(p.GetString("message"));
        if (req.message.empty()) {
            char buf[96];
            std::snprintf(buf, sizeof(buf), "Mark E%d on the live image, then OK", station);
            req.message = buf;
        }
        // Where E2 is allowed to land, drawn on the image the operator is
        // marking. This is the whole re-mark loop: the crosshair moves until it
        // is inside the band, and accepting one outside takes its own button.
        FillPickBand(ctx, *proc, station, prefix, &req.band);
        if (!ctx.AwaitPick(req, &pick))
            return CommandResult::Fail("cancelled by operator");
        haveE = false;   // the operator's mark, not whatever the detector said
    }

    // The world point: where the gantry is, plus where in the frame the feature
    // was marked (by hand or by the detector). Already in hand when an auto
    // capture had to be measured against the band.
    if (!haveE) {
        const CommandResult captured = CaptureWorldPoint(ctx, m, pick, &e);
        if (!captured.ok) return captured;
    }

    // Feedback to the live overlays, the moment the point exists. align_solve
    // only fires after BOTH stations, so without this the Aux window would have
    // nothing to show while station 2 is being marked — which is exactly when
    // an operator wants to see that E1 landed where they expected.
    ctx.Wf().QueueAlignPoint(proc->uid, station, e);

    ctx.CtxSet(PointKey(prefix, station, "gx"), e.gx);
    ctx.CtxSet(PointKey(prefix, station, "gy"), e.gy);
    ctx.CtxSet(PointKey(prefix, station, "px"), e.px);
    ctx.CtxSet(PointKey(prefix, station, "py"), e.py);
    ctx.CtxSet(PointKey(prefix, station, "wx"), e.wx);
    ctx.CtxSet(PointKey(prefix, station, "wy"), e.wy);
    // Written or ERASED, never left: a stale gz from an earlier capture under
    // this prefix would hand the solve a height this point was never marked at.
    if (e.hasZ) ctx.CtxSet(PointKey(prefix, station, "gz"), e.gz);
    else        ctx.CtxErase(PointKey(prefix, station, "gz"));

    FD_INFO("align", "'%s' E%d (%s): gantry (%.4f, %.4f) + mark (%+.4f, %+.4f) "
                     "= (%.4f, %.4f) mm",
            proc->name.c_str(), station, manual ? "manual" : "auto",
            e.gx, e.gy, pick.dxMM, pick.dyMM, e.wx, e.wy);
    return CommandResult::Success();
}

// ---- align_solve ---------------------------------------------------------

CommandResult ExecAlignSolve(ExecutionContext& ctx, const ParamValues& p)
{
    const AlignProc* proc = ctx.FindAlign(p.GetString("alignment"));
    if (!proc) return CommandResult::Fail("alignment procedure not found: '" +
                                          p.GetString("alignment") + "'");

    const std::string prefix = p.GetString("ctx_prefix");
    AlignPoint e1, e2;
    bool ok = true, f = false;
    e1.gx = CtxNumber(ctx, PointKey(prefix, 1, "gx"), &f); ok = ok && f;
    e1.gy = CtxNumber(ctx, PointKey(prefix, 1, "gy"), &f); ok = ok && f;
    e1.px = CtxNumber(ctx, PointKey(prefix, 1, "px"), &f);
    e1.py = CtxNumber(ctx, PointKey(prefix, 1, "py"), &f);
    e1.wx = CtxNumber(ctx, PointKey(prefix, 1, "wx"), &f); ok = ok && f;
    e1.wy = CtxNumber(ctx, PointKey(prefix, 1, "wy"), &f); ok = ok && f;
    e2.gx = CtxNumber(ctx, PointKey(prefix, 2, "gx"), &f); ok = ok && f;
    e2.gy = CtxNumber(ctx, PointKey(prefix, 2, "gy"), &f); ok = ok && f;
    e2.px = CtxNumber(ctx, PointKey(prefix, 2, "px"), &f);
    e2.py = CtxNumber(ctx, PointKey(prefix, 2, "py"), &f);
    e2.wx = CtxNumber(ctx, PointKey(prefix, 2, "wx"), &f); ok = ok && f;
    e2.wy = CtxNumber(ctx, PointKey(prefix, 2, "wy"), &f); ok = ok && f;
    // Z is optional: a device without one, or a capture from before Z was
    // recorded, simply yields a frame with no working height.
    e1.gz = CtxNumber(ctx, PointKey(prefix, 1, "gz"), &e1.hasZ);
    e2.gz = CtxNumber(ctx, PointKey(prefix, 2, "gz"), &e2.hasZ);
    if (!ok)
        return CommandResult::Fail("no captured E1/E2 in ctx under prefix '" + prefix +
                                   "' - run align_station for both stations first");

    AlignResult r;
    std::string err;
    if (!SolveAlignFrame(e1, e2, &r, &err)) {
        ctx.CtxSet(prefix + "_valid", false);
        return CommandResult::Fail(err);
    }

    ctx.CtxSet(prefix + "_origin_x", r.originX);
    ctx.CtxSet(prefix + "_origin_y", r.originY);
    ctx.CtxSet(prefix + "_theta_deg", r.thetaDeg);
    ctx.CtxSet(prefix + "_span_mm", r.spanMm);
    if (r.hasZ) {
        ctx.CtxSet(prefix + "_z", r.z);
        // E1 and E2 are normally marked at one focus height. A large split says
        // the part is tilted or one station was refocused - worth a line in the log.
        const double dz = std::fabs(r.e2.gz - r.e1.gz);
        if (dz > 0.05)
            FD_WARN("align", "'%s': E1/E2 were marked %.4f mm apart in Z - the frame's "
                             "working Z is their mean, %.4f", proc->name.c_str(), dz, r.z);
    } else {
        ctx.CtxErase(prefix + "_z");
    }
    ctx.CtxSet(prefix + "_valid", true);

    // The guard band's verdict, into ctx beside the frame. It is REPORTED here,
    // not enforced: by the time a solve runs, an out-of-band point has already
    // been shown to the operator on the image and consciously accepted, or an
    // auto station without a fallback has already failed. Failing the step now
    // would only throw away a frame somebody deliberately kept.
    //
    // The ctx keys are what let a recipe act on it anyway — `dut_record` writes
    // the verdict onto the part's record, and a branch can refuse to carry on.
    double spanErr = 0.0, thetaErr = 0.0;
    const bool inBand = AlignGuardCheck(proc->guard, r.spanMm, r.thetaDeg,
                                        &spanErr, &thetaErr);
    ctx.CtxSet(prefix + "_guard_ok", inBand);
    ctx.CtxSet(prefix + "_span_err_mm", spanErr);
    ctx.CtxSet(prefix + "_theta_err_deg", thetaErr);
    if (!inBand) {
        FD_WARN("align", "'%s' solved OUT OF BAND: span %.4f (%.4f +/- %.4f), "
                         "theta %.4f (%.4f +/- %.4f)",
                proc->name.c_str(), r.spanMm, proc->guard.spanMm, proc->guard.spanTolMm,
                r.thetaDeg, proc->guard.thetaDeg, proc->guard.thetaTolDeg);
    }

    // The LIVE frame — what align_apply / move_local will actually use. Set
    // unconditionally, even when `save` is off: `save` decides whether the
    // frame is RECORDED, not whether this session may move against the part it
    // just measured. This is also what makes the frame outlive the run that
    // taught it, so a program can align in one slot and work in the next.
    // `keep` lets it survive a program start (never DUT open / product load).
    ctx.Wf().SetLiveFrame(proc->uid, r, p.GetBool("keep"));
    ctx.CtxSet(prefix + "_kept", p.GetBool("keep"));

    if (p.GetBool("save")) {
        // The document is UI-thread state: hand the frame over and let
        // App::Update store and persist it. That copy is a RECORD only — no run
        // ever consumes it (see Workflow::SetLiveFrame).
        ctx.Wf().QueueAlignResult(proc->uid, r);
    }
    FD_INFO("align", "'%s' solved: origin (%.4f, %.4f) mm, theta %.4f deg, span %.4f mm",
            proc->name.c_str(), r.originX, r.originY, r.thetaDeg, r.spanMm);
    return CommandResult::Success();
}

// ---- align_apply ---------------------------------------------------------

// Resolve the procedure AND its LIVE frame — the one check every consumer of a
// frame owes the operator.
//
// The frame comes from Workflow, never from `proc->result`. The document copy
// is a record of the LAST teach, which may have been against a different part
// on a different day; moving against it would look exactly like a correct move
// and be wrong by however far the part has shifted. Only a frame taught in this
// session counts, and it is cleared whenever the part might have changed.
// `allowCarried` = the step's `use_kept_frame`: accept a frame an earlier
// program run taught and marked KEPT. Off, only a frame taught in THIS program
// run (or outside any program) is usable.
const AlignProc* LiveProc(ExecutionContext& ctx, const std::string& name,
                          AlignResult* frame, std::string* err, bool allowCarried = false)
{
    const AlignProc* proc = ctx.FindAlign(name);
    if (!proc) { *err = "alignment procedure not found: '" + name + "'"; return nullptr; }
    if (!ctx.Wf().LiveFrame(proc->uid, frame, allowCarried)) {
        if (ctx.Wf().LiveFrameCarried(proc->uid)) {
            *err = "alignment '" + proc->name + "' has only a KEPT frame from an "
                   "earlier program run - turn on use_kept_frame on this step to "
                   "move against it, or re-run the alignment";
            return nullptr;
        }
        *err = "alignment '" + proc->name + "' has no live frame - run it in THIS "
               "program first (a saved frame is a record of the last teach, not a "
               "licence to move)";
        return nullptr;
    }
    return proc;
}

CommandResult ExecAlignApply(ExecutionContext& ctx, const ParamValues& p)
{
    std::string err;
    AlignResult frame;
    const AlignProc* proc = LiveProc(ctx, p.GetString("alignment"), &frame, &err,
                                      p.GetBool("use_kept_frame"));
    if (!proc) return CommandResult::Fail(err);

    double mx = 0.0, my = 0.0;
    AlignLocalToMachine(frame, p.GetDouble("x_mm"), p.GetDouble("y_mm"), &mx, &my);
    const std::string prefix = p.GetString("ctx_prefix");
    ctx.CtxSet(prefix + "_x", mx);
    ctx.CtxSet(prefix + "_y", my);
    return CommandResult::Success();
}

// ---- nominal_point -------------------------------------------------------
//
// Where a nominal IS, in machine XY, with nothing marked and nothing moved.
//
// `align_apply` one function up already converts a local coordinate through the
// frame — but it takes two raw numbers. This one names the NOMINAL, which is
// what a registered device's first approach needs: to send a gripper to n1,
// something has to be able to say where n1 is before anything has looked at
// the object.
//
// There is deliberately no `_z`. The frame is 2-D rigid and an AlignNominal
// stores lx/ly and nothing else (core/AlignFrame.h), so a Z reported here would
// be invented, and an invented Z on a gripper approach is a lens driven into a
// substrate. A caller that needs one takes it from a focused camera reading or
// from a stored placement offset — see docs/device-registration-design.md.
CommandResult ExecNominalPoint(ExecutionContext& ctx, const ParamValues& p)
{
    std::string err;
    AlignResult frame;
    const AlignProc* proc = LiveProc(ctx, p.GetString("alignment"), &frame, &err,
                                      p.GetBool("use_kept_frame"));
    if (!proc) return CommandResult::Fail(err);

    const AlignNominal* nom = FindAlignNominal(*proc, p.GetString("nominal"));
    if (!nom)
        return CommandResult::Fail("alignment '" + proc->name + "' has no nominal '" +
                                   p.GetString("nominal") + "'");

    double mx = 0.0, my = 0.0;
    AlignLocalToMachine(frame, nom->lx, nom->ly, &mx, &my);

    const std::string prefix = p.GetString("ctx_prefix");
    ctx.CtxSet(prefix + "_x", mx);
    ctx.CtxSet(prefix + "_y", my);
    // The local pair travels with it. A nominal taught against the wrong feature
    // shows up here as two numbers that do not match the drawing; the machine
    // pair alone can never show that, because it is supposed to change with
    // every part.
    ctx.CtxSet(prefix + "_lx", nom->lx);
    ctx.CtxSet(prefix + "_ly", nom->ly);
    ctx.CtxSet(prefix + "_name", nom->name);

    FD_INFO("align", "'%s': nominal '%s' local (%.4f, %.4f) -> machine (%.4f, %.4f)",
            proc->name.c_str(), nom->name.c_str(), nom->lx, nom->ly, mx, my);
    return CommandResult::Success();
}

// ---- move_local ----------------------------------------------------------

// The one body behind move_local and move_to_nominal: local (lx, ly) through
// the live frame to machine XY, plus the optional machine Z, on `device` (empty
// = the procedure's own). One copy, so the two cannot drift on velocity,
// approach or the Z rule.
CommandResult MoveToLocalPoint(ExecutionContext& ctx, const ParamValues& p,
                               const AlignProc& proc, const AlignResult& frame,
                               const std::string& device,
                               double lx, double ly, const char* what,
                               const double* defaultZ = nullptr)
{
    std::string err;
    int idx = -1;
    IMotionController* m = ResolveMotion(ctx, device, &idx, &err);
    if (!m) return CommandResult::Fail(err);

    double mx = 0.0, my = 0.0;
    AlignLocalToMachine(frame, lx, ly, &mx, &my);

    std::vector<std::pair<std::string, double>> targets = {{"X", mx}, {"Y", my}};
    // A typed z_mm always wins; otherwise the caller's default Z (the frame's
    // working height, for move_to_nominal); otherwise Z is left where it is.
    if (p.Has("z_mm"))  targets.emplace_back("Z", p.GetDouble("z_mm"));
    else if (defaultZ)  targets.emplace_back("Z", *defaultZ);

    const double vel = p.GetDouble("velocity_mm_s");
    if (vel > 0.0 && !ctx.Wf().SetJogVelocity(static_cast<std::size_t>(idx), vel))
        return CommandResult::Fail("velocity change rejected");

    FD_INFO("align", "'%s': %s local (%.4f, %.4f) -> machine (%.4f, %.4f) on %s",
            proc.name.c_str(), what, lx, ly, mx, my, device.c_str());
    return MoveWithApproach(ctx, m, idx, targets, p, what);
}

CommandResult ExecMoveLocal(ExecutionContext& ctx, const ParamValues& p)
{
    std::string err;
    AlignResult frame;
    const AlignProc* proc = LiveProc(ctx, p.GetString("alignment"), &frame, &err,
                                      p.GetBool("use_kept_frame"));
    if (!proc) return CommandResult::Fail(err);
    const std::string device = p.GetString("device").empty() ? proc->device
                                                             : p.GetString("device");
    return MoveToLocalPoint(ctx, p, *proc, frame, device, p.GetDouble("x_mm"),
                            p.GetDouble("y_mm"), "the local coordinate");
}

// ---- move_to_nominal -----------------------------------------------------
// move_local with the coordinates taken from a taught nominal instead of typed:
// the step names the POINT, so a re-taught nominal moves every recipe with it.
CommandResult ExecMoveToNominal(ExecutionContext& ctx, const ParamValues& p)
{
    std::string err;
    AlignResult frame;
    const AlignProc* proc = LiveProc(ctx, p.GetString("alignment"), &frame, &err,
                                      p.GetBool("use_kept_frame"));
    if (!proc) return CommandResult::Fail(err);
    const AlignNominal* nom = FindAlignNominal(*proc, p.GetString("nominal"));
    if (!nom)
        return CommandResult::Fail("alignment '" + proc->name + "' has no nominal '" +
                                   p.GetString("nominal") + "'");
    const std::string what = "nominal '" + nom->name + "'";
    // The procedure's OWN device, never another: the nominal was measured
    // through this device's camera, and another device's origin has never been
    // related to it (that crossing is a DeviceReg's job - move_to_point).
    //
    // Z: the frame's working height (the mean gantry Z E1/E2 were marked at)
    // unless z_mm is typed or z_from_frame is off. Axis space both ways - read
    // off this device, commanded to this device - so no transform touches it.
    const double* defZ = nullptr;
    if (p.GetBool("z_from_frame")) {
        if (frame.hasZ) {
            defZ = &frame.z;
        } else if (!p.Has("z_mm")) {
            FD_INFO("align", "'%s': frame has no recorded Z (taught before Z was "
                             "recorded, or the device has none) - Z left where it is; "
                             "re-run the alignment to record one",
                    proc->name.c_str());
        }
    }
    return MoveToLocalPoint(ctx, p, *proc, frame, proc->device, nom->lx, nom->ly,
                            what.c_str(), defZ);
}

// ---- pick_point ------------------------------------------------------------
//
// The crosshair pick, with no alignment procedure behind it: show the operator
// the live image, take the mark, and report where that mark IS in machine mm.
//
// It lives in this file rather than in a new one for a single reason —
// CaptureWorldPoint. That function's comment says a second copy reading the pose
// in a different order would be wrong in a way only a part placed 20 um out
// would show, and a needle offset measured through a second copy would be wrong
// the same way. Same maths, same file, one implementation.
//
// Nothing moves here, exactly as in align_station. The feature has
// to be in view already; the pick window's own right-click-drive is how an
// operator brings a stray one in.
CommandResult ExecPickPoint(ExecutionContext& ctx, const ParamValues& p)
{
    const std::string device = p.GetString("device");
    std::string err;
    int idx = -1;
    IMotionController* m = ResolveMotion(ctx, device, &idx, &err);
    if (!m) return CommandResult::Fail(err);

    ExecutionContext::PickRequest req;
    req.label     = ctx.Substitute(p.GetString("label"));
    if (req.label.empty()) req.label = "the point";
    req.alignName = ctx.Substitute(p.GetString("title"));
    if (req.alignName.empty()) req.alignName = device;
    req.message   = ctx.Substitute(p.GetString("message"));
    if (req.message.empty())
        req.message = "Put the crosshair on " + req.label + ", then OK";
    // stepIcons stays false: this is one mark, not two corners of a part.

    // An optional aim point, drawn as the pick window's target — and, when it is
    // outside the field of view, as an edge arrow with its distance. For a needle
    // imprint that is the difference between "the dot is 4 mm to the left" and a
    // blank grey picture the operator has to hunt across.
    if (p.GetBool("hint")) {
        ExecutionContext::PickHint h;
        h.wx    = p.GetDouble("hint_x_mm");
        h.wy    = p.GetDouble("hint_y_mm");
        h.label = req.label;
        req.hints.push_back(std::move(h));
    }

    // The live residual readout. Separate from the hint on purpose: a hint is
    // where to LOOK, and is drawn on the image; this is how far off the mark
    // still is, in numbers, including a Z the picture cannot show. The device
    // travels with it because the window has to read that Z from somewhere and
    // guessing "the gantry" would be wrong the day a second camera is added.
    if (p.GetBool("target")) {
        req.target.have   = true;
        req.target.x      = p.GetDouble("target_x_mm");
        req.target.y      = p.GetDouble("target_y_mm");
        req.target.z      = p.GetDouble("target_z_mm");
        req.target.tolMm  = p.GetDouble("target_tol_mm");
        req.target.device = device;
    }

    ExecutionContext::PickResult pick;
    if (!ctx.AwaitPick(req, &pick)) return CommandResult::Fail("cancelled by operator");

    AlignPoint e;
    const CommandResult captured = CaptureWorldPoint(ctx, m, pick, &e);
    if (!captured.ok) return captured;

    // Underscore keys, not the dots CommandsVision uses: this is the align
    // family's handshake and its siblings all spell a captured point this way.
    const std::string prefix = p.GetString("ctx_prefix");
    ctx.CtxSet(prefix + "_gx", e.gx);
    ctx.CtxSet(prefix + "_gy", e.gy);
    ctx.CtxSet(prefix + "_px", e.px);
    ctx.CtxSet(prefix + "_py", e.py);
    ctx.CtxSet(prefix + "_dx_mm", pick.dxMM);
    ctx.CtxSet(prefix + "_dy_mm", pick.dyMM);
    ctx.CtxSet(prefix + "_wx", e.wx);
    ctx.CtxSet(prefix + "_wy", e.wy);

    FD_INFO("align", "picked %s: %s (%.4f, %.4f) + mark (%+.4f, %+.4f) = (%.4f, %.4f) mm",
            req.label.c_str(), device.c_str(), e.gx, e.gy, pick.dxMM, pick.dyMM,
            e.wx, e.wy);
    return CommandResult::Success();
}

// ---- nominal_view ----------------------------------------------------------
//
// Drive the CAMERA to the nominal's taught focus pose (`algn_<procUid>_<nomUid>_c`,
// hidden): X, Y and the focused Z, plus optional offsets. An untaught pose is not
// an error unless `required` is set.
CommandResult DriveToNominalPose(ExecutionContext& ctx, const ParamValues& p,
                                 double offX, double offY, double offZ)
{
    const AlignProc* proc = ctx.FindAlign(p.GetString("alignment"));
    if (!proc) return CommandResult::Fail("alignment procedure not found: '" +
                                          p.GetString("alignment") + "'");
    const AlignNominal* nom = FindAlignNominal(*proc, p.GetString("nominal"));
    if (!nom)
        return CommandResult::Fail("alignment '" + proc->name + "' has no nominal '" +
                                   p.GetString("nominal") + "'");

    const std::string uid    = nom->camUid;
    const char* const what   = "camera focus position";
    const char* const button = "Save cam focus";

    if (uid.empty()) {
        if (p.GetBool("required"))
            return CommandResult::Fail("nominal '" + nom->name + "' has no " + what +
                                       " taught (Alignment page: " + button + ")");
        FD_INFO("align", "'%s': nominal '%s' has no %s - leaving %s where it is",
                proc->name.c_str(), nom->name.c_str(), what, proc->device.c_str());
        return CommandResult::Success();
    }

    std::string err;
    int idx = -1;
    IMotionController* m = ResolveMotion(ctx, proc->device, &idx, &err);
    if (!m) return CommandResult::Fail(err);

    const SavedPos* pos = ctx.FindPosition(proc->device, uid);
    if (!pos)
        return CommandResult::Fail("nominal '" + nom->name + "' names a " + what +
                                   " that no longer exists - re-teach it");
    auto targets = TargetsOf(*pos, m);
    if (targets.empty())
        return CommandResult::Fail(std::string(what) + " shares no axes with '" +
                                   proc->device + "'");

    // The offsets, added to the taught pose in the DEVICE'S OWN AXIS SPACE -
    // the same thing move_to_position_offset does to a saved position, and for
    // the same reason: "that pose, but 0.5 mm higher" is a step somebody writes
    // constantly, and the alternative is a second taught pose per variation.
    //
    // Axis space, not global: this pose was captured axis-by-axis off the
    // device, and gantry-main's transform inverts Z, so a number that took a
    // detour through the global frame would descend where it meant to retreat.
    // An axis the pose does not command is not invented here either - offsetting
    // an axis that was never taught would be a move to a place nobody chose.
    if (offX != 0.0 || offY != 0.0 || offZ != 0.0) {
        bool any = false;
        for (auto& [axis, v] : targets) {
            if (axis == "X")      { v += offX; any = any || offX != 0.0; }
            else if (axis == "Y") { v += offY; any = any || offY != 0.0; }
            else if (axis == "Z") { v += offZ; any = any || offZ != 0.0; }
        }
        if (!any)
            return CommandResult::Fail(std::string(what) + " commands none of the axes "
                                       "the offsets name - check X/Y/Z against '" +
                                       proc->device + "'");
        FD_INFO("align", "'%s': %s offset by (%+.4f, %+.4f, %+.4f) in '%s' axis space",
                proc->name.c_str(), what, offX, offY, offZ, proc->device.c_str());
    }

    if (!ctx.Wf().MoveToPosition(static_cast<std::size_t>(idx), targets))
        return CommandResult::Fail(std::string("move to the ") + what + " rejected");
    const auto arrived = WaitArrive(ctx, m, targets, p.GetDouble("tol_mm"),
                                    p.GetInt("timeout_ms"));
    if (arrived != ExecutionContext::WaitResult::Ok) {
        ctx.Wf().StopMotion(static_cast<std::size_t>(idx));   // defined state on any abort
        return CommandResult::Fail(arrived == ExecutionContext::WaitResult::Stopped
                                       ? "stopped"
                                       : std::string("timeout driving to the ") + what);
    }
    // Settle before anything looks through the lens: a mark taken while the
    // stage is still ringing carries that ring into the placement.
    const CommandResult settled = WaitSettle(ctx, m, idx);
    if (!settled.ok) return settled;

    FD_INFO("align", "'%s': %s at the %s for '%s'", proc->name.c_str(),
            proc->device.c_str(), what, nom->name.c_str());
    return CommandResult::Success();
}

// The focused view of the nominal ITSELF. Its Z is the reason it exists: the
// local frame is 2-D, so a nominal converted through it gives an XY and no
// height at all — this is the one place the height that focuses this point on
// this substrate was ever written down.
CommandResult ExecNominalView(ExecutionContext& ctx, const ParamValues& p)
{
    return DriveToNominalPose(ctx, p,
                              p.GetDouble("offset_x_mm"), p.GetDouble("offset_y_mm"),
                              p.GetDouble("offset_z_mm"));
}

// The opt-in every frame CONSUMER carries: one spec, so the name and the help
// cannot drift between align_apply, nominal_point and move_local.
ParamSpec UseKeptFrameParam()
{
    return {"use_kept_frame", ParamType::Bool, false, false, 0, 0, {},
            "also accept a frame an EARLIER program run taught with align_run "
            "keep_frame on; off = only a frame taught in this program run"};
}

} // namespace

void RegisterAlignCommands(CommandRegistry& reg, Workflow& wf, HardwareManager& hw)
{
    (void)wf;   // every action goes through ExecutionContext::Wf() at run time
    (void)hw;
    using PT = ParamType;

    reg.Register({"align_station", "Align",
                  "Drive to an alignment station and capture its E point",
                  {
                      {"alignment", PT::AlignUid, true, {}, 0, 0, {},
                       "alignment procedure (uid or name)"},
                      {"station", PT::IntChoice, true, {}, 1, 2,
                       {"E1 - origin", "E2 - X+"},
                       "1 = E1 (origin), 2 = E2 (X+)"},
                      {"message", PT::String, false, "", 0, 0, {},
                       "instruction shown to the operator ({key} substitutes ctx values)"},
                      {"timeout_ms", PT::Int, false, 60000, 100, 600000, {}, "arrival timeout"},
                      {"tol_mm", PT::Double, false, kMoveTolMM, 0.0001, 1.0, {},
                       "arrival tolerance"},
                      {"ctx_prefix", PT::String, false, "frame", 0, 0, {},
                       "captured point goes to <prefix>_e<N>_gx/gy/px/py/wx/wy"},
                      {"fresh_frames", PT::Int, false, 2, 0, 10, {},
                       "auto stations only: new frames required after arrival "
                       "before detecting (0 = use the latest, blind)"},
                      PreDelayParam(), PostDelayParam(),
                  },
                  CommandSpec::Needs::None, ExecAlignStation});

    reg.Register({"align_solve", "Align",
                  "Build the local frame (O1) from the captured E1/E2 and store it",
                  {
                      {"alignment", PT::AlignUid, true, {}, 0, 0, {},
                       "alignment procedure (uid or name)"},
                      {"ctx_prefix", PT::String, false, "frame", 0, 0, {},
                       "reads <prefix>_e1_*/<prefix>_e2_*, writes <prefix>_origin_x/_origin_y/"
                       "_theta_deg/_span_mm/_valid"},
                      {"save", PT::Bool, false, true, 0, 0, {},
                       "persist the taught frame into the procedure document"},
                      {"keep", PT::Bool, false, false, 0, 0, {},
                       "KEEP the live frame across program starts (steps must opt in "
                       "with use_kept_frame); DUT open, product load and app close "
                       "still clear it"},
                  },
                  CommandSpec::Needs::None, ExecAlignSolve});

    reg.Register({"align_apply", "Align",
                  "Convert a local (O1) coordinate to machine XY into the run context",
                  {
                      {"alignment", PT::AlignUid, true, {}, 0, 0, {},
                       "alignment procedure (uid or name)"},
                      {"x_mm", PT::Double, true, {}, 0, 0, {}, "local X (mm from E1 along E1->E2)"},
                      {"y_mm", PT::Double, true, {}, 0, 0, {}, "local Y (mm, X rotated +90 deg)"},
                      {"ctx_prefix", PT::String, false, "local", 0, 0, {},
                       "result goes to <prefix>_x / <prefix>_y"},
                      UseKeptFrameParam(),
                  },
                  CommandSpec::Needs::None, ExecAlignApply});

    // align_apply for a nominal instead of two typed numbers. Registered here,
    // beside it, because it is the same conversion — what it adds is that the
    // caller does not have to already know the coordinates of the thing it is
    // asking about, which is what stood between a taught nominal and a
    // registered device's first approach.
    reg.Register({"nominal_point", "Align",
                  "Where a nominal IS in machine XY, through the live frame - no "
                  "marks and no motion (<prefix>_x/_y/_lx/_ly/_name)",
                  {
                      {"alignment", PT::AlignUid, true, {}, 0, 0, {},
                       "alignment procedure (uid or name)"},
                      {"nominal", PT::NominalUid, true, {}, 0, 0, {},
                       "the nominal to resolve (uid or name)"},
                      {"ctx_prefix", PT::String, false, "nom", 0, 0, {},
                       "result goes to <prefix>_x / _y / _lx / _ly / _name"},
                      UseKeptFrameParam(),
                  },
                  CommandSpec::Needs::None, ExecNominalPoint});

    reg.Register({"move_local", "Align",
                  "Move to a coordinate expressed in an alignment's local frame",
                  {
                      {"alignment", PT::AlignUid, true, {}, 0, 0, {},
                       "alignment procedure (uid or name)"},
                      {"device", PT::DeviceMotion, false, "", 0, 0, {},
                       "motion device; empty = the procedure's own device"},
                      {"x_mm", PT::Double, true, {}, 0, 0, {}, "local X (mm)"},
                      {"y_mm", PT::Double, true, {}, 0, 0, {}, "local Y (mm)"},
                      {"z_mm", PT::Double, false, {}, 0, 0, {},
                       "optional machine Z (the frame is 2-D; Z is not transformed)"},
                      UseKeptFrameParam(),
                      {"timeout_ms", PT::Int, false, 60000, 100, 600000, {}, "arrival timeout"},
                      {"tol_mm", PT::Double, false, kMoveTolMM, 0.0001, 1.0, {},
                       "arrival tolerance"},
                      {"velocity_mm_s", PT::Double, false, 0.0, 0, 0, {},
                       "optional velocity for this move; 0 = keep current"},
                      ApproachVelocityParam(),
                      ApproachMmParam(),
                      ApproachPctParam(),
                      PreDelayParam(), PostDelayParam(),
                  },
                  CommandSpec::Needs::None, ExecMoveLocal});

    // move_local for a taught nominal: the step names the point, not numbers,
    // so a re-teach moves every recipe that uses it. No device parameter - it
    // always moves the procedure's own device, which measured the point.
    reg.Register({"move_to_nominal", "Align",
                  "Move the alignment's device (the camera) onto a taught nominal, "
                  "through the live frame",
                  {
                      {"alignment", PT::AlignUid, true, {}, 0, 0, {},
                       "alignment procedure (uid or name)"},
                      {"nominal", PT::NominalUid, true, {}, 0, 0, {},
                       "the nominal to move onto (uid or name)"},
                      {"z_from_frame", PT::Bool, false, true, 0, 0, {},
                       "drive Z to the frame's working height - the gantry Z E1/E2 "
                       "were marked at (their mean). Off = leave Z where it is"},
                      {"z_mm", PT::Double, false, {}, 0, 0, {},
                       "optional machine Z; when set it WINS over z_from_frame"},
                      UseKeptFrameParam(),
                      {"timeout_ms", PT::Int, false, 60000, 100, 600000, {}, "arrival timeout"},
                      {"tol_mm", PT::Double, false, kMoveTolMM, 0.0001, 1.0, {},
                       "arrival tolerance"},
                      {"velocity_mm_s", PT::Double, false, 0.0, 0, 0, {},
                       "optional velocity for this move; 0 = keep current"},
                      ApproachVelocityParam(),
                      ApproachMmParam(),
                      ApproachPctParam(),
                      PreDelayParam(), PostDelayParam(),
                  },
                  CommandSpec::Needs::None, ExecMoveToNominal});

    // Drives to the nominal's taught FOCUS pose (a fixed X/Y/Z, not through the
    // frame). move_to_nominal is the frame-following move.
    reg.Register({"nominal_view", "Align",
                  "Drive the camera to a nominal's taught FOCUS pose - the point "
                  "itself, in focus, including the Z - plus an optional XYZ offset",
                  {
                      {"alignment", PT::AlignUid, true, {}, 0, 0, {},
                       "alignment procedure - its device is the one carrying the camera"},
                      {"nominal", PT::NominalUid, true, {}, 0, 0, {},
                       "nominal whose camera FOCUS position to drive to (Alignment "
                       "page: Centre camera, focus, Save cam focus)"},
                      {"required", PT::Bool, false, false, 0, 0, {},
                       "fail when the nominal has no focus pose taught; off = leave "
                       "the camera where it is"},
                      {"offset_x_mm", PT::Double, false, 0.0, -100.0, 100.0, {},
                       "added to the taught pose's X, in the DEVICE'S OWN axis space "
                       "(not global) - the same thing move_to_position_offset does"},
                      {"offset_y_mm", PT::Double, false, 0.0, -100.0, 100.0, {},
                       "added to the taught pose's Y, device axis space"},
                      {"offset_z_mm", PT::Double, false, 0.0, -100.0, 100.0, {},
                       "added to the taught pose's Z, device axis space - a standoff "
                       "from the focused height. Which way +Z counts is a property of "
                       "the machine (gantry-main's transform inverts Z), so try a "
                       "small value and watch before trusting a large one"},
                      {"timeout_ms", PT::Int, false, 60000, 100, 600000, {}, "arrival timeout"},
                      {"tol_mm", PT::Double, false, kMoveTolMM, 0.0001, 1.0, {},
                       "arrival tolerance"},
                      PreDelayParam(), PostDelayParam(),
                  },
                  CommandSpec::Needs::None, ExecNominalView});

    reg.Register({"pick_point", "Vision",
                  "Ask the operator to mark a point on the live image; report it in "
                  "machine mm as <prefix>_wx/_wy",
                  {
                      {"device", PT::DeviceMotion, true, {}, 0, 0, {},
                       "the device the camera rides on - its pose is what the mark is "
                       "measured against"},
                      {"label", PT::String, false, "", 0, 0, {},
                       "what the operator is marking, e.g. \"the needle imprint\""},
                      {"message", PT::String, false, "", 0, 0, {},
                       "instruction line; empty builds one from the label"},
                      {"title", PT::String, false, "", 0, 0, {},
                       "window subtitle; empty uses the device name"},
                      {"hint", PT::Bool, false, false, 0, 0, {},
                       "draw an aim point at hint_x_mm/hint_y_mm - shown as an edge "
                       "arrow with its distance when it is outside the field of view"},
                      {"hint_x_mm", PT::Double, false, 0.0, 0, 0, {}, "aim point X, machine mm"},
                      {"hint_y_mm", PT::Double, false, 0.0, 0, 0, {}, "aim point Y, machine mm"},
                      {"target", PT::Bool, false, false, 0, 0, {},
                       "show the LIVE residual against target_x/y/z_mm while the "
                       "window is open - dX/dY from the crosshair, dZ from the "
                       "device's own Z, so a hand focus can be watched in"},
                      {"target_x_mm", PT::Double, false, 0.0, 0, 0, {}, "predicted X, machine mm"},
                      {"target_y_mm", PT::Double, false, 0.0, 0, 0, {}, "predicted Y, machine mm"},
                      {"target_z_mm", PT::Double, false, 0.0, 0, 0, {}, "predicted Z, machine mm"},
                      {"target_tol_mm", PT::Double, false, 0.0, 0, 0, {},
                       "what counts as on target; 0 shows the number with no verdict"},
                      {"ctx_prefix", PT::String, false, "pick", 0, 0, {},
                       "ctx prefix for the captured point"},
                  },
                  CommandSpec::Needs::Camera, ExecPickPoint});

    // Typed one-step wrapper over the procedure script — the `xcan` pattern, so
    // the procedure stays an editable .lua while a recipe runs it as one step.
    CommandRegistry* rp = &reg;
    reg.Register({"align_run", "Align",
                  "Run a whole alignment procedure (alignment_procedure.lua)",
                  {
                      {"alignment", PT::AlignUid, true, {}, 0, 0, {},
                       "alignment procedure (uid or name)"},
                      {"ctx_prefix", PT::String, false, "frame", 0, 0, {},
                       "ctx prefix the script captures and solves under"},
                      {"file", PT::ScriptFile, false,
                       "config/scripts/alignment_procedure.lua", 0, 0, {},
                       "script override; this name is a STANDARD procedure and always "
                       "resolves to the shipped file - point it at a different name "
                       "to run something else"},
                      {"keep_frame", PT::Bool, false, false, 0, 0, {},
                       "keep this frame across PROGRAM STARTS, for later runs whose "
                       "steps set use_kept_frame. Still cleared by DUT open, product "
                       "load and app close"},
                  },
                  CommandSpec::Needs::None,
                  [rp](ExecutionContext& c, const ParamValues& p) {
                      c.CtxSet("align_name", p.GetString("alignment"));
                      c.CtxSet("align_prefix", p.GetString("ctx_prefix"));
                      c.CtxSet("align_keep", p.GetBool("keep_frame"));
                      // A product's own alignment_procedure.lua wins over the
                      // shipped one — the Alignment page's Run button already
                      // prefers it, and "the same procedure" must not mean a
                      // different file depending on which surface launched it.
                      const std::string file = c.ResolveScript(p.GetString("file"));
                      const std::string err = RunLuaFile(file, *rp, c, {}, false, nullptr);
                      return err.empty() ? CommandResult::Success()
                                         : CommandResult::Fail(err);
                  }});

}

} // namespace firedog
