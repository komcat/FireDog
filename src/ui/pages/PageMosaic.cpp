// PageMosaic.cpp : the Mosaic page (a full-page takeover, F11). Definitions of
// App members — see App.h. Design: docs/mosaic-design.md.
//
// One camera field of view is a few millimetres; a part is inches. This page
// photographs a whole area by walking the gantry camera over it frame by frame
// and shows the frames as ONE picture you can pan, zoom, mark up and export.
//
//   left    the save folder, the AREAS (3 or 4 taught corners, a Z mode, the
//           overlap), the plan the walk would take, Capture / Stop, and the
//           mosaics already in the save folder
//   centre  the picture: wheel zooms about the cursor, middle-drag (or the Pan
//           tool, or Space + drag) pans; the Marker tool drops a named point,
//           the Crop label tool drags out a named rectangle
//   right   the open mosaic's markers and labels, and the exports
//
// ---- Who writes what ------------------------------------------------------------
//
// The area record is machine state (config/mosaic.local.json): this page is its
// only writer and saves on every change, because the capture READS the file at
// Start. Every editing surface is disabled while orch_.Busy(), so the page and a
// running capture never write and read it at once.
//
// A capture is the mosaic_capture command run as a one-step recipe (the quick
// autofocus pattern): Stop, Busy and the safety abort come with it. Its tiles
// arrive through Workflow's queue as quarter-size previews and fill the picture
// in while the gantry walks (UpdateMosaic, from App::Update).
//
// The picture is never one texture: a 2" x 2" mosaic is ~20k x 20k px. Each
// tile is drawn at the resolution the zoom needs — 1/16, 1/4 or full — and the
// bigger two are paged in by MosaicWorker (decoding off the UI thread) and out
// again least-recently-used.
#include "App.h"

#include "imgui.h"

#include "core/Log.h"
#include "core/Product.h"   // GenerateUid / NowStamp
#include "vision/MosaicLayout.h"
#include "ui/Widgets.h"

#include <algorithm>
#include <cctype>
#include <cfloat>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <map>
#include <set>
#include <string>
#include <system_error>
#include <vector>

namespace firedog {

namespace fs = std::filesystem;

namespace {

const ImVec4 kGood(0.35f, 0.85f, 0.45f, 1.0f);
const ImVec4 kWarn(0.98f, 0.72f, 0.32f, 1.0f);
const ImVec4 kBad (0.95f, 0.45f, 0.35f, 1.0f);

constexpr ImU32 kColOutline  = IM_COL32(250, 210, 60, 230);
constexpr ImU32 kColGrid     = IM_COL32(120, 170, 255, 110);
constexpr ImU32 kColMarker   = IM_COL32(255, 80, 80, 255);
constexpr ImU32 kColLabel    = IM_COL32(80, 230, 120, 255);
constexpr ImU32 kColSelected = IM_COL32(60, 220, 255, 255);
constexpr ImU32 kColCamera   = IM_COL32(60, 220, 255, 160);
constexpr ImU32 kColTileHover = IM_COL32(200, 220, 255, 200);
constexpr ImU32 kColFaded     = IM_COL32(160, 160, 160, 70);   // filtered out
constexpr ImU32 kColPending  = IM_COL32(60, 60, 64, 255);

constexpr const char* kMosaicRecipeName = "Mosaic capture";
constexpr const char* kMosaicCalName    = "Mosaic camera calibration";

// Export scales offered, as divisors.
constexpr int kScales[] = {1, 2, 4, 8};
constexpr int kScaleCount = static_cast<int>(sizeof(kScales) / sizeof(kScales[0]));

// Texture paging. Quarter previews are under 1 MB each on the GPU; a full 5 MP
// frame is ~15-20 MB, and at a zoom that needs full resolution a screen shows
// a handful, so a few more than that is enough.
constexpr std::size_t kMaxFullTex    = 16;
constexpr std::size_t kMaxQuarterTex = 400;
constexpr std::size_t kUploadsPerFrame = 6;
constexpr int         kMaxFullInFlight = 6;   // decodes asked for and not back yet

// Screen px per canvas px at which a level stops being enough: past 0.3 a
// quarter would be magnified more than ~1.2x, below 0.08 a quarter is
// minified more than 3x and the 1/16 level looks better.
constexpr double kFullFromScale    = 0.3;
constexpr double kQuarterFromScale = 0.08;

std::string CornerLine(const MosaicCorner& c)
{
    char b[96];
    std::snprintf(b, sizeof(b), "X %.3f  Y %.3f  Z %.3f", c.x, c.y, c.z);
    return b;
}

// "M1", "M2", ...: the first of `prefix`<n> nobody has.
template <class T>
std::string NextName(const std::vector<T>& list, const char* prefix)
{
    for (std::size_t n = list.size() + 1;; ++n) {
        const std::string name = prefix + std::to_string(n);
        bool taken = false;
        for (const T& e : list)
            if (e.name == name) { taken = true; break; }
        if (!taken) return name;
    }
}

std::string FolderName(const std::string& dir)
{
    return fs::path(dir).filename().string();
}

std::string MosaicTrim(const char* s)
{
    std::string t = s;
    const auto b = t.find_first_not_of(" \t");
    if (b == std::string::npos) return {};
    const auto e = t.find_last_not_of(" \t");
    return t.substr(b, e - b + 1);
}

// Every tag in use on this mosaic's marks, sorted, once each.
std::vector<std::string> MosaicTags(const MosaicAnnotations& a)
{
    std::set<std::string> s;
    for (const MosaicMarker& m : a.markers)
        if (!m.tag.empty()) s.insert(m.tag);
    for (const MosaicLabel& l : a.labels)
        if (!l.tag.empty()) s.insert(l.tag);
    return {s.begin(), s.end()};
}

// The annotation filter: `find` anywhere in the name, case-blind, and the tag
// exactly when one is chosen. One predicate for the lists, the picture and
// the crop export, so they cannot disagree about what is "shown".
bool MosaicAnnShown(const std::string& name, const std::string& tag, const char* find,
                    const std::string& tagFilter)
{
    if (!tagFilter.empty() && tag != tagFilter) return false;
    if (!find || !find[0]) return true;
    const auto lower = [](std::string v) {
        for (char& c : v) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
        return v;
    };
    return lower(name).find(lower(find)) != std::string::npos;
}

} // namespace

// ---- Config and lifecycle ----------------------------------------------------------

void App::SaveMosaicCfg()
{
    if (!SaveMosaicConfig(mosaicCfg_)) mosaicMsg_ = "could not save the mosaic config";
}

void App::ReleaseMosaicTextures()
{
    for (MosaicTileTex& t : mosaicTex_)
        for (unsigned int& tex : t.tex) {
            host_.DeleteTexture(tex);
            tex = 0;
        }
    mosaicTex_.clear();
}

void App::OpenMosaic(const std::string& dir, bool live)
{
    ReleaseMosaicTextures();
    mosaicWorker_.DropLoads();
    ++mosaicGen_;
    mosaicDir_       = dir;
    mosaicLive_      = live;
    mosaicLiveTotal_ = 0;
    mosaicRec_       = MosaicRecord{};
    mosaicAnn_       = MosaicAnnotations{};
    mosaicSelMarker_ = mosaicSelLabel_ = -1;
    mosaicSelTile_   = -1;
    mosaicDragging_  = false;
    mosaicVs_        = 0.0;   // fit on the next frame
    mosaicExportMsg_.clear();
    mosaicExportPath_.clear();

    std::string why;
    mosaicHaveRec_ = LoadMosaicRecord(dir, &mosaicRec_, &why);
    if (!mosaicHaveRec_) {
        mosaicMsg_ = "cannot open " + dir + ": " + why;
        FD_WARN("mosaic", "%s", mosaicMsg_.c_str());
        return;
    }
    LoadMosaicAnnotations(dir, &mosaicAnn_);
    LoadMosaicPlacement(dir, &mosaicRec_);
    mosaicTex_.assign(mosaicRec_.tiles.size(), MosaicTileTex{});
    // Every quarter up front: they are small, and they are what the whole
    // picture is drawn from until the zoom asks for more.
    for (std::size_t i = 0; i < mosaicRec_.tiles.size(); ++i) {
        const MosaicTileRec& t = mosaicRec_.tiles[i];
        if (t.quarter.empty()) continue;
        mosaicTex_[i].requested[1] = true;
        mosaicWorker_.Load(mosaicGen_, static_cast<int>(i), 1, (fs::path(dir) / t.quarter).string());
    }
    FD_INFO("mosaic", "opened %s (%zu tiles, %s)", dir.c_str(), mosaicRec_.tiles.size(),
            mosaicRec_.status.c_str());
}

void App::CloseMosaic()
{
    ReleaseMosaicTextures();
    mosaicWorker_.DropLoads();
    ++mosaicGen_;
    mosaicDir_.clear();
    mosaicRec_     = MosaicRecord{};
    mosaicAnn_     = MosaicAnnotations{};
    mosaicHaveRec_ = false;
    mosaicLive_    = false;
    mosaicSelMarker_ = mosaicSelLabel_ = -1;
    mosaicSelTile_ = -1;
    mosaicVs_      = 0.0;
}

void App::SaveMosaicAnnotationsNow()
{
    if (mosaicDir_.empty()) return;
    if (!SaveMosaicAnnotations(mosaicDir_, mosaicAnn_))
        mosaicMsg_ = "could not save the annotations in " + mosaicDir_;
}

MosaicPlan App::MosaicPlanFor(const MosaicArea& a)
{
    const PylonCamera::Info info = hardware_.Camera().GetInfo();
    const int w = info.width > 0 ? info.width : cameraW_;
    const int h = info.height > 0 ? info.height : cameraH_;
    double px2mmX = 0.0, px2mmY = 0.0;
    workflow_.CameraPixelToMm(px2mmX, px2mmY);
    double m[4];
    MosaicEffectiveM(mosaicCfg_.cal, px2mmX, px2mmY, m);
    return PlanMosaic(a, m, w, h);
}

void App::StartMosaicCapture(const MosaicArea& a)
{
    if (orch_.Busy()) {
        mosaicMsg_ = "a run is active";
        return;
    }
    if (!jogArmed_) {
        mosaicMsg_          = "arm jog first - the capture drives the gantry";
        mosaicOpenArmPopup_ = true;
        return;
    }
    SaveMosaicCfg();   // the run reads the FILE: what is on screen must be what it reads

    RecipeStep step;
    step.type     = "mosaic_capture";
    step.nickname = "Mosaic '" + a.name + "'";
    step.params   = {{"area", a.uid}};
    Recipe one;
    one.uid  = "mosaic-capture";
    one.name = kMosaicRecipeName;
    one.steps.push_back(std::move(step));
    std::string err;
    if (!orch_.recipe.Start(one, session_.MakeRecipeEnv(), &err)) {
        mosaicMsg_ = "capture not started: " + err;
        FD_WARN("mosaic", "%s", mosaicMsg_.c_str());
        return;
    }
    mosaicMsg_.clear();
}

void App::StartMosaicCalibrate()
{
    if (!CanUse(UserRole::Admin)) {
        mosaicMsg_ = "calibrating the camera needs Admin";
        return;
    }
    if (orch_.Busy()) {
        mosaicMsg_ = "a run is active";
        return;
    }
    if (!jogArmed_) {
        mosaicMsg_          = "arm jog first - the calibration steps the gantry";
        mosaicOpenArmPopup_ = true;
        return;
    }
    RecipeStep step;
    step.type     = "mosaic_calibrate";
    step.nickname = "Calibrate the camera map";
    step.params   = {{"step_mm", mosaicCalStepMm_}};
    Recipe one;
    one.uid  = "mosaic-calibrate";
    one.name = kMosaicCalName;
    one.steps.push_back(std::move(step));
    std::string err;
    if (!orch_.recipe.Start(one, session_.MakeRecipeEnv(), &err)) {
        mosaicMsg_ = "calibration not started: " + err;
        FD_WARN("mosaic", "%s", mosaicMsg_.c_str());
        return;
    }
    mosaicMsg_ = "calibrating: stepping the gantry +-X and +-Y";
}

void App::ProposeMosaicCal(const double m[4], const std::string& from, const std::string& note)
{
    mosaicCalPendingHave_ = true;
    for (int i = 0; i < 4; ++i) mosaicCalPendingM_[i] = m[i];
    mosaicCalPendingFrom_ = from;
    mosaicCalPendingNote_ = note;
    mosaicMsg_ = "a camera map was measured - review it under Camera map, then Apply or Discard";
    FD_INFO("mosaic", "camera map proposed (from %s): %s", from.c_str(), note.c_str());
}

void App::GoToMosaicPoint(double cx, double cy, const std::string& what)
{
    if (!mosaicHaveRec_) return;
    if (!CanUse(kRoleJog)) {
        mosaicMsg_ = "moving the gantry needs Technician";
        return;
    }
    if (!jogArmed_) {
        mosaicMsg_          = "arm jog first - this drives the gantry";
        mosaicOpenArmPopup_ = true;
        return;
    }
    if (orch_.Busy()) {
        mosaicMsg_ = "a run is active";
        return;
    }
    double gx = 0.0, gy = 0.0;
    MosaicCanvasToGantry(mosaicRec_.geom, cx, cy, &gx, &gy);
    const std::string device = mosaicRec_.device.empty() ? workflow_.GantryName() : mosaicRec_.device;

    std::map<std::string, double> now;
    if (!workflow_.DevicePose(device, now)) {
        mosaicMsg_ = "'" + device + "' is not connected";
        return;
    }
    std::map<std::string, double> pose = {{"X", gx}, {"Y", gy}};
    // At the Z the nearest frame was shot at: that is the Z the picture of this
    // point was in focus at.
    if (now.count("Z") && !mosaicRec_.tiles.empty()) {
        double best = 1e300, z = now["Z"];
        for (const MosaicTileRec& t : mosaicRec_.tiles) {
            const double dx = t.x + t.w * 0.5 - cx, dy = t.y + t.h * 0.5 - cy;
            if (dx * dx + dy * dy < best) {
                best = dx * dx + dy * dy;
                z    = t.measZ;
            }
        }
        pose["Z"] = z;
    }
    if (!StartWatchedMove(device, pose, what))
        mosaicMsg_ = "'" + device + "' refused the move - is it connected?";
    else
        mosaicMsg_.clear();
}

void App::GoToMosaicTile(int i)
{
    if (!mosaicHaveRec_ || i < 0 || i >= static_cast<int>(mosaicRec_.tiles.size())) return;
    if (!CanUse(kRoleJog)) {
        mosaicMsg_ = "moving the gantry needs Technician";
        return;
    }
    if (!jogArmed_) {
        mosaicMsg_          = "arm jog first - this drives the gantry";
        mosaicOpenArmPopup_ = true;
        return;
    }
    if (orch_.Busy()) {
        mosaicMsg_ = "a run is active";
        return;
    }
    const MosaicTileRec& t = mosaicRec_.tiles[static_cast<std::size_t>(i)];
    const std::string device = mosaicRec_.device.empty() ? workflow_.GantryName() : mosaicRec_.device;
    std::map<std::string, double> now;
    if (!workflow_.DevicePose(device, now)) {
        mosaicMsg_ = "'" + device + "' is not connected";
        return;
    }
    // The MEASURED pose: where the axes said they were when the frame was
    // taken, so the live picture lines up with the filed one.
    std::map<std::string, double> pose = {{"X", t.measX}, {"Y", t.measY}};
    if (now.count("Z")) pose["Z"] = t.measZ;
    char what[64];
    std::snprintf(what, sizeof(what), "mosaic tile (%d,%d)", t.row + 1, t.col + 1);
    if (!StartWatchedMove(device, pose, what))
        mosaicMsg_ = "'" + device + "' refused the move - is it connected?";
    else
        mosaicMsg_.clear();
}

// ---- App::Update's share ------------------------------------------------------------

void App::UpdateMosaic()
{
    ++mosaicFrame_;

    // The 1/16 level is made here from a quarter, never loaded: it is 1/4 of
    // something already in hand.
    const auto makeTiny = [&](MosaicTileTex& tt, const unsigned char* rgb, int w, int h) {
        std::vector<unsigned char> tiny;
        int tw = 0, th = 0;
        DownsampleRgb(rgb, w, h, 4, &tiny, &tw, &th);
        tt.tex[0] = host_.EnsureTextureRGB(tt.tex[0], tw, th, tiny.data());
        tt.w[0] = tw;
        tt.h[0] = th;
    };

    // ---- The capture's start and end ------------------------------------------
    for (const Workflow::MosaicDoneEvent& ev : workflow_.TakeMosaicDone()) {
        mosaicFoldersStale_ = true;
        if (ev.started) {
            OpenMosaic(ev.dir, true);
            mosaicMsg_ = "capturing into " + ev.dir;
            continue;
        }
        if (ev.dir == mosaicDir_) {
            // The file is the record; what the events built was the preview.
            MosaicRecord rec;
            std::string why;
            if (LoadMosaicRecord(ev.dir, &rec, &why)) {
                const bool same = rec.tiles.size() == mosaicTex_.size();
                mosaicRec_     = std::move(rec);
                mosaicHaveRec_ = true;
                if (!same) OpenMosaic(ev.dir, false);
            }
            mosaicLive_ = false;
        }
        char b[256];
        if (ev.ok) std::snprintf(b, sizeof(b), "captured %d tiles", ev.tiles);
        else       std::snprintf(b, sizeof(b), "capture ended after %d tiles: %s", ev.tiles, ev.why.c_str());
        mosaicMsg_ = b;
    }

    // ---- Tiles as the capture files them -----------------------------------------
    for (Workflow::MosaicTileEvent& ev : workflow_.TakeMosaicTiles()) {
        if (!mosaicLive_ || ev.dir != mosaicDir_) continue;
        MosaicTileRec tr;
        char name[48];
        std::snprintf(name, sizeof(name), "r%02d_c%02d", ev.row + 1, ev.col + 1);
        tr.file    = std::string("tiles/") + name + ".png";
        tr.quarter = std::string("tiles/") + name + "_q.png";
        tr.row     = ev.row;
        tr.col     = ev.col;
        tr.x       = ev.x;
        tr.y       = ev.y;
        tr.w       = ev.w;
        tr.h       = ev.h;
        mosaicRec_.tiles.push_back(tr);
        MosaicTileTex tt;
        if (!ev.thumb.empty() && ev.tw > 0 && ev.th > 0) {
            tt.tex[1] = host_.EnsureTextureRGB(0, ev.tw, ev.th, ev.thumb.data());
            tt.w[1] = ev.tw;
            tt.h[1] = ev.th;
            tt.requested[1] = true;
            tt.used[1] = mosaicFrame_;
            makeTiny(tt, ev.thumb.data(), ev.tw, ev.th);
        }
        mosaicTex_.push_back(tt);
        mosaicLiveTotal_ = ev.total;
    }

    // ---- Decoded tiles from the worker ---------------------------------------------
    for (MosaicWorker::Loaded& l : mosaicWorker_.TakeLoaded(kUploadsPerFrame)) {
        if (l.gen != mosaicGen_ || l.tile < 0 || l.tile >= static_cast<int>(mosaicTex_.size()) ||
            l.level < 1 || l.level > 2 || !l.ok)
            continue;   // a stale mosaic's, or a failed read (not retried; logged by the worker)
        MosaicTileTex& tt = mosaicTex_[static_cast<std::size_t>(l.tile)];
        tt.tex[l.level]  = host_.EnsureTextureRGB(tt.tex[l.level], l.w, l.h, l.rgb.data());
        tt.w[l.level]    = l.w;
        tt.h[l.level]    = l.h;
        tt.used[l.level] = mosaicFrame_;
        if (l.level == 1 && tt.tex[0] == 0) makeTiny(tt, l.rgb.data(), l.w, l.h);
    }

    // ---- Paging: the least recently drawn go first, never one drawn last frame ----
    const auto evict = [&](int lvl, std::size_t cap) {
        std::size_t n = 0;
        for (const MosaicTileTex& t : mosaicTex_)
            if (t.tex[lvl]) ++n;
        while (n > cap) {
            std::size_t victim = mosaicTex_.size();
            unsigned long long oldest = ~0ull;
            for (std::size_t i = 0; i < mosaicTex_.size(); ++i) {
                const MosaicTileTex& t = mosaicTex_[i];
                if (t.tex[lvl] && t.used[lvl] + 2 <= mosaicFrame_ && t.used[lvl] < oldest) {
                    oldest = t.used[lvl];
                    victim = i;
                }
            }
            if (victim == mosaicTex_.size()) break;   // all on screen: over the cap for now
            MosaicTileTex& t = mosaicTex_[victim];
            host_.DeleteTexture(t.tex[lvl]);
            t.tex[lvl]       = 0;
            t.requested[lvl] = false;
            --n;
        }
    };
    evict(2, kMaxFullTex);
    evict(1, kMaxQuarterTex);

    // ---- A measured camera map ------------------------------------------------------
    // Machine state, filed here for the needle calibration's reason: a worker
    // must not write a config file it could be halfway through when Stop lands.
    // A proposal, not the map in use: an Admin reviews it under Camera map.
    for (const Workflow::MosaicCalEvent& ev : workflow_.TakeMosaicCal())
        ProposeMosaicCal(ev.m, "stepping the gantry", ev.note);

    // ---- An export (or a seam refinement) finishing -----------------------------------
    const MosaicWorker::Progress pr = mosaicWorker_.GetProgress();
    if (pr.done != mosaicExportSeen_) {
        mosaicExportSeen_ = pr.done;
        mosaicExportMsg_  = pr.result;
        mosaicExportPath_ = pr.outPath;
        if (pr.kind == MosaicWorker::JobKind::Refine && pr.ok && mosaicHaveRec_) {
            LoadMosaicPlacement(mosaicDir_, &mosaicRec_);
            mosaicUseRefined_ = true;
        }
        if (pr.kind == MosaicWorker::JobKind::FitMap && pr.ok) {
            char note[200];
            std::snprintf(note, sizeof(note), "%d of %d seams, %.2f px rms", pr.fit.used,
                          pr.fit.pairs, pr.fit.rmsPx);
            ProposeMosaicCal(pr.fit.m, "the seams of " + FolderName(mosaicFitDir_), note);
        }
    }
}

// ---- The page -------------------------------------------------------------------------

void App::DrawMosaicPanel()
{
    constexpr float kSplitW   = 6.0f;
    constexpr float kMinSide  = 260.0f;
    constexpr float kMinView  = 240.0f;

    const ImVec2 avail = ImGui::GetContentRegionAvail();
    mosaicLeftW_  = std::max(mosaicLeftW_, kMinSide);
    mosaicRightW_ = std::max(mosaicRightW_, kMinSide);
    const float spare = avail.x - 2.0f * kSplitW - kMinView;
    if (mosaicLeftW_ + mosaicRightW_ > spare) {
        const float k = std::max(0.1f, spare / (mosaicLeftW_ + mosaicRightW_));
        mosaicLeftW_  = std::max(120.0f, mosaicLeftW_ * k);
        mosaicRightW_ = std::max(120.0f, mosaicRightW_ * k);
    }
    const float viewW = std::max(40.0f, avail.x - mosaicLeftW_ - mosaicRightW_ - 2.0f * kSplitW);

    ImGui::BeginChild("mosaicSetup", ImVec2(mosaicLeftW_, avail.y), ImGuiChildFlags_Borders);
    DrawMosaicSetup();
    ImGui::EndChild();

    ImGui::SameLine(0.0f, 0.0f);
    if (Splitter("##mosaicsplitL", true, kSplitW, avail.y)) {
        mosaicLeftW_ += ImGui::GetIO().MouseDelta.x;
        mosaicSplitL_ = true;
    } else if (mosaicSplitL_) {
        mosaicSplitL_ = false;
        SaveUiSettings();
    }

    ImGui::SameLine(0.0f, 0.0f);
    ImGui::BeginChild("mosaicViewCol", ImVec2(viewW, avail.y), ImGuiChildFlags_Borders,
                      ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
    DrawMosaicView();
    ImGui::EndChild();

    ImGui::SameLine(0.0f, 0.0f);
    if (Splitter("##mosaicsplitR", true, kSplitW, avail.y)) {
        mosaicRightW_ -= ImGui::GetIO().MouseDelta.x;
        mosaicSplitR_ = true;
    } else if (mosaicSplitR_) {
        mosaicSplitR_ = false;
        SaveUiSettings();
    }

    ImGui::SameLine(0.0f, 0.0f);
    ImGui::BeginChild("mosaicSide", ImVec2(mosaicRightW_, avail.y), ImGuiChildFlags_Borders);
    DrawMosaicSide();
    ImGui::EndChild();

    // Said out loud: a Go or a Capture with jog unarmed raises this alert. Safe
    // as a modal: both are disabled while orch_.Busy(), so no worker is behind
    // a prompt.
    if (mosaicOpenArmPopup_) { ImGui::OpenPopup("Jog not armed##mosaic"); mosaicOpenArmPopup_ = false; }
    ImGui::SetNextWindowPos(ImGui::GetMainViewport()->GetCenter(), ImGuiCond_Appearing,
                            ImVec2(0.5f, 0.5f));
    if (ImGui::BeginPopupModal("Jog not armed##mosaic", nullptr,
                               ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::TextColored(kWarn, "This drives the gantry.");
        ImGui::Text("Jog is not armed, so nothing moved.");
        ImGui::TextDisabled("Arm jog (the Arm checkbox in the Jog window, or the\n"
                            "button below), then click again.");
        ImGui::Spacing();
        if (ImGui::Button("Arm jog", ImVec2(120.0f, 0.0f))) {
            jogArmed_ = true;
            mosaicMsg_.clear();
            FD_INFO("mosaic", "jog armed from the Mosaic page's alert");
            ImGui::CloseCurrentPopup();
        }
        ImGui::SameLine();
        if (ImGui::Button("Close", ImVec2(120.0f, 0.0f))) ImGui::CloseCurrentPopup();
        ImGui::EndPopup();
    }
}

// ---- Left: folder, areas, capture ---------------------------------------------------

void App::DrawMosaicSetup()
{
    const bool busy = orch_.Busy();
    std::vector<MosaicArea>& areas = mosaicCfg_.areas;
    if (mosaicAreaSel_ >= static_cast<int>(areas.size())) mosaicAreaSel_ = areas.empty() ? -1 : 0;
    if (mosaicAreaSel_ < 0 && !areas.empty()) mosaicAreaSel_ = 0;
    MosaicArea* a = mosaicAreaSel_ >= 0 ? &areas[static_cast<std::size_t>(mosaicAreaSel_)] : nullptr;

    // ---- Save folder --------------------------------------------------------------
    ImGui::SeparatorText("Save folder");
    if (!mosaicDirBuf_[0])
        std::snprintf(mosaicDirBuf_, sizeof(mosaicDirBuf_), "%s", mosaicCfg_.saveDir.c_str());
    ImGui::BeginDisabled(busy);
    ImGui::SetNextItemWidth(-130.0f);
    ImGui::InputText("##mosaicdir", mosaicDirBuf_, sizeof(mosaicDirBuf_));
    ImGui::SameLine();
    if (ImGui::Button("Apply")) {
        std::error_code ec;
        fs::create_directories(mosaicDirBuf_, ec);
        if (!fs::is_directory(mosaicDirBuf_, ec)) {
            mosaicMsg_ = std::string("cannot create '") + mosaicDirBuf_ + "'";
        } else {
            mosaicCfg_.saveDir = mosaicDirBuf_;
            SaveMosaicCfg();
            mosaicFoldersStale_ = true;
            mosaicMsg_ = "mosaics now go to " + mosaicCfg_.saveDir;
            FD_INFO("mosaic", "save folder -> %s", mosaicCfg_.saveDir.c_str());
        }
    }
    ImGui::EndDisabled();
    ImGui::SameLine();
    if (ImGui::Button("Explorer##dir")) OpenFolderInExplorer(mosaicCfg_.saveDir);
    if (mosaicCfg_.saveDir != mosaicDirBuf_)
        ImGui::TextDisabled("in use: %s (Apply to change)", mosaicCfg_.saveDir.c_str());

    // ---- Areas ----------------------------------------------------------------------
    ImGui::SeparatorText("Areas");
    ImGui::BeginDisabled(busy);
    if (ImGui::SmallButton("New")) {
        MosaicArea n;
        n.uid  = GenerateUid();
        n.name = NextName(areas, "area ");
        std::map<std::string, double> pose;
        if (workflow_.DevicePose(workflow_.GantryName(), pose) && pose.count("Z"))
            n.zFixedMm = pose["Z"];   // the focus the operator is looking at now
        areas.push_back(std::move(n));
        mosaicAreaSel_ = static_cast<int>(areas.size()) - 1;
        a = &areas.back();
        SaveMosaicCfg();
        mosaicMsg_.clear();
    }
    ImGui::SameLine();
    ImGui::BeginDisabled(!a);
    if (ImGui::SmallButton("Duplicate") && a) {
        MosaicArea copy = *a;         // by value: push_back reallocates
        copy.uid   = GenerateUid();   // a copy is a new identity
        copy.name += " copy";
        areas.push_back(std::move(copy));
        mosaicAreaSel_ = static_cast<int>(areas.size()) - 1;
        a = &areas.back();
        SaveMosaicCfg();
    }
    ImGui::SameLine();
    if (ImGui::SmallButton("Delete") && a) {
        FD_INFO("mosaic", "area '%s' deleted", a->name.c_str());
        areas.erase(areas.begin() + mosaicAreaSel_);
        mosaicAreaSel_ = areas.empty() ? -1 : 0;
        a = areas.empty() ? nullptr : &areas[0];
        SaveMosaicCfg();
    }
    ImGui::EndDisabled();
    ImGui::EndDisabled();

    ImGui::SetNextItemWidth(-1.0f);
    if (ImGui::BeginListBox("##mosaicareas", ImVec2(0.0f, 76.0f))) {
        for (int i = 0; i < static_cast<int>(areas.size()); ++i) {
            ImGui::PushID(i);
            const MosaicArea& row = areas[static_cast<std::size_t>(i)];
            const bool usable = MosaicAreaValidate(row).empty();
            char lbl[160];
            std::snprintf(lbl, sizeof(lbl), "%s   (%d corners, %s Z)%s", row.name.c_str(),
                          row.nCorners, MosaicZModeName(row.zMode), usable ? "" : "  - not ready");
            if (ImGui::Selectable(lbl, i == mosaicAreaSel_)) {
                mosaicAreaSel_ = i;
                mosaicMsg_.clear();
                if (mosaicDir_.empty()) mosaicVs_ = 0.0;   // re-fit the plan preview
            }
            ImGui::PopID();
        }
        ImGui::EndListBox();
    }

    if (a) {
        ImGui::BeginDisabled(busy);
        char buf[64];
        std::snprintf(buf, sizeof(buf), "%s", a->name.c_str());
        ImGui::SetNextItemWidth(-1.0f);
        if (ImGui::InputText("##areaname", buf, sizeof(buf))) a->name = buf;
        if (ImGui::IsItemDeactivatedAfterEdit()) SaveMosaicCfg();

        // ---- Corners ------------------------------------------------------------------
        ImGui::SeparatorText("Corners");
        int n = a->nCorners;
        bool nChanged = ImGui::RadioButton("3 corners", &n, 3);
        ImGui::SameLine();
        nChanged |= ImGui::RadioButton("4 corners", &n, 4);
        if (nChanged) {
            if (n != a->nCorners) {
                a->nCorners = n;
                SaveMosaicCfg();
                if (mosaicDir_.empty()) mosaicVs_ = 0.0;
            }
        }
        ImGui::TextWrapped(a->nCorners == 3
                               ? "A parallelogram: C1 one corner, C2 along one edge, C3 the far "
                                 "corner (the waffle tray's convention). C4 is derived."
                               : "Any convex outline: teach C1..C4 in order round its edge.");
        ImGui::TextDisabled("Jog the camera onto each corner, in focus, then Teach here.");
        const std::string gantry = workflow_.GantryName();
        std::map<std::string, double> pose;
        const bool live = !gantry.empty() && workflow_.DevicePose(gantry, pose) &&
                          pose.count("X") && pose.count("Y");
        for (int i = 0; i < a->nCorners; ++i) {
            ImGui::PushID(i);
            MosaicCorner& c = a->corners[i];
            ImGui::TextUnformatted(MosaicCornerLabel(a->nCorners, i));
            ImGui::SameLine(110.0f);
            ImGui::BeginDisabled(!live);
            if (ImGui::SmallButton(c.have ? "Re-teach here" : "Teach here")) {
                c.have     = true;
                c.x        = pose["X"];
                c.y        = pose["Y"];
                c.z        = pose.count("Z") ? pose["Z"] : 0.0;
                c.taughtAt = NowStamp();
                SaveMosaicCfg();
                mosaicMsg_ = std::string(MosaicCornerLabel(a->nCorners, i)) + " taught";
                FD_INFO("mosaic", "'%s' %s taught: %s", a->name.c_str(),
                        MosaicCornerLabel(a->nCorners, i), CornerLine(c).c_str());
                if (mosaicDir_.empty()) mosaicVs_ = 0.0;
            }
            ImGui::EndDisabled();
            ImGui::SameLine();
            ImGui::BeginDisabled(!c.have || !CanUse(kRoleJog));
            if (ImGui::SmallButton("Go")) {
                if (!jogArmed_) {
                    mosaicMsg_          = "arm jog first - this drives the gantry";
                    mosaicOpenArmPopup_ = true;
                } else if (const std::map<std::string, double> to = {{"X", c.x}, {"Y", c.y}, {"Z", c.z}};
                           !StartWatchedMove(gantry, to,
                                             std::string(MosaicCornerLabel(a->nCorners, i)) +
                                                 " of '" + a->name + "'")) {
                    mosaicMsg_ = "'" + gantry + "' refused the move - is it connected?";
                }
            }
            ImGui::EndDisabled();
            if (c.have) ImGui::TextDisabled("  %s", CornerLine(c).c_str());
            else        ImGui::TextColored(kWarn, "  not taught");
            ImGui::PopID();
        }
        if (a->nCorners == 3) {
            double q[4][2];
            if (MosaicAreaQuad(*a, q))
                ImGui::TextDisabled("%s  X %.3f  Y %.3f", MosaicCornerLabel(3, 3), q[3][0], q[3][1]);
        }
        if (!live) ImGui::TextColored(kWarn, "the gantry is not connected - nothing to teach from");

        // ---- Focus --------------------------------------------------------------------
        ImGui::SeparatorText("Focus (Z)");
        int zm = a->zMode == MosaicZMode::Plane ? 1 : 0;
        bool zChanged = ImGui::RadioButton("Fixed Z", &zm, 0);
        ImGui::SameLine();
        zChanged |= ImGui::RadioButton("Plane through the corners", &zm, 1);
        if (zChanged) {
            const MosaicZMode want = zm == 1 ? MosaicZMode::Plane : MosaicZMode::Fixed;
            if (want != a->zMode) {
                a->zMode = want;
                SaveMosaicCfg();
            }
        }
        if (a->zMode == MosaicZMode::Fixed) {
            ImGui::SetNextItemWidth(140.0f);
            ImGui::InputDouble("Z mm", &a->zFixedMm, 0.01, 0.1, "%.4f");
            if (ImGui::IsItemDeactivatedAfterEdit()) SaveMosaicCfg();
            ImGui::SameLine();
            ImGui::BeginDisabled(!live || !pose.count("Z"));
            if (ImGui::SmallButton("Use current Z")) {
                a->zFixedMm = pose["Z"];
                SaveMosaicCfg();
            }
            ImGui::EndDisabled();
            ImGui::TextDisabled("every tile is shot at this Z");
        } else {
            double pa = 0.0, pb = 0.0, pc = 0.0;
            std::string why;
            if (MosaicPlaneFit(*a, &pa, &pb, &pc, &why)) {
                ImGui::TextDisabled("z = %.4f %+.6f x %+.6f y", pa, pb, pc);
                ImGui::TextDisabled("tilt %.2f um/mm in X, %.2f um/mm in Y", pb * 1000.0, pc * 1000.0);
                ImGui::TextDisabled("each tile's Z comes from the corners' own Zs - teach each "
                                    "corner in focus");
            } else {
                ImGui::TextColored(kWarn, "%s", why.c_str());
            }
        }

        // ---- Walk ---------------------------------------------------------------------
        ImGui::SeparatorText("Walk");
        {
            float ov = static_cast<float>(a->overlapPct);
            ImGui::SetNextItemWidth(160.0f);
            if (ImGui::SliderFloat("overlap %", &ov, static_cast<float>(kMosaicMinOverlapPct),
                                   static_cast<float>(kMosaicMaxOverlapPct), "%.0f"))
                a->overlapPct = ov;
            if (ImGui::IsItemDeactivatedAfterEdit()) SaveMosaicCfg();
            ImGui::SetNextItemWidth(160.0f);
            if (ImGui::InputInt("settle ms", &a->settleMs)) a->settleMs = std::clamp(a->settleMs, 0, 10000);
            if (ImGui::IsItemDeactivatedAfterEdit()) SaveMosaicCfg();
            ImGui::SetNextItemWidth(160.0f);
            if (ImGui::InputInt("fresh frames", &a->freshFrames))
                a->freshFrames = std::clamp(a->freshFrames, 0, 10);
            if (ImGui::IsItemDeactivatedAfterEdit()) SaveMosaicCfg();
            ImGui::SetNextItemWidth(160.0f);
            if (ImGui::InputDouble("velocity mm/s", &a->velocityMmS, 1.0, 10.0, "%.1f"))
                a->velocityMmS = std::max(0.0, a->velocityMmS);
            if (ImGui::IsItemDeactivatedAfterEdit()) SaveMosaicCfg();
            ImGui::TextDisabled("velocity 0 = keep the gantry's current speed");
        }
        ImGui::EndDisabled();   // busy

        // ---- Plan ---------------------------------------------------------------------
        ImGui::SeparatorText("Plan");
        const MosaicPlan plan = MosaicPlanFor(*a);
        if (!plan.ok) {
            ImGui::TextColored(kWarn, "%s", plan.why.c_str());
        } else {
            double q[4][2];
            MosaicAreaQuad(*a, q);
            double x0 = q[0][0], x1 = q[0][0], y0 = q[0][1], y1 = q[0][1];
            for (int k = 1; k < 4; ++k) {
                x0 = std::min(x0, q[k][0]); x1 = std::max(x1, q[k][0]);
                y0 = std::min(y0, q[k][1]); y1 = std::max(y1, q[k][1]);
            }
            const double fps = std::max(1.0, hardware_.Camera().GetInfo().fps);
            const double stepMm = std::fabs(plan.stepX * plan.geom.m[0]) + std::fabs(plan.stepX * plan.geom.m[2]);
            const double vel = a->velocityMmS > 0.0 ? a->velocityMmS : 20.0;
            // Travel + settle + the frames + filing two PNGs. A guess, said as one.
            const double perTile = stepMm / vel + 0.25 + a->settleMs / 1000.0 +
                                   (a->freshFrames + 1) / fps + 0.5;
            const double mb = plan.imgW * static_cast<double>(plan.imgH) * 3.0 * 0.85 / 1e6;
            ImGui::Text("%zu tiles  (%d x %d grid)", plan.tiles.size(), plan.rows, plan.cols);
            ImGui::Text("area %.2f x %.2f mm", x1 - x0, y1 - y0);
            ImGui::Text("canvas %d x %d px (%.0f MP)", plan.canvasW, plan.canvasH,
                        plan.canvasW * static_cast<double>(plan.canvasH) / 1e6);
            ImGui::TextDisabled("about %.1f min, about %.1f GB on disk",
                                perTile * plan.tiles.size() / 60.0,
                                mb * plan.tiles.size() / 1000.0);
        }

        // ---- Capture ------------------------------------------------------------------
        ImGui::SeparatorText("Capture");
        const RecipeEngine::Status rs = orch_.recipe.GetStatus();
        const bool ours = rs.recipeName == kMosaicRecipeName;
        if (ours && rs.state == RecipeEngine::State::Running) {
            const int done  = mosaicLive_ ? static_cast<int>(mosaicRec_.tiles.size()) : 0;
            const int total = mosaicLive_ && mosaicLiveTotal_ > 0 ? mosaicLiveTotal_ : 0;
            char ov[64];
            std::snprintf(ov, sizeof(ov), "%d / %d tiles", done, total);
            ImGui::ProgressBar(total > 0 ? static_cast<float>(done) / static_cast<float>(total) : 0.0f,
                               ImVec2(-FLT_MIN, 0.0f), ov);
            if (ImGui::Button("Stop##mosaic", ImVec2(-FLT_MIN, 0.0f))) orch_.StopAll();
        } else {
            const bool camOk = hardware_.Camera().IsConnected();
            ImGui::BeginDisabled(!plan.ok || busy || !camOk || !CanUse(kRoleJog));
            if (ImGui::Button("Capture", ImVec2(-FLT_MIN, ImGui::GetFrameHeight() * 1.6f)))
                StartMosaicCapture(*a);
            ImGui::EndDisabled();
            if (!plan.ok)            ImGui::TextDisabled("the area is not ready (see Plan)");
            else if (busy)           ImGui::TextDisabled("another run is active");
            else if (!camOk)         ImGui::TextDisabled("camera not connected");
            else if (!jogArmed_)     ImGui::TextDisabled("needs jog armed - it drives the gantry");
            if (ours && !rs.error.empty() && (rs.state == RecipeEngine::State::Failed ||
                                              rs.state == RecipeEngine::State::Stopped))
                ImGui::TextColored(kBad, "%s", rs.error.c_str());
        }
    } else {
        ImGui::TextWrapped("No area. New makes one; teach its corners with the camera over "
                           "them, and Capture walks the gantry over it frame by frame.");
    }
    if (!mosaicMsg_.empty()) ImGui::TextWrapped("%s", mosaicMsg_.c_str());

    // ---- The camera map ------------------------------------------------------------------
    ImGui::SeparatorText("Camera map");
    {
        const MosaicCameraCal& cal = mosaicCfg_.cal;
        if (cal.calibrated) {
            const double sx  = std::hypot(cal.m[0], cal.m[2]), sy = std::hypot(cal.m[1], cal.m[3]);
            const double rot = std::atan2(cal.m[2], cal.m[0]) * 180.0 / 3.14159265358979323846;
            ImGui::Text("%.6f x %.6f mm/px, rotated %.3f deg", sx, sy, rot);
            ImGui::TextDisabled("calibrated %s", cal.calibratedAt.c_str());
        } else {
            double px2mmX = 0.0, px2mmY = 0.0;
            workflow_.CameraPixelToMm(px2mmX, px2mmY);
            ImGui::Text("%.6f x %.6f mm/px, no rotation", px2mmX, px2mmY);
            ImGui::TextColored(kWarn, "uncalibrated: pixel_to_mm from hardware.json, and a "
                                      "camera turned against the gantry shows at every seam");
        }

        // The proposal: the measured map beside the one in use. Nothing reads
        // it until Apply - a calibration that matched the wrong thing must be
        // caught by a person looking at the numbers, not by the next capture.
        if (mosaicCalPendingHave_) {
            double nowM[4];
            double px2mmX = 0.0, px2mmY = 0.0;
            workflow_.CameraPixelToMm(px2mmX, px2mmY);
            MosaicEffectiveM(cal, px2mmX, px2mmY, nowM);
            double r0, k0, sx0, sy0, r1, k1, sx1, sy1;
            MosaicMapDescribe(nowM, &r0, &k0, &sx0, &sy0);
            MosaicMapDescribe(mosaicCalPendingM_, &r1, &k1, &sx1, &sy1);
            ImGui::Spacing();
            ImGui::TextColored(kWarn, "Proposed - not in use yet");
            ImGui::TextDisabled("from %s", mosaicCalPendingFrom_.c_str());
            if (ImGui::BeginTable("##calprop", 3, ImGuiTableFlags_SizingFixedFit |
                                                      ImGuiTableFlags_RowBg)) {
                ImGui::TableSetupColumn("");
                ImGui::TableSetupColumn("proposed");
                ImGui::TableSetupColumn("in use");
                ImGui::TableHeadersRow();
                const auto row = [](const char* name, const char* fmt, double a, double b) {
                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0);
                    ImGui::TextUnformatted(name);
                    ImGui::TableSetColumnIndex(1);
                    ImGui::Text(fmt, a);
                    ImGui::TableSetColumnIndex(2);
                    ImGui::TextDisabled(fmt, b);
                };
                row("rotation deg", "%+.3f", r1, r0);
                row("skew deg", "%+.3f", k1, k0);
                row("mm/px x", "%.6f", sx1, sx0);
                row("mm/px y", "%.6f", sy1, sy0);
                ImGui::EndTable();
            }
            ImGui::TextDisabled("%s", mosaicCalPendingNote_.c_str());
            // What the difference does at a seam: the step a frame's width of
            // the rotation change puts between two neighbours.
            const PylonCamera::Info info = hardware_.Camera().GetInfo();
            const int fw = info.width > 0 ? info.width : cameraW_;
            if (fw > 0)
                ImGui::TextDisabled("changes a seam by about %.1f px across a %d px frame",
                                    std::fabs(std::sin((r1 - r0) * 3.14159265358979323846 / 180.0)) * fw,
                                    fw);
            if (CanUse(UserRole::Admin)) {
                ImGui::BeginDisabled(busy);
                if (ImGui::Button("Apply##calprop")) {
                    mosaicCfg_.cal.calibrated = true;
                    for (int i = 0; i < 4; ++i) mosaicCfg_.cal.m[i] = mosaicCalPendingM_[i];
                    mosaicCfg_.cal.calibratedAt = NowStamp();
                    mosaicCfg_.cal.note = "from " + mosaicCalPendingFrom_ + ": " + mosaicCalPendingNote_;
                    SaveMosaicCfg();
                    mosaicCalPendingHave_ = false;
                    mosaicMsg_ = "camera map applied - captures from now on use it";
                    FD_INFO("mosaic", "camera map applied: rotation %.3f deg, skew %.3f deg, "
                            "%.6f x %.6f mm/px (%s)", r1, k1, sx1, sy1,
                            mosaicCfg_.cal.note.c_str());
                    if (mosaicDir_.empty()) mosaicVs_ = 0.0;   // the preview's plan changed
                }
                ImGui::EndDisabled();
                ImGui::SameLine();
                if (ImGui::Button("Discard##calprop")) {
                    mosaicCalPendingHave_ = false;
                    mosaicMsg_ = "proposed camera map discarded";
                }
            } else {
                ImGui::TextDisabled("an Admin applies or discards it");
            }
        }
        if (CanUse(UserRole::Admin)) {
            const bool camOk = hardware_.Camera().IsConnected();
            const RecipeEngine::Status rs = orch_.recipe.GetStatus();
            const bool ours = rs.recipeName == kMosaicCalName;
            ImGui::BeginDisabled(busy);
            ImGui::SetNextItemWidth(120.0f);
            if (ImGui::InputDouble("step mm##cal", &mosaicCalStepMm_, 0.05, 0.5, "%.3f"))
                mosaicCalStepMm_ = std::clamp(mosaicCalStepMm_, 0.01, 5.0);
            ImGui::SameLine();
            ImGui::BeginDisabled(!camOk);
            if (ImGui::Button("Calibrate here")) StartMosaicCalibrate();
            ImGui::EndDisabled();
            ImGui::SameLine();
            ImGui::BeginDisabled(!mosaicCfg_.cal.calibrated);
            if (ImGui::Button("Forget")) {
                mosaicCfg_.cal = MosaicCameraCal{};
                SaveMosaicCfg();
                mosaicMsg_ = "camera map back to pixel_to_mm";
                FD_INFO("mosaic", "camera map calibration cleared");
            }
            ImGui::EndDisabled();
            ImGui::EndDisabled();
            ImGui::TextDisabled("put something textured, in focus, under the camera; it steps "
                                "the gantry +-X and +-Y by the step and back");
            if (ours && rs.state == RecipeEngine::State::Running)
                ImGui::TextColored(kWarn, "calibrating...");
            else if (ours && !rs.error.empty() && (rs.state == RecipeEngine::State::Failed ||
                                                   rs.state == RecipeEngine::State::Stopped))
                ImGui::TextColored(kBad, "%s", rs.error.c_str());
        }
    }

    // ---- Mosaics on disk -----------------------------------------------------------------
    ImGui::SeparatorText("Mosaics");
    if (ImGui::SmallButton("Refresh")) mosaicFoldersStale_ = true;
    if (mosaicFoldersStale_) {
        mosaicFolders_      = ListMosaicFolders(mosaicCfg_.saveDir);
        mosaicFoldersStale_ = false;
    }
    ImGui::SameLine();
    ImGui::TextDisabled("%zu in %s", mosaicFolders_.size(), mosaicCfg_.saveDir.c_str());
    // A running capture owns the view: opening another folder mid-walk would
    // leave its tiles with nowhere to land.
    ImGui::BeginDisabled(mosaicLive_);
    const std::string openName = mosaicDir_.empty() ? std::string() : FolderName(mosaicDir_);
    if (ImGui::BeginListBox("##mosaicfolders", ImVec2(-1.0f, std::max(80.0f, ImGui::GetContentRegionAvail().y - 4.0f)))) {
        for (const std::string& name : mosaicFolders_) {
            if (ImGui::Selectable(name.c_str(), name == openName) && name != openName)
                OpenMosaic((fs::path(mosaicCfg_.saveDir) / name).string(), false);
        }
        ImGui::EndListBox();
    }
    ImGui::EndDisabled();
}

// ---- Centre: the picture --------------------------------------------------------------

void App::DrawMosaicView()
{
    // ---- What there is to show -------------------------------------------------------
    // An open mosaic, else the plan of the selected area as a preview.
    MosaicPlan preview;
    const MosaicGeom* geom = nullptr;
    double ex0 = 0.0, ey0 = 0.0, ex1 = 0.0, ey1 = 0.0;
    const double (*quad)[2] = nullptr;
    if (mosaicHaveRec_) {
        geom = &mosaicRec_.geom;
        quad = mosaicRec_.quad;
        ex1  = mosaicRec_.canvasW;
        ey1  = mosaicRec_.canvasH;
        double a0, b0, a1, b1;
        if (MosaicRecordExtent(mosaicRec_, &a0, &b0, &a1, &b1, mosaicUseRefined_)) {
            ex0 = std::min(ex0, a0); ey0 = std::min(ey0, b0);
            ex1 = std::max(ex1, a1); ey1 = std::max(ey1, b1);
        }
    } else if (mosaicAreaSel_ >= 0 && mosaicAreaSel_ < static_cast<int>(mosaicCfg_.areas.size())) {
        preview = MosaicPlanFor(mosaicCfg_.areas[static_cast<std::size_t>(mosaicAreaSel_)]);
        if (preview.ok) {
            geom = &preview.geom;
            quad = preview.quad;
            ex1  = preview.canvasW;
            ey1  = preview.canvasH;
        }
    }

    // ---- Toolbar -------------------------------------------------------------------------
    ImGui::RadioButton("Pan", &mosaicTool_, 0);
    ImGui::SameLine();
    ImGui::RadioButton("Marker", &mosaicTool_, 1);
    ImGui::SameLine();
    ImGui::RadioButton("Crop label", &mosaicTool_, 2);
    ImGui::SameLine();
    if (ImGui::Button("Fit")) mosaicVs_ = 0.0;
    ImGui::SameLine();
    if (ImGui::Button("1:1") && mosaicVs_ > 0.0) mosaicVs_ = 1.0;
    ImGui::SameLine();
    ImGui::Checkbox("Tiles", &mosaicShowGrid_);
    ImGui::SameLine();
    ImGui::Checkbox("Outline", &mosaicShowOutline_);
    if (mosaicHaveRec_) {
        ImGui::SameLine();
        if (mosaicLive_)
            ImGui::TextColored(kWarn, "capturing %zu / %d", mosaicRec_.tiles.size(), mosaicLiveTotal_);
        else
            ImGui::TextDisabled("%s  (%s)", FolderName(mosaicDir_).c_str(), mosaicRec_.status.c_str());
    } else if (geom) {
        ImGui::SameLine();
        ImGui::TextDisabled("plan preview: %zu tiles", preview.tiles.size());
    }

    const ImVec2 origin = ImGui::GetCursorScreenPos();
    const ImVec2 size(std::max(8.0f, ImGui::GetContentRegionAvail().x),
                      std::max(8.0f, ImGui::GetContentRegionAvail().y));
    ImGui::InvisibleButton("##mosaicCanvas", size,
                           ImGuiButtonFlags_MouseButtonLeft | ImGuiButtonFlags_MouseButtonRight |
                               ImGuiButtonFlags_MouseButtonMiddle);
    const bool hovered = ImGui::IsItemHovered();
    const bool active  = ImGui::IsItemActive();
    ImDrawList* dl = ImGui::GetWindowDrawList();
    const ImVec2 end(origin.x + size.x, origin.y + size.y);
    dl->AddRectFilled(origin, end, IM_COL32(18, 18, 20, 255));

    if (!geom || ex1 - ex0 <= 0.0 || ey1 - ey0 <= 0.0) {
        dl->AddText(ImVec2(origin.x + 12.0f, origin.y + 12.0f), IM_COL32(160, 160, 160, 255),
                    "Open a mosaic (left, Mosaics), or select an area whose corners are "
                    "taught to preview its plan.");
        return;
    }

    // ---- The viewport ---------------------------------------------------------------------
    const double fitS = std::min(size.x / (ex1 - ex0), size.y / (ey1 - ey0)) * 0.96;
    if (mosaicVs_ <= 0.0) {
        mosaicVs_  = fitS;
        mosaicVcx_ = (ex0 + ex1) * 0.5;
        mosaicVcy_ = (ey0 + ey1) * 0.5;
    }
    const double minS = fitS * 0.25, maxS = 8.0;
    const float  mx = origin.x + size.x * 0.5f, my = origin.y + size.y * 0.5f;
    const auto toScreen = [&](double cx, double cy) {
        return ImVec2(static_cast<float>(mx + (cx - mosaicVcx_) * mosaicVs_),
                      static_cast<float>(my + (cy - mosaicVcy_) * mosaicVs_));
    };
    const ImGuiIO& io = ImGui::GetIO();
    const double mcx = mosaicVcx_ + (io.MousePos.x - mx) / mosaicVs_;
    const double mcy = mosaicVcy_ + (io.MousePos.y - my) / mosaicVs_;

    // Zoom about the cursor: the canvas point under it stays under it.
    if (hovered && io.MouseWheel != 0.0f) {
        const double s = std::clamp(mosaicVs_ * std::pow(1.25, io.MouseWheel), minS, maxS);
        mosaicVcx_ = mcx - (io.MousePos.x - mx) / s;
        mosaicVcy_ = mcy - (io.MousePos.y - my) / s;
        mosaicVs_  = s;
    }
    // Pan: middle drag anywhere, left drag with the Pan tool or Space held.
    const bool spacePan = ImGui::IsKeyDown(ImGuiKey_Space);
    if (active && (ImGui::IsMouseDragging(ImGuiMouseButton_Middle, 0.0f) ||
                   (ImGui::IsMouseDragging(ImGuiMouseButton_Left, 0.0f) &&
                    (mosaicTool_ == 0 || spacePan) && !mosaicDragging_))) {
        mosaicVcx_ -= io.MouseDelta.x / mosaicVs_;
        mosaicVcy_ -= io.MouseDelta.y / mosaicVs_;
    }

    dl->PushClipRect(origin, end, true);

    int hitTile = -1;   // Tiles mode: the frame under the cursor

    // ---- The tiles ------------------------------------------------------------------------
    if (mosaicHaveRec_ && !mosaicRec_.tiles.empty()) {
        const double s = mosaicVs_;
        const int want = s >= kFullFromScale ? 2 : (s >= kQuarterFromScale ? 1 : 0);
        // Each tile is cropped to the middle of its overlaps where it has a
        // neighbour: the seam falls halfway, as the export's does.
        const int rows = mosaicRec_.rows, cols = mosaicRec_.cols;
        std::vector<char> occ(rows > 0 && cols > 0 ? static_cast<std::size_t>(rows) * cols : 0, 0);
        const auto has = [&](int r, int c) {
            return r >= 0 && c >= 0 && r < rows && c < cols &&
                   occ[static_cast<std::size_t>(r) * cols + c] != 0;
        };
        for (const MosaicTileRec& t : mosaicRec_.tiles)
            if (t.row >= 0 && t.col >= 0 && t.row < rows && t.col < cols)
                occ[static_cast<std::size_t>(t.row) * cols + t.col] = 1;

        int inFlight = 0;
        for (const MosaicTileTex& tt : mosaicTex_)
            if (tt.requested[2] && !tt.tex[2]) ++inFlight;

        const double viewX0 = mosaicVcx_ - (size.x * 0.5) / s, viewX1 = mosaicVcx_ + (size.x * 0.5) / s;
        const double viewY0 = mosaicVcy_ - (size.y * 0.5) / s, viewY1 = mosaicVcy_ + (size.y * 0.5) / s;
        for (std::size_t i = 0; i < mosaicRec_.tiles.size() && i < mosaicTex_.size(); ++i) {
            const MosaicTileRec& t = mosaicRec_.tiles[i];
            MosaicTileTex& tt = mosaicTex_[i];
            const double tx = MosaicTileX(mosaicRec_, t, mosaicUseRefined_);
            const double ty = MosaicTileY(mosaicRec_, t, mosaicUseRefined_);
            const double hx = std::max(0.0, (t.w - mosaicRec_.stepX) * 0.5);
            const double hy = std::max(0.0, (t.h - mosaicRec_.stepY) * 0.5);
            const double l = has(t.row, t.col - 1) ? hx : 0.0, r = has(t.row, t.col + 1) ? hx : 0.0;
            const double u = has(t.row - 1, t.col) ? hy : 0.0, d = has(t.row + 1, t.col) ? hy : 0.0;
            const double x0 = tx + l, x1 = tx + t.w - r, y0 = ty + u, y1 = ty + t.h - d;
            if (x1 < viewX0 || x0 > viewX1 || y1 < viewY0 || y0 > viewY1) continue;

            // Ask for what the zoom wants; draw the best of what is here.
            const auto request = [&](int lvl) {
                const std::string& rel = lvl == 2 ? t.file : t.quarter;
                if (rel.empty() || tt.requested[lvl]) return;
                if (lvl == 2 && inFlight >= kMaxFullInFlight) return;
                if (lvl == 2) ++inFlight;
                tt.requested[lvl] = true;
                mosaicWorker_.Load(mosaicGen_, static_cast<int>(i), lvl,
                                   (fs::path(mosaicDir_) / rel).string());
            };
            if (want >= 1 && !tt.tex[want]) request(want);
            if (want == 0 && !tt.tex[0] && !tt.tex[1]) request(1);
            int lv = -1;
            for (int k = want; k >= 0 && lv < 0; --k)
                if (tt.tex[k]) lv = k;
            for (int k = want + 1; k < 3 && lv < 0; ++k)
                if (tt.tex[k]) lv = k;
            if (want >= 1 && tt.tex[want]) tt.used[want] = mosaicFrame_;
            if (lv < 0) {
                dl->AddRectFilled(toScreen(x0, y0), toScreen(x1, y1), kColPending);
                continue;
            }
            tt.used[lv] = mosaicFrame_;
            const ImVec2 uv0(static_cast<float>(l / t.w), static_cast<float>(u / t.h));
            const ImVec2 uv1(static_cast<float>(1.0 - r / t.w), static_cast<float>(1.0 - d / t.h));
            dl->AddImage((ImTextureID)(intptr_t)tt.tex[lv], toScreen(x0, y0), toScreen(x1, y1),
                         uv0, uv1);
        }
        if (mosaicShowGrid_) {
            // Tiles mode: every frame outlined, the one under the cursor (the
            // nearest centre, which is whose pixel it is in the picture)
            // highlighted, the selected one outlined and named.
            double best = 1e300;
            for (int i = 0; i < static_cast<int>(mosaicRec_.tiles.size()); ++i) {
                const MosaicTileRec& t = mosaicRec_.tiles[static_cast<std::size_t>(i)];
                const double tx = MosaicTileX(mosaicRec_, t, mosaicUseRefined_);
                const double ty = MosaicTileY(mosaicRec_, t, mosaicUseRefined_);
                dl->AddRect(toScreen(tx, ty), toScreen(tx + t.w, ty + t.h), kColGrid);
                if (!hovered || mcx < tx || mcy < ty || mcx >= tx + t.w || mcy >= ty + t.h) continue;
                const double dx = mcx - (tx + t.w * 0.5), dy = mcy - (ty + t.h * 0.5);
                if (dx * dx + dy * dy < best) {
                    best    = dx * dx + dy * dy;
                    hitTile = i;
                }
            }
            const auto frame = [&](int i, ImU32 col, float thick) {
                const MosaicTileRec& t = mosaicRec_.tiles[static_cast<std::size_t>(i)];
                const double tx = MosaicTileX(mosaicRec_, t, mosaicUseRefined_);
                const double ty = MosaicTileY(mosaicRec_, t, mosaicUseRefined_);
                const ImVec2 p0 = toScreen(tx, ty);
                dl->AddRect(p0, toScreen(tx + t.w, ty + t.h), col, 0.0f, 0, thick);
                char b[32];
                std::snprintf(b, sizeof(b), "(%d,%d)", t.row + 1, t.col + 1);
                dl->AddText(ImVec2(p0.x + 4.0f, p0.y + 3.0f), col, b);
            };
            if (hitTile >= 0 && hitTile != mosaicSelTile_) frame(hitTile, kColTileHover, 1.5f);
            if (mosaicSelTile_ >= 0 && mosaicSelTile_ < static_cast<int>(mosaicRec_.tiles.size()))
                frame(mosaicSelTile_, kColOutline, 2.5f);
        }
    } else if (!mosaicHaveRec_) {
        // The plan preview: where each frame would be, in the order it is shot.
        for (std::size_t i = 0; i < preview.tiles.size(); ++i) {
            const MosaicPlanTile& t = preview.tiles[i];
            const ImVec2 p0 = toScreen(t.cx - preview.imgW * 0.5, t.cy - preview.imgH * 0.5);
            const ImVec2 p1 = toScreen(t.cx + preview.imgW * 0.5, t.cy + preview.imgH * 0.5);
            dl->AddRect(p0, p1, kColGrid);
            if (p1.x - p0.x > 28.0f) {
                char b[16];
                std::snprintf(b, sizeof(b), "%zu", i + 1);
                dl->AddText(ImVec2(p0.x + 3.0f, p0.y + 2.0f), IM_COL32(170, 190, 230, 200), b);
            }
        }
    }

    // ---- The outline, and where the camera is now -------------------------------------------
    if (mosaicShowOutline_ && quad) {
        ImVec2 pts[4];
        for (int k = 0; k < 4; ++k) pts[k] = toScreen(quad[k][0], quad[k][1]);
        dl->AddPolyline(pts, 4, kColOutline, ImDrawFlags_Closed, 2.0f);
    }
    {
        double gx = 0.0, gy = 0.0, ccx = 0.0, ccy = 0.0;
        const int iw = mosaicHaveRec_ ? mosaicRec_.imgW : preview.imgW;
        const int ih = mosaicHaveRec_ ? mosaicRec_.imgH : preview.imgH;
        if (workflow_.GantryXY(gx, gy) && iw > 0 && ih > 0) {
            MosaicGantryToCanvas(*geom, gx, gy, &ccx, &ccy);
            dl->AddRect(toScreen(ccx - iw * 0.5, ccy - ih * 0.5), toScreen(ccx + iw * 0.5, ccy + ih * 0.5),
                        kColCamera, 0.0f, 0, 1.5f);
            const ImVec2 c = toScreen(ccx, ccy);
            dl->AddLine(ImVec2(c.x - 6.0f, c.y), ImVec2(c.x + 6.0f, c.y), kColCamera);
            dl->AddLine(ImVec2(c.x, c.y - 6.0f), ImVec2(c.x, c.y + 6.0f), kColCamera);
        }
    }

    // ---- Annotations ----------------------------------------------------------------------
    int hitMarker = -1, hitLabel = -1;
    if (mosaicHaveRec_) {
        for (int i = 0; i < static_cast<int>(mosaicAnn_.labels.size()); ++i) {
            const MosaicLabel& lb = mosaicAnn_.labels[static_cast<std::size_t>(i)];
            const ImVec2 p0 = toScreen(lb.x0, lb.y0), p1 = toScreen(lb.x1, lb.y1);
            // Filtered out: faded and not clickable - still there, not in the way.
            const bool on = MosaicAnnShown(lb.name, lb.tag, mosaicAnnFind_, mosaicAnnTag_);
            const ImU32 col = i == mosaicSelLabel_ ? kColSelected : on ? kColLabel : kColFaded;
            dl->AddRect(p0, p1, col, 0.0f, 0, i == mosaicSelLabel_ ? 2.5f : 1.5f);
            dl->AddText(ImVec2(p0.x + 3.0f, p0.y - ImGui::GetTextLineHeight() - 1.0f), col,
                        lb.name.c_str());
            if (on && hovered && io.MousePos.x >= p0.x && io.MousePos.x <= p1.x &&
                io.MousePos.y >= p0.y && io.MousePos.y <= p1.y)
                hitLabel = i;
        }
        for (int i = 0; i < static_cast<int>(mosaicAnn_.markers.size()); ++i) {
            const MosaicMarker& mk = mosaicAnn_.markers[static_cast<std::size_t>(i)];
            const ImVec2 c = toScreen(mk.cx, mk.cy);
            const bool on = MosaicAnnShown(mk.name, mk.tag, mosaicAnnFind_, mosaicAnnTag_);
            const ImU32 col = i == mosaicSelMarker_ ? kColSelected : on ? kColMarker : kColFaded;
            const float k = i == mosaicSelMarker_ ? 9.0f : 7.0f;
            dl->AddLine(ImVec2(c.x - k, c.y), ImVec2(c.x + k, c.y), col, 2.0f);
            dl->AddLine(ImVec2(c.x, c.y - k), ImVec2(c.x, c.y + k), col, 2.0f);
            dl->AddText(ImVec2(c.x + k + 2.0f, c.y - k - 2.0f), col, mk.name.c_str());
            const float dx = io.MousePos.x - c.x, dy = io.MousePos.y - c.y;
            if (on && hovered && dx * dx + dy * dy <= 100.0f) hitMarker = i;
        }
        if (hitMarker >= 0) hitLabel = -1;   // the point is the smaller target

        // Marker tool: a click drops one where the cursor is.
        if (mosaicTool_ == 1 && !spacePan && ImGui::IsItemClicked(ImGuiMouseButton_Left)) {
            if (hitMarker >= 0) {
                mosaicSelMarker_ = hitMarker;
                mosaicSelLabel_  = -1;
            } else {
                MosaicMarker mk;
                mk.uid  = GenerateUid();
                mk.name = NextName(mosaicAnn_.markers, "M");
                mk.cx   = mcx;
                mk.cy   = mcy;
                mk.tag  = mosaicAnnTag_;   // stays in view under the current filter
                mk.at   = NowStamp();
                mosaicAnn_.markers.push_back(mk);
                mosaicSelMarker_ = static_cast<int>(mosaicAnn_.markers.size()) - 1;
                mosaicSelLabel_  = -1;
                SaveMosaicAnnotationsNow();
            }
        }
        // Crop label tool: drag out a rectangle.
        if (mosaicTool_ == 2 && !spacePan && ImGui::IsItemClicked(ImGuiMouseButton_Left)) {
            mosaicDragging_ = true;
            mosaicDragX0_   = mcx;
            mosaicDragY0_   = mcy;
        }
        if (mosaicDragging_) {
            dl->AddRect(toScreen(mosaicDragX0_, mosaicDragY0_), toScreen(mcx, mcy), kColSelected,
                        0.0f, 0, 1.5f);
            if (!ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
                mosaicDragging_ = false;
                const double wpx = std::fabs(mcx - mosaicDragX0_) * mosaicVs_;
                const double hpx = std::fabs(mcy - mosaicDragY0_) * mosaicVs_;
                if (wpx >= 4.0 && hpx >= 4.0) {   // a click is not a label
                    MosaicLabel lb;
                    lb.uid  = GenerateUid();
                    lb.name = NextName(mosaicAnn_.labels, "L");
                    lb.x0   = std::min(mosaicDragX0_, mcx);
                    lb.y0   = std::min(mosaicDragY0_, mcy);
                    lb.x1   = std::max(mosaicDragX0_, mcx);
                    lb.y1   = std::max(mosaicDragY0_, mcy);
                    lb.tag  = mosaicAnnTag_;
                    lb.at   = NowStamp();
                    mosaicAnn_.labels.push_back(lb);
                    mosaicSelLabel_  = static_cast<int>(mosaicAnn_.labels.size()) - 1;
                    mosaicSelMarker_ = -1;
                    SaveMosaicAnnotationsNow();
                } else if (hitLabel >= 0) {
                    mosaicSelLabel_  = hitLabel;
                    mosaicSelMarker_ = -1;
                }
            }
        }
        // Pan tool: a click selects what is under it.
        if (mosaicTool_ == 0 && ImGui::IsItemClicked(ImGuiMouseButton_Left) && !spacePan) {
            mosaicSelMarker_ = hitMarker;
            mosaicSelLabel_  = hitLabel;
        }
        // Tiles mode, Pan tool: a click that did not become a drag selects the
        // tile under it (on RELEASE, so panning never changes the selection).
        // (GetMouseDragDelta still reports the drag on the release frame.)
        if (mosaicShowGrid_ && mosaicTool_ == 0 && hovered && !spacePan &&
            ImGui::IsMouseReleased(ImGuiMouseButton_Left) && hitMarker < 0 && hitLabel < 0) {
            const ImVec2 dd = ImGui::GetMouseDragDelta(ImGuiMouseButton_Left, 0.0f);
            if (dd.x * dd.x + dd.y * dd.y < 16.0f) mosaicSelTile_ = hitTile;
        }
        // Right-click: the item's menu - a marker or label first, else the tile.
        if (ImGui::IsItemClicked(ImGuiMouseButton_Right) && (hitMarker >= 0 || hitLabel >= 0)) {
            mosaicSelMarker_ = hitMarker;
            mosaicSelLabel_  = hitLabel;
            ImGui::OpenPopup("##mosaicItemMenu");
        } else if (ImGui::IsItemClicked(ImGuiMouseButton_Right) && hitTile >= 0) {
            mosaicSelTile_ = hitTile;
            ImGui::OpenPopup("##mosaicTileMenu");
        }
        // Arrow keys walk the selected tile to its neighbour.
        if (mosaicShowGrid_ && hovered && mosaicSelTile_ >= 0 &&
            mosaicSelTile_ < static_cast<int>(mosaicRec_.tiles.size())) {
            int dr = 0, dc = 0;
            if (ImGui::IsKeyPressed(ImGuiKey_LeftArrow))  dc = -1;
            if (ImGui::IsKeyPressed(ImGuiKey_RightArrow)) dc = 1;
            if (ImGui::IsKeyPressed(ImGuiKey_UpArrow))    dr = -1;
            if (ImGui::IsKeyPressed(ImGuiKey_DownArrow))  dr = 1;
            if (dr || dc) {
                const MosaicTileRec& cur = mosaicRec_.tiles[static_cast<std::size_t>(mosaicSelTile_)];
                for (int i = 0; i < static_cast<int>(mosaicRec_.tiles.size()); ++i) {
                    const MosaicTileRec& t = mosaicRec_.tiles[static_cast<std::size_t>(i)];
                    if (t.row == cur.row + dr && t.col == cur.col + dc) {
                        mosaicSelTile_ = i;
                        break;
                    }
                }
            }
        }
        if (hovered && ImGui::IsKeyPressed(ImGuiKey_Delete, false)) {
            if (mosaicSelMarker_ >= 0 && mosaicSelMarker_ < static_cast<int>(mosaicAnn_.markers.size())) {
                mosaicAnn_.markers.erase(mosaicAnn_.markers.begin() + mosaicSelMarker_);
                mosaicSelMarker_ = -1;
                SaveMosaicAnnotationsNow();
            } else if (mosaicSelLabel_ >= 0 &&
                       mosaicSelLabel_ < static_cast<int>(mosaicAnn_.labels.size())) {
                mosaicAnn_.labels.erase(mosaicAnn_.labels.begin() + mosaicSelLabel_);
                mosaicSelLabel_ = -1;
                SaveMosaicAnnotationsNow();
            }
        }
    }

    // ---- Readouts ------------------------------------------------------------------------
    {
        char b[160];
        std::snprintf(b, sizeof(b), "zoom %.1f %%", mosaicVs_ * 100.0);
        const ImVec2 ts = ImGui::CalcTextSize(b);
        dl->AddText(ImVec2(end.x - ts.x - 8.0f, origin.y + 6.0f), IM_COL32(200, 200, 200, 220), b);
        if (hovered) {
            double gx = 0.0, gy = 0.0;
            MosaicCanvasToGantry(*geom, mcx, mcy, &gx, &gy);
            std::snprintf(b, sizeof(b), "px %.0f, %.0f   gantry X %.4f  Y %.4f mm", mcx, mcy, gx, gy);
            dl->AddText(ImVec2(origin.x + 8.0f, end.y - ImGui::GetTextLineHeight() - 6.0f),
                        IM_COL32(220, 220, 220, 230), b);
        }
    }
    dl->PopClipRect();

    // ---- The item menu -----------------------------------------------------------------
    if (ImGui::BeginPopup("##mosaicItemMenu")) {
        if (mosaicSelMarker_ >= 0 && mosaicSelMarker_ < static_cast<int>(mosaicAnn_.markers.size())) {
            const MosaicMarker mk = mosaicAnn_.markers[static_cast<std::size_t>(mosaicSelMarker_)];
            ImGui::TextDisabled("marker %s", mk.name.c_str());
            if (CanUse(kRoleJog) && ImGui::MenuItem("Drive the camera here"))
                GoToMosaicPoint(mk.cx, mk.cy, "marker '" + mk.name + "'");
            if (ImGui::MenuItem("Delete")) {
                mosaicAnn_.markers.erase(mosaicAnn_.markers.begin() + mosaicSelMarker_);
                mosaicSelMarker_ = -1;
                SaveMosaicAnnotationsNow();
            }
        } else if (mosaicSelLabel_ >= 0 && mosaicSelLabel_ < static_cast<int>(mosaicAnn_.labels.size())) {
            const MosaicLabel lb = mosaicAnn_.labels[static_cast<std::size_t>(mosaicSelLabel_)];
            ImGui::TextDisabled("crop label %s", lb.name.c_str());
            if (CanUse(kRoleJog) && ImGui::MenuItem("Drive the camera to its centre"))
                GoToMosaicPoint((lb.x0 + lb.x1) * 0.5, (lb.y0 + lb.y1) * 0.5,
                                "crop label '" + lb.name + "'");
            if (ImGui::MenuItem("Delete")) {
                mosaicAnn_.labels.erase(mosaicAnn_.labels.begin() + mosaicSelLabel_);
                mosaicSelLabel_ = -1;
                SaveMosaicAnnotationsNow();
            }
        }
        ImGui::EndPopup();
    }
    if (ImGui::BeginPopup("##mosaicTileMenu")) {
        if (mosaicSelTile_ >= 0 && mosaicSelTile_ < static_cast<int>(mosaicRec_.tiles.size())) {
            const MosaicTileRec& t = mosaicRec_.tiles[static_cast<std::size_t>(mosaicSelTile_)];
            ImGui::TextDisabled("tile (%d,%d)  X %.3f  Y %.3f  Z %.3f", t.row + 1, t.col + 1,
                                t.measX, t.measY, t.measZ);
            if (CanUse(kRoleJog)) {
                if (ImGui::MenuItem("Drive the camera to this tile", nullptr, false, !orch_.Busy()))
                    GoToMosaicTile(mosaicSelTile_);
            }
        }
        ImGui::EndPopup();
    }
}

// ---- Right: annotations and exports ----------------------------------------------------

void App::DrawMosaicSide()
{
    if (!mosaicHaveRec_) {
        ImGui::TextWrapped("No mosaic open. Capture one (left), or pick one from the Mosaics "
                           "list.\n\nIn the picture: the wheel zooms about the cursor, "
                           "middle-drag pans (or the Pan tool, or Space + drag). The Marker "
                           "tool drops a named point; the Crop label tool drags out a named "
                           "rectangle that Export crops writes as its own PNG.");
        return;
    }
    const MosaicRecord& rec = mosaicRec_;

    // ---- The mosaic -----------------------------------------------------------------------
    ImGui::SeparatorText("Mosaic");
    ImGui::TextWrapped("%s", FolderName(mosaicDir_).c_str());
    const ImVec4 stCol = rec.status == "complete" ? kGood : kWarn;
    ImGui::TextColored(stCol, "%s", mosaicLive_ ? "capturing" : rec.status.c_str());
    if (!rec.why.empty()) ImGui::TextColored(kWarn, "%s", rec.why.c_str());
    ImGui::Text("%zu of %d tiles, %d x %d grid", rec.tiles.size(), rec.planned, rec.rows, rec.cols);
    {
        double x0 = 0, y0 = 0, x1 = 0, y1 = 0;
        if (MosaicRecordExtent(rec, &x0, &y0, &x1, &y1)) {
            const double wmm = std::hypot(rec.geom.m[0], rec.geom.m[2]) * (x1 - x0);
            const double hmm = std::hypot(rec.geom.m[1], rec.geom.m[3]) * (y1 - y0);
            ImGui::Text("%.0f x %.0f px, %.2f x %.2f mm", x1 - x0, y1 - y0, wmm, hmm);
        }
    }
    ImGui::TextDisabled("area '%s', %s Z, %.0f %% overlap", rec.area.name.c_str(),
                        MosaicZModeName(rec.area.zMode), rec.area.overlapPct);
    ImGui::TextDisabled("%s  -  camera map %s", rec.created.c_str(),
                        rec.calibrated ? "calibrated" : "from pixel_to_mm");
    if (ImGui::SmallButton("Explorer##mosaic")) OpenFolderInExplorer(mosaicDir_);
    ImGui::SameLine();
    ImGui::BeginDisabled(mosaicLive_);
    if (ImGui::SmallButton("Close##mosaic")) CloseMosaic();
    ImGui::EndDisabled();
    if (!mosaicHaveRec_) return;   // closed just now

    // Seam refinement: measured on the worker, filed as placement.json.
    {
        const bool running = mosaicWorker_.GetProgress().active;
        ImGui::BeginDisabled(mosaicLive_ || running || rec.tiles.size() < 2);
        if (ImGui::SmallButton(rec.refined ? "Refine seams again" : "Refine seams")) {
            MosaicWorker::Export job;
            job.kind = MosaicWorker::JobKind::Refine;
            job.dir  = mosaicDir_;
            job.rec  = rec;
            std::string why;
            if (!mosaicWorker_.StartExport(std::move(job), &why)) mosaicExportMsg_ = why;
        }
        ImGui::EndDisabled();
        if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
            ImGui::SetTooltip("Match every neighbour pair in its overlap and move the tiles\n"
                              "to agree with all of them - removes the steps at the seams\n"
                              "that stage error and the camera map leave. The stage positions\n"
                              "stay in mosaic.json; this writes placement.json beside them.");
        if (rec.refined) {
            ImGui::SameLine();
            ImGui::Checkbox("use refined", &mosaicUseRefined_);
            ImGui::TextDisabled("%s", rec.refinedNote.c_str());
        }
        if (CanUse(UserRole::Admin)) {
            ImGui::BeginDisabled(mosaicLive_ || running || rec.tiles.size() < 4);
            if (ImGui::SmallButton("Fit camera map to this mosaic")) {
                MosaicWorker::Export job;
                job.kind = MosaicWorker::JobKind::FitMap;
                job.dir  = mosaicDir_;
                job.rec  = rec;
                std::string why;
                if (mosaicWorker_.StartExport(std::move(job), &why)) mosaicFitDir_ = mosaicDir_;
                else mosaicExportMsg_ = why;
            }
            ImGui::EndDisabled();
            if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
                ImGui::SetTooltip("Measure every seam of this capture and fit the camera's\n"
                                  "rotation, skew and scale to all of them - no extra motion.\n"
                                  "The result is a PROPOSAL under Camera map (left): review\n"
                                  "it, then Apply. Needs rows AND columns (at least 2 x 2).");
        }
    }

    const auto centreOn = [&](double cx, double cy) {
        mosaicVcx_ = cx;
        mosaicVcy_ = cy;
    };

    // ---- Tile (Tiles mode) -------------------------------------------------------------------
    if (mosaicShowGrid_) {
        ImGui::SeparatorText("Tile");
        if (mosaicSelTile_ >= 0 && mosaicSelTile_ < static_cast<int>(rec.tiles.size())) {
            const MosaicTileRec& t = rec.tiles[static_cast<std::size_t>(mosaicSelTile_)];
            ImGui::Text("(%d,%d)   %d of %zu in the walk", t.row + 1, t.col + 1, mosaicSelTile_ + 1,
                        rec.tiles.size());
            ImGui::TextDisabled("shot at  X %.4f  Y %.4f  Z %.4f", t.measX, t.measY, t.measZ);
            ImGui::TextDisabled("sent to  X %.4f  Y %.4f  Z %.4f", t.cmdX, t.cmdY, t.cmdZ);
            if (CanUse(kRoleJog)) {
                ImGui::BeginDisabled(orch_.Busy());
                if (ImGui::SmallButton("Go to tile")) GoToMosaicTile(mosaicSelTile_);
                ImGui::EndDisabled();
                ImGui::SameLine();
            }
            if (ImGui::SmallButton("Centre##tile")) {
                mosaicVcx_ = MosaicTileX(rec, t, mosaicUseRefined_) + t.w * 0.5;
                mosaicVcy_ = MosaicTileY(rec, t, mosaicUseRefined_) + t.h * 0.5;
            }
            ImGui::SameLine();
            if (ImGui::SmallButton("Clear##tile")) mosaicSelTile_ = -1;
            ImGui::TextDisabled("arrow keys (over the picture) step to a neighbour");
        } else {
            ImGui::TextDisabled("Pan tool: click a tile to select it; right-click to drive there");
        }
    }

    // ---- Annotations: filter, two lists, one editor ------------------------------------------
    // The lists are lists: a click selects and centres the view, nothing else.
    // Whatever is selected - in a list or in the picture - is edited in ONE
    // place below them, so a row is never a form. The name search and the tag
    // filter narrow both lists, fade what they hide in the picture, and choose
    // what Export crops writes; a new mark takes the tag being filtered on, so
    // it stays in view.
    const std::vector<std::string> tags = MosaicTags(mosaicAnn_);
    if (!mosaicAnnTag_.empty() && std::find(tags.begin(), tags.end(), mosaicAnnTag_) == tags.end())
        mosaicAnnTag_.clear();   // the last mark with that tag is gone
    ImGui::SeparatorText("Annotations");
    ImGui::SetNextItemWidth(std::max(80.0f, ImGui::GetContentRegionAvail().x * 0.55f));
    ImGui::InputTextWithHint("##annfind", "search name", mosaicAnnFind_, sizeof(mosaicAnnFind_));
    ImGui::SameLine();
    ImGui::SetNextItemWidth(-FLT_MIN);
    if (ImGui::BeginCombo("##anntag", mosaicAnnTag_.empty() ? "all tags" : mosaicAnnTag_.c_str())) {
        if (ImGui::Selectable("all tags", mosaicAnnTag_.empty())) mosaicAnnTag_.clear();
        for (const std::string& t : tags)
            if (ImGui::Selectable(t.c_str(), t == mosaicAnnTag_)) mosaicAnnTag_ = t;
        ImGui::EndCombo();
    }
    const bool filtering = mosaicAnnFind_[0] || !mosaicAnnTag_.empty();
    if (!mosaicAnnTag_.empty())
        ImGui::TextDisabled("new marks are tagged '%s'", mosaicAnnTag_.c_str());

    const auto shown = [&](const std::string& name, const std::string& tag) {
        return MosaicAnnShown(name, tag, mosaicAnnFind_, mosaicAnnTag_);
    };
    const double mmx = std::hypot(rec.geom.m[0], rec.geom.m[2]);
    const double mmy = std::hypot(rec.geom.m[1], rec.geom.m[3]);
    const auto listHeight = [&](std::size_t rows) {
        return ImGui::GetTextLineHeightWithSpacing() * static_cast<float>(std::clamp<std::size_t>(rows, 2, 8)) +
               ImGui::GetStyle().FramePadding.y * 2.0f;
    };
    char head[64];

    // ---- Markers ----------------------------------------------------------------------------
    std::vector<int> shownM;
    for (int i = 0; i < static_cast<int>(mosaicAnn_.markers.size()); ++i)
        if (shown(mosaicAnn_.markers[static_cast<std::size_t>(i)].name,
                  mosaicAnn_.markers[static_cast<std::size_t>(i)].tag))
            shownM.push_back(i);
    if (filtering) std::snprintf(head, sizeof(head), "Markers (%zu of %zu)", shownM.size(), mosaicAnn_.markers.size());
    else           std::snprintf(head, sizeof(head), "Markers (%zu)", mosaicAnn_.markers.size());
    ImGui::SeparatorText(head);
    if (mosaicAnn_.markers.empty()) {
        ImGui::TextDisabled("Marker tool: click the picture");
    } else if (shownM.empty()) {
        ImGui::TextDisabled("none match the filter");
    } else if (ImGui::BeginListBox("##mklist", ImVec2(-FLT_MIN, listHeight(shownM.size())))) {
        for (int i : shownM) {
            ImGui::PushID(i);
            const MosaicMarker& mk = mosaicAnn_.markers[static_cast<std::size_t>(i)];
            double gx = 0.0, gy = 0.0;
            MosaicCanvasToGantry(rec.geom, mk.cx, mk.cy, &gx, &gy);
            char lbl[192];
            std::snprintf(lbl, sizeof(lbl), "%s%s%s%s   X %.3f  Y %.3f", mk.name.c_str(),
                          mk.tag.empty() ? "" : "  [", mk.tag.c_str(), mk.tag.empty() ? "" : "]",
                          gx, gy);
            if (ImGui::Selectable(lbl, i == mosaicSelMarker_)) {
                mosaicSelMarker_ = i;
                mosaicSelLabel_  = -1;
                centreOn(mk.cx, mk.cy);
            }
            ImGui::PopID();
        }
        ImGui::EndListBox();
    }

    // ---- Crop labels ---------------------------------------------------------------------------
    std::vector<int> shownL;
    for (int i = 0; i < static_cast<int>(mosaicAnn_.labels.size()); ++i)
        if (shown(mosaicAnn_.labels[static_cast<std::size_t>(i)].name,
                  mosaicAnn_.labels[static_cast<std::size_t>(i)].tag))
            shownL.push_back(i);
    if (filtering) std::snprintf(head, sizeof(head), "Crop labels (%zu of %zu)", shownL.size(), mosaicAnn_.labels.size());
    else           std::snprintf(head, sizeof(head), "Crop labels (%zu)", mosaicAnn_.labels.size());
    ImGui::SeparatorText(head);
    if (mosaicAnn_.labels.empty()) {
        ImGui::TextDisabled("Crop label tool: drag a rectangle");
    } else if (shownL.empty()) {
        ImGui::TextDisabled("none match the filter");
    } else if (ImGui::BeginListBox("##lblist", ImVec2(-FLT_MIN, listHeight(shownL.size())))) {
        for (int i : shownL) {
            ImGui::PushID(1000 + i);
            const MosaicLabel& lb = mosaicAnn_.labels[static_cast<std::size_t>(i)];
            char lbl[192];
            std::snprintf(lbl, sizeof(lbl), "%s%s%s%s   %.2f x %.2f mm", lb.name.c_str(),
                          lb.tag.empty() ? "" : "  [", lb.tag.c_str(), lb.tag.empty() ? "" : "]",
                          (lb.x1 - lb.x0) * mmx, (lb.y1 - lb.y0) * mmy);
            if (ImGui::Selectable(lbl, i == mosaicSelLabel_)) {
                mosaicSelLabel_  = i;
                mosaicSelMarker_ = -1;
                centreOn((lb.x0 + lb.x1) * 0.5, (lb.y0 + lb.y1) * 0.5);
            }
            ImGui::PopID();
        }
        ImGui::EndListBox();
    }

    // ---- The selected one --------------------------------------------------------------------------
    // Name and tag, then what it is and what can be done with it. Saved when an
    // edit is finished (focus leaves the field), like every other name here.
    ImGui::SeparatorText("Selected");
    const auto nameAndTag = [&](std::string& name, std::string& tag) {
        const float labelW = ImGui::CalcTextSize("name ").x + ImGui::GetStyle().ItemSpacing.x;
        char buf[64];
        std::snprintf(buf, sizeof(buf), "%s", name.c_str());
        ImGui::AlignTextToFramePadding();
        ImGui::TextUnformatted("name");
        ImGui::SameLine(labelW);
        ImGui::SetNextItemWidth(-FLT_MIN);
        if (ImGui::InputText("##selname", buf, sizeof(buf)) && buf[0]) name = buf;
        if (ImGui::IsItemDeactivatedAfterEdit()) SaveMosaicAnnotationsNow();

        char tbuf[48];
        std::snprintf(tbuf, sizeof(tbuf), "%s", tag.c_str());
        ImGui::AlignTextToFramePadding();
        ImGui::TextUnformatted("tag");
        ImGui::SameLine(labelW);
        ImGui::SetNextItemWidth(-ImGui::GetFrameHeight() - ImGui::GetStyle().ItemSpacing.x);
        if (ImGui::InputTextWithHint("##seltag", "none", tbuf, sizeof(tbuf))) tag = MosaicTrim(tbuf);
        if (ImGui::IsItemDeactivatedAfterEdit()) SaveMosaicAnnotationsNow();
        ImGui::SameLine();
        // The tags already in use, one click away - a typo is a second tag.
        if (ImGui::BeginCombo("##seltagpick", nullptr, ImGuiComboFlags_NoPreview)) {
            if (tags.empty()) ImGui::TextDisabled("no tags yet - type one");
            for (const std::string& t : tags)
                if (ImGui::Selectable(t.c_str(), t == tag)) {
                    tag = t;
                    SaveMosaicAnnotationsNow();
                }
            if (!tag.empty() && ImGui::Selectable("(no tag)")) {
                tag.clear();
                SaveMosaicAnnotationsNow();
            }
            ImGui::EndCombo();
        }
    };
    if (mosaicSelMarker_ >= 0 && mosaicSelMarker_ < static_cast<int>(mosaicAnn_.markers.size())) {
        MosaicMarker& mk = mosaicAnn_.markers[static_cast<std::size_t>(mosaicSelMarker_)];
        ImGui::PushID("selmk");
        ImGui::TextDisabled("marker");
        nameAndTag(mk.name, mk.tag);
        double gx = 0.0, gy = 0.0;
        MosaicCanvasToGantry(rec.geom, mk.cx, mk.cy, &gx, &gy);
        ImGui::TextDisabled("gantry X %.4f  Y %.4f   (px %.1f, %.1f)", gx, gy, mk.cx, mk.cy);
        if (CanUse(kRoleJog)) {
            ImGui::BeginDisabled(orch_.Busy());
            if (ImGui::SmallButton("Go to")) GoToMosaicPoint(mk.cx, mk.cy, "marker '" + mk.name + "'");
            ImGui::EndDisabled();
            ImGui::SameLine();
        }
        if (ImGui::SmallButton("Centre")) centreOn(mk.cx, mk.cy);
        ImGui::SameLine();
        if (ImGui::SmallButton("Delete")) {
            FD_INFO("mosaic", "marker '%s' deleted", mk.name.c_str());
            mosaicAnn_.markers.erase(mosaicAnn_.markers.begin() + mosaicSelMarker_);
            mosaicSelMarker_ = -1;
            SaveMosaicAnnotationsNow();
        }
        ImGui::PopID();
    } else if (mosaicSelLabel_ >= 0 && mosaicSelLabel_ < static_cast<int>(mosaicAnn_.labels.size())) {
        MosaicLabel& lb = mosaicAnn_.labels[static_cast<std::size_t>(mosaicSelLabel_)];
        ImGui::PushID("sellb");
        ImGui::TextDisabled("crop label");
        nameAndTag(lb.name, lb.tag);
        ImGui::TextDisabled("%.0f x %.0f px, %.3f x %.3f mm", lb.x1 - lb.x0, lb.y1 - lb.y0,
                            (lb.x1 - lb.x0) * mmx, (lb.y1 - lb.y0) * mmy);
        ImGui::TextDisabled("exports to crops/%s%s.png",
                            lb.tag.empty() ? "" : (MosaicSafeName(lb.tag) + "/").c_str(),
                            MosaicSafeName(lb.name).c_str());
        const double cx = (lb.x0 + lb.x1) * 0.5, cy = (lb.y0 + lb.y1) * 0.5;
        if (CanUse(kRoleJog)) {
            ImGui::BeginDisabled(orch_.Busy());
            if (ImGui::SmallButton("Go to centre"))
                GoToMosaicPoint(cx, cy, "crop label '" + lb.name + "'");
            ImGui::EndDisabled();
            ImGui::SameLine();
        }
        if (ImGui::SmallButton("Centre")) centreOn(cx, cy);
        ImGui::SameLine();
        if (ImGui::SmallButton("Delete")) {
            FD_INFO("mosaic", "crop label '%s' deleted", lb.name.c_str());
            mosaicAnn_.labels.erase(mosaicAnn_.labels.begin() + mosaicSelLabel_);
            mosaicSelLabel_ = -1;
            SaveMosaicAnnotationsNow();
        }
        ImGui::PopID();
    } else {
        ImGui::TextDisabled("click a marker or crop label - in a list, or in the picture");
    }

    // ---- Export -----------------------------------------------------------------------------------
    ImGui::SeparatorText("Export");
    const MosaicWorker::Progress pr = mosaicWorker_.GetProgress();
    if (pr.active) {
        ImGui::ProgressBar(pr.frac, ImVec2(-FLT_MIN, 0.0f), pr.what.c_str());
        if (ImGui::Button("Cancel export", ImVec2(-FLT_MIN, 0.0f))) mosaicWorker_.CancelExport();
    } else {
        const bool canExport = !mosaicLive_ && !rec.tiles.empty();
        // What the filter shows is what is written: filter by tag, export that set.
        ImGui::BeginDisabled(!canExport || shownL.empty());
        char b[80];
        if (filtering) std::snprintf(b, sizeof(b), "Export crops (%zu shown of %zu)", shownL.size(), mosaicAnn_.labels.size());
        else           std::snprintf(b, sizeof(b), "Export crops (%zu)", shownL.size());
        if (ImGui::Button(b, ImVec2(-FLT_MIN, 0.0f))) {
            MosaicWorker::Export job;
            job.dir        = mosaicDir_;
            job.rec        = rec;
            job.kind       = MosaicWorker::JobKind::Crops;
            job.useRefined = mosaicUseRefined_;
            for (int i : shownL) job.labels.push_back(mosaicAnn_.labels[static_cast<std::size_t>(i)]);
            std::string why;
            if (!mosaicWorker_.StartExport(std::move(job), &why)) mosaicExportMsg_ = why;
        }
        ImGui::EndDisabled();

        double x0 = 0, y0 = 0, x1 = 0, y1 = 0;
        MosaicRecordExtent(rec, &x0, &y0, &x1, &y1, mosaicUseRefined_);
        mosaicExportScale_ = std::clamp(mosaicExportScale_, 0, kScaleCount - 1);
        const auto scaleLabel = [&](int i, char* out, std::size_t n) {
            const int s = kScales[i];
            const double w = std::ceil((x1 - x0) / s), h = std::ceil((y1 - y0) / s);
            std::snprintf(out, n, "1:%d  (%.0f x %.0f px, %.0f MP)", s, w, h, w * h / 1e6);
        };
        char cur[96];
        scaleLabel(mosaicExportScale_, cur, sizeof(cur));
        ImGui::SetNextItemWidth(-1.0f);
        if (ImGui::BeginCombo("##mosaicscale", cur)) {
            for (int i = 0; i < kScaleCount; ++i) {
                char item[96];
                scaleLabel(i, item, sizeof(item));
                if (ImGui::Selectable(item, i == mosaicExportScale_)) mosaicExportScale_ = i;
            }
            ImGui::EndCombo();
        }
        ImGui::BeginDisabled(!canExport);
        if (ImGui::Button("Export mosaic", ImVec2(-FLT_MIN, 0.0f))) {
            MosaicWorker::Export job;
            job.dir   = mosaicDir_;
            job.rec   = rec;
            job.scale      = kScales[mosaicExportScale_];
            job.useRefined = mosaicUseRefined_;
            std::string why;
            if (!mosaicWorker_.StartExport(std::move(job), &why)) mosaicExportMsg_ = why;
        }
        ImGui::EndDisabled();
        if (mosaicLive_) ImGui::TextDisabled("exports wait for the capture to finish");
        ImGui::TextDisabled("a 1:1 export past 40 MP is written uncompressed");
    }
    if (!mosaicExportMsg_.empty()) {
        ImGui::TextWrapped("%s", mosaicExportMsg_.c_str());
        if (!mosaicExportPath_.empty() && ImGui::SmallButton("Open export folder")) {
            std::error_code ec;
            const fs::path p(mosaicExportPath_);
            OpenFolderInExplorer(fs::is_directory(p, ec) ? p.string() : p.parent_path().string());
        }
    }
}

} // namespace firedog
