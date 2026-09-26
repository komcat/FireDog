// PageRun.cpp : the Run page (View menu, full-page) — the program deck, the
// execution view, the Scan band (Raster | Climb | XCAN) and the scan-preset
// modals.
// Definitions of App members — see App.h. Design: docs/run-page-design.md.
//
// Execution adds NO engine: StartDeckRun resolves one queue from the deck
// selection (slot / recipe) and ProgramRunner sequences ordinary RecipeEngine
// runs from it, so each recipe keeps its own validation, progress, prompts and
// $run DUT event. Ctx does NOT flow between recipes.
//
// The three scan launchers inject their form values as the scripts' ctx keys
// via LuaEngine::Env::initialCtx — the same channel recipe ctx_set steps use.
// Exit targets are entered in uA and multiplied by 1e-6 into feedback units
// here.
#include "App.h"

#include "imgui.h"

#include "ui/Widgets.h"
#include "ui/Theme.h"   // kTeachOrange
#include "core/DataBus.h"
#include "core/DutSession.h"
#include "core/Log.h"
#include "core/Topics.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <map>
#include <set>
#include <string>

namespace firedog {

// ---- Run page (View menu, full page) — programs = slots of recipes ----------
// Operator surface: the run strip on top (Execute, Save, the loaded program +
// Edit), the deck on the left (ONE slot and its recipes, selectable), execution
// in the middle (queue with marks + progress), DUT on the right. Both run
// granularities fill the SAME runner queue; one Stop kills the whole chain.
// Design: docs/run-page-design.md.

// Raster first-light exit levels, in uA (feedback amps; injected x1e-6).
static constexpr double kRasterTargetsUa[] = {5.0, 20.0};

// Built-in XCAN target presets: the three product power classes (the
// zone_low must sit AT the raster handoff level — see the micro-lobe field
// note in hexapod_align.lua); slots 4-5 are spares the operator can switch
// on in the target settings modal (right-click a Target button).
static XcanTarget DefaultXcanTarget(int i)
{
    XcanTarget t;
    switch (i) {
    case 0: t.targetUa = 150.0;  t.zoneLowUa = 10.0;  t.zoneHighUa = 200.0;
            t.enabled = true; break;
    case 1: t.targetUa = 400.0;  t.zoneLowUa = 20.0;  t.zoneHighUa = 400.0;
            t.enabled = true; break;
    case 2: t.targetUa = 3400.0; t.zoneLowUa = 200.0; t.zoneHighUa = 3400.0;
            t.enabled = true; break;
    default: break;   // spare slot, struct defaults, hidden
    }
    return t;
}

// Target buttons are labeled by their value ("400 uA").
static std::string XcanTargetLabel(const XcanTarget& t)
{
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%g uA", t.targetUa);
    return buf;
}

// Built-in XCAN launch presets: slots 1/2 are bobcat's Coarse/Fine ladders
// (enabled), 3-5 are blank spares the operator can switch on in the
// settings modal (right-click any preset button).
static XcanPreset DefaultXcanPreset(int i)
{
    XcanPreset p;
    switch (i) {
    case 0:  p.label = "Coarse"; p.stepsUm = "10,5,2,1"; p.enabled = true; break;
    case 1:  p.label = "Fine";   p.stepsUm = "2,1,0.5";  p.enabled = true; break;
    default: p.label = "P" + std::to_string(i + 1);      p.enabled = false; break;
    }
    return p;
}

static nlohmann::json XcanPresetToJson(const XcanPreset& p)
{
    return nlohmann::json{
        {"label", p.label},
        {"enabled", p.enabled},
        {"steps_um", p.stepsUm},
        {"axes", p.axes},
        {"settle_ms", p.settleMs},
        {"settle_min_ms", p.settleMinMs},
        {"fresh_samples", p.freshSamples},
        {"avg_count", p.avgCount},
        {"consec_limit", p.consecLimit},
        {"sig_drop_pct", p.sigDropPct},
        {"max_travel_um", p.maxTravelUm},
        {"improve_pct", p.improvePct},
        {"min_signal", p.minSignal},
        {"tol_mm", p.tolMm},
        {"move_timeout_ms", p.moveTimeoutMs},
        {"velocity_mm_s", p.velocityMmS},
        {"skip_opposite", p.skipOpposite},
        {"dir_memory", p.dirMemory},
        {"reuse_sample", p.reuseSample},
    };
}

// Tolerant overlay: missing keys keep the preset's current (default) value.
static void XcanPresetFromJson(const nlohmann::json& j, XcanPreset& p)
{
    p.label         = j.value("label", p.label);
    p.enabled       = j.value("enabled", p.enabled);
    p.stepsUm       = j.value("steps_um", p.stepsUm);
    p.axes          = j.value("axes", p.axes);
    p.settleMs      = j.value("settle_ms", p.settleMs);
    p.settleMinMs   = j.value("settle_min_ms", p.settleMinMs);
    p.freshSamples  = j.value("fresh_samples", p.freshSamples);
    p.avgCount      = j.value("avg_count", p.avgCount);
    p.consecLimit   = j.value("consec_limit", p.consecLimit);
    p.sigDropPct    = j.value("sig_drop_pct", p.sigDropPct);
    p.maxTravelUm   = j.value("max_travel_um", p.maxTravelUm);
    p.improvePct    = j.value("improve_pct", p.improvePct);
    p.minSignal     = j.value("min_signal", p.minSignal);
    p.tolMm         = j.value("tol_mm", p.tolMm);
    p.moveTimeoutMs = j.value("move_timeout_ms", p.moveTimeoutMs);
    p.velocityMmS   = j.value("velocity_mm_s", p.velocityMmS);
    p.skipOpposite  = j.value("skip_opposite", p.skipOpposite);
    p.dirMemory     = j.value("dir_memory", p.dirMemory);
    p.reuseSample   = j.value("reuse_sample", p.reuseSample);
}

// Built-in climb-raster launch presets, coarse -> fine like the XCAN ladders
// beside them — same word, same meaning: how big a bite each pass takes.
//   Coarse - closing a real gap. 2 um Z steps over the full 20 um budget, a
//            1 um plane grid, and the Z probe on so planes are skipped while
//            stepping alone is still paying.
//   Fine   - the last approach. 0.5 um Z steps over 5 um, a 0.25 um plane
//            grid, and NO probe: every plane is searched, because this pass
//            runs where XY misalignment (not the gap) is what is left.
// The plane-scan shape (cross vs spiral) is deliberately NOT what separates
// them — it answers "is this peak axis-separable?", not "how close am I?", so
// it stays a checkbox on both. 3-5 are blank spares the operator can switch on
// in the settings modal (right-click any preset button).
static ClimbPreset DefaultClimbPreset(int i)
{
    ClimbPreset p;
    switch (i) {
    case 0:
        p.label = "Coarse"; p.enabled = true;
        p.zStepUm = 2.0;  p.maxZUm   = 20.0;
        p.xyStepUm = 1.0; p.zProbePct = 5.0;
        break;
    case 1:
        p.label = "Fine"; p.enabled = true;
        p.zStepUm = 0.5;  p.maxZUm   = 5.0;
        p.xyStepUm = 0.25; p.zProbePct = 0.0;   // search every plane
        break;
    default:
        p.label = "C" + std::to_string(i + 1); p.enabled = false; break;
    }
    return p;
}

static nlohmann::json ClimbPresetToJson(const ClimbPreset& p)
{
    return nlohmann::json{
        {"label", p.label},
        {"enabled", p.enabled},
        {"target_pct", p.targetPct},
        {"z_step_um", p.zStepUm},
        {"max_z_um", p.maxZUm},
        {"z_probe_pct", p.zProbePct},
        {"max_skips", p.maxSkips},
        {"spiral", p.spiral},
        {"xy_step_um", p.xyStepUm},
        {"xy_steps", p.xySteps},
        {"peak_drop", p.peakDrop},
        {"rings_stop", p.ringsStop},
        {"improve_pct", p.improvePct},
        {"contact_drop_pct", p.contactDropPct},
        {"stall_planes", p.stallPlanes},
        {"z_axis", p.zAxis},
        {"plane_axes", p.planeAxes},
        {"settle_ms", p.settleMs},
        {"settle_min_ms", p.settleMinMs},
        {"z_settle_ms", p.zSettleMs},
        {"fresh_samples", p.freshSamples},
        {"avg_count", p.avgCount},
        {"tol_mm", p.tolMm},
        {"move_timeout_ms", p.moveTimeoutMs},
        {"velocity_mm_s", p.velocityMmS},
    };
}

// Tolerant overlay: missing keys keep the preset's current (default) value.
static void ClimbPresetFromJson(const nlohmann::json& j, ClimbPreset& p)
{
    p.label          = j.value("label", p.label);
    p.enabled        = j.value("enabled", p.enabled);
    p.targetPct      = j.value("target_pct", p.targetPct);
    p.zStepUm        = j.value("z_step_um", p.zStepUm);
    p.maxZUm         = j.value("max_z_um", p.maxZUm);
    p.zProbePct      = j.value("z_probe_pct", p.zProbePct);
    p.maxSkips       = j.value("max_skips", p.maxSkips);
    p.spiral         = j.value("spiral", p.spiral);
    p.xyStepUm       = j.value("xy_step_um", p.xyStepUm);
    p.xySteps        = j.value("xy_steps", p.xySteps);
    p.peakDrop       = j.value("peak_drop", p.peakDrop);
    p.ringsStop      = j.value("rings_stop", p.ringsStop);
    p.improvePct     = j.value("improve_pct", p.improvePct);
    p.contactDropPct = j.value("contact_drop_pct", p.contactDropPct);
    p.stallPlanes    = j.value("stall_planes", p.stallPlanes);
    p.zAxis          = j.value("z_axis", p.zAxis);
    p.planeAxes      = j.value("plane_axes", p.planeAxes);
    p.settleMs       = j.value("settle_ms", p.settleMs);
    p.settleMinMs    = j.value("settle_min_ms", p.settleMinMs);
    p.zSettleMs      = j.value("z_settle_ms", p.zSettleMs);
    p.freshSamples   = j.value("fresh_samples", p.freshSamples);
    p.avgCount       = j.value("avg_count", p.avgCount);
    p.tolMm          = j.value("tol_mm", p.tolMm);
    p.moveTimeoutMs  = j.value("move_timeout_ms", p.moveTimeoutMs);
    p.velocityMmS    = j.value("velocity_mm_s", p.velocityMmS);
}

// Activatable preset button: theme color normally, yellow when active.
static bool PresetButton(const char* label, bool active, const ImVec2& size)
{
    if (active) {
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.85f, 0.68f, 0.10f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.95f, 0.78f, 0.18f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(1.00f, 0.85f, 0.25f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.08f, 0.08f, 0.08f, 1.0f));
    }
    const bool clicked = ImGui::Button(label, size);
    if (active) ImGui::PopStyleColor(4);
    return clicked;
}

// SI-prefixed reading for the scan header (bobcat FormatSI: 2.09e-05 -> 20.888u).
static std::string FormatSI(double v)
{
    if (v == 0.0) return "0";
    const double a = std::fabs(v);
    const char*  suf;
    double       s;
    if      (a < 1e-9) { suf = "p"; s = v * 1e12; }
    else if (a < 1e-6) { suf = "n"; s = v * 1e9;  }
    else if (a < 1e-3) { suf = "u"; s = v * 1e6;  }
    else if (a < 1.0)  { suf = "m"; s = v * 1e3;  }
    else               { suf = "";  s = v;        }
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%.3f%s", s, suf);
    return buf;
}

// ---- Deck selection: one program, one slot ---------------------------------
// The deck shows exactly ONE slot, and the pair (program, slot) is what the
// operator comes back to after a restart — so every place that moves it goes
// through these three, and nowhere assigns progSel_/deckSlot_ on its own.

// Persist the current pair by UID (a rename must not lose it). Called from the
// pick sites rather than from Draw generally: this writes a file.
void App::RememberDeckSelection()
{
    const bool have = progSel_ >= 0 &&
                      progSel_ < static_cast<int>(session_.Programs().size());
    runLastProgUid_ = have ? session_.Programs()[progSel_].uid : std::string();
    runLastSlot_    = deckSlot_ > 0 ? deckSlot_ : 0;
    SaveUiSettings();
}

// Show one slot of the current program. -1 (or an empty program) is the only
// no-slot state; a slot change always drops the node selection, because a node
// uid from one slot resolves to nothing in another.
void App::SelectDeckSlot(int slot)
{
    const bool have = progSel_ >= 0 &&
                      progSel_ < static_cast<int>(session_.Programs().size());
    const int n = have ? static_cast<int>(session_.Programs()[progSel_].slots.size()) : 0;
    if (n == 0)            deckSlot_ = -1;
    else if (slot < 0)     deckSlot_ = 0;
    else if (slot >= n)    deckSlot_ = n - 1;
    else                   deckSlot_ = slot;
    deckNode_.clear();
    RememberDeckSelection();
}

// Load a program into the deck (the Load Program modal, New, Delete). Opening a
// program lands on its first slot — the remembered slot belongs to whichever
// program was showing, not to this one.
void App::SelectDeckProgram(int idx)
{
    progSel_ = (idx >= 0 && idx < static_cast<int>(session_.Programs().size())) ? idx : -1;
    // The organizer's slot belongs to whichever program was open in it; carried
    // into a different program it would silently address a different station.
    orgSlot_ = -1;
    orgSel_.clear();
    SelectDeckSlot(0);
}

// After a document load: re-point at the remembered pair if this set still has
// it, else the first program's first slot. Does NOT re-save — a product that
// simply lacks the remembered program must not erase it.
void App::RestoreDeckSelection()
{
    progSel_ = session_.Programs().empty() ? -1 : 0;
    if (!runLastProgUid_.empty()) {
        for (int i = 0; i < static_cast<int>(session_.Programs().size()); ++i) {
            if (session_.Programs()[i].uid == runLastProgUid_) { progSel_ = i; break; }
        }
    }
    deckSlot_ = -1;
    if (progSel_ >= 0) {
        const int n = static_cast<int>(session_.Programs()[progSel_].slots.size());
        if (n > 0) deckSlot_ = (runLastSlot_ >= 0 && runLastSlot_ < n) ? runLastSlot_ : 0;
    }
    deckNode_.clear();
    orgSlot_ = -1;      // the organizer re-seeds off the deck on its next frame
    orgSel_.clear();
}

// The deck's Execute button. The UI contributes the SELECTION; resolving it into
// a queue is BuildRunQueue's job (command/ProgramRunner), and executing that
// queue is the runner's.
RecipeEngine::StartOpts App::RecipeStartOpts() const
{
    RecipeEngine::StartOpts opts;
    opts.teach = teachMode_;
    return opts;
}

void App::StartDeckRun(RunScope scope)
{
    runStartErr_.clear();
    if (progSel_ < 0 || progSel_ >= static_cast<int>(session_.Programs().size())) {
        runStartErr_ = "no program selected";
        return;
    }
    const Program& prog = session_.Programs()[progSel_];

    RunQueue q = BuildRunQueue(prog, scope, deckSlot_, deckNode_,
                               session_.Recipes(), &runStartErr_);
    if (q.items.empty()) return;   // reason already in runStartErr_

    // Every deck run starts with no FRESH taught frame. A frame from an earlier
    // run was measured against a part that may since have been handled, and
    // move_local would drive to it without complaint — so the run must teach
    // its own or fail loudly. The one exception is a frame its align_run marked
    // KEPT: it survives as `carried`, and only a step that opts in with
    // use_kept_frame will move against it. DUT open / product load still wipe
    // it. Here rather than inside ProgramRunner because the frames are Workflow
    // state and layer 2.5 does not own them.
    workflow_.CarryLiveFrames("program start");

    // Running the START state is what "a new part" means, so it — and a slot
    // run, which necessarily contains it — is the ONE thing that wipes the
    // cycle's accents. Nothing else does: they are the operator's record of how
    // far this part got, and a page switch or a restart must not erase it.
    // Beside ClearLiveFrames because it is the same kind of boundary, declared
    // in the same place, for the same reason.
    // A TEACH run is none of this: it re-teaches positions and says nothing
    // about a part, so the accents and the numbers of the part on the machine
    // stay exactly as they were.
    if (!teachMode_ && deckSlot_ >= 0 && deckSlot_ < static_cast<int>(prog.slots.size())) {
        const ProgramSlot& slot = prog.slots[static_cast<std::size_t>(deckSlot_)];
        if (scope == RunScope::Slot ||
            (!deckNode_.empty() && deckNode_ == slot.startNodeUid)) {
            deckMarks_.clear();
            cycleVars_.clear();   // same boundary: the ticks and the numbers
                                  // must never describe two different parts
            FD_INFO("program", "deck cycle reset at '%s'", slot.name.c_str());
        }
    }

    RecipeEngine::Env env = session_.MakeRecipeEnv();
    if (!orch_.programs.Start(q.kind, q.label, prog.uid, std::move(q.items),
                              std::move(env), &runStartErr_, RecipeStartOpts())) {
        FD_WARN("program", "run rejected: %s", runStartErr_.c_str());
        return;
    }
    // What actually launched, for AdvanceDeckCycle. A slot run leaves it empty:
    // it ran the whole chain, so there is nowhere to advance to. A teach run
    // leaves it empty too: the cycle does not move for a run that taught.
    runNodeUid_ = (scope == RunScope::Node && !teachMode_) ? deckNode_ : std::string();
}

// ---- Deck cycle -------------------------------------------------------------
// Design: docs/deck-cycle-design.md. None of this touches the runner: the marks
// are read out of the status it already keeps, and the advance is a selection
// change on the UI thread.

bool App::DeckCycleRange(const ProgramSlot& slot, int* startIdx, int* endIdx) const
{
    if (slot.startNodeUid.empty() || slot.endNodeUid.empty()) return false;
    int si = -1, ei = -1;
    for (int i = 0; i < static_cast<int>(slot.items.size()); ++i) {
        const std::string& u = slot.items[static_cast<std::size_t>(i)].uid;
        if (u == slot.startNodeUid) si = i;
        if (u == slot.endNodeUid)   ei = i;
    }
    // Not found at the TOP level (the node was dragged into a folder), or the
    // end sits above the start: that is a mistake, not a wrapping cycle. Either
    // way the cycle is undefined — the organizer says so and offers Clear.
    if (si < 0 || ei < 0 || ei < si) return false;
    if (startIdx) *startIdx = si;
    if (endIdx)   *endIdx   = ei;
    return true;
}

App::NodeMark App::DeckMarkOf(const ProgramNode& n) const
{
    if (!n.IsFolder()) {
        const auto it = deckMarks_.find(n.uid);
        return it == deckMarks_.end() ? NodeMark::None : it->second;
    }
    // Only recipe leaves become queue items, so a folder has no mark of its
    // own. One failed leaf makes the folder failed; it is green only once
    // every leaf under it is.
    bool anyPassed = true, sawOne = false;
    for (const ProgramNode& c : n.children) {
        const NodeMark m = DeckMarkOf(c);
        if (m == NodeMark::Failed) return NodeMark::Failed;
        if (m == NodeMark::None)   anyPassed = false;
        else                       sawOne = true;
    }
    return (sawOne && anyPassed) ? NodeMark::Passed : NodeMark::None;
}

void App::DrainDeckMarks()
{
    // A teach run's items are not outcomes for the part: no accent from them.
    if (orch_.programs.GetStatus().teach) return;
    // Queue order, and Pending writes nothing — which is what makes a x100
    // folder that failed on pass 3 stay yellow instead of being overwritten by
    // the ninety-seven passes that never ran.
    for (const ProgramRunner::ItemStatus& it : orch_.programs.GetStatus().items) {
        if (it.nodeUid.empty()) continue;
        switch (it.mark) {
        case ProgramRunner::Mark::Done:
            deckMarks_[it.nodeUid] = NodeMark::Passed;
            break;
        case ProgramRunner::Mark::Failed:
        case ProgramRunner::Mark::Stopped:
            // A Stop is yellow too. To somebody looking at a part the useful
            // distinction is passed / not-passed; HOW it failed to pass is what
            // the log band is for.
            deckMarks_[it.nodeUid] = NodeMark::Failed;
            break;
        default:
            break;
        }
    }
}

void App::AdvanceDeckCycle()
{
    const std::string ran = runNodeUid_;
    runNodeUid_.clear();
    if (ran.empty()) return;   // slot scope: it ran the whole chain
    // Hold on a failure, so the very same press retries the step that failed.
    if (orch_.programs.GetStatus().result.rfind("Completed", 0) != 0) return;
    if (progSel_ < 0 || progSel_ >= static_cast<int>(session_.Programs().size())) return;
    const Program& prog = session_.Programs()[progSel_];
    if (deckSlot_ < 0 || deckSlot_ >= static_cast<int>(prog.slots.size())) return;
    const ProgramSlot& slot = prog.slots[static_cast<std::size_t>(deckSlot_)];

    int si = 0, ei = 0;
    if (!DeckCycleRange(slot, &si, &ei)) return;
    int idx = -1;
    for (int i = si; i <= ei; ++i) {
        if (slot.items[static_cast<std::size_t>(i)].uid == ran) { idx = i; break; }
    }
    if (idx < 0) return;   // ran something outside the chain (troubleshooting)
    // At the end, wrap to the start: ready for the next part, with this part's
    // accents still up until that start press clears them.
    deckNode_ = slot.items[static_cast<std::size_t>(idx == ei ? si : idx + 1)].uid;
}

// ---- Cycle values ----------------------------------------------------------

bool App::IsOperatorCtxKey(const std::string& key)
{
    // The alignment scripts push a dozen scratch values through
    // the same ctx a recipe measures into. They are engine bookkeeping, and
    // burying DarkCurrent under them makes the panel useless at a glance. An
    // underscore is the escape hatch for a recipe author who wants a key kept
    // out of the operator's sight.
    if (key.empty() || key[0] == '_') return false;
    if (key.rfind("align_", 0) == 0)  return false;
    return true;
}

void App::DrainCycleVars()
{
    const bool eng = orch_.recipe.Running();
    const bool justFinished = engineWasRunning_ && !eng;
    engineWasRunning_ = eng;
    if (!justFinished) return;

    // Status.ctx is EMPTY by now (the engine drops its pointer before the state
    // goes terminal) — lastCtx is the snapshot it took at that boundary.
    const RecipeEngine::Status es = orch_.recipe.GetStatus();
    if (es.teach) return;   // a teach run measured nothing about the part
    for (const auto& [key, val] : es.lastCtx) {
        if (!IsOperatorCtxKey(key)) continue;
        cycleVars_[key] = CycleVar{CtxValueToString(val), es.recipeName};
    }
}

// Worker -> UI: turn the parked still into a texture. From App::Update, never
// from a Draw — EnsureTextureRGB is a GL call, and the pixels have crossed a
// thread to get here.
//
// The frame is MOVED out of the Workflow slot, so the ~15 MB lives in exactly
// one place at a time. It is dropped again the moment the texture exists: the
// bytes were only ever a means to that texture, and the GPU has them now.
void App::DrainCapturedView()
{
    Workflow::CapturedView v;
    if (!workflow_.TakeCapturedView(&v)) return;
    if (v.w <= 0 || v.h <= 0 || v.rgb.empty()) return;

    runShotTex_   = host_.EnsureTextureRGB(runShotTex_, v.w, v.h, v.rgb.data());
    runShotW_     = v.w;
    runShotH_     = v.h;
    runShotLabel_ = std::move(v.label);
    runShotStamp_ = std::move(v.stamp);
    runShotPath_  = std::move(v.savedPath);
}

// The image box, top of the Run page's right column — above the DUT block
// because that is the order the operator works in: look at the part, read what
// it says, THEN type the serial and open the record.
//
// One still, not a stream. The Aux window already shows what the camera sees
// right now; this shows what it was pointed at when a step said so, and it
// stays put while the stage drives on. Those are different questions and a live
// view cannot answer the second one.
void App::DrawRunShotPanel()
{
    ImGui::BeginChild("runShot", ImVec2(0.0f, runShotPanelH_), ImGuiChildFlags_Borders);
    // Emptiness is the SIZE being zero, not the texture id being zero: Clear
    // keeps the id (see below), so testing the id would leave the box drawing
    // a 0x0 header over last part's picture.
    if (runShotTex_ == 0 || runShotW_ <= 0 || runShotH_ <= 0) {
        ImGui::TextDisabled("Image");
        ImGui::Separator();
        ImGui::TextDisabled("Nothing captured yet.");
        ImGui::TextDisabled("A recipe step (capture_view) puts");
        ImGui::TextDisabled("a picture here.");
        ImGui::EndChild();
        return;
    }

    // Header: caption, and a Clear that says the picture is stale rather than
    // leaving last part's label over this part's run.
    ImGui::TextUnformatted(runShotLabel_.empty() ? "Image" : runShotLabel_.c_str());
    ImGui::SameLine();
    const float clrW = ImGui::CalcTextSize("Clear").x +
                       ImGui::GetStyle().FramePadding.x * 2.0f;
    ImGui::SetCursorPosX(ImGui::GetContentRegionMax().x - clrW);
    if (ImGui::SmallButton("Clear")) {
        // The texture id is KEPT and reused by the next capture. Freeing it
        // would buy nothing back: one texture is the ceiling either way, and a
        // reused id is what EnsureTextureRGB is for.
        runShotW_ = runShotH_ = 0;
        runShotLabel_.clear();
        runShotStamp_.clear();
        runShotPath_.clear();
        ImGui::EndChild();
        return;
    }
    ImGui::Separator();

    // Fit the whole frame inside what is left, aspect preserved, centred. FIT
    // and not cover: a serial number half outside the box is the one failure
    // this panel exists to avoid.
    const ImVec2 avail = ImGui::GetContentRegionAvail();
    const float  footH = ImGui::GetTextLineHeightWithSpacing();
    const float  imgH  = avail.y - footH;
    if (imgH > 8.0f && avail.x > 8.0f && runShotW_ > 0 && runShotH_ > 0) {
        const float sx = avail.x / static_cast<float>(runShotW_);
        const float sy = imgH / static_cast<float>(runShotH_);
        const float sc = sx < sy ? sx : sy;
        const ImVec2 sz(static_cast<float>(runShotW_) * sc,
                        static_cast<float>(runShotH_) * sc);
        ImGui::SetCursorPosX(ImGui::GetCursorPosX() + (avail.x - sz.x) * 0.5f);
        ImGui::Image((ImTextureID)(intptr_t)runShotTex_, sz);   // C-style cast: void*/ImU64 ImTextureID
        // Hover to read it at full size: at this width a serial is a few pixels
        // tall, and shrinking the deck to read one is not a reasonable ask.
        if (ImGui::IsItemHovered()) {
            ImGui::BeginTooltip();
            const float tw = 640.0f;
            const float ts = tw / static_cast<float>(runShotW_);
            ImGui::Image((ImTextureID)(intptr_t)runShotTex_,
                         ImVec2(tw, static_cast<float>(runShotH_) * ts));
            ImGui::EndTooltip();
        }
    }
    // Footer: what it is and whether it was filed. "saved" is the difference
    // between a picture that is evidence and one that is only a glance.
    ImGui::TextDisabled("%dx%d  %s%s", runShotW_, runShotH_, runShotStamp_.c_str(),
                        runShotPath_.empty() ? "" : "  saved");
    if (!runShotPath_.empty() && ImGui::IsItemHovered())
        ImGui::SetTooltip("Filed in the DUT record as %s", runShotPath_.c_str());
    ImGui::EndChild();
}

// The Run page's right column, under the DUT block: what this part measured.
// The live keys of a RUNNING recipe are merged over the accumulated ones and
// marked, so there is one list to read rather than two.
void App::DrawRunValuesPanel()
{
    const RecipeEngine::Status es = orch_.recipe.GetStatus();
    std::map<std::string, CycleVar> shown = cycleVars_;
    std::set<std::string> live;
    if (orch_.recipe.Running()) {
        for (const auto& [key, val] : es.ctx) {
            if (!IsOperatorCtxKey(key)) continue;
            shown[key] = CycleVar{CtxValueToString(val), es.recipeName};
            live.insert(key);
        }
    }

    ImGui::SeparatorText(shown.empty() ? "Values"
                                       : ("Values (" + std::to_string(shown.size()) + ")").c_str());
    if (shown.empty()) {
        ImGui::TextDisabled("Nothing measured yet this cycle.");
        return;
    }
    // Two columns so the numbers line up in a column the eye can run down; the
    // recipe that wrote each one is a tooltip rather than a third column,
    // because the right panel is narrow and the name is the rarer question.
    if (ImGui::BeginTable("runvals", 2,
                          ImGuiTableFlags_SizingStretchProp | ImGuiTableFlags_RowBg)) {
        for (const auto& [key, v] : shown) {
            const bool isLive = live.count(key) != 0;
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            if (isLive) ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.85f, 0.3f, 1.0f));
            ImGui::TextUnformatted(key.c_str());
            if (isLive) ImGui::PopStyleColor();
            if (ImGui::IsItemHovered() && !v.source.empty())
                ImGui::SetTooltip("written by '%s'", v.source.c_str());
            ImGui::TableNextColumn();
            ImGui::TextUnformatted(v.value.c_str());
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("%s", v.value.c_str());
        }
        ImGui::EndTable();
    }
}

// The instrument readouts the operator can pin under the Values list. A fixed
// catalogue of the three bench instruments' streams rather than every topic on
// the bus: a motion axis or a vision score has its own page, and this block is
// for "is the laser warm, is the rail up" while a run goes by. The SMU has no
// voltage stream — its bias is a setting, not a measurement — so only its two
// current channels are offered.
namespace {
enum class ReadoutKind { Temp, MilliAmp, Volt, Amp };
struct ReadoutDef {
    std::string  topic;
    std::string  label;
    ReadoutKind  kind;
};
std::vector<ReadoutDef> ReadoutCatalogue()
{
    std::vector<ReadoutDef> cat;
    cat.push_back({topics::kCldTemp,    "CLD temp",    ReadoutKind::Temp});
    cat.push_back({topics::kCldCurrent, "CLD current", ReadoutKind::MilliAmp});
    for (int ch = 1; ch <= 3; ++ch) {
        cat.push_back({topics::PsuVoltage(ch), "PSU ch" + std::to_string(ch) + " V", ReadoutKind::Volt});
        cat.push_back({topics::PsuCurrent(ch), "PSU ch" + std::to_string(ch) + " I", ReadoutKind::Amp});
    }
    cat.push_back({topics::kKeithleyCh1, "SMU ch1 I", ReadoutKind::Amp});
    cat.push_back({topics::kKeithleyCh2, "SMU ch2 I", ReadoutKind::Amp});
    return cat;
}
const ReadoutDef* FindReadout(const std::vector<ReadoutDef>& cat, const std::string& topic)
{
    for (const ReadoutDef& d : cat) if (d.topic == topic) return &d;
    return nullptr;
}
std::string FormatReadout(ReadoutKind kind, double v)
{
    char buf[48];
    switch (kind) {
    case ReadoutKind::Temp:     std::snprintf(buf, sizeof(buf), "%.2f C",  v); return buf;
    case ReadoutKind::MilliAmp: std::snprintf(buf, sizeof(buf), "%.2f mA", v); return buf;
    case ReadoutKind::Volt:     std::snprintf(buf, sizeof(buf), "%.3f V",  v); return buf;
    case ReadoutKind::Amp:      return FormatCurrent(v);
    }
    return "?";
}
constexpr std::size_t kMaxRunReadouts   = 4;
constexpr double      kRunReadoutPeriod = 0.25;   // s between pulls off the bus
}

void App::DrawRunReadouts()
{
    const std::vector<ReadoutDef> cat = ReadoutCatalogue();

    // Pull on a timer. Only topics the bus already HAS are touched:
    // DataBus::Stream creates on demand, and a phantom empty "psu/ch3/v"
    // would then show up in every chart's topic picker.
    const double now = ImGui::GetTime();
    if (runReadoutVals_.size() != runReadouts_.size() ||
        now - runReadoutAt_ >= kRunReadoutPeriod) {
        runReadoutAt_ = now;
        runReadoutVals_.assign(runReadouts_.size(), 0.0);
        runReadoutHas_.assign(runReadouts_.size(), false);
        const std::vector<std::string> present = DataBus::Instance().Topics();
        const std::set<std::string> have(present.begin(), present.end());
        for (std::size_t i = 0; i < runReadouts_.size(); ++i) {
            if (!have.count(runReadouts_[i].topic)) continue;
            const DataStream& s = DataBus::Instance().Stream(runReadouts_[i].topic);
            if (s.Count() == 0) continue;
            runReadoutHas_[i]  = true;
            runReadoutVals_[i] = s.LatestValue();
        }
    }

    // The + goes first: SeparatorText takes the rest of the line.
    const bool full = runReadouts_.size() >= kMaxRunReadouts;
    ImGui::BeginDisabled(full);
    if (ImGui::SmallButton("+")) ImGui::OpenPopup("runreadpick");
    ImGui::EndDisabled();
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
        ImGui::SetTooltip(full ? "Four readouts is the most this block shows"
                               : "Pin an instrument value here");
    ImGui::SameLine();
    ImGui::SeparatorText("Readouts");
    if (ImGui::BeginPopup("runreadpick")) {
        for (const ReadoutDef& d : cat) {
            bool pinned = false;
            for (const RunReadout& r : runReadouts_) if (r.topic == d.topic) { pinned = true; break; }
            if (pinned) continue;
            if (ImGui::Selectable(d.label.c_str())) {
                RunReadout r;
                r.topic = d.topic;
                runReadouts_.push_back(std::move(r));
                runReadoutAt_ = 0.0;   // pull on the next frame
                SaveUiSettings();
            }
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("%s", d.topic.c_str());
        }
        ImGui::EndPopup();
    }

    if (runReadouts_.empty()) {
        ImGui::TextDisabled("None pinned. Press + to choose.");
        return;
    }
    // A limit colours the value and nothing else: green inside, red outside,
    // plain when none is set or no data has arrived. It is a readout, not an
    // interlock - nothing here stops a run.
    const auto inBand = [](const RunReadout& r, double v, bool* judged) -> bool {
        *judged = true;
        if (r.mode == "above")   return v > r.lo;
        if (r.mode == "below")   return v < r.hi;
        if (r.mode == "between") return v >= r.lo && v <= r.hi;
        *judged = false;
        return true;
    };
    static const char* kModes[4]      = {"none", "above", "below", "between"};
    static const char* kModeLabels[4] = {"No limit", "Green above", "Green below", "Green between"};
    int  remove  = -1;
    bool changed = false;
    if (ImGui::BeginTable("runreadouts", 4,
                          ImGuiTableFlags_SizingStretchProp | ImGuiTableFlags_RowBg)) {
        ImGui::TableSetupColumn("##lbl", ImGuiTableColumnFlags_WidthStretch, 1.0f);
        ImGui::TableSetupColumn("##val", ImGuiTableColumnFlags_WidthStretch, 1.0f);
        ImGui::TableSetupColumn("##lim", ImGuiTableColumnFlags_WidthFixed);
        ImGui::TableSetupColumn("##rm",  ImGuiTableColumnFlags_WidthFixed);
        for (std::size_t i = 0; i < runReadouts_.size(); ++i) {
            RunReadout&       r = runReadouts_[i];
            const ReadoutDef* d = FindReadout(cat, r.topic);
            ImGui::PushID(static_cast<int>(i));
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::TextUnformatted(d ? d->label.c_str() : r.topic.c_str());
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("%s", r.topic.c_str());
            ImGui::TableNextColumn();
            const bool has = i < runReadoutHas_.size() && runReadoutHas_[i];
            if (!has) {
                ImGui::TextDisabled("--");
                if (ImGui::IsItemHovered()) ImGui::SetTooltip("no data on the bus yet");
            } else {
                bool judged = false;
                const bool ok = inBand(r, runReadoutVals_[i], &judged);
                if (judged)
                    ImGui::PushStyleColor(ImGuiCol_Text, ok ? ImVec4(0.30f, 0.85f, 0.40f, 1.0f)
                                                            : ImVec4(1.00f, 0.35f, 0.30f, 1.0f));
                if (d) ImGui::TextUnformatted(FormatReadout(d->kind, runReadoutVals_[i]).c_str());
                else   ImGui::Text("%.4g", runReadoutVals_[i]);
                if (judged) ImGui::PopStyleColor();
            }
            ImGui::TableNextColumn();
            // The limit button: its label says the mode at a glance.
            const char* limLbl = r.mode == "above"   ? ">"
                               : r.mode == "below"   ? "<"
                               : r.mode == "between" ? "><"
                                                     : "lim";
            if (ImGui::SmallButton(limLbl)) ImGui::OpenPopup("runreadlim");
            if (ImGui::IsItemHovered()) {
                if (r.mode == "above")        ImGui::SetTooltip("green when > %.4g", r.lo);
                else if (r.mode == "below")   ImGui::SetTooltip("green when < %.4g", r.hi);
                else if (r.mode == "between") ImGui::SetTooltip("green when %.4g .. %.4g", r.lo, r.hi);
                else                          ImGui::SetTooltip("set a green/red limit");
            }
            if (ImGui::BeginPopup("runreadlim")) {
                ImGui::TextUnformatted(d ? d->label.c_str() : r.topic.c_str());
                int mode = 0;
                for (int m = 0; m < 4; ++m) if (r.mode == kModes[m]) mode = m;
                ImGui::SetNextItemWidth(160.0f);
                if (ImGui::Combo("##mode", &mode, kModeLabels, 4)) {
                    r.mode  = kModes[mode];
                    changed = true;
                }
                if (mode == 1 || mode == 3) {
                    ImGui::SetNextItemWidth(160.0f);
                    ImGui::InputDouble(mode == 3 ? "low" : "min", &r.lo, 0.0, 0.0, "%.4g");
                    if (ImGui::IsItemDeactivatedAfterEdit()) changed = true;
                }
                if (mode == 2 || mode == 3) {
                    ImGui::SetNextItemWidth(160.0f);
                    ImGui::InputDouble(mode == 3 ? "high" : "max", &r.hi, 0.0, 0.0, "%.4g");
                    if (ImGui::IsItemDeactivatedAfterEdit()) changed = true;
                }
                if (mode != 0)
                    ImGui::TextDisabled("in the units shown (%s)",
                                        d ? (d->kind == ReadoutKind::Temp     ? "C"
                                           : d->kind == ReadoutKind::MilliAmp ? "mA"
                                           : d->kind == ReadoutKind::Volt     ? "V"
                                                                              : "A")
                                          : "raw");
                ImGui::EndPopup();
            }
            ImGui::TableNextColumn();
            if (ImGui::SmallButton("x")) remove = static_cast<int>(i);
            ImGui::PopID();
        }
        ImGui::EndTable();
    }
    if (remove >= 0) {
        runReadouts_.erase(runReadouts_.begin() + remove);
        runReadoutAt_ = 0.0;
        changed = true;
    }
    if (changed) SaveUiSettings();
}

// Deck cycle accents. Immutable file-scope constants, not App state, and not
// Theme: the page's other run colours (the gold running row, the red missing
// one) are literals here for the same reason — they say what a RUN did, not how
// the app is styled.
namespace {
const ImVec4 kMarkPassCol(0.28f, 0.78f, 0.38f, 1.0f);   // green: it completed
const ImVec4 kMarkFailCol(0.95f, 0.76f, 0.20f, 1.0f);   // yellow: it did not

// The tick and the cross are DRAWN, not typed. The deck font is a fixed
// monospace and a glyph outside its range would silently render as a box —
// on the one mark an operator reads from three feet away.
void DrawDeckMarkGlyph(ImDrawList* dl, ImVec2 c, float r, ImU32 col, bool passed)
{
    const float t = 2.5f;
    if (passed) {
        dl->AddLine(ImVec2(c.x - r,         c.y + r * 0.10f),
                    ImVec2(c.x - r * 0.25f, c.y + r * 0.75f), col, t);
        dl->AddLine(ImVec2(c.x - r * 0.25f, c.y + r * 0.75f),
                    ImVec2(c.x + r,         c.y - r * 0.75f), col, t);
    } else {
        dl->AddLine(ImVec2(c.x - r * 0.7f, c.y - r * 0.7f),
                    ImVec2(c.x + r * 0.7f, c.y + r * 0.7f), col, t);
        dl->AddLine(ImVec2(c.x + r * 0.7f, c.y - r * 0.7f),
                    ImVec2(c.x - r * 0.7f, c.y + r * 0.7f), col, t);
    }
}
} // namespace

// The deck's node tree, drawn READ-ONLY (recursive: folders hold folders).
// `runningNodeUid` is the node the runner is executing right now — matched by
// UID rather than by name, because a loop runs the same recipe many times over
// and a name match would light every copy of it at once.
void App::DrawDeckNodes(std::vector<ProgramNode>& items, float rowH, bool running,
                        const std::string& runningNodeUid,
                        const std::string& startUid, const std::string& endUid)
{
    const ProgramRunner::Status& rs = orch_.programs.GetStatus();
    // Paints the cycle furniture over a row that has just been submitted: the
    // colour bar down its left edge, the pass/fail glyph and the start/end
    // caption at its right. Reads the ITEM rect, so it must be called while the
    // row is still the current item.
    const auto decorate = [&](const ProgramNode& n) {
        const ImVec2 mn = ImGui::GetItemRectMin(), mx = ImGui::GetItemRectMax();
        ImDrawList* dl  = ImGui::GetWindowDrawList();
        const NodeMark mk = DeckMarkOf(n);
        if (mk != NodeMark::None) {
            const bool passed = (mk == NodeMark::Passed);
            const ImU32 col   = ImGui::GetColorU32(passed ? kMarkPassCol : kMarkFailCol);
            // The bar is the part read from across the bench; the glyph is what
            // you confirm it with once you are standing at the screen.
            //
            // It goes in the DECK'S left gutter, not at the row's own left
            // edge: the rows are indented by the tree, so a per-row bar would
            // stagger and stop being a column the eye can run down — and on a
            // folder row it would land on top of the open/close arrow.
            const float gx = ImGui::GetWindowPos().x + 2.0f;
            dl->AddRectFilled(ImVec2(gx, mn.y + 1.0f),
                              ImVec2(gx + 5.0f, mx.y - 1.0f), col);
            const float r = ImGui::GetTextLineHeight() * 0.34f;
            DrawDeckMarkGlyph(dl, ImVec2(mx.x - 12.0f - r, (mn.y + mx.y) * 0.5f),
                              r, col, passed);
        }
        const char* cap = (!startUid.empty() && n.uid == startUid) ? "start"
                        : (!endUid.empty()   && n.uid == endUid)   ? "end"
                                                                   : nullptr;
        if (cap) {
            const float w = ImGui::CalcTextSize(cap).x;
            dl->AddText(ImVec2(mx.x - 34.0f - w,
                               (mn.y + mx.y) * 0.5f - ImGui::GetTextLineHeight() * 0.5f),
                        ImGui::GetColorU32(ImGuiCol_TextDisabled), cap);
        }
    };
    for (ProgramNode& n : items) {
        ImGui::PushID(n.uid.c_str());
        if (n.IsFolder()) {
            // Folder rows stay ONE line tall while recipe rows are three: the
            // tall touch target exists for the thing an operator picks between
            // parts, and a folder is structure they read past.
            ImGuiTreeNodeFlags flags = ImGuiTreeNodeFlags_OpenOnArrow |
                                       ImGuiTreeNodeFlags_SpanAvailWidth |
                                       ImGuiTreeNodeFlags_DefaultOpen;
            if (deckNode_ == n.uid) flags |= ImGuiTreeNodeFlags_Selected;
            const bool open = (n.loop > 1)
                ? ImGui::TreeNodeEx("##fold", flags, "%s   x%d", n.name.c_str(), n.loop)
                : ImGui::TreeNodeEx("##fold", flags, "%s", n.name.c_str());
            if (ImGui::IsItemClicked() && !ImGui::IsItemToggledOpen()) deckNode_ = n.uid;
            decorate(n);
            // The pass counter belongs to the INNERMOST folder around the
            // running item — the one whose repetitions the operator is
            // counting — so it is drawn only where that item is a direct child.
            if (running && rs.index < rs.items.size() && rs.items[rs.index].passes > 1) {
                bool direct = false;
                for (const ProgramNode& c : n.children)
                    if (c.uid == runningNodeUid) { direct = true; break; }
                if (direct) {
                    ImGui::SameLine();
                    ImGui::TextColored(ImVec4(1.0f, 0.85f, 0.3f, 1.0f), "pass %d/%d",
                                       rs.items[rs.index].pass, rs.items[rs.index].passes);
                }
            }
            if (open) {
                // Captions do not descend: only a top-level node can be a
                // cycle state, so a nested uid can never legitimately match.
                DrawDeckNodes(n.children, rowH, running, runningNodeUid,
                              std::string(), std::string());
                if (n.children.empty()) ImGui::TextDisabled("  (empty folder)");
                ImGui::TreePop();
            }
            ImGui::PopID();
            continue;
        }

        const Recipe* r = RecipeByUid(session_.Recipes(), n.recipeUid);
        const bool missing = (r == nullptr);
        const bool sel     = deckNode_ == n.uid;
        const bool cur     = running && n.uid == runningNodeUid;
        if (cur)          ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.85f, 0.3f, 1.0f));
        else if (missing) ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.4f, 0.3f, 1.0f));
        const std::string label = missing ? (n.recipeUid + " (missing!)") : r->name;
        // The row is kDeckRowScale text-lines tall and carries an EMPTY label:
        // a Selectable draws its text at the top-left of its box, which in a
        // tall row leaves the name floating above the band it names. The label
        // is drawn separately, centred in the box, at the same x the Selectable
        // would have used. An empty label plus the enclosing PushID(uid) also
        // means renaming a recipe no longer changes the row's ID.
        const bool picked = ImGui::Selectable("##rcp", sel, ImGuiSelectableFlags_AllowOverlap,
                                              ImVec2(0.0f, rowH));
        // Every IsItem* below must read the SELECTABLE, so capture them before
        // anything else becomes the current item.
        const bool rowRClick = ImGui::IsItemClicked(ImGuiMouseButton_Right);
        const bool rowHover  = ImGui::IsItemHovered();
        const ImVec2 rowMin  = ImGui::GetItemRectMin();
        ImGui::GetWindowDrawList()->AddText(
            ImVec2(rowMin.x, rowMin.y + (rowH - ImGui::GetTextLineHeight()) * 0.5f),
            ImGui::GetColorU32(ImGuiCol_Text), label.c_str());
        decorate(n);
        if (picked) deckNode_ = n.uid;
        // Right-click shortcut: open this recipe in the Recipe Editor. Level
        // checked here rather than left to SwitchPage's refusal, because the
        // shortcut re-points prevPage_ and the editor's selection BEFORE it
        // switches — an operator's right-click would rearrange editor state it
        // is never going to see, and the tooltip would advertise a door that
        // answers with a warning.
        if (!missing && rowRClick && CanAccess(Page::RecipeEdit)) {
            deckNode_ = n.uid;
            for (int k = 0; k < static_cast<int>(session_.Recipes().size()); ++k)
                if (session_.Recipes()[k].uid == n.recipeUid) { recipeSel_ = k; break; }
            ClearStepSel();
            // Takeover -> takeover leaves prevPage_ alone, so the return page is
            // named here; the switch itself still routes through SwitchPage like
            // every other one.
            prevPage_ = Page::Run;   // editor Close comes back here
            SwitchPage(Page::RecipeEdit);
        }
        if (!missing && rowHover && CanAccess(Page::RecipeEdit))
            ImGui::SetTooltip("Right-click: open in Recipe Editor");
        if (cur || missing) ImGui::PopStyleColor();
        ImGui::PopID();
    }
}

void App::DrawRunPanel()
{
    const ProgramRunner::Status& rs = orch_.programs.GetStatus();
    const RecipeEngine::Status   es = orch_.recipe.GetStatus();
    const bool running = orch_.programs.Active();

    const ImGuiStyle& style  = ImGui::GetStyle();
    // The run strip is the operator's one reach-for-it row, so its controls are
    // twice frame height — a glove finds Execute/Stop without aiming.
    const float  stripBtnH = ImGui::GetFrameHeight() * 2.0f;
    const float  stripH = stripBtnH + style.WindowPadding.y * 2.0f +
                          style.ChildBorderSize * 2.0f;
    const ImVec2 avail  = ImGui::GetContentRegionAvail();
    const float  paneH  = avail.y - stripH - style.ItemSpacing.y;
    // Deck | execution | DUT, the two side columns dragged by the splitters
    // below and persisted (runDeckW_/runDutW_). Clamped every frame so a window
    // resize can never leave the execution view narrower than it can use.
    constexpr float kSplitW  = 6.0f;
    constexpr float kMinSide = 180.0f;
    constexpr float kMinMidW = 260.0f;
    const auto clampf = [](float v, float lo, float hi) {
        if (hi < lo) hi = lo;
        return v < lo ? lo : (v > hi ? hi : v);
    };
    runDeckW_ = clampf(runDeckW_, kMinSide,
                       avail.x - runDutW_ - 2.0f * kSplitW - kMinMidW);
    runDutW_  = clampf(runDutW_,  kMinSide,
                       avail.x - runDeckW_ - 2.0f * kSplitW - kMinMidW);
    const float  leftW  = runDeckW_;
    const float  rightW = runDutW_;

    const bool haveProg = progSel_ >= 0 && progSel_ < static_cast<int>(session_.Programs().size());

    // ---- Top strip: run controls + which program is loaded ----
    ImGui::BeginChild("runStrip", ImVec2(0.0f, stripH), ImGuiChildFlags_Borders);
    const bool canRun = !orch_.Busy() && haveProg && deckSlot_ >= 0;
    // Everything shorter than the double-height buttons is centred against
    // them, or the row reads as a caption stuck to the ceiling.
    const auto centerNext = [stripBtnH](float itemH) {
        ImGui::SetCursorPosY(ImGui::GetCursorPosY() + (stripBtnH - itemH) * 0.5f);
    };
    // One Execute button, two granularities: the selected recipe, or every
    // recipe in the ONE slot the deck is showing. There is deliberately no
    // whole-program scope — a multi-slot program is a station sequence an
    // operator walks, not something to launch in one press.
    // While a run is active the button becomes Stop.
    //
    // GREEN means the machine is yours to start, RED means it is moving. The
    // colour is the part read from across the bench — the label is only
    // legible once you are standing at the screen, and by then you already
    // know. Disabled-green (nothing selected, or another engine is busy) dims
    // to a grey-green through BeginDisabled, so "not ready" never looks like
    // "go". The pair is deliberately the only saturated colour in the strip.
    const bool onNode = deckSlot_ >= 0 && !deckNode_.empty();
    const ImVec4 goCol(0.16f, 0.52f, 0.24f, 1.0f), goHov(0.22f, 0.68f, 0.32f, 1.0f),
                 goAct(0.11f, 0.40f, 0.18f, 1.0f);
    const ImVec4 stCol(0.60f, 0.17f, 0.15f, 1.0f), stHov(0.78f, 0.24f, 0.21f, 1.0f),
                 stAct(0.45f, 0.12f, 0.11f, 1.0f);
    ImGui::PushStyleColor(ImGuiCol_Button,        running ? stCol : goCol);
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, running ? stHov : goHov);
    ImGui::PushStyleColor(ImGuiCol_ButtonActive,  running ? stAct : goAct);
    if (running) {
        if (IconButton(UiIcon("Stop"), "Stop", ImVec2(260.0f, stripBtnH)))
            orch_.programs.Stop();
    } else {
        ImGui::BeginDisabled(!canRun);
        // In teach mode the go button says so itself: the press that follows
        // teaches positions instead of running the part.
        if (IconButton(UiIcon("run_execute"), teachMode_ ? "Teach run" : "Execute",
                       ImVec2(260.0f, stripBtnH)))
            StartDeckRun(onNode ? RunScope::Node : RunScope::Slot);
        if (ImGui::IsItemHovered()) {
            if (teachMode_)
                ImGui::SetTooltip("TEACH: run only the moves to saved positions,\n"
                                  "and after each one fine-tune and Save that position.");
            else
                ImGui::SetTooltip(onNode ? "Run the selected item (a folder runs its loops)"
                                         : "Run every recipe in this slot");
        }
        ImGui::EndDisabled();
    }
    ImGui::PopStyleColor(3);
    ImGui::SameLine();
    centerNext(ImGui::GetTextLineHeight());
    ImGui::TextDisabled(onNode ? "item" : "slot");
    // The program is LOADED, not picked from a combo: which program is up is a
    // deliberate operator act, and the name is the answer to "what am I
    // running?" — the reason it sits in the run strip and not down in the
    // deck. Edit mode joins it for the same reason.
    ImGui::SameLine();
    centerNext(ImGui::GetTextLineHeight());
    if (haveProg) ImGui::Text("%s", session_.Programs()[progSel_].name.c_str());
    else          ImGui::TextDisabled("(no program)");
    // Edit no longer grows buttons in the deck: it opens the Program Organizer
    // window. The checkbox mirrors that window's own close box, so the two can
    // never disagree about whether it is up.
    //
    // Absent below Technician: the Run page is the one surface an Operator has,
    // and this checkbox is a door out of it into program editing. Gated HERE as
    // well as at the window, because the strip is the only way in that does not
    // go through the View menu.
    if (CanUse(kRoleOrg)) {
        ImGui::SameLine();
        centerNext(ImGui::GetFrameHeight());
        ImGui::BeginDisabled(running);
        if (ImGui::Checkbox("Edit", &runEdit_)) {
            orgOpen_ = runEdit_;
            // Opening it lands on the slot the deck is showing: the operator
            // asked to edit THIS station, not whichever one was last
            // reorganised.
            if (orgOpen_) { orgSlot_ = deckSlot_; orgSel_.clear(); }
        }
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("Open the Program Organizer");
        ImGui::EndDisabled();
    }
    // Teach mode toggle. Gated like re-teaching a position anywhere else (the
    // Motion page's level): it writes positions, not programs. Disabled while
    // anything runs, so a queue can never change mode between its items — the
    // runner carries its own copy of the flag regardless.
    if (CanAccess(Page::Motion)) {
        ImGui::SameLine();
        centerNext(ImGui::GetFrameHeight());
        ImGui::BeginDisabled(orch_.Busy());
        // The push/pop pair keys off the value BEFORE the click: the button
        // flips the flag, and a pop decided on the new value pops colours that
        // were never pushed (ImGui asserts, the process aborts).
        const bool wasOn = teachMode_;
        if (wasOn) {
            ImGui::PushStyleColor(ImGuiCol_Button,        kTeachOrange);
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, kTeachOrangeHov);
            ImGui::PushStyleColor(ImGuiCol_ButtonActive,  kTeachOrangeAct);
        }
        if (ImGui::Button(wasOn ? "TEACH ON" : "Teach")) {
            teachMode_ = !teachMode_;
            FD_INFO("program", "teach mode %s", teachMode_ ? "ON" : "off");
        }
        if (wasOn) ImGui::PopStyleColor(3);
        if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
            ImGui::SetTooltip("Teach mode: a run executes ONLY its moves to saved positions\n"
                              "(move_to_position and variants, move_path); everything else is\n"
                              "skipped. After each move, jog onto the exact spot and Save to\n"
                              "write it into that position, or Skip to keep it. Loops run once,\n"
                              "both branches of an if are walked. No deck marks, no DUT record.\n"
                              "Applies to every Run/Execute while on; off again at sign-out.");
        ImGui::EndDisabled();
    } else if (teachMode_) {
        // Cannot happen after LogOut clears it, but never show a hidden mode.
        teachMode_ = false;
    }
    if (teachMode_ || rs.teach) {
        ImGui::SameLine();
        centerNext(ImGui::GetTextLineHeight());
        ImGui::TextColored(kTeachOrange, rs.active && rs.teach ? "TEACH RUN" : "TEACH");
    }
    {
        const DutSession::Info di = DutSession::Instance().GetInfo();
        ImGui::SameLine();
        centerNext(ImGui::GetTextLineHeight());
        ImGui::TextDisabled("DUT: %s", di.open ? di.serial.c_str() : "none");
    }
    if (!runStartErr_.empty()) {
        ImGui::SameLine();
        centerNext(ImGui::GetTextLineHeight());
        ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.3f, 1.0f), "%s", runStartErr_.c_str());
    }
    // Save/Load are document chores, not run controls — parked at the far right
    // so nothing sits between Execute and the operator's eye.
    // Same height as Execute: one row, one reach, one size of target. Load is
    // wider than before because it carries an icon now (assets/recipe_icons/
    // load.png; IconButton scales it to text height whatever the file's size).
    constexpr float kSaveW = 110.0f;
    constexpr float kLoadW = 100.0f;
    // Load stays with the Operator — WHICH job is in the deck is the operator's
    // call, and it mutates nothing. Save writes the programs document, so it
    // goes with the level that may edit programs at all; below that the deck is
    // read-only and there is nothing a Save could legitimately persist.
    const bool canSave = CanUse(kRoleOrg);
    const float rightX = ImGui::GetWindowContentRegionMax().x -
                         (canSave ? kSaveW + kLoadW + style.ItemSpacing.x : kLoadW);
    ImGui::SameLine();
    // Never step BACK onto what is already drawn: a long start error would
    // otherwise be overprinted by the buttons.
    if (rightX > ImGui::GetCursorPosX()) ImGui::SetCursorPosX(rightX);
    ImGui::BeginDisabled(running);
    if (canSave) {
        if (IconButton(UiIcon("Save"), session_.ProgramsDirty() ? "Save*" : "Save",
                       ImVec2(kSaveW, stripBtnH)))
            session_.SavePrograms();
        ImGui::SameLine();
    }
    if (IconButton(UiIcon("load"), "Load", ImVec2(kLoadW, stripBtnH))) runOpenLoad_ = true;
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("Load a program into the deck");
    ImGui::EndDisabled();
    ImGui::EndChild();

    // ---- Left: the deck ----
    ImGui::BeginChild("runDeck", ImVec2(leftW, paneH), ImGuiChildFlags_Borders);
    // The DUT first: which part is on the machine is the thing an operator
    // reads before anything else, so it heads the deck at 4x the body size
    // (shrunk to fit the column). Outside the running-disable on purpose -
    // Pass / Fail / Close behave exactly as they did in the right column.
    DrawDutBlock(4.0f);
    ImGui::Separator();
    ImGui::BeginDisabled(running);
    // Active slot: the deck shows exactly ONE slot, so this combo IS the view,
    // and it is also what Execute runs. It survives a restart (UiSettings
    // run_prog_uid / run_slot) — an operator working one station comes back to
    // that station, not to the top of the program.
    {
        Program* prog = haveProg ? &session_.Programs()[progSel_] : nullptr;
        const int nSlots = prog ? static_cast<int>(prog->slots.size()) : 0;
        // Clamp both ways every frame: -1 is legal ONLY while the program has
        // no slots, so a stale index (slot deleted) or a fresh slot on an empty
        // program can never leave the deck showing nothing it could show.
        if (nSlots == 0)              deckSlot_ = -1;
        else if (deckSlot_ < 0)       deckSlot_ = 0;
        else if (deckSlot_ >= nSlots) deckSlot_ = nSlots - 1;
        char prev[64];
        if (prog && deckSlot_ >= 0)
            std::snprintf(prev, sizeof(prev), "%d  %s", deckSlot_ + 1,
                          prog->slots[deckSlot_].name.c_str());
        else
            std::snprintf(prev, sizeof(prev), "(no slots)");
        ImGui::BeginDisabled(!haveProg);
        ImGui::SetNextItemWidth(160.0f);
        if (ImGui::BeginCombo("##actslot", prev)) {
            for (int i = 0; i < nSlots; ++i) {
                ImGui::PushID(i);
                char lbl[64];
                std::snprintf(lbl, sizeof(lbl), "%d  %s", i + 1,
                              prog->slots[i].name.c_str());
                if (ImGui::Selectable(lbl, deckSlot_ == i)) SelectDeckSlot(i);
                ImGui::PopID();
            }
            ImGui::EndCombo();
        }
        ImGui::SameLine();
        ImGui::TextDisabled("slot");
        ImGui::EndDisabled();
    }
    ImGui::EndDisabled();
    ImGui::Separator();

    // The deck is READ-ONLY. Every edit — add, delete, move, folder, loop
    // count — lives in the Program Organizer window, because the two are
    // different jobs: the deck is what an operator reads while running one
    // station, and an editing control in it is a mis-click away from changing
    // the program under a part. The Edit checkbox in the strip opens the
    // organizer; it no longer grows buttons here.
    //
    // Rows are 2.25 text-lines tall. This is the list an operator picks from
    // between parts, often with a glove on: the row is a touch target first and
    // a line of text second. It scales off the text line rather than a fixed
    // pixel count so the font-size picker keeps working — and yes, fewer rows
    // fit at once; the deck scrolls, and the whole point of showing ONE slot is
    // that the list is short.
    constexpr float kDeckRowScale = 2.25f;
    const float deckRowH = ImGui::GetTextLineHeightWithSpacing() * kDeckRowScale;

    if (!haveProg) {
        ImGui::TextWrapped(session_.Programs().empty()
                               ? "No programs yet - Edit creates one."
                               : "Load a program above.");
    } else if (deckSlot_ >= 0) {
        Program& prog = session_.Programs()[progSel_];
        ProgramSlot& slot = prog.slots[static_cast<std::size_t>(deckSlot_)];
        // Slot row: clicking it is how you get BACK to slot scope after picking
        // a node. Only the ACTIVE slot is ever drawn — the combo above IS the
        // view selector.
        {
            ImGuiTreeNodeFlags flags = ImGuiTreeNodeFlags_SpanAvailWidth |
                                       ImGuiTreeNodeFlags_DefaultOpen |
                                       ImGuiTreeNodeFlags_OpenOnArrow;
            if (deckNode_.empty()) flags |= ImGuiTreeNodeFlags_Selected;
            const bool open = ImGui::TreeNodeEx("##slot", flags, "%d  %s",
                                                deckSlot_ + 1, slot.name.c_str());
            if (ImGui::IsItemClicked() && !ImGui::IsItemToggledOpen()) deckNode_.clear();
            if (open) {
                // The node the runner is on right now — matched by UID, not by
                // name: a loop runs the same recipe many times and a name match
                // would light every copy of it at once.
                std::string runningUid;
                if (running && rs.index < rs.items.size())
                    runningUid = rs.items[rs.index].nodeUid;
                // The captions arrive empty unless the cycle RESOLVES, so an
                // invalid one cannot label a row it no longer describes.
                std::string cycStart, cycEnd;
                if (DeckCycleRange(slot, nullptr, nullptr)) {
                    cycStart = slot.startNodeUid;
                    cycEnd   = slot.endNodeUid;
                }
                DrawDeckNodes(slot.items, deckRowH, running, runningUid,
                              cycStart, cycEnd);
                if (slot.items.empty())
                    ImGui::TextDisabled("  (empty - tick Edit to fill it)");
                ImGui::TreePop();
            }
        }
    }
    ImGui::EndChild();

    // Drag either splitter to re-proportion the row; both persist on the drag
    // END, like every other splitter in the app.
    bool splitting = false;
    ImGui::SameLine(0.0f, 0.0f);
    if (Splitter("##runsplitL", true, kSplitW, paneH)) {
        runDeckW_ += ImGui::GetIO().MouseDelta.x;
        splitting = true;
    }
    ImGui::SameLine(0.0f, 0.0f);

    // ---- Middle column: execution view / Scan band / log ----
    const float midW = avail.x - leftW - rightW - 2.0f * kSplitW;
    ImGui::BeginChild("runMid", ImVec2(midW, paneH));
    // The Scan band holds three tabs of parameter forms now, so its height is
    // the operator's to set (dragged by the splitter above it, persisted in
    // runScanH_) rather than a fixed row count that silently crops a tab. The
    // default is generous for the same reason; the band scrolls if it is ever
    // dragged smaller than its content.
    const float rowH = ImGui::GetFrameHeightWithSpacing();
    const float scanDefaultH = rowH * 8.0f + style.WindowPadding.y * 2.0f;
    if (runScanH_ <= 0.0f) runScanH_ = scanDefaultH;
    constexpr float kSplitH  = 6.0f;
    constexpr float kMinExec = 60.0f;
    const float logH  = 190.0f;
    const float midAvailH = ImGui::GetContentRegionAvail().y;
    // The quick-align band is FIXED height and sits outside runExec: it is the
    // operator's five one-press launches, so a long queue must never be able to
    // scroll them out of reach. Caption row + two button rows + child padding.
    const float quickBtnH = ImGui::GetFrameHeight() * 1.8f;
    const float quickH    = ImGui::GetTextLineHeightWithSpacing() +
                            quickBtnH * 2.0f + style.ItemSpacing.y +
                            style.WindowPadding.y * 2.0f +
                            style.ChildBorderSize * 2.0f;
    // runExec + quick + splitter + runScan + runLog, four spacings between them.
    const float fixedH = logH + kSplitH + quickH + style.ItemSpacing.y * 4.0f;
    const float scanH  = clampf(runScanH_, rowH * 3.0f,
                                midAvailH - fixedH - kMinExec);
    runScanH_ = scanH;
    float execH = midAvailH - scanH - fixedH;
    if (execH < kMinExec) execH = kMinExec;
    ImGui::BeginChild("runExec", ImVec2(0.0f, execH), ImGuiChildFlags_Borders);
    if (running || !rs.result.empty()) {
        ImGui::Text("%s: %s", rs.kind.c_str(), rs.label.c_str());
        if (running) {
            ImGui::Text("recipe %zu / %zu", rs.index + 1, rs.total);
            if (!es.currentStep.empty()) ImGui::TextDisabled("%s", es.currentStep.c_str());
            if (es.leafTotal > 0) {
                float frac = static_cast<float>(es.leafDone) /
                             static_cast<float>(es.leafTotal);
                if (frac > 1.0f) frac = 1.0f;
                ImGui::ProgressBar(frac, ImVec2(-1.0f, 0.0f));
            }
        } else {
            const bool ok = rs.result.rfind("Completed", 0) == 0;
            ImGui::TextColored(ok ? ImVec4(0.4f, 1.0f, 0.5f, 1.0f)
                                  : ImVec4(1.0f, 0.4f, 0.3f, 1.0f),
                               "%s", rs.result.c_str());
        }
        ImGui::Separator();
        // A loop expands into the queue, so this list is as long as the RUN is
        // — thousands of rows for a x100 folder. Clip it (only the visible rows
        // cost anything) and keep the running row in view, or the one line the
        // operator wants scrolls away the moment the queue gets interesting.
        ImGuiListClipper clip;
        clip.Begin(static_cast<int>(rs.items.size()));
        // The running row must be SUBMITTED even when it is scrolled out, or
        // there is nothing to scroll to — the clipper would simply skip it.
        if (running && rs.index < rs.items.size())
            clip.IncludeItemByIndex(static_cast<int>(rs.index));
        while (clip.Step()) {
            for (int i = clip.DisplayStart; i < clip.DisplayEnd; ++i) {
                const ProgramRunner::ItemStatus& it = rs.items[static_cast<std::size_t>(i)];
                const char* mark = "  ";
                ImVec4 col = ImGui::GetStyle().Colors[ImGuiCol_TextDisabled];
                switch (it.mark) {
                case ProgramRunner::Mark::Running: mark = "> "; col = ImVec4(1.0f, 0.85f, 0.3f, 1.0f); break;
                case ProgramRunner::Mark::Done:    mark = "OK"; col = ImVec4(0.4f, 1.0f, 0.5f, 1.0f); break;
                case ProgramRunner::Mark::Failed:  mark = "X "; col = ImVec4(1.0f, 0.4f, 0.3f, 1.0f); break;
                case ProgramRunner::Mark::Stopped: mark = "! "; col = ImVec4(1.0f, 0.7f, 0.3f, 1.0f); break;
                case ProgramRunner::Mark::Pending: break;
                }
                std::string where = it.slotName;
                if (!it.path.empty()) where += " / " + it.path;
                if (it.passes > 1) {
                    char pass[32];
                    std::snprintf(pass, sizeof(pass), " [%d/%d]", it.pass, it.passes);
                    where += pass;
                }
                ImGui::TextColored(col, "%s  %s / %s", mark, where.c_str(),
                                   it.recipeName.c_str());
                // Keep it in view, but only when it has actually left: an
                // unconditional SetScrollHereY fights the operator every frame
                // they try to scroll back and read what already ran.
                if (running && i == static_cast<int>(rs.index)) {
                    const float y   = ImGui::GetItemRectMin().y;
                    const float top = ImGui::GetWindowPos().y;
                    const float bot = top + ImGui::GetWindowSize().y -
                                      ImGui::GetTextLineHeightWithSpacing();
                    if (y < top || y > bot) ImGui::SetScrollHereY(0.5f);
                }
            }
        }
        clip.End();
    } else {
        ImGui::TextDisabled("Pick the slot above (or an item in it), then Execute.");
        if (haveProg && !session_.Programs()[progSel_].description.empty()) {
            ImGui::Separator();
            ImGui::TextWrapped("%s", session_.Programs()[progSel_].description.c_str());
        }
    }
    ImGui::EndChild();

    // Quick-align band: the five presses this product's operators actually
    // make, each one a target slot + a step ladder + a hexapod in one click.
    ImGui::BeginChild("runQuick", ImVec2(0.0f, quickH), ImGuiChildFlags_Borders);
    DrawRunQuickAlign();
    ImGui::EndChild();

    // Drag to give the scan forms more (or less) room; persists on the drag END
    // with the two column splitters.
    if (Splitter("##runsplitS", false, kSplitH, ImGui::GetContentRegionAvail().x)) {
        runScanH_ -= ImGui::GetIO().MouseDelta.y;
        splitting = true;
    }

    // Scan band: shared channel/Trace header over Raster | Climb | XCAN tabs.
    ImGui::BeginChild("runScan", ImVec2(0.0f, scanH), ImGuiChildFlags_Borders);
    DrawRunScanBlock();
    ImGui::EndChild();

    // Log band (same viewer as the Overview lower panel).
    ImGui::BeginChild("runLog", ImVec2(0.0f, 0.0f), ImGuiChildFlags_Borders);
    DrawLogPanel();
    ImGui::EndChild();

    ImGui::EndChild();   // runMid

    ImGui::SameLine(0.0f, 0.0f);
    if (Splitter("##runsplitR", true, kSplitW, paneH)) {
        runDutW_ -= ImGui::GetIO().MouseDelta.x;
        splitting = true;
    }
    ImGui::SameLine(0.0f, 0.0f);

    // ---- Right: the picture, then DUT (the operator flow is look -> scan ->
    // run -> pass/fail, and the panel is stacked in that order) ----
    ImGui::BeginChild("runDut", ImVec2(rightW, paneH), ImGuiChildFlags_Borders);
    // Seeded from a proportion on the first frame and clamped every frame, so
    // a saved height from a bigger monitor cannot swallow the DUT block below.
    // The floor leaves room for the header and footer lines; the ceiling leaves
    // the DUT block its serial row.
    {
        const float rowH = ImGui::GetTextLineHeightWithSpacing();
        if (runShotPanelH_ <= 0.0f) runShotPanelH_ = paneH * 0.34f;
        runShotPanelH_ = clampf(runShotPanelH_, rowH * 4.0f, paneH - rowH * 8.0f);
    }
    DrawRunShotPanel();
    if (Splitter("##runsplitShot", false, kSplitH, ImGui::GetContentRegionAvail().x)) {
        runShotPanelH_ += ImGui::GetIO().MouseDelta.y;
        splitting = true;
    }
    // (The DUT block moved to the top of the deck, left column.)
    // The engine line is pinned to the BOTTOM and the values scroll above it:
    // "what is the engine doing" is glanced at, and a long value list must not
    // push it off the panel.
    // The pinned instrument readouts sit between the two, at a fixed height
    // for their row count, so they neither scroll away with the values nor
    // push the engine line.
    {
        const float lineH = ImGui::GetTextLineHeightWithSpacing();
        const float engH  = lineH * 2.0f + ImGui::GetStyle().ItemSpacing.y;
        const float rows  = static_cast<float>((std::max)(runReadouts_.size(), std::size_t{1}));
        const float readH = ImGui::GetFrameHeightWithSpacing() + lineH * rows +
                            ImGui::GetStyle().ItemSpacing.y * 2.0f;
        const float valsH = ImGui::GetContentRegionAvail().y - engH - readH;
        ImGui::BeginChild("runVals", ImVec2(0.0f, valsH > 0.0f ? valsH : 0.0f));
        DrawRunValuesPanel();
        ImGui::EndChild();
        DrawRunReadouts();
    }
    ImGui::Separator();
    ImGui::Text("Engine: %s", RecipeEngine::StateName(es.state));
    if (!es.error.empty() && es.state != RecipeEngine::State::Running)
        ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.3f, 1.0f), "%s", es.error.c_str());
    ImGui::EndChild();

    if (runSplitting_ && !splitting) SaveUiSettings();
    runSplitting_ = splitting;

    // ---- Load Program ----
    // A modal list rather than a combo in the deck: loading a program is a
    // deliberate act (it re-points the whole page), and the list has room to
    // show the slot count, which is what tells two similar programs apart.
    if (runOpenLoad_) {
        ImGui::OpenPopup("Load Program");
        runOpenLoad_ = false;
    }
    ImGui::SetNextWindowPos(ImGui::GetMainViewport()->GetCenter(),
                            ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
    if (ImGui::BeginPopupModal("Load Program", nullptr,
                               ImGuiWindowFlags_AlwaysAutoResize)) {
        if (session_.Programs().empty())
            ImGui::TextDisabled("No programs yet - tick Edit, then New.");
        for (int i = 0; i < static_cast<int>(session_.Programs().size()); ++i) {
            const Program& p = session_.Programs()[i];
            ImGui::PushID(i);
            char lbl[128];
            std::snprintf(lbl, sizeof(lbl), "%s   (%d slots)", p.name.c_str(),
                          static_cast<int>(p.slots.size()));
            if (ImGui::Selectable(lbl, progSel_ == i, 0, ImVec2(280.0f, 0.0f))) {
                SelectDeckProgram(i);
                runStartErr_.clear();
                ImGui::CloseCurrentPopup();
            }
            ImGui::PopID();
        }
        ImGui::Separator();
        if (ImGui::Button("Cancel", ImVec2(100.0f, 0.0f))) ImGui::CloseCurrentPopup();
        ImGui::EndPopup();
    }

}

void App::LoadScanPresets()
{
    for (int i = 0; i < kXcanPresetCount; ++i) {
        xcanPresets_[i] = DefaultXcanPreset(i);
        xcanTargets_[i] = DefaultXcanTarget(i);
    }
    for (int i = 0; i < kClimbPresetCount; ++i)
        climbPresets_[i] = DefaultClimbPreset(i);
    std::ifstream f("config/scan_presets.local.json");
    if (!f.is_open()) f.open("config/scan_presets.json");
    if (!f.is_open()) return;   // first run — defaults stand
    const nlohmann::json j = nlohmann::json::parse(f, nullptr, false);
    if (!j.is_object()) return;
    if (j.contains("xcan_presets") && j["xcan_presets"].is_array()) {
        const nlohmann::json& arr = j["xcan_presets"];
        for (std::size_t i = 0;
             i < arr.size() && i < static_cast<std::size_t>(kXcanPresetCount); ++i) {
            if (arr[i].is_object())
                XcanPresetFromJson(arr[i], xcanPresets_[static_cast<int>(i)]);
        }
    }
    if (j.contains("xcan_targets") && j["xcan_targets"].is_array()) {
        const nlohmann::json& arr = j["xcan_targets"];
        for (std::size_t i = 0;
             i < arr.size() && i < static_cast<std::size_t>(kXcanPresetCount); ++i) {
            if (!arr[i].is_object()) continue;
            XcanTarget& t = xcanTargets_[static_cast<int>(i)];
            t.enabled    = arr[i].value("enabled", t.enabled);
            t.targetUa   = arr[i].value("target_ua", t.targetUa);
            t.zoneLowUa  = arr[i].value("zone_low_ua", t.zoneLowUa);
            t.zoneHighUa = arr[i].value("zone_high_ua", t.zoneHighUa);
        }
    }
    if (j.contains("climb_presets") && j["climb_presets"].is_array()) {
        const nlohmann::json& arr = j["climb_presets"];
        for (std::size_t i = 0;
             i < arr.size() && i < static_cast<std::size_t>(kClimbPresetCount); ++i) {
            if (arr[i].is_object())
                ClimbPresetFromJson(arr[i], climbPresets_[static_cast<int>(i)]);
        }
    }
}

void App::SaveScanPresets() const
{
    nlohmann::json arr = nlohmann::json::array();
    for (const XcanPreset& p : xcanPresets_) arr.push_back(XcanPresetToJson(p));
    nlohmann::json tarr = nlohmann::json::array();
    for (const XcanTarget& t : xcanTargets_) {
        tarr.push_back(nlohmann::json{
            {"enabled", t.enabled},
            {"target_ua", t.targetUa},
            {"zone_low_ua", t.zoneLowUa},
            {"zone_high_ua", t.zoneHighUa},
        });
    }
    nlohmann::json carr = nlohmann::json::array();
    for (const ClimbPreset& p : climbPresets_) carr.push_back(ClimbPresetToJson(p));
    nlohmann::json j;
    j["xcan_presets"]  = std::move(arr);
    j["xcan_targets"]  = std::move(tarr);
    j["climb_presets"] = std::move(carr);
    std::ofstream f("config/scan_presets.local.json");
    if (f.is_open()) {
        f << j.dump(2) << "\n";
        FD_INFO("ui", "scan presets saved to config/scan_presets.local.json");
    } else {
        FD_WARN("ui", "could not save config/scan_presets.local.json");
    }
}

// XCAN preset launcher (bobcat Run page scan controls): hexapod_align.lua with
// the pressed launch preset (step ladder + tunables, right-click to edit) +
// the ACTIVE target preset (exit threshold + adaptive zone range). Uses the
// same product-script override and Env wiring as the Scanning tab.
void App::StartXcanPreset(const std::string& device, int presetIdx)
{
    std::string script = "config/scripts/hexapod_align.lua";
    for (const ScriptInfo& si : session_.Scripts()) {
        if (si.name == "hexapod_align") { script = si.path; break; }
    }
    const XcanPreset& p  = xcanPresets_[presetIdx];
    const XcanTarget& tp = xcanTargets_[xcanPresetSel_];
    LuaEngine::Env env = session_.MakeLuaEnv();
    env.initialCtx = {
        {"align_device", device},
        {"align_topic", std::string(runXcanTopic_)},
        {"align_steps_um", p.stepsUm},
        {"align_axes", p.axes},
        {"align_settle_ms", p.settleMs},
        {"align_settle_min_ms", p.settleMinMs},
        {"align_fresh_samples", static_cast<double>(p.freshSamples)},
        {"align_avg_count", static_cast<double>(p.avgCount)},
        {"align_consec_limit", static_cast<double>(p.consecLimit)},
        {"align_sig_drop_pct", p.sigDropPct},
        {"align_max_travel_um", p.maxTravelUm},
        {"align_improve_pct", p.improvePct},
        {"align_min_signal", p.minSignal},
        {"align_tol_mm", p.tolMm},
        {"align_move_timeout_ms", p.moveTimeoutMs},
        {"align_velocity_mm_s", p.velocityMmS},
        {"align_skip_opposite", p.skipOpposite ? 1.0 : 0.0},
        {"align_dir_memory", p.dirMemory ? 1.0 : 0.0},
        {"align_reuse_sample", p.reuseSample ? 1.0 : 0.0},
        {"align_trace", xcanTrace_ ? 1.0 : 0.0},
        // MAX: no threshold (0 = off in the script) and the plain ladder — the
        // adaptive zones are defined off a target this run does not have.
        {"align_exit_threshold", xcanTargetMax_ ? 0.0 : tp.targetUa * 1e-6},
        {"align_adaptive", xcanTargetMax_ ? 0.0 : 1.0},
        {"align_zone_low", tp.zoneLowUa * 1e-6},
        {"align_zone_high", tp.zoneHighUa * 1e-6},
    };
    if (xcanTargetMax_)
        FD_INFO("script", "xcan preset '%s' on %s: MAX (no exit threshold)", p.label.c_str(),
                device.c_str());
    scriptStartErr_.clear();
    if (!orch_.lua.Start(script, std::move(env), &scriptStartErr_))
        FD_WARN("script", "xcan preset '%s' rejected: %s", p.label.c_str(),
                scriptStartErr_.c_str());
}

// Climb preset launcher: climb_raster.lua with the pressed launch preset (step
// sizes + plane shape + guards, right-click to edit) and the ACTIVE XCAN target
// slot — the climb drives at the same product power class the XCAN pass does,
// so the target lives in one place; only the exit FRACTION is the preset's.
void App::StartClimbPreset(const std::string& device, int presetIdx)
{
    std::string script = "config/scripts/climb_raster.lua";
    for (const ScriptInfo& si : session_.Scripts()) {
        if (si.name == "climb_raster") { script = si.path; break; }
    }
    const ClimbPreset& p  = climbPresets_[presetIdx];
    const XcanTarget&  tp = xcanTargets_[xcanPresetSel_];
    LuaEngine::Env env = session_.MakeLuaEnv();
    env.initialCtx = {
        {"climb_device", device},
        {"climb_topic", std::string(runXcanTopic_)},
        {"climb_target", tp.targetUa * 1e-6},
        {"climb_target_pct", p.targetPct},
        {"climb_z_axis", p.zAxis},
        {"climb_plane_axes", p.planeAxes},
        {"climb_z_step_um", p.zStepUm},
        {"climb_max_z_um", p.maxZUm},
        {"climb_z_probe_pct", p.zProbePct},
        {"climb_max_skips", static_cast<double>(p.maxSkips)},
        {"climb_plane_scan", std::string(p.spiral ? "spiral" : "cross")},
        {"climb_xy_step_um", p.xyStepUm},
        {"climb_xy_steps", static_cast<double>(p.xySteps)},
        {"climb_peak_drop", static_cast<double>(p.peakDrop)},
        {"climb_rings_stop", static_cast<double>(p.ringsStop)},
        {"climb_improve_pct", p.improvePct},
        {"climb_contact_drop_pct", p.contactDropPct},
        {"climb_stall_planes", static_cast<double>(p.stallPlanes)},
        {"climb_settle_ms", p.settleMs},
        {"climb_settle_min_ms", p.settleMinMs},
        {"climb_z_settle_ms", p.zSettleMs},
        {"climb_fresh_samples", static_cast<double>(p.freshSamples)},
        {"climb_avg_count", static_cast<double>(p.avgCount)},
        {"climb_tol_mm", p.tolMm},
        {"climb_move_timeout_ms", p.moveTimeoutMs},
        {"climb_velocity_mm_s", p.velocityMmS},
        {"climb_trace", xcanTrace_ ? 1.0 : 0.0},
    };
    scriptStartErr_.clear();
    if (!orch_.lua.Start(script, std::move(env), &scriptStartErr_))
        FD_WARN("script", "climb preset '%s' rejected: %s", p.label.c_str(),
                scriptStartErr_.c_str());
}

// The Run page's Scan band: ONE shared header — feedback-channel picker +
// live value + Trace — over three tabs: Raster (first-light area scan), Climb
// (gap-closing Z + XY raster) and XCAN (align hill climb). Each tab owns its
// target preset buttons.
void App::DrawRunScanBlock()
{
    // Shared header row: channel picker (live DataBus topics) + current value.
    ImGui::SetNextItemWidth(150.0f);
    if (ImGui::BeginCombo("##scanch", runXcanTopic_)) {
        const std::vector<std::string> topics = DataBus::Instance().Topics();
        if (topics.empty()) ImGui::TextDisabled("(no channels published yet)");
        for (const std::string& t : topics) {
            if (ImGui::Selectable(t.c_str(), t == runXcanTopic_))
                std::snprintf(runXcanTopic_, sizeof(runXcanTopic_), "%s", t.c_str());
        }
        ImGui::EndCombo();
    }
    ImGui::SameLine();
    {
        std::vector<float> t, v;
        DataBus::Instance().Stream(runXcanTopic_).Snapshot(t, v);
        if (!v.empty()) {
            // Keithley topics publish amps; other topics get the bare SI prefix.
            const bool amps =
                std::string(runXcanTopic_).compare(0, 9, "keithley/") == 0;
            ImGui::Text("= %s%s", FormatSI(static_cast<double>(v.back())).c_str(),
                        amps ? "A" : "");
        } else {
            ImGui::TextDisabled("= (no data)");
        }
    }

    // Targets live per-tab now (activatable preset buttons); the header keeps
    // only the shared channel + Trace. Trace sits at a FIXED column so the
    // varying width of the live reading doesn't shove it around.
    ImGui::SameLine(150.0f + ImGui::GetStyle().ItemSpacing.x +
                    ImGui::CalcTextSize("= -888.888mA  ").x);
    ImGui::Checkbox("Trace", &xcanTrace_);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Log every scan sample as 'xcan,'/'rast,' CSV lines\n"
                          "(also in the daily log file).");

    // One tab per scan method, in the order the machine uses them: raster to
    // first light, climb the gap, then XCAN to the peak.
    if (ImGui::BeginTabBar("##scantabs")) {
        if (ImGui::BeginTabItem("Raster")) {
            DrawRunRasterBlock();
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Climb")) {
            DrawRunClimbBlock();
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("XCAN")) {
            DrawRunXcanBlock();
            ImGui::EndTabItem();
        }
        ImGui::EndTabBar();
    }
}

// ---- Quick-align band (Run page, under the execution view) ------------------
// The five presses a SIPHOG operator actually makes, each collapsing what the
// XCAN tab spreads over two rows -- pick a target, pick a ladder, pick a
// hexapod -- into one button. Section 1 is the first-light pass at the low
// target; section 2 is the four production passes at the high one.
//
// The slots are fixed but the LABELS are rendered from the live target, so an
// operator who retunes slot 2 in the target modal gets a button that still says
// what it will do rather than a stale number. A button whose target slot or
// step preset has been switched off in the settings modal is disabled, not
// hidden -- a missing button reads as a broken build.
constexpr int kQuickTargetLo = 1;   // xcanTargets_ slot: 400 uA by default
constexpr int kQuickTargetHi = 2;   // xcanTargets_ slot: 3400 uA by default
constexpr int kQuickCoarse   = 0;   // xcanPresets_ slot: "Coarse" 10,5,2,1 um
constexpr int kQuickFine     = 1;   // xcanPresets_ slot: "Fine"   2,1,0.5 um

// Resolve a motion device by its config name; nullptr when it is not built.
static IMotionController* MotionByName(const HardwareManager& hw, const char* name)
{
    for (std::size_t i = 0; i < hw.MotionCount(); ++i) {
        if (hw.MotionName(i) == name) return hw.MotionDevice(i);
    }
    return nullptr;
}

void App::DrawRunQuickAlign()
{
    const ImGuiStyle& style = ImGui::GetStyle();
    // Gate on the WHOLE orchestrator, not orch_.lua: the program runner goes
    // momentarily idle between queue items, so a lua-only check would let a
    // press slip an align in between two recipes of a running program.
    const bool busy = orch_.Busy();

    const float gap  = style.ItemSpacing.x;
    const float btnH = ImGui::GetFrameHeight() * 1.8f;
    // Three button columns (one low-target, two high-target) plus the divider.
    const float availW = ImGui::GetContentRegionAvail().x;
    float btnW = (availW - gap * 5.0f) / 3.0f;
    btnW = btnW < 90.0f ? 90.0f : (btnW > 240.0f ? 240.0f : btnW);
    const float blockH = ImGui::GetTextLineHeightWithSpacing() +
                         btnH * 2.0f + style.ItemSpacing.y;

    // One launch button: activate the target slot the label was drawn from,
    // then start the ladder on that hexapod. Same channel as the XCAN tab --
    // StartXcanPreset reads the ACTIVE slot, so the assignment is the argument.
    const auto launch = [&](const char* label, int targetIdx, int presetIdx,
                            const char* dev, const ImVec2& size) {
        const XcanTarget& t = xcanTargets_[targetIdx];
        const XcanPreset& p = xcanPresets_[presetIdx];
        const IMotionController* m = MotionByName(hardware_, dev);
        const bool ready = m && m->IsConnected() && t.enabled && p.enabled;
        ImGui::BeginDisabled(busy || !ready);
        if (ImGui::Button(label, size)) {
            xcanPresetSel_ = targetIdx;
            xcanTargetMax_ = false;   // this button names a target
            StartXcanPreset(dev, presetIdx);
        }
        ImGui::EndDisabled();
        if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
            if (!m)                   ImGui::SetTooltip("%s is not configured", dev);
            else if (!m->IsConnected()) ImGui::SetTooltip("%s is not connected", dev);
            else if (!t.enabled)      ImGui::SetTooltip("target slot %d is off "
                                                        "(right-click a Target button)",
                                                        targetIdx + 1);
            else if (!p.enabled)      ImGui::SetTooltip("preset slot %d is off "
                                                        "(right-click a preset button)",
                                                        presetIdx + 1);
            else ImGui::SetTooltip("%s  %s: %s um\ntarget %g uA (zones %g-%g uA)",
                                   dev, p.label.c_str(), p.stepsUm.c_str(),
                                   t.targetUa, t.zoneLowUa, t.zoneHighUa);
        }
    };

    // Section 1: first light at the low target -- one tall button.
    ImGui::PushID("quickLo");
    ImGui::BeginGroup();
    ImGui::TextDisabled("%s", XcanTargetLabel(xcanTargets_[kQuickTargetLo]).c_str());
    launch("Left Coarse", kQuickTargetLo, kQuickCoarse, "hex-left",
           ImVec2(btnW, btnH * 2.0f + style.ItemSpacing.y));
    ImGui::EndGroup();
    ImGui::PopID();

    // Divider between the two power classes (the drawn line the operator reads
    // as "these are different passes", not an ImGui separator -- a vertical one
    // is imgui_internal and this TU does not include it).
    ImGui::SameLine(0.0f, gap);
    const ImVec2 dv = ImGui::GetCursorScreenPos();
    ImGui::Dummy(ImVec2(1.0f, blockH));
    ImGui::GetWindowDrawList()->AddLine(ImVec2(dv.x, dv.y),
                                        ImVec2(dv.x, dv.y + blockH),
                                        ImGui::GetColorU32(ImGuiCol_Separator));
    ImGui::SameLine(0.0f, gap);

    // Section 2: the production passes at the high target -- left then right,
    // coarse over fine, the order they are run in.
    ImGui::PushID("quickHi");
    ImGui::BeginGroup();
    ImGui::TextDisabled("%s", XcanTargetLabel(xcanTargets_[kQuickTargetHi]).c_str());
    launch("Left Coarse", kQuickTargetHi, kQuickCoarse, "hex-left", ImVec2(btnW, btnH));
    ImGui::SameLine();
    launch("Left Fine", kQuickTargetHi, kQuickFine, "hex-left", ImVec2(btnW, btnH));
    launch("Right Coarse", kQuickTargetHi, kQuickCoarse, "hex-right", ImVec2(btnW, btnH));
    ImGui::SameLine();
    launch("Right Fine", kQuickTargetHi, kQuickFine, "hex-right", ImVec2(btnW, btnH));
    ImGui::EndGroup();
    ImGui::PopID();

    // Stop rides in the band too: the operator who pressed a button here must
    // not have to find the XCAN tab to call it off.
    ImGui::SameLine(0.0f, gap);
    ImGui::BeginGroup();
    ImGui::TextDisabled(" ");
    ImGui::BeginDisabled(!orch_.lua.Running());
    if (IconButton(UiIcon("Stop"), "Stop",
                   ImVec2(0.0f, btnH * 2.0f + style.ItemSpacing.y)))
        orch_.lua.RequestStop();
    ImGui::EndDisabled();
    ImGui::EndGroup();
}

// XCAN tab of the Scan band: target/zone preset buttons (one active, yellow)
// + bobcat step presets — Coarse (10,5,2,1 um) / Fine (2,1,0.5 um) per
// hexapod. Channel/Trace live in the shared Scan-band header.
void App::DrawRunXcanBlock()
{
    const LuaEngine::Status st = orch_.lua.GetStatus();
    const bool running = orch_.lua.Running();

    // Target preset row: exit target + adaptive zone range per product power
    // class; the ACTIVE (yellow) preset is injected into every launch.
    // Left-click activates, RIGHT-click edits in the target settings modal.
    ImGui::TextDisabled("Target");
    bool anyTarget = false;
    for (int i = 0; i < kXcanPresetCount; ++i) {
        const XcanTarget& t = xcanTargets_[i];
        if (!t.enabled) continue;
        anyTarget = true;
        ImGui::SameLine();
        ImGui::PushID(i);
        if (PresetButton(XcanTargetLabel(t).c_str(), xcanPresetSel_ == i && !xcanTargetMax_,
                         ImVec2(76.0f, 0.0f))) {
            xcanPresetSel_ = i;
            xcanTargetMax_ = false;
        }
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Exit target %g uA, adaptive zones %g-%g uA\n"
                              "right-click: settings",
                              t.targetUa, t.zoneLowUa, t.zoneHighUa);
            if (ImGui::IsMouseClicked(ImGuiMouseButton_Right)) {
                xcanTgtCfgSel_  = i;
                xcanTgtCfgOpen_ = true;
            }
        }
        ImGui::PopID();
    }
    if (!anyTarget) {   // escape hatch — no button left to right-click
        ImGui::SameLine();
        if (ImGui::SmallButton("targets...")) xcanTgtCfgOpen_ = true;
    }
    // MAX: no exit threshold — run the whole ladder, stay on the best point.
    // The manual maximisation for a part whose power class is not known yet.
    ImGui::SameLine();
    if (PresetButton("MAX", xcanTargetMax_, ImVec2(56.0f, 0.0f))) xcanTargetMax_ = true;
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("No exit threshold: every rung of the ladder runs and the\n"
                          "hexapod parks on the best point found (adaptive zones off).\n"
                          "Click a target to leave MAX. Climb needs a target and\n"
                          "will not run under MAX.");

    // Launch row: hex-left [presets...]   hex-right [presets...]   Stop.
    // Left-click launches on that hexapod; RIGHT-click opens the preset's
    // settings modal (works even while the button is disabled).
    const bool busy = orch_.Busy();
    bool anyPreset = false;
    for (const XcanPreset& p : xcanPresets_) anyPreset |= p.enabled;
    const auto presets = [&](const char* dev) {
        int idx = -1;
        for (std::size_t i = 0; i < hardware_.MotionCount(); ++i) {
            if (hardware_.MotionName(i) == dev) { idx = static_cast<int>(i); break; }
        }
        IMotionController* m =
            idx >= 0 ? hardware_.MotionDevice(static_cast<std::size_t>(idx)) : nullptr;
        ImGui::TextDisabled("%s", dev);
        ImGui::PushID(dev);
        ImGui::BeginDisabled(busy || !m || !m->IsConnected());
        for (int i = 0; i < kXcanPresetCount; ++i) {
            const XcanPreset& p = xcanPresets_[i];
            if (!p.enabled) continue;
            ImGui::SameLine();
            ImGui::PushID(i);
            if (ImGui::Button(p.label.c_str(), ImVec2(70.0f, 0.0f)))
                StartXcanPreset(dev, i);
            if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
                ImGui::SetTooltip("steps: %s um\nright-click: settings",
                                  p.stepsUm.c_str());
                if (ImGui::IsMouseClicked(ImGuiMouseButton_Right)) {
                    xcanCfgSel_  = i;
                    xcanCfgOpen_ = true;
                }
            }
            ImGui::PopID();
        }
        ImGui::EndDisabled();
        ImGui::PopID();
    };
    presets("hex-left");
    ImGui::SameLine();
    ImGui::TextDisabled(" | ");
    ImGui::SameLine();
    presets("hex-right");
    ImGui::SameLine();
    ImGui::BeginDisabled(!running);
    if (IconButton(UiIcon("Stop"), "Stop", ImVec2(92.0f, 0.0f)))
        orch_.lua.RequestStop();
    ImGui::EndDisabled();
    if (!anyPreset) {   // escape hatch — no button left to right-click
        ImGui::SameLine();
        if (ImGui::SmallButton("presets...")) xcanCfgOpen_ = true;
    }

    // Status line: the running/last hexapod_align result (shared engine ctx).
    if (st.scriptName == "hexapod_align") {
        ImGui::Text("%s", LuaEngine::StateName(st.state));
        for (const auto& [key, val] : st.ctx) {
            if (key == "align_baseline" || key == "align_best" ||
                key == "align_gain" || key == "align_exit_hit") {
                ImGui::SameLine();
                ImGui::TextDisabled("%s=%s", key.c_str() + 6,   // strip "align_"
                                    CtxValueToString(val).c_str());
            }
        }
        if (!st.error.empty() && st.state != LuaEngine::State::Running) {
            ImGui::SameLine();
            ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.3f, 1.0f), "%s", st.error.c_str());
        }
    } else if (!scriptStartErr_.empty()) {
        ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.3f, 1.0f), "start rejected: %s",
                           scriptStartErr_.c_str());
    } else {
        std::string s;
        for (const XcanPreset& p : xcanPresets_) {
            if (!p.enabled) continue;
            s += (s.empty() ? "" : "   ") + p.label + ": " + p.stepsUm + " um";
        }
        if (s.empty()) s = "no presets enabled";
        ImGui::TextDisabled("%s   (right-click to edit)", s.c_str());
    }

    DrawXcanPresetModal();
    DrawXcanTargetModal();
}

// Right-click settings for an XCAN target preset: exit target + the adaptive
// step-zone range (lower/upper limits, in uA). Save persists all five target
// slots (and the launch presets) to config/scan_presets.local.json.
void App::DrawXcanTargetModal()
{
    if (xcanTgtCfgOpen_) {
        ImGui::OpenPopup("XCAN Target");
        xcanTgtCfgOpen_ = false;
    }
    ImGui::SetNextWindowPos(ImGui::GetMainViewport()->GetCenter(),
                            ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
    if (!ImGui::BeginPopupModal("XCAN Target", nullptr,
                                ImGuiWindowFlags_AlwaysAutoResize))
        return;

    // Slot selector: all five targets, the edited one yellow.
    for (int i = 0; i < kXcanPresetCount; ++i) {
        if (i > 0) ImGui::SameLine();
        ImGui::PushID(i);
        if (PresetButton(XcanTargetLabel(xcanTargets_[i]).c_str(),
                         xcanTgtCfgSel_ == i, ImVec2(76.0f, 0.0f)))
            xcanTgtCfgSel_ = i;
        ImGui::PopID();
    }
    XcanTarget& t = xcanTargets_[xcanTgtCfgSel_];
    ImGui::Separator();

    ImGui::Checkbox("show button", &t.enabled);
    const float w = 150.0f;
    ImGui::SetNextItemWidth(w);
    ImGui::InputDouble("target uA (exit threshold)", &t.targetUa, 0.0, 0.0, "%g");
    ImGui::SeparatorText("adaptive step zones");
    ImGui::SetNextItemWidth(w);
    ImGui::InputDouble("zone low uA (lower limit)", &t.zoneLowUa, 0.0, 0.0, "%g");
    ImGui::SetNextItemWidth(w);
    ImGui::InputDouble("zone high uA (upper limit)", &t.zoneHighUa, 0.0, 0.0, "%g");
    if (t.zoneHighUa < t.zoneLowUa)
        ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.3f, 1.0f),
                           "zone high is below zone low");
    ImGui::TextDisabled("zone low should sit at the raster handoff level");

    ImGui::Separator();
    if (IconButton(UiIcon("Save"), "Save", ImVec2(92.0f, 0.0f))) {
        SaveScanPresets();
        ImGui::CloseCurrentPopup();
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Persist ALL presets to config/scan_presets.local.json");
    ImGui::SameLine();
    if (ImGui::Button("Close", ImVec2(90.0f, 0.0f))) ImGui::CloseCurrentPopup();
    ImGui::SameLine();
    if (ImGui::Button("Defaults", ImVec2(90.0f, 0.0f)))
        t = DefaultXcanTarget(xcanTgtCfgSel_);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Reset THIS target to its built-in values");
    ImGui::EndPopup();
}

// Right-click settings for an XCAN launch preset — the same schema-form idea
// as the recipe parameter inspector, over the hexapod_align.lua tunables.
// Save persists all five presets to config/scan_presets.local.json; Close
// keeps the edits for this session only.
void App::DrawXcanPresetModal()
{
    if (xcanCfgOpen_) {
        ImGui::OpenPopup("XCAN Preset");
        xcanCfgOpen_ = false;
    }
    ImGui::SetNextWindowPos(ImGui::GetMainViewport()->GetCenter(),
                            ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
    if (!ImGui::BeginPopupModal("XCAN Preset", nullptr,
                                ImGuiWindowFlags_AlwaysAutoResize))
        return;

    // Slot selector: all five presets, the edited one yellow.
    for (int i = 0; i < kXcanPresetCount; ++i) {
        if (i > 0) ImGui::SameLine();
        ImGui::PushID(i);
        if (PresetButton(xcanPresets_[i].label.c_str(), xcanCfgSel_ == i,
                         ImVec2(76.0f, 0.0f)))
            xcanCfgSel_ = i;
        ImGui::PopID();
    }
    XcanPreset& p = xcanPresets_[xcanCfgSel_];
    ImGui::Separator();

    ImGui::Checkbox("show button", &p.enabled);
    const float w = 150.0f;
    char lb[32];
    std::snprintf(lb, sizeof(lb), "%s", p.label.c_str());
    ImGui::SetNextItemWidth(w);
    if (ImGui::InputText("label", lb, sizeof(lb))) p.label = lb;
    char sb[64];
    std::snprintf(sb, sizeof(sb), "%s", p.stepsUm.c_str());
    ImGui::SetNextItemWidth(w);
    if (ImGui::InputText("steps_um (coarse -> fine)", sb, sizeof(sb)))
        p.stepsUm = sb;
    char ab[32];
    std::snprintf(ab, sizeof(ab), "%s", p.axes.c_str());
    ImGui::SetNextItemWidth(w);
    if (ImGui::InputText("axes (descent order)", ab, sizeof(ab))) p.axes = ab;

    ImGui::SeparatorText("measure");
    ImGui::SetNextItemWidth(w);
    ImGui::InputDouble("settle_ms", &p.settleMs, 0.0, 0.0, "%g");
    ImGui::SetNextItemWidth(w);
    ImGui::InputDouble("settle_min_ms", &p.settleMinMs, 0.0, 0.0, "%g");
    ImGui::SetNextItemWidth(w);
    ImGui::InputInt("fresh_samples", &p.freshSamples);
    ImGui::SetNextItemWidth(w);
    ImGui::InputInt("avg_count", &p.avgCount);

    ImGui::SeparatorText("convergence");
    ImGui::SetNextItemWidth(w);
    ImGui::InputInt("consec_limit", &p.consecLimit);
    ImGui::SetNextItemWidth(w);
    ImGui::InputDouble("sig_drop_pct", &p.sigDropPct, 0.0, 0.0, "%g");
    ImGui::SetNextItemWidth(w);
    ImGui::InputDouble("improve_pct", &p.improvePct, 0.0, 0.0, "%g");
    ImGui::SetNextItemWidth(w);
    ImGui::InputDouble("min_signal", &p.minSignal, 0.0, 0.0, "%g");

    ImGui::SeparatorText("motion");
    ImGui::SetNextItemWidth(w);
    ImGui::InputDouble("max_travel_um", &p.maxTravelUm, 0.0, 0.0, "%g");
    ImGui::SetNextItemWidth(w);
    ImGui::InputDouble("velocity_mm_s", &p.velocityMmS, 0.0, 0.0, "%g");
    ImGui::SetNextItemWidth(w);
    ImGui::InputDouble("tol_mm", &p.tolMm, 0.0, 0.0, "%g");
    ImGui::SetNextItemWidth(w);
    ImGui::InputDouble("move_timeout_ms", &p.moveTimeoutMs, 0.0, 0.0, "%g");

    ImGui::SeparatorText("heuristics");
    ImGui::Checkbox("skip_opposite", &p.skipOpposite);
    ImGui::SameLine();
    ImGui::Checkbox("dir_memory", &p.dirMemory);
    ImGui::SameLine();
    ImGui::Checkbox("reuse_sample", &p.reuseSample);

    ImGui::Separator();
    if (IconButton(UiIcon("Save"), "Save", ImVec2(92.0f, 0.0f))) {
        SaveScanPresets();
        ImGui::CloseCurrentPopup();
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Persist ALL five presets to config/scan_presets.local.json");
    ImGui::SameLine();
    if (ImGui::Button("Close", ImVec2(90.0f, 0.0f))) ImGui::CloseCurrentPopup();
    ImGui::SameLine();
    if (ImGui::Button("Defaults", ImVec2(90.0f, 0.0f)))
        p = DefaultXcanPreset(xcanCfgSel_);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Reset THIS preset to its built-in values");
    ImGui::EndPopup();
}

// Raster scan launcher: raster_scan.lua with the band's spans/steps + the
// tab's active first-light target preset; feedback channel and Trace are
// shared with the XCAN band. Axes stay at the script's defaults (slow = X
// stepping down from start, fast = Y sweeping centered).
void App::StartRasterScan(const std::string& device)
{
    std::string script = "config/scripts/raster_scan.lua";
    for (const ScriptInfo& si : session_.Scripts()) {
        if (si.name == "raster_scan") { script = si.path; break; }
    }
    LuaEngine::Env env = session_.MakeLuaEnv();
    env.initialCtx = {
        {"raster_device", device},
        {"raster_topic", std::string(runXcanTopic_)},
        {"raster_slow_span_um", rasterSlowSpanUm_},
        {"raster_slow_step_um", rasterSlowStepUm_},
        {"raster_fast_span_um", rasterFastSpanUm_},
        {"raster_fast_step_um", rasterFastStepUm_},
        {"raster_settle_ms", 150.0},
        {"raster_trace", xcanTrace_ ? 1.0 : 0.0},
        {"raster_exit_threshold", kRasterTargetsUa[rasterTgtSel_] * 1e-6},
    };
    scriptStartErr_.clear();
    if (!orch_.lua.Start(script, std::move(env), &scriptStartErr_))
        FD_WARN("script", "raster scan rejected: %s", scriptStartErr_.c_str());
}

// Raster tab of the Scan band: span/step knobs + per-hexapod Scan buttons.
// Channel/Target/Trace live in the shared Scan-band header (DrawRunScanBlock).
void App::DrawRunRasterBlock()
{
    const LuaEngine::Status st = orch_.lua.GetStatus();
    const bool running = orch_.lua.Running();

    // Knob row: X- span/step, Y span/step (um).
    ImGui::TextDisabled("X- span/step");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(72.0f);
    ImGui::InputDouble("##rss", &rasterSlowSpanUm_, 0.0, 0.0, "%.0f");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(64.0f);
    ImGui::InputDouble("##rsp", &rasterSlowStepUm_, 0.0, 0.0, "%.1f");
    ImGui::SameLine();
    ImGui::TextDisabled("  Y+/- span/step");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(72.0f);
    ImGui::InputDouble("##rfs", &rasterFastSpanUm_, 0.0, 0.0, "%.0f");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(64.0f);
    ImGui::InputDouble("##rfp", &rasterFastStepUm_, 0.0, 0.0, "%.1f");
    ImGui::SameLine();
    ImGui::TextDisabled("um");
    if (rasterSlowStepUm_ < 0.1) rasterSlowStepUm_ = 0.1;
    if (rasterFastStepUm_ < 0.1) rasterFastStepUm_ = 0.1;
    if (rasterSlowSpanUm_ < 0.0) rasterSlowSpanUm_ = 0.0;
    if (rasterFastSpanUm_ < 0.0) rasterFastSpanUm_ = 0.0;

    // Target preset row: first-light exit level, activatable (yellow = live).
    ImGui::TextDisabled("Target");
    for (int i = 0; i < static_cast<int>(std::size(kRasterTargetsUa)); ++i) {
        ImGui::SameLine();
        ImGui::PushID(i);
        char lbl[16];
        std::snprintf(lbl, sizeof(lbl), "%g uA", kRasterTargetsUa[i]);
        if (PresetButton(lbl, rasterTgtSel_ == i, ImVec2(64.0f, 0.0f)))
            rasterTgtSel_ = i;
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Stop-and-stay the moment a grid node reaches %g uA",
                              kRasterTargetsUa[i]);
        ImGui::PopID();
    }

    // Launch row: one Scan per hexapod + Stop; channel and Trace come from
    // the shared Scan-band header.
    const bool busy = orch_.Busy();
    const auto scanBtn = [&](const char* dev) {
        int idx = -1;
        for (std::size_t i = 0; i < hardware_.MotionCount(); ++i) {
            if (hardware_.MotionName(i) == dev) { idx = static_cast<int>(i); break; }
        }
        IMotionController* m =
            idx >= 0 ? hardware_.MotionDevice(static_cast<std::size_t>(idx)) : nullptr;
        ImGui::TextDisabled("%s", dev);
        ImGui::SameLine();
        ImGui::PushID(dev);
        ImGui::BeginDisabled(busy || !m || !m->IsConnected());
        if (ImGui::Button("Scan", ImVec2(70.0f, 0.0f))) StartRasterScan(dev);
        ImGui::EndDisabled();
        ImGui::PopID();
    };
    scanBtn("hex-left");
    ImGui::SameLine();
    ImGui::TextDisabled(" | ");
    ImGui::SameLine();
    scanBtn("hex-right");
    ImGui::SameLine();
    ImGui::BeginDisabled(!running);
    if (IconButton(UiIcon("Stop"), "Stop", ImVec2(92.0f, 0.0f)))
        orch_.lua.RequestStop();
    ImGui::EndDisabled();

    // Status line: the running/last raster_scan result (shared engine ctx).
    if (st.scriptName == "raster_scan") {
        ImGui::Text("%s", LuaEngine::StateName(st.state));
        for (const auto& [key, val] : st.ctx) {
            if (key == "raster_baseline" || key == "raster_best" ||
                key == "raster_points" || key == "raster_exit_hit" ||
                key == "raster_best_slow_um" || key == "raster_best_fast_um") {
                ImGui::SameLine();
                ImGui::TextDisabled("%s=%s", key.c_str() + 7,   // strip "raster_"
                                    CtxValueToString(val).c_str());
            }
        }
        if (!st.error.empty() && st.state != LuaEngine::State::Running) {
            ImGui::SameLine();
            ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.3f, 1.0f), "%s", st.error.c_str());
        }
    } else {
        ImGui::TextDisabled("Serpentine area scan: X steps down from start, Y sweeps +/- half span.");
    }
}

// Climb tab of the Scan band: the Target row (SHARED with the XCAN tab — one
// product power class), an inline row of the knobs that actually get retuned
// between parts, and the per-hexapod launch buttons. Everything else is behind
// the right-click settings modal, as with XCAN.
//
// The inline knobs edit climbPresets_[climbCfgSel_], and a left-click on a
// launch button both starts that preset and makes it the one the knobs edit —
// so the operator's loop is "press Climb, watch, nudge the Z step, press again"
// without a modal in the way. Persisting still goes through the modal's Save.
void App::DrawRunClimbBlock()
{
    const LuaEngine::Status st = orch_.lua.GetStatus();
    const bool running = orch_.lua.Running();

    // Target row: the SAME slots the XCAN tab drives at, and the same active
    // selection. The climb's own exit criterion is the fraction of it below.
    ImGui::TextDisabled("Target");
    bool anyTarget = false;
    for (int i = 0; i < kXcanPresetCount; ++i) {
        const XcanTarget& t = xcanTargets_[i];
        if (!t.enabled) continue;
        anyTarget = true;
        ImGui::SameLine();
        ImGui::PushID(i);
        if (PresetButton(XcanTargetLabel(t).c_str(), xcanPresetSel_ == i && !xcanTargetMax_,
                         ImVec2(76.0f, 0.0f))) {
            xcanPresetSel_ = i;
            xcanTargetMax_ = false;
        }
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Climb until a plane's best reaches %g%% of %g uA\n"
                              "(shared with the XCAN tab; right-click: settings)",
                              climbPresets_[climbCfgSel_].targetPct, t.targetUa);
            if (ImGui::IsMouseClicked(ImGuiMouseButton_Right)) {
                xcanTgtCfgSel_  = i;
                xcanTgtCfgOpen_ = true;
            }
        }
        ImGui::PopID();
    }
    if (!anyTarget) {
        ImGui::SameLine();
        if (ImGui::SmallButton("targets...")) xcanTgtCfgOpen_ = true;
    }
    ImGui::SameLine();
    {
        const XcanTarget& t = xcanTargets_[xcanPresetSel_];
        if (xcanTargetMax_)
            ImGui::TextColored(ImVec4(0.98f, 0.72f, 0.32f, 1.0f),
                               "  XCAN is on MAX - pick a target to climb");
        else
            ImGui::TextDisabled("  exit at %.1f uA",
                                t.targetUa * climbPresets_[climbCfgSel_].targetPct / 100.0);
    }

    // Inline knob row: the four numbers that get retuned on a real part.
    ClimbPreset& p = climbPresets_[climbCfgSel_];
    ImGui::TextDisabled("%s:", p.label.c_str());
    ImGui::SameLine();
    ImGui::TextDisabled("Z step/budget");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(56.0f);
    ImGui::InputDouble("##czs", &p.zStepUm, 0.0, 0.0, "%.2f");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(56.0f);
    ImGui::InputDouble("##czb", &p.maxZUm, 0.0, 0.0, "%.0f");
    ImGui::SameLine();
    ImGui::TextDisabled("um   XY step/half");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(56.0f);
    ImGui::InputDouble("##cxy", &p.xyStepUm, 0.0, 0.0, "%.2f");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(46.0f);
    ImGui::InputInt("##cxn", &p.xySteps, 0, 0);
    ImGui::SameLine();
    ImGui::TextDisabled("   probe");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(46.0f);
    ImGui::InputDouble("##cpp", &p.zProbePct, 0.0, 0.0, "%.0f");
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Skip the XY search while one Z step alone gains more\n"
                          "than this %%. 0 = search every plane.");
    ImGui::SameLine();
    ImGui::TextDisabled("%%");
    ImGui::SameLine();
    ImGui::Checkbox("spiral", &p.spiral);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Off: cross scan — a line per axis, stops past the peak (few points).\n"
                          "On:  spiral — the full grid from the middle; assumes nothing\n"
                          "     about the peak's shape, at the cost of every point.");
    // Same clamps the script asserts on, applied here so a typo is corrected in
    // the form rather than failing the run.
    if (p.zStepUm  < 0.01) p.zStepUm  = 0.01;
    if (p.xyStepUm < 0.01) p.xyStepUm = 0.01;
    if (p.maxZUm   < p.zStepUm) p.maxZUm = p.zStepUm;
    if (p.xySteps  < 1) p.xySteps = 1;
    if (p.zProbePct < 0.0)  p.zProbePct = 0.0;
    if (p.zProbePct > 99.0) p.zProbePct = 99.0;

    // Launch row: hex-left [presets...]   hex-right [presets...]   Stop.
    // Left-click launches (and selects for editing); RIGHT-click opens the
    // settings modal, which works even while the button is disabled.
    const bool busy = orch_.Busy();
    bool anyPreset = false;
    for (const ClimbPreset& cp : climbPresets_) anyPreset |= cp.enabled;
    // Under MAX there is no target for the climb to take a fraction of.
    const bool haveTarget = xcanTargets_[xcanPresetSel_].targetUa > 0.0 && !xcanTargetMax_;
    const auto presets = [&](const char* dev) {
        int idx = -1;
        for (std::size_t i = 0; i < hardware_.MotionCount(); ++i) {
            if (hardware_.MotionName(i) == dev) { idx = static_cast<int>(i); break; }
        }
        IMotionController* m =
            idx >= 0 ? hardware_.MotionDevice(static_cast<std::size_t>(idx)) : nullptr;
        ImGui::TextDisabled("%s", dev);
        ImGui::PushID(dev);
        ImGui::BeginDisabled(busy || !haveTarget || !m || !m->IsConnected());
        for (int i = 0; i < kClimbPresetCount; ++i) {
            if (!climbPresets_[i].enabled) continue;
            ImGui::SameLine();
            ImGui::PushID(i);
            // Wider than the XCAN row's 70, which clips a six-letter label.
            if (ImGui::Button(climbPresets_[i].label.c_str(), ImVec2(84.0f, 0.0f))) {
                climbCfgSel_ = i;          // the knob row follows what was launched
                StartClimbPreset(dev, i);
            }
            if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
                ImGui::SetTooltip("Z %g um up to %g um, %s plane %g um +/-%d\n"
                                  "right-click: settings",
                                  climbPresets_[i].zStepUm, climbPresets_[i].maxZUm,
                                  climbPresets_[i].spiral ? "spiral" : "cross",
                                  climbPresets_[i].xyStepUm, climbPresets_[i].xySteps);
                if (ImGui::IsMouseClicked(ImGuiMouseButton_Right)) {
                    climbCfgSel_  = i;
                    climbCfgOpen_ = true;
                }
            }
            ImGui::PopID();
        }
        ImGui::EndDisabled();
        ImGui::PopID();
    };
    presets("hex-left");
    ImGui::SameLine();
    ImGui::TextDisabled(" | ");
    ImGui::SameLine();
    presets("hex-right");
    ImGui::SameLine();
    ImGui::BeginDisabled(!running);
    if (IconButton(UiIcon("Stop"), "Stop", ImVec2(92.0f, 0.0f)))
        orch_.lua.RequestStop();
    ImGui::EndDisabled();
    if (!anyPreset) {   // escape hatch — no button left to right-click
        ImGui::SameLine();
        if (ImGui::SmallButton("presets...")) climbCfgOpen_ = true;
    }

    // Status line: the running/last climb_raster result (shared engine ctx).
    // climb_stop is the one to read — "contact" is a stop the operator has to
    // see, not a tidy finish, so it is colored like a failure.
    if (st.scriptName == "climb_raster") {
        ImGui::Text("%s", LuaEngine::StateName(st.state));
        for (const auto& [key, val] : st.ctx) {
            if (key == "climb_best" || key == "climb_final" ||
                key == "climb_target_frac" || key == "climb_best_z_um" ||
                key == "climb_planes" || key == "climb_skipped" ||
                key == "climb_stop") {
                ImGui::SameLine();
                const bool contact = key == "climb_stop" &&
                                     CtxValueToString(val) == "contact";
                if (contact)
                    ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.3f, 1.0f),
                                       "stop=contact (retreated to the peak)");
                else
                    ImGui::TextDisabled("%s=%s", key.c_str() + 6,   // strip "climb_"
                                        CtxValueToString(val).c_str());
            }
        }
        if (!st.error.empty() && st.state != LuaEngine::State::Running) {
            ImGui::SameLine();
            ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.3f, 1.0f), "%s", st.error.c_str());
        }
    } else if (!haveTarget) {
        ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.3f, 1.0f),
                           "no target selected - the climb has no other exit criterion");
    } else {
        ImGui::TextDisabled("Step Z in, probe, search the XY plane when the step stops paying; "
                            "retreats to the peak on a suspected contact.");
    }

    DrawClimbPresetModal();
}

// Right-click settings for a climb launch preset — the same schema-form idea as
// the XCAN preset modal, over the climb_raster.lua tunables. Save persists all
// presets to config/scan_presets.local.json; Close keeps the edits for this
// session only.
void App::DrawClimbPresetModal()
{
    if (climbCfgOpen_) {
        ImGui::OpenPopup("Climb Preset");
        climbCfgOpen_ = false;
    }
    ImGui::SetNextWindowPos(ImGui::GetMainViewport()->GetCenter(),
                            ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
    if (!ImGui::BeginPopupModal("Climb Preset", nullptr,
                                ImGuiWindowFlags_AlwaysAutoResize))
        return;

    // Slot selector: all five presets, the edited one yellow.
    for (int i = 0; i < kClimbPresetCount; ++i) {
        if (i > 0) ImGui::SameLine();
        ImGui::PushID(i);
        if (PresetButton(climbPresets_[i].label.c_str(), climbCfgSel_ == i,
                         ImVec2(84.0f, 0.0f)))   // same width as the launch row
            climbCfgSel_ = i;
        ImGui::PopID();
    }
    ClimbPreset& p = climbPresets_[climbCfgSel_];
    ImGui::Separator();

    ImGui::Checkbox("show button", &p.enabled);
    const float w = 150.0f;
    char lb[32];
    std::snprintf(lb, sizeof(lb), "%s", p.label.c_str());
    ImGui::SetNextItemWidth(w);
    if (ImGui::InputText("label", lb, sizeof(lb))) p.label = lb;

    ImGui::SeparatorText("climb");
    ImGui::SetNextItemWidth(w);
    ImGui::InputDouble("z_step_um", &p.zStepUm, 0.0, 0.0, "%g");
    ImGui::SetNextItemWidth(w);
    ImGui::InputDouble("max_z_um (travel budget)", &p.maxZUm, 0.0, 0.0, "%g");
    ImGui::SetNextItemWidth(w);
    ImGui::InputDouble("target_pct (exit)", &p.targetPct, 0.0, 0.0, "%g");
    ImGui::TextDisabled("of the ACTIVE Target slot (%g uA)",
                        xcanTargets_[xcanPresetSel_].targetUa);
    ImGui::SetNextItemWidth(w);
    ImGui::InputDouble("z_probe_pct (0 = raster every plane)", &p.zProbePct, 0.0, 0.0, "%g");
    ImGui::SetNextItemWidth(w);
    ImGui::InputInt("max_skips (0 = unlimited)", &p.maxSkips);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Force a plane search after this many skipped in a row.\n"
                          "A Z move is not perfectly decoupled from the plane, so an\n"
                          "unwatched run of skips lets XY error accumulate behind a\n"
                          "gain test that cannot see it.");

    ImGui::SeparatorText("plane");
    ImGui::Checkbox("spiral (off = cross scan)", &p.spiral);
    ImGui::SetNextItemWidth(w);
    ImGui::InputDouble("xy_step_um", &p.xyStepUm, 0.0, 0.0, "%g");
    ImGui::SetNextItemWidth(w);
    ImGui::InputInt("xy_steps (half-width, cells)", &p.xySteps);
    ImGui::SetNextItemWidth(w);
    ImGui::InputInt("peak_drop (cross: past-peak)", &p.peakDrop);
    ImGui::SetNextItemWidth(w);
    ImGui::InputInt("rings_stop (spiral: 0 = full grid)", &p.ringsStop);
    ImGui::SetNextItemWidth(w);
    ImGui::InputDouble("improve_pct (noise margin)", &p.improvePct, 0.0, 0.0, "%g");
    {
        const int pts = p.spiral ? (2 * p.xySteps + 1) * (2 * p.xySteps + 1)
                                 : 1 + 8 * p.xySteps;
        ImGui::TextDisabled("<= %d points/plane, +/-%g um", pts,
                            p.xySteps * p.xyStepUm);
    }

    ImGui::SeparatorText("guards");
    ImGui::SetNextItemWidth(w);
    ImGui::InputDouble("contact_drop_pct (0 = off)", &p.contactDropPct, 0.0, 0.0, "%g");
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Stop and retreat to the peak when a plane's best falls\n"
                          "this far below it — read as the part having been touched.");
    ImGui::SetNextItemWidth(w);
    ImGui::InputInt("stall_planes (0 = off)", &p.stallPlanes);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Stop after this many planes with no new peak, instead of\n"
                          "spending the whole Z budget past a peak already found.");

    ImGui::SeparatorText("measure");
    ImGui::SetNextItemWidth(w);
    ImGui::InputDouble("settle_ms (fresh-sample ceiling)", &p.settleMs, 0.0, 0.0, "%g");
    ImGui::SetNextItemWidth(w);
    ImGui::InputDouble("settle_min_ms", &p.settleMinMs, 0.0, 0.0, "%g");
    ImGui::SetNextItemWidth(w);
    ImGui::InputDouble("z_settle_ms (after a Z step)", &p.zSettleMs, 0.0, 0.0, "%g");
    ImGui::SetNextItemWidth(w);
    ImGui::InputInt("fresh_samples", &p.freshSamples);
    ImGui::SetNextItemWidth(w);
    ImGui::InputInt("avg_count (baseline only)", &p.avgCount);

    ImGui::SeparatorText("motion");
    char zb[16];
    std::snprintf(zb, sizeof(zb), "%s", p.zAxis.c_str());
    ImGui::SetNextItemWidth(w);
    if (ImGui::InputText("z_axis", zb, sizeof(zb))) p.zAxis = zb;
    char pb[32];
    std::snprintf(pb, sizeof(pb), "%s", p.planeAxes.c_str());
    ImGui::SetNextItemWidth(w);
    if (ImGui::InputText("plane_axes", pb, sizeof(pb))) p.planeAxes = pb;
    ImGui::SetNextItemWidth(w);
    ImGui::InputDouble("velocity_mm_s", &p.velocityMmS, 0.0, 0.0, "%g");
    ImGui::SetNextItemWidth(w);
    ImGui::InputDouble("tol_mm", &p.tolMm, 0.0, 0.0, "%g");
    ImGui::SetNextItemWidth(w);
    ImGui::InputDouble("move_timeout_ms", &p.moveTimeoutMs, 0.0, 0.0, "%g");

    ImGui::Separator();
    if (IconButton(UiIcon("Save"), "Save", ImVec2(92.0f, 0.0f))) {
        SaveScanPresets();
        ImGui::CloseCurrentPopup();
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Persist ALL presets to config/scan_presets.local.json");
    ImGui::SameLine();
    if (ImGui::Button("Close", ImVec2(90.0f, 0.0f))) ImGui::CloseCurrentPopup();
    ImGui::SameLine();
    if (ImGui::Button("Defaults", ImVec2(90.0f, 0.0f)))
        p = DefaultClimbPreset(climbCfgSel_);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Reset THIS preset to its built-in values");
    ImGui::EndPopup();
}

} // namespace firedog
