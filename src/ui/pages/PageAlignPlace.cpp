// PageAlignPlace.cpp : the Placement page (a full-page takeover, reached only
// from a nominal on the Alignment page). Definitions of App members —
// see App.h.
//
// The question it answers: an object is gripped by hex-left (or hex-right), a
// nominal says where it is supposed to end up — how far, and along which of
// THAT device's axes, would it have to move?
//
// It shows every intermediate value, then issues the move behind the shared
// arming flag. The numbers stay on screen because the destination is COMPUTED:
// a mis-taught frame or a mis-marked object produces an ordinary-looking number
// that happens to be wrong, and the only defence is a human reading it.
//
// Two guards beyond the arming flag:
//   * a visible, editable CEILING on the correction (`plcMaxMM_`) — a constant
//     nobody can see would be raised by editing the source;
//   * the marks are CLEARED by a move, because they describe where the object
//     was before it. Leaving them would show a correction already spent, and a
//     second press would apply it twice. Re-measuring is what closes the loop:
//     the residual is the accuracy number, and no transform can promise it.
//
// ---- Why the correction is a DELTA, and only ever a delta -------------------
//
// A nominal is a point in the gantry's world, because that is where the camera
// measures. The hexapod's origin bears no known relation to it — nobody has
// ever measured one against the other — and `AxisTransform`
// (hardware/HardwareConfig.h) is a signed permutation with NO offset, so it can
// carry a direction between the two frames but never a position. An absolute
// machine XY therefore cannot be commanded to hex-left at all; only a
// difference can, and a difference is all this page computes:
//
//     object   = midpoint of two marks           (machine mm, via the camera)
//     nominal  = LocalToMachine(live frame, n)   (machine mm, this part's pose)
//     delta    = nominal - object                (machine mm — a VECTOR)
//     per-axis = transform * delta               (the mover's own axes)
//
// The frame's origin cancels in that subtraction, which is exactly why the
// result is transportable to a device that knows nothing about it. Rotation
// does not cancel — both points are converted through the SAME live frame, so
// the vector already carries the part's actual orientation.
//
// The mapping goes through Workflow::GlobalDeltaToAxes and not JogGlobal:
// JogGlobal collapses a direction to its single dominant axis, which is right
// for a jog button and would silently drop half of a diagonal correction here.
// The move itself is Workflow::MoveRelativeGlobal, which re-derives the same
// per-axis deltas from the same transform — so what the page prints and what
// the machine is told cannot drift apart.
#include "App.h"

#include "imgui.h"

#include "command/ExecutionContext.h"   // SystemScriptPath — the standard procedures
#include "core/Log.h"
#include "ui/Widgets.h"
#include "vision/HalconDetector.h"   // Available() — object detection needs a licence

#include <cmath>
#include <cstdio>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace firedog {

namespace {

const ImVec4 kGood(0.35f, 0.85f, 0.45f, 1.0f);
const ImVec4 kWarn(0.98f, 0.72f, 0.32f, 1.0f);
const ImVec4 kBad (0.95f, 0.45f, 0.35f, 1.0f);

} // namespace

// ---- Opening ---------------------------------------------------------------

void App::OpenAlignPlace(const AlignProc& a, const std::string& nomUid)
{
    // Level checked at the TOP, not left to SwitchPage's refusal: everything
    // below mutates state (the model, the marks, the selections) before it
    // switches, so a refused switch would leave those edits behind on a page
    // the caller never reaches. Same reason the Run deck's right-click checks
    // early. Every caller today sits on an Admin page, so this is insurance,
    // not a fix — but it is what makes the rule true everywhere.
    if (!CanAccess(Page::AlignPlace)) return;
    plcProcUid_ = a.uid;
    plcNomUid_  = nomUid;
    plcHave_[0] = plcHave_[1] = false;
    plcPt_[0]   = AlignPoint{};
    plcPt_[1]   = AlignPoint{};
    plcActive_  = 0;
    plcZoom_    = 1.5f;
    plcLastMove_.clear();
    plcSamples_.clear();
    plcPx_ = (cameraW_ > 0) ? static_cast<float>(cameraW_) * 0.5f : 0.0f;
    plcPy_ = (cameraH_ > 0) ? static_cast<float>(cameraH_) * 0.5f : 0.0f;

    // Default the mover to the first device that is NOT the one carrying the
    // camera: moving that one moves the view, not the object. Only a default —
    // the combo can pick anything, including (with a warning) that device.
    if (plcDevice_.empty()) {
        for (std::size_t i = 0; i < hardware_.MotionCount(); ++i) {
            const std::string n = hardware_.MotionName(i);
            if (n != a.device) { plcDevice_ = n; break; }
        }
    }

    // Drive the camera to this nominal's view on the way in — the same move the
    // page's own "Go to view" button makes, and the one place_view makes at run
    // time. The first thing this page asks for is a mark on the gripped object,
    // which can only be made through the camera, so opening it anywhere else is
    // opening it somewhere the object is not. A no-op when no view is taught or
    // the device is down; both are reported on the page itself.
    if (const AlignNominal* n = FindAlignNominal(a, nomUid)) GoToAlignNominalView(a, *n);

    SwitchPage(Page::AlignPlace);   // every page switch routes through here
}

// Launch the procedure script with this page's settings. Same channel the
// Alignment page's Run button uses: parameters go in through
// LuaEngine::Env::initialCtx, where fd.param reads them — which is exactly what
// place_run does from a recipe, so the guided run and the production run are
// one code path. Gated on the WHOLE orchestration, never a subset of engines.
void App::StartPlacementRun(const AlignProc& a, const AlignNominal& n,
                            const std::string& device)
{
    // The SHIPPED file, always — the same rule ExecutionContext::ResolveScript
    // applies for place_run. A product that forked this file would go on
    // skipping whatever step the app added last (place_view was exactly that),
    // and the symptom would be a camera that simply did not move.
    const std::string script = SystemScriptPath("placement_procedure.lua");
    LuaEngine::Env env = session_.MakeLuaEnv();
    env.initialCtx = {
        {"place_alignment", a.uid},        // uid, not name: a rename mid-run is harmless
        {"place_nominal",   n.uid},
        {"place_device",    device},
        {"place_align_first", plcAlignFirst_},
        {"place_tries",     static_cast<double>(plcTries_)},
        {"place_tol_mm",    static_cast<double>(plcTolMM_)},
        {"place_max_mm",    static_cast<double>(plcMaxMM_)},
        {"place_prefix",    std::string("place")},
    };
    plcStartErr_.clear();
    if (!orch_.lua.Start(script, std::move(env), &plcStartErr_))
        FD_WARN("align", "placement run rejected: %s", plcStartErr_.c_str());
}

// ---- The page --------------------------------------------------------------

void App::DrawAlignPlacePanel()
{
    std::vector<AlignProc>& list = session_.Aligns();
    AlignProc* a = FindAlignProc(list, plcProcUid_);
    if (!a) {
        ImGui::TextDisabled("The alignment procedure this page was opened for is gone.");
        if (ImGui::Button("Back to Alignment")) SwitchPage(Page::Align);
        return;
    }

    // The LIVE frame, for the reason §5b of the design doc gives: both the
    // object measurement and the nominal have to be expressed against the part
    // that is on the machine now. A correction computed half from this part and
    // half from a record would be wrong by the difference and look right.
    AlignResult frame;
    const bool haveLive = workflow_.LiveFrame(a->uid, &frame);

    // Resolved here rather than in the panel below, because the IMAGE (drawn
    // first) needs the nominal's machine position to mark it. Recomputed every
    // frame from the live frame — this is the number that follows the part.
    const AlignNominal* nom = FindAlignNominal(*a, plcNomUid_);
    double nomMx = 0.0, nomMy = 0.0;
    if (nom && haveLive)
        AlignLocalToMachine(frame, nom->lx, nom->ly, &nomMx, &nomMy);

    // The pose the marks are measured against: the procedure's device, because
    // that is the one carrying the camera.
    IMotionController* cam = nullptr;
    for (std::size_t i = 0; i < hardware_.MotionCount(); ++i) {
        if (hardware_.MotionName(i) == a->device) { cam = hardware_.MotionDevice(i); break; }
    }
    double gx = 0.0, gy = 0.0;
    bool   havePose = false;
    if (cam && cam->IsConnected()) {
        cam->RequestFastPoll();
        const std::map<std::string, double> pos = cam->Positions();
        const auto ix = pos.find("X"), iy = pos.find("Y");
        if (ix != pos.end() && iy != pos.end()) {
            gx = ix->second;
            gy = iy->second;
            havePose = true;
        }
    }

    // The mover: the device actually holding the object.
    int moverIdx = -1;
    for (std::size_t i = 0; i < hardware_.MotionCount(); ++i) {
        if (hardware_.MotionName(i) == plcDevice_) { moverIdx = static_cast<int>(i); break; }
    }
    IMotionController* mover =
        moverIdx >= 0 ? hardware_.MotionDevice(static_cast<std::size_t>(moverIdx)) : nullptr;

    UpdateCameraTexture();
    const bool haveImg = cameraTex_ && cameraW_ > 0 && cameraH_ > 0;
    const bool canMark = haveImg && havePose;

    const ImVec2 avail  = ImGui::GetContentRegionAvail();
    const float  rightW = 400.0f;
    const float  imgW   = avail.x - rightW - ImGui::GetStyle().ItemSpacing.x;

    // ---- Left: the live image ----------------------------------------------
    ImGui::BeginChild("apImg", ImVec2(imgW, avail.y), ImGuiChildFlags_Borders);
    {
        // Three marks: the two being placed, plus the NOMINAL they are aimed at.
        // The nominal is usually outside the field of view, which is exactly
        // when it matters — it becomes an edge arrow with its distance, so
        // measuring one feature while aiming at another is visible immediately
        // rather than after the correction is refused.
        const CameraMark marks[3] = {
            {plcPt_[0].wx, plcPt_[0].wy, plcHave_[0], "pt1", 0},
            {plcPt_[1].wx, plcPt_[1].wy, plcHave_[1], "pt2", 0},
            {nomMx, nomMy, (nom != nullptr) && haveLive,
             nom ? nom->name.c_str() : "", IM_COL32(255, 90, 90, 255)},
        };
        MarkPicker mp;
        mp.tex           = cameraTex_;
        mp.imgW          = cameraW_;
        mp.imgH          = cameraH_;
        mp.pixelToMmX    = pixelToMmX_;
        mp.pixelToMmY    = pixelToMmY_;
        mp.havePose      = havePose;
        mp.gx            = gx;
        mp.gy            = gy;
        mp.marks         = marks;
        mp.markCount     = 3;
        mp.midpointLabel = "object";
        EnsureMarkIcons();
        mp.icons[0]      = markIcons_[0];
        mp.icons[1]      = markIcons_[1];
        mp.activeMark    = plcActive_;
        mp.px            = &plcPx_;
        mp.py            = &plcPy_;
        mp.zoom          = &plcZoom_;
        const MarkPickerResult r =
            DrawMarkPicker("##plcInput", ImGui::GetContentRegionAvail(), mp);
        if (r.moveRequested) {
            if (jogArmed_) workflow_.MoveGantryRelativeMM(r.dxMM, r.dyMM);
            else FD_WARN("align", "click-to-move ignored - arm first");
        }
    }
    ImGui::EndChild();
    ImGui::SameLine();

    // ---- Right: target, measurement, correction -----------------------------
    ImGui::BeginChild("apCtl", ImVec2(rightW, avail.y), ImGuiChildFlags_Borders);

    ImGui::TextDisabled("place a gripped object on a taught nominal");
    // Self-sufficient: pick the procedure here rather than only inheriting the
    // Alignment page's selection, which is what lets this page be a View-menu
    // entry instead of something reachable only from a nominal's button.
    ImGui::SetNextItemWidth(200.0f);
    if (ImGui::BeginCombo("procedure", a->name.c_str())) {
        for (const AlignProc& other : list) {
            if (!ImGui::Selectable(other.name.c_str(), other.uid == a->uid)) continue;
            plcProcUid_ = other.uid;
            plcNomUid_  = other.nominals.empty() ? std::string()
                                                 : other.nominals.front().uid;
            // A measurement belongs to the procedure it was taken under.
            plcHave_[0] = plcHave_[1] = false;
            plcSamples_.clear();
            plcLastMove_.clear();
        }
        ImGui::EndCombo();
    }
    ImGui::Separator();

    if (haveLive) {
        ImGui::TextColored(kGood, "LIVE frame  taught %s", frame.capturedAt.c_str());
    } else {
        ImGui::TextColored(kBad, "NO LIVE FRAME");
        ImGui::TextWrapped("Both the target and the measurement are expressed through the "
                           "frame, so it has to be the one taught for the part on the "
                           "machine now.");
    }

    // Teach it here rather than sending the operator to another page for the
    // one thing this page cannot work without. Same launcher the Alignment
    // page's Run button uses (StartAlignmentRun -> alignment_procedure.lua), so
    // it stays the single code path a frame comes into existence through.
    {
        const bool ready = !a->s1.posUid.empty() && !a->s2.posUid.empty();
        ImGui::BeginDisabled(orch_.Busy() || !ready);
        if (ImGui::Button(haveLive ? "Re-teach frame" : "Run alignment",
                          ImVec2(150.0f, 0.0f)))
            StartAlignmentRun(*a);
        ImGui::EndDisabled();
        ImGui::SameLine();
        ImGui::BeginDisabled(!orch_.lua.Running());
        if (ImGui::Button("Stop##frame", ImVec2(70.0f, 0.0f))) orch_.lua.RequestStop();
        ImGui::EndDisabled();
        if (!ready)
            ImGui::TextColored(kWarn, "both stations need a position (Alignment page)");
        else if (orch_.Busy() && !orch_.lua.Running())
            ImGui::TextDisabled("another run is active");
        if (!alignStartErr_.empty())
            ImGui::TextColored(kBad, "%s", alignStartErr_.c_str());
    }

    // ---- Target -------------------------------------------------------------
    ImGui::SeparatorText("Target (nominal)");
    if (a->nominals.empty()) {
        ImGui::TextColored(kWarn, "this procedure has no nominals taught");
    } else {
        ImGui::SetNextItemWidth(200.0f);
        if (ImGui::BeginCombo("nominal", nom ? nom->name.c_str() : "(pick)")) {
            for (const AlignNominal& n : a->nominals) {
                if (ImGui::Selectable(n.name.c_str(), nom && n.uid == nom->uid))
                    plcNomUid_ = n.uid;
            }
            ImGui::EndCombo();
        }
    }
    if (nom) {
        // Detection models for the object's two marks — the only thing standing
        // between this page and an unattended placement. Same teach page the
        // alignment stations use; a model is private to THIS nominal and dies
        // with it. Fallback stays available and is logged when it fires, so a
        // procedure can never quietly claim to be automatic.
        AlignNominal& mn = *FindAlignNominal(*a, nom->uid);

        // Where the CAMERA has to stand to see the object being placed here.
        // It belongs to the nominal because it is a property of this placement —
        // the object is gripped somewhere else and focuses at another height —
        // and because owning it is what lets the guided run drive there itself
        // instead of an operator jogging first or a recipe carrying an extra
        // move_to_position step.
        {
            // NOTE: `vp` points into the positions vector — teaching pushes into
            // that same vector, so it is re-resolved after the button below.
            const SavedPos* vp = AlignNominalView(*a, mn);
            const bool camLive = cam && cam->IsConnected();
            ImGui::BeginDisabled(orch_.Busy() || !camLive || !vp);
            if (ImGui::Button("Go to view")) GoToAlignNominalView(*a, mn);
            ImGui::EndDisabled();
            ImGui::SameLine();
            ImGui::BeginDisabled(orch_.Busy() || !camLive);
            if (ImGui::Button(vp ? "Re-teach view" : "Teach view here"))
                TeachAlignNominalView(*a, mn);
            ImGui::EndDisabled();
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Save the CURRENT %s pose as this nominal's camera\n"
                                  "view. The guided run drives here before it marks\n"
                                  "the object.", a->device.c_str());
            vp = AlignNominalView(*a, mn);   // the teach above may have reallocated
            if (vp) {
                std::string line;
                for (const auto& [axis, v] : vp->vals) {
                    char buf[48];
                    std::snprintf(buf, sizeof(buf), "%s%s %.3f", line.empty() ? "" : "  ",
                                  axis.c_str(), v);
                    line += buf;
                }
                ImGui::TextDisabled("view  %s", line.c_str());
            } else {
                ImGui::TextDisabled("no camera view - drive there yourself first");
            }
        }

        std::string why;
        const bool halcon = HalconDetector::Available(&why);
        for (int k = 1; k <= 2; ++k) {
            ImGui::PushID(k);
            std::string& mode  = (k == 1) ? mn.mode1 : mn.mode2;
            const bool   isAuto = (mode == "auto");
            // The mode is a CHOICE the operator can take back, not a consequence
            // of a model existing — which is what teaching one used to be. The
            // radio pair is the station's, one level down; manual leaves the
            // taught patch on the shelf instead of destroying it.
            ImGui::Text("point %d", k);
            ImGui::SameLine();
            if (ImGui::RadioButton("manual", !isAuto) && isAuto) {
                mode = "manual";
                session_.SaveAligns();   // this page has no Save button
            }
            ImGui::SameLine();
            if (ImGui::RadioButton("detect", isAuto) && !isAuto) {
                mode = "auto";
                session_.SaveAligns();
            }
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Locate this mark with a taught detection model.\n"
                                  "Needs HALCON; manual is a crosshair pick and\n"
                                  "never does.");
            if (mode == "auto") {
                ImGui::Indent();
                const VisionModel* dm = NominalDetector(*a, mn, k);
                ImGui::BeginDisabled(orch_.Busy());
                if (ImGui::Button(dm ? "Teach detection..." : "Set up detection..."))
                    OpenVisionTeachForNominal(*a, mn, k);
                ImGui::EndDisabled();
                ImGui::SameLine();
                if (!halcon)     ImGui::TextColored(kBad, "%s", why.c_str());
                else if (!dm)    ImGui::TextColored(kWarn, "nothing taught");
                else             ImGui::TextDisabled("%s", dm->method.c_str());
                // What this mark looks for, with a cross on the point it
                // reports — the same picture the fallback shows when detection
                // fails and the operator is asked for that mark by hand.
                if (dm) {
                    PatchTex& pt = alignTmpl_[(k == 2) ? 1 : 0];
                    if (EnsurePatchTex(pt, *dm)) {
                        DrawPatchThumb(pt, 96.0f, 96.0f);
                        ImGui::SameLine();
                        ImGui::TextDisabled("the cross is the point reported");
                    }
                }
                ImGui::Unindent();
            }
            ImGui::PopID();
        }
        ImGui::Checkbox("ask the operator when detection fails", &mn.autoFallbackManual);
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("On, a failed detection prompts for that mark instead of\n"
                              "failing the step - always with a warning in the log, so a\n"
                              "procedure cannot look automatic while a human marks it.");

        ImGui::Text("local     %.4f, %.4f mm", nom->lx, nom->ly);
        // Where that local point sits on THIS part (computed above, before the
        // image, which marks it).
        if (haveLive) ImGui::Text("machine   %.4f, %.4f mm", nomMx, nomMy);
    }

    // ---- The mover ----------------------------------------------------------
    ImGui::SeparatorText("Mover (holds the object)");
    ImGui::SetNextItemWidth(200.0f);
    if (ImGui::BeginCombo("device", plcDevice_.empty() ? "(pick)" : plcDevice_.c_str())) {
        for (std::size_t i = 0; i < hardware_.MotionCount(); ++i) {
            const std::string n = hardware_.MotionName(i);
            if (ImGui::Selectable(n.c_str(), n == plcDevice_)) plcDevice_ = n;
        }
        ImGui::EndCombo();
    }
    if (!plcDevice_.empty() && plcDevice_ == a->device)
        ImGui::TextColored(kWarn, "that is the device carrying the camera - moving it\n"
                                  "moves the view, not the object");
    if (mover) {
        mover->RequestFastPoll();
        if (!mover->IsConnected()) {
            ImGui::TextColored(kBad, "%s not connected", plcDevice_.c_str());
        } else {
            if (transformCfg_.find(plcDevice_) == transformCfg_.end())
                ImGui::TextColored(kWarn, "no axis transform configured - assuming identity");
            // Live pose, always — a move is issued and returns immediately (no
            // blocking wait on the UI thread), so this is where an operator
            // watches it arrive before re-measuring. Without it the from/to
            // readout vanishes with the marks and there is nothing to watch.
            const std::map<std::string, double> mp = mover->Positions();
            std::string cur;
            for (const auto& [axis, v] : mp) {
                char buf[48];
                std::snprintf(buf, sizeof(buf), "%s%s %.4f", cur.empty() ? "" : "  ",
                              axis.c_str(), v);
                cur += buf;
            }
            ImGui::TextDisabled("%s", cur.c_str());
        }
    } else if (!plcDevice_.empty()) {
        ImGui::TextColored(kBad, "'%s' is not in the fleet", plcDevice_.c_str());
    }

    // ---- Guided run ---------------------------------------------------------
    // The scripted twin of everything below: it prompts for the two marks in
    // the crosshair pick window, corrects, and re-measures until the residual
    // is inside tol. Armed like the manual button — a scripted correction is
    // still a COMPUTED destination, and the arm checkbox is right here.
    ImGui::SeparatorText("Guided run (placement_procedure.lua)");
    {
        const bool running = orch_.lua.Running();
        // The same flag the manual Move button arms, with its own widget id:
        // that one lives below the correction, which does not render until both
        // marks exist — so without a copy here the guided run could never be
        // armed at all.
        ImGui::Checkbox("Arm##guided", &jogArmed_);
        ImGui::SameLine();
        // On, the script teaches the frame itself, so the run needs no live one
        // to start — which is what makes this button work on a fresh app.
        ImGui::Checkbox("align first", &plcAlignFirst_);
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Re-teach the frame for THIS part before placing.\n"
                              "Turn off only when something already aligned it in\n"
                              "this session - a nominal cannot be used without a\n"
                              "live frame, and it is cleared at program start,\n"
                              "DUT open and product load.");
        ImGui::SetNextItemWidth(70.0f);
        ImGui::InputInt("tries", &plcTries_, 0, 0);
        if (plcTries_ < 1) plcTries_ = 1;
        if (plcTries_ > 5) plcTries_ = 5;
        ImGui::SameLine();
        ImGui::SetNextItemWidth(80.0f);
        ImGui::InputFloat("tol mm", &plcTolMM_, 0.0f, 0.0f, "%.4f");
        if (plcTolMM_ < 0.0001f) plcTolMM_ = 0.0001f;

        // No live frame needed when the run teaches one — that is the whole
        // point of align_first, and requiring both would make the button dead
        // on exactly the fresh-app case it exists for.
        const bool canRun = nom && (haveLive || plcAlignFirst_) &&
                            mover && mover->IsConnected() &&
                            jogArmed_ && !orch_.Busy();
        ImGui::BeginDisabled(!canRun);
        if (ImGui::Button("Run guided placement", ImVec2(200.0f, 0.0f)))
            StartPlacementRun(*a, *nom, plcDevice_);
        ImGui::EndDisabled();
        ImGui::SameLine();
        ImGui::BeginDisabled(!running);
        if (ImGui::Button("Stop", ImVec2(70.0f, 0.0f))) orch_.lua.RequestStop();
        ImGui::EndDisabled();

        if (!nom)                                ImGui::TextDisabled("pick a nominal");
        else if (!haveLive && !plcAlignFirst_)
            ImGui::TextDisabled("no live frame - tick 'align first' or run the alignment");
        else if (!mover || !mover->IsConnected())ImGui::TextDisabled("mover not connected");
        else if (!jogArmed_)                     ImGui::TextDisabled("arm first");
        else if (orch_.Busy() && !running)       ImGui::TextDisabled("another run is active");
        else {
            // Say whether the camera will move: with a view taught the run is
            // self-contained, without one the object has to be in front of the
            // lens already, and those are very different instructions.
            const char* view = AlignNominalView(*a, *nom) ? ", drives to the view" : "";
            if (plcAlignFirst_)
                ImGui::TextDisabled("teaches the frame%s, then marks the object", view);
            else
                ImGui::TextDisabled("marks the object%s, corrects, re-measures", view);
        }
        if (!plcStartErr_.empty())
            ImGui::TextColored(kBad, "%s", plcStartErr_.c_str());
        if (running) {
            const LuaEngine::Status lst = orch_.lua.GetStatus();
            ImGui::TextDisabled("%s  %s", LuaEngine::StateName(lst.state),
                                lst.error.empty() ? "" : lst.error.c_str());
        }
    }

    // ---- Measurement --------------------------------------------------------
    ImGui::SeparatorText("Object (measure it now)");
    if (havePose)
        ImGui::TextDisabled("%s  X %.4f  Y %.4f mm", a->device.c_str(), gx, gy);
    else
        ImGui::TextColored(kBad, "%s not connected (or reports no X/Y)", a->device.c_str());
    if (!haveImg) ImGui::TextColored(kBad, "no camera feed");
    ImGui::TextDisabled("Mark two features whose midpoint IS the object's");
    ImGui::TextDisabled("reference point - the same pair the nominal used.");

    const auto capture = [&](int i) {
        AlignPoint p;
        p.gx = gx;
        p.gy = gy;
        p.px = plcPx_;
        p.py = plcPy_;
        p.wx = gx + static_cast<double>(plcPx_ - static_cast<float>(cameraW_) * 0.5f) *
                        static_cast<double>(pixelToMmX_);
        p.wy = gy + static_cast<double>(plcPy_ - static_cast<float>(cameraH_) * 0.5f) *
                        static_cast<double>(pixelToMmY_);
        plcPt_[i]   = p;
        plcHave_[i] = true;
        plcActive_  = 1 - i;
    };

    for (int i = 0; i < 2; ++i) {
        ImGui::PushID(i);
        char btn[32];
        std::snprintf(btn, sizeof(btn), "Set pt%d here", i + 1);
        ImGui::BeginDisabled(!canMark);
        if (ImGui::Button(btn, ImVec2(120.0f, 0.0f))) capture(i);
        ImGui::EndDisabled();
        ImGui::SameLine();
        if (plcHave_[i]) {
            ImGui::Text("%.4f, %.4f", plcPt_[i].wx, plcPt_[i].wy);
            ImGui::SameLine();
            if (ImGui::SmallButton("Clear")) { plcHave_[i] = false; plcActive_ = i; }
        } else {
            ImGui::TextDisabled(plcActive_ == i ? "(not marked - next)" : "(not marked)");
        }
        ImGui::PopID();
    }

    const bool   both  = plcHave_[0] && plcHave_[1];
    const double objMx = (plcPt_[0].wx + plcPt_[1].wx) * 0.5;
    const double objMy = (plcPt_[0].wy + plcPt_[1].wy) * 0.5;

    // ---- Repeatability ------------------------------------------------------
    // Measure the same stationary object several times and read the spread.
    // That spread is the marking error, and it is the floor under every
    // placement: a residual smaller than it is not a miss, and a correction
    // smaller than it is noise being chased. Nothing here is persisted — it is
    // a measurement of the operator and the optics, on this part, right now.
    ImGui::Spacing();
    ImGui::BeginDisabled(!both);
    if (ImGui::SmallButton("Log measure")) {
        plcSamples_.emplace_back(objMx, objMy);
        plcHave_[0] = plcHave_[1] = false;   // an independent sample means re-marking
        plcActive_  = 0;
    }
    ImGui::EndDisabled();
    ImGui::SameLine();
    ImGui::BeginDisabled(plcSamples_.empty());
    if (ImGui::SmallButton("Reset")) plcSamples_.clear();
    ImGui::EndDisabled();
    ImGui::SameLine();
    ImGui::TextDisabled("n = %zu", plcSamples_.size());
    if (plcSamples_.size() >= 2) {
        double loX = plcSamples_[0].first,  hiX = loX;
        double loY = plcSamples_[0].second, hiY = loY;
        double sumX = 0.0, sumY = 0.0;
        for (const auto& [sx, sy] : plcSamples_) {
            loX = sx < loX ? sx : loX;  hiX = sx > hiX ? sx : hiX;
            loY = sy < loY ? sy : loY;  hiY = sy > hiY ? sy : hiY;
            sumX += sx;
            sumY += sy;
        }
        const double n    = static_cast<double>(plcSamples_.size());
        const double meanX = sumX / n, meanY = sumY / n;
        double worst = 0.0;
        for (const auto& [sx, sy] : plcSamples_) {
            const double d = std::sqrt((sx - meanX) * (sx - meanX) +
                                       (sy - meanY) * (sy - meanY));
            worst = d > worst ? d : worst;
        }
        ImGui::TextDisabled("spread  X %.4f  Y %.4f mm   worst %.4f from mean",
                            hiX - loX, hiY - loY, worst);
    } else if (plcSamples_.size() == 1) {
        ImGui::TextDisabled("mark the same two features again and log a second one");
    }

    // ---- The correction -----------------------------------------------------
    ImGui::SeparatorText("Correction");
    // Above the gate on purpose: the marks are cleared by a move, so this is
    // exactly the moment the operator wants to see what was just issued —
    // it is the number the re-measure will be compared against.
    if (!plcLastMove_.empty())
        ImGui::TextWrapped("last: %s", plcLastMove_.c_str());
    if (!both || !nom || !haveLive) {
        ImGui::TextDisabled("needs a live frame, a nominal, and both marks");
        ImGui::EndChild();
        return;
    }

    double objLx = 0.0, objLy = 0.0;
    AlignMachineToLocal(frame, objMx, objMy, &objLx, &objLy);

    ImGui::Text("object   machine  %.4f, %.4f mm", objMx, objMy);
    ImGui::Text("         local    %.4f, %.4f mm", objLx, objLy);

    // The delta. Taken in MACHINE mm as nominal - object: the frame's origin
    // cancels, which is what makes the result meaningful to a device that has
    // never been related to that origin. Shown in local too, because that is
    // the number an operator can compare against the drawing.
    const double dMx = nomMx - objMx;
    const double dMy = nomMy - objMy;
    const double dLx = nom->lx - objLx;
    const double dLy = nom->ly - objLy;
    ImGui::Spacing();
    ImGui::TextColored(kGood, "delta local    %+.4f, %+.4f mm", dLx, dLy);
    ImGui::Text("delta machine  %+.4f, %+.4f mm", dMx, dMy);
    ImGui::Text("distance       %.4f mm", std::sqrt(dMx * dMx + dMy * dMy));

    // ...and the same vector expressed in the mover's own axes. Layer 2 owns
    // this arithmetic (Workflow::GlobalDeltaToAxes) — it is the same call the
    // real move will make, so what the page prints and what it would eventually
    // command cannot drift apart.
    ImGui::Spacing();
    std::vector<std::pair<std::string, double>> axes;
    if (moverIdx < 0 || !workflow_.GlobalDeltaToAxes(static_cast<std::size_t>(moverIdx),
                                                     dMx, dMy, 0.0, &axes)) {
        ImGui::TextColored(kBad, "pick a mover device to map the correction onto its axes");
        ImGui::EndChild();
        return;
    }
    ImGui::SeparatorText("Relative move it would take");
    std::string line;
    for (const auto& [axis, d] : axes) {
        char buf[48];
        std::snprintf(buf, sizeof(buf), "%s%s %+.4f", line.empty() ? "" : "   ",
                      axis.c_str(), d);
        line += buf;
    }
    ImGui::TextColored(kGood, "%s   %s", plcDevice_.c_str(), line.c_str());
    ImGui::TextDisabled("(Z is 0: the frame is 2-D)");

    // The resulting absolute pose, so travel limits can be eyeballed before
    // anybody wires the move.
    if (mover && mover->IsConnected()) {
        const std::map<std::string, double> pos = mover->Positions();
        std::string now, tgt;
        for (const auto& [axis, d] : axes) {
            const auto it = pos.find(axis);
            if (it == pos.end()) continue;
            char b1[48], b2[48];
            std::snprintf(b1, sizeof(b1), "%s%s %.4f", now.empty() ? "" : "   ",
                          axis.c_str(), it->second);
            std::snprintf(b2, sizeof(b2), "%s%s %.4f", tgt.empty() ? "" : "   ",
                          axis.c_str(), it->second + d);
            now += b1;
            tgt += b2;
        }
        ImGui::TextDisabled("from   %s", now.c_str());
        ImGui::TextDisabled("to     %s", tgt.c_str());
    }

    // ---- The move -----------------------------------------------------------
    // Behind the shared arming flag, like the Alignment page's Move here and for
    // the same reason: this destination is COMPUTED, so a mis-taught frame or a
    // mis-marked object sends the stage somewhere nobody chose. Same risk class
    // as click-to-move; a station card's Go, which drives to a pose a human
    // taught, is not.
    ImGui::Separator();
    const double dist = std::sqrt(dMx * dMx + dMy * dMy);
    ImGui::Checkbox("Arm", &jogArmed_);   // the same flag the Jog window arms
    ImGui::SameLine();
    ImGui::SetNextItemWidth(90.0f);
    ImGui::InputFloat("max mm", &plcMaxMM_, 0.0f, 0.0f, "%.2f");
    if (plcMaxMM_ < 0.0f) plcMaxMM_ = 0.0f;

    const bool tooFar    = dist > static_cast<double>(plcMaxMM_);
    const bool canMove   = jogArmed_ && !tooFar && !orch_.Busy() &&
                           mover && mover->IsConnected();
    ImGui::BeginDisabled(!canMove);
    if (ImGui::Button("Move object to nominal", ImVec2(200.0f, 0.0f))) {
        // Layer 2 owns the move: it re-derives the per-axis deltas from the same
        // transform, adds them to the LIVE pose and issues one coordinated
        // command. The page hands over the machine-frame vector and nothing else.
        if (workflow_.MoveRelativeGlobal(static_cast<std::size_t>(moverIdx), dMx, dMy, 0.0)) {
            char buf[160];
            std::snprintf(buf, sizeof(buf),
                          "issued %s: %s   (was %.4f mm off)",
                          plcDevice_.c_str(), line.c_str(), dist);
            plcLastMove_ = buf;
            // Drop the marks. They describe where the object was BEFORE the
            // move, so leaving them would show a correction that is already
            // spent — and a second press would apply it twice. Clearing forces
            // the re-measure that turns this into a closed loop.
            plcHave_[0] = plcHave_[1] = false;
            plcActive_  = 0;
            // The samples were of the object where it USED to be. Keeping them
            // would report the move itself as scatter — the one number the
            // repeatability check must never contain.
            plcSamples_.clear();
        } else {
            plcLastMove_ = "move rejected - see the log";
        }
    }
    ImGui::EndDisabled();

    if (!mover || !mover->IsConnected())
        ImGui::TextDisabled("%s not connected", plcDevice_.c_str());
    else if (tooFar)
        ImGui::TextColored(kBad, "%.4f mm exceeds the %.2f mm ceiling - check the frame\n"
                                 "and the marks, then raise it deliberately",
                           dist, static_cast<double>(plcMaxMM_));
    else if (!jogArmed_)
        ImGui::TextDisabled("arm first");
    else if (orch_.Busy())
        ImGui::TextDisabled("a run is active");
    else
        ImGui::TextDisabled("marks clear after the move - measure again to see the residual");

    ImGui::EndChild();
}

} // namespace firedog
