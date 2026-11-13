// WndPrompts.cpp : the GLOBAL run windows — the recipe/script `prompt`, the
// `manual_adjust` jog window, and the `align_station` crosshair pick.
// Definitions of App members — see App.h.
//
// All are NON-modal and kept always-on-top via BringWindowToDisplayFront (the
// only imgui_internal.h use in the project): only the engine worker blocks on
// the answer, so the rest of the UI stays usable for camera and jog while a run
// waits. All edge-detect a NEW pending request, because a back-to-back one can
// arrive while the window is still shown.
#include "App.h"

#include "imgui.h"
#include "imgui_internal.h"   // BringWindowToDisplayFront (always-on-top windows)

#include "ui/Widgets.h"
#include "ui/Theme.h"   // kTeachOrange (the teach prompt's Save row)
#include "core/Log.h"
#include "vision/VisionModel.h"   // FindVisionModel — the fallback's taught patch

#include <algorithm>
#include <cfloat>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <vector>

namespace firedog {

// The operator's panel switches (core/PanelSwitches.h), from Update() every
// frame whether or not anything is on screen - an edge needs the previous
// level. Reads the driver's cached Status() only (never the hardware), so
// nothing blocks. An OFF -> ON edge is a press: a switch already held ON when a
// prompt appears answers nothing until it is released and pressed again, and a
// device that is offline (or not yet read twice) yields no edge at all.
// Ambiguous pairs answer nothing: confirm + cancel in one frame is ignored, and
// stop + run in one frame is a stop.
const std::string& App::PanelSwName(int k) const
{
    switch (k) {
    case kSwConfirm: return panelSw_.confirm;
    case kSwCancel:  return panelSw_.cancel;
    case kSwRun:     return panelSw_.run;
    default:         return panelSw_.stop;
    }
}

void App::UpdatePanelSwitches()
{
    for (bool& r : panelSwRose_) r = false;
    panelSwFound_  = false;
    panelSwOnline_ = false;
    if (!panelSw_.enabled) return;

    IIoDevice*  d  = nullptr;
    std::size_t di = 0;
    for (std::size_t i = 0; i < hardware_.IoCount(); ++i) {
        IIoDevice* c = hardware_.IoDevice(i);
        if (c && c->Name() == panelSw_.device) { d = c; di = i; break; }
    }
    if (!d) {
        for (PanelSwitchPin& p : panelSwPin_) p = PanelSwitchPin{};
        return;
    }
    panelSwFound_ = true;
    const IIoDevice::IoStatus s = d->Status();
    panelSwOnline_ = hardware_.IoState(di) == ConnState::Connected && s.valid;

    const auto now = std::chrono::steady_clock::now();
    for (int k = 0; k < kSwCount; ++k) {
        PanelSwitchPin& p = panelSwPin_[k];
        const std::string& name = PanelSwName(k);
        const int pin = name.empty() ? -1 : d->InputPinByName(name);
        p.resolved = pin >= 0;
        if (!panelSwOnline_ || pin < 0) {
            p.hasPrev = false;   // a reconnect must not read as an edge
            continue;
        }
        const bool on = ((s.inputs >> pin) & 1ull) != 0;
        const bool rising = p.hasPrev && !p.prev && on;
        p.prev    = on;
        p.hasPrev = true;
        if (!rising) continue;
        if (p.hasEdge &&
            now - p.lastEdge < std::chrono::milliseconds(panelSw_.debounceMs))
            continue;   // bounce, not a second press
        p.lastEdge = now;
        p.hasEdge  = true;
        panelSwRose_[k] = true;
    }
    if (panelSwRose_[kSwConfirm] && panelSwRose_[kSwCancel]) {
        FD_WARN("panel", "%s and %s pressed together - ignored",
                panelSw_.confirm.c_str(), panelSw_.cancel.c_str());
        panelSwRose_[kSwConfirm] = panelSwRose_[kSwCancel] = false;
    }
    if (panelSwRose_[kSwStop]) {
        panelSwRose_[kSwRun] = false;   // stop + run together is a stop
        // The Run page's Stop button, from any page: a stop needs nothing on
        // screen, and the operator reaching for it may be anywhere.
        if (orch_.programs.Active()) {
            FD_INFO("panel", "program stopped by panel switch %s (%s)",
                    panelSw_.stop.c_str(), panelSw_.device.c_str());
            orch_.programs.Stop();
        }
        panelSwRose_[kSwStop] = false;
    }
}

// Global orchestration prompt — drawn every frame from DrawMainWindow, so a
// blocking recipe `prompt` step or script fd.prompt() surfaces no matter which
// page or window the operator is on. Only one engine runs at a time, so at
// most one prompt can be pending. Deliberately NON-modal: only the engine
// worker is blocked, the rest of the UI (jog, camera, click-to-move) stays
// usable — the window is just kept on top so the prompt can't get buried.
void App::DrawRecipePromptWindow()
{
    const RecipeEngine::Status rst = orch_.recipe.GetStatus();
    const LuaEngine::Status    lst = orch_.lua.GetStatus();
    const bool fromScript = rst.pendingPrompt.empty() && !lst.pendingPrompt.empty();
    const std::string& pending = fromScript ? lst.pendingPrompt : rst.pendingPrompt;
    const bool wantsInput =
        fromScript ? lst.pendingPromptInput : rst.pendingPromptInput;
    // > 0: the engine answers OK by itself in this long. The countdown is the
    // engine's, not ours - this window only shows it.
    const int autoOkMs =
        fromScript ? lst.pendingPromptAutoOkMs : rst.pendingPromptAutoOkMs;
    const PromptLevel level =
        fromScript ? lst.pendingPromptLevel : rst.pendingPromptLevel;
    // Non-empty: a CHOICE (ExecutionContext::AwaitChoice) - these buttons
    // replace OK / Cancel, and the answer is the index pressed.
    const std::vector<std::string>& choices =
        fromScript ? lst.pendingPromptButtons : rst.pendingPromptButtons;
    // Picture FILE NAMES (prompt `image1`/`image2`, fd.prompt's 4th argument).
    const std::vector<std::string>& images =
        fromScript ? lst.pendingPromptImages : rst.pendingPromptImages;

    // Edge-detect a NEW prompt (a back-to-back prompt can arrive while the
    // window is still visible, so IsWindowAppearing alone is not enough).
    const bool newPrompt = !pending.empty() && !promptWasPending_;
    promptWasPending_ = !pending.empty();
    // This frame's panel-switch press (UpdatePanelSwitches). Consumed only
    // while a prompt is showing, so it answers the prompt drawn this frame; with
    // none, confirm is left for the Operator Notice drawn after this window
    // (Update rewrites the flags every frame, so nothing is kept for later).
    const bool swHitOk     = panelSwRose_[kSwConfirm];
    const bool swHitCancel = panelSwRose_[kSwCancel];
    if (!pending.empty()) panelSwRose_[kSwConfirm] = panelSwRose_[kSwCancel] = false;
    if (pending.empty()) {   // nothing pending (or stop released it)
        promptSwKey_.clear();
        // The pictures served the prompt that just ended: free them, so the
        // next one re-reads the file (an edited picture shows) and nothing
        // full-size sits on the GPU between prompts.
        for (auto& [path, ic] : promptImageTex_)
            if (ic.tex) host_.DeleteTexture(ic.tex);
        promptImageTex_.clear();
        return;
    }
    if (newPrompt) promptInputBuf_[0] = '\0';
    // When did THIS prompt appear? A different text with no empty frame
    // between (back-to-back prompts) is a new prompt too.
    const auto swNow = std::chrono::steady_clock::now();
    if (newPrompt || pending != promptSwKey_) {
        promptSwKey_   = pending;
        promptShownAt_ = swNow;
    }
    const bool swArmed =
        swNow - promptShownAt_ >= std::chrono::milliseconds(panelSw_.minShownMs);
    if ((swHitOk || swHitCancel) && !swArmed)
        FD_INFO("panel", "panel switch ignored - the prompt appeared %d ms ago",
                static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(
                                     swNow - promptShownAt_).count()));
    const bool swOk     = swArmed && swHitOk;
    const bool swCancel = swArmed && swHitCancel;

    // Operator-facing dialog: it defines its OWN high-contrast palette (opaque
    // near-black body, pure-white text, amber accent frame, dark grey buttons)
    // so it reads identically in every app theme, and runs at 1.5x the app font
    // so it stands out from whatever page is behind it. The buttons are grey
    // rather than a saturated colour on purpose: they are the only things here
    // that can be pressed, and a neutral fill leaves the amber frame as the
    // one accent — a blue slab the width of the box competes with the sentence
    // the operator is meant to read first.
    // The prompt's LEVEL (PromptLevel) picks the title bar, the frame and a
    // banner word above the message - and nothing else. The dark body and the
    // white text stay put at every level, so the contrast argument below holds
    // whatever the author chose; the level is read before the sentence is.
    struct LevelLook {
        const char* title;    // window title (the ID stays "###Recipe Prompt")
        const char* banner;
        ImVec4      bar;      // title bar
        ImVec4      accent;   // frame, separator, banner
    };
    static const LevelLook kLooks[] = {
        {"Information###Recipe Prompt", "INFORMATION",
         ImVec4(0.13f, 0.34f, 0.64f, 1.00f), ImVec4(0.45f, 0.70f, 1.00f, 1.00f)},
        {"Instruction###Recipe Prompt", "INSTRUCTION",
         ImVec4(0.10f, 0.45f, 0.25f, 1.00f), ImVec4(0.35f, 0.88f, 0.50f, 1.00f)},
        {"Warning###Recipe Prompt", "WARNING",
         ImVec4(0.72f, 0.45f, 0.00f, 1.00f), ImVec4(1.00f, 0.76f, 0.25f, 1.00f)},
        {"Error###Recipe Prompt", "ERROR",
         ImVec4(0.62f, 0.10f, 0.10f, 1.00f), ImVec4(1.00f, 0.38f, 0.32f, 1.00f)},
    };
    const int li = static_cast<int>(level);
    const LevelLook& look = kLooks[(li >= 0 && li < 4) ? li : 0];
    const ImVec4 sep(look.accent.x, look.accent.y, look.accent.z, 0.60f);

    const float promptFont = ImGui::GetStyle().FontSizeBase * 1.5f;
    ImGui::PushFont(nullptr, promptFont);
    ImGui::PushStyleColor(ImGuiCol_WindowBg,      ImVec4(0.06f, 0.08f, 0.12f, 1.00f));
    ImGui::PushStyleColor(ImGuiCol_Text,          ImVec4(1.00f, 1.00f, 1.00f, 1.00f));
    ImGui::PushStyleColor(ImGuiCol_TitleBg,       look.bar);
    ImGui::PushStyleColor(ImGuiCol_TitleBgActive, look.bar);
    ImGui::PushStyleColor(ImGuiCol_Border,        look.accent);
    ImGui::PushStyleColor(ImGuiCol_Button,        ImVec4(0.22f, 0.23f, 0.25f, 1.00f));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.34f, 0.35f, 0.38f, 1.00f));
    ImGui::PushStyleColor(ImGuiCol_ButtonActive,  ImVec4(0.14f, 0.15f, 0.17f, 1.00f));
    ImGui::PushStyleColor(ImGuiCol_FrameBg,       ImVec4(0.14f, 0.17f, 0.24f, 1.00f));
    ImGui::PushStyleColor(ImGuiCol_Separator,     sep);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 2.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(18.0f, 14.0f));
    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(14.0f, 8.0f));

    const ImGuiViewport* pvp = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(pvp->GetCenter(), ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
    // WIDE on purpose. This is the window that stops the line: the machine is
    // standing still until somebody reads it, often from a step back at the
    // bench, so it takes a share of the viewport instead of shrink-wrapping
    // the sentence. A size CONSTRAINT rather than SetNextWindowSize, because
    // AlwaysAutoResize must keep owning the height — constraints are applied
    // after the auto-fit, so the window widens to the floor and still grows
    // downward exactly as far as its content needs. It scales off the viewport
    // so the same prompt reads the same on a laptop panel and a bench monitor.
    const float promptW = fminf(fmaxf(pvp->Size.x * 0.55f, 640.0f), 1400.0f);
    ImGui::SetNextWindowSizeConstraints(ImVec2(promptW, 0.0f), ImVec2(FLT_MAX, FLT_MAX));
    ImGui::Begin(look.title, nullptr,
                 ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoCollapse |
                 ImGuiWindowFlags_NoDocking | ImGuiWindowFlags_NoSavedSettings);
    ImGui::BringWindowToDisplayFront(ImGui::GetCurrentWindow());   // always on top
    // The level's banner word, centred in its accent colour, above the message.
    {
        const float avail = ImGui::GetContentRegionAvail().x;
        const float textW = ImGui::CalcTextSize(look.banner).x;
        if (textW < avail)
            ImGui::SetCursorPosX(ImGui::GetCursorPosX() + (avail - textW) * 0.5f);
        ImGui::TextColored(look.accent, "%s", look.banner);
        ImGui::Spacing();
    }
    // Wrap at the window's own edge (0.0f), not at a fixed 560 px: the width is
    // the viewport's to decide now, and a hard-coded wrap would keep breaking
    // the message at the old width inside a wider box.
    // The message sits at the top, centred when it fits on one line, with a
    // block of empty space under it before the input and buttons: the box is
    // taller than its content needs so the sentence stands alone at the top
    // instead of being crowded by the controls. The font is unchanged — it is
    // the room around the text, not its size, that makes it read first.
    {
        const float avail = ImGui::GetContentRegionAvail().x;
        const float textW = ImGui::CalcTextSize(pending.c_str()).x;
        if (textW < avail)
            ImGui::SetCursorPosX(ImGui::GetCursorPosX() + (avail - textW) * 0.5f);
    }
    ImGui::PushTextWrapPos(0.0f);
    ImGui::TextUnformatted(pending.c_str());
    ImGui::PopTextWrapPos();
    // An auto-OK prompt says so, in the frame's amber: silence continues the
    // run, so the operator must be able to see how long Cancel stays open.
    if (autoOkMs > 0) {
        char line[64];
        std::snprintf(line, sizeof(line), "Continues automatically in %d s",
                      (autoOkMs + 999) / 1000);
        const float avail = ImGui::GetContentRegionAvail().x;
        const float textW = ImGui::CalcTextSize(line).x;
        if (textW < avail)
            ImGui::SetCursorPosX(ImGui::GetCursorPosX() + (avail - textW) * 0.5f);
        ImGui::TextColored(ImVec4(1.00f, 0.76f, 0.25f, 1.00f), "%s", line);
    }
    // ---- Visual aid: up to two pictures, side by side under the message ----
    // Each is FITTED into one box - scaled up or down, aspect kept - so a
    // thumbnail is not lost in the window and a 12 MP photo does not push the
    // buttons off the screen. The box: an equal share of the row, at most
    // 640 px wide, and 38 % of the viewport tall (220..520 px). Decoded on the
    // first frame the prompt shows and kept until it ends (promptImageTex_). A
    // file that will not load is drawn as a labelled empty frame, never a
    // refusal: the prompt is still the instruction, the picture only helps.
    if (!images.empty()) {
        namespace fs = std::filesystem;
        const std::size_t n = std::min<std::size_t>(images.size(), 2);
        const float gap   = ImGui::GetStyle().ItemSpacing.x * 2.0f;
        const float avail = ImGui::GetContentRegionAvail().x;
        const float boxW  = fminf(640.0f, (avail - gap * static_cast<float>(n - 1)) /
                                              static_cast<float>(n));
        const float boxH  = fminf(fmaxf(pvp->Size.y * 0.38f, 220.0f), 520.0f);

        struct Pic { const MarkIcon* ic; ImVec2 size; std::string name; };
        std::vector<Pic> pics;
        float rowW = 0.0f;
        for (std::size_t i = 0; i < n; ++i) {
            const fs::path rel(images[i]);
            const std::string path = rel.is_absolute()
                                         ? rel.string()
                                         : (fs::path(session_.PromptImagesDir()) / rel).string();
            auto it = promptImageTex_.find(path);
            if (it == promptImageTex_.end()) {
                MarkIcon ic;
                std::vector<unsigned char> rgba;
                int w = 0, h = 0;
                if (LoadPngRGBA(path.c_str(), &rgba, &w, &h) && w > 0 && h > 0) {
                    ic.tex = host_.MakeTextureRGBA(w, h, rgba.data());
                    ic.w = w;
                    ic.h = h;
                } else {
                    FD_WARN("prompt", "picture '%s' not loaded - shown as missing",
                            path.c_str());
                }
                it = promptImageTex_.emplace(path, ic).first;   // cache the miss too
            }
            const MarkIcon& ic = it->second;
            ImVec2 sz(boxW, boxH * 0.5f);   // a missing picture: a half-height frame
            if (ic.tex) {
                const float k = fminf(boxW / static_cast<float>(ic.w),
                                      boxH / static_cast<float>(ic.h));
                sz = ImVec2(static_cast<float>(ic.w) * k, static_cast<float>(ic.h) * k);
            }
            pics.push_back({&ic, sz, images[i]});
            rowW += sz.x + (i > 0 ? gap : 0.0f);
        }
        ImGui::Spacing();
        if (rowW < avail) ImGui::SetCursorPosX(ImGui::GetCursorPosX() + (avail - rowW) * 0.5f);
        for (std::size_t i = 0; i < pics.size(); ++i) {
            if (i > 0) ImGui::SameLine(0.0f, gap);
            const ImVec2 p0 = ImGui::GetCursorScreenPos();
            const ImVec2 p1(p0.x + pics[i].size.x, p0.y + pics[i].size.y);
            ImDrawList* dl = ImGui::GetWindowDrawList();
            if (pics[i].ic->tex) {
                dl->AddImage((ImTextureID)(intptr_t)pics[i].ic->tex, p0, p1);
                dl->AddRect(p0, p1, ImGui::GetColorU32(look.accent), 0.0f, 0, 1.0f);
            } else {
                dl->AddRect(p0, p1, ImGui::GetColorU32(ImGuiCol_TextDisabled), 0.0f, 0, 1.0f);
                const std::string msg = "picture not found: " + pics[i].name;
                ImGui::PushClipRect(p0, p1, true);
                dl->AddText(ImVec2(p0.x + 8.0f, p0.y + 8.0f),
                            ImGui::GetColorU32(ImGuiCol_TextDisabled), msg.c_str());
                ImGui::PopClipRect();
            }
            ImGui::Dummy(pics[i].size);
        }
    }
    ImGui::Dummy(ImVec2(0.0f, promptFont * 2.0f));   // the breathing room
    const auto answer = [&](bool ok) {
        if (fromScript) orch_.lua.AnswerPrompt(ok, promptInputBuf_);
        else            orch_.recipe.AnswerPrompt(ok, promptInputBuf_);
    };
    bool submit = false;
    if (wantsInput) {
        // Barcode scanners type + Enter, so Enter submits like OK.
        if (newPrompt || ImGui::IsWindowAppearing()) ImGui::SetKeyboardFocusHere();
        ImGui::SetNextItemWidth(-FLT_MIN);   // full width, like everything else here
        // Twice the prompt's own size: what is typed (usually a serial) is the
        // thing the operator checks against the part, and a mistyped digit has
        // to be visible from where they are standing. Pushed around the box
        // only, so the message and the buttons keep their size.
        ImGui::PushFont(nullptr, promptFont * 2.0f);
        submit = ImGui::InputText("##promptin", promptInputBuf_,
                                  sizeof(promptInputBuf_),
                                  ImGuiInputTextFlags_EnterReturnsTrue);
        ImGui::PopFont();
    }
    ImGui::Separator();
    if (!choices.empty()) {
        // The author's buttons share the row equally, at the same double height
        // as OK / Cancel. The FIRST is the primary answer and gets OK's green;
        // the rest keep the window's grey - the same "go on vs. anything else"
        // reading the two-button row gives at a glance.
        const int n = static_cast<int>(choices.size());
        const float sp = ImGui::GetStyle().ItemSpacing.x;
        const float avail = ImGui::GetContentRegionAvail().x;
        const ImVec2 cbtn(fmaxf(promptFont * 4.0f, (avail - sp * (n - 1)) / n),
                          ImGui::GetFrameHeight() * 2.0f);
        // The panel switches do not answer a choice: which of the author's
        // buttons "OK" would mean is the author's to say, not the wiring's.
        if (swOk || swCancel)
            FD_INFO("panel", "panel switch ignored - this prompt asks for a choice");
        for (int i = 0; i < n; ++i) {
            if (i > 0) ImGui::SameLine();
            ImGui::PushID(i);
            if (i == 0) {
                ImGui::PushStyleColor(ImGuiCol_Button,        ImVec4(0.16f, 0.55f, 0.26f, 1.00f));
                ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.22f, 0.68f, 0.34f, 1.00f));
                ImGui::PushStyleColor(ImGuiCol_ButtonActive,  ImVec4(0.12f, 0.42f, 0.20f, 1.00f));
            }
            const bool hit = ImGui::Button(choices[i].c_str(), cbtn);
            if (i == 0) ImGui::PopStyleColor(3);
            ImGui::PopID();
            if (hit) {
                if (fromScript) orch_.lua.AnswerChoice(i);
                else            orch_.recipe.AnswerChoice(i);
            }
        }
        ImGui::End();
        ImGui::PopStyleVar(3);
        ImGui::PopStyleColor(10);
        ImGui::PopFont();
        return;
    }
    // The two answers split the row. They scale with the font as before, but a
    // wide window with two small buttons parked in its corner reads as an empty
    // box — and OK/Cancel are pressed with a glove on, in a hurry, by somebody
    // who has just walked back to the bench.
    const float btnAvail = ImGui::GetContentRegionAvail().x;
    const float btnW = fmaxf(promptFont * 5.0f,
                             (btnAvail - ImGui::GetStyle().ItemSpacing.x) * 0.5f);
    // Double height, and the two answers coloured apart: green says "go on",
    // grey says "no" — a glance tells them apart before the label is read.
    const ImVec2 btn(btnW, ImGui::GetFrameHeight() * 2.0f);
    const float  rowX = ImGui::GetCursorPosX();   // the hint line lines up under it
    ImGui::PushStyleColor(ImGuiCol_Button,        ImVec4(0.16f, 0.55f, 0.26f, 1.00f));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.22f, 0.68f, 0.34f, 1.00f));
    ImGui::PushStyleColor(ImGuiCol_ButtonActive,  ImVec4(0.12f, 0.42f, 0.20f, 1.00f));
    char okLabel[32] = "OK";
    if (autoOkMs > 0)   // "###OK" keeps the ID steady while the number counts down
        std::snprintf(okLabel, sizeof(okLabel), "OK (%d)###OK", (autoOkMs + 999) / 1000);
    // The confirm switch is the OK button, input prompts included: it submits
    // whatever is in the box, exactly as a click would.
    const bool ok = ImGui::Button(okLabel, btn) || submit || swOk;
    ImGui::PopStyleColor(3);
    if (ok) answer(true);
    ImGui::SameLine();
    // Cancel keeps the grey pushed for the whole window above.
    const bool cancel = ImGui::Button("Cancel", btn) || swCancel;
    if (cancel && !ok) answer(false);
    if (swOk || swCancel)
        FD_INFO("panel", "prompt answered %s by panel switch %s (%s)", swOk ? "OK" : "Cancel",
                (swOk ? panelSw_.confirm : panelSw_.cancel).c_str(),
                panelSw_.device.c_str());

    // Under each button, the panel input that presses it - so the operator at
    // the bench knows which switch is which without asking. Dimmed, with the
    // reason, when that switch cannot answer right now.
    if (panelSw_.enabled && panelSwFound_) {
        ImGui::PushFont(nullptr, promptFont * 0.7f);
        const float y  = ImGui::GetCursorPosY();
        const float sp = ImGui::GetStyle().ItemSpacing.x;
        for (int k = 0; k < 2; ++k) {   // kSwConfirm under OK, kSwCancel under Cancel
            ImGui::SetCursorPosY(y);
            DrawPanelSwitchHint(k, rowX + static_cast<float>(k) * (btnW + sp), btnW,
                                ImVec4(0.80f, 0.82f, 0.86f, 1.00f));
        }
        ImGui::PopFont();
    }
    ImGui::End();

    ImGui::PopStyleVar(3);
    ImGui::PopStyleColor(10);
    ImGui::PopFont();
}

// "Switch: <pin>" centred in [cellX, cellX + cellW) on the current line, in
// `col` when the switch can answer, else dimmed with the reason. Nothing when
// that switch is not wired. The caller sets the font and the line's Y.
// `onLight`: darker warning shades, for the notice's yellow background.
void App::DrawPanelSwitchHint(int k, float cellX, float cellW, const ImVec4& col,
                              bool onLight)
{
    const std::string& name = PanelSwName(k);
    if (name.empty()) return;
    std::string line = "Switch: " + name;
    ImVec4 c = col;
    if (!panelSwPin_[k].resolved) {
        line += "  (no such input on " + panelSw_.device + ")";
        c = onLight ? ImVec4(0.60f, 0.08f, 0.04f, 1.00f) : ImVec4(1.00f, 0.55f, 0.45f, 1.00f);
    } else if (!panelSwOnline_) {
        line += "  (" + panelSw_.device + " offline)";
        c = onLight ? ImVec4(0.38f, 0.34f, 0.20f, 1.00f) : ImVec4(0.55f, 0.56f, 0.60f, 1.00f);
    }
    const float textW = ImGui::CalcTextSize(line.c_str()).x;
    ImGui::SetCursorPosX(cellX + fmaxf(0.0f, (cellW - textW) * 0.5f));
    ImGui::TextColored(c, "%s", line.c_str());
}

// The notify_operator window: the machine calling somebody over (UV curing
// complete). YELLOW, so it reads as "come here" and not as the dark prompt that
// asks a question; ONE button, because there is nothing to decide - OK only
// says "I am here", and it is also what silences the buzzer. Non-modal and
// kept on top like the prompt. No App state: the slot lives on Workflow, and
// the window disappears when the command ends it (OK, timeout or Stop).
void App::DrawOperatorNoticeWindow()
{
    const Workflow::OperatorNotice n = workflow_.PendingNotice();
    // The confirm panel switch is this window's OK (the prompt window, drawn
    // first, leaves it alone when it shows nothing). Consumed here either way:
    // a press with nothing to press does nothing.
    const bool swHit = panelSwRose_[kSwConfirm];
    panelSwRose_[kSwConfirm] = false;
    if (n.id == 0) {
        noticeSwId_ = 0;
        return;
    }
    // Armed minShownMs after THIS notice first showed, like a prompt: a press
    // meant for the window before must not acknowledge one nobody has read.
    const auto swNow = std::chrono::steady_clock::now();
    if (n.id != noticeSwId_) {
        noticeSwId_    = n.id;
        noticeShownAt_ = swNow;
    }
    const bool swArmed =
        swNow - noticeShownAt_ >= std::chrono::milliseconds(panelSw_.minShownMs);
    if (swHit && !swArmed)
        FD_INFO("panel", "panel switch ignored - the notice appeared %d ms ago",
                static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(
                                     swNow - noticeShownAt_).count()));
    const bool swOk = swHit && swArmed;

    const float font = ImGui::GetStyle().FontSizeBase * 1.5f;
    const ImVec4 yellow(1.00f, 0.85f, 0.10f, 1.00f);
    const ImVec4 ink(0.08f, 0.07f, 0.02f, 1.00f);   // near-black on the yellow
    ImGui::PushFont(nullptr, font);
    ImGui::PushStyleColor(ImGuiCol_WindowBg,      yellow);
    ImGui::PushStyleColor(ImGuiCol_Text,          ink);
    ImGui::PushStyleColor(ImGuiCol_TitleBg,       ImVec4(0.80f, 0.62f, 0.00f, 1.00f));
    ImGui::PushStyleColor(ImGuiCol_TitleBgActive, ImVec4(0.80f, 0.62f, 0.00f, 1.00f));
    ImGui::PushStyleColor(ImGuiCol_Border,        ink);
    ImGui::PushStyleColor(ImGuiCol_Separator,     ImVec4(0.08f, 0.07f, 0.02f, 0.45f));
    ImGui::PushStyleColor(ImGuiCol_Button,        ImVec4(0.16f, 0.55f, 0.26f, 1.00f));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.22f, 0.68f, 0.34f, 1.00f));
    ImGui::PushStyleColor(ImGuiCol_ButtonActive,  ImVec4(0.12f, 0.42f, 0.20f, 1.00f));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 2.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(18.0f, 14.0f));
    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(14.0f, 8.0f));

    const ImGuiViewport* vp = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(vp->GetCenter(), ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
    const float w = fminf(fmaxf(vp->Size.x * 0.45f, 560.0f), 1200.0f);
    ImGui::SetNextWindowSizeConstraints(ImVec2(w, 0.0f), ImVec2(FLT_MAX, FLT_MAX));
    ImGui::Begin("Operator Notice", nullptr,
                 ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoCollapse |
                 ImGuiWindowFlags_NoDocking | ImGuiWindowFlags_NoSavedSettings);
    ImGui::BringWindowToDisplayFront(ImGui::GetCurrentWindow());   // always on top

    {
        const float avail = ImGui::GetContentRegionAvail().x;
        const float textW = ImGui::CalcTextSize(n.message.c_str()).x;
        if (textW < avail)
            ImGui::SetCursorPosX(ImGui::GetCursorPosX() + (avail - textW) * 0.5f);
    }
    ImGui::PushTextWrapPos(0.0f);
    ImGui::TextUnformatted(n.message.c_str());
    ImGui::PopTextWrapPos();

    // How long the buzzer keeps calling - the command's deadline, shown only.
    {
        const long long left = std::chrono::duration_cast<std::chrono::seconds>(
                                   n.deadline - std::chrono::steady_clock::now()).count();
        const long long s = left > 0 ? left : 0;
        char line[64];
        std::snprintf(line, sizeof(line), "Waiting for OK - %lld:%02lld left", s / 60, s % 60);
        ImGui::PushFont(nullptr, font * 0.7f);
        const float avail = ImGui::GetContentRegionAvail().x;
        const float textW = ImGui::CalcTextSize(line).x;
        if (textW < avail)
            ImGui::SetCursorPosX(ImGui::GetCursorPosX() + (avail - textW) * 0.5f);
        ImGui::TextUnformatted(line);
        ImGui::PopFont();
    }
    ImGui::Dummy(ImVec2(0.0f, font));
    ImGui::Separator();
    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 1.0f, 1.0f, 1.0f));   // white on green
    const float rowX = ImGui::GetCursorPosX();
    const float btnW = ImGui::GetContentRegionAvail().x;
    const bool  click = ImGui::Button("OK", ImVec2(-FLT_MIN, ImGui::GetFrameHeight() * 2.0f));
    ImGui::PopStyleColor();
    if (click || swOk) workflow_.AckNotice(n.id);
    if (swOk && !click)
        FD_INFO("panel", "notice acknowledged by panel switch %s (%s)",
                panelSw_.confirm.c_str(), panelSw_.device.c_str());
    // Which panel switch presses OK, as under the prompt's buttons.
    if (panelSw_.enabled && panelSwFound_) {
        ImGui::PushFont(nullptr, font * 0.7f);
        DrawPanelSwitchHint(kSwConfirm, rowX, btnW, ink, true);
        ImGui::PopFont();
    }
    ImGui::End();

    ImGui::PopStyleVar(3);
    ImGui::PopStyleColor(9);
    ImGui::PopFont();
}

// The manual_adjust window (bobcat's ManualAdjustWidget): the engine worker
// blocks in AwaitAdjustInternal while the operator jogs the device from here.
// Jogs go through Workflow::JogGlobal exactly like the Jog window (transform-
// mapped, safety-inhibited); positions stay fresh via RequestFastPoll.
// NON-modal on purpose: while adjusting, the operator still needs the rest of
// the app (camera view, Jog window, click-to-move) — the window is only kept
// on top, it doesn't block input to anything else.
void App::DrawManualAdjustWindow()
{
    const RecipeEngine::Status rst = orch_.recipe.GetStatus();
    const LuaEngine::Status    lst = orch_.lua.GetStatus();
    const bool fromScript = !rst.pendingAdjust && lst.pendingAdjust;
    const bool pending    = rst.pendingAdjust || lst.pendingAdjust;
    const ExecutionContext::AdjustRequest& req =
        fromScript ? lst.adjustReq : rst.adjustReq;

    const bool newAdjust = pending && !adjustWasPending_;
    adjustWasPending_ = pending;
    if (!pending) return;   // nothing pending (or stop released it)
    if (newAdjust) {
        // Seed the preset selection from the step's step_um (closest match).
        adjustStepIdx_ = 0;
        const double stepMm = req.stepUm / 1000.0;
        for (int i = 1; i < kStepCount; ++i) {
            if (std::fabs(kSteps[i].mm - stepMm) <
                std::fabs(kSteps[adjustStepIdx_].mm - stepMm))
                adjustStepIdx_ = i;
        }
    }
    // Larger than the body text: the operator reads it from the jog position,
    // a step back from the screen. Every fixed width below scales with it.
    const float k = 1.4f;
    ImGui::PushFont(nullptr, ImGui::GetStyle().FontSizeBase * k);
    ImGui::SetNextWindowPos(ImGui::GetMainViewport()->GetCenter(),
                            ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
    ImGui::Begin("Manual Adjust", nullptr,
                 ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoCollapse |
                 ImGuiWindowFlags_NoDocking | ImGuiWindowFlags_NoSavedSettings);
    ImGui::BringWindowToDisplayFront(ImGui::GetCurrentWindow());   // always on top

    // Header: the message on the left, and on the right the DEVICE this prompt
    // is about, big, on a solid block. Two hexapods and a gantry share this
    // window, and jogging the wrong one is the mistake the box exists to
    // prevent - the small "Device:" line below was not enough from a step back.
    {
        const ImGuiStyle& style = ImGui::GetStyle();
        const float boxW = 220.0f * k;
        const float boxH = ImGui::GetTextLineHeight() * 3.0f + style.WindowPadding.y * 2.0f;
        const float textW = 300.0f * k;
        ImGui::BeginGroup();
        if (req.teach)
            ImGui::TextColored(kTeachOrange, req.untaught ? "TEACH  %s  (NOT TAUGHT HERE)"
                                                          : "TEACH  %s",
                               req.positionName.c_str());
        ImGui::PushTextWrapPos(ImGui::GetCursorPos().x + textW);
        ImGui::TextUnformatted(req.message.c_str());
        ImGui::PopTextWrapPos();
        ImGui::EndGroup();
        ImGui::SameLine(textW + style.ItemSpacing.x * 2.0f);
        // Orange for a teach (it matches the Save button it leads to), the
        // arm's amber for a plain manual_adjust; dark text on both for contrast.
        ImGui::PushStyleColor(ImGuiCol_ChildBg,
                              req.teach ? kTeachOrange : ImVec4(1.0f, 0.75f, 0.20f, 1.0f));
        ImGui::BeginChild("adjdev", ImVec2(boxW, boxH), ImGuiChildFlags_None,
                          ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
        DrawBigLabel(req.device.c_str(),
                     ImVec2(boxW - style.WindowPadding.x * 2.0f, boxH - style.WindowPadding.y * 2.0f),
                     ImVec4(0.08f, 0.08f, 0.08f, 1.0f));
        ImGui::EndChild();
        ImGui::PopStyleColor();
    }
    ImGui::Separator();

    // Resolve the device each frame (cheap; the command validated it exists).
    int di = -1;
    for (std::size_t i = 0; i < hardware_.MotionCount(); ++i) {
        if (hardware_.MotionName(i) == req.device) { di = static_cast<int>(i); break; }
    }
    IMotionController* dev =
        di >= 0 ? hardware_.MotionDevice(static_cast<std::size_t>(di)) : nullptr;

    ImGui::TextDisabled("Device: %s", req.device.c_str());
    if (dev) {
        dev->RequestFastPoll();   // live readout + accurate relative jogs
        std::string line;
        for (const auto& [axis, val] : dev->Positions()) {
            char buf[48];
            std::snprintf(buf, sizeof(buf), "%s%s %.4f", line.empty() ? "" : "  ",
                          axis.c_str(), val);
            line += buf;
        }
        ImGui::TextUnformatted(line.empty() ? "(no position data)" : line.c_str());
        // A teach prompt shows what Save would replace, and by how much the
        // operator has moved off it - a wrong step size shows up as a number.
        if (req.teach && !req.target.empty()) {
            const std::map<std::string, double> live = dev->Positions();
            std::string saved, delta;
            for (const auto& [axis, val] : req.target) {
                char buf[48];
                std::snprintf(buf, sizeof(buf), "%s%s %.4f", saved.empty() ? "" : "  ",
                              axis.c_str(), val);
                saved += buf;
                const auto li = live.find(axis);
                if (li == live.end()) continue;
                std::snprintf(buf, sizeof(buf), "%s%s %+.4f", delta.empty() ? "" : "  ",
                              axis.c_str(), li->second - val);
                delta += buf;
            }
            // A placeholder's numbers are another machine's: shown as such, so
            // the delta reads as "how far from the master", never as a target.
            ImGui::TextDisabled(req.untaught ? "master %s" : "saved  %s", saved.c_str());
            if (!delta.empty()) ImGui::TextColored(kTeachOrange, "delta  %s", delta.c_str());
        }
    }
    ImGui::Spacing();

    // Step presets (same table as the Jog window).
    ImGui::TextUnformatted("Step");
    ImGui::SameLine();
    for (int i = 0; i < kStepCount; ++i) {
        if (i % 8 != 0) ImGui::SameLine();
        const bool sel = (adjustStepIdx_ == i);
        if (sel) ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.20f, 0.50f, 0.85f, 1.0f));
        if (ImGui::SmallButton(kSteps[i].label)) adjustStepIdx_ = i;
        if (sel) ImGui::PopStyleColor();
    }
    ImGui::Spacing();

    // Jog row (global frame, like the Jog window; per-axis enables from the
    // step). Workflow inhibits motion during a critical safety alert anyway —
    // the visual disable just says so up front.
    const double step = kSteps[adjustStepIdx_].mm;
    // Same critical-alert query the engines use (SafetyBlocked is private).
    const bool safetyBlocked =
        workflow_.SafetyAlertActive() && workflow_.SafetyAlert().critical;
    const bool canJog = dev && !safetyBlocked;
    const std::size_t udi = static_cast<std::size_t>(di < 0 ? 0 : di);
    const ImVec2 jb(52.0f * k, 0.0f);
    ImGui::BeginDisabled(!canJog || !req.enableX);
    if (ImGui::Button("X-", jb)) workflow_.JogGlobal(udi, -1.0f, 0.0f, 0.0f, step);
    ImGui::SameLine();
    if (ImGui::Button("X+", jb)) workflow_.JogGlobal(udi, 1.0f, 0.0f, 0.0f, step);
    ImGui::EndDisabled();
    ImGui::SameLine();
    ImGui::BeginDisabled(!canJog || !req.enableY);
    if (ImGui::Button("Y-", jb)) workflow_.JogGlobal(udi, 0.0f, -1.0f, 0.0f, step);
    ImGui::SameLine();
    if (ImGui::Button("Y+", jb)) workflow_.JogGlobal(udi, 0.0f, 1.0f, 0.0f, step);
    ImGui::EndDisabled();
    ImGui::SameLine();
    ImGui::BeginDisabled(!canJog || !req.enableZ);
    if (ImGui::Button("Z-", jb)) workflow_.JogGlobal(udi, 0.0f, 0.0f, -1.0f, step);
    ImGui::SameLine();
    if (ImGui::Button("Z+", jb)) workflow_.JogGlobal(udi, 0.0f, 0.0f, 1.0f, step);
    ImGui::EndDisabled();
    ImGui::SameLine();
    ImGui::PushStyleColor(ImGuiCol_Button,        ImVec4(0.75f, 0.20f, 0.20f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.88f, 0.26f, 0.26f, 1.0f));
    if (ImGui::Button("STOP", jb) && di >= 0) workflow_.StopMotion(udi);
    ImGui::PopStyleColor(2);
    if (safetyBlocked)
        ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.3f, 1.0f),
                           "Safety alert active - motion inhibited");
    ImGui::Separator();

    const auto answer = [&](bool ok) {
        if (fromScript) orch_.lua.AnswerAdjust(ok);
        else            orch_.recipe.AnswerAdjust(ok);
    };
    if (req.teach) {
        // Same bool channel; different words, because the answer means
        // something else: true writes the position, false keeps it and the
        // run goes on. Stop (the run strip) is the only abort.
        ImGui::PushStyleColor(ImGuiCol_Button,        kTeachOrange);
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, kTeachOrangeHov);
        ImGui::PushStyleColor(ImGuiCol_ButtonActive,  kTeachOrangeAct);
        // 170 px: "Skip (keep saved)" clipped at the 140 the OK/Cancel pair use.
        if (ImGui::Button("Save position", ImVec2(170.0f * k, 0.0f))) answer(true);
        ImGui::PopStyleColor(3);
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Write the device's current pose into '%s'.",
                              req.positionName.c_str());
        ImGui::SameLine();
        if (ImGui::Button(req.untaught ? "Skip (leave untaught)" : "Skip (keep saved)",
                          ImVec2(170.0f * k, 0.0f)))
            answer(false);
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip(req.untaught ? "Leave '%s' untaught and continue - every run\n"
                                             "refuses it until it is taught."
                                           : "Leave '%s' as it is and continue.",
                              req.positionName.c_str());
        ImGui::SameLine();
        // Done teaching: this position and every one after it keep their
        // saved values, and the run ends HERE - nothing else moves. It is the
        // run's own stop (program queue + recipe engine; the stop releases
        // this wait as a skip), NOT the global STOP: the device is parked at a
        // prompt, so there is no motion to halt and no alarm to raise. A teach
        // run writes no $run and no deck mark, so ending it early leaves no
        // record behind - the strip shows it as Stopped.
        if (ImGui::Button("Skip all (end teach)", ImVec2(200.0f * k, 0.0f))) {
            FD_INFO("program", "teach: skip all at '%s' - remaining positions kept, run ended",
                    req.positionName.c_str());
            if (fromScript) {
                orch_.lua.RequestStop();
            } else {
                orch_.programs.Stop();
                orch_.recipe.RequestStop();
            }
        }
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Keep '%s' and every remaining position as saved,\n"
                              "and end the teach run now. Nothing else moves.",
                              req.positionName.c_str());
    } else {
        if (ImGui::Button("OK (Complete)", ImVec2(140.0f * k, 0.0f))) answer(true);
        ImGui::SameLine();
        if (ImGui::Button("Cancel (Fail Step)", ImVec2(140.0f * k, 0.0f))) answer(false);
    }
    ImGui::End();
    ImGui::PopFont();
}

// The align_station crosshair pick: the engine worker blocks in
// AwaitPickInternal while the operator marks a feature on the live image.
// MARKING never moves anything — the answer is a point in the current field of
// view. One thing here does: a RIGHT click drives the gantry so the clicked
// feature lands under the centre, which is behind the shared arming flag. The
// feature exists because the thing to mark is often outside a 3 mm field of
// view, and hunting for it with the Jog window while a run waits is worse.
//
// The pixel->mm conversion happens HERE, not in the command: only the UI knows
// the live pixelToMm calibration. The crosshair starts at the image centre, so
// OK without a click means "the feature is dead centre" (offset 0,0).
void App::DrawAlignPickWindow()
{
    const RecipeEngine::Status rst = orch_.recipe.GetStatus();
    const LuaEngine::Status    lst = orch_.lua.GetStatus();
    const bool fromScript = !rst.pendingPick && lst.pendingPick;
    const bool pending    = rst.pendingPick || lst.pendingPick;
    const ExecutionContext::PickRequest& req = fromScript ? lst.pickReq : rst.pickReq;

    const bool newPick = pending && !pickWasPending_;
    pickWasPending_ = pending;
    if (!pending) return;   // nothing pending (or stop released it)

    UpdateCameraTexture();   // pull the newest camera frame into the GL texture
    const bool haveImg = cameraTex_ && cameraW_ > 0 && cameraH_ > 0;
    if (newPick) {   // start dead centre: OK with no click means "no offset"
        pickPx_ = haveImg ? static_cast<float>(cameraW_) * 0.5f : 0.0f;
        pickPy_ = haveImg ? static_cast<float>(cameraH_) * 0.5f : 0.0f;
    }

    ImGui::SetNextWindowPos(ImGui::GetMainViewport()->GetCenter(),
                            ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
    ImGui::SetNextWindowSize(ImVec2(900.0f, 720.0f), ImGuiCond_FirstUseEver);
    // BringWindowToDisplayFront only changes DRAW ORDER, not focus — without
    // this the window comes up on top but the keyboard still belongs to
    // whatever had it, so the arrow-key nudge would do nothing until the
    // operator happened to click the window.
    if (newPick) ImGui::SetNextWindowFocus();
    // Same window, same handshake, two vocabularies: a frame teach marks E
    // points, a placement marks points on an object. The ### id is fixed so it
    // stays ONE window whichever asked.
    char title[160];
    if (req.label.empty())
        std::snprintf(title, sizeof(title), "Alignment - mark E%d###AlignPick", req.station);
    else
        std::snprintf(title, sizeof(title), "Mark %s###AlignPick", req.label.c_str());
    ImGui::Begin(title, nullptr,
                 ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoDocking |
                 ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoNavInputs |
                 ImGuiWindowFlags_NoScrollWithMouse);   // the wheel zooms, not scrolls
    ImGui::BringWindowToDisplayFront(ImGui::GetCurrentWindow());   // always on top

    ImGui::TextUnformatted(req.message.c_str());
    char what[64];
    if (req.label.empty()) std::snprintf(what, sizeof(what), "E%d", req.station);
    else                   std::snprintf(what, sizeof(what), "%s", req.label.c_str());
    ImGui::TextDisabled("%s  -  %s  -  left click places, RIGHT click drives there "
                        "(arm first), arrows nudge 1 px (Shift = 10), wheel zooms",
                        req.alignName.c_str(), what);

    // The FALLBACK banner. This window is also the ordinary manual pick, so it
    // only appears when detection was tried and failed — and then it is the
    // most important thing on screen. The operator at the machine is rarely the
    // person who tuned the model: without its taught patch they are marking
    // their own reading of "the corner", and the wrong corner of the same
    // feature is a frame wrong by the whole feature, with nothing on screen
    // saying so.
    if (!req.autoError.empty()) {
        ImGui::TextColored(ImVec4(1.0f, 0.7f, 0.3f, 1.0f),
                           "detection failed - mark %s by hand:", what);
        ImGui::SameLine();
        ImGui::TextWrapped("%s", req.autoError.c_str());

        const VisionModel* fbModel =
            req.detectorUid.empty()
                ? nullptr
                : FindVisionModel(session_.VisionModels(), req.detectorUid);
        if (fbModel && EnsurePatchTex(pickTmpl_, *fbModel)) {
            ImGui::BeginGroup();
            DrawPatchThumb(pickTmpl_, 110.0f, 110.0f);
            ImGui::EndGroup();
            ImGui::SameLine();
            ImGui::BeginGroup();
            ImGui::TextUnformatted("This is what the detector was looking for.");
            ImGui::TextDisabled("Put the crosshair where the cross sits on the patch -");
            ImGui::TextDisabled("that is the point this procedure was taught with.");
            ImGui::EndGroup();
        } else if (fbModel) {
            // A blob method matches a SHAPE, so there is no picture to show;
            // saying which point it reports still beats saying nothing.
            ImGui::TextDisabled("'%s' (%s) reports the %s - mark that point.",
                                fbModel->name.c_str(), fbModel->method.c_str(),
                                fbModel->method == "circle" ? "centre of the circle"
                                                            : "corner of the square");
        }
    }
    ImGui::Separator();

    // Footer height reserved so the image never pushes the buttons off-window.
    // The button row is twice as tall when it leads with the step diagrams.
    const float footerH = ImGui::GetFrameHeightWithSpacing() * 2.0f + 8.0f +
                          (req.stepIcons ? ImGui::GetFrameHeight() : 0.0f) +
                          // the guard band's two readout lines, when there is one
                          (req.band.have ? ImGui::GetTextLineHeightWithSpacing() * 2.0f
                                         : 0.0f) +
                          // the live-residual readout: two lines and the focus row
                          (req.target.have ? ImGui::GetTextLineHeightWithSpacing() * 2.0f +
                                                 ImGui::GetFrameHeightWithSpacing()
                                           : 0.0f);
    const ImVec2 origin = ImGui::GetCursorScreenPos();
    ImVec2 avail = ImGui::GetContentRegionAvail();
    avail.y = avail.y > footerH + 40.0f ? avail.y - footerH : 40.0f;
    ImDrawList* dl = ImGui::GetWindowDrawList();

    // ---- View transform: screen px per CAMERA px, and which camera-px window
    // is on screen. Shared with the vision teach window (ui/Widgets.h) so a
    // pixel means the same thing in both surfaces: a cover fit times a zoom,
    // with the window following the MARK so a zoomed view can never hide the
    // crosshair and the arrow-key nudge always stays visible.
    const CameraView view = MakeCameraView(origin, avail, cameraW_, cameraH_,
                                           pickPx_, pickPy_, pickZoom_);
    const auto toScreen = [&](float px, float py) { return view.ToScreen(px, py); };

    if (haveImg) {
        ImGui::Image((ImTextureID)(intptr_t)cameraTex_, avail, view.Uv0(), view.Uv1());
        DrawZoomBadge(dl, view, pickZoom_);
    } else {
        dl->AddRectFilled(origin, ImVec2(origin.x + avail.x, origin.y + avail.y),
                          IM_COL32(18, 18, 22, 255));
        const char* msg = "no camera feed - cannot mark a point";
        const ImVec2 ts = ImGui::CalcTextSize(msg);
        dl->AddText(ImVec2(origin.x + (avail.x - ts.x) * 0.5f,
                           origin.y + (avail.y - ts.y) * 0.5f),
                    IM_COL32(220, 140, 140, 255), msg);
        ImGui::Dummy(avail);
    }

    ImGui::SetCursorScreenPos(origin);
    const bool imgClicked = ImGui::InvisibleButton("##pickInput", avail);
    const bool imgHovered = ImGui::IsItemHovered();

    if (haveImg && imgClicked)
        view.ToCamera(ImGui::GetIO().MousePos, &pickPx_, &pickPy_);

    // RIGHT click drives the gantry so the clicked feature lands under the
    // centre. This is the one thing in this window that MOVES, so it is behind
    // the shared arming flag — click-to-move is named in the rules for exactly
    // this reason, and the window is otherwise safe with jog disarmed.
    //
    // No orch_.Busy() gate, deliberately: this window only ever exists DURING a
    // run, and being able to bring a feature into view is the whole point of
    // asking a human. manual_adjust jogs mid-run on the same argument.
    //
    // The crosshair follows to the centre, because that is where the clicked
    // feature is going. Leaving it on its old PIXEL would be wrong in a way
    // nothing on screen shows: after the move, that pixel looks somewhere else,
    // and OK would capture a point off by the whole move.
    if (haveImg && imgHovered && ImGui::IsMouseClicked(ImGuiMouseButton_Right) &&
        pixelToMmX_ > 0.0f && pixelToMmY_ > 0.0f) {
        float cx = 0.0f, cy = 0.0f;
        view.ToCamera(ImGui::GetIO().MousePos, &cx, &cy);
        const double mdx = static_cast<double>(cx - static_cast<float>(cameraW_) * 0.5f) *
                           static_cast<double>(pixelToMmX_);
        const double mdy = static_cast<double>(cy - static_cast<float>(cameraH_) * 0.5f) *
                           static_cast<double>(pixelToMmY_);
        if (jogArmed_) {
            workflow_.MoveGantryRelativeMM(mdx, mdy);
            pickPx_ = static_cast<float>(cameraW_) * 0.5f;
            pickPy_ = static_cast<float>(cameraH_) * 0.5f;
        } else {
            FD_WARN("align", "click-to-move ignored - tick Arm in the pick window");
        }
    }
    // Wheel zoom over the image. Applies from the NEXT frame (the view was
    // already computed above) — a frame of latency nobody can see.
    if (haveImg && imgHovered && ImGui::GetIO().MouseWheel != 0.0f) {
        pickZoom_ *= (ImGui::GetIO().MouseWheel > 0.0f) ? 1.25f : 0.8f;
        pickZoom_ = fminf(fmaxf(pickZoom_, 1.0f), 12.0f);
    }
    // Refinement without a zoom: one press = one CAMERA pixel (hold to repeat,
    // Shift for 10). Gated on focus-or-hover rather than taken globally — the
    // window is non-modal, so an arrow press aimed elsewhere must stay there;
    // hover counts because the mouse is naturally over the image while marking.
    const bool keysHere =
        ImGui::IsWindowFocused(ImGuiFocusedFlags_RootAndChildWindows) ||
        ImGui::IsWindowHovered(ImGuiHoveredFlags_RootAndChildWindows |
                               ImGuiHoveredFlags_AllowWhenBlockedByActiveItem);
    if (haveImg && keysHere) {
        const float nudge = ImGui::GetIO().KeyShift ? 10.0f : 1.0f;
        if (ImGui::IsKeyPressed(ImGuiKey_LeftArrow, true))  pickPx_ -= nudge;
        if (ImGui::IsKeyPressed(ImGuiKey_RightArrow, true)) pickPx_ += nudge;
        if (ImGui::IsKeyPressed(ImGuiKey_UpArrow, true))    pickPy_ -= nudge;
        if (ImGui::IsKeyPressed(ImGuiKey_DownArrow, true))  pickPy_ += nudge;
        pickPx_ = fminf(fmaxf(pickPx_, 0.0f), static_cast<float>(cameraW_));
        pickPy_ = fminf(fmaxf(pickPy_, 0.0f), static_cast<float>(cameraH_));
    }

    const float dxPix = pickPx_ - static_cast<float>(cameraW_) * 0.5f;
    const float dyPix = pickPy_ - static_cast<float>(cameraH_) * 0.5f;
    const double dxMM = static_cast<double>(dxPix) * pixelToMmX_;
    const double dyMM = static_cast<double>(dyPix) * pixelToMmY_;

    // ---- The guard band ----------------------------------------------------
    // Set only for the E2 of a procedure that has a band: E1 is already
    // captured, so where E2 is ALLOWED to be is known, and it is a region on
    // this image. The verdict is recomputed every frame from the crosshair, so
    // moving the mark is the re-try — there is no loop to run and no second
    // question to answer. Both the pose and the calibration are needed to
    // project it; without them the band is simply not shown, exactly like a hint.
    double gxNow = 0.0, gyNow = 0.0;
    const bool havePose = workflow_.GantryXY(gxNow, gyNow) &&
                          pixelToMmX_ > 0.0f && pixelToMmY_ > 0.0f;
    const bool bandOn = req.band.have && havePose;
    double bandSpan = 0.0, bandTheta = 0.0, bandSpanErr = 0.0, bandThetaErr = 0.0;
    bool   bandOk = true;
    if (bandOn) {
        AlignPoint e1, e2;
        e1.wx = req.band.cx;
        e1.wy = req.band.cy;
        e2.wx = gxNow + dxMM;
        e2.wy = gyNow + dyMM;
        AlignSpanTheta(e1, e2, &bandSpan, &bandTheta);
        AlignGuard g;
        g.enabled     = true;
        g.spanMm      = req.band.spanRef;
        g.spanTolMm   = req.band.spanTol;
        g.thetaDeg    = req.band.thetaRef;
        g.thetaTolDeg = req.band.thetaTol;
        bandOk = AlignGuardCheck(g, bandSpan, bandTheta, &bandSpanErr, &bandThetaErr);
    }

    if (haveImg) {
        // Clip the overlay to the image box: zoomed in, the image-centre
        // reference can sit outside the visible window entirely.
        dl->PushClipRect(origin, ImVec2(origin.x + avail.x, origin.y + avail.y), true);

        // Everything drawn from MACHINE mm goes through this: the guard band
        // and the run's reference points alike. Built once, because two copies
        // of the pose and calibration is how they start disagreeing.
        WorldMarkView wv;
        wv.view       = view;
        wv.boxMin     = origin;
        wv.boxMax     = ImVec2(origin.x + avail.x, origin.y + avail.y);
        wv.gx         = gxNow;
        wv.gy         = gyNow;
        wv.pixelToMmX = pixelToMmX_;
        wv.pixelToMmY = pixelToMmY_;
        wv.imgW       = cameraW_;
        wv.imgH       = cameraH_;

        // The band FIRST, under everything: it is a region to aim inside, not a
        // mark, and it must never sit on top of the crosshair being placed. It
        // turns orange the moment the mark leaves it, so the colour answers
        // "am I inside?" without reading the numbers below.
        if (bandOn) {
            WorldBand wb;
            wb.have = true;
            wb.cx   = req.band.cx;
            wb.cy   = req.band.cy;
            wb.rMin = req.band.spanRef - req.band.spanTol;
            wb.rMax = req.band.spanRef + req.band.spanTol;
            wb.aMin = req.band.thetaRef - req.band.thetaTol;
            wb.aMax = req.band.thetaRef + req.band.thetaTol;
            wb.col  = bandOk ? IM_COL32(90, 230, 130, 200)
                             : IM_COL32(255, 150, 60, 220);
            DrawWorldBand(dl, wv, wb);
        }

        // The optical/stage reference: image centre, wherever it now maps to.
        const ImVec2 ctr = toScreen(static_cast<float>(cameraW_) * 0.5f,
                                    static_cast<float>(cameraH_) * 0.5f);
        dl->AddLine(ImVec2(origin.x, ctr.y), ImVec2(origin.x + avail.x, ctr.y),
                    IM_COL32(80, 230, 120, 140), 1.0f);
        dl->AddLine(ImVec2(ctr.x, origin.y), ImVec2(ctr.x, origin.y + avail.y),
                    IM_COL32(80, 230, 120, 140), 1.0f);

        const ImVec2 mk = toScreen(pickPx_, pickPy_);
        const ImU32 markCol = IM_COL32(255, 200, 60, 255);
        const float mh = 22.0f;
        dl->AddLine(ImVec2(mk.x - mh, mk.y), ImVec2(mk.x + mh, mk.y), markCol, 2.0f);
        dl->AddLine(ImVec2(mk.x, mk.y - mh), ImVec2(mk.x, mk.y + mh), markCol, 2.0f);
        dl->AddCircle(mk, 5.0f, markCol, 0, 2.0f);

        char lbl[128];
        std::snprintf(lbl, sizeof(lbl), "px %.1f, %.1f   dX %+.4f mm   dY %+.4f mm",
                      pickPx_, pickPy_, dxMM, dyMM);
        const ImVec2 lts = ImGui::CalcTextSize(lbl);
        ImVec2 lp(mk.x + 16.0f, mk.y + 10.0f);
        lp.x = fminf(lp.x, origin.x + avail.x - lts.x - 8.0f);
        lp.y = fminf(lp.y, origin.y + avail.y - lts.y - 6.0f);
        dl->AddRectFilled(ImVec2(lp.x - 4.0f, lp.y - 2.0f),
                          ImVec2(lp.x + lts.x + 4.0f, lp.y + lts.y + 2.0f),
                          IM_COL32(0, 0, 0, 170), 3.0f);
        dl->AddText(lp, IM_COL32(255, 235, 150, 255), lbl);

        // The run's own reference points: the nominal being aimed at, and any
        // mark already taken. Projected through the LIVE gantry pose, so they
        // stay on their features as the stage moves — and an off-screen one
        // becomes an arrow on the edge with its distance, which is the only
        // thing that distinguishes "the target is over there" from "the target
        // is where I am looking".
        if (!req.hints.empty() && havePose) {
            for (const ExecutionContext::PickHint& h : req.hints) {
                CameraMark cm;
                cm.wx    = h.wx;
                cm.wy    = h.wy;
                cm.have  = true;
                cm.label = h.label.c_str();
                // The nominal reads as the target, not as another mark: it is
                // the only one here the operator is NOT placing.
                cm.col   = (&h == &req.hints.front()) ? IM_COL32(255, 90, 90, 255)
                                                      : IM_COL32(120, 200, 255, 255);
                DrawWorldMark(dl, wv, cm, 18.0f);
            }
        }
        dl->PopClipRect();
    }

    ImGui::Separator();

    // The band in numbers, under the picture of it. Both halves are shown even
    // when only one is out: "the span is right and the angle is not" is a
    // different fault (a part turned) from its opposite (the wrong feature), and
    // the operator is the one who has to tell them apart.
    if (bandOn) {
        const ImVec4 okCol(0.4f, 0.9f, 0.5f, 1.0f);
        const ImVec4 badCol(1.0f, 0.65f, 0.25f, 1.0f);
        ImGui::TextColored(std::fabs(bandSpanErr) <= req.band.spanTol ? okCol : badCol,
                           "span %.4f mm (%.4f +/- %.4f, %+.4f)",
                           bandSpan, req.band.spanRef, req.band.spanTol, bandSpanErr);
        ImGui::SameLine();
        ImGui::TextColored(std::fabs(bandThetaErr) <= req.band.thetaTol ? okCol : badCol,
                           "   theta %.3f deg (%.3f +/- %.3f, %+.3f)",
                           bandTheta, req.band.thetaRef, req.band.thetaTol, bandThetaErr);
        if (bandOk) ImGui::TextColored(okCol, "IN BAND");
        else        ImGui::TextColored(badCol, "OUT OF BAND - move the mark into the "
                                               "shaded band, or accept it as it is");
    } else if (req.band.have) {
        ImGui::TextDisabled("guard band not shown - no gantry pose or no pixel calibration");
        ImGui::TextDisabled(" ");
    }

    // ---- The live residual --------------------------------------------------
    // Set only by the registration check, and it is the whole reason that check
    // can be driven by hand: the operator is being asked to measure a distance,
    // and until now that distance only existed in the log AFTER the run had
    // already failed. It is recomputed every frame from the crosshair and the
    // live pose, so nudging the mark or jogging Z closes the number while you
    // watch it.
    //
    // dZ is the DEVICE Z, not anything read off the picture: a camera cannot
    // show height, and the taught tip Z is simply the gantry Z that focuses on
    // the tip. Focus by hand and the Z error IS the number.
    if (req.target.have) {
        double liveZ = 0.0;
        bool   haveZ = false;
        std::map<std::string, double> pose;
        if (!req.target.device.empty() && workflow_.DevicePose(req.target.device, pose)) {
            const auto iz = pose.find("Z");
            if (iz != pose.end()) { liveZ = iz->second; haveZ = true; }
        }
        const double ex  = havePose ? (gxNow + dxMM) - req.target.x : 0.0;
        const double ey  = havePose ? (gyNow + dyMM) - req.target.y : 0.0;
        const double ez  = haveZ    ? liveZ - req.target.z          : 0.0;
        const double res = std::sqrt(ex * ex + ey * ey + ez * ez);
        // Neutral when no tolerance was given: a number wearing a colour is a
        // verdict, and the caller has to have asked for one.
        const bool   judge = req.target.tolMm > 0.0;
        const ImVec4 tOk(0.4f, 0.9f, 0.5f, 1.0f);
        const ImVec4 tBad(1.0f, 0.65f, 0.25f, 1.0f);
        const ImVec4 tCol = !judge ? ImGui::GetStyleColorVec4(ImGuiCol_Text)
                                   : (res <= req.target.tolMm ? tOk : tBad);
        ImGui::TextColored(tCol,
                           "off prediction  dX %+.4f  dY %+.4f  dZ %+.4f  =  %.4f mm%s",
                           ex, ey, ez, res,
                           judge ? (res <= req.target.tolMm ? "   ON TARGET" : "   OUT")
                                 : "");
        if (!havePose || !haveZ)
            ImGui::TextDisabled("partial - %s%s",
                                havePose ? "" : "no gantry pose or pixel calibration  ",
                                haveZ ? "" : "no live Z from the device");
        else if (judge)
            ImGui::TextDisabled("tolerance %.4f mm - jog Z until dZ closes, then mark the "
                                "tip and OK", req.target.tolMm);
        else
            ImGui::TextDisabled("jog Z until dZ closes, then mark the tip and OK");

        // The focus score, here and not only in the Jog window: with the check's
        // autofocus switched off this IS how Z gets set, and an operator should
        // not have to keep a second window open to see the number they are
        // maximising. Off by default - it costs a score per frame.
        bool stream = workflow_.FocusStreaming();
        if (ImGui::Checkbox("live focus score", &stream))
            workflow_.SetFocusStreaming(stream, focusRoiPx_);
        ImGui::SameLine();
        const Workflow::FocusSample fs = workflow_.LatestFocus();
        if (!stream)     ImGui::TextDisabled("not sampling");
        else if (!fs.ok) ImGui::TextDisabled("waiting for frames...");
        else             ImGui::Text("%.1f   luma %.0f", fs.score, fs.meanLuma);
    }

    const auto answer = [&](bool ok) {
        ExecutionContext::PickResult r;
        r.px   = pickPx_;
        r.py   = pickPy_;
        r.dxMM = dxMM;
        r.dyMM = dyMM;
        if (fromScript) orch_.lua.AnswerPick(ok, r);
        else            orch_.recipe.AnswerPick(ok, r);
    };
    // The step diagrams lead the button row, and ONLY when an object is being
    // marked: they draw two corners of a part, which is what place_mark asks
    // for and is not what E1/E2 are (two features that fix an axis, anywhere on
    // the part). The CALLER says so — a non-empty label used to imply it, until
    // a pick that was neither case (one needle imprint) inherited the wrong
    // picture along with its title.
    if (req.stepIcons) {
        EnsureMarkIcons();
        const MarkIcon icons[2] = {markIcons_[0], markIcons_[1]};
        DrawMarkIconsInline(icons, req.station - 1, ImGui::GetFrameHeight() * 2.0f);
    }
    // Out of band, OK changes its NAME rather than disappearing. The band is a
    // sanity check, not an interlock: the person at the machine can see the
    // part and the software cannot, so refusing them would only teach them to
    // switch the band off. What it must not do is let the accept pass for an
    // ordinary OK — hence the label, the colour, and the warning in the log,
    // which is what a run that comes out 200 um wrong is read back from.
    const bool outOfBand = bandOn && !bandOk;
    ImGui::BeginDisabled(!haveImg);
    if (outOfBand) {
        ImGui::PushStyleColor(ImGuiCol_Button,        ImVec4(0.60f, 0.36f, 0.10f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.72f, 0.44f, 0.14f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonActive,  ImVec4(0.82f, 0.50f, 0.16f, 1.0f));
    }
    if (ImGui::Button(outOfBand ? "Accept out of band" : "OK (Mark here)",
                      ImVec2(outOfBand ? 200.0f : 160.0f, 0.0f))) {
        if (outOfBand) {
            FD_WARN("align", "'%s': %s accepted OUT OF BAND - span %.4f (%.4f +/- %.4f), "
                             "theta %.3f (%.3f +/- %.3f)",
                    req.alignName.c_str(), what, bandSpan, req.band.spanRef,
                    req.band.spanTol, bandTheta, req.band.thetaRef, req.band.thetaTol);
        }
        answer(true);
    }
    if (outOfBand) ImGui::PopStyleColor(3);
    ImGui::EndDisabled();
    ImGui::SameLine();
    if (ImGui::Button("Cancel (Fail Step)", ImVec2(160.0f, 0.0f))) answer(false);
    ImGui::SameLine();
    ImGui::TextUnformatted("zoom");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(120.0f);
    ImGui::SliderFloat("##pickzoom", &pickZoom_, 1.0f, 12.0f, "x%.2f",
                       ImGuiSliderFlags_Logarithmic);
    ImGui::SameLine();
    if (ImGui::SmallButton("Fit")) pickZoom_ = 1.0f;
    ImGui::SameLine();
    // The gate for right-click-to-move, and the only control here that arms
    // anything. Shared with the Jog window, so a session that armed once stays
    // armed — which is the behaviour an operator marking several points wants.
    ImGui::Checkbox("Arm move", &jogArmed_);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Right-click on the image drives the gantry so that point\n"
                          "lands under the centre crosshair. Marking itself never\n"
                          "moves anything.");
    ImGui::SameLine();
    ImGui::TextDisabled("%.5f / %.5f mm per px", pixelToMmX_, pixelToMmY_);
    ImGui::End();
}

} // namespace firedog
