// PageInterlocks.cpp : the Interlock Editor (View menu, full-page takeover,
// Admin by the MinRoleFor default). Definitions of App members - see App.h.
// Design: docs/interlock-design.md.
//
// Edits ilEdit_, a WORKING COPY of the rule set: Apply hands it to
// Workflow::ApplyInterlocks (live at once, for the gate and the watch), Save
// also writes <machine root>/config/interlocks.local.json. Every condition and
// rule shows its LIVE truth evaluated on the working copy, so an edit can be
// judged against the machine before it is applied. Everything is referenced by
// NAME, and a name that resolves to nothing FAILS CLOSED - shown in red here.
// Editing gates on orch_.Busy(): a rule set changed under a run is a run that
// was validated against different rules than it moves under.
#include "App.h"

#include "imgui.h"

#include "core/Interlock.h"
#include "core/Log.h"
#include "core/Product.h"   // GenerateUid

#include <algorithm>
#include <cstdio>
#include <string>
#include <vector>

namespace firedog {

namespace {

const ImVec4 kRed(0.95f, 0.35f, 0.30f, 1.0f);
const ImVec4 kAmber(1.00f, 0.72f, 0.25f, 1.0f);
const ImVec4 kGreen(0.45f, 0.85f, 0.50f, 1.0f);
const ImVec4 kGrey(0.55f, 0.55f, 0.55f, 1.0f);

void Dot(const ImVec4& c)
{
    const float  h = ImGui::GetFrameHeight();
    const ImVec2 p = ImGui::GetCursorScreenPos();
    ImGui::GetWindowDrawList()->AddCircleFilled(ImVec2(p.x + h * 0.5f, p.y + h * 0.5f),
                                                h * 0.25f, ImGui::GetColorU32(c));
    ImGui::Dummy(ImVec2(h, h));
}

// A std::string edited through a fixed buffer; true when it changed.
bool InputStr(const char* id, std::string& s, float width, std::size_t cap = 128)
{
    std::vector<char> buf(cap, '\0');
    std::snprintf(buf.data(), buf.size(), "%s", s.c_str());
    ImGui::SetNextItemWidth(width);
    if (!ImGui::InputText(id, buf.data(), buf.size())) return false;
    s = buf.data();
    return true;
}

} // namespace

void App::DrawInterlockEditPanel()
{
    const bool busy = orch_.Busy();

    // ---- Top bar: master switch, watch period, Apply / Save / Revert ----
    ImGui::BeginDisabled(busy);
    if (ImGui::Checkbox("Interlocks enabled", &ilEdit_.enabled)) ilDirty_ = true;
    ImGui::SameLine();
    ImGui::SetNextItemWidth(110.0f);
    if (ImGui::InputInt("watch ms", &ilEdit_.watchMs)) {
        ilEdit_.watchMs = std::clamp(ilEdit_.watchMs, 10, 1000);
        ilDirty_        = true;
    }
    ImGui::SameLine();
    const auto apply = [&] {
        IlConfig c = ilEdit_;
        c.loadError.clear();   // what is applied is what the editor shows
        workflow_.ApplyInterlocks(c);
        ilEdit_.loadError.clear();
        ilDirty_ = false;
        FD_INFO("interlock", "rule set applied by %s", user_.Label().c_str());
    };
    if (ImGui::Button("Apply")) {
        apply();
        ilMsg_ = "Applied - live now, not saved.";
    }
    ImGui::SameLine();
    if (ImGui::Button("Save")) {
        apply();
        std::string err;
        if (SaveInterlocks(ilEdit_, &err)) {
            ilMsg_ = "Applied and saved to " + InterlocksLocalPath();
            FD_INFO("interlock", "rule set saved to %s by %s", InterlocksLocalPath().c_str(),
                    user_.Label().c_str());
        } else {
            ilMsg_ = "Applied, but NOT saved: " + err;
            FD_ERROR("interlock", "rule set save failed: %s", err.c_str());
        }
    }
    ImGui::SameLine();
    if (ImGui::Button("Revert")) {
        ilEdit_  = workflow_.Interlocks();
        ilDirty_ = false;
        ilMsg_   = "Reverted to the live rule set.";
        if (ilSel_ >= static_cast<int>(ilEdit_.rules.size())) ilSel_ = -1;
    }
    ImGui::EndDisabled();
    ImGui::SameLine();
    if (ilDirty_) ImGui::TextColored(kAmber, "edited - not applied");
    else if (!ilMsg_.empty()) ImGui::TextDisabled("%s", ilMsg_.c_str());
    if (busy) {
        ImGui::SameLine();
        ImGui::TextColored(kAmber, "(a run is active - editing is locked)");
    }
    if (!ilEdit_.loadError.empty()) {
        ImGui::PushTextWrapPos(0.0f);
        ImGui::TextColored(kRed, "The interlock file cannot be read (%s) - every move is refused. "
                                 "Save replaces it with what is shown here.",
                           ilEdit_.loadError.c_str());
        ImGui::PopTextWrapPos();
    }
    ImGui::TextDisabled("Axis values are in AXIS space - what the Motion page shows for the "
                        "device (gantry-main Z counts down as it rises). Anything unreadable "
                        "counts as TRUE.");
    ImGui::Separator();

    // The live state, once per frame; every dot below is the WORKING COPY
    // judged on it, so an edit can be checked before it is applied.
    const IlSnapshot snap = workflow_.InterlockSnapshot(&ilEdit_);
    std::vector<IlRuleStatus> st = IlEvaluateAll(ilEdit_, snap);

    // ---- Left: the rule list ----
    const float listW = std::max(260.0f, ImGui::GetContentRegionAvail().x * 0.28f);
    ImGui::BeginChild("ilrules", ImVec2(listW, 0.0f), ImGuiChildFlags_Borders);
    for (int i = 0; i < static_cast<int>(ilEdit_.rules.size()); ++i) {
        const IlRule&       r = ilEdit_.rules[static_cast<std::size_t>(i)];
        const IlRuleStatus& s = st[static_cast<std::size_t>(i)];
        ImGui::PushID(i);
        Dot(!s.enabled ? kGrey : !s.active ? kGreen : s.unknown ? kAmber : kRed);
        ImGui::SameLine();
        ImGui::AlignTextToFramePadding();
        std::string label = r.name + (s.active ? "   ACTIVE" : "");
        if (ImGui::Selectable(label.c_str(), ilSel_ == i)) ilSel_ = i;
        ImGui::PopID();
    }
    if (ilEdit_.rules.empty()) ImGui::TextDisabled("no rules yet");
    ImGui::Separator();
    ImGui::BeginDisabled(busy);
    if (ImGui::Button("+ Rule")) {
        IlRule r;
        r.uid  = GenerateUid();
        r.name = "new rule";
        ilEdit_.rules.push_back(r);
        ilSel_   = static_cast<int>(ilEdit_.rules.size()) - 1;
        ilDirty_ = true;
    }
    const bool haveSel = ilSel_ >= 0 && ilSel_ < static_cast<int>(ilEdit_.rules.size());
    ImGui::BeginDisabled(!haveSel);
    ImGui::SameLine();
    if (ImGui::Button("Dup")) {
        IlRule r = ilEdit_.rules[static_cast<std::size_t>(ilSel_)];
        r.uid  = GenerateUid();   // a copy is a second rule, never the same one twice
        r.name += " (copy)";
        ilEdit_.rules.insert(ilEdit_.rules.begin() + ilSel_ + 1, r);
        ++ilSel_;
        ilDirty_ = true;
    }
    ImGui::SameLine();
    if (ImGui::Button("Del")) {
        ilEdit_.rules.erase(ilEdit_.rules.begin() + ilSel_);
        ilSel_   = -1;
        ilDirty_ = true;
    }
    ImGui::SameLine();
    if (ImGui::ArrowButton("##up", ImGuiDir_Up) && haveSel && ilSel_ > 0) {
        std::swap(ilEdit_.rules[static_cast<std::size_t>(ilSel_)],
                  ilEdit_.rules[static_cast<std::size_t>(ilSel_ - 1)]);
        --ilSel_;
        ilDirty_ = true;
    }
    ImGui::SameLine();
    if (ImGui::ArrowButton("##dn", ImGuiDir_Down) && haveSel &&
        ilSel_ + 1 < static_cast<int>(ilEdit_.rules.size())) {
        std::swap(ilEdit_.rules[static_cast<std::size_t>(ilSel_)],
                  ilEdit_.rules[static_cast<std::size_t>(ilSel_ + 1)]);
        ++ilSel_;
        ilDirty_ = true;
    }
    ImGui::EndDisabled();
    ImGui::EndDisabled();
    ImGui::EndChild();

    // + Rule / Dup / Del / move above may have changed the list THIS frame:
    // re-judge it, or the panel below indexes a status list of the old size.
    st = IlEvaluateAll(ilEdit_, snap);

    ImGui::SameLine();

    // ---- Right: the selected rule ----
    ImGui::BeginChild("ilrule", ImVec2(0.0f, 0.0f), ImGuiChildFlags_Borders);
    if (ilSel_ < 0 || ilSel_ >= static_cast<int>(ilEdit_.rules.size())) {
        ImGui::TextDisabled("Select a rule, or add one.");
        ImGui::Spacing();
        ImGui::PushTextWrapPos(0.0f);
        ImGui::TextDisabled(
            "A rule is: WHEN every condition holds, REFUSE the listed motions. Conditions are a "
            "slide's state, an IO input, or an axis value; a rule may refuse axes of any motion "
            "device (in one direction or both) and slide extends. The gate refuses a move before "
            "it starts; the watch stops a device whose refused axis is already moving when the "
            "rule becomes true, and latches a trip.");
        ImGui::PopTextWrapPos();
        ImGui::EndChild();
        return;
    }
    IlRule&             rule = ilEdit_.rules[static_cast<std::size_t>(ilSel_)];
    const IlRuleStatus& rst  = st[static_cast<std::size_t>(ilSel_)];
    ImGui::BeginDisabled(busy);

    if (InputStr("Name", rule.name, 320.0f)) ilDirty_ = true;
    ImGui::SameLine();
    if (ImGui::Checkbox("enabled", &rule.enabled)) ilDirty_ = true;
    ImGui::SameLine();
    ImGui::TextColored(!rst.enabled ? kGrey : !rst.active ? kGreen : rst.unknown ? kAmber : kRed,
                       "%s", !rst.enabled ? "disabled" : !rst.active ? "inactive" : "ACTIVE now");
    if (InputStr("Note", rule.note, 480.0f, 256)) ilDirty_ = true;

    // Pickers over the live fleet.
    const auto motionCombo = [&](const char* id, std::string& dev) -> bool {
        bool changed = false;
        ImGui::SetNextItemWidth(130.0f);
        if (ImGui::BeginCombo(id, dev.empty() ? "(device)" : dev.c_str())) {
            for (std::size_t i = 0; i < hardware_.MotionCount(); ++i) {
                const std::string& n = hardware_.MotionName(i);
                if (ImGui::Selectable(n.c_str(), n == dev)) { dev = n; changed = true; }
            }
            ImGui::EndCombo();
        }
        return changed;
    };
    const auto axesOf = [&](const std::string& dev) -> std::vector<std::string> {
        const auto it = snap.motion.find(dev);
        return it != snap.motion.end() ? it->second.axes : std::vector<std::string>{};
    };
    std::vector<std::string> slideNames;
    for (std::size_t i = 0; i < hardware_.SlideCount(); ++i)
        if (const PneumaticSlide* s = hardware_.Slide(i)) slideNames.push_back(s->Name());

    // ---- WHEN ----
    ImGui::Spacing();
    ImGui::SeparatorText("WHEN all of");
    int delCond = -1;
    for (int ci = 0; ci < static_cast<int>(rule.when.size()); ++ci) {
        IlCond& c = rule.when[static_cast<std::size_t>(ci)];
        ImGui::PushID(1000 + ci);

        static const char* kKinds[] = {"Slide", "IO input", "Axis"};
        int kind = static_cast<int>(c.kind);
        ImGui::SetNextItemWidth(90.0f);
        if (ImGui::Combo("##kind", &kind, kKinds, 3)) {
            IlCond fresh;
            fresh.kind = static_cast<IlCondKind>(kind);
            c          = fresh;
            ilDirty_   = true;
        }
        ImGui::SameLine();

        switch (c.kind) {
        case IlCondKind::Slide: {
            std::string preview;
            for (const std::string& s : c.slides) preview += (preview.empty() ? "" : ", ") + s;
            ImGui::SetNextItemWidth(200.0f);
            if (ImGui::BeginCombo("##slides", preview.empty() ? "any slide" : preview.c_str())) {
                if (ImGui::Selectable("any slide", c.slides.empty())) {
                    c.slides.clear();
                    ilDirty_ = true;
                }
                for (const std::string& n : slideNames) {
                    bool on = std::find(c.slides.begin(), c.slides.end(), n) != c.slides.end();
                    if (ImGui::Checkbox(n.c_str(), &on)) {
                        if (on) c.slides.push_back(n);
                        else    std::erase(c.slides, n);
                        ilDirty_ = true;
                    }
                }
                ImGui::EndCombo();
            }
            ImGui::SameLine();
            static const char* kWants[] = {"extended", "not retracted", "retracted"};
            int w = static_cast<int>(c.slideWant);
            ImGui::SetNextItemWidth(130.0f);
            if (ImGui::Combo("##want", &w, kWants, 3)) {
                c.slideWant = static_cast<IlSlideWant>(w);
                ilDirty_    = true;
            }
            break;
        }
        case IlCondKind::IoInput: {
            ImGui::SetNextItemWidth(120.0f);
            if (ImGui::BeginCombo("##iodev", c.device.empty() ? "(device)" : c.device.c_str())) {
                for (std::size_t i = 0; i < hardware_.IoCount(); ++i) {
                    const std::string& n = hardware_.IoName(i);
                    if (ImGui::Selectable(n.c_str(), n == c.device)) { c.device = n; ilDirty_ = true; }
                }
                ImGui::EndCombo();
            }
            ImGui::SameLine();
            const IIoDevice* d = nullptr;
            for (std::size_t i = 0; i < hardware_.IoCount(); ++i)
                if (hardware_.IoName(i) == c.device) d = hardware_.IoDevice(i);
            ImGui::SetNextItemWidth(170.0f);
            if (ImGui::BeginCombo("##iopin", c.pin.empty() ? "(input)" : c.pin.c_str())) {
                if (d) {
                    for (int p = 0; p < d->InputCount(); ++p) {
                        const std::string n = d->InputPinName(p);
                        if (n.empty()) continue;   // pins are referenced by name
                        if (ImGui::Selectable(n.c_str(), n == c.pin)) { c.pin = n; ilDirty_ = true; }
                    }
                }
                ImGui::EndCombo();
            }
            ImGui::SameLine();
            int on = c.on ? 0 : 1;
            static const char* kOn[] = {"is ON", "is OFF"};
            ImGui::SetNextItemWidth(80.0f);
            if (ImGui::Combo("##on", &on, kOn, 2)) { c.on = (on == 0); ilDirty_ = true; }
            break;
        }
        case IlCondKind::Axis: {
            if (motionCombo("##adev", c.device)) {
                c.axis.clear();
                ilDirty_ = true;
            }
            ImGui::SameLine();
            ImGui::SetNextItemWidth(60.0f);
            if (ImGui::BeginCombo("##axis", c.axis.empty() ? "(axis)" : c.axis.c_str())) {
                for (const std::string& a : axesOf(c.device))
                    if (ImGui::Selectable(a.c_str(), a == c.axis)) { c.axis = a; ilDirty_ = true; }
                ImGui::EndCombo();
            }
            ImGui::SameLine();
            static const char* kOps[] = {">", ">=", "<", "<="};
            int op = static_cast<int>(c.op);
            ImGui::SetNextItemWidth(55.0f);
            if (ImGui::Combo("##op", &op, kOps, 4)) { c.op = static_cast<IlOp>(op); ilDirty_ = true; }
            ImGui::SameLine();
            ImGui::SetNextItemWidth(100.0f);
            if (ImGui::InputDouble("##val", &c.value, 0.0, 0.0, "%.3f")) ilDirty_ = true;
            // "Use current": the threshold is the number the axis reads NOW -
            // the honest way to type a value in a frame whose sign may surprise.
            const auto dm = snap.motion.find(c.device);
            if (dm != snap.motion.end()) {
                const auto ax = dm->second.measured.find(c.axis);
                if (ax != dm->second.measured.end()) {
                    ImGui::SameLine();
                    if (ImGui::SmallButton("= now")) { c.value = ax->second; ilDirty_ = true; }
                }
            }
            break;
        }
        }

        // Live truth of THIS condition, as edited.
        const IlCondStatus cs = IlEvaluateCond(c, snap);
        ImGui::SameLine();
        Dot(cs.unknown ? kAmber : cs.truth ? kRed : kGreen);
        ImGui::SameLine();
        ImGui::AlignTextToFramePadding();
        ImGui::TextColored(cs.unknown ? kAmber : cs.truth ? kRed : kGreen, "%s",
                           cs.truth ? "TRUE" : "false");
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("%s", cs.text.c_str());
        ImGui::SameLine();
        ImGui::TextDisabled("%s", cs.text.c_str());
        ImGui::SameLine();
        if (ImGui::SmallButton("x")) delCond = ci;
        ImGui::PopID();
    }
    if (delCond >= 0) {
        rule.when.erase(rule.when.begin() + delCond);
        ilDirty_ = true;
    }
    if (ImGui::Button("+ condition")) {
        rule.when.push_back(IlCond{});
        ilDirty_ = true;
    }
    if (rule.when.empty())
        ImGui::TextColored(kAmber, "No condition: this rule never activates.");

    // ---- THEN refuse ----
    ImGui::Spacing();
    ImGui::SeparatorText("THEN refuse");
    int delBlock = -1;
    for (int bi = 0; bi < static_cast<int>(rule.blocks.size()); ++bi) {
        IlBlock& b = rule.blocks[static_cast<std::size_t>(bi)];
        ImGui::PushID(2000 + bi);
        if (b.kind == IlBlockKind::SlideExtend) {
            ImGui::AlignTextToFramePadding();
            ImGui::TextUnformatted("extending");
            ImGui::SameLine();
            ImGui::SetNextItemWidth(170.0f);
            if (ImGui::BeginCombo("##bslide", b.device.empty() ? "any slide" : b.device.c_str())) {
                if (ImGui::Selectable("any slide", b.device.empty())) { b.device.clear(); ilDirty_ = true; }
                for (const std::string& n : slideNames)
                    if (ImGui::Selectable(n.c_str(), n == b.device)) { b.device = n; ilDirty_ = true; }
                ImGui::EndCombo();
            }
            if (!b.device.empty() &&
                std::find(slideNames.begin(), slideNames.end(), b.device) == slideNames.end()) {
                ImGui::SameLine();
                ImGui::TextColored(kRed, "no such slide");
            }
        } else {
            if (motionCombo("##bdev", b.device)) {
                b.axes.clear();
                ilDirty_ = true;
            }
            const std::vector<std::string> axes = axesOf(b.device);
            ImGui::SameLine();
            bool all = b.axes.empty();
            if (ImGui::Checkbox("all axes", &all)) {
                if (all) b.axes.clear();
                else if (!axes.empty()) b.axes = {axes.front()};
                ilDirty_ = true;
            }
            if (!all) {
                for (const std::string& a : axes) {
                    ImGui::SameLine();
                    bool on = std::find(b.axes.begin(), b.axes.end(), a) != b.axes.end();
                    if (ImGui::Checkbox(a.c_str(), &on)) {
                        if (on) b.axes.push_back(a);
                        else    std::erase(b.axes, a);
                        ilDirty_ = true;
                    }
                }
                // An axis the device no longer reports is still written - and
                // still refused - but say so.
                for (const std::string& a : b.axes)
                    if (std::find(axes.begin(), axes.end(), a) == axes.end()) {
                        ImGui::SameLine();
                        ImGui::TextColored(kRed, "%s?", a.c_str());
                    }
            }
            ImGui::SameLine();
            static const char* kDirs[] = {"either way", "+ only", "- only"};
            int d = b.dir == 0 ? 0 : b.dir > 0 ? 1 : 2;
            ImGui::SetNextItemWidth(100.0f);
            if (ImGui::Combo("##dir", &d, kDirs, 3)) {
                b.dir    = d == 0 ? 0 : d == 1 ? 1 : -1;
                ilDirty_ = true;
            }
            if (!b.device.empty() && snap.motion.find(b.device) == snap.motion.end()) {
                ImGui::SameLine();
                ImGui::TextColored(kRed, "no such device - refuses nothing");
            }
        }
        ImGui::SameLine();
        if (ImGui::SmallButton("x")) delBlock = bi;
        ImGui::PopID();
    }
    if (delBlock >= 0) {
        rule.blocks.erase(rule.blocks.begin() + delBlock);
        ilDirty_ = true;
    }
    if (ImGui::Button("+ device axes")) {
        rule.blocks.push_back(IlBlock{});
        ilDirty_ = true;
    }
    ImGui::SameLine();
    if (ImGui::Button("+ slide extend")) {
        IlBlock b;
        b.kind = IlBlockKind::SlideExtend;
        rule.blocks.push_back(b);
        ilDirty_ = true;
    }
    if (rule.blocks.empty())
        ImGui::TextColored(kAmber, "Nothing refused: this rule has no effect.");

    ImGui::EndDisabled();

    // ---- What it means, in one sentence ----
    ImGui::Spacing();
    ImGui::Separator();
    std::string when, then;
    for (const IlCond& c : rule.when) when += (when.empty() ? "" : " AND ") + IlCondText(c);
    for (const IlBlock& b : rule.blocks) then += (then.empty() ? "" : "; ") + IlBlockText(b);
    ImGui::PushTextWrapPos(0.0f);
    ImGui::TextDisabled("WHEN %s  ->  refuse %s", when.empty() ? "(never)" : when.c_str(),
                        then.empty() ? "(nothing)" : then.c_str());
    ImGui::PopTextWrapPos();
    ImGui::EndChild();
}

} // namespace firedog
