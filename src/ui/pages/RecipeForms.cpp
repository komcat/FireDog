// RecipeForms.cpp : the schema-driven parameter forms of the Recipe Editor.
// Definitions of App members — see App.h.
//
// DrawParamWidget renders one row per ParamSpec straight off the
// CommandRegistry, so a newly registered command gets a working form for free
// — keep it that way, and add a ParamSpec kind rather than a per-command
// special case. DrawConditionEditor is shared by if / loop-while / wait_until.
#include "App.h"

#include "imgui.h"

#include "ui/Widgets.h"
#include "core/DataBus.h"
#include "core/Log.h"
#include "core/Topics.h"

#include <cstdio>
#include <string>

namespace firedog {

// Step inspector: common fields + the schema-driven parameter form. Editable
// unless a run is active (then everything renders disabled).
void App::DrawRecipeInspector(RecipeStep& s, bool canEdit)
{
    ImGui::BeginDisabled(!canEdit);
    ImGui::Text("Type: %s", s.type.c_str());
    char nbuf[64];
    std::snprintf(nbuf, sizeof(nbuf), "%s", s.nickname.c_str());
    if (ImGui::InputText("nickname", nbuf, sizeof(nbuf))) s.nickname = nbuf;
    ImGui::SameLine();
    if (ImGui::SmallButton("gen")) s.nickname = AutoNickname(s);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Fill the nickname from the params.\n"
                          "An EMPTY nickname already shows this text in the tree,\n"
                          "auto-updating as params change.");
    ImGui::Checkbox("enabled", &s.enabled);
    int oe = static_cast<int>(s.onError);
    ImGui::SetNextItemWidth(120.0f);
    if (ImGui::Combo("on error", &oe, "abort\0continue\0retry\0"))
        s.onError = static_cast<RecipeStep::OnError>(oe);
    if (s.onError == RecipeStep::OnError::Retry) {
        ImGui::SetNextItemWidth(120.0f);
        if (ImGui::InputInt("retries", &s.retryCount) && s.retryCount < 0)
            s.retryCount = 0;
    }
    ImGui::Separator();

    if (s.type == "loop") {
        const bool isWhile = s.params.contains("while");
        if (ImGui::RadioButton("count", !isWhile) && isWhile) {
            s.params.erase("while");
            s.params.erase("max_iterations");
            s.params["count"] = 1;
        }
        ImGui::SameLine();
        if (ImGui::RadioButton("while", isWhile) && !isWhile) {
            s.params.erase("count");
            s.params["while"] = nlohmann::json::object();
        }
        if (s.params.contains("count")) {
            int c = s.params["count"].is_number()
                        ? static_cast<int>(s.params["count"].get<double>()) : 1;
            ImGui::SetNextItemWidth(120.0f);
            if (ImGui::InputInt("count##loop", &c)) s.params["count"] = c < 1 ? 1 : c;
        } else if (s.params.contains("while")) {
            DrawConditionEditor(s.params["while"]);
            int mi = s.params.value("max_iterations", kDefaultMaxIterations);
            ImGui::SetNextItemWidth(120.0f);
            if (ImGui::InputInt("max iterations", &mi))
                s.params["max_iterations"] = mi < 1 ? 1 : mi;
        }
    } else if (s.type == "if") {
        DrawConditionEditor(s.params);
        ImGui::TextDisabled("Palette 'else' mode adds else-branch steps.");
    } else if (s.type == "sub_recipe") {
        const std::string uid = s.params.value("uid", std::string());
        const Recipe* target = nullptr;
        for (const Recipe& rr : session_.Recipes()) {
            if (rr.uid == uid) { target = &rr; break; }
        }
        if (ImGui::BeginCombo("recipe", target ? target->name.c_str() : "(pick)")) {
            for (const Recipe& rr : session_.Recipes()) {
                // Obvious self-reference filtered; deeper cycles are caught by
                // InlineSubRecipes at run start.
                if (recipeSel_ >= 0 && rr.uid == session_.Recipes()[recipeSel_].uid) continue;
                if (ImGui::Selectable(rr.name.c_str(), target == &rr))
                    s.params["uid"] = rr.uid;
            }
            ImGui::EndCombo();
        }
    } else if (s.type == "stop_recipe") {
        bool ok = s.params.value("ok", true);
        if (ImGui::Checkbox("success", &ok)) s.params["ok"] = ok;
        char mbuf[128];
        std::snprintf(mbuf, sizeof(mbuf), "%s", s.params.value("message", std::string()).c_str());
        if (ImGui::InputText("message", mbuf, sizeof(mbuf)))
            s.params["message"] = std::string(mbuf);
    } else if (s.type == "group") {
        ImGui::TextDisabled("Group: organizational only.");
    } else if (const CommandSpec* spec = orch_.registry.Find(s.type)) {
        ImGui::TextWrapped("%s", spec->description.c_str());
        ImGui::Spacing();
        for (const ParamSpec& p : spec->params) DrawParamWidget(p, s.params);
        const std::string err = ValidateParams(*spec, s.params, nullptr);
        if (!err.empty())
            ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.3f, 1.0f), "%s", err.c_str());
    } else {
        ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.3f, 1.0f), "unknown command '%s'",
                           s.type.c_str());
    }
    ImGui::EndDisabled();
}

// One schema-driven form row: reads params[p.name] (schema default when
// absent), writes back on edit. The widget is chosen by ParamType — the whole
// reason the typed registry exists. Returns true when the value changed.
bool App::DrawParamWidget(const ParamSpec& p, nlohmann::json& params)
{
    bool changed = false;
    ImGui::PushID(p.name.c_str());
    nlohmann::json cur = params.contains(p.name) ? params[p.name] : p.defaultValue;
    const char* label = p.name.c_str();
    const float w = 170.0f;

    // Device-scoped reference params (position uid / map / pin) go stale when
    // the operator hot-swaps the step's device: the stored value no longer
    // resolves for the new device, so the combo shows a bare uid (or a name
    // the device doesn't have). Render that loud — red text on a yellow frame,
    // plus a hint line — until the operator re-picks. Push before BeginCombo,
    // pop right after it so the dropdown's own items render normally.
    constexpr ImVec4 kStaleRed{0.85f, 0.08f, 0.08f, 1.0f};
    const auto pushStale = [&kStaleRed] {
        ImGui::PushStyleColor(ImGuiCol_Text,           kStaleRed);
        ImGui::PushStyleColor(ImGuiCol_FrameBg,        ImVec4(0.95f, 0.80f, 0.15f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_FrameBgHovered, ImVec4(1.00f, 0.88f, 0.30f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_FrameBgActive,  ImVec4(1.00f, 0.92f, 0.45f, 1.0f));
    };

    switch (p.type) {
    case ParamType::Bool: {
        bool v = cur.is_boolean() && cur.get<bool>();
        if (ImGui::Checkbox(label, &v)) { params[p.name] = v; changed = true; }
        break;
    }
    case ParamType::Int: {
        int v = cur.is_number() ? static_cast<int>(cur.get<double>()) : 0;
        ImGui::SetNextItemWidth(w);
        if (ImGui::InputInt(label, &v)) {
            if (p.minVal < p.maxVal) {
                if (v < p.minVal) v = static_cast<int>(p.minVal);
                if (v > p.maxVal) v = static_cast<int>(p.maxVal);
            }
            params[p.name] = v;
            changed = true;
        }
        break;
    }
    case ParamType::IntChoice: {
        // `options[i]` names the value minVal + i. A value outside the list is
        // shown as the bare number rather than snapped to a neighbour: it came
        // from somewhere (a hand-edited recipe, a spec that has since grown a
        // choice), and silently rewriting it would hide that.
        const int lo = static_cast<int>(p.minVal);
        const int v  = cur.is_number() ? static_cast<int>(cur.get<double>()) : lo;
        const int idx = v - lo;
        const int  n     = static_cast<int>(p.options.size());
        const bool known = idx >= 0 && idx < n;
        char fallback[32];
        std::snprintf(fallback, sizeof(fallback), "%d", v);
        ImGui::SetNextItemWidth(w);
        if (ImGui::BeginCombo(label, known ? p.options[idx].c_str() : fallback)) {
            for (int i = 0; i < n; ++i) {
                const int val = lo + i;
                if (ImGui::Selectable(p.options[i].c_str(), val == v)) {
                    params[p.name] = val;
                    changed = true;
                }
            }
            ImGui::EndCombo();
        }
        break;
    }
    case ParamType::Double: {
        double v = cur.is_number() ? cur.get<double>() : 0.0;
        ImGui::SetNextItemWidth(w);
        if (ImGui::InputDouble(label, &v, 0.0, 0.0, "%g")) {
            if (p.minVal < p.maxVal) {
                if (v < p.minVal) v = p.minVal;
                if (v > p.maxVal) v = p.maxVal;
            }
            params[p.name] = v;
            changed = true;
        }
        break;
    }
    case ParamType::String: {
        char buf[256];
        std::snprintf(buf, sizeof(buf), "%s",
                      cur.is_string() ? cur.get<std::string>().c_str() : "");
        if (ImGui::InputText(label, buf, sizeof(buf))) {
            params[p.name] = std::string(buf);
            changed = true;
        }
        break;
    }
    case ParamType::Value: {
        int t = cur.is_boolean() ? 0 : cur.is_number() ? 1 : 2;
        ImGui::SetNextItemWidth(80.0f);
        if (ImGui::Combo("##vtype", &t, "bool\0number\0string\0")) {
            params[p.name] = t == 0   ? nlohmann::json(false)
                             : t == 1 ? nlohmann::json(0.0)
                                      : nlohmann::json("");
            cur     = params[p.name];
            changed = true;
        }
        ImGui::SameLine();
        if (t == 0) {
            bool v = cur.is_boolean() && cur.get<bool>();
            if (ImGui::Checkbox(label, &v)) { params[p.name] = v; changed = true; }
        } else if (t == 1) {
            double v = cur.is_number() ? cur.get<double>() : 0.0;
            ImGui::SetNextItemWidth(120.0f);
            if (ImGui::InputDouble(label, &v, 0.0, 0.0, "%g")) {
                params[p.name] = v;
                changed = true;
            }
        } else {
            char buf[256];
            std::snprintf(buf, sizeof(buf), "%s",
                          cur.is_string() ? cur.get<std::string>().c_str() : "");
            if (ImGui::InputText(label, buf, sizeof(buf))) {
                params[p.name] = std::string(buf);
                changed = true;
            }
        }
        break;
    }
    case ParamType::Enum: {
        const std::string v = cur.is_string() ? cur.get<std::string>() : "";
        ImGui::SetNextItemWidth(w);
        if (ImGui::BeginCombo(label, v.c_str())) {
            for (const std::string& o : p.options) {
                if (ImGui::Selectable(o.c_str(), o == v)) {
                    params[p.name] = o;
                    changed = true;
                }
            }
            ImGui::EndCombo();
        }
        break;
    }
    case ParamType::DeviceMotion:
    case ParamType::DeviceHexapod:
    case ParamType::DeviceIo:
    case ParamType::Slide: {
        const std::string v = cur.is_string() ? cur.get<std::string>() : "";
        ImGui::SetNextItemWidth(w);
        if (ImGui::BeginCombo(label, v.empty() ? "(pick)" : v.c_str())) {
            const auto item = [&](const std::string& name) {
                if (ImGui::Selectable(name.c_str(), name == v)) {
                    params[p.name] = name;
                    changed = true;
                }
            };
            if (p.type == ParamType::DeviceMotion ||
                p.type == ParamType::DeviceHexapod) {
                for (std::size_t i = 0; i < hardware_.MotionCount(); ++i) {
                    if (p.type == ParamType::DeviceHexapod &&
                        hardware_.MotionDevice(i)->Type() != "PI")
                        continue;   // scan commands are PI-only
                    item(hardware_.MotionName(i));
                }
            } else if (p.type == ParamType::DeviceIo) {
                for (std::size_t i = 0; i < hardware_.IoCount(); ++i)
                    item(hardware_.IoName(i));
            } else {
                for (std::size_t i = 0; i < hardware_.SlideCount(); ++i)
                    if (hardware_.Slide(i)) item(hardware_.Slide(i)->Name());
            }
            ImGui::EndCombo();
        }
        break;
    }
    case ParamType::PositionUid: {
        const std::string dev = params.value("device", std::string());
        const std::string v   = cur.is_string() ? cur.get<std::string>() : "";
        std::string preview   = v.empty() ? "(pick)" : v;
        bool resolved         = v.empty();   // empty = nothing to warn about yet
        const auto it = session_.Positions().find(dev);
        if (it != session_.Positions().end()) {
            for (const SavedPos& sp : it->second)
                if (sp.uid == v) { preview = sp.name; resolved = true; break; }
        }
        if (!resolved) pushStale();
        ImGui::SetNextItemWidth(w);
        const bool open = ImGui::BeginCombo(label, preview.c_str());
        if (!resolved) ImGui::PopStyleColor(4);
        if (open) {
            if (it == session_.Positions().end() || it->second.empty())
                ImGui::TextDisabled("(no saved positions for '%s')", dev.c_str());
            else
                for (const SavedPos& sp : it->second) {
                    // System/private rows (e.g. an alignment procedure's own
                    // station positions) are not part of the recipe vocabulary.
                    // An existing reference still RESOLVES above — only the
                    // offer is withheld.
                    if (sp.hidden) continue;
                    if (ImGui::Selectable(sp.name, sp.uid == v)) {
                        params[p.name] = sp.uid;   // stores the uid, shows the name
                        changed = true;
                    }
                }
            ImGui::EndCombo();
        }
        if (!resolved)
            ImGui::TextColored(kStaleRed, "not a position of '%s' - re-pick", dev.c_str());
        break;
    }
    case ParamType::AlignUid: {
        // NOT device-scoped (unlike PositionUid): a procedure carries its own
        // device. Stores the uid, shows the name.
        const std::string v = cur.is_string() ? cur.get<std::string>() : "";
        std::string preview = v.empty() ? "(pick)" : v;
        bool resolved       = v.empty();
        if (const AlignProc* ap = FindAlignProc(session_.Aligns(), v)) {
            preview  = ap->name;
            resolved = true;
        }
        if (!resolved) pushStale();
        ImGui::SetNextItemWidth(w);
        const bool open = ImGui::BeginCombo(label, preview.c_str());
        if (!resolved) ImGui::PopStyleColor(4);
        if (open) {
            if (session_.Aligns().empty())
                ImGui::TextDisabled("(no alignment procedures - see the Alignment page)");
            else
                for (const AlignProc& ap : session_.Aligns()) {
                    if (ImGui::Selectable(ap.name.c_str(), ap.uid == v)) {
                        params[p.name] = ap.uid;
                        changed = true;
                    }
                }
            ImGui::EndCombo();
        }
        if (!resolved)
            ImGui::TextColored(kStaleRed, "unknown alignment procedure - re-pick");
        break;
    }
    case ParamType::NominalUid: {
        // Scoped to the step's OWN "alignment", the way PositionUid is scoped to
        // its "device": a nominal is a point on one procedure's part, and the
        // list of every nominal in the product would be a list of points that
        // mostly cannot be placed from here.
        //
        // Picks store the uid, but an old recipe that holds a NAME still shows
        // as resolved — FindAlignNominal takes either, and re-typing a hand-
        // written recipe's params is not an upgrade the operator asked for.
        const std::string al = params.value("alignment", std::string());
        const AlignProc*  ap = FindAlignProc(session_.Aligns(), al);
        const std::string v  = cur.is_string() ? cur.get<std::string>() : "";
        std::string preview  = v.empty() ? "(pick)" : v;
        // Optional and unset is a real setting here (place_mark: "empty = always
        // ask"), so an empty value is never stale.
        bool resolved = v.empty();
        if (ap) {
            if (const AlignNominal* nm = FindAlignNominal(*ap, v)) {
                preview  = nm->name;
                resolved = true;
            }
        } else {
            // No procedure picked yet: nothing to resolve AGAINST, so the value
            // is unjudged rather than wrong. The alignment row is already red.
            resolved = true;
        }
        if (!resolved) pushStale();
        ImGui::SetNextItemWidth(w);
        const bool open = ImGui::BeginCombo(label, preview.c_str());
        if (!resolved) ImGui::PopStyleColor(4);
        if (open) {
            if (!ap) {
                ImGui::TextDisabled("(pick the alignment procedure first)");
            } else if (ap->nominals.empty()) {
                ImGui::TextDisabled("(no nominals on '%s' - teach one from the "
                                    "Alignment page)", ap->name.c_str());
            } else {
                if (!p.required && ImGui::Selectable("(none)", v.empty())) {
                    params[p.name] = std::string();
                    changed = true;
                }
                for (const AlignNominal& nm : ap->nominals) {
                    // Match the name too: a recipe written by hand holds one.
                    const bool sel = nm.uid == v || nm.name == v;
                    if (ImGui::Selectable(nm.name.c_str(), sel)) {
                        params[p.name] = nm.uid;
                        changed = true;
                    }
                }
            }
            ImGui::EndCombo();
        }
        if (!resolved)
            ImGui::TextColored(kStaleRed, "not a nominal of '%s' - re-pick",
                               ap->name.c_str());
        break;
    }
    case ParamType::VisionUid: {
        // Same shape as AlignUid: not device-scoped, stores the uid, shows the
        // name. The method rides along in the row because "circle" and "ncc"
        // fail for completely different reasons and the name rarely says which.
        const std::string v = cur.is_string() ? cur.get<std::string>() : "";
        std::string preview = v.empty() ? "(pick)" : v;
        bool resolved       = v.empty();
        if (const VisionModel* vm = FindVisionModel(session_.VisionModels(), v)) {
            preview  = vm->name;
            resolved = true;
        }
        if (!resolved) pushStale();
        ImGui::SetNextItemWidth(w);
        const bool open = ImGui::BeginCombo(label, preview.c_str());
        if (!resolved) ImGui::PopStyleColor(4);
        if (open) {
            if (session_.VisionModels().empty())
                ImGui::TextDisabled("(no detection models - teach one from an "
                                    "alignment station)");
            else
                for (const VisionModel& vm : session_.VisionModels()) {
                    const std::string row = vm.name + "  (" + vm.method + ")";
                    if (ImGui::Selectable(row.c_str(), vm.uid == v)) {
                        params[p.name] = vm.uid;
                        changed = true;
                    }
                }
            ImGui::EndCombo();
        }
        if (!resolved)
            ImGui::TextColored(kStaleRed, "unknown detection model - re-pick");
        break;
    }
    case ParamType::MapName: {
        const std::string dev = params.value("device", std::string());
        const std::string v   = cur.is_string() ? cur.get<std::string>() : "";
        const auto it = session_.Maps().find(dev);
        bool resolved = v.empty();
        if (!resolved && it != session_.Maps().end())
            for (const MotionMap& m : it->second)
                if (m.name == v) { resolved = true; break; }
        if (!resolved) pushStale();
        ImGui::SetNextItemWidth(w);
        const bool open = ImGui::BeginCombo(label, v.empty() ? "(pick)" : v.c_str());
        if (!resolved) ImGui::PopStyleColor(4);
        if (open) {
            if (it == session_.Maps().end() || it->second.empty())
                ImGui::TextDisabled("(no maps for '%s')", dev.c_str());
            else
                for (const MotionMap& m : it->second) {
                    if (ImGui::Selectable(m.name.c_str(), m.name == v)) {
                        params[p.name] = m.name;
                        changed = true;
                    }
                }
            ImGui::EndCombo();
        }
        if (!resolved)
            ImGui::TextColored(kStaleRed, "not a map of '%s' - re-pick", dev.c_str());
        break;
    }
    case ParamType::PinName: {
        const std::string dev = params.value("device", std::string());
        const std::string v   = cur.is_string() ? cur.get<std::string>() : "";
        const IIoDevice* d = nullptr;
        for (std::size_t i = 0; i < hardware_.IoCount(); ++i)
            if (hardware_.IoName(i) == dev) { d = hardware_.IoDevice(i); break; }
        bool resolved = v.empty();
        if (!resolved && d) {
            const auto has = [&](bool inputs) {
                const int n = inputs ? d->InputCount() : d->OutputCount();
                for (int pin = 0; pin < n; ++pin)
                    if ((inputs ? d->InputPinName(pin) : d->OutputPinName(pin)) == v)
                        return true;
                return false;
            };
            resolved = (p.pinDir != PinDir::Output && has(true)) ||
                       (p.pinDir != PinDir::Input && has(false));
        }
        if (!resolved) pushStale();
        ImGui::SetNextItemWidth(w);
        const bool open = ImGui::BeginCombo(label, v.empty() ? "(pick)" : v.c_str());
        if (!resolved) ImGui::PopStyleColor(4);
        if (open) {
            if (!d) {
                ImGui::TextDisabled("(pick a device first)");
            } else {
                const auto pins = [&](bool inputs) {
                    const int n = inputs ? d->InputCount() : d->OutputCount();
                    for (int pin = 0; pin < n; ++pin) {
                        const std::string name =
                            inputs ? d->InputPinName(pin) : d->OutputPinName(pin);
                        if (name.empty()) continue;
                        if (ImGui::Selectable(name.c_str(), name == v)) {
                            params[p.name] = name;
                            changed = true;
                        }
                    }
                };
                if (p.pinDir != PinDir::Output) pins(true);    // inputs
                if (p.pinDir != PinDir::Input)  pins(false);   // outputs
            }
            ImGui::EndCombo();
        }
        if (!resolved)
            ImGui::TextColored(kStaleRed, "not a pin of '%s' - re-pick", dev.c_str());
        break;
    }
    case ParamType::TopicName: {
        char buf[128];
        std::snprintf(buf, sizeof(buf), "%s",
                      cur.is_string() ? cur.get<std::string>().c_str() : "");
        ImGui::SetNextItemWidth(w);
        if (ImGui::InputText(label, buf, sizeof(buf))) {
            params[p.name] = std::string(buf);
            changed = true;
        }
        const std::vector<std::string> topics = DataBus::Instance().Topics();
        if (!topics.empty()) {
            ImGui::SameLine();
            if (ImGui::BeginCombo("##topics", "", ImGuiComboFlags_NoPreview)) {
                for (const std::string& t : topics) {
                    if (ImGui::Selectable(t.c_str())) {
                        params[p.name] = t;
                        changed = true;
                    }
                }
                ImGui::EndCombo();
            }
        }
        break;
    }
    case ParamType::ScriptFile: {
        // Picker over the same scanned list the Scripts page shows (product
        // scripts dir, else config/scripts). Stores the path; shows the name.
        const std::string v = cur.is_string() ? cur.get<std::string>() : "";
        std::string preview = v.empty() ? "(pick)" : v;
        for (const ScriptInfo& si : session_.Scripts()) {
            if (si.path == v) { preview = si.name; break; }
        }
        ImGui::SetNextItemWidth(w);
        if (ImGui::BeginCombo(label, preview.c_str())) {
            if (session_.Scripts().empty())
                ImGui::TextDisabled("(no scripts in %s)", session_.ScriptsDirUsed().c_str());
            for (const ScriptInfo& si : session_.Scripts()) {
                if (ImGui::Selectable(si.name.c_str(), si.path == v)) {
                    params[p.name] = si.path;
                    changed = true;
                }
                if (ImGui::IsItemHovered() && !si.description.empty())
                    ImGui::SetTooltip("%s", si.description.c_str());
            }
            ImGui::EndCombo();
        }
        break;
    }
    }
    if (!p.help.empty() && ImGui::IsItemHovered()) ImGui::SetTooltip("%s", p.help.c_str());
    ImGui::PopID();
    return changed;
}

// Shared editor for {key|topic, op, value} condition objects (an if step's
// params, a loop's "while"). Normalizes the object in place so a fresh
// condition is immediately valid.
bool App::DrawConditionEditor(nlohmann::json& cond)
{
    bool changed = false;
    if (!cond.is_object()) { cond = nlohmann::json::object(); changed = true; }
    if (!cond.contains("op")) { cond["op"] = "is_true"; changed = true; }
    if (!cond.contains("key") && !cond.contains("topic")) {
        cond["key"] = "";
        changed = true;
    }

    int src = cond.contains("topic") ? 1 : 0;
    ImGui::SetNextItemWidth(100.0f);
    if (ImGui::Combo("source", &src, "ctx key\0topic\0")) {
        const std::string old = src == 1 ? cond.value("key", std::string())
                                         : cond.value("topic", std::string());
        cond.erase("key");
        cond.erase("topic");
        cond[src == 1 ? "topic" : "key"] = old;
        changed = true;
    }
    const char* fk = cond.contains("topic") ? "topic" : "key";
    char buf[128];
    std::snprintf(buf, sizeof(buf), "%s", cond.value(fk, std::string()).c_str());
    if (ImGui::InputText(fk, buf, sizeof(buf))) {
        cond[fk] = std::string(buf);
        changed = true;
    }

    static const char* kOps[] = {">", "<", ">=", "<=", "==", "!=",
                                 "is_true", "is_false", "exists", "not_exists"};
    const std::string op = cond.value("op", std::string("is_true"));
    ImGui::SetNextItemWidth(120.0f);
    if (ImGui::BeginCombo("op", op.c_str())) {
        for (const char* o : kOps) {
            if (ImGui::Selectable(o, op == o)) {
                cond["op"] = o;
                changed = true;
            }
        }
        ImGui::EndCombo();
    }
    const bool needsValue = op != "is_true" && op != "is_false" &&
                            op != "exists" && op != "not_exists";
    if (needsValue) {
        nlohmann::json v = cond.contains("value") ? cond["value"] : nlohmann::json(0.0);
        int t = v.is_string() ? 1 : 0;
        ImGui::SetNextItemWidth(80.0f);
        if (ImGui::Combo("##ctype", &t, "number\0string\0")) {
            cond["value"] = t == 1 ? nlohmann::json("") : nlohmann::json(0.0);
            v             = cond["value"];
            changed       = true;
        }
        ImGui::SameLine();
        if (t == 1) {
            char vb[128];
            std::snprintf(vb, sizeof(vb), "%s",
                          v.is_string() ? v.get<std::string>().c_str() : "");
            if (ImGui::InputText("value", vb, sizeof(vb))) {
                cond["value"] = std::string(vb);
                changed = true;
            }
        } else {
            double d = v.is_number() ? v.get<double>() : 0.0;
            ImGui::SetNextItemWidth(130.0f);
            if (ImGui::InputDouble("value", &d, 0.0, 0.0, "%g")) {
                cond["value"] = d;
                changed = true;
            }
        }
    }
    return changed;
}

} // namespace firedog
