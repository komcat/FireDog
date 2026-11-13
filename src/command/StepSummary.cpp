// StepSummary.cpp : see StepSummary.h.
#include "command/StepSummary.h"

#include "command/CommandRegistry.h"
#include "command/CommandSpec.h"
#include "command/RoiFilterView.h"
#include "core/CameraSetup.h"

#include <cmath>
#include <cstdio>
#include <string>

namespace firedog {

// Shown in the tree whenever the nickname is empty (so it always tracks edits),
// and materialized into the nickname field by the inspector's "gen" button.
// Skips plumbing params (timeouts, tolerances, poll rates) and resolves
// position uids to names.
std::string StepSummary(const RecipeStep& s, const StepSummaryDocs& docs)
{
    const auto num = [](double v) {
        char b[32];
        std::snprintf(b, sizeof(b), "%g", v);
        return std::string(b);
    };
    const auto condText = [&num](const nlohmann::json& c) {
        std::string t = c.value("key", c.value("topic", std::string()));
        t += " " + c.value("op", std::string());
        if (c.contains("value")) {
            const nlohmann::json& v = c["value"];
            t += " " + (v.is_string() ? v.get<std::string>()
                                      : v.is_number() ? num(v.get<double>()) : v.dump());
        }
        return t;
    };

    if (s.type == "loop") {
        if (s.params.contains("while"))
            return "loop while " + condText(s.params["while"]);
        return "loop x" + num(s.params.value("count", 1.0));
    }
    if (s.type == "if") return "if " + condText(s.params);
    if (s.type == "group") return "group";
    if (s.type == "parallel") return "parallel";
    if (s.type == "sub_recipe") {
        const std::string uid = s.params.value("uid", std::string());
        if (docs.recipes) {
            for (const Recipe& r : *docs.recipes)
                if (r.uid == uid) return "run " + r.name;
        }
        return "sub_recipe";
    }
    if (s.type == "stop_recipe")
        return s.params.value("ok", true) ? "stop (ok)" : "stop (fail)";

    const CommandSpec* spec = docs.registry ? docs.registry->Find(s.type) : nullptr;
    if (!spec) return s.type;

    // Seven numbers that read as one thing: what the view will look like.
    if (s.type == "roi_filter_view_setup")
        return "view: " + CameraSetupSummary(RoiFilterViewSetupFromParams(
                              ParamValues(EffectiveParams(*spec, s.params))));
    if (s.type == "roi_filter_view_clear") return "view: clear the filter";

    const auto skip = [](const std::string& n) {
        return n == "timeout_ms" || n == "leg_timeout_ms" || n == "tol_mm" ||
               n == "start_tol_mm" || n == "poll_ms" || n == "max_iterations" ||
               n == "velocity_mm_s" || n == "exposure_us" || n == "settle_min_ms" ||
               n == "pre_delay_ms" || n == "post_delay_ms";
    };
    std::string out = s.type;
    const std::string dev = EffectiveString(*spec, s.params, "device");
    int added = 0;
    for (const ParamSpec& p : spec->params) {
        if (added >= 3) break;
        if (skip(p.name) || !s.params.contains(p.name)) continue;
        const nlohmann::json& v = s.params[p.name];
        std::string piece;
        if (p.type == ParamType::PositionUid && v.is_string()) {
            piece = v.get<std::string>();
            if (docs.positions) {
                const auto it = docs.positions->find(dev);
                if (it != docs.positions->end()) {
                    for (const SavedPos& sp : it->second)
                        if (sp.uid == piece) { piece = sp.name; break; }
                }
            }
        } else if (p.type == ParamType::NominalUid && v.is_string()) {
            // Same reason as PositionUid: the step stores a uid, and a uid in
            // the step tree tells the operator nothing.
            piece = v.get<std::string>();
            const AlignProc* ap =
                docs.aligns ? FindAlignProc(*docs.aligns,
                                            s.params.value("alignment", std::string()))
                            : nullptr;
            if (ap) {
                if (const AlignNominal* nm = FindAlignNominal(*ap, piece)) piece = nm->name;
            }
        } else if (p.type == ParamType::GripUid && v.is_string()) {
            piece = v.get<std::string>();
            if (docs.grips) {
                if (const GripProfile* g = FindGripProfile(*docs.grips, piece))
                    piece = g->name;
            }
        } else if (p.type == ParamType::TrayUid && v.is_string()) {
            piece = v.get<std::string>();
            if (docs.trays) {
                if (const WaffleTray* t = FindWaffleTray(*docs.trays, piece))
                    piece = t->name;
            }
        } else if (p.type == ParamType::DurationMs && v.is_number()) {
            piece = FormatDurationMs(std::llround(v.get<double>()));   // "delay 5 s"
        } else if (p.type == ParamType::IntChoice && v.is_number()) {
            // The choice's name, for the same reason: "align_station 2" makes
            // the reader look the numbering up.
            const int idx = static_cast<int>(v.get<double>()) - static_cast<int>(p.minVal);
            if (idx >= 0 && idx < static_cast<int>(p.options.size())) piece = p.options[idx];
            else piece = num(v.get<double>());
        } else if (p.type == ParamType::PointXyz && v.is_object() &&
                   v.value("X", nlohmann::json()).is_number() &&
                   v.value("Y", nlohmann::json()).is_number()) {
            // Where on the part, which is what tells two dispense steps apart.
            piece = "@" + num(v["X"].get<double>()) + "," + num(v["Y"].get<double>());
        } else if (p.type == ParamType::ScriptFile && v.is_string()) {
            piece = v.get<std::string>();   // show the file name, not the path
            const std::size_t cut = piece.find_last_of("/\\");
            if (cut != std::string::npos) piece = piece.substr(cut + 1);
        } else if (v.is_string()) {
            piece = v.get<std::string>();
        } else if (v.is_boolean()) {
            piece = v.get<bool>() ? "on" : "off";
        } else if (v.is_number()) {
            piece = num(v.get<double>());
        }
        if (piece.empty()) continue;
        out += " " + piece;
        ++added;
    }
    return out;
}

std::string FormatDurationMs(long long ms) {
    if (ms < 0) ms = 0;
    if (ms < 1000) return std::to_string(ms) + " ms";
    const long long h   = ms / 3600000;
    const long long m   = (ms / 60000) % 60;
    const long long rem = ms % 60000;   // the seconds part, in ms
    std::string out;
    if (h > 0) out += std::to_string(h) + " h";
    if (m > 0) out += (out.empty() ? "" : " ") + std::to_string(m) + " m";
    if (rem > 0) {
        std::string sec;
        if (rem % 1000 == 0) {
            sec = std::to_string(rem / 1000);
        } else {
            char buf[32];
            std::snprintf(buf, sizeof(buf), "%.3f", static_cast<double>(rem) / 1000.0);
            sec = buf;   // 30.500 -> 30.5
            while (!sec.empty() && sec.back() == '0') sec.pop_back();
        }
        out += (out.empty() ? "" : " ") + sec + " s";
    }
    return out;
}

} // namespace firedog
