// PageOee.cpp : the OEE page — how the station spent each 24 h day. A live
// state badge with the DOWN / UP declaration, a day picker, a 24 h timeline,
// the KPI tiles, the time per state, the downtime log and the last seven days.
// Definitions of App members — see App.h. Design: docs/oee-design.md.
//
// Operator level: it only reads. The DOWN / UP form signs the maintenance log
// and is kRoleDowntime (Technician). Ported from Horizon's FAAM.UI OEE page, so
// the two machines' numbers mean the same thing.
#include "App.h"

#include "imgui.h"

#include "core/OeeTracker.h"

#include <algorithm>
#include <cstdio>
#include <string>

namespace firedog {

namespace {

constexpr OeeState kOeeStates[] = {OeeState::Running, OeeState::Standby, OeeState::Idle,
                                   OeeState::Down, OeeState::Offline};

// Horizon's OeeColors: one colour per state, shared by the badge, the timeline,
// the legend and the breakdown swatches.
ImU32 OeeColor(OeeState s)
{
    switch (s) {
        case OeeState::Running: return IM_COL32(0x43, 0xA0, 0x47, 255);
        case OeeState::Standby: return IM_COL32(0xA5, 0xD6, 0xA7, 255);
        case OeeState::Idle:    return IM_COL32(0xFF, 0xB3, 0x00, 255);
        case OeeState::Down:    return IM_COL32(0xE5, 0x39, 0x35, 255);
        default:                return IM_COL32(0x9E, 0x9E, 0x9E, 255);
    }
}

// Readable text on OeeColor: white on the dark green and the red.
ImU32 OeeTextOn(OeeState s)
{
    return (s == OeeState::Running || s == OeeState::Down) ? IM_COL32(255, 255, 255, 255)
                                                            : IM_COL32(0, 0, 0, 255);
}

void Swatch(OeeState s)
{
    const float  h = ImGui::GetTextLineHeight() * 0.8f;
    const ImVec2 p = ImGui::GetCursorScreenPos();
    const float  y = p.y + (ImGui::GetTextLineHeight() - h) * 0.5f;
    ImGui::GetWindowDrawList()->AddRectFilled(ImVec2(p.x, y), ImVec2(p.x + h, y + h),
                                              OeeColor(s), 2.0f);
    ImGui::Dummy(ImVec2(h, ImGui::GetTextLineHeight()));
}

// The 24 h bar: a coloured span per segment, hour ticks, a tooltip per span.
// Time that has not happened yet (today) is left as the frame background.
void DrawTimeline(const OeeReport& r, float height)
{
    const float  w   = ImGui::GetContentRegionAvail().x;
    const ImVec2 p   = ImGui::GetCursorScreenPos();
    ImDrawList*  dl  = ImGui::GetWindowDrawList();
    const double tot = static_cast<double>(std::max<OeeMs>(r.Total(), 1));
    const auto   xAt = [&](OeeMs t) {
        return p.x + w * static_cast<float>(static_cast<double>(t - r.from) / tot);
    };

    dl->AddRectFilled(p, ImVec2(p.x + w, p.y + height), ImGui::GetColorU32(ImGuiCol_FrameBg));
    for (const OeeSegment& s : r.segments)
        dl->AddRectFilled(ImVec2(xAt(s.start), p.y), ImVec2(xAt(s.end), p.y + height),
                          OeeColor(s.state));
    if (r.elapsedEnd < r.to) {   // "now" on today's bar
        const float x = xAt(r.elapsedEnd);
        dl->AddLine(ImVec2(x, p.y - 3.0f), ImVec2(x, p.y + height + 3.0f),
                    ImGui::GetColorU32(ImGuiCol_Text), 2.0f);
    }
    dl->AddRect(p, ImVec2(p.x + w, p.y + height), ImGui::GetColorU32(ImGuiCol_Border));

    ImGui::InvisibleButton("##oeetimeline", ImVec2(w, height));
    if (ImGui::IsItemHovered()) {
        const float mx = ImGui::GetIO().MousePos.x;
        for (const OeeSegment& s : r.segments) {
            if (mx < xAt(s.start) || mx >= xAt(s.end)) continue;
            ImGui::SetTooltip("%s  %s - %s  (%s)", OeeStateLabel(s.state),
                              OeeFormatLocal(s.start, "%H:%M").c_str(),
                              OeeFormatLocal(s.end, "%H:%M").c_str(),
                              OeeDuration(s.end - s.start).c_str());
            break;
        }
    }

    // Hour labels every 3 h, by the LOCAL hour at each tick, so a DST day still
    // reads true.
    const float ly = ImGui::GetCursorScreenPos().y;
    for (OeeMs t = r.from; t <= r.to; t += 3LL * 3600 * 1000) {
        const float x = xAt(t);
        dl->AddLine(ImVec2(x, p.y + height), ImVec2(x, p.y + height + 4.0f),
                    ImGui::GetColorU32(ImGuiCol_TextDisabled));
        const std::string lbl = (t == r.to) ? std::string("24") : OeeFormatLocal(t, "%H");
        const float tw = ImGui::CalcTextSize(lbl.c_str()).x;
        dl->AddText(ImVec2(std::clamp(x - tw * 0.5f, p.x, p.x + w - tw), ly + 4.0f),
                    ImGui::GetColorU32(ImGuiCol_TextDisabled), lbl.c_str());
    }
    ImGui::Dummy(ImVec2(w, ImGui::GetTextLineHeight() + 6.0f));
}

void KpiTile(const char* id, const char* caption, const std::string& value, const char* sub,
             float w, float h)
{
    ImGui::BeginChild(id, ImVec2(w, h), ImGuiChildFlags_Borders);
    ImGui::TextDisabled("%s", caption);
    DrawBigLabel(value.c_str(), ImVec2(ImGui::GetContentRegionAvail().x, h * 0.42f),
                 ImGui::GetStyleColorVec4(ImGuiCol_Text));
    if (sub && *sub) ImGui::TextDisabled("%s", sub);
    ImGui::EndChild();
}

std::string OptPct(const std::optional<double>& v) { return v ? OeePct(*v) : std::string("-"); }

} // namespace

void App::RefreshOeeReports()
{
    const OeeMs  now   = OeeNowMs();
    const OeeMs  today = OeeLocalDayStart(now);
    const OeeMs  day   = (oeeDay_ > 0 && oeeDay_ < today) ? oeeDay_ : today;
    const double t     = ImGui::GetTime();
    const bool   stale = oeeRepRev_ != oee_.Revision() || oeeRepDay_ != day ||
                       (day == today && t - oeeRepAt_ >= 1.0);
    if (!stale) return;
    oeeRepRev_ = oee_.Revision();
    oeeRepDay_ = day;
    oeeRepAt_  = t;
    oeeWeek_.clear();
    for (int i = 0; i < 7; ++i) oeeWeek_.push_back(oee_.DayReport(OeeAddLocalDays(day, -i), now));
    oeeReport_ = oeeWeek_.front();
}

void App::DrawOeePanel()
{
    const OeeMs now   = OeeNowMs();
    const OeeMs today = OeeLocalDayStart(now);

    // ---- The live state, and the declaration ----
    const OeeTracker::Live live = oee_.Status(now);
    {
        const char*  lbl = OeeStateLabel(live.state);
        const ImVec2 sz(170.0f, ImGui::GetFrameHeight() * 1.6f);
        const ImVec2 p  = ImGui::GetCursorScreenPos();
        ImDrawList*  dl = ImGui::GetWindowDrawList();
        dl->AddRectFilled(p, ImVec2(p.x + sz.x, p.y + sz.y), OeeColor(live.state), 4.0f);
        const ImVec2 ts = ImGui::CalcTextSize(lbl);
        dl->AddText(ImVec2(p.x + (sz.x - ts.x) * 0.5f, p.y + (sz.y - ts.y) * 0.5f),
                    OeeTextOn(live.state), lbl);
        ImGui::Dummy(sz);
    }
    ImGui::SameLine();
    ImGui::BeginGroup();
    ImGui::Text("since %s  -  %s", OeeFormatLocal(live.since, "%a %H:%M").c_str(),
                OeeDuration(now - live.since).c_str());
    if (const OeeDowntime* d = oee_.OpenDowntime()) {
        ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(OeeColor(OeeState::Down)),
                           "declared by %s  [%s]  %s", d->downByName.c_str(),
                           d->category.c_str(), d->reason.c_str());
    } else {
        ImGui::TextDisabled("standby turns idle after %d min with nothing running",
                            static_cast<int>(kOeeIdleThresholdMs / 60000));
    }
    ImGui::EndGroup();

    ImGui::SameLine();
    const char* btn = oee_.OpenDowntime() ? "Bring station UP..." : "Declare DOWN...";
    const float bw  = ImGui::CalcTextSize(btn).x + ImGui::GetStyle().FramePadding.x * 2.0f;
    ImGui::SetCursorPosX(std::max(ImGui::GetCursorPosX(), ImGui::GetContentRegionMax().x - bw));
    if (CanUse(kRoleDowntime)) {
        if (ImGui::Button(btn) && !oeeFormOpen_) {
            oeeFormOpen_   = true;
            oeeReason_[0]  = '\0';
            oeeRootCause_[0] = '\0';
            oeeAction_[0]  = '\0';
            oeeMsg_.clear();
        }
    } else {
        ImGui::BeginDisabled();
        ImGui::Button(btn);
        ImGui::EndDisabled();
        if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
            ImGui::SetTooltip("A technician declares the station down and brings it back up.");
    }
    if (oeeFormOpen_ && CanUse(kRoleDowntime)) DrawOeeDowntimeForm();
    if (!oeeMsg_.empty()) ImGui::TextWrapped("%s", oeeMsg_.c_str());
    ImGui::Separator();

    if (oeeWeek_.empty()) {
        ImGui::TextDisabled("Reading the OEE record...");
        return;
    }
    const OeeReport& r   = oeeReport_;
    const OeeMs      day = oeeRepDay_;

    // ---- Which day ----
    if (ImGui::ArrowButton("##oeeprev", ImGuiDir_Left)) oeeDay_ = OeeAddLocalDays(day, -1);
    ImGui::SameLine();
    ImGui::BeginDisabled(day >= today);
    if (ImGui::Button("Today")) oeeDay_ = 0;
    ImGui::SameLine();
    if (ImGui::ArrowButton("##oeenext", ImGuiDir_Right)) {
        const OeeMs next = OeeAddLocalDays(day, 1);
        oeeDay_ = (next >= today) ? 0 : next;
    }
    ImGui::EndDisabled();
    ImGui::SameLine();
    if (day >= today)
        ImGui::Text("Today so far - %.1f h of %.0f h elapsed",
                    static_cast<double>(r.Elapsed()) / 3.6e6,
                    static_cast<double>(r.Total()) / 3.6e6);
    else
        ImGui::Text("%s - %.0f h", OeeFormatLocal(day, "%a %Y-%m-%d").c_str(),
                    static_cast<double>(r.Total()) / 3.6e6);

    // ---- Timeline ----
    ImGui::Spacing();
    DrawTimeline(r, ImGui::GetFrameHeight() * 1.5f);
    for (OeeState s : kOeeStates) {
        Swatch(s);
        ImGui::SameLine();
        ImGui::TextDisabled("%s", OeeStateLabel(s));
        ImGui::SameLine(0.0f, 18.0f);
    }
    ImGui::NewLine();
    ImGui::Spacing();

    // ---- KPI tiles ----
    {
        const float sp = ImGui::GetStyle().ItemSpacing.x;
        const float w  = (ImGui::GetContentRegionAvail().x - sp * 4.0f) / 5.0f;
        const float h  = ImGui::GetFrameHeight() * 4.2f;
        char quality[96];
        std::snprintf(quality, sizeof(quality), "%d pass / %d fail%s", r.passed, r.failed,
                      r.aborted > 0 ? " (+aborted)" : "");
        KpiTile("##kpiup", "Uptime - online and not down", OeePct(r.Uptime()), "", w, h);
        ImGui::SameLine();
        KpiTile("##kpiav", "Availability - time not down", OeePct(r.Availability()), "", w, h);
        ImGui::SameLine();
        KpiTile("##kpiut", "Utilization - running + standby", OeePct(r.Utilization()), "", w, h);
        ImGui::SameLine();
        KpiTile("##kpiq", "Quality - pass / (pass + fail)", OptPct(r.Quality()), quality, w, h);
        ImGui::SameLine();
        KpiTile("##kpioee", "OEE - utilization x quality", OptPct(r.Oee()), "", w, h);
    }
    ImGui::Spacing();

    // ---- Time breakdown ----
    ImGui::SeparatorText("Time breakdown");
    if (ImGui::BeginTable("##oeebreak", 4,
                          ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerH |
                              ImGuiTableFlags_SizingFixedFit)) {
        ImGui::TableSetupColumn("State", ImGuiTableColumnFlags_WidthFixed, 160.0f);
        ImGui::TableSetupColumn("Time", ImGuiTableColumnFlags_WidthFixed, 90.0f);
        ImGui::TableSetupColumn("% elapsed", ImGuiTableColumnFlags_WidthFixed, 90.0f);
        ImGui::TableSetupColumn("% of day", ImGuiTableColumnFlags_WidthFixed, 90.0f);
        ImGui::TableHeadersRow();
        for (OeeState s : kOeeStates) {
            const OeeMs t = r.TimeIn(s);
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            Swatch(s);
            ImGui::SameLine();
            ImGui::TextUnformatted(OeeStateLabel(s));
            ImGui::TableNextColumn();
            ImGui::TextUnformatted(OeeHm(t).c_str());
            ImGui::TableNextColumn();
            ImGui::TextUnformatted(r.Elapsed() > 0 ? OeePct(r.Ratio(t)).c_str() : "-");
            ImGui::TableNextColumn();
            ImGui::TextUnformatted(
                r.Total() > 0
                    ? OeePct(static_cast<double>(t) / static_cast<double>(r.Total())).c_str()
                    : "-");
        }
        ImGui::EndTable();
    }

    // ---- Downtime log ----
    ImGui::SeparatorText("Downtime log");
    if (ImGui::Button("Maintenance history...")) showMaint_ = true;
    if (r.downtimes.empty()) {
        ImGui::TextDisabled("No downtime on this day.");
    } else if (ImGui::BeginTable("##oeedown", 8,
                                 ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerH |
                                     ImGuiTableFlags_Resizable | ImGuiTableFlags_SizingStretchProp)) {
        ImGui::TableSetupColumn("Down", ImGuiTableColumnFlags_WidthFixed, 90.0f);
        ImGui::TableSetupColumn("Up", ImGuiTableColumnFlags_WidthFixed, 100.0f);
        ImGui::TableSetupColumn("Duration", ImGuiTableColumnFlags_WidthFixed, 80.0f);
        ImGui::TableSetupColumn("Category");
        ImGui::TableSetupColumn("Reason");
        ImGui::TableSetupColumn("Root cause");
        ImGui::TableSetupColumn("Action taken");
        ImGui::TableSetupColumn("Down / up by");
        ImGui::TableHeadersRow();
        for (const OeeDowntime& d : r.downtimes) {
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::TextUnformatted(OeeFormatLocal(d.start, "%a %H:%M").c_str());
            ImGui::TableNextColumn();
            ImGui::TextUnformatted(d.Open() ? "(still down)"
                                            : OeeFormatLocal(d.end, "%a %H:%M").c_str());
            ImGui::TableNextColumn();
            ImGui::TextUnformatted(OeeDuration(d.Duration(now)).c_str());
            ImGui::TableNextColumn();
            ImGui::TextWrapped("%s", d.category.c_str());
            ImGui::TableNextColumn();
            ImGui::TextWrapped("%s", d.reason.c_str());
            ImGui::TableNextColumn();
            ImGui::TextWrapped("%s", d.rootCause.c_str());
            ImGui::TableNextColumn();
            ImGui::TextWrapped("%s", d.actionTaken.c_str());
            ImGui::TableNextColumn();
            if (d.Open()) ImGui::TextWrapped("%s", d.downByName.c_str());
            else ImGui::TextWrapped("%s / %s", d.downByName.c_str(), d.upByName.c_str());
        }
        ImGui::EndTable();
    }

    // ---- The last seven days ----
    ImGui::SeparatorText("Last 7 days");
    if (ImGui::BeginTable("##oeeweek", 11,
                          ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerH |
                              ImGuiTableFlags_SizingFixedFit)) {
        const char* cols[] = {"Day",    "Running", "Standby",     "Idle",      "Down",   "Offline",
                              "Uptime", "Utilization", "Units P/F", "Quality", "OEE"};
        for (const char* c : cols) ImGui::TableSetupColumn(c);
        ImGui::TableHeadersRow();
        for (const OeeReport& w : oeeWeek_) {
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::TextUnformatted(OeeFormatLocal(w.from, "%a %Y-%m-%d").c_str());
            for (OeeState s : kOeeStates) {
                ImGui::TableNextColumn();
                ImGui::TextUnformatted(OeeHm(w.TimeIn(s)).c_str());
            }
            ImGui::TableNextColumn();
            ImGui::TextUnformatted(OeePct(w.Uptime()).c_str());
            ImGui::TableNextColumn();
            ImGui::TextUnformatted(OeePct(w.Utilization()).c_str());
            ImGui::TableNextColumn();
            ImGui::Text("%d / %d", w.passed, w.failed);
            ImGui::TableNextColumn();
            ImGui::TextUnformatted(OptPct(w.Quality()).c_str());
            ImGui::TableNextColumn();
            ImGui::TextUnformatted(OptPct(w.Oee()).c_str());
        }
        ImGui::EndTable();
    }

    ImGui::Spacing();
    ImGui::PushTextWrapPos(0.0f);
    ImGui::TextDisabled(
        "Total available time = 24 h. Running = a program / recipe / script / path move executing. "
        "Standby = online with nothing executing for under %d min (handling between units - "
        "counts as production). Idle = nothing executed for %d min or more (the whole gap). "
        "Down = declared by a technician (wins over everything, even while FireDog is closed). "
        "Offline = FireDog not running. Uptime = (production + idle) / elapsed - Availability = "
        "1 - down / elapsed - Utilization = production / elapsed - Quality = pass / (pass + fail) "
        "of closed DUTs - OEE = utilization x quality (performance not tracked). Data: %s",
        static_cast<int>(kOeeIdleThresholdMs / 60000),
        static_cast<int>(kOeeIdleThresholdMs / 60000), oee_.Dir().c_str());
    ImGui::PopTextWrapPos();
}

// Inline, never a modal: a technician may declare DOWN while a run is blocked
// on the always-on-top prompt window, and a modal would freeze that window.
// Signed by the employee number at the keyboard — there is no name to type,
// because the login already says who this is.
void App::DrawOeeDowntimeForm()
{
    const OeeDowntime* open = oee_.OpenDowntime();
    const ImVec4 red  = ImGui::ColorConvertU32ToFloat4(OeeColor(OeeState::Down));
    const ImVec4 green = ImGui::ColorConvertU32ToFloat4(OeeColor(OeeState::Running));
    const float  boxH = ImGui::GetTextLineHeight() * 3.5f;

    ImGui::BeginChild("##oeeform", ImVec2(0.0f, 0.0f),
                      ImGuiChildFlags_Borders | ImGuiChildFlags_AutoResizeY);
    if (!open) {
        ImGui::TextColored(red, "Declare station DOWN");
        ImGui::TextWrapped("Down time stops counting as up time until someone brings the station "
                           "back UP with the root cause and the action taken.");
        ImGui::Text("Signed by: %s", user_.Label().c_str());

        const std::vector<std::string>& cats = OeeTracker::Categories();
        oeeCategory_ = std::clamp(oeeCategory_, 0, static_cast<int>(cats.size()) - 1);
        ImGui::SetNextItemWidth(280.0f);
        if (ImGui::BeginCombo("Category", cats[static_cast<std::size_t>(oeeCategory_)].c_str())) {
            for (int i = 0; i < static_cast<int>(cats.size()); ++i)
                if (ImGui::Selectable(cats[static_cast<std::size_t>(i)].c_str(), i == oeeCategory_))
                    oeeCategory_ = i;
            ImGui::EndCombo();
        }
        ImGui::TextUnformatted("Reason for down - what is wrong *");
        ImGui::InputTextMultiline("##oeereason", oeeReason_, sizeof(oeeReason_),
                                  ImVec2(-1.0f, boxH));

        ImGui::PushStyleColor(ImGuiCol_Button, red);
        const bool go = ImGui::Button("Confirm DOWN");
        ImGui::PopStyleColor();
        if (go) {
            std::string err;
            if (oee_.GoDown(user_.id, user_.Label(), cats[static_cast<std::size_t>(oeeCategory_)],
                            oeeReason_, &err)) {
                oeeFormOpen_ = false;
                oeeMsg_      = "Station declared DOWN.";
            } else {
                oeeMsg_ = err;
            }
        }
    } else {
        ImGui::TextColored(green, "Bring station UP");
        ImGui::TextWrapped("Record what was really wrong and what was done - it goes into the "
                           "maintenance history.");
        ImGui::TextWrapped("Down since %s (%s), declared by %s  [%s]: %s",
                           OeeFormatLocal(open->start, "%a %Y-%m-%d %H:%M").c_str(),
                           OeeDuration(open->Duration(OeeNowMs())).c_str(),
                           open->downByName.c_str(), open->category.c_str(),
                           open->reason.c_str());
        ImGui::Text("Signed by: %s", user_.Label().c_str());
        ImGui::TextUnformatted("Root cause - what was really wrong *");
        ImGui::InputTextMultiline("##oeecause", oeeRootCause_, sizeof(oeeRootCause_),
                                  ImVec2(-1.0f, boxH));
        ImGui::TextUnformatted("Troubleshooting / action taken *");
        ImGui::InputTextMultiline("##oeeaction", oeeAction_, sizeof(oeeAction_),
                                  ImVec2(-1.0f, boxH));

        ImGui::PushStyleColor(ImGuiCol_Button, green);
        const bool go = ImGui::Button("Confirm UP");
        ImGui::PopStyleColor();
        if (go) {
            std::string err;
            if (oee_.GoUp(user_.id, user_.Label(), oeeRootCause_, oeeAction_, &err)) {
                oeeFormOpen_ = false;
                oeeMsg_      = "Station is back UP.";
            } else {
                oeeMsg_ = err;
            }
        }
    }
    ImGui::SameLine();
    if (ImGui::Button("Cancel")) {
        oeeFormOpen_ = false;
        oeeMsg_.clear();
    }
    ImGui::EndChild();
}

} // namespace firedog
