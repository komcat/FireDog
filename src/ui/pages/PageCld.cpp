// PageCld.cpp : the CLD101x page (Thorlabs laser + TEC controller).
// Definitions of App members — see App.h. Every command goes through Workflow;
// nothing is written to the outputs at connect, so laser ON is always an
// explicit action here.
#include "App.h"

#include "imgui.h"

#include "ui/Widgets.h"
#include "core/DataBus.h"
#include "core/Log.h"
#include "core/Topics.h"

#include <cfloat>
#include <cstdio>

namespace firedog {

void App::DrawCldListPanel()
{
    ImGui::TextUnformatted("CLD101x");
    ImGui::Separator();
    if (cldId_.empty()) {
        ImGui::TextDisabled("no CLD101x configured in hardware.json");
        return;
    }

    const ConnState st = hardware_.CldState();
    ImGui::TextColored(StateColor(st), "%s", StateGlyph(st));
    ImGui::SameLine();
    ImGui::TextUnformatted(hardware_.CldName().c_str());
    ImGui::SameLine();
    ImGui::TextColored(StateColor(st), "%s", StateText(st));

    ImGui::Spacing();
    const bool busy = (st == ConnState::Connecting);
    ImGui::BeginDisabled(busy || st == ConnState::Connected);
    if (ImGui::Button("Connect", ImVec2(-FLT_MIN, 0.0f))) workflow_.ConnectCld();
    ImGui::EndDisabled();
    ImGui::BeginDisabled(busy || st != ConnState::Connected);
    if (ImGui::Button("Disconnect", ImVec2(-FLT_MIN, 0.0f))) {
        workflow_.DisconnectCld();
        cldPulled_ = false;   // re-seed edits on the next connect
    }
    ImGui::EndDisabled();

    if (st == ConnState::Failed && hardware_.Cld()) {
        const std::string err = hardware_.Cld()->LastError();
        if (!err.empty()) {
            ImGui::Spacing();
            ImGui::PushStyleColor(ImGuiCol_Text, StateColor(ConnState::Failed));
            ImGui::TextWrapped("%s", err.c_str());
            ImGui::PopStyleColor();
        }
    }
}

void App::DrawCldDetailPanel()
{
    const ConnState st  = hardware_.CldState();
    CLD101x*        cld = hardware_.Cld();
    const bool connected = (st == ConnState::Connected) && cld;
    const CLD101x::Status s = cld ? cld->GetStatus() : CLD101x::Status{};

    if (connected && !cldPulled_) {   // seed edits from the device's setpoints
        if (s.tempSetC   != 0.0) cldTempSet_    = static_cast<float>(s.tempSetC);
        if (s.laserSetMA != 0.0) cldCurrentSet_ = static_cast<float>(s.laserSetMA);
        cldPulled_ = true;
    }
    if (!connected) cldPulled_ = false;

    // ---- TEC (temperature control; run it before enabling the laser) ----
    ImGui::TextUnformatted("TEC");
    ImGui::Separator();
    ImGui::BeginDisabled(!connected);
    ImGui::SetNextItemWidth(140.0f);
    ImGui::InputFloat("C##temp", &cldTempSet_, 0.0f, 0.0f, "%.2f");
    if (cldTempSet_ < 15.0f) cldTempSet_ = 15.0f;   // sane laser-diode window
    if (cldTempSet_ > 35.0f) cldTempSet_ = 35.0f;
    ImGui::SameLine();
    if (ImGui::Button("Apply##temp")) workflow_.SetCldTemperature(cldTempSet_);
    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.20f, 0.60f, 0.25f, 1.0f));
    if (ImGui::Button("TEC ON")) workflow_.CldTecOn();
    ImGui::PopStyleColor();
    ImGui::SameLine();
    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.75f, 0.20f, 0.20f, 1.0f));
    if (ImGui::Button("TEC OFF")) workflow_.CldTecOff();
    ImGui::PopStyleColor();
    ImGui::EndDisabled();
    if (connected) {
        char tval[32];
        if (s.tempValid) std::snprintf(tval, sizeof(tval), "%.2f C", s.tempC);
        else             std::snprintf(tval, sizeof(tval), "-- C (no sensor)");
        if (s.tecOn) ImGui::TextColored(ImVec4(0.30f, 0.85f, 0.40f, 1.0f),
                                        "TEC ON  |  %s (set %.2f)", tval, s.tempSetC);
        else         ImGui::TextDisabled("TEC OFF  |  %s", tval);
        if (s.valid && !s.tempValid)
            ImGui::TextColored(ImVec4(1.0f, 0.45f, 0.20f, 1.0f),
                               "temperature reads invalid - thermistor disconnected?");
    } else {
        ImGui::TextDisabled("connect the controller to operate it");
    }

    ImGui::Spacing();
    // ---- Laser (current control; ON is always an explicit action) ----
    ImGui::TextUnformatted("Laser");
    ImGui::Separator();
    ImGui::BeginDisabled(!connected);
    ImGui::SetNextItemWidth(140.0f);
    ImGui::InputFloat("mA##ld", &cldCurrentSet_, 0.0f, 0.0f, "%.1f");
    if (cldCurrentSet_ < 0.0f)   cldCurrentSet_ = 0.0f;
    if (cldCurrentSet_ > 500.0f) cldCurrentSet_ = 500.0f;   // CLD101x LD range cap
    ImGui::SameLine();
    if (ImGui::Button("Apply##ld")) workflow_.SetCldLaserCurrentMA(cldCurrentSet_);
    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.20f, 0.60f, 0.25f, 1.0f));
    if (ImGui::Button("Laser ON")) workflow_.CldLaserOn();
    ImGui::PopStyleColor();
    ImGui::SameLine();
    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.75f, 0.20f, 0.20f, 1.0f));
    if (ImGui::Button("Laser OFF")) workflow_.CldLaserOff();
    ImGui::PopStyleColor();
    ImGui::EndDisabled();
    if (connected) {
        if (s.laserOn) ImGui::TextColored(ImVec4(1.0f, 0.75f, 0.20f, 1.0f),
                                          "LASER ON  |  %.1f mA (set %.1f)",
                                          s.laserActMA, s.laserSetMA);
        else           ImGui::TextDisabled("laser OFF  |  set %.1f mA", s.laserSetMA);
    }
    if (connected && s.laserOn && !s.tecOn) {
        ImGui::TextColored(ImVec4(1.0f, 0.45f, 0.20f, 1.0f),
                           "warning: laser is ON with TEC OFF");
    }

    ImGui::Spacing();
    if (ImGui::Button("Save to config")) {
        if (HardwareConfig::SaveCldSettings("config/hardware.json", cldId_,
                                            cldTempSet_, cldCurrentSet_))
            FD_INFO("app", "CLD101x defaults saved to config/hardware.json");
        else
            FD_ERROR("app", "failed to save CLD101x defaults");
    }
    ImGui::TextDisabled("Save stores the setpoints as the config defaults (UI seeds — "
                        "never applied automatically); a rebuild recopies the repo config.");
}

void App::DrawCldInfoPanel()
{
    ImGui::TextUnformatted("Controller info");
    ImGui::Separator();

    const ConnState st  = hardware_.CldState();
    CLD101x*        cld = hardware_.Cld();

    ImGui::TextDisabled("State");
    ImGui::TextColored(StateColor(st), "%s", StateText(st));
    ImGui::Spacing();
    ImGui::TextDisabled("Device ID");
    ImGui::TextWrapped("%s", (cld && !cld->DeviceId().empty()) ? cld->DeviceId().c_str() : "-");

    if (cld) {
        const CLD101x::Status s = cld->GetStatus();
        ImGui::Spacing();
        ImGui::TextDisabled("TEC");
        ImGui::TextUnformatted(s.tecOn ? "ON" : "OFF");
        if (s.valid && s.tempValid) ImGui::Text("%.2f C (set %.2f)", s.tempC, s.tempSetC);
        else if (s.valid)           ImGui::TextDisabled("no sensor reading (thermistor?)");
        else                        ImGui::TextDisabled("no reading yet");
        ImGui::Spacing();
        ImGui::TextDisabled("Laser");
        if (s.laserOn) ImGui::TextColored(ImVec4(1.0f, 0.75f, 0.20f, 1.0f), "ON");
        else           ImGui::TextUnformatted("OFF");
        ImGui::Text("%.1f mA (set %.1f)", s.laserActMA, s.laserSetMA);
        ImGui::Spacing();
        ImGui::TextDisabled("Polling");
        ImGui::TextUnformatted(cld->IsPolling() ? "yes" : "no");
    }
}

} // namespace firedog
