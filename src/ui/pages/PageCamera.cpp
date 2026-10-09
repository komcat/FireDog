// PageCamera.cpp : the Camera page (the single Pylon camera) + the shared
// camera texture upload. Definitions of App members — see App.h. Connect /
// disconnect / exposure all go through Workflow; the pixel->mm calibration is
// edited live here and saved back into the camera's hardware.json entry.
#include "App.h"

#include "imgui.h"

#include "ui/Widgets.h"
#include "core/Log.h"
#include "vision/HalconEngine.h"

#include <cfloat>
#include <cstdio>
#include <string>

namespace firedog {

// Pull the newest camera frame into the shared GL texture. Safe to call from
// several views in one frame — only a fresh frame re-uploads.
void App::UpdateCameraTexture()
{
    bool isNew = false;
    if (hardware_.Camera().GetLatestFrame(frameBuf_, cameraW_, cameraH_, isNew) &&
        cameraW_ > 0 && cameraH_ > 0) {
        if (isNew || cameraTex_ == 0) {
            cameraTex_ = host_.EnsureTextureRGB(cameraTex_, cameraW_, cameraH_, frameBuf_.data());
        }
    }
}

void App::DrawCameraListPanel()
{
    ImGui::TextUnformatted("Camera");
    ImGui::Separator();
    if (cameraId_.empty()) {
        ImGui::TextDisabled("no camera configured in hardware.json");
        return;
    }

    const ConnState st = hardware_.CameraState();
    ImGui::TextColored(StateColor(st), "%s", StateGlyph(st));
    ImGui::SameLine();
    ImGui::TextUnformatted(hardware_.CameraName().c_str());
    ImGui::SameLine();
    ImGui::TextColored(StateColor(st), "%s", StateText(st));

    ImGui::Spacing();
    const bool busy = (st == ConnState::Connecting);
    ImGui::BeginDisabled(busy || st == ConnState::Connected);
    if (ImGui::Button("Connect", ImVec2(-FLT_MIN, 0.0f))) workflow_.ConnectCamera();
    ImGui::EndDisabled();
    ImGui::BeginDisabled(busy || st != ConnState::Connected);
    if (ImGui::Button("Disconnect", ImVec2(-FLT_MIN, 0.0f))) {
        workflow_.DisconnectCamera();
        camExpoPulled_ = false;   // re-pull exposure on the next connect
    }
    ImGui::EndDisabled();

    if (st == ConnState::Failed) {
        const std::string err = hardware_.Camera().LastError();
        if (!err.empty()) {
            ImGui::Spacing();
            ImGui::PushStyleColor(ImGuiCol_Text, StateColor(ConnState::Failed));
            ImGui::TextWrapped("%s", err.c_str());
            ImGui::PopStyleColor();
        }
    }
}

// Exposure controls (µs; log slider — useful ranges span decades). Lives here
// because this page owns them, but the Jog window's Camera tab calls it too, so
// both surfaces share one seeding rule and one apply path.
void App::DrawCameraExposureControls()
{
    const ConnState st  = hardware_.CameraState();
    PylonCamera&    cam = hardware_.Camera();

    if (st != ConnState::Connected) {
        ImGui::TextDisabled("connect the camera to adjust exposure");
        camExpoPulled_ = false;
        return;
    }
    if (!camExpoPulled_) {   // seed the edit value from the camera once
        const double cur = cam.ExposureUs();
        if (cur > 0.0) { camExposureUs_ = static_cast<float>(cur); camExpoPulled_ = true; }
    }
    double lo = 0.0, hi = 0.0;
    cam.ExposureRange(lo, hi);
    if (lo <= 0.0 || hi <= lo) { lo = 20.0; hi = 100000.0; }   // sane fallback

    ImGui::SetNextItemWidth(-FLT_MIN);
    ImGui::SliderFloat("##expo", &camExposureUs_,
                       static_cast<float>(lo), static_cast<float>(hi),
                       "%.0f us", ImGuiSliderFlags_Logarithmic);
    if (ImGui::IsItemDeactivatedAfterEdit())          // apply once on release
        workflow_.SetCameraExposure(camExposureUs_);

    ImGui::SetNextItemWidth(140.0f);
    ImGui::InputFloat("us##expoIn", &camExposureUs_, 0.0f, 0.0f, "%.0f");
    ImGui::SameLine();
    if (ImGui::Button("Apply")) workflow_.SetCameraExposure(camExposureUs_);
    ImGui::TextDisabled("range %.0f - %.0f us", lo, hi);
}

void App::DrawCameraDetailPanel()
{
    // ---- Exposure ----
    ImGui::TextUnformatted("Exposure");
    ImGui::Separator();
    DrawCameraExposureControls();

    ImGui::Spacing();
    // ---- Pixel -> mm calibration (drives Aux overlay + click-to-move live) ----
    ImGui::TextUnformatted("Pixel -> mm calibration");
    ImGui::Separator();
    ImGui::SetNextItemWidth(160.0f);
    bool calEdited = ImGui::InputFloat("X mm/px", &pixelToMmX_, 0.0f, 0.0f, "%.5f");
    ImGui::SetNextItemWidth(160.0f);
    calEdited |= ImGui::InputFloat("Y mm/px", &pixelToMmY_, 0.0f, 0.0f, "%.5f");
    // Push edits into layer 2 as well: the UI converts pixels for the manual
    // crosshair pick and click-to-move, but an AUTO alignment station converts
    // on a worker thread with no UI in the loop, and the two must not disagree.
    if (calEdited)
        workflow_.SetCameraPixelToMm(static_cast<double>(pixelToMmX_),
                                     static_cast<double>(pixelToMmY_));
    if (ImGui::Button("Save to config")) {
        if (HardwareConfig::SaveCameraSettings(HardwareConfigPath(), cameraId_,
                                               pixelToMmX_, pixelToMmY_,
                                               camExpoPulled_ ? camExposureUs_ : 0.0))
            FD_INFO("app", "camera settings saved to %s", HardwareConfigPath().c_str());
        else
            FD_ERROR("app", "failed to save camera settings");
    }
    ImGui::TextDisabled("Edits apply immediately (Aux overlay + click-to-move). Save also "
                        "stores the exposure in this machine's config (a build never "
                        "overwrites it).");

    ImGui::Spacing();
    // ---- Live preview (fit to the panel, aspect preserved) ----
    ImGui::TextUnformatted("Preview");
    ImGui::Separator();
    UpdateCameraTexture();
    const ImVec2 avail = ImGui::GetContentRegionAvail();
    if (cameraTex_ && cameraW_ > 0 && cameraH_ > 0 && avail.x > 20.0f && avail.y > 20.0f) {
        float w = avail.x;
        float h = w * static_cast<float>(cameraH_) / static_cast<float>(cameraW_);
        if (h > avail.y) { h = avail.y; w = h * static_cast<float>(cameraW_) / static_cast<float>(cameraH_); }
        ImGui::Image((ImTextureID)(intptr_t)cameraTex_, ImVec2(w, h));   // C-style cast: void*/ImU64 ImTextureID
    } else {
        ImGui::TextDisabled("no frames yet");
    }
}

void App::DrawCameraInfoPanel()
{
    ImGui::TextUnformatted("Camera info");
    ImGui::Separator();

    const ConnState         st   = hardware_.CameraState();
    const PylonCamera::Info info = hardware_.Camera().GetInfo();

    ImGui::TextDisabled("State");
    ImGui::TextColored(StateColor(st), "%s", StateText(st));
    ImGui::Spacing();
    ImGui::TextDisabled("Model");
    ImGui::TextWrapped("%s", info.model.empty() ? "-" : info.model.c_str());
    ImGui::TextDisabled("Serial");
    ImGui::TextUnformatted(info.serial.empty() ? "-" : info.serial.c_str());
    ImGui::TextDisabled("IP");
    ImGui::TextUnformatted(info.ip.empty() ? "-" : info.ip.c_str());
    ImGui::Spacing();
    ImGui::TextDisabled("Stream");
    if (info.grabbing) {
        if (info.width > 0) ImGui::Text("%d x %d px", info.width, info.height);
        else                ImGui::TextUnformatted("waiting for first frame");
        ImGui::Text("%.1f fps", info.fps);
    } else {
        ImGui::TextUnformatted("not grabbing");
    }
    ImGui::Spacing();
    ImGui::TextDisabled("Calibration");
    ImGui::Text("X %.5f mm/px", pixelToMmX_);
    ImGui::Text("Y %.5f mm/px", pixelToMmY_);

    ImGui::Spacing();
    ImGui::Separator();
    DrawHalconPanel(false);
}

// ---- HALCON: build / runtime / licence -------------------------------------
// Three independent gates, shown separately because they fail separately —
// "compiled in but not licensed" and "not built with the SDK" need different
// fixes, and only this panel can tell an operator which it is. Auto-focus does
// NOT depend on any of it (plain C++ focus metric).
void App::DrawHalconPanel(bool detail)
{
    ImGui::TextDisabled("Vision (HALCON)");
    const HalconStatus& hal = HalconEngine::Instance().Status();
    const ImVec4 okCol(0.35f, 0.85f, 0.45f, 1.0f), badCol(0.95f, 0.45f, 0.35f, 1.0f);
    ImGui::TextColored(hal.Available() ? okCol : badCol, "%s",
                       hal.Available() ? "available" : "unavailable");
    ImGui::TextColored(hal.compiledIn ? okCol : badCol, "%s", hal.compiledIn ? "[+]" : "[-]");
    ImGui::SameLine(); ImGui::TextUnformatted("SDK compiled in");
    ImGui::TextColored(hal.runtimeLoaded ? okCol : badCol, "%s", hal.runtimeLoaded ? "[+]" : "[-]");
    ImGui::SameLine(); ImGui::TextUnformatted("runtime DLLs");
    ImGui::TextColored(hal.licensed ? okCol : badCol, "%s", hal.licensed ? "[+]" : "[-]");
    ImGui::SameLine(); ImGui::TextUnformatted("licence");
    if (!hal.version.empty()) ImGui::TextDisabled("version %s", hal.version.c_str());
    // The runtime must be the release the SDK was: the probe refuses any other.
    if (!hal.builtVersion.empty()) ImGui::TextDisabled("built for %s", hal.builtVersion.c_str());

    if (detail) {
        // Where it came from: with several releases installed, "which HALCON
        // am I running?" is the first question, and the licence dir is where
        // an operator drops a .dat.
        if (ImGui::BeginTable("##halconwhere", 2, ImGuiTableFlags_SizingFixedFit)) {
            auto row = [](const char* k, const std::string& v) {
                ImGui::TableNextRow();
                ImGui::TableNextColumn(); ImGui::TextDisabled("%s", k);
                ImGui::TableNextColumn(); ImGui::TextUnformatted(v.empty() ? "-" : v.c_str());
            };
            row("install", hal.root);
            row("arch", hal.arch);
            row("licence dir", hal.licenseDir);
            ImGui::EndTable();
        }
        // What each licence file CLAIMS. "valid to" is checked against the
        // running release; whether HALCON accepts the key is the [+] licence
        // gate above, which actually ran a licensed operator.
        if (!hal.licences.empty()) {
            const ImGuiTableFlags tf = ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg |
                                       ImGuiTableFlags_SizingFixedFit;
            if (ImGui::BeginTable("##halconlic", 6, tf)) {
                ImGui::TableSetupColumn("file");
                ImGui::TableSetupColumn("key");
                ImGui::TableSetupColumn("product");
                ImGui::TableSetupColumn("holder");
                ImGui::TableSetupColumn("valid to");
                ImGui::TableSetupColumn("sessions");
                ImGui::TableHeadersRow();
                for (const HalconLicence& l : hal.licences) {
                    ImGui::TableNextRow();
                    ImGui::TableNextColumn(); ImGui::TextUnformatted(l.file.c_str());
                    ImGui::TableNextColumn();
                    // CM: is a CodeMeter container (dongle or soft licence):
                    // the key travels with it, not with this PC.
                    if (l.id.rfind("CM:", 0) == 0)
                        ImGui::Text("CodeMeter %s", l.id.c_str() + 3);
                    else
                        ImGui::TextUnformatted(l.id.empty() ? "-" : l.id.c_str());
                    ImGui::TableNextColumn(); ImGui::TextUnformatted(l.info.empty() ? "-" : l.info.c_str());
                    ImGui::TableNextColumn(); ImGui::TextUnformatted(l.holder.empty() ? "-" : l.holder.c_str());
                    ImGui::TableNextColumn();
                    // "YY.MM" strings of equal width compare as versions.
                    const std::string run = !hal.version.empty() && hal.version != "?"
                                                ? hal.version : hal.builtVersion;
                    const bool covers = run.empty() || run.size() != l.upTo.size() || l.upTo >= run;
                    ImGui::TextColored(covers ? okCol : badCol, "%s", l.upTo.c_str());
                    if (!covers && ImGui::IsItemHovered())
                        ImGui::SetTooltip("this licence does not cover HALCON %s", run.c_str());
                    ImGui::TableNextColumn(); ImGui::TextUnformatted(l.sessions.empty() ? "-" : l.sessions.c_str());
                }
                ImGui::EndTable();
            }
        } else if (!hal.licenseDir.empty()) {
            ImGui::TextDisabled("no *.dat in %s", hal.licenseDir.c_str());
        }
    } else if (!hal.licenseFiles.empty()) {
        ImGui::TextDisabled("%zu licence file(s):", hal.licenseFiles.size());
        for (const std::string& f : hal.licenseFiles) ImGui::TextDisabled("  %s", f.c_str());
    } else if (!hal.licenseDir.empty()) {
        ImGui::TextDisabled("no *.dat in");
        ImGui::TextWrapped("%s", hal.licenseDir.c_str());
    }
    if (!hal.error.empty()) {
        ImGui::PushStyleColor(ImGuiCol_Text, badCol);
        ImGui::TextWrapped("%s", hal.error.c_str());
        ImGui::PopStyleColor();
    }
    // Re-probe without restarting: the usual fix is dropping a .dat into the
    // licence directory, and that should not cost a restart.
    if (ImGui::SmallButton("Re-check")) HalconEngine::Instance().Refresh();
}

} // namespace firedog
