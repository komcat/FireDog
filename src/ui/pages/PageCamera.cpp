// PageCamera.cpp : the Camera page (the single Pylon camera) + the shared
// camera texture upload. Definitions of App members — see App.h. Connect /
// disconnect / exposure all go through Workflow; the pixel->mm calibration is
// edited live here and saved back into the camera's hardware.json entry.
#include "App.h"

#include "imgui.h"

#include "ui/Widgets.h"
#include "core/Log.h"

#include <cstdio>

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

void App::DrawCameraDetailPanel()
{
    const ConnState st  = hardware_.CameraState();
    PylonCamera&    cam = hardware_.Camera();

    // ---- Exposure (µs; log slider — useful ranges span decades) ----
    ImGui::TextUnformatted("Exposure");
    ImGui::Separator();
    if (st != ConnState::Connected) {
        ImGui::TextDisabled("connect the camera to adjust exposure");
        camExpoPulled_ = false;
    } else {
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

    ImGui::Spacing();
    // ---- Pixel -> mm calibration (drives Aux overlay + click-to-move live) ----
    ImGui::TextUnformatted("Pixel -> mm calibration");
    ImGui::Separator();
    ImGui::SetNextItemWidth(160.0f);
    ImGui::InputFloat("X mm/px", &pixelToMmX_, 0.0f, 0.0f, "%.5f");
    ImGui::SetNextItemWidth(160.0f);
    ImGui::InputFloat("Y mm/px", &pixelToMmY_, 0.0f, 0.0f, "%.5f");
    if (ImGui::Button("Save to config")) {
        if (HardwareConfig::SaveCameraSettings("config/hardware.json", cameraId_,
                                               pixelToMmX_, pixelToMmY_,
                                               camExpoPulled_ ? camExposureUs_ : 0.0))
            FD_INFO("app", "camera settings saved to config/hardware.json");
        else
            FD_ERROR("app", "failed to save camera settings");
    }
    ImGui::TextDisabled("Edits apply immediately (Aux overlay + click-to-move). Save also "
                        "stores the exposure; a rebuild recopies the repo config over the "
                        "exe-side file.");

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
}

} // namespace firedog
