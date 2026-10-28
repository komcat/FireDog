// PageVersion.cpp : the Version page — this build, and the software update.
//
// The update is three steps on one page: Check (GitHub, on the updater's
// thread), Download (verified by Velopack, beside the install - nothing that
// runs changes) and Restart. Restart only ASKS: it marks the update and closes
// FireDog through the normal exit, and main() hands it to Update.exe once App
// and every device are gone. Design: docs/software-update-design.md.
#include "App.h"

#include "imgui.h"

#include "core/BuildInfo.h"
#include "core/Paths.h"
#include "update/AppUpdater.h"

namespace firedog {

void App::DrawVersionPanel()
{
    const BuildInfo&         b  = GetBuildInfo();
    const AppUpdater::Status us = updater_.GetStatus();
    using S = AppUpdater::State;

    ImGui::TextUnformatted("This build");
    ImGui::Separator();
    if (ImGui::BeginTable("##verbuild", 2, ImGuiTableFlags_SizingFixedFit)) {
        auto row = [](const char* k, const std::string& v) {
            ImGui::TableNextRow();
            ImGui::TableNextColumn(); ImGui::TextDisabled("%s", k);
            ImGui::TableNextColumn(); ImGui::TextUnformatted(v.empty() ? "-" : v.c_str());
        };
        row("version", b.describe);
        row("commit", b.commit + (b.commitDate.empty() ? "" : "  (" + b.commitDate + ")"));
        row("built", b.builtAt);
        row("installed", us.state == S::NotInstalled ? std::string("not a Setup install")
                                                     : us.installedVersion);
        row("exe", b.exePath);
        row("FireDog.ini", IniPath());
        row("machine root", MachineRoot());
        ImGui::EndTable();
    }

    ImGui::Spacing();
    ImGui::Spacing();
    ImGui::TextUnformatted("Software update");
    ImGui::Separator();

    if (us.state == S::NotInstalled) {
        ImGui::TextWrapped("This copy was not installed by the release's Setup, so it cannot "
                           "update itself. Install with Install-FireDog.cmd from the GitHub "
                           "release (it installs to C:\\FireDog); later releases then arrive "
                           "here.");
        ImGui::TextDisabled("%s", us.message.c_str());
        return;
    }

    switch (us.state) {
        case S::Idle:        ImGui::TextUnformatted("Not checked yet."); break;
        case S::Checking:    ImGui::TextUnformatted("Checking GitHub for a newer release..."); break;
        case S::UpToDate:    ImGui::Text("Up to date (checked %s).", us.checkedAt.c_str()); break;
        case S::Available:   ImGui::Text("FireDog %s is available.", us.latestVersion.c_str()); break;
        case S::Downloading:
            ImGui::Text("Downloading FireDog %s...", us.latestVersion.c_str());
            ImGui::ProgressBar(static_cast<float>(us.progress) / 100.0f, ImVec2(-FLT_MIN, 0.0f));
            break;
        case S::Ready:
            ImGui::Text("FireDog %s is downloaded. Restart to install it.",
                        us.latestVersion.c_str());
            break;
        case S::Failed:
            ImGui::TextColored(ImVec4(0.95f, 0.45f, 0.35f, 1.0f), "%s", us.message.c_str());
            break;
        case S::NotInstalled: break;
    }

    const bool working = us.state == S::Checking || us.state == S::Downloading;
    ImGui::BeginDisabled(working);
    if (ImGui::Button("Check now")) updater_.RequestCheck();
    ImGui::EndDisabled();

    if (us.state == S::Available) {
        ImGui::SameLine();
        // Downloading changes nothing that runs, so it is allowed mid-run.
        if (ImGui::Button(("Download " + us.latestVersion).c_str())) updater_.RequestDownload();
    }

    if (us.state == S::Ready) {
        // Restart closes the application without the exit confirm, so it is
        // refused while anything could be lost: a run (the same gate as every
        // Run/Go surface) or an editor with unsaved changes.
        const bool busy  = orch_.Busy();
        const bool dirty = session_.RecipesDirty() || session_.ProgramsDirty() || ScriptDirty();
        ImGui::SameLine();
        ImGui::BeginDisabled(busy || dirty);
        if (ImGui::Button(("Restart and install " + us.latestVersion).c_str()) &&
            updater_.RequestRestart()) {
            FD_INFO("update", "%s (%s) restarts FireDog to install %s", user_.id.c_str(),
                    RoleName(user_.role), us.latestVersion.c_str());
            exitConfirmed_ = true;
            host_.SetShouldClose(true);
        }
        ImGui::EndDisabled();
        if ((busy || dirty) && ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
            ImGui::SetTooltip(busy ? "A run is in progress - stop it first"
                                   : "Save or discard the unsaved recipe / program / script "
                                     "edits first");
        ImGui::TextDisabled("FireDog closes, every device disconnects, the new version is "
                            "installed and starts again. The machine root is not touched.");
    }

    if (!us.notes.empty()) {
        ImGui::Spacing();
        ImGui::Text("Release notes - %s", us.latestVersion.c_str());
        ImGui::BeginChild("##relnotes", ImGui::GetContentRegionAvail(), ImGuiChildFlags_Borders);
        ImGui::TextWrapped("%s", us.notes.c_str());
        ImGui::EndChild();
    }
}

} // namespace firedog
