// FireDogPor : compare a master product with the same product on this (or any)
// machine, read-only - docs/por-design.md.
//
// The order below is load-bearing. Nothing here may resolve MachinePath() or
// DataPath(): that runs InitPaths, whose first-start side effects (a template
// FireDog.ini, folders, the migration, seeding) a read-only tool must never
// trigger. Hence the log file is redirected BEFORE the first FD_* line (the
// logger would otherwise ask MachinePath for its file), the machine root is
// only PREVIEWED (ResolveRootsReadOnly), and the tool's own state goes under
// %LOCALAPPDATA%\FireDogPor.
#include "PorTool.h"

#include "command/CommandRegistry.h"
#include "command/Commands.h"
#include "core/Log.h"
#include "core/Paths.h"
#include "hardware/HardwareManager.h"
#include "ui/GlHost.h"
#include "ui/Theme.h"
#include "workflow/Workflow.h"

#include <cstdlib>
#include <filesystem>
#include <string>
#include <system_error>

namespace {

std::string Env(const char* name)
{
    char*       v   = nullptr;
    std::size_t len = 0;
    std::string out;
    if (_dupenv_s(&v, &len, name) == 0 && v) {
        out = v;
        std::free(v);
    }
    return out;
}

// The tool's own folder: per user, never beside the exe, never a machine root.
std::string LocalDir()
{
    std::string base = Env("LOCALAPPDATA");
    if (base.empty()) base = Env("TEMP");
    if (base.empty()) base = ".";
    return std::filesystem::path(base).append("FireDogPor").generic_string();
}

} // namespace

int main()
{
    using namespace firedog;

    const std::string local = LocalDir();
    std::error_code ec;
    std::filesystem::create_directories(local + "/logs", ec);
    Logger::SetFileBeforeFirstUse(local + "/logs/firedog_por.log");

    const RootsPreview roots = ResolveRootsReadOnly();

    // The command specs: registration only stores what each command takes. The
    // hardware manager and workflow are constructed and never started - no
    // thread, no connect (docs/por-design.md §5.2).
    HardwareManager hw;
    Workflow        wf(hw);
    CommandRegistry reg;
    RegisterBuiltinCommands(reg, wf, hw);
    FD_INFO("por", "FireDogPor: %zu commands; this machine's root is %s%s", reg.Count(),
            roots.root.c_str(), roots.iniFound ? "" : " (default - no FireDog.ini)");

    PorTool tool(reg, roots, local);
    tool.LoadSettings();

    GlHost host;
    if (!host.Init("FireDog POR - master vs target", 1600, 960, local + "/imgui.ini",
                   tool.Placement())) {
        FD_ERROR("por", "window init failed");
        return 1;
    }
    Theme::Apply(0);
    host.SetFont("C:/Windows/Fonts/segoeui.ttf", 17.0f);

    while (!host.ShouldClose()) {
        host.BeginFrame();
        tool.Draw();
        host.EndFrame();
    }

    tool.SetPlacement(host.Placement());
    tool.SaveSettings();
    host.Shutdown();
    return 0;
}
