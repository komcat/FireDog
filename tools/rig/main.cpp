// main.cpp : FireDogRig — console hardware bring-up rig for FireDog.
//
// Same drivers, same HardwareManager, same Workflow::Start() sequence as the
// GUI client; no window. Connects what the machine's config/hardware.json
// describes (under the machine root, core/Paths.h), reports what each device
// answers, and edits the connection/tuning fields of that file. It never
// commands hardware — see RigProbe.h. Full reference: docs/rig-console.md.
#include "RigConsole.h"
#include "RigMenu.h"

#include "core/Paths.h"
#include "hardware/HardwareConfig.h"   // HardwareConfigPath

#include <cstdio>
#include <exception>
#include <filesystem>
#include <io.h>
#include <iostream>
#include <string>
#include <vector>

namespace {

void PrintUsage()
{
    std::cout <<
        "\n"
        "  FireDogRig - console hardware bring-up rig for FireDog.\n"
        "\n"
        "  FireDogRig [options]                 numbered menu (default)\n"
        "\n"
        "  Options\n"
        "    --config <path>    hardware config to use (default: the machine's\n"
        "                       config/hardware.json under the root FireDog.ini\n"
        "                       names; the same file the GUI reads)\n"
        "    --timeout <sec>    how long to wait for connects (default 20)\n"
        "    --log <level>      trace | debug | info | warn | error (default info)\n"
        "    --repl             typed command prompt instead of the menu (also the\n"
        "                       automatic choice when stdin is a pipe or a file)\n"
        "\n"
        "  One-shot actions (skip the console; exit 0 only if everything succeeded)\n"
        "    --list                        print the configured devices\n"
        "    --report [<file>]             write the whole configuration as readable\n"
        "                                  text for handover (no hardware needed;\n"
        "                                  default firedog-config-report.txt under\n"
        "                                  the machine root)\n"
        "    --probe [<device>|all]        connect, print live read-back, exit\n"
        "    --set <device>.<field>=<val>  edit one field (repeatable)\n"
        "    --save                        write the --set edits back to the config\n"
        "    --help\n"
        "\n"
        "  Examples\n"
        "    FireDogRig --report\n"
        "    FireDogRig --probe all\n"
        "    FireDogRig --probe hex-left\n"
        "    FireDogRig --set hex-left.ip_address=192.168.0.11 --save\n"
        "\n";
}

// "hex-left.ip_address=192.168.0.11" -> "set hex-left ip_address 192.168.0.11".
// Empty if the argument isn't in that form.
std::string SetArgToCommand(const std::string& arg)
{
    const std::size_t dot = arg.find('.');
    if (dot == std::string::npos || dot == 0) return {};
    const std::size_t eq = arg.find('=', dot + 1);
    if (eq == std::string::npos || eq == dot + 1) return {};

    const std::string device = arg.substr(0, dot);
    const std::string field  = arg.substr(dot + 1, eq - dot - 1);
    const std::string value  = arg.substr(eq + 1);
    if (field.empty() || value.empty()) return {};
    return "set " + device + " " + field + " " + value;
}

} // namespace

int main(int argc, char** argv)
{
    std::string              configPath;   // "" = the machine's (resolved after InitPaths)
    bool                     configExplicit = false;
    double                   timeoutSec = 20.0;
    std::string              logLevel;
    std::vector<std::string> actions;      // executed in order, after the edits
    std::vector<std::string> edits;        // --set commands
    bool                     save = false;
    bool                     forceRepl = false;

    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];

        // Consume the following argument, or complain and bail out.
        std::string val;
        auto next = [&](const char* what) -> bool {
            if (i + 1 >= argc) {
                std::cerr << "[rig] " << a << " needs " << what << "\n";
                return false;
            }
            val = argv[++i];
            return true;
        };

        if (a == "--help" || a == "-h" || a == "/?") { PrintUsage(); return 0; }
        else if (a == "--config") {
            if (!next("a path")) return 2;
            configPath = val;
            configExplicit = true;
        }
        else if (a == "--log") {
            if (!next("a level")) return 2;
            logLevel = val;
        }
        else if (a == "--timeout") {
            if (!next("seconds")) return 2;
            try { timeoutSec = std::stod(val); }
            catch (const std::exception&) {
                std::cerr << "[rig] --timeout expects a number, got '" << val << "'\n";
                return 2;
            }
        }
        else if (a == "--list")  { actions.push_back("list"); }
        else if (a == "--report") {
            // Optional file: "--report handover.txt", or just "--report".
            std::string file;
            if (i + 1 < argc && std::string(argv[i + 1]).rfind("--", 0) != 0) file = argv[++i];
            if (file.empty()) {
                actions.push_back("report");   // the default name, under the machine root
            } else {
                // Resolve against the caller's cwd, like --config: we move to the
                // exe directory below, and a report is written where asked for.
                try { file = std::filesystem::absolute(file).string(); }
                catch (const std::exception& e) {
                    std::cerr << "[rig] bad --report path: " << e.what() << "\n";
                    return 2;
                }
                actions.push_back("report " + file);
            }
        }
        else if (a == "--save")  { save = true; }
        else if (a == "--repl")  { forceRepl = true; }
        else if (a == "--probe") {
            // Optional target: "--probe hex-left", or just "--probe".
            std::string target;
            if (i + 1 < argc && std::string(argv[i + 1]).rfind("--", 0) != 0) target = argv[++i];
            actions.push_back(target.empty() ? "probe all" : "probe " + target);
        }
        else if (a == "--set") {
            if (!next("<device>.<field>=<value>")) return 2;
            const std::string cmd = SetArgToCommand(val);
            if (cmd.empty()) {
                std::cerr << "[rig] --set expects <device>.<field>=<value>, got '"
                          << val << "'\n";
                return 2;
            }
            edits.push_back(cmd);
        }
        else {
            std::cerr << "[rig] unknown argument '" << a << "' (try --help)\n";
            return 2;
        }
    }

    // Resolve an explicit --config against the caller's cwd BEFORE InitPaths
    // moves us to the exe directory. The default is the machine's own copy -
    // the same file the GUI reads and its pages save into.
    if (configExplicit) {
        try { configPath = std::filesystem::absolute(configPath).string(); }
        catch (const std::exception& e) {
            std::cerr << "[rig] bad --config path: " << e.what() << "\n";
            return 2;
        }
    }
    firedog::InitPaths();
    for (const firedog::PathNote& n : firedog::PathNotes())
        (n.error ? std::cerr : std::cout) << "[paths] " << n.text << "\n";
    if (!configExplicit) configPath = firedog::HardwareConfigPath();

    firedog::rig::RigConsole console;
    if (!logLevel.empty()) {
        bool ignored = false;
        console.Execute("loglevel " + logLevel, ignored);
    }

    const bool loaded = console.Init(configPath, timeoutSec);

    const bool oneShot = !actions.empty() || !edits.empty() || save;
    if (!oneShot) {
        console.PrintList();
        // Piped or redirected stdin can't drive a menu sensibly — fall back to
        // the typed prompt so `echo probe all | FireDogRig` still works.
        const bool interactive = _isatty(_fileno(stdin)) != 0;
        if (forceRepl || !interactive) {
            console.PrintHelp();
            return console.RunRepl();
        }
        return firedog::rig::RunMenu(console);
    }

    if (!loaded) return 1;

    bool ok = true;
    for (const std::string& cmd : edits) {
        bool stop = false;
        ok = console.Execute(cmd, stop) && ok;
    }
    if (save) {
        bool stop = false;
        ok = console.Execute("save all", stop) && ok;
    }
    for (const std::string& cmd : actions) {
        bool stop = false;
        ok = console.Execute(cmd, stop) && ok;
    }
    return ok ? 0 : 1;
}
