// RigMenu.cpp
#include "RigMenu.h"

#include "RigConfigEdit.h"
#include "RigConsole.h"

#include <cstddef>
#include <exception>
#include <iostream>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace firedog::rig {
namespace {

constexpr const char* kRule =
    "  ==========================================================================";
constexpr const char* kThin =
    "  --------------------------------------------------------------------------";

std::string Trim(const std::string& s)
{
    const std::size_t a = s.find_first_not_of(" \t\r\n");
    if (a == std::string::npos) return {};
    const std::size_t b = s.find_last_not_of(" \t\r\n");
    return s.substr(a, b - a + 1);
}

// Prompt and read one trimmed line. False means EOF — the caller unwinds.
bool ReadLine(const char* label, std::string& out)
{
    std::cout << label << std::flush;
    std::string raw;
    if (!std::getline(std::cin, raw)) { std::cout << "\n"; return false; }
    out = Trim(raw);
    return true;
}

// The line's numeric value, or -1 if it isn't a plain non-negative number.
int AsNumber(const std::string& s)
{
    if (s.empty()) return -1;
    for (const char c : s) {
        if (c < '0' || c > '9') return -1;
    }
    try { return std::stoi(s); }
    catch (const std::exception&) { return -1; }
}

void Note(const std::string& msg) { std::cout << "   " << msg << "\n"; }

// ---- Device picker ---------------------------------------------------------
// Returns the chosen device id, "all" when allowAll and the operator picks it,
// or nullopt for back/EOF.
std::optional<std::string> PickDevice(RigConsole& c, const char* title, bool allowAll)
{
    const std::vector<DeviceRef>& devs = c.Store().Devices();
    if (devs.empty()) { Note("no devices in the config"); return std::nullopt; }

    for (;;) {
        std::cout << "\n  " << title << "\n" << kThin << "\n";
        int n = 0;
        if (allowAll) {
            ++n;
            std::cout << "    " << n << ") every enabled device\n";
        }
        for (const DeviceRef& d : devs) {
            ++n;
            std::cout << "   " << (n < 10 ? " " : "") << n << ")"
                      << c.Store().Summary(d) << "\n";
        }
        std::cout << "    0) back\n";

        std::string in;
        if (!ReadLine("  device> ", in)) return std::nullopt;
        if (in.empty()) continue;

        const int pick = AsNumber(in);
        if (pick == 0) return std::nullopt;
        if (pick > 0) {
            if (allowAll && pick == 1) return std::string("all");
            const std::size_t idx = static_cast<std::size_t>(pick - 1 - (allowAll ? 1 : 0));
            if (idx < devs.size()) return devs[idx].id;
        }
        // Typing a name works too — it is what the command form takes anyway.
        if (c.Store().Find(in)) return in;
        Note("pick a number from the list, or type a device name");
    }
}

// ---- Edit one device -------------------------------------------------------
void EditDevice(RigConsole& c, const std::string& deviceId)
{
    for (;;) {
        const std::optional<DeviceRef> d = c.Store().Find(deviceId);
        if (!d) { Note("device '" + deviceId + "' is gone (was the config reloaded?)"); return; }

        const std::vector<std::pair<std::string, std::string>> fields = c.Store().Fields(*d);
        std::cout << "\n  Edit " << d->id << "  [" << KindName(d->kind) << "]"
                  << (c.Store().Dirty(*d) ? "   *unsaved*" : "") << "\n" << kThin << "\n";
        for (std::size_t i = 0; i < fields.size(); ++i) {
            const std::size_t n = i + 1;
            std::cout << "   " << (n < 10 ? " " : "") << n << ") "
                      << std::string(fields[i].first.size() < 22u ? 22u - fields[i].first.size() : 1u, ' ')
                      << fields[i].first << " : " << fields[i].second << "\n";
        }
        std::cout << "    s) save this device to the config file\n"
                  << "    0) back\n";

        std::string in;
        if (!ReadLine("  field> ", in)) return;
        if (in.empty()) continue;
        if (in == "0") return;

        bool stop = false;
        if (in == "s" || in == "S") { c.Execute("save " + d->id, stop); continue; }

        const int pick = AsNumber(in);
        if (pick < 1 || static_cast<std::size_t>(pick) > fields.size()) {
            Note("pick a field number, `s` to save, or 0 to go back");
            continue;
        }

        const std::string& field = fields[static_cast<std::size_t>(pick) - 1].first;
        const std::string& shown = fields[static_cast<std::size_t>(pick) - 1].second;
        std::cout << "\n   " << field << "  (currently: "
                  << (shown.empty() ? "<empty>" : shown) << ")\n"
                  << "   blank cancels.\n";

        std::string value;
        if (!ReadLine("  new value> ", value)) return;
        if (value.empty()) { Note("cancelled"); continue; }

        c.Execute("set " + d->id + " " + field + " " + value, stop);
    }
}

// ---- Log level -------------------------------------------------------------
void PickLogLevel(RigConsole& c)
{
    static const char* kLevels[] = {"trace", "debug", "info", "warn", "error"};
    std::cout << "\n  Log level\n" << kThin << "\n";
    for (int i = 0; i < 5; ++i) std::cout << "    " << (i + 1) << ") " << kLevels[i] << "\n";
    std::cout << "    0) back\n";

    std::string in;
    if (!ReadLine("  level> ", in)) return;
    const int pick = AsNumber(in);
    if (pick < 1 || pick > 5) return;

    bool stop = false;
    c.Execute(std::string("loglevel ") + kLevels[pick - 1], stop);
}

// ---- Main menu -------------------------------------------------------------
void PrintMainMenu(const RigConsole& c)
{
    const std::size_t devCount = c.Store().Devices().size();
    const std::size_t dirty    = c.Store().DirtyDevices().size();

    std::string state = "not connected";
    if (c.Connected()) {
        state = c.SessionTarget() == "all" ? "connected: every enabled device"
                                           : "connected: " + c.SessionTarget() + " only";
    }

    std::cout << "\n" << kRule << "\n"
              << "   FireDogRig - hardware bring-up            " << state << "\n"
              << "   " << c.Store().Path() << "   (" << devCount << " devices"
              << (dirty > 0 ? ", " + std::to_string(dirty) + " unsaved" : "") << ")\n"
              << kRule << "\n"
              << "    1) Connect all              5) List devices\n"
              << "    2) Connect one device...    6) Show / edit a device...\n"
              << "    3) Probe (read-back)...     7) Save changes to config\n"
              << "    4) Disconnect               8) Reload config (drop edits)\n"
              << kThin << "\n"
              << "    9) Connection status       10) Log level...\n"
              << "    c) Type a command           h) Command help\n"
              << "    q) Quit\n";
}

} // namespace

int RunMenu(RigConsole& console)
{
    std::cout << "\n  Pick a number and press Enter. Enter on its own reprints the menu.\n";

    for (;;) {
        PrintMainMenu(console);

        std::string in;
        if (!ReadLine("  select> ", in)) break;   // EOF
        if (in.empty()) continue;                 // just reprint

        bool stop = false;
        if (in == "q" || in == "Q") break;
        if (in == "h" || in == "H") { console.PrintHelp(); continue; }
        if (in == "c" || in == "C") {
            std::string cmd;
            if (!ReadLine("  command> ", cmd)) break;
            if (!cmd.empty()) console.Execute(cmd, stop);
            if (stop) break;                      // they typed `quit`
            continue;
        }

        switch (AsNumber(in)) {
            case 1: console.Execute("connect all", stop); break;
            case 2: {
                const std::optional<std::string> d = PickDevice(console, "Connect which device?", false);
                if (d) console.Execute("connect " + *d, stop);
                break;
            }
            case 3: {
                const std::optional<std::string> d = PickDevice(console, "Probe which device?", true);
                if (d) console.Execute("probe " + *d, stop);
                break;
            }
            case 4:  console.Execute("disconnect", stop); break;
            case 5:  console.PrintList(); break;
            case 6: {
                const std::optional<std::string> d = PickDevice(console, "Which device?", false);
                if (d) EditDevice(console, *d);
                break;
            }
            case 7:  console.Execute("save all", stop); break;
            case 8:  console.Execute("reload", stop); break;
            case 9:  console.Execute("status", stop); break;
            case 10: PickLogLevel(console); break;
            default: Note("unknown choice - pick a number from the menu"); break;
        }
    }

    // Close the device sessions here, while the logger is still alive.
    console.CloseSession();
    std::cout << "\n  bye\n";
    return 0;
}

} // namespace firedog::rig
