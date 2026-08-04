// RigConsole.cpp
#include "RigConsole.h"

#include "RigProbe.h"
#include "RigReport.h"

#include "core/Log.h"

#include <algorithm>
#include <cctype>
#include <exception>
#include <filesystem>
#include <iostream>
#include <sstream>

namespace firedog::rig {
namespace {

std::vector<std::string> Tokenize(const std::string& line)
{
    std::vector<std::string> tok;
    std::istringstream is(line);
    std::string t;
    while (is >> t) tok.push_back(t);
    return tok;
}

// tok[from..] rejoined — `set` values may contain spaces ("X Y Z U V W").
std::string Rest(const std::vector<std::string>& tok, std::size_t from)
{
    std::string s;
    for (std::size_t i = from; i < tok.size(); ++i) {
        if (!s.empty()) s += " ";
        s += tok[i];
    }
    return s;
}

void Fail(const std::string& msg) { std::cout << "   error: " << msg << "\n"; }

} // namespace

bool RigConsole::Init(const std::string& configPath, double connectTimeoutSec)
{
    timeoutSec_ = connectTimeoutSec;
    const bool any = store_.Load(configPath);
    std::cout << "config: " << configPath << "  ("
              << store_.Devices().size() << " devices)\n";
    if (!any) {
        Fail("no devices in the config - wrong working directory, or the file is missing");
    }
    return any;
}

void RigConsole::PrintHelp() const
{
    std::cout <<
        "\n"
        "  FireDogRig - console hardware bring-up for FireDog.\n"
        "  Read-only: it connects and reports. No move, no IO write, no output switch.\n"
        "\n"
        "  list                        devices in the config (* = unsaved edit)\n"
        "  show <device>               editable fields and their current values\n"
        "  set <device> <field> <val>  edit in memory (takes effect on the next connect)\n"
        "  save [<device>|all]         write edited entries back to the config file\n"
        "  reload                      re-read the config file, dropping unsaved edits\n"
        "  report [<file>]             write the WHOLE configuration as readable text,\n"
        "                              for handover (default: firedog-config-report.txt)\n"
        "\n"
        "  connect [<device>|all]      bring up hardware the way the client does\n"
        "  status                      connection state of everything brought up\n"
        "  probe [<device>|all]        connect (if needed) and print live read-back\n"
        "  disconnect                  tear the session down (closes the device sessions)\n"
        "\n"
        "  loglevel <trace|debug|info|warn|error>\n"
        "  help                        this text\n"
        "  quit                        exit\n"
        "\n"
        "  Notes\n"
        "   - `connect all` runs the client's exact bring-up sequence, io_watch and\n"
        "     safety_watch included. `connect <device>` enables only that device and\n"
        "     turns both watches plus the pneumatic slides off, because their bindings\n"
        "     reference pins on devices that are not in a one-device fleet.\n"
        "   - `probe <device>` re-connects to that device first; plain `probe` reports\n"
        "     on the session already up.\n"
        "   - Slide state machines only advance while a connect or probe is running.\n"
        "   - `report` needs no hardware: it describes the config file, including the\n"
        "     parts no rig command edits (pin names, slides, io_watch, safety_watch).\n"
        "\n";
}

void RigConsole::PrintList() const
{
    std::cout << "\n";
    for (const DeviceRef& d : store_.Devices()) {
        std::cout << store_.Summary(d) << "\n";
    }
    if (store_.AnyDirty()) {
        std::cout << "\n   * unsaved - `save all` writes them to " << store_.Path() << "\n";
    }
    std::cout << "\n";
}

bool RigConsole::EnsureSession(const std::string& target, bool reportStatus)
{
    const bool wantNew = !target.empty() && target != sessionTarget_;
    if (session_ && !wantNew) return session_->AllExpectedConnected();

    const std::string want = target.empty() ? std::string("all") : target;

    if (want == "all") {
        session_.reset();   // close the old sessions before opening new ones
        std::cout << "   connecting every enabled device...\n";
        session_ = std::make_unique<RigSession>(store_.Config(), /*connectSimFleet=*/true);
    } else {
        const std::optional<DeviceRef> d = store_.Find(want);
        if (!d) {
            Fail("no device '" + want + "' in the config (try: list)");
            return false;
        }
        session_.reset();
        std::cout << "   connecting " << d->id << " only...\n";
        session_ = std::make_unique<RigSession>(
            FilterToDevice(store_.Config(), *d), /*connectSimFleet=*/false);
    }
    sessionTarget_ = want;

    const bool settled = session_->WaitForConnects(timeoutSec_);
    if (!settled) {
        Fail("timed out after " + std::to_string(static_cast<int>(timeoutSec_))
             + " s with a device still connecting");
    }
    if (reportStatus) PrintStatus(*session_);
    return settled && session_->AllExpectedConnected();
}

void RigConsole::CloseSession()
{
    session_.reset();
    sessionTarget_.clear();
}

bool RigConsole::CmdShow(const std::vector<std::string>& tok)
{
    if (tok.size() < 2) { Fail("usage: show <device>"); return false; }
    const std::optional<DeviceRef> d = store_.Find(tok[1]);
    if (!d) { Fail("no device '" + tok[1] + "' (try: list)"); return false; }

    std::cout << "\n   " << d->id << "  [" << KindName(d->kind) << "]"
              << (store_.Dirty(*d) ? "  (unsaved edits)" : "") << "\n";
    for (const auto& [name, value] : store_.Fields(*d)) {
        const std::size_t pad = name.size() < 22u ? 22u - name.size() : 1u;
        std::cout << "     " << std::string(pad, ' ') << name << " : " << value << "\n";
    }
    std::cout << "\n";
    return true;
}

bool RigConsole::CmdSet(const std::vector<std::string>& tok)
{
    if (tok.size() < 4) { Fail("usage: set <device> <field> <value>"); return false; }
    const std::optional<DeviceRef> d = store_.Find(tok[1]);
    if (!d) { Fail("no device '" + tok[1] + "' (try: list)"); return false; }

    const std::string value = Rest(tok, 3);
    std::string err;
    if (!store_.Set(*d, tok[2], value, err)) { Fail(err); return false; }

    std::cout << "   " << d->id << "." << tok[2] << " = " << value
              << "   (in memory; `save " << d->id << "` to keep it";
    std::cout << (session_ ? ", reconnect to apply it)" : ")") << "\n";
    return true;
}

bool RigConsole::CmdSave(const std::vector<std::string>& tok)
{
    std::vector<DeviceRef> targets;
    if (tok.size() < 2 || tok[1] == "all") {
        targets = store_.DirtyDevices();
        if (targets.empty()) { std::cout << "   nothing to save\n"; return true; }
    } else {
        const std::optional<DeviceRef> d = store_.Find(tok[1]);
        if (!d) { Fail("no device '" + tok[1] + "' (try: list)"); return false; }
        targets.push_back(*d);
    }

    bool allOk = true;
    for (const DeviceRef& d : targets) {
        std::string err;
        if (store_.Save(d, err)) {
            std::cout << "   saved " << d.id << "\n";
        } else {
            Fail(err);
            allOk = false;
        }
    }
    std::cout << "   -> " << store_.Path()
              << "\n      a rebuild recopies the repo's config/ over this file - mirror"
                 " durable edits into the repo copy.\n";
    return allOk;
}

bool RigConsole::CmdReload()
{
    if (store_.AnyDirty()) {
        std::cout << "   discarding unsaved edits\n";
    }
    const std::string path = store_.Path();
    const bool ok = store_.Load(path);
    std::cout << "   reloaded " << path << " (" << store_.Devices().size() << " devices)\n";
    if (!ok) Fail("no devices found in the file");
    return ok;
}

bool RigConsole::CmdReport(const std::vector<std::string>& tok)
{
    // Rest() rather than tok[1]: a handover path is very often "D:\My Docs\...".
    const std::string path = tok.size() >= 2 ? Rest(tok, 1) : std::string(kDefaultReportFile);

    std::string err;
    if (!WriteConfigReport(store_, path, err)) { Fail(err); return false; }

    std::string shown = path;
    try { shown = std::filesystem::absolute(path).string(); }
    catch (const std::exception&) { /* show the path as typed */ }

    std::cout << "   configuration report written:\n      " << shown << "\n"
              << "   plain text, every section of " << store_.Path()
              << " - hand it over as is.\n";
    if (store_.AnyDirty()) {
        std::cout << "   note: it reports the unsaved rig edits too, marked *."
                     " `save all` first if\n         the other team should see the file's"
                     " own values.\n";
    }
    return true;
}

bool RigConsole::CmdConnect(const std::vector<std::string>& tok)
{
    const std::string target = tok.size() >= 2 ? tok[1] : std::string("all");
    session_.reset();          // an explicit connect always starts clean
    sessionTarget_.clear();
    return EnsureSession(target, /*reportStatus=*/true);
}

bool RigConsole::CmdProbe(const std::vector<std::string>& tok)
{
    const std::string target = tok.size() >= 2 ? tok[1] : std::string();
    // A named target always gets a fresh session for exactly that device; a bare
    // `probe` reports on whatever is already up (connecting all if nothing is).
    const bool connected = EnsureSession(target, /*reportStatus=*/false);
    if (!session_) return false;

    PrintProbe(*session_);
    if (!connected) Fail("not every configured device connected - see the states above");
    return connected;
}

bool RigConsole::CmdLogLevel(const std::vector<std::string>& tok)
{
    if (tok.size() < 2) { Fail("usage: loglevel <trace|debug|info|warn|error>"); return false; }
    std::string v = tok[1];
    std::transform(v.begin(), v.end(), v.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

    if      (v == "trace") Logger::Instance().SetMinLevel(LogLevel::Trace);
    else if (v == "debug") Logger::Instance().SetMinLevel(LogLevel::Debug);
    else if (v == "info")  Logger::Instance().SetMinLevel(LogLevel::Info);
    else if (v == "warn")  Logger::Instance().SetMinLevel(LogLevel::Warn);
    else if (v == "error") Logger::Instance().SetMinLevel(LogLevel::Error);
    else { Fail("unknown level '" + tok[1] + "'"); return false; }

    std::cout << "   log level = " << v << "\n";
    return true;
}

bool RigConsole::Execute(const std::string& line, bool& stop)
{
    stop = false;
    const std::vector<std::string> tok = Tokenize(line);
    if (tok.empty()) return true;

    const std::string& cmd = tok[0];
    if (cmd == "quit" || cmd == "exit") { stop = true; return true; }
    if (cmd == "help" || cmd == "?")    { PrintHelp(); return true; }
    if (cmd == "list")                  { PrintList(); return true; }
    if (cmd == "show")                  return CmdShow(tok);
    if (cmd == "set")                   return CmdSet(tok);
    if (cmd == "save")                  return CmdSave(tok);
    if (cmd == "reload")                return CmdReload();
    if (cmd == "report")                return CmdReport(tok);
    if (cmd == "connect")               return CmdConnect(tok);
    if (cmd == "probe")                 return CmdProbe(tok);
    if (cmd == "loglevel")              return CmdLogLevel(tok);
    if (cmd == "status") {
        if (!session_) { Fail("nothing connected - run `connect` first"); return false; }
        PrintStatus(*session_);
        return session_->AllExpectedConnected();
    }
    if (cmd == "disconnect") {
        if (!session_) { std::cout << "   nothing connected\n"; return true; }
        CloseSession();
        std::cout << "   disconnected\n";
        return true;
    }

    Fail("unknown command '" + cmd + "' (try: help)");
    return false;
}

int RigConsole::RunRepl()
{
    std::cout << "\nType `help` for commands, `quit` to exit.\n";
    std::string line;
    for (;;) {
        std::cout << "rig> " << std::flush;
        if (!std::getline(std::cin, line)) break;   // EOF (piped input, Ctrl+Z)
        bool stop = false;
        Execute(line, stop);
        if (stop) break;
    }
    // Tear the session down here rather than at static destruction time, so the
    // device sessions close while the logger is still alive.
    session_.reset();
    std::cout << "bye\n";
    return 0;
}

} // namespace firedog::rig
