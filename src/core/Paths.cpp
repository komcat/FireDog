// Paths.cpp
//
// No FD_* logging anywhere in this file: the logger asks MachinePath() for its
// own file, so a log call from inside InitPaths would re-enter the call_once.
// Everything worth saying goes into PathNotes() instead.
#include "core/Paths.h"

#include <nlohmann/json.hpp>

#include <cctype>
#include <chrono>
#include <cstdlib>
#include <ctime>
#include <exception>
#include <filesystem>
#include <fstream>
#include <map>
#include <mutex>

// Last, and narrowed: windows.h's min/max macros would otherwise break the
// standard headers above.
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>

namespace fs = std::filesystem;

namespace firedog {

namespace {

constexpr const char* kDefaultRoot = "D:/ProgramData/FireDog";
constexpr const char* kIniName     = "FireDog.ini";

// The machine files the app edits IN PLACE - they have no xxx.local.json pair,
// so the machine root needs its own copy before the first page saves into it.
// Seeded from the exe-side copy when missing: on an existing machine that copy
// IS the machine's wiring (cmake/StageConfig.cmake never overwrote it), on a
// fresh install it is the repo's default.
constexpr const char* kSeeded[] = {"config/hardware.json", "config/motion_transforms.json"};

constexpr const char* kIniTemplate =
    "; FireDog.ini - where this machine keeps its state. Read once, at startup.\n"
    "; FireDog wrote this template because none existed; edit and restart.\n"
    "; It is never shipped in a release, so unzipping a new version keeps it.\n"
    "; In a Setup install it sits above current\\, where an update cannot reach.\n"
    ";\n"
    ";   root  machine state: hardware.json, calibrations, *.local.json,\n"
    ";         products\\ (documents + the employee roster), logs\\, imgui.ini.\n"
    ";         Default D:\\ProgramData\\FireDog, or %ProgramData%\\FireDog\n"
    ";         (C:) when this PC has no fixed D: disk\n"
    ";   data  production output: DUT records, xcan\\, align_attempts\\, mosaic\\.\n"
    ";         Default <root>\\data\n"
    ";\n"
    "; A relative path is taken from this file's folder; %VARIABLES% expand.\n"
    "; The first start against an empty root COPIES the old exe-side state\n"
    "; into it (see migrated_from.txt there).\n"
    "\n"
    "[paths]\n"
    ";root = D:\\ProgramData\\FireDog\n"
    ";data = D:\\ProgramData\\FireDog\\data\n";

struct State {
    std::string           exeDir, root, data, ini;
    std::vector<PathNote> notes;
};

State& S()
{
    static State s;
    return s;
}

std::once_flag g_once;

void Note(bool error, std::string text) { S().notes.push_back({error, std::move(text)}); }

// Forward slashes, no trailing separator: every caller appends "/<rel>".
std::string Generic(const fs::path& p)
{
    std::string s = p.lexically_normal().generic_string();
    while (s.size() > 3 && s.back() == '/') s.pop_back();   // keep "D:/"
    return s;
}

fs::path ExeFolder()
{
    std::wstring buf(MAX_PATH, L'\0');
    for (;;) {
        const DWORD n = GetModuleFileNameW(nullptr, buf.data(), static_cast<DWORD>(buf.size()));
        if (n == 0) {
            std::error_code ec;
            return fs::current_path(ec);   // cannot ask Windows; trust the cwd
        }
        if (n < buf.size()) { buf.resize(n); break; }
        buf.resize(buf.size() * 2);        // a path longer than MAX_PATH
    }
    return fs::path(buf).parent_path();
}

std::string Trim(const std::string& s)
{
    std::size_t b = 0, e = s.size();
    while (b < e && std::isspace(static_cast<unsigned char>(s[b]))) ++b;
    while (e > b && std::isspace(static_cast<unsigned char>(s[e - 1]))) --e;
    return s.substr(b, e - b);
}

std::string ExpandEnv(const std::string& s)
{
    if (s.find('%') == std::string::npos) return s;
    const DWORD n = ExpandEnvironmentStringsA(s.c_str(), nullptr, 0);
    if (n == 0) return s;
    std::string out(n, '\0');
    if (ExpandEnvironmentStringsA(s.c_str(), out.data(), n) == 0) return s;
    out.resize(n - 1);   // n counts the terminator
    return out;
}

// [paths] key = value. ';' or '#' starts a comment line, keys are
// case-insensitive, a value may be quoted. Other sections are ignored so the
// file can grow without this reader learning about it.
std::map<std::string, std::string> ReadIni(const fs::path& p)
{
    std::map<std::string, std::string> out;
    std::ifstream f(p);
    if (!f.is_open()) return out;
    std::string line, section;
    while (std::getline(f, line)) {
        line = Trim(line);
        if (line.empty() || line[0] == ';' || line[0] == '#') continue;
        if (line.front() == '[' && line.back() == ']') {
            section = Trim(line.substr(1, line.size() - 2));
            for (char& c : section) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
            continue;
        }
        if (section != "paths") continue;
        const std::size_t eq = line.find('=');
        if (eq == std::string::npos) continue;
        std::string key = Trim(line.substr(0, eq));
        std::string val = Trim(line.substr(eq + 1));
        for (char& c : key) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
        if (val.size() >= 2 && val.front() == '"' && val.back() == '"')
            val = val.substr(1, val.size() - 2);
        if (!key.empty() && !val.empty()) out[key] = ExpandEnv(val);
    }
    return out;
}

// A configured path; a relative one is taken from the ini's own folder.
fs::path FromIni(const std::string& v, const fs::path& iniDir)
{
    fs::path p(v);
    return p.is_absolute() ? p : iniDir / p;
}

// Where FireDog.ini lives. A Velopack install (Setup.exe / Portable.zip) runs
// from <install>/current/, and every update REPLACES current/ whole - so there
// the ini sits one level up, beside Update.exe, where an update cannot reach
// it. Anywhere else (a build tree, an unzipped release) it is beside the exe.
fs::path IniFolder(const fs::path& exe)
{
    std::error_code ec;
    const fs::path up = exe.parent_path();
    if (exe.filename() == "current" && fs::exists(up / "Update.exe", ec)) return up;
    return exe;
}

bool MakeDir(const fs::path& p, std::string* why)
{
    std::error_code ec;
    fs::create_directories(p, ec);
    std::error_code ec2;
    if (fs::is_directory(p, ec2)) return true;
    if (why) *why = ec ? ec.message() : std::string("not a directory");
    return false;
}

std::string ProgramDataFallback()
{
    char*       env = nullptr;
    std::size_t len = 0;
    std::string base = "C:/ProgramData";
    if (_dupenv_s(&env, &len, "ProgramData") == 0 && env) {
        if (*env) base = env;
        std::free(env);
    }
    return Generic(fs::path(base) / "FireDog");
}

// The root when the ini names none: D: on a machine that has a fixed D: disk,
// else %ProgramData% (C:). Removable media is not a D: drive here - a USB
// stick that is there on some boots would flip the root between starts, and
// each flip would look like a first start against an empty folder.
fs::path DefaultRoot()
{
    if (GetDriveTypeA("D:\\") == DRIVE_FIXED) return fs::path(kDefaultRoot);
    Note(false, "no fixed D: drive - default machine root is " + ProgramDataFallback());
    return fs::path(ProgramDataFallback());
}

bool SameDir(const fs::path& a, const fs::path& b)
{
    std::error_code ec;
    return fs::equivalent(a, b, ec);
}

// ---- First-start migration -------------------------------------------------

class Migration {
public:
    Migration(fs::path exe, fs::path root, fs::path data)
        : exe_(std::move(exe)), root_(std::move(root)), data_(std::move(data)) {}

    void Run()
    {
        Config();
        Products();
        Logs();
        Data();
        Copy(exe_ / "imgui.ini", root_ / "imgui.ini");
        WriteMarker();
    }

private:
    // A file or a whole tree; never overwrites, never deletes the source.
    void Copy(const fs::path& from, const fs::path& to)
    {
        std::error_code ec;
        if (!fs::exists(from, ec)) return;
        fs::create_directories(to.parent_path(), ec);
        ec.clear();
        fs::copy(from, to, fs::copy_options::recursive | fs::copy_options::skip_existing, ec);
        const std::string what = Generic(from) + " -> " + Generic(to);
        if (ec) {
            Note(true, "migration: could not copy " + what + ": " + ec.message());
            return;
        }
        Note(false, "migration: copied " + what);
        copied_.push_back(what);
    }

    static bool EndsWith(const std::string& s, const char* tail)
    {
        const std::size_t n = std::char_traits<char>::length(tail);
        return s.size() >= n && s.compare(s.size() - n, n, tail) == 0;
    }

    // What the app wrote: the *.local.json files, their journals, the
    // product-less vision patches. The shipped defaults stay where they are;
    // the in-place files (hardware.json, transforms) come across by seeding.
    void Config()
    {
        std::error_code ec;
        const fs::path dir = exe_ / "config";
        for (const auto& e : fs::directory_iterator(dir, ec)) {
            const std::string name = e.path().filename().string();
            if (e.is_regular_file(ec) &&
                (EndsWith(name, ".local.json") || EndsWith(name, ".history.jsonl")))
                Copy(e.path(), root_ / "config" / name);
        }
        Copy(dir / "vision", root_ / "config" / "vision");
    }

    // A product's documents go to the machine root, its DUT records (the
    // manifest's data/images folders) to the data root.
    void Products()
    {
        std::error_code ec;
        const fs::path dir = exe_ / "products";
        for (const auto& e : fs::directory_iterator(dir, ec)) {
            const std::string name = e.path().filename().string();
            if (!e.is_directory(ec) || !fs::exists(e.path() / "manifest.json", ec)) {
                Copy(e.path(), root_ / "products" / name);   // employees.json, zips, ...
                continue;
            }
            std::string dataDir = "data", imagesDir = "images";
            try {
                std::ifstream f(e.path() / "manifest.json");
                nlohmann::json j;
                f >> j;
                if (j.contains("folders") && j["folders"].is_object()) {
                    dataDir   = j["folders"].value("data", dataDir);
                    imagesDir = j["folders"].value("images", imagesDir);
                }
            } catch (const std::exception&) {
                // Unreadable manifest: the defaults are what the app would use.
            }
            for (const auto& c : fs::directory_iterator(e.path(), ec)) {
                const std::string child = c.path().filename().string();
                const bool isData = c.is_directory(ec) && (child == dataDir || child == imagesDir);
                Copy(c.path(), (isData ? data_ : root_) / "products" / name / child);
            }
        }
    }

    void Logs()
    {
        std::error_code ec;
        for (const auto& e : fs::directory_iterator(exe_ / "logs", ec)) {
            const std::string name = e.path().filename().string();
            Copy(e.path(), name == "align_attempts" ? data_ / name : root_ / "logs" / name);
        }
    }

    void Data()   // data/xcan and anything else under exe/data
    {
        std::error_code ec;
        for (const auto& e : fs::directory_iterator(exe_ / "data", ec))
            Copy(e.path(), data_ / e.path().filename());
    }

    void WriteMarker()
    {
        const std::time_t t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
        std::tm tm{};
        localtime_s(&tm, &t);
        char stamp[32];
        std::strftime(stamp, sizeof(stamp), "%Y-%m-%d %H:%M:%S", &tm);

        std::ofstream f(root_ / "migrated_from.txt");
        if (!f.is_open()) return;
        f << "FireDog first start against this folder: " << stamp << "\n"
          << "Old state was COPIED (not moved) from " << Generic(exe_) << "\n"
          << "The originals are untouched; delete them once this machine runs well.\n\n";
        if (copied_.empty()) f << "(nothing to copy)\n";
        for (const std::string& c : copied_) f << c << "\n";
    }

    fs::path                 exe_, root_, data_;
    std::vector<std::string> copied_;
};

void Seed(const fs::path& exe, const fs::path& root)
{
    for (const char* rel : kSeeded) {
        std::error_code ec;
        const fs::path to = root / rel;
        if (fs::exists(to, ec)) continue;
        const fs::path from = exe / rel;
        if (!fs::exists(from, ec)) continue;
        fs::create_directories(to.parent_path(), ec);
        ec.clear();
        fs::copy_file(from, to, fs::copy_options::skip_existing, ec);
        if (ec)
            Note(true, "could not seed " + Generic(to) + " from " + Generic(from) + ": " +
                           ec.message());
        else
            Note(false, "seeded " + Generic(to) + " from " + Generic(from));
    }
}

void DoInit()
{
    State& s = S();
    const fs::path exe    = ExeFolder();
    const fs::path iniDir = IniFolder(exe);
    s.exeDir = Generic(exe);
    s.ini    = Generic(iniDir / kIniName);

    // Shipped paths stay relative ("config/scripts/...", "assets/..."), so the
    // working directory must be the exe folder whatever the shortcut said.
    {
        std::error_code ec;
        fs::current_path(exe, ec);
        if (ec) Note(true, "could not switch to the exe folder " + s.exeDir + ": " + ec.message());
    }

    std::map<std::string, std::string> ini;
    {
        std::error_code ec;
        if (fs::exists(iniDir / kIniName, ec)) {
            ini = ReadIni(iniDir / kIniName);
        } else {
            std::ofstream f(iniDir / kIniName);   // best effort: a read-only folder is fine
            if (f.is_open()) {
                f << kIniTemplate;
                Note(false, "wrote a template " + s.ini + " (all defaults)");
            }
        }
    }

    // ---- The machine root ----
    fs::path root = ini.count("root") ? FromIni(ini["root"], iniDir) : DefaultRoot();
    std::error_code ec;
    bool firstStart = !fs::exists(root / "config", ec);
    std::string why;
    if (!MakeDir(root / "config", &why)) {
        const std::string fallback = ProgramDataFallback();
        Note(true, "machine root " + Generic(root) + " is unusable (" + why + ") - using " +
                       fallback + "; set [paths] root in " + s.ini);
        root       = fallback;
        firstStart = !fs::exists(root / "config", ec);
        if (!MakeDir(root / "config", &why)) {
            Note(true, "fallback root " + fallback + " is unusable too (" + why +
                           ") - keeping state beside the exe");
            root       = exe;
            firstStart = false;
        }
    }
    s.root = Generic(root);

    // ---- The data root ----
    fs::path data = ini.count("data") ? FromIni(ini["data"], iniDir) : root / "data";
    if (!MakeDir(data, &why)) {
        Note(true, "data root " + Generic(data) + " is unusable (" + why + ") - using " +
                       Generic(root / "data"));
        data = root / "data";
        MakeDir(data, nullptr);
    }
    s.data = Generic(data);

    Note(false, "machine root " + s.root + ", data root " + s.data +
                    (ini.empty() ? " (defaults)" : " (from " + s.ini + ")"));

    if (firstStart && !SameDir(root, exe)) Migration(exe, root, data).Run();
    Seed(exe, root);
}

} // namespace

void InitPaths() { std::call_once(g_once, DoInit); }

const std::string& ExeDir()      { InitPaths(); return S().exeDir; }
const std::string& MachineRoot() { InitPaths(); return S().root; }
const std::string& DataRoot()    { InitPaths(); return S().data; }
const std::string& IniPath()     { InitPaths(); return S().ini; }

std::string MachinePath(const std::string& rel) { return MachineRoot() + "/" + rel; }
std::string DataPath(const std::string& rel)    { return DataRoot() + "/" + rel; }

const std::vector<PathNote>& PathNotes() { InitPaths(); return S().notes; }

} // namespace firedog
