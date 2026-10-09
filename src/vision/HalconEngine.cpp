// HalconEngine.cpp : the three-gate HALCON probe. See HalconEngine.h.
//
// This is the ONLY translation unit that includes a HALCON header. Everything
// else asks HalconEngine::Instance().Available() and gets a bool plus a reason.
#include "vision/HalconEngine.h"

#include "core/Log.h"

#include <cstdlib>
#include <filesystem>
#include <fstream>

#ifdef HAS_HALCON
  #ifndef WIN32_LEAN_AND_MEAN
    #define WIN32_LEAN_AND_MEAN
  #endif
  #ifndef NOMINMAX
    #define NOMINMAX
  #endif
  #include <windows.h>
  #include <winver.h>   // WIN32_LEAN_AND_MEAN drops it; the probe reads DLL versions

  #include <HVersNum.h>
  #include <halconcpp/HalconCpp.h>

  #include <cstdio>
#endif

// Baked by CMake from the SDK it actually linked against. Two jobs: the DLL
// probe still works where HALCONROOT is absent from the environment (a service
// account), and it names the install whose DLLs are certain to match these
// import libs — preferred over the environment for exactly that reason.
#ifndef FIREDOG_HALCON_ROOT
  #define FIREDOG_HALCON_ROOT ""
#endif
// The arch of THIS build (x64-win64 / x86sse2-win32), not the machine's default
// from HALCONARCH: a 32-bit FireDog must load 32-bit HALCON even on a box whose
// environment advertises the 64-bit install.
#ifndef FIREDOG_HALCON_ARCH
  #define FIREDOG_HALCON_ARCH ""
#endif

namespace firedog {

namespace {

std::string EnvOrEmpty(const char* name)
{
    char   buf[1024] = {};
    size_t len       = 0;
    if (getenv_s(&len, buf, sizeof(buf), name) != 0 || len == 0) return {};
    return std::string(buf);
}

std::string Trim(const std::string& s)
{
    const size_t a = s.find_first_not_of(" \t\r\n#");
    if (a == std::string::npos) return {};
    const size_t b = s.find_last_not_of(" \t\r\n#");
    return s.substr(a, b - a + 1);
}

// The LICENSE lines of one .dat. The format: '#' comment header (holder line
// first, then the product), then `LICENSE <feature> <up-to version> KEY=VALUE
// ...` with '\' continuations and quoted values. The SIGNATURE is skipped —
// it is the key itself, not something to put on a screen.
void ParseLicenceFile(const std::filesystem::path& p, std::vector<HalconLicence>& out)
{
    std::ifstream in(p);
    if (!in) return;
    std::string holder, line, joined;
    std::vector<std::string> statements;
    while (std::getline(in, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (joined.empty() && !line.empty() && line[0] == '#') {
            // First non-empty comment line names the holder; drop its "(Id: ...)".
            if (holder.empty()) {
                std::string t = Trim(line);
                const size_t id = t.find("(Id:");
                if (id != std::string::npos) t = Trim(t.substr(0, id));
                holder = t;
            }
            continue;
        }
        const bool cont = !line.empty() && line.back() == '\\';
        joined += cont ? line.substr(0, line.size() - 1) + " " : line;
        if (!cont) {
            if (!Trim(joined).empty()) statements.push_back(joined);
            joined.clear();
        }
    }
    if (!Trim(joined).empty()) statements.push_back(joined);

    for (const std::string& st : statements) {
        // Tokenise on whitespace, keeping "quoted values" whole.
        std::vector<std::string> tok;
        std::string cur;
        bool quoted = false;
        for (char c : st) {
            if (c == '"') { quoted = !quoted; continue; }
            if (!quoted && (c == ' ' || c == '\t')) {
                if (!cur.empty()) { tok.push_back(cur); cur.clear(); }
                continue;
            }
            cur += c;
        }
        if (!cur.empty()) tok.push_back(cur);
        if (tok.size() < 3 || tok[0] != "LICENSE") continue;

        HalconLicence l;
        l.file   = p.filename().string();
        l.holder = holder;
        l.upTo   = tok[2];
        for (size_t i = 3; i < tok.size(); ++i) {
            const size_t eq = tok[i].find('=');
            if (eq == std::string::npos) continue;
            const std::string k = tok[i].substr(0, eq), v = tok[i].substr(eq + 1);
            if      (k == "INFO")     l.info     = v;
            else if (k == "ID")       l.id       = v;
            else if (k == "MODULES")  l.modules  = v;
            else if (k == "FLAGS")    l.flags    = v;
            else if (k == "SESSIONS") l.sessions = v;
        }
        out.push_back(std::move(l));
    }
}

// Licence discovery mirrors HALCON's own order: HALCONLICENSEDIR wins, else
// <root>/license. Listing the .dat files is worth it on its own — "no *.dat in
// <dir>" is the single most useful thing to show an operator.
void ScanLicenses(HalconStatus& s)
{
    namespace fs = std::filesystem;
    s.licenseFiles.clear();   // re-scannable: the root can change mid-Refresh
    s.licences.clear();
    s.licenseDir = EnvOrEmpty("HALCONLICENSEDIR");
    if (s.licenseDir.empty() && !s.root.empty()) s.licenseDir = s.root + "\\license";
    if (s.licenseDir.empty()) return;

    std::error_code ec;
    if (!fs::is_directory(s.licenseDir, ec)) return;
    for (const auto& e : fs::directory_iterator(s.licenseDir, ec)) {
        if (!e.is_regular_file(ec)) continue;
        const fs::path p = e.path();
        if (p.extension() != ".dat") continue;
        s.licenseFiles.push_back(p.filename().string());
        ParseLicenceFile(p, s.licences);
    }
}

} // namespace

std::string HalconStatus::Summary() const
{
    if (Available())
        return "HALCON " + (version.empty() ? std::string("(unknown version)") : version) +
               " ready";
    return "HALCON unavailable: " + (error.empty() ? std::string("unknown reason") : error);
}

HalconEngine& HalconEngine::Instance()
{
    static HalconEngine inst;
    return inst;
}

HalconEngine::HalconEngine()  { Refresh(); }
HalconEngine::~HalconEngine() = default;

#ifndef HAS_HALCON

// ---- Built without the SDK: one honest answer, no HALCON symbols at all ----
const HalconStatus& HalconEngine::Refresh()
{
    status_       = HalconStatus{};
    status_.root  = EnvOrEmpty("HALCONROOT");
    status_.arch  = EnvOrEmpty("HALCONARCH");
    status_.error = "built without the HALCON SDK (HAS_HALCON not defined) - "
                    "HALCON-backed vision is disabled; re-run cmake --preset with "
                    "HALCONROOT set to enable it";
    ScanLicenses(status_);   // still useful: shows a licence IS present, only the build is not
    FD_INFO("vision", "%s", status_.Summary().c_str());
    return status_;
}

#else

namespace {

// The release this exe was compiled against, from the SDK's own header. Not a
// constant typed here: the CMake search follows whatever HALCON is installed,
// so a rebuild against 26.05 must move the requirement with it.
constexpr int kBuiltMajor = HLIB_MAJOR_NUM;
constexpr int kBuiltMinor = HLIB_MINOR_NUM;

std::string FormatVersion(int major, int minor)
{
    char buf[16];
    std::snprintf(buf, sizeof(buf), "%d.%02d", major, minor);
    return buf;
}

// The YY.MM a module FILE carries in its version resource, read without loading
// it. False when the file has no readable resource.
bool ModuleFileVersion(const std::string& path, int& major, int& minor)
{
    DWORD       ignored = 0;
    const DWORD size    = GetFileVersionInfoSizeA(path.c_str(), &ignored);
    if (size == 0) return false;
    std::vector<char> buf(size);
    if (!GetFileVersionInfoA(path.c_str(), 0, size, buf.data())) return false;
    VS_FIXEDFILEINFO* fi  = nullptr;
    UINT              len = 0;
    if (!VerQueryValueA(buf.data(), "\\", reinterpret_cast<void**>(&fi), &len) || !fi ||
        len < sizeof(VS_FIXEDFILEINFO))
        return false;
    major = HIWORD(fi->dwFileVersionMS);
    minor = LOWORD(fi->dwFileVersionMS);
    return true;
}

// Load the HALCON runtime explicitly BEFORE any delay-loaded call, trying each
// candidate install in turn and finally the bare name (PATH). Returns false with
// a reason when the modules are simply not on this machine — the case /DELAYLOAD
// exists to survive. On success `usedRoot` names the install that answered, so
// the UI reports where the running code actually came from.
//
// Both modules must come from the SAME install AND the same YY.MM release as
// the SDK we linked: HALCON does not promise binary compatibility between
// releases, and a halconcpp.dll missing an export these import libs name is a
// delay-load SEH exception on first call, not a message. So each file's version
// resource is read before it is loaded, and an install of another release is
// skipped — a correct install further down the list still wins. `foundVersion`
// reports the release that was refused, for the panel.
bool LoadHalconModules(const std::vector<std::string>& roots, const std::string& arch,
                       std::string& usedRoot, std::string& foundVersion, std::string& err)
{
    static const char* kModules[] = {"halcon.dll", "halconcpp.dll"};
    const std::string  built      = FormatVersion(kBuiltMajor, kBuiltMinor);

    // "" when `path` is this build's release, else why it is not.
    auto versionProblem = [&](const std::string& path) -> std::string {
        int major = 0, minor = 0;
        if (!ModuleFileVersion(path, major, minor))
            return "'" + path + "' has no readable version";
        if (major == kBuiltMajor && minor == kBuiltMinor) return {};
        foundVersion = FormatVersion(major, minor);
        return "'" + path + "' is HALCON " + foundVersion;
    };

    // Already in the process (a plug-in host, or a second Refresh) — nothing to
    // choose, the loaded pair is what we will call, so it must still match.
    bool allPresent = true;
    for (const char* mod : kModules) if (!GetModuleHandleA(mod)) allPresent = false;
    if (allPresent) {
        for (const char* mod : kModules) {
            char path[MAX_PATH] = {};
            GetModuleFileNameA(GetModuleHandleA(mod), path, MAX_PATH);
            const std::string why = versionProblem(path);
            if (!why.empty()) {
                err = "already loaded " + why + ", this build needs HALCON " + built;
                return false;
            }
        }
        return true;
    }

    std::vector<std::string> binDirs;
    for (const std::string& r : roots)
        if (!r.empty()) binDirs.push_back(r + "\\bin\\" + arch);
    binDirs.push_back({});   // last resort: bare name, resolved through PATH

    std::vector<std::string> refused;
    for (const std::string& dir : binDirs) {
        // Resolve both files first and check both versions before loading
        // either, so a refused install never leaves a module in the process.
        std::vector<std::string> full;
        std::string              why;
        for (const char* mod : kModules) {
            std::string path;
            if (!dir.empty()) {
                path = dir + "\\" + mod;
                if (GetFileAttributesA(path.c_str()) == INVALID_FILE_ATTRIBUTES) path.clear();
            } else {
                char found[MAX_PATH] = {};
                if (SearchPathA(nullptr, mod, nullptr, MAX_PATH, found, nullptr)) path = found;
            }
            if (path.empty()) { why = "-"; break; }   // not here: no message, try the next
            why = versionProblem(path);
            if (!why.empty()) break;
            full.push_back(path);
        }
        if (!why.empty()) {
            if (why != "-") refused.push_back(why);
            continue;
        }

        bool ok = true;
        for (size_t i = 0; i < full.size(); ++i) {
            if (GetModuleHandleA(kModules[i])) continue;
            if (!LoadLibraryExA(full[i].c_str(), nullptr, LOAD_WITH_ALTERED_SEARCH_PATH)) {
                ok = false;
                break;
            }
        }
        if (ok) {
            if (!dir.empty()) usedRoot = dir.substr(0, dir.size() - (arch.size() + 5));
            return true;
        }
    }

    if (!refused.empty()) {
        err = "HALCON version mismatch: this build needs HALCON " + built + " (" + arch +
              ") - ";
        for (size_t i = 0; i < refused.size(); ++i) {
            if (i) err += "; ";
            err += refused[i];
        }
        err += ". Install HALCON " + built + " or rebuild FireDog against the installed release";
        return false;
    }

    err = "halcon.dll / halconcpp.dll could not be loaded (tried ";
    for (size_t i = 0; i < binDirs.size(); ++i) {
        if (i) err += ", ";
        err += binDirs[i].empty() ? "PATH" : ("'" + binDirs[i] + "'");
    }
    err += ")";
    return false;
}

} // namespace

const HalconStatus& HalconEngine::Refresh()
{
    status_              = HalconStatus{};
    status_.compiledIn   = true;
    status_.builtVersion = FormatVersion(kBuiltMajor, kBuiltMinor);

    // Arch: this build's, then the environment, then our own bitness. Never just
    // HALCONARCH — that is the machine's default install, not necessarily one
    // this process can load.
    status_.arch = FIREDOG_HALCON_ARCH;
    if (status_.arch.empty()) status_.arch = EnvOrEmpty("HALCONARCH");
    if (status_.arch.empty())
        status_.arch = (sizeof(void*) == 8) ? "x64-win64" : "x86sse2-win32";

    // Root candidates, in the order they are tried: the install this binary was
    // linked against first (its DLLs certainly match these import libs), then
    // whatever the machine advertises. A version upgrade that moves HALCONROOT
    // is handled by the second entry without a rebuild.
    std::vector<std::string> roots;
    const std::string builtRoot = FIREDOG_HALCON_ROOT;
    const std::string envRoot   = EnvOrEmpty("HALCONROOT");
    if (!builtRoot.empty()) roots.push_back(builtRoot);
    if (!envRoot.empty() && envRoot != builtRoot) roots.push_back(envRoot);
    status_.root = roots.empty() ? std::string() : roots.front();
    ScanLicenses(status_);

    // Gate 2: the runtime. Everything below this line may call HALCON; nothing
    // above it may, because a missing delay-loaded import is an SEH exception
    // that catch(...) does not reliably handle.
    std::string err, usedRoot, foundVersion;
    if (!LoadHalconModules(roots, status_.arch, usedRoot, foundVersion, err)) {
        status_.version = foundVersion;   // the refused release, when that was the reason
        status_.error   = err;
        FD_WARN("vision", "%s", status_.Summary().c_str());
        return status_;
    }
    status_.runtimeLoaded = true;
    // Report where the loaded code actually came from — with several releases
    // installed, "which HALCON am I running?" is the first question.
    if (!usedRoot.empty() && usedRoot != status_.root) {
        status_.root = usedRoot;
        ScanLicenses(status_);   // licences live under the install that answered
    }

    // Version is a system query — no licence needed, so it is readable even on
    // a box whose licence has expired (useful in the failure message).
    try {
        HalconCpp::HTuple info;
        HalconCpp::GetSystem("version", &info);
        if (info.Length() > 0) status_.version = HalconCpp::HString(info[0].S()).Text();
    }
    catch (const HalconCpp::HException& e) {
        status_.version = "?";
        FD_WARN("vision", "HALCON version query failed: %s",
                HalconCpp::HString(e.ErrorMessage()).Text());
    }

    // Gate 3: the licence. Asking the file system is not enough — a .dat can be
    // expired or issued for another host id, and HALCON only says so when a
    // licensed operator runs. gen_image_const + mean_image is the cheapest pair
    // that touches the licensed core.
    try {
        HalconCpp::HObject img, meanImg;
        HalconCpp::GenImageConst(&img, "byte", 16, 16);
        HalconCpp::MeanImage(img, &meanImg, 3, 3);
        status_.licensed = true;
    }
    catch (const HalconCpp::HException& e) {
        status_.error = std::string("licence check failed: ") +
                        HalconCpp::HString(e.ErrorMessage()).Text();
        if (status_.licenseFiles.empty())
            status_.error += " (no *.dat in '" +
                             (status_.licenseDir.empty() ? std::string("<unknown>")
                                                         : status_.licenseDir) + "')";
    }
    catch (...) {
        status_.error = "licence check failed with an unknown HALCON error";
    }

    if (status_.Available()) FD_INFO("vision", "%s", status_.Summary().c_str());
    else                     FD_WARN("vision", "%s", status_.Summary().c_str());
    return status_;
}

#endif // HAS_HALCON

} // namespace firedog
