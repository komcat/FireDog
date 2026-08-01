// HalconEngine.cpp : the three-gate HALCON probe. See HalconEngine.h.
//
// This is the ONLY translation unit that includes a HALCON header. Everything
// else asks HalconEngine::Instance().Available() and gets a bool plus a reason.
#include "vision/HalconEngine.h"

#include "core/Log.h"

#include <cstdlib>
#include <filesystem>

#ifdef HAS_HALCON
  #ifndef WIN32_LEAN_AND_MEAN
    #define WIN32_LEAN_AND_MEAN
  #endif
  #ifndef NOMINMAX
    #define NOMINMAX
  #endif
  #include <windows.h>

  #include <halconcpp/HalconCpp.h>
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

// Licence discovery mirrors HALCON's own order: HALCONLICENSEDIR wins, else
// <root>/license. Listing the .dat files is worth it on its own — "no *.dat in
// <dir>" is the single most useful thing to show an operator.
void ScanLicenses(HalconStatus& s)
{
    namespace fs = std::filesystem;
    s.licenseFiles.clear();   // re-scannable: the root can change mid-Refresh
    s.licenseDir = EnvOrEmpty("HALCONLICENSEDIR");
    if (s.licenseDir.empty() && !s.root.empty()) s.licenseDir = s.root + "\\license";
    if (s.licenseDir.empty()) return;

    std::error_code ec;
    if (!fs::is_directory(s.licenseDir, ec)) return;
    for (const auto& e : fs::directory_iterator(s.licenseDir, ec)) {
        if (!e.is_regular_file(ec)) continue;
        const fs::path p = e.path();
        if (p.extension() == ".dat") s.licenseFiles.push_back(p.filename().string());
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

// Load the HALCON runtime explicitly BEFORE any delay-loaded call, trying each
// candidate install in turn and finally the bare name (PATH). Returns false with
// a reason when the modules are simply not on this machine — the case /DELAYLOAD
// exists to survive. On success `usedRoot` names the install that answered, so
// the UI reports where the running code actually came from.
//
// Both modules must come from the SAME install: mixing halcon.dll from one
// release with halconcpp.dll from another is a crash waiting for its first
// operator call.
bool LoadHalconModules(const std::vector<std::string>& roots, const std::string& arch,
                       std::string& usedRoot, std::string& err)
{
    static const char* kModules[] = {"halcon.dll", "halconcpp.dll"};

    // Already in the process (a plug-in host, or a second Refresh) — nothing to
    // choose, the loaded pair is what we will call.
    bool allPresent = true;
    for (const char* mod : kModules) if (!GetModuleHandleA(mod)) allPresent = false;
    if (allPresent) return true;

    std::vector<std::string> binDirs;
    for (const std::string& r : roots)
        if (!r.empty()) binDirs.push_back(r + "\\bin\\" + arch);
    binDirs.push_back({});   // last resort: bare name, resolved through PATH

    for (const std::string& dir : binDirs) {
        bool ok = true;
        for (const char* mod : kModules) {
            if (GetModuleHandleA(mod)) continue;
            HMODULE h = nullptr;
            if (!dir.empty()) {
                const std::string full = dir + "\\" + mod;
                h = LoadLibraryExA(full.c_str(), nullptr, LOAD_WITH_ALTERED_SEARCH_PATH);
            } else {
                h = LoadLibraryA(mod);
            }
            if (!h) { ok = false; break; }
        }
        if (ok) {
            if (!dir.empty()) usedRoot = dir.substr(0, dir.size() - (arch.size() + 5));
            return true;
        }
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
    status_            = HalconStatus{};
    status_.compiledIn = true;

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
    std::string err, usedRoot;
    if (!LoadHalconModules(roots, status_.arch, usedRoot, err)) {
        status_.error = err;
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
