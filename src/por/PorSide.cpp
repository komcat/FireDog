// PorSide.cpp : see PorSide.h. Reads only.
#include "por/PorSide.h"

#include <algorithm>
#include <filesystem>
#include <system_error>

namespace fs = std::filesystem;

namespace firedog {

namespace {

std::string Slash(const std::string& p)
{
    std::string s = fs::path(p).lexically_normal().generic_string();
    while (s.size() > 3 && s.back() == '/') s.pop_back();
    return s;
}

bool IsFile(const std::string& p)
{
    std::error_code ec;
    return fs::is_regular_file(p, ec);
}

// File names (not paths) of the regular files in `dir`; nothing when absent.
void AddFileNames(const std::string& dir, std::set<std::string>* out)
{
    std::error_code ec;
    if (!fs::is_directory(dir, ec)) return;
    for (const auto& e : fs::directory_iterator(dir, ec)) {
        std::error_code ec2;
        if (e.is_regular_file(ec2)) out->insert(e.path().filename().string());
    }
}

// The product's file for `role`, else (missing or EMPTY) the side's
// config/<local>. `load` returns the document and whether it holds anything.
template <class Doc, class LoadFn, class EmptyFn>
Doc LoadRole(PorSide& s, const char* role, const char* local, LoadFn load, EmptyFn empty)
{
    const std::string prod = s.manifest.FilePath(role);
    if (!prod.empty() && IsFile(prod)) {
        Doc d = load(prod);
        if (!empty(d)) {
            s.notes.push_back(std::string(role) + ": " + prod);
            return d;
        }
    }
    const std::string fb = s.root + "/config/" + local;
    if (IsFile(fb)) {
        Doc d = load(fb);
        s.notes.push_back(std::string(role) + ": " +
                          (prod.empty() ? "no manifest entry" : "product file empty") +
                          " - using " + fb + ", as the app would");
        return d;
    }
    s.notes.push_back(std::string(role) + ": none");
    return Doc{};
}

void LoadHardware(PorSide& s)
{
    const std::string path = s.root + "/config/hardware.json";
    if (!IsFile(path)) {
        s.notes.push_back("hardware: no " + path + " - device and pin names are not checked");
        return;
    }
    s.hwConfig    = HardwareConfig::Load(path);
    s.hasHardware = true;
    const HardwareConfig& hc = s.hwConfig;
    for (const MotionConfig& m : hc.motionDevices)
        if (m.enabled) s.hw.motion[m.name] = m.controllerType;
    for (const IoDeviceConfig& io : hc.ioDevices) {
        if (!io.enabled) continue;
        PorHardwareNames::Io& names = s.hw.io[io.name];
        for (const auto& [pin, name] : io.inputNames)  names.inputs.insert(name);
        for (const auto& [pin, name] : io.outputNames) names.outputs.insert(name);
    }
    for (const PsuConfig& p : hc.psus)
        if (p.enabled) s.hw.psus.push_back(p.name);
    for (const PneumaticSlideConfig& sl : hc.pneumaticSlides)
        if (sl.enabled) s.hw.slides.insert(sl.name);
    for (const LightConfig& l : hc.lights) {
        if (!l.enabled) continue;
        s.hw.hasLight = true;
        for (const LightChannelConfig& ch : l.channels)
            if (ch.enabled) s.hw.lightChannels.push_back(ch);
    }
    s.notes.push_back("hardware: " + path);
}

} // namespace

std::vector<std::string> ListPorProducts(const std::string& root)
{
    std::vector<std::string> out;
    std::error_code ec;
    const fs::path dir = fs::path(root) / "products";
    if (!fs::is_directory(dir, ec)) return out;
    for (const auto& e : fs::directory_iterator(dir, ec)) {
        std::error_code ec2;
        if (e.is_directory(ec2) && fs::is_regular_file(e.path() / "manifest.json", ec2))
            out.push_back(e.path().filename().string());
    }
    std::sort(out.begin(), out.end());
    return out;
}

bool LoadPorSide(const std::string& root, const std::string& product, PorSide* out)
{
    *out         = PorSide{};
    PorSide& s   = *out;
    s.root       = Slash(root);
    s.product    = product;
    const std::string dir = s.root + "/products/" + product;

    // The machine's own files first: a target that does not have the product
    // yet still has hardware worth comparing.
    const std::string maps = s.root + "/config/maps.local.json";
    if (IsFile(maps)) {
        s.maps = LoadMotionMapsFile(maps);
        s.notes.push_back("maps: " + maps);
    } else {
        s.notes.push_back("maps: none (" + maps + ")");
    }
    LoadHardware(s);
    const std::string xf = s.root + "/config/motion_transforms.json";
    if (IsFile(xf)) {
        s.transforms    = LoadMotionTransforms(xf);
        s.hasTransforms = true;
        s.notes.push_back("transforms: " + xf);
    } else {
        s.notes.push_back("transforms: none (" + xf + ")");
    }

    s.manifest = LoadProductManifestAt(dir);
    if (!s.manifest.Loaded()) {
        s.notes.push_back("no readable manifest in " + dir);
        return false;
    }

    s.positions = LoadRole<SavedPosMap>(
        s, "positions", "positions.local.json",
        [](const std::string& p) { return LoadSavedPositionsFile(p); },
        [](const SavedPosMap& d) { return d.empty(); });
    s.recipes = LoadRole<std::vector<Recipe>>(
        s, "recipes", "recipes.local.json",
        [](const std::string& p) { return LoadRecipesFile(p); },
        [](const std::vector<Recipe>& d) { return d.empty(); });
    s.programs = LoadRole<std::vector<Program>>(
        s, "programs", "programs.local.json",
        [](const std::string& p) { return LoadProgramsFile(p); },
        [](const std::vector<Program>& d) { return d.empty(); });
    s.aligns = LoadRole<std::vector<AlignProc>>(
        s, "alignments", "alignments.local.json",
        [](const std::string& p) { return LoadAlignProcsFile(p); },
        [](const std::vector<AlignProc>& d) { return d.empty(); });
    s.vision = LoadRole<std::vector<VisionModel>>(
        s, "vision", "vision.local.json",
        [](const std::string& p) { return LoadVisionModelsFile(p); },
        [](const std::vector<VisionModel>& d) { return d.empty(); });
    s.grips = LoadRole<std::vector<GripProfile>>(
        s, "grips", "grips.local.json",
        [](const std::string& p) { return LoadGripProfilesFile(p); },
        [](const std::vector<GripProfile>& d) { return d.empty(); });
    s.trays = LoadRole<std::vector<WaffleTray>>(
        s, "trays", "trays.local.json",
        [](const std::string& p) { return LoadWaffleTraysFile(p); },
        [](const std::vector<WaffleTray>& d) { return d.empty(); });

    AddFileNames(dir + "/scripts", &s.scripts);
    AddFileNames(s.root + "/config/scripts", &s.scripts);
    AddFileNames(dir + "/prompt_images", &s.promptImages);
    AddFileNames(s.root + "/config/prompt_images", &s.promptImages);
    return true;
}

} // namespace firedog
