// Program.cpp
#include "command/Program.h"

#include "core/Log.h"
#include "core/Product.h"   // GenerateUid — backfill for programs without one

#include <exception>
#include <fstream>

namespace firedog {

namespace {
constexpr const char* kLocalPath = "config/programs.local.json";
constexpr const char* kBasePath  = "config/programs.json";
} // namespace

nlohmann::json ProgramToJson(const Program& p)
{
    nlohmann::json j;
    j["uid"]  = p.uid;
    j["name"] = p.name;
    if (!p.description.empty()) j["description"] = p.description;
    nlohmann::json slots = nlohmann::json::array();
    for (const ProgramSlot& s : p.slots) {
        nlohmann::json sj;
        sj["name"]    = s.name;
        sj["recipes"] = s.recipes;
        slots.push_back(std::move(sj));
    }
    j["slots"] = std::move(slots);
    return j;
}

Program ProgramFromJson(const nlohmann::json& j)
{
    Program p;
    p.uid         = j.value("uid", std::string());
    p.name        = j.value("name", std::string("program"));
    p.description = j.value("description", std::string());
    if (p.uid.empty()) p.uid = GenerateUid();
    if (j.contains("slots") && j["slots"].is_array()) {
        for (const nlohmann::json& sj : j["slots"]) {
            if (!sj.is_object()) continue;
            ProgramSlot s;
            s.name = sj.value("name", std::string("slot"));
            if (sj.contains("recipes") && sj["recipes"].is_array()) {
                for (const nlohmann::json& r : sj["recipes"]) {
                    if (r.is_string()) s.recipes.push_back(r.get<std::string>());
                }
            }
            p.slots.push_back(std::move(s));
        }
    }
    return p;
}

std::vector<Program> LoadProgramsFile(const std::string& path, std::string* uidOut)
{
    std::ifstream f(path);
    if (!f.is_open()) return {};
    try {
        nlohmann::json j;
        f >> j;
        if (uidOut) *uidOut = j.value("uid", std::string());
        std::vector<Program> out;
        if (j.contains("programs") && j["programs"].is_array()) {
            for (const nlohmann::json& pj : j["programs"]) {
                if (pj.is_object()) out.push_back(ProgramFromJson(pj));
            }
        }
        FD_INFO("program", "loaded %zu program(s) from %s", out.size(), path.c_str());
        return out;
    } catch (const std::exception& e) {
        FD_WARN("program", "programs load failed (%s): %s", path.c_str(), e.what());
        return {};
    }
}

std::vector<Program> LoadPrograms()
{
    std::ifstream probe(kLocalPath);
    if (probe.is_open()) {
        probe.close();
        return LoadProgramsFile(kLocalPath);
    }
    return LoadProgramsFile(kBasePath);
}

bool SaveProgramsFile(const std::string& path, const std::vector<Program>& programs,
                      const std::string& uid)
{
    nlohmann::json j;
    j["uid"] = uid;
    nlohmann::json arr = nlohmann::json::array();
    for (const Program& p : programs) arr.push_back(ProgramToJson(p));
    j["programs"] = std::move(arr);

    std::ofstream f(path);
    if (!f.is_open()) {
        FD_WARN("program", "programs save failed: cannot open %s", path.c_str());
        return false;
    }
    f << j.dump(2) << '\n';
    return true;
}

const Recipe* RecipeByUid(const std::vector<Recipe>& library, const std::string& uid)
{
    for (const Recipe& r : library) {
        if (r.uid == uid) return &r;
    }
    return nullptr;
}

} // namespace firedog
