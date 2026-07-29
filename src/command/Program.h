// Program.h : layer 2.5 — the production program: an ordered deck of SLOTS,
// each slot an ordered bundle of recipes (referenced by uid, rename-safe).
//
// This is bobcat's Program level minus the DAG (which executed sequentially
// in practice) — an honest ordered list; conditional flow lives inside
// recipes/scripts, not between them. A program's uid is minted at creation
// and never changes: DUT $run events reference it. Persistence follows the
// recipes conventions (product manifest "programs" role, else
// config/programs.local.json -> programs.json). Design: docs/run-page-design.md.
#pragma once

#include "command/Recipe.h"

#include <string>
#include <vector>

namespace firedog {

struct ProgramSlot {
    std::string name = "slot";
    std::vector<std::string> recipes;   // recipe uids, in execution order
};

struct Program {
    std::string uid;
    std::string name = "program";
    std::string description;
    std::vector<ProgramSlot> slots;
};

// One consolidated file: { "uid": ..., "programs": [...] }. Tolerant load.
std::vector<Program> LoadProgramsFile(const std::string& path,
                                      std::string* uidOut = nullptr);
std::vector<Program> LoadPrograms();   // config/programs.local.json -> programs.json
bool SaveProgramsFile(const std::string& path, const std::vector<Program>& programs,
                      const std::string& uid);

nlohmann::json ProgramToJson(const Program& p);
Program        ProgramFromJson(const nlohmann::json& j);

// Find a recipe by uid in the library; nullptr if missing.
const Recipe* RecipeByUid(const std::vector<Recipe>& library, const std::string& uid);

} // namespace firedog
