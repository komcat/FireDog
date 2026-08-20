// Program.h : layer 2.5 — the production program: an ordered deck of SLOTS,
// each slot a TREE of nodes — a recipe reference, or a FOLDER holding more
// nodes and a loop count.
//
// This is bobcat's Program level minus the DAG (which executed sequentially
// in practice) — an honest ordered tree; conditional flow lives inside
// recipes/scripts, not between them. A program's uid is minted at creation
// and never changes: DUT $run events reference it. Persistence follows the
// recipes conventions (product manifest "programs" role, else
// config/programs.local.json -> programs.json).
// Design: docs/run-page-design.md, docs/program-organizer-design.md.
#pragma once

#include "command/Recipe.h"

#include <string>
#include <vector>

namespace firedog {

// One entry in a slot. A FOLDER is the unit of repetition and — reserved for
// a later pass — the unit of a taught frame's lifetime: `alignProcUid` names
// the alignment procedure a run re-teaches at the start of EVERY loop pass,
// because each pass handles a different physical part.
struct ProgramNode {
    enum class Kind { Recipe, Folder };

    Kind        kind = Kind::Recipe;
    std::string uid;          // node identity: selection, run marks, persistence
    std::string recipeUid;    // Kind::Recipe — the library reference (rename-safe)
    std::string name;         // Kind::Folder — the label
    int         loop = 1;     // Kind::Folder — passes (>= 1)
    std::string alignProcUid; // Kind::Folder — reserved; empty = no folder frame
    std::vector<ProgramNode> children;   // Kind::Folder

    bool IsFolder() const { return kind == Kind::Folder; }
};

struct ProgramSlot {
    std::string name = "slot";
    std::vector<ProgramNode> items;   // in execution order
};

struct Program {
    std::string uid;
    std::string name = "program";
    std::string description;
    std::vector<ProgramSlot> slots;
};

// ---- Node factories --------------------------------------------------------
// Both mint a node uid: it is the identity the organizer selects by and the
// runner marks against, so it exists from the moment the node does.
ProgramNode MakeRecipeNode(const std::string& recipeUid);
ProgramNode MakeFolderNode(const std::string& name = "folder");

// ---- Tree edits ------------------------------------------------------------
// All of them address a node by uid and search the whole subtree, so the
// organizer never has to carry a path. Every one is a no-op that returns false
// on a uid the tree does not contain.

// Non-null while the node lives in `items`; `parentOut`/`indexOut` (optional)
// receive the list it sits in and its position there.
ProgramNode*       FindNode(std::vector<ProgramNode>& items, const std::string& uid,
                            std::vector<ProgramNode>** parentOut = nullptr,
                            int* indexOut = nullptr);
const ProgramNode* FindNode(const std::vector<ProgramNode>& items, const std::string& uid);

// True when `uid` is `folder` itself or anything beneath it. The guard that
// stops a folder being dropped into its own belly.
bool NodeContains(const ProgramNode& folder, const std::string& uid);

// Cut the node out of the tree, moving it into *out. The caller owns it after
// this: an unpaired Extract is a delete.
bool ExtractNode(std::vector<ProgramNode>& items, const std::string& uid, ProgramNode* out);

// Drop `node` in. `targetUid` empty = append to the slot's top level.
bool InsertAfter(std::vector<ProgramNode>& items, const std::string& targetUid,
                 ProgramNode node);
// Append inside the folder named by `folderUid` (fails if it is not a folder).
bool InsertInto(std::vector<ProgramNode>& items, const std::string& folderUid,
                ProgramNode node);

// Swap with the previous/next SIBLING (never crosses into another folder —
// that is what indent/outdent are for).
bool MoveNodeUp(std::vector<ProgramNode>& items, const std::string& uid);
bool MoveNodeDown(std::vector<ProgramNode>& items, const std::string& uid);
// Into the folder immediately above it / out to sit after its own parent.
bool IndentNode(std::vector<ProgramNode>& items, const std::string& uid);
bool OutdentNode(std::vector<ProgramNode>& items, const std::string& uid);

// Recipe leaves in the subtree, ignoring loop counts (what the tree HOLDS,
// not what a run would execute).
int CountRecipeNodes(const std::vector<ProgramNode>& items);

// ---- Persistence -----------------------------------------------------------
// One consolidated file: { "uid": ..., "programs": [...] }. Tolerant load, and
// it still reads the pre-folder shape (a slot's flat "recipes": [uid, ...]).
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
