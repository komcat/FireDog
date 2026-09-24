// Program.cpp
#include "command/Program.h"

#include "core/Log.h"
#include "core/Product.h"   // GenerateUid — node/program identities

#include <exception>
#include <fstream>
#include <map>
#include <utility>

namespace firedog {

namespace {
constexpr const char* kLocalPath = "config/programs.local.json";
constexpr const char* kBasePath  = "config/programs.json";
// A folder's loop count. The ceiling is not a policy about production runs —
// it is what stops a mistyped "1000000" from expanding into a queue that eats
// the process before the first recipe starts.
constexpr int kMaxLoop = 100000;
} // namespace

// ---- Node factories --------------------------------------------------------

ProgramNode MakeRecipeNode(const std::string& recipeUid)
{
    ProgramNode n;
    n.kind      = ProgramNode::Kind::Recipe;
    n.uid       = GenerateUid();
    n.recipeUid = recipeUid;
    return n;
}

ProgramNode MakeFolderNode(const std::string& name)
{
    ProgramNode n;
    n.kind = ProgramNode::Kind::Folder;
    n.uid  = GenerateUid();
    n.name = name;
    n.loop = 1;
    return n;
}

// ---- Copies ----------------------------------------------------------------

namespace {
// The copy, recording old -> new so a slot can re-point its cycle states.
ProgramNode CloneNodeInto(const ProgramNode& n, std::map<std::string, std::string>* remap)
{
    ProgramNode c = n;
    c.uid = GenerateUid();
    if (remap) (*remap)[n.uid] = c.uid;
    c.children.clear();
    c.children.reserve(n.children.size());
    for (const ProgramNode& k : n.children) c.children.push_back(CloneNodeInto(k, remap));
    return c;
}
} // namespace

ProgramNode CloneNode(const ProgramNode& n)
{
    return CloneNodeInto(n, nullptr);
}

ProgramSlot CloneSlot(const ProgramSlot& s)
{
    std::map<std::string, std::string> remap;
    ProgramSlot c;
    c.name = s.name;
    c.items.reserve(s.items.size());
    for (const ProgramNode& n : s.items) c.items.push_back(CloneNodeInto(n, &remap));
    const auto re = [&remap](const std::string& uid) {
        const auto it = remap.find(uid);
        return it == remap.end() ? std::string() : it->second;
    };
    c.startNodeUid = re(s.startNodeUid);
    c.endNodeUid   = re(s.endNodeUid);
    return c;
}

Program CloneProgram(const Program& p)
{
    Program c;
    c.uid         = GenerateUid();
    c.name        = p.name + " copy";
    c.description = p.description;
    c.slots.reserve(p.slots.size());
    for (const ProgramSlot& s : p.slots) c.slots.push_back(CloneSlot(s));
    return c;
}

// ---- Tree edits ------------------------------------------------------------

ProgramNode* FindNode(std::vector<ProgramNode>& items, const std::string& uid,
                      std::vector<ProgramNode>** parentOut, int* indexOut)
{
    if (uid.empty()) return nullptr;
    for (int i = 0; i < static_cast<int>(items.size()); ++i) {
        if (items[static_cast<std::size_t>(i)].uid == uid) {
            if (parentOut) *parentOut = &items;
            if (indexOut)  *indexOut  = i;
            return &items[static_cast<std::size_t>(i)];
        }
        ProgramNode& n = items[static_cast<std::size_t>(i)];
        if (n.IsFolder()) {
            if (ProgramNode* hit = FindNode(n.children, uid, parentOut, indexOut))
                return hit;
        }
    }
    return nullptr;
}

const ProgramNode* FindNode(const std::vector<ProgramNode>& items, const std::string& uid)
{
    return FindNode(const_cast<std::vector<ProgramNode>&>(items), uid);
}

bool NodeContains(const ProgramNode& folder, const std::string& uid)
{
    if (folder.uid == uid) return true;
    for (const ProgramNode& c : folder.children) {
        if (NodeContains(c, uid)) return true;
    }
    return false;
}

bool ExtractNode(std::vector<ProgramNode>& items, const std::string& uid, ProgramNode* out)
{
    std::vector<ProgramNode>* parent = nullptr;
    int idx = -1;
    if (!FindNode(items, uid, &parent, &idx)) return false;
    if (out) *out = std::move((*parent)[static_cast<std::size_t>(idx)]);
    parent->erase(parent->begin() + idx);
    return true;
}

bool InsertAfter(std::vector<ProgramNode>& items, const std::string& targetUid,
                 ProgramNode node)
{
    if (targetUid.empty()) {          // no anchor: the slot's top level
        items.push_back(std::move(node));
        return true;
    }
    std::vector<ProgramNode>* parent = nullptr;
    int idx = -1;
    if (!FindNode(items, targetUid, &parent, &idx)) return false;
    parent->insert(parent->begin() + idx + 1, std::move(node));
    return true;
}

bool InsertInto(std::vector<ProgramNode>& items, const std::string& folderUid,
                ProgramNode node)
{
    ProgramNode* f = FindNode(items, folderUid);
    if (!f || !f->IsFolder()) return false;
    f->children.push_back(std::move(node));
    return true;
}

bool MoveNodeUp(std::vector<ProgramNode>& items, const std::string& uid)
{
    std::vector<ProgramNode>* parent = nullptr;
    int idx = -1;
    if (!FindNode(items, uid, &parent, &idx) || idx <= 0) return false;
    std::swap((*parent)[static_cast<std::size_t>(idx - 1)],
              (*parent)[static_cast<std::size_t>(idx)]);
    return true;
}

bool MoveNodeDown(std::vector<ProgramNode>& items, const std::string& uid)
{
    std::vector<ProgramNode>* parent = nullptr;
    int idx = -1;
    if (!FindNode(items, uid, &parent, &idx)) return false;
    if (idx + 1 >= static_cast<int>(parent->size())) return false;
    std::swap((*parent)[static_cast<std::size_t>(idx)],
              (*parent)[static_cast<std::size_t>(idx + 1)]);
    return true;
}

bool IndentNode(std::vector<ProgramNode>& items, const std::string& uid)
{
    std::vector<ProgramNode>* parent = nullptr;
    int idx = -1;
    if (!FindNode(items, uid, &parent, &idx) || idx <= 0) return false;
    // Only into a folder, and only the one directly above: "indent" is a
    // one-step gesture, not a search for somewhere plausible to land.
    if (!(*parent)[static_cast<std::size_t>(idx - 1)].IsFolder()) return false;
    ProgramNode moved = std::move((*parent)[static_cast<std::size_t>(idx)]);
    parent->erase(parent->begin() + idx);
    (*parent)[static_cast<std::size_t>(idx - 1)].children.push_back(std::move(moved));
    return true;
}

namespace {
// The list holding `uid`, plus the uid of the folder that owns that list
// (empty at the slot's top level). Written as a scan rather than a back-link
// because ProgramNode has no parent pointer — a tree that owns its children by
// value cannot keep one valid across a std::vector reallocation.
bool ParentFolderOf(std::vector<ProgramNode>& items, const std::string& uid,
                    std::string* folderUidOut)
{
    for (ProgramNode& n : items) {
        if (n.uid == uid) { if (folderUidOut) folderUidOut->clear(); return true; }
        if (!n.IsFolder()) continue;
        for (const ProgramNode& c : n.children) {
            if (c.uid == uid) { if (folderUidOut) *folderUidOut = n.uid; return true; }
        }
        if (ParentFolderOf(n.children, uid, folderUidOut)) return true;
    }
    return false;
}
} // namespace

bool OutdentNode(std::vector<ProgramNode>& items, const std::string& uid)
{
    std::string folderUid;
    if (!ParentFolderOf(items, uid, &folderUid)) return false;
    if (folderUid.empty()) return false;   // already at the top level
    ProgramNode moved;
    if (!ExtractNode(items, uid, &moved)) return false;
    // Lands immediately AFTER the folder it came out of, which is where the
    // eye expects it: outdent moves a step out, not a step to the end.
    return InsertAfter(items, folderUid, std::move(moved));
}

int CountRecipeNodes(const std::vector<ProgramNode>& items)
{
    int n = 0;
    for (const ProgramNode& it : items) {
        if (it.IsFolder()) n += CountRecipeNodes(it.children);
        else               ++n;
    }
    return n;
}

// ---- Persistence -----------------------------------------------------------

namespace {

nlohmann::json NodeToJson(const ProgramNode& n)
{
    nlohmann::json j;
    j["uid"] = n.uid;
    if (n.IsFolder()) {
        j["kind"] = "folder";
        j["name"] = n.name;
        j["loop"] = n.loop;
        if (!n.alignProcUid.empty()) j["align"] = n.alignProcUid;
        nlohmann::json kids = nlohmann::json::array();
        for (const ProgramNode& c : n.children) kids.push_back(NodeToJson(c));
        j["items"] = std::move(kids);
    } else {
        j["kind"]   = "recipe";
        j["recipe"] = n.recipeUid;
    }
    return j;
}

ProgramNode NodeFromJson(const nlohmann::json& j)
{
    ProgramNode n;
    n.uid = j.value("uid", std::string());
    if (n.uid.empty()) n.uid = GenerateUid();   // backfill: pre-uid files
    if (j.value("kind", std::string("recipe")) == "folder") {
        n.kind         = ProgramNode::Kind::Folder;
        n.name         = j.value("name", std::string("folder"));
        n.loop         = j.value("loop", 1);
        n.alignProcUid = j.value("align", std::string());
        if (n.loop < 1)        n.loop = 1;
        if (n.loop > kMaxLoop) n.loop = kMaxLoop;
        if (j.contains("items") && j["items"].is_array()) {
            for (const nlohmann::json& cj : j["items"]) {
                if (cj.is_object()) n.children.push_back(NodeFromJson(cj));
            }
        }
    } else {
        n.kind      = ProgramNode::Kind::Recipe;
        n.recipeUid = j.value("recipe", std::string());
    }
    return n;
}

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
        sj["name"] = s.name;
        if (!s.startNodeUid.empty()) sj["cycle_start"] = s.startNodeUid;
        if (!s.endNodeUid.empty())   sj["cycle_end"]   = s.endNodeUid;
        nlohmann::json items = nlohmann::json::array();
        for (const ProgramNode& n : s.items) items.push_back(NodeToJson(n));
        sj["items"] = std::move(items);
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
            s.name         = sj.value("name", std::string("slot"));
            s.startNodeUid = sj.value("cycle_start", std::string());
            s.endNodeUid   = sj.value("cycle_end",   std::string());
            if (sj.contains("items") && sj["items"].is_array()) {
                for (const nlohmann::json& nj : sj["items"]) {
                    if (nj.is_object()) s.items.push_back(NodeFromJson(nj));
                }
            } else if (sj.contains("recipes") && sj["recipes"].is_array()) {
                // Pre-folder shape: a flat list of recipe uids. Read forever —
                // saved programs are production documents, and the migration
                // costs one line here against a file nobody can re-author.
                for (const nlohmann::json& r : sj["recipes"]) {
                    if (r.is_string()) s.items.push_back(MakeRecipeNode(r.get<std::string>()));
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
