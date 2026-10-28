// WndProgramOrganizer.cpp : the Program Organizer — recipe library | 1st slot
// tree | aisle | 2nd slot tree, the ONE surface that edits a program.
// Definitions of App members — see App.h. Design: docs/program-organizer-design.md.
//
// It is a WINDOW, not a page: reshaping a program is done while looking at the
// deck and the DUT panel it will drive, and multi-viewport lets it be dragged
// onto a second monitor. The Run strip's Edit checkbox and this window's close
// box are the same bit (runEdit_ / orgOpen_ are mirrored), so the two can never
// disagree about whether it is up.
//
// The deck stayed READ-ONLY when this landed. The two are different jobs: the
// deck is what an operator reads while running one station, and an editing
// control sitting in it is one mis-click away from changing the program under a
// part that is already in the machine.
//
// Every edit goes through the command/Program tree helpers (ExtractNode /
// InsertAfter / InsertInto / MoveNode* / Indent / Outdent / Clone*) rather than
// being hand-rolled here: a folder is a subtree, and "move this row" is only
// ever correct if the row's children come with it.
//
// Two panels, one program. The 1st panel is the slot being organized (the
// top row's + Slot / name / ^ v / x Slot act on it, the library adds into it);
// the 2nd shows any slot of the same program - or the same slot again - and
// the aisle between them moves or copies the selection across. A slot is
// only ever addressed through the panel that shows it (`panel` 0 or 1), so
// the two selections, the two drop targets and the clipboard can never
// disagree about which tree they mean.
#include "App.h"

#include "imgui.h"

#include "ui/Theme.h"     // kDryRunCyan
#include "ui/Widgets.h"
#include "core/Log.h"

#include <cctype>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

namespace firedog {

namespace {
// Drag payload ids. Two, because the sources carry different things: the
// library hands over a RECIPE uid (a new node is minted on the drop), the tree
// hands over a NODE uid (an existing subtree moves).
constexpr const char* kPayloadRecipe = "ORG_RECIPE";
constexpr const char* kPayloadNode   = "ORG_NODE";

// A folder's loop count. Matches the ceiling command/Program.cpp clamps a
// loaded file to — see there for why the number exists at all.
constexpr int kMaxLoop = 100000;

// Case-insensitive substring, for the library filter box.
bool Contains(const std::string& hay, const char* needle)
{
    if (!needle || !*needle) return true;
    const std::size_t n = std::strlen(needle);
    if (n > hay.size()) return false;
    for (std::size_t i = 0; i + n <= hay.size(); ++i) {
        std::size_t k = 0;
        while (k < n && std::tolower(static_cast<unsigned char>(hay[i + k])) ==
                        std::tolower(static_cast<unsigned char>(needle[k]))) ++k;
        if (k == n) return true;
    }
    return false;
}

// What one press of Execute on this subtree would actually run — loops
// multiplied through. The number an operator needs before starting a x100.
long long ExpandedCount(const std::vector<ProgramNode>& items)
{
    long long n = 0;
    for (const ProgramNode& it : items) {
        if (it.IsFolder()) n += static_cast<long long>(it.loop < 1 ? 1 : it.loop) *
                                ExpandedCount(it.children);
        else               ++n;
    }
    return n;
}

// A node leaving a slot cannot stay one of its cycle states.
void ForgetCycleState(ProgramSlot& slot, const std::string& uid)
{
    if (slot.startNodeUid == uid) slot.startNodeUid.clear();
    if (slot.endNodeUid   == uid) slot.endNodeUid.clear();
}
} // namespace

// ---- Insert helpers --------------------------------------------------------

// The ONE landing rule. Inside the selected folder, else after the selected
// row, else at the end of the slot — i.e. where the eye last was, which is what
// makes clicking down a list of recipes build the deck in order. A new FOLDER
// lands beside a selected folder (`besideFolder`): nesting is a deliberate act
// (drag, or the > button), not what you get for pressing "+ Folder" twice.
void App::OrgPlace(ProgramSlot& slot, int panel, ProgramNode n, bool besideFolder)
{
    std::string& sel = OrgSel(panel);
    const std::string uid = n.uid;
    const ProgramNode* at = FindNode(slot.items, sel);
    if (at && at->IsFolder() && !besideFolder) InsertInto(slot.items, sel, std::move(n));
    else if (at)                               InsertAfter(slot.items, sel, std::move(n));
    else                                       slot.items.push_back(std::move(n));
    sel = uid;   // the new row is the next insertion point
}

// Library -> the panel the library's "add to" choice names. Said out loud in
// the library header rather than implied, because with two trees on screen
// "add" has two possible meanings and a silent default is a recipe landing in
// the wrong slot. A drag still aims wherever it is dropped.
void App::OrgAddRecipe(const std::string& recipeUid)
{
    if (progSel_ < 0 || progSel_ >= static_cast<int>(session_.Programs().size())) return;
    Program& prog = session_.Programs()[progSel_];
    const int panel = (orgTwo_ && orgAddPanel_ == 1) ? 1 : 0;
    const int si    = panel == 0 ? orgSlot_ : orgSlot2_;
    if (si < 0 || si >= static_cast<int>(prog.slots.size())) return;
    OrgPlace(prog.slots[static_cast<std::size_t>(si)], panel, MakeRecipeNode(recipeUid));
}

void App::OrgAddFolder(ProgramSlot& slot, int panel)
{
    OrgPlace(slot, panel, MakeFolderNode(), /*besideFolder=*/true);
}

// One row's drop target. `intoFolder` decides the two meanings a drop can
// have: onto a FOLDER row it goes inside, onto a recipe row it lands after.
//
// It only RECORDS the landing. Acting here would insert into a vector that
// DrawOrgTree is in the middle of walking by reference — the reallocation
// leaves the loop holding a dangling node. ApplyOrgDrop runs it once the tree
// is drawn.
void App::DrawOrgNodeDropTarget(const std::string& anchorUid, bool intoFolder, int panel)
{
    if (!ImGui::BeginDragDropTarget()) return;
    if (const ImGuiPayload* pl = ImGui::AcceptDragDropPayload(kPayloadRecipe)) {
        orgDropRecipe_  = std::string(static_cast<const char*>(pl->Data),
                                      static_cast<std::size_t>(pl->DataSize));
        orgDropNode_.clear();
        orgDropAnchor_  = anchorUid;
        orgDropInto_    = intoFolder;
        orgDropPanel_   = panel;
        orgDropPending_ = true;
    }
    if (const ImGuiPayload* pl = ImGui::AcceptDragDropPayload(kPayloadNode)) {
        orgDropNode_    = std::string(static_cast<const char*>(pl->Data),
                                      static_cast<std::size_t>(pl->DataSize));
        orgDropRecipe_.clear();
        orgDropAnchor_  = anchorUid;
        orgDropInto_    = intoFolder;
        orgDropPanel_   = panel;
        orgDropPending_ = true;
    }
    ImGui::EndDragDropTarget();
}

// The recorded drop, applied after the tree draw, by the panel it was recorded
// on. An empty anchor means the slot's top level, at the end. A node uid the
// slot does not hold is looked for in the OTHER panel's slot and moved across
// — dragging between the panels is the same act as the aisle's Move button.
void App::ApplyOrgDrop(ProgramSlot& slot, ProgramSlot* other, int panel)
{
    if (!orgDropPending_ || orgDropPanel_ != panel) return;
    orgDropPending_ = false;
    std::string& sel = OrgSel(panel);

    if (!orgDropRecipe_.empty()) {
        ProgramNode n = MakeRecipeNode(orgDropRecipe_);
        sel = n.uid;
        if (orgDropAnchor_.empty())   slot.items.push_back(std::move(n));
        else if (orgDropInto_)        InsertInto(slot.items, orgDropAnchor_, std::move(n));
        else                          InsertAfter(slot.items, orgDropAnchor_, std::move(n));
        orgDropRecipe_.clear();
        return;
    }
    if (orgDropNode_.empty()) return;

    // Where the node comes from: this slot, else the other panel's.
    ProgramSlot* from = FindNode(slot.items, orgDropNode_) ? &slot
                      : (other && FindNode(other->items, orgDropNode_)) ? other
                      : nullptr;
    if (!from) { orgDropNode_.clear(); return; }

    // Refuse a folder dropped onto itself or into its own belly: the extract
    // would take the destination with it and the subtree would vanish.
    const ProgramNode* moving = FindNode(from->items, orgDropNode_);
    if (!orgDropAnchor_.empty() &&
        (orgDropAnchor_ == orgDropNode_ ||
         (moving->IsFolder() && NodeContains(*moving, orgDropAnchor_)))) {
        orgDropNode_.clear();
        return;
    }
    ProgramNode n;
    if (ExtractNode(from->items, orgDropNode_, &n)) {
        if (from != &slot) {
            ForgetCycleState(*from, n.uid);
            if (OrgSel(1 - panel) == n.uid) OrgSel(1 - panel).clear();
        }
        // The inserts take the node BY VALUE, so a failed insert would destroy
        // the subtree it was asked to move. Hold a copy until one succeeds:
        // losing a folder full of recipes to a mis-aimed drop is not a bug an
        // operator can undo.
        const std::string uid = n.uid;
        ProgramNode backup = n;
        bool ok = true;
        if (orgDropAnchor_.empty())   slot.items.push_back(std::move(n));
        else if (orgDropInto_)        ok = InsertInto(slot.items, orgDropAnchor_, std::move(n));
        else                          ok = InsertAfter(slot.items, orgDropAnchor_, std::move(n));
        if (!ok) {
            FD_WARN("program", "organizer: drop anchor vanished; item kept at the end");
            slot.items.push_back(std::move(backup));
        }
        sel = uid;
    }
    orgDropNode_.clear();
}

// ---- The tree --------------------------------------------------------------

void App::DrawOrgTree(ProgramSlot& slot, std::vector<ProgramNode>& items, bool topLevel,
                      int panel)
{
    std::string& sel = OrgSel(panel);
    // The cycle context menu. Assigning a uid cannot reallocate anything, so
    // unlike a drop it is safe to apply inside the walk.
    const auto cycMenu = [&](const ProgramNode& n, bool rClick) {
        if (!topLevel) return;   // a nested node is not something Execute launches
        if (rClick) ImGui::OpenPopup("cyc");
        if (!ImGui::BeginPopup("cyc")) return;
        ImGui::TextDisabled("deck cycle");
        ImGui::Separator();
        if (ImGui::MenuItem("Set as cycle start")) slot.startNodeUid = n.uid;
        if (ImGui::MenuItem("Set as cycle end"))   slot.endNodeUid   = n.uid;
        if (ImGui::MenuItem("Clear cycle")) {
            slot.startNodeUid.clear();
            slot.endNodeUid.clear();
        }
        ImGui::EndPopup();
    };
    // The chip that says which state this is. Drawn AFTER the drag source, so
    // it cannot become the item the drag binds to.
    const auto cycChip = [&](const ProgramNode& n) {
        const char* cap = (!slot.startNodeUid.empty() && n.uid == slot.startNodeUid) ? "start"
                        : (!slot.endNodeUid.empty()   && n.uid == slot.endNodeUid)   ? "end"
                                                                                     : nullptr;
        if (!cap) return;
        ImGui::SameLine();
        ImGui::TextDisabled("[%s]", cap);
    };
    // Safe as a range-for because every drop is DEFERRED (ApplyOrgDrop): no
    // insert or erase happens while this walk is holding a node by reference.
    for (ProgramNode& n : items) {
        ImGui::PushID(n.uid.c_str());

        if (n.IsFolder()) {
            ImGuiTreeNodeFlags flags = ImGuiTreeNodeFlags_OpenOnArrow |
                                       ImGuiTreeNodeFlags_SpanAvailWidth |
                                       ImGuiTreeNodeFlags_DefaultOpen;
            if (sel == n.uid) flags |= ImGuiTreeNodeFlags_Selected;
            const long long runs = static_cast<long long>(n.loop < 1 ? 1 : n.loop) *
                                   ExpandedCount(n.children);
            const bool open = ImGui::TreeNodeEx("##f", flags, "[+] %s   x%d   (%lld runs)",
                                                n.name.c_str(), n.loop, runs);
            const bool fRClick = ImGui::IsItemClicked(ImGuiMouseButton_Right);
            if (ImGui::IsItemClicked() && !ImGui::IsItemToggledOpen()) sel = n.uid;
            if (ImGui::BeginDragDropSource()) {
                ImGui::SetDragDropPayload(kPayloadNode, n.uid.c_str(), n.uid.size());
                ImGui::Text("%s (folder)", n.name.c_str());
                ImGui::EndDragDropSource();
            }
            DrawOrgNodeDropTarget(n.uid, /*intoFolder=*/true, panel);
            if (n.dryRun) {
                ImGui::SameLine();
                ImGui::TextColored(kDryRunCyan, "[DRY RUN]");
            }
            cycChip(n);
            if (open) {
                DrawOrgTree(slot, n.children, /*topLevel=*/false, panel);
                if (n.children.empty()) {
                    ImGui::TextDisabled("(empty - drop recipes here)");
                    // The empty folder's own body is a target too, or a folder
                    // you just made is the one thing you cannot fill by drag.
                    DrawOrgNodeDropTarget(n.uid, /*intoFolder=*/true, panel);
                }
                ImGui::TreePop();
            }
            cycMenu(n, fRClick);
            ImGui::PopID();
            continue;
        }

        const Recipe* r  = RecipeByUid(session_.Recipes(), n.recipeUid);
        const bool missing = (r == nullptr);
        if (missing) ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.4f, 0.3f, 1.0f));
        char row[128];
        std::snprintf(row, sizeof(row), "    %s",
                      missing ? (n.recipeUid + " (missing!)").c_str() : r->name.c_str());
        if (ImGui::Selectable(row, sel == n.uid)) sel = n.uid;
        const bool rRClick = ImGui::IsItemClicked(ImGuiMouseButton_Right);
        if (missing) ImGui::PopStyleColor();
        if (ImGui::BeginDragDropSource()) {
            ImGui::SetDragDropPayload(kPayloadNode, n.uid.c_str(), n.uid.size());
            ImGui::TextUnformatted(missing ? n.recipeUid.c_str() : r->name.c_str());
            ImGui::EndDragDropSource();
        }
        DrawOrgNodeDropTarget(n.uid, /*intoFolder=*/false, panel);
        cycChip(n);
        cycMenu(n, rRClick);
        ImGui::PopID();
    }
}

// ---- One panel -------------------------------------------------------------

void App::DrawOrgPanel(ProgramSlot& slot, ProgramSlot* other, int panel)
{
    std::string& sel = OrgSel(panel);

    // Organize row: everything acts on the SELECTED node, so one set of buttons
    // serves every depth. > / < are indent/outdent — the keyboard-free way into
    // and out of a folder for anyone who would rather not drag.
    if (ImGui::SmallButton("+ Folder")) OrgAddFolder(slot, panel);
    ImGui::SameLine();
    const bool haveSel = FindNode(slot.items, sel) != nullptr;
    // Square icon tool buttons, the Recipe Editor's set (ui/Widgets
    // IconToolButton): move | remove | clipboard.
    ImGui::BeginDisabled(!haveSel);
    if (IconToolButton(UiIcon("tb_move_up"), "orgUp", "^", "Move up (within its folder)"))
        MoveNodeUp(slot.items, sel);
    ImGui::SameLine();
    if (IconToolButton(UiIcon("tb_move_down"), "orgDown", "v", "Move down (within its folder)"))
        MoveNodeDown(slot.items, sel);
    ImGui::SameLine();
    if (IconToolButton(UiIcon("tb_indent"), "orgIn", ">", "Move into the folder above it"))
        IndentNode(slot.items, sel);
    ImGui::SameLine();
    if (IconToolButton(UiIcon("tb_outdent"), "orgOut", "<", "Move out of its folder"))
        OutdentNode(slot.items, sel);
    ToolGroupGap();
    if (IconToolButton(UiIcon("tb_delete"), "orgRemove", "Remove",
                       "Remove from the slot (a folder goes with its contents)", true)) {
        // A folder goes WITH its children: the tree owns them, and leaving them
        // behind at the parent's level is a silent restructure nobody asked for.
        // A node that no longer exists cannot stay a cycle state. (A node
        // dragged INTO a folder needs no cleanup: DeckCycleRange stops
        // resolving it and the line below says so.)
        ForgetCycleState(slot, sel);
        ExtractNode(slot.items, sel, nullptr);
        sel.clear();
    }
    // The clipboard. Cut is Remove that keeps the subtree; Copy leaves it;
    // Paste lands a CLONE at this panel's selection - fresh uids, so the same
    // clip can be pasted twice and a pasted folder never answers to the
    // original's name in a cycle or a run mark.
    ToolGroupGap();
    if (IconToolButton(UiIcon("tb_cut"), "orgCut", "Cut",
                       "Cut: remove it and keep it on the clipboard")) {
        ProgramNode n;
        if (ExtractNode(slot.items, sel, &n)) {
            ForgetCycleState(slot, sel);
            orgClip_.clear();
            orgClip_.push_back(std::move(n));
            sel.clear();
        }
    }
    ImGui::SameLine();
    if (IconToolButton(UiIcon("tb_copy"), "orgCopy", "Copy", "Copy to the clipboard")) {
        if (const ProgramNode* n = FindNode(slot.items, sel)) {
            orgClip_.clear();
            orgClip_.push_back(*n);
        }
    }
    ImGui::EndDisabled();
    ImGui::SameLine();
    // The tooltip names what is on the clipboard, so it is built first.
    std::string pasteTip = "Paste (the clipboard is empty)";
    if (!orgClip_.empty()) {
        const ProgramNode& c = orgClip_.front();
        const Recipe* r = c.IsFolder() ? nullptr : RecipeByUid(session_.Recipes(), c.recipeUid);
        pasteTip = std::string("Paste ") + (c.IsFolder() ? "folder" : "recipe") + " '" +
                   (c.IsFolder() ? c.name : r ? r->name : c.recipeUid) + "' at the selection";
    }
    ImGui::BeginDisabled(orgClip_.empty());
    if (IconToolButton(UiIcon("tb_paste"), "orgPaste", "Paste", pasteTip.c_str()))
        OrgPlace(slot, panel, CloneNode(orgClip_.front()));
    ImGui::EndDisabled();
    // Rename THIS panel's slot. The top row's name field only reaches the 1st
    // panel's slot; the 2nd panel needs its own way in. A popup rather than a
    // permanent field, so the toolbar stays one row.
    ImGui::SameLine();
    if (ImGui::SmallButton("Rename")) {
        std::snprintf(orgSlotNameBuf_, sizeof(orgSlotNameBuf_), "%s", slot.name.c_str());
        ImGui::OpenPopup("renameSlot");
    }
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("Rename this slot");
    if (ImGui::BeginPopup("renameSlot")) {
        ImGui::TextDisabled("slot name");
        if (ImGui::IsWindowAppearing()) ImGui::SetKeyboardFocusHere();
        ImGui::SetNextItemWidth(220.0f);
        const bool enter = ImGui::InputText("##slotname", orgSlotNameBuf_,
                                            sizeof(orgSlotNameBuf_),
                                            ImGuiInputTextFlags_EnterReturnsTrue);
        if (enter || ImGui::Button("OK", ImVec2(90.0f, 0.0f))) {
            if (orgSlotNameBuf_[0] != '\0') slot.name = orgSlotNameBuf_;
            ImGui::CloseCurrentPopup();
        }
        ImGui::SameLine();
        if (ImGui::Button("Cancel", ImVec2(90.0f, 0.0f))) ImGui::CloseCurrentPopup();
        ImGui::EndPopup();
    }

    // The selected folder's own knobs. A recipe node has nothing to edit — it
    // IS a reference; its content is the Recipe Editor's business.
    if (ProgramNode* fsel = FindNode(slot.items, sel)) {
        if (fsel->IsFolder()) {
            ImGui::Separator();
            char fbuf[64];
            std::snprintf(fbuf, sizeof(fbuf), "%s", fsel->name.c_str());
            ImGui::SetNextItemWidth(180.0f);
            if (ImGui::InputText("folder", fbuf, sizeof(fbuf))) fsel->name = fbuf;
            ImGui::SameLine();
            ImGui::SetNextItemWidth(140.0f);
            if (ImGui::InputInt("loop", &fsel->loop)) {
                if (fsel->loop < 1)        fsel->loop = 1;
                if (fsel->loop > kMaxLoop) fsel->loop = kMaxLoop;
            }
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Passes over this folder's contents.\n"
                                  "The whole loop is expanded into the run queue at Execute.");
            ImGui::SameLine();
            ImGui::Checkbox("dry run", &fsel->dryRun);
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Dry run: every recipe in this folder (nested folders too)\n"
                                  "runs ONLY its motion, IO and pneumatic steps. Prompts,\n"
                                  "manual adjust, instruments, lights, vision, scans, scripts,\n"
                                  "DUT and saved-position writes are skipped. Loops and ifs run\n"
                                  "as written; an if on a skipped measurement takes its else.\n"
                                  "No DUT record, no deck marks, the cycle does not move.");
        }
    }
    // The cycle, in one line. Right-click a top-level row to set either end.
    {
        // A node's display name. A lambda rather than a free helper: the recipe
        // library is reached through session_, which is App's.
        const auto nodeLabel = [&](const ProgramNode& n) -> std::string {
            if (n.IsFolder()) return n.name;
            const Recipe* r = RecipeByUid(session_.Recipes(), n.recipeUid);
            return r ? r->name : (n.recipeUid + " (missing)");
        };
        ImGui::Separator();
        const bool none = slot.startNodeUid.empty() && slot.endNodeUid.empty();
        int si = 0, ei = 0;
        if (none) {
            ImGui::TextDisabled("cycle: none - right-click a row to set start/end");
        } else if (DeckCycleRange(slot, &si, &ei)) {
            ImGui::TextDisabled("cycle:");
            ImGui::SameLine();
            ImGui::Text("%s  ->  %s",
                        nodeLabel(slot.items[static_cast<std::size_t>(si)]).c_str(),
                        nodeLabel(slot.items[static_cast<std::size_t>(ei)]).c_str());
        } else {
            // Half a cycle is not a cycle: say which half is wrong rather than
            // quietly behaving as if none were set.
            const char* why = (slot.startNodeUid.empty() || slot.endNodeUid.empty())
                                  ? "cycle: only one end is set"
                              : (FindNode(slot.items, slot.startNodeUid) == nullptr ||
                                 FindNode(slot.items, slot.endNodeUid) == nullptr)
                                  ? "cycle: a state no longer exists in this slot"
                                  : "cycle: end is above start, or a state moved into a folder";
            ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.3f, 1.0f), "%s", why);
        }
        if (!none) {
            ImGui::SameLine();
            if (ImGui::SmallButton("Clear##cyc")) {
                slot.startNodeUid.clear();
                slot.endNodeUid.clear();
            }
        }
    }
    ImGui::Separator();

    ImGui::BeginChild("orgTreeBody", ImVec2(0.0f, 0.0f));
    DrawOrgTree(slot, slot.items, /*topLevel=*/true, panel);
    // Tail target: the drop that means "at the end, top level". Without it the
    // only way to append by drag is onto the last row, which lands you INSIDE
    // it whenever that row happens to be a folder.
    ImGui::Dummy(ImVec2(-1.0f, ImGui::GetTextLineHeight()));
    DrawOrgNodeDropTarget(std::string(), /*intoFolder=*/false, panel);
    if (slot.items.empty())
        ImGui::TextDisabled(panel == 0
                                ? "Empty slot - double-click a recipe on the left, or drag one here."
                                : "Empty slot - drag a node here, or use the aisle buttons.");
    ImGui::EndChild();
    // Every drop recorded during the draw lands HERE, with nothing iterating
    // the tree any more.
    ApplyOrgDrop(slot, other, panel);
}

// ---- The window ------------------------------------------------------------

void App::DrawProgramOrganizer()
{
    const ImGuiViewport* mv = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(ImVec2(mv->Pos.x + 120.0f, mv->Pos.y + 100.0f),
                            ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(1040.0f, 560.0f), ImGuiCond_FirstUseEver);
    if (!ImGui::Begin("Program Organizer", &orgOpen_)) {
        ImGui::End();
        runEdit_ = orgOpen_;   // the close box and the strip checkbox are one bit
        return;
    }

    // A run outranks editing: while the orchestrator is busy this window turns
    // into a read-only view rather than closing itself or raising a modal —
    // gated on orch_.Busy(), never on one engine, since the program runner goes
    // momentarily idle between queue items.
    const bool busy = orch_.Busy();
    if (busy) {
        ImGui::TextColored(ImVec4(1.0f, 0.7f, 0.3f, 1.0f),
                           "A run is active - editing is disabled until it finishes.");
        ImGui::Separator();
    }
    ImGui::BeginDisabled(busy);

    // Not const: New, Duplicate and Delete all re-point progSel_ further down
    // this same frame, and reading a stale `true` afterwards indexes
    // Programs()[-1].
    bool haveProg = progSel_ >= 0 &&
                    progSel_ < static_cast<int>(session_.Programs().size());

    // ---- Program row ----
    if (IconToolButton(UiIcon("tb_new"), "progNew", "New", "New program")) {
        Program p;
        p.uid = GenerateUid();   // the traceability identity, minted once
        session_.Programs().push_back(std::move(p));
        SelectDeckProgram(static_cast<int>(session_.Programs().size()) - 1);
        orgSlot_ = deckSlot_;
        orgSlot2_ = -1;
        orgSel_.clear();
        orgSel2_.clear();
    }
    ImGui::SameLine();
    ImGui::BeginDisabled(!haveProg);
    // A whole program, copied: every slot, every node, with fresh uids all the
    // way down and a new program uid (the DUT records name a program by it).
    // The copy becomes the selected program, so what you see is what you got.
    if (IconToolButton(UiIcon("tb_duplicate"), "progDup", "Duplicate",
                       "Copy this program (new uid, every slot)")) {
        Program copy = CloneProgram(session_.Programs()[progSel_]);
        session_.Programs().push_back(std::move(copy));
        SelectDeckProgram(static_cast<int>(session_.Programs().size()) - 1);
        orgSlot_ = deckSlot_;
        orgSlot2_ = -1;
        orgSel_.clear();
        orgSel2_.clear();
    }
    ToolGroupGap();
    // The confirm is a plain popup, not a MODAL: this window can be open on a
    // second monitor while the Run page is up, and a modal would block the
    // prompt / manual_adjust windows an engine worker can be waiting on.
    if (IconToolButton(UiIcon("tb_delete"), "progDel", "Delete", "Delete this program", true))
        orgDelConfirm_ = true;
    ImGui::SameLine();
    if (haveProg) {
        Program& prog = session_.Programs()[progSel_];
        char nbuf[64];
        std::snprintf(nbuf, sizeof(nbuf), "%s", prog.name.c_str());
        ImGui::SetNextItemWidth(220.0f);
        if (ImGui::InputText("program", nbuf, sizeof(nbuf))) prog.name = nbuf;
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("Rename: type here");
        ImGui::SameLine();
        ImGui::TextDisabled("uid: %s", prog.uid.c_str());
    } else {
        ImGui::TextDisabled("no program loaded - New, or Load on the Run page");
    }
    // Save and Close sit on this top row, not under the trees: on a tall
    // library the foot scrolls out of reach of a hand that is up here anyway.
    ImGui::SameLine();
    ImGui::TextDisabled("|");
    ImGui::SameLine();
    if (IconSmallButton(UiIcon("Save"), session_.ProgramsDirty() ? "Save*" : "Save"))
        session_.SavePrograms();
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("Write the programs file");
    ImGui::SameLine();
    if (ImGui::SmallButton("Close")) orgOpen_ = false;
    ImGui::EndDisabled();
    if (orgDelConfirm_) { ImGui::OpenPopup("orgDel"); orgDelConfirm_ = false; }
    if (ImGui::BeginPopup("orgDel")) {
        if (haveProg)
            ImGui::Text("Delete program '%s'?", session_.Programs()[progSel_].name.c_str());
        if (ImGui::Button("Delete", ImVec2(100.0f, 0.0f)) && haveProg) {
            session_.Programs().erase(session_.Programs().begin() + progSel_);
            SelectDeckProgram(session_.Programs().empty() ? -1 : 0);
            orgSlot_ = -1;
            orgSlot2_ = -1;
            orgSel_.clear();
            orgSel2_.clear();
            ImGui::CloseCurrentPopup();
        }
        ImGui::SameLine();
        if (ImGui::Button("Cancel", ImVec2(100.0f, 0.0f))) ImGui::CloseCurrentPopup();
        ImGui::EndPopup();
    }

    // Re-read it: New pushed a program, Delete erased one, and either may have
    // left progSel_ pointing somewhere else — or nowhere.
    haveProg = progSel_ >= 0 && progSel_ < static_cast<int>(session_.Programs().size());
    if (!haveProg) {
        ImGui::EndDisabled();
        ImGui::End();
        runEdit_ = orgOpen_;
        return;
    }
    Program& prog = session_.Programs()[progSel_];

    // ---- Slot row (the 1st panel's slot) ----
    // The organizer's slot is its OWN: reorganising slot 3 must not drag the
    // operator's deck off the station they are standing at.
    {
        const int nSlots = static_cast<int>(prog.slots.size());
        if (nSlots == 0)              orgSlot_ = -1;
        else if (orgSlot_ < 0)        orgSlot_ = (deckSlot_ >= 0 && deckSlot_ < nSlots) ? deckSlot_ : 0;
        else if (orgSlot_ >= nSlots)  orgSlot_ = nSlots - 1;
        // The slot these buttons act on is picked in the 1st panel's own
        // header, right above the tree it shows - a combo up here read as if
        // it controlled both panels. The row says whose slot it edits.
        ImGui::TextDisabled("1st slot:");
        ImGui::SameLine();
        // A new slot is NAMED as it is made: the popup asks, and OK creates
        // it. The row used to carry a permanent name field for the current
        // slot; renaming now lives on each panel's toolbar (Rename), so the
        // row is only the slot's place in the program.
        if (ImGui::SmallButton("+ Slot")) {
            std::snprintf(orgSlotNameBuf_, sizeof(orgSlotNameBuf_), "slot %d", nSlots + 1);
            ImGui::OpenPopup("newSlot");
        }
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("Add a slot (asks for its name)");
        if (ImGui::BeginPopup("newSlot")) {
            ImGui::TextDisabled("new slot name");
            if (ImGui::IsWindowAppearing()) ImGui::SetKeyboardFocusHere();
            ImGui::SetNextItemWidth(220.0f);
            const bool enter = ImGui::InputText("##newslot", orgSlotNameBuf_,
                                                sizeof(orgSlotNameBuf_),
                                                ImGuiInputTextFlags_EnterReturnsTrue);
            if (enter || ImGui::Button("Create", ImVec2(90.0f, 0.0f))) {
                ProgramSlot s;
                if (orgSlotNameBuf_[0] != '\0') s.name = orgSlotNameBuf_;
                prog.slots.push_back(std::move(s));
                orgSlot_ = static_cast<int>(prog.slots.size()) - 1;
                orgSel_.clear();
                ImGui::CloseCurrentPopup();
                ImGui::EndPopup();
                ImGui::EndDisabled();
                ImGui::End();
                runEdit_ = orgOpen_;
                return;   // container changed; redraw next frame
            }
            ImGui::SameLine();
            if (ImGui::Button("Cancel", ImVec2(90.0f, 0.0f))) ImGui::CloseCurrentPopup();
            ImGui::EndPopup();
        }
        if (orgSlot_ >= 0) {
            ProgramSlot& slot = prog.slots[static_cast<std::size_t>(orgSlot_)];
            ImGui::SameLine();
            // The slot, copied with fresh uids, right after itself; the copy is
            // what you are looking at afterwards.
            if (ImGui::SmallButton("Dup Slot")) {
                ProgramSlot copy = CloneSlot(slot);
                copy.name += " copy";
                prog.slots.insert(prog.slots.begin() + orgSlot_ + 1, std::move(copy));
                ++orgSlot_;
                orgSel_.clear();
                if (orgSlot2_ > orgSlot_ - 1) ++orgSlot2_;   // it still shows the same slot
                ImGui::EndDisabled();
                ImGui::End();
                runEdit_ = orgOpen_;
                return;   // container changed; redraw next frame
            }
            ImGui::SameLine();
            ImGui::BeginDisabled(orgSlot_ == 0);
            if (ImGui::SmallButton("^##slot")) {
                std::swap(prog.slots[static_cast<std::size_t>(orgSlot_ - 1)],
                          prog.slots[static_cast<std::size_t>(orgSlot_)]);
                if (orgSlot2_ == orgSlot_)          --orgSlot2_;   // the 2nd panel follows
                else if (orgSlot2_ == orgSlot_ - 1) ++orgSlot2_;   // ... or the slot it showed
                --orgSlot_;
            }
            ImGui::EndDisabled();
            ImGui::SameLine();
            ImGui::BeginDisabled(orgSlot_ + 1 >= static_cast<int>(prog.slots.size()));
            if (ImGui::SmallButton("v##slot")) {
                std::swap(prog.slots[static_cast<std::size_t>(orgSlot_)],
                          prog.slots[static_cast<std::size_t>(orgSlot_ + 1)]);
                if (orgSlot2_ == orgSlot_)          ++orgSlot2_;
                else if (orgSlot2_ == orgSlot_ + 1) --orgSlot2_;
                ++orgSlot_;
            }
            ImGui::EndDisabled();
            ImGui::SameLine();
            if (ImGui::SmallButton("x Slot")) {
                // The deck filters on ITS OWN slot index. Erasing a slot ABOVE
                // it shifts every index below down by one, so the deck would
                // quietly start showing a different station than it names —
                // follow the erase rather than merely clamping after it.
                int deck = deckSlot_;
                if (orgSlot_ < deck) --deck;
                if (orgSlot2_ == orgSlot_)     { orgSlot2_ = -1; orgSel2_.clear(); }
                else if (orgSlot2_ > orgSlot_) --orgSlot2_;
                prog.slots.erase(prog.slots.begin() + orgSlot_);
                orgSel_.clear();
                SelectDeckSlot(deck);
                ImGui::EndDisabled();
                ImGui::End();
                runEdit_ = orgOpen_;
                return;   // container changed; redraw next frame
            }
        }
        ImGui::SameLine();
        ImGui::Checkbox("2nd panel", &orgTwo_);
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Show a second slot beside the first, to move or copy nodes across");
    }
    ImGui::Separator();

    if (orgSlot_ < 0) {
        ImGui::TextDisabled("This program has no slots yet - press + Slot.");
        ImGui::EndDisabled();
        ImGui::End();
        runEdit_ = orgOpen_;
        return;
    }
    ProgramSlot& slot = prog.slots[static_cast<std::size_t>(orgSlot_)];
    // The 2nd panel's slot: any slot of this program, the 1st's by default.
    // Clamped, never re-pointed, so it keeps showing what the operator chose.
    {
        const int nSlots = static_cast<int>(prog.slots.size());
        if (orgSlot2_ < 0 || orgSlot2_ >= nSlots) { orgSlot2_ = orgSlot_; orgSel2_.clear(); }
    }
    ProgramSlot& slot2 = prog.slots[static_cast<std::size_t>(orgSlot2_)];
    ProgramSlot* other1 = orgTwo_ ? &slot2 : nullptr;   // what a drag INTO panel 1 may come from

    // ---- Columns: the library | 1st slot | aisle | 2nd slot ----
    const float footH = ImGui::GetFrameHeightWithSpacing() * 2.0f;
    const float bodyH = ImGui::GetContentRegionAvail().y - footH;
    // The library column is dragged and persisted like every other splitter in
    // the app. Clamped against the CURRENT width each frame, so shrinking the
    // window can never leave the trees — the side you are actually editing —
    // narrower than they can use.
    constexpr float kOrgSplitW = 6.0f;
    constexpr float kOrgMinLib = 140.0f;
    constexpr float kOrgMinTree = 260.0f;
    constexpr float kOrgAisleW = 84.0f;
    {
        const float trees = orgTwo_ ? kOrgMinTree * 2.0f + kOrgAisleW : kOrgMinTree;
        const float room  = ImGui::GetContentRegionAvail().x - kOrgSplitW - trees;
        const float hi    = room < kOrgMinLib ? kOrgMinLib : room;
        if (orgLibW_ < kOrgMinLib) orgLibW_ = kOrgMinLib;
        if (orgLibW_ > hi)         orgLibW_ = hi;
    }
    const float libW = orgLibW_;

    ImGui::BeginChild("orgLib", ImVec2(libW, bodyH), ImGuiChildFlags_Borders);
    ImGui::TextDisabled("Recipes");
    // Where "add" and a double-click land. With one panel there is no choice
    // to make, so the control only appears with two.
    if (orgTwo_) {
        ImGui::SameLine();
        ImGui::TextDisabled("add to:");
        ImGui::SameLine();
        if (ImGui::RadioButton("1st", orgAddPanel_ == 0)) orgAddPanel_ = 0;
        ImGui::SameLine();
        if (ImGui::RadioButton("2nd", orgAddPanel_ == 1)) orgAddPanel_ = 1;
    } else {
        orgAddPanel_ = 0;
    }
    const char* addTo = (orgTwo_ && orgAddPanel_ == 1) ? "2nd" : "1st";
    ImGui::SetNextItemWidth(-1.0f);
    ImGui::InputTextWithHint("##filt", "filter", orgFilter_, sizeof(orgFilter_));
    ImGui::Separator();
    // The row is sized to STOP where the add button starts, so a long recipe
    // name is clipped by the Selectable rather than drawn across the button —
    // the button was previously pinned at a fixed offset and the two overlapped.
    // The hover tooltip is what gets the clipped name back.
    const float addW = ImGui::CalcTextSize("add").x + ImGui::GetStyle().FramePadding.x * 2.0f;
    for (const Recipe& r : session_.Recipes()) {
        // Hidden means hidden from the pickers; a program that already names
        // the recipe still runs it (RecipeByUid above never looks at the flag).
        // The text filter matches a tag too, so "left" finds what was filed as
        // left and not only what was named that way.
        if (r.hidden) continue;
        if (!Contains(r.name, orgFilter_)) {
            bool tagHit = false;
            for (const std::string& t : r.tags)
                if (Contains(t, orgFilter_)) { tagHit = true; break; }
            if (!tagHit) continue;
        }
        ImGui::PushID(r.uid.c_str());
        const float nameW = ImGui::GetContentRegionAvail().x - addW -
                            ImGui::GetStyle().ItemSpacing.x;
        // Double-click adds at the selection; drag drops it exactly where the
        // operator wants it. Both exist because the first is faster for
        // building a list and the second is the only way to aim into a folder.
        if (ImGui::Selectable(r.name.c_str(), false, ImGuiSelectableFlags_AllowDoubleClick,
                              ImVec2(nameW > 0.0f ? nameW : 0.0f, 0.0f))) {
            if (ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) OrgAddRecipe(r.uid);
        }
        const bool nameHovered = ImGui::IsItemHovered();
        if (ImGui::BeginDragDropSource()) {
            ImGui::SetDragDropPayload(kPayloadRecipe, r.uid.c_str(), r.uid.size());
            ImGui::TextUnformatted(r.name.c_str());
            ImGui::EndDragDropSource();
        }
        if (nameHovered) ImGui::SetTooltip("%s\ndouble-click to add to the %s panel, or drag",
                                   r.name.c_str(), addTo);
        ImGui::SameLine();
        if (ImGui::SmallButton("add")) OrgAddRecipe(r.uid);
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("Add to the %s panel's selection", addTo);
        ImGui::PopID();
    }
    if (session_.Recipes().empty())
        ImGui::TextWrapped("No recipes in this product yet.");
    ImGui::EndChild();

    // Drag to re-proportion; persists on the drag END, like every other
    // splitter in the app. Deliberately OUTSIDE the busy gate: resizing is
    // layout, not editing, and a run is exactly when somebody wants to widen
    // the tree to read it.
    ImGui::EndDisabled();
    ImGui::SameLine(0.0f, 0.0f);
    if (Splitter("##orgsplit", true, kOrgSplitW, bodyH)) {
        orgLibW_ += ImGui::GetIO().MouseDelta.x;
        orgSplitting_ = true;
    } else if (orgSplitting_ && !ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
        orgSplitting_ = false;
        SaveUiSettings();
    }
    ImGui::SameLine(0.0f, 0.0f);
    ImGui::BeginDisabled(busy);

    // The two trees share what is left equally; the aisle sits between them.
    const float treesW = ImGui::GetContentRegionAvail().x;
    const float treeW  = orgTwo_ ? (treesW - kOrgAisleW) * 0.5f : 0.0f;

    ImGui::BeginChild("orgTree", ImVec2(treeW, bodyH), ImGuiChildFlags_Borders);
    {
        const int nSlots = static_cast<int>(prog.slots.size());
        char prev[64];
        std::snprintf(prev, sizeof(prev), "%d  %s", orgSlot_ + 1, slot.name.c_str());
        ImGui::TextDisabled("1st:");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(200.0f);
        if (ImGui::BeginCombo("##slot1", prev)) {
            for (int i = 0; i < nSlots; ++i) {
                ImGui::PushID(i);
                char lbl[64];
                std::snprintf(lbl, sizeof(lbl), "%d  %s", i + 1, prog.slots[i].name.c_str());
                if (ImGui::Selectable(lbl, orgSlot_ == i)) { orgSlot_ = i; orgSel_.clear(); }
                ImGui::PopID();
            }
            ImGui::EndCombo();
        }
    }
    // The combo may have re-pointed orgSlot_ this frame; draw what it points
    // at NOW. (The foot and the aisle below use `slot` and `slot2` taken
    // before the combos: one frame behind at worst, after a pick.)
    ProgramSlot& s1 = prog.slots[static_cast<std::size_t>(orgSlot_)];
    DrawOrgPanel(s1, other1, 0);
    ImGui::EndChild();

    if (orgTwo_) {
        // ---- The aisle: the selection of one panel, moved or copied to the
        // other's selection. Move keeps the node's uid (it is the same node,
        // elsewhere); Copy lands a clone with fresh ones. Both land by the same
        // rule a paste does. The same slot on both sides is allowed - it is
        // then just a second view, and Move is an in-slot move.
        ImGui::SameLine();
        ImGui::BeginChild("orgAisle", ImVec2(kOrgAisleW, bodyH), ImGuiChildFlags_None,
                          ImGuiWindowFlags_NoScrollbar);
        const auto across = [&](ProgramSlot& from, int fromPanel, ProgramSlot& to, int toPanel,
                                bool copy) {
            std::string& fsel = OrgSel(fromPanel);
            const ProgramNode* n = FindNode(from.items, fsel);
            if (!n) return;
            if (copy) {
                OrgPlace(to, toPanel, CloneNode(*n));
                return;
            }
            // A move onto itself or into its own belly would extract the
            // destination along with the node.
            const std::string& tsel = OrgSel(toPanel);
            if (&from == &to && !tsel.empty() &&
                (tsel == fsel || (n->IsFolder() && NodeContains(*n, tsel))))
                return;
            ProgramNode moved;
            if (!ExtractNode(from.items, fsel, &moved)) return;
            ForgetCycleState(from, moved.uid);
            fsel.clear();
            OrgPlace(to, toPanel, std::move(moved));
        };
        const bool sel1 = FindNode(slot.items, orgSel_) != nullptr;
        const bool sel2 = FindNode(slot2.items, orgSel2_) != nullptr;
        const ImVec2 bw(-1.0f, 0.0f);
        ImGui::Dummy(ImVec2(0.0f, ImGui::GetFrameHeight() * 2.0f));
        ImGui::BeginDisabled(!sel1);
        if (ImGui::Button("Copy >", bw)) across(slot, 0, slot2, 1, true);
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("Copy the 1st panel's selection into the 2nd");
        if (ImGui::Button("Move >", bw)) across(slot, 0, slot2, 1, false);
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("Move the 1st panel's selection into the 2nd");
        ImGui::EndDisabled();
        ImGui::Dummy(ImVec2(0.0f, ImGui::GetFrameHeight()));
        ImGui::BeginDisabled(!sel2);
        if (ImGui::Button("< Copy", bw)) across(slot2, 1, slot, 0, true);
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("Copy the 2nd panel's selection into the 1st");
        if (ImGui::Button("< Move", bw)) across(slot2, 1, slot, 0, false);
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("Move the 2nd panel's selection into the 1st");
        ImGui::EndDisabled();
        ImGui::EndChild();

        ImGui::SameLine();
        ImGui::BeginChild("orgTree2", ImVec2(0.0f, bodyH), ImGuiChildFlags_Borders);
        {
            const int nSlots = static_cast<int>(prog.slots.size());
            char prev[64];
            std::snprintf(prev, sizeof(prev), "%d  %s", orgSlot2_ + 1, slot2.name.c_str());
            ImGui::TextDisabled("2nd:");
            ImGui::SameLine();
            ImGui::SetNextItemWidth(200.0f);
            if (ImGui::BeginCombo("##slot2", prev)) {
                for (int i = 0; i < nSlots; ++i) {
                    ImGui::PushID(i);
                    char lbl[64];
                    std::snprintf(lbl, sizeof(lbl), "%d  %s", i + 1, prog.slots[i].name.c_str());
                    if (ImGui::Selectable(lbl, orgSlot2_ == i)) { orgSlot2_ = i; orgSel2_.clear(); }
                    ImGui::PopID();
                }
                ImGui::EndCombo();
            }
            if (orgSlot2_ == orgSlot_) {
                ImGui::SameLine();
                ImGui::TextDisabled("(same slot as the 1st)");
            }
        }
        // The combo may have re-pointed orgSlot2_ this frame; draw what it
        // points at NOW, not the reference taken above.
        ProgramSlot& s2 = prog.slots[static_cast<std::size_t>(orgSlot2_)];
        DrawOrgPanel(s2, &slot, 1);
        ImGui::EndChild();
    }

    ImGui::EndDisabled();

    // ---- Foot: what the 1st slot costs (Save and Close are on the top row) ----
    ImGui::Separator();
    const long long runs = ExpandedCount(slot.items);
    ImGui::Text("%d item(s) in slot %d, expanding to %lld recipe run(s).",
                CountRecipeNodes(slot.items), orgSlot_ + 1, runs);
    ImGui::SameLine();
    ImGui::TextDisabled("Edits live in memory until Save.");

    ImGui::End();
    runEdit_ = orgOpen_;
}

} // namespace firedog
