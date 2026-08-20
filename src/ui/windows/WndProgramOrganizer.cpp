// WndProgramOrganizer.cpp : the Program Organizer — recipe library | program
// tree, the ONE surface that edits a program. Definitions of App members —
// see App.h. Design: docs/program-organizer-design.md.
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
// InsertAfter / InsertInto / MoveNode* / Indent / Outdent) rather than being
// hand-rolled here: a folder is a subtree, and "move this row" is only ever
// correct if the row's children come with it.
#include "App.h"

#include "imgui.h"

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
} // namespace

// ---- Insert helpers --------------------------------------------------------

// Library -> tree. Lands INSIDE the selected folder, else after the selected
// row, else at the end of the slot — i.e. where the eye last was, which is what
// makes clicking down a list of recipes build the deck in order.
void App::OrgAddRecipe(const std::string& recipeUid)
{
    if (progSel_ < 0 || progSel_ >= static_cast<int>(session_.Programs().size())) return;
    Program& prog = session_.Programs()[progSel_];
    if (orgSlot_ < 0 || orgSlot_ >= static_cast<int>(prog.slots.size())) return;
    ProgramSlot& slot = prog.slots[static_cast<std::size_t>(orgSlot_)];

    ProgramNode n = MakeRecipeNode(recipeUid);
    const std::string uid = n.uid;
    const ProgramNode* sel = FindNode(slot.items, orgSel_);
    if (sel && sel->IsFolder())      InsertInto(slot.items, orgSel_, std::move(n));
    else if (sel)                    InsertAfter(slot.items, orgSel_, std::move(n));
    else                             slot.items.push_back(std::move(n));
    orgSel_ = uid;   // the new row is the next insertion point
}

void App::OrgAddFolder()
{
    if (progSel_ < 0 || progSel_ >= static_cast<int>(session_.Programs().size())) return;
    Program& prog = session_.Programs()[progSel_];
    if (orgSlot_ < 0 || orgSlot_ >= static_cast<int>(prog.slots.size())) return;
    ProgramSlot& slot = prog.slots[static_cast<std::size_t>(orgSlot_)];

    ProgramNode n = MakeFolderNode();
    const std::string uid = n.uid;
    const ProgramNode* sel = FindNode(slot.items, orgSel_);
    // A new folder lands BESIDE the selection even when a folder is selected —
    // nesting is a deliberate act (drag, or the > button), not what you get for
    // pressing "+ Folder" twice.
    if (sel) InsertAfter(slot.items, orgSel_, std::move(n));
    else     slot.items.push_back(std::move(n));
    orgSel_ = uid;
}

// One row's drop target. `intoFolder` decides the two meanings a drop can
// have: onto a FOLDER row it goes inside, onto a recipe row it lands after.
//
// It only RECORDS the landing. Acting here would insert into a vector that
// DrawOrgTree is in the middle of walking by reference — the reallocation
// leaves the loop holding a dangling node. ApplyOrgDrop runs it once the tree
// is drawn.
void App::DrawOrgNodeDropTarget(const std::string& anchorUid, bool intoFolder)
{
    if (!ImGui::BeginDragDropTarget()) return;
    if (const ImGuiPayload* pl = ImGui::AcceptDragDropPayload(kPayloadRecipe)) {
        orgDropRecipe_  = std::string(static_cast<const char*>(pl->Data),
                                      static_cast<std::size_t>(pl->DataSize));
        orgDropNode_.clear();
        orgDropAnchor_  = anchorUid;
        orgDropInto_    = intoFolder;
        orgDropPending_ = true;
    }
    if (const ImGuiPayload* pl = ImGui::AcceptDragDropPayload(kPayloadNode)) {
        orgDropNode_    = std::string(static_cast<const char*>(pl->Data),
                                      static_cast<std::size_t>(pl->DataSize));
        orgDropRecipe_.clear();
        orgDropAnchor_  = anchorUid;
        orgDropInto_    = intoFolder;
        orgDropPending_ = true;
    }
    ImGui::EndDragDropTarget();
}

// The recorded drop, applied after the tree draw. An empty anchor means the
// slot's top level, at the end.
void App::ApplyOrgDrop(ProgramSlot& slot)
{
    if (!orgDropPending_) return;
    orgDropPending_ = false;

    if (!orgDropRecipe_.empty()) {
        ProgramNode n = MakeRecipeNode(orgDropRecipe_);
        orgSel_ = n.uid;
        if (orgDropAnchor_.empty())   slot.items.push_back(std::move(n));
        else if (orgDropInto_)        InsertInto(slot.items, orgDropAnchor_, std::move(n));
        else                          InsertAfter(slot.items, orgDropAnchor_, std::move(n));
        orgDropRecipe_.clear();
        return;
    }
    if (orgDropNode_.empty()) return;

    // Refuse a folder dropped onto itself or into its own belly: the extract
    // would take the destination with it and the subtree would vanish.
    const ProgramNode* moving = FindNode(slot.items, orgDropNode_);
    if (!moving) { orgDropNode_.clear(); return; }
    if (!orgDropAnchor_.empty() &&
        (orgDropAnchor_ == orgDropNode_ ||
         (moving->IsFolder() && NodeContains(*moving, orgDropAnchor_)))) {
        orgDropNode_.clear();
        return;
    }
    ProgramNode n;
    if (ExtractNode(slot.items, orgDropNode_, &n)) {
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
        orgSel_ = uid;
    }
    orgDropNode_.clear();
}

// ---- The tree --------------------------------------------------------------

void App::DrawOrgTree(ProgramSlot& slot, std::vector<ProgramNode>& items, bool topLevel)
{
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
            if (orgSel_ == n.uid) flags |= ImGuiTreeNodeFlags_Selected;
            const long long runs = static_cast<long long>(n.loop < 1 ? 1 : n.loop) *
                                   ExpandedCount(n.children);
            const bool open = ImGui::TreeNodeEx("##f", flags, "[+] %s   x%d   (%lld runs)",
                                                n.name.c_str(), n.loop, runs);
            const bool fRClick = ImGui::IsItemClicked(ImGuiMouseButton_Right);
            if (ImGui::IsItemClicked() && !ImGui::IsItemToggledOpen()) orgSel_ = n.uid;
            if (ImGui::BeginDragDropSource()) {
                ImGui::SetDragDropPayload(kPayloadNode, n.uid.c_str(), n.uid.size());
                ImGui::Text("%s (folder)", n.name.c_str());
                ImGui::EndDragDropSource();
            }
            DrawOrgNodeDropTarget(n.uid, /*intoFolder=*/true);
            cycChip(n);
            if (open) {
                DrawOrgTree(slot, n.children, /*topLevel=*/false);
                if (n.children.empty()) {
                    ImGui::TextDisabled("(empty - drop recipes here)");
                    // The empty folder's own body is a target too, or a folder
                    // you just made is the one thing you cannot fill by drag.
                    DrawOrgNodeDropTarget(n.uid, /*intoFolder=*/true);
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
        if (ImGui::Selectable(row, orgSel_ == n.uid)) orgSel_ = n.uid;
        const bool rRClick = ImGui::IsItemClicked(ImGuiMouseButton_Right);
        if (missing) ImGui::PopStyleColor();
        if (ImGui::BeginDragDropSource()) {
            ImGui::SetDragDropPayload(kPayloadNode, n.uid.c_str(), n.uid.size());
            ImGui::TextUnformatted(missing ? n.recipeUid.c_str() : r->name.c_str());
            ImGui::EndDragDropSource();
        }
        DrawOrgNodeDropTarget(n.uid, /*intoFolder=*/false);
        cycChip(n);
        cycMenu(n, rRClick);
        ImGui::PopID();
    }
}

// ---- The window ------------------------------------------------------------

void App::DrawProgramOrganizer()
{
    const ImGuiViewport* mv = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(ImVec2(mv->Pos.x + 120.0f, mv->Pos.y + 100.0f),
                            ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(720.0f, 520.0f), ImGuiCond_FirstUseEver);
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

    // Not const: New and Delete both re-point progSel_ further down this same
    // frame, and reading a stale `true` afterwards indexes Programs()[-1].
    bool haveProg = progSel_ >= 0 &&
                    progSel_ < static_cast<int>(session_.Programs().size());

    // ---- Program row ----
    if (IconSmallButton(UiIcon("New"), "New")) {
        Program p;
        p.uid = GenerateUid();   // the traceability identity, minted once
        session_.Programs().push_back(std::move(p));
        SelectDeckProgram(static_cast<int>(session_.Programs().size()) - 1);
        orgSlot_ = deckSlot_;
        orgSel_.clear();
    }
    ImGui::SameLine();
    ImGui::BeginDisabled(!haveProg);
    // The confirm is a plain popup, not a MODAL: this window can be open on a
    // second monitor while the Run page is up, and a modal would block the
    // prompt / manual_adjust windows an engine worker can be waiting on.
    if (ImGui::SmallButton("Delete")) orgDelConfirm_ = true;
    ImGui::SameLine();
    if (haveProg) {
        Program& prog = session_.Programs()[progSel_];
        char nbuf[64];
        std::snprintf(nbuf, sizeof(nbuf), "%s", prog.name.c_str());
        ImGui::SetNextItemWidth(220.0f);
        if (ImGui::InputText("program", nbuf, sizeof(nbuf))) prog.name = nbuf;
        ImGui::SameLine();
        ImGui::TextDisabled("uid: %s", prog.uid.c_str());
    } else {
        ImGui::TextDisabled("no program loaded - New, or Load on the Run page");
    }
    ImGui::EndDisabled();
    if (orgDelConfirm_) { ImGui::OpenPopup("orgDel"); orgDelConfirm_ = false; }
    if (ImGui::BeginPopup("orgDel")) {
        if (haveProg)
            ImGui::Text("Delete program '%s'?", session_.Programs()[progSel_].name.c_str());
        if (ImGui::Button("Delete", ImVec2(100.0f, 0.0f)) && haveProg) {
            session_.Programs().erase(session_.Programs().begin() + progSel_);
            SelectDeckProgram(session_.Programs().empty() ? -1 : 0);
            orgSlot_ = -1;
            orgSel_.clear();
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

    // ---- Slot row ----
    // The organizer's slot is its OWN: reorganising slot 3 must not drag the
    // operator's deck off the station they are standing at.
    {
        const int nSlots = static_cast<int>(prog.slots.size());
        if (nSlots == 0)              orgSlot_ = -1;
        else if (orgSlot_ < 0)        orgSlot_ = (deckSlot_ >= 0 && deckSlot_ < nSlots) ? deckSlot_ : 0;
        else if (orgSlot_ >= nSlots)  orgSlot_ = nSlots - 1;
        char prev[64];
        if (orgSlot_ >= 0)
            std::snprintf(prev, sizeof(prev), "%d  %s", orgSlot_ + 1,
                          prog.slots[static_cast<std::size_t>(orgSlot_)].name.c_str());
        else
            std::snprintf(prev, sizeof(prev), "(no slots)");
        ImGui::SetNextItemWidth(180.0f);
        if (ImGui::BeginCombo("slot", prev)) {
            for (int i = 0; i < nSlots; ++i) {
                ImGui::PushID(i);
                char lbl[64];
                std::snprintf(lbl, sizeof(lbl), "%d  %s", i + 1, prog.slots[i].name.c_str());
                if (ImGui::Selectable(lbl, orgSlot_ == i)) { orgSlot_ = i; orgSel_.clear(); }
                ImGui::PopID();
            }
            ImGui::EndCombo();
        }
        ImGui::SameLine();
        if (ImGui::SmallButton("+ Slot")) {
            prog.slots.push_back({});
            orgSlot_ = static_cast<int>(prog.slots.size()) - 1;
            orgSel_.clear();
        }
        if (orgSlot_ >= 0) {
            ProgramSlot& slot = prog.slots[static_cast<std::size_t>(orgSlot_)];
            ImGui::SameLine();
            char sbuf[48];
            std::snprintf(sbuf, sizeof(sbuf), "%s", slot.name.c_str());
            ImGui::SetNextItemWidth(160.0f);
            if (ImGui::InputText("name", sbuf, sizeof(sbuf))) slot.name = sbuf;
            ImGui::SameLine();
            ImGui::BeginDisabled(orgSlot_ == 0);
            if (ImGui::SmallButton("^##slot")) {
                std::swap(prog.slots[static_cast<std::size_t>(orgSlot_ - 1)],
                          prog.slots[static_cast<std::size_t>(orgSlot_)]);
                --orgSlot_;
            }
            ImGui::EndDisabled();
            ImGui::SameLine();
            ImGui::BeginDisabled(orgSlot_ + 1 >= static_cast<int>(prog.slots.size()));
            if (ImGui::SmallButton("v##slot")) {
                std::swap(prog.slots[static_cast<std::size_t>(orgSlot_)],
                          prog.slots[static_cast<std::size_t>(orgSlot_ + 1)]);
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
                prog.slots.erase(prog.slots.begin() + orgSlot_);
                orgSel_.clear();
                SelectDeckSlot(deck);
                ImGui::EndDisabled();
                ImGui::End();
                runEdit_ = orgOpen_;
                return;   // container changed; redraw next frame
            }
        }
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

    // ---- Two panels: the library | the slot tree ----
    const float footH = ImGui::GetFrameHeightWithSpacing() * 3.0f;
    const float bodyH = ImGui::GetContentRegionAvail().y - footH;
    // The library column is dragged and persisted like every other splitter in
    // the app. Clamped against the CURRENT width each frame, so shrinking the
    // window can never leave the tree — the side you are actually editing —
    // narrower than it can use.
    constexpr float kOrgSplitW = 6.0f;
    constexpr float kOrgMinLib = 140.0f;
    constexpr float kOrgMinTree = 260.0f;
    {
        const float room = ImGui::GetContentRegionAvail().x - kOrgSplitW - kOrgMinTree;
        const float hi   = room < kOrgMinLib ? kOrgMinLib : room;
        if (orgLibW_ < kOrgMinLib) orgLibW_ = kOrgMinLib;
        if (orgLibW_ > hi)         orgLibW_ = hi;
    }
    const float libW = orgLibW_;

    ImGui::BeginChild("orgLib", ImVec2(libW, bodyH), ImGuiChildFlags_Borders);
    ImGui::TextDisabled("Recipes");
    ImGui::SetNextItemWidth(-1.0f);
    ImGui::InputTextWithHint("##filt", "filter", orgFilter_, sizeof(orgFilter_));
    ImGui::Separator();
    // The row is sized to STOP where the add button starts, so a long recipe
    // name is clipped by the Selectable rather than drawn across the button —
    // the button was previously pinned at a fixed offset and the two overlapped.
    // The hover tooltip is what gets the clipped name back.
    const float addW = ImGui::CalcTextSize("add").x + ImGui::GetStyle().FramePadding.x * 2.0f;
    for (const Recipe& r : session_.Recipes()) {
        if (!Contains(r.name, orgFilter_)) continue;
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
        if (nameHovered) ImGui::SetTooltip("%s\ndouble-click to add, or drag",
                                   r.name.c_str());
        ImGui::SameLine();
        if (ImGui::SmallButton("add")) OrgAddRecipe(r.uid);
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

    ImGui::BeginChild("orgTree", ImVec2(0.0f, bodyH), ImGuiChildFlags_Borders);
    // Organize row: everything acts on the SELECTED node, so one set of buttons
    // serves every depth. > / < are indent/outdent — the keyboard-free way into
    // and out of a folder for anyone who would rather not drag.
    if (ImGui::SmallButton("+ Folder")) OrgAddFolder();
    ImGui::SameLine();
    const bool haveSel = FindNode(slot.items, orgSel_) != nullptr;
    ImGui::BeginDisabled(!haveSel);
    if (ImGui::SmallButton("^"))  MoveNodeUp(slot.items, orgSel_);
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("Move up (within its folder)");
    ImGui::SameLine();
    if (ImGui::SmallButton("v"))  MoveNodeDown(slot.items, orgSel_);
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("Move down (within its folder)");
    ImGui::SameLine();
    if (ImGui::SmallButton(">"))  IndentNode(slot.items, orgSel_);
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("Move into the folder above it");
    ImGui::SameLine();
    if (ImGui::SmallButton("<"))  OutdentNode(slot.items, orgSel_);
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("Move out of its folder");
    ImGui::SameLine();
    if (ImGui::SmallButton("Remove")) {
        // A folder goes WITH its children: the tree owns them, and leaving them
        // behind at the parent's level is a silent restructure nobody asked for.
        // A node that no longer exists cannot stay a cycle state. (A node
        // dragged INTO a folder needs no cleanup: DeckCycleRange stops
        // resolving it and the line below says so.)
        if (slot.startNodeUid == orgSel_) slot.startNodeUid.clear();
        if (slot.endNodeUid   == orgSel_) slot.endNodeUid.clear();
        ExtractNode(slot.items, orgSel_, nullptr);
        orgSel_.clear();
    }
    ImGui::EndDisabled();

    // The selected folder's own knobs. A recipe node has nothing to edit — it
    // IS a reference; its content is the Recipe Editor's business.
    if (ProgramNode* sel = FindNode(slot.items, orgSel_)) {
        if (sel->IsFolder()) {
            ImGui::Separator();
            char fbuf[64];
            std::snprintf(fbuf, sizeof(fbuf), "%s", sel->name.c_str());
            ImGui::SetNextItemWidth(180.0f);
            if (ImGui::InputText("folder", fbuf, sizeof(fbuf))) sel->name = fbuf;
            ImGui::SameLine();
            ImGui::SetNextItemWidth(140.0f);
            if (ImGui::InputInt("loop", &sel->loop)) {
                if (sel->loop < 1)        sel->loop = 1;
                if (sel->loop > kMaxLoop) sel->loop = kMaxLoop;
            }
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Passes over this folder's contents.\n"
                                  "The whole loop is expanded into the run queue at Execute.");
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
    DrawOrgTree(slot, slot.items, /*topLevel=*/true);
    // Tail target: the drop that means "at the end, top level". Without it the
    // only way to append by drag is onto the last row, which lands you INSIDE
    // it whenever that row happens to be a folder.
    ImGui::Dummy(ImVec2(-1.0f, ImGui::GetTextLineHeight()));
    DrawOrgNodeDropTarget(std::string(), /*intoFolder=*/false);
    if (slot.items.empty())
        ImGui::TextDisabled("Empty slot - double-click a recipe on the left, or drag one here.");
    ImGui::EndChild();
    // Every drop recorded during the draw lands HERE, with nothing iterating
    // the tree any more.
    ApplyOrgDrop(slot);
    ImGui::EndChild();

    ImGui::EndDisabled();

    // ---- Foot: what this slot costs, and Save ----
    ImGui::Separator();
    const long long runs = ExpandedCount(slot.items);
    ImGui::Text("%d item(s) in this slot, expanding to %lld recipe run(s).",
                CountRecipeNodes(slot.items), runs);
    ImGui::BeginDisabled(busy);
    if (IconButton(UiIcon("Save"), session_.ProgramsDirty() ? "Save*" : "Save",
                   ImVec2(110.0f, 0.0f)))
        session_.SavePrograms();
    ImGui::SameLine();
    if (ImGui::Button("Close", ImVec2(90.0f, 0.0f))) orgOpen_ = false;
    ImGui::EndDisabled();
    ImGui::SameLine();
    ImGui::TextDisabled("Edits live in memory until Save.");

    ImGui::End();
    runEdit_ = orgOpen_;
}

} // namespace firedog
