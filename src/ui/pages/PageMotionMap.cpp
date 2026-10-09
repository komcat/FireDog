// PageMotionMap.cpp : the Mapped Path tab of the Motion page — the node-edge
// graph editor over saved positions. Definitions of App members — see App.h.
//
// Nodes reference a SavedPos by UID (one node per position per map) plus
// editor canvas coordinates; edges are bidirectional and encode which direct
// moves are safe. Structural edits autosave through the session. The Go button
// plans with BuildMapRoute - the same planner the recipe engine's move_path
// uses - and executes through Workflow's path executor, which App::Update
// drives leg by leg via TickPath.
#include "App.h"

#include "imgui.h"

#include "ui/Widgets.h"
#include "command/MotionWaits.h"   // TargetsOf: the direct move commands a position's axes
#include "core/Log.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <map>
#include <string>

namespace firedog {

// Mapped Path tab — node-edge motion maps. Nodes reference saved positions by
// uid (one node per position per map); edges are bidirectional. The canvas is
// a direct-manipulation editor: drag nodes, Edge mode links two clicked nodes,
// right-click adds/deletes. "Go" runs Dijkstra (XYZ distance weights) over the
// edges and executes the node chain through Workflow's path executor.
void App::DrawMotionMapTab(std::size_t devIdx, IMotionController* m)
{
    constexpr float  kNodeR       = 16.0f;   // canvas node radius (px)
    constexpr double kAtNodeTolMM = 0.05;    // "device is at this node" tolerance

    const std::string devName = hardware_.MotionName(devIdx);
    std::vector<MotionMap>& maps = session_.Maps()[devName];

    if (mapSelDev_ != static_cast<int>(devIdx)) {   // device switched — reset state
        mapSelDev_ = static_cast<int>(devIdx);
        mapSel_ = 0;
        mapEdgeMode_ = false;
        mapEdgeFrom_.clear();
        mapDragNode_.clear();
        mapSelEdgeA_.clear(); mapSelEdgeB_.clear();
        mapSelNode_.clear();
        mapLastPath_.clear();
        mapFrom_ = 0;
        mapTo_   = -1;
    }
    if (mapSel_ >= static_cast<int>(maps.size())) mapSel_ = maps.empty() ? -1 : 0;

    const std::vector<SavedPos>& saved = session_.Positions()[devName];
    auto posOf = [&saved](const std::string& uid) -> const SavedPos* {
        for (const SavedPos& p : saved) {
            if (p.uid == uid) return &p;
        }
        return nullptr;
    };

    // ---- Map selector + New / Rename / Delete / Save ----
    ImGui::SetNextItemWidth(200.0f);
    const bool mapValid = mapSel_ >= 0 && mapSel_ < static_cast<int>(maps.size());
    if (ImGui::BeginCombo("##mapsel", mapValid ? maps[mapSel_].name.c_str() : "(no maps)")) {
        for (int i = 0; i < static_cast<int>(maps.size()); ++i) {
            ImGui::PushID(i);
            if (ImGui::Selectable(maps[i].name.c_str(), i == mapSel_)) {
                mapSel_ = i;
                mapEdgeFrom_.clear();
                mapSelEdgeA_.clear(); mapSelEdgeB_.clear();
                mapSelNode_.clear();
                mapLastPath_.clear();
                mapFrom_ = 0; mapTo_ = -1;
            }
            ImGui::PopID();
        }
        ImGui::EndCombo();
    }
    ImGui::SameLine();
    if (ImGui::SmallButton("+ New")) { mapNameBuf_[0] = '\0'; openNewMap_ = true; }
    if (mapValid) {
        ImGui::SameLine();
        if (ImGui::SmallButton("Rename")) {
            std::snprintf(mapNameBuf_, sizeof(mapNameBuf_), "%s", maps[mapSel_].name.c_str());
            openRenameMap_ = true;
        }
        ImGui::SameLine();
        if (ImGui::SmallButton("Delete")) openDeleteMap_ = true;
        ImGui::SameLine();
        if (ImGui::SmallButton("Save")) session_.SaveMaps();   // edits also autosave
    }

    // ---- New / Rename / Delete map dialogs ----
    if (openNewMap_) { ImGui::OpenPopup("New Map"); openNewMap_ = false; }
    ImGui::SetNextWindowPos(ImGui::GetMainViewport()->GetCenter(), ImGuiCond_Appearing,
                            ImVec2(0.5f, 0.5f));
    if (ImGui::BeginPopupModal("New Map", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::SetNextItemWidth(280.0f);
        if (ImGui::IsWindowAppearing()) ImGui::SetKeyboardFocusHere();
        const bool submit = ImGui::InputText("##mapname", mapNameBuf_, sizeof(mapNameBuf_),
                                             ImGuiInputTextFlags_EnterReturnsTrue);
        const bool valid = mapNameBuf_[0] != '\0';
        ImGui::BeginDisabled(!valid);
        if (ImGui::Button("Create") || (submit && valid)) {
            MotionMap mm;
            mm.name = mapNameBuf_;
            mm.uid  = GenerateUid();
            maps.push_back(std::move(mm));
            mapSel_ = static_cast<int>(maps.size()) - 1;
            mapLastPath_.clear();
            session_.SaveMaps();
            FD_INFO("map", "created map '%s' for %s", mapNameBuf_, devName.c_str());
            ImGui::CloseCurrentPopup();
        }
        ImGui::EndDisabled();
        ImGui::SameLine();
        if (ImGui::Button("Cancel")) ImGui::CloseCurrentPopup();
        ImGui::EndPopup();
    }

    if (openRenameMap_) { ImGui::OpenPopup("Rename Map"); openRenameMap_ = false; }
    ImGui::SetNextWindowPos(ImGui::GetMainViewport()->GetCenter(), ImGuiCond_Appearing,
                            ImVec2(0.5f, 0.5f));
    if (ImGui::BeginPopupModal("Rename Map", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        if (!mapValid) {
            ImGui::CloseCurrentPopup();
        } else {
            ImGui::SetNextItemWidth(280.0f);
            if (ImGui::IsWindowAppearing()) ImGui::SetKeyboardFocusHere();
            const bool submit = ImGui::InputText("##maprename", mapNameBuf_, sizeof(mapNameBuf_),
                                                 ImGuiInputTextFlags_EnterReturnsTrue);
            const bool valid = mapNameBuf_[0] != '\0';
            ImGui::BeginDisabled(!valid);
            if (ImGui::Button("OK") || (submit && valid)) {
                maps[mapSel_].name = mapNameBuf_;
                session_.SaveMaps();
                ImGui::CloseCurrentPopup();
            }
            ImGui::EndDisabled();
            ImGui::SameLine();
            if (ImGui::Button("Cancel")) ImGui::CloseCurrentPopup();
        }
        ImGui::EndPopup();
    }

    if (openDeleteMap_) { ImGui::OpenPopup("Delete Map"); openDeleteMap_ = false; }
    ImGui::SetNextWindowPos(ImGui::GetMainViewport()->GetCenter(), ImGuiCond_Appearing,
                            ImVec2(0.5f, 0.5f));
    if (ImGui::BeginPopupModal("Delete Map", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        if (!mapValid) {
            ImGui::CloseCurrentPopup();
        } else {
            ImGui::Text("Delete map '%s' on %s?", maps[mapSel_].name.c_str(), devName.c_str());
            ImGui::TextDisabled("This cannot be undone.");
            ImGui::Spacing();
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.72f, 0.20f, 0.20f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.85f, 0.25f, 0.25f, 1.0f));
            if (ImGui::Button("Delete")) {
                FD_INFO("map", "deleted map '%s' (%s)", maps[mapSel_].name.c_str(), devName.c_str());
                maps.erase(maps.begin() + mapSel_);
                mapSel_ = maps.empty() ? -1 : 0;
                mapLastPath_.clear();
                session_.SaveMaps();
                ImGui::CloseCurrentPopup();
            }
            ImGui::PopStyleColor(2);
            ImGui::SameLine();
            if (ImGui::Button("Cancel")) ImGui::CloseCurrentPopup();
        }
        ImGui::EndPopup();
    }

    // Re-checked, not `mapValid`: the Delete dialog above may have just erased
    // the selected map (the last one leaves mapSel_ == -1).
    if (mapSel_ < 0 || mapSel_ >= static_cast<int>(maps.size())) {
        ImGui::Spacing();
        ImGui::TextDisabled("No maps for this device yet - create one with \"+ New\".");
        return;
    }
    MotionMap& map = maps[mapSel_];

    // ---- Edit toolbar ----
    if (ImGui::SmallButton("+ Add node")) ImGui::OpenPopup("mapaddnode");
    ImGui::SameLine();
    if (ImGui::Checkbox("Edge mode", &mapEdgeMode_) && !mapEdgeMode_) mapEdgeFrom_.clear();
    const bool edgeSelected = !mapSelEdgeA_.empty() && map.HasEdge(mapSelEdgeA_, mapSelEdgeB_);
    if (edgeSelected) {
        ImGui::SameLine();
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.72f, 0.20f, 0.20f, 1.0f));
        if (ImGui::SmallButton("Delete edge (Del)")) {
            map.RemoveEdge(mapSelEdgeA_, mapSelEdgeB_);
            mapSelEdgeA_.clear(); mapSelEdgeB_.clear();
            mapLastPath_.clear();
            session_.SaveMaps();
        }
        ImGui::PopStyleColor();
    }
    ImGui::SameLine();
    ImGui::TextDisabled("%zu node(s), %zu edge(s)  |  drag nodes; click an edge to select it; "
                        "right-click to add/delete",
                        map.nodes.size(), map.edges.size());
    if (ImGui::BeginPopup("mapaddnode")) {
        ImGui::TextDisabled("Add a saved position as a node:");
        ImGui::Separator();
        bool anyLeft = false;
        for (size_t i = 0; i < saved.size(); ++i) {
            if (map.FindNode(saved[i].uid)) continue;   // one node per position
            anyLeft = true;
            ImGui::PushID(static_cast<int>(i));
            if (ImGui::Selectable(saved[i].name)) {
                MotionMapNode n;
                n.uid = saved[i].uid;
                n.x   = 50.0f + static_cast<float>(map.nodes.size() % 5) * 120.0f;
                n.y   = 50.0f + static_cast<float>(map.nodes.size() / 5) * 90.0f;
                map.nodes.push_back(std::move(n));
                session_.SaveMaps();
            }
            ImGui::PopID();
        }
        if (!anyLeft) ImGui::TextDisabled("(every saved position is already a node)");
        ImGui::EndPopup();
    }

    // ---- Canvas ----
    const float goH = ImGui::GetFrameHeightWithSpacing() * 2.5f;
    ImVec2 avail = ImGui::GetContentRegionAvail();
    const ImVec2 cs(avail.x > 50.0f ? avail.x : 50.0f,
                    (avail.y - goH) > 140.0f ? (avail.y - goH) : 140.0f);
    const ImVec2 p0 = ImGui::GetCursorScreenPos();
    const ImVec2 p1(p0.x + cs.x, p0.y + cs.y);
    ImDrawList* dl = ImGui::GetWindowDrawList();
    dl->AddRectFilled(p0, p1, IM_COL32(20, 22, 27, 255), 4.0f);
    dl->AddRect(p0, p1, IM_COL32(70, 74, 82, 255), 4.0f);
    // The side panel (below) sits ON the canvas, so the canvas must let later
    // items win the hover, and its own hit-test must ignore the panel's rect —
    // or a click on "Path here" first lands on the canvas as "click on empty
    // space", which clears the very selection the panel is showing.
    const ImVec2 mp = ImGui::GetIO().MousePos;
    const bool   selNodeValid = !mapSelNode_.empty() && map.FindNode(mapSelNode_) != nullptr;
    const ImVec2 panelSize(230.0f, ImGui::GetFrameHeightWithSpacing() * 4.0f +
                                       ImGui::GetStyle().WindowPadding.y * 2.0f);
    const ImVec2 panelPos(p1.x - panelSize.x - 8.0f, p0.y + 8.0f);
    const bool   overPanel = selNodeValid &&
        mp.x >= panelPos.x && mp.x <= panelPos.x + panelSize.x &&
        mp.y >= panelPos.y && mp.y <= panelPos.y + panelSize.y;
    ImGui::SetNextItemAllowOverlap();
    ImGui::InvisibleButton("mapcanvas", cs,
                           ImGuiButtonFlags_MouseButtonLeft | ImGuiButtonFlags_MouseButtonRight);
    const bool   canvasHovered = ImGui::IsItemHovered() && !overPanel;
    const ImVec2 afterCanvas   = ImGui::GetCursorScreenPos();
    const ImVec2 lm(mp.x - p0.x, mp.y - p0.y);   // mouse in canvas coords

    auto nodeCenter = [&p0](const MotionMapNode& n) { return ImVec2(p0.x + n.x, p0.y + n.y); };
    auto hitNode = [&]() -> std::string {
        for (auto it = map.nodes.rbegin(); it != map.nodes.rend(); ++it) {
            const float dx = lm.x - it->x, dy = lm.y - it->y;
            if (dx * dx + dy * dy <= kNodeR * kNodeR) return it->uid;
        }
        return {};
    };
    auto hitEdge = [&]() -> int {   // point-to-segment distance < 6 px
        for (size_t i = 0; i < map.edges.size(); ++i) {
            const MotionMapNode* a = map.FindNode(map.edges[i].a);
            const MotionMapNode* b = map.FindNode(map.edges[i].b);
            if (!a || !b) continue;
            const float abx = b->x - a->x, aby = b->y - a->y;
            const float len2 = abx * abx + aby * aby;
            float t = len2 > 0.0f ? ((lm.x - a->x) * abx + (lm.y - a->y) * aby) / len2 : 0.0f;
            t = t < 0.0f ? 0.0f : (t > 1.0f ? 1.0f : t);
            const float dx = lm.x - (a->x + t * abx), dy = lm.y - (a->y + t * aby);
            if (dx * dx + dy * dy < 36.0f) return static_cast<int>(i);
        }
        return -1;
    };

    // Which node the device currently sits at (all saved axes within tol).
    std::string currentUid;
    if (m->IsConnected()) {
        const std::map<std::string, double> pos = m->Positions();
        for (const MotionMapNode& n : map.nodes) {
            const SavedPos* p = posOf(n.uid);
            if (!p || p->vals.empty()) continue;
            bool at = true;
            for (const auto& [axis, v] : p->vals) {
                const auto it = pos.find(axis);
                if (it == pos.end() || std::fabs(it->second - v) > kAtNodeTolMM) { at = false; break; }
            }
            if (at) { currentUid = n.uid; break; }
        }
    }

    auto edgeOnPath = [&](const std::string& a, const std::string& b) {
        for (size_t i = 0; i + 1 < mapLastPath_.size(); ++i) {
            if ((mapLastPath_[i] == a && mapLastPath_[i + 1] == b) ||
                (mapLastPath_[i] == b && mapLastPath_[i + 1] == a)) return true;
        }
        return false;
    };

    dl->PushClipRect(p0, p1, true);
    for (const MotionMapEdge& e : map.edges) {
        const MotionMapNode* a = map.FindNode(e.a);
        const MotionMapNode* b = map.FindNode(e.b);
        if (!a || !b) continue;
        const bool selected = (e.a == mapSelEdgeA_ && e.b == mapSelEdgeB_) ||
                              (e.a == mapSelEdgeB_ && e.b == mapSelEdgeA_);
        const bool onPath   = edgeOnPath(e.a, e.b);
        ImU32 col   = IM_COL32(130, 135, 145, 255);
        float width = 2.5f;
        if (onPath)   { col = IM_COL32(80, 220, 120, 255);  width = 4.0f; }
        if (selected) { col = IM_COL32(240, 160, 60, 255);  width = 4.0f; }
        dl->AddLine(nodeCenter(*a), nodeCenter(*b), col, width);
    }
    if (mapEdgeMode_ && !mapEdgeFrom_.empty()) {   // preview: picked node -> mouse
        if (const MotionMapNode* a = map.FindNode(mapEdgeFrom_))
            dl->AddLine(nodeCenter(*a), mp, IM_COL32(240, 200, 80, 180), 1.5f);
    }
    for (const MotionMapNode& n : map.nodes) {
        const SavedPos* p = posOf(n.uid);
        const ImVec2 c = nodeCenter(n);
        dl->AddCircleFilled(c, kNodeR, p ? IM_COL32(52, 120, 200, 255)
                                         : IM_COL32(150, 60, 60, 255));
        if (n.uid == currentUid)
            dl->AddCircle(c, kNodeR + 3.0f, IM_COL32(80, 220, 120, 255), 0, 3.0f);
        if (!mapEdgeMode_ && n.uid == mapSelNode_)   // selected: white ring outside
            dl->AddCircle(c, kNodeR + 6.0f, IM_COL32(235, 235, 240, 255), 0, 2.0f);
        if (mapEdgeMode_ && n.uid == mapEdgeFrom_)
            dl->AddCircle(c, kNodeR + 3.0f, IM_COL32(240, 200, 80, 255), 0, 3.0f);
        const char*  label = p ? p->name : "(missing)";
        const ImVec2 ts    = ImGui::CalcTextSize(label);
        dl->AddText(ImVec2(c.x - ts.x * 0.5f, c.y + kNodeR + 3.0f),
                    IM_COL32(225, 228, 235, 255), label);
    }
    dl->PopClipRect();

    // ---- Canvas interactions ----
    const std::string hovered = canvasHovered ? hitNode() : std::string();
    if (canvasHovered && ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
        if (mapEdgeMode_) {
            if (hovered.empty()) {
                mapEdgeFrom_.clear();
            } else if (mapEdgeFrom_.empty()) {
                mapEdgeFrom_ = hovered;
            } else if (hovered != mapEdgeFrom_) {
                if (!map.HasEdge(mapEdgeFrom_, hovered)) {
                    map.edges.push_back({mapEdgeFrom_, hovered});
                    session_.SaveMaps();
                }
                mapEdgeFrom_ = hovered;   // chain: keep linking from here
            }
        } else if (!hovered.empty()) {
            // A press both selects and starts a drag: a click that never moves
            // is a selection, a click that moves is a reposition.
            mapDragNode_ = hovered;
            mapSelNode_  = hovered;
            mapSelEdgeA_.clear(); mapSelEdgeB_.clear();
        } else {
            const int ei = hitEdge();   // click an edge line to select it
            if (ei >= 0) {
                mapSelEdgeA_ = map.edges[ei].a;
                mapSelEdgeB_ = map.edges[ei].b;
            } else {
                mapSelEdgeA_.clear(); mapSelEdgeB_.clear();
            }
            mapSelNode_.clear();   // empty space or an edge: the node panel goes
        }
    }
    if (edgeSelected && !ImGui::GetIO().WantTextInput &&
        ImGui::IsKeyPressed(ImGuiKey_Delete)) {
        map.RemoveEdge(mapSelEdgeA_, mapSelEdgeB_);
        mapSelEdgeA_.clear(); mapSelEdgeB_.clear();
        mapLastPath_.clear();
        session_.SaveMaps();
    }
    if (!mapDragNode_.empty()) {
        if (ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
            for (MotionMapNode& n : map.nodes) {
                if (n.uid != mapDragNode_) continue;
                n.x += ImGui::GetIO().MouseDelta.x;
                n.y += ImGui::GetIO().MouseDelta.y;
                n.x = n.x < kNodeR ? kNodeR : (n.x > cs.x - kNodeR ? cs.x - kNodeR : n.x);
                n.y = n.y < kNodeR ? kNodeR : (n.y > cs.y - kNodeR ? cs.y - kNodeR : n.y);
                break;
            }
        } else {
            mapDragNode_.clear();
            session_.SaveMaps();   // persist the final spot once the drag ends
        }
    }
    if (canvasHovered && ImGui::IsMouseClicked(ImGuiMouseButton_Right)) {
        if (mapEdgeMode_ && !mapEdgeFrom_.empty()) {
            mapEdgeFrom_.clear();   // cancel the pending edge pick (yellow line)
        } else {
            mapCtxNode_ = hovered;
            mapCtxEdge_ = hovered.empty() ? hitEdge() : -1;
            if (hovered.empty() && mapCtxEdge_ < 0) mapAddAt_ = lm;
            ImGui::OpenPopup("mapctx");
        }
    }
    if (ImGui::BeginPopup("mapctx")) {
        if (!mapCtxNode_.empty()) {
            const SavedPos* p = posOf(mapCtxNode_);
            ImGui::TextDisabled("node '%s'", p ? p->name : "(missing)");
            ImGui::Separator();
            if (ImGui::MenuItem("Delete node (and its edges)")) {
                map.RemoveNode(mapCtxNode_);
                if (mapSelNode_ == mapCtxNode_) mapSelNode_.clear();
                mapLastPath_.clear();
                session_.SaveMaps();
            }
        } else if (mapCtxEdge_ >= 0 && mapCtxEdge_ < static_cast<int>(map.edges.size())) {
            ImGui::TextDisabled("edge");
            ImGui::Separator();
            if (ImGui::MenuItem("Delete edge")) {
                map.edges.erase(map.edges.begin() + mapCtxEdge_);
                mapLastPath_.clear();
                session_.SaveMaps();
            }
        } else {
            ImGui::TextDisabled("Add node here:");
            ImGui::Separator();
            bool anyLeft = false;
            for (size_t i = 0; i < saved.size(); ++i) {
                if (map.FindNode(saved[i].uid)) continue;
                anyLeft = true;
                ImGui::PushID(static_cast<int>(i));
                if (ImGui::MenuItem(saved[i].name)) {
                    MotionMapNode n;
                    n.uid = saved[i].uid;
                    n.x   = mapAddAt_.x;
                    n.y   = mapAddAt_.y;
                    map.nodes.push_back(std::move(n));
                    session_.SaveMaps();
                }
                ImGui::PopID();
            }
            if (!anyLeft) ImGui::TextDisabled("(every saved position is already a node)");
        }
        ImGui::EndPopup();
    }

    const Workflow::PathProgress pp = workflow_.PathStatus();
    const bool running = pp.active && pp.devIdx == devIdx;

    // Runs the planned chain through the path executor: shared by the side
    // panel's "Path here" and the strip's Go below, so a route drawn on the
    // canvas is exactly the route driven.
    const auto startPath = [&](const std::string& fromUid, const std::string& toUid) {
        std::string routeErr;
        const std::vector<MapRouteLeg> legs =
            BuildMapRoute(map, saved, m->Axes(), fromUid, toUid, &routeErr);
        if (legs.empty()) {
            FD_WARN("path", "%s: %s", devName.c_str(), routeErr.c_str());
            return;
        }
        mapLastPath_.clear();
        std::vector<Workflow::PathWaypoint> wps;
        wps.reserve(legs.size());
        for (const MapRouteLeg& leg : legs) {
            mapLastPath_.push_back(leg.uid);
            wps.push_back({leg.label, leg.targets});
        }
        workflow_.StartPathMove(devIdx, std::move(wps));
    };

    // ---- Side panel: the selected node, and how to get there ----
    // On the canvas, top-right. Two offers, never both: when the device sits
    // on a node that the map connects to the selection, a path along the
    // edges - the whole reason the map exists. Otherwise a DIRECT move, behind
    // a confirmation, because the edges are the record of which straight
    // lines are known to be clear and a move that ignores them may hit
    // whatever the map was drawn to avoid.
    if (selNodeValid) {
        const SavedPos* selPos = posOf(mapSelNode_);
        const bool here = mapSelNode_ == currentUid;
        std::string routeErr;
        const bool routed = !here && !currentUid.empty() && selPos &&
            !BuildMapRoute(map, saved, m->Axes(), currentUid, mapSelNode_, &routeErr).empty();
        const bool canMove = !running && !orch_.Busy() && m->IsConnected() && selPos;

        ImGui::SetCursorScreenPos(panelPos);
        ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.10f, 0.11f, 0.14f, 0.96f));
        ImGui::BeginChild("mapside", panelSize, ImGuiChildFlags_Borders,
                          ImGuiWindowFlags_NoScrollbar);
        ImGui::PopStyleColor();
        ImGui::TextUnformatted(selPos ? selPos->name : "(missing position)");
        if (!selPos) {
            ImGui::TextDisabled("node has no saved position");
        } else if (here) {
            ImGui::TextColored(ImVec4(0.30f, 0.85f, 0.40f, 1.0f), "device is here");
        } else if (routed) {
            const SavedPos* fromPos = posOf(currentUid);
            ImGui::TextDisabled("path from '%s'", fromPos ? fromPos->name : "?");
            ImGui::BeginDisabled(!canMove);
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.18f, 0.50f, 0.28f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.22f, 0.62f, 0.34f, 1.0f));
            if (ImGui::Button("Path here", ImVec2(-1.0f, 0.0f)))
                startPath(currentUid, mapSelNode_);
            ImGui::PopStyleColor(2);
            ImGui::EndDisabled();
        } else {
            if (currentUid.empty()) {
                ImGui::TextDisabled("device is not on a node");
            } else {
                const SavedPos* fromPos = posOf(currentUid);
                ImGui::TextDisabled("no path from '%s'", fromPos ? fromPos->name : "?");
            }
            ImGui::BeginDisabled(!canMove);
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.62f, 0.42f, 0.12f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.75f, 0.52f, 0.16f, 1.0f));
            if (ImGui::Button("Move directly...", ImVec2(-1.0f, 0.0f))) openMapDirect_ = true;
            ImGui::PopStyleColor(2);
            ImGui::EndDisabled();
        }
        ImGui::BeginDisabled(!m->IsConnected());
        if (ImGui::SmallButton("Stop")) {
            if (running) workflow_.CancelPathMove();
            else         workflow_.StopMotion(devIdx);
        }
        ImGui::EndDisabled();
        ImGui::SameLine();
        if (ImGui::SmallButton("Deselect")) mapSelNode_.clear();
        ImGui::EndChild();
        ImGui::SetCursorScreenPos(afterCanvas);   // back below the canvas
    }

    if (openMapDirect_) { ImGui::OpenPopup("Direct Move"); openMapDirect_ = false; }
    ImGui::SetNextWindowPos(ImGui::GetMainViewport()->GetCenter(), ImGuiCond_Appearing,
                            ImVec2(0.5f, 0.5f));
    if (ImGui::BeginPopupModal("Direct Move", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        const SavedPos* selPos = selNodeValid ? posOf(mapSelNode_) : nullptr;
        if (!selPos || running || orch_.Busy() || !m->IsConnected()) {
            ImGui::CloseCurrentPopup();
        } else {
            ImGui::Text("Move %s directly to '%s'?", devName.c_str(), selPos->name);
            ImGui::TextColored(ImVec4(0.95f, 0.70f, 0.25f, 1.0f),
                               "This is a straight-line move that ignores the map's edges.");
            ImGui::TextDisabled("%s", currentUid.empty()
                                          ? "The device is not on a node, so no path can be planned."
                                          : "The map has no path from the node the device is on.");
            ImGui::TextDisabled("Make sure nothing is in the way before confirming.");
            ImGui::Spacing();
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.62f, 0.42f, 0.12f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.75f, 0.52f, 0.16f, 1.0f));
            if (ImGui::Button("Move directly")) {
                const auto targets = TargetsOf(*selPos, m);
                if (targets.empty()) {
                    FD_WARN("path", "%s: '%s' shares no axis with the device",
                            devName.c_str(), selPos->name);
                } else {
                    FD_INFO("path", "%s: DIRECT move to '%s' (map edges bypassed, confirmed)",
                            devName.c_str(), selPos->name);
                    mapLastPath_.clear();
                    if (!workflow_.MoveToPosition(devIdx, targets))
                        FD_WARN("path", "%s: direct move to '%s' rejected",
                                devName.c_str(), selPos->name);
                }
                ImGui::CloseCurrentPopup();
            }
            ImGui::PopStyleColor(2);
            ImGui::SameLine();
            if (ImGui::Button("Cancel")) ImGui::CloseCurrentPopup();
        }
        ImGui::EndPopup();
    }

    // ---- Move along path (node -> node via edges) ----
    struct GoNode { std::string uid; const char* label; };
    std::vector<GoNode> goNodes;
    for (const MotionMapNode& n : map.nodes) {
        if (const SavedPos* p = posOf(n.uid)) goNodes.push_back({n.uid, p->name});
    }
    if (mapTo_ >= static_cast<int>(goNodes.size()))       mapTo_   = -1;
    if (mapFrom_ > static_cast<int>(goNodes.size()))      mapFrom_ = 0;

    ImGui::TextUnformatted("Move");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(160.0f);
    if (ImGui::BeginCombo("##gofrom", mapFrom_ == 0 ? "(nearest node)"
                                                    : goNodes[mapFrom_ - 1].label)) {
        if (ImGui::Selectable("(nearest node)", mapFrom_ == 0)) mapFrom_ = 0;
        for (int i = 0; i < static_cast<int>(goNodes.size()); ++i) {
            ImGui::PushID(i);
            if (ImGui::Selectable(goNodes[i].label, mapFrom_ == i + 1)) mapFrom_ = i + 1;
            ImGui::PopID();
        }
        ImGui::EndCombo();
    }
    ImGui::SameLine();
    ImGui::TextUnformatted("->");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(160.0f);
    if (ImGui::BeginCombo("##goto", mapTo_ >= 0 ? goNodes[mapTo_].label : "(target)")) {
        for (int i = 0; i < static_cast<int>(goNodes.size()); ++i) {
            ImGui::PushID(i);
            if (ImGui::Selectable(goNodes[i].label, mapTo_ == i)) mapTo_ = i;
            ImGui::PopID();
        }
        ImGui::EndCombo();
    }
    ImGui::SameLine();
    ImGui::BeginDisabled(running || orch_.Busy() ||
                         !m->IsConnected() || goNodes.empty() || mapTo_ < 0);
    if (ImGui::Button("Go")) {
        // Both the start resolution and the route come from core/MotionMap —
        // the UI contributes the selection, not the geometry.
        const std::string fromUid =
            (mapFrom_ == 0) ? NearestMapNodeUid(map, saved, m->Positions())
                            : goNodes[mapFrom_ - 1].uid;
        // Route planning is shared with the recipe engine's move_path command
        // (core/MotionMap::BuildMapRoute) — one planner, two executors.
        startPath(fromUid, goNodes[mapTo_].uid);
    }
    ImGui::EndDisabled();

    if (running) {
        ImGui::TextColored(ImVec4(0.30f, 0.85f, 0.40f, 1.0f), "driving to '%s' (%zu/%zu)",
                           pp.label.c_str(), pp.current + 1, pp.total);
        ImGui::SameLine();
        if (ImGui::SmallButton("Cancel")) workflow_.CancelPathMove();
    } else if (!mapLastPath_.empty()) {
        std::string line = "last path: ";
        for (size_t i = 0; i < mapLastPath_.size(); ++i) {
            const SavedPos* p = posOf(mapLastPath_[i]);
            if (i) line += " -> ";
            line += p ? p->name : "?";
        }
        ImGui::TextDisabled("%s", line.c_str());
    }
}

} // namespace firedog
