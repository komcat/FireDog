// MotionMap.h : node-edge motion maps ("mapped paths"), persisted per device.
//
// A map is a graph over saved positions: each node references a SavedPos by
// its stable uid (one node per position per map) and carries editor canvas
// coordinates; edges are bidirectional and connect two node uids. Devices can
// have several maps. Path execution moves node-to-node along edges only —
// the graph encodes which direct moves are safe.
//
// One consolidated JSON for all devices (keyed by config device name), stored
// as `config/maps.local.json` next to the exe with a `maps.json` fallback —
// the project's *.local.json convention. Tolerant load: a missing or
// malformed file yields an empty set, never throws.
#pragma once

#include "core/SavedPositions.h"   // BuildMapRoute resolves node uids to positions

#include <functional>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace firedog {

struct MotionMapNode {
    std::string uid;        // saved-position uid this node represents
    float       x = 0.0f;   // editor canvas position (px)
    float       y = 0.0f;
};

struct MotionMapEdge {
    std::string a, b;       // node (position) uids; traversable both ways
};

struct MotionMap {
    std::string name = "map";
    std::string uid;                    // stable map identity
    std::vector<MotionMapNode> nodes;
    std::vector<MotionMapEdge> edges;

    const MotionMapNode* FindNode(const std::string& uid) const;
    bool HasEdge(const std::string& a, const std::string& b) const;   // either direction
    // Remove a node and every edge touching it.
    void RemoveNode(const std::string& uid);
    void RemoveEdge(const std::string& a, const std::string& b);      // either direction
};

// Device name -> ordered list of maps.
using MotionMapSet = std::map<std::string, std::vector<MotionMap>>;

// Reads config/maps.local.json, falling back to config/maps.json.
MotionMapSet LoadMotionMaps();

// Writes config/maps.local.json. Best-effort; false if unwritable.
bool SaveMotionMaps(const MotionMapSet& maps);

// Dijkstra over the map's edges from node `from` to node `to` (position uids).
// `cost` returns the edge weight between two position uids (e.g. XYZ distance;
// must be > 0). Returns the node uids of the path INCLUDING both endpoints,
// or an empty vector if unreachable / unknown nodes.
std::vector<std::string> FindMapPath(
    const MotionMap& map, const std::string& from, const std::string& to,
    const std::function<double(const std::string&, const std::string&)>& cost);

// One leg of a planned route: a node's label + its per-axis absolute targets
// (restricted to the device's axes). Same shape as Workflow::PathWaypoint —
// core/ cannot depend on workflow/, so the caller converts trivially.
struct MapRouteLeg {
    std::string uid;     // node (position) uid — the UI's canvas highlight
    std::string label;
    std::vector<std::pair<std::string, double>> targets;
};

// Plan a route from -> to over the map's edges (Dijkstra; edge weight =
// Euclidean distance between the two saved positions over their shared axes).
// `positions` is the device's saved-position list (node uids resolve into it);
// `axes` restricts each leg's targets to the device's axes. Returns the legs
// INCLUDING both endpoints, or empty on failure (unknown nodes, unreachable,
// node without a position) with the reason in *err. Shared by the Motion
// page's Go button and the recipe engine's move_path command — one planner,
// two executors.
std::vector<MapRouteLeg> BuildMapRoute(const MotionMap& map,
                                       const std::vector<SavedPos>& positions,
                                       const std::vector<std::string>& axes,
                                       const std::string& fromUid,
                                       const std::string& toUid,
                                       std::string* err = nullptr);

// The map node closest to a live device pose — the "(nearest node)" start
// option of the Motion page's Go combo. Distance is Euclidean over the axes
// the position and the pose share; axes missing from `pose` are ignored, so a
// partial reading still ranks. Only nodes whose uid resolves in `positions`
// are considered. Returns an empty string if none do.
//
// It lives beside BuildMapRoute on purpose: both answer "where on this map am
// I, and how do I get across it", and both must agree on how a node maps to a
// pose. Same spirit as the one-planner-two-executors rule above.
std::string NearestMapNodeUid(const MotionMap& map,
                              const std::vector<SavedPos>& positions,
                              const std::map<std::string, double>& pose);

} // namespace firedog
