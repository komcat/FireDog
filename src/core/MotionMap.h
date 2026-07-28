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

#include <functional>
#include <map>
#include <string>
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

} // namespace firedog
