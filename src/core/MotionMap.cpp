// MotionMap.cpp
#include "core/MotionMap.h"

#include "core/Log.h"
#include "core/Product.h"   // GenerateUid — backfill for maps without one

#include <nlohmann/json.hpp>

#include <algorithm>
#include <exception>
#include <fstream>
#include <limits>
#include <queue>
#include <utility>

namespace firedog {

namespace {
constexpr const char* kLocalPath = "config/maps.local.json";
constexpr const char* kBasePath  = "config/maps.json";
} // namespace

const MotionMapNode* MotionMap::FindNode(const std::string& id) const
{
    for (const MotionMapNode& n : nodes) {
        if (n.uid == id) return &n;
    }
    return nullptr;
}

bool MotionMap::HasEdge(const std::string& a, const std::string& b) const
{
    for (const MotionMapEdge& e : edges) {
        if ((e.a == a && e.b == b) || (e.a == b && e.b == a)) return true;
    }
    return false;
}

void MotionMap::RemoveNode(const std::string& id)
{
    nodes.erase(std::remove_if(nodes.begin(), nodes.end(),
                               [&](const MotionMapNode& n) { return n.uid == id; }),
                nodes.end());
    edges.erase(std::remove_if(edges.begin(), edges.end(),
                               [&](const MotionMapEdge& e) { return e.a == id || e.b == id; }),
                edges.end());
}

void MotionMap::RemoveEdge(const std::string& a, const std::string& b)
{
    edges.erase(std::remove_if(edges.begin(), edges.end(),
                               [&](const MotionMapEdge& e) {
                                   return (e.a == a && e.b == b) || (e.a == b && e.b == a);
                               }),
                edges.end());
}

MotionMapSet LoadMotionMaps()
{
    std::string path = kLocalPath;
    std::ifstream f(path);
    if (!f.is_open()) {
        path = kBasePath;
        f.open(path);
        if (!f.is_open()) return MotionMapSet{};   // first run — start empty
    }

    MotionMapSet out;
    try {
        nlohmann::json j;
        f >> j;
        if (!j.contains("maps") || !j["maps"].is_object()) return out;
        size_t total = 0;
        for (const auto& [dev, list] : j["maps"].items()) {
            if (!list.is_array()) continue;
            std::vector<MotionMap>& maps = out[dev];
            for (const auto& mj : list) {
                if (!mj.is_object()) continue;
                MotionMap m;
                m.name = mj.value("name", std::string("map"));
                m.uid  = mj.value("uid", std::string());
                if (m.uid.empty()) m.uid = GenerateUid();
                if (mj.contains("nodes") && mj["nodes"].is_array()) {
                    for (const auto& nj : mj["nodes"]) {
                        MotionMapNode n;
                        n.uid = nj.value("pos_uid", std::string());
                        n.x   = nj.value("x", 0.0f);
                        n.y   = nj.value("y", 0.0f);
                        if (!n.uid.empty()) m.nodes.push_back(std::move(n));
                    }
                }
                if (mj.contains("edges") && mj["edges"].is_array()) {
                    for (const auto& ej : mj["edges"]) {
                        MotionMapEdge e;
                        e.a = ej.value("a", std::string());
                        e.b = ej.value("b", std::string());
                        // Keep only edges whose endpoints exist and differ.
                        if (!e.a.empty() && !e.b.empty() && e.a != e.b &&
                            m.FindNode(e.a) && m.FindNode(e.b) && !m.HasEdge(e.a, e.b)) {
                            m.edges.push_back(std::move(e));
                        }
                    }
                }
                maps.push_back(std::move(m));
                ++total;
            }
        }
        FD_INFO("map", "loaded %zu map(s) for %zu device(s) from %s",
                total, out.size(), path.c_str());
    } catch (const std::exception& e) {
        FD_WARN("map", "maps load failed (%s): starting empty", e.what());
        return MotionMapSet{};
    }
    return out;
}

bool SaveMotionMaps(const MotionMapSet& maps)
{
    nlohmann::json devs = nlohmann::json::object();
    for (const auto& [dev, list] : maps) {
        nlohmann::json arr = nlohmann::json::array();
        for (const MotionMap& m : list) {
            nlohmann::json mj;
            mj["uid"]  = m.uid;
            mj["name"] = m.name;
            nlohmann::json nodes = nlohmann::json::array();
            for (const MotionMapNode& n : m.nodes) {
                nodes.push_back({{"pos_uid", n.uid}, {"x", n.x}, {"y", n.y}});
            }
            mj["nodes"] = std::move(nodes);
            nlohmann::json edges = nlohmann::json::array();
            for (const MotionMapEdge& e : m.edges) {
                edges.push_back({{"a", e.a}, {"b", e.b}});
            }
            mj["edges"] = std::move(edges);
            arr.push_back(std::move(mj));
        }
        devs[dev] = std::move(arr);
    }
    nlohmann::json j;
    j["maps"] = std::move(devs);

    std::ofstream f(kLocalPath);
    if (!f.is_open()) {
        FD_WARN("map", "maps save failed: cannot open %s", kLocalPath);
        return false;
    }
    f << j.dump(2) << '\n';
    return true;
}

std::vector<std::string> FindMapPath(
    const MotionMap& map, const std::string& from, const std::string& to,
    const std::function<double(const std::string&, const std::string&)>& cost)
{
    if (!map.FindNode(from) || !map.FindNode(to)) return {};
    if (from == to) return {from};

    // Adjacency over the bidirectional edge list.
    std::map<std::string, std::vector<std::string>> adj;
    for (const MotionMapEdge& e : map.edges) {
        adj[e.a].push_back(e.b);
        adj[e.b].push_back(e.a);
    }

    // Dijkstra: (distance, uid) min-heap with lazy deletion.
    std::map<std::string, double>      dist;
    std::map<std::string, std::string> prev;
    using QE = std::pair<double, std::string>;
    std::priority_queue<QE, std::vector<QE>, std::greater<QE>> q;
    dist[from] = 0.0;
    q.push({0.0, from});
    while (!q.empty()) {
        const auto [d, u] = q.top();
        q.pop();
        if (d > dist[u]) continue;          // stale entry
        if (u == to) break;
        for (const std::string& v : adj[u]) {
            double w = cost ? cost(u, v) : 1.0;
            if (!(w > 0.0)) w = 1.0;        // guard bad/zero weights
            const double nd = d + w;
            const auto   it = dist.find(v);
            if (it == dist.end() || nd < it->second) {
                dist[v] = nd;
                prev[v] = u;
                q.push({nd, v});
            }
        }
    }
    if (dist.find(to) == dist.end()) return {};   // unreachable

    std::vector<std::string> path;
    for (std::string u = to; ; u = prev[u]) {
        path.push_back(u);
        if (u == from) break;
    }
    std::reverse(path.begin(), path.end());
    return path;
}

} // namespace firedog
