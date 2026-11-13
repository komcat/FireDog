// PorExport.cpp : see PorExport.h.
#include "PorExport.h"

#include "core/BuildInfo.h"
#include "core/Product.h"   // NowStamp

#include <nlohmann/json.hpp>

#include <filesystem>
#include <fstream>
#include <system_error>

namespace fs = std::filesystem;

namespace firedog {

namespace {

bool EndsWith(const std::string& s, const std::string& tail)
{
    return s.size() >= tail.size() && s.compare(s.size() - tail.size(), tail.size(), tail) == 0;
}

// Backups and the position journal stay with the machine they describe.
bool Skip(const fs::path& p)
{
    const std::string n = p.filename().string();
    return EndsWith(n, ".bak") || EndsWith(n, ".history.jsonl");
}

bool CopyOne(const fs::path& from, const fs::path& to, const fs::path& destRoot,
             std::vector<std::string>* log, std::string* err)
{
    std::error_code ec;
    fs::create_directories(to.parent_path(), ec);
    fs::copy_file(from, to, fs::copy_options::none, ec);
    if (ec) {
        *err = "could not copy " + from.generic_string() + ": " + ec.message();
        return false;
    }
    log->push_back(fs::relative(to, destRoot, ec).generic_string());
    return true;
}

} // namespace

bool ExportMaster(const std::string& masterRoot, const std::string& product,
                  const std::string& dest, std::vector<std::string>* log, std::string* err)
{
    log->clear();
    err->clear();
    if (masterRoot.empty() || product.empty() || dest.empty()) {
        *err = "master root, product and destination are all needed";
        return false;
    }
    std::error_code ec;
    const fs::path src     = fs::weakly_canonical(fs::path(masterRoot) / "products" / product, ec);
    const fs::path destAbs = fs::weakly_canonical(fs::path(dest), ec);
    if (!fs::is_regular_file(src / "manifest.json", ec)) {
        *err = "no product '" + product + "' under " + masterRoot;
        return false;
    }
    // An export inside the folder being exported would copy itself forever.
    const std::string s = src.generic_string(), d = destAbs.generic_string();
    if (d == s || (d.size() > s.size() && d.compare(0, s.size(), s) == 0 && d[s.size()] == '/')) {
        *err = "the destination is inside the product folder";
        return false;
    }
    // Never mixed into an older export: one folder, one master, one moment.
    if (fs::exists(destAbs, ec) && !fs::is_empty(destAbs, ec)) {
        *err = "the destination is not empty - choose an empty folder";
        return false;
    }

    const fs::path outProduct = destAbs / "products" / product;
    for (auto it = fs::recursive_directory_iterator(src, ec); !ec && it != fs::recursive_directory_iterator();
         it.increment(ec)) {
        std::error_code ec2;
        if (!it->is_regular_file(ec2) || Skip(it->path())) continue;
        const fs::path rel = fs::relative(it->path(), src, ec2);
        if (!CopyOne(it->path(), outProduct / rel, destAbs, log, err)) return false;
    }
    if (ec) {
        *err = "could not read " + src.generic_string() + ": " + ec.message();
        return false;
    }
    for (const char* f : {"hardware.json", "maps.local.json", "motion_transforms.json"}) {
        const fs::path from = fs::path(masterRoot) / "config" / f;
        if (!fs::is_regular_file(from, ec)) continue;
        if (!CopyOne(from, destAbs / "config" / f, destAbs, log, err)) return false;
    }

    nlohmann::json j;
    j["product"]     = product;
    j["source_root"] = fs::path(masterRoot).generic_string();
    j["created"]     = NowStamp();
    j["tool"]        = GetBuildInfo().describe;
    j["files"]       = *log;
    std::ofstream f(destAbs / "por_export.json");
    if (!f.is_open()) {
        *err = "could not write por_export.json";
        return false;
    }
    f << j.dump(2) << '\n';
    log->push_back("por_export.json");
    return true;
}

} // namespace firedog
