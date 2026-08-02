// Product.cpp
#include "core/Product.h"

#include "core/Log.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <ctime>
#include <exception>
#include <filesystem>
#include <fstream>
#include <random>

namespace fs = std::filesystem;

namespace firedog {

namespace {

constexpr const char* kProductsRoot = "products";

} // namespace

std::string NowStamp()
{
    const std::time_t t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::tm tm{};
    localtime_s(&tm, &t);
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%04d-%02d-%02dT%02d:%02d:%02d",
                  tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
                  tm.tm_hour, tm.tm_min, tm.tm_sec);
    return buf;
}

std::string ProductManifest::Dir() const
{
    return std::string(kProductsRoot) + "/" + name;
}

const ManifestFile* ProductManifest::Entry(const std::string& role) const
{
    for (const ManifestFile& f : files) if (f.role == role) return &f;
    return nullptr;
}

std::string ProductManifest::FilePath(const std::string& role) const
{
    const ManifestFile* e = Entry(role);
    return e ? Dir() + "/" + e->file : std::string{};
}

std::string ProductManifest::DataDir()   const { return Dir() + "/" + dataDir; }
std::string ProductManifest::ImagesDir() const { return Dir() + "/" + imagesDir; }

std::string GenerateUid()
{
    static std::mt19937 rng{std::random_device{}()};
    std::uniform_int_distribution<unsigned> dist(0, 0xF);
    char buf[9];
    for (int i = 0; i < 8; ++i) buf[i] = "0123456789abcdef"[dist(rng)];
    buf[8] = '\0';
    return buf;
}

std::vector<std::string> ListProducts()
{
    std::vector<std::string> out;
    std::error_code ec;   // error_code overloads: never throw on a missing root
    for (const auto& entry : fs::directory_iterator(kProductsRoot, ec)) {
        if (!entry.is_directory(ec)) continue;
        if (fs::exists(entry.path() / "manifest.json", ec))
            out.push_back(entry.path().filename().string());
    }
    std::sort(out.begin(), out.end());
    return out;
}

ProductManifest LoadProductManifest(const std::string& name)
{
    ProductManifest m;
    const std::string path = std::string(kProductsRoot) + "/" + name + "/manifest.json";
    std::ifstream f(path);
    if (!f.is_open()) {
        FD_WARN("product", "no manifest at %s", path.c_str());
        return m;
    }
    try {
        nlohmann::json j;
        f >> j;
        m.name    = j.value("product", name);
        m.uid     = j.value("uid", "");
        m.created = j.value("created", "");
        if (j.contains("files") && j["files"].is_array()) {
            for (const auto& row : j["files"]) {
                if (!row.is_object()) continue;
                ManifestFile mf;
                mf.role = row.value("role", "");
                mf.uid  = row.value("uid", "");
                mf.file = row.value("file", "");
                if (!mf.role.empty() && !mf.file.empty()) m.files.push_back(std::move(mf));
            }
        }
        if (j.contains("folders") && j["folders"].is_object()) {
            m.dataDir   = j["folders"].value("data", m.dataDir);
            m.imagesDir = j["folders"].value("images", m.imagesDir);
        }
        // The folder name is the identity the app navigates by — keep it
        // authoritative even if the "product" field inside disagrees.
        if (m.name != name) {
            FD_WARN("product", "manifest name '%s' != folder '%s' (using folder)",
                    m.name.c_str(), name.c_str());
            m.name = name;
        }
        FD_INFO("product", "manifest '%s' loaded (%zu file(s), uid %s)",
                m.name.c_str(), m.files.size(), m.uid.c_str());
    } catch (const std::exception& e) {
        FD_WARN("product", "manifest load failed (%s)", e.what());
        return ProductManifest{};
    }
    return m;
}

bool SaveProductManifest(const ProductManifest& m)
{
    if (!m.Loaded()) return false;
    nlohmann::json j;
    j["product"] = m.name;
    j["uid"]     = m.uid;
    j["created"] = m.created;
    nlohmann::json files = nlohmann::json::array();
    for (const ManifestFile& mf : m.files)
        files.push_back({{"role", mf.role}, {"uid", mf.uid}, {"file", mf.file}});
    j["files"]   = std::move(files);
    j["folders"] = {{"data", m.dataDir}, {"images", m.imagesDir}};

    const std::string path = m.Dir() + "/manifest.json";
    std::ofstream f(path);
    if (!f.is_open()) {
        FD_WARN("product", "manifest save failed: cannot open %s", path.c_str());
        return false;
    }
    f << j.dump(2) << '\n';
    return true;
}

ProductManifest CreateProduct(const std::string& name)
{
    ProductManifest m;
    m.name    = name;
    m.uid     = GenerateUid();
    m.created = NowStamp();
    for (const char* role : {"positions", "recipes", "programs", "alignments", "vision"}) {
        ManifestFile mf;
        mf.role = role;
        mf.uid  = GenerateUid();
        mf.file = mf.uid + "." + mf.role + ".json";
        m.files.push_back(std::move(mf));
    }

    std::error_code ec;
    fs::create_directories(m.DataDir(), ec);
    fs::create_directories(m.ImagesDir(), ec);
    if (!fs::is_directory(m.Dir(), ec)) {
        FD_WARN("product", "create failed: cannot make %s", m.Dir().c_str());
        return ProductManifest{};
    }

    // Seed each core file with its uid; positions additionally get an empty map
    // so a product is loadable immediately after creation.
    for (const ManifestFile& mf : m.files) {
        nlohmann::json j;
        j["uid"] = mf.uid;
        if (mf.role == "positions") j["positions"] = nlohmann::json::object();
        std::ofstream f(m.Dir() + "/" + mf.file);
        if (f.is_open()) f << j.dump(2) << '\n';
    }
    if (!SaveProductManifest(m)) return ProductManifest{};

    FD_INFO("product", "created '%s' (uid %s) at %s", name.c_str(), m.uid.c_str(), m.Dir().c_str());
    return m;
}

bool EnsureManifestRole(ProductManifest& m, const std::string& role)
{
    if (!m.Loaded() || role.empty() || m.Entry(role)) return false;

    ManifestFile mf;
    mf.role = role;
    mf.uid  = GenerateUid();
    mf.file = mf.uid + "." + mf.role + ".json";
    m.files.push_back(mf);
    if (!SaveProductManifest(m)) {
        m.files.pop_back();   // keep memory and disk in step
        FD_WARN("product", "cannot add role '%s' to '%s': manifest not writable",
                role.c_str(), m.name.c_str());
        return false;
    }
    FD_INFO("product", "product '%s' gained role '%s' (uid %s)",
            m.name.c_str(), role.c_str(), mf.uid.c_str());
    return true;
}

} // namespace firedog
