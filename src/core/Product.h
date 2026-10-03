// Product.h : product manifest — a per-product folder of core data files.
//
// A product is a folder under `products/` (next to the exe) described by a
// manifest.json. Each core file (positions / recipes / programs) carries a uid
// both in its filename ({uid}.positions.json) and inside the file, so a stray
// file can always be traced back to its product. The manifest also names the
// data/ and images/ subfolders (future save-data locations).
// Tolerant load: a missing or malformed manifest yields an empty (not-loaded)
// ProductManifest, never throws.
#pragma once

#include <string>
#include <vector>

namespace firedog {

// One core-file entry in the manifest.
struct ManifestFile {
    std::string role;   // "positions" | "recipes" | "programs" | "alignments" | "vision"
    std::string uid;    // 8-hex uid, also the filename prefix
    std::string file;   // "{uid}.{role}.json", relative to the product folder
};

struct ProductManifest {
    std::string name;      // product name == folder name under products/
    std::string uid;       // the product's own uid
    std::string created;   // ISO-ish local timestamp
    std::vector<ManifestFile> files;
    std::string dataDir   = "data";     // relative subfolder for save data
    std::string imagesDir = "images";   // relative subfolder for images

    bool        Loaded() const { return !name.empty(); }
    std::string Dir() const;                              // <machine root>/products/<name>
    std::string FilePath(const std::string& role) const;  // "" if role missing
    const ManifestFile* Entry(const std::string& role) const;
    // DUT records are production output, so they live under the DATA root,
    // mirrored by product name (core/Paths.h):
    std::string DataDir() const;                          // <data root>/products/<name>/data
    std::string ImagesDir() const;                        // <data root>/products/<name>/images
};

std::string GenerateUid();   // 8 random hex chars
std::string NowStamp();      // "YYYY-MM-DDTHH:MM:SS", local time

// Scan products/ for folders containing a manifest.json (sorted by name).
std::vector<std::string> ListProducts();

// Read products/<name>/manifest.json. Not-loaded manifest on any failure.
ProductManifest LoadProductManifest(const std::string& name);

// Create products/<name>/ (+ data/, images/), fresh uids, manifest.json and
// one {uid}.<role>.json seed file per role. Not-loaded manifest on failure.
ProductManifest CreateProduct(const std::string& name);

// Give an older product a role it predates: append {role, fresh uid,
// "{uid}.{role}.json"} and rewrite the manifest. False (no write) when the role
// is already there, so it is cheap to call on every load. Without this a
// product created before the role existed would keep falling back to config/
// and its saves would never reach the product folder.
bool EnsureManifestRole(ProductManifest& m, const std::string& role);

// Rewrite products/<name>/manifest.json. Best-effort; false if unwritable.
bool SaveProductManifest(const ProductManifest& m);

} // namespace firedog
