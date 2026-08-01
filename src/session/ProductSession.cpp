#include "session/ProductSession.h"

#include "core/Log.h"

namespace firedog {

void ProductSession::LoadPositions()
{
    positions_ = LoadSavedPositions();   // config fallback (no product yet)
}

void ProductSession::LoadMaps()
{
    maps_ = LoadMotionMaps();
}

void ProductSession::SavePositions()
{
    // Product loaded -> autosave into the product's positions file (with its
    // uid); otherwise the machine-global config fallback. Warns if unwritable.
    if (const ManifestFile* e = product_.Entry("positions"))
        SaveSavedPositionsFile(product_.FilePath("positions"), positions_, e->uid);
    else
        SaveSavedPositions(positions_);
}

void ProductSession::SaveMaps()
{
    // Machine-global for now (config/maps.local.json) — maps reference
    // position uids, which stay stable across products.
    if (SaveMotionMaps(maps_))
        FD_INFO("map", "maps saved to config/maps.local.json");
    else
        FD_WARN("map", "could not save motion maps");
}

void ProductSession::MergeTaughtPositions(
    const std::vector<std::pair<std::string, SavedPos>>& taught)
{
    if (taught.empty()) return;
    for (const auto& [dev, sp] : taught) {
        std::vector<SavedPos>& list = positions_[dev];
        bool found = false;
        for (SavedPos& e : list) {
            if (std::string(e.name) == sp.name) {
                e.vals = sp.vals;
                found  = true;
                break;
            }
        }
        if (!found) list.push_back(sp);
    }
    SavePositions();
}

} // namespace firedog
