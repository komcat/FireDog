// ProductSession.h : the application document — everything the operator's
// *product* consists of (saved positions, motion maps, recipes, programs,
// scripts) plus the manifest they belong to, and the ONE place that knows the
// product-file / config-fallback persistence convention.
//
// UI-THREAD ONLY. No locks, no ImGui, no Workflow/HardwareManager. The engines
// never read this live: they receive a COPY through Env at Start — copy-at-
// Start IS the concurrency contract — which is why mutable collection
// accessors are safe here and dirty tracking can stay snapshot-compare instead
// of observers.
//
// UI selection state (recipeSel_, progSel_, deckSlot_ …) deliberately does NOT
// live here; App resets it in thin wrappers around the load calls, so the
// session never needs a callback back into the UI.
#pragma once

#include <string>
#include <utility>
#include <vector>

#include "core/MotionMap.h"
#include "core/Product.h"
#include "core/SavedPositions.h"

namespace firedog {

class ProductSession {
public:
    // ---- collections (the UI edits these in place) ----
    SavedPosMap&        Positions()       { return positions_; }
    const SavedPosMap&  Positions() const { return positions_; }
    MotionMapSet&       Maps()            { return maps_; }
    const MotionMapSet& Maps()      const { return maps_; }

    // ---- product manifest ----
    const ProductManifest& Product() const { return product_; }
    bool ProductLoaded() const { return product_.Loaded(); }
    void SetProduct(ProductManifest m) { product_ = std::move(m); }

    // ---- load / save ----
    void LoadPositions();   // config fallback (startup, before any product)
    void LoadMaps();        // node-edge maps (Mapped Path tab)
    void SavePositions();   // product positions file when loaded, else config
    void SaveMaps();        // machine-global: maps reference stable position uids

    // Teach hand-backs from save_position steps (queued on Workflow by engine
    // worker threads, drained on the UI thread): merge by name — the app-side
    // uid wins for existing names so recipe references stay stable — then
    // persist once per batch. No-op for an empty batch.
    void MergeTaughtPositions(const std::vector<std::pair<std::string, SavedPos>>& taught);

private:
    ProductManifest product_;
    SavedPosMap     positions_;
    MotionMapSet    maps_;
};

} // namespace firedog
