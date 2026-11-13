// PorSide.h : one side of a POR compare — a product and the machine files it
// leans on, loaded READ-ONLY from a folder (docs/por-design.md §5).
//
// A side is a ROOT folder laid out like a machine root:
//     <root>/products/<product>/manifest.json (+ the role files, scripts/, prompt_images/)
//     <root>/config/hardware.json             (device, PSU, slide, light, pin NAMES)
//     <root>/config/maps.local.json           (named motion maps)
//     <root>/config/motion_transforms.json    (per-device global->local axes)
// The target is the machine root itself; a master export is a folder of the same
// shape. Nothing here resolves MachinePath/DataPath or writes a byte: every file
// is opened by the path built from `root`, through the plain Load*File readers.
#pragma once

#include "command/Program.h"
#include "command/Recipe.h"
#include "core/AlignFrame.h"
#include "core/GripProfile.h"
#include "core/MotionMap.h"
#include "core/Product.h"
#include "core/SavedPositions.h"
#include "core/WaffleTray.h"
#include "hardware/HardwareConfig.h"
#include "vision/VisionModel.h"

#include <map>
#include <set>
#include <string>
#include <vector>

namespace firedog {

// The names a recipe may refer to, from one side's hardware.json (enabled
// entries only - a disabled device is not one a run can use).
struct PorHardwareNames {
    std::map<std::string, std::string> motion;    // device name -> controller type ("PI" | "ACS")
    struct Io {
        std::set<std::string> inputs, outputs;     // pin names
    };
    std::map<std::string, Io>        io;          // io device name -> its pin names
    std::vector<std::string>         psus;        // config order; "" in a step = the first
    std::set<std::string>            slides;
    std::vector<LightChannelConfig>  lightChannels;   // every enabled channel of every light
    bool                             hasLight = false;
};

struct PorSide {
    std::string root;      // forward slashes
    std::string product;   // folder name under <root>/products

    ProductManifest          manifest;
    SavedPosMap              positions;
    std::vector<Recipe>      recipes;
    std::vector<Program>     programs;
    std::vector<AlignProc>   aligns;
    std::vector<VisionModel> vision;
    std::vector<GripProfile> grips;
    std::vector<WaffleTray>  trays;

    MotionMapSet     maps;          // machine-global on that side
    bool             hasHardware = false;
    HardwareConfig   hwConfig;      // the whole file, for the hardware compare
    PorHardwareNames hw;            // the names a recipe may use, from hwConfig
    bool             hasTransforms = false;
    std::map<std::string, AxisTransform> transforms;   // config/motion_transforms.json

    std::set<std::string> scripts;        // file names: <product>/scripts, then <root>/config/scripts
    std::set<std::string> promptImages;   // file names: <product>/prompt_images, then <root>/config/prompt_images

    std::vector<std::string> notes;       // what was read from where, and what could not be

    bool Loaded() const { return manifest.Loaded(); }
};

// Folders under <root>/products that hold a manifest.json, sorted.
std::vector<std::string> ListPorProducts(const std::string& root);

// Load one side. False (with *out->notes saying why) when the product's
// manifest cannot be read - the machine files (hardware, maps, transforms) are
// loaded all the same; a missing role file is a note and an empty document.
// A role file that is EMPTY falls back to <root>/config/<role>.local.json, the
// same way the app falls back for a product that never saved that role - so the
// compare sees what a run on that machine would.
bool LoadPorSide(const std::string& root, const std::string& product, PorSide* out);

} // namespace firedog
