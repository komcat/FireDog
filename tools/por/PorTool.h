// PorTool.h : the FireDogPor window - a master product compared, read-only,
// with the same product on a target machine (docs/por-design.md §5-§6a).
//
// Left: an explorer (Deck > programs > slots, Recipes, Positions, Hardware >
// sections, Notes), each item badged with the worst result beneath it. Right:
// ONE table whose two columns are the master and the target, so paired rows
// line up and scroll together by construction; a row opens to its params.
//
// The tool writes nothing to either side. Its own state (the roots last used,
// the window, the log) lives under %LOCALAPPDATA%\FireDogPor - never beside the
// exe and never under a machine root.
#pragma once

#include "core/Paths.h"   // RootsPreview
#include "por/PorCompare.h"
#include "por/PorHardware.h"
#include "por/PorPositions.h"
#include "por/PorSide.h"
#include "ui/GlHost.h"    // WindowPlacement

#include <string>
#include <vector>

namespace firedog {

class CommandRegistry;

class PorTool {
public:
    PorTool(const CommandRegistry& reg, RootsPreview roots, std::string localDir);

    void LoadSettings();
    void SaveSettings() const;
    const WindowPlacement& Placement() const { return placement_; }
    void SetPlacement(const WindowPlacement& p) { placement_ = p; }

    void Draw();

private:
    enum class View { None, Program, Slot, Recipe, Positions, Hardware, Notes };

    void DrawTopBar();
    void DrawExplorer();
    void DrawContent();
    void DrawExportWindow();
    void DrawRowsTable(const char* id, const std::vector<PorRow>& rows,
                       const std::vector<PorParam>* header);
    void DrawRow(const PorRow& r, int depth);
    void DrawParam(const PorParam& p);
    void DrawPositions();
    void DrawNotes();

    void RefreshProducts();
    void Compare();
    void ComparePositionsOnly();
    bool Visible(const PorRow& r) const;
    int  VisibleParams(const PorRow& r) const;
    int  FindRecipe(const std::string& name) const;
    bool ExplorerItem(const char* label, bool selected, bool leaf, PorStatus badge,
                      bool defaultOpen = false);

    const CommandRegistry& reg_;
    RootsPreview           roots_;
    std::string            localDir_;
    WindowPlacement        placement_;

    char masterRoot_[512] = {};
    char targetRoot_[512] = {};
    char exportDir_[512]  = {};
    std::string              product_;
    std::vector<std::string> products_;
    float mmThreshold_  = 1.0f;
    float degThreshold_ = 0.1f;
    bool  diffOnly_     = true;
    bool  allParams_    = false;
    bool  workingOnly_  = false;
    bool  showExport_   = false;
    float explorerW_    = 330.0f;

    bool               compared_ = false;
    bool               masterOk_ = false, targetOk_ = false;
    PorSide            master_, target_;
    PorReport          report_;
    PorPositionsReport positions_;
    PorHardwareReport  hardware_;
    std::string        summary_;

    View view_ = View::None;
    int  sel_  = -1, sel2_ = -1;
    int  jumpRecipe_ = -1;   // set while drawing the table, applied after it

    std::vector<std::string> exportLog_;
    std::string              exportMsg_;
    bool                     exportOk_ = false;
};

} // namespace firedog
