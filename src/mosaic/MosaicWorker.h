// MosaicWorker.h : the Mosaic page's background thread — decodes tile PNGs for
// the viewer and writes the exports (label crops, the stitched picture), so
// neither ever runs on the UI thread. Design: docs/mosaic-design.md.
//
// ONE thread, owned: started by the constructor, stopped and JOINED by the
// destructor (no detached threads, ever). It never touches GL - it hands RGB
// back and the UI uploads it - and it never touches the document or the
// machine: its inputs are a copy of the capture record and file paths. The
// one file it writes inside a mosaic folder, besides export/, is
// placement.json (seam refinement), which nothing else writes.
//
// Loads are serviced between an export's bands too, so a long 1:1 export does
// not freeze the viewer's zoom-in.
#pragma once

#include "mosaic/MosaicDoc.h"

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace firedog {

class MosaicWorker {
public:
    MosaicWorker();
    ~MosaicWorker();
    MosaicWorker(const MosaicWorker&) = delete;
    MosaicWorker& operator=(const MosaicWorker&) = delete;

    // ---- Tile loads ----------------------------------------------------------
    // `gen` is the caller's mosaic generation: a result from an older one is
    // for a mosaic nobody is looking at any more, and the caller drops it.
    struct Loaded {
        unsigned                   gen = 0;
        int                        tile = 0;
        int                        level = 0;   // the caller's own tag
        std::vector<unsigned char> rgb;         // w * h * 3
        int                        w = 0, h = 0;
        bool                       ok = false;
    };
    void Load(unsigned gen, int tile, int level, std::string path);
    void DropLoads();                                 // forget everything queued
    std::vector<Loaded> TakeLoaded(std::size_t max);  // at most `max`, oldest first

    // ---- Jobs: exports, and the seam refinement ------------------------------------
    enum class JobKind {
        Picture,   // the whole stitched picture at 1:scale
        Crops,     // one full-resolution PNG per crop label
        Refine,    // measure the seams, write placement.json
        FitMap,    // fit the camera map to the seams: a PROPOSAL, nothing written
    };
    struct Export {
        JobKind                  kind = JobKind::Picture;
        std::string              dir;      // the mosaic folder
        MosaicRecord             rec;      // a copy: the worker reads nothing live
        bool                     useRefined = true;   // compose from placement.json's positions
        std::vector<MosaicLabel> labels;              // Crops
        int                      scale = 1;           // Picture: 1, 2, 4 or 8
    };
    // False + *why when an export is already running.
    bool StartExport(Export job, std::string* why);
    void CancelExport();

    struct Progress {
        bool        active = false;
        float       frac = 0.0f;
        std::string what;        // what it is doing now
        // The last finished export: its verdict and where it wrote.
        unsigned    done = 0;    // bumps when an export finishes
        JobKind     kind = JobKind::Picture;   // of the last finished one
        bool        ok = false;
        std::string result;
        std::string outPath;
        MosaicMapFit fit;        // FitMap: the fitted camera map, for review
    };
    Progress GetProgress() const;

private:
    struct LoadReq {
        unsigned    gen;
        int         tile;
        int         level;
        std::string path;
    };

    void Main();
    void ServiceLoads();   // run every queued load; called between export bands too
    bool RunExport(const Export& job, std::string* result, std::string* outPath);
    bool ExportCrops(const Export& job, std::string* result, std::string* outPath);
    bool ExportPicture(const Export& job, std::string* result, std::string* outPath);
    bool RefineSeams(const Export& job, std::string* result, std::string* outPath);
    bool FitMap(const Export& job, std::string* result, MosaicMapFit* fit);
    // The input + grey loader both seam jobs measure with.
    MosaicRefineInput SeamInput(const Export& job, int searchPx) const;
    bool SeamGray(const Export& job, const MosaicRefineInput& in, std::size_t i,
                  std::vector<unsigned char>* out, int* w, int* h);
    void SetWhat(const std::string& what, float frac);

    mutable std::mutex      mu_;
    std::condition_variable cv_;
    std::deque<LoadReq>     loads_;
    std::deque<Loaded>      loaded_;
    std::unique_ptr<Export> pending_;   // the export waiting to start
    Progress                progress_;
    std::atomic<bool>       cancel_{false};
    bool                    quit_ = false;
    std::thread             thread_;    // last: started after everything above exists
};

} // namespace firedog
