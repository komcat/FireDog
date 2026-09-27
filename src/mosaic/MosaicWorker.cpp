// MosaicWorker.cpp
#include "mosaic/MosaicWorker.h"

#include "core/Log.h"
#include "vision/FocusMetric.h"   // RgbToGray8
#include "vision/MosaicLayout.h"

#pragma warning(push)
#pragma warning(disable : 4244 4456 4996)
#include <stb_image.h>         // implementation lives in ui/Widgets.cpp
#include <stb_image_write.h>   // implementation lives in command/CommandsDut.cpp
#pragma warning(pop)

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <exception>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <map>
#include <set>
#include <system_error>

namespace firedog {

namespace fs = std::filesystem;

namespace {

// Up to this many output pixels the picture is composed in memory and written
// as an ordinary compressed PNG; past it, band by band into a stored PNG.
constexpr double kInMemoryPixels = 40.0e6;

// Source rows composed per band of the streamed export.
constexpr int kBandSourceRows = 512;

// A label bigger than this is a mis-drag across the whole picture; its crop is
// refused rather than allocated.
constexpr double kMaxCropPixels = 400.0e6;

std::string Stamp()
{
    const std::time_t t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::tm tm{};
    localtime_s(&tm, &t);
    char b[32];
    std::snprintf(b, sizeof(b), "%04d%02d%02d_%02d%02d%02d", tm.tm_year + 1900, tm.tm_mon + 1,
                  tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
    return b;
}

bool DecodeRgb(const std::string& path, std::vector<unsigned char>* rgb, int* w, int* h)
{
    int n = 0;
    unsigned char* px = stbi_load(path.c_str(), w, h, &n, 3);
    if (!px) return false;
    rgb->assign(px, px + static_cast<std::size_t>(*w) * static_cast<std::size_t>(*h) * 3u);
    stbi_image_free(px);
    return true;
}

// ---- A PNG written as it is composed ---------------------------------------------
// The one thing stb cannot do: its encoder wants the whole picture in memory,
// and a 2" x 2" mosaic at full resolution is ~1.3 GB of RGB before compression.
// This writes the zlib stream as STORED (uncompressed) deflate blocks, which
// needs no compressor at all and keeps memory to one band: a valid PNG, about
// as big as the raw pixels.
class PngStream {
public:
    ~PngStream()
    {
        if (f_) std::fclose(f_);
    }

    bool Open(const std::string& path, int w, int h)
    {
        w_ = w;
        if (fopen_s(&f_, path.c_str(), "wb") != 0 || !f_) {
            f_ = nullptr;
            return false;
        }
        static const unsigned char sig[8] = {0x89, 'P', 'N', 'G', 0x0D, 0x0A, 0x1A, 0x0A};
        ok_ = std::fwrite(sig, 1, 8, f_) == 8;
        unsigned char ihdr[13];
        Be32(ihdr, static_cast<std::uint32_t>(w));
        Be32(ihdr + 4, static_cast<std::uint32_t>(h));
        ihdr[8]  = 8;   // bit depth
        ihdr[9]  = 2;   // colour type: RGB
        ihdr[10] = 0;   // deflate
        ihdr[11] = 0;   // filter method
        ihdr[12] = 0;   // no interlace
        Chunk("IHDR", ihdr, sizeof(ihdr));
        pending_.reserve(kBlock);
        return ok_;
    }

    // `n` rows of w * 3 bytes each. Every row goes out with filter type 0.
    bool Rows(const unsigned char* rgb, int n)
    {
        const std::size_t stride = static_cast<std::size_t>(w_) * 3u;
        for (int r = 0; r < n && ok_; ++r) {
            const unsigned char filter = 0;
            Feed(&filter, 1);
            Feed(rgb + static_cast<std::size_t>(r) * stride, stride);
        }
        return ok_;
    }

    bool Close()
    {
        if (!f_) return false;
        EmitBlock(true);
        Chunk("IEND", nullptr, 0);
        const bool closed = std::fclose(f_) == 0;
        f_ = nullptr;
        return ok_ && closed;
    }

private:
    static constexpr std::size_t kBlock = 65535;   // a stored block's largest LEN

    static void Be32(unsigned char* p, std::uint32_t v)
    {
        p[0] = static_cast<unsigned char>(v >> 24);
        p[1] = static_cast<unsigned char>(v >> 16);
        p[2] = static_cast<unsigned char>(v >> 8);
        p[3] = static_cast<unsigned char>(v);
    }

    static std::uint32_t Crc(std::uint32_t crc, const unsigned char* p, std::size_t n)
    {
        static const std::array<std::uint32_t, 256> table = [] {
            std::array<std::uint32_t, 256> t{};
            for (std::uint32_t i = 0; i < 256; ++i) {
                std::uint32_t c = i;
                for (int k = 0; k < 8; ++k) c = (c & 1u) ? 0xEDB88320u ^ (c >> 1) : c >> 1;
                t[i] = c;
            }
            return t;
        }();
        for (std::size_t i = 0; i < n; ++i) crc = table[(crc ^ p[i]) & 0xFFu] ^ (crc >> 8);
        return crc;
    }

    void Chunk(const char* type, const unsigned char* data, std::size_t n)
    {
        if (!ok_) return;
        unsigned char head[8];
        Be32(head, static_cast<std::uint32_t>(n));
        std::memcpy(head + 4, type, 4);
        std::uint32_t crc = Crc(0xFFFFFFFFu, head + 4, 4);
        if (n) crc = Crc(crc, data, n);
        crc ^= 0xFFFFFFFFu;
        unsigned char tail[4];
        Be32(tail, crc);
        ok_ = std::fwrite(head, 1, 8, f_) == 8 && (n == 0 || std::fwrite(data, 1, n, f_) == n) &&
              std::fwrite(tail, 1, 4, f_) == 4;
    }

    void Feed(const unsigned char* p, std::size_t n)
    {
        // Adler-32 of the uncompressed stream, in runs short enough that the
        // sums cannot overflow before the modulo.
        std::size_t left = n;
        const unsigned char* q = p;
        while (left > 0) {
            const std::size_t run = std::min<std::size_t>(left, 5552);
            for (std::size_t i = 0; i < run; ++i) {
                a_ += q[i];
                b_ += a_;
            }
            a_ %= 65521u;
            b_ %= 65521u;
            q += run;
            left -= run;
        }
        while (n > 0) {
            const std::size_t take = std::min(n, kBlock - pending_.size());
            pending_.insert(pending_.end(), p, p + take);
            p += take;
            n -= take;
            if (pending_.size() == kBlock) EmitBlock(false);
        }
    }

    void EmitBlock(bool final)
    {
        payload_.clear();
        if (first_) {           // the zlib header: deflate, 32K window, no dict
            payload_.push_back(0x78);
            payload_.push_back(0x01);
            first_ = false;
        }
        const std::uint16_t len  = static_cast<std::uint16_t>(pending_.size());
        const std::uint16_t nlen = static_cast<std::uint16_t>(~len);
        payload_.push_back(final ? 0x01 : 0x00);   // BFINAL, BTYPE = 00 (stored)
        payload_.push_back(static_cast<unsigned char>(len & 0xFFu));
        payload_.push_back(static_cast<unsigned char>(len >> 8));
        payload_.push_back(static_cast<unsigned char>(nlen & 0xFFu));
        payload_.push_back(static_cast<unsigned char>(nlen >> 8));
        payload_.insert(payload_.end(), pending_.begin(), pending_.end());
        if (final) {
            unsigned char ad[4];
            Be32(ad, (b_ << 16) | a_);
            payload_.insert(payload_.end(), ad, ad + 4);
        }
        Chunk("IDAT", payload_.data(), payload_.size());
        pending_.clear();
    }

    std::FILE*                 f_ = nullptr;
    int                        w_ = 0;
    bool                       ok_ = false;
    bool                       first_ = true;
    std::uint32_t              a_ = 1, b_ = 0;
    std::vector<unsigned char> pending_, payload_;
};

// Decoded full frames an export is working from, keyed by tile index. The
// caller says which tiles the next band needs; everything else is dropped, so
// memory stays at about one row of tiles.
struct TileCache {
    const MosaicRecord& rec;
    std::string         dir;
    bool                refined;   // where the tiles are placed: refined or stage
    std::map<std::size_t, std::vector<unsigned char>> px;
    std::set<std::size_t> bad;   // failed to decode: warned once, then absent

    const unsigned char* Get(std::size_t i)
    {
        if (i >= rec.tiles.size() || bad.count(i)) return nullptr;
        auto it = px.find(i);
        if (it != px.end()) return it->second.data();
        const MosaicTileRec& t = rec.tiles[i];
        std::vector<unsigned char> rgb;
        int w = 0, h = 0;
        const std::string path = (fs::path(dir) / t.file).string();
        if (!DecodeRgb(path, &rgb, &w, &h) || w != t.w || h != t.h) {
            FD_WARN("mosaic", "tile %s unreadable or the wrong size - left out of the export",
                    path.c_str());
            bad.insert(i);
            return nullptr;
        }
        return px.emplace(i, std::move(rgb)).first->second.data();
    }

    // Keep only tiles that overlap canvas rows [y0, y1) and columns [x0, x1).
    void KeepOnly(double x0, double y0, double x1, double y1)
    {
        for (auto it = px.begin(); it != px.end();) {
            const MosaicTileRec& t = rec.tiles[it->first];
            const double tx = MosaicTileX(rec, t, refined), ty = MosaicTileY(rec, t, refined);
            const bool keep = tx < x1 && tx + t.w > x0 && ty < y1 && ty + t.h > y0;
            it = keep ? std::next(it) : px.erase(it);
        }
    }
};

}   // namespace

MosaicWorker::MosaicWorker() : thread_(&MosaicWorker::Main, this) {}

MosaicWorker::~MosaicWorker()
{
    {
        const std::lock_guard<std::mutex> lock(mu_);
        quit_ = true;
    }
    cancel_ = true;
    cv_.notify_all();
    if (thread_.joinable()) thread_.join();
}

void MosaicWorker::Load(unsigned gen, int tile, int level, std::string path)
{
    {
        const std::lock_guard<std::mutex> lock(mu_);
        loads_.push_back({gen, tile, level, std::move(path)});
    }
    cv_.notify_all();
}

void MosaicWorker::DropLoads()
{
    const std::lock_guard<std::mutex> lock(mu_);
    loads_.clear();
    loaded_.clear();
}

std::vector<MosaicWorker::Loaded> MosaicWorker::TakeLoaded(std::size_t max)
{
    std::vector<Loaded> out;
    const std::lock_guard<std::mutex> lock(mu_);
    while (!loaded_.empty() && out.size() < max) {
        out.push_back(std::move(loaded_.front()));
        loaded_.pop_front();
    }
    return out;
}

bool MosaicWorker::StartExport(Export job, std::string* why)
{
    {
        const std::lock_guard<std::mutex> lock(mu_);
        if (pending_ || progress_.active) {
            if (why) *why = "an export is already running";
            return false;
        }
        cancel_           = false;
        pending_          = std::make_unique<Export>(std::move(job));
        progress_.active  = true;
        progress_.frac    = 0.0f;
        progress_.what    = "starting";
    }
    cv_.notify_all();
    return true;
}

void MosaicWorker::CancelExport() { cancel_ = true; }

MosaicWorker::Progress MosaicWorker::GetProgress() const
{
    const std::lock_guard<std::mutex> lock(mu_);
    return progress_;
}

void MosaicWorker::SetWhat(const std::string& what, float frac)
{
    const std::lock_guard<std::mutex> lock(mu_);
    progress_.what = what;
    progress_.frac = frac;
}

void MosaicWorker::Main()
{
    for (;;) {
        std::unique_ptr<Export> job;
        {
            std::unique_lock<std::mutex> lock(mu_);
            cv_.wait(lock, [&] { return quit_ || !loads_.empty() || pending_; });
            if (quit_) return;
            job = std::move(pending_);
        }
        ServiceLoads();
        if (!job) continue;

        std::string result, outPath;
        MosaicMapFit fit;
        bool ok = false;
        try {
            ok = job->kind == JobKind::FitMap ? FitMap(*job, &result, &fit)
                                              : RunExport(*job, &result, &outPath);
        } catch (const std::exception& e) {   // bad_alloc on a monster export, mostly
            result = std::string("export failed: ") + e.what();
        }
        if (ok) FD_INFO("mosaic", "%s", result.c_str());
        else    FD_WARN("mosaic", "%s", result.c_str());
        const std::lock_guard<std::mutex> lock(mu_);
        progress_.active  = false;
        progress_.frac    = ok ? 1.0f : progress_.frac;
        progress_.what.clear();
        progress_.kind    = job->kind;
        progress_.ok      = ok;
        progress_.result  = result;
        progress_.outPath = outPath;
        progress_.fit     = fit;
        ++progress_.done;
    }
}

void MosaicWorker::ServiceLoads()
{
    for (;;) {
        LoadReq req;
        {
            const std::lock_guard<std::mutex> lock(mu_);
            if (quit_ || loads_.empty()) return;
            req = std::move(loads_.front());
            loads_.pop_front();
        }
        Loaded l;
        l.gen   = req.gen;
        l.tile  = req.tile;
        l.level = req.level;
        l.ok    = DecodeRgb(req.path, &l.rgb, &l.w, &l.h);
        if (!l.ok) FD_WARN("mosaic", "cannot read %s", req.path.c_str());
        const std::lock_guard<std::mutex> lock(mu_);
        loaded_.push_back(std::move(l));
    }
}

bool MosaicWorker::RunExport(const Export& job, std::string* result, std::string* outPath)
{
    if (job.rec.tiles.empty()) {
        *result = "the mosaic has no tiles";
        return false;
    }
    if (job.kind == JobKind::Refine) return RefineSeams(job, result, outPath);
    std::error_code ec;
    fs::create_directories(fs::path(job.dir) / "export", ec);
    if (ec) {
        *result = "cannot create " + (fs::path(job.dir) / "export").string() + ": " + ec.message();
        return false;
    }
    return job.kind == JobKind::Crops ? ExportCrops(job, result, outPath)
                                      : ExportPicture(job, result, outPath);
}

bool MosaicWorker::ExportCrops(const Export& job, std::string* result, std::string* outPath)
{
    const fs::path out = fs::path(job.dir) / "export" / "crops";
    std::error_code ec;
    fs::create_directories(out, ec);
    if (job.labels.empty()) {
        *result = "no crop labels to export";
        return false;
    }
    const std::vector<MosaicTileRef> refs = MosaicTileRefs(job.rec, job.useRefined);
    TileCache cache{job.rec, job.dir, job.useRefined, {}, {}};
    std::set<std::string> used;
    int written = 0;
    std::string failed;
    std::vector<unsigned char> rgb;
    for (std::size_t i = 0; i < job.labels.size(); ++i) {
        if (cancel_) {
            *result = "crop export cancelled";
            return false;
        }
        const MosaicLabel& l = job.labels[i];
        SetWhat("crop '" + l.name + "'",
                static_cast<float>(i) / static_cast<float>(job.labels.size()));
        ServiceLoads();
        const int x0 = static_cast<int>(std::floor(l.x0)), y0 = static_cast<int>(std::floor(l.y0));
        const int w  = std::max(1, static_cast<int>(std::ceil(l.x1)) - x0);
        const int h  = std::max(1, static_cast<int>(std::ceil(l.y1)) - y0);
        if (static_cast<double>(w) * h > kMaxCropPixels) {
            failed += (failed.empty() ? "" : ", ") + l.name + " (too big)";
            continue;
        }
        cache.KeepOnly(x0, y0, x0 + w, y0 + h);
        ComposeMosaicRegion(refs, x0, y0, w, h, 1,
                            [&](std::size_t t) { return cache.Get(t); }, &rgb);

        // One file per label, named after it, in a folder per tag (crops/ for
        // none) - a tagged set exports as a ready-sorted dataset. A repeated
        // name gets a suffix rather than silently overwriting the first.
        const fs::path folder = l.tag.empty() ? out : out / MosaicSafeName(l.tag);
        if (!l.tag.empty()) fs::create_directories(folder, ec);
        const std::string base = MosaicSafeName(l.name);
        std::string name = base;
        const auto key = [&](const std::string& n) { return (folder / n).string(); };
        for (int n = 2; used.count(key(name)); ++n) name = base + "_" + std::to_string(n);
        used.insert(key(name));
        const std::string path = (folder / (name + ".png")).string();
        if (stbi_write_png(path.c_str(), w, h, 3, rgb.data(), w * 3)) ++written;
        else failed += (failed.empty() ? "" : ", ") + l.name;
    }
    *outPath = out.string();
    char b[256];
    std::snprintf(b, sizeof(b), "%d of %zu label crops written to %s", written, job.labels.size(),
                  out.string().c_str());
    *result = b;
    if (!failed.empty()) *result += " - failed: " + failed;
    return failed.empty();
}

bool MosaicWorker::ExportPicture(const Export& job, std::string* result, std::string* outPath)
{
    const int s = std::clamp(job.scale, 1, 64);
    double ex0 = 0.0, ey0 = 0.0, ex1 = 0.0, ey1 = 0.0;
    MosaicRecordExtent(job.rec, &ex0, &ey0, &ex1, &ey1, job.useRefined);
    const int sx0 = static_cast<int>(std::floor(ex0)), sy0 = static_cast<int>(std::floor(ey0));
    const int srcW = static_cast<int>(std::ceil(ex1)) - sx0;
    const int srcH = static_cast<int>(std::ceil(ey1)) - sy0;
    const int outW = (srcW + s - 1) / s, outH = (srcH + s - 1) / s;
    if (outW <= 0 || outH <= 0) {
        *result = "the mosaic covers no pixels";
        return false;
    }
    const std::string path =
        (fs::path(job.dir) / "export" / ("mosaic_1to" + std::to_string(s) + "_" + Stamp() + ".png"))
            .string();
    const std::vector<MosaicTileRef> refs = MosaicTileRefs(job.rec, job.useRefined);
    TileCache cache{job.rec, job.dir, job.useRefined, {}, {}};

    const bool inMemory = static_cast<double>(outW) * outH <= kInMemoryPixels;
    std::vector<unsigned char> whole;
    PngStream stream;
    if (inMemory) {
        whole.resize(static_cast<std::size_t>(outW) * outH * 3u);
    } else if (!stream.Open(path, outW, outH)) {
        *result = "cannot write " + path;
        return false;
    }

    const int bandOut = std::max(1, kBandSourceRows / s);
    std::vector<unsigned char> band;
    for (int oy = 0; oy < outH; oy += bandOut) {
        if (cancel_) {
            *result = "mosaic export cancelled";
            if (!inMemory) {
                stream.Close();
                std::error_code ec;
                fs::remove(path, ec);   // half a picture is not a picture
            }
            return false;
        }
        char what[96];
        std::snprintf(what, sizeof(what), "1:%d  row %d of %d", s, oy, outH);
        SetWhat(what, static_cast<float>(oy) / static_cast<float>(outH));
        ServiceLoads();

        const int rows = std::min(bandOut, outH - oy);
        const int y0   = sy0 + oy * s;
        cache.KeepOnly(sx0, y0, sx0 + srcW, y0 + rows * s);
        ComposeMosaicRegion(refs, sx0, y0, outW, rows, s,
                            [&](std::size_t t) { return cache.Get(t); }, &band);
        if (inMemory) {
            std::copy(band.begin(), band.end(),
                      whole.begin() + static_cast<std::ptrdiff_t>(oy) * outW * 3);
        } else if (!stream.Rows(band.data(), rows)) {
            *result = "write failed: " + path;
            return false;
        }
    }
    SetWhat("writing " + path, 1.0f);
    const bool ok = inMemory ? stbi_write_png(path.c_str(), outW, outH, 3, whole.data(), outW * 3) != 0
                             : stream.Close();
    *outPath = path;
    char b[320];
    std::snprintf(b, sizeof(b), "%s mosaic 1:%d (%d x %d px) %s %s", ok ? "wrote" : "FAILED to write",
                  s, outW, outH, ok ? "to" : "", path.c_str());
    *result = b;
    return ok;
}

MosaicRefineInput MosaicWorker::SeamInput(const Export& job, int searchPx) const
{
    // Always from the STAGE positions: a second refinement re-measures, it
    // never compounds on the first; and the map fit must see what the stage
    // did, not what an earlier refinement made of it.
    MosaicRefineInput in;
    in.tiles = MosaicTileRefs(job.rec, false);
    for (const MosaicTileRec& t : job.rec.tiles) in.rowCol.emplace_back(t.row, t.col);
    // The quarter previews every capture files, unless the frames are so small
    // that a quarter of the overlap is too thin to match in (the simulated
    // 640 x 480 camera, a 1280 x 1024 one): then the full frames.
    const bool small = job.rec.imgW > 0 && job.rec.imgW <= 1280;
    in.scale    = small ? 1 : 4;
    in.searchPx = searchPx;
    in.minScore = 0.6;
    return in;
}

bool MosaicWorker::SeamGray(const Export& job, const MosaicRefineInput& in, std::size_t i,
                            std::vector<unsigned char>* out, int* w, int* h)
{
    char what[64];
    std::snprintf(what, sizeof(what), "seams: tile %zu of %zu", i + 1, job.rec.tiles.size());
    SetWhat(what, static_cast<float>(i + 1) / static_cast<float>(job.rec.tiles.size()));
    ServiceLoads();
    const MosaicTileRec& t = job.rec.tiles[i];
    const std::string& rel = in.scale == 1 ? t.file : t.quarter;
    if (rel.empty()) return false;
    std::vector<unsigned char> rgb;
    int rw = 0, rh = 0;
    if (!DecodeRgb((fs::path(job.dir) / rel).string(), &rgb, &rw, &rh)) return false;
    if (rw != (t.w + in.scale - 1) / in.scale || rh != (t.h + in.scale - 1) / in.scale)
        return false;   // not this frame at that scale: its offsets would be a lie
    RgbToGray8(rgb.data(), rw, rh, FocusRoi{}, *out);
    *w = rw;
    *h = rh;
    return true;
}

bool MosaicWorker::FitMap(const Export& job, std::string* result, MosaicMapFit* fit)
{
    if (job.rec.tiles.size() < 4) {
        *result = "camera map: the mosaic needs at least 2 x 2 tiles";
        return false;
    }
    // A wider search than refinement's: this is the job that exists because
    // the map in the record may be off by a degree or two, and a degree over
    // a 2448 px frame is ~40 px at the seam. Bounded by the overlap, though:
    // the template shrinks by the search on every side, and a 1280 x 1024
    // frame at 15 % has only ~150 px of overlap to shrink.
    const double ovX = job.rec.imgW - job.rec.stepX, ovY = job.rec.imgH - job.rec.stepY;
    const int scale0 = SeamInput(job, 0).scale;
    const int search = std::clamp(static_cast<int>(std::min(ovX, ovY) / 2.0) - 12 * scale0, 16, 96);
    const MosaicRefineInput in = SeamInput(job, search);
    const auto gray = [&](std::size_t i, std::vector<unsigned char>* out, int* w, int* h) {
        return SeamGray(job, in, i, out, w, h);
    };
    int candidates = 0;
    bool stopped = false;
    const std::vector<MosaicPairMeasure> pairs =
        MeasureMosaicPairs(in, gray, [&] { return cancel_.load(); }, &candidates, &stopped);
    if (stopped) {
        *result = "camera map fit cancelled";
        return false;
    }
    std::vector<std::pair<double, double>> gantry;
    for (const MosaicTileRec& t : job.rec.tiles) gantry.emplace_back(t.measX, t.measY);
    *fit = FitMosaicCameraMap(pairs, gantry, candidates);
    if (!fit->ok) {
        *result = "camera map: " + fit->why;
        return false;
    }
    // A scale far from the one the capture used is a repeating pattern
    // matched one period off, not a camera that changed lenses.
    double sx0 = 0.0, sy0 = 0.0;
    MosaicMapDescribe(job.rec.geom.m, nullptr, nullptr, &sx0, &sy0);
    const double rx = fit->scaleX / sx0, ry = fit->scaleY / sy0;
    if (!(rx > 0.7 && rx < 1.4 && ry > 0.7 && ry < 1.4)) {
        char b[200];
        std::snprintf(b, sizeof(b), "camera map: fitted %.6f x %.6f mm/px is too far from the "
                      "%.6f x %.6f the capture used - a repeating pattern?", fit->scaleX,
                      fit->scaleY, sx0, sy0);
        *result   = b;
        fit->ok   = false;
        fit->why  = b;
        return false;
    }
    char b[256];
    std::snprintf(b, sizeof(b), "camera map fitted to %d of %d seams (%.2f px rms): rotation "
                  "%.3f deg, skew %.3f deg, %.6f x %.6f mm/px - review it under Camera map",
                  fit->used, fit->pairs, fit->rmsPx, fit->rotDeg, fit->skewDeg, fit->scaleX,
                  fit->scaleY);
    *result = b;
    return true;
}

bool MosaicWorker::RefineSeams(const Export& job, std::string* result, std::string* outPath)
{
    const MosaicRefineInput in = SeamInput(job, 24);
    const auto gray = [&](std::size_t i, std::vector<unsigned char>* out, int* w, int* h) {
        return SeamGray(job, in, i, out, w, h);
    };
    const MosaicRefineResult r = RefineMosaicPlacement(in, gray, [&] { return cancel_.load(); });
    if (!r.ok) {
        *result = "seam refinement: " + r.why;
        return false;
    }
    char note[200];
    std::snprintf(note, sizeof(note), "%d of %d seams matched, residual %.2f px rms, largest "
                  "correction %.1f px", r.used, r.pairs, r.rmsPx, r.maxMovePx);
    if (!SaveMosaicPlacement(job.dir, r.x, r.y, note)) {
        *result = "seam refinement: cannot write placement.json";
        return false;
    }
    *outPath = (fs::path(job.dir) / kMosaicPlacementFile).string();
    *result  = std::string("seams refined: ") + note;
    return true;
}

} // namespace firedog
