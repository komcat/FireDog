// ExecutionContext.cpp
#include "command/ExecutionContext.h"

#include "core/DataBus.h"
#include "core/Log.h"
#include "core/Paths.h"     // MachinePath — the machine's script folder
#include "core/Product.h"   // GenerateUid — a newly taught position's identity
#include "hardware/HardwareManager.h"
#include "workflow/Workflow.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <system_error>
#include <thread>

namespace firedog {

std::string CtxValueToString(const CtxValue& v)
{
    if (std::holds_alternative<bool>(v)) return std::get<bool>(v) ? "true" : "false";
    if (std::holds_alternative<double>(v)) {
        char buf[32];
        std::snprintf(buf, sizeof(buf), "%g", std::get<double>(v));
        return buf;
    }
    return std::get<std::string>(v);
}

double CtxNumber(const ExecutionContext& ctx, const std::string& key, bool* found)
{
    const auto v = ctx.CtxGet(key);
    if (!v || !std::holds_alternative<double>(*v)) { if (found) *found = false; return 0.0; }
    if (found) *found = true;
    return std::get<double>(*v);
}

ExecutionContext::ExecutionContext(Workflow& wf, HardwareManager& hw,
                                   SavedPosMap& positions,
                                   const MotionMapSet& maps,
                                   const std::vector<AlignProc>& aligns,
                                   const std::vector<VisionModel>& visionModels,
                                   const std::vector<GripProfile>& grips,
                                   const std::vector<WaffleTray>& trays,
                                   std::string visionDir, std::string scriptsDir,
                                   Hooks hooks)
    : wf_(wf), hw_(hw), positions_(positions), maps_(maps), aligns_(aligns),
      visionModels_(visionModels), grips_(grips), trays_(trays),
      visionDir_(std::move(visionDir)),
      scriptsDir_(std::move(scriptsDir)), hooks_(std::move(hooks))
{
}

bool IsSystemScript(const std::string& pathOrFile)
{
    const std::string file = std::filesystem::path(pathOrFile).filename().string();
    return file == "alignment_procedure.lua" ||
           file == "needle_calibration.lua" || file == "dispense_procedure.lua" ||
           file == "alignment_repeat.lua" ||
           file == "device_reg_check.lua" || file == "grip_measure.lua" ||
           file == "xcan_to_target.lua" || file == "xcan_two_lens.lua";
}

std::string SystemScriptPath(const std::string& pathOrFile)
{
    if (!IsSystemScript(pathOrFile)) return std::string();
    return LocateShippedScript("config/scripts/" +
                               std::filesystem::path(pathOrFile).filename().string());
}

std::string MachineScriptsDir() { return MachinePath("config/scripts"); }

std::string LocateShippedScript(const std::string& path)
{
    std::error_code ec;
    if (path.empty() || std::filesystem::exists(path, ec)) return path;
    const std::filesystem::path p(path);
    if (p.is_absolute() || p.parent_path().generic_string() != "config/scripts") return path;
    const std::string machine = MachineScriptsDir() + "/" + p.filename().string();
    return std::filesystem::exists(machine, ec) ? machine : path;
}

bool IsShippedScript(const std::string& path)
{
    std::error_code ec;
    return std::filesystem::equivalent(std::filesystem::path(path).parent_path(),
                                       "config/scripts", ec);
}

std::string ExecutionContext::ResolveScript(const std::string& path) const
{
    if (path.empty()) return path;
    // Match on the FILENAME, not the whole path: the caller's default names the
    // shipped copy, and what we are asking is "does this product have its own
    // version of that script".
    const std::string file = std::filesystem::path(path).filename().string();
    if (file.empty()) return path;

    // A SYSTEM procedure is never taken from a product, and never from wherever
    // the caller pointed either: these two files are edited in lockstep with the
    // commands they call, so "the alignment procedure" has to mean one file on
    // every machine. The warning names the copy being ignored — silently
    // redirecting would be the same trap in the other direction.
    if (const std::string sys = SystemScriptPath(file); !sys.empty()) {
        std::error_code ec;
        // The string check first: a MISSING shipped file makes equivalent() say
        // "different", and warning that the default is being ignored in favour
        // of itself would send the reader looking for a copy nobody made. The
        // same goes for a shipped spelling that located the machine root's copy.
        if (path != sys && LocateShippedScript(path) != sys &&
            !std::filesystem::equivalent(path, sys, ec))
            FD_WARN("script", "'%s' is a standard procedure - running the shipped %s "
                              "and ignoring %s", file.c_str(), sys.c_str(), path.c_str());
        return sys;
    }

    // The product's copy, then the machine's, then what the caller named.
    const std::string machineDir = MachineScriptsDir();
    for (const std::string& dir : {scriptsDir_, machineDir}) {
        if (dir.empty()) continue;
        const std::string candidate = dir + "/" + file;

        std::error_code sameEc;
        if (std::filesystem::equivalent(candidate, path, sameEc)) return path;   // same file

        std::error_code ec;
        if (!std::filesystem::exists(candidate, ec) || ec) continue;
        FD_INFO("script", "using the %s copy of %s (%s)",
                dir == machineDir ? "machine's" : "product's", file.c_str(), candidate.c_str());
        return candidate;
    }
    return path;
}

void ExecutionContext::RecordPosition(const std::string& device,
                                      const std::string& name,
                                      const std::map<std::string, double>& vals)
{
    std::vector<SavedPos>& list = positions_[device];
    SavedPos* tgt = nullptr;
    for (SavedPos& sp : list)
        if (name == sp.name) { tgt = &sp; break; }
    if (!tgt) {
        SavedPos np;
        std::snprintf(np.name, sizeof(np.name), "%s", name.c_str());
        np.uid = GenerateUid();
        list.push_back(std::move(np));
        tgt = &list.back();
    }
    tgt->vals = vals;
    wf_.QueueSavedPosition(device, *tgt);
    FD_INFO("recipe", "taught '%s' on %s (%zu axes)", tgt->name, device.c_str(),
            vals.size());
}

bool ExecutionContext::RecordPositionByUid(const std::string& device,
                                           const std::string& uid,
                                           const std::map<std::string, double>& vals)
{
    const auto it = positions_.find(device);
    if (it == positions_.end()) return false;
    for (SavedPos& sp : it->second) {
        if (sp.uid != uid) continue;
        for (const auto& [axis, v] : vals) sp.vals[axis] = v;
        wf_.QueueSavedPosition(device, sp);
        FD_INFO("recipe", "taught '%s' on %s by uid (%zu axes)", sp.name, device.c_str(),
                vals.size());
        return true;
    }
    FD_WARN("recipe", "teach: position uid '%s' not found on %s - nothing written",
            uid.c_str(), device.c_str());
    return false;
}

bool ExecutionContext::ShouldStop() const
{
    return hooks_.shouldStop && hooks_.shouldStop();
}

bool ExecutionContext::Sleep(int ms)
{
    using clock = std::chrono::steady_clock;
    const auto deadline = clock::now() + std::chrono::milliseconds(std::max(ms, 0));
    while (clock::now() < deadline) {
        if (ShouldStop()) return false;
        const auto left = std::chrono::duration_cast<std::chrono::milliseconds>(
            deadline - clock::now());
        std::this_thread::sleep_for(
            std::min<std::chrono::milliseconds>(left, std::chrono::milliseconds(20)));
    }
    return !ShouldStop();
}

ExecutionContext::WaitResult ExecutionContext::WaitUntil(
    const std::function<bool()>& pred, int timeoutMs, int pollMs)
{
    using clock = std::chrono::steady_clock;
    const int poll = std::max(pollMs, 1);
    // Clamp so no wait is ever unbounded (validation should catch this first).
    const auto deadline =
        clock::now() + std::chrono::milliseconds(std::max(timeoutMs, poll));
    for (;;) {
        if (ShouldStop()) return WaitResult::Stopped;
        if (pred()) return WaitResult::Ok;
        if (clock::now() >= deadline) return WaitResult::Timeout;
        std::this_thread::sleep_for(std::chrono::milliseconds(poll));
    }
}

void ExecutionContext::CtxSet(const std::string& key, CtxValue v)
{
    const std::scoped_lock lock(ctxMu_);
    ctx_[key] = std::move(v);
}

void ExecutionContext::CtxErase(const std::string& key)
{
    const std::scoped_lock lock(ctxMu_);
    ctx_.erase(key);
}

std::optional<CtxValue> ExecutionContext::CtxGet(const std::string& key) const
{
    const std::scoped_lock lock(ctxMu_);
    const auto it = ctx_.find(key);
    if (it == ctx_.end()) return std::nullopt;
    return it->second;
}

std::vector<std::pair<std::string, CtxValue>> ExecutionContext::CtxSnapshot() const
{
    const std::scoped_lock lock(ctxMu_);
    return {ctx_.begin(), ctx_.end()};
}

std::string ExecutionContext::Substitute(const std::string& text) const
{
    std::string out;
    out.reserve(text.size());
    std::size_t i = 0;
    while (i < text.size()) {
        const std::size_t open = text.find('{', i);
        if (open == std::string::npos) {
            out.append(text, i, std::string::npos);
            break;
        }
        const std::size_t close = text.find('}', open + 1);
        if (close == std::string::npos) {
            out.append(text, i, std::string::npos);
            break;
        }
        out.append(text, i, open - i);
        const std::string key = text.substr(open + 1, close - open - 1);
        if (const std::optional<CtxValue> v = CtxGet(key)) {
            out += CtxValueToString(*v);
        } else {
            out.append(text, open, close - open + 1);   // unknown — leave as-is
        }
        i = close + 1;
    }
    return out;
}

int ExecutionContext::MotionIndexByName(const std::string& name) const
{
    for (std::size_t i = 0; i < hw_.MotionCount(); ++i) {
        if (hw_.MotionName(i) == name) return static_cast<int>(i);
    }
    return -1;
}

int ExecutionContext::IoIndexByName(const std::string& name) const
{
    for (std::size_t i = 0; i < hw_.IoCount(); ++i) {
        if (hw_.IoName(i) == name) return static_cast<int>(i);
    }
    return -1;
}

int ExecutionContext::SlideIndexByName(const std::string& name) const
{
    for (std::size_t i = 0; i < hw_.SlideCount(); ++i) {
        if (hw_.Slide(i) && hw_.Slide(i)->Name() == name) return static_cast<int>(i);
    }
    return -1;
}

const SavedPos* ExecutionContext::FindPosition(const std::string& device,
                                               const std::string& uid) const
{
    const auto it = positions_.find(device);
    if (it == positions_.end()) return nullptr;
    for (const SavedPos& p : it->second) {
        if (p.uid == uid) return &p;
    }
    return nullptr;
}

const MotionMap* ExecutionContext::FindMap(const std::string& device,
                                           const std::string& mapName) const
{
    const auto it = maps_.find(device);
    if (it == maps_.end()) return nullptr;
    for (const MotionMap& m : it->second) {
        if (m.name == mapName) return &m;
    }
    return nullptr;
}

const AlignProc* ExecutionContext::FindAlign(const std::string& uidOrName) const
{
    return FindAlignProc(aligns_, uidOrName);
}

const VisionModel* ExecutionContext::FindVisionModel(const std::string& uidOrName) const
{
    return firedog::FindVisionModel(visionModels_, uidOrName);
}

const GripProfile* ExecutionContext::FindGrip(const std::string& uidOrName) const
{
    return firedog::FindGripProfile(grips_, uidOrName);
}

const WaffleTray* ExecutionContext::FindTray(const std::string& uidOrName) const
{
    return firedog::FindWaffleTray(trays_, uidOrName);
}

bool ExecutionContext::AwaitPrompt(const std::string& message)
{
    if (!hooks_.awaitPrompt) return false;
    return hooks_.awaitPrompt(message);
}

bool ExecutionContext::AwaitInput(const std::string& message, std::string* textOut)
{
    if (!hooks_.awaitInput) return false;
    return hooks_.awaitInput(message, textOut);
}

bool ExecutionContext::AwaitAdjust(const AdjustRequest& req)
{
    if (!hooks_.awaitAdjust) return false;
    return hooks_.awaitAdjust(req);
}

bool ExecutionContext::AwaitPick(const PickRequest& req, PickResult* out)
{
    if (!hooks_.awaitPick) return false;
    return hooks_.awaitPick(req, out);
}

// ---- Condition evaluation ---------------------------------------------------

namespace {

const char* const kOps[] = {">", "<", ">=", "<=", "==", "!=",
                            "is_true", "is_false", "exists", "not_exists"};

bool IsKnownOp(const std::string& op)
{
    for (const char* k : kOps) {
        if (op == k) return true;
    }
    return false;
}

} // namespace

std::string CheckConditionShape(const nlohmann::json& cond)
{
    if (!cond.is_object()) return "condition must be an object";
    const bool hasKey   = cond.contains("key") && cond["key"].is_string();
    const bool hasTopic = cond.contains("topic") && cond["topic"].is_string();
    if (hasKey == hasTopic)   // neither, or both
        return "condition needs exactly one of \"key\" or \"topic\"";
    if (!cond.contains("op") || !cond["op"].is_string())
        return "condition needs an \"op\" string";
    const std::string op = cond["op"].get<std::string>();
    if (!IsKnownOp(op)) return "unknown condition op '" + op + "'";
    const bool needsValue = op != "is_true" && op != "is_false" &&
                            op != "exists" && op != "not_exists";
    if (needsValue &&
        (!cond.contains("value") ||
         !(cond["value"].is_number() || cond["value"].is_string() ||
           cond["value"].is_boolean())))
        return "condition op '" + op + "' needs a \"value\"";
    return {};
}

std::optional<bool> EvalCondition(const nlohmann::json& cond, ExecutionContext& ctx,
                                  std::string* err)
{
    const std::string shape = CheckConditionShape(cond);
    if (!shape.empty()) {
        if (err) *err = shape;
        return std::nullopt;
    }
    const std::string op = cond["op"].get<std::string>();

    // Resolve the current value: ctx key or DataBus topic latest sample.
    bool exists = false;
    std::optional<CtxValue> cur;
    if (cond.contains("key")) {
        cur    = ctx.CtxGet(cond["key"].get<std::string>());
        exists = cur.has_value();
    } else {
        DataStream& s = DataBus::Instance().Stream(cond["topic"].get<std::string>());
        exists = s.Count() > 0;
        if (exists) cur = CtxValue(s.LatestValue());
    }

    if (op == "exists")     return exists;
    if (op == "not_exists") return !exists;
    if (!exists) return false;   // comparisons on missing data: false, keep waiting

    if (op == "is_true" || op == "is_false") {
        bool truthy = false;
        if (std::holds_alternative<bool>(*cur))        truthy = std::get<bool>(*cur);
        else if (std::holds_alternative<double>(*cur)) truthy = std::get<double>(*cur) != 0.0;
        else                                           truthy = !std::get<std::string>(*cur).empty();
        return op == "is_true" ? truthy : !truthy;
    }

    const nlohmann::json& rhs = cond["value"];

    // String equality (only == / != make sense for strings).
    if (rhs.is_string() || std::holds_alternative<std::string>(*cur)) {
        if (op != "==" && op != "!=") {
            if (err) *err = "op '" + op + "' needs numeric operands";
            return std::nullopt;
        }
        const std::string a = CtxValueToString(*cur);
        const std::string b = rhs.is_string() ? rhs.get<std::string>() : rhs.dump();
        return op == "==" ? a == b : a != b;
    }

    const double a = std::holds_alternative<bool>(*cur)
                         ? (std::get<bool>(*cur) ? 1.0 : 0.0)
                         : std::get<double>(*cur);
    const double b = rhs.is_boolean() ? (rhs.get<bool>() ? 1.0 : 0.0)
                                      : rhs.get<double>();
    if (op == ">")  return a > b;
    if (op == "<")  return a < b;
    if (op == ">=") return a >= b;
    if (op == "<=") return a <= b;
    if (op == "==") return a == b;
    if (op == "!=") return a != b;

    if (err) *err = "unknown condition op '" + op + "'";
    return std::nullopt;
}

} // namespace firedog
