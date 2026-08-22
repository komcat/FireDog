// DutSession.cpp
#include "core/DutSession.h"

#include "core/Log.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <ctime>
#include <exception>
#include <filesystem>
#include <fstream>

namespace firedog {

namespace {

// UTC ISO-8601 with milliseconds, e.g. 2026-07-28T19:31:04.120Z
std::string NowStamp()
{
    using namespace std::chrono;
    const auto   now = system_clock::now();
    const time_t t   = system_clock::to_time_t(now);
    const int    ms  = static_cast<int>(
        duration_cast<milliseconds>(now.time_since_epoch()).count() % 1000);
    std::tm tm{};
    gmtime_s(&tm, &t);
    char buf[40];
    std::snprintf(buf, sizeof(buf), "%04d-%02d-%02dT%02d:%02d:%02d.%03dZ",
                  tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour,
                  tm.tm_min, tm.tm_sec, ms);
    return buf;
}

bool SerialOk(const std::string& s)
{
    if (s.empty() || s.size() > 64) return false;
    for (const char c : s) {
        const bool ok = (c >= '0' && c <= '9') || (c >= 'a' && c <= 'z') ||
                        (c >= 'A' && c <= 'Z') || c == '-' || c == '_' || c == '.';
        if (!ok) return false;
    }
    return true;
}

const char* TypeOf(const nlohmann::json& v)
{
    if (v.is_boolean())        return "bool";
    if (v.is_number_integer()) return "integer";
    if (v.is_number())         return "double";
    if (v.is_object())         return "object";
    return "string";
}

} // namespace

DutSession& DutSession::Instance()
{
    static DutSession s;
    return s;
}

void DutSession::SetProduct(const std::string& name, const std::string& uid,
                            const std::string& dataDir)
{
    const std::scoped_lock lock(mu_);
    if (open_ && name != prodName_) {
        FD_INFO("dut", "product changed - auto-closing DUT '%s'", serial_.c_str());
        CloseLocked("auto");
    }
    prodName_ = name;
    prodUid_  = uid;
    dataDir_  = dataDir;
}

void DutSession::SetOperator(const std::string& id, const std::string& name,
                             const std::string& level)
{
    const std::scoped_lock lock(mu_);
    if (id == opId_) return;   // a re-login as the same number is not a handover
    opId_    = id;
    opName_  = name;
    opLevel_ = level;

    // A record already open has just changed hands. The top-level object is
    // rewritten by the next save either way, but a record that was half built
    // by one person and finished by another has to SAY so — an append-only log
    // that silently swapped its operator would read as one continuous shift.
    if (open_ && !id.empty()) {
        std::string aerr;
        if (!AppendLocked("$operator", "object", OperatorJson(), &aerr))
            FD_WARN("dut", "operator handover not recorded: %s", aerr.c_str());
    }
}

DutSession::Info DutSession::GetInfo() const
{
    const std::scoped_lock lock(mu_);
    Info i;
    i.productSet = !prodName_.empty();
    i.open       = open_;
    i.product    = prodName_;
    i.serial     = serial_;
    i.openedAt   = openedAt_;
    i.status     = status_;
    i.entries    = entries_.size();
    return i;
}

std::string DutSession::BrowseDir() const
{
    const std::scoped_lock lock(mu_);
    return open_ ? dirPath_ : dataDir_;
}

bool DutSession::Open(const std::string& serial, bool destructiveNew,
                      std::string* err)
{
    const std::scoped_lock lock(mu_);
    if (prodName_.empty()) {
        if (err) *err = "no product loaded - a DUT lives in the product's data folder";
        return false;
    }
    if (!SerialOk(serial)) {
        if (err) *err = "invalid serial '" + serial +
                        "' (use letters, digits, - _ . ; max 64 chars)";
        return false;
    }

    // Build and validate the NEW record fully before touching the current one.
    namespace fs = std::filesystem;
    const std::string dir  = dataDir_ + "/" + serial;
    const std::string file = dir + "/dut.json";

    nlohmann::json newEntries = nlohmann::json::array();
    std::string    newOpened  = NowStamp();
    std::size_t    newSeq     = 0;
    const char*    openMode   = "create";
    if (!destructiveNew && fs::exists(file)) {
        try {
            std::ifstream f(file);
            nlohmann::json j;
            f >> j;
            if (j.contains("entries") && j["entries"].is_array())
                newEntries = j["entries"];
            newOpened = j.value("opened_at", newOpened);
            for (const nlohmann::json& e : newEntries)
                newSeq = std::max<std::size_t>(newSeq, e.value("seq", 0u));
            openMode = "reopen";
        } catch (const std::exception& e) {
            if (err) *err = "existing record for '" + serial +
                            "' is unreadable (" + e.what() +
                            ") - use new=true to start fresh";
            return false;
        }
    }
    std::error_code ec;
    fs::create_directories(dir + "/images", ec);
    if (ec) {
        if (err) *err = "cannot create '" + dir + "': " + ec.message();
        return false;
    }

    // Commit point: auto-close the previous DUT (validated-first, bobcat rule).
    if (open_) {
        FD_INFO("dut", "auto-closing DUT '%s' for new DUT '%s'", serial_.c_str(),
                serial.c_str());
        CloseLocked("auto");
    }

    open_     = true;
    serial_   = serial;
    openedAt_ = newOpened;
    closedAt_ = "";
    status_   = "";
    dirPath_  = dir;
    filePath_ = file;
    entries_  = std::move(newEntries);
    seq_      = newSeq;

    std::string aerr;
    if (!AppendLocked("$open", "event", openMode, &aerr)) {
        open_ = false;
        if (err) *err = aerr;
        return false;
    }
    // Who opened it, in the log itself. Appended rather than folded into $open's
    // value so no existing reader of that entry has to change.
    if (!opId_.empty() && !AppendLocked("$operator", "object", OperatorJson(), &aerr))
        FD_WARN("dut", "operator not recorded on open: %s", aerr.c_str());
    FD_INFO("dut", "DUT '%s' open (%s, %zu entrie(s))", serial_.c_str(), openMode,
            entries_.size());
    return true;
}

bool DutSession::Close(const std::string& status, std::string* err)
{
    const std::scoped_lock lock(mu_);
    if (!open_) {
        if (err) *err = "no DUT is open";
        return false;
    }
    if (!status.empty() && status != "pass" && status != "fail") {
        if (err) *err = "status must be empty, pass, or fail";
        return false;
    }
    CloseLocked(status.empty() ? "closed" : status);
    return true;
}

void DutSession::CloseLocked(const std::string& statusValue)
{
    closedAt_ = NowStamp();
    if (statusValue == "pass" || statusValue == "fail") status_ = statusValue;
    std::string aerr;
    if (!AppendLocked("$close", "event", statusValue, &aerr))
        FD_WARN("dut", "close event write failed: %s", aerr.c_str());
    FD_INFO("dut", "DUT '%s' closed (%s, %zu entrie(s))", serial_.c_str(),
            statusValue.c_str(), entries_.size());
    open_ = false;
}

bool DutSession::Insert(const std::string& key, const nlohmann::json& value,
                        std::string* err)
{
    const std::scoped_lock lock(mu_);
    if (!open_) {
        if (err) *err = "no DUT is open (dut_open first)";
        return false;
    }
    if (key.empty() || key[0] == '$') {
        if (err) *err = "invalid DUT key ('$' prefix is reserved for events)";
        return false;
    }
    return AppendLocked(key, TypeOf(value), value, err);
}

std::string DutSession::ImagesDir() const
{
    const std::scoped_lock lock(mu_);
    return open_ ? dirPath_ + "/images" : std::string();
}

bool DutSession::InsertImage(const std::string& key, const std::string& relPath,
                             std::string* err)
{
    const std::scoped_lock lock(mu_);
    if (!open_) {
        if (err) *err = "no DUT is open (dut_open first)";
        return false;
    }
    if (key.empty() || key[0] == '$') {
        if (err) *err = "invalid DUT key ('$' prefix is reserved for events)";
        return false;
    }
    return AppendLocked(key, "image", relPath, err);
}

void DutSession::RecordRun(const std::string& kind, const std::string& name,
                           const std::string& result, const std::string& uid,
                           int recipesDone)
{
    const std::scoped_lock lock(mu_);
    if (!open_) return;   // deliberate soft path: runs without a part are fine
    nlohmann::json v;
    v["kind"]   = kind;
    v["name"]   = name;
    v["result"] = result;
    if (!uid.empty()) v["uid"] = uid;
    if (recipesDone >= 0) v["recipes_done"] = recipesDone;
    // The "who ran this" answer, on the run rather than only on the record: a
    // record open across a shift change has runs belonging to two people.
    if (!opId_.empty()) v["operator"] = opId_;
    std::string aerr;
    if (!AppendLocked("$run", "object", std::move(v), &aerr))
        FD_WARN("dut", "run event write failed: %s", aerr.c_str());
}

bool DutSession::AppendLocked(const std::string& key, const char* type,
                              nlohmann::json value, std::string* err)
{
    nlohmann::json e;
    e["seq"]   = ++seq_;
    e["t"]     = NowStamp();
    e["key"]   = key;
    e["type"]  = type;
    e["value"] = std::move(value);
    entries_.push_back(std::move(e));
    std::string serr;
    if (!SaveLocked(&serr)) {
        entries_.erase(entries_.end() - 1);   // in-memory rollback
        --seq_;
        if (err) *err = serr;
        return false;
    }
    return true;
}

nlohmann::json DutSession::OperatorJson() const
{
    nlohmann::json o;
    o["id"]    = opId_;
    o["name"]  = opName_;
    o["level"] = opLevel_;
    return o;
}

bool DutSession::SaveLocked(std::string* err)
{
    nlohmann::json j;
    j["product"]     = prodName_;
    j["product_uid"] = prodUid_;
    j["serial"]      = serial_;
    j["opened_at"]   = openedAt_;
    j["closed_at"]   = closedAt_;
    j["status"]      = status_;
    // Omitted rather than written empty when nobody is signed in: a record with
    // no operator key is honestly silent, where {"id":""} looks like a field
    // that was written and lost.
    if (!opId_.empty()) j["operator"] = OperatorJson();
    j["entries"]     = entries_;

    // Atomic-enough on Windows: write a sibling temp file, then rename over
    // the live one (std::filesystem::rename replaces the destination).
    const std::string tmp = filePath_ + ".tmp";
    {
        std::ofstream f(tmp);
        if (!f.is_open()) {
            if (err) *err = "cannot write " + tmp;
            return false;
        }
        f << j.dump(2) << '\n';
        if (!f.good()) {
            if (err) *err = "write failed for " + tmp;
            return false;
        }
    }
    std::error_code ec;
    std::filesystem::rename(tmp, filePath_, ec);
    if (ec) {
        if (err) *err = "rename to " + filePath_ + " failed: " + ec.message();
        return false;
    }
    return true;
}

} // namespace firedog
