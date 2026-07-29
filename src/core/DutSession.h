// DutSession.h : layer core — the active DUT (device under test / part being
// produced), the traceability record everything else writes into.
//
// One active DUT process-wide (bobcat's proven model), identity =
// (product, serial). Process-wide singleton like DataBus/Logger — recipes,
// scripts, and the UI all touch it, so every method is thread-safe.
//
// The record is an append-only entry log persisted after EVERY mutation via
// write-temp-then-rename (a crash can never truncate it), stored inside the
// loaded product's data folder: products/<name>/data/<serial>/dut.json
// (+ images/ reserved for future capture commands). A loaded product is
// REQUIRED — the product folder is the traceability container.
//
// Lifecycle: Open (load-or-create by default; destructiveNew starts a fresh
// record) auto-closes any previous DUT, but only AFTER the new identity
// validates — a failed open never loses the current record. Close stamps an
// end timestamp and an optional pass/fail status. Design: docs/dut-design.md.
#pragma once

#include <cstddef>
#include <mutex>
#include <string>

#include <nlohmann/json.hpp>

namespace firedog {

class DutSession {
public:
    static DutSession& Instance();

    // App wires these on product load/unload; switching products auto-closes
    // the active DUT (its record is already current on disk).
    void SetProduct(const std::string& name, const std::string& uid,
                    const std::string& dataDir);

    struct Info {
        bool        productSet = false;
        bool        open = false;
        std::string product, serial, openedAt, status;
        std::size_t entries = 0;
    };
    Info GetInfo() const;

    // Lifecycle. Serial must be filesystem-safe (alnum - _ .). Open validates
    // and loads/creates the NEW record before auto-closing the previous one.
    bool Open(const std::string& serial, bool destructiveNew, std::string* err);
    bool Close(const std::string& status, std::string* err);   // ""|"pass"|"fail"

    // Append one typed entry (type derived from the json value) and flush.
    // Fails hard when no DUT is open — no silent no-ops.
    bool Insert(const std::string& key, const nlohmann::json& value,
                std::string* err);

    // Image support (dut_capture_image): the record's images/ folder while a
    // DUT is open ("" otherwise), and a type="image" entry whose value is the
    // record-relative file path — images stay LINKED to the record (bobcat's
    // were joined by folder convention alone).
    std::string ImagesDir() const;
    bool InsertImage(const std::string& key, const std::string& relPath,
                     std::string* err);

    // Automatic run attribution ($run event) from the engines' terminal
    // states. Deliberately a soft no-op when no DUT is open: testing without
    // a part must not fail runs.
    void RecordRun(const std::string& kind, const std::string& name,
                   const std::string& result);

private:
    DutSession() = default;
    DutSession(const DutSession&) = delete;
    DutSession& operator=(const DutSession&) = delete;

    // All assume mu_ is held.
    bool AppendLocked(const std::string& key, const char* type,
                      nlohmann::json value, std::string* err);
    bool SaveLocked(std::string* err);          // atomic temp+rename
    void CloseLocked(const std::string& statusValue);   // best-effort ($close)

    mutable std::mutex mu_;

    // Product wiring (set by App).
    std::string prodName_, prodUid_, dataDir_;

    // Active record.
    bool        open_ = false;
    std::string serial_, openedAt_, closedAt_, status_;
    std::string dirPath_, filePath_;
    nlohmann::json entries_ = nlohmann::json::array();
    std::size_t    seq_ = 0;
};

} // namespace firedog
