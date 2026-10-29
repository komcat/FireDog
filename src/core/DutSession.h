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
#include <vector>

#include <nlohmann/json.hpp>

namespace firedog {

class DutSession {
public:
    static DutSession& Instance();

    // App wires these on product load/unload; switching products auto-closes
    // the active DUT (its record is already current on disk).
    void SetProduct(const std::string& name, const std::string& uid,
                    const std::string& dataDir);

    // Who is at the keyboard, wired by App at login/logout (the ONE writer —
    // see App::LogIn). The record carries it two ways, and both are needed:
    // the top-level object answers "whose part is this" at a glance, while a
    // $operator ENTRY at open and at every handover keeps the append-only log
    // honest about a record that changed hands mid-build. Empty id = nobody,
    // which is only reachable before the first login.
    void SetOperator(const std::string& id, const std::string& name,
                     const std::string& level);

    struct Info {
        bool        productSet = false;
        bool        open = false;
        std::string product, serial, openedAt, status;
        std::size_t entries = 0;
    };
    Info GetInfo() const;

    // Read-only views of the App wiring, for records written outside the DUT
    // (the xcan scan log): the loaded product's uid, and the signed-in
    // operator as {id, name, level} — null when nobody is signed in.
    std::string    ProductUid() const;
    nlohmann::json OperatorInfo() const;

    // Folder for the UI's "open data folder" shortcut: the open record's own
    // folder, else the product's data/ root ("" when no product is set).
    std::string BrowseDir() const;

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
    // states and the ProgramRunner. Deliberately a soft no-op when no DUT is
    // open: testing without a part must not fail runs. `uid` (program uid)
    // and `recipesDone` (>= 0) are included when provided.
    void RecordRun(const std::string& kind, const std::string& name,
                   const std::string& result, const std::string& uid = std::string(),
                   int recipesDone = -1);

    // Every record that CLOSED — by Close, or auto-closed by the next Open — for
    // the OEE quality ratio (core/OeeTracker). Close runs on engine workers as
    // well as the UI, so it is queued here and App::Update drains it. `status`
    // is "pass" / "fail", or whatever else the close said (not judged).
    struct ClosedUnit {
        std::string serial, product, status;
    };
    std::vector<ClosedUnit> TakeClosedUnits();

private:
    DutSession() = default;
    DutSession(const DutSession&) = delete;
    DutSession& operator=(const DutSession&) = delete;

    // All assume mu_ is held.
    bool AppendLocked(const std::string& key, const char* type,
                      nlohmann::json value, std::string* err);
    nlohmann::json OperatorJson() const;        // {id, name, level}
    bool SaveLocked(std::string* err);          // atomic temp+rename
    void CloseLocked(const std::string& statusValue);   // best-effort ($close)

    mutable std::mutex mu_;

    // Product wiring (set by App).
    std::string prodName_, prodUid_, dataDir_;

    // Operator wiring (set by App at login). Not part of the active record: it
    // outlives any one DUT, exactly like the product.
    std::string opId_, opName_, opLevel_;

    // Active record.
    bool        open_ = false;
    std::string serial_, openedAt_, closedAt_, status_;
    std::string dirPath_, filePath_;
    nlohmann::json entries_ = nlohmann::json::array();
    std::size_t    seq_ = 0;

    std::vector<ClosedUnit> closedUnits_;   // drained by TakeClosedUnits
};

} // namespace firedog
