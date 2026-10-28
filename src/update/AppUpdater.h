// AppUpdater.h : software update from the GitHub Releases of komcat/FireDog,
// through Velopack. Design: docs/software-update-design.md.
//
// A station installed by the release's Setup.exe (or its Portable.zip) runs
// from <install>\current\. An update downloads beside it, and the install is
// applied only AFTER FireDog has exited — main() hands it to Velopack's
// Update.exe once App (and with it every device) is gone. Update.exe then
// replaces current\ as a whole and starts the new FireDog.exe. The machine
// root is never touched, and FireDog.ini lives one level above current\
// (core/Paths) so the swap cannot take it.
//
// A copy that Setup did not install (a build tree, a hand-unzipped folder)
// cannot update itself; the page says so rather than offering a button.
//
// Threading: ONE worker, owned and joined by the destructor. The network calls
// (check, download) run there; the UI only reads Status() and posts requests.
// Velopack's download cannot be cancelled, so closing FireDog mid-download
// waits for it to finish — the exit confirm says so.
#pragma once

#include <memory>
#include <string>

namespace firedog {

class AppUpdater {
public:
    enum class State {
        NotInstalled,   // not a Velopack install: nothing to update
        Idle,           // installed, never checked
        Checking,
        UpToDate,
        Available,      // a newer release exists (latestVersion)
        Downloading,    // progress 0..100
        Ready,          // downloaded and verified; applies on Restart
        Failed,         // the last check or download failed (message)
    };

    struct Status {
        State       state = State::Idle;
        std::string installedVersion;   // Velopack's, "0.14.0"; "" when not installed
        std::string latestVersion;      // what the last check found newer
        std::string notes;              // its release notes (Markdown, as published)
        std::string message;            // why NotInstalled / Failed
        std::string checkedAt;          // local time of the last finished check
        int         progress = 0;       // while Downloading
        bool        restartRequested = false;
    };

    // Velopack's install / update / uninstall hooks. FIRST in main(): during a
    // Setup or an update FireDog.exe is started with a hook argument, does what
    // it asks and EXITS here, before anything opens a file. A downloaded
    // update is never applied silently at start-up: only Restart installs.
    static void RunStartupHook();

    AppUpdater();
    ~AppUpdater();
    AppUpdater(const AppUpdater&)            = delete;
    AppUpdater& operator=(const AppUpdater&) = delete;

    // Locate the install (no network), start the worker and queue one quiet
    // check. Call once, after logging is up.
    void Start();

    Status GetStatus() const;

    // False (and nothing queued) when not installed or a job is in flight.
    bool RequestCheck();
    bool RequestDownload();   // only from Available

    // From Ready: mark the update to be applied when FireDog exits. The caller
    // then closes the application through its normal exit path.
    bool RequestRestart();

    // main(), AFTER App has been destroyed: when a restart was requested, start
    // Update.exe, which waits for this process to end, installs and relaunches.
    // Never throws — a failure means FireDog exits without updating.
    void ApplyAtExit();

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

const char* UpdateStateName(AppUpdater::State s);

} // namespace firedog
