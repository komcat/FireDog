// AppUpdater.cpp : the only TU that includes a Velopack header.
#include "update/AppUpdater.h"

#include "core/Log.h"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <ctime>
#include <exception>
#include <mutex>
#include <optional>
#include <thread>

// Velopack's generated C++ wrapper is not written for /W4.
#pragma warning(push, 0)
#include "Velopack.hpp"
#pragma warning(pop)

namespace firedog {

namespace {

// The update feed: this repository's GitHub Releases, where /release uploads
// the Velopack packages beside the zip. Public, so no token; an unauthenticated
// check costs a request or two of GitHub's 60 per hour per public IP.
constexpr const char* kRepoUrl = "https://github.com/komcat/FireDog";

std::string Now()
{
    const std::time_t t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::tm tm{};
    localtime_s(&tm, &t);
    char buf[32];
    std::strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M", &tm);
    return buf;
}

} // namespace

const char* UpdateStateName(AppUpdater::State s)
{
    switch (s) {
        case AppUpdater::State::NotInstalled: return "not installed";
        case AppUpdater::State::Idle:         return "idle";
        case AppUpdater::State::Checking:     return "checking";
        case AppUpdater::State::UpToDate:     return "up to date";
        case AppUpdater::State::Available:    return "update available";
        case AppUpdater::State::Downloading:  return "downloading";
        case AppUpdater::State::Ready:        return "ready to install";
        case AppUpdater::State::Failed:       return "failed";
    }
    return "?";
}

struct AppUpdater::Impl {
    enum class Job { None, Check, Download };

    mutable std::mutex      mu;
    std::condition_variable cv;
    Status                  st;          // under mu, except progress (below)
    std::atomic<int>        progress{0};
    Job                     job  = Job::None;
    bool                    quit = false;
    std::thread             worker;

    // Touched by the worker while a job runs, and by ApplyAtExit only once the
    // state is Ready (no job can be in flight then).
    std::unique_ptr<Velopack::UpdateManager> mgr;
    std::optional<Velopack::UpdateInfo>      info;

    static void OnProgress(void* user, size_t pct)
    {
        static_cast<Impl*>(user)->progress.store(static_cast<int>(pct));
    }

    void Run()
    {
        for (;;) {
            Job j;
            {
                std::unique_lock<std::mutex> lk(mu);
                cv.wait(lk, [&] { return quit || job != Job::None; });
                if (quit) return;
                j = job;
            }
            if (j == Job::Check) DoCheck();
            else                 DoDownload();
            std::lock_guard<std::mutex> lk(mu);
            job = Job::None;
        }
    }

    void DoCheck()
    {
        try {
            std::optional<Velopack::UpdateInfo> found = mgr->CheckForUpdates();
            std::lock_guard<std::mutex> lk(mu);
            st.checkedAt = Now();
            st.message.clear();
            if (found) {
                st.state         = State::Available;
                st.latestVersion = found->TargetFullRelease.Version;
                st.notes         = found->TargetFullRelease.NotesMarkdown;
                info             = std::move(found);
                FD_INFO("update", "FireDog %s is available (installed %s)",
                        st.latestVersion.c_str(), st.installedVersion.c_str());
            } else {
                st.state = State::UpToDate;
                st.latestVersion.clear();
                st.notes.clear();
                info.reset();
                FD_INFO("update", "FireDog %s is up to date", st.installedVersion.c_str());
            }
        } catch (const std::exception& e) {
            std::lock_guard<std::mutex> lk(mu);
            st.state     = State::Failed;
            st.message   = std::string("check failed: ") + e.what();
            st.checkedAt = Now();
            // Info, not a warning: a station with no internet fails this on
            // every start, and that is how most of them are set up.
            FD_INFO("update", "%s", st.message.c_str());
        }
    }

    void DoDownload()
    {
        Velopack::UpdateInfo target;
        {
            std::lock_guard<std::mutex> lk(mu);
            target = *info;
        }
        try {
            // Delta when the previous package is on disk, else the full one;
            // Velopack checks the SHA against the feed either way.
            mgr->DownloadUpdates(target, &Impl::OnProgress, this);
            std::lock_guard<std::mutex> lk(mu);
            st.state = State::Ready;
            st.message.clear();
            FD_INFO("update", "FireDog %s downloaded - installs on Restart",
                    target.TargetFullRelease.Version.c_str());
        } catch (const std::exception& e) {
            std::lock_guard<std::mutex> lk(mu);
            st.state   = State::Failed;
            st.message = std::string("download failed: ") + e.what();
            FD_WARN("update", "%s", st.message.c_str());
        }
    }

    bool Post(Job j)
    {
        {
            std::lock_guard<std::mutex> lk(mu);
            if (!mgr || job != Job::None) return false;
            if (j == Job::Download && (st.state != State::Available || !info)) return false;
            job = j;
            if (j == Job::Check) {
                st.state = State::Checking;
            } else {
                st.state = State::Downloading;
                progress.store(0);
            }
        }
        cv.notify_one();
        return true;
    }
};

void AppUpdater::RunStartupHook()
{
    Velopack::VelopackApp::Build().SetAutoApplyOnStartup(false).Run();
}

AppUpdater::AppUpdater() : impl_(std::make_unique<Impl>()) {}

AppUpdater::~AppUpdater()
{
    {
        std::lock_guard<std::mutex> lk(impl_->mu);
        impl_->quit = true;
    }
    impl_->cv.notify_one();
    // A download in flight cannot be cancelled: this waits for it.
    if (impl_->worker.joinable()) impl_->worker.join();
}

void AppUpdater::Start()
{
    try {
        impl_->mgr = std::make_unique<Velopack::UpdateManager>(
            std::make_unique<Velopack::GithubSource>(kRepoUrl, "", false));
    } catch (const std::exception& e) {
        std::lock_guard<std::mutex> lk(impl_->mu);
        impl_->st.state   = State::NotInstalled;
        impl_->st.message = e.what();
        FD_INFO("update", "software update off - not a Setup install (%s)", e.what());
        return;
    }
    {
        std::lock_guard<std::mutex> lk(impl_->mu);
        std::string v = impl_->mgr->GetCurrentVersion();
        while (!v.empty() && v.back() == '\0') v.pop_back();   // the C buffer's terminator
        impl_->st.installedVersion = std::move(v);
        impl_->st.state            = State::Idle;
    }
    FD_INFO("update", "installed FireDog %s; updates from %s",
            impl_->st.installedVersion.c_str(), kRepoUrl);
    impl_->worker = std::thread([this] { impl_->Run(); });
    impl_->Post(Impl::Job::Check);   // the quiet first look
}

AppUpdater::Status AppUpdater::GetStatus() const
{
    std::lock_guard<std::mutex> lk(impl_->mu);
    Status s   = impl_->st;
    s.progress = impl_->progress.load();
    return s;
}

bool AppUpdater::RequestCheck()    { return impl_->Post(Impl::Job::Check); }
bool AppUpdater::RequestDownload() { return impl_->Post(Impl::Job::Download); }

bool AppUpdater::RequestRestart()
{
    std::lock_guard<std::mutex> lk(impl_->mu);
    if (impl_->st.state != State::Ready) return false;
    impl_->st.restartRequested = true;
    return true;
}

void AppUpdater::ApplyAtExit()
{
    Velopack::UpdateInfo target;
    {
        std::lock_guard<std::mutex> lk(impl_->mu);
        if (!impl_->st.restartRequested || impl_->st.state != State::Ready || !impl_->info)
            return;
        target = *impl_->info;
    }
    try {
        // Update.exe waits up to 60 s for this process to end — which is why
        // this runs after App, and every device it owned, is already gone.
        impl_->mgr->WaitExitThenApplyUpdates(target, false, true);
        FD_INFO("update", "Update.exe started - installing FireDog %s after exit",
                target.TargetFullRelease.Version.c_str());
    } catch (const std::exception& e) {
        FD_ERROR("update", "could not start the update: %s", e.what());
    }
}

} // namespace firedog
