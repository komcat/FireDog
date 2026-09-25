// Log.h : layer-0 logging — the lowest common facility.
//
// A thin, spdlog-free wrapper: any layer may log without pulling in spdlog or
// depending on anything above core/. Hardware poll threads, async connect
// lambdas, the workflow, and the UI all call the same process-wide, thread-safe
// Logger. The backend (spdlog) and its sinks live entirely in Log.cpp:
//     - console  (stdout, colored)
//     - file     (daily logs/firedog_YYYY-MM-DD.log, newest 30 days kept)
//     - ringbuffer (in-memory; the ImGui Log window snapshots it each frame)
//
// Use the macros for printf-style call sites:
//     FD_INFO("camera", "connected to %s", ip.c_str());
//     FD_WARN("keithley", "read failed (%d)", code);
#pragma once

#include <atomic>
#include <cstdarg>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

namespace firedog {

enum class LogLevel { Trace, Debug, Info, Warn, Error };

const char* LevelName(LogLevel lvl);

struct LogEntry {
    double      t;         // seconds since logger start
    char        stamp[16]; // local wall-clock "HH:MM:SS.mmm" — what the panel shows
    LogLevel    level;
    std::string text;      // "[source] message"
};

// Process-wide, thread-safe. One instance, created on first use (like DataBus).
class Logger {
public:
    static Logger& Instance();

    // printf-style. Thread-safe; entries below MinLevel() are dropped cheaply.
    void Write(LogLevel level, const char* source, const char* fmt, ...);
    void WriteV(LogLevel level, const char* source, const char* fmt, va_list args);

    std::vector<LogEntry> Snapshot() const;   // copy the ringbuffer for the UI
    std::size_t Count() const;                // monotonic; UI change-detection
    void        Clear();                      // hides prior entries in the viewer

    void     SetMinLevel(LogLevel lvl);
    LogLevel MinLevel() const;

private:
    Logger();
    ~Logger();
    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;

    struct Impl;
    std::unique_ptr<Impl>    p_;                            // spdlog lives here
    std::atomic<LogLevel>    minLevel_{LogLevel::Info};
    std::atomic<std::size_t> total_{0};
};

} // namespace firedog

// ---- Ergonomic call-site macros ----
#define FD_LOG(level, src, ...) ::firedog::Logger::Instance().Write((level), (src), __VA_ARGS__)
#define FD_TRACE(src, ...) FD_LOG(::firedog::LogLevel::Trace, src, __VA_ARGS__)
#define FD_DEBUG(src, ...) FD_LOG(::firedog::LogLevel::Debug, src, __VA_ARGS__)
#define FD_INFO(src, ...)  FD_LOG(::firedog::LogLevel::Info,  src, __VA_ARGS__)
#define FD_WARN(src, ...)  FD_LOG(::firedog::LogLevel::Warn,  src, __VA_ARGS__)
#define FD_ERROR(src, ...) FD_LOG(::firedog::LogLevel::Error, src, __VA_ARGS__)
