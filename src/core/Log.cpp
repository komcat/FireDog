// Log.cpp : spdlog-backed implementation of the layer-0 Logger.
//
// Sinks (configured once, here): colored stdout, a daily file under logs/,
// and an in-memory ringbuffer the ImGui Log window reads via Snapshot().
#include "core/Log.h"

#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/daily_file_sink.h>
#include <spdlog/sinks/ringbuffer_sink.h>

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <ctime>
#include <exception>
#include <vector>

namespace firedog {

const char* LevelName(LogLevel lvl)
{
    switch (lvl) {
        case LogLevel::Trace: return "TRACE";
        case LogLevel::Debug: return "DEBUG";
        case LogLevel::Info:  return "INFO";
        case LogLevel::Warn:  return "WARN";
        case LogLevel::Error: return "ERROR";
    }
    return "?";
}

namespace {
spdlog::level::level_enum ToSpd(LogLevel l)
{
    switch (l) {
        case LogLevel::Trace: return spdlog::level::trace;
        case LogLevel::Debug: return spdlog::level::debug;
        case LogLevel::Info:  return spdlog::level::info;
        case LogLevel::Warn:  return spdlog::level::warn;
        case LogLevel::Error: return spdlog::level::err;
    }
    return spdlog::level::info;
}
LogLevel FromSpd(spdlog::level::level_enum l)
{
    switch (l) {
        case spdlog::level::trace: return LogLevel::Trace;
        case spdlog::level::debug: return LogLevel::Debug;
        case spdlog::level::info:  return LogLevel::Info;
        case spdlog::level::warn:  return LogLevel::Warn;
        case spdlog::level::err:   return LogLevel::Error;
        case spdlog::level::critical: return LogLevel::Error;
        default:                   return LogLevel::Info;
    }
}
std::int64_t NowNs()
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               spdlog::log_clock::now().time_since_epoch()).count();
}
} // namespace

struct Logger::Impl {
    std::shared_ptr<spdlog::logger>                        logger;
    std::shared_ptr<spdlog::sinks::ringbuffer_sink_mt>     ring;
    std::int64_t                                           startNs = 0;
    std::atomic<std::int64_t>                              clearNs{0};   // hide entries before this
};

Logger::Logger() : p_(std::make_unique<Impl>())
{
    p_->startNs = NowNs();
    try {
        auto console = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
        // One file per calendar day (logs/firedog_YYYY-MM-DD.log), switched at
        // midnight even mid-run; spdlog prunes files beyond the newest 30 so a
        // day of heavy xcan tracing can't rotate an older day's data away.
        auto file    = std::make_shared<spdlog::sinks::daily_file_sink_mt>(
                           "logs/firedog.log", 0, 0, false, 30);
        p_->ring     = std::make_shared<spdlog::sinks::ringbuffer_sink_mt>(5000);

        std::vector<spdlog::sink_ptr> sinks{console, file, p_->ring};
        p_->logger = std::make_shared<spdlog::logger>("firedog", sinks.begin(), sinks.end());
        // The wrapper gates by minLevel_; keep spdlog wide open so the ringbuffer
        // captures everything we choose to emit.
        p_->logger->set_level(spdlog::level::trace);
        p_->logger->set_pattern("[%H:%M:%S.%e] [%^%-5l%$] %v");
        p_->logger->flush_on(spdlog::level::warn);
    } catch (const std::exception& e) {
        std::fprintf(stderr, "[log] init failed: %s\n", e.what());
    }
}

Logger::~Logger()
{
    if (p_ && p_->logger) p_->logger->flush();
    spdlog::shutdown();
}

Logger& Logger::Instance()
{
    static Logger inst;
    return inst;
}

void Logger::Write(LogLevel level, const char* source, const char* fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    WriteV(level, source, fmt, args);
    va_end(args);
}

void Logger::WriteV(LogLevel level, const char* source, const char* fmt, va_list args)
{
    if (level < minLevel_.load(std::memory_order_relaxed)) return;
    if (!p_->logger) return;

    char buf[1024];
    va_list copy;
    va_copy(copy, args);
    const int n = std::vsnprintf(buf, sizeof(buf), fmt ? fmt : "", args);
    std::string msg;
    if (n < 0) {
        msg = fmt ? fmt : "";
    } else if (static_cast<std::size_t>(n) < sizeof(buf)) {
        msg.assign(buf, static_cast<std::size_t>(n));
    } else {
        msg.resize(static_cast<std::size_t>(n));
        std::vsnprintf(msg.data(), static_cast<std::size_t>(n) + 1, fmt, copy);
    }
    va_end(copy);

    std::string line;
    if (source && *source) {
        line.reserve(msg.size() + 3 + std::char_traits<char>::length(source));
        line.push_back('[');
        line += source;
        line += "] ";
        line += msg;
    } else {
        line = std::move(msg);
    }

    // "{}" so any braces in the message aren't treated as fmt fields.
    p_->logger->log(ToSpd(level), "{}", line);
    total_.fetch_add(1, std::memory_order_relaxed);
}

std::vector<LogEntry> Logger::Snapshot() const
{
    std::vector<LogEntry> out;
    if (!p_->ring) return out;

    const std::int64_t floor = p_->clearNs.load(std::memory_order_relaxed);
    const auto raws = p_->ring->last_raw();
    out.reserve(raws.size());
    for (const auto& m : raws) {
        const std::int64_t ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                    m.time.time_since_epoch()).count();
        if (ns < floor) continue;   // hidden by a Clear()
        LogEntry e;
        e.t     = static_cast<double>(ns - p_->startNs) / 1e9;
        // Formatted here, once per entry, not per frame: the panel redraws
        // every visible row each frame and a localtime per row would be the
        // most expensive thing on the Overview page.
        {
            const std::time_t secs = static_cast<std::time_t>(ns / 1000000000LL);
            const int ms = static_cast<int>((ns / 1000000LL) % 1000);
            std::tm tm{};
            localtime_s(&tm, &secs);
            std::snprintf(e.stamp, sizeof(e.stamp), "%02d:%02d:%02d.%03d", tm.tm_hour,
                          tm.tm_min, tm.tm_sec, ms);
        }
        e.level = FromSpd(m.level);
        e.text.assign(m.payload.data(), m.payload.size());
        out.push_back(std::move(e));
    }
    return out;
}

std::size_t Logger::Count() const
{
    return total_.load(std::memory_order_relaxed);
}

void Logger::Clear()
{
    // spdlog's ringbuffer has no clear(); mark a floor so the viewer hides
    // everything already buffered. The on-disk file is intentionally untouched.
    p_->clearNs.store(NowNs(), std::memory_order_relaxed);
    total_.fetch_add(1, std::memory_order_relaxed);   // force a UI refresh
}

void Logger::SetMinLevel(LogLevel lvl)
{
    minLevel_.store(lvl, std::memory_order_relaxed);
}

LogLevel Logger::MinLevel() const
{
    return minLevel_.load(std::memory_order_relaxed);
}

} // namespace firedog
