// TowerLight.cpp : the stack light pump. Design: docs/oee-design.md.
#include "workflow/TowerLight.h"

#include "core/Log.h"
#include "core/Paths.h"
#include "hardware/HardwareManager.h"
#include "hardware/io/IIoDevice.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <exception>
#include <filesystem>
#include <fstream>
#include <system_error>

namespace firedog {

namespace {

constexpr auto kTick        = std::chrono::milliseconds(250);
constexpr auto kRetryAfter  = std::chrono::seconds(5);
constexpr int  kTicksPerSec = 4;

std::string ConfigPath() { return MachinePath("config/tower_light.local.json"); }

nlohmann::json ToJson(const TowerLightConfig& c)
{
    return {{"enabled", c.enabled},
            {"device", c.device},
            {"green", c.green},
            {"yellow", c.yellow},
            {"red", c.red},
            {"buzzer", c.buzzer},
            {"safety_beeps", c.safetyBeeps},
            {"down_buzzer_seconds", c.downBuzzerSeconds}};
}

} // namespace

TowerLightConfig LoadTowerLightConfig()
{
    TowerLightConfig c;
    const std::string path = ConfigPath();
    std::ifstream f(path);
    if (!f.is_open()) {
        // First start: leave an editable file behind, holding the defaults.
        std::error_code ec;
        std::filesystem::create_directories(std::filesystem::path(path).parent_path(), ec);
        std::ofstream o(path);
        if (o.is_open()) {
            o << ToJson(c).dump(2) << '\n';
            FD_INFO("tower", "wrote the default tower light config to %s", path.c_str());
        }
        return c;
    }
    try {
        nlohmann::json j;
        f >> j;
        c.enabled           = j.value("enabled", c.enabled);
        c.device            = j.value("device", c.device);
        c.green             = j.value("green", c.green);
        c.yellow            = j.value("yellow", c.yellow);
        c.red               = j.value("red", c.red);
        c.buzzer            = j.value("buzzer", c.buzzer);
        c.safetyBeeps       = j.value("safety_beeps", c.safetyBeeps);
        c.downBuzzerSeconds = j.value("down_buzzer_seconds", c.downBuzzerSeconds);
    } catch (const std::exception& e) {
        FD_WARN("tower", "%s is malformed (%s) - using the defaults", path.c_str(), e.what());
        return TowerLightConfig{};
    }
    return c;
}

TowerLight::TowerLight(HardwareManager& hw, std::function<bool()> safetyTripped)
    : hw_(hw), safetyTripped_(std::move(safetyTripped))
{
}

TowerLight::~TowerLight()
{
    {
        std::lock_guard<std::mutex> lk(mu_);
        stop_ = true;
    }
    cv_.notify_all();
    if (worker_.joinable()) worker_.join();

    // FireDog closed = every lamp off. Forced: whatever was last written, a
    // manual toggle may have changed since.
    if (dev_ >= 0 && hw_.IoState(static_cast<std::size_t>(dev_)) == ConnState::Connected) {
        haveWritten_ = false;
        retryAt_     = {};
        Write(Lamps{});
    }
}

void TowerLight::Start(const TowerLightConfig& cfg)
{
    if (worker_.joinable()) return;
    cfg_ = cfg;
    if (!cfg_.enabled) {
        FD_INFO("tower", "tower light disabled (config/tower_light.local.json)");
        return;
    }
    for (std::size_t i = 0; i < hw_.IoCount(); ++i)
        if (hw_.IoName(i) == cfg_.device) dev_ = static_cast<int>(i);
    if (dev_ < 0) {
        FD_WARN("tower", "IO device '%s' is not in hardware.json - tower light off",
                cfg_.device.c_str());
        return;
    }
    IIoDevice* d = hw_.IoDevice(static_cast<std::size_t>(dev_));
    const auto pin = [&](const std::string& name) {
        if (name.empty() || !d) return -1;
        const int p = d->OutputPinByName(name);
        if (p < 0)
            FD_WARN("tower", "%s has no output named '%s' - that lamp stays dark",
                    cfg_.device.c_str(), name.c_str());
        return p;
    };
    pinG_ = pin(cfg_.green);
    pinY_ = pin(cfg_.yellow);
    pinR_ = pin(cfg_.red);
    pinB_ = pin(cfg_.buzzer);
    worker_ = std::thread([this] { Run(); });
    FD_INFO("tower", "tower light on %s", cfg_.device.c_str());
}

void TowerLight::Run()
{
    std::unique_lock<std::mutex> lk(mu_);
    while (!stop_) {
        lk.unlock();
        Write(Next());
        lk.lock();
        cv_.wait_for(lk, kTick, [this] { return stop_; });
    }
}

// The pattern for this tick. Blink = 500 ms on / 500 ms off (two ticks each).
TowerLight::Lamps TowerLight::Next()
{
    tick_ = (tick_ + 1) % 4000;
    const bool blinkOn = (tick_ / 2) % 2 == 0;

    const bool tripped = safetyTripped_ && safetyTripped_();
    if (tripped && !wasTripped_) beepTicks_ = std::max(0, cfg_.safetyBeeps) * kTicksPerSec;
    wasTripped_ = tripped;
    if (beepTicks_ > 0) --beepTicks_;

    const TowerMode mode = static_cast<TowerMode>(mode_.load());
    const bool down = (mode == TowerMode::Down);
    if (down && wasDown_ == 0)
        downBuzzTicks_ = static_cast<int>(
            std::ceil(std::clamp(cfg_.downBuzzerSeconds, 0.0, 30.0) * kTicksPerSec));
    else if (!down)
        downBuzzTicks_ = 0;   // declared UP again: silence at once
    wasDown_ = down ? 1 : 0;
    const bool downBuzz = downBuzzTicks_ > 0;
    if (downBuzz) --downBuzzTicks_;

    Lamps l;
    if (tripped) {
        l.red    = blinkOn;
        l.buzzer = (beepTicks_ > 0 && blinkOn) || downBuzz;
        return l;
    }
    switch (mode) {
        case TowerMode::Down:    l.red = true; l.buzzer = downBuzz; break;
        case TowerMode::Waiting: l.yellow = blinkOn; break;
        case TowerMode::Running: l.green = true; break;
        case TowerMode::Standby: l.green = blinkOn; break;
        case TowerMode::Idle:    l.yellow = true; break;
        default: break;
    }
    return l;
}

void TowerLight::Write(const Lamps& want)
{
    const auto idx = static_cast<std::size_t>(dev_);
    if (hw_.IoState(idx) != ConnState::Connected) {
        haveWritten_ = false;   // reconnects unknown: rewrite everything
        return;
    }
    if (haveWritten_ && written_ == want) return;
    if (std::chrono::steady_clock::now() < retryAt_) return;
    IIoDevice* d = hw_.IoDevice(idx);
    if (!d) return;

    bool ok = true;
    const auto set = [&](int pin, bool on, bool was, const char* name) {
        if (pin < 0 || (haveWritten_ && was == on)) return;
        if (!d->SetOutput(pin, on)) {
            FD_WARN("tower", "%s.%s -> %s failed", cfg_.device.c_str(), name, on ? "ON" : "OFF");
            ok = false;
        }
    };
    set(pinG_, want.green, written_.green, cfg_.green.c_str());
    set(pinY_, want.yellow, written_.yellow, cfg_.yellow.c_str());
    set(pinR_, want.red, written_.red, cfg_.red.c_str());
    set(pinB_, want.buzzer, written_.buzzer, cfg_.buzzer.c_str());

    if (ok) {
        written_     = want;
        haveWritten_ = true;
    } else {
        haveWritten_ = false;
        retryAt_     = std::chrono::steady_clock::now() + kRetryAfter;
    }
}

} // namespace firedog
