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
            {"buzzer_device", c.buzzerDevice},
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
        c.buzzerDevice      = j.value("buzzer_device", c.buzzerDevice);
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
    if (dev_ >= 0) {
        for (Out* o : {&outG_, &outY_, &outR_, &outB_}) o->known = false;
        retryAt_ = {};
        Write(Lamps{});   // each pin only if its own device is still connected
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
    const auto devIndex = [&](const std::string& name) {
        for (std::size_t i = 0; i < hw_.IoCount(); ++i)
            if (hw_.IoName(i) == name) return static_cast<int>(i);
        return -1;
    };
    dev_ = devIndex(cfg_.device);
    if (dev_ < 0) {
        FD_WARN("tower", "IO device '%s' is not in hardware.json - tower light off",
                cfg_.device.c_str());
        return;
    }
    const std::string buzzDevName = cfg_.buzzerDevice.empty() ? cfg_.device : cfg_.buzzerDevice;
    const int         buzzDev     = devIndex(buzzDevName);
    if (buzzDev < 0 && !cfg_.buzzer.empty())
        FD_WARN("tower", "buzzer IO device '%s' is not in hardware.json - no buzzer",
                buzzDevName.c_str());
    const auto bind = [&](Out& o, int dev, const std::string& devName, const std::string& name) {
        o = Out{};
        if (name.empty() || dev < 0) return;
        IIoDevice* d = hw_.IoDevice(static_cast<std::size_t>(dev));
        const int  p = d ? d->OutputPinByName(name) : -1;
        if (p < 0) {
            FD_WARN("tower", "%s has no output named '%s' - it stays off", devName.c_str(),
                    name.c_str());
            return;
        }
        o.dev   = dev;
        o.pin   = p;
        o.label = devName + "." + name;
    };
    bind(outG_, dev_, cfg_.device, cfg_.green);
    bind(outY_, dev_, cfg_.device, cfg_.yellow);
    bind(outR_, dev_, cfg_.device, cfg_.red);
    bind(outB_, buzzDev, buzzDevName, cfg_.buzzer);
    worker_ = std::thread([this] { Run(); });
    FD_INFO("tower", "tower light on %s, buzzer %s", cfg_.device.c_str(),
            outB_.pin >= 0 ? outB_.label.c_str() : "(none)");
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

    // Operator-notice buzzer: its own cycle counter, restarted by a new
    // pattern so the first beep sounds on the first tick, not mid-cycle.
    const int buzz = buzz_.load();
    if (buzz != buzzWas_) buzzTick_ = 0;
    buzzWas_ = buzz;
    bool noticeBuzz = false;
    switch (static_cast<BuzzPattern>(buzz)) {
        case BuzzPattern::Beep:   noticeBuzz = (buzzTick_ % 4) < 2; break;
        case BuzzPattern::Double: noticeBuzz = buzzTick_ % 8 == 0 || buzzTick_ % 8 == 2; break;
        case BuzzPattern::Triple: {
            const int t = buzzTick_ % 12;
            noticeBuzz  = t == 0 || t == 2 || t == 4;
            break;
        }
        case BuzzPattern::Steady: noticeBuzz = true; break;
        default: break;
    }
    buzzTick_ = (buzzTick_ + 1) % 24;   // a multiple of every cycle above

    Lamps l;
    if (tripped) {
        l.red    = blinkOn;
        l.buzzer = (beepTicks_ > 0 && blinkOn) || downBuzz;
        return l;
    }
    l.buzzer = noticeBuzz;
    switch (mode) {
        case TowerMode::Down:    l.red = true; l.buzzer = l.buzzer || downBuzz; break;
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
    if (std::chrono::steady_clock::now() < retryAt_) return;

    bool ok = true;
    const auto set = [&](Out& o, bool on) {
        if (o.pin < 0 || (o.known && o.state == on)) return;
        const auto idx = static_cast<std::size_t>(o.dev);
        IIoDevice* d   = hw_.IoState(idx) == ConnState::Connected ? hw_.IoDevice(idx) : nullptr;
        if (!d) {             // its device is down: unknown until it is back
            o.known = false;
            return;
        }
        if (!d->SetOutput(o.pin, on)) {
            FD_WARN("tower", "%s -> %s failed", o.label.c_str(), on ? "ON" : "OFF");
            o.known = false;
            ok      = false;
            return;
        }
        o.known = true;
        o.state = on;
    };
    set(outG_, want.green);
    set(outY_, want.yellow);
    set(outR_, want.red);
    set(outB_, want.buzzer);
    if (!ok) retryAt_ = std::chrono::steady_clock::now() + kRetryAfter;
}

} // namespace firedog
