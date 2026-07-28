// IoWatch.cpp
//
// Edge/debounce/action logic ported from bobcat_engine's IoWatchManager.cpp
// (PollingThreadFunction :210, ExecuteAction :284, ShouldTrigger :333), with
// pins resolved up-front instead of per-tick name lookups.
#include "hardware/io/IoWatch.h"

#include "core/Log.h"

namespace firedog {
namespace {

bool ShouldTrigger(bool prev, bool curr, IoWatch::Edge edge)
{
    if (prev == curr) return false;
    switch (edge) {
        case IoWatch::Edge::Rising:  return !prev && curr;
        case IoWatch::Edge::Falling: return prev && !curr;
        case IoWatch::Edge::Both:    return true;
    }
    return false;
}

} // namespace

const char* IoWatch::EdgeName(Edge e)
{
    switch (e) {
        case Edge::Falling: return "falling";
        case Edge::Both:    return "both";
        default:            return "rising";
    }
}

const char* IoWatch::BehaviorName(Behavior b)
{
    switch (b) {
        case Behavior::SetOn:     return "set_on";
        case Behavior::SetOff:    return "set_off";
        case Behavior::Momentary: return "momentary";
        default:                  return "toggle";
    }
}

IoWatch::Edge IoWatch::ParseEdge(const std::string& s)
{
    if (s == "falling") return Edge::Falling;
    if (s == "both")    return Edge::Both;
    return Edge::Rising;
}

IoWatch::Behavior IoWatch::ParseBehavior(const std::string& s)
{
    if (s == "set_on")    return Behavior::SetOn;
    if (s == "set_off")   return Behavior::SetOff;
    if (s == "momentary") return Behavior::Momentary;
    return Behavior::Toggle;
}

IoWatch::IoWatch(std::vector<Binding> bindings, int intervalMs)
    : intervalMs_(intervalMs > 0 ? intervalMs : 50)
{
    slots_.reserve(bindings.size());
    for (Binding& b : bindings) {
        auto slot = std::make_unique<Slot>();
        slot->b = std::move(b);
        slots_.push_back(std::move(slot));
    }
}

IoWatch::~IoWatch() { Stop(); }

void IoWatch::Start()
{
    if (running_.exchange(true)) return;
    thread_ = std::thread(&IoWatch::Loop, this);
    FD_INFO("iowatch", "watching %zu bindings every %d ms", slots_.size(), intervalMs_);
}

void IoWatch::Stop()
{
    if (running_.exchange(false)) {
        if (thread_.joinable()) thread_.join();
    }
}

void IoWatch::Loop()
{
    auto nextPoll = std::chrono::steady_clock::now();
    while (running_.load()) {
        if (active_.load()) {
            for (auto& sp : slots_) {
                Slot& s = *sp;
                const IIoDevice::IoStatus st = s.b.inputDev->Status();
                if (!st.valid) continue;   // device still connecting — no data yet
                const bool curr = ((st.inputs >> s.b.inputPin) & 1ull) != 0;
                s.inputOn.store(curr);

                if (!s.hasPrev) {   // first valid read seeds the edge detector
                    s.prev = curr;
                    s.hasPrev = true;
                    continue;
                }
                const bool prev = s.prev;
                s.prev = curr;
                if (curr == prev) continue;

                // Momentary follows the input on both edges; the others
                // respect the configured trigger edge.
                const bool momentary = s.b.behavior == Behavior::Momentary;
                if (!momentary && !ShouldTrigger(prev, curr, s.b.trigger)) continue;

                const auto now = std::chrono::steady_clock::now();
                if (s.hasTrigger &&
                    now - s.lastTrigger < std::chrono::milliseconds(s.b.debounceMs)) {
                    continue;   // debounced
                }

                Execute(s, curr);
                s.lastTrigger = now;
                s.hasTrigger  = true;
            }
        }
        nextPoll += std::chrono::milliseconds(intervalMs_);
        std::this_thread::sleep_until(nextPoll);
    }
}

void IoWatch::Execute(Slot& s, bool inputState)
{
    bool newState = false;
    switch (s.b.behavior) {
        case Behavior::Toggle:
            newState = ((s.b.outputDev->Status().outputs >> s.b.outputPin) & 1ull) == 0;
            break;
        case Behavior::SetOn:     newState = true;       break;
        case Behavior::SetOff:    newState = false;      break;
        case Behavior::Momentary: newState = inputState; break;
    }

    if (!s.b.outputDev->SetOutput(s.b.outputPin, newState)) {
        FD_WARN("iowatch", "'%s': failed to set %s pin %d -> %s: %s",
                s.b.name.c_str(), s.b.outputDev->Name().c_str(), s.b.outputPin,
                newState ? "ON" : "OFF", s.b.outputDev->LastError().c_str());
        return;
    }
    s.hits.fetch_add(1);
    FD_INFO("iowatch", "%s '%s': %s pin %d -> %s", BehaviorName(s.b.behavior),
            s.b.name.c_str(), s.b.outputDev->Name().c_str(), s.b.outputPin,
            newState ? "ON" : "OFF");
}

} // namespace firedog
