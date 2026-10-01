// AlignLighting.cpp
#include "core/AlignLighting.h"

#include <cstdio>

namespace firedog {

nlohmann::json AlignLightingToJson(const AlignLighting& l)
{
    nlohmann::json j;
    j["kind"]    = l.kind;
    j["device"]  = l.device;
    j["channel"] = l.channel;
    j["color"]   = l.color;
    j["value"]   = l.value;
    return j;
}

AlignLighting AlignLightingFromJson(const nlohmann::json& j)
{
    AlignLighting l;
    if (!j.is_object()) return l;
    l.kind    = j.value("kind", "");
    l.device  = j.value("device", "");
    l.channel = j.value("channel", "");
    l.color   = j.value("color", "");
    l.value   = j.value("value", 0.0);
    return l;
}

std::string AlignLightingSummary(const AlignLighting& l)
{
    char buf[160];
    if (l.kind == "light")
        std::snprintf(buf, sizeof(buf), "%s %s %.0f %%", l.channel.c_str(), l.color.c_str(), l.value);
    else if (l.kind == "io")
        std::snprintf(buf, sizeof(buf), "io %s/%s %s", l.device.c_str(), l.channel.c_str(),
                      l.value != 0.0 ? "ON" : "OFF");
    else if (l.kind == "psu")
        std::snprintf(buf, sizeof(buf), "psu ch%s %.2f V", l.channel.c_str(), l.value);
    else
        return "-";
    return buf;
}

} // namespace firedog
