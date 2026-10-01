// AlignLighting.h : the illumination a picture is taken under.
//
// Carried by a detection MODEL only — the light it was taught and tuned at,
// travelling with it like its exposure. A station has none and there is no
// fallback. A detector judges contrast; the same part under another colour is
// another picture.
//
// Kinds:
//   ""       leave the lighting alone (the default — a procedure taught before
//            lighting existed still runs unchanged)
//   "light"  a colour call-out on the lighting controller: `channel` is the
//            LIGHT (a channel group, e.g. "main_camera_light"), `color` the call-out
//            ("red", "white", "collimate", ... — core/LightCallout.h), `value`
//            the intensity in percent. `device` is the controller's name, ""
//            = the one configured.
//   "io"     an IO output: `device`/`channel` = io device / pin name,
//            value non-zero = on
//   "psu"    a PSU channel: `channel` = number as text, value = volts
//
// Applied by Workflow::ApplyLighting — the one implementation.
#pragma once

#include <string>

#include <nlohmann/json.hpp>

namespace firedog {

struct AlignLighting {
    std::string kind;             // "" (none) | "light" | "io" | "psu"
    std::string device;           // controller / io device name ("" = the configured one)
    std::string channel;          // light name | io pin name | PSU channel number as text
    std::string color;            // "light" only: the call-out
    double      value = 0.0;      // light: percent;  io: non-zero = on;  psu: volts

    bool Set() const { return !kind.empty(); }
    bool operator==(const AlignLighting&) const = default;
};

nlohmann::json AlignLightingToJson(const AlignLighting& l);
AlignLighting  AlignLightingFromJson(const nlohmann::json& j);   // tolerant

// One line for logs and cards: "main_camera_light white 40 %", "io IOTop/Ring ON", "-".
std::string AlignLightingSummary(const AlignLighting& l);

} // namespace firedog
