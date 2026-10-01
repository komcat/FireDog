// LightCallout.h : colour call-outs for a multi-channel light (ported from
// Horizon's LightColors). A LIGHT is a group of controller channels (or one
// ungrouped channel, a light by its own name); a CALL-OUT names which of its
// channels burn and the rest of that light goes dark:
//
//   red | green | blue      the channel(s) configured with that colour
//   white                   red + green + blue together (RGB mix)
//   collimate               the channel(s) configured "collimate"
//   white_collimate         white and collimate together
//
// Plain strings in, so the commands, the pages and the alignment station all
// share ONE matcher without depending on the driver's types.
#pragma once

#include <string>
#include <vector>

namespace firedog {

enum class LightColor { Red, Green, Blue, White, Collimate, WhiteCollimate };

// Wire names in display order: "red", "green", "blue", "white", "collimate",
// "white_collimate" — what recipes, scripts and saved stations carry.
const std::vector<std::string>& LightColorNames();

// Case-insensitive; also takes "white collimate" / "whitecollimate".
bool        ParseLightColor(const std::string& text, LightColor& out);
std::string LightColorName(LightColor c);    // wire name
std::string LightColorLabel(LightColor c);   // operator-facing, e.g. "White + collimate"

// Whether a channel configured with `channelColor` burns under `callout`.
bool LightColorMatches(LightColor callout, const std::string& channelColor);

// Whether a channel belongs to `light`: its group, or — ungrouped — its name.
bool ChannelInLight(const std::string& group, const std::string& name, const std::string& light);

} // namespace firedog
