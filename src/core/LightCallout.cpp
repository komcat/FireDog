// LightCallout.cpp
#include "core/LightCallout.h"

#include <algorithm>
#include <cctype>

namespace firedog {
namespace {

std::string Lower(std::string s)
{
    std::transform(s.begin(), s.end(), s.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return s;
}

std::string Trim(const std::string& s)
{
    const auto b = s.find_first_not_of(" \t");
    if (b == std::string::npos) return {};
    const auto e = s.find_last_not_of(" \t");
    return s.substr(b, e - b + 1);
}

bool IsPrimary(const std::string& lc) { return lc == "red" || lc == "green" || lc == "blue"; }

} // namespace

const std::vector<std::string>& LightColorNames()
{
    static const std::vector<std::string> names = {
        "red", "green", "blue", "white", "collimate", "white_collimate"};
    return names;
}

bool ParseLightColor(const std::string& text, LightColor& out)
{
    const std::string t = Lower(Trim(text));
    if (t == "red")       { out = LightColor::Red;       return true; }
    if (t == "green")     { out = LightColor::Green;     return true; }
    if (t == "blue")      { out = LightColor::Blue;      return true; }
    if (t == "white")     { out = LightColor::White;     return true; }
    if (t == "collimate") { out = LightColor::Collimate; return true; }
    if (t == "white_collimate" || t == "white collimate" || t == "whitecollimate") {
        out = LightColor::WhiteCollimate;
        return true;
    }
    return false;
}

std::string LightColorName(LightColor c)
{
    switch (c) {
    case LightColor::Red:            return "red";
    case LightColor::Green:          return "green";
    case LightColor::Blue:           return "blue";
    case LightColor::White:          return "white";
    case LightColor::Collimate:      return "collimate";
    case LightColor::WhiteCollimate: return "white_collimate";
    }
    return "white";
}

std::string LightColorLabel(LightColor c)
{
    switch (c) {
    case LightColor::Red:            return "Red";
    case LightColor::Green:          return "Green";
    case LightColor::Blue:           return "Blue";
    case LightColor::White:          return "White (RGB)";
    case LightColor::Collimate:      return "Collimate";
    case LightColor::WhiteCollimate: return "White + collimate";
    }
    return "White (RGB)";
}

bool LightColorMatches(LightColor callout, const std::string& channelColor)
{
    const std::string lc = Lower(channelColor);
    switch (callout) {
    case LightColor::White:          return IsPrimary(lc);
    case LightColor::WhiteCollimate: return IsPrimary(lc) || lc == "collimate";
    default:                         return lc == LightColorName(callout);
    }
}

bool ChannelInLight(const std::string& group, const std::string& name, const std::string& light)
{
    const std::string l = Lower(light);
    return group.empty() ? Lower(name) == l : Lower(group) == l;
}

} // namespace firedog
