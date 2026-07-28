// CommandSpec.cpp
#include "command/CommandSpec.h"

namespace firedog {

const char* ParamTypeName(ParamType t)
{
    switch (t) {
    case ParamType::Bool:         return "bool";
    case ParamType::Int:          return "int";
    case ParamType::Double:       return "double";
    case ParamType::String:       return "string";
    case ParamType::Value:        return "value";
    case ParamType::DeviceMotion: return "motion device";
    case ParamType::DeviceIo:     return "io device";
    case ParamType::Slide:        return "slide";
    case ParamType::PositionUid:  return "position uid";
    case ParamType::MapName:      return "map name";
    case ParamType::PinName:      return "pin name";
    case ParamType::TopicName:    return "topic";
    case ParamType::Enum:         return "enum";
    }
    return "?";
}

bool ParamValues::Has(const char* name) const
{
    return j_.contains(name) && !j_[name].is_null();
}

bool ParamValues::GetBool(const char* name) const
{
    const auto it = j_.find(name);
    return it != j_.end() && it->is_boolean() ? it->get<bool>() : false;
}

int ParamValues::GetInt(const char* name) const
{
    const auto it = j_.find(name);
    return it != j_.end() && it->is_number() ? static_cast<int>(it->get<double>()) : 0;
}

double ParamValues::GetDouble(const char* name) const
{
    const auto it = j_.find(name);
    return it != j_.end() && it->is_number() ? it->get<double>() : 0.0;
}

std::string ParamValues::GetString(const char* name) const
{
    const auto it = j_.find(name);
    return it != j_.end() && it->is_string() ? it->get<std::string>() : std::string();
}

namespace {

bool TypeMatches(ParamType t, const nlohmann::json& v)
{
    switch (t) {
    case ParamType::Bool:   return v.is_boolean();
    case ParamType::Int:    return v.is_number_integer() || v.is_number_unsigned();
    case ParamType::Double: return v.is_number();
    case ParamType::Value:  return v.is_boolean() || v.is_number() || v.is_string();
    case ParamType::String:
    case ParamType::DeviceMotion:
    case ParamType::DeviceIo:
    case ParamType::Slide:
    case ParamType::PositionUid:
    case ParamType::MapName:
    case ParamType::PinName:
    case ParamType::TopicName:
    case ParamType::Enum:
        return v.is_string();
    }
    return false;
}

} // namespace

std::string ValidateParams(const CommandSpec& spec, const nlohmann::json& params,
                           nlohmann::json* merged)
{
    if (!params.is_object() && !params.is_null())
        return "params must be a JSON object";

    nlohmann::json out = nlohmann::json::object();

    // Reject unknown keys — catches typos ("timeout" vs "timeout_ms").
    if (params.is_object()) {
        for (const auto& [key, val] : params.items()) {
            (void)val;
            bool known = false;
            for (const ParamSpec& p : spec.params) {
                if (p.name == key) { known = true; break; }
            }
            if (!known) return "unknown parameter '" + key + "'";
        }
    }

    for (const ParamSpec& p : spec.params) {
        const bool present = params.is_object() && params.contains(p.name) &&
                             !params[p.name].is_null();
        if (!present) {
            if (p.required) return "missing required parameter '" + p.name + "'";
            out[p.name] = p.defaultValue;
            continue;
        }
        const nlohmann::json& v = params[p.name];
        if (!TypeMatches(p.type, v)) {
            return "parameter '" + p.name + "' must be a " +
                   std::string(ParamTypeName(p.type));
        }
        if (p.type == ParamType::Enum) {
            const std::string s = v.get<std::string>();
            bool ok = false;
            for (const std::string& opt : p.options) {
                if (opt == s) { ok = true; break; }
            }
            if (!ok) return "parameter '" + p.name + "' has invalid value '" + s + "'";
        }
        if (v.is_number() && p.minVal < p.maxVal) {
            const double d = v.get<double>();
            if (d < p.minVal || d > p.maxVal) {
                return "parameter '" + p.name + "' out of range [" +
                       std::to_string(p.minVal) + ", " + std::to_string(p.maxVal) + "]";
            }
        }
        out[p.name] = v;
    }

    if (merged) *merged = std::move(out);
    return {};
}

} // namespace firedog
