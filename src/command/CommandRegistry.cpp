// CommandRegistry.cpp
#include "command/CommandRegistry.h"

#include "core/Log.h"

#include <algorithm>

namespace firedog {

bool CommandRegistry::Register(CommandSpec spec)
{
    if (spec.name.empty() || !spec.execute) {
        FD_ERROR("command", "rejected registration: empty name or no execute fn");
        return false;
    }
    if (byName_.count(spec.name) != 0) {
        FD_ERROR("command", "duplicate command '%s' - registration rejected",
                 spec.name.c_str());
        return false;
    }
    byName_.emplace(spec.name, std::move(spec));
    return true;
}

const CommandSpec* CommandRegistry::Find(const std::string& name) const
{
    const auto it = byName_.find(name);
    return it != byName_.end() ? &it->second : nullptr;
}

std::vector<const CommandSpec*> CommandRegistry::List() const
{
    std::vector<const CommandSpec*> out;
    out.reserve(byName_.size());
    for (const auto& [name, spec] : byName_) {
        (void)name;
        out.push_back(&spec);
    }
    std::sort(out.begin(), out.end(), [](const CommandSpec* a, const CommandSpec* b) {
        return a->category != b->category ? a->category < b->category
                                          : a->name < b->name;
    });
    return out;
}

} // namespace firedog
