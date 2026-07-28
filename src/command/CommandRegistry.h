// CommandRegistry.h : layer 2.5 — the single catalog of scriptable actions.
//
// One registry, three clients: the RecipeEngine executes registered commands,
// the future recipe editor renders parameter forms from their schemas, and the
// future Lua tier binds each one as a function. A command added here appears
// in all three automatically.
//
// Registration happens once at startup (App, after Workflow::Start) and the
// registry is read-only afterwards — that is what makes lock-free reads from
// the engine's worker thread safe. Do not register while a run is active.
#pragma once

#include "command/CommandSpec.h"

#include <map>
#include <string>
#include <vector>

namespace firedog {

class CommandRegistry {
public:
    bool Register(CommandSpec spec);                    // false + log on duplicate
    const CommandSpec* Find(const std::string& name) const;
    std::vector<const CommandSpec*> List() const;       // sorted by category, name
    std::size_t Count() const { return byName_.size(); }

private:
    std::map<std::string, CommandSpec> byName_;
};

} // namespace firedog
