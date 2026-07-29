// StepRunner.h : layer 2.5 — the recipe step executor, extracted from
// RecipeEngine so BOTH engines can run recipe steps: RecipeEngine wraps it in
// its worker thread + status plumbing, and the Lua tier's fd.run_recipe()
// invokes it synchronously on the Lua worker thread with the same
// ExecutionContext (shared ctx, same stop token).
//
// Pure execution: no threads, no locks, no status of its own — the caller
// observes through the callbacks and the shared ExecutionContext. Cooperative
// stop/safety aborts come from ctx.ShouldStop() (the engines' hook installs
// both), checked between steps and inside every command wait.
#pragma once

#include "command/ExecutionContext.h"
#include "command/Recipe.h"

#include <cstddef>
#include <functional>
#include <string>
#include <vector>

namespace firedog {

class CommandRegistry;

class StepRunner {
public:
    enum class Outcome { Ok, Fail, StopOk, StopFail, Aborted };

    struct Callbacks {
        // A leaf is starting: display label + numeric step path. May be null.
        std::function<void(const std::string&, const std::vector<std::size_t>&)> onStep;
        // A leaf finished successfully (or was continued past). May be null.
        std::function<void()> onLeafDone;
    };

    StepRunner(const CommandRegistry& reg, ExecutionContext& ctx, Callbacks cb);
    StepRunner(const StepRunner&) = delete;
    StepRunner& operator=(const StepRunner&) = delete;

    // Execute the (already validated + sub-recipe-inlined) steps. On Fail /
    // StopFail, Error() carries the cause.
    Outcome Run(const std::vector<RecipeStep>& steps);
    const std::string& Error() const { return error_; }

private:
    Outcome ExecSteps(const std::vector<RecipeStep>& steps,
                      const std::string& pathPrefix,
                      const std::vector<std::size_t>& basePath, int depth);
    Outcome ExecLeaf(const RecipeStep& s, const std::string& where,
                     const std::vector<std::size_t>& path);

    const CommandRegistry& reg_;
    ExecutionContext&      ctx_;
    Callbacks              cb_;
    std::string            error_;
};

} // namespace firedog
