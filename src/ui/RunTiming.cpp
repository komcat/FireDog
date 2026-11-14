// RunTiming.cpp — cycle time: the one writer of the per-recipe history, the
// ETA computed from it, and the two lines that show it (Recipe Editor and Run
// page). Silent by design: nothing here refuses, waits or moves.
// docs/cycle-time-design.md.
#include "App.h"

#include "imgui.h"

#include "core/DutSession.h"
#include "core/Log.h"
#include "core/Product.h"   // NowStamp

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <ctime>
#include <string>

namespace firedog {

namespace {

const ImVec4 kWaitColor{1.0f, 0.85f, 0.3f, 1.0f};   // the prompt window's amber

std::int64_t SinceMs(std::chrono::steady_clock::time_point t0,
                     std::chrono::steady_clock::time_point t1)
{
    const auto end = t1 != std::chrono::steady_clock::time_point{}
                         ? t1 : std::chrono::steady_clock::now();
    return std::chrono::duration_cast<std::chrono::milliseconds>(end - t0).count();
}

} // namespace

void App::UpdateCycleTimes()
{
    // ---- Follow the product (a string compare; the store drops its cache on
    //      a switch). Product load is gated on Busy, so a queued timing below
    //      can never land in the wrong product's folder.
    cycleStore_.SetDir(CycleTimesDir(DutSession::Instance().ProductDataDir()));

    // ---- The one writer: every finished, timed run the engine queued.
    for (const RecipeEngine::RunTiming& t : orch_.recipe.TakeRunTimings()) {
        CycleTimeRecord rec;
        rec.t          = static_cast<std::int64_t>(std::time(nullptr));
        rec.ts         = NowStamp();
        rec.uid        = t.recipeUid;
        rec.name       = t.recipeName;
        rec.hash       = t.runHash;
        rec.wallMs     = t.wallMs;
        rec.waitMs     = t.waitMs;
        rec.steps      = t.steps;
        rec.serial     = t.serial;
        rec.operatorId = user_.id;
        std::string err;
        if (!cycleStore_.Append(rec, &err)) {
            // A missing history line never fails a run that already completed.
            if (err != cycleWriteErr_)
                FD_WARN("cycle", "cycle time of '%s' not kept: %s", rec.name.c_str(),
                        err.c_str());
            cycleWriteErr_ = err;
        } else {
            cycleWriteErr_.clear();
            FD_DEBUG("cycle", "'%s' took %s (operator %s)", rec.name.c_str(),
                     FormatClockMs(rec.wallMs).c_str(),
                     FormatClockMs(rec.waitMs).c_str());
        }
    }

    CycleEta eta;

    // ---- The recipe engine's run.
    const RecipeEngine::Status es = orch_.recipe.GetStatus();
    const bool engRunning = es.state == RecipeEngine::State::Running;
    eta.have      = es.state != RecipeEngine::State::Idle || es.elapsedMs > 0;
    eta.running   = engRunning;
    eta.timed     = es.timed;
    eta.untimedWhy = es.teach ? "teach run" : es.dryRun ? "dry run" : "started from a step";
    eta.waiting   = es.waitingOperator;
    eta.elapsedMs = es.elapsedMs;
    eta.waitMs    = es.waitMs;
    if (eta.have && es.timed && !es.recipeUid.empty())
        eta.est = cycleStore_.Estimate(es.recipeUid, es.runHash);
    const std::int64_t machineMs = std::max<std::int64_t>(0, es.elapsedMs - es.waitMs);
    if (engRunning && es.timed && eta.est.Valid()) {
        if (eta.est.machineMs > 0)
            eta.frac = std::min(0.99f, static_cast<float>(machineMs) /
                                           static_cast<float>(eta.est.machineMs));
        eta.remainMs = std::max<std::int64_t>(0, eta.est.machineMs - machineMs) +
                       std::max<std::int64_t>(0, eta.est.waitMs - es.waitMs);
        eta.overMs   = std::max<std::int64_t>(0, machineMs - eta.est.machineMs);
    }

    // ---- The program / slot queue.
    const ProgramRunner::Status& rs = orch_.programs.GetStatus();
    if (rs.seq != 0 && !rs.items.empty()) {
        eta.prog          = true;
        eta.progRunning   = rs.active;
        eta.progTimed     = !rs.teach && !rs.dryRun;
        eta.progElapsedMs = SinceMs(rs.t0, rs.t1);

        // Queue sums: recomputed only when the run, the item or the history moved.
        if (rs.seq != cycleQSeq_ || rs.index != cycleQIndex_ || rs.active != cycleQActive_ ||
            cycleStore_.Seq() != cycleQStoreSeq_) {
            cycleQSeq_      = rs.seq;
            cycleQIndex_    = rs.index;
            cycleQActive_   = rs.active;
            cycleQStoreSeq_ = cycleStore_.Seq();
            cycleQDoneWallMs_ = cycleQDoneWaitMs_ = cycleQDoneMachineMs_ = 0;
            cycleQPendTypicalMs_ = cycleQPendMachineMs_ = 0;
            cycleQPendUnknown_ = 0;
            for (std::size_t i = 0; i < rs.items.size(); ++i) {
                const ProgramRunner::ItemStatus& it = rs.items[i];
                if (it.wallMs >= 0) {   // ended: its ACTUAL time
                    cycleQDoneWallMs_    += it.wallMs;
                    cycleQDoneWaitMs_    += it.waitMs;
                    cycleQDoneMachineMs_ += std::max<std::int64_t>(0, it.wallMs - it.waitMs);
                    continue;
                }
                if (i <= rs.index) continue;   // the running item is measured live
                // A dry item is not timed, and a real run's history would
                // overstate it - it counts as unknown.
                const CycleEstimate e = it.dryRun ? CycleEstimate{}
                                                  : cycleStore_.Estimate(it.recipeUid, it.runHash);
                if (!e.Valid()) { ++cycleQPendUnknown_; continue; }
                cycleQPendTypicalMs_ += e.TypicalMs();
                cycleQPendMachineMs_ += e.machineMs;
            }
        }

        // The running item, when there is one: its live clock is not in the
        // done sums yet (wallMs is set only once it ends).
        const bool curLive = rs.active && engRunning && rs.index < rs.items.size() &&
                             rs.items[rs.index].wallMs < 0;
        eta.progWaitMs = cycleQDoneWaitMs_ + (curLive ? es.waitMs : 0);
        if (eta.progTimed && rs.active) {
            std::size_t unknown = cycleQPendUnknown_;
            std::int64_t curRemain = 0, curMachine = 0, curEstMachine = 0;
            if (curLive) {
                curMachine = machineMs;
                if (eta.remainMs >= 0) {
                    curRemain     = eta.remainMs;
                    curEstMachine = std::max(eta.est.machineMs, machineMs);
                } else {
                    ++unknown;
                    curEstMachine = machineMs;
                }
            }
            eta.progUnknown  = unknown;
            eta.progRemainMs = curRemain + cycleQPendTypicalMs_;
            eta.progTotalMs  = eta.progElapsedMs + eta.progRemainMs;
            const std::int64_t denom = cycleQDoneMachineMs_ + curEstMachine + cycleQPendMachineMs_;
            if (unknown == 0 && denom > 0)
                eta.progFrac = std::min(0.99f, static_cast<float>(cycleQDoneMachineMs_ + curMachine) /
                                                   static_cast<float>(denom));
        }
    }
    cycleEta_ = eta;
}

float App::RecipeRunBarFrac(std::size_t leafDone, std::size_t leafTotal) const
{
    if (cycleEta_.running && cycleEta_.frac >= 0.0f) return cycleEta_.frac;
    if (leafTotal == 0) return 0.0f;
    const float f = static_cast<float>(leafDone) / static_cast<float>(leafTotal);
    return f > 1.0f ? 1.0f : f;   // while-loops can overshoot the estimate
}

std::string App::RecipeRunBarText(std::size_t leafDone, std::size_t leafTotal) const
{
    char buf[96];
    std::snprintf(buf, sizeof(buf), "%zu / %zu", leafDone, leafTotal);
    std::string s = buf;
    if (!cycleEta_.have) return s;
    s += "   " + FormatClockMs(cycleEta_.elapsedMs);
    if (cycleEta_.running && cycleEta_.est.Valid())
        s += " of ~" + FormatClockMs(cycleEta_.est.TypicalMs());
    return s;
}

void App::DrawRecipeTimeLine()
{
    const CycleEta& e = cycleEta_;
    if (!e.have) return;
    const std::string op = FormatClockMs(e.waitMs);

    if (!e.running) {
        ImGui::TextDisabled("last run %s (operator %s)", FormatClockMs(e.elapsedMs).c_str(),
                            op.c_str());
        return;
    }
    if (e.waiting) {
        ImGui::TextColored(kWaitColor, "waiting for operator - %s so far this run", op.c_str());
        ImGui::SameLine();
    }
    if (!e.timed) {
        ImGui::TextDisabled("elapsed %s - %s, not timed", FormatClockMs(e.elapsedMs).c_str(),
                            e.untimedWhy);
        return;
    }
    if (!e.est.Valid()) {
        ImGui::TextDisabled("elapsed %s - learning (no timed run of this version yet)",
                            FormatClockMs(e.elapsedMs).c_str());
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Every completed run is timed. An edited recipe starts\n"
                              "learning again: the old times describe other steps.");
        return;
    }
    const std::string range = FormatClockMs(e.est.loMs) + "-" + FormatClockMs(e.est.hiMs);
    if (e.overMs > 0) {
        ImGui::TextDisabled("+%s over typical - machine %s (%d run%s) - operator %s",
                            FormatClockMs(e.overMs).c_str(), range.c_str(), e.est.n,
                            e.est.n == 1 ? "" : "s", op.c_str());
    } else {
        ImGui::TextDisabled("ETA %s - machine %s (%d run%s) - operator %s",
                            FormatClockMs(e.remainMs).c_str(), range.c_str(), e.est.n,
                            e.est.n == 1 ? "" : "s", op.c_str());
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Estimated from the last %d completed run(s) of this exact\n"
                          "recipe version: median machine time %s + median operator\n"
                          "time %s. The bar is MACHINE time - it holds still while\n"
                          "the run waits on the operator. An aid only: nothing is\n"
                          "judged on it.",
                          e.est.n, FormatClockMs(e.est.machineMs).c_str(),
                          FormatClockMs(e.est.waitMs).c_str());
}

void App::DrawProgramTimeLine()
{
    const CycleEta& e = cycleEta_;
    if (!e.prog) return;
    const ProgramRunner::Status& rs = orch_.programs.GetStatus();

    if (!e.progRunning) {
        ImGui::TextDisabled("took %s (operator %s)", FormatClockMs(e.progElapsedMs).c_str(),
                            FormatClockMs(e.progWaitMs).c_str());
        return;
    }

    // The queue bar: machine time when every item has history, else items.
    float frac = e.progFrac;
    if (frac < 0.0f)
        frac = rs.total > 0 ? static_cast<float>(rs.index) / static_cast<float>(rs.total) : 0.0f;
    char overlay[96];
    std::snprintf(overlay, sizeof(overlay), "recipe %zu / %zu   %s", rs.index + 1, rs.total,
                  FormatClockMs(e.progElapsedMs).c_str());
    ImGui::ProgressBar(frac, ImVec2(-1.0f, 0.0f), overlay);

    if (!e.progTimed) {
        ImGui::TextDisabled("elapsed %s - %s, not timed", FormatClockMs(e.progElapsedMs).c_str(),
                            rs.teach ? "teach run" : "dry run");
        return;
    }
    if (e.progRemainMs < 0) return;
    if (e.progUnknown == 0) {
        ImGui::TextDisabled("cycle ~%s - ETA %s - operator %s",
                            FormatClockMs(e.progTotalMs).c_str(),
                            FormatClockMs(e.progRemainMs).c_str(),
                            FormatClockMs(e.progWaitMs).c_str());
    } else {
        ImGui::TextDisabled("cycle >= %s - ETA >= %s (%zu recipe%s not timed yet)",
                            FormatClockMs(e.progTotalMs).c_str(),
                            FormatClockMs(e.progRemainMs).c_str(), e.progUnknown,
                            e.progUnknown == 1 ? "" : "s");
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Recipes already run count at their actual time, the\n"
                          "running one and every one still queued at the median of\n"
                          "their last completed runs (machine + operator time).\n"
                          "A recipe with no history yet makes the total a floor.");
}

} // namespace firedog
