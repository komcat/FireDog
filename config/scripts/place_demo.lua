-- place_demo.lua : a worked example of the alignment + placement commands.
--
-- Runnable straight from the Scripts page, unlike placement_procedure.lua:
-- everything it needs is named in the EDIT ME block below rather than injected
-- by a step. Change those four lines to match your product and press Run.
--
-- What it does, which is the whole point of the alignment layer:
--   1. teaches the local frame O1 on the part that is on the machine NOW;
--   2. for each nominal, measures where the gripped object actually is and
--      moves the holder so it lands on that nominal;
--   3. re-measures and reports the residual.
--
-- Every fd.* call below is a registered command, so the identical sequence can
-- be built in the Recipe Editor as steps -- this file is the readable version
-- of that recipe, not a different mechanism.

-- ---- EDIT ME ---------------------------------------------------------------
local ALIGN   = "calibrate plate"   -- the alignment procedure (name or uid)
local DEVICE  = "hex-left"          -- the device HOLDING the object
local NOMINALS = { "ct1" }          -- add "ct2", "ct3" to place several
local VIEW    = ""                  -- optional: a saved position on the gantry
                                    -- that brings the object into view, by UID
-- ----------------------------------------------------------------------------

local TOL   = 0.02   -- residual that counts as placed (mm)
local TRIES = 1      -- measure/correct attempts per nominal.
                     -- 1 = measure once, correct once, stop: TWO prompts for the
                     -- whole placement, and the run succeeds with a warning that
                     -- nothing re-measured it. Raise to 2 to add a verify pass
                     -- (two more prompts) once the correction gain is trusted --
                     -- today a commanded mm moves the object about a third of
                     -- that, so a single pass cannot reach TOL anyway.
local MAX   = 8.0    -- refuse any single correction larger than this (mm).
                     -- Raised from the 2.0 default deliberately: a gripper that
                     -- presents the object several mm out is a real case. It is
                     -- still a CEILING, not a target -- check it against the
                     -- holder's remaining travel before raising it further,
                     -- because the move is relative to wherever it is now.

-- 1. Teach the frame. Do this ONCE per part: every nominal below is expressed
--    in it, and it stays live for the rest of this run. With auto stations
--    nothing is asked of the operator here.
fd.logf("demo: teaching the frame for '%s'", ALIGN)
fd.align_run{ alignment = ALIGN }

-- A frame is 2-D and rigid: origin at E1, X+ towards E2. The numbers land in
-- ctx under the prefix align_run used ("frame" by default), so a recipe can
-- branch on them -- a span far from the drawing means a mis-picked point.
fd.logf("demo: O1 origin (%.4f, %.4f) theta %.4f deg span %.4f mm",
        fd.ctx["frame_origin_x"], fd.ctx["frame_origin_y"],
        fd.ctx["frame_theta_deg"], fd.ctx["frame_span_mm"])

for i, NOM in ipairs(NOMINALS) do
    fd.logf("demo: placing on nominal '%s' (%d of %d)", NOM, i, #NOMINALS)

    -- 2. Bring the object into view. Nothing in the align family drives the
    --    camera, on purpose -- so it is an ordinary move step here.
    if VIEW ~= "" then
        fd.move_to_position{ device = "gantry-main", position = VIEW }
    end

    -- 3. Place it. align_first = false because step 1 already taught the frame
    --    and it is still live; leaving it true would re-teach per nominal.
    --
    --    This prompts for the object's two marks unless that nominal has taught
    --    detection models, in which case it runs with no UI at all. Either way
    --    it corrects, re-measures, and fails if the residual is still outside
    --    TOL after TRIES attempts.
    fd.place_run{ alignment = ALIGN, nominal = NOM, device = DEVICE,
                  align_first = false, tries = TRIES, tol_mm = TOL, max_mm = MAX }

    -- 4. What it ended up being. place_apply/place_move leave the last solved
    --    numbers in ctx, so this is the residual the loop settled on.
    fd.logf("demo: '%s' placed, last measured offset %.4f mm", NOM,
            fd.ctx["place_dist"] or 0.0)
end

-- Where else the frame is useful: move_local drives the procedure's OWN device
-- to a coordinate in O1 -- a dispense point, an inspection pose -- and follows
-- the part exactly as a nominal does. Uncomment to try it.
--
-- fd.move_local{ alignment = ALIGN, x_mm = 3.5, y_mm = 0.0 }

fd.logf("demo: done")
