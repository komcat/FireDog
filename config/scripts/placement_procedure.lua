-- Placement procedure: put an object held by ANOTHER device onto a nominal
--
-- The object (a lens in a gripper, say) is measured through the gantry's
-- camera: the operator marks two features whose midpoint is the object's
-- reference point. The correction to the nominal is then a RELATIVE move on the
-- device holding it -- only a delta can cross between two devices whose origins
-- have never been related to each other.
--
-- This is a LOOP on purpose. One correction lands within the accuracy of the
-- marking and the transform; re-measuring is what turns that into a number
-- instead of a hope, and a second pass usually removes what the first one left.
-- Stop when the residual is inside tol, fail when the tries run out -- a
-- placement that silently accepted a 0.3 mm miss would be worse than no
-- placement at all.
--
-- It TEACHES THE FRAME FIRST by default (place_align_first), because a nominal
-- is meaningless without one and the live frame is cleared at every program
-- start, DUT open and product load -- so a placement that assumed one would
-- fail on its first step about half the time it was used. Turn it off when a
-- program has already aligned this part and is placing several objects on it:
-- the frame belongs to the part, not to each placement.
--
-- IT DRIVES THE CAMERA to the nominal's own taught view (place_view), after the
-- alignment and before the first mark. A nominal is looked at from somewhere
-- else than it sits -- the object is gripped elsewhere and focuses at another
-- height -- so that pose belongs to the nominal, and owning it is what removes
-- the move_to_position step every placement used to need in front of it.
-- A nominal with no view taught is not an error: nothing moves, and the old
-- contract holds -- get the object in view yourself, above this script.
--
-- Parameters (fd.param -> ctx keys). Two surfaces inject them: a `place_run`
-- recipe step, and the Placement page's "Run guided placement" button. Running
-- this file straight from the Scripts page fails on the first parameter, by
-- design -- nothing there says which object is being placed onto what.
--   place_alignment    the alignment procedure's uid or name  (required)
--   place_nominal      the nominal on it, uid or name         (required)
--   place_device       the device HOLDING the object          (required)
--   place_align_first  re-teach the frame before placing      (default true)
--   place_go_to_view   drive to the nominal's camera view     (default true)
--   place_tries        measure/correct attempts               (default 2)
--   place_tol_mm       residual that counts as placed         (default 0.02)
--   place_max_mm       largest single correction allowed      (default 2.0)
--   place_prefix       ctx prefix for the marks               (default "place")

local ALIGN  = fd.param("place_alignment", "")
local NOM    = fd.param("place_nominal", "")
local DEV    = fd.param("place_device", "")
local FIRST  = fd.param("place_align_first", true)
local GOVIEW = fd.param("place_go_to_view", true)
local TRIES  = fd.param("place_tries", 2)
local TOL    = fd.param("place_tol_mm", 0.02)
local MAXMM  = fd.param("place_max_mm", 2.0)
local PREFIX = fd.param("place_prefix", "place")

-- Nothing here can be guessed, so say WHERE the values come from rather than
-- just that they are missing: this file has no launcher of its own, and the
-- Scripts page injects nothing.
local function required(value, key, what)
    if value ~= "" then return end
    error(key .. " is not set (" .. what .. "). Launch this from the Placement " ..
          "page's 'Run guided placement' button or a place_run recipe step - " ..
          "running the file directly from the Scripts page injects no parameters.")
end
required(ALIGN, "place_alignment", "the alignment procedure to use")
required(NOM,   "place_nominal",   "the nominal to place the object on")
required(DEV,   "place_device",    "the device holding the object")

-- Teach the frame on THIS part, in THIS run. The nominal is stored in local
-- coordinates, so without a live frame there is nothing to convert it through;
-- and a live frame from an earlier session has been cleared on purpose, because
-- it described a part that may no longer be on the machine.
if FIRST then
    fd.logf("placement: teaching the frame for '%s' first", ALIGN)
    fd.align_run{ alignment = ALIGN }
else
    fd.logf("placement: using the frame already taught this program")
end

-- Then look at the object. AFTER the alignment on purpose: align_run leaves the
-- camera at station 2, so a view taken before it would be thrown away. Once,
-- outside the loop -- nothing moves the camera between passes, and re-driving
-- would only add settle time to a re-measure.
if GOVIEW then
    fd.place_view{ alignment = ALIGN, nominal = NOM }
else
    fd.logf("placement: leaving the camera where it is (place_go_to_view off)")
end

local placed      = false
local residual    = 0.0
local corrections = 0   -- moves applied; tells "not converging" from "not checked"

for attempt = 1, TRIES do
    fd.logf("placement attempt %d of %d: mark the object", attempt, TRIES)

    -- Two marks. Nothing moves during either one, so the operator can take the
    -- time to put the crosshair where it belongs.
    -- `nominal` is what lets a mark be DETECTED: if that nominal has a taught
    -- model for this point, place_mark finds it and never prompts. Without one
    -- -- or when detection fails and the nominal allows the fallback -- the
    -- crosshair window comes up instead, with a warning naming what failed.
    -- Say WHICH pass this is, in the window title. Every pass after the first
    -- is a re-measure of the SAME nominal, and without this it looks like a
    -- second object being placed -- the marks are identical, so nothing else on
    -- screen distinguishes them.
    local pass = (attempt == 1)
        and string.format("%s: point %%d of 2 on the object", NOM)
        or  string.format("%s: point %%d of 2 - VERIFY pass %d of %d", NOM, attempt, TRIES)

    fd.place_mark{ alignment = ALIGN, nominal = NOM, index = 1, ctx_prefix = PREFIX,
                   label   = string.format(pass, 1),
                   message = "Mark point 1 of 2 on the object, then OK" }
    fd.place_mark{ alignment = ALIGN, nominal = NOM, index = 2, ctx_prefix = PREFIX,
                   label   = string.format(pass, 2),
                   message = "Mark point 2 of 2 (opposite the first), then OK" }

    -- Compute first, always: the log carries the number even on the pass that
    -- decides not to move.
    fd.place_apply{ alignment = ALIGN, nominal = NOM, ctx_prefix = PREFIX }
    residual = fd.ctx[PREFIX .. "_dist"]
    fd.logf("attempt %d: object is %.4f mm off the nominal (local %+.4f, %+.4f)",
            attempt, residual,
            fd.ctx[PREFIX .. "_dlx"], fd.ctx[PREFIX .. "_dly"])

    if residual <= TOL then
        placed = true
        break
    end

    -- Correct. place_move re-solves from the same marks, refuses anything over
    -- max_mm, waits for arrival AND settle, then drops the marks so the next
    -- pass cannot reuse a correction that has already been applied.
    fd.place_move{ alignment = ALIGN, nominal = NOM, device = DEV,
                   ctx_prefix = PREFIX, max_mm = MAXMM }
    corrections = corrections + 1
end

-- Two different outcomes, and which one it is matters.
--
-- tries = 1 asked for a single pass: correct once and stop. The correction was
-- applied and nothing looked at the result, so this SUCCEEDS -- refusing work
-- the caller explicitly asked for would be wrong -- but it says so loudly, the
-- same bargain auto_fallback_manual makes. Never let an unverified placement
-- read like a verified one.
--
-- tries >= 2 means a pass DID re-measure and the object is still out. That is a
-- real failure: the corrections are not converging, which points at a transform
-- sign or at marks that do not repeat, and neither is fixed by trying again.
if not placed then
    if TRIES == 1 then
        fd.logf("WARNING: '%s' corrected by %.4f mm and NOT re-measured (tries = 1) "..
                "- set tries >= 2 to verify the result", NOM, residual)
        return
    end
    error(string.format(
        "placement not within %.4f mm after %d attempt(s) - last residual %.4f mm "..
        "(%d correction(s) applied; a residual that does not shrink points at the "..
        "axis transform, not at the tolerance)",
        TOL, TRIES, residual, corrections))
end

fd.logf("placed '%s' on '%s': residual %.4f mm (%d correction(s))",
        DEV, NOM, residual, corrections)
