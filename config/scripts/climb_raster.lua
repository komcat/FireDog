-- Climb Z + XY raster: gap-closing search (Horizon RasterClimbEngine port)
--
-- One iteration = step Z toward the part, probe once where the stage already
-- stands, and - unless the probe says otherwise - search the XY plane and move
-- to its maximum. Repeat until the signal reaches a fraction of the target.
--
-- The move, defined. The climb axis (default Z) advances by z_step_um toward
-- the part; the two plane axes (default X,Y) are searched on a grid centered on
-- wherever that plane started. A plane's verdict only exists once its scan has
-- FINISHED - no exit is taken part-way through a plane, because near first
-- light the signal at a given Z is not knowable until the plane was searched.
--
-- The Z PROBE is the fast path. After a Z step, one sample is taken at the XY
-- the stage is already on - the same point, one step deeper. While that step
-- alone gains more than climb_z_probe_pct, the GAP (not the alignment) is what
-- limits coupling and the plane search would only re-find the same spot, so the
-- plane is skipped and Z steps again. The XY plane is searched once that gain
-- flattens out or the signal falls. The probe can only end the run by REACHING
-- THE TARGET; it never stops on a drop - a fall is exactly the case that hands
-- over to a full plane scan, still the only thing entitled to call a contact.
--
-- How much of a plane gets measured is climb_plane_scan:
--   cross  (default) - bidirectional line scan along A, move to that axis's
--                      best, then the same along B. Each direction stops once
--                      it is past the peak, so a plane costs a handful of
--                      points. Assumes the peak is roughly axis-separable,
--                      which holds for fibre-to-chip coupling.
--   spiral           - square spiral outward from the plane centre over the
--                      whole (2N+1)^2 grid, assuming nothing about the peak's
--                      shape; climb_rings_stop still stops it expanding once
--                      successive rings stop improving.
--
-- The run stops on the first of: a plane best reaching climb_target_pct of
-- climb_target; a plane best collapsing climb_contact_drop_pct below the peak
-- (read as the FAU having hit the part); the climb_max_z_um travel budget. On
-- EVERY stop - including an error - the stage returns to the best point of the
-- whole run, so a suspected contact retreats to the last known-good Z.
--
-- Differences from the Horizon original, all deliberate (see
-- docs/climb-raster-design.md §5):
--   * the probe sample SEEDS the plane instead of the plane re-measuring its
--     own centre - same XY, no motion between them;
--   * climb_max_skips caps consecutive skipped planes, so Z<->XY cross-coupling
--     cannot accumulate unwatched behind a satisfied gain test;
--   * each line-scan direction is judged against the value at the LINE ORIGIN,
--     not against a best the opposite direction may have just raised;
--   * one sample per point with fresh-publish gating (the FireDog house
--     measure), averaging only the baseline - not an N-sample average per point;
--   * a confirming measurement after the final park, reported as climb_final.
--
-- Results land in ctx (climb_*) for dut_record / conditionals.

-- ---- Parameters (inject from a recipe with ctx_set "climb_..." steps) -------
local P = {
  device         = fd.param("climb_device", "hex-left"),
  topic          = fd.param("climb_topic", "keithley/ch1"),  -- feedback signal
  z_axis         = fd.param("climb_z_axis", "Z"),            -- the climb axis
  plane_axes     = fd.param("climb_plane_axes", "X,Y"),      -- the searched plane (A,B)
  z_dir          = fd.param("climb_z_dir", 1),               -- +1 = Z+ closes the gap
  z_step_um      = fd.param("climb_z_step_um", 1.0),         -- travel per climb step
  z_settle_ms    = fd.param("climb_z_settle_ms", 200),       -- extra wait after a Z step
  xy_step_um     = fd.param("climb_xy_step_um", 0.5),        -- plane grid pitch
  xy_steps       = fd.param("climb_xy_steps", 6),            -- half-width in cells (13x13)
  plane_scan     = fd.param("climb_plane_scan", "cross"),    -- cross | spiral
  peak_drop      = fd.param("climb_peak_drop", 2),           -- cross: non-improvements = past the peak
  cross_passes   = fd.param("climb_cross_passes", 1),        -- cross: A->B passes per plane
  rings_stop     = fd.param("climb_rings_stop", 2),          -- spiral: rings without improvement (0 = full grid)
  improve_pct    = fd.param("climb_improve_pct", 2),         -- noise margin on "is this better?"
  -- Z-probe fast path. 0 = raster every plane (no probe at all).
  z_probe_pct    = fd.param("climb_z_probe_pct", 5),         -- gain that means "Z alone is still paying"
  max_skips      = fd.param("climb_max_skips", 5),           -- forced raster after N skips (0 = unlimited)
  -- Exit criteria. target is REQUIRED: unlike a raster this search has no area
  -- budget, so the target is what ends it.
  target         = fd.param("climb_target", 0),              -- feedback units (UI injects uA x 1e-6)
  target_pct     = fd.param("climb_target_pct", 85),         -- fraction of target that counts as reached
  max_z_um       = fd.param("climb_max_z_um", 20),           -- total climb travel budget
  max_z_steps    = fd.param("climb_max_z_steps", 2000),      -- absolute step cap, whatever the step size
  contact_drop_pct = fd.param("climb_contact_drop_pct", 10), -- plane best this far under the peak = contact
  stall_planes   = fd.param("climb_stall_planes", 0),        -- stop after N planes with no run-best gain (0 = off)
  raster_start   = fd.param("climb_raster_start", 0),        -- search the starting plane before the first Z step
  -- Measurement (same shapes as hexapod_align / raster_scan)
  settle_ms      = fd.param("climb_settle_ms", 300),         -- fresh-sample ceiling
  settle_min_ms  = fd.param("climb_settle_min_ms", 0),       -- floor for slow receivers
  fresh_samples  = fd.param("climb_fresh_samples", 2),       -- new publishes per measure (0 = blind sleep)
  avg_count      = fd.param("climb_avg_count", 3),           -- baseline averaging samples
  tol_mm         = fd.param("climb_tol_mm", 0.001),          -- move arrival tolerance
  move_timeout   = fd.param("climb_move_timeout_ms", 10000),
  velocity_mm_s  = fd.param("climb_velocity_mm_s", 1),       -- scan velocity, set once up front (0 = keep)
  trace          = fd.param("climb_trace", 0),               -- 1 = per-sample CSV trace
}

local function flag(v) return (v == true) or ((tonumber(v) or 0) ~= 0) end

-- ---- Helpers ----------------------------------------------------------------
local function split(s)
  local out = {}
  for w in string.gmatch(s, "[^,%s]+") do out[#out + 1] = w end
  return out
end

local PL = split(P.plane_axes)
assert(#PL == 2, "climb_plane_axes needs exactly two axes, e.g. X,Y")
local A, B, Z = PL[1], PL[2], P.z_axis
assert(A ~= B and A ~= Z and B ~= Z, "the plane axes and the climb axis must all differ")
assert(P.z_step_um > 0, "climb_z_step_um must be > 0")
assert(P.xy_step_um > 0, "climb_xy_step_um must be > 0")
assert(P.target > 0,
       "climb_target must be > 0 - it is this search's only exit criterion")
assert(P.target_pct > 0 and P.target_pct <= 100, "climb_target_pct must be in (0, 100]")

local N        = math.floor(tonumber(P.xy_steps) or 6)
local ZDIR     = ((tonumber(P.z_dir) or 1) >= 0) and 1 or -1
local ZSTEP    = P.z_step_um / 1000.0
local XYSTEP   = P.xy_step_um / 1000.0
local SPIRAL   = tostring(P.plane_scan):lower() == "spiral"
local EXIT_LVL = P.target * P.target_pct / 100.0
assert(N >= 1, "climb_xy_steps must be >= 1")

-- Worst case per plane: the full grid for a spiral, or the two full lines
-- (centre shared) per pass for a cross. Early stops only ever reduce it.
local PTS_PER_PLANE = SPIRAL and ((2 * N + 1) * (2 * N + 1))
                              or (1 + P.cross_passes * 8 * N)

local function pos(axis)
  local v = fd.read("motion/" .. P.device .. "/" .. axis)
  if v == nil then error("no position stream for " .. P.device .. "/" .. axis) end
  return v
end

local function move_rel(axis, delta)
  fd.move_relative{ device = P.device, axis = axis, delta_mm = delta,
                    tol_mm = P.tol_mm, timeout_ms = P.move_timeout }
end

local function move_abs(axis, target)
  local delta = target - pos(axis)
  if math.abs(delta) < 1e-6 then return end   -- already there (sub-nm)
  move_rel(axis, delta)
end

-- Position of the three axes this scan owns. A "best point" is always a full
-- snapshot: the plane axes say where in the plane, Z says which plane.
local function snapshot()
  return { [A] = pos(A), [B] = pos(B), [Z] = pos(Z) }
end

local function goto_snapshot(s)
  move_abs(A, s[A])
  move_abs(B, s[B])
  move_abs(Z, s[Z])
end

-- One sample per measure, fresh-publish gated (see hexapod_align for the
-- rationale: the mechanics are already settled when the move command returns;
-- the only stale thing is the instrument sample that integrated DURING the
-- move, and publish #fresh_samples is guaranteed post-move). settle_ms caps
-- the wait - a stalled stream degrades to "use the latest value" and warns once.
local FRESH = tonumber(P.fresh_samples) or 0
local settle_warned = false

local function measure()
  if FRESH <= 0 then
    fd.sleep(P.settle_ms)                        -- original blind settle
  else
    local _, n0 = fd.read(P.topic)
    if P.settle_min_ms > 0 then fd.sleep(P.settle_min_ms) end
    local t0 = fd.clock()
    while true do
      local _, n = fd.read(P.topic)
      if n >= n0 + FRESH then break end
      if (fd.clock() - t0) * 1000.0 >= P.settle_ms then
        if not settle_warned then
          settle_warned = true
          fd.logf("measure: only %d/%d fresh samples on %s within %g ms - using latest",
                  n - n0, FRESH, P.topic, P.settle_ms)
        end
        break
      end
      fd.sleep(10)
    end
  end
  local t0 = fd.clock()
  while true do
    local v = fd.read(P.topic)
    if v ~= nil then return v end
    if fd.clock() - t0 > 5.0 then error("no signal on " .. P.topic) end
    fd.sleep(50)
  end
end

-- True when cand beats ref by more than the noise margin. The margin scales by
-- |ref| rather than reading ref * (1 + pct): the feedback is SIGNED, and with a
-- negative reference the multiplicative form moves the bar the wrong way (it
-- would accept a WORSE value) - which near first light is exactly where this
-- search operates. It also decides ties toward the point already held, which is
-- the one nearer the plane centre.
local function improves(cand, ref)
  return cand > ref + math.abs(ref) * P.improve_pct / 100.0
end

-- ---- Optional per-sample trace (climb_trace=1) ------------------------------
--   clmb,seq,t_s,phase,plane,z_um,ia,ib,<A>=pos,<B>=pos,<Z>=pos,v=value,dv=delta,note
-- phase in base | probe | plane | final. Lines land in the log band AND the
-- persistent daily log - extract with:  findstr "clmb," logs\firedog_*.log
local TRACE = flag(P.trace)
local T  = { n = 0, t0 = fd.clock(), v = nil }
local TC = { phase = "base", plane = 0, z = 0.0 }
local function trace(v, ia, ib, note)
  if not TRACE then return end
  T.n = T.n + 1
  local p = snapshot()
  fd.logf("clmb,%d,%.2f,%s,%d,%+.2f,%d,%d,%s=%.5f,%s=%.5f,%s=%.5f,v=%.6g,dv=%+.4g,%s",
          T.n, fd.clock() - T.t0, TC.phase, TC.plane, TC.z, ia, ib,
          A, p[A], B, p[B], Z, p[Z], v, T.v and (v - T.v) or 0, note or "")
  T.v = v
end

-- ---- Run state --------------------------------------------------------------
local zTravel        = 0.0     -- cumulative climb travel, um (signed by ZDIR)
local planeIdx       = 0
local planesRastered = 0
local planesSkipped  = 0
local points         = 0
local stall          = 0
-- Last value measured at the XY the stage is standing on: the Z probe's
-- reference. Only ever set from a point the stage actually occupies.
local vAtPoint       = nil
local runBest        = { v = -math.huge, pos = nil, z = 0.0, a = 0.0, b = 0.0 }
local baseline       = 0.0

-- ---- One plane --------------------------------------------------------------
-- Running state of a plane's scan, shared by both strategies. bestPos is a full
-- snapshot; bia/bib are the best cell's offsets from the plane centre.
local function sample_point(plane, ia, ib)
  local v = measure()
  points = points + 1
  plane.sampled = plane.sampled + 1
  local best = (plane.bestPos == nil) or improves(v, plane.bestV)
  if best then
    plane.bestV, plane.bestPos, plane.bia, plane.bib = v, snapshot(), ia, ib
  end
  trace(v, ia, ib, best and "plane-best" or "")
  return v, best
end

-- One bidirectional line scan through the plane's current best, along A or B.
-- Walks outward +1 then -1 from the line origin, stopping each direction at the
-- grid edge or once it is past the peak, and leaves the stage on the best point.
--
-- Both directions are judged against originV - the value AT the line origin -
-- rather than against plane.bestV, which the first direction may have just
-- raised. Seeding the second direction from the first one's win makes it start
-- already "failing to improve", so a noise spike on one side truncates the
-- other side to peak_drop samples.
local function scan_line(plane, along_a)
  local axis   = along_a and A or B
  local fixed  = along_a and plane.bib or plane.bia
  local origin = along_a and plane.bia or plane.bib
  local originPos = plane.bestPos
  local originV   = plane.bestV

  for _, sign in ipairs({ 1, -1 }) do
    goto_snapshot(originPos)          -- every direction starts from the origin
    local lineBest = originV          -- ...and is judged against the value there
    local drops = 0
    for k = 1, 2 * N do               -- bounded by the grid half-width
      local cell = origin + sign * k
      if math.abs(cell) > N then break end
      move_rel(axis, sign * XYSTEP)
      local ia = along_a and cell or fixed
      local ib = along_a and fixed or cell
      local v = sample_point(plane, ia, ib)
      if improves(v, lineBest) then
        lineBest, drops = v, 0
      else
        drops = drops + 1
        if drops >= P.peak_drop then break end   -- past the peak this way
      end
    end
  end
  goto_snapshot(plane.bestPos)        -- park on the best the line found
end

-- Cross scan: a line along A, then a line along B, through the running best.
-- The plane centre is already sampled by the caller (probe seed or an explicit
-- sample), so no line re-measures it.
local function scan_cross(plane)
  for _ = 1, P.cross_passes do
    scan_line(plane, true)
    scan_line(plane, false)
  end
end

local function advance(ia, ib, dir)
  if dir == 0 then return ia + 1, ib end
  if dir == 1 then return ia, ib + 1 end
  if dir == 2 then return ia - 1, ib end
  return ia, ib - 1
end

-- Square spiral outward from the plane centre over the (2N+1)^2 grid, starting
-- where the signal is most likely and stepping one cell at a time (no wasted
-- travel). With rings_stop > 0 the sweep stops expanding once that many
-- consecutive rings fail to improve the plane's best: the signal falls off with
-- radius, so once it does the outer cells are known to be worse.
-- The centre is already sampled by the caller.
local function scan_spiral(plane)
  local total = (2 * N + 1) * (2 * N + 1)
  local ia, ib = 0, 0
  local legLen, dir, legsAtLen = 1, 0, 0
  local visited = 1                       -- the centre
  local ring, ringsNoImp, ringImproved = 0, 0, true

  while visited < total do
    -- Step to the next spiral cell. Legs run 1,1,2,2,3,3,... so the path is a
    -- square spiral outward; within (2N+1)^2 samples it never leaves the box.
    local alongA   = (dir == 0 or dir == 2)
    local positive = (dir == 0 or dir == 1)
    ia, ib = advance(ia, ib, dir)
    move_rel(alongA and A or B, (positive and 1 or -1) * XYSTEP)
    legLen = legLen - 1
    if legLen == 0 then
      dir = (dir + 1) % 4
      legsAtLen = legsAtLen + 1
      legLen = math.floor(legsAtLen / 2) + 1
    end

    -- A ring closed - decide whether the next one is worth sweeping.
    local cellRing = math.max(math.abs(ia), math.abs(ib))
    if cellRing ~= ring then
      if P.rings_stop > 0 then
        ringsNoImp = ringImproved and 0 or (ringsNoImp + 1)
        if ringsNoImp >= P.rings_stop then
          fd.logf("  plane %d: past the peak at ring %d (%d rings without improvement) - %d/%d points",
                  planeIdx, ring, ringsNoImp, visited, total)
          return
        end
      end
      ring, ringImproved = cellRing, false
    end

    local _, best = sample_point(plane, ia, ib)
    if best then ringImproved = true end
    visited = visited + 1
  end
end

-- Searches the current plane for its maximum and moves there, then judges the
-- plane. seedV, when given, is the probe sample taken at this plane's centre -
-- the stage has not moved since, so it IS the centre's measurement and the scan
-- must not pay for it twice.
-- Returns "continue" | "target" | "contact" | "stall".
local function raster_plane(seedV)
  TC.phase, TC.plane, TC.z = "plane", planeIdx, zTravel
  local plane = { bestV = nil, bestPos = nil, bia = 0, bib = 0, sampled = 0 }
  if seedV ~= nil then
    plane.bestV, plane.bestPos, plane.sampled = seedV, snapshot(), 1
  else
    sample_point(plane, 0, 0)
  end

  if SPIRAL then scan_spiral(plane) else scan_cross(plane) end

  -- Only now is the plane's signal known - go to its maximum.
  goto_snapshot(plane.bestPos)
  planesRastered = planesRastered + 1
  vAtPoint = plane.bestV        -- the stage stands on it: the next probe's reference

  -- Suspected contact: judge against the peak BEFORE folding this plane in, and
  -- only once a positive peak exists (a relative drop means nothing while the
  -- signal is at or below zero). Planes the probe skipped count as prior
  -- measurement - they left a real sample behind - so a collapse on the first
  -- plane rastered after a run of skips is still judged.
  local contact = P.contact_drop_pct > 0
                  and (planesRastered + planesSkipped) > 1
                  and runBest.v > 0
                  and plane.bestV < runBest.v - math.abs(runBest.v) * P.contact_drop_pct / 100.0

  local gained = plane.bestV > runBest.v
  if gained then
    runBest = { v = plane.bestV, pos = plane.bestPos, z = zTravel,
                a = plane.bia * P.xy_step_um, b = plane.bib * P.xy_step_um }
  end

  fd.logf("  plane %d (Z%+.1f um, %s): best=%.4g (%.1f%% of target) at %s%+.1f/%s%+.1f um, %d pts",
          planeIdx, zTravel, SPIRAL and "spiral" or "cross", plane.bestV,
          100.0 * plane.bestV / P.target, A, plane.bia * P.xy_step_um,
          B, plane.bib * P.xy_step_um, plane.sampled)
  planeIdx = planeIdx + 1

  if plane.bestV >= EXIT_LVL then return "target" end

  if contact then
    fd.logf("  plane best %.4g is %.1f%% below the peak %.4g (limit %g%%) at Z%+.1f um - suspected contact, retreating",
            plane.bestV, 100.0 * (runBest.v - plane.bestV) / math.abs(runBest.v),
            runBest.v, P.contact_drop_pct, zTravel)
    return "contact"
  end

  if P.stall_planes > 0 then
    stall = gained and 0 or (stall + 1)
    if stall >= P.stall_planes then
      fd.logf("  %d planes without a new peak - stopping at the peak %.4g", stall, runBest.v)
      return "stall"
    end
  end
  return "continue"
end

-- ---- The Z probe -------------------------------------------------------------
-- One sample taken straight after a Z step, where the stage already stands: the
-- XY has not moved since the previous plane's best (or the previous probe), so
-- this compares like with like - the same XY point, one Z step deeper.
--
-- The probe can only end the run by REACHING THE TARGET; a measured point at
-- the exit level is the exit criterion whatever the search was doing. It never
-- stops on a drop: a fall is exactly the case that hands over to a full plane
-- scan, the only thing entitled to declare a suspected contact.
--
-- Returns "target" | "skip" | "raster", and the sample (the plane's centre
-- measurement when the caller goes on to raster).
local function probe_after_z(skipRun)
  TC.phase, TC.plane, TC.z = "probe", planeIdx, zTravel
  local ref = vAtPoint
  local v   = measure()
  points   = points + 1
  vAtPoint = v

  -- A probe is a real measured point: fold it in so the fail-safe park has it.
  if v > runBest.v then
    runBest = { v = v, pos = snapshot(), z = zTravel, a = 0.0, b = 0.0 }
  end

  if v >= EXIT_LVL then
    trace(v, 0, 0, "probe-target")
    fd.logf("  probe at Z%+.1f um: %.4g reached the exit level %.4g without an XY search",
            zTravel, v, EXIT_LVL)
    planeIdx = planeIdx + 1
    return "target", v
  end

  -- Only a positive reference can be grown by a fraction; at or below zero the
  -- ratio is meaningless (and would flip sign), so those planes are always
  -- rastered.
  local paying = (ref ~= nil) and ref > 0
                 and v > ref + ref * P.z_probe_pct / 100.0
  if not paying then
    trace(v, 0, 0, "probe")
    return "raster", v
  end

  if P.max_skips > 0 and skipRun >= P.max_skips then
    -- Z is still paying, but the plane has not been searched for max_skips
    -- steps. A Z move is not perfectly decoupled from the plane (pivot offsets,
    -- stage cross-talk), so an unwatched run of skips lets XY error accumulate
    -- behind a gain test that cannot see it.
    trace(v, 0, 0, "probe-capped")
    fd.logf("  plane %d (Z%+.1f um): %d planes skipped in a row - searching the plane anyway",
            planeIdx, zTravel, skipRun)
    return "raster", v
  end

  planesSkipped = planesSkipped + 1
  trace(v, 0, 0, "probe-skip")
  fd.logf("  plane %d (Z%+.1f um): probe %.4g -> %.4g (+%.1f%%) - Z is still paying, skipping the XY search",
          planeIdx, zTravel, ref, v, 100.0 * (v - ref) / ref)
  planeIdx = planeIdx + 1
  return "skip", v
end

-- ---- The climb ---------------------------------------------------------------
-- Optionally search the starting plane, then repeat "step Z -> probe -> (search
-- the plane -> move to its max)" until the target is reached, the signal
-- collapses, or the travel budget is spent. The loop is bounded by a fixed step
-- count, never by the signal.
local function climb()
  local skipRun = 0            -- consecutive planes skipped by the probe

  if flag(P.raster_start) then
    local out = raster_plane(nil)
    if out ~= "continue" then return out end
  end

  local budget = math.max(1, math.min(math.floor(P.max_z_steps),
                                      math.floor(P.max_z_um / P.z_step_um)))
  for _ = 1, budget do
    move_rel(Z, ZDIR * ZSTEP)
    zTravel = zTravel + ZDIR * P.z_step_um
    if P.z_settle_ms > 0 then fd.sleep(P.z_settle_ms) end

    local seedV, skipped = nil, false
    if P.z_probe_pct > 0 then
      local verdict, v = probe_after_z(skipRun)
      if verdict == "target" then return "target" end
      if verdict == "skip" then
        skipRun, skipped = skipRun + 1, true
      else
        skipRun = 0
        seedV   = v          -- the probe IS the plane's centre sample
      end
    end

    if not skipped then
      local out = raster_plane(seedV)
      if out ~= "continue" then return out end
    end
  end

  fd.logf("Z budget spent (%d x %g um = %g um) without reaching %.4g - best was %.4g",
          budget, P.z_step_um, budget * P.z_step_um, EXIT_LVL, runBest.v)
  return "z_budget"
end

-- ---- Main -------------------------------------------------------------------
fd.logf("climb_raster: %s on %s, Z%s %g um x<=%d (budget %g um), %s plane %s/%s %g um +/-%d (<=%d pts), target %.4g -> exit %g%% (%.4g)",
        P.device, P.topic, ZDIR > 0 and "+" or "-", P.z_step_um,
        math.max(1, math.min(math.floor(P.max_z_steps), math.floor(P.max_z_um / P.z_step_um))),
        P.max_z_um, SPIRAL and "spiral" or "cross", A, B, P.xy_step_um, N,
        PTS_PER_PLANE, P.target, P.target_pct, EXIT_LVL)
fd.logf("  Z-probe fast path: %s; contact guard: %s",
        P.z_probe_pct > 0
          and string.format("skip the XY search while a step gains > %g%% (max %s in a row)",
                            P.z_probe_pct, P.max_skips > 0 and tostring(math.floor(P.max_skips)) or "unlimited")
          or "off (every plane rastered)",
        P.contact_drop_pct > 0 and string.format("-%g%% of peak", P.contact_drop_pct) or "off")

-- Deterministic scan velocity: set ONCE up front (persists on the device), so
-- every micro-move has the same profile and settle behaviour.
if (tonumber(P.velocity_mm_s) or 0) > 0 then
  fd.set_velocity{ device = P.device, velocity_mm_s = P.velocity_mm_s }
end

if TRACE then
  fd.logf("clmb,header: seq,t_s,phase,plane,z_um,ia,ib,%s,%s,%s,value,dv,note", A, B, Z)
end

-- Averaged baseline at the start pose (the only averaging in the climb).
local sum = 0
for i = 1, P.avg_count do
  local v = measure()
  points = points + 1
  trace(v, 0, 0, "baseline")
  sum = sum + v
  if i < P.avg_count then fd.sleep(50) end
end
baseline = sum / P.avg_count
vAtPoint = baseline
runBest  = { v = baseline, pos = snapshot(), z = 0.0, a = 0.0, b = 0.0 }
fd.logf("baseline = %.4g", baseline)

-- Whatever ends the climb - target, contact, budget or a failure - the stage
-- goes back to the best point of the whole run. A suspected contact must
-- retreat to the last known-good Z, and that must not depend on the stop being
-- a tidy one, so the climb body runs under pcall.
--
-- An operator STOP is the exception, and deliberately so: it means "no more
-- motion", and the engine's stop hook re-fires on the next line anyway, so a
-- retreat could not run even if it should. Every other failure gets the
-- fail-safe park - this search drives toward a part, and must not be left
-- standing wherever it broke.
local ok, stop = pcall(climb)
if not ok then
  if not tostring(stop):find("stopped", 1, true) and runBest.pos ~= nil then
    pcall(goto_snapshot, runBest.pos)
  end
  error(stop, 0)
end

goto_snapshot(runBest.pos)
TC.phase, TC.plane, TC.z = "final", planeIdx, runBest.z
local finalV = measure()
points = points + 1
trace(finalV, 0, 0, "final@peak")

fd.ctx.climb_baseline    = baseline
fd.ctx.climb_best        = runBest.v
fd.ctx.climb_final       = finalV
fd.ctx.climb_target      = P.target
fd.ctx.climb_target_frac = runBest.v / P.target
fd.ctx.climb_stop        = stop
fd.ctx.climb_exit_hit    = (stop == "target")
fd.ctx.climb_z_um        = zTravel
fd.ctx.climb_best_z_um   = runBest.z
fd.ctx.climb_best_a_um   = runBest.a
fd.ctx.climb_best_b_um   = runBest.b
fd.ctx.climb_planes      = planesRastered
fd.ctx.climb_skipped     = planesSkipped
fd.ctx.climb_points      = points
-- A ratio only means something when both readings are real positive signal;
-- on dark-noise runs it would be a ratio of noise samples.
fd.ctx.climb_gain = (baseline > 0 and runBest.v > 0) and (runBest.v / baseline) or 0

fd.logf("climb_raster done (%s): baseline=%.4g best=%.4g final=%.4g (%.1f%% of %.4g) at Z%+.1f um, %s%+.1f/%s%+.1f um; %d planes searched + %d skipped, %d points, Z travel %+.1f um",
        stop, baseline, runBest.v, finalV, 100.0 * runBest.v / P.target, P.target,
        runBest.z, A, runBest.a, B, runBest.b,
        planesRastered, planesSkipped, points, zTravel)
