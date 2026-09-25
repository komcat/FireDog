-- Alignment procedure: teach the local frame O1 from E1 (origin) and E2 (X+)
--
-- Drives the procedure's device to station 1, captures E1, then station 2 and
-- E2, then solves the frame and stores it in the product document. In manual
-- mode each capture pauses for the operator to place a crosshair on the live
-- image; nothing moves during a pick.
--
-- Parameters (fd.param -> ctx keys, injected by the align_run step, the
-- Alignment page's Run button, or a ctx_set ahead of run_script):
--   align_name    the procedure's uid or name           (required)
--   align_prefix  ctx prefix for the captured points    (default "frame")
--   align_tol_mm / align_timeout_ms   station arrival contract
--   align_keep    keep the frame across program starts  (default false)

local NAME   = fd.param("align_name", "")
local PREFIX = fd.param("align_prefix", "frame")
local TOL    = fd.param("align_tol_mm", 0.01)
local TMO    = fd.param("align_timeout_ms", 60000)
local KEEP   = fd.param("align_keep", false)

if NAME == "" then
    error("align_name is not set - name the alignment procedure to run")
end

fd.logf("alignment '%s': station 1 -> E1 (origin)", NAME)
fd.align_station{ alignment = NAME, station = 1, ctx_prefix = PREFIX,
                  tol_mm = TOL, timeout_ms = TMO,
                  message = "Mark E1 (the local origin), then OK" }

fd.logf("alignment '%s': station 2 -> E2 (local X+)", NAME)
fd.align_station{ alignment = NAME, station = 2, ctx_prefix = PREFIX,
                  tol_mm = TOL, timeout_ms = TMO,
                  message = "Mark E2 (a point on the local X+ axis), then OK" }

fd.align_solve{ alignment = NAME, ctx_prefix = PREFIX, save = true, keep = KEEP }

fd.logf("O1: origin (%.4f, %.4f) mm, theta %.4f deg, span %.4f mm",
        fd.ctx[PREFIX .. "_origin_x"], fd.ctx[PREFIX .. "_origin_y"],
        fd.ctx[PREFIX .. "_theta_deg"], fd.ctx[PREFIX .. "_span_mm"])
