-- Alignment repeatability: run one procedure N times and leave the spread behind
--
-- Every iteration is a WHOLE alignment run through fd.align_run, which is the
-- same code path a recipe step and the Alignment page's Run button take. There
-- is deliberately no shortcut that captures points without moving: what this
-- measures is the repeatability of the procedure as it will actually be used --
-- the travel, the settle, the exposure and the detector, together. A sweep that
-- skipped the move would report the detector's noise and call it the machine's.
--
-- Each run stores its own frame, so the rows the Frame check pane computes Cpk
-- from are the ordinary run history: nothing here writes a result of its own.
--
-- A failed iteration ENDS the sweep, on purpose. A capability figure computed
-- over the runs that happened to succeed is a lie by omission -- the failures
-- are the part of the distribution that matters most.
--
-- Parameters (fd.param -> ctx keys, injected by the Alignment page):
--   align_name    the procedure's uid or name        (required)
--   align_prefix  ctx prefix for the captured points (default "frame")
--   align_repeat  how many times to run it           (default 30)

local NAME   = fd.param("align_name", "")
local PREFIX = fd.param("align_prefix", "frame")
local N      = math.floor(fd.param("align_repeat", 30))

if NAME == "" then
    error("align_name is not set - name the alignment procedure to repeat")
end
-- Below three there is no spread to speak of and a Cpk would be arithmetic
-- dressed up as evidence; above a hundred the sweep outlives the operator's
-- attention and the ring that keeps the rows.
if N < 3 or N > 100 then
    error(string.format("align_repeat is %d - it must be between 3 and 100", N))
end

fd.logf("alignment '%s': repeatability sweep, %d runs", NAME, N)

for i = 1, N do
    fd.logf("alignment '%s': run %d of %d", NAME, i, N)
    fd.align_run{ alignment = NAME, ctx_prefix = PREFIX }
    fd.logf("  span %.4f mm  theta %.4f deg",
            fd.ctx[PREFIX .. "_span_mm"], fd.ctx[PREFIX .. "_theta_deg"])
end

fd.logf("alignment '%s': sweep finished - %d runs recorded", NAME, N)
