-- Demo: sample Keithley ch1, track the best reading, prompt to finish
-- Exercises: fd commands, fd.read, fd.ctx, fd.sleep, loops, prompt, errors.
fd.log("demo_align starting")

local first = fd.read("keithley/ch1")
if first == nil then
    error("no keithley data - connect the Keithley first")
end

local best = first
for i = 1, 5 do
    fd.keithley_read{ channel = 1, ctx = "pd" }
    local pd = fd.ctx.pd
    if pd > best then best = pd end
    fd.logf("sample %d: pd=%.3e  best=%.3e", i, pd, best)
    fd.sleep(300)
end
fd.ctx.best = best

if fd.prompt(string.format("Best sample %.3e A - finish?", best)) then
    fd.log("demo_align complete")
else
    error("cancelled by operator")
end
