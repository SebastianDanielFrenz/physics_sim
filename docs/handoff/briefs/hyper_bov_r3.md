# Brief: BOV round 3 - a self-consistent recirculating blow-off valve on the hypercar - physics_sim

You are an agent (Opus) on S:\claude_code\physics_sim, in your own git worktree. The coordinator (Claude Opus) reviews, runs CI and
merges. Diagnose with measurements first, then fix.

READ FIRST:
- C:\Users\sebas\AppData\Local\Temp\claude\S--claude-code-physics-sim\094bd4b1-17d1-4c89-9e37-5d1204e63823\scratchpad\briefs\physics_sim_context.md (standing rules)
- The round-1/2 brief C:\Users\sebas\AppData\Local\Temp\claude\S--claude-code-physics-sim\094bd4b1-17d1-4c89-9e37-5d1204e63823\scratchpad\briefs\hyper_bov.md
- The previous agent's progress file S:\claude_code\physics_sim\.claude\worktrees\hyper-bov\out\hyper_bov_progress.md and its branch
  hyper-bov (commits d0259f7 round 1, b405780 round-2 WIP, 2943cdc merge; an uncommitted flow-weighted surge-ratio edit in that worktree
  is NOT part of what you take). Do not edit that worktree.
- CLAUDE.md turbo entries (turbo_pair.h, I7d D2 blow-off valve / D3 surge / D4 intercooler, stage seam S0, boost B0, pedal P1 once
  merged), docs/data_formats/turbo.md, tests/unit/test_turbo_bov.cpp, test_turbo_surge.cpp, test_boost_shift_hold.cpp.

## History
Owner (2026-10-13): "Add BOV to hypercar", then "Fix model, then ship BOV". Round 1 (data only, valve 920 mm2/turbo recirculate,
crack 1.1 / full 1.6 bar) cut the closed-plate in-shift spike from 5.4-6.2 to 2.7-2.8 bar, but the lift-off rotor collapsed to 36.3k rpm
at +0.5 s (91.2k without a valve), because the vented flow was charged as compressor load at the full charge pressure ratio.
Round 2 replaced that by an [AS] shortcut: vent flow compressed only to pr_vent = 1 + (pr-1)*(1-opening) while the lumped charge
plenum stays at ~2 bar - NOT self-consistent (the coordinator rejected it; it also broke the D3 surge pins: pair-level fx_valve surge_s
0.057 -> 0.179 s vs < 0.1, hyper V8 x1.5 line).

## Work
1. Measure first, on master + round 1's data: during a lift with the valve open, the charge pressure, compressor operating point
   (speed, corrected flow, PR, efficiency, map cell / extended region), engine flow, vent flow, compressor work, turbine work, rotor
   decel. Answer: why does the charge stay high while the valve vents? Is the valve undersized for the compressor's flow at that speed,
   is the vent flow computed against the wrong pressures, or is the compressor PR set by speed rather than by the plenum flow balance?
2. Model: the charge pressure must come out of the plenum mass balance - compressor delivered flow(speed, PR) = engine flow + vent flow
   (+ dp/dt * V/(R T)) - and compressor work = delivered flow * isentropic head(PR) / efficiency at THAT operating point, same as with
   the valve shut. No special unloading formula. If the plenum is already integrated that way and the result is physical, say so with
   numbers (then the right fix may be valve size/position - recirculation to the compressor inlet vs atmosphere - not the model).
   Record alternatives with pro/contra. A real recirculating BOV moves the compressor toward low PR / high flow; state what rotor
   retention after a 6000 rpm lift that implies [D] before you set the band (round 2's >= 70 % band was set to the shortcut's taste;
   re-derive it).
3. Bands (commit before the first run): in-shift peak <= 3.0 bar on car_hyper_n2o stock WOT 1->2..6->7; valve shut at idle and steady
   60/120 km/h cruise; WOT torque bit-identical (valve shut at WOT); boost recovery after each upshift per B0 V1 (P1's rule: no dip
   below 90 % before the next upshift counts 0); lift-off rotor retention per your [D] derivation; D3 surge tests green WITHOUT
   re-pinning (the valve's purpose is to prevent surge - if a surge pin moves, explain it physically before touching it).
4. Re-pin what moves (car_hyper/_n2o tests, N7/N7b dyno, test_turbo_io N7b equality, audio bank pin 79cc5e377994856c if the bank key
   covers the turbo config), old -> new with reason, never widening a band. No HashCheckScenario uses a hypercar - prove all 20
   unchanged at 1 and 10 workers. Recorded sabotage: valve removed -> spike band fails; and one for the model change.
5. Docs: turbo.md, data/engines/HYPER_ONE1.md (replace the round-1 text).

## Coordination
Base: master (P1 pedal law is being merged right now; once master has the merge "Merge pedal P1", merge master again before final
validation - P1 touches turbo_pair.h's boost target / plate split and assist ceilings). You may cherry-pick round 1's data and test
file (d0259f7, tests/unit/test_hyper_bov.cpp) as a starting point.

## Rules
- Build: release preset in your worktree with -DFETCHCONTENT_SOURCE_DIR_JOLTPHYSICS=S:/claude_code/physics_sim/out/build/release/_deps/joltphysics-src
  and the same for _CATCH2, _NLOHMANN_JSON and _MINIAUDIO; vcvars64.bat; CMAKE_BUILD_PARALLEL_LEVEL=12. Windows release
  (+ msvc-release once before the final commit). Engine maps cache dir under your worktree's out/.
- Build lock: mkdir S:\claude_code\physics_sim\out\build.lock before ANY cmake/ctest (retry every 30 s); owner.txt = "opus hyper bov r3";
  take it from Bash, only around build/test commands; remove only if owner.txt still holds your string. Never leave a polling loop
  unattended.
- Commits: message file, `git commit -F`, last line "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>". Never merge to master or
  push. No vault, no CLAUDE.md, no ci.ps1. Read-only: data/tracks/test_ground, data/models, tools/assetgen, racing_game. No downloads.
  Never launch Godot. Every number [C]/[D]/[AS]/[M]. Zero alloc per tick; deterministic (1 vs 10 workers).
- Progress file out/hyper_bov_r3_progress.md (git-ignored): measurements, diagnosis, decision + alternatives, moved pins, sabotages,
  proposed CLAUDE.md line, proposed vault text.
- Final message (short): branch + commits, diagnosis, model change, measured spike/recovery/rotor before/after, moved pins, open points.
