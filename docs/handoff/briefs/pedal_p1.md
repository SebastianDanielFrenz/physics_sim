# Brief: P1 - manual pedal law for all electronic engines - physics_sim

You are an agent (Opus) on S:\claude_code\physics_sim, in your own git worktree based on master cbeba77 or later. The
coordinator (Claude Opus) reviews, runs CI and merges. This is the largest increment of the transmission/pedal plan;
expect 2-3 sessions - commit in coherent steps, keep the progress file current so a resumed run can continue.

READ FIRST:
- C:\Users\sebas\AppData\Local\Temp\claude\S--claude-code-physics-sim\094bd4b1-17d1-4c89-9e37-5d1204e63823\scratchpad\briefs\physics_sim_context.md (standing rules)
- S:\claude_code\physics_sim\out\transmission_pedal_plan.md sections 1, 2.1-2.5, 2.7, 6.1 and section 8 "P1" (the binding design,
  bands, sabotages and the expected moved hashes/pins). Owner rulings are at the top of the plan and in vault PHYS-076.
- CLAUDE.md entries: throttle stage (I7a C3/C5b), SimulatedEngine idle/DFCO/live ambient (LIVE_ATMOSPHERE_PERFORMANCE.md),
  turbo_pair.h incl. boost-aware idle feed-forward (9ea2cfb) and the boost B0 entry (merge de43b93), engine_dyno (P3),
  drive modes (assists I7/I7b/I7c), launch/traction assists.
- Vault PHYS-058 (turbo, incl. the B0 addendum), PHYS-074 (assists), PHYS-076 (this plan).
- B0's measurements: tests/unit/test_boost_b0_diag.cpp (hidden [.][diag][boost_b0]) and test_boost_shift_hold.cpp (V2 hunting
  [!mayfail]); the owner high-boost fixture tests/support/fixtures/assists/v12/vehicle_v12.json (target 15.48 bar, reachable
  ~6.5-7.6 bar at WOT).

## Owner rulings (binding)
- Every engine with an ECU (electronic throttle) gets the law; governor engines (tractor) and mechanical-linkage engines keep
  today's law.
- Manual: pedal c sets gross torque T* = Z(n) + c(F(n) - Z(n)), linear between the zero-pedal ECU state (fuel cut / idle
  controller) and full throttle at the REACHABLE boost. Non-decreasing in c everywhere, strictly increasing through the
  negative range outside the small DFCO residual band (plan 2.3, "a bit of dead-band is fine").
- Drive-mode pedal bends apply only to automatic/DCT/CVT: gate the bend by gearbox kind at build (all shipped boxes are manual,
  so the bend is off everywhere until A3; the other per-mode values still apply).
- Turbo: gate-first split (plan 2.5) by default, plus the optional ECU setting part_load_boost "spooled" (off by default).

## Additions from boost B0 (binding, not in the plan text)
1. F(n) for a turbo engine is the dyno-SETTLED reachable WOT state (run_dyno_sweep at throttle 1, end-of-window value where it
   does not settle), never the configured target. It is path dependent near the boost threshold (cold vs spun rotor settle on
   different branches) and has cliffs: state which start state the build-time sweep uses and why.
2. The boost target p_c* in plan 2.5 step 3 is clamped to [p_amb, min(p_amb + target_boost, p_amb * PR_full(n))] - the
   reachable pressure ratio, not only the configured target (the owner setup's 15.48 bar target is unreachable).
3. B0's shift hold (TcuRequest::hold_boost_control: gate held, integral frozen during a TCU cut / ceiling < 1) must keep working
   under the new law; B0's tests B1-B3, V1, V3 stay green; V2 (300 km/h 6<->7 hunting on the owner setup) - measure it after
   P1 and report; the real fix is a shift policy on reachable power per gear (A2/A3). Do not build a shift policy in P1; if a
   small, clearly correct change to the manual auto-shift's kickdown check (use F, not the target) removes the hunting, propose
   it with numbers first.
4. A parallel agent is adding a blow-off valve to data/turbo_configurations/hyper_twin_default.json and hyper_twin_n2o.json
   (moves car_hyper/_n2o pins). Do not touch those files; merge master before your final validation and re-pin on top.

## Validation
- The plan's P1 bands (2.4 (a)-(c) on every shipped electronic engine; c = 1 bit-identical incl. dyno pins D1-D8 and the hyper
  n2o dyno tables; hyper 4000 rpm c = 0.5 settled within +-3 % of Z + 0.5(F - Z); idle T(0.05) >= T(0) at idle..idle+300 rpm;
  part-load fuel of the new split <= the old at equal torque) plus: the owner v12 fixture at 5000 rpm is monotone in c (B0
  measured a 149 -> 778 kW jump between c 0.5 and 0.6 today), and no cold/spun bistability at the command levels B0 measured.
- Bands written into the test header and committed BEFORE the first run; every assertion with a recorded sabotage (the plan
  lists five); 1 vs 10 workers state hash; debug_alloc zero own allocation per tick; balance_error <= 5e-8 per tick.
- Hashes/pins: the plan lists the EXPECTED moved scenario hashes and test pins - re-pin those with old -> new and reason. Any
  other moved hash or pin: stop and report before re-pinning. All other HashCheckScenarios at 1 and 10 workers unchanged.

## Rules
- Build: release preset in your worktree with -DFETCHCONTENT_SOURCE_DIR_JOLTPHYSICS=S:/claude_code/physics_sim/out/build/release/_deps/joltphysics-src
  and the same for _CATCH2, _NLOHMANN_JSON and _MINIAUDIO; vcvars64.bat; CMAKE_BUILD_PARALLEL_LEVEL=12. Windows release
  (+ msvc-release once before the final commit, C4701). Engine maps cache dir under your worktree's out/.
- Build lock: mkdir S:\claude_code\physics_sim\out\build.lock before ANY cmake/ctest (retry every 30 s - shared with the
  coordinator and another agent); owner.txt = "opus pedal P1"; remove only if owner.txt still holds your string; never hold it
  while thinking or editing - only around build/test commands.
- Commits: message file, `git commit -F`, last line "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>". Never merge to
  master or push. No vault, no CLAUDE.md, no ci.ps1. Read-only: data/tracks/test_ground, data/models, tools/assetgen,
  racing_game. No downloads. Never launch Godot (the coordinator runs the Godot smoke test). Every number [C]/[D]/[AS]/[M].
  Zero alloc per tick; deterministic. A loop that runs a measurement must not run in the background unattended forever.
- Progress file out/pedal_p1_progress.md (git-ignored): design deviations with alternatives (pro/contra, reason), measurements,
  moved hashes/pins old -> new, sabotages, proposed CLAUDE.md lines, proposed vault text, open points.
- Final message (short): branch + commits, what changed, measured bands, moved hashes/pins, V2 result, deviations, open points.
