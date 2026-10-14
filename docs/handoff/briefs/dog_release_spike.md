# Brief: D0 - one-tick driveline spike at a dog release (possible "barrel roll" cause) - physics_sim

You are an agent (Opus) on S:\claude_code\physics_sim, in your own git worktree. Diagnose with measurements first, then fix.
The coordinator (Claude Opus) reviews, runs CI and merges.

READ FIRST:
- C:\Users\sebas\AppData\Local\Temp\claude\S--claude-code-physics-sim\094bd4b1-17d1-4c89-9e37-5d1204e63823\scratchpad\briefs\physics_sim_context.md (standing rules)
- CLAUDE.md: "Drivetrain" (network.h/ShaftNetwork, box_lcp, topology switch apply_topology_switch, Gearbox phases
  Releasing/Selecting/Synchronising, compliant shafts C13, ManualTcu), "Vehicle system" (WheelInstance pre/post_solve, shadow chassis).
- The P1 agent's progress file S:\claude_code\physics_sim\.claude\worktrees\pedal-p1\out\pedal_p1_progress.md - its section on the
  V11-all 28.8 m/s failure and the reproduction recipe (if not there yet, use the description below), and its prototype patch
  S:\claude_code\physics_sim\.claude\worktrees\pedal-p1\out\pedal_b_proto.patch (env-gated). Do NOT edit the pedal-p1 worktree.

## Observation [M, P1 agent, 2026-10-13]
car_hyper_n2o, tests/unit/test_assist_all_together.cpp V11-all, on pedal-p1 1dccf7c with its fix (a) undone and the prototype's
"spooled split" TC mapping: the TC-capped shift request upshifts 1->2 at 4.6 m/s right at the launch-control hand-over. At the
dog release (tick 411, shift phase Releasing -> Selecting), in ONE tick: gearbox.out 15.1 -> 2067.7 rad/s, RR hub 4.5 -> 1264.9 rad/s,
FR/RR fz -> 0, chassis vx 5.165 -> 1.142 m/s, roll rate -3.26 rad/s. Max speed of the run 28.8 m/s instead of ~50. The owner has also
reported the car "doing barrel rolls for little discernible reason" in Godot - this is a candidate cause.

## Work
1. Reproduce on a branch from pedal-p1 commit 1dccf7c (git worktree/branch of your own; that commit is the P1 agent's, do not
   amend it). Prefer a reproduction that does not need the prototype: e.g. force a 1->2 upshift (gear_request) at the same state
   (hand-over, low speed, high boost, TC cutting) on master/1dccf7c. Record the minimal recipe.
2. Trace per SUBSTEP through the tick: topology state, every node omega, friction-row stick flags/LCP stage, the gear row set before
   and after apply_topology_switch, hub terms (tau0, d) from each wheel, Assembly matrix conditioning, compliant-shaft twist. Find
   which quantity first goes non-physical and why (momentum not conserved across the switch? a node left with ~0 inertia when the dog
   releases - gearbox.out / diff carrier inertia under Releasing->Selecting? a friction row with a huge capacity? a coupling torque
   from a compliant shaft's stored twist dumped into a now-free node?).
3. Fix the root cause in the drivetrain (not by clamping omegas). The topology switch must conserve momentum per PLAN.md (existing
   test "a topology switch conserving momentum" in test_powertrain_network.cpp) - find why that guarantee does not cover this case and
   extend the test so it does.
4. Whole-car guard: a test (bands committed before the first run) that drives the reproduction and asserts no hub/gearbox omega jump
   above a physically derived bound across the dog release, chassis roll rate bounded, and fz not lost on both rear wheels in a tick;
   recorded sabotage = the fix reverted.
5. Check whether the same path can occur in a normal manual or auto-shift on car_hyper/car_sedan (sweep shift speeds/boost) and report.
- No scenario hash should move unless the bug occurs in it; if one moves, stop and report old -> new and why.

## Coordination
P1 (pedal-p1) is still being finished by another agent and will be merged to master first; merge master before your final
validation. A BOV agent edits data/turbo_configurations/hyper_twin_*.json - do not touch those.

## Rules
- Build: release preset in your worktree with -DFETCHCONTENT_SOURCE_DIR_JOLTPHYSICS=S:/claude_code/physics_sim/out/build/release/_deps/joltphysics-src
  and the same for _CATCH2, _NLOHMANN_JSON and _MINIAUDIO; vcvars64.bat; CMAKE_BUILD_PARALLEL_LEVEL=12. Windows release
  (+ msvc-release once before the final commit). Engine maps cache dir under your worktree's out/.
- Build lock: mkdir S:\claude_code\physics_sim\out\build.lock before ANY cmake/ctest (retry every 30 s; shared with two other agents);
  owner.txt = "opus dog release D0"; remove only if owner.txt still holds your string; hold it only around build/test commands. Take it
  from Bash, not PowerShell. Never leave a polling loop running unattended.
- Commits: message file, `git commit -F`, last line "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>". Never merge to master or
  push. No vault, no CLAUDE.md, no ci.ps1. Read-only: data/tracks/test_ground, data/models, tools/assetgen, racing_game. No downloads.
  Never launch Godot. Every number [C]/[D]/[AS]/[M]. Zero alloc per tick; deterministic (1 vs 10 workers).
- Progress file out/dog_release_d0_progress.md (git-ignored): recipe, trace, root cause, fix + alternatives (pro/contra), moved
  pins, sabotages, proposed CLAUDE.md lines, proposed vault text.
- Final message (short): branch + commits, root cause, fix, before/after numbers, moved pins, whether normal shifts can hit it.
