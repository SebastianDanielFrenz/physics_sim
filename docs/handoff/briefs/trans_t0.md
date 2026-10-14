# Brief: transmission T0 - gearbox data, rating, findings, sound fields - physics_sim

You are a coding agent (Sonnet) on S:\claude_code\physics_sim, in your own git worktree based on master 33da826 or later.
The coordinator (Claude Opus) reviews, runs CI and merges.

READ FIRST:
- C:\Users\sebas\AppData\Local\Temp\claude\S--claude-code-physics-sim\094bd4b1-17d1-4c89-9e37-5d1204e63823\scratchpad\briefs\physics_sim_context.md (standing rules)
- The plan: S:\claude_code\physics_sim\out\transmission_pedal_plan.md - section 1, the gearbox-data / rating / sound /
  gear_type / classify_gearbox_change parts, and the T0 increment in section 8 (scope, bands, sabotages). The plan's T0
  section is your spec; this brief adds the owner/game decisions below.
- CLAUDE.md: gearbox (GearboxDesc, Gearbox, data/gearboxes), limit_findings (P3b: LimitKind append-only, deferred
  GearboxInputTorque), engine_rules (P4: check_buildable, engine_rules_table.h), engine_change (classify_engine_change).

## Decisions (binding)
- Owner 2026-10-13: approved (go T0). The garage warns at Near/Limiting and refuses only Exceeded. Owner: refusal limits
  and advice thresholds must become CONFIG values, not hard-coded - that is a separate increment L0 right after you.
  For T0: put every new gearbox threshold (rating grades, margins) next to the existing ones in
  core/src/drivetrain/limit_thresholds.h / engine_rules_table.h with source tags, so L0 can move them all at once.
- Field name `rated_input_torque_nm` (the game will read it). Game answers G5/G8: publish (unhashed) in the snapshot
  `gear_type` (straight_cut/helical), `mesh_frequency_hz`, `mesh_load_nm`; G1 selector/converter/CVT fields belong to
  later increments - do not add them now unless the plan's T0 says so.
- No shipped hash or pin may move in T0 (the plan says none moves) - verify every HashCheckScenario at 1 and 10 workers.

## Rules
- Build: release preset in your worktree with -DFETCHCONTENT_SOURCE_DIR_JOLTPHYSICS=S:/claude_code/physics_sim/out/build/release/_deps/joltphysics-src
  and the same for _CATCH2, _NLOHMANN_JSON and _MINIAUDIO; vcvars64.bat; CMAKE_BUILD_PARALLEL_LEVEL=12. Windows release
  only (+ msvc-release once before the final commit).
- Build lock: mkdir S:\claude_code\physics_sim\out\build.lock before ANY cmake/ctest (retry every 30 s - shared with the
  coordinator and two other agents); owner.txt = "sonnet trans T0"; remove only if owner.txt still holds your string.
- Bands committed in the test file before the first run; never widen one; each guard with a recorded sabotage.
- Commits: message file, `git commit -F`, last line "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>". Never
  merge or push. No vault, no CLAUDE.md, no ci.ps1. Read-only: data/tracks/test_ground, data/models, tools/assetgen,
  racing_game. No downloads. Never launch Godot. Every number [C]/[D]/[AS]/[M]. Zero alloc per tick; deterministic.
- Progress file out/trans_t0_progress.md (git-ignored): changes, decisions with alternatives, measurements, sabotages,
  proposed CLAUDE.md lines, proposed vault text, open points.
- Final message (short): branch + commits, field/format names, validation numbers, deviations, open points.
