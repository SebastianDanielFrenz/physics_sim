# Brief: S0 - turbo stage-map / fallback seam (continuous blend + coverage check) - physics_sim

You are an agent (Opus) on S:\claude_code\physics_sim, in your own git worktree based on master cbeba77 or later. The
coordinator (Claude Opus) reviews, runs CI and merges. Diagnose with measurements first, then fix.

READ FIRST:
- C:\Users\sebas\AppData\Local\Temp\claude\S--claude-code-physics-sim\094bd4b1-17d1-4c89-9e37-5d1204e63823\scratchpad\briefs\physics_sim_context.md (standing rules)
- CLAUDE.md: turbo entries (turbo_pair.h, ps/drivetrain/turbo_stage_maps.h - experimental offline stage maps
  physics_sim.turbo_stage_maps/1 from tools/turbo_sim, docs/data_formats/turbo_stage_maps.md, test_turbo_stage_maps.cpp),
  the boost B0 entry; vault PHYS-058 incl. the B0 addendum.
- B0's diag tests/unit/test_boost_b0_diag.cpp ("B0 frozen rotor sweep") and B0's progress file
  S:\claude_code\physics_sim\.claude\worktrees\agent-aa7fd6334e4554eae\out\boost_b0_progress.md (item 3).
- The owner high-boost fixture tests/support/fixtures/assists/v12/ (vehicle_v12.json, turbo_map_large.json = racing_game's
  large.json). racing_game's stock config hyper_twin_standard_n2o + turbo_maps/standard.json is READ-ONLY reference
  (S:\claude_code\racing_game\...; copy what you need into a physics_sim fixture).

## Problem [M, B0]
Where the operating point lies outside a stage map's coverage (flow beyond the map's choke edge at that speed, or speed outside
the map), TurboPair falls back to the authored head/work law, which under-states head and Euler work ~4x there (owner setup:
0.64 bar and 87 kW compressor work at 100k vs ~360 kW Euler estimate [D]); from ~102k the map blends in and charge jumps
0.64 -> 1.14 -> 2.60 -> 3.53 bar over 100-112k. With racing_game's standard map, turbine power swings 287 -> 159 -> 300 -> 53 kW
over 92.5-107.5k (fallback clears ~105k) - the likely cause of their stock WastegateSaturated at 6000 rpm. Any stage map whose
edge sits in the operating range hits this.

## Decision taken (coordinator + racing_game): fix in physics, two parts
1. A continuous, physically-motivated treatment across the map edge: compressor/turbine head and work must be continuous (C0 at
   least, preferably C1 in speed and flow) when the operating point leaves/enters coverage, and beyond the choke edge the work
   must not drop below what the map gives at its edge (e.g. extend the map along its own choke line / speed lines by affinity
   laws, or blend into a fallback that is first calibrated to match the map along the edge). Choose from measurement; record
   alternatives with pro/contra.
2. A loader / garage check that flags a stage map not covering the configured operating range (speed up to the rotor speed
   reached at target boost, flow up to the engine's max air flow) - as a load warning and as a limit finding if it fits the
   existing LimitKind scheme (append-only; a new kind needs the coordinator's OK - propose). Thresholds go through L0's
   LimitPolicy if they are advice thresholds (read docs/data_formats/limit_policy.md).

## Validation (bands committed before the first run)
- Frozen-rotor sweep over 60k..edge for the v12 fixture and a racing_game-standard fixture: max relative step in charge and
  turbine/compressor power between adjacent 2.5k speed samples <= a band you commit from first principles ([AS], state why) -
  today's 287 -> 159 -> 300 -> 53 kW swing must fail it (recorded sabotage = the old blend).
- No change where the operating point is inside the map: every existing turbo_stage_maps / turbo / dyno / n2o pin unchanged; if
  one moves, stop and report. Engines without stage maps bit-identical. All 20 HashCheckScenarios at 1 and 10 workers unchanged.
- The coverage check fires on the owner v12 setup and on a deliberately truncated map, and not on a map covering the range.
- 1 vs 10 workers equal; zero own allocation per tick.

## Coordination
- P1 (pedal law, another agent) edits turbo_pair.h's boost target / plate split; you edit only the stage-map/fallback evaluation.
  A BOV data agent edits data/turbo_configurations/hyper_twin_*.json - do not touch those. Merge master before final validation.

## Rules
- Build: release preset in your worktree with -DFETCHCONTENT_SOURCE_DIR_JOLTPHYSICS=S:/claude_code/physics_sim/out/build/release/_deps/joltphysics-src
  and the same for _CATCH2, _NLOHMANN_JSON and _MINIAUDIO; vcvars64.bat; CMAKE_BUILD_PARALLEL_LEVEL=12. Windows release
  (+ msvc-release once before the final commit). Maps cache dir under your worktree's out/.
- Build lock: mkdir S:\claude_code\physics_sim\out\build.lock before ANY cmake/ctest (retry every 30 s - shared with two other
  agents); owner.txt = "opus stage seam S0"; remove only if owner.txt still holds your string; hold it only around build/test
  commands. Never leave a polling loop running unattended; status files you grep must be written as UTF-8/ASCII.
- Commits: message file, `git commit -F`, last line "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>". Never merge to
  master or push. No vault, no CLAUDE.md, no ci.ps1. Read-only: data/tracks/test_ground, data/models, tools/assetgen,
  racing_game. No downloads. Never launch Godot. Every number [C]/[D]/[AS]/[M]. Deterministic.
- Progress file out/stage_seam_s0_progress.md (git-ignored): measurements, decision + alternatives, moved pins, sabotages,
  proposed CLAUDE.md lines, proposed vault text, what racing_game must change (if anything).
- Final message (short): branch + commits, root cause, fix, measured before/after, moved pins, open points.
