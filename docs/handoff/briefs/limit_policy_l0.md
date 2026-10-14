# Brief: L0 - limit policy as config (physics_sim.limit_policy/1) - physics_sim

You are a coding agent (Sonnet) on S:\claude_code\physics_sim, in your own git worktree based on master 8116d4b or later
(transmission T0 merged: its gearbox rating thresholds kGearboxOverLimiting/kGearboxOverExceeded/kGearboxRemedyMargin sit in
engine_rules_table.h's kAllEngineRuleThresholds; grading helper core/src/drivetrain/gearbox_rating.h). The coordinator (Claude Opus) reviews,
runs CI and merges.

READ FIRST:
- C:\Users\sebas\AppData\Local\Temp\claude\S--claude-code-physics-sim\094bd4b1-17d1-4c89-9e37-5d1204e63823\scratchpad\briefs\physics_sim_context.md (standing rules)
- CLAUDE.md entries: limit_findings (P3b-1), limit_remedies (P3b-2), engine_rules (P4), engine_dyno (P3), engine_parts
  ("extends", load_json_with_extends, the user layer), and T0's gearbox rating entry.
- core/src/drivetrain/limit_thresholds.h, core/src/drivetrain/engine_rules_table.h (both: Threshold{name, value, source},
  kAll... lists), docs/data_formats/engine.md "Limit findings", "Driveline limits", "Simulated remedies", "Buildability rules".
- S:\claude_code\physics_sim\out\transmission_pedal_plan.md section 1 (gearbox rating in the garage) and Q7.

## Owner decision (binding, 2026-10-13)
"such refusal limits, advice thresholds and more should not be hard-coded, but instead config values". The garage warns at
Near/Limiting and refuses only Exceeded - and that refusal rule itself is part of the config.

## Design (binding unless you find a hard reason; then report before deviating)
1. Classify EVERY entry of kAllLimitThresholds and kAllEngineRuleThresholds (plus T0's) into
   - POLICY: decides only what the player is told / refused (Near/Limiting/Exceeded bands, window fractions, remedy margins
     such as x1.3 clutch, x1.10 wastegate, kGateNear 0.92 (owner decision, stays the default), bottle/kit targets, rule bands
     like CR/rod-ratio/valve-size/piston-speed, gearbox rating grades) -> configurable;
   - NOT POLICY: mathematical constants and anything the map generator or a loader uses to BUILD something (pack densities,
     row chord fraction, valve-layout list, boost_grid_*, na_grid_top_fraction, anything in a cache key or a state hash) ->
     stays constexpr. A config value must never move a map key, turbo key, content hash or state hash.
   Put the classification table in the progress file and as a comment in each header.
2. Format physics_sim.limit_policy/1, data/limit_policies/default.json = exactly today's literals (every POLICY entry by its
   existing `name` string, value + source text), plus `refuse_at` ("exceeded" default; "limiting"/"never" allowed) and
   `warn_at` ("near" default). Keys grouped as you see fit; unknown keys and wrong types rejected with the file path and key;
   "extends" via ps_environment_io's load_json_with_extends, user layer last (same precedence as part files). Docs
   docs/data_formats/limit_policy.md + links from engine.md.
3. Runtime: a public ps::drivetrain::LimitPolicy (value struct, no strings on the hot path) with `default_limit_policy()`
   returning the compiled defaults (= the JSON's values; a test asserts the shipped default.json equals the compiled defaults
   entry by entry, so neither can drift). Every consumer (run_dyno_sweep / limit findings, check_driveline_limits,
   solve_remedy / solve_power_target, check_buildable, T0's gearbox findings, and any garage refusal helper) takes an
   optional `const LimitPolicy*` (null = defaults) through its existing options/spec struct - additive, every existing call
   site unchanged and bit-identical. Loader ps::io::load_limit_policy_json(path, user_path = {}).
4. No game behaviour change: shipped defaults reproduce every existing finding/rule result exactly.

## Validation (bands committed in the test file before the first run)
- Every existing limit_findings / limit_remedies / engine_rules / dyno / T0 test passes unchanged (they use the defaults).
- New tests [limit_policy][drivetrain][io]: default.json == compiled defaults (every entry, exact); every POLICY threshold
  is reachable from the file (a test perturbs each entry through a policy and shows the corresponding finding/rule result
  or remedy margin moves - table-driven; an entry no consumer reads is a failure); refuse_at/warn_at semantics; extends +
  user layer precedence; loader rejections; a policy change leaves engine_map_cache_key, the turbo table key and a World
  state hash unchanged.
- Each guard with a recorded sabotage (e.g. a consumer ignoring the policy pointer, a default drifted in default.json, a
  NOT-POLICY entry made configurable that then moves a map key).
- All 20 HashCheckScenarios at 1 and 10 workers unchanged.

## Rules
- Build: release preset in your worktree with -DFETCHCONTENT_SOURCE_DIR_JOLTPHYSICS=S:/claude_code/physics_sim/out/build/release/_deps/joltphysics-src
  and the same for _CATCH2, _NLOHMANN_JSON and _MINIAUDIO; vcvars64.bat; CMAKE_BUILD_PARALLEL_LEVEL=12. Windows release
  only (+ msvc-release once before the final commit, C4701).
- Build lock: mkdir S:\claude_code\physics_sim\out\build.lock before ANY cmake/ctest (retry every 30 s - shared with the
  coordinator and other agents); owner.txt = "sonnet limit policy L0"; remove only if owner.txt still holds your string.
- Commits: message file, `git commit -F`, last line "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>". Never
  merge or push. No vault, no CLAUDE.md, no ci.ps1. Read-only: data/tracks/test_ground, data/models, tools/assetgen,
  racing_game. No downloads. Never launch Godot. Every number [C]/[D]/[AS]/[M]. Zero alloc per tick; deterministic.
- Progress file out/limit_policy_l0_progress.md (git-ignored): classification table, API, decisions with alternatives,
  sabotages, proposed CLAUDE.md lines, proposed vault text, open points.
- Final message (short): branch + commits, format/API names, classification counts, validation numbers, deviations.
