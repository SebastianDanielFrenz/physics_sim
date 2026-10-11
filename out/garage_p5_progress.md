# Engine garage P5 - forced-induction conversions (progress)

Branch worktree-agent-a7382f5b35995683d (base 131f3b3). Brief: scratchpad/briefs/garage_p5.md (design A13).
Status: complete, ready to merge. Release and msvc-release builds clean; ps_unit_tests FI1-FI7 and [engine_rules] green on both.

## Added
- `data/turbos/{small_gt28,medium_gtx30,large_gtx42}_class.json`: three hardware sizes ([AS]/[D], no download), formats of hyper_turbo_approx.json plus an
  informational `rated_power_kw_per_turbo` {min,max,source} key (ignored by the loader).
- `data/turbo_configurations/{single,twin}_{small,medium,large}.json`: six templates, 0.8 bar gauge, intercooler 0.75, the hyper's PID.
- `core/src/drivetrain/engine_rules.cpp` + `engine_rules_table.h` + `engine_rules.h`: `boost_grid_p_im` adds a node AT ISA ambient when the last stock node at
  or below ambient is more than 2 kPa under it (`kBoostGridAmbientNodeGapPa`, [AS], in `kAllEngineRuleThresholds`). Hyper and demo axes unchanged
  (both end at 100 kPa); A25A axis for 0.8 bar: 97.27, 101.325, 161.325, 200 kPa.
- `tests/unit/test_forced_induction_conversions.cpp` (registered in tests/unit/CMakeLists.txt): FI1-FI7 `[turbo][drivetrain][cycle]` (7 cases, all under 2 s, none
  `[long]`, 322 assertions) plus hidden `FI-D1` diag (the measured-fit table).
- `tests/unit/test_engine_rules.cpp`: only the G2 comment (points at FI4).
- Docs: `docs/data_formats/turbo.md` ("Hardware family and templates": table, method, templates, how to pick, measured fit), `docs/data_formats/engine.md`
  ("Adding or removing forced induction", ambient-node note and NA-hyper limit in "Grid helpers").

## Deliverable 1: lumped engines need no new grid path
`compressor_supply` exists only for pipe engines (loader sets it iff a turbo exists). A lumped cycle (toyota_a25a_fks_gen) runs every grid point between fixed
reservoirs at the node pressure, so nodes above ambient already generate; nothing was added to the generator or any cache key (shipped engine keys, scenario
hashes unchanged). The only change a lumped boosted grid needed is the helper's ambient node (below).

## Measurements (release, 10 workers, dyno-held WOT, target 0.8 bar gauge)
- A25A gen (lumped, Full maps, CR 10, single_medium; load 0.39 s): charge 0.801/0.800/0.800 bar at 3000/4500/6000 rpm (worst error 0.0005), manifold 0.785/0.766/0.765;
  torque 482.6/464.8/304.4 N*m vs 267.1/260.4/170.0 without the turbo (x1.81/1.78/1.79); trapped air top row / ambient row 2.047 at 4500 rpm; strictly rising
  with p at 2000/4500/6000 rpm.
- demo_i4_pipes + single templates (Sketch): charge held from 2000 (small) / 2500 (medium) / 3500 (large) rpm to 6500; below that late spool (small 0.097 at 1500;
  medium 0.143 at 2000, 0.301 at 2250 unsettled; large 0.079 at 2500, 0.193 at 3000, 0.318 at 3250). Worst charge error in band 0.0002-0.0009 bar. Map key identical
  for all three hardware: the second and third load ran no generator (cache hit).
- hyper_v8_one1_audio + 0.8 bar templates (Sketch): single_large and twin_medium hold 2000-8000 rpm (manifold 0.74-0.79 gauge, rotor 73000 / 82000 rpm at 8000);
  twin_small holds to 6000 and then runs away (1.095 bar at 7000, 1.396 bar at 8000, gate saturated, rotor 327000 rpm, not settled; findings WastegateSaturated,
  BoostAboveTarget, TurboRotorCap, CompressorChoke); twin_large 0.084 bar at 2000 rpm, 0.242 at 2500 (unsettled), holds from 3000.
- hyper with forced_induction "none" + ambient_grid_p_im (Sketch, 0.8 s with 2 dyno points): 374.2 N*m at 3000, 458.5 N*m at 6000 rpm (1371 boosted at 6000);
  the dyno reports MapGridEdge Exceeded at both points (plenum 99-101 kPa vs the helper's 96.26 kPa top node); old boosted grid refused by the loader.
- Cache keys (demo): key(boosted) == stock desc with exactly grid and compressor_supply replaced; flag alone, grid alone, both each differ; six templates give one key.
- Hash check: all 20 `$script:HashCheckScenarios` at --workers 1 and 10 with the worktree ps_run equal master's ps_run (1 worker) and the values recorded in CLAUDE.md.
- FI timings (release): FI1 1.4 s, FI2 0.9 s, FI3 1.3 s, FI4 0.9 s, FI5 0.6 s, FI6 0.007 s, FI7 0.2 s.

## Recorded sabotages (all reverted)
- engine_rules.cpp ambient-node line disabled: FI7 axis check fails (0 == 1 ambient nodes) and the Preview boosted A25A throws "steady torque is not non-decreasing in
  plate position (rpm 1250, plate 0.25 -> 0.275: 186.19 -> 185.71 N*m)". FI1 (Full maps) passes with it - Full is not sensitive, only Preview/Sketch.
- vehicle_io.cpp compressor_supply forced false: FI5 key mismatch (0xcdf1efee2e0d7200 vs 0x9db968a1e0386ce5), FI2/FI3 generation throws "throttle fully open: mean
  plenum 101.2 kPa", FI4 premise fails.
- large turbine area halved: FI2 shortfall (large spools at 2500 rpm: 0.7998 not < 0.7), FI3 twin_large (0.7998 not < 0.4), FI6 monotone sizes.
- large wastegate 3.0 -> 2.36 x throat: FI3 single_large charge 0.994 bar at 8000 rpm (error 0.194 vs 0.05); FI2 still passes (limit within the 6500 rpm tested).
- twin_small count 2 -> 1: FI6 inertia/flow/throat/gate checks fail (e.g. 4.85e-5 vs 9.7e-5). large reference_rpm 105000 -> 120000: FI6 tip-speed similarity fails (106658).

## PROVISIONAL DECISIONS
1. **Ambient node in `boost_grid_p_im` (kept in the helper, not in the test).** Options: (a) helper change; (b) the test/garage passes a hand-built grid; (c) generator
   tolerance for non-monotone coarse tables. (a) pro: every caller gets it, one place, the hyper/demo axes are unchanged (gap under 2 kPa), cache keys of shipped
   engines untouched; con: the helper output for the A25A differs from P4's recorded one (no P4 test pins it). (b) con: every garage caller must remember. (c) changes
   the engine's validation and possibly existing engines' load behaviour. Chosen (a). Threshold 2 kPa [AS]: the A25A is 4.05 kPa short, the hyper/demo 0-1.3 kPa.
2. **Large hardware wastegate 3.0 x the throat (small and medium 2.36 x).** Options: keep 2.36 for the whole family; scale with engine size. 2.36 saturated on the 5 L
   hyper above 7000 rpm (0.994 bar at 8000). A bigger engine per turbo needs a bigger gate; raising it is data only. Pro: single_large holds 2000-8000 on the V8;
   con: one more [AS] number to defend, not derived from a published gate.
3. **Family by similarity, not by three independent guesses.** Tip-speed similarity (rpm x sqrt(flow) const within 1 %), flow/throat/shaft loss x (D2/82)^2, inertia from
   disc formulas (~D^5), efficiencies 0.76/0.78/0.80. Alternative: individual datasheet figures (not allowed: no download; recalled values would be unverifiable).
4. **Garrett naming convention recalled, not re-read** (GT28 60 mm, GTX30 71, GTX42 94): marked [AS] in each file; the quantities that matter (flow, inertia, throat) are
   stated directly, the names are labels.
5. **Informational `rated_power_kw_per_turbo` key in the hardware files.** Alternative: a separate index file. Pro: one file per part, the loader tolerates unknown keys;
   con: a loader that later rejects unknown keys would need to learn this one. FI6 pins the key's shape.
6. **Test quality choices.** Sketch for the pipe engines (0.6 s loads), Full for the lumped A25A (0.39 s; Sketch and old-axis Preview fail), CR 10 for the A25A (stock 13 is
   an Exceeded CompressionRatio issue with forced induction). All cases are fast, so none is `[long]`.
7. **No change to the loader or generator** (brief: only previously failing configs may move keys): the lumped path needed none.

## Open points
- Sketch maps on the boosted lumped A25A are unreliable (3 p_im rows interpolate toward the boosted row and break the steady-torque monotonicity); the garage must use
  Preview or Full for a lumped boosted engine (documented in engine.md). A coarse-table fix in SimulatedEngine would be the real cure (owner decision).
- ECU tables of the A25A (lambda, spark) clamp above 105 kPa and there is no knock limit (C17b): a boosted A25A runs NA spark/lambda at 2 bar absolute; torque is optimistic.
- ambient_grid_p_im's 96.26 kPa top node makes the dyno report MapGridEdge Exceeded for every point of an NA conversion of the hyper (held row, few percent of air).
- twin_small on the 5 L hyper overboosts above 6000 rpm (modelled as designed, not blocked); the garage advisor will show WastegateSaturated/BoostAboveTarget.
- Twins are offered for any engine in the data; whether the garage restricts them to four or more cylinders is the garage's call (nothing enforces it).
- Compression ratio: the garage has to lower it with the conversion (CR <= 10.5); no helper exists for that, the tests write `compression_ratio` into the user engine.
- The turbo family is quasi-steady (no compressor maps, no surge line); spool times are from the lumped inertia only.

## Proposed CLAUDE.md lines (not applied; CLAUDE.md is not mine to edit)
Under `data/` layout, after `turbo_configurations/`:
`turbos/ (P5): small_gt28_class.json / medium_gtx30_class.json / large_gtx42_class.json - three [AS]/[D] hardware sizes by tip-speed similarity from hyper_turbo_approx.json (informational rated_power_kw_per_turbo key); turbo_configurations/ (P5): {single,twin}_{small,medium,large}.json, 0.8 bar gauge, intercooler 0.75, the hyper's PID. Measured bands (2 L demo: small from 2000, medium 2500, large 3500 rpm; 5 L V8: single_large/twin_medium 2000-8000, twin_small overboosts above 6000, twin_large from 3000 rpm) in docs/data_formats/turbo.md.`
Under `drivetrain/engine_rules.cpp` entry: `boost_grid_p_im (P5): adds a node at ISA ambient when the last stock node at or below ambient is more than 2 kPa under it (kBoostGridAmbientNodeGapPa [AS]); the A25A axis for 0.8 bar ends 97.27, 101.325, 161.325, 200 kPa.`
Under `tests/unit`: `test_forced_induction_conversions (engine garage P5, [turbo][drivetrain][cycle], FI1-FI7): lumped A25A + single_medium Full (charge within 0.05 bar, trapped air rising, torque x1.8), demo I4 x three singles (band, shortfall, one map key for all hardware), hyper single_large/twin_medium hold and twin_small/twin_large misfits, hyper forced_induction none via the loader, cache-key relations (flag + grid only), hardware family/template well-formedness, ambient-node guard with a Preview boosted A25A; recorded sabotages in each case comment; hidden FI-D1 prints the measured-fit table.`
