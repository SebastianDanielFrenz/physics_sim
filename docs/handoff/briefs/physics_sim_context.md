# physics_sim agent context (maintained by the coordinator - read this first)

Repo: S:\claude_code\physics_sim (master). CLAUDE.md = architecture map (already in your context; Grep it, do
not re-read it). PLAN.md "I7 scope and order" = owner rulings.
Detailed I7 plan: S:\claude_code\physics_sim\out\i7_plan_backup.md (sections 3 MVEM runtime,
3.4 throttle path, 5 validation T1-T9, 6 commit table C9-C22, 7 decisions).

## Standing rules (binding for every agent)
- CONTEXT BUDGET (a Sonnet agent died of context overflow 2026-09-27): CLAUDE.md (172 KB) is
  already in your context - do NOT Read it again, and do not "read the Drivetrain section" in full;
  Grep it. Never Read a whole file over ~300 lines (powertrain.cpp 125 KB, world.cpp 42 KB,
  i7_plan_backup.md 77 KB, vehicle_instance.cpp 29 KB): Grep -n, then Read offset/limit <= 80
  lines. Pipe build/test/ps_run output through `tail -40` or grep for errors. Keep your progress
  file current so a restart can resume.
- Work in your own git worktree only. Never edit PLAN.md, CLAUDE.md, tools/ci.ps1. Never run
  tools/ci.ps1. No WSL. Do not kill processes you did not start. Install nothing.
- Do not stop to ask: if a decision is needed, take the plan's recommended option, write it into
  your progress file under "PROVISIONAL DECISIONS" with alternatives + pros/cons + reason, and
  continue. The coordinator records it.
- Build: `cmake --build --preset release --parallel 2` (clang-cl OOMs with more), from a VS dev
  shell: `cmd /c "call \"C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat\" >nul && cmake --build --preset release --parallel 2 --target <targets>"`
  (verify the vcvars path exists; configure first with `cmake --preset release` in a fresh worktree).
  Build only the targets you need (ps_unit_tests, ps_determinism_tests, ps_validation_tests, ps_run).
- BUILD LOCK (several agents share the machine): before any cmake build, acquire the lock by
  creating the directory `S:\claude_code\physics_sim\out\build.lock` (mkdir is atomic; if it
  exists, wait 30 s and retry). Write your agent name + UTC time into
  `out\build.lock\owner.txt`. Remove the directory right after the build finishes (success or
  fail). Hold it for ONE build at a time. A lock older than 60 min may be removed (say so in
  your progress file). Low-priority agents (tractor demo) wait 2 min after releasing before
  re-acquiring.
- Tests per change: only the new/changed tests (by name/tag) plus the four hash gates via
  `out\build\release\tools\ps_run\ps_run.exe <scenario> --workers 1` and `--workers 10`:
  vehicle_step_steer 0xbab69b300eba41a3, vehicle_powertrain 0xc72122c8fe8d0228,
  terrain_multi_interest 0xd27f5f629161cee4, vehicle_tractor_i4 0xe471f1f3e5ad4328,
  vehicle_fuel_tank 0xc89f5ed4df82b2dc, vehicle_gen_engine 0x84518e7edcdc3b6a (2026-10-04
  auto-clutch fix values). No full suites.
- No `[!mayfail]` without a discriminating measurement. Every guard test must be shown to fail
  on the defect it guards (do the sabotage, report it, revert it). A ledger identity fed by the
  solver's own quantities checks bookkeeping, not physics (vault TOOL-044 pattern 4) - test
  each linearisation term (tau0, d) white-box against literal values in a regime where it is
  non-zero.
- Data "source" text: only measured or cited facts; derived values labelled DERIVED with the
  method; assumed/copied values labelled ASSUMED. Never invent numbers. car_sedan.json and
  every other shipped data file stay untouched (new variant files only).
- Engine-neutral: no Godot logic in core; adapters stay thin.
- Commits end with `Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>`. Commit on your
  branch; the coordinator merges.
- Keep a progress file `out/<task>_progress.md` in your worktree (usage limits may kill you):
  done / next / measured numbers / provisional decisions.
- Vault (S:\claude_knowledge_base): you may SEARCH it
  (`S:\claude_knowledge_base\knowledge-manager\gradlew.bat -p S:\claude_knowledge_base\knowledge-manager run --quiet --args="search 'terms' 3"`),
  never write to it - draft tickets into your progress file.
- Downloads (owner ruling 2026-09-27): public-domain / open-access sources only (EPA,
  data.gov, open-access papers/reports), saved under S:\claude_code\physics_sim\out\sources\
  (ignored), each logged with URL + size in your progress file. Nothing else.

## Code anchors (written at master 56b5c9c - Grep, lines have moved)
- Engine interface: core/src/drivetrain/engine.h:179 `IEngine` (pre_solve(Sensors, ThrottleCommand,
  TcuRequest, h) -> PreSolveResult{tau0, d, theta_c, torque_demand, ...}; post_solve(omega_new, h);
  idle_integrator/crank_angle/idle_rpm/fold_state_hash/reset). Only impl: EngineTorqueMap
  (engine_torque_map.h/.cpp, 378 lines).
- Powertrain builds the engine at core/src/drivetrain/powertrain.cpp:1211
  (`engine_ = std::make_unique<EngineTorqueMap>(to_engine_desc(e, tank_desc))`); to_engine_desc
  at :74/:153 (fuel resolution, crude-model eta). Engine Sensors filled ~:1719.
- Throttle stage: core/src/drivetrain/throttle_stage.h (ThrottleCommand{command, floor, ceiling}).
- Fuel tank / starvation: energy_store.h, Sensors::fuel_available_kg, s_starve (see CLAUDE.md
  "Fuel tank").
- Loader: core/src/io/vehicle_io.cpp - engine "kind" dispatcher ~:787 ("simulated" currently
  rejected by name - I7b C10 replaces that); read_torque_map_fields :376.
- Public descs: core/include/ps/drivetrain/powertrain_desc.h (TorqueMapEngineDesc, ComponentDesc
  variant, ThrottleControlDesc, FuelTankDesc), fuel_desc.h, table2d.h, ps/vehicle/curve1d.h.
- VehicleTickEnv: core/src/vehicle/vehicle_instance.h:61 (gravity, air_density) - C9 adds
  AmbientSample{p, T}.
- Tests style: tests/unit/test_engine_fuel_map.cpp, test_fuel_tank.cpp, test_throttle_control.cpp,
  test_powertrain_engine.cpp (white-box, core/src on include path).
- Docs: docs/data_formats/engine.md, fuel.md, vehicle.md, scenario.md.

## Current state (updated by coordinator)
- 2026-10-12: aarch64 support 774196c (PHYS-073; PS_TARGET_X86 gating, linux-aarch64-check preset/CI step); assists diag rig 26c523b (PHYS-074, plan pending owner Q1-Q10); P10 Valvetronic merged 1ae7f28 (PHYS-075: intake_lift_family + ECU lift_control, one map set per lift node, pressure-target plate, data bmw_n52b30, test_lift_family.cpp).
- 2026-10-12 master c4e859e: P9 VTEC cam profiles merged 65a37ae (vault PHYS-072): cam `profiles` (2) + ECU `switching`, one .psmaps per profile (engine_map_profile_cache_keys, generate_engine_profile_maps), SimulatedEngine blend, lumped non-breathing fill, rules 11-13, data honda_k20a_dc5 (engine/ecu/cam), tests test_cam_profiles.cpp [vvl]. Audio refuses profile engines (G10).
- 2026-10-12: master d53cf54 (pushed). Engine garage P7 (d4f3b07, knock-limited spark: core/include/ps/engine_cycle/knock.h,
  fuel "knock" block, ECU ignition_strategy "knock_limited", LimitKind::Knock, tests/unit/test_knock_limit.cpp, vault PHYS-069),
  P7b (a037124, knock calibration kKnockCalPrefactorScale, tests/unit/test_knock_calibration.cpp), P7c (a4fedfe, gas-exchange
  diagnostics only, tests/unit/test_gas_exchange.cpp, vault PHYS-070), P7d (6d6d947, pipe WOT row: map_pressure_index.h,
  SimulatedEngineMapsDesc::wot_p_im_pa, kPipeWotGeneratorVersion 4 for ambient-fed pipe engines, tests/unit/test_wot_generation.cpp,
  vault PHYS-071) and P8 (7f52832, launch control + assist fitment, ps/io/electronics_io.h) merged. No cited intake runner lengths
  exist for the A25A or Mazda (runners stay [AS]). Cam part format: docs/data_formats/cam.md (data/cams/hyper_v8_one1.json);
  ECU part: docs/data_formats/ecu.md; slot merge in core/src/io/engine_parts.cpp (assemble_engine).
- 2026-10-11 evening: master 812cbe4 (pushed). Engine garage P2 (445e9f8: part slots ecu/cam/forced_induction/
  installation/thermal, "extends", user layer - core/src/io/engine_parts.*), P4 (05b6e36: ps/drivetrain/engine_rules.h
  check_buildable + engine_change classify_engine_change + boost_grid_p_im/ambient_grid_p_im), P5 (3076403: turbo hardware
  family data/turbos/{small_gt28,medium_gtx30,large_gtx42}_class.json, templates data/turbo_configurations/{single,twin}_*,
  boost_grid_p_im ambient node, tests/unit/test_forced_induction_conversions.cpp FI1-FI7) and P6 (8e2980f: engine_tools/
  ps_engine_tools audio bank baker) merged. Nothing else running.
- 2026-10-11 later: master cb1f475. Engine garage P3 (e47f506: run_dyno_sweep, ps/drivetrain/engine_dyno.h), P3b-1
  (3bdd9dc: limit findings, ps/drivetrain/limit_findings.h, core/src/drivetrain/limit_thresholds.h) and P3b-2 (faf4a01:
  ps/drivetrain/limit_remedies.h solve_remedy/solve_power_target, nitrous in the sweep) merged. Owner: keep kGateNear 0.92.
  Engine garage P2 (slots/extends/user layer, Opus) and P4 (check_buildable, classify_engine_change, grid helpers,
  Sonnet) run in PARALLEL from cb1f475: P2 owns core/src/io/* loaders + data/engines, data/turbo_configurations; P4 owns
  NEW files only (plus CMakeLists/doc additions) - neither edits the other's files.
- 2026-10-11: master 72c0f18 (also on GitHub, origin = SebastianDanielFrenz/physics_sim; agents never push).
  Merged since 10-06 by the coordinator: N7b (car_hyper_n2o turbo override), engine garage P1 (5634f04: MapQuality
  Sketch = 2, EngineMapOptions cancel/on_progress/generate, JobSystemOptions background_priority, public
  engine_map_cache_key - tests/unit/test_engine_map_control.cpp), boost-aware idle feed-forward (9ea2cfb,
  tests/unit/test_hyper_idle_recovery.cpp), tyre rolling-resistance speed term (d1e1144, qsy3 +
  "rolling_resistance" block; every vehicle scenario hash moved). Landed 10-07..10-09 by OTHER sessions (read
  their docs before touching the area): offline aero coefficient maps (AERO_COEFFICIENT_MAPS.md,
  ps/aero/coefficient_map.h), adaptive engine pressure maps (ps/drivetrain/adaptive_pressure.h,
  docs/data_formats/adaptive_pressure.md), offline turbo stage maps (ps/drivetrain/turbo_stage_maps.h,
  docs/data_formats/turbo_stage_maps.md, turbo_pair.h changes), inlet shock entropy / compressible exhaust option,
  a quadratic rolling-resistance option (accf416, docs/data_formats/tyre.md), traction control
  (ps/vehicle/traction_control.h, tests/unit/test_traction_control.cpp), hypercar mesh, tracked custom nitrous parts.
- 2026-10-04 later: master b486223 = auto-clutch fix b26f318 + docs 61b10f4 + I7d D1-D4 merged (blow-off valve,
  surge flag, intercooler drop, turbo sound fields, docs/data_formats/turbo.md; tests tests/unit/test_turbo_*.cpp,
  fixtures tests/support/turbo_fixtures.h, tests/support/fixtures/turbo_config_hyper_bov.json). Hash gates above
  are the committed master values.
- 2026-10-04 (earlier): I7a-I7c done; I7d partial. Turbo = core/src/drivetrain/turbo_pair.h (TurboPair, header-only, lumped
  pair: quasi-steady head, 25 ms charge lag, PID wastegate, rotor energy), used only by SimulatedEngine
  (engine_simulated.cpp: Grep `turbo_` - construction ~:507, hot swap replace_turbo_configuration ~:579,
  step call ~:821, charge/exhaust used ~:663/:894, PreSolveResult fill ~:913, hash fold ~:965). Desc:
  TurboPairDesc in ps/drivetrain/powertrain_desc.h ~:287, TurboPerformanceMaps in turbo_maps.h. Loader:
  load_turbo_configuration_json in core/src/io/vehicle_io.cpp ~:2340. Sound state: EngineSoundState in
  ps/drivetrain/powertrain_state.h, filled by core/src/drivetrain/engine_sound_state.cpp. Data:
  data/turbos/hyper_turbo_approx.json, data/turbo_configurations/hyper_twin_default.json, engine
  data/engines/hyper_v8_one1_audio.json, car data/vehicles/car_hyper.json. Existing turbo test:
  tests/unit/test_simulated_engine.cpp "Twin turbo hyper V8 ..." [turbo][sim_engine] (its engine load
  generates maps - slow; use --engine-cache style options if the test offers them). Plan for the rest:
  S:\claude_code\physics_sim\out\i7d_completion_plan.md.
- 2026-09-29 early (master 56bfaec, default CI green): A8 (radiation, ps_audio/source.h Calibration 200 Pa
  at 1 m, SourceBlock, flow noise off by default), A10 (public ps::EngineRig in ps/drivetrain/engine_rig.h,
  ps::io::load_engine_json, drivetrain::fill_engine_sound_state shared with VehicleInstance, tool
  tools/ps_engine_audio with --wav/--live via miniaudio, demo engine data/engines/demo_i4_pipes.json [AS] +
  data/installations/demo_i4_exhaust.json, scripts data/scripts/engine_audio/) and A8b (runtime CFL
  sub-cycling in PipeNetwork - the sustained-WOT blow-up; catalyst laminar resistance; open-end inflow fix;
  pipe generator v3) are merged. The demo engine is back on its default audio dx (32/17 mm) and renders at
  0.83x real time in the tool (EngineRig + voice, scalar) - NOT real time. The EngineRig->Voice glue
  (EngineSoundState -> ps_audio::EngineInput) lives in tools/ps_engine_audio/main.cpp.
- 2026-09-28 evening (master 039d8ac): ps_audio A1-A5 merged. ps_engine_cycle target (never links ps_core)
  holds the cycle model, PipeNetwork (A2, FV), WaveguideNetwork (A3) and CycleEngine (A5:
  core/include/ps/engine_cycle/cycle_engine.h - n CycleSteppers in firing order + one PipeNetwork from the
  engine's intake/exhaust blocks and the resolved installation; flow-balance throttle trim; exact port-source
  hand-off). Pipe engines generate maps as generator v2, lumped ones stay v1. Pipe fixture engine:
  tests/support/cycle_pipe_fixtures.h. Measured: ~12 us per CycleEngine step at power grade (dx 50 mm), 0.22 s
  per Full grid point. Installations: data/installations/*.json (A4), docs/data_formats/installation.md.
  Spec for all ps_audio increments: out/audio_plan.md.
- 2026-09-28 night (master 6594ec0): ps_audio A7 merged - new top-level audio/ (target ps_audio, links
  ps_engine_cycle only): LiveEngine (steps EngineCore per 48 kHz sample), Voice, TimeMap, Runner, SPSC queue/ring;
  tests tests/unit/test_audio_voice.cpp / test_audio_time_map.cpp; bench tests/bench/bench_audio_voice.cpp (not
  CTest). EngineCore (shared by CycleEngine and LiveEngine) is in core/include/ps/engine_cycle/cycle_engine.h.
  Scalar I4 voice ~0.9 core. ci.ps1 topics now include `cycle` and `audio`.
- 2026-09-28 (master 3d2a0b5, default CI green): I7a C1-C7, I7b C9-C12, I7c C15a (0D cycle
  model, core/src/engine_cycle/*), C16a (map generator: core/include/ps/drivetrain/engine_maps.h,
  core/src/engine_cycle/map_generator.cpp, loader `cycle` block in vehicle_io.cpp, ps_run
  --map-workers) and C17a-MB (toyota_a25a_fks_sim.json, car_sedan_sim.json) are all merged.
  SimulatedEngine: core/src/drivetrain/engine_simulated.h/.cpp; its fold_state_hash folds the maps'
  content hash (engine_maps_content_hash). Spec for the remaining I7c: out/i7c_plan.md.
  The bullets below are HISTORICAL (C7 era); line numbers in "Code anchors" may have moved - Grep.
- I7a C1-C6 merged. C7 (cited Toyota A25A-FKS EPA data: data/engines/toyota_a25a_fks.json,
  data/fuels/tier3_e10_cert.json, car_sedan_fuel.json, tractor_fuel.json, vehicle_fuel_tank
  scenario) in review fixes, merges tonight. C8 (fuel docs) after.
- EPA A25A source data (read-only): S:\claude_code\physics_sim\out\c7_sources\ (test_data\ =
  measured xlsx/report/plots/SAE 2019-01-0249; alpha_map\ = EPA's derived map, cross-check only).
  Stdlib xlsx reading only (no openpyxl). C7 review findings (enrichment rows at low rpm/high
  load, lambda 0.63-0.73; Willans low-load artefact):
  C:\Users\sebas\AppData\Local\Temp\claude\S--claude-code-physics-sim\094bd4b1-17d1-4c89-9e37-5d1204e63823\scratchpad\c7_review\review.md
- Owner rulings 2026-09-27 (night): I7b then I7c overnight; the FIRST simulated engine (I7c C17)
  is the Toyota A25A-FKS (EPA test-cell data = dyno reference), second held-out engine = Mazda
  SkyActiv-G 2.0 EPA data. Priority: a car with a simulated engine by morning beats the tractor
  demo; the tractor demo runs in parallel at low priority.

## Engine internals you would otherwise look up (graph engine = EngineTorqueMap)
- Structs in core/src/drivetrain/engine.h (namespace ps::drivetrain, namespace scope):
  - `Sensors{omega (crank omega_old), ignition (>0.5 on), starter (>0.5), clutch_engaged_fraction,
    gearbox_in_neutral, fuel_available_kg (=+inf without a tank)}` - C9 adds the ambient here.
  - `ThrottleCommand{command, floor, ceiling}` (from throttle_stage.h; electronic: floor/ceiling =
    lagged TCU auto-blip floor / post-upshift ceiling; mechanical: 0/1 fixed).
  - `TcuRequest{ignition_cut}` (one substep lagged).
  - `PreSolveResult{tau0, d, friction_cap_static, friction_cap_kinetic, state, rpm, throttle_eff,
    limiter_active, dfco_active, starter_active, torque_estimate, governor_setpoint_rpm,
    torque_demand (net crank torque the demand asks for IGNORING the limiter - TCU clutch-close
    feed-forward), theta_c (post-limiter combustion fraction, sound "load"), fuel_flow_g_s,
    fuel_supply_fraction (s_starve)}`.
  - `PostSolveResult{engine_out_delta, starter_in_delta, indicated_work_delta (= h*omega_new*tau_comb),
    fuel_mass_delta (kg), fuel_energy_in_delta (J)}`.
  - `IEngine` virtuals: pre_solve, post_solve, idle_integrator, crank_angle (NOT hashed), idle_rpm,
    fold_state_hash, reset(EngineState). EngineState (ps/drivetrain/powertrain_state.h):
    Off/Stalled/Cranking/Running (uint8).
- EngineTorqueMap::Desc (engine_torque_map.h:77): wot/motoring Curve1D (motoring x[0]==0, y<=0;
  y[0] = -c_k, the Coulomb part, carried by a separate friction row to ground with
  breakaway_torque_nm static capacity), inertia_kgm2, idle/stall/fire/start_complete rpm,
  dfco_enable/reenable rpm, IdleControlDesc (PI on throttle), LimiterDesc (FuelCut with hysteresis
  latch, or SoftCut), StarterDesc (ideal DC starter referred to the crank, interlock), initial_state,
  governor (all-speed, droop), fuel (FuelDesc), fuel_map (Table2D g/s over rpm x theta_c),
  has_fuel_model, eta_i (crude model). Hashed state: state_, idle_integrator_, limiter_cut_,
  dfco_active_. Net torque: tau0 = s*(m + c_k) + theta_c*(w - m) + tau_start; d from curve slopes.
  Idle feed-forward theta_ff_ is computed once at construction. The idle PI's own
  d(theta)/d(omega) is NOT in d (known backlog item).
- Powertrain (core/src/drivetrain/powertrain.cpp): engine built ~:1211 behind
  `std::unique_ptr<IEngine> engine_`; `engine_friction_row_` = the Coulomb row;
  `engine_has_fuel_model_` gates fuel booking; if initial_state is Running the crank omega is seeded
  to idle (bootstrap, ~:1226). Tank check + FuelTank construction just above (~:1190-1209). Public
  desc variant: powertrain_desc.h:428 `using ComponentParams = std::variant<TorqueMapEngineDesc,
  ClutchDesc, GearboxDesc, ...>`; the engine is found via `std::get_if<TorqueMapEngineDesc>` at
  powertrain.cpp:218 and :303 (component scan / port setup) - a simulated engine desc needs a sibling
  alternative and those sites (+ declare_control_channels, to_engine_desc users, vehicle_io)
  extended. Per substep (pre_solve_components ~:1700-1730): clutch pre_solve -> throttle stage
  (evaluated earlier, before the TCU, when gearbox+TCU exist) -> Sensors filled ->
  `engine_->pre_solve(s, throttle_cmd, req, h)` -> `assembly_.torque(engine_crank_, tau0, d)` +
  `set_friction_capacity(engine_friction_row_, ...)` -> gearbox loss/synchro -> components ->
  compliant shafts -> solve. post_solve_components ~:1898: `engine_->post_solve(omega_new, h)`;
  ledger engine_out/starter_in/indicated_work always; fuel_mass/fuel_energy_in/engine_heat/
  fuel_efficiency_violations and the tank withdraw only with a fuel model; then the Coulomb row's
  engine_friction_loss = -h*v*lambda.
- Snapshot / sound: VehicleInstance fills PowertrainSnapshot (ps/drivetrain/powertrain_state.h:
  engine_rpm, engine_state, engines[0] = EngineSoundState{crank angle, load = theta_c, limiter, dfco,
  starter, throttle_command, fuel_flow_g_s}, fuel_flow_g_s, fuel_mass_kg) - never hashed.
  PowertrainDebug (tests only) has throttle_eff, idle_integrator, engine_torque_demand.
- Loader: core/src/io/vehicle_io.cpp - an engine component's "ref" loads a physics_sim.engine/1
  file; kind dispatcher ~:787 ("torque_map" or absent -> read_torque_map_fields :376; "simulated"
  rejected by name); throttle_input parse ~:493; fuels/fuel/ignition/indicated_efficiency/fuel_map
  parsed in the same function (C5/C5d). load_fuel_json lives in core/src/io. Docs describe every
  field: docs/data_formats/engine.md (fuel_map section ~:217-262).
- Tests to copy the style from: tests/unit/test_powertrain_engine.cpp (white-box EngineTorqueMap
  pre_solve with hand values, finite-difference slope checks, state machine), test_fuel_tank.cpp
  (T10 = white-box pre_solve per regime), test_engine_fuel_map.cpp, test_throttle_control.cpp (P1
  static_assert that the engine input has no pedal field; P2 bit-identity),
  test_vehicle_parallel_alloc.cpp (debug_alloc-armed zero-alloc World stepping),
  tests/determinism/test_worker_count_hash.cpp (scenario hash at 1/2/4/10/24 workers + an effect
  case). Rigs: tests/support/vehicle_test_helpers.h build_vehicle_rig()/build_vehicle_rig_from_desc().
  Register a new test file by adding one `test_xxx.cpp` line to tests/unit/CMakeLists.txt's source
  list (~:51 area); catch_discover_tests uses ADD_TAGS_AS_LABELS. Topic tags: [drivetrain], [fuel],
  [io], [vehicle], [determinism]. Run one test:
  `out\build\release\tests\unit\ps_unit_tests.exe "[tag]"` or by test name; determinism:
  `out\build\release\tests\determinism\ps_determinism_tests.exe`.
- Determinism rules: no platform libm transcendental in per-substep code (only + - * / sqrt and
  table lookups; ps::math::pow/sincos/exp at load time only); never iterate an unordered container
  for anything that decides state; hashing = FNV-1a fold of raw bytes in a fixed order.
- Machine: 24 logical cores, but clang-cl OOMs with parallel builds - hence --parallel 2 + the lock.
- A fresh worktree needs `cmake --preset release` (FetchContent: Jolt/Catch2/json - the first build
  takes ~10-20 min). Build only the targets you need.

## Tractor Godot demo anchors (low-priority task)
- adapters/godot/demo/scripts/main.gd (1014 lines): hard-coded sedan chassis consts at :42-51
  (CHASSIS_MASS 1500, CHASSIS_HALF_EXTENTS (2.0, 0.4, 0.15), CHASSIS_Z_OFFSET 0.6);
  @export vehicle_relative_path/model_relative_path :53-55; spawn_id; command-line args
  `--m1-vehicle=<name>` (~:309), `--m1-autopilot`, `--m1-spawn=`; control channels set in _process
  :655-675 (steer/throttle/brake/handbrake/clutch/shift_up_count/shift_down_count/ignition/starter,
  assist toggles). Tractor channels to add: awd, diff_lock, pto, pto_speed (540/1000), park_lock,
  hand_throttle (all declared by tractor.json - see CLAUDE.md "Drivetrain").
- Tractor chassis body as validated in data/scenarios/vehicle_tractor_i4.json: mass 6000 kg, box
  half_extents [1.2, 0.35, 0.9], position z 1.28, friction 0.5, allow_sleep false. Model:
  data/models/tractor/tractor.glb + tractor.rig.json (wheel pivots/radii; READ-ONLY, owned by
  another session). Sedan wheel placement: demo/scripts/vehicle_visual.gd (234 lines).
- Input: adapters/godot/src/input_map.cpp / input_map_core.* (PsInputMap, Xbox + keyboard); HUD
  demo/scripts/hud.gd, tach_gauge.gd; PsSimulation (src/ps_simulation.cpp) publishes
  PowertrainSnapshot (awd_engagement, park_lock_engaged, pto_engagement/pto_mode/pto_rpm,
  governor_setpoint_rpm, hand_throttle, diffs[].locked).
- Check: adapters/godot/smoke_test.ps1 (own build dir out/build/godot-adapter, runs the demo
  headless with --m1-autopilot, fails on a FALL DETECTED block). Headless smoke runs are allowed;
  never open an interactive Godot window, and never close a Godot the owner started.
- Keep the logic engine-neutral (thin adapter); keep the sedan the default and the tractor
  selectable (e.g. --m1-vehicle=tractor + a per-vehicle chassis/model table, not hard-coded consts).
