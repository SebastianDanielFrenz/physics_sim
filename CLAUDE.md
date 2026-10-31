Also look at AGENTS.md.

# physics_sim — architecture map

Pure map of what exists and how to build/test/run it. Rationale and design
decisions live in the knowledge vault (`S:\claude_knowledge_base`), not here.
Full design/phase plan: `PLAN.md`.

## Layout

```
physics_sim/
  CMakeLists.txt, CMakePresets.json   top-level build config
  README.md                           setup pointer
  CLAUDE.md                           this file
  PLAN.md                             design and phase plan
  AGENTS.md, RUNTIME_LOCKS.md         shared Godot runtime coordination: out/runtime_locks/godot_<PID>.json libps_godot.dll ownership records, overnight sanitizer rule
  AERO_COEFFICIENT_MAPS.md              optional offline-CFD table schema, public selection loader, native interpolation/ownership and telemetry
  AERODYNAMICS.md, LIVE_ATMOSPHERE_PERFORMANCE.md, HYPER_CLUTCH_FLARE.md, NATIVE_ENGINE_OUTPUT.md, HANDOFF_TURBO_AUDIO.md
                                      subsystem notes from the racing_game integration sessions; every repo .md with a one-line summary: vault PHYS-060
  drive.ps1                           incremental adapter build + launch Godot on the demo ("build and go drive"); dot-sources adapters/godot/godot_common.ps1
  drive.cmd                           thin cmd.exe wrapper around drive.ps1 (so `drive` works the same from cmd.exe, PowerShell or double-click)
  tools/
    setup_dev_env.ps1                 installs/verifies the dev toolchain (-CheckOnly to only verify)
    ci.ps1                            CI at three levels: -Affected/-Topic (per change), default fast, -Full (see "CI" below)
    tsan_suppressions.txt             ThreadSanitizer suppressions, loaded only by the linux-tsan TEST preset's TSAN_OPTIONS (one entry: Jolt's debug-assert read in JPH::Mutex::try_lock)
    common/                           tools/tests-only helpers, never linked into ps_core; target ps_headless_env
      headless_env.h/.cpp             installs Windows headless-CRT handlers (no blocking assert/abort dialogs)
      engine_sound_glue.h             ps_audio A11: header-only ps_tools::make_engine_input (EngineSoundState -> ps_audio::EngineInput), the ONE mapping shared by tools/ps_engine_audio and the Godot adapter's VehicleVoice (test_engine_sound_glue)
    ps_run/                           headless scenario runner (JSON scenario -> CSV, pure function); target ps_run
    ps_engine_audio/                  ps_audio A10 listening tool: `ps_engine_audio <engine.json> --wav out.wav --script events.json [--installation f] [--seconds S] [--seed N] [--map-workers N] [--engine-cache DIR]` or `--live` (miniaudio, FetchContent, tool-only); hosts ps::EngineRig + a ps_audio Voice (thin glue), own WAV writer; scripts in data/scripts/engine_audio/ (free_rev.json, stall.json); 3db5525/e1a9671 add --input-csv (replay a recorded PS_VOICE_INPUT_CSV), --diag-csv, --plenum-*-tau-s, --dyno-viscous-nms, --hitch-*, --dx-scale, --audio-bank (generate/load the bank exactly as the demo does, via engine_tools/ ps_engine_tools, engine garage P6), --audio-bank-v1, --bank-reference-playback; tools/plots/engine_order_track.py measures engine-order amplitude tracks (Hann STFT, band around order*rpm/60)
    sweep/                            fans N scenarios out over local ps_run processes (--jobs N); target sweep
    plots/                            python telemetry tools, dev only: rpm_to_wav.py (stdlib-only crude 4-rotor synth from a t/engine_rpm CSV, firing 4*rpm/60 - the 787B lift-off preview; pulse shape/resonance are artistic, labelled so; optional --fm-reference REF.csv --fm-exaggerate G magnifies the rpm difference to a reference run, labelled EXAGGERATED, a listening aid only - default output unchanged)
    enginedata/                       python engine-data generators, dev only: a25a_mb_engine.py (PHYS-033 P4 I7c C17a fallback M-B: EPA NVFEL A25A xlsx -> data/engines/toyota_a25a_fks_sim.json; --sabotage {transpose_ve,reverse_ve_rows,kpa_as_pa} for the data test's corruption check); a25a_gen_engine.py (I7c C17a M-A: out/a25a/a25a_tables.json -> data/engines/toyota_a25a_fks_gen.json's cycle block, deterministic, same bytes on re-run); k20a_vtec_cam.py (garage P9: writes data/cams/honda_k20a_dc5_vtec.json, [AS] lifts); n52_valvetronic_cam.py (garage P10: writes data/cams/bmw_n52b30_valvetronic.json)
                                      knock calibration phase A: epa_turbo_extract.py (EPA NVFEL turbo workbooks -> out/epa_turbo/ + tests/support/fixtures/epa_turbo/<package>_rows.json, physics_sim.epa_turbo_rows/1: high-load rows, envelope = highest torque per 50-rpm bin), epa_turbo_fuels.py (data/fuels/epa_*.json with [C] knock blocks), epa_turbo_engines.py (tests/support/fixtures/epa_turbo/{honda_l15b7,ford_27,ford_16,mazda_25t}/engine.json + turbo_configuration.json, knock_limited, A25A sizing rules); smoke test tests/unit/test_knock_cal_fixtures.cpp KC1-KC2 [knock_cal]
    turbo_sim/                        offline Python (stdlib) turbo stage-map generator: meanline.py (physics_sim.turbo_geometry/1 -> physics_sim.turbo_stage_maps/1), build_examples.py, benchmark_maps.py, test_meanline.py (python -m unittest discover -s tools/turbo_sim); not CTest; ci.ps1 PathRules kind none; the REFERENCE for the C++ port ps/drivetrain/turbo_meanline.h (below)
    assetgen/                         Godot art-asset generator (Python; coordinate source ownership before edits); hyper_topology_check.py verifies final hypercar node topology, flow selection ranges/hash and rig; data/models/car_hyper/GEOMETRY.md describes visual closure and unresolved CFD preparation
  cmake/toolchains/aarch64-linux-gnu-clang.cmake   clang cross toolchain for native Linux ARM64 (Steam Frame; merge 774196c, vault PHYS-073): --target aarch64-linux-gnu + lld, optional env PS_AARCH64_SYSROOT (Steam Runtime 4 ARM64 SDK later, untested) / PS_AARCH64_GCC_TOOLCHAIN, qemu as CMAKE_CROSSCOMPILING_EMULATOR when present; preset linux-aarch64-check
  core/
    include/ps/                       public API headers (no Jolt or Godot types); target ps_core (STATIC)
      version.h                       ps::version_string(), ps::jolt_backend_self_check()
      types.h                         ps::real = double; id_index_t/id_generation_t; SurfaceId (uint16_t) + kInvalidSurfaceId
      math/                           vec3.h, quat.h, mat3.h, pose.h (Pose/Motion/Wrench incl. Wrench::operator*(real)), simd.h (f64x4 skeleton)
      jobs/                           job_fn.h (JobFn), job_system.h (JobSystem, WaitGroup, parallel_for/parallel_batches), task_graph.h, debug_alloc.h
      machine/                        attachment A1, public descriptors: hitch_class.h (HitchClassDesc, HitchAxis free/limited/locked per Cardan angle, row frequency and damping, latch time and limits), hitch_point.h (HitchPointDesc: a vehicle file's hitch_points entry, HitchRole, find_hitch_point), tow_brake.h (TowBrakeDesc: channel link or overrun), coupling.h (CouplingDesc, AttachCheck, CouplingState, last tick's outcome, unhashed), hitch_limits.h (check_hitch_limits, HitchLoads)
      world/                          ids.h (BodyId/JointId/ForceElementId/ArticulationId/HitchId), frame.h (Frame, ILinkStateSource, ArticulationLinkRef), force_element.h (IForceElement), snapshot.h (WorldSnapshot incl. ArticulationSnapshot/contacts, SnapshotTripleBuffer), world.h (World, WorldConfig), ambient.h (AmbientSample{pressure_pa, temperature_k}, ISA sea-level default - PHYS-033 P4 I7b C9) ; door.h (kinematic doors, merge claude/kinematic-bodies: DoorDesc{kind Hinged|Sliding (a roller door is a rigid slide), closed pose, slide / pivot+axis+open_angle_rad, speed, max_penetration_m, max_push_mass_kg, unpushable_penetration_m, probe_shape} - all config, DoorState{fraction, command, moving, blocked, blocker}, DoorDriver value type: closed-form pose_at(fraction), allocation-free tick(state, dt, probe) -> DoorTick{move, stop, target}; no backend/Godot dependency)
      backend/                        i_rigid_backend.h (IRigidBackend, BodyDesc/JointDesc, ShapeContactHit, ContactEvent, BodyIntegrationParams (PHYS-033 I1), create_jolt_backend()); shape_desc.h (ShapeDesc/BoxShape/SphereShape/CapsuleShape/MeshShape/HeightfieldShape - split out of i_rigid_backend.h so articulation_desc.h can use ShapeDesc without pulling in the backend's JointType); shape_handle.h (R2: ShapeKind, opaque PreparedShape, ShapeHandle = shared_ptr<const PreparedShape>, debug_live_prepared_shape_count - see "Jolt backend")
      articulation/                   articulation_desc.h (ArticulationDesc incl. contact_strategy, RootDesc, LinkDesc incl. contact, JointType, JointLimits, VelocityActuator, LoopClosureDesc/LoopClosureType, ContactStrategy, ContactMaterial, LinkContactDesc) - descriptors only, no algorithm code
      terrain/                        terrain_source.h (ITerrainSource, TileKey, TileSample), terrain_config.h (TerrainConfig - pool sizing only, NOT grid resolution, see "Terrain tile system" below), procedural_terrain.h (load_procedural_terrain_source() factory)
      vehicle/                        public, additive descriptors/seams only (PLAN.md 5.1/P3, no algorithm code - see "Vehicle system" below): vehicle_desc.h (VehicleDesc/WheelDesc/SuspensionDesc/SteeringDesc - PHYS-033 I2 commit 5a: BrakeDesc/DriveDesc/WheelDesc::driven are gone, replaced by VehicleDesc::brake_system/powertrain below), curve1d.h (Curve1D - split out of vehicle_desc.h in I2 commit 5a so drivetrain/powertrain_desc.h can use it without a vehicle_desc.h <-> powertrain_desc.h include cycle), tyre_desc.h (TyreDesc, Pacejka MF5.2/6.1-subset coefficients, qsy3 (tyre crr d1e1144: MF 6.1 linear speed term, default 0; crr = qsy1 + surface crr + qsy2|Fx|/Fz0 + qsy3|v/v0| + qsy4(v/v0)^4; tyre_rolling_speed_term(desc, vx) is the ONE speed law - the wheel and every test reference call it; derive_rolling_resistance(anchors, v0) closed form B = ((F_r-1)f-(F_a-1))/(f-f^4), A = (F_r-1)-B; a tyre file's optional rolling_resistance block {rated_speed_kmh, crr_base, rise_at_fraction{fraction, factor}, factor_at_rated} derives qsy1/qsy3/qsy4, explicit qsy1/3/4 next to it, unknown keys, A or B < 0 and out-of-range inputs rejected 'rolling_resistance: ...'; all five data/tyres files use 0.8 -> 1.5x, rated -> 2.0x [AS], rated 270 (passenger, W), 500 (hyper [AS]), 65 (tractor, agricultural D) km/h; docs/data_formats/tyre.md; tests test_tyre_rolling_resistance.cpp R1-R5, tests/validation/test_hyper_top_speed.cpp (fixture vehicle_hyper_torque.json: flat-road reference 446.3 km/h limiter-limited, sim 450.3; was 371.7 drag-limited), car_sedan reference top speed 218.0 km/h), low_speed_damping_ratio/low_speed_damping_vx - PHYS-033 I1, rolling_resistance_v_eps - parking-slope creep fix D1), tyre_model.h (ITyreModel, TyreSlipInput/TyreForces, TyreForcesWithSlope, evaluate_with_slope(), create_pacejka_tyre_model(), SlipRelaxationState/step_slip_relaxation()/relaxation_step_axis() - PHYS-033 I2), wheel_telemetry.h (WheelTelemetry, returned by World::wheel_telemetry()), wheel_state.h (WheelState, returned by World::wheel_state() and carried in WorldSnapshot::vehicles - PLAN.md M1)
      drivetrain/                     public, additive descriptors only (PHYS-033 P4 I2 plan section 2, no algorithm code - see "Vehicle system" below): powertrain_desc.h (PortRef/ShaftDesc/EngineLimiterDesc/TorqueMapEngineDesc/GearPairDesc/DifferentialDesc/ComponentDesc/PowertrainDesc; I7a C3: ThrottleInput, TorqueMapEngineDesc::throttle_input, ThrottleControlDesc; I7a C6: FuelTankDesc (a ComponentDesc variant), FluidLinkDesc, PowertrainDesc::fluid; I7b C10: SimulatedEngineDesc (a ComponentDesc variant, an engine file's "kind": "simulated" - docs/data_formats/engine.md)), brake_system_desc.h (BrakeCornerDesc/HandbrakeDesc/BrakeSystemDesc), energy_ledger.h (PowertrainEnergy; I7a C5 adds the unhashed fuel sub-ledger, see "Drivetrain"; I7b C12 pumping_work/friction_work - cumulative, unhashed, outside balance_error, 0 from the graph engine), table2d.h (Table2D - PHYS-033 P4 I7a C1: header-only bilinear 2D lookup, row-major values[ix*y.size()+iy], clamped at every edge like Curve1D, dvalue_dx/dvalue_dy with Curve1D's slope convention (interior knot = left segment, 0 at/beyond an edge), validate(); for the I7a fuel map), engine_maps.h (PHYS-033 P4 I7c C16a: EngineMaps, MapQuality Preview 0 / Full 1 / Sketch 2 (engine garage P1, merge 5634f04; never renumber - the value is folded into the .psmaps key, content hash and header; Sketch = <= 8 rpm x 3 p_im rows (lowest, nearest ambient, top) + 6 spark + 5 lambda points = 35, Preview step settings, own key, for the garage dyno preview, never driven), EngineMapOptions{quality, pool, cache_dir (I7c C16b: opt-in .psmaps disk cache, empty = no disk access), generation_count_out test hook; P1: cancel (atomic flag read before each point - GenerationCancelled after the running points finish, no cache file, also checked before the cache lookup), on_progress (done 1..total once per finished point, always on the CALLING thread, workers only bump an atomic so jobs stay allocation-free; no call on a cache hit), generate = false (validate-only: every loader rejection plus validate_engine_map_generation, no generator/cache/turbo-cache access, SimulatedEngineDesc::maps_pending = true and SimulatedEngine/World refuse a pending engine)}, GenerationCancelled, MapGenerationControl{cancel, on_progress} as generate_engine_maps' trailing argument, public engine_map_cache_key; an unknown quality byte in a .psmaps header is a miss, EngineMapGenTask, generate_engine_maps, engine_maps_content_hash, kGeneratorVersion 2 (ps_audio A5: kLumpedGeneratorVersion 1 for engines without pipe blocks - maps and cache keys bit-identical to v1 - and kPipeGeneratorVersion 3 (A8b: catalyst monolith + inflow fix; was 2; since P7d only compressor-fed or adaptive pipe engines), kPipeWotGeneratorVersion 4 (engine garage P7d: ambient-fed pipe engines, engine_map_generates_wot_rows; kGeneratorVersion 4); engine_map_generator_version(desc) picks one, the cache key and the load use it)), installation_desc.h (ps_audio A4: InstallationDesc - intake airbox/snorkel and the exhaust element chain (pipe, catalyst, silencer, branch, outlets) downstream of the engine's first flange; loaded from data/installations/*.json, physics_sim.installation/1, docs/data_formats/installation.md; engine_cycle_desc.h gained IntakeCycleDesc/ExhaustCycleDesc (runners, primaries, merge DAG), ResolutionCycleDesc and CycleModelDesc::{intake, exhaust, default_installation_path, installation, installation_path, resolution} - data only until A5), fuel_desc.h (FuelDesc/FuelPhase - PHYS-033 P4 I7a C4: name, phase (liquid only), lhv_j_kg, afr_stoich, density_kg_m3, source; loaded by ps::io::load_fuel_json)
      io/                             telemetry.h (TelemetryRegistry, CSV via IO thread), scenario.h (Scenario incl. ScenarioArticulation/ScenarioTerrain/ScenarioVehicle/ScenarioControlEvent, load_scenario, build_scenario), surface_table.h (SurfaceTable), test_ground_loader.h (load_test_ground_collision()), fuel_io.h (load_fuel_json() - physics_sim.fuel/1, PHYS-033 P4 I7a C4: every field required, phase must be "liquid", numbers > 0 and finite, unknown keys ignored), vehicle_io.h (load_tyre_json(), load_vehicle_json() - physics_sim.vehicle/2 as of PHYS-033 I2 commit 5a, see "Vehicle system" below; I7a C5: an engine file's "kind" dispatches - "torque_map" (or absent) loads, "simulated" is rejected until I7b; optional "fuel_map" (Table2D, g/s over rpm x theta_c) + "fuel" (relative path to a physics_sim.fuel/1 file); I7a C5d: "fuels" (list of relative path strings, the first is burnt until C6; "fuel" is shorthand for a one-entry list; object entries, both keys, an empty list, a duplicate file (std::filesystem::equivalent) and "fuel_map" with "fuels" rejected), "ignition" ("spark"/"compression", required on any fuelled engine - an ENGINE property, fuel files carry none) and optional "indicated_efficiency" in (0,1) (rejected with a fuel_map) - docs/data_formats/engine.md; ps_audio A4: a cycle block's intake/exhaust pipe pair (both or neither; mutually exclusive with runner_length_mm/runner_diameter_mm, which are then zero - A5 made CycleStepper pipe-aware: no lumped runner, its ports take constant per-step sources from the PipeNetwork port flux), resolution (CFL floor checked against literal copies of the pipe wave-speed/step constants), default_installation (required with pipes, rejected without); load_installation_json(); a vehicle's per-engine "installation" override, legal only on a simulated engine with pipe blocks; N7b: a vehicle's per-engine forced-induction override (engine garage P2: the "forced_induction" slot, was "turbo_configuration" - the old key is rejected with a rename message), resolved relative to the vehicle file, allowed next to "ref", replaces the default before the turbo tables are built; since P2 it may also ADD forced induction to a simulated engine without one (still rejected on torque-map engines); engine garage P2 (merge 445e9f8): PartOverride{ref, slots}, VehicleLoadOptions{maps, part_overrides} and load_vehicle_json(path, VehicleLoadOptions) (the user layer, keyed by component id, paths relative to the vehicle file unless absolute), EngineSlotOverrides on load_engine_json, load_engine_json_from_string(text, base_dir, options, slots); engine keys default_ecu, default_cam, default_forced_induction (a path or "none"; renamed from default_turbo_configuration, the old key rejected), optional mass_kg/mass_source (in no key and no hash); vehicle engine component slots ecu, cam, forced_induction, installation, thermal; a slot's file is chosen by the highest level that sets it (extends chain < inline < engine default_* < vehicle component < user layer) - content edits go through "extends" in the part file; inline content AND a slot for the same part is rejected; "none" on a pipe engine whose grid_p_im (or pressure_sampling.max_kpa) exceeds 101.325 kPa is rejected with the fix (a user engine that extends this one with an ambient grid)); tyre injection (merge c79622b): VehicleLoadOptions::tyres (std::map wheel name -> TyreDesc) replaces that wheel's file tyre (the file tyre is not read), a key naming no wheel is rejected, a wheel with neither "tyre" nor an injected desc is a load error (checked before map generation); validate_tyre_desc is the one tyre validation (non-empty source, v_eps/low-speed damping ranges); tests tests/unit/test_tyre_injection.cpp TI1-TI8 [io][vehicle][tyre]; docs vehicle.md "Injected tyres"; decision vault RACE-018
    src/                              implementation
      version.cpp, jolt_proof.cpp
      math/math_check.cpp             compiles every math header once + static_asserts (math/ is otherwise header-only)
      jobs/
        job_system.cpp, task_graph.cpp, topology.cpp (physical_core_count()), debug_alloc.cpp (global operator new/delete override)
        jolt_job_system.h             INTERNAL: JoltJobSystemBridge (JPH::JobSystemWithBarrier over ps::jobs::JobSystem); only included from backend/jolt/
      world/world.cpp                 World's staged tick pipeline (built as a jobs::TaskGraph)
      backend/jolt/jolt_backend.cpp   the only P1 IRigidBackend implementation; every Jolt type is confined to this file + jolt_job_system.h
      articulation/                   INTERNAL (ps::articulation, not a public header); see "Articulation solver" below, incl. contact.h/.cpp (ContactCoupling) and contact_source.h (IContactSubstepSource)
      terrain/                        INTERNAL; see "Terrain tile system" below: procedural_terrain_source.h/.cpp (TerrainSpec, ProceduralTerrainSource), procedural_terrain.cpp (factory impl), tile_manager.h/.cpp (TileManager)
      io/telemetry.cpp, io/scenario.cpp, io/surface_table.cpp, io/test_ground_loader.cpp, io/vehicle_io.cpp
      vehicle/                        INTERNAL (ps::vehicle, not a public header); see "Vehicle system" below: tyre_desc.h+tyre_model.h's implementation is pacejka_tyre_model.cpp, tir_parser.h/.cpp (.tir file parser), suspension.h (header-only spring/damper/bump-stop/ARB force functions), wheel_instance.h/.cpp (WheelInstance: contact measurement (measure_contact) + a pre_solve/post_solve split around the drivetrain network's solve - PHYS-033 I2 commit 5b, wheel spin moved to the network's hub node), vehicle_instance.h/.cpp (VehicleInstance: chassis extrapolation, Ackermann steering, owns one drivetrain::Powertrain - commit 5b replaced the TEMPORARY P3Shim/build_p3_shim from commit 5a), system.h/.cpp (System, mirrors articulation::System)
      drivetrain/                     INTERNAL (ps::drivetrain, not a public header); PHYS-033 P4 I2 shaft-network powertrain, see "Vehicle system" below and the "Drivetrain" section: dense.h (SmallVec/SmallMat, Cholesky), box_lcp.h/.cpp (F1 box-LCP: block principal pivoting + Murty fallback), nullspace.h (build_nullspace), network.h/.cpp (NetworkBuilder, Assembly, ShaftNetwork), throttle_stage.h (ThrottleStage - PHYS-033 P4 I7a C3: the typed throttle-command stage, see "Drivetrain"), energy_store.h (IEnergyStore/FuelTank - PHYS-033 P4 I7a C6: header-only lumped store, content/capacity/withdraw/set_content/fold_state_hash; a future I9 battery implements the same interface), engine.h (IEngine - PHYS-033 P4 I7a C2: pre_solve/post_solve/idle_integrator/crank_angle/idle_rpm/fold_state_hash/reset - plus the engine's Sensors/TcuRequest/PreSolveResult (incl. theta_c, the post-limiter combustion fraction, published as EngineSoundState::load)/PostSolveResult at namespace level; Powertrain holds unique_ptr<IEngine>, EngineTorqueMap in engine_torque_map.h/.cpp and (I7b C11) SimulatedEngine in engine_simulated.h/.cpp - the mean-value engine model (throttle/manifold/VE/eta_i/spark/pumping/friction tables from a SimulatedEngineDesc; C12: built from a desc copy whose fuels = the burnt fuel, per-engine ambient-dependent SimulatedSteadyTables refilled in place (allocation-free, ~68 ms) when Sensors::ambient changes exactly - callers must quantise ambient - 5ec8b7d: a live ambient change no longer rebuilds the grid: set_live_ambient + live_steady_torque/live_inverse_plate with bisections bounded to 24 steps, steady_rebuild_count() stays 0 (test_live_engine_ambient.cpp [ambient_live]); set_live_ambient fills an IdleControlOut POD - never a SimulatedSteadyTables, whose ten vectors cost ten debug-STL _Container_proxy allocations per pre_solve (debug_alloc violations on the debug leg only, fixed 2026-10-06), set_ambient is the setup/offline path, LIVE_ATMOSPHERE_PERFORMANCE.md; turbo engines: SimulatedEngineDesc::turbo_pair, core/src/drivetrain/turbo_pair.h, ps/drivetrain/turbo_maps.h, EngineSoundState turbo_rpm/boost_bar/p_exhaust_pa/p_charge_pa/t_charge_k plus (I7d D1-D3, unhashed) wastegate_opening/compressor_flow_kg_s/compressor_pressure_ratio/bov_opening/bov_flow_kg_s/bov_vent/bov_events/surge_active/surge_events; I7d D2 blow-off valve (vent flow through the shared Psi table core/src/drivetrain/psi_table.h, added to the compressor flow, opening hashed only with a valve), D3 surge flag (surge_line, never hashed, read by nothing), D4 intercooler drop (charge_pa = post-cooler), D5 anti-lag (09a1ea6: TurboPairDesc::anti_lag from a turbo configuration's "anti_lag" block - channel, default_enabled, arm_below_command, rpm window, retard_deg, plate_floor, max_active_s; while active DFCO is inhibited, the plate floor applies, spark = scheduled - retard, the torque lost to retard is held (0.5 s decay) and added at the turbine inlet via TurboPair::step's extra_exhaust_w; sound fields anti_lag_active/anti_lag_activations/anti_lag_extra_exhaust_w; no shipped data until D6; 4deb7cc: exhaust port temperature from the engine's per-substep energy balance (released = min(1,lambda)*Q_fuel, sensible = released - indicated - wall, wall fraction 0.20 [AS] in the turbo configuration's "exhaust_thermal" block) through a lumped backward-Euler manifold/turbine wall, no temperature cap; anti-lag extension AntiLagDesc spark_advance_deg (absolute), cylinder_cut_fraction (cut charges burn in the manifold), manifold_burn_fraction, lambda, boost_target_fraction; the wastegate is a bypass orifice IN PARALLEL with the turbine (A = A_t + A_wg*opening, turbine share A_t/A, p_exh from the total flow; optional turbo hardware key wastegate_area_m2, default 2.4 x turbine_area_m2 [AS], hashed, not in the map key) and compressor power is bounded by the head of the current rotor speed; known limit: steady idle rotor ~10 rpm (mean-value turbine, no pulse energy); boost-aware idle feed-forward (9ea2cfb): with the pedal released and charge > ambient + 1 kPa the idle plate is solved against the charge pressure/temperature and turbine-inlet pressure (quantised 1 kPa / 1 K, cached, unhashed) - a spinning-down rotor no longer makes car_hyper hunt 1000-1300 rpm through DFCO after a lift (tests/unit/test_hyper_idle_recovery.cpp R1); stage seam S0 (merge e8c603b, vault PHYS-058): outside a stage map's covered cells TurboPair reads TurboStageMaps::compressor_extended (along a speed line past its choke edge PR falls linearly to 1 at the first generator-invalid point with work/kg held at the edge value; between lines a same-flow blend up to the smaller edge flow, then following the moving choke edge; below/above the line set fan-law scaling) instead of blending into the authored head/work law (2.5-4x low near choke - the charge jump at ~100-112k); covered cells unchanged; unhashed flag turbo_compressor_stage_extended (PreSolveResult, EngineSoundState); turbine side unchanged; check_turbo_stage_coverage / BuildRule::TurboStageMapCoverage 17 (Near: air demand at target boost needs uncertified cells; Limiting: top speed line cannot reach the PR) also as a load warning via EngineMapOptions::warnings_out; tests tests/unit/test_turbo_stage_seam.cpp [turbo_stage]; boost B0 (merge de43b93, vault PHYS-058): while the ManualTcu shift cut or a throttle ceiling < 1 is active (TcuRequest::hold_boost_control) the wastegate controller HOLDS (gate at its position, integral frozen) - the plate < 0.10 open-and-reset rule now applies only to a driver lift/dyno; the PID error is normalised by min(target, kBoostErrorScalePa 1.8 bar) [D] (bit-identical for every shipped config; a 15.48 bar target no longer weakens the loop 8.6x); stock WOT boost recovery after 1->2..6->7 0.56/0.50/0.43/0/0/0 s (was 0.78-2.25 s); closed-plate in-shift spike 5.6-6.1 bar without a BOV; known: stage-map choke-edge fallback seam (authored head/work law under-states Euler work ~4x below the map, charge jumps 0.64 -> 3.5 bar over 100-112k on the owner setup), part-throttle bistability on unreachable targets, 300 km/h 6<->7 hunting = plant limit (P1 shift policy); owner: no rotor-overspeed limiter (garage TurboRotorCap warns); tests tests/unit/test_boost_shift_hold.cpp B1-B3 V1-V3 [turbo] (V2 [!mayfail] hunting), hidden test_boost_b0_diag.cpp [.][diag][boost_b0]; tests test_turbo_exhaust_thermal/als_cut/wastegate_split.cpp ([turbo], WG1 is the flow-split guard); decisions vault PHYS-058); tests tests/unit/test_turbo_{pair,bov,surge,intercooler,io,vehicle,anti_lag}.cpp [turbo], fixtures tests/support/turbo_fixtures.h + fixtures/turbo_config_hyper_bov.json (test_turbo_io N7b: the vehicle forced_induction slot equals the same path in the engine file, no override bit-identical, 1 vs 10 workers, rejections); vault PHYS-058; a throttle_control kind differing from its throttle_input throws; rotary still refused until I7e C13)), nitrous_flow.h/.cpp (N2O plan N0, be04d60: OxidizerTables - NIST saturation table, HEM critical flux and Dyer flow tables), nitrous_system.h/.cpp (N1: NitrousSystem - a Powertrain-owned typed member like the fuel tank, not an IComponent; bottle fill + data-given pressure, kit jets, controller with declared channels - arm switch a plain 0/1 level named by the controller's arm_channel, shipped kits "nitrous_arm" - N2O in the fuel sub-ledger, telemetry vehicle<i>.n2o.* and hash fold only with a kit; engines receive Sensors::nitrous (NitrousInput, N2 adds the OxidizerTables pointer); the graph engine ignores it; N2 (525dc04, merge d0563e7): SimulatedEngine's wet charge model, active only while active && mdot_n > 0 && oxidizer != nullptr (else bit-identical) - N2O displaces and cools the charge (mole + energy balance, closed-form quadratic per oxidizer gas-table segment, solve_n2o_air in nitrous_flow.h), its O2 enriches (a_eq = a + 1.571 n), decomposition heat 1.8643 MJ/kg, kit fuel booked through cuts (unburnt), n2o_mass from the bottle's delivery, n2o_unburnt/n2o_energy_in/kit_fuel_mass in the fuel sub-ledger (engine_heat gains the N2O term only when nonzero), released heat counts burnt fuel only, torque_demand takes the turbo rule (max with the present uncut torque); N3 (merge e4756a8): with the wet charge model the turbo ExhaustSplit follows D-N2-9 - fuel_w + n2o_w = indicated + wall + sensible + chemical - clamp, n2o_w = burnt N2O x dH (== n2o_energy_in/h) released in full, only fired-charge fuel burns at min(1, lambda_combined), cut-cylinder kit fuel is chemical, ExhaustSplit::n2o_retard_w reports the kit retard's lost work (already inside sensible_w, D5 P2 rule against the knock-limited schedule); entered only under n2o_on_, so the D5 byte path is unchanged; PROVISIONAL: liquid-to-gas enthalpy deficit not subtracted, the 0.20 wall fraction applies to decomposition heat too; tests test_n2o_turbo.cpp T1-T6 [n2o][turbo][drivetrain] (T3/T6 [long], hyper maps 9 s cold; T3 3rd-gear WOT from 3000 rpm: 1.5 bar at 0.97 s vs 1.46 s), test fixture tests/support/fixtures/n2o/vehicle_hyper_n2o.json; N4 (merge a2205bd): neutral and reverse never spray (gear >= max(1, min_gear) in NitrousSystem::step, the controller loader rejects min_gear < 1), heater/arm/purge must be distinct channels (vehicle loader at the bottle->kit link, Powertrain backstop for code-built descs), EngineSoundState n2o_solenoid_open/n2o_flow_fraction/n2o_purge_active/n2o_injection (unhashed, 0 without a kit), tests test_n2o_controller.cpp C1-C8 [n2o][drivetrain]; PROVISIONAL (N2): fire*s_starve decomposition, Edelbrock/NACA numbers are the plan's transcription (no download); tests test_n2o_flow.cpp F1-F5, test_n2o_system.cpp S1-S9, test_n2o_engine.cpp E1-E8 (E7 NACA MR E5F26 within 2.4 pp, E8 hash 0xf38d2ff4c3d3d193 at 1/10 workers) [n2o]; docs docs/data_formats/oxidizer.md, nitrous.md; vault PHYS-062), components.h/.cpp (IComponent, TorqueMapEngine, GearPair, Differential (I4 C4, was OpenDifferential), TransferCase (I4 C9), ParkLock (I4 C10), Pto/LoadDyno (I4 C11), DirectTorqueBrakes - I2 commit 5b), powertrain.h/.cpp (Powertrain - builds the node/rigid-row/friction-row graph from a vehicle's PowertrainDesc/BrakeSystemDesc/WheelDesc list, drives the per-substep pre_solve/solve/post_solve sequence, owns the PowertrainEnergy ledger - I2 commit 5b)
      engine_cycle/                   ps::engine_cycle (PHYS-033 P4 I7c C15a; ps_audio A1: the live-model files geometry/cycle_stepper/cycle_point/firing_order .cpp build as target ps_engine_cycle, their headers (plus gas.h, heat_transfer.h, valve_flow.h, wiebe.h, firing_order.h) are public under core/include/ps/engine_cycle/; map_generator.cpp and map_cache.cpp (map_cache.h stays internal here) build into ps_core because they need JobSystem and file IO; firing_order.h: validate_firing_order/validate_firing_angles/resolve_firing_angles_deg - explicit firing_angles_deg is indexed by CYLINDER number, not firing position; ps_audio A2: pipe_network.h/.cpp - PipeNetwork, the quasi-1D finite-volume pipe core shared by maps (A5) and audio (A7), no consumer yet: MUSCL-Hancock + HLLC with a well-balanced area source and a passive burned fraction, Fanning friction + Reynolds-analogy heat, one characteristic pressure-reservoir face for Open/Volume/Port ends (Port = the cylinder hand-off: set_port_state before step, port_flux after), end correction as a pipe extension, explicit conservative 0D volumes, orifices only between volumes, build-time CFL checks throwing "PipeNetwork: ..."; A8b: a runtime CFL check per pipe per step (max |u|+a) sub-cycles that pipe n = ceil(h*s/(cfl*dx)) times at h/n, n clamped to [2, kMaxPipeSubcycles = 8] [A] (silently saturates above 8), volumes/ports/WG see start-of-step state and the end faces the time-averaged flux, subcycled_pipe_steps() is an unhashed diagnostic, a network that never exceeds it is bit-identical - a supersonic intake backflow jet after a WOT hold (|u|+a ~1260 m/s vs the assumed 500) had blown up the live model; the open-end reservoir face matches p and u on INFLOW (Newton, <= 8 iterations) - the old interior-Riemann-invariant closure at the reservoir sound speed made a velocity deadband at u = 0 whenever pipe and ambient temperatures differed; 74 ns per cell-step with wall terms (~0.57 core for an I4's audio-grade pipes, so A9 SIMD matters); pipe_network_types.h holds the enums/constants shared with the waveguide; ps_audio A3: waveguide_network.h/.cpp - WaveguideNetwork, the linear downstream pipe model: segments = delay ring + 6-shelf Kirchhoff wall-loss IIR + Thiran allpass, Norton junctions with WDF compliance and a flow source, Closed/Open (Levine-Schwinger radiation, one-pole)/Anechoic ends, a Resistance 2-port linearised with the incremental R (catalyst/silencer loss), LaminarFriction as a series R lumped at Junction/Closed ends (a distortionless exp(-alpha*L) gain halves a Helmholtz neck's decay); 20.9 ns per sample per segment (9.6 lossless). pipe_network's PipeEndDesc::Kind::Waveguide couples an FV pipe end to a WG Junction: the WHOLE flow crosses as one more junction port of admittance A/(rho*a) on the MUSCL-Hancock half-step state (a running-mean split bypassed the WG impedance below 1/tau - rejected), p_ref = ambient + mean_pressure_drop - linearised_mean_drop restores the quasi-steady resistance drop; the WG is linear about a fixed mean, so the split sits at the exhaust flange and upstream of the throttle; WG state hashed only when a WG exists; ps_audio A5: cycle_engine.h/.cpp - CycleEngine, n CycleSteppers (pipe mode set_port_sources/step_phased; reservoir reference mode set_port_reservoirs) in A1 firing phases coupled to one PipeNetwork from make_engine_pipe_network_desc (intake: snorkel -> airbox -> throttle orifice -> plenum -> runners -> ports, no snorkel = throttle from ambient; exhaust: ports -> primaries -> merge junction volumes (sum of pipe areas x 0.1 m [AS]) -> flange, then a WG chain from the installation or an Open end); per step set_port_state -> network step -> constant port sources -flux/h -> RK4 per cylinder (mass closure <= 1e-13); flow-balance throttle trim on the cycle-mean plenum pressure once per cycle; 8 Full / 6 Preview settling cycles; one CycleEngine per grid point built inside the job, run() single-use and allocation-free; ~12 us per step at dx 50 mm, 0.22 s per Full point (12x20 Full 61.9 s on 1 worker / 7.2 s on 10); an unreachable p_im at a grid node throws; the loader resolves the installation (vehicle override or engine default) BEFORE generation; ps_audio A7 split cycle_engine into EngineCore (phased steppers, pipe network + layout, throttle Cd*A, port<->pipe exchange, radiating_ends()/radiating_outflow() built in the ctor, never read by the generator) shared by CycleEngine (generator policy) and ps_audio's LiveEngine, plus combustion_variation.h (header-only counter-hash cycle-to-cycle variation, audio only) and the CycleStepper live API set_speed/step_live/set_combustion_event/set_charge (audio only)): 0D single-cylinder cycle model between fixed intake/exhaust reservoirs - geometry.h/.cpp (slider-crank with pin offset), gas.h ((m, U, m_b) state, linear cv from NIST-JANAF fits, U referenced to 298.15 K, closed-form T), valve_flow.h (choked/unchoked/reverse, Psi linear above pressure ratio 0.99, gamma blended by burned fraction), heat_transfer.h (Woschni), wiebe.h (a 5, m 2), cycle_stepper.h/.cpp (fixed-step RK4 on ps::math, integer steps per cycle), cycle_point.h/.cpp (one operating point: h_full 1/(960*48) s, h_preview 1/(960*24) s, N 5/4 cycles - PROVISIONAL, RK4 stability edge 30-36 us at 1000 rpm / 30 kPa, larger valve CdA moves it down); descriptor ps/drivetrain/engine_cycle_desc.h (A1 adds firing_order/firing_angles_deg/banks/bank_angle_deg - loaded and validated, firing_order default 1-3-4-2 only for 4 cylinders, else required - firing_order/firing_angles_deg are read by the A5 CycleEngine and folded into the C16b cache key for pipe engines only (lumped keys unchanged); banks/bank_angle_deg are not generator inputs and not in the key; key and the H3 table must be extended together); ISA constants in core/include/ps/drivetrain/isa_air.h. No friction model in the cycle itself. map_generator.cpp (I7c C16a): EngineMapGenTask (run_point(i) writes slot i only, finish() assembles in index order) extracts ve/eta_i/pmep_excess (signed)/friction etc. from cycle points over the cycle block's grid_rpm x grid_p_im_kpa, friction = D13 FMEP polynomial over mean piston speed; boosted nodes (2026-10-02, vault PHYS-059): IntakeCycleDesc::compressor_supply (set by the loader iff the engine has a turbo, folded into the C16b key only when true) makes every node feed the throttle from OperatingPoint::p_supply_pa = max(ambient, 1.05 p_node) at ISA temperature via PipeNetwork::set_orifice_supply, so grid_p_im may exceed ambient (G5); without it a node above ambient still throws (G3); a node below the closed-throttle manifold floor (valve-overlap backflow keeps the plenum above it) is regenerated with the plenum held, p_supply_pa = p_node, counted in the unhashed EngineMaps::held_plenum_points, and a cycle missing the trim tolerance first gets up to CycleEngineSettings::max_extra_trim_cycles 12 extra trim cycles (CycleEngineResult::extra_trim_cycles) - only previously-failing points take either path, so existing maps and keys are unchanged (G6, a 32 deg-overlap twin-turbo V8); held rpm rows (rev limits raised past the breathing limit, vault PHYS-082): when the maps cannot be assembled, finish() holds rows r0..top (r0 = lowest failing rpm row) as copies of row r0 - 1 if every row from r0 up fails or the top node's air flow (trapped air x rpm) at r0 - 1, or on every valid row above, is below its peak over the rows below r0, else rethrows unchanged; unhashed EngineMaps::held_rpm_rows; non-WOT pipe trim failures are judged in finish() (node_status) instead of thrown in run_point; grids that generated before are bit-identical; tests tests/unit/test_held_rpm_rows.cpp [cycle][drivetrain]; turbo engines (core/src/drivetrain/turbo_pair.h): the wastegate is a PID on charge pressure (turbo configuration "boost_control" kp/ki_per_s/kd_s/actuator_tau_s, conditional anti-windup, integral hashed), turbo files are in bar (target_boost_bar gauge, reference_head_bar; the *_pa keys are rejected); called by the loader when an engine file has a "cycle" block and its maps lack any of the 7 keys (a complete override never runs it); the maps' content hash is folded into SimulatedEngine::fold_state_hash. Full 12x20 grid 2.2 s single-thread / 0.23 s on 10 workers, Preview 0.33 / 0.034 s. map_cache.h/.cpp (I7c C16b): the .psmaps disk cache - key FNV-1a 64 over the RESOLVED CycleModelDesc (incl. source) + FuelDesc + kGeneratorVersion + quality (not JSON text; every new generator input must join the key, guarded by a table-driven H3 test), file <cache_dir>/<key:016x>.psmaps = 40-byte little-endian header + raw doubles, content hash re-checked on load, any mismatch/corruption a silent miss (regenerate + rewrite), temp file per process+thread + rename, catch(...) only (TOOL-019); cache hit 0.2-0.4 ms vs 0.17-2.2 s generation
      aero/aerodynamics.cpp           ps::aero (8b04f0e), public header core/include/ps/aero/aerodynamics.h: SurfaceDesc/BodyDesc/GroundEffectDesc/WakeDesc/FanDesc/Desc (max_surfaces 32, max_fans 8), Environment, WakeSource, State, Telemetry; validate/sample_air/step/fold_state_hash. Stepped per substep in VehicleInstance with ground probes via IRigidBackend::ray_cast_excluding (new virtual, Jolt override); wake sources = the tick-start vehicle snapshots + World::set_aero_wakes (via VehicleTickEnv). Loader core/src/io/aero_io.cpp (read_aero: the vehicle "aero" block - drag_area_m2, body, surfaces, fans, wake; unknown keys throw); VehicleDesc's AeroDesc = aero::Desc. Actuator and fan-energy state hashed, environment and commands not. World: set_aero_environment/aero_environment, set_aero_wakes, set_aero_surface_command (-1..1), set_aero_fan_command (0..1), credit_aero_fan_energy, vehicle_aero_telemetry; WorldConfig::wind_world; VehicleSnapshot::aero. No telemetry channels. Description: AERODYNAMICS.md (docs/data_formats/vehicle.md still documents only drag_area_m2). Tests test_aerodynamics.cpp [aero]; bench_aero_ground (not CTest-registered)
      drivetrain/turbo_meanline.cpp     (2026-10-14) C++ port of tools/turbo_sim/meanline.py, public ps/drivetrain/turbo_meanline.h (ps::drivetrain::meanline): compressor_point/turbine_point/generate/adaptive_generate (JobSystem optional, bit-identical for any worker count, ps::math only), cache_key (FNV-1a 64: solver, canonical profile JSON, initial axes, adaptive options), to_stage_maps, and the runtime extension demand_points + extend_sampling_for_demand (ExtensionPolicy = caller config: margins, 500 rpm lines, 0.01 kg/s knots); invalid geometry/axes throw std::invalid_argument. JSON side io/turbo_stage_io.cpp (public ps/io/turbo_stage_io.h: parse_turbo_geometry_json (profile or a map's embedded geometry), parse_turbo_stage_sampling_json, turbo_stage_maps_to_json/write_turbo_stage_maps_json, turbo_stage_maps_from_json = the loader's own parse, moved out of vehicle_io.cpp into internal io/turbo_stage_json.h detail::stage_maps_from_json). engine_rules.h turbo_stage_demand(engine) = the numbers check_turbo_stage_coverage reads (refactor, same arithmetic). Tests test_turbo_meanline.cpp [turbo_meanline] against Python fixtures tests/fixtures/turbo_meanline/ (make_reference.py): statuses identical, values 1e-9 relative (measured 1.9e-15), adaptive axes bitwise; standard recipe 137 ms / 1 thread vs 6.4 s Python; docs/data_formats/turbo_stage_maps.md "C++ generator"
      drivetrain/ brake blending      regen-first brake blend (branch claude/regen-blend): brake_system.regen_blend (RegenBlendDesc in ps/drivetrain/brake_system_desc.h: enabled, taper_start_soc 0.90, full_soc 0.98, fade_start/end_speed_mps 2.5/0.8, abs_hold_s 0.3), electric_motor regen_power_w; Powertrain derives per motor the rim torque per shaft torque from the shaft graph (gear pairs, open diffs; RegenMap), prepare_regen/reflect_regen in pre_solve_components, DirectTorqueBrakes::set_regen_reflected lowers each corner's pad capacity by the reflected regeneration, ComponentSubstepCtx::wheel_abs_scale (ABS applied scale) withdraws regeneration; ElectricMotor regeneration tau0 fixed to -d*omega_old (it braked ~nothing before); PowertrainSnapshot regen_power_w/regen_brake_torque_nm/pad_brake_torque_nm/regen_brake_share/pad_brake_share (unhashed); ICE hashes unchanged; tests tests/unit/test_brake_blend.cpp B1-B9 L1 [electric][brake]; docs docs/electric_drive.md 'Brake blending', data_formats/vehicle.md; motors behind a gearbox / clutch / transfer case (parallel hybrids): RegenMap is a path tree (RegenNode, RegenEdge Fixed|Gearbox|Clutch|TransferFront) evaluated per substep over the CLOSED edges (gear engaged and no shift pending, clutch engagement >= path_min_engagement 0.95, transfer front engaged; open branches split equally), a motor upstream of the clutch capped at clutch_capacity_margin 0.9 x clutch kinetic capacity, regeneration ramps in over path_ramp_s 0.05 after a path closes (ramp hashed only with a dynamic path); an automatic gearbox stays outside; tests tests/unit/test_brake_blend_gearbox.cpp G1-G7 [electric][brake][gearbox]; docs electric_drive.md 'Motors behind a gearbox'
      drivetrain/ hybrid shift        hybrid supervisor (Powertrain::pre_solve_components hybrid block, branch claude/hybrid-shift-pedal): the driver command (throttle_stage_ evaluated on the incoming ctx = pedal curve + drive-mode bend, BEFORE TC/LC ceilings) = hybrid_driver_command_ feeds the power request (x requested_power_w, no ceiling folded in), the ICE share (request - available)/(requested - available), and the schedule: ManualTcu Sensors::shift_throttle_request and AutoTransmission Sensors::throttle read it (the DCT used to read requested / (motors + ICE envelope power), ~0.3 at full pedal, so Racing upshifts came at ~4100-4700 rpm; now 8000-8260 = up_rpm_at_full_throttle); ComponentSubstepCtx::throttle_is_command makes ThrottleStage skip curve and bend for the ICE share (no double bend); a TC ceiling is applied once downstream (engine stage min with the ICE command, motors min with their pedal); non-hybrid bit-identical; tests tests/unit/test_hybrid_shift.cpp [hybrid][tcu][drivetrain][vehicle] (pre-fix: 4931 TC off, 4081-4342 TC on); docs electric_drive.md 'Hydrogen and hybrid development packages'
      drivetrain/vehicle additions 2026-10-07..09 (another assistant; repaired in merge bd12d96): ps/drivetrain/adaptive_pressure.h (cycle block "pressure_sampling": adaptive p_im node placement for generated maps, reusable operating points; docs/data_formats/adaptive_pressure.md; tests test_adaptive_pressure.cpp [adaptive_pressure][drivetrain][cycle]); ps/drivetrain/turbo_stage_maps.h (experimental offline turbo stage maps physics_sim.turbo_stage_maps/1 from tools/turbo_sim, read by TurboPair; TurboPairDesc::compressible_exhaust option; docs/data_formats/turbo_stage_maps.md; tests test_turbo_stage_maps.cpp [turbo_stage][drivetrain] - each test process uses its own temp dir); ps/vehicle/traction_control.h (sensor-only TC: wheel peripheral speeds + IMU, no ground truth; ECU torque intervention through the throttle stage; optional VehicleDesc::traction_control, channel assist.traction_control registered for EVERY vehicle like the other assist.* channels, default = the block's enabled (0 for every shipped car); assist bit 3, traction_* snapshot fields; state hashed only while enabled/initialised; speed estimate = IMU prediction corrected every call from the wheels (ESC reference speed): free wheels (not braked - TractionWheelSensor::braked, per-corner brake signal from the brake system - and not driven, or all while coasting) as a measurement at 20/s capped 20 m/s^2, braked wheels as a lower bound, driven AWD wheels as an upper bound (inside it only while the torque is cut to <= 0.3 and the slowest wheel's rate matches the IMU for 0.1 s, 5/s capped 3 m/s^2), a standstill update; constants kTraction* [AS], no data keys; estimator state (wheel_reference_m_s, wheel_accel_m_s2, consistent_s, reference) hashed with the TC state; the virtual IMU is sampled every tick (a stale sample read as a jump when TC came back on); constant AWD common-mode spin with torque on is unobservable; tests test_traction_control.cpp [traction_control][vehicle], the hypercar case [long], test_traction_control_low_speed.cpp [traction_control][low_speed] (stale estimate RWD/AWD, locked-wheel braking, real wheelspin, hypercar launch through 30 km/h in all TC modes), test_traction_control_drift.cpp [traction_control][drift] (full hypercar: locked-wheel braking, handbrake slide, power slide, slopes, kerbs, TC off/on - estimate vs truth at contact height <= 0.10 m/s crawling, <= 0.5 m/s overall)); tyre rolling_resistance_speed_law fourth_power (default) | quadratic via ps::vehicle::select_rolling_resistance_speed_law (docs/data_formats/tyre.md; hashed only when not the default)
      drivetrain/engine_dyno.cpp       engine garage P3 (merge e47f506), public header core/include/ps/drivetrain/engine_dyno.h: run_dyno_sweep(ComponentParams, EngineMapsPtr (null = the desc's own; explicit maps complete a maps_pending desc), DynoSweepSpec, pool, cancel) -> DynoResult{points} (a struct so P3b appends limits); torque map = the WOT curve (instant), NA simulated = SimulatedEngine::steady_point (exact steady solve, bit-identical to the steady tables at their nodes; build_model_maps skips the ~70 ms tables), turbo = the real runtime engine held at omega, 960 Hz, net torque (tau0 - c_k) averaged over settle_window_s 0.5 s; settled after two consecutive window pairs with |dboost/dt| < 0.005 bar/s AND relative torque rate < 0.2 %/s [AS] (the design's boost-only rule exited at the wastegate overshoot peak, +1.3 % torque at 7500 rpm), else converged = false at max_settle_s 15 s (hyper 1950/2200 rpm); limiter ignored; one job per rpm point, bit-identical for any worker count; cancel throws GenerationCancelled like P1. Hyper idle..limiter (31 points, maps loaded) 0.10 s on 10 workers / 0.2 s serial. EngineRig gains desc-taking constructors (path ones delegate, bit-identical). P3b hook: settled_window_point/WindowStats in engine_dyno.cpp. Docs docs/data_formats/engine.md "Dyno sweep"; tests tests/unit/test_engine_dyno.cpp D1-D8 [drivetrain][dyno] (D3 Full maps [long])
      drivetrain/turbo_envelope (Q5)    public header core/include/ps/drivetrain/turbo_envelope.h: the turbo full-load envelope built off the swapping thread - World::prepare_turbo_envelope(id, component, desc) (caller thread between steps, copies the desc, holds the engine's maps) -> build_turbo_envelope(PreparedTurboEnvelope&) (any thread, the same memoised full_load_envelope) -> World::replace_turbo_configuration(id, component, prepared) (installs prepared.turbo and the prebuilt envelope when its maps are still the engine's, else builds synchronously); hypercar memo miss on the caller 442-483 ms -> < 0.3 ms; clear_full_load_envelope_memo() test hook (engine_simulated.h). Tests tests/unit/test_turbo_envelope_prebuilt.cpp E1/E2/W1 [q5][long] (bit-identical curves, stale/unbuilt never installed, World tick hashes sync == prebuilt at 1 and 10 workers, zero job allocation after the swap)
      drivetrain/powertrain_dyno.cpp  powertrain dyno (branch claude/powertrain-dyno, 2026-10-19): public headers ps/drivetrain/electric_envelope.h (motor_full_load_point -> {torque, power, electrical, MotorLimit PeakTorque|RatedPower|BatteryDischarge|MaxRpm|Empty}, motor_drive_request/motor_drive_torque/motor_drives, battery_discharge_budget_w/_j, battery_charge_budget_j, fuel_cell_request_j/fuel_cell_supply_w; implemented in electric_envelope.cpp, CALLED by Battery/ElectricMotor/FuelCell in electric_drive.h) and ps/drivetrain/hybrid_split.h (hybrid_power_split/hybrid_full_pedal_split/hybrid_update_charge_latch, called by the Powertrain's hybrid supervisor) - extraction bit-identical (pins tests/unit/test_electric_pins.cpp [electric][pin]); Battery::reserve_discharge allocates a short budget in proportion to the driving motors' claims (ElectricMotor::drive_claim_j registered before the component loop; was component order - the last motor of a pack-limited dual-motor car got 0; pins car_electric 100 kW and car_fuel_cell 10 kW + 30 kW re-recorded); ps/drivetrain/powertrain_dyno.h run_powertrain_dyno(VehicleDesc, maps, PowertrainDynoSpec{axis EngineRpm|MotorRpm|GearboxOutputRpm|RoadSpeed, points, motor_index, gear, pedal, battery_soc, hydrogen_fraction, drive_mode, ambient, channels, settle_*}, pool, cancel): a chassis dyno - the car's Powertrain without chassis, every hub held at v/r by an implicit speed hold (D 1e6), SOC/H2 held (Powertrain::dyno_hold_energy_state), gear held (Powertrain::set_dyno_gear_hold; 2026-10-21 also automatic / DCT / CVT through AutoTransmission::set_dyno_hold - TCU frozen on the gear, starting clutch and converter locked, CVT variator held at virtual step `gear` or PowertrainDynoSpec::cvt_ratio; tests A1-A6 [auto_gearbox], fixtures tests/support/fixtures/vehicles/car_hybrid_{dct,at,cvt}.json), settles like run_dyno_sweep; per point engine/motor/pack/stack/system numbers and the supervisor state; one job per point, bit-identical for any worker count; car_electric 29 points 22 ms serial / 4.3 ms on 10 workers. Docs docs/electric_drive.md 'Envelope and dyno', 'Discharge allocation'; tests tests/unit/test_powertrain_dyno.cpp E1 S1 A1 P1-P6 [drivetrain][electric][dyno] (P4: an ICE car's crank torque = run_dyno_sweep to 1e-14 for car_sedan (torque map) and car_sedan_sim (NA simulated); P4b [long] car_hyper twin-turbo Sketch maps in 4th within 4e-7)
      drivetrain/b_mode.h             EV one-pedal B and hybrid Electric mode (branch claude/ev-b-mode, 2026-10-21, docs/electric_drive.md "B mode", "Hybrid power split", "Hybrid Electric mode"): public header ps/drivetrain/b_mode.h (BModeDesc: slip_target/slip_floor_mps/kp/ki/attack/recovery of the sensor-only wheel-slip guard, auto_hold/hold_release_s/hold_rise_per_s/hold_blend_level/hold_pedal_margin; pure b_system_power = -R + pedal (R + P_max), b_zero_power_pedal = R/(R+P_max), b_slip_fraction_step), loaded from the vehicle's `regen_blend.b_mode` (unknown keys rejected); Powertrain::b_update_selector (D <-> B on the shift counters or gear_request kGearRequestB 98, edge-triggered, B refused in N/R, only cars with traction motors on the wheel path and no gearbox, or a hybrid in Eco or Electric mode), prepare_b (per-substep law in the torque domain through ElectricMotor::set_b_feed/inputs(); B regeneration adds to the brake pedal's regeneration and is NOT scaled by the TC ceiling, B drive is (PR #37 ceiling_scale); speed-floor 1 rad/s; auto hold through DirectTorqueBrakes::set_hold_level with the fade-matched hold_blend_level), snapshot b_mode_available/active, b_regen_capacity_w, b_drive_capacity_w, b_power_command_w, b_zero_power_pedal, b_slip_fraction, b_hold_level, hybrid_electric_mode/leave_reason (never hashed); state hashed under tag 0x42 only while non-default. Hybrid: drive mode 4 Electric (DriveMode::Electric, kDriveModeLegacyCount 4, offered only by a drive_modes part that lists `electric`, which needs a hybrid_control component; ICE off and disconnected, linear pedal, pedal_bend rejected, shift counters swallowed into the EV selector, a pack at or below low_soc latches hybrid_electric_lost_ and queues the part's default into assist.drive_mode); HybridControlDesc::requested_power_derived (key requested_power_w absent): P_full = available + Powertrain::hybrid_ice_full_power_w (max over rpm of the engine's request_envelope f_nm*omega, rebuilt on turbo/maps swap, never per substep), hybrid_power_split gets ice_full_power_w: Normal electric first then ICE tops up by (request - available)/P_ice_full, Sport/Racing both sources = command, Eco unchanged; authored requested_power_w is bit-identical; the powertrain dyno (drive_mode 0..4) reads the same function. Tests tests/unit/test_b_mode.cpp BM1-BM9 BL1 BP1 [electric][b_mode] (low_mu guard with a recorded sabotage), tests/unit/test_hybrid_power_split.cpp HS1-HS5 HE1-HE2 [hybrid][electric]
      drivetrain/limit_findings.cpp    engine garage P3b-1 (merge 3bdd9dc), public header core/include/ps/drivetrain/limit_findings.h: LimitKind (explicit append-only uint16 values 0..21, 20 TowedMass and 21 TongueLoad from ps/machine/hitch_limits.h in the design order, never renumber), LimitSeverity Near/Limiting/Exceeded, LimitUnit, PartRef{slot, file, JSON-pointer field}, Remedy{part, current, required_min, required_margin, unit, cost Closed|Simulated, solved}, LimitFinding, to_string. DynoResult::limits (sorted rpm, kind) + point_limits (one range per point); DynoSweepSpec::part_files fills PartRef::file. Produced from each turbo point's settled window: WastegateSaturated, BoostBelow/AboveTarget, TurboRotorCap (vs TurboPair::speed_edge_rpm, the performance tables' speed edge - there is no rotor-speed ceiling since 16b12a7), CompressorSurge (only with a surge_line - none shipped), CompressorChoke (flow over max_mass_flow_kg_s); for every engine kind RevLimiter, FuelCut, MeanPistonSpeed, MapGridEdge (ve table axes; Near only on p_im); check_driveline_limits(dyno, PowertrainDesc, clutch_file) -> ClutchCapacity with a Closed remedy (required x1.3 [AS]). Deferred: ChargeTemperature (no limit in data), GearboxInputTorque (GearboxDesc has no rating), ValveFlowChoke (no valve Mach diagnostic), Knock (C17b), KitFuelLean/InjectorDuty/N2O kinds and Simulated remedies (P3b-2). Every threshold in core/src/drivetrain/limit_thresholds.h, each [C]/[D]/[AS]/[M]. Stock pins: hyper BoostBelowTarget 2000 rpm, MeanPistonSpeed 8000 (25.4 m/s), FuelCut+MeanPistonSpeed at 8250; toyota_a25a_fks_sim MapGridEdge Exceeded at every point (its ve p_im axis ends at 97.5 kPa < ambient - a data gap, fixing it moves the sim/gen engine scenario hashes); i4_2l_na FuelCut at 6800. Docs engine.md "Limit findings"/"Driveline limits"; tests tests/unit/test_limit_findings.cpp L1-L21 [drivetrain][dyno] (L6 = the owner's wastegate example: stock gate saturates at air demand x2, a x1.26 gate clears it)
      drivetrain/limiter_range.h      G1 limiter-range robustness (public header core/include/ps/drivetrain/limiter_range.h, pure functions): a garage limiter anywhere in 2000..20000 rpm loads, drives and sweeps. rpm_rolloff: past the engine's own data top n_e (torque map: WOT/motoring/fuel axes; simulated: every map's rpm axis, the cycle grid while pending; ECU tables excluded) the FIRED torque falls linearly to 0 at n_z = n_e (1 + rpm_rolloff_band_fraction, [AS] 0.2) - torque map scales the span (w - m) and fuel, simulated scales VE; armed only when the limiter (or a governor's high idle) is above n_e, branch-gated so every rpm inside the data is bit-identical. effective_limiter_rpm = min(FuelCut, SoftCut taper start, governor high idle, armed n_z). fit_launch_rpm: P8 two-step / assist-unit launch rpm at or above max_limiter_fraction ([AS] 0.8) x limiter is held there with a load warning (was a load error). shift_point_scale: auto_shift points (car's and drive-mode overrides) above AutoShiftDesc::max_limiter_fraction ([AS] 0.985) x effective limiter are scaled by one factor in ManualTcu. LimitKind::BeyondEngineData = 19 (Limiting in the band, Exceeded from n_z). SimulatedEngineDesc::plate_monotone_tolerance ([AS] 0.04, measured toyota_a25a_fks_gen Sketch dip 3.0 %): steady-torque plate check vs the row's running max. Only shipped engine armed: demo_i4_pipes (no limiter key -> 6800 above its 6500 grid; 6500..6800 now rolls off). Docs engine.md "Beyond the authored rpm range", electronics.md, vehicle.md auto_shift; tests tests/unit/test_limiter_range.cpp R1/R2/S1-S3/L1/L2/A1 + [long] A2/P1/E1 (every shipped car at 2000 and 20000 rpm) [limiter_range], recorded sabotages in its header
      drivetrain/limit_remedies.cpp    engine garage P3b-2 (merge faf4a01), public header core/include/ps/drivetrain/limit_remedies.h: solve_remedy(RemedyRequest{engine, maps, spec}, finding, RemedyOptions{pool, cancel, on_progress, tolerance_rel 0.005, evaluations_out}) - the wastegate area for WastegateSaturated/BoostAboveTarget/TurboRotorCap at the finding's ONE rpm (factor in [1, 8] over the current area until no WastegateSaturated (mean gate < 0.92 [AS]) and no BoostAboveTarget, rotor cap also cleared; required_margin x1.10 [AS]; not clear at 8x -> solved false, rotor cap names 'the turbo hardware' without a number); solve_power_target(request, NitrousJet, rpm, target_gain_kw) - N2O jet over [0, 4x], fuel jet scaled with it. Fixed 4-section search (3 interior points per round, one job each), bit-identical for any pool; ~16 settled points, 0.5-0.6 s on 10 workers with Sketch maps. run_dyno_sweep never solves; sweeps attach unsolved Simulated placeholders. Nitrous in the sweep: DynoSweepSpec::nitrous (DynoNitrous, nitrous_from_powertrain) hosts a NitrousSystem per point (no ramp/duty cap, no gear/speed window, bottle held, never committed), DynoPoint n2o_g_s/kit_fuel_g_s/lambda_combined, DynoSweepSpec::turbo_count (per-turbo file value); no kit = bit-identical. New kinds produced: KitFuelLean (Closed remedy: fuel jet mm), N2OBottleDuration (Closed: bottle kg, near factor 1.1), N2OBottlePressureDrop (vs_fill bottles without heater only, no remedy); InjectorDuty unproduced (no injector model). Owner example: stock gate + car_hyper_n2o kit needs x1.32 at 7000 rpm (margin x1.45) vs N7b's shipped x1.26 - the gap is the 0.92 criterion vs N7b's 0.965 (kGateNear, an owner decision). Tests tests/unit/test_limit_remedies.cpp R1-R13 [dyno][n2o][turbo] (R1 vs the dyno_shot harness 2-3e-4 torque, R12 [long]); docs engine.md 'Simulated remedies', nitrous.md 'Dyno use'
      drivetrain/ gearbox rating     transmission T0 (merge 8116d4b): GearboxDesc gains rated_input_torque_nm, gear_type (helical|straight_cut), teeth{forward[], reverse[], input_pair} ({drive, driven}) and mesh_order; gearbox part files support "extends" (parts::load_part); BuildRule::GearboxRating 16 (check_gearbox_rating, peak_net_torque_nm - the caller passes the peak, check_buildable stays static) graded OVER the rating >0/5/15 % Near/Limiting/Exceeded (kGearboxOverLimiting/kGearboxOverExceeded [AS] in engine_rules_table.h, helper core/src/drivetrain/gearbox_rating.h); LimitKind::GearboxInputTorque (8) produced by check_driveline_limits(dyno, powertrain, clutch_file, gearbox_file) with a Closed remedy x1.10 (kGearboxRemedyMargin); classify_gearbox_change -> GearboxChange {None, Runtime, Respawn}; DrivelineSoundState gear_type / mesh_frequency_hz[2] (input pair + active gear from tooth counts, else mesh_order) / mesh_load_nm (reconstructed also for lossless gears, unhashed - no hash moved). No final-drive teeth, tooth counts not cross-checked against ratios. Docs gearbox.md, engine.md; tests tests/unit/test_gearbox_rating.cpp G1-G10 G7b [drivetrain][gearbox], fixtures tests/support/fixtures/trans_t0/
      drivetrain/ automatic gearbox  automatic gearbox plan A1: GearboxType Automatic (torque converter) and DualClutch (two wet clutches, odd/even shafts) in GearboxDesc (`shift_clutch` ShiftClutchDesc, `converter` TorqueConverterDesc; docs gearbox.md "Automatic and dual-clutch gearboxes"); core/src/drivetrain/torque_converter.h (K-factor/torque-ratio law, stall/coupling/overrun, implicit pump-turbine slip coupling; lock-up = a friction row; loss -> PowertrainEnergy::converter_loss) and auto_transmission.h/.cpp (AutoTransmission: clutch-to-clutch shift rows A/B set per substep, phases Idle/Engage/Preselect/Torque/Inertia/Closing, hold capacity min(C, 1.3 max(|T_in|, 0.15C)), power-on upshift inertia boost x1.5 + throttle cap upshift_torque_fraction, DCT predictive preselection, schedule/sport schedule, kickdown, lock-up policy, DCT launch on clutch slip + creep); controller `automatic_tcu` (AutomaticTcuDesc in controller_desc.h, docs vehicle.md "automatic_tcu"): selector via gear_request (0 N, < 0 R, k > 0 D up to gear k), P = N + park_lock, shift counters = manual mode, channel transmission.mode (>= 0.5 sport); no clutch component, no drive_modes. Snapshot transmission_kind/selector/manual/sport/converter_speed_ratio/lockup_engagement (unhashed); TCU state hashed only with an automatic_tcu - manual scenario hashes unchanged. Data data/gearboxes/sedan_8at.json, sedan_7dct.json [AS]; fixtures tests/support/fixtures/vehicles/car_sedan_at.json, car_sedan_dct.json; tests tests/validation/test_automatic_gearbox.cpp (stall 2399 rpm, upshifts, lock-up, kickdown, creep, PRND, DCT launch, 1->2 handover 0.133 s with Fx >= 84 %, 1 vs 10 workers) and tests/unit/test_automatic_gearbox_io.cpp [drivetrain][automatic]. DCT kickdown one gear per shift
      drivetrain/ cvt                CVT plan C1: GearboxType Cvt (counts as clutch-to-clutch) with GearboxDesc::cvt (CvtDesc: ratio_low/high, rate limits continuous/stepped, virtual steps = forward_ratios, efficiency_vs_ratio, pump drag, CvtBeltDesc clamping force lag + belt capacity 2 mu F R_p / cos(beta)); optional converter (lock-up launch) or the forward clutch's slip launch (ratio held at ratio_low until it locks, clamp raised to the clutch's static capacity under throttle); AutomaticTcuDesc::cvt (CvtTcuDesc ratio law: target_rpm_vs_throttle / sport curve / min_rpm_vs_output_rpm, stepped_sport = virtual steps on the sport schedule; paddles = steps); D = gear 1, AutoTransmission CVT mode (ratio_, step_, t_step_, clamp_ hashed only on a cvt); belt slip -> clutch_loss; snapshot transmission_kind 3, cvt_ratio/step/belt_capacity/clamp_force/belt_slip (unhashed). Data data/gearboxes/sedan_cvt.json, sedan_cvt_clutch.json [AS]; fixtures car_sedan_cvt.json, car_sedan_cvt_clutch.json; tests tests/validation/test_cvt_gearbox.cpp, tests/unit/test_cvt_gearbox_io.cpp [cvt] (docs gearbox.md "CVT gearboxes", vehicle.md automatic_tcu "cvt"). DCT launch control: the assist unit's launch_control arms on a dual_clutch box (VehicleInstance launch_on_dct_), the TCU holds the launch clutch open when Armed and caps it at fraction x max(0, T_demand + T_sync) in Launch/Handover; [launch] test in test_cvt_gearbox.cpp, docs electronics.md "DCT launch control"; no shipped car hash moved
      drivetrain/limit_policy.cpp      limit policy L0 (merge 3485601, owner: refusal/advice thresholds are config, vault PHYS-076), public header core/include/ps/drivetrain/limit_policy.h: LimitPolicy (value struct, one field per POLICY threshold named like its `name` string, refuse_at exceeded|limiting|never default exceeded, warn_at near|limiting|exceeded|never default near; warns()/refuses() are predicates - findings are never dropped), default_limit_policy(), validate_limit_policy(), has_refusing_finding(); one row table core/src/drivetrain/limit_policy_table.h drives loader, defaults, validation and tests; loader ps/io/limit_policy_io.h load_limit_policy_json(path, user_path = {}) (physics_sim.limit_policy/1, extends + user layer last, unknown keys/wrong types rejected); data/limit_policies/default.json = today's literals (a test pins file == compiled defaults). Every consumer takes a trailing optional const LimitPolicy* (null = defaults): DynoSweepSpec::policy (also solve_remedy), check_driveline_limits, check_buildable, check_gearbox_rating, has_blocking_issue. Classification (comments in limit_thresholds.h, engine_rules_table.h): 58 POLICY (all 31 limit thresholds + 27 rule bands) / 13 NOT POLICY constexpr (valve layouts, row chord fraction, pack densities, boost/NA grid constants, relative_tolerance - generator/key inputs); no key or hash moved. Not classified: dyno-internal constants (boost sampling plate floor 0.10, rotor f > 1 Exceeded, surge ratio 1.0). Docs docs/data_formats/limit_policy.md; tests tests/unit/test_limit_policy.cpp LP1-LP6 [limit_policy][drivetrain][io] (6 recorded sabotages)
      io/engine_parts.h/.cpp           engine garage P2 (merge 445e9f8): assemble_engine merges the chosen ECU (physics_sim.ecu/1, mirrors the engine key layout: idle/stall/fire/start_complete rpm, DFCO, idle_control, limiter, throttle_actuator_tau_s, ecu.lambda_target/spark_advance_deg, cycle.spark_advance_deg, cycle.cam) and cam (physics_sim.cam/1: intake_lift/exhaust_lift, a checked "fits" block) part into the engine JSON BEFORE read_engine_fields, so a split engine and the same engine inline give the same desc, keys and state hash; "extends" works in every engine-family part (engine, ecu, cam, turbo configuration, turbo hardware, installation, nitrous bottle/kit/controller; depth 8, cycles rejected) via ps_environment_io's load_json_with_extends rebase overload - an ancestor's path-valued keys (listed per format next to that overload) are rebased to the root file's directory; data/ecus/hyper_v8_one1.json and data/cams/hyper_v8_one1.json split verbatim out of hyper_v8_one1_audio.json (pinned against tests/support/fixtures/garage_p2/hyper_inline.json); docs docs/data_formats/ecu.md, cam.md, engine.md "Part slots, extends and the user layer"; tests tests/unit/test_engine_parts.cpp ([io][drivetrain][turbo]: split == inline keys/turbo hash/state hash at 1 and 10 workers, extends chains, slot precedence at three levels, user layer, rejections, "none", from_string; 8 recorded sabotages); decisions vault PHYS-065
      drivetrain/engine_rules.cpp      engine garage P4 (merge 05b6e36), public header core/include/ps/drivetrain/engine_rules.h: check_buildable(ComponentParams) -> BuildIssue{rule, severity, slot, field_pointer, message, source_label, value, limit, unit}; one sourced table core/src/drivetrain/engine_rules_table.h (valve layout {1x1,2x1,2x2,3x2}, intake valve/bore band per layout, row and area packing, exhaust/intake 0.78-0.90, rod/crank radius 3.0-4.2, mean piston speed reusing P3b's thresholds, CR 8-14.5 NA / 7-10.5 boosted, limiter above grid_rpm / boost above grid_p_im / torque-map limiter above its curves graded Near/Limiting/Exceeded at 0-5/5-15/>15 % - all [AS] or [D], no readable Heywood copy, so no [C]); grid rules only for engines with a cycle block, a governed torque-map engine is checked at its high idle; shipped: no Limiting/Exceeded, Near demo_i4_pipes (limiter 4.6 % over the grid) and the hyper (26.2 m/s). engine_change.cpp: classify_engine_change/list_engine_changes -> EngineChange {None, Runtime, TurboTables, Maps, Respawn}, most expensive wins, DERIVED from engine_map_cache_key and turbo_table_key (a copy of load_turbo_configuration_json's FNV list, guarded by K4 against the loader's content_hash); Respawn = kind/architecture/ignition/cylinder count; deviations from the design's Regen column: wastegate area and intercooler drop are Runtime (not in the turbo key). Grid helpers boost_grid_p_im (stock nodes + rows to 1 + target + 0.2 bar, hyper spacing - reproduces [25,50,100,160,220,300] at 1.8 bar), ambient_grid_p_im (caps the top node at 0.95 x ambient [AS] - the NA hyper reaches only 97.9 kPa at 8000 rpm), extend_grid_rpm. Docs engine.md 'Buildability rules'; tests tests/unit/test_engine_rules.cpp R1 S1-S2 C1-C11 K1-K4 G0-G3 [engine_rules][drivetrain][cycle] (2.6 s; G1 demo I4 + turbo in code 623.6 N*m at 5000 rpm, G2 hyper NA in code - moves to the loader's forced_induction none path after P2 - done in P5's FI4); engine garage P5 (merge 3076403): boost_grid_p_im adds a node at ISA ambient when the last stock node at/below ambient is more than 2 kPa under it (kBoostGridAmbientNodeGapPa [AS]; the A25A axis for 0.8 bar ends 97.27, 101.325, 161.325, 200 kPa); lumped cycle engines needed no generator change (a lumped point runs between reservoirs at the node pressure); docs engine.md 'Adding or removing forced induction'; tests tests/unit/test_forced_induction_conversions.cpp FI1-FI7 [turbo][drivetrain][cycle] (FI1 lumped A25A + single_medium Full maps CR 10: charge 0.800 bar, torque x1.8; FI2 demo I4 x three singles, one map key for all hardware; FI3 hyper single_large/twin_medium hold, twin_small/twin_large misfit; FI4 hyper forced_induction none via the loader; FI5 keys differ by the compressor_supply flag + grid only; FI6 family/template well-formedness; FI7 ambient-node guard with a Preview boosted A25A; Sketch maps are unreliable on a lumped boosted engine; hidden FI-D1 prints the fit table)
      drivetrain/launch_control.h      engine garage P8 (merge 7f52832, garage design A15/A16): LaunchControl, a two-step SECOND rev limit passed as TcuRequest::limiter_rpm through the engine's own limiter (EngineTorqueMap and SimulatedEngine: fuel cut with its hysteresis, soft cut over its band) when idle < rpm < own limiter; latch arms on switch && effective clutch engagement < 0.5 [AS] && speedo < max_speed_mps - 0.25 [AS], holds through the launch, ends on switch off or speed > max_speed_mps; one hashed byte, only on cars with the part. ControllerParams = variant<ManualTcuDesc, LaunchControlDesc> (at most one of each). Assist fitment: `fitted` (default true) on AutoClutch/AutoBlip/AutoShift/TractionControl/LaunchControl and PowertrainDesc::anti_lag_fitted (vehicle key anti_lag_switch.fitted) - a not-fitted assist keeps its channel (default 0) and ignores it; assist.launch_control is registered only on a car with the part. Snapshot (never hashed): PowertrainSnapshot::assist_fitted_bits (bit0-5 auto_clutch, auto_blip, auto_shift, TC, launch, anti-lag switch), assist_active_bits bit4 launch, launch_control_active, EngineSoundState::launch_active. Electronics parts: ps/io/electronics_io.h load_electronics_json (physics_sim.electronics/1, kinds launch_control / assist_fitment, extends, unknown keys rejected), attached as a vehicle controllers entry of type electronics or via VehicleLoadOptions::electronics (path replaces/adds, "none" removes); data/electronics/, docs/data_formats/electronics.md. Tests tests/unit/test_assist_fitment.cpp F1-F2 L1-L5 G1-G2 R1-R2 S1-S5 [assist][drivetrain][vehicle] (car_sedan fuel-cut launch 3847-4008 rpm at a 4000 rpm limit; 1 vs 10 workers; no shipped hash moves)
      drivetrain/ pedal law           pedal plan P1 (merge 96d34da + 0c0c3d5, vault PHYS-076): every electronic engine without a governor maps the pedal to T* = Z(n) + c (F(n) - Z(n)) (graph engine theta_idle + c (1 - theta_idle); c = 0 and 1 bit-identical); SimulatedEngine DFCO residual band with 1 % hysteresis (6.9-8.1 % of pedal at 1500 rpm, owner: accepted, to be closed by a closed-plate fired floor later); turbo F = full_load_envelope (engine_dyno.cpp: dyno-settled WOT, the higher of a cold and a pre-spun run per rpm (pre-spin 5 s at rpm+1500, crank ramped down at 250 rpm/s [AS]), memoised 32 entries, 0.61 s on a hyper miss, on the caller thread - a turbo swap rebuilds it) and a gate-first split (ECU part_load_boost efficient|spooled), boost target clamped to [p_amb, min(p_amb + target, p_amb PR_full)]; TurboPair::step target_pa; drive-mode pedal bend only on automatic/DCT/CVT (gearbox_takes_drive_bend, none shipped); IEngine::request_envelope; audio bank keeps the old law (set_pedal_law_override(0)). Auto-shift/kickdown/N->1 read the driver's command BEFORE the TC/LC ceilings (ManualTcu Sensors::shift_throttle_request). On a turbo engine with an envelope a TC/LC ceiling acts as spark retard when the running TC mode's ignition_cut is true (torque x f = ceiling/command, charge and fuel unchanged, the lost work leaves as exhaust enthalpy; plate and gate follow the command); ignition_cut false (shipped Drift; Sport cuts since racing_game PR #5), NA engines or no envelope close the plate as before; TcuRequest::traction_cut marks a TC-only cut. Tests tests/unit/test_pedal_law.cpp [pedal] (NB [long][!mayfail] the DFCO band); B0 V1 counts 'no dip below 90 % before the next upshift' as recovery 0; docs engine.md 'The manual pedal law', 'Assist ceilings on a turbo engine', ecu.md part_load_boost, electronics.md ignition_cut
      vehicle/ assist unit            assists I1 (merge fcb1968, vault PHYS-074): electronics kind `assist_unit` (physics_sim.electronics/1: grade street|sport|racing, traction_control{modes, default_mode, mode_params}, extends, unknown keys rejected; launch_control/aero_control blocks rejected - "grade does not allow" or "not implemented yet (I4/I5)"), stored as VehicleDesc::assist_unit, at most one per vehicle, attached like P8 (controllers entry of type electronics or VehicleLoadOptions::electronics). Public header ps/vehicle/assist_unit_desc.h (TractionMode, resolve_traction_mode, traction_modes_available_bits, traction_default_channel_value, resolve_traction_mode_descs). Channel assist.traction_control codes 0 Off, 1 On (the car's own traction_control block, bit-identical), 2 Relaxed, 3 Sport, 4 Drift (round, clamp; 0.5 stays Off); an unavailable mode maps to the nearest MORE-intervening available mode (On > Relaxed > Sport > Drift), Off never substituted (a unit without Off maps 0 to its least-intervening mode); default channel value = the unit's default_mode, else traction_control.enabled. TractionControlDesc gained min_fraction (default 0) and ignition_cut (default true). Snapshot (never hashed): PowertrainSnapshot traction_mode_requested/traction_mode_effective/traction_modes_available_bits (0 when fitted = false). The mode table is hashed only for a unit offering modes beyond Off/On (a Street unit hashes like no unit). Data data/electronics/assist_unit_{street,sport,racing}.json [AS] (Street Off/On, Sport +Relaxed/Sport, Racing all five; owner-approved mapping), no shipped vehicle carries one yet. Tests tests/unit/test_assist_unit.cpp [assist][vehicle][io] (V6 car_hyper + Racing unit, grass 0.63, 5 %, WOT from 8 m/s: mean rear kappa Off 1.31 / On 0.0966 / Relaxed 0.1433 / Sport 0.2129 / Drift 0.4253; V7 available bits 0b00011/0b01111/0b11111; 8 recorded sabotages); docs electronics.md "assist_unit", vehicle.md
      vehicle/ assist aero control    assists I5 (merge 6523969, vault PHYS-074): an assist unit's `aero_control` block (grade racing only; surface default "rear_wing", default_enabled, cruise_offset_deg 4, cruise_ramp_start/end_m_s 10/30, airbrake_offset_deg 45, airbrake_min_speed_m_s 20, airbrake_brake_start/full 0.45/1, launch_offset_deg 10 [AS]; defaults = racing_game's [D], unknown keys/ranges rejected); core/src/vehicle/assist_unit.cpp aero_control_command - the stateless port of racing_game session.cpp's wing law (+ - * / only), AeroControlDesc/find_aero_control_surface in ps/vehicle/assist_unit_desc.h. Fitted = block AND the car's aero has the named surface (a missing surface is not a load error); channel assist.aero_control registered only then, default = default_enabled (data/electronics/assist_unit_racing.json: true); with the channel >= 0.5 VehicleInstance overrides that surface's command for the tick from the forward AIRSPEED (vehicle wind) and the brake channel, the World command is parked and restored after the substeps (channel 0 = bit-identical to no block); hook set_aero_launch_request(bool) for I4 (LC > airbrake > cruise; always false until I4). Snapshot (never hashed): PowertrainSnapshot aero_control_active/aero_control_target_deg, assist_fitted_bits/assist_active_bits bit6. Tests tests/unit/test_assist_aero.cpp F0-F3 W0-W2 C0-C2 L1 D1 V11 [assist][aero][vehicle][io] (V10 port exact to 0 over 28 cases, 5 recorded sabotages); docs electronics.md, AERODYNAMICS.md, vehicle.md
      vehicle/ assist low-speed fix   assists I2 (merge 5f14b67, vault PHYS-074): while TC's effective mode != Off, the auto-clutch is in Launch/Locked and the ground-speed input rpm of the current gear < 1.25 x anti_stall_rpm [AS], the TC fraction caps the CLUTCH capacity instead of the engine (ManualTcu::Sensors::clutch_capacity_cap_nm, cap = f*T_d + (1-f)*max(0, min_fraction*T_d), output e_auto only, D-LS1) and the engine and launch law keep the driver's command (D-LS3); Launch -> Locked waits while the ground-speed input rpm < anti_stall_rpm (Sensors::ground_speed_mps = the TC estimate, else the mean undriven peripheral speed, NaN with TC Off, D-LS2); ComponentSubstepCtx traction_torque_fraction/traction_floor_fraction/ground_speed_mps set only while TC runs - TC Off is bit-identical, no scenario hash moved. PowertrainDebug::clutch_capacity_cap_tc/tc_via_clutch; diag World::vehicle_substep_telemetry_count/wheel_substep_telemetry, WheelTelemetry::vx_contact. Data: grass lambda_mu 0.42 -> 0.6 [AS] (owner, pending a cited value); car_hyper.json and car_hyper_n2o.json carry the Racing assist unit (controller id "assists", TC default On, aero automation on) and racing_game b5ff167's springs x1.3 / dampers x sqrt(1.3) / attachment z +8.077 mm. Tests tests/unit/test_assist_low_speed.cpp [assist][vehicle][drivetrain] (V1-V4 per surface incl. fixtures fx_grass 0.42 / fx_lambda_035 - the latter creeps 0.046 m/s at 12 % WOT in every mode incl. TC Off, [!mayfail], open: a crawl slip-speed target; V12 the owner's high-boost setup, fixture tests/support/fixtures/assists/v12, a regression guard - it passes before the fix; its "hop" is a wheel-cast artefact against a >= 5 km static ground box, hidden diag PS_I2_V12_GROUND_BOX; V11; hidden I2a spring-back), test_powertrain_tcu.cpp D-LS1/D-LS2 white-box; hyper tests set assist.traction_control/assist.aero_control explicitly
      vehicle/launch_assist.h         assists I4 (merge 6f58dc6, vault PHYS-074): launch control as an assist-unit block `launch_control` (LaunchAssistDesc; grades sport/racing; a car has EITHER a P8 two-step OR a unit LC - loader rejection in either order, VehicleInstance backstop). Phases Idle/Armed/Launch/Handover: arms on throttle > 0.9 + brake > 0.5 + speed < 0.5 m/s + gear 1 [AS]; the engine is held at launch_rpm through the engine's own limiter (P8 TcuRequest::limiter_rpm path) through Armed, Launch and a soft hand-over (in the soft hand-over the clutch keeps regulating the slip until it is closed, open-loop again after max_launch_s; racing_game PR #5, test_traction_control_owner_repro.cpp); the CLUTCH capacity fraction is the primary slip regulator (clutch_up_per_s 6 / clutch_down_per_s 12 [AS]) toward the TC mode's slip target (On 0.10, Sport 0.14, Drift 0.2 [AS]) with a throttle PI as the secondary line; TC is neutral while LC owns the car (the I2 crawl path never fires); the wing hook set_aero_launch_request (I5) asks for launch_offset_deg; anti-lag untouched (Q6). Channel assist.launch_control (registered with either variant); snapshot launch_phase (0-3)/launch_slip_target, assist bit4 = launch for either variant. Data: Racing unit launch_rpm 5000 [AS, tuned by V8: boost at release 0.288 bar at 4500, 0.502 at 5000], Sport unit launch_rpm_fraction 0.55 of the limiter. car_hyper: 0-100 km/h 4.51 s with LC vs 4.73 s TC-On WOT. Armed-rpm band derived from the limiter: every tick in [launch_rpm - h - 25, launch_rpm + 50], mean in [launch_rpm - h, launch_rpm], h = the engine's fuel-cut hysteresis. Tests tests/unit/test_assist_launch.cpp [assist][vehicle][drivetrain] (V8a-V8e, V8c [long], V9, Arm1, P1 precedence, 8 recorded sabotages); docs electronics.md#assist-launch
      vehicle/ drive modes            assists I7 (merge daafb42, vault PHYS-074): public header ps/vehicle/drive_modes_desc.h - DriveMode Eco 0 / Normal 1 / Sport 2 (append-only), PedalBend (one point, exact end points, + - * / only), DriveModeParams (bend, the four auto-shift rpm points, clutch_close_time_s -> AutoClutchDesc::close_time_s, traction_control_default, launch_control_allowed, nitrous_arm_allowed), DriveModesDesc, resolve_drive_mode (an unoffered request = the default mode), validate_drive_modes. Electronics kind `drive_modes` (car-specific part, at most one, extends, unknown keys rejected; needs a manual_tcu), stored as VehicleDesc::drive_modes. Channel assist.drive_mode registered only with the part, default = default_mode; the TC channel default = the default mode's TC default. VehicleInstance: on an effective-mode CHANGE the TC request becomes the mode's default until the driver changes assist.traction_control (state hashed only with the part); bend applied in ThrottleStage after the car's pedal curve, before the hand lever (TC/LC ceilings are fractions of the bent command); LC gate on assist_launch, nitrous gate arm = 0 in Powertrain. ManualTcu Sensors NaN overrides for the shift points/close time; AutoShiftDesc::kickdown_throttle (0 = off; a downshift with the command >= it ignores min_time_in_gear_s, spent by a gear change, re-armed below the threshold, kickdown_spent_ hashed only when set). Snapshot (never hashed) drive_mode_requested/drive_mode_effective/drive_modes_available_bits. Data data/electronics/drive_modes_car_hyper_n2o.json (owner values, default normal), car_hyper_n2o.json min_time_in_gear_s 2.0 [AS, H2: 1.0 s bounced the car in a WOT pull] + kickdown_throttle 0.95 [AS]. Finding: the per-mode close time is nearly inert on a torque-following close (0.05 -> 0.5 rise 0.1125 s in every mode). Tests tests/unit/test_assist_drive_modes.cpp [assist][drive_modes] (S1/S2/H1/H2 [long], 9 recorded sabotages); docs electronics.md "Drive modes", vehicle.md; assists I6 (merge 112f7f5): docs/data_formats/electronics.md#assists-overview is the one overview (where each assist lives, every assist.* channel with codes/default/registration/threshold - LC on is strict > 0.5, aero >= 0.5 - the precedence rules, assist bit table bit0-bit6); tests/unit/test_assist_all_together.cpp V11-all (car_hyper_n2o, 12.5 s with LC, TC modes, drive-mode changes, wing airbrake, kickdown and N2O: 1 vs 10 workers equal hash every tick, own allocation 0, external = the Jolt QuadTree gap) + P1 (a same-value TC rewrite keeps a drive mode's TC override) [assist][vehicle][drivetrain][determinism]; assists I7b (merge 78317a1): DriveModeParams::launch_control_default (optional bool) follows the TC-default rule for the LC enable (a mode change applies it until the driver changes assist.launch_control; launch_control_allowed still gates; default true next to allowed false rejected; ignored on a car without LC); the assist.launch_control channel default = the default mode's value; state hashed only when a mode sets one; data car_hyper_n2o Sport true, Eco/Normal false [owner]; tests tests/unit/test_assist_launch_default.cpp LD1-LD6 [assist][drive_modes]; assists I7c (merge b9cb570, vault PHYS-074): the I7/I7b override (released only on a channel VALUE change - a UI toggle needed two presses) is gone - on an effective-mode change the mode's traction_control_default / launch_control_default is used that tick AND queued as a pending channel write (VehicleInstance::pending_channel_writes(), PendingChannelWrite, max 2), applied by World::step after the pipeline in vehicle id order , allocation-free; get_control then reads what the car does and the driver's next write of any value wins; same-tick conflict: the mode default wins that tick. DriveModeState = {initialised, last_mode}. Adapters that re-send every channel each tick must adopt physics writes after the step with a CAS from the pushed value: adapters/godot/src/channel_mirror.h (Godot-free push_channels/adopt_world_channel_writes, used by PsSimulation pre/post_step). Tests tests/unit/test_assist_channel_writeback.cpp W1-W9 [assist][drive_modes][world]
      vehicle/ abs, esc               assists I5 part 2 / I6 (ABS, ESC): public header ps/vehicle/abs_esc_desc.h (AbsDesc, EscMode Off 0 / On 1 / Sport 2 append-only, EscModeParams, EscDesc, esc_mode_requested/resolve_esc_mode, validate_abs/validate_esc, abs/esc_default_channel_value), law core/src/vehicle/abs_esc.h (step_abs_esc per substep, allocation-free, sensor-only like TC: wheel peripheral speeds, gyro yaw rate, IMU longitudinal accel, brake/handbrake, mean front road-wheel angle). Two blocks of the assist unit (`abs`, `esc`; esc needs abs; any grade): ABS = per-wheel PI on slip vs a signed reference speed (IMU-integrated, lifted to the fastest wheel while braking, re-anchored toward the fastest wheel when every regulated wheel is released - the pull-down bounded by AbsDesc::reanchor_max_decel_mps2 (2.0 [AS]) so it cannot follow locked wheels on ice -, follows the slowest free wheel while only ESC brakes), rear select-low, front split limit (yaw-moment build-up); ESC = bicycle-model yaw target, oversteer -> outer front brake, understeer -> inner rear, engine torque fraction min'd with TC's through the throttle ceiling. Both act only through DirectTorqueBrakes rows via ComponentSubstepCtx::wheel_brake (null = every row reads ctx.brake as before). Channels assist.abs / assist.esc only on a car whose unit has the block; assist bits 8/9; drive-mode keys abs_default/esc_default (pending channel writes); state, blocks and IMU memory hashed only when fitted (20-scenario hash gate unchanged). Data data/electronics/assist_unit_abs_esc.json (street, all [AS]); no shipped car carries it. Tests tests/unit/test_abs_esc.cpp L1 L2 C1 A1-A3 L3a-L3d (low grip: ice/dirt/snow-like, negative control) L4a-L4c (launch then full brake from 20 m/s: the braking reference lifts to the fastest UNDRIVEN wheel, abs_esc.h abs_lift_speed - v_ref within 0.04 m/s, hyper 17.3 m vs 19.7 ABS off, was 31-89 m) E1-E4 D1 H1 H2 [abs][esc] (E1/H1 [long]; 9 recorded sabotages; A1 37.92 vs 38.37 m, split-mu 1.6 deg with a heading-holding driver, FMVSS 126 SWD on a rear-grip x0.75 fixture ESC on YRR 0.19/0.00, lateral 3.16 m, ESC off spins); docs electronics.md#abs-esc
      vehicle/ epb                    Q3 / I8 (racing drive mode + electronic parking brake): DriveMode Racing 3 appended (drive_modes key `racing`, bit 3 of drive_modes_available_bits, a channel value >= 3 = racing); DriveModeParams::epb_mode (park_only | handbrake, optional). Public header ps/vehicle/epb_desc.h (EpbMode, EpbState 0 released / 1 latched / 2 handbrake, EpbDesc max_engage_speed_mps 2.0 [AS] + epb_mode, validate_epb); electronics kind `epb` (at most one, needs brake_system.handbrake rows), VehicleDesc::epb; channel `epb` (level 0/1, on >= 0.5, only with the part). VehicleInstance law: park_only latches a NEW request (rising edge, or handbrake -> park_only by a mode switch) below the limit, else refuses and drops it until the level falls (no retry latch); handbrake applies at any speed; a non-released state raises controls.handbrake to 1 (the car's own handbrake rows). Snapshot epb_state/epb_refused, assist bit 7; state hashed only with the part (no shipped scenario has one: the 20-scenario hash gate is unchanged). Data data/electronics/epb_generic.json, drive_modes_car_hyper.json (normal inert, racing [AS]: TC sport, LC default on, epb handbrake), drive_modes_car_hyper_n2o.json gained racing; car_hyper/car_hyper_n2o carry both. Tests tests/unit/test_epb.cpp L1-L2 C1 E1-E5 R1 [epb] (E5 [long], 5 recorded sabotages); docs electronics.md#epb
      drivetrain/ knock opt-in all    2026-10-18 (owner "opt all engines in to knock", branch claude/knock-all): ignition_strategy "knock_limited" + knock_margin_deg 2.0 in data/ecus/hyper_v8_one1.json, data/ecus/honda_k20a_dc5.json, data/ecus/bmw_n52b30.json (cam-profile and lift-family engines generate their per-set knock tables), data/engines/demo_i4_pipes.json, data/engines/toyota_a25a_fks_gen.json; not possible: toyota_a25a_fks_sim (maps block, no cycle block), torque-map engines (i4_2l_na, r26b_4rotor, diesel, toyota_a25a_fks, hyper_v8_torque). Explicit spark schedules stay (min with theta_kl). Hyper WOT peak 1385 -> 1120 N*m, 1006 -> 809 kW on E5 (retard ~20 deg at 1.8 bar); owner then moved the hyper (engine file, car_hyper, car_hyper_n2o and the hyper fixtures) to E85 (RON 104): 1237 N*m @5000, 897 kW @8000, retard 12-17 deg - calibration bands 1371 N*m / 1 MW still not met, N2O kit fuel jet sized for gasoline now leans the mixture (lambda_combined 0.96). Tests whose premise is the schedule-only engine (K2/K7 in test_knock_limit, limiter_range P1, engine_rules K1/K3) use an opted-out copy; key/hash pins re-recorded (test_wot_generation, test_engine_map_gen_v2, audio bank prep, vehicle_gen_engine scenario hash).
      drivetrain/ cam profiles        engine garage P9 (merge 65a37ae, VTEC-style discrete cam profiles, vault PHYS-072): cam part `profiles` (exactly 2, CycleModelDesc::cam_profiles/kMaxCamProfiles; `profiles: null` removes them) + ECU part `switching` (discrete: rpm_on/rpm_off hysteresis, min_p_im_kpa, min_oil_c, transition_s -> SimulatedEngineDesc::cam_switching; a cam with profiles and no switching, or the reverse, is rejected); one map set and .psmaps per profile, engine_map_profile_cache_keys = the single-lift key of each profile's lifts (engine_map_cache_key throws on a profile desc; profile-free keys unchanged, H3 extended; the profile id is in no key), generate_engine_profile_maps (progress summed over the sets, cancel stops all, files stored only once every set exists), EngineMaps::cam_profiles; LUMPED non-breathing grid nodes are now filled from the nearest breathing node above (they threw before, so every engine that loaded is bit-identical; K20A high cam 34-36 of 171 Full nodes, all <= 30 kPa); SimulatedEngine blends every lookup low + w*(high-low), w ramps over transition_s, idle control on the low cam, torque_demand and the inverse plate blended, oil gate met without a thermal model, (cam_high, w) hashed only with profiles, hot swap swaps all sets together; dyno DynoSweepSpec::cam_profile (-1 = the steady-state ECU choice)/DynoPoint::cam_profile; classify_engine_change switching Runtime / lift Maps / count Respawn / id None; check_buildable CamSwitchWindow 11, CamProfileOrder 12, CamLiftOverValve 13; audio bank, ps_engine_audio and the Godot voice refuse a profile engine (G10 open). Data data/engines/honda_k20a_dc5.json (lumped cycle block; geometry/CR/ratings/6000 rpm switch/8400 limiter/62 mm throttle cited from the Honda Integra fact book 2001) + data/ecus/honda_k20a_dc5.json + data/cams/honda_k20a_dc5_vtec.json (every lift and the 587 mm runner [AS]); generation both profiles on 10 workers Sketch 0.03 / Preview 0.08 / Full 0.39 s; WOT curves cross at 4421 vs 6000 rpm and peak 241 vs cited 206 N*m (reported, not tuned - [AS] cam/runner data, lumped model without exhaust reversion). Tests tests/unit/test_cam_profiles.cpp L1-L3 K1 K2/G1 R1 V1 [!mayfail] V2 H1 D1 C1 B1 Z1 P1 A1 [vvl][drivetrain][cycle] (hidden 'P9 V1 diag' moves the crossing 3520-5125 rpm with the [AS] durations/runner); docs cam.md 'profiles', ecu.md 'switching', engine.md 'Cam profiles'
      drivetrain/ intake lift family  engine garage P10 (merge 1ae7f28, Valvetronic-style continuous intake lift, vault PHYS-075): a cam's `intake_lift_family` (2..8 nodes {u, intake_lift}, ascending u, last u = 1, fixed exhaust_lift; exclusive with P9 profiles) + the ECU's `lift_control` (actuator_tau_s, max_rate_per_s, min_u, idle_u, plate_open_above_u, vacuum_target_kpa; the three u values must be node u values); one complete map set per node, each keyed as the single-lift engine (cycle_desc_for_lift_node; engine_map_set_descs u = 1 first, then ascending u; EngineMaps::lift_nodes; no Table3D), progress/cancel over all sets; SimulatedEngine: plant = linear blend of the two bracketing node sets (exact at a node), ECU torque target from the u = 1 node, pressure-target plate solved against the CURRENT lift every substep (vacuum range p_amb - vacuum), shaft lag + rate cap + snap 1e-6, u hashed only with a family, idle/DFCO on idle_u, hot swap needs the same node u values; rejected with a family: forced induction, mechanical throttle, maps block, profiles; dyno DynoSweepSpec::lift_node / DynoPoint::lift_u; check_buildable LiftFamilyOrder 14 / LiftMinimumBand 15 (+ CamLiftOverValve per node); classify lift_control Runtime, node lift Maps, node u Respawn; audio bank/voice refuse (G10). Data data/engines/bmw_n52b30.json + data/ecus/bmw_n52b30.json + data/cams/bmw_n52b30_valvetronic.json (geometry/CR/ratings/lift range/300 ms stroke/50 mbar vacuum cited, rest [AS]; no car file - no cited gearing); generation 6 sets on 10 workers Sketch 0.11 / Preview 0.29 / Full 1.22 s; WOT at u = 1 bit-identical to the family-less engine; peak 403 vs cited 315 N*m (assumed-runner air excess as A25A/K20A). Part-load fuel saving at 2000 rpm 2.35/1.22/0.59/0.15 % at 2/3/4/5 bar BMEP misses the declared bands (V1 [long][!mayfail]): the [AS] lift family shrinks about a fixed intake centre line, so small lifts close near BDC; the hidden 'P10 V1 diag' with early-closing small lifts gives 8.1/6.7/5.8/4.6 % (cited N52 lift phasing would decide). Tests tests/unit/test_lift_family.cpp L1-L4 K1-K2 R1-R7 V1 H1 D1 C1 B1 Z1 P1 A1 [vvl][drivetrain][cycle]; docs cam.md 'intake_lift_family', ecu.md 'lift_control', engine.md 'Intake lift family'
      engine_cycle/knock.cpp           engine garage P7 (merge d4f3b07, C17b part 1, vault PHYS-069), public header core/include/ps/engine_cycle/knock.h: knock-limited spark per grid node and per listed fuel, generated in map_generator.cpp's run_point job by a closed-cycle replay from the node's converged IVC state (CycleStepper/CycleEngine IVC snapshot, read only by the replay); Douaud-Eyzat ignition delay (OI = RON - 0.5 S) integrated Livengood-Wu over the end gas (air + residual + fuel vapour, iso-octane cv fit [AS]) up to Wiebe x_b 0.90 or EVO, bisected over MBT-40..MBT+10 deg, ECU margin subtracted only at limited nodes. Opt-in only: the ECU part's ignition_strategy "knock_limited" + knock_margin_deg [0, 10] (no ECU spark table = MBT tracking, runtime min(MBT, theta_kl)); octane/margin/model version join engine_map_cache_key and the content hash only then (H3 extended), so no shipped key/hash/pin moved; a loaded maps.theta_kl_deg wins. Tables theta_kl_by_fuel (one per listed fuel, fuels[0] = theta_kl_deg); the Powertrain selects the tank fuel's at build and on a map swap (fuel switch among listed fuels = Runtime; octane/margin/fuel-order edit = Maps). Fuel files gained an optional knock block {model douaud_eyzat, ron, mon, source} (gasoline/e5/e10 95/85 [AS], e25 molar rule [D], e85 104/88 [AS], ethanol 108.6/89.7 [AS], tier3_e10_cert 92/84 [C], diesel none). LimitKind::Knock (18) produced by the dyno: Near > 0.05 deg retard, Limiting >= 5 deg, LimitUnit CrankDegree (11), part = the fuel's /knock/ron, unsolved Simulated remedy. Cost on 10 workers: A25A Full 0.35 -> 0.44 s, hyper Sketch 0.74 -> 0.75 s. Tests tests/unit/test_knock_limit.cpp K1-K8 + KV1 [knock][drivetrain][cycle] (KV1 [!mayfail]: A25A theta_kl 4-13 deg below the measured WOT spark - Douaud-Eyzat uncalibrated; K8 A25A CR 10 + single_medium: knock on 339-352 vs off 479-512 N*m, Limiting Knock). Docs fuel.md, ecu.md, engine.md 'Knock limit'. P7b (merge a037124, knock model v2): the replay runs at the ECU lambda target (CycleModelDesc::knock.lambda_target, copied by the loader and engine_change, in the key only for opted-in engines); one calibrated constant kKnockCalPrefactorScale 1.26 [M] fitted on the A25A's 17 WOT rows at their MEASURED boundary conditions (rms 2.06 deg; B 3800 K [AS], not identifiable) - superseded by knock model v4 (knock calibration phase B, branch claude/knock-calibration; v3 n 1.05 / B 5800 K without dilution was an intermediate step): pre-factor scale 0.004938, pressure exponent n 0.95, B 6100 K, residual-dilution factor (1 - x_res)^5.5 on the ignition rate [M], fitted on the A25A KV1-BC rows + Honda L15B7 Tier 3 envelope rows (EPA NVFEL turbo extracts, tests/support/fixtures/epa_turbo/) at measured BCs subject to the light-load no-knock rows of every EPA package (*_light_rows.json, MAP <= 60 kPa: knock limit at most 2 deg below min(spark, MBT)), constants in core/include/ps/engine_cycle/knock_model.h (KnockModelParams incl. dilution_exponent, carried as CycleModelDesc::knock.model, folded into the map key with kKnockModelVersion 4; KnockReplayInput::dilution_exponent) == data/knock_models/douaud_eyzat_v4.json (physics_sim.knock_model/1, ps/io/knock_model_io.h load_knock_model_json); tests tests/unit/test_knock_cal_turbo.cpp [knock][knock_cal] (KCB file pin, KCB fit incl. the light-load rows, KCB holdout Honda T2 + Ford 2.7, KCB report Ford 1.6/Mazda 2.5T [!mayfail]; hidden KCB diags incl. the engine WOT table, dilution grids, light-load rows, v2/v3/env-candidate generator grid); fit table docs/data_formats/engine.md "Calibration v4"; tests/unit/test_knock_calibration.cpp KV1-BC (asserted, every row within +-5 deg), KV2 held-out Mazda 2.0 SKYACTIV-G [!mayfail] (-14..-19 deg above 2500 rpm), KV2-T (asserted: the Mazda's spark-consistent IVC temperature lies inside the A25A's range +-40 K (v4; +-15 K at v2/v3), the generated Mazda charge is hotter than the spark needs, mean offset < -15 K (v4 -21.3, v2 < -50) - the remaining error is mostly the generator's gas exchange); fixture tests/support/fixtures/mazda_skyactiv_g20/ (grid 70-100 kPa only). P7c (merge a4fedfe, vault PHYS-070, NO model change): tests/unit/test_gas_exchange.cpp [gas_exchange][cycle] - A1/A2 trapped air vs the EPA WOT rows and A3 Mazda IVC temperature vs the KV2-T range, both [!mayfail] (A25A measured/model air 0.834-0.864 at 2996-4493 rpm, guard 'the A25A's air excess is the runner's ram' - the [AS] 709 mm quarter-wave runner; Mazda T_ivc 407-480 K vs 359.8-423.0 K, long-overlap exhaust re-breathing); hidden diags per-term budget, runner/exhaust candidates, runner sweeps; held-out pipe experiment mazda_skyactiv_g20/engine_pipes_p7c.json + installation.json (cited 4-2-1, failed A3, not the KV2 fixture). Next lever: cited runner lengths (2026-10-12 search: none published for either engine, PHYS-070); pipe-mode VE indexed by p_im/p_amb. P7d (merge 6d6d947, vault PHYS-071): ambient-fed pipe engines get one wide-open-throttle cycle point per grid rpm stored as the top p_im row (101325 Pa) of ve/pmep_excess/theta_kl, each rpm's WOT plenum pressure in SimulatedEngineMapsDesc::wot_p_im_pa; lookups go through ps/drivetrain/map_pressure_index.h (p referred to ISA, [last authored row, p_wot(rpm)] stretched onto the top row; an empty curve is the old lookup bit for bit); authored rows at/above the lowest p_wot or trapping >= WOT air are dropped (EngineMaps::wot_dropped_rows, unhashed); held nodes that do not breathe are filled from the nearest breathing node above (EngineMaps::nonbreathing_points, unhashed); .psmaps optional tagged trailing block for the curve, content hash folds it only when present; a file maps block may not mix p_im tables with the generated top row; with_engine_maps fills an empty knock-MBT spark table and SimulatedEngine throws on an empty one. Hyper (compressor-fed, v3) and lumped engines bit-identical; the Mazda pipe experiment now generates Sketch/Preview/Full (0.9/2.0/4.2 s on 10 workers, Sketch hash 0x5e472b63c181ca5a) but its WOT air is still 1.66-1.69x low at 1750-2000 rpm and T_ivc 430-522 K (cam/pipe data); demo_i4_pipes WOT torque -1.4..+0.5 %. Tests tests/unit/test_wot_generation.cpp W1 W5h W8a-W8h [cycle][pipe][drivetrain] (W1 Preview/Full, W8b, W5h Preview/Full [long]; W8c zero allocation covers only the WOT run - building the per-point CycleEngine allocates in every pipe generation job since A5); hidden 'P7d W2/W3/W4 report'; docs engine.md 'Wide-open-throttle row'
      machine/                        attachment A1 (vault FARM-003): coupling_solver.h/.cpp - solve_hitch, an implicit mass-normalised spring-damper on the socket rows plus one row per constrained Cardan angle, solved exactly per substep (Cholesky up to 6x6, ps::drivetrain dense.h), limit rows one-sided, a latch target per row; measure_hitch; 60 Hz, zeta 1 in every shipped class; only vehicle::System calls it. hitch_class.cpp validate_hitch_class, hitch_limits.cpp. vehicle::System owns the couplings: islands (one job, one substep loop per island, step_all takes that path only with a live coupling), Gauss-Seidel sweeps along a chain, attach/detach/retire rebuilding islands, cast exclusions and the collision filter between steps, the tow brakes, the hash fold. VehicleInstance::align_backend_with_shadow, coupled vehicles only: the chassis is set to x - dt*v before Jolt's step so its step lands on the shadow's end pose and keeps its contact correction (free swing at 240/960 Hz: -0.5 % energy over 60 s, was -41 %). Tests tests/unit/test_machine_coupling.cpp [machine][vehicle], hidden [.][diag] measurement tables
  audio/                            target ps_audio (STATIC, ps_audio A7): links ps_engine_cycle (+ ps_detmath/ps_warnings/ps_options) only, NEVER ps_core; core/ never includes ps_audio/ (ci.ps1 include grep; the detmath grep also covers audio/)
    native_output.h, audio_bank.h     976e8d7: RunnerOutput (exclusive Runner lease; invalidate() before the Runner dies) + OutputPump (own producer thread, latency 5-50 ms default 12, ring capped at 50 ms, 10 ms gain smoothing, OutputPumpStats) - samples reach a device callback without render-frame reads; Runner::claim_consumer/read_owned (legacy read() rejected and counted while leased). 40923cf: AudioBank (baked rpm x load-plane recordings, checksum + key checked on read, BankPlayback; Voice::set_audio_bank crossfades 120 ms; e1a9671 PSAB0002: crank-synchronous - each plane is a whole number of 720-deg cycles at an rpm with an integer frame count per cycle (rpm = 48000*120/n), ~1 s, crank0 stored per plane, band-limited at record time (zero-phase 97-tap Blackman sinc), played back by the voice's crank angle with Catmull-Rom and a >= 50 ms whole-cycle seam crossfade; the old sample-time playback kept as a reference (audio_bank_reference.h/.cpp, VoiceSettings::bank_sample_time_reference); an old .psab is a cache miss; tests test_audio_bank.cpp B1-B7; vault PHYS-054 - the old playback caused the demo's ~0.5 s audio steps); LiveEngine substeps 1/2/4/8/16 (half-band cascade) and threads (A9b: persistent ps::engine_cycle StepTeam, step_team.h, bit-identical for any thread count). Tests test_native_audio_output.cpp [native_output], test_voice_threads.cpp; bench_engine_output (not CTest-registered)
    include/ps_audio/, src/         engine_input.h (EngineInput snapshot + flags ignition cut / fuel cut / DFCO), spsc.h (SpscQueue<T,N>, SpscFloatRing - acquire/release on the two indices), time_map.h/.cpp (TimeMap: type-2 rate PLL per arrival, read point ~1 interval behind, rate clamp [0.5, 1.02], hitch > 2 intervals holds, resume jumps without catch-up), halfband.h/.cpp (31-tap exactly symmetric half-band decimator, used only at 2 substeps), live_engine.h/.cpp (LiveEngine: EngineCore stepped per sample at h = 1/48000 on audio dx, crank integrated at audio rate, plenum loop (10 ms filter + multiplicative integral tau 50 ms, frozen at omega 0), per-IVC combustion events with seeded variation, fuel cut/DFCO at IVC, ignition cut at start of burn, radiating ends as a PROVISIONAL monopole p = rho/(4 pi) dq/dt at 1 m [Pa] until A8; probe_stability()), voice.h/.cpp (Voice: SPSC input queue, snapshot interpolation, pitch from omega only, crank-phase correction capped at 1 % of omega while locked, pause fade 50 ms, render(frames); render_to_buffer offline), runner.h/.cpp (Runner: one thread, 256-frame blocks, SPSC float ring, one mono channel per source); A8: source.h/.cpp (SourceKind ExhaustOutlet/IntakeMouth, SourceInfo with installation position, Calibration{pa_full_scale} default 200 Pa at 1 m [A], SourceBlock via the free function render_to_source_blocks - Voice::render keeps its interleaved signature), a Strouhal-scaled flow-noise layer in Voice (f = 0.2 u/D, seeded, level_coeff_pa default 0 = off), and FV open-end radiation on the live path (PipeEndDesc::radiation, the WaveguideNetwork's Levine-Schwinger one-pole reused; the generator never sets it). A8b fixed the sustained-WOT blow-up (runtime CFL sub-cycling), the F1 post-stop ring (catalyst laminar monolith resistance r_lin += 2*fRe*mu*L/(d_h^2*OFA), fRe 14.227 Shah & London [C], generic 400 cpsi/6.5 mil substrate [A], WgNodeDesc::monolith_*; plus the inflow deadband) and W2's radiation excess (inflow fix); V9 has no fitted coefficient. The minmod FV loses ~14 dB/m at 2 kHz at the default 32 mm exhaust dx (3 dB/m at 1 kHz) - part of the HF deficit is numerical. Scalar cost of the audio-grade fixture I4: 18.6-19.4 us per sample = 0.89-0.93 core (FV pipes 83 %, cylinders 16 %, WG 1 %) - live play needs A9.
  engine_tools/                     target ps_engine_tools (STATIC, engine garage P6, merge 8e2980f): include/ps_engine_tools/audio_bank_prep.h prepare_engine_audio_bank(resolved SimulatedEngineDesc, forced-induction TurboPairDesc* or nullptr, AudioBankPrepOptions{cache_dir, workers, cancel, cancel_poll, on_progress}) -> {bank, cache_hit, key, path, seconds, bytes, band-limit energy fractions}; also engine_audio_bank_key, audio_bank_cache_path, prepare_engine_audio_bank_from_files; links ps_core + ps_audio (core/src privately), neither links it; used by tools/ps_engine_audio --audio-bank and the Godot adapter (tools/common/engine_bank_generator is gone); key = settings + engine_map_cache_key (free text blanked) + the 10 operating points the recorder reads (changed once vs the old file-scanning key; bank bytes unchanged, pins 3bf1282f66ec4f9f demo_i4_pipes (knock model v4 2026-10-19, was c983bd04834bc403 (v3), 5611596ff3be5702; knock opt-in 2026-10-18; was 1b0b7ef18220095b, P7d; was 1d178f47cb870602) / 898b073253882eb3 hyper (knock model v4 2026-10-19, was aed7766ad5d31620 (v3), e1cd3181553397a5; knock opt-in + E85 2026-10-18; was 56711496c481c4a7 knock-only, 79cc5e377994856c before)); cancel throws GenerationCancelled and writes nothing; progress 1..total per render tick on the calling thread; tests tests/unit/test_audio_bank_prep.cpp P1-P6 [audio][engine_tools] (P1, P1b, P3, P6 [long]); docs NATIVE_ENGINE_OUTPUT.md "Baking the audio bank"; decision vault PHYS-065
  tests/
    support/                          test-only shared helpers (not part of ps_core)
      counter_force_element.h         deterministic counter-hash IForceElement used by determinism/bench tests
      catch_main.cpp                  custom Catch2 entry point installing headless CRT handlers; target ps_test_catch_main
      vehicle_test_helpers.h          PLAN.md P3: VehicleTestRig/build_vehicle_rig()/settle() - a World with a static ground box (flat or tilted) plus one vehicle built directly from a data/vehicles/*.json file; see "Vehicle system" below for the chassis-half-extents-vs-wheel-shape-cast constraint documented on build_vehicle_rig(). build_vehicle_rig_from_desc() (PHYS-033 I1) is the VehicleDesc-taking core build_vehicle_rig() wraps - takes an already-loaded (test-mutable) desc plus a tilt_axis (unit_y() default = longitudinal slope, unit_x() = cross slope) and tick_hz/substep_hz (for a fine-tick reference run). Both (and build_vehicle_rig()) also take a trailing ground_half_extent (PHYS-033 P4 I3 plan section 13, default 1000 matching every pre-I3 caller) - the accel/top-speed validation tests drive far enough (2000/4000 m) that the old hardcoded 1000 m half-extent would run the car off the edge. Also build_vehicle_rig_on_surface()/flat_surface_mesh() - same, but a flat MeshShape ground tagged with a named data/surfaces/surfaces.json SurfaceId instead of an untagged BoxShape, for tests comparing behaviour across surfaces (a plain BoxShape has no Jolt material and always resolves to kInvalidSurfaceId); build_vehicle_rig_on_surface() also gained trailing optional tick_hz/substep_hz params (PHYS-033 I1b, default 240/960 matching every existing call site) for a fine-tick reference run on a non-asphalt surface. flat_surface_mesh() is tessellated into <= 20 m cells (PHYS-047, 2026-09-26): the old 2-triangle mesh's ~2828 m diagonal ran right under the (0,0) spawn point and fed Jolt's large-triangle CastShape artefact into every wheel cast; pinned by test_vehicle_flat_surface_mesh
      powertrain_reference.h          PHYS-033 P4 I3 plan section 13: RefCar, an INDEPENDENT reference model built field-for-field from the public VehicleDesc (never hand-duplicated JSON constants) - engine curves, clutch, gearbox ratios/efficiencies/inertias, diff, driven/undriven hub inertias, the Pacejka tyre model, aero. ref_gear_loss_params() derives the locked-gear reflected-inertia/torque-gain chain (K_half, J_refl) from the engine/clutch/gearbox/diff node equations (plan section 13's own staged T_m->T_out->T_fd->per-hub derivation); ref_locked_gear_step() is one semi-implicit-Euler substep of the resulting ODE (test 5's "Engaged(k), Locked" case - test_powertrain_engine_braking.cpp). Deliberately simplified (no suspension/pitch dynamics, one-substep mesh-loss lag, event quantisation, lateral dynamics - plan's own "expected combined error <= 1%"), not a byte-exact replica of core/src/drivetrain's own implicit solver
      powertrain_multi_diff_desc.h    PHYS-033 P4 I4 C4: multi-differential test topologies built by rewiring car_sedan.json's VehicleDesc in the test only (no data file) - make_awd_desc(centre_eff) (open centre diff ratio 1 -> open front/rear axle diffs copied from car_sedan's own diff), make_parallel_4wd_desc() (gearbox.out shaft-joined to both axle carriers, no centre diff)
    unit/                             target ps_unit_tests: test_hello, test_math, test_engine_sound_glue (ps_audio A11, [audio]: the shared EngineSoundState -> EngineInput mapping), test_engine_rig_audio (ps_audio A10, tags [audio][drivetrain], R1-R6 [long] - each generates the pipe demo engine's maps, 20-90 s: R1 free rev to the limiter and back, R2 starter -> Running, R3 ignition off -> Off at once (not Stalled), R4 WAV bytes identical twice / seed changes them, R5 ledger closes from tick 1, R6 zero alloc; A8b B1 [long] WOT hold then close at the default audio dx stays bounded, max 22.4 Pa), test_audio_time_map (ps_audio A7: T1 time map, T2 SPSC, T3 half-band), test_audio_voice (ps_audio A7, tags [audio][cycle][pipe]: S1 stability probe 1500 rpm / 30 kPa rel 2.77e-3 -> 1 substep; V1 half/odd orders <= -112.6 dB rel. order 2; V11 seed/PCM/state pins, split render, pinned CycleEngine hash 0x3d50ccb4707a1c36 (the generator never applies the variation); V12a ramp under a 1.5 % clock mismatch tracks the firing frequency to 1.79e-5 away from the ramp corners; V12b 200 ms hitch; V12c 600 s at 0.3 % mismatch, lag within [0.42, 1.42] intervals; V12d crank correction cap 1e-2; V13 stop tail finite, -42.0 dB (bound -36 dB, A8b; exhaust tail tau 0.60 s); A8b V9 order-2 outlet pressure = the monopole rho*dq/dt/(4 pi r) within 1 % (0.99608; rho/4 pi sabotages fail), V10 with flow noise off; V11 pins pcm 0xe4e8cf0d04b01008 / state 0xa798da1cb19bfb03; tie vs CycleEngine within 1 % at 3 points (bound 3 %); Z1 zero alloc over 60 Runner blocks; P1 pause fade; R1 threaded Runner (TSan-able); [.][diag] variants), test_cycle_engine / test_engine_map_gen_v2 / test_cycle_engine_studies / test_engine_rig_wheelless (ps_audio A5, tags [drivetrain][cycle][pipe], fixture tests/support/cycle_pipe_fixtures.h - an [AS] generic I4 with runners, primaries and an installation: S1-S3 pipe-mode stepper, E1-E5 CycleEngine closure/trim/reservoir mode, V7 lumped maps bit-identical to v1 (pinned content hashes and keys), G1 pipe-engine map content hash 0xc1e76ee44ccdef40 (P7d; 0x171173a11cd5cd05 at A8b, 0xeced05eb5892fd4a before), G2 [long] 1 vs 10 workers, G3 unreachable node, G4 version in the key, V8a runner sweep 0.25/0.35/0.50 m peak within +-15 % of the 1/L_eff estimate (tracks one tuning order), V8b primary sweep 1.2/1.8/2.4 m moves the residual minimum 4132/3153/2607 rpm, C1 h vs h/2 <= 0.25 %, C2 dx 50 vs 25 mm within 0.5 % [!mayfail] (-0.72 % at 1500 rpm / 30 kPa, owner accepted 50 mm 2026-09-28, P17 - revisit after A9), EngineRig: a wheel-less crank + LoadDyno Powertrain builds and closes its ledger; [.][diag] trim trace, studies, grid cost), test_spark_schedule (I7c C17a follow-up, X1 no schedule + delay 0 bit-identical maps, X2 runtime eta_i*eta_spark equals the cycle model at scheduled spark with a delay, X3 loader rejections incl. both cycle and ecu schedules given), test_waveguide_network (ps_audio A3, tags [drivetrain][cycle][pipe]: W1-W8 - closed-open/open-open/Helmholtz within 0.02 %, radiation |R| within 0.00226, Kirchhoff fit 0.29 % / mode decay 0.41 %, lossless energy +2.2e-16, pinned hash, resistance exact to 1e-12; A8b W9 catalyst monolith laminar resistance - and V6 expansion-chamber TL within 2.6e-3 dB below the first cross-mode; each with a recorded sabotage), test_pipe_waveguide (ps_audio A3: V4 FV vs WG on a 2->4 mm cone within 0.41 dB per third-octave to 1.5 kHz, I1-I8 face transparency/energy hand-over/quarter wave across the split/mean-flow closure/hybrid vs pure FV 0.19 dB/rejections/zero alloc/resistance mean drop/WG->FV reflection 3.1e-8 (the only check that catches a doubled face admittance); [.][diag] FV numerical dissipation - a sigma 0.05 m pulse at dx 8.5 mm loses 8/15/21 % in 1/2/3 ms), test_pipe_network (ps_audio A2, tags [drivetrain][cycle][pipe]: P1 Sod vs exact Riemann within bounds + no new extrema, P2 closed network mass/energy/burned drift <= 5e-15 over 20000 steps, P3 quiescent taper stays at exactly 0 m/s, P4 closed-open/open-open/Helmholtz resonances within 0.13 %, P5 port hand-off closure, P6 pinned CFL rejection, P7 zero alloc, P8 run-to-run identical; A8b: P9 a supersonic jet stays bounded (sub-cycling), P10 a ring decays identically with ambient +-0.01 K about the pipe temperature, W2 radiation decay 0.861x analytic (bound [0.75, 1.0], was 5.26x), P5 checked on a 50 ms tail mean; each with a recorded sabotage), test_engine_installation_io (ps_audio A4, tags [io][cycle][drivetrain]: engine + installation round trips incl. the vehicle-level override, an engine without pipe blocks bit-identical to pre-A4, every intake/exhaust/resolution/installation rejection message; fixtures built inline), test_engine_map_control (engine garage P1, [drivetrain][cycle][io]: T1 cancel incl. no cache file, T2 progress 1..total on the calling thread at 0/1/10 workers, T3 validate-only - 16 loader rejections give the full-load message, pending engine refused, T4/T4b Sketch numbering/grid/complete/1 vs 10 workers/own key/cache hit, T5 progress+cancel hash-neutral, T6 zero alloc in jobs, T7 background-priority pool same hash; each with a recorded sabotage; hidden [.][diag] 'Garage P1 timing': cold release, 10 workers - hyper_v8_one1_audio Sketch 0.70 s / Full 4.7 s, demo_i4_pipes 0.58 / 2.6 s, a25a_gen 0.02 / 0.38 s), test_engine_map_cache / test_engine_hot_swap (I7c C16b; A4 extended H3: every acoustic pipe/installation field moves the key, positions and source/path strings do not, topology names (interface/into/from) do, tags [drivetrain][cycle][io]: H1 hit bit-identical, H2/R5 corrupt/truncated/wrong-key files regenerated through the loader, H3 key covers every generator input, H4 atomic write, H5 swap visible exactly at the consuming tick (engine work differs; the state hash alone cannot time a swap), H6 1 vs 10 workers + zero alloc/free inside jobs on the swap tick, H7 rejections incl. unknown vehicle id), test_engine_map_generator (I7c C16a, tags [drivetrain][cycle]: M1 runtime reproduces the cycle net torque at nodes to 5e-16, eta/pump/friction off-node <= 1 %, air worst 9 %/mean 2 % (a real ~260 Hz intake-runner Helmholtz ripple, +-2.5 % at 22.5 kPa, sub-grid); M2 eta_i resampling exact at nodes; M3 1/4/10 workers and reverse order hash-identical; M4 content-hash coverage; M5 a complete override runs the generator 0 times; M6 maps hash folded into the state hash (fresh engines differing only in a far-corner VE node); M7 zero alloc; F4 Ecotec motored-friction fit pinned (a 66619.52 Pa, b 91.085, c 346.81 - biased low at high speed, the subtracted fixture pumping 65 kPa at 5000 rpm comes from its ASSUMED valvetrain); [.][diag] ripple, M8 timing and the review's runner-length/pumping-split cases), test_engine_cycle (I7c C15a, tag [cycle], fixture tests/support/cycle_fixtures.h - a generic ASSUMED 2 L I4 plus an LIVC variant: G1-G11 geometry/closed motoring/Otto limit/Wiebe/blowdown both directions/bookkeeping/step convergence/Woschni literals/cycle count/zero alloc/low-speed VE, each with a sabotage; [.][diag] G7/G9 tables, cost (165 ns/RHS), review h sweep and 1-deg valve-area probe), test_simulated_engine / test_simulated_engine_io (PHYS-033 P4 I7b C11/C10: SimulatedEngine white-box against tests/support/mvem_fixtures.h, SimulatedEngineDesc loader rejections), test_simulated_engine_powertrain (I7b C12, cases "C12 *": idle, ambient T3a-d (air/rev vs speed-density 2.2e-16, refill count and bit-identical ISA refill), T4c stage command, T8 WOT launch ledger/tank closure from tick 1 (tick 0 carries the unbooked idle seed, vault PHYS-049 F-T0), F9 burnt fuel, pairing, determinism 1 vs 10 workers + zero own alloc; plus "C12/C17a integration": shipped car_sedan_sim.json idles, WOT 1st->2nd, v > 18.5 m/s at 5 s, balance <= 5e-8 from tick 1), test_a25a_sim_engine_data (I7c C17a-MB: literal pins of toyota_a25a_fks_sim.json/car_sedan_sim.json, WOT anchors vs the EPA xlsx, round trip vs C7), test_fuel_tank (PHYS-033 P4 I7a C6, tags [drivetrain][fuel], fixtures tests/support/fixtures/vehicle_tank_sedan.json / _e10.json / vehicle_tank_fuel_map_sedan.json, cases named "C6 T*": T1 a never-empty tank bit-identical to the plain car every tick; T2 initial - m_tank == fuel_mass - fuel_external_added (worst 1.46e-12 kg); T3/T5 crude and map engines starve at WOT - the emptying substep books exactly the rest (tank exactly 0.0), stall time within 0.3 % + 2h of an integral of J/|m(omega)| over the loaded desc (measured +0.050 %/+0.035 %, idle T3b 0.26 s), dry cranking never fires, refuel + starter restarts; T4 tank selects the fuel, E10/E5 fuel ratio = literal LHV ratio to 1e-12; T6 set_fuel_tank_mass at the tick boundary, clamped, booked signed, T6b reset_vehicle does not refill; T7 every loader rejection message; T8 balance_error while starving (7.5e-16); T9 1 vs 10 workers, zero own allocations; T10 white-box pre_solve with supply +inf/0/half the demand, tau0 and d against literal curve knots in the WOT-slope, soft-cut and governor regimes - each missing s factor on d fails it (e.g. governor d 23.88 vs 0.19 at s = 0)), test_cited_engine_data (PHYS-033 P4 I7a C7: toyota_a25a_fks.json's WOT/motoring curves equal the cited EPA xlsx rows exactly, the fuel map equals its own literal grid at every node plus an aggregate check against every measured fired row (n=182, max 41.10 % on a Pareto-excluded ECU-enriched row, median 0.50 %), tier3_e10_cert.json's literal properties and an independent AFR re-derivation, car_sedan_fuel.json WOT launch without stall with the energy balance held), test_fuel_io (PHYS-033 P4 I7a C4: both shipped fuel files load, each required-field/format/phase rejection; C5c: ethanol and the four blends load and equal the test's own mixing-rule recomputation from gasoline.json + ethanol.json to 1e-6 - fails on a volume-for-mass-fraction mixup and on one corrupted digit; E5/E10/E85 density inside EN 228 / EN 15293 as sanity cross-checks only, no E25 range exists), test_engine_fuel_map (PHYS-033 P4 I7a C5, tags [drivetrain][fuel], fixture tests/support/fixtures/vehicle_fuel_map_sedan.json: fuel-map loader rejections, no-map engines bit-identical, indicated_work = h*omega_new*tau_comb incl. a DFCO-recovery case (the old tau_comb/tau0 split was singular as tau0 -> 0, a grid-search demo reads 2304.85 vs. 40.51), and two 960/960 Hz real-Powertrain cross-checks - fuel flow and the booked fuel_mass equal fuel_map(pre-solve rpm, theta_c) every substep through a free rev, DFCO and idle PI; fuel_mass/fuel_energy_in/engine_heat equal independent per-substep sums over a WOT launch - each shown to fail on a sabotaged build: post-solve rpm lookup 1.33 %, command instead of theta_c 100 %, a dropped booking line 100 %; I7a C5d crude no-map model: tests go through the production to_engine_desc resolver with literal expected constants - dyno-held integral fuel_mass == sum h*max(0,tau_comb*omega_old)/(0.325*LHV_E5) to 1e-12, settled World idle mdot = -m(rpm)*omega/(eta*LHV) within 2 % (the Powertrain's Coulomb row cancels c_k), exact zeros in DFCO/fuel cut/ignition cut/Stalled/Off, engine_heat >= 0 crank->overrun, tractor fuel_mass*0.48*42e6 == sum h*tau_comb*omega_old to 1e-12, World ratios fuels [E10,E5] = LHV_E5/LHV_E10, indicated_efficiency 0.40 = 0.325/0.40, ignition flipped = 0.325/0.48, state_hash identical with/without fuels, zero-alloc, 1 vs 10 workers; sabotages: fuels.back() ratio 0.979 vs 1.021, swapped defaults 1.477 vs 0.677, ignored override 1.0 vs 0.8125, gasoline for the tractor 4.5 %, net torque tau0 instead of tau_comb 21.5 % (idle flow 0), omega_new caught by the WOT free-rev crude test (~0.26 %) not by the dyno-held ones), test_throttle_control (PHYS-033 P4 I7a C3, tags [drivetrain][throttle]: P1 the engine input has no pedal field (static_assert); P2 an explicit electronic stage with a pedal_map flat over the tested pedal range leaves every engine result and both state hashes bit-identical over 1920 substeps (fails at substep 0 when the engine is fed the raw pedal); P3 pedal map at a dyno-held rpm, throttle_eff/torque demand to 1e-12; P4 tests/support/fixtures/vehicle_mechanical_throttle.json - linkage curve applied, floor/ceiling exactly 0/1 while the TCU asks for less, upshift cut zeroes theta_c; P6 tractor over vehicle_tractor_i4's control events compared exactly against throttle_eff/torque demand/governor setpoint/hand_throttle recorded on the pre-C3 code (ba0f486); car_sedan with an extra hand_throttle channel unaffected, snapshot hand_throttle 0; C5b, tag [tcu]: a pedal_map moves the auto-shift upshift point (target stays 1 at 4021.7 rpm, 2 with the TCU fed the raw pedal), and tractor.json's hand throttle reaches the auto-clutch launch law (engagement 0.2289 = the command's e_tgt vs. 0.2722 with the raw pedal)), test_table2d (PHYS-033 P4 I7a C1: Table2D exact at nodes, bilinear vs. hand values, clamping on every side/corner, slopes vs. central differences, zero slope outside, validate() rejections), test_jobs (incl. the queue-full inline-fallback cases), test_world_basic, test_world_snapshot (ArticulationSnapshot/contacts/vehicles additions), test_articulation_core (white-box, core/src on its include path), test_articulation_four_bar (white-box), test_articulation_parallel_alloc (white-box: System::step_all is one job per articulation, debug_alloc-armed), test_aba_fast_transform (white-box: fast vs. reference ABA congruence transform, 1e-12 relative), test_scenario_articulation, test_articulation_contact (Compliant/ProxyBody smoke tests on a single-DOF rail rig, incl. a ProxyBody-at-high-speed [!mayfail] case), test_mesh_heightfield_shapes (MeshShape/HeightfieldShape collision + surface ids), test_terrain_golden (ProceduralTerrainSource vs. terrain.py's golden points), test_terrain_tile_manager (white-box: seams, streaming, throttling, zero-alloc steady state), test_test_ground_loader (load_test_ground_collision + SurfaceTable), test_test_ground_edge_lengths ([.][diag], PHYS-047: read-only triangle edge-length/bad-cast report for data/tracks/test_ground - asphalt max edge 15.03 m, kerb 2.14, low_mu 5.59, dirt/grass one 174.64 m-edge triangle pair each with 0 bad casts over 3.3M probes per triangle), test_terrain_static_overlap (PHYS-021: a parked vehicle's wheels and a sliding probe crossing the test_ground straight's edge, plus a zero-gap sweep of both skidpad ring edges - the terrain/static-feature coincident-collision fix, see "Terrain tile system"), test_terrain_suppression_precompute (white-box, real test_ground data: load-time suppression precompute is bit-identical to the lazy path for every tile, worker-count invariant, and hit at runtime), test_terrain_tile_datum (white-box, R1: per-tile vertical datum for real-world 90-880 m ASL terrain - synthetic sources at +800 m/-350 m land probes and a dropped body within 1 cm, adjacent tiles with different datums stay seam-continuous, relief-overflow counting, plus a 1 m-spacing collision-resolution measurement at datum 0 and ~768/~896 m for flat/30% slope/5 mm step cases, both at sample points and cell centres), test_pacejka_tyre_model (PLAN.md P3: MF5.2-subset pure/combined-slip curves, relaxation ODE, rolling resistance, cross-checked against an independent re-derivation and exact combined-slip-degeneracy identities; plus every shipped tyre's longitudinal slide-to-peak shape - Fx(kappa=1)/peak, Fx(kappa=3)/peak and peak slip at fz0 and 0.75*fz0), test_vehicle_ackermann_symmetry (front wheel Ackermann steer angles are exact left/right mirror images at +-full lock, and each wheel turns toward the commanded direction - see "Vehicle system" below and vault PHYS-022), test_vehicle_parallel_alloc (PLAN.md P4 I0: a full World::step with a vehicle, debug_alloc-armed - PHYS-033 I1b commit C3 added a T3 negative-control case proving the Jolt-allocator hook is wired, and [!mayfail]-tagged the existing zero-alloc case once that hook revealed a pre-existing Jolt-internal allocation, see "Job system"; PHYS-033 P4 I4 C15: a tractor case with a viscous load_dyno on pto.out toggling awd/pto/diff_lock and park_lock after a stop - own count 0, external 620 = the known Jolt QuadTree gap), test_vehicle_io_validation (load_vehicle_json/load_tyre_json error paths: ackermann_fraction range, "format" field incl. /1 rejection, legacy driven/drive/brakes keys, differential efficiency != 1 - PHYS-033 I2 commit 5a; auto_clutch sync_time_s/max_sync_torque_nm must be > 0; auto_blip upshift_cut_release_slip_rpm/upshift_cut_max_hold_s > 0, upshift_torque_ramp_s >= 0), test_body_integration_params (PHYS-033 I1: IRigidBackend::get_integration_params vs. the box-inertia formula, damping/gravity-flag round-trip), test_wheel_contact_frame (PHYS-033 I1b, white-box, core/src on its include path: ContactFrame's normal/long_dir/lat_dir and the true ground contact patch, incl. bottomed/rim-contact/side-wall-fallback cases - see "Vehicle system"), test_sub_threshold_rotation (parking-slope creep fix D2: a free box with a small steady angular velocity does not rotate by default, does when opted in via IRigidBackend::set_integrate_sub_threshold_rotation - see "Jolt backend"), test_powertrain_engine (PHYS-033 P4 I3, white-box: EngineTorqueMap's torque blend against hand values and a central-finite-difference slope check, the Off/Stalled/Cranking/Running state machine's every transition, the idle PI's small/large-dip response, FuelCut limiter hysteresis at WOT in neutral, cranking equilibrium against the starter/Coulomb/smooth-friction balance; PHYS-033 P4 I4 C12 governor: theta_gov against hand values on a lever/rpm grid (the hand_throttle/pedal max() combination and the no-governor case moved to test_throttle_control in I7a C3), the governor dtau/domega term against a central finite difference inside the droop band (1e-6 relative) and absent when saturated, absent when the idle PI or the TCU floor wins strictly inside the band (the round-2 tautology guard - the idle-PI SECTION fails on the old `>=` code), a single governed inertia under constant load settling to the bisection steady speed), test_powertrain_gearbox (white-box: Gearbox's position<->state-index mapping, a full N->1st shift cycle through Selecting/Synchronising, dog release refused while loaded then releasing once torque drops, synchro time matching J_in*|domega|/capacity within 2 substeps, engaging reverse while rolling grinding then aborting to N, Sequential walking one position at a time toward a far target; plus Clutch: engagement = min(pedal curve, auto), capacities scale from it), test_powertrain_tcu (white-box: ManualTcu's counter differences/first-tick baseline/reset/clamped-pending, gear_request precedence over the counter path and the "none" sentinel, the auto-clutch launch law's e(rpm) and Launch->Locked->Launch cycle, the anti-stall panic release rate vs. the ordinary release_rate_per_s, blip/ignition-cut flags following shift phase, launching from Neutral below vs. above the speed threshold, the clutch held open while the gearbox engages from Neutral whatever the rpm, the upshift cut held through the post-shift close until speed-matched (released below upshift_cut_release_slip_rpm, never re-armed in the same shift, released after upshift_cut_max_hold_s, not extended with a manual clutch) and the pedal ceiling's ramp-in, auto-shift thresholds and hysteresis), test_powertrain_network (white-box: dense.h's Cholesky factor/solve against reference Gauss-Jordan and its PSD regularisation, build_nullspace against the plan's own car/tractor worked examples, two inertias locking through a friction row momentum-conservingly, a gear chain's power reproducing ratio*efficiency*torque with lagged mesh loss, an open differential splitting torque and holding a locked hub, a topology switch conserving momentum and booking the exact engagement loss, a friction row's ratio generalising its coupling (synchro row shape), clutch lock-up under a ramping capacity showing no spike - plan section 13.3(a)'s literal criterion on a test-only network, test_powertrain_clutch_lockup.cpp below is the whole-vehicle companion), test_powertrain_friction_lcp (white-box: box_lcp's block-principal-pivoting stage against brute force over batches 1/3/4/physical and a [long] 10^5-case sweep, the k=1 closed form and c=0 caller-level filtering, test_powertrain_control_channels (white-box, core/src on its include path, PHYS-033 P4 I4 C3: declared control-channel list unchanged for car_sedan/tractor, declare_control_channels duplicate-merge and empty-for-real-vehicle cases, a declared channel's default/set_control value arriving in ComponentSubstepCtx::channels exactly one tick late, debug_alloc-armed zero-alloc check with a declared channel present), test_powertrain_multi_diff (white-box, PHYS-033 P4 I4 C4: car_sedan's rigid-row order/Leaf mode/speedo/reset unchanged, AWD and parallel-4WD networks build with the expected recon modes/rows/speedo (4.06, not 8.12, for two axle diffs on one carrier), a lossy diff whose outputs carry other rigid couplings is rejected, reconstruct_mesh_torque_pair against the carrier's truth with a lock row/friction row/coupling on the outputs, nested (centre) reconstruction against truth and bit-identical across post_solve orders, multi-diff reset derivation order leaves-up), test_powertrain_topology (white-box, PHYS-033 P4 I4 C5: composed-state mixed-radix round trip and per-state row lists, no lockable diff -> s == g, lock pending above the mismatch threshold then engaging, engagement loss vs. an analytic projection (1e-12), lossless release, locked |w_L - w_R| <= 1e-12 over 2000 substeps, LeafPair vs. carrier truth, reset keeps the composed state, 4 locks = 128 states / 5 rejected, lockable centre diff rejected, debug_alloc-armed lock toggling while driving), test_powertrain_diff (white-box + whole-vehicle, PHYS-033 P4 I4 C6: spool |w_L - w_R| at rounding level over 2000 substeps lossy and lossless, viscous hub-torque difference = 2*c*|w_L - w_R| and diff_loss = sum h*c*v^2 with a balanced ledger, LeafPair truth for a locked leaf, Nested exact for a lossless locked/viscous centre, lossy non-open centre rejected, plain open bit-identical, debug_alloc-armed zero-alloc World stepping with a viscous diff; I4 C7: clutch_lsd saturation 2|lambda| = preload + f*T_axle (1e-9) with drive -> coast after a carrier-torque reversal, capacity lagging T_axle by exactly one substep, torsen T_high/T_low = TBR (1e-9), LeafPair truth with the row active, whole-Powertrain ledger (<= 1e-12) with diff_loss = sum -h*v*lambda, centre LSD/Torsen rejected, lockable LSD capacity 0 while locked, zero-alloc World stepping and a 1 vs 10 worker state_hash check for both kinds), test_powertrain_transfer_case (white-box + whole-vehicle, PHYS-033 P4 I4 C9: raw-network TransferCase slip/stuck/disengaged identities, tractor.json's rear/front diff ratios 20/15.30 with all four hubs driven, reset derivation order [rear diff, front_diff, transfer], speedo bit-identical 20.0/1.025 to the pre-C9 baseline), test_powertrain_park_lock (white-box + whole-vehicle, PHYS-033 P4 I4 C10: a request above max_engage_speed_rad_s stays pending while spinning down then engages at the threshold and holds, release is immediate, a torque above capacity slips at exactly the kinetic capacity with -h*v*lambda booked into brake_loss, tractor's park row sits between the transfer_case row and the brake rows (brakes shift by one) while car_sedan has none, debug_alloc-armed park_lock toggling (own 0), 1 vs 10 worker state_hash), test_powertrain_pto (white-box + whole-vehicle, PHYS-033 P4 I4 C11: Pto slip bounded by e*capacity and dissipative, engagement ramp at 1/engage_time_s clamped to [0,1], mode change refused while engaged or |omega_out| >= 1 rad/s, disengaged stub brake decaying omega_out per its exact implicit recurrence, efficiency < 1 booking a lagged loss into gear_loss, pto_clutch_loss = row slip loss + stub-brake loss; LoadDyno Brake/Viscous/Quadratic against their exact implicit updates; a load_dyno added to car_sedan in code inserting its row without shifting existing ones, tractor's pto row between the park_lock row and the brake rows; debug_alloc-armed pto toggling with a load_dyno on pto.out, 1 vs 10 worker state_hash with pto toggled and a mode change mid-run), test_powertrain_compliance (white-box + whole-vehicle, PHYS-033 P4 I4 C13: undamped two-inertia period vs 2*pi/omega_n (0.5 %) and the trapezoidal discrete frequency, KE+PE drift over 10^4 substeps (measured 4.7e-13), damped log decrement vs zeta (2 %), backward Euler exactly after a reset / a friction-row stick change / a topology switch, reset relaxes every compliant shaft, twist and the BE flag hashed while a Rigid vehicle's hash ignores stiffness, whole-Powertrain ledger closure per tick, debug_alloc-armed zero alloc, compliant sedan (clutch.out-gearbox.in k 600, c 2) launch at throttle 0.4/1.0 without stall, 1 vs 10 worker state_hash; the implicit_reaction guard: gearbox mesh torque closes gearbox.in's node balance per substep through throttle steps (7.1e-15 N*m; 7.063 N*m with the fix disabled) - a quasi-steady compliant-vs-rigid mesh-torque comparison passes either way, the twist rate is ~0 there; [.][diag] WOT launch speed vs stiffness)
    validation/                       target ps_validation_tests: test_car_sedan_pipes ([long] since 2026-10-06, ~45 s per car in release; ps_audio A11: car_sedan_pipes.json and car_sedan_pipes_rwd.json - 2026-09-29: a WOT free-rev lift never dips below idle, min 800.045 (the SimulatedEngine idle rework: DFCO dashpot at the no-load idle position, manifold-fill term, one-sided PI with the pedal released, idle_control.max_torque_nm authority cap - demo_i4_pipes.json 15 N*m [AS], creeping in 1st/2nd needs 9.8/10.4); settles at idle - last 2 s of 10 s range 0.653 rpm, mean 800.054, within +-10 rpm from 3.05 s (was a ~1040-1300 rpm hunt: demo_i4_pipes.json grid_rpm now starts at 800, closed-throttle leak 0.002, a 25 kPa grid_p_im node under the ~29.5 kPa idle point, the A25A sim engine's throttle curve shape and idle gains; data only, sabotage ranges in the file header) - free-revs to the 6800 rpm limiter, WOT 1st-gear launch to 12.2 m/s without stalling; the same checks run for car_sedan_pipes_rwd.json, launch 13.3 m/s), test_gen_engine_dyno (I7c C17a M-A, tags [drivetrain][validation]: the generated A25A on a dyno-held harness vs EPA NVFEL WOT rows - S1 torque [!mayfail]: with the measured spark schedule and the cited 40.3 deg burn (C17a second follow-up) +2.4/+2.9/+18.9/+16.9/+17.0/+11.1/+0.3 % at 2017..4954 rpm, 4 of 7 outside +-10 % (measured spark + 60 deg burn: -4.9..+11.2 %, only 2996 outside - the slow burn's eta deficit masked the air excess); per-term budget in the file header: trapped air +40..+49 N*m at 3000-4500 rpm from the [AS] quarter-wave 709 mm runner (no cited A25A runner length exists), eta now +0..+8 N*m (was -9..-19), F4 friction 7-8 N*m MORE lossy than the A25A's motoring; nothing tuned, D1; S2 WOT BSFC [!mayfail] (owner-confirmed) now within +-10 % at every point (-0.4..+7.4 %) - fuel and torque both follow the air excess, so the pass does not validate air; S4 pedal law 4.2e-7 of span; check 4 steady torque monotone in plate; [.][diag] RK4 convergence, runner ripple, load cost, budget, spark/motoring review diag), test_free_fall, test_pendulum, test_joint_chain_energy, test_box_on_slope, test_tunnelling, test_resting_contact, test_huge_forces, test_mass_ratio (one case [!mayfail]), test_articulation_pendulum, test_articulation_double_pendulum, test_articulation_energy, test_articulation_mass_ratio, test_articulation_floating_base, test_articulation_actuator, test_articulation_four_bar_energy (white-box, core/src on its include path), test_articulation_contact_measurements (Compliant/ProxyBody: a fixed-base arm resting on the ground, the same rig pressed with a known force, a rail rig pushing a dynamic crate, a floating-base vehicle drop, 5/20 m/s impacts; two [!mayfail] ProxyBody cases), test_terrain_seam_crossing (tile seams vs. a seamless single-heightfield baseline on a synthetic flat/slope/wavy ITerrainSource over the real TileManager pool: sliding sphere/box at 5/20/40 m/s peak |az| <= 2*seamless + 30 and <= 120 m/s^2, the original 256 m/64-sample x = -512 repro likewise; car_sedan.json at 5/20/40 m/s chassis |az| <= 2*seamless + 0.5 m/s^2, peak pitch acceleration <= 2*seamless + 0.5 rad/s^2, peak wheel Fz <= 1.05*seamless; [.][diag] per-case dump cases), test_vehicle_static (corner weights vs. hand calculation), test_vehicle_flat_surface_mesh (PHYS-047: the real wheel cast probed over flat_surface_mesh() near (0,0) - 0 of 160801 wrong vs. 31 for the old 2-triangle mesh, reproduced by a [.][diag] case), test_vehicle_parking_slope (15% slope, <1mm creep in 30s - [!mayfail] history through PHYS-033 I1b was chasing the wrong cause ("elastic hold" weakness); the real mechanism was two numerical defects, a standstill rolling-resistance sign(vx) chatter (TyreDesc::rolling_resistance_v_eps) plus Jolt's own dropped sub-threshold rotation (IRigidBackend::set_integrate_sub_threshold_rotation) - both fixed, [!mayfail] removed, measured drift 2.5e-6 m (1.03e-6 m as of car_sedan.json's PHYS-033 P4 I5 prep 60/40 split, 2026-09-25); a [.][diag] re-check case still runs the 240Hz/960Hz comparison), test_vehicle_step_steer (yaw-rate response does not grow after a step-steer input), test_vehicle_skidpad (PHYS-036 rewrite: speed is held by a tangential CoM force element with throttle fixed at 0 - free rolling, not a traction limit through the FWD drivetrain - walked up through a sequence of held-speed trials (a slow ramp across trials) gated on |mean(dv/dt)| < 0.02 m/s^2, curvature error < 0.5% and no steer/thrust saturation; compared against TyreLimitedSteadyStateSolver, an in-test quasi-static steady-circle solve using the vehicle's own real VehicleDesc/TyreDesc data and the real Pacejka tyre model (replacing the old per-axle peak-sum estimate, whose /2 halving bug - vault PHYS-018 - and missing Mz term made it unusable once Mz's sign was corrected, see PHYS-036) - PASSES, measured 8.8538 vs. estimate 8.7871 m/s^2 (0.76%; 8.6778 vs. 8.6024 m/s^2, 0.88%, as of car_sedan.json's PHYS-033 P4 I5 prep 60/40 split, 2026-09-25, still well under the 4% tolerance), [!mayfail] removed; does not assert which axle limits), test_vehicle_braking ([validation][vehicle][abs][long], PHYS-019 closed by assists I5 part 2: 100-0 km/h with the REAL ABS of data/electronics/assist_unit_abs_esc.json fitted to car_sedan, within 3 % of the desc-derived load-transfer-corrected mu estimate - 37.50 vs 37.95 m (-1.2 %); [!mayfail] removed - the old test-code ABS emulation stayed at +9..+10 %), test_vehicle_accel_standing_start (full-throttle 0-50 km/h average acceleration from an ALREADY-SETTLED rest vs. a point-mass estimate from car_sedan.json's own torque curve, within 15%, driven-wheel slip bounded (no runaway wheelspin); "acceleration on dirt/grass lower than on asphalt" PASSES, [!mayfail] removed (PHYS-047, 2026-09-26): its "chaotic" behaviour was flat_surface_mesh()'s old diagonal seam under the spawn point corrupting wheel casts, not vehicle dynamics - bit-identical over 3 reruns (2 s speeds asphalt 4.714 / dirt 4.289 / grass 2.074 m/s); a third case ("...stays bounded and does not stall (tyre refit 5064b8b + 60/40 split)"): full throttle commanded before the suspension has settled, from t=0 - it used to drive the front tyres to the +-3.0 kappa clamp (documented, not asserted against); since the tyre refit plus car_sedan.json's 60/40 split it asserts max |kappa| < 0.8 over the run (measured 0.461) plus continuous forward progress - see the file's own header comment and out/handover_accel.md for the history (final_drive_ratio 10.0 -> 8.5 in I1b C8, replaced by the real gearbox in I3)), test_vehicle_steer_mirror (full-throttle-from-settled-rest steering response is left/right mirror-symmetric: yaw rate opposite sign but equal magnitude, forward speed gained equally in both directions - see "Vehicle system" below and vault PHYS-022), test_vehicle_standstill_wiggle (a car arriving at near-standstill with residual yaw rate and/or steering settles instead of oscillating - the live-demo skidpad "wiggle" bug; two driven-then-braked cases plus a clean pure-yaw-rate-kick isolation, all asserting decaying not growing/holding amplitude, a worker-count state_hash check for the low-speed code path, a [.][diag] "zeta sweep diagnostic" (PHYS-033 I1 commit E), and PHYS-033 spec test 10 - a yaw kick with zero low-speed damping is neutral and discretization-converged (240 vs. 960 Hz), the same with rolling resistance on, and a [!mayfail] check of the default-zeta decay rate against the section 4.2 modal estimate (measured off by ~14x, a known limitation of that single-mode estimate, not a code bug) - see "Vehicle system" below; Spec test 10.1's tail indices k=17-23 fail its 5% discretization-convergence check (~25% off) - pre-existing (PHYS-037, fails identically at be36545), root cause not diagnosed), test_vehicle_shadow_chassis (PHYS-033 I1 W1: shadow_dv_error/dw_error stay within 1e-4 - measured ~1e-6/1e-7 - for a settled car under step steer, a constant force element and an alternating one matching the predicted one-tick-lag estimate; a chassis forced into contact shows dv_error > 1e-2; a per-vehicle-file low-speed-damper stability bound gamma < 1, split into separate car_sedan/tractor cases by PHYS-033 I1b commit C7 (the patch-lever gamma: car_sedan 0.950 (0.956 as of the PHYS-033 P4 I5 prep 60/40 split, 2026-09-25 - still under 1), tractor 0.652, both still under 1)), test_vehicle_tractor_cross_slope (PHYS-033 I1 spec test 11: a tractor holds a 10% cross slope / 15% longitudinal slope on the handbrake alone - [!mayfail] history through PHYS-033 I1b (same wrong "elastic hold" diagnosis as test_vehicle_parking_slope.cpp) removed now the real cause (same two defects) is fixed - measured drift 3.9e-6 m / 1.3e-4 m; a [.][diag] A/B harness against the H0-H6 cross-slope hypotheses, confirming H0 (data/vehicles/tractor.json's inconsistent rear attachment z) as the root cause of the ORIGINAL owner-reported bug; PHYS-033 P4 I4 C10 park lock cases: park_lock alone in neutral on 15% longitudinal drifts 1.38e-4 m over 30 s while the same rig with park_lock = 0 rolls 519.6 m (negative control), park_lock plus handbrake on the 10% cross slope 4.45e-6 m), test_vehicle_contact_geometry (PHYS-033 I1b: whole-vehicle pins for the contact-patch/force-decomposition fixes - a pitched-at-rest chassis does not self-propel, steady-cornering/straight-braking load-transfer ratios match statics within the plan's own bands - plus a [.][diag] "contact geometry diagnostics" case (skidpad steady-state check, braking/accel traction diagnostics); "Straight braking" LTR_long 1.010 vs. 1.01 +- 0.12 passes, measured against the analytic static axle load m*g*|x_rear|/L (its old unsettled telemetry static sample caused the 1.125-1.149 excess reported from I1b C7 on); "Steady cornering" LTR_lat 1.061 passes), test_powertrain_engine_braking (PHYS-033 P4 I3 plan section 13.4/13.5, tests 5-6: engine braking in 4th from 100 km/h vs. tests/support/powertrain_reference.h's RefCar locked-gear ODE - fixed (commit eb920bf, "test 5: RefCar applies mesh losses direction-aware on overrun": the reference, not the sim, was wrong - RefCar applied the drive-direction mesh efficiency on overrun too, shrinking the modelled engine-braking torque), [!mayfail] removed, measured 0.05% (0.0496% as of car_sedan.json's PHYS-033 P4 I5 prep 60/40 split, 2026-09-25 - unaffected by the split, confirmed deterministic across 3 reruns), see the test's own header comment for the full term-by-term breakdown; plus "stall under load" (handbraked uphill clutch dump stalls the engine), "gentle idle-only launch" (creeps away without stalling), "auto-clutch anti-stall" (never stalls braking to a stop) - all three PASS), test_powertrain_clutch_lockup (plan section 13.3(b), the whole-vehicle companion to test_powertrain_network.cpp's test-only-network case: manual rolling-start clutch lock-up PASSES (unaffected by the PHYS-033 P4 I5 prep 60/40 split, 2026-09-25); auto-clutch launch from rest is [!mayfail] - halfshaft torque continues climbing ~12% above the pre-lock level for up to 0.5s after lock, specific to an abrupt full-throttle auto-clutch launch, not a general driveline bug; still [!mayfail] as of the 60/40 split, measured max_post/mean_steady ~2.23x on both driven halfshafts (626.0 vs. 280.4 Nm, bound 294.5 Nm) and ~1.79x on the clutch itself, same failure category, not re-diagnosed; PHYS-033 P4 I4 C14 compliant variant (car_sedan desc mutated: clutch.out-gearbox.in k 600 c 2, 960/960 Hz, the clutch locks 0.42 s into the pedal ramp): the rigid gates hold over the post-ramp window (clutch 21.80 vs. 22.89, halfshafts 143.5 vs. 150.7) and over ramp + window incl. the lock event (clutch 74.49 vs. 79.14, halfshafts 492.9 vs. 515.9); the post-lock mode 7.711 Hz (zero crossings of a fitted-baseline-detrended shaft torque, first 4 half-periods - zeta ~0.29 leaves only ~2 usable cycles; window/offset spread 2.3 %) is within 2.5 % of a THREE-mass reference - engine side J1 0.164 / hubs 0.0223 / vehicle mass 0.793 kg*m^2 reflected, joined by the tyre slip-relaxation spring k_t = K_x*r^2/sigma_x (804 N*m/rad at gearbox.in, K_x from the real tyre model at the post-lock window mean) in series with its K_x*r^2/V damper - vs. -72 %/-27 % for the two-mass hubs-only/rigid-m*r^2 references; asserted ratio checks sigma_x x2 (0.863 vs. 0.848), pkx1 x2 (1.161 vs. 1.161), chassis mass x2 (1.108 vs. 1.121); measured stays +1.3..+4.4 % above the three-mass model (chassis pitch suspected, not modelled); zeta 0.290 vs. 0.283 report-only), test_car_sedan_gen_n2o (N2O N2 smoke, [n2o][validation]: car_sedan_gen_n2o armed in 3rd, crank power gain and ledger/bottle closure reported), test_car_hyper_n2o (N2O N7, [n2o]: short Preview parts/flow case (5.2 s on a cold map cache), [long] dyno table 3000-7000 rpm with post-N3 bands, [long] 3rd/6th-gear pulls (B0R, merge f20024e: run-up with TC On, TC Off at the arm point, 2->3 at 4500 rpm, 5->6 at 6000 rpm; after pedal P1 the 3rd-gear run-up shifts 1->2 at 5500 and 2->3 at 4000 rpm (arm 3122 rpm, 0.283 s pull, 33.8 g) because TC now retards instead of closing the plate; the second pull moved 5th -> 6th because in 5th the full-boost car plus the shot exceeds rear grip before the 1 s ramp ends; 6th +626/+650 kW at 6000/7000 rpm) - clutch locked every tick, balance/bottle/tank/N2O-energy closure; [.][diag] cases driven by PS_JET/PS_FJET/PS_PULL_GEAR), test_vehicle_787b_shuffle (2026-09-26: car_sedan_787b.json, a lift to throttle 0 in 4th after a steady pull - 1500 kg at WOT and 830 kg at throttle 0.5 (830 kg at WOT spins the passenger tyre onto the limiter in every gear): the lift-off shuffle, fitted as a damped sinusoid plus a quadratic trend over 0.2 s from the first extremum, at 23.87/25.47 Hz (halfshaft torque) and 23.88/24.80 Hz (engine rpm, +-236/+-81 rpm) vs. an n-mass spring/damper model built from the loaded VehicleDesc with the tyre as a Maxwell element (25.03/25.78 Hz), asserted within 7 % and zeta within 0.05; the same car forced Rigid shows no mode (amp/step 0.0004/0.0013, 0 extrema); damping comes from the tyre, not mesh efficiency (all eta = 1 moves f <= 0.13 Hz); [.][diag] cases write the lift-off CSVs rpm_to_wav.py reads; further [.][diag] cases (2026-09-26 ring-duration sweep): one-factor parameter sweep, best-case combinations, an aliasing verification (a c x0.1 input-shaft mode at 180.8 Hz aliases to 76.5 Hz at a 240 Hz tick), a lumped lash ODE, listening CSVs - base zeta 0.226 splits into tyre slip damper ~0.145, shaft dampers ~0.075, engine motoring slope ~0.006, mesh loss ~0; ~500 ms of ring (zeta*omega ~6.5 1/s) only with 830 kg, mu 1.7, 6th at 8500 rpm, Je 0.03, shaft c x0.1 and halfshaft k x0.5 stacked; lash 0-8 deg adds a clunk, no long ring), test_powertrain_energy_balance (plan section 13.6, test 7: the PowertrainEnergy ledger's own balance_error identity stays within a measured-with-margin 5e-8 relative per tick (max observed 1.485e-8, a brief relaxation-lag transient during hard braking with wheel slip - the same class of effect as tyre_loss's "steady sign" caveat) and within the literal <= 1% of (engine_out+starter_in) end-of-run acceptance bound - PASSES; PHYS-033 P4 I4 C15 tractor case: MFWD on, pto plus a dyno, diff_lock toggled while driving, braked to a stop - same per-tick and end-of-run bounds (end-of-run 2.85e-11 J vs. bound 1431.68 J), with awd_clutch_loss/pto_clutch_loss/external_absorbed checked > 0), test_powertrain_accel_topspeed (plan section 13.1/13.2: only test 2, top speed within 3% of a steady-state-per-gear bisection reference, is implemented - PASSES with a wide margin (car_sedan.json: gear 5, v_ref=56.79 m/s / 204.4 km/h, sim 0.24% off; v_ref=56.83 m/s / 204.6 km/h, sim 0.22% off as of the PHYS-033 P4 I5 prep 60/40 split, 2026-09-25 - top speed on flat ground is largely drag/gearing-limited, not split-sensitive); test 1 (0-100 km/h) is explicitly deferred, not forgotten - needs an event-driven Neutral/Engaged(k) x {Slipping,Locked} clutch state machine plus an independent re-derivation of the gearbox's own shift-phase timing, substantially more complex than test 5's locked-gear-only reference which itself needed two real attempts - see the file's own header comment), test_powertrain_auto_clutch_shifts (owner test drive 2026-09-25, auto-clutch around upshifts on car_sedan.json: selecting 2nd at walking pace (5 km/h) never stalls and pulls away on throttle - before the fix the rpm-blind Shift-mode reclose dragged the engine onto a ~350 rpm input shaft and stalled it; a part-throttle 1->2 upshift at 15 km/h locks within 1 s without wheelspin; the torque-following post-shift close (AutoClutchDesc::sync_time_s/max_sync_torque_nm): WOT 1->2 from a non-spinning 1st keeps the driven wheels below kappa 0.2 and locks within 2 s, WOT in 2nd from a steady 20 km/h does not spin, a full-throttle 1->2 at 15 km/h locks within 1 s, a part-throttle 2->3 from ~2500 rpm locks within 1 s; N->1 requested at WOT from rest (flat and 15% uphill) engages within 1.5 s and drives away - the Launch law used to close the clutch on a flaring engine while the gearbox was still synchronising 1st; the assist.auto_shift channel alone enables auto-shift (the TCU desc's enabled flags are only the assist.* channels' defaults) and WOT upshifts near up_rpm_at_full_throttle; the speed-matched upshift cut (AutoBlipDesc::upshift_cut_release_slip_rpm/upshift_cut_max_hold_s/upshift_torque_ramp_s): WOT 1->2 and 2->3 near the shift point, and both auto-shift WOT upshifts, hold the cut through the post-shift close so the car coasts (a < 0.3 m/s^2) from clutch-open to cut release, the torque hole (request -> pedal ceiling back at 1 and clutch torque >= engine torque demand) stays inside a per-case measured bound (1->2 0.78 s, 2->3 0.57 s), the clutch never carries more than torque demand + max_sync_torque_nm after the cut, and the clutch locks within 1.5 s; the clutch-torque ceiling it checks against is the close law's own max(0, T_demand + max_sync_torque_nm) - without the max(0, .) a motoring demand below -50 N*m made a fully open clutch read as an overshoot), test_powertrain_multi_diff_awd (PHYS-033 P4 I4 C4, tests/support/powertrain_multi_diff_desc.h's AWD at centre efficiency 1.0 and 0.99: WOT from rest drives all four wheels, per-tick balance_error <= 5e-8 relative and end-of-run <= 1%, front/rear hub-torque share 0.5 +- 0.02, max |kappa| < 0.5, speedo within 3% of ground speed at a light-throttle cruise; plus a 1 vs 10 worker state_hash equality case), test_reference_cars (baseline smoke for the three reference sedans car_sedan/_rwd/_awd: driven-wheel set, settled corner loads sum to m*g within 2%, a 3 s WOT launch in 1st from settled rest - no stall, speed > 7 m/s, per-tick balance_error <= 5e-8 relative, per-car measured-with-margin kappa bounds (FWD 2.02 / AWD 1.36 driven kappa is real wheelspin without traction control, RWD 0.10) - plus a 1 vs 10 worker state_hash equality case per car), test_powertrain_split_mu (PHYS-033 P4 I4 C8, plan Test 4, re-diagnosed 2026-09-26: car_sedan.json with its front diff mutated per kind, left wheels on asphalt / right on low_mu, on a LOCAL split ground tessellated into 20 m cells - the original 4-triangle +-1000 m ground gave wrong wheel shape casts near its 2000 m seam edge (276 of 28471 probes: phantom fraction-0 hits ~0.12 m above the plane, wrong-side surface ids), which was the whole of C8's "diff hop", viscous wrong-surface drift and locked-braking divergence; a "rig ground validity" cast-probe case pins it, a [.][diag] case reproduces 0/6/80/276 bad casts for half-extent 100/300/500/1000 m. PASS: open launch; locked launch slip/force and yaw (sum(-y*fx), I_z*dr/dt); open/locked braking Mz; clutch_lsd and torsen saturation identities at throttle 0.6 with limiter_active asserted false (at WOT the 60 ms fuel-cut cycle flips drive/coast - not the one-substep capacity lag, whose loop gain is <= 0.45/0.5); viscous 2*c*|dw|; the hub-torque cross-check at 960/960 Hz (one substep per tick, so tick-mean and last-substep torques agree) for all five kinds over [t0+0.2, t0+2], worst 3.6e-11 N*m. [!mayfail]: only the plan's literal telemetry Mz formula for the locked launch (-68.7 vs < -100 N*m) - genuine: lateral tyre forces cancel ~97% of the fx moment at steer 0, and the formula omits aligning moment and rolling resistance), test_powertrain_mfwd (PHYS-033 P4 I4 C9, tractor.json on a local 20 m-cell tessellated ground (ground-validity probe: 2501 casts, 0 wrong): MFWD engaged at gear 6 / 3 m/s PI-held shows the kinematic front lead computed from the desc (1.02025; kappa_F - kappa_R 0.0201545 vs. 0.0202455, +-0.003), clutch slip ~1e-12 and wind-up (fx_f 4422 N, fx_r -2389 N on asphalt); MFWD off (PHYS-033 P4 I4 C12 rewrite): awd_active_ticks == 0, and the front |fx| peak (817.95 N at t=0.346 s, a launch transient) is the front hub plus reflected front-driveline inertia - hub and driveline torque closures <= 1e-9 at 960/960 Hz, |fx_inertial| > 100 N, |fx/fx_inertial - 1| <= 1 - eta = 0.06 (the lagged 6 % mesh loss; measured 1.033), [.][diag] case prints both rates; engagement ramps both ways; drawbar pull (C12, OECD Code 2 style): gear 1, hand_throttle 1, pedal 0, settle at zero load until consecutive 1 s window mean speeds differ by < 0.1 % (60 s timeout), then a 100 N/s rearward ramp; limit = drawbar force at the argmax of the trailing-0.25 s-mean summed driven fx (traction saturation), the 0.5*v0 speed gate only stops the ramp (CHECK limit < stop); within +-25% of a static-Fz Pacejka estimate - off 1666.9 vs. 1564.6 N (+6.5 %, stop 1882.1 N), on 3141.9 vs. 3338.6 N (-5.9 %, stop 3352.9 N), ratio 1.885 (the C9 1 m/s speed-PI target was unreachable - gear 1 at 2400 rpm high idle is 0.613 m/s; a slip-threshold stop at WOT measured wheelspin onset, max kappa 0.76 at zero load, [.][diag] case); 1 vs 10 worker state_hash with awd on/off; debug_alloc-armed awd toggling (own 0, external = the known Jolt QuadTree gap); balance_error worst 2.22e-15), test_powertrain_pto (PHYS-033 P4 I4 C11, tractor.json in N with park_lock, a viscous load_dyno (c 5 N*m*s/rad, J 0.02) shafted to pto.out: 540 mode at throttle 0.35 and 1000 mode at throttle 1.0 after a clean disengage/mode switch, both on the C12 governor's droop line (1355.6 rpm toward n_nl 1392.5 / 2243.49 rpm toward high idle 2400; pre-C12 soft_cut 2213.7 / 2243.5 rpm), window drift of omega_out and engine rpm <= 1e-3 (measured 0), engine_rpm/pto_rpm = 3.61 / 1.95 to 1e-9, |pto_clutch_slip| < 1e-9 (~5e-12), external_absorbed rate = c*mean(omega_out^2) (7731.66 / 72578 W), the pto share of gear_loss = (1-eta)*|lambda|*omega_in to 1e-3 (309.27 / 2903 W), balance_error <= 5e-8 relative (worst 8.2e-16). Pre-C12 (ungoverned throttle) the 1000-mode point at throttle 0.35 was not steady (the engine lugged from ~947 to ~696 rpm over 60 s) - why the case commands WOT, not a longer settle), test_powertrain_pto_governor ([long], PHYS-033 P4 I4 C12: tractor.json's all-speed governor measured through the pto as a dynamometer - a Brake-mode load_dyno at 0/25/50/75/100/120 % of full-rack pto torque at full lever settles at 2368.02 / 2324.15 / 2281.60 / 2240.25 / 2200.00 / 1935.00 rpm, each within 7e-7 of an independent bisection of m(n) + theta_gov(n)*(w(n) - m(n)) = (2 - eta)*tau_dyno/ratio, cross-checked against PowertrainDebug::engine_torque_demand; the same load fraction gives the same crank rpm in 540 and 1000 mode (2281.6 at 50 %))
    determinism/                      target ps_determinism_tests: test_worker_count_hash (1/2/4/10/24 workers + 1000-run randomised-scheduling stress test; one case adds several articulations to the same scenario, one case mixes Compliant/ProxyBody contact-bearing rail rigs with plain chains, one case adds two vehicles built directly from a VehicleDesc with a mid-run step-steer input - migrated to the full PHYS-033 P4 I3 engine/clutch/gearbox/diff chain by I3 plan section 11, one case loads data/scenarios/terrain_multi_interest.json from PS_SOURCE_DIR - two interest points plus suppression, one case (I3 plan section 13.9) hands the same two-vehicle fixture off to assist.auto_shift=1 at full throttle - the fixture's gearbox has no synchro capacity (dog only), so the resulting auto-upshift genuinely grinds (measured grind_events=1, asserted as a scenario-sanity precondition since that field itself is sound-only/not hashed) - hashes still identical across every worker count; PHYS-033 P4 I4 C15: data/scenarios/vehicle_tractor_i4.json at 1/2/4/10/24 workers equal to the recorded 0xee2c08f11ee76f55 (D0 phantom wheel cast re-cast, was 0x44327cc4b10c57b5; tyre crr d1e1144, was 0xe471f1f3e5ad4328) (2026-10-04, was 0xac386af7129b9ffe), plus a companion case asserting each control event's live effect (pto_engagement/pto_rpm, gear 3, awd_engagement 1, rear diff locked in [5,6) and released, stopped with park_lock_engaged and engine Running at t=10) - run_scenario_file now applies scenario.control_events like ps_run (it silently ignored them before))
    bench/                            target bench_force_elements (plain main, also CTest-registered): 2000-force-element worker scaling; target bench_contact_coupling (plain main, also CTest-registered): cost per tick per contact strategy, scaling with contact count; target bench_terrain_tiles (plain main, also CTest-registered): tile-streaming cost/tick against the real test_ground terrain spec, simulated 50 km drive; target ps_energy_study (plain main, NOT CTest-registered): Part A solver-settings energy-loss sweep plus the P1.5a Jolt-vs-articulation table and 10-link cost benchmark, energy_study.cpp
  data/
    vehicles/                         JSON ps::vehicle::VehicleDesc files (ps::io::load_vehicle_json - see "Vehicle system" below): SHIPPED here only car_sedan, car_sedan_sim, car_hyper, car_hyper_n2o, tractor (owner 2026-10-13, merge d3eac89); every other car below lives in tests/support/fixtures/vehicles/ (part paths ../../../../data/..., the demo's --m1-vehicle=NAME falls back there, a value with "/" or ".json" is a repo-relative path): car_sedan.json (FWD 60/40 reference), car_sedan_rwd.json (RWD 50/50 reference variant, same engine/gearbox/tyres/brakes), car_sedan_awd.json (AWD reference: the RWD file's 50/50 geometry, gearbox.out -> open centre diff (ratio 1, lossless) -> open front/rear axle diffs; needs I4 C4 multi-diff), car_sedan_fuel.json (PHYS-033 P4 I7a C7: car_sedan.json with engines/toyota_a25a_fks.json plus a fuel_tank), tractor_fuel.json (I7a C7: tractor.json plus a fuel_tank, no diesel fuel map - owner-deferred), car_sedan_sim.json (PHYS-033 P4 I7c C17a-MB: car_sedan.json with engines/toyota_a25a_fks_sim.json (kind "simulated") and a 350 N*m clutch - PROVISIONAL, the morning-drivable simulated-engine car), car_sedan_gen.json (I7c C17a M-A: car_sedan_sim.json with engines/toyota_a25a_fks_gen.json), car_sedan_pipes.json (ps_audio A11: car_sedan_sim.json with engines/demo_i4_pipes.json [AS] - the car whose physics maps and live engine voice come from the same pipe cycle model; car_sedan's 300 N*m clutch; not the demo default), car_sedan_pipes_rwd.json (owner 2026-09-29: car_sedan_rwd.json with engines/demo_i4_pipes.json), car_sedan_gen_n2o.json (N2O N2: car_sedan_gen.json with a fuel tank and the Edelbrock 50 hp kit/10 lb bottle/basic controller, arm channel nitrous_arm; +41 kW crank at 4000-5500 rpm in 3rd vs the kit's 37.3 kW rating, report only), car_sedan_787b.json (2026-09-26, owner request: car_sedan_rwd.json with engines/r26b_4rotor.json, a 1200 N*m clutch, sedan_6mt/4.06 reused (no citable 787B ratios), compliance compliant: clutch.out-gearbox.in k 21020 c 1.35 (input shaft) and gearbox.out-diff.carrier k 1530.3 c 1.30 = both halfshafts LUMPED through the final drive (C13 allows no compliant diff-output shaft - diff outputs have no inertia); shaft dimensions and zeta assumed, k = G*J_p/L; still the passenger 225/45R17 tyre), tractor.json (MFWD as of PHYS-033 P4 I4 C9: gearbox.out -> transfer_case in; rear -> rear diff (ratio 20, eff 0.94); front -> open front_diff (15.30) -> FL/FR; "awd" channel default 0 = RWD; C10: a park_lock pawl on the gearbox.out shaft, channel "park_lock"; C11: a pto on the engine.crank/clutch.in shaft, 540/1000 modes (ratios 3.61/1.95), channels "pto"/"pto_speed"; C12: the engine's governor adds channel "hand_throttle"; C15: the rear diff is lockable, channel "diff_lock") - geometry derived from data/models/*/*.rig.json's wheel pivots, plausible/published parameters with source noted per PLAN.md section 8. Both are physics_sim.vehicle/2 (PHYS-033 I2 commit 5a): each references an engines/*.json and a gearboxes/*.json file via its powertrain.components' "ref" fields (see "Vehicle system" below)
                                      car_hyper.json (One:1-inspired hypercar: engines/hyper_v8_one1_audio.json - simulated twin-turbo V8, 92 x 95.25 mm, a cycle block and NO maps block, so every map incl. boosted nodes is generated at load; calibrated net 1371 N*m at 6000 rpm and 1000 kW at 7500 rpm (data/engines/HYPER_ONE1.md, bench_hyper_calibration, not CTest-registered); gearboxes/hyper_7mt.json, a park_lock component (channel park_lock), auto-clutch anti_stall_rpm 850, an aero block with splitter/underfloor/rear_wing; engines/hyper_v8_torque.json is a torque_map alternative from the published anchors); car_hyper_n2o.json (N2O N7, owner 2026-10-06 'make it 800' on the hyper car: car_hyper.json + n2o_bottles/bottle_15lb_950psi.json + nitrous_kits/custom_wet_plate_800_bigjet.json (N7b: 3.81 mm N2O / 1.845 mm fuel jet at 4 bar over the manifold, 246 g/s N2O; the fuel jet was 1.535 mm for gasoline and is re-jetted x1.2018 for E85, the car burns E85, kit fuel 46.96 g/s, lambda_c 0.83; the N7 kit custom_wet_plate_800.json, 3.6/1.45 mm, stays as data), controller nitrous_controllers/custom_wet_800.json: progressive 0.4->1 over 1 s, min gear 2, 3000-8100 rpm, 6 deg retard, 35 s cap) - all [AS]; only driveline change clutch 1650 -> 3650 N*m [AS] (shot peak 3153 N*m); N7b (merge 606c428, owner option b): the engine component's "forced_induction" slot (was "turbo_configuration" until engine garage P2) -> turbo_configurations/hyper_twin_n2o.json (turbos/hyper_turbo_n2o.json, wastegate 2.975e-3 m2 per pair vs stock 2.36e-3, [AS] from a bisection + 10 %), car_hyper and its files stay stock; dyno-held mean gain +596.1 kW = 799 hp over 3000..7000 rpm, gate at most 0.965, rotor at most 126500 rpm, boost 1.71-1.78 bar, bottle 27.6 s; with the stock turbo configuration the gate saturates, boost overshoots to 2.0-2.4 bar and the rotor reaches the 180000 rpm cap (sabotage PS_SABOTAGE_STOCK_TURBO=1, 12 failures))
    tyres/                            JSON ps::vehicle::TyreDesc files (ps::io::load_tyre_json) referenced by data/vehicles/*.json: passenger_car_225_45r17.json, tractor_front.json, tractor_rear.json
    engines/                          JSON ps::drivetrain::TorqueMapEngineDesc files (PHYS-033 P4 I3 commit 6, ps::io::load_vehicle_json's component loader): i4_2l_na.json (car_sedan.json and its _rwd/_awd variants), r26b_4rotor.json (car_sedan_787b.json: Mazda R26B, WOT 4000-9000 rpm digitised from SAE 920309 Fig. 6 with the cited 608 N*m/515 kW points, below 4000 rpm a labelled linear extrapolation, fuel_cut 9000, inertia 0.06 derived from 13B rotor mass plus an assumed 0.05, idle 1500 assumed), diesel_i4_4l5_turbo.json (tractor.json; PHYS-033 P4 I4 C12: a "governor" block - all_speed, high_idle 2400, rated 2200, droop 0.0909, channel hand_throttle - and a fuel_cut 2650/100 overspeed limiter replacing the soft_cut) - WOT/motoring torque curves, idle/limiter/starter parameters; I7a C5d: each lists its fuels and ignition - both i4 [e5, e10] spark, r26b [e5] spark (a stand-in, its race fuel is no EN 228 blend), tractor [diesel] compression; toyota_a25a_fks.json (I7a C7: 2018 Toyota 2.5 L A25A-FKS from EPA NVFEL Report 26781 plus its Tier 3 test-data xlsx - WOT = per-speed-line measured torque envelope, motoring measured, fuel map fitted per speed line on Pareto-non-dominated fired rows only (the lambda 0.63-0.73 ECU protection-enrichment rows are excluded and labelled; that ECU behaviour is not modelled), interpolated down to the motoring point, 807-6600 rpm; limiter 6600 ASSUMED; fuel tier3_e10_cert), toyota_a25a_fks_sim.json (I7c C17a fallback M-B, kind "simulated": MVEM tables back-solved from the same EPA NVFEL A25A data by tools/enginedata/a25a_mb_engine.py - own motoring-row friction, measured spark as MBT; WOT reproduced to <= 3.6e-6 relative, part-load +30-60 N*m (no cam-phase axis); the dyno match is tautological and does NOT validate the I7c generator (vault PHYS-050); fuels [tier3_e10_cert, e10], spark, electronic throttle), toyota_a25a_fks_gen.json (I7c C17a M-A, kind "simulated" with a `cycle` block and NO maps - generated at load by C16a from cited geometry (SAE 2019-01-0249), EPA-measured cam, lambda and (C17a follow-up) spark schedules (Tier 1b, cam and spark median-3 smoothed along MAP; cycle.spark_advance_deg is copied into ecu.spark_advance_deg by the loader, both given is rejected; burn_duration_deg 40.3205 CITED (C17a second follow-up: MIT 2.61 lec. 9, modern fast-burn 10-90 % ~20 deg through the frozen Wiebe a 5 m 2; was 60 [AS]), ignition_delay_deg 3.8632 DERIVED (cited spark-to-10 % 15 deg minus the Wiebe's own 0-10 % span 11.14 deg); eta_i stored MBT-equivalent so runtime eta_i*eta_spark reproduces the cycle at nodes), frozen literature constants and [AS] sizing rules, never fitted to the dyno; tier table in its source; 2.96 s to generate on 1 thread, 0.32 s on 10, 7 ms from the C16b cache)
    gearboxes/                        JSON ps::drivetrain::GearboxDesc files (same loader): sedan_6mt.json (car_sedan.json, 6-speed manual), tractor_12f4r.json (tractor.json, 12F/4R), hyper_7mt.json (car_hyper.json, synchronised reverse, reverse_capacity_nm 140) - forward/reverse ratios+efficiencies, synchro capacities, neutral_phase_s/grind_after_s/abort_after_s
    turbos/                           physics_sim.turbo/1 hardware files (pressures in bar): engine garage P5 family small_gt28_class.json / medium_gtx30_class.json / large_gtx42_class.json (three [AS]/[D] sizes by tip-speed similarity from hyper_turbo_approx.json, informational rated_power_kw_per_turbo key, large wastegate 3.0 x throat vs 2.36 x); hyper_turbo_n2o.json (N7b, car_hyper_n2o: hyper_turbo_approx.json with wastegate_area_m2 1.4875e-3 [AS]); hyper_turbo_approx.json [AS] (rotor 2.3e-4 kg*m^2 per turbo [D] from GTX3582R-class wheel sizes (was 2e-5 [AS]; 3rd-gear spool to 90 % of 1.8 bar 1.37 s), wastegate_area_m2 1.18e-3 (one 50 mm valve x Cd 0.6), reference 120000 rpm, efficiencies 0.8/0.8, reference_head_bar 4.5; no surge_line - no cited map); format docs/data_formats/turbo.md (I7d D1)
    turbo_configurations/             physics_sim.turbo_configuration/1 (ps::io::load_turbo_configuration_json: count <= 8, references a turbos/ file, derived maps cached under <cache_dir>/turbo/<hex>.json): hyper_twin_n2o.json (N7b: hyper_twin_default.json on turbos/hyper_turbo_n2o.json); hyper_twin_default.json (count 2, target_boost_bar 1.8 gauge, intercooler 0.95, boost_control kp 1 / ki_per_s 1 / kd_s 0.05 / actuator_tau_s 0.1; I7d D4 intercooler_pressure_drop_bar_at_max_flow not set; blow_off_valve since BOV r3 (merge 09d150c, also in hyper_twin_n2o.json): recirculate, 1330 mm2 per turbo [AS], crack 1.1 / full 1.6 bar, tau 20 ms - no model change (the lumped compressor already balances flow = engine + vent on the speed line); car_hyper_n2o in-shift peak 2.63-2.90 bar gauge (4.7-8.2 without), boost recovery 0.34-0.83 s, shut at idle/cruise, WOT bit-identical; tests tests/unit/test_hyper_bov.cpp H1-H5 [turbo] (H1/H2 [long]; H5 vs an independent flow balance within 6.2 %); no scenario hash moved); an engine names one via "default_forced_induction" (engine garage P2; was "default_turbo_configuration", now rejected), World::replace_turbo_configuration swaps it live; engine garage P5 templates {single,twin}_{small,medium,large}.json (0.8 bar gauge, intercooler 0.75, the hyper's PID; measured bands - 2 L demo I4: small from 2000, medium 2500, large 3500 rpm; 5 L V8: single_large/twin_medium 2000-8000, twin_small overboosts above 6000, twin_large from 3000 - in docs/data_formats/turbo.md)
    electronics/                      physics_sim.electronics/1 parts (engine garage P8, docs/data_formats/electronics.md, all [AS]): launch_two_step_generic.json (4000 rpm, 2 m/s), fit_auto_assists.json, fit_traction_control.json, fit_anti_lag_switch.json
    installations/                    physics_sim.installation/1 files (ps_audio A4 format, docs/data_formats/installation.md): demo_i4_exhaust.json (A10, [AS])
    scripts/engine_audio/             ps_engine_audio event scripts (A10): free_rev.json, stall.json
    n2o_bottles/, nitrous_kits/, nitrous_controllers/  N2O N2/N7 data (docs/data_formats/nitrous.md): controllers carry the optional anti-cycle keys throttle_hysteresis / max_rpm_fade_rpm / progressive.resume_within_s (default 0 = old behaviour; custom_wet_800/2000 set them, tests/validation/test_n2o_oscillation.cpp + test_n2o_controller.cpp C9, docs nitrous.md); bottle_10lb_950psi.json, bottle_15lb_950psi.json ([AS], N7); custom_wet_plate_800.json, custom_wet_plate_800_bigjet.json (N7b) and custom_wet_800.json ([AS], N7, car_hyper_n2o); edelbrock_70011_square_50.json, edelbrock_70060_square_150.json (numbers transcribed in out/n2o_plan.md, not downloaded); edelbrock_basic.json (arm_channel nitrous_arm)
    oxidizers/                        physics_sim.oxidizer/1 (N2O N0): nitrous_oxide.json, generated by tools/oxidizerdata/nist_n2o.py (deterministic, same bytes on re-run)
    knock_models/                     physics_sim.knock_model/1 (knock model v4): douaud_eyzat_v4.json - the calibrated Douaud-Eyzat parameter set with sources and fit record, pinned == the compiled defaults
    fuels/                            JSON ps::drivetrain::FuelDesc files (ps::io::load_fuel_json, PHYS-033 P4 I7a C4): gasoline.json (Heywood 1988 Table 9.7: LHV 44.0 MJ/kg, A/F 14.5; EN 228 density range midpoint 747.5), diesel.json (Heywood Problem 4.12: LHV 42 MJ/kg, F/A 0.067; EN 590 temperate density range midpoint 830) - Heywood App. D Table D.4 has no usable text layer in the scans available; ethanol.json (anhydrous, Heywood Table 9.7 p. 477: LHV 26.8 MJ/kg, SG 0.794, A/F 9.0) and e5/e10/e25/e85.json (I7a C5c: DERIVED from gasoline.json + ethanol.json by ideal volume mixing, rho = v*rho_e + (1-v)*rho_g, LHV and A/F mass-weighted by w_e = v*rho_e/rho - docs/data_formats/fuel.md). No ignition field: spark vs compression is an engine property. tier3_e10_cert.json (I7a C7): the EPA Tier 3 E10 certification fuel as cited by NVFEL Report 26781
    engine_thermal/, cooling_systems/  engine thermal T1 part files (physics_sim.engine_thermal/1, physics_sim.cooling_system/1, "extends" chains, docs/data_formats/engine_thermal.md, cooling_system.md): class_si_passenger.json (5W-30 Walther KV40 69.1 / KV100 12.0, thermostat 88/100 C, wall-heat fraction 0.20 [AS], water_jacket 0.2 m2/L / 3.0 L/L / 5000 W/(m2 K) at 30 l/min/L [AS]), i4_2l_na.json (hydrodynamic_fraction 0.2 [D]), toyota_a25a_fks.json; class_passenger_car.json, sedan.json. Engines reference them via "default_thermal"; only the *_thermal vehicles (car_sedan_thermal.json, car_sedan_sim_thermal.json) install a cooling_system - every other vehicle is unchanged
    surface_conditions/               physics_sim.surface_conditions/1: default.json, 13 classes since 2026-10-05 (788fd39) - the 6 of surfaces.json plus concrete, paving_stones, sett, cobblestone (sealed) and gravel, compacted, sand (soil, sand wet factor 1.10 [AS]; joints of setts/cobbles/pavers modelled as sealed, no infiltration - PROVISIONAL); every number carries a [C]/[D]/[M]/[AS] source entry; a class no surface-table name uses is ignored (unused_entries), so surfaces.json and every hash are unchanged; tests/unit/test_environment_surface_classes.cpp ([environment][surface][io], C1-C5, three recorded sabotages)
    tyre_profiles/                    physics_sim.tyre_profile/1 (weather W6): per-condition friction factors for six [AS] tyre classes, docs/data_formats/tyre_profile.md
    hitch_classes/                    physics_sim.hitch_class/1 (attachment A1, docs/data_formats/hitch.md, limits [AS]): ball, drawbar_pin, fifth_wheel, three_point_locked; data/vehicles/tractor.json lists socket_hitch and socket_3pt, fixture tests/support/fixtures/vehicles/trailer_tipper.json the matching eye and a channel tow brake
    aircraft/                         empty (.gitkeep) until P6+
    surfaces/surfaces.json            ps::io::SurfaceTable input: name -> {lambda_mu, crr} (shared, deterministic, alphabetically-assigned SurfaceId space)
    scenarios/                        JSON scenario files (ps::io::Scenario schema): free_fall.json, pendulum.json, joint_chain.json, articulation_chain.json, multi_articulation.json (several open chains of different sizes, a floating base, a four-bar loop closure - Windows/Linux hash-compare and worker-count determinism), contact_articulation.json (one Compliant and one ProxyBody rail rig - same hash-compare/determinism coverage for P1.5b contact coupling), terrain_tiles.json + terrain_spec_small.json (a small synthetic ITerrainSource + tile-manager scenario, distinct from data/tracks/test_ground - same hash-compare/worker-count-determinism coverage for the M1 tile system), terrain_multi_interest.json (same small spec with TWO interest points, a static pad triggering suppression and dynamic balls on the streamed terrain - guards the cross-platform tile-slot assignment order, vault PHYS-026), vehicle_step_steer.json (car_sedan.json on a flat static ground box, a step-steer control_event - ps_run-drivable vehicle scenario per PLAN.md P3 scope item 7; same hash-compare/worker-count-determinism coverage for the vehicle system, see tools/ci.ps1's `$script:HashCheckScenarios`), vehicle_powertrain.json (PHYS-033 P4 I3 plan section 13.9: car_sedan.json on a flat static ground box, control_events driving shift_up_count/throttle/brake/shift_down_count/a starter pulse over 10s - same hash-compare/worker-count-determinism coverage for the I3 drivetrain chain; state_hash=0xec3f01c51673f663 (pedal P1, was 0xaadddd7bcd24ac15; tyre crr d1e1144, was 0xc72122c8fe8d0228) (re-recorded 2026-10-04 for ca118ec's instant anti-stall release with the rolling re-close back on rolling_reengage_rpm, vault PHYS-040; was 0x5380949669212c71) - updated PHYS-033 P4 I5 prep 2026-09-25 for car_sedan.json's 60/40 front/rear static split, was 0xf797fc7c7bd967d1), vehicle_tractor_i4.json (PHYS-033 P4 I4 C15: tractor.json on a flat static ground box, control_events hand_throttle 0.6, pto 1, shift_up_count 1/2/3, awd 1, diff_lock 1 then 0, brake to a stop, park_lock 1 over 10 s; state_hash=0xee2c08f11ee76f55 (D0 phantom wheel cast re-cast, was 0x44327cc4b10c57b5; tyre crr d1e1144, was 0xe471f1f3e5ad4328; re-recorded 2026-10-04 for ca118ec's instant anti-stall release with the rolling re-close back on rolling_reengage_rpm, vault PHYS-040; was 0xac386af7129b9ffe) at 1/2/4/10/24 workers; ScenarioTopics vehicle+drivetrain), vehicle_fuel_tank.json (PHYS-033 P4 I7a C7: tests/support/fixtures/vehicle_fuel_tank_car.json - a 0.005 kg tank car launched at WOT in 1st - runs dry, the anti-stall release opens the clutch, the engine stalls within 10 s; state_hash=0xb08736ed4b3a8d4f (tyre crr d1e1144, was 0xc89f5ed4df82b2dc) (re-recorded 2026-10-04 for ca118ec's instant anti-stall release with the rolling re-close back on rolling_reengage_rpm, vault PHYS-040; was 0xe545d5f57136415f); hash-compared, ScenarioTopics vehicle+drivetrain), vehicle_sim_engine.json (PHYS-033 P4 I7b C12/I7c C17a-MB: vehicle_powertrain.json's bodies and control events on ../vehicles/car_sedan_sim.json - the simulated engine through World; state_hash=0x92b0090ff4b70656 (pedal P1, was 0x3aed0da9057e5829; tyre crr d1e1144, was 0x5fe7f8d90bbe23e0) (measured 2026-10-04 after the auto-clutch fix, not pinned in a test; 0x063a507387ea89fd at 1 and 10 workers at the 2026-09-29 idle rework; was 0x67872eb9fb8b3396); hash-compared, ScenarioTopics vehicle+drivetrain), vehicle_gen_engine.json (I7c C17a M-A: the same events on ../../tests/support/fixtures/vehicles/car_sedan_gen.json, the generated-map engine; state_hash=0x2fad7071a9ce65c1 (knock model v4, was 0x82bf30120b3212bb (v3), 0x785c1851b2424506; knock opt-in of the A25A gen 2026-10-18, was 0x2164909873144ac3; pedal P1, was 0x194fc20db754f52c; tyre crr d1e1144, was 0x84518e7edcdc3b6a) at 1/2/4/10/24 workers (re-recorded 2026-10-04 for ca118ec's instant anti-stall release with the rolling re-close back on rolling_reengage_rpm, vault PHYS-040; was 0x4030651924c4dbd2 at the 2026-09-29 idle rework; 0x5e88a765a7e748e4 at the C17a second follow-up, cited burn duration; 0xbc61f26e2d05379e with the measured spark schedule, 0xe2a2ebb8616b9f2f before); hash-compared, ScenarioTopics vehicle+drivetrain)
    models/                           Godot art assets (glb + rig.json); owned by a separate session, do not touch
    tracks/test_ground/               owned by a separate session (trackgen), READ-ONLY from ps_core: terrain.json/terrain_golden.json (ProceduralTerrainSource's golden reference), test_ground_collision.json/.bin (ps::io::load_test_ground_collision's input - see "Test-ground static collision" below for the .bin's byte-offset layout)
  adapters/
    godot/                            GDExtension adapter + demo project (P2/M1); built by its own adapters/godot/smoke_test.ps1 (own build dir out/build/godot-adapter) or with PS_BUILD_GODOT_ADAPTER=ON. src/fall_detector.h: Godot-free fall diagnostic called from PsSimulation's sim-thread post_step - prints ONE "ps_godot FALL DETECTED ... END FALL REPORT" block per event when a vehicle's chassis centre drops more than 2.0 m below the terrain source's own height at its XY (tick, sim time, chassis pose/velocity, tile key under the car, resident/starved/precomputed tile counts, static collision under the chassis, per-wheel contact/surface); smoke_test.ps1 fails if it fires during the --m1-autopilot drive
      src/render_interp.h, sim_thread.h  render-time pose sampling (2d8f0b8): SimThread stamps each tick with its NOMINAL scheduled time (tick_nominal_time, tick_clock_break on the first tick / a stall resync); PoseHistory (8-frame ring, mutex held microseconds) is sampled once per Godot frame at now - D (default 2 ticks, set_render_delay_ticks / --m1-render-delay=), clamped at the newest frame, teleports and clock breaks step; bodies and articulation links share one sample; the old publish-time alpha (render_interp_alpha) stays behind --m1-render-legacy for A/B; get_render_diagnostics; measurement flags --m1-wot-drive/--m1-render-log/--m1-render-seconds; tests adapters/godot/tests/test_render_interp.cpp. Inputs: gamepad Y / key B toggles the "nitrous_arm" channel, RB / V the camera (804137e). Headless runs create no tyre-audio players (--quit-after leaked their generator playbacks); a window close shuts them down before quitting
      src/vehicle_voice.h/.cpp        ps_audio A11: VehicleVoice::try_create builds a ps_audio Voice + Runner from the vehicle's OWN engine desc when it is a simulated engine with a cycle block and pipes (else nullptr, reason logged once; no separate sound engine), live dx = kLiveDxScale 2.0 x the engine's audio dx (scalar voice at the default dx measured 20.89 us/sample = 1.00 core, no margin; 2x = 13.78 us, 0.66 core) on a desc copy only; PsSimulation feeds it from post_step and exposes has_vehicle_voice/get_vehicle_voice_sample_rate_hz/get_vehicle_voice_channel_count/read_vehicle_voice_audio (divided by set_voice_full_scale_pa, default 15 Pa)/get_vehicle_voice_underrun_count (empty-ring reads); demo main.gd mixes the sources to mono into an AudioStreamGenerator, keeping only ENGINE_AUDIO_TARGET_S 0.05 s queued (filling its 0.2 s buffer to the brim put ~250 ms between an rpm change and the speaker), Runner ring 4 x 256 frames (21 ms), and prints Godot playback skips. Drive with sound: --m1-vehicle=car_sedan_pipes
      src/spatial_audio.h/.cpp        PsSpatialAudio: Windows Spatial Sound (ISpatialAudioClient dynamic objects, Dolby Atmos) on a private audio thread; fed by push_audio from the main thread, or by attach_native_source(OutputSource, latency_ms, gain) (976e8d7: a native OutputPump feeds it, get_frames_needed returns 0 and push_audio is rejected and counted while attached; get_output_diagnostics). PsSimulation::connect_vehicle_voice_spatial(name, spatial, latency_ms = 12) attaches a VehicleVoice's native output with gain 1/voice_full_scale_pa (15 Pa); the AudioStreamGenerator path above remains the render-fed fallback. Turbo/audio-bank live swap: request_vehicle_turbo_configuration, get_engine_audio_bank_status, get_turbo_configuration_status; set_engine_audio_bank_enabled (default false - the demo's --m1-audio-bank turns it on; disabled, the bank worker reports "Disabled" and only the turbo physics swap applies). Diagnostics (3db5525): env PS_VOICE_INPUT_CSV records the EngineInput stream (+ .timing.txt voice timing lines), PS_VOICE_OUTPUT_RAW captures the voice output (raw float32 + .reads.csv), PsSimulation::take_vehicle_voice_timing, demo --m1-audio-queue-ms=N (fallback queue 20-200 ms, default 50). Docs: adapters/godot/SPATIAL_AUDIO.md, VEHICLE_SURFACE_AUDIO.md (telemetry-driven tyre audio), NATIVE_ENGINE_OUTPUT.md, HANDOFF_TURBO_AUDIO.md
      godot_common.ps1                shared helper (dot-sourced by smoke_test.ps1 and the root drive.ps1, never run directly): Enter-VsDevShell, Find-GodotConsoleExe/Find-GodotExe, Invoke-AdapterCMakeConfigure/Build (return cmake's exit code; their own stdout is piped through Out-Host, not left in the function's return value - concatenating the two silently broke `-ne 0` exit-code checks once, see the file's own comment), Test-AdapterConfigured, Test-DllLocked (drive.ps1's locked-DLL-before-build check)
```

## Targets

- `ps_core` (STATIC library): public headers in `core/include/ps`, sources in
  `core/src`. Links JoltPhysics and nlohmann_json `PRIVATE`. No Jolt or
  Godot type appears in any public header.
- `ps_detmath` (STATIC, `core/src/math/transcendental.cpp` + `core/src/math/detmath/*.cpp`, ps_audio A1):
  the deterministic `ps::math` transcendental implementations, no dependencies; linked
  PRIVATE by `ps_core` and `ps_engine_cycle`.
- `ps_engine_cycle` (STATIC, ps_audio A1): the live 0D cycle model (geometry,
  cycle_stepper, cycle_point, firing_order) shared by `ps_core` (map generator)
  and the future `ps_audio`; links only `ps_detmath`/`ps_warnings`/`ps_options`,
  NEVER `ps_core`. Guard: `ps_engine_cycle_standalone` (`tests/unit/
  engine_cycle_standalone_main.cpp`, plain `main`, add_test label `cycle`) links
  it WHOLE-ARCHIVE without `ps_core` and runs one Preview cycle point, a short
  pipe network, a small FV+waveguide network and (A5) a Preview multi-cylinder pipe I4 CycleEngine - putting
  map_generator.cpp back into it fails the link (JobSystem::submit/wait).
- `ps_environment` (STATIC, `core/src/environment`, public headers `core/include/ps/environment/`; weather plan W1,
  3e76232, out/weather_plan.md, vault PHYS-061): EnvClock (closed form from the tick count, scale 0 freezes),
  Calendar (seasons of n days, Dynamic/FixedSeason/FixedDay, per-day values interpolated between season
  midpoints), Sun (Spencer declination - 0.4-0.5 deg off near the equinoxes, astronomical-warped onto the game's
  day/night lengths or an authored curve; both day-length forms), Haurwitz clear-sky GHI + Kasten-Czeplak cloud
  factor. Links ps_detmath/ps_warnings/ps_options only, NEVER ps_core (guard `ps_environment_standalone`).
  `ps_environment_io` (STATIC, nlohmann PRIVATE): ps::io::json_merge_patch (RFC 7396), load_json_with_extends
  ("extends" chains bottom-up, user file last, cycle detection, depth 8), load_climate_json
  (physics_sim.climate/1, docs/data_formats/climate.md; keys of later increments rejected as reserved). Tests in
  the separate exe `ps_environment_tests` (no ps_core on its link line), tag [environment]: E1-E5, RFC 7396
  appendix, loader rejections. data/climates/test_fixed_summer.json. W2 (118f93c): `Environment`
  (environment.h/.cpp: attach/set_time/set_time_scale/begin_tick/current/fold_state_hash - the clock anchor
  t = anchor + scale*(tick - anchor_tick)/tick_hz, never accumulated) and the POD `EnvironmentSnapshot`
  (environment_snapshot.h: time, season/day, day/night lengths, clock_hours, sun elevation/azimuth/direction
  in the World frame, GHI, cloud cover). World: install_environment (anchors at the current tick)/
  remove_environment/environment()/environment_snapshot()/set_environment_time/set_environment_time_scale;
  runs in the existing freeze_tick_start_state stage (no new stage or telemetry channel), anchor/scale/mode
  hashed only while installed, WorldSnapshot::has_environment/environment. Scenario "environment" block
  (climate, seed, time.start/scale, season, north_yaw_deg, explicit events jump_time_s/set_time_scale;
  io::apply_environment_events, called by ps_run and the determinism harness; docs/data_formats/scenario.md);
  data/scenarios/environment_day.json (hash-compared, 0xce55f12949e0ee73 (tyre crr d1e1144, was 0x2af395ee8a54f0b5) at 1/2/4/10/24 workers). Tests
  test_environment_object.cpp (ps_environment_tests), test_environment_world.cpp ([environment], zero alloc
  armed). Godot: PsSimulation::install_environment/has_environment/get_environment; demo flags --m1-climate,
  --m1-time-scale, --m1-time-of-day (24 h clock), --m1-season, --m1-day; the DirectionalLight/sky follow the
  snapshot sun, the HUD shows clock/season. W3 (2b90f35): weather as a pure function of (climate, seed,
  overrides, env time) - weather_desc.h/.cpp + weather.h/.cpp (per-season regimes, semi-Markov chain with
  log-normal durations from an Irwin-Hall n=4 normal, WeatherCursor 64-segment ring - a query before the ring
  restarts at segment 0, convective triangle cells inside a segment, first-order lags, smoothstep override
  windows chained in t_start order with exact restore, closed-form trailing accumulations, thinned-Poisson
  lightning, fold_hash; SplitMix64 counter-hash RNG weather_rng.h), air_temperature.h/.cpp (Parton-Logan on the
  game's day/night lengths, Magnus dew point/RH), weather_classes.h/.cpp (WMO/DWD/Beaufort/TORRO scales,
  classify()), weather_io.h/.cpp (physics_sim.weather_preset/1, physics_sim.weather_classes/1). Climate blocks
  weather, lags_s, precip_phase, humidity, diurnal_temperature, lightning (docs/data_formats/climate.md,
  weather_preset.md, weather_classes.md). Data: data/weather_presets/*.json (12 presets, [AS]),
  data/weather_classes/wmo_dwd_torro.json, data/climates/test_weather.json ([AS] fixture). Tests
  test_environment_weather*.cpp ([environment][weather], ps_environment_tests); evaluate() 0.28 us sequential, 5.5 us random access at 30 days.
  W4 (c2aca55): atmosphere.h/.cpp (ISO 2533 ISA lift of the weather sample to an altitude, humid-air density from the
  vapour pressure), wind.h/.cpp (GustField: Shinozuka sum of 32 Kaimal lines per axis, sigma ratios 1:0.8:0.5, Taylor
  frozen-turbulence shift so nearby cars see coherent gusts, counter-hash phases, 600 s windows blended over 30 s,
  stateless - a pure function of seed, window, sim time and position; gusts on SIM time, weather on env time),
  vehicle_air.h (POD VehicleAir: ambient, density, mean wind + gust). Environment weather source Off/Dynamic/Preset
  with altitude_offset_m and sample_air (weather folded into the hash only while on). Climate "gusts" block
  (reference_gust_factor default 1.35 - MEASURED gust factor of the Gaussian field at the WMO open-land intensity
  0.1515, not the plan's cited 1.66, which a Gaussian field reaches only at I ~0.29); scenario "weather" and
  "altitude_offset_m". Altitude = location.elevation_m + offset + chassis z (no terrain lookup, no wind height
  profile). World computes one VehicleAir per vehicle per tick in freeze_tick_start_state and passes it through
  VehicleTickEnv: the engine's live ambient (SimulatedEngine stays on set_live_ambient, steady_rebuild_count 0), aero
  density and wind; VehicleSnapshot::air, World::vehicle_air/vehicle_engine_ambient/
  vehicle_engine_steady_rebuild_count; AmbientSample::vapour_pressure_pa (default 0 = dry, never hashed).
  World::set_ambient/set_aero_environment throw std::logic_error while an environment WITH weather is installed
  (a weather-less W2 environment keeps the old behaviour). Without an environment bit-identical to before.
  Cost: begin_tick 2.29 us/tick, sample_air 1.60 us per vehicle. Data data/climates/test_weather_gusts.json,
  data/scenarios/environment_weather.json (0x6dd1b38a446aed23 (pedal P1, was 0xa3ca509638ae73cc; tyre crr d1e1144, was 0x2520d724148fa822) at 1/2/4/10/24 workers, hash-compared). Tests
  test_environment_atmosphere/test_environment_wind (ps_environment_tests), test_world_atmosphere (ps_unit_tests,
  [environment][vehicle]). Adapter: PsSimulation::get_vehicle_air, demo flags --m1-weather, --m1-altitude-offset,
  HUD outside temperature/pressure/density/altitude/wind. Decisions vault PHYS-061, notes out/weather_w4_progress.md.
  W3b (owner answers Q2/Q12/Q5): third day/night form `{"astronomical": true, "day_share_multiplier": m}`
  (DayNightKind AstronomicalScaled, share = clamp(H0/pi*m, kDayShareMin 0.02, kDayShareMax 0.98) [AS], m in (0, 10],
  per season, total stays cycle_s; m = 1 bit-identical to Astronomical). Season-bound weather override ("manual
  weather overrides the season, resets with the next"): WeatherTimeline::add_season_override/set_calendar/
  end_all_overrides, the ramp-out FINISHES at the season boundary (shortened to half the remaining time if needed) and
  the weight is exactly 0.0 afterwards, so the base weather is bit-exact from the boundary; under a locked calendar
  (FixedSeason/FixedDay) it lasts until ended. Environment add_season_weather_override/remove_weather_override/
  clear_weather_overrides/end_weather_overrides/set_calendar_lock; World add_weather_override_to_season_end/
  end_weather_overrides/remove_weather_override/clear_weather_overrides/set_environment_calendar_lock (logic_error
  without an environment / without weather). A runtime lock re-anchors the clock keeping year, game day and fraction
  of day (env seconds jump when day lengths differ - PROVISIONAL); the weather cursor is invalidated by a calendar
  generation. Season-bound overrides hash only when present. Scenario environment events set_weather{preset, ramp_s},
  end_weather, clear_weather, set_season (name or "dynamic"). Adapter PsSimulation set_weather_override/
  end_weather_overrides/clear_weather_overrides/set_season_lock (pending queue applied in pre_step); the demo's
  env time scale defaults to 24 (24 h game = 1 h real; World default stays 1.0). Trailing 1 h/6 h precipitation sums
  keep override rain for up to 6 h after it ends (physical accumulation). Tests test_environment_day_share/
  test_environment_season_override (ps_environment_tests), test_world_weather_override (ps_unit_tests).
  W5 (4a43f37, 5d0466c): surface condition core - surface_condition_desc.h (physics_sim.surface_conditions/1 desc, climate optional
  "condition" block dt_env_s 60 / spinup_s 7 d / max_fast_forward_s 30 d), surface_condition.h/.cpp (1D vertex-centred FV
  conduction column per class, 5 mm top cell x1.3, <= 32 nodes, <= 10 m, backward Euler + Thomas, dense reference in the test;
  surface balance (1-albedo)GHI + Geiger/cloud sky longwave + linearised emission + Vehrencamp h_c; Dirichlet bottom at the
  day's ground_temp_c; ConditionGrid S[k]/S[k+1] on floor(t/dt), forward <= max_ff stepped, else spin-up; checkpoint/restore
  bit-exact; W6-W8 zero placeholders), surface_conditions_io (ps_environment_io). Environment steps it in begin_tick,
  EnvironmentSnapshot::surfaces (unhashed), fold only with surfaces; World::install_environment checks names vs the surface
  table; scenario key "surface_conditions" (needs surface_table_path + weather). data/surface_conditions/default.json,
  docs/data_formats/surface_conditions.md. Tests test_environment_surface.cpp ([environment][surface], ps_environment_tests:
  S1 SHRP +1.03 K, S2 Kallas range +1.6 % / t_max +0.26 h / 12 in rms 1.11 K, S2b night min -4.5 K [!mayfail], S3 0.033 K,
  S4), test_environment_surface_world.cpp ([environment][vehicle][world]). data/scenarios/environment_surface.json
  (0x030df2de793ce721 at W5, 0xa413eb2036d292e8 (pedal P1, was 0xcef127a9c2643925; tyre crr d1e1144, was 0x64326439851d8dfc) since W6). Adapter: get_surface_temperature_c, demo --m1-surfaces, HUD road line. Decisions vault PHYS-061, notes out/weather_w5_progress.md.
  W6 (222103b, DeepSeek draft finished by Sonnet): water and wet grip. SurfaceClassState gains hashed water_mm/canopy_water_mm/
  soil_moisture/drain_tau_s, stepped in the same env-time step as the thermal column (surface_condition.cpp step_column): rain ->
  canopy (vegetated) -> infiltration min(ksat*dt, pore space) -> film -> evaporation/dew -> drainage. Dalton flux h_m(rho_sat(Ts) -
  rho_air), h_m = h_c/(rho cp) (Lewis, Le 1 [AS]); one VapourExchange (surface_vapour_exchange) feeds both the latent flux L_v*E on the
  surface node and the mass update; dew cap 1e-4 kg/m2/s, none at or below 0 C (frost W8); snow precipitation adds no liquid (W8).
  Sealed drainage: Gallaway WFD (FHWA-RD-79-31, intensity capped at 150 mm/h [AS]), tau = WFD/R, backward Euler only above
  W_tex = 0.5 MTD (steady film = WFD exactly). Soil/vegetated: gravity drain above theta_fc (tau 3 d [AS]), pond drain 1 h [AS],
  start at theta_fc. ConditionGrip (condition_grip.h/.cpp, built once per grid, update_state per tick, no alloc): wet weight
  (sealed W/W_tex; soil from theta, ponded ramp over 1 mm [AS]; grass max(canopy fraction, wet)), PIARC IFI speed table for
  sealed classes (Sp = 14.2 + 89.7 MTD km/h, v_ref 60 km/h, 33 samples, linear, clamped to damp; soil flat), factor =
  (1-w)*dry + w*wet(v) (vegetated: wet_grass) times the tyre profile's per-condition factors; it multiplies the SurfaceTable
  lambda_mu in WheelInstance::pre_solve at the contact-patch speed sqrt(vx^2+vy^2) for a valid contact SurfaceId -
  Environment::condition_grip() is nullptr without surface conditions (VehicleTickEnv::grip), so such a World is bit-identical.
  physics_sim.tyre_profile/1 (ps::io::load_tyre_profile_json, data/tyre_profiles/*.json: slick, summer_generic (environment
  default), all_season, winter, offroad_at, agricultural_lug - all [AS], docs/data_formats/tyre_profile.md); scenario environment
  key default_tyre_profile; wheels[i].tyre_profile is parsed only (per-wheel resolution W7). Snapshot (unhashed) water_mm/
  soil_moisture/canopy_water_mm/water_above_texture_mm/condition/condition_factor; adapter get_surface_water_mm/
  get_surface_condition + HUD line. Tests test_environment_surface.cpp W6-G1/G1b/G2/G8/G9/G7/G5/G6b (ps_environment_tests),
  test_environment_surface_world.cpp W6-G3 (car_sedan wet/dry stop ratio 1.402 from 20 m/s, 1.861 from 30 m/s, inside the
  Wong/PIARC bands)/G4b/G10, zero alloc, 1 vs 10 workers (ps_unit_tests); every one with a recorded sabotage. Notes
  out/weather_w6_progress.md.
- `ps_audio` (STATIC, `audio/`, ps_audio A7): the live engine voice; links
  `ps_engine_cycle` only, never `ps_core`. Guard `ps_audio_standalone`
  (`tests/unit/audio_standalone_main.cpp`, plain `main`, WHOLE_ARCHIVE
  ps_audio + ps_engine_cycle + ps_detmath without ps_core, labels
  `audio;cycle` - a JobSystem reference in ps_audio fails its link).
  `bench_audio_voice` (`tests/bench`, plain `main`, NOT CTest-registered):
  per-sample cost split cylinders/FV/WG of an audio-grade I4.
- `ps_engine_tools` (STATIC, `engine_tools/`, engine garage P6): the audio bank
  baker as a library (`prepare_engine_audio_bank`, see the layout entry); links
  `ps_core` + `ps_audio`; neither of them links it.
- `ps_headless_env` (STATIC, `tools/common`): Windows-only headless CRT
  setup (`ps_tools::install_headless_crt_handlers()`); linked by every
  tool/test executable (`ps_run`, `sweep`, `bench_force_elements`,
  `ps_test_catch_main`), never by `ps_core`.
- `ps_test_catch_main` (STATIC, `tests/support`): custom Catch2 entry point
  linking `Catch2::Catch2` + `ps_headless_env`; linked by every Catch2 test
  executable instead of `Catch2::Catch2WithMain`.
- `ps_run` (executable): `tools/ps_run/main.cpp`. Links `ps_core`,
  `ps_headless_env`. CLI: `ps_run <scenario.json> [output.csv] [--workers N] [--map-workers N] [--engine-cache DIR]`
  (`--map-workers`: worker count for load-time engine-map generation, I7c C16a; `--engine-cache`: .psmaps cache directory, I7c C16b).
- `ps_engine_audio` (executable, ps_audio A10): `tools/ps_engine_audio`; links
  `ps_core`, `ps_audio`, `ps_headless_env`, miniaudio (tool only). Uses the
  public `ps::EngineRig` (`ps/drivetrain/engine_rig.h`: a wheel-less crank +
  Powertrain with starter/throttle/ignition/optional dyno, 240/960 Hz, publishes
  EngineSoundState via `drivetrain::fill_engine_sound_state`, shared with
  VehicleInstance) and `ps::io::load_engine_json`. Demo engine
  `data/engines/demo_i4_pipes.json` ([AS] generic I4 with pipes; default audio dx,
  32/17 mm - A8b fixed the sustained-WOT blow-up).
- `sweep` (executable): `tools/sweep/main.cpp`. Links only
  `ps_headless_env` (no `ps_core` - it launches `ps_run` as child
  processes, platform process APIs only). CLI:
  `sweep <scenario.json>... [--jobs N] [--ps-run path]`.
- `ps_unit_tests`, `ps_validation_tests`, `ps_determinism_tests`
  (executables): Catch2 v3 via `ps_test_catch_main`; link `ps_core`.
  Registered with CTest via `catch_discover_tests`. `ps_unit_tests` and
  `ps_validation_tests` additionally get `core/src` on their include path
  (`target_include_directories ... PRIVATE`) for the white-box tests that
  `#include "articulation/..."` directly (internal, not part of `ps_core`'s
  public API) - `test_articulation_core.cpp`, `test_articulation_four_bar.cpp`,
  `test_articulation_four_bar_energy.cpp`, `test_articulation_parallel_alloc.cpp`,
  `test_aba_fast_transform.cpp`, `test_terrain_tile_manager.cpp`, `test_terrain_suppression_precompute.cpp` (`ps::terrain::TileManager` is internal).
- `bench_force_elements` (executable, `tests/bench`): plain `main()`
  (not Catch2, so its measured numbers are always on stdout); links
  `ps_core`, `ps_headless_env`. Registered with CTest via `add_test()`.
  Asserts its >= 6x scaling on every non-Debug build type (explicit
  `PS_BENCH_ASSERT_TIMING=0/1` compile definition from
  `tests/bench/CMakeLists.txt`, which also gives exactly those instances
  the `walltime` label + `PROCESSORS 24` - see "CI").
- `bench_contact_coupling` (executable, `tests/bench`): plain `main()`;
  links `ps_core`, `ps_headless_env`. Registered with CTest via
  `add_test()` as a smoke check only (no scaling assertion, unlike
  `bench_force_elements`) - PLAN.md P1.5b measurement (f), cost per tick
  per contact strategy over 1/2/4/8/16 independent contact-bearing rigs.
- `bench_terrain_tiles` (executable, `tests/bench`): plain `main()`; links
  `ps_core`, `ps_headless_env`. Registered with CTest via `add_test()` as a
  smoke check (250 ms/tick bound, deliberately loose - see the file's own
  comment for measured numbers) against `data/tracks/test_ground/terrain.json`
  loaded at `PS_SOURCE_DIR`, simulating a 50 km drive by advancing one
  interest point 250 m/tick for 200 ticks. Like `bench_force_elements`,
  a wall-clock measurement: wherever it asserts timing it carries the
  `walltime` label + `PROCESSORS 24` and runs in `tools/ci.ps1`'s serial
  phase B, never alongside other load - contention alone once pushed its
  worst tick well past the bound under `-Full`'s WSL legs (338-530 ms
  measured vs. 55-90 ms run alone right after). Sanitizer policy: on a
  `PS_SANITIZER_BUILD` it still runs in full but only REPORTS its timing
  bounds (250 ms drive / 12000 ms teleport worst tick); the no-starved-tile
  checks are asserted everywhere, the timing bounds on every non-sanitizer
  build. The value reaches the source as an always-defined
  `PS_SANITIZER_BUILD=0/1` compile definition (`#error` if missing).
- `ps_energy_study` (executable, `tests/bench/energy_study.cpp`): plain
  `main()`; links `ps_core`, `ps_headless_env`. Deliberately NOT
  registered with CTest - a measurement tool (Jolt solver-settings sweep
  over hinged-capsule/3-link-chain release angles, plus candidate-cause
  isolation runs) whose CSV output feeds a written report, run by hand on
  the `release` preset.
- `ps_warnings` (INTERFACE): warnings-as-errors, linked into every one of
  the above - never into third-party targets.
- `ps_options` (INTERFACE): `/arch:AVX2` / `-mavx2 -mfma`, `/fp:precise`,
  `/EHsc` (MSVC family); same targets only.
- `adapters/godot` (P2, not yet built): GDExtension adapter, gated by
  `PS_BUILD_GODOT_ADAPTER` (default `OFF`).

## CMake options

- `PS_BUILD_TESTS` (default `ON`): build `ps_unit_tests`,
  `ps_validation_tests`, `ps_determinism_tests`, `bench_force_elements`,
  `bench_contact_coupling`, `bench_terrain_tiles` and fetch Catch2.
- `PS_BUILD_GODOT_ADAPTER` (default `OFF`): fetch godot-cpp and build
  `adapters/godot`.
- `PS_TARGET_X86` / `PS_TARGET_AARCH64` (derived after project(), not user options): target from CMAKE_CXX_COMPILER_ARCHITECTURE_ID, CMAKE_CXX_COMPILER_TARGET, `--target=` in CMAKE_CXX_FLAGS, then CMAKE_SYSTEM_PROCESSOR (empty = x86). `/arch:AVX2`, `-mavx2 -mfma` and Jolt's USE_AVX2/USE_FMADD/USE_AVX512 apply only on x86 (x86 compile lines byte-identical to before, checked over all 563 compile_commands entries for release/msvc-release/linux-release); on aarch64 every Jolt x86 option is forced OFF (Jolt picks NEON itself), `-ffp-contract=off` stays (clang contracts by default on aarch64), ps::simd takes its scalar path (no NEON backend), step_team spins with `yield`, topology.cpp returns the logical core count on ARM (no 'core id' in /proc/cpuinfo).
- `PS_SANITIZER_BUILD` (default `OFF`): set `ON` explicitly by the `asan`,
  `linux-asan-ubsan` and `linux-tsan` presets; wall-clock bench bounds are
  reported, not asserted, on such a build (see `bench_terrain_tiles`
  above). Configure warns if `CMAKE_CXX_FLAGS` contains `-fsanitize=`
  while it is `OFF`.

## Dependencies (FetchContent, pinned)

| Dependency   | Pin      | Notes |
|--------------|----------|-------|
| JoltPhysics  | `v5.6.0` | `DOUBLE_PRECISION ON`, `CROSS_PLATFORM_DETERMINISTIC ON`, `USE_AVX2 ON`, `USE_FMADD OFF`, GPU-compute options OFF, sample/test/viewer targets OFF |
| Catch2       | `v3.16.0`| only when `PS_BUILD_TESTS=ON`; its two targets get `/EHsc` PRIVATE under MSVC-family compilers (else clang-cl builds its exception translator as an aborting stub - REQUIRE_THROWS_WITH/_MATCHES would abort), plus `-mllvm -asan-stack=0` under clang-cl ASan only (clang-cl 19.1.5 ASan stack instrumentation corrupts an optimised function's catch parameter; see "Windows ASan caveat" below) |
| nlohmann/json| `v3.12.0`| linked `PRIVATE` into `ps_core` (JSON parsing, confined to `core/src/io/*.cpp` and `core/src/terrain/procedural_terrain_source.cpp`); also linked into `ps_unit_tests` for `test_terrain_golden.cpp`'s direct golden-file parse |
| godot-cpp    | branch `4.5` | only when `PS_BUILD_GODOT_ADAPTER=ON` |

**Windows ASan caveat (clang-cl 19.1.5, vault TOOL-019):** in the `asan` preset
(RelWithDebInfo + `-fsanitize=address`), any function that `catch`es an
exception by reference or value and then touches it (`e.what()`) reads
garbage and crashes (access-violation at `0xffffffffffffffff`) - ASan's
stack instrumentation moves the catch parameter away from the slot MSVC's
EH runtime writes. Catching only by type (`CHECK_THROWS_AS`, `catch (...)`)
is fine. Catch2 is built with `-asan-stack=0` to avoid it (above). Our own
`catch (const json::parse_error& e)` sites in `core/src/io/*.cpp`,
`procedural_terrain_source.cpp` and `ps_run`'s top-level handler still hit it,
so an error-path test for those would crash in this leg only (exception: core/src/io/scenario.cpp's environment probe catch lives in its own PS_NO_ASAN helper, `__attribute__((no_sanitize("address")))` under clang, so the scenario environment block's rejections are tested on the asan leg); Linux ASan
(Itanium EH) is unaffected.

## CMake presets

Windows (Ninja; run from a VS 2022 Build Tools developer environment -
`tools/ci.ps1` enters it automatically via `vcvars64.bat`):

| Preset | Compiler | Build type |
|---|---|---|
| `debug` | clang-cl | Debug |
| `release` | clang-cl | Release |
| `relwithdebinfo` | clang-cl | RelWithDebInfo |
| `asan` | clang-cl | RelWithDebInfo + AddressSanitizer |
| `msvc-debug` | cl | Debug |
| `msvc-release` | cl | Release |

Linux/WSL (Ninja, clang + lld; binary dir `~/build/physics_sim/<preset>`,
never under `/mnt/s`):

| Preset | Build type |
|---|---|
| `linux-release` | Release |
| `linux-asan-ubsan` | Debug + AddressSanitizer + UndefinedBehaviorSanitizer |
| `linux-aarch64-check` | Release cross build for aarch64 (toolchain file above, PS_BUILD_TESTS OFF): ps_core, ps_engine_cycle, ps_audio, ps_environment(_io), ps_engine_tools, ps_run; run under `qemu-aarch64 -L /usr/aarch64-linux-gnu` |
| `linux-tsan` | Debug + ThreadSanitizer |

## Job system (`core/src/jobs/`, PLAN.md D8)

`JobSystem` (`job_system.h`/`.cpp`): one injector queue plus one queue per
worker, each a fixed-capacity ring buffer (`JobSystem::RingQueue`, private
nested class) sized once at construction (`kQueueCapacity` = 4096) rather
than a `std::deque` (PHYS-009 Step 3: a `std::deque<JobEntry>` allocated a
new internal node on Linux/libstdc++ every time a worker's own queue's
`push_back` crossed a block boundary a thief's `pop_front` had just
vacated - reproducible with a no-op nested `parallel_for` alone, Windows
unaffected). `submit()`'s `push_back` returns `false` on a full queue; the
caller then runs the job inline/synchronously on the submitting thread
(`JobSystem::run_job(entry, /*counts_toward_pending=*/false)`) instead of
aborting or falling back to heap allocation. Counted via
`JobSystem::queue_full_fallback_count()`, exposed as a `WorldConfig`-driven
telemetry channel `jobs.queue_full_fallbacks` (`World`'s constructor/
`step()`). `JobSystem`/`WorldConfig` also carry an explicit
`queue_capacity`/`job_queue_capacity` (default `0` = auto = 4096) so a test
can force the tiny-queue path deliberately. `deques_` is
`std::vector<std::unique_ptr<WorkerQueue>>` (`WorkerQueue` holds a
`std::mutex`, so it cannot live directly in a plain growable `std::vector`).
`JobSystemOptions{queue_capacity, background_priority}` + `JobSystem(int, options)` (engine garage P1;
the `(int, queue_capacity)` constructor is unchanged): with `background_priority` every pool and
background-lane thread lowers its own OS priority (Windows `THREAD_PRIORITY_BELOW_NORMAL`, Linux nice +5
per thread via `setpriority` on the tid; a failing call is ignored) - for load-time map generation that
must not starve a renderer; never changes results.
`debug_alloc.cpp` additionally supports an opt-in backtrace dump on Linux
(`PS_DEBUG_ALLOC_BACKTRACE=<n>` env var, `execinfo.h`) and on Windows (same
env var, `dbghelp.h` - symbolized when a PDB is present, e.g. a debug
build) for the first `n` violations - a root-cause diagnostic, not part of
`debug_alloc.h`'s public contract; used during PHYS-033 I1b to find the
`BroadPhaseQuadTree::UpdatePrepare` site below.

**PHYS-033 I1b (D-D): `debug_alloc` now also sees Jolt's own allocator.**
`note_external_allocation()`/`external_violation_count()` (`ps/jobs/
debug_alloc.h`) are the same armed/in-job/re-entrancy-guarded path as the
existing global `operator new`/`delete` overrides, for a third-party
allocator routed AROUND them entirely - `jolt_backend.cpp`'s
`JoltGlobalRegistration` wires Jolt's own `JPH::Allocate`/`Reallocate`/
`AlignedAllocate` through it (see "Jolt backend" below). Both counters
share `violation_count()`'s reset; the external one is separate only for
attribution. This is why `test_vehicle_parallel_alloc.cpp`'s existing
zero-alloc case now also catches `JPH::QuadTree::UpdatePrepare`'s own
periodic `new NodeID[mNumBodies]` (Jolt's internal broadphase-tree
rebuild) - a pre-existing D8 gap since P1, invisible until this hook
existed, `[!mayfail]`-documented rather than fixed (out of I1b's scope).

## World tick pipeline (`ps::World::step`, `core/src/world/world.cpp`)

Built once (constructor) as a `jobs::TaskGraph` of barrier stages, re-run
every tick:

```
freeze_tick_start_state (stub) -> scene_queries (stub) -> vehicle_islands_substep (articulation stepping, PLAN.md D10/P1.5a)
  -> force_elements (parallel_for over IForceElement::evaluate, scatter into per-element wrench slots)
  -> gather (sequential, fixed element-id order: backend->apply_wrench, or articulation::System::accumulate_external_wrench for an articulation-link Frame)
  -> solver_seam (named hook, World::set_between_forces_and_backend_step_hook; no-op in P1)
  -> contact_coupling_pre (PLAN.md P1.5b: ProxyBody pose/velocity-syncs its proxy Jolt bodies; no-op for Compliant/None)
  -> backend_step (IRigidBackend::step, i.e. JoltPhysics PhysicsSystem::Update via the Jolt job-system bridge)
  -> contact_coupling_post (PLAN.md P1.5b: ProxyBody reads back each proxy's post-step velocity change and accumulates the resulting wrench for next tick; no-op for Compliant/None)
  -> terrain_tiles (PLAN.md M1: TileManager::update, no-op until World::set_terrain_source is called) -> snapshot (publishes to SnapshotTripleBuffer, records one telemetry row)
```

`vehicle_islands_substep` (`World::stage_vehicle_islands`) calls
`articulation::System::step_all`, which sub-steps every live articulation
instance at `WorldConfig::substep_rate_hz` (default 960 Hz) inside the
240 Hz tick, using the external wrenches gathered during the *previous*
tick's `gather` stage (held constant across all of a tick's substeps,
PLAN.md D3), then clears them so this tick's `force_elements`/`gather`
can accumulate fresh ones for next tick. A no-op (near-zero cost) when
`World` has no articulations. `System::step_all` (PLAN.md D8/P1.5a,
PHYS-009 Step 1) runs one job per articulation via `ps::jobs::parallel_for`
(batched by `WorldConfig::articulation_grain`, default auto - same formula
as `force_element_grain`); instances are already in `ArticulationId` order
so no separate ordered-gather step is needed (each job touches only its
own `Instance`'s state, PLAN.md D8's scatter/compute pattern). The same
stage then calls `vehicle::System::step_all` (PLAN.md 5.1/P3, see "Vehicle
system" below) right after the articulation pass, with the current
`World::set_control`/`get_control` channel values (one namespace per vehicle,
see "Adapter seams" below) - also a no-op when `World` has no vehicles.

Each stage records its own wall-clock duration to a
`stage.<name>.ms` telemetry channel; `World::step` also records one
`jobs.worker<N>.utilization` channel per worker (busy time / tick wall
time), and one set of seven per-wheel channels (`vehicle<i>.<wheel
name>.{slip_ratio,slip_angle,fx,fy,fz,surface,omega}` - `omega` added
PHYS-033 P4 I2 commit 5b, the drivetrain hub's own angular velocity) per
wheel of every vehicle created via `World::create_vehicle`, plus (also
commit 5b) one set of eleven per-vehicle channels right after that
vehicle's chassis channels: `vehicle<i>.energy.{engine_out,wheel_out,
tyre_loss,brake_loss,gear_loss,integrator_loss,stored,balance_error}` (the
`drivetrain::PowertrainEnergy` ledger, `stored` = `rotor_ke`) and
`vehicle<i>.powertrain.{friction_cap_hits,nonspd_hits,lcp_stage3_hits}`,
followed by seven CSV-only powertrain state channels (2026-09-25 wheelspin
diagnosis) `vehicle<i>.powertrain.{gear,engine_rpm,clutch_engagement,
clutch_slip,clutch_torque,shift_phase,engine_state}` (plain reads of the
tick's `PowertrainSnapshot`/`PowertrainDebug`, enums as their integer
value, never hashed), plus (PHYS-033 P4 I4 C8) one `vehicle<i>.<diff id>.{bias,locked}`
pair per differential that is not plain non-lockable `open` (component order; none for
any shipped vehicle, so their CSV headers are unchanged), plus (PHYS-033 P4 I7a C5)
`vehicle<i>.fuel.{flow_g_s,energy_in,heat}` for every fuelled engine (C5d: all shipped ones), plus (I7a C6) `vehicle<i>.fuel.tank_kg` only for a vehicle with a fuel_tank (none shipped), plus (I7c C16b) `vehicle<i>.engine.maps_hash_lo32` only for a vehicle with a simulated engine (CSV-only, unhashed) - all registered up front in `create_vehicle` itself (PLAN.md P3 scope item
7) so the CSV header is complete before the first `record_row`.
`World::state_hash()` folds `IRigidBackend::compute_state_hash` (FNV-1a
over every live body's pose+velocity bytes, in ascending `BodyId` order),
then `articulation::System::fold_state_hash` (same FNV-1a fold, every live
articulation's q/qd and, if floating, root pose), then
`vehicle::System::fold_state_hash` (same fold, every live vehicle's
per-wheel compression/relaxed-slip state plus, PHYS-033 P4 I2 commit 5b,
`drivetrain::Powertrain::fold_state_hash` - the shaft network's own state
(every node's omega, friction-row stick flags/LCP sets, topology state)
and each lossy gear's `lambda_mesh_prev_` - not otherwise observable from
the chassis body alone) - PLAN.md D7; a `World` with no
articulations/vehicles hashes bit-identically to before those fields
existed.

`stage_snapshot` additionally fills `WorldSnapshot::articulations` (every
live articulation's link poses/motions, root first then each public link -
`ps::articulation::Instance::link_count()`/`System::count()`) and
`WorldSnapshot::contacts` (a copy of `last_contact_events_` - see "Adapter
seams" below for why this must be a copy, not a reference, for a
cross-thread reader). Both reuse each `WorldSnapshot` triple-buffer slot's
own existing capacity across ticks (`resize()`/`clear()`+`push_back()`,
never rebuilt from scratch) to stay allocation-free in the job this stage
runs in (PLAN.md D8) once warmed up - `tests/unit/test_articulation_parallel_alloc.cpp`
checks this for a full pipeline including articulations/contacts.

## Articulation solver (`core/src/articulation/`, PLAN.md D10/P1.5a)

Reduced-coordinate (Featherstone) solver for articulated chains, replacing
Jolt joints wherever energy conservation and mass-ratio robustness matter
(D10). Entirely internal except the descriptors in
`ps/articulation/articulation_desc.h` and the `World`/`Frame` seams below;
no Jolt type anywhere in it, and articulations do not collide with
anything or touch Jolt at all (P1.5a scope - coupling to Jolt contacts is
P1.5b).

- `spatial.h` - 6D spatial vector algebra in `ps::real` (motion/force
  vectors, Plucker transforms, spatial rigid-body inertia), Featherstone
  ch. 2. Header-only.
- `small_matrix.h` - dense matrix up to 6x6 (Gauss-Jordan inverse), used
  for a joint's `D` block in ABA (at most 6x6 because only a floating base
  needs more than 1x1).
- `model.h`/`.cpp` - builds a flat `Model` (`BodyModel` array; body 0 is
  the floating root if `ArticulationDesc::root.floating`, else
  `ArticulationDesc::links[i]` is body `i` directly) from a public
  `ArticulationDesc`; `ArticulationState` (`q`/`qd`/`root_pose`);
  `compute_xup`, `motion_subspace`, `joint_velocity`.
- `rnea.h`/`.cpp` - Recursive Newton-Euler (Featherstone ch. 3): inverse
  dynamics; also used (via `qd=0`, `qdd=e_j`, no gravity) to build mass-
  matrix columns and (via `qdd=0`) bias forces for cross-checks.
- `crba.h`/`.cpp` - Composite Rigid Body Algorithm (ch. 3): the joint-space
  mass matrix `M(q)`; reference/test-only, not on ABA's hot path.
- `aba.h`/`.cpp` - Articulated-Body Algorithm (ch. 7): O(n) forward
  dynamics, the production per-substep call. Pass 2's per-body inertia
  congruence transform up into the parent has two implementations
  (PHYS-009 Step 2): `transform_inertia_to_parent_fast` (production,
  `aba_into`/`aba`) computes the congruence directly from the Plucker
  transform's `(E, r)` block structure via `Mat3` operations instead of
  building two dense 6x6 matrices and multiplying them;
  `transform_inertia_to_parent_reference` is the original dense-`Mat6`
  form, kept byte-identical as a test-only reference (`aba_reference()`,
  `tests/unit/test_aba_fast_transform.cpp` cross-checks both to 1e-12
  relative on random trees). `AbaScratch` (per-`Instance`, preallocated at
  construction - PHYS-009 Step 1) holds every `aba()` call's n-sized
  scratch, including a reusable `AbaInput` (`ain`), so the open-chain
  hot path (`Instance::step_tick` via `*_into` helpers) allocates nothing;
  a loop-closure-bearing instance (e.g. a four-bar linkage) also allocates
  nothing as of PHYS-009 Step 3 (see `loop_closure.h` below).
- `loop_closure.h`/`.cpp` - closed-loop constraints (ch. 8):
  acceleration-level Lagrange-multiplier solve via the Schur complement,
  reusing ABA as the `M^-1` operator, plus mass-weighted position/velocity
  drift-stabilisation projection (run every substep). Only
  `LoopClosureType::Point` is implemented; `Hinge` is parsed but not yet
  given its axis-alignment rows. `ClosureLinearAlgebra` (J/minv_jrow/A,
  flattened row-major) and every other workspace buffer this file's solve
  needs live in `LoopClosureScratch`, owned per-`Instance`
  (`Instance::loop_closure_scratch_`) and preallocated at construction
  (`LoopClosureScratch::ensure`, PHYS-009 Step 3) - `solve_constrained_dynamics_into`/
  `project_velocity_onto_constraints`/`project_position_onto_constraints`
  take it by pointer/reference; passing `nullptr` (their default) falls
  back to a fresh stack-local scratch for non-hot-path callers (the
  `AbaOutput`-returning `solve_constrained_dynamics` compatibility
  wrapper, and the white-box tests that call these functions directly).
- `instance.h`/`.cpp` - one runtime articulation: state, direct/actuator
  joint forces, external-wrench accumulation, and the integrator. Default
  integrator is classical RK4 in joint coordinates (chosen by measurement
  over semi-implicit Euler - see the file's header comment and
  `tests/bench/energy_study.cpp`'s table); floating-base orientation on
  the quaternion manifold (exponential map for Euler, renormalised linear
  rate per RK4 stage).
- `system.h`/`.cpp` - owns every `World` articulation instance; implements
  `ps::ILinkStateSource` (`ps/world/frame.h`) so a `Frame` can be anchored
  to a link without `frame.h` depending on anything in this internal
  directory.

`World` additions (additive, `ps/world/world.h`): `create_articulation`,
`joint_position`/`joint_velocity`, `set_joint_force` (direct actuation),
`link_frame` (a `Frame` anchored to a link, `link_index == -1` == the
root), `root_pose`/`root_motion`. `Frame` additions (additive,
`ps/world/frame.h`): a constructor taking an `ILinkStateSource` and
`ArticulationLinkRef` instead of a `BodyId`; `is_articulation_link()`/
`articulation_link()`.

### Contact coupling (`core/src/articulation/contact.h`/`.cpp`, PLAN.md P1.5b)

How an articulation's contact-enabled links (`LinkDesc::contact`,
`ps/articulation/articulation_desc.h`) interact with Jolt geometry, selected
per articulation via `ArticulationDesc::contact_strategy`
(`ContactStrategy::None` default - zero behaviour change - `Compliant`, or
`ProxyBody`). `ContactCoupling` (one per contact-enabled articulation,
owned by `World::articulation_contacts_`, constructed in
`World::create_articulation`) implements both:

- `Compliant`: `IContactSubstepSource` (`contact_source.h`), wired via
  `Instance::set_contact_source` and called from inside
  `Instance::step_rk4`/`step_semi_implicit_euler`
  (`evaluate_stage_ext_force_into`) at EVERY RK4 stage (up to 4x per 960 Hz
  substep, not once per substep) - a stiff Hunt-Crossley force sampled and
  frozen across a whole substep's stages can still manufacture energy at a
  fast impact even though the substep rate itself is fine (see
  `contact_source.h`'s header comment). Queries geometry via
  `IRigidBackend::collide_shape_into` (pure narrowphase overlap, no side
  effects); force law (Hunt-Crossley normal + regularised Coulomb friction)
  and the reaction wrench applied directly to the other body (only from the
  substep-start stage - `apply_reaction_to_other`, never from k2/k3/k4, or
  the reaction would be over-counted) live in `contact.cpp`. Instance
  kinematics for an arbitrary (e.g. RK4-offset) state are computed by
  `Instance::compute_world_kinematics_into`, temporarily swapped into
  `body_world_pose_`/`body_world_motion_` for the source's call and swapped
  back immediately after.
- `ProxyBody`: one Jolt dynamic body per contact-enabled link, pose/
  velocity-synced from the link every tick (`ContactCoupling::pre_backend_step`,
  `World`'s `contact_coupling_pre` stage) with real gravity enabled (needed
  for Jolt's velocity-level contact solver to have anything to react to at
  rest, not just the much weaker penetration/baumgarte position corrector);
  `post_backend_step` (`contact_coupling_post` stage) reads back the
  proxy's velocity change, subtracts the proxy's own known gravity
  contribution back out, and accumulates the remainder as the link's
  external wrench for next tick via `System::accumulate_external_wrench`.
  The proxy's mass is `link.mass * ContactMaterial::proxy_mass_scale` - for
  a contact-bearing link rigidly carrying much more mass behind it (e.g. a
  floating-base chassis behind a Fixed-jointed leg), `proxy_mass_scale`
  must reflect the full carried mass or the proxy's contact impulse is
  sized for the link alone and is far too weak.

`System::step_all` (`system.cpp`) steps any contact-source-bearing instance
(`Compliant` only) SEQUENTIALLY FIRST, in fixed creation order, before the
ordinary `parallel_for` over everything else - `apply_wrench` must never be
called concurrently for the same body, which two different `Compliant`
articulations both touching the same dynamic body could otherwise race on.

## Terrain tile system (`core/src/terrain/`, PLAN.md M1/section 6)

Collision shapes (`ps/backend/shape_desc.h`): `MeshShape` (vertices,
`uint32` triangle indices, one `SurfaceId` per triangle or empty for
`kInvalidSurfaceId` everywhere - static geometry only, `JPH::MeshShape::
MustBeStatic()`) and `HeightfieldShape` (square `sample_count x
sample_count` grid, `sample_count` must be even - Jolt's own
`HeightFieldShape` rounds up to a multiple of its block size otherwise -
row-major `heights`/`cell_surface_ids`, body-local Z-up). Both propagate
their `SurfaceId`s through every collision query/event
(`ShapeCastHit`/`ShapeContactHit`/`ContactEvent`'s `surface`/`surface_a`/
`surface_b` fields, `ps/backend/i_rigid_backend.h`). Jolt-side
implementation lives in `jolt_backend.cpp` (see "Jolt backend" below for
the Y-up/Z-up axis fix-up and the shared material list this needs).

`ITerrainSource` (`ps/terrain/terrain_source.h`): `TileKey` (opaque
`int64_t`, PLAN.md section 6 - never a raw planar index, so a future
geodetic/cube-sphere source, P11, can reuse the same interface),
`TileSample` (heights + cell_surface_ids + `samples_per_tile` + tile
origin), `tile_size_m()`/`samples_per_tile()`/`key_for_world_xy()`/
`key_offset()`/`tile_origin_xy()`/`fill_tile()`. `samples_per_tile()` is
fixed for a source's whole lifetime and is the ONLY source of truth for
grid resolution - `TerrainConfig` (`ps/terrain/terrain_config.h`,
`TileManager`'s pool-sizing knobs: `max_resident_tiles`,
`max_tile_fills_per_tick`) deliberately does NOT carry its own resolution
field, so a manager-side value can never disagree with the source's own
(found the hard way: an earlier version had both, independently settable,
and a mismatch overran a tile's sample buffer - see the knowledge vault
for the write-up).

`ProceduralTerrainSource` (internal, `core/src/terrain/
procedural_terrain_source.h/.cpp`; public factory `ps::terrain::
load_procedural_terrain_source(path, samples_per_tile = 64)`,
`ps/terrain/procedural_terrain.h`): a line-for-line port of
`tools/trackgen/trackgen/terrain.py` (read-only reference, owned by a
separate session) - integer-hash value noise, disc/corridor flat zones,
per-cell surface rules from a `terrain.json` spec. No platform-libm
transcendental anywhere except two FIXED corridor headings resolved once
at spec-load time via `ps::math::sincos` (PLAN.md D7).
`tests/unit/test_terrain_golden.cpp` matches all of
`data/tracks/test_ground/terrain_golden.json`'s points within 2e-4 m
(dominated by the golden file's own independent coordinate rounding, not
a formula tolerance - see that test's header comment).

`TileManager` (internal, `core/src/terrain/tile_manager.h/.cpp`, owned by
`World` via `terrain_manager_`): a fixed pool of `TerrainConfig::
max_resident_tiles` heightfield bodies, all created up front
(constructor, not lazily) so first-touch Jolt allocation never happens
inside a job. Single fixed resolution, no LOD (PLAN.md explicitly allows
this - "keep one resolution and say so"); adjacent tiles' shared border
samples are the literal same world-coordinate evaluation, so
`test_terrain_tile_manager.cpp`'s seam test checks EXACT equality, not a
tolerance. `update()` per tick: compute needed keys for every interest
point -> queue newly-needed keys -> process up to
`max_tile_fills_per_tick` (bounded per-tick cost; a burst, e.g. a 50 km
teleport, drains over several ticks) -> `jobs.parallel_batches` fills
each newly-assigned slot's `TileSample` (allocation-free,
`ITerrainSource::fill_tile` writes into the slot's own preallocated
buffers) -> sequential `IRigidBackend::update_heightfield_body` +
`set_pose` per changed slot. `starved_tile_count()` reports (never
silently drops) tiles needed but with no free/reusable slot. Slot
(re)assignment uses a sorted vector reserved to the pool size; row
crossings with prefetch enabled are checked allocation-free. The lazy
suppression-cache insertion remains an exception for a synchronous fill
outside the complete load-time precompute.

Terrain prefetch is opt-in: both `TerrainConfig::prefetch_margin_tiles`
and `prefetch_max_tiles` must be nonzero. `ITerrainSource` defaults to
`PrefetchSupportKind::TickOnly`; procedural sources declare `Immutable`.
A `Snapshot` source captures immutable inputs on the tick thread via
`acquire_prefetch_inputs`, fills from those inputs on the background
lane via `fill_tile_from`, and validates them at install with
`prefetch_inputs_current`. Releasing inputs in a tick must not free
snapshot storage; the source retains it for an owner-thread purge.

The JobSystem owns a separate background lane. Fixed tickets and
detached heightfield spares move generation and upload off the tick.
Installation keeps the existing tick, pool slot and BodyId: a ready
tile swaps its spare into the body; a queued build is cancelled and
filled synchronously; a running build is waited for. Stale snapshots
also fall back to synchronous filling. Suppression inputs must be
cached or provably empty before enqueue. `World::terrain_prefetch_stats()`
exposes lifetime install, cancellation, stale-input and late counters,
including wait duration. These counters do not affect state hashes.

`bench_terrain_tiles --crossings-only` measures six row crossings at
256×256 samples, one fill/tick and 900 km/h; the enabled run is paced
to 240 Hz. On Windows release (2026-10-01), terrain-stage p99/max was
8.094 ms off and 0.031 ms on across 72 crossing-window ticks, with
42 background installs, no driven late builds and no starvation.
This is a procedural-source benchmark, not a geo2map integration result;
startup and slower background scheduling can still cause late waits.

Tile seams: adjacent tiles are separate Jolt bodies whose border edges Jolt always treats as active; `jolt_backend.cpp`'s cross-tile edge weld (see "Jolt backend") removes the resulting ghost-edge contact spike (`tests/validation/test_terrain_seam_crossing.cpp`).

Ordering rule (PLAN.md D7, vault PHYS-026, stated on `tile_manager.h`'s
container block): nothing that decides simulation state, pool-slot
assignment or backend-call order may ITERATE an unordered container -
MSVC's and libstdc++'s iteration orders differ, and a different fill order
hands the same tile to a different pool `BodyId` (moves `state_hash` and
Jolt's contact-solve order). `interest_points_` is therefore a `std::map`
keyed by interest-point id (updating an existing id does not allocate);
`key_to_slot_` is a sorted vector; `own_pool_body_ids_` and
`suppression_cache_` stay `unordered_*`
because they are lookup-only; the precompute's tile-key set
is sorted by `TileKey::packed` before use.

Per-tile vertical datum (R1, real-world terrain 90-880 m ASL fed through
`ITerrainSource`): `ITerrainSource::fill_tile` always produces ABSOLUTE
heights; `TileManager::update` (step 5, after suppression) subtracts each
tile's own mid-height snapped to the nearest 128 m before
`update_heightfield_body` and adds it back via the tile body's pose z, so
Jolt's `HeightFieldShape` fixed +-200 m quantisation range only ever has to
cover one tile's local relief. `relief_overflow_count()` (+ `World::
terrain_relief_overflow_count()`) counts (does not drop) a tile whose own
relief alone exceeds that range; `resident_tile_datum(TileKey)` is a
white-box test hook. See `ps/terrain/terrain_source.h`'s `TileSample`
comment for the full contract.

`World` wiring (additive, `ps/world/world.h`): `set_terrain_source`
(constructs the `TileManager`, then runs the suppression precompute below),
`set_terrain_interest_point`/`remove_terrain_interest_point`,
`terrain_resident_tile_count`/`terrain_starved_tile_count`/
`terrain_precomputed_tile_count`/`terrain_relief_overflow_count`; the `terrain_tiles` pipeline stage is a
no-op tick until `set_terrain_source` is called. `World::create_body`/
`destroy_body` (now out of line in `world.cpp`) call
`TileManager::on_static_geometry_changed` for a Static body once a terrain
source is set (drops every cached suppression result). `bench_terrain_tiles`
(`tests/bench/`) measures real cost against `data/tracks/test_ground`'s
terrain AND static collision (demo config: 512-tile pool, 16 fills/tick,
700 m radius), Windows `release`, 3 runs each (2026-09-23, after the
stale-cache fix - earlier figures were taken on an unoptimised build of
our own code): drive avg 1.88-1.93 ms/tick (worst 2.7-3.3 ms); cold
teleport onto split_mu worst tick 3.53-3.55 ms; `set_terrain_source`
225-237 ms. With `--no-precompute`: teleport worst tick 263-310 ms (the
lazy per-fill suppression cost lives almost entirely in Jolt's queries,
which were optimised all along), drive avg 2.20-2.29 ms (worst 3.9-4.8 ms),
`set_terrain_source` ~77 ms. Flags `--no-precompute` and `--workers N` are
for A/B measurement only.

### Terrain/static-feature coincident-collision suppression (PHYS-021)

A track's static feature meshes (`load_test_ground_collision`, below) sit
on `terrain.json`'s flat zones at height exactly 0, the same height the
terrain heightfield has there, so a wheel query would hit either body
depending on Jolt tie-breaking (surface/friction flicker). `TileManager`
therefore punches holes (`ps::kHeightfieldNoCollision`, `ps/backend/
shape_desc.h` - Jolt's own `cNoCollisionValue`) into a tile's samples
wherever foreign static geometry covers them, right before
`update_heightfield_body`. Gated by
`TerrainConfig::suppress_terrain_under_static_geometry` (default on).

Computation (`TileManager::compute_suppression_holes`, pure: tile sample +
per-caller `SuppressionScratch` -> ascending hole indices):
1. one `collide_shape_into` box over the whole tile footprint; no foreign
   static hit -> no holes;
2. 8x8-quad coarse blocks, one `collide_shape_into` each; only covered
   blocks refine;
3. 5-point rule per quad: its 4 corners (probed once per vertex, only if
   that vertex's height is exactly 0.0) AND its centre must all be covered;
   a vertex is holed only if EVERY quad touching it passed. This fails
   toward flicker (a covered quad kept), never toward a hole a body can
   fall through.
Point probe = `collide_shape_into` with a `{0.05, 0.05, 50}` half-extent
box centred at `(x, y, 0)` - NOT `ray_cast`, which misses the static
feature meshes at some points. Back-face caveat: Jolt's collide ignores
triangles whose plane lies beyond the query box's centre (default
`IgnoreBackFaces`), so every probe here sees only surfaces at or below
z=0. It detects the flat z=0 features only because the centre lies exactly
on their plane. Any geometry it misses (slightly raised features, ramp
decks overhead) is simply not treated as covering, which is the safe
direction.

"Foreign" = `backend.is_static(body) && !is_own_pool_body(body)`
(`own_pool_body_ids_`, every pool slot's `BodyId`, fixed at construction).
Unassigned pool slots are parked at `kUnassignedSlotParkingZ` (-1e6 m),
never at the world origin.

Caching and precompute: `suppression_cache_` maps `TileKey::packed` ->
hole list (a pure function of the key, static geometry being fixed).
`TileManager::precompute_suppression` (called by `set_terrain_source`,
gated by `TerrainConfig::precompute_suppression_at_load`, default on)
enumerates every tile touched by a foreign static body's actual triangles
(`IRigidBackend::body_triangle_xy_bounds_into`, 1 m binning margin - never
a body's AABB), computes each with the same `compute_suppression_holes`,
parallel across tiles (`jobs.parallel_for`, per-batch scratch), and fills
the cache in ascending key order (78 tiles for test_ground). A runtime fill
of such a tile is an allocation-free cache hit; any other tile takes the
lazy path in `suppress_covered_samples` (counted by
`lazy_suppression_count()`) and is cached. White-box hooks:
`precomputed_keys()`, `cached_suppression(key)`,
`compute_suppression_uncached(key)`; `tests/unit/
test_terrain_suppression_precompute.cpp` asserts precompute == lazy for
every tile and worker-count invariance. `tests/unit/
test_terrain_static_overlap.cpp` covers the straight edge and the skidpad
ring (curved edge).

## Test-ground static collision (`core/src/io/`, PLAN.md M1)

`ps::io::SurfaceTable` (`ps/io/surface_table.h`/`surface_table.cpp`):
loads a `surfaces.json`-shaped file (`data/surfaces/surfaces.json`: name
-> `{lambda_mu, crr}`) and assigns each name a `SurfaceId` by
ALPHABETICAL order (not raw file order - `nlohmann::json`'s default
object type does not preserve a file's own key order, and PLAN.md D7
needs this deterministic) - `asphalt=0, dirt=1, field=2, grass=3, kerb=4,
low_mu=5` for the current file.

`ps::io::load_test_ground_collision` (`ps/io/test_ground_loader.h`/
`test_ground_loader.cpp`): reads a track directory's
`test_ground_collision.json` + `.bin` (data-driven - only those two
filenames are hardcoded, `track_dir` is a parameter) and creates one
static `MeshShape` body per surface group. The .bin's `vertex_offset`/
`index_offset` are absolute BYTE offsets into the file (not vertex/index
counts) - confirmed by cross-checking that each group's positions+indices
block ends exactly where the next group's `vertex_offset` begins, not
merely from the JSON's own prose. Assumes a little-endian host (this
project's only build targets, Windows/WSL, are both x86_64).
`data/tracks/test_ground`'s own per-surface AABBs are much larger than
that surface's actual triangles (a real winding track, not a filled
rectangle) - do not pick test coordinates from the AABB alone; probe with
`IRigidBackend::shape_cast` first (`tests/unit/test_test_ground_loader.cpp`'s
header comment records the two points its own tests use and how they were
found).

## Vehicle system (`core/src/vehicle/`, PLAN.md 5.1/P3)

Per-corner suspension + tyre + steering, plus (PHYS-033 P4 I2 commit 5a)
`ps::drivetrain`'s shaft-network brakes/powertrain, on an existing chassis
rigid body. Entirely internal except the descriptors in
`ps/vehicle/{vehicle_desc.h,curve1d.h,tyre_desc.h,tyre_model.h}`, `ps/vehicle/
{wheel_telemetry.h,wheel_state.h}` and `ps/drivetrain/{powertrain_desc.h,
brake_system_desc.h,energy_ledger.h}`, plus the `World`/`ps/io/scenario.h`/
`ps/io/vehicle_io.h` seams below - mirrors `ps::articulation`'s D2/D10 public-descriptor-plus-
internal-implementation seam. No Jolt type appears in any public header.
`VehicleDesc::brake_system`/`::powertrain` replace the pre-I2 `BrakeDesc`/
`DriveDesc`/`WheelDesc::driven` (physics_sim.vehicle/2, `core/src/io/
vehicle_io.cpp`'s loader rejects `/1` and the old `driven`/`drive`/`brakes`
keys - PHYS-033 Q7, no converter); `data/vehicles/{car_sedan,tractor}.json`
are `/2`. `VehicleInstance` owns one `drivetrain::Powertrain` (commit 5b;
see the "Drivetrain" section below), replacing commit 5a's TEMPORARY
`P3Shim`/`build_p3_shim`. `World::wheel_is_driven(VehicleId, wheel_index)`
(now backed by `Powertrain::wheel_is_driven` - a hub is driven iff its
shaft port was connected to anything) is the replacement for the deleted
`WheelDesc::driven` field for read-only callers (e.g. the Godot adapter's
`get_wheel_driven`). `World::wheel_omega`/`::powertrain_energy` (also
commit 5b) expose the hub's own angular velocity and the vehicle's
`drivetrain::PowertrainEnergy` ledger.

- `tyre_desc.h`/`tyre_model.h` (public) + `pacejka_tyre_model.cpp`
  (internal) - Pacejka Magic Formula MF5.2/6.1-SUBSET: pure-slip
  `Fx0(kappa, Fz, gamma)`/`Fy0(alpha, Fz, gamma)` (sine-arctangent B/C/D/E
  form), combined-slip `Gxa`/`Gyk` cosine-weighted normalisation, a
  simplified `Mz0` (its own B/C/D/E curve, NOT the full pneumatic-trail x
  `Fy` decomposition - PHYS-036: qdz1/qdz2 carry the OPPOSITE sign to
  pdy1/pdy2 so Mz0 = -t*Fy0, Pacejka's own aligning-moment convention, and
  qbz1 sets a realistic ~4-4.5 deg peak slip angle; both are data-file
  fields, not formula code - see data/tyres/*.json's "aligning" source
  notes and tyre_desc.h's own field comment) and `Mx` (reduced linear-in-camber form), rolling
  resistance (`qsy1 + surface crr + qsy2*|Fx|/Fz0 + qsy4*(v/v0)^4`).
  `create_pacejka_tyre_model()` is the only `ITyreModel` factory. Slip
  relaxation (`step_slip_relaxation`, Pacejka ch. 7's transient model) is a
  velocity-forced deflection ODE, `d(kappa_eq)/dt = (slip_velocity_x -
  Vx*kappa_eq)/sigma_kappa` (and the analogous `alpha_eq`/`Vy` form),
  integrated EXACTLY over a substep via the scalar exponential - degenerates
  to plain integration of slip velocity at `Vx ~ 0` (a stiffness/deflection
  model, needed so a parked tyre on a slope still builds a restoring force
  instead of freezing at whatever slip ratio it last had) and recovers the
  usual quasi-steady slip ratio at speed. `kappa_eq`/`alpha_eq` are
  additionally clamped to `[-3, 3]`/`[-1.6, 1.6]` (`step_slip_relaxation`'s
  own safety clamp) - found necessary when a stalled/reversing wheel with
  `Vx ~ 0` combined with a persistent nonzero slip velocity (e.g. a brake
  controller overshoot) left the ODE's decay term at zero, so pure
  unbounded integration ran away to -7.9 and beyond well past the point
  any real tyre's force curve means anything.
- `tir_parser.h`/`.cpp` - reads a standard MF `.tir` file into a `TyreDesc`
  via a key-name-to-pointer-to-member table (never commit a proprietary
  `.tir` file - PLAN.md section 8; `data/tyres/*.json` are this project's
  own JSON format instead, loaded via `ps::io::load_tyre_json`).
- `suspension.h` (header-only) - spring (rate or `Curve1D`, plus preload),
  damper (bump/rebound rates or `Curve1D`), bump-stop and anti-roll-bar
  force functions from a measured compression `x`/rate `xdot`.
- `wheel_instance.h`/`.cpp` (`WheelInstance`) - `measure_contact()`: wheel
  vertical position is KINEMATIC, a `CylinderShape` shape-cast (PLAN.md
  5.1: "not a ray") along `SuspensionDesc::axis` from full-bump toward full
  extension, clamped to `[0, travel_max]` - not a separate unsprung-mass
  ODE (deferred: no wheel-hop/unsprung-resonance mode; the shipped
  acceptance tests don't need one). PHYS-033 P4 I2 commit 5b: the old single
  `step()` is now `pre_solve()`/`post_solve()`, split around the drivetrain
  network's own solve (`ps::drivetrain::ShaftNetwork::solve`, called by
  `VehicleInstance::step_tick` between the two) - wheel spin (`omega_`) is
  gone from `WheelInstance` entirely, now the network's own hub-node state.
  `pre_solve()`: suspension force from the measured `x`/`xdot`; alpha's
  relaxation step (explicit, final for the substep); kappa's relaxation
  PREDICTOR (linear in the hub's still-unknown new omega,
  `ITyreModel::evaluate_with_slope`/`relaxation_step_axis`); returns this
  wheel's `HubTerms{tau0, d}` for the network's `Assembly` (`tau0 =
  -R*Fx(kappa_pred)`, `d = R*max(dFx/dkappa,0)*d(kappa_pred)/d(omega)`).
  `post_solve(omega_new)`: finishes kappa/Fx at the hub's actual new speed
  (still linear, so the wheel's own torque balance and the chassis force
  agree exactly), assembles the contact wrench and this substep's
  `wheel_out`/`tyre_loss` energy-ledger deltas (`ps/drivetrain/
  energy_ledger.h`). `pre_solve()`/`post_solve()` also add
  `low_speed_lateral_damping_force` (a low-speed-only
  viscous term on `fy`, `TyreDesc::low_speed_damping_ratio`/
  `low_speed_damping_vx` - PHYS-033 I1, `ps/vehicle/tyre_desc.h`'s own field
  comment): `c = 2*ratio*sqrt(k_lat*m_ref)`,
  `k_lat = pky1*fz0/relaxation_length_y`, `m_ref = fz0/9.81` (the tyre's
  FIXED nominal load, never the live/instantaneous `fz` - PHYS-024), faded
  to zero above `low_speed_damping_vx` on TOTAL contact-patch speed (not
  just longitudinal). Physical carcass/relaxation damping the tyre model's
  own relaxation ODE is missing at vanishing rolling speed - NOT a
  compensation for any tick-level delay (that delay is removed outright by
  the W1 shadow chassis below; see PHYS-024/PHYS-033 in the vault for the
  full history of both). Shipped default `low_speed_damping_ratio = 1.5`
  for every `data/tyres/*.json` file: a `{0, 0.1, ..., 1.5}` sweep run
  against the W1 shadow chassis (`test_vehicle_standstill_wiggle.cpp`'s
  `"zeta sweep diagnostic"`, `[.][diag]`) found every value neutral for the
  yaw/lateral modes themselves, but the full-throttle standing-start and
  parking-slope acceptance tests are both non-monotonic in this ratio over
  `[0, 1.5]` - 1.5 is the only value tried that keeps both passing
  simultaneously. A per-vehicle-file explicit-integration stability bound
  (`gamma < 1`) on this term lives in `test_vehicle_shadow_chassis.cpp`
  (PHYS-033 I1b commit C7: `gamma`'s lever is the wheel's CONTACT-PATCH
  offset, `{attachment.x, attachment.y, -h_cg}`, not the wheel-centre
  attachment point - see below for why the two now differ; car_sedan's
  gamma rises to 0.950 at the shipped ratio 1.5, tractor to 0.652, both
  still under 1).
- **PHYS-033 I1b (`ContactFrame`, `measure_contact`/`step()`, defects 1/2
  of the plan's "vehicle contact geometry" increment):** every wheel force
  now resolves along the GROUND contact plane and acts at the true ground
  contact PATCH, not the chassis-fixed suspension/forward/lateral axes at
  the wheel's own CENTRE the P3 model used (a statics violation: it
  deleted the moment `R_wheel x F` from the car's own moment balance,
  understating pitch/roll load transfer by ~53%, and let a chassis at
  rest but not level relative to the ground self-propel with no controls
  at all - PHYS-033 I1b plan section 0). `measure_contact()` (pass 1)
  builds a `ContactFrame{point, normal, long_dir, lat_dir,
  normal_fallback}`: `normal` is the shape-cast's own hit normal
  (`ps/backend/i_rigid_backend.h`'s `ShapeCastHit::normal`), validated
  (finite, unit length, `started_penetrating` false, within ~75.5 deg of
  the suspension axis) and falling back to the suspension axis otherwise
  (bottomed/self-hit/wall), with a second fallback to the same axis if the
  normal is nearly parallel to the wheel's own spin axis (a side-wall
  hit) - `spin` (the `CylinderShape`'s local Y) is perpendicular to the
  suspension axis BY CONSTRUCTION (steer only rotates about the
  suspension axis itself), so a fallback normal always yields a
  non-degenerate in-plane component. `long_dir = normalize(spin x
  normal)` (the wheel's rolling direction, in the contact plane),
  `lat_dir = normal x long_dir`. The PATCH point (decision D-A): the
  ground normal projected into the wheel's own mid-plane
  (`n - spin*dot(n,spin)`, normalized) gives a `radial` direction;
  `centre - R*radial` is the lowest point of the wheel's mid-plane circle
  toward the ground; when the normal is trusted, Jolt's own
  `ShapeCastHit::contact_point` (I1b commit C4, additive) nudges the
  patch along `normal` by at most half the wheel width, removing a rim
  contact's line-contact ambiguity without touching the in-plane
  position. `step()` (pass 2) measures slip velocity at the patch along
  `long_dir`/`lat_dir` (not the old chassis-built forward/lateral axes)
  and assembles the wheel's ENTIRE reaction as one vector via the
  file-local `assemble_contact_wrench()`: `R = N*normal +
  (Fx+F_roll)*long_dir + Fy*lat_dir` at the patch, plus the couple
  `Mz*normal + Mx*long_dir` - replacing the old separate suspension-
  force-at-attachment + tyre-force-at-contact assembly.
  `WheelSubstepOutput` accordingly carries `contact_force_world`/
  `contact_point_world`/`extra_torque_world`, not the old
  `suspension_force_world`/`attachment_point_world`/`tyre_force_world`.
  Bilateral in `N` exactly as before (a negative `N`, the rebound damper,
  still pulls the chassis toward the ground while `hit_ground_`). Camber
  is unchanged (no geometric inclination from `dot(normal, spin)`).
  White-box pins: `tests/unit/test_wheel_contact_frame.cpp` (core/src on
  its include path); whole-vehicle pins:
  `tests/validation/test_vehicle_contact_geometry.cpp` (a pitched-at-rest
  chassis does not self-propel; steady-cornering/straight-braking load-
  transfer ratios match statics, `LTR_lat`/`LTR_long` ~1.0). Moved several
  existing acceptance tests past their own gates (reported, not tuned
  away - see the I1b report/vault): `test_vehicle_skidpad.cpp` (steady_a_lat
  -7.8% vs. estimate; the test's own steady-state premise does not hold
  at this operating point - a `[.][diag]` case in
  `test_vehicle_contact_geometry.cpp` confirms `v*omega_z` diverges from
  the true measured lateral acceleration there), `test_vehicle_parking_
  slope.cpp` and `test_vehicle_tractor_cross_slope.cpp` (both slopes) -
  a pre-existing "elastic hold" (static tyre-deflection spring, not exact
  Coulomb friction) weakness the larger patch lever pushes past 1 mm,
  which PLAN.md's P4 exact-Coulomb-friction brake rows (I2) are expected
  to fix, not attempted here. Also required `car_sedan.json`'s
  `final_drive_ratio` 10.0 -> 8.5 (I1b commit C8, interim - deleted in I3):
  the corrected load transfer unloads the front axle enough under
  acceleration that 10.0 pushed the settled full-throttle standing start
  into sustained wheelspin (`max_abs_slip` 1.39 vs. the test's own < 0.3
  bound); see that data file's own "source" field for the exact
  arithmetic.
- **PHYS-033 I1b (D-C, D-D): allocation-free wheel query shapes and a
  Jolt-visible `debug_alloc`.** `measure_contact`'s `shape_cast` (and
  `collide_shape_into`) now build their Box/Sphere/Capsule/Cylinder query
  shape on the STACK (`jolt_backend.cpp`'s `with_query_shape`, see "Jolt
  backend" below) instead of heap-allocating a fresh one every call (16
  per tick per vehicle, previously invisible to `debug_alloc` - see "Job
  system" below for why). `ShapeCastHit` also gained `contact_point`/
  `penetration_depth`/`started_penetrating` (I1b commit C4, additive -
  `point`'s existing meaning, the cast shape's own origin, is unchanged).
- `vehicle_instance.h`/`.cpp` (`VehicleInstance`) - `step_tick()` = `tick_begin`, then per
  substep `substep_forces` + `substep_advance`, then `tick_end`, on one `VehicleTickWork`
  (`System::step_island` runs the same pieces with `solve_coupling` between them, so the
  vehicles of a hitch share one job and each substep's socket force reaches both shadows;
  `System::step_all` takes that path only with a live coupling).
  It runs a per-substep SHADOW chassis (PHYS-033 I1 W1): a local pose/twist,
  seeded from the tick's real start pose/motion, integrated substep-by-
  substep using `IRigidBackend::get_integration_params(chassis)` (inverse
  mass/inertia at the TICK-START rotation, gravity factor, no gyroscopic
  term) exactly the way the backend's own velocity integrator will apply
  the tick's substep-averaged wrench in the next `backend_step` - within a
  substep the wheels see the shadow's own CURRENT twist (not a tick-start
  value held constant across all substeps, the previous scheme), so their
  relaxation state (`alpha_eq`/`kappa_eq`) advances against real intra-tick
  velocity. `shadow_dv_error()`/`shadow_dw_error()` (also
  `World::vehicle_shadow_dv_error`/`_dw_error` and two telemetry channels
  per vehicle, `vehicle<i>.chassis.shadow_dv_error`/`_dw_error`) report the
  gap between the shadow's prediction made last tick and the backend's
  actual chassis velocity this tick - near the float noise floor
  (~1e-6/1e-7) whenever nothing outside the shadow's model (a contact, a
  constraint, a changing external wrench) touches the chassis; see
  `test_vehicle_shadow_chassis.cpp`. `VehicleTickEnv{gravity}` carries the
  one piece of `World` state the shadow needs; the external wrench it uses
  is last tick's gathered force-element wrench on the chassis body
  (`vehicle::System::note_body_wrench`, called from `World::stage_gather`'s
  `else` branch - an O(1) array lookup, no allocation). Still computes
  every wheel's contact (pass 1, so the anti-roll-bar's partner-compression
  read in pass 2 is fresh), then each wheel's suspension+tyre+spin substep
  (pass 2); still applies the substep-AVERAGED wrench once per tick via
  `IRigidBackend::apply_wrench` directly (not through `IForceElement`,
  whose `evaluate()` is const/stateless by contract and cannot own wheel
  state) - mirrors `articulation::ContactCoupling`'s existing "apply
  directly to the other body from inside the substep stage" precedent.
  Ackermann steering (`ackermann_wheel_angle()`): per-wheel angle from turn radius `R =
  wheelbase/tan(delta_avg)` and each wheel's attachment point relative to
  the rear axle (both derived from wheel attachment points at construction,
  not stored in `SteeringDesc`), blended by `ackermann_fraction`; a rear-
  steered wheel is proportional/opposite-sign only (no speed-dependent
  phase reversal). A front wheel's angle is `atan(dx / (R - dy))`, NOT
  `atan2(dx, R - dy)` - `dx` (the wheelbase) is always positive, so
  `atan2` could never return a negative (right-lock) wheel angle; see
  `tests/unit/test_vehicle_ackermann_symmetry.cpp`,
  `tests/validation/test_vehicle_steer_mirror.cpp` and vault PHYS-022.
  `ackermann_fraction` accepts `[-1, 1]`, validated at load time
  (`ps::io::load_vehicle_json`) - negative is anti-Ackermann (outer wheel
  gets the larger-magnitude angle instead of the inner); see
  `vehicle_desc.h`'s `SteeringDesc::ackermann_fraction` comment and vault
  PHYS-031 for the measured effect: a closed-loop (radius-holding,
  quasi-steady speed ramp) skidpad limit test shows car_sedan.json is
  REAR-limited at its cornering limit (front tyres still below their own
  peak slip angle when the rear axle already exceeds it), so no front
  steering geometry - Ackermann, parallel or anti-Ackermann - moves this
  vehicle's true grip limit; a fixed-large-steer sweep (the first attempt)
  falsely showed anti-Ackermann "reducing" grip only because it measured
  every wheel well past peak slip angle (scrubbing), not a limit condition.
  PHYS-033 P4 I2 commit 5b: brake/drive torque is no longer computed here at
  all - `step_tick` instead calls `Powertrain::pre_solve_components`,
  collects each wheel's `HubTerms` from `WheelInstance::pre_solve` into the
  shaft network's `Assembly`, calls `Powertrain::solve` (one implicit solve
  per substep, not per tick), then `Powertrain::post_solve_components` and
  each wheel's `WheelInstance::post_solve` - see "Drivetrain" below.
- `system.h`/`.cpp` (`System`) - owns every `World` vehicle instance;
  mirrors `articulation::System`'s pattern exactly (one job per vehicle via
  `parallel_for`, safe unparallelised because each vehicle owns a distinct
  chassis body - no two jobs ever call `apply_wrench` on the same body,
  unlike articulation's Compliant-contact case).

`ps/vehicle/wheel_state.h` (public, PLAN.md M1 - added for the Godot
adapter's wheel-mesh placement, separate from `wheel_telemetry.h`'s
`WheelTelemetry` since that one also carries `Fx`/`Fy`/`omega` a mesh-
placement consumer does not need): `WheelState{steer_angle,
suspension_travel, spin_angle, load, slip_ratio, slip_angle, surface}` -
`World::wheel_state(VehicleId, wheel_index)` (live, sim-thread-only, mirrors
`wheel_telemetry`'s own contract) and `WorldSnapshot::vehicles` (race-free,
see "Adapter seams" below) both return it. `WheelInstance` (`wheel_instance.h`)
gained two members feeding it: `spin_angle_` (plain explicit-Euler
integration of the hub's OLD omega times `dt` each substep, in
`post_solve()` - PHYS-033 P4 I2 commit 5b: `omega_` itself moved out of
`WheelInstance` into the drivetrain network's hub-node state, see
"Drivetrain" below; `spin_angle_` still lives here since nothing else reads
it back into dynamics) and `last_steer_angle_` (the `steer_angle`
`WheelPreSolveInput` already carried in, now also stored). Both are
deliberately EXCLUDED from
`fold_state_hash()` - neither is ever read back by anything that feeds into
future dynamics (write-only as far as the simulation itself is concerned),
so excluding them cannot hide a worker-count-dependent divergence, and every
existing scenario's `state_hash` had to stay bit-identical (confirmed:
`vehicle_step_steer.json` still hashed to `0x1c2b775be0dc3339` at 1 and 10
workers, matching the value recorded before this field existed - superseded
PHYS-033 P4 I5 prep 2026-09-25 by car_sedan.json's 60/40 front/rear static
split, now `0xe77fd6569d82786c (tyre crr d1e1144, was 0xbab69b300eba41a3)` at 1 and 10 workers, still worker-count
invariant).

`World` additions (additive, `ps/world/world.h`): `create_vehicle`
(PHYS-033 P4 I4 C3: trailing defaulted `const std::vector<std::pair<
std::string, real>>& test_only_extra_channels = {}`, resolved into
`ControlChannelIndices::component` - test-only, always empty for every
production caller, see "Drivetrain" below), `vehicle_wheel_count`,
`wheel_telemetry`, `wheel_name`, `wheel_state`, `vehicle_shadow_dv_error`,
`vehicle_shadow_dw_error` (PHYS-033 I1 W1, see above), `wheel_omega`,
`powertrain_energy` (PHYS-033 P4 I2 commit 5b, see "Drivetrain" below),
`vehicle_component_channel_values` (PHYS-033 P4 I4 C3, mirrors
`wheel_omega`/`powertrain_energy`'s read-back pattern - see "Drivetrain"
below), `replace_engine_maps(VehicleId, component_id, EngineMapsPtr)` (I7c C16b:
sim thread between steps; validates and prefills the SimulatedEngine's steady
tables on the caller thread, throws before any mutation on a bad id/null maps/
non-simulated engine; the pointer swap and the old maps' release happen in
`World::step` right before the pipeline runs, never inside a job - every
TaskGraph stage is a job; only the one engine component a vehicle has is
addressable).
`ps/io/scenario.h`
additions: `ScenarioVehicle` (a vehicle on an already-described chassis
body, `vehicle_json_path` resolved relative to the scenario file, loaded
via `ps::io::load_vehicle_json`), `ScenarioControlEvent` (`{t, channel,
value}`, `World::set_control` applied once `sim_time` reaches `t` -
`tools/ps_run/main.cpp` applies these in time order every tick),
`Scenario::surface_table_path` (installs a `SurfaceTable` for every
vehicle's tyre model to read `lambda_mu`/`crr` from). `ps/backend/
shape_desc.h` addition: `CylinderShape` (added to the `ShapeDesc` variant
for the wheel contact shape-cast above; every exhaustive `std::visit` over
`ShapeDesc` - `jolt_backend.cpp::make_shape` - and `scenario.cpp`'s JSON
shape reader (`"type": "cylinder"`) handle it).

`tests/support/vehicle_test_helpers.h`'s `build_vehicle_rig()` documents a
load-bearing constraint for anyone assembling a chassis body around this
system: the chassis collision shape must stay narrower/higher than the
wheels' full suspension-travel envelope by a margin bigger than one wheel
radius (a STEERED wheel's query cylinder sweeps a cone-like footprint as it
rotates, growing its effective margin need beyond the unrotated case) -
`IRigidBackend::shape_cast` excludes no body (`shape_cast_excluding`, used only by
the wheels of a coupled vehicle, leaves out the island's chassis), and Jolt's `CastShape` reports an immediate fraction-0 "hit" against
a shape the query already starts inside of, inconsistently depending on
broadphase iteration order. An undersized chassis box produced wildly wrong
per-wheel `Fz` (some wheels reading bottomed-out, others airborne) that fed
a runaway tyre force and chassis roll/pitch - found and fixed during this
phase's own validation-test debugging.

Data files: `data/vehicles/car_sedan.json` + `data/tyres/
passenger_car_225_45r17.json`, `data/vehicles/tractor.json` +
`data/tyres/{tractor_front,tractor_rear}.json` - geometry derived from
`data/models/*/*.rig.json`'s wheel pivots/radii, plausible/published
parameters with source noted per PLAN.md section 8 (never a proprietary
`.tir` file). `data/tyres/*.json` each set `low_speed_damping_ratio`/
`low_speed_damping_vx` explicitly (PHYS-033 I1, see above).
`data/vehicles/tractor.json`'s rear wheel attachment z is `-0.275` (PHYS-033
I1 commit G1 - was `-0.725`, inconsistent with the front/rear wheel radii
`data/models/tractor/tractor.rig.json` implies; see the file's own `source`
field). Both vehicle files are `physics_sim.vehicle/2` (PHYS-033 I2 commit
5a): each `wheels[]` entry lost its `driven` flag, and the top-level
`brakes`/`drive` blocks are replaced by `brake_system`/`powertrain` (see
"Vehicle system" above).

### Drivetrain (`ps::drivetrain`, `core/src/drivetrain/`, PHYS-033 P4 I2)

A reduced-coordinate shaft network (one scalar DOF per node: an angular
velocity) shared by every vehicle's engine/gearing/differential/brakes,
replacing the P3 `P3Shim`'s direct torque-to-wheel wiring (commit 5a) with
an implicit per-substep linear solve (commit 5b) that also owns wheel spin
itself (moved out of `WheelInstance`, see "Vehicle system" above). Public
descriptors only in `ps/drivetrain/{powertrain_desc.h,brake_system_desc.h,
energy_ledger.h}`; everything below is internal (`core/src/drivetrain/`,
not on `ps_core`'s public include path).

- `dense.h` - header-only small dense linear algebra (`SmallVec`/`SmallMat`,
  Cholesky decomposition) sized for a single vehicle's network (at most a
  handful of nodes/rows), not a general sparse solver.
- `box_lcp.h`/`.cpp` - exact box-constrained LCP solve for Coulomb friction
  rows (a row's Lagrange multiplier is bounded to `[-capacity, capacity]`,
  the static/kinetic switch commit 5b uses for brakes instead of a hard
  torque clamp): three-stage fallback - block principal pivoting first,
  then Murty's least-index scheme, then exhaustive complementary-basis
  enumeration for the (rare, small) cases neither converges on. Exercised
  directly by a brute-force unit test sweeping ~10⁴ random small problems
  against a reference solve.
- `nullspace.h` (`build_nullspace`) - reduces a rigid constraint row (a gear
  mesh ratio, a differential's Willis equation) to a null-space basis at
  graph-build time, so the runtime solve never carries redundant DOFs for a
  ratio that never changes.
- `network.h`/`.cpp` (`NetworkBuilder`, `Assembly`, `ShaftNetwork`) - the
  generic solver: `NetworkBuilder` assigns `NodeId`s and rigid/friction rows
  from a graph description; `Assembly` is the per-substep scratch a
  component's `pre_solve` writes `tau0`/`d` (linear torque-vs-omega) terms
  into per node; `ShaftNetwork::solve(h)` does one linearly-implicit-Euler
  step of the whole coupled system (rigid rows eliminated via the
  precomputed null-space basis, friction rows resolved via `box_lcp`),
  returning each node's new omega. `node_inertia`/`friction_row_a`/
  `friction_row_b` are small additive read accessors `components.cpp`'s
  `GearPair::post_solve` and `Powertrain::solve` use to reconstruct
  per-component quantities (mesh-loss torque, ledger terms) the generic
  solver itself does not track. `NetworkBuilder::add_coupling(a, b)`/
  `Assembly::set_coupling(id, ra, rb, c)` (PHYS-033 P4 I4 C2): implicit
  two-node viscous couplings on `v = ra*w_a - rb*w_b` (merge the two nodes'
  clusters, add `h*c*gt*gt^T` to the reduced matrix, rhs unchanged, skipped
  at `c == 0`); `coupling_velocity(id)` returns `v_new` for the caller's
  `h*c*v^2` ledger booking.
- `components.h`/`.cpp` (`IComponent` and four concrete types, commit 5b) -
  `IComponent`: `pre_solve(ctx, network, assembly)` (write this substep's
  `tau0`/`d` terms), `post_solve(ctx, network, ledger)` (read back the
  solved omegas, accumulate this component's energy-ledger contribution),
  `fold_state_hash` (default no-op; only a component with its own
  persistent state beyond the network's node omegas overrides it).
  `ComponentSubstepCtx` also carries `const real* channels`/`int
  channel_count` (PHYS-033 P4 I4 C3, see `Powertrain::declare_control_
  channels` below) - read by the lockable-diff lock bits (C5),
  TransferCase (C9), ParkLock (C10), Pto (C11) and the engine governor's
  hand_throttle (C12).
  - `TorqueMapEngine` - `wot_torque_nm_vs_rpm` curve times `ctx.throttle`,
    clamped to `[idle_rpm, limiter.rpm]` and zeroed above the limiter;
    `d` (the linearised damping term the implicit solve needs) comes from
    the curve's own local slope, not a separate tuning constant.
    Optional all-speed governor with droop (PHYS-033 P4 I4 C12,
    `TorqueMapEngineDesc::has_governor`/`governor`, docs/data_formats/
    engine.md): lever = max(hand_throttle channel, pedal), n_nl = idle +
    lever*(high_idle - idle), band = droop*rated, theta_gov = clamp((n_nl -
    n)/band, 0, 1) replaces the pedal as the fuel request (ignition cut,
    TCU ceiling/floor still apply). Its slope term enters `d` only while
    the governor actually sets the fuel (0 < theta_gov < 1, theta_req ==
    theta_gov, theta_eff == theta_req). No new hashed state. The idle PI's
    own d(theta_idle)/d(omega) is still not in `d` (pre-existing).
  - Throttle-command stage (PHYS-033 P4 I7a C3, `throttle_stage.h`, a
    typed `Powertrain` member, not an `IComponent`): the engine never reads
    the pedal. Right before the engine's `pre_solve`, same substep, no
    state, the stage turns `ctx.throttle` into `ThrottleCommand{command,
    floor, ceiling}`: `pedal' = curve ? curve(pedal) : pedal`; `command =
    hand lever ? max(clamp(hand,0,1), clamp(pedal',0,1)) : pedal'`;
    floor/ceiling = the lagged TCU limits (electronic) or 0/1 (mechanical).
    The engine keeps governor (lever = command), `max(min(req, ceiling),
    floor)`, ignition cut, idle PI, DFCO, limiter. Kind comes from an
    explicit `"type": "throttle_control"` component (`electronic` with an
    optional `pedal_map`, `mechanical_linkage` with an optional
    `linkage_curve`, optional `hand_lever_channel`) or, absent one, an
    implicit stage of the engine's `throttle_input` (default electronic, no
    curve, hand lever = the governor's channel iff a governor exists - so
    every pre-C3 vehicle is bit-identical). Loader rejections: kind vs
    throttle_input mismatch either way, more than one stage, no engine or a
    non-engine ref, a mechanical path with nonzero auto_blip
    kp_per_rpm/max_throttle/upshift_torque_ramp_s after defaults (the
    upshift ignition cut stays available), a curve not spanning x 0..1
    exactly / y outside [0,1] / y decreasing, a hand_lever_channel
    differing from a governor's channel. I7a C5b: the stage runs once per substep BEFORE the
    ManualTcu, which reads its `command` (`Sensors::throttle_request`) for
    the launch law, auto-shift thresholds and the N->1 auto-select - the
    assists act on the torque request, not the raw pedal. `EngineSoundState::throttle_command` = the command.
  - Auto-clutch anti-stall (ManualTcu, `manual_tcu.cpp`; ca118ec and its
    2026-10-04 fix): a Locked clutch releases AT ONCE when engine rpm OR the
    SIGNED gearbox-input rpm falls below `AutoClutchDesc::anti_stall_rpm`.
    The rolling re-close in Launch mode and the post-shift launch-vs-shift
    decision use the signed input rpm against `rolling_reengage_rpm`
    (car_hyper 900, default 1300). Reverse-rotation hold (`reverse_hold_`,
    hashed only when set): in a forward gear the clutch is held open once
    the input is clearly reversing (below -anti_stall_rpm, or below
    -lock_slip_rpm right after an anti-stall release, `anti_stall_released_`)
    until it is above -lock_slip_rpm again; brief negative transients of a
    launch (an unsettled start, locked-diff wind-up) do not trigger it.
    Assist off = the manual clutch is authoritative. Turbo engines
    (60d60fa): `torque_demand` = max(request, the uncut net torque of the
    CURRENT charge at the live exhaust pressure), so limiter pulses do not
    reopen the clutch while shift cuts and DFCO still lower it.
    Tests `[hyper_clutch][tcu]`; notes HYPER_CLUTCH_FLARE.md, vault PHYS-040.
  - `GearPair` - a fixed-ratio mesh, realised as one of `network.h`'s rigid
    rows; carries `lambda_mesh_prev_` (the previous substep's reconstructed
    mesh-loss multiplier, folded into the state hash - the one `IComponent`
    that overrides `fold_state_hash`) so a lossy mesh's dissipation is a
    lagged reconstruction from momentum balance rather than an extra
    implicit unknown.
  - `Differential` (PHYS-033 P4 I4 C4, was `OpenDifferential`) - an open
    differential's Willis constraint is one of `NetworkBuilder`'s rigid rows
    (carrier +1, left/right -ratio/2, built once at graph-build time); the
    component only reconstructs the row's multiplier for mesh loss/debug.
    `ReconMode` (set by `Powertrain` via `configure_reconstruction`): `Leaf`
    (the I3 left-only `reconstruct_mesh_torque` call, unchanged), `Nested`
    (outputs are child diffs' carriers: `reconstruct_mesh_torque_pair` over
    both outputs, children's own row lambdas - `compute_row_lambda`, pure
    and const, so post_solve order does not matter - passed as the other
    rigid reactions), `None` (not reconstructible; `Powertrain` rejects it
    when efficiency < 1), `LeafPair` (I4 C5, every LOCKABLE diff: the pair
    path with zero other reactions - the diff's own lock-row reaction
    cancels, exact locked or not). `lambda_mesh_prev_` (hashed) only when lossy;
    `lambda_row_` (unhashed) always, for `hub_shaft_torque`/`diff_mesh_torque`
    debug. `reconstruct_mesh_torque_pair` (components.h) sums both outputs'
    torque balances so a lock/LSD/equal-ratio coupling reaction between them
    cancels. `DifferentialKind` (I4 C6): `Open`, `Locked` (spool - an
    unconditional `{L:+1, R:-1}` row in EVERY topology state, emitted before
    C5's per-bit lock rows, spending no composed-state bit, excluded from the
    touch counts like them; `Powertrain::always_locked_diffs_` averages L/R in
    `reset()`), `Viscous` (the open row plus an implicit L-R
    `add_coupling`, `set_coupling(id, 1, 1, viscous_nm_s_rad)` every substep;
    `has_viscous_coupling_` guards because `CouplingId{}`'s default index 0
    is a real id; dissipation booked as `PowertrainEnergy::diff_loss`, in
    `end_tick()`'s balance). A non-`Open` leaf diff uses `LeafPair`; a
    lossless non-`Open` centre diff may be `Nested`; a lossy non-`Open` centre
    diff is rejected. Loader: `kind` "open"/"locked"/"viscous"
    (`viscous_nm_s_rad` > 0 required for viscous only; "active" rejected as
    not yet supported; `lockable` rejected on "locked"). `ClutchLsd`/`Torsen`
    (I4 C7): the open row plus one L-R friction row (ratio_b 1, every topology
    state, no bit, appended after every pre-C7 friction row in component
    order) whose capacity comes from the PREVIOUS substep's `lambda_row_`
    (one-substep lag, owner-approved): `T_axle = |ratio*lambda_prev|`;
    clutch_lsd `cap = (preload_nm + f*T_axle)/2`, `f = drive_factor` iff
    `hub_coef*lambda_prev*(w_L+w_R) > 0` else `coast_factor`, static =
    `static_factor*kinetic`; torsen `cap = (T_axle/2)*(TBR-1)/(TBR+1)`. Slip
    loss `-h*v*lambda` goes to `diff_loss`. Leaf-only (`LeafPair`) whatever
    the efficiency; a lockable one has capacity 0 while locked (lock bit read
    from `net.topology_state()`, decided before component pre_solve).
    `lambda_row_` is hashed only for these two kinds; Open/Locked/Viscous hash
    as before. Loader: "clutch_lsd" (`preload_nm` >= 0, required
    `ramp{drive_factor, coast_factor}` in [0,1), `static_factor` >= 1) and
    "torsen" (required `torque_bias_ratio` > 1); another kind's field is
    rejected.
  - `TransferCase` (PHYS-033 P4 I4 C9, tractor MFWD) - ports in/rear/front;
    a rigid row `w_in = rear_ratio*w_rear` in every topology state (built
    like a gear pair) plus one friction row (in, front) with `ratio_b =
    front_ratio` (re-set each substep via `Assembly::set_friction_row_ratio`),
    appended after clutch/engine/synchro rows and before the brakes so no
    existing vehicle's rows shift. Engagement `e` ramps at `1/engage_time_s`
    toward (channel >= 0.5; channel "awd" by default, declared via
    `declare_control_channels`); kinetic capacity `e*capacity_nm`, static =
    `static_factor*kinetic`. Slip loss `-h*v*lambda` -> `PowertrainEnergy::
    awd_clutch_loss` (in `end_tick()`'s balance). Only `e` is hashed. At most
    one per vehicle. Speedo walk recurses in -> rear only; reset derives
    `in = rear_ratio*rear`. Loader rejects ratio 0, capacity <= 0,
    static_factor < 1, engage_time < 0, empty channel.
  - `ParkLock` (PHYS-033 P4 I4 C10) - port `shaft` (tractor: the
    gearbox.out/transfer.in shaft), one friction row to ground placed right
    after the transfer-case row (Phase G) and before the brake rows. Engages
    when channel ("park_lock", default 0) >= 0.5 and the shaft's pre-solve
    |omega| <= `max_engage_speed_rad_s` (else pending, retried every
    substep, unhashed); once engaged it holds whatever the solved speed;
    releases the substep the channel reads < 0.5. Engaged: kinetic cap =
    `capacity_nm`, static = `static_factor*kinetic`. Slip `-h*v*lambda` ->
    `brake_loss`. Only `engaged_` is hashed. At most one per vehicle. Loader
    rejects capacity <= 0, static_factor < 1, negative speed, empty channel.
  - `Pto` (PHYS-033 P4 I4 C11) - ports `in` (tractor: shafted with
    engine.crank/clutch.in, live ahead of the main clutch) and `out` (exempt
    from the port-connectivity check: may stand alone with
    `out_inertia_kgm2` or be shafted to an external load). One friction row
    (in, out) with `ratio_b` = the current mode's ratio (540 or 1000),
    re-set each substep; placed after the park_lock row, before the brakes.
    Engagement `e` ramps at `1/engage_time_s` toward (channel "pto" >= 0.5);
    kinetic cap `e*capacity_nm`, static `static_factor*kinetic`. Mode
    (channel "pto_speed", >= 770 selects 1000) switches only while the
    pre-ramp `e == 0` and `|omega_out| < 1` rad/s; initial mode 540 if
    present. While post-ramp `e == 0` an implicit viscous stub brake
    (`c = J_out/disengaged_brake_time_s`) decays `out`. Row slip and stub
    loss -> `PowertrainEnergy::pto_clutch_loss`; efficiency < 1 adds a
    lagged loss torque `(1-eta)*|lambda_prev|*sign(omega_in_old)` at `in`,
    booked into `gear_loss`. Hashed: `e`, mode, and `lambda_prev` when
    eta < 1. Reset sets `out = in/ratio` if the row is stuck or `e >= 1`.
    At most one per vehicle.
  - `LoadDyno` (PHYS-033 P4 I4 C11, test/bench only - no shipped vehicle)
    - one port `shaft`; `Brake` (a friction row to ground, cap =
    `torque_nm*scale`, static = kinetic, placed after the pto row),
    `Viscous` (implicit `c*omega`) or `Quadratic` (`k*omega*|omega|`
    linearised about omega_old); optional `scale_channel` (default 1).
    Absorbed work -> `PowertrainEnergy::external_absorbed`. Stateless, not
    hashed.
  - Fuel tank (PHYS-033 P4 I7a C6, `FuelTankDesc` + `PowertrainDesc::fluid`
    link `"<tank>.out" -> "<engine>.fuel"`, docs/data_formats/vehicle.md) -
    not an `IComponent`: no port, node or row; `Powertrain` owns a
    `FuelTank` (`energy_store.h`). The tank's fuel must be in the fed
    engine's `fuels` (loader: path equivalence; Powertrain backstop for
    code-built descs: value equality) and is the fuel that engine burns
    (without a tank: the first `fuels` entry). Per substep the engine gets
    `Sensors::fuel_available_kg` (the tank content; +inf without a tank) and
    applies `s_starve = clamp(m_tank/(h*mdot_demand), 0, 1)` (1 at zero
    demand) to tau_comb, every combustion term of `d`, theta_c and the booked
    fuel, held for the substep (no ds/domega); `torque_demand` stays
    unscaled. post_solve withdraws exactly the booked mass (the emptying
    substep books the whole rest, tank exactly 0.0). An empty tank stalls the
    engine through the existing state machine (the idle integrator winds to
    its existing clamp, reset on Stalled). `m_tank` is hashed only when a
    tank exists. `World::fuel_tank_mass`/`set_fuel_tank_mass` (applied in
    the next tick's `begin_tick`, clamped to [0, capacity], booked signed into
    `fuel_external_added`; sim thread between steps only);
    `reset_vehicle` does not refill. The chassis mass does not change with
    fuel in I7a (PLAN.md "Fuel mass moves the chassis" - later).
  - Contents mass (racing_game PR #22, docs/data_formats/vehicle.md "Contents mass"): optional VehicleDesc::contents_mass
    {enabled, update_threshold_kg 0.1}; with it the chassis body mass = its creation (dry) mass + fuel tank + N2O bottle
    contents, the initial fill applied in create_vehicle, then semi-live at the start of World::step whenever the contents
    moved by the threshold (IRigidBackend::set_mass: Jolt ScaleToMass of the shape's mass properties, COM unchanged);
    applied kg hashed only when enabled, off = bit-identical (no shipped vehicle enables it); World::chassis_contents_mass/
    chassis_dry_mass. Tests tests/unit/test_contents_mass.cpp [contents_mass].
  - Engine thermal (engine thermal plan T1/T1b, out/engine_thermal_plan.md, vault PHYS-063; engine_thermal.h/.cpp,
    descriptors ps/drivetrain/engine_thermal_desc.h, loader ps/io/engine_thermal_io.h) - a Powertrain-owned member
    like the fuel tank, opt-in (a vehicle with a cooling_system and an engine with thermal data): two lumps (engine
    block+coolant, oil) plus wax thermostat (lag + hysteresis play), fan flag - 5 hashed values, backward Euler once
    per tick on SIM time (end_tick), heat = wall fraction x released fuel energy + friction work (30 % to oil),
    eps-NTU radiator with ram air, fan 98/93 C. Oil-viscosity friction factor k = max(1, 1 + phi_h*(sqrt(mu/mu_ref)
    - 1)), Walther fit, NO upper cap (owner 2026-10-05; friction_scale_max is rejected). T1b: k scales ONLY the
    hydrodynamic part, dF = (k-1)*F(n_ref)*sqrt(n/n_ref) (hydrodynamic_friction.h, linear below 50 rpm [AS],
    oil.hydrodynamic_reference_rpm required, 2000) - EngineTorqueMap uses F(n_ref) = phi_f*|m(n_ref)| (pumping
    never scaled), SimulatedEngine the map friction; the Coulomb row caps (c_k, breakaway) are never scaled. The
    ECU sees its own estimate (Sensors::ecu_friction_scale): graph idle feed-forward/authority and torque_demand,
    SimulatedEngine live idle recompute while k != 1 (~17 us/tick during warm-up; A5 shortcut steady(k) = steady(1)
    - (k-1)F not done). k == 1 bit-identical. Both thermal cars crank and start at -30 C on the shipped starter
    (I9); fire_rpm 100 [C SI minimum starting speed 40-100 rpm] in i4_2l_na.json and toyota_a25a_fks_sim.json
    (was 250 [AS]). Snapshot/gauge PowertrainSnapshot thermal fields (unhashed), World engine_thermal_state/
    set_engine_thermal_state, EngineRig thermal setters, PsSimulation::get_engine_thermal + HUD line. Tests
    test_engine_thermal.cpp (U1-U8), test_engine_thermal_powertrain.cpp, test_engine_thermal_coupling.cpp (I1),
    test_engine_thermal_scenarios.cpp (I2-I6, I9; [long] warm-up runs) - [thermal][drivetrain]; scenario
    data/scenarios/vehicle_thermal.json 0x79bdd9ac60402967 (block cooling water jacket on every shipped engine, was 0xfb37e62eaf06cabb; pedal P1, was 0x3178e9e670d1cb5d; tyre crr d1e1144, was 0xd7205b0b06295bd8) at 1/2/4/10/24 workers, hash-compared (ScenarioTopics
    vehicle+drivetrain). Notes out/thermal_t1_progress.md, out/thermal_t1b_progress.md.
    Water jacket (block cooling, optional, docs engine_thermal.md "Water jacket"): engine_thermal `water_jacket`
    {wetted_area_m2(_per_l), coolant_volume_l(_per_l), reference_flow_l_min(_per_l), htc_ref_w_m2_k,
    htc_min_w_m2_k, flow_exponent 0.8} splits the engine lump into metal (C_engine - C_coolant) and coolant, joined
    by G = A*max(htc_min, htc_ref*(Q/Q_ref)^n) at the full pump flow; wall heat/friction/block-air/oil act on the
    metal, radiator/thermostat/fan/gauge on the coolant; 3-lump backward Euler, metal hashed only with a jacket.
    cooling_system `pump.flow_factor` (0, 4] scales the pump flow (absent = 1, bit-identical). Snapshot metal_c,
    jacket_heat_w, jacket_conductance_w_k, coolant_flow_l_min (unhashed; also PsSimulation::get_engine_thermal).
    ps/drivetrain/engine_thermal_steady.h solve_steady_thermal = the fixed point of the step for constant inputs
    (bisection on the coolant temperature; thermostat play offset, fan off/on/cycling) plus the shared helpers.
    Tests test_engine_thermal_jacket.cpp J1-J7 [thermal][drivetrain]. Every shipped engine carries a jacket (owner
    2026-10-16), inherited from class_si_passenger.json (0.2 m2/L, 3.0 L/L, 5000 W/(m2 K) at 30 l/min/L, n 0.8, floor
    500, [AS]); fixture tests/support/fixtures/thermal/i4_2l_na_no_jacket.json ("water_jacket": null) keeps the
    two-lump path tested. CSV vehicle<i>.thermal.{metal_c,jacket_heat_w,coolant_flow_l_min} only with a jacket.
    Gauge, warnings, thermostat and fan read the coolant (owner: publish the metal temperature, no gauge change).
  - `DirectTorqueBrakes` - one friction row per corner (`Row{row,
    corner_max_nm, handbrake_max_nm}`, service and handbrake capacities
    merged per hub at graph-build time) - the exact-Coulomb replacement for
    P3's hard brake-torque clamp; commit 5b's stop condition 5 tracks how
    often a row's capacity is actually hit (`friction_cap_hits`) versus the
    LCP falling back past stage 1 (`nonspd_hits`/`lcp_stage3_hits`, folded
    up from `box_lcp`'s three-stage fallback above).
- Compliant shafts (PHYS-033 P4 I4 C13, `PowertrainDesc::compliance` =
  `Compliance::Compliant`; under `Rigid` shaft stiffness/damping are ignored):
  a shaft with stiffness > 0 keeps its 2 ports as separate nodes (both must
  have inertia > 0) joined by a C2 coupling. Trapezoidal by default
  (`c_impl = c/2 + k*h/4`, explicit torque `k*theta + c_impl*v_old` via
  `Assembly::torque`), backward Euler for one substep after a topology
  switch, any friction-row stick change or `reset()` (`be_next_`). Twist per
  shaft and `be_next_` hashed only when a compliant shaft exists.
  `Assembly::implicit_reaction` (filled after the solve, read only when
  `has_implicit_reactions()`) lets the gear/diff node-balance
  reconstructions see the coupling's implicit torque. Ledger:
  `shaft_pe` (state) and `shaft_damping_loss`; `PowertrainDebug::
  shaft_twist_rad[8]`/`shaft_torque[8]`. In car_sedan only
  clutch.out-gearbox.in and gearbox.out-diff.carrier qualify (clutch.in
  and the diff's left/right ports have no inertia of their own).
- `powertrain.h`/`.cpp` (`Powertrain`, one per `VehicleInstance`) - built
  once at vehicle-construction time from `PowertrainDesc`/
  `BrakeSystemDesc`/the wheel list (constructor throws `std::runtime_error`
  on a malformed graph): assigns each component's ports plus one hub port
  per wheel, unions them into shafts (union-find, rejecting duplicate/
  unknown port references), rejects any non-wheel port left unconnected,
  sums each union's inertia, numbers `NodeId`s in first-appearance order,
  builds rigid rows for every `gear_pair`/`differential` (validating a
  lossy gear has exactly one rigid row at its input port) and friction rows
  for the brake system, then constructs the `IComponent` objects
  (engine/gear/differential in `PowertrainDesc::components` order,
  `DirectTorqueBrakes` appended last) - all of this is graph-build-time
  work, never repeated per tick/substep. Any number of differentials (I4
  C4): each topology state emits every diff row in component order, then
  the gearbox row (`build_rigid_rows()` white-box accessor; car_sedan's
  order unchanged). Speedo (`speedo_ratio_`/`speedo_radius_`, used by
  `gearbox_output_ratio_magnitude()`/`approx_speed_mps`): walk down from
  gearbox.out with K(node) = omega/v at no-slip rolling - hub 1/r, diff
  ratio*(K_L+K_R)/2, gear pair ratio*K_out, several estimates at one node
  averaged; if every reached leaf's path ratio and radius are exactly equal,
  `speedo_ratio_ = |path ratio|` and `speedo_radius_` = mean reached radius
  (car_sedan bit-identical), else `1` and `1/|K(gearbox.out)|`. Reset
  (`reset_derivations()`): derived nodes leaves-up, a diff carrier =
  `ratio*((w_L + w_R)/2)`, a gear pair's in = `ratio*w_out`.
  `fill_hub_shaft_torque_debug` runs over every diff; `diff_mesh_torque_debug`
  reads the first. Composed topology states (I4 C5): state index
  `s = g + n_g * sum_i lock_i*2^i` over LOCKABLE diffs (`DifferentialDesc::
  lockable`/`lock` = `DiffLockDesc{max_engage_mismatch_rad_s, channel}`, leaf
  diffs only), checked against `kMaxStates` (128) before any row is built;
  each state's list is diff rows, gearbox row, gear pairs, then one
  `{L:+1, R:-1}` lock row per set bit (lock rows excluded from the
  reconstruction-mode touch counts). `update_lock_bits` (after the gearbox's
  `phase_step`, every substep) engages a requested lock only while
  `|w_L - w_R| <= max_engage_mismatch_rad_s` (else `lock_pending_`, unhashed)
  and releases at once; a gear and a lock change share one
  `apply_topology_switch`. Each lockable diff declares its lock channel
  (default 0) via `declare_control_channels`; `reset()` keeps the lock bits.
  With no lockable diff `s == g` and every row list is byte-identical to
  pre-C5. Per substep,
  `VehicleInstance::step_tick` drives: `begin_substep()` ->
  `pre_solve_components(ctx)` -> per-wheel `WheelInstance::pre_solve` into
  `add_hub_terms` -> `solve(h)` (one `ShaftNetwork::solve` call) ->
  `post_solve_components(ctx)` -> per-wheel `WheelInstance::post_solve`
  reading the hub's new omega back via `hub_omega(i)`, its `wheel_out`/
  `tyre_loss` energy fed back in via `add_wheel_energy`. `begin_tick()`/
  `end_tick()` bracket the whole substep loop once per tick:
  `end_tick()` computes `balance_error` (this tick's ledger closure
  residual - measured ~1e-16 relative on `vehicle_step_steer.json`, far
  under the 1e-9 PLAN.md bound) and resets the running ledger for the next
  tick. `hub_node`/`hub_omega`/`seed_hub_omega`/`wheel_is_driven` are the
  per-wheel read/seed seam `WheelInstance` and `World::wheel_is_driven`/
  `wheel_omega` use; `friction_cap_hits`/`nonspd_hits`/`lcp_stage3_hits`
  and `fold_state_hash` are the counters/hash-fold described under "World
  tick pipeline" above. Every buffer `pre_solve_components`/`solve`/
  `post_solve_components` touches (`Assembly`, the omega snapshots, the
  ledger accumulators) is preallocated at construction
  (`tests/unit/test_vehicle_parallel_alloc.cpp`'s debug_alloc-armed
  braking-only/full-throttle-only/mixed-control phases all check this, PLAN.md D8).
  `Powertrain::declare_control_channels(const PowertrainDesc&, test_only_extra
  = {})` (PHYS-033 P4 I4 C3, static, build-time): returns `std::vector<
  ChannelDecl{name, default_value}>` in component-array order, duplicate
  names merged (first occurrence's default wins); car_sedan and its
  variants declare none; tractor.json declares diff_lock, awd, park_lock,
  pto, pto_speed and hand_throttle. `World::
  create_vehicle` resolves each declared name via the existing
  `ensure_control_channel_index` mechanism into `ControlChannelIndices::
  component`; `VehicleInstance` copies the resolved values once per tick
  (`step_tick`, not per substep) into `component_channel_values_`
  (preallocated at construction, exposed via `component_channel_values()`/
  `World::vehicle_component_channel_values`) and points `ComponentSubstepCtx::
  channels`/`channel_count` at it for the substep loop. `test_only_extra` is
  the test hook: reached through `World::create_vehicle`'s own trailing
  `test_only_extra_channels` parameter (always empty/default for every
  production caller) rather than a throwaway public `ComponentDesc` variant
  or a hand-built `VehicleInstance` bypassing `World` - see
  `tests/unit/test_powertrain_control_channels.cpp`'s header comment.

`drivetrain::PowertrainEnergy` (`ps/drivetrain/energy_ledger.h`): the
mechanical-energy ledger a `Powertrain` accumulates every substep and
exposes per tick via `World::powertrain_energy` -
`engine_out`/`wheel_out`/`tyre_loss`/`brake_loss`/`gear_loss` (component
contributions), `integrator_loss` (the linearly-implicit Euler step's own
`1/2 * delta_omega^T * J * delta_omega` dissipation - not a physical loss
channel, an artifact of the integrator itself), `rotor_ke` (instantaneous
stored rotational kinetic energy, not cumulative), `diff_loss`/
`awd_clutch_loss` (I4 C6/C7 diff slip/viscous and C9 transfer-case clutch
slip, in the balance but not surfaced as telemetry), `pto_clutch_loss`/
`external_absorbed` (I4 C11 pto clutch slip plus stub brake, and work
absorbed by a test load_dyno; in the balance, not telemetry),
the fuel sub-ledger (PHYS-033 P4 I7a C5; cumulative, unhashed, OUTSIDE
balance_error): `fuel_mass` (kg, `h*mdot`, mdot = `fuel_map(rpm at
pre_solve, theta_c)` looked up in pre_solve, booked in post_solve),
`fuel_energy_in` (`fuel_mass*LHV`), `indicated_work` (`h*omega_new*
tau_comb` for EVERY torque-map engine, map or not), `engine_heat`
(`fuel_energy_in - indicated_work`), `fuel_external_added` (kg, for the
future tank) and `fuel_efficiency_violations` (substeps whose engine_heat delta
was < 0, i.e. the map claims > 100 % efficiency; the loader also rejects
such a map at sampled points); an engine with a fuel but no map uses the
crude model (I7a C5d): mdot = max(0, tau_comb*omega_old)/(eta_i*LHV) in
pre_solve, booked through the same path as a map; eta_i = the engine's
`indicated_efficiency` or the ignition default - spark 0.325 (Heywood 1988
section 15.2 p. 827, full load, midpoint of 0.31-0.34), compression 0.48
(section 15.5.1 p. 859, DI diesel at minimum bsfc); it overstates fuel at WOT
(m includes closed-throttle pumping) and understates it at idle; a violation
is possible when omega_new/omega_old > 1/eta_i. An engine without any fuel
books none,
`balance_error` (see
`end_tick()` above). Surfaced as CSV telemetry per vehicle,
`vehicle<i>.energy.*` (eight channels) and `vehicle<i>.powertrain.*`
(`friction_cap_hits`/`nonspd_hits`/`lcp_stage3_hits`, three channels) -
see "World tick pipeline" above for exactly when these are registered/
recorded.

## Hitches (`World::attach`, PLAN.md section 4)

`World::check_attach`/`attach`, by `CouplingDesc` or by the names in the vehicles' `hitch_points` lists,
`detach`, `hitch_alive`, `coupling_state`, `hitch_loads`, `hitch_limits`, `destroy_vehicle`: sim thread
between steps, `std::logic_error` from inside one, `std::invalid_argument` with the measured and allowed
values for a refused attach. `WorldConfig::coupling_passes` (2). A destroyed vehicle keeps its slot, so no
id or channel moves; its snapshot entry has an invalid id and no wheels. Coupled vehicles do not collide
and their wheels do not see each other's chassis. Formats docs/data_formats/hitch.md, vehicle.md. Out of
scope so far: the three-point lift, PTO shafts, a breakaway brake.

## Adapter seams (`ps/world/snapshot.h`, `World::set_control`/`get_control`, PLAN.md P2)

`World::set_ambient(AmbientSample)`/`ambient()` (PHYS-033 P4 I7b C9): sim thread between steps,
applied at the next tick; plumbed via `VehicleTickEnv::ambient` -> `ComponentSubstepCtx::ambient` ->
engine `Sensors::ambient` (`Powertrain::last_engine_sensors_ambient()` is the white-box hook). The graph
engine ignores it (owner ruling Q10), `WorldConfig::air_density` (aero) is NOT derived from it, and it is
never hashed; `tests/unit/test_ambient.cpp` guards both. Weather W4: while an environment with weather is installed, set_ambient/set_aero_environment throw and each vehicle gets its own ambient/density/wind from the environment (see ps_environment).

`World::set_control(name, value)`/`get_control(name)`: a `std::string`-keyed
`real` map (`control_channels_`), resolved to flat indices once per vehicle at
`World::create_vehicle`, so the tick hashes no string. Every vehicle has its own
namespace: `create_vehicle`'s `channel_scope` names it (`trailer_1.brake`), and
without a scope vehicle index 0 reads bare names (`steer`, the designated input
vehicle) and vehicle i `v<i>.` ones, which depend on creation order.
`World::vehicle_channel(id, name)` returns the full name; a caller with several
vehicles scopes them and goes through it. Read every tick by every vehicle
(`"steer"`/`"throttle"`/`"brake"`/`"handbrake"`, see "Vehicle system" below).

`WorldSnapshot` (`ps/world/snapshot.h`) additive fields, filled by
`World::stage_snapshot()`: `articulations` (`std::vector<ArticulationSnapshot>`
- every live articulation's link poses/motions, root first
(`link_index == -1`) then each public link in `ArticulationDesc::links`
order), `vehicles` (`std::vector<VehicleSnapshot>{id, wheels}` - every live
vehicle's per-wheel `WheelState`, PLAN.md M1, in `WheelDesc` order, same
index a wheel has via `wheel_telemetry`/`wheel_state`/`wheel_name`) and
`contacts` (`std::vector<ContactEvent>`, a COPY of this tick's contact
events). All exist because `World::last_contact_events()` and the
live-state accessors (`joint_position`/`link_frame`/`wheel_state`/...) are
safe only from the sim's own thread - `WorldSnapshot` travels through
`SnapshotTripleBuffer::publish`'s existing lock-free hand-off, so a reader
on another thread (PLAN.md D3: "the adapter runs the sim on its own
thread") only ever sees a complete, already-published tick's data, never
a live reference the sim thread might be overwriting concurrently.

`VehicleSnapshot::powertrain` (`ps::drivetrain::PowertrainSnapshot`,
`ps/drivetrain/powertrain_state.h`, PHYS-033 P4 I3 plan section 4.3/7.2):
the adapter-facing per-vehicle powertrain snapshot, filled once per tick
in `World::stage_snapshot()` from `VehicleInstance::powertrain_state()` -
`gear`/`gear_target`/`shift_phase` (`ShiftPhase::Idle/ClutchOpening/
Releasing/Selecting/Synchronising/ClutchClosing`), `engine_rpm`/
`engine_state` (`EngineState::Off/Stalled/Cranking/Running`),
`speedo_mps` (gearbox-output-derived, plan section 7.6's own formula,
not the chassis's raw ground speed), `clutch_engagement` (effective =
min(pedal, auto)), `assist_active_bits` (bit0 auto_clutch, bit1
auto_blip, bit2 auto_shift), `engines` (0 or 1 `EngineSoundState` - crank
angle/load/limiter/dfco/starter flags, sound-only fields excluded from
`fold_state_hash`) and `driveline` (`DrivelineSoundState` - gearbox in/
out omega, gear_load_torque, clutch_slip/torque, grind_active/
grind_events). `World::powertrain_debug(VehicleId)` (`drivetrain::
PowertrainDebug`, by value, tests only - not part of the adapter-facing
snapshot) additionally exposes `clutch_torque`/`clutch_slip`/
`clutch_cap_kinetic`/`clutch_cap_static`/`clutch_stuck`,
`gearbox_mesh_torque`/`diff_mesh_torque`/`hub_shaft_torque[8]`
(the diff row's torque on each hub, `coef*lambda`, 0 for an undriven
hub), `throttle_eff`/`idle_integrator`/`synchro_torque` - wired for real
in I3 commit 7 (`core/src/vehicle/vehicle_instance.h`'s
`powertrain_debug()`, `core/src/drivetrain/powertrain.h`'s
`clutch_cap_kinetic`/`clutch_cap_static`/`clutch_stuck`/
`gearbox_mesh_torque_debug`/`diff_mesh_torque_debug`/
`fill_hub_shaft_torque_debug`/`throttle_eff`/`idle_integrator`/
`synchro_torque_debug`, `core/src/drivetrain/components.h`'s
`Differential::lambda_mesh`/`left_node`/`right_node`/`hub_coef`) -
until then it was a default-constructed stub (`powertrain_state.h`'s own
"filled with defaults only until commit 6/7" comments), which had been
silently masking two real test bugs (`clutch_stuck` always false,
`clutch_slip` always 0) until commit 7's validation tests exercised it -
see `test_powertrain_clutch_lockup.cpp`/`test_powertrain_engine_braking.cpp`
above. Every one of these accessors reads state ALREADY being computed
and hashed for real (`car_sedan.json`/`tractor.json`'s diff and gearbox
are both efficiency < 1, so `lambda_mesh_prev_`/`gearbox_lambda_mesh_prev_`
already reconstruct every substep) - no new hashed state, no physics
change, confirmed by a full `ps_determinism_tests` pass unchanged.

`PowertrainSnapshot::diffs`/`diff_count` (PHYS-033 P4 I4 C8): up to 4
`DiffSnapshot{bias, locked, lock_pending}` in component order - bias =
(T_L - T_R)/(|T_L| + |T_R|), 0 at zero torque; locked = a lock row is active
(kind Locked, or a lockable diff's bit set); lock_pending = C5's flag.
`PowertrainDebug::diff_transfer_torque[4]` (tests only) is each diff's
lock/LSD/viscous transfer torque, and `hub_shaft_torque[8]` now includes it
(the real total shaft torque per hub). Never hashed.
`PowertrainSnapshot::awd_engagement` (I4 C9, the transfer case's `e`, 0
without one) and `PowertrainDebug::awd_clutch_torque`/`awd_clutch_slip`
(tests only). `PowertrainSnapshot::park_lock_engaged` (I4 C10, 0/1) and
`PowertrainDebug::park_lock_torque` (tests only).
`PowertrainSnapshot::pto_engagement`/`pto_mode` (540.0/1000.0, 0 without a
pto)/`pto_rpm` (|omega(out)| in rpm) (I4 C11) and `PowertrainDebug::
pto_clutch_torque`/`pto_clutch_slip`/`dyno_torque` (the first load_dyno;
tests only). `PowertrainSnapshot::governor_setpoint_rpm`/`hand_throttle`
(I4 C12, the governor's n_nl and the hand_throttle value it read this tick,
0 without a governor; never hashed). `PowertrainSnapshot::fuel_flow_g_s` and
`EngineSoundState::fuel_flow_g_s` (I7a C5, this substep's fuel flow - map or C5d
crude model - 0 without a fuel; never hashed). `PowertrainSnapshot::
fuel_mass_kg` (I7a C6, the tank content at the end of the tick, 0 without a
fuel_tank; the hashed state lives in the Powertrain's tank).

The adapter (`adapters/godot/`) consumes both `WorldSnapshot::vehicles`
(wheel state) and `VehicleSnapshot::powertrain` as of I3 commit 8
(`adapters/godot/src/ps_simulation.h`/`.cpp` - `PsSimulation`'s own
snapshot type carries a `ps::drivetrain::PowertrainSnapshot powertrain`
field, filled the same race-free way as the per-wheel state); it still
carries its own duplicate `control_channels_` map rather than reading
`World::control_channels_` directly (an intentional seam, not a gap -
the adapter's own map is what `World::set_control` is called FROM).

## Jolt backend (`core/src/backend/jolt/jolt_backend.cpp`)

`JoltRigidBackend : IRigidBackend`. Two-layer broadphase/object-layer setup
(`NON_MOVING`/`MOVING`). Bodies: box/sphere/capsule/cylinder shapes
(`CylinderShape` added PLAN.md P3, for the vehicle system's wheel contact
shape-cast - see "Vehicle system" above), mass via
`EOverrideMassProperties::CalculateInertia`. Joints: `Fixed`, `Hinge` and `Point`
(`JPH::FixedConstraint`/`HingeConstraint`/`PointConstraint`, world-space anchors/axes computed
from each `JointDesc`'s body-local inputs at creation time; `body_b`
invalid -> `JPH::Body::sFixedToWorld`). Every body's Jolt `UserData` field
carries a packed `ps::BodyId` (index low 32 bits, generation high 32 bits)
so ray/shape-cast and contact-listener results translate back to our own
id in O(1). `IRigidBackend::live_body_ids_into`/`drain_contact_events_into`
(hot-path: write into a caller-owned, already-sized `out` - `World` reuses
`live_body_ids_scratch_`/`last_contact_events_` every tick) back the
by-value `live_body_ids()`/`drain_contact_events()` convenience wrappers
(non-hot-path callers only, both now non-virtual base-class methods).
`BackendContactListener::drain_into` copies `pending_` into `out` and then
`clear()`s `pending_` in place (PHYS-009 Step 3: the previous `drain()`
`swap()`'d `pending_`'s buffer away every tick, so the very next
`OnContactAdded`/`OnContactPersisted` reallocated).
**PHYS-033 I1b (D-C): `with_query_shape(desc, fn)`** (next to `make_shape`) -
builds a Box/Sphere/Capsule/Cylinder query shape ON THE STACK with
`SetEmbedded()` (Jolt's own `VehicleCollisionTesterCastCylinder` pattern),
using the SAME constructor arguments as `make_shape`, so the geometry is
bit-identical to the heap-allocated shape it replaces; only Mesh/Heightfield
(never used as a query/cast shape on any hot path) still fall through to
`make_shape`'s heap allocation. Both `shape_cast` and `collide_shape_into`
call this instead of `make_shape` directly - the fix for defect 3 of
PHYS-033 I1b's "vehicle contact geometry" increment (`measure_contact`'s
per-substep `CylinderShape` cast previously heap-allocated 16 times/tick/
vehicle, invisible to `debug_alloc` until the allocator hook below existed
to see it). `shape_cast` additionally fills `ShapeCastHit::contact_point`/
`penetration_depth`/`started_penetrating` (I1b commit C4, additive) from
Jolt's own `mContactPointOn2`/`mPenetrationDepth`/`mFraction <= 0` - `point`
itself (the cast shape's own origin at the hit) is unchanged.

**PHYS-033 I1b (D-D): the Jolt allocator is now visible to `debug_alloc`.**
`JoltGlobalRegistration`'s constructor, right after `RegisterDefaultAllocator()`
and before any Jolt allocation can happen (the `Factory` is next), wraps
`JPH::Allocate`/`Reallocate`/`AlignedAllocate` (saving and chaining to the
real defaults) so every allocation Jolt itself makes - shape construction,
`PhysicsSystem::Update`'s own internal arrays, anything inside a job -
calls `ps::jobs::debug_alloc::note_external_allocation()` first (same
armed/in-job/re-entrancy rules as the existing global `operator new` hook,
folded into the same `violation_count()`, plus a separate
`external_violation_count()` for attribution - see "Job system" below).
Gated on `debug_alloc::hook_available()` (false under ASan/TSan) and
`#ifndef JPH_DISABLE_CUSTOM_ALLOCATOR` (never defined by this project's
presets). `Free`/`AlignedFree` are left untouched. Found by this: Jolt's
OWN `BroadPhaseQuadTree::UpdatePrepare` (`JPH::QuadTree::UpdatePrepare`,
`new NodeID[mNumBodies]`) allocates on roughly every other tick as part of
its internal incremental broadphase-tree rebuild, even for a fully
settled 2-body scene - a genuine, PRE-EXISTING (since P1) D8 gap this hook
was always going to surface, marked `[!mayfail]` on
`test_vehicle_parallel_alloc.cpp`'s existing zero-alloc case (with the
measured count and full backtrace-derived site in that test's own header
comment) rather than fixed (a Jolt-side broadphase-sizing change, out of
scope for I1b) or hidden.

`IRigidBackend::collide_shape_into` (PLAN.md P1.5b, `ps::articulation`'s
Compliant contact strategy): pure narrowphase overlap query via
`JPH::NarrowPhaseQuery::CollideShape` + `AllHitCollisionCollector`, no side
effects on the backend; every live body overlapping the queried
`ShapeDesc`/`Pose` is appended into caller-owned `out`
(`ShapeContactHit`: body/point/normal/penetration). Default
`JPH::CollideShapeSettings` (`IgnoreBackFaces`): a triangle whose plane
lies beyond the query shape's CENTRE is not reported, so for upward-facing
ground only surfaces at or below the query centre are seen - centre a
"what is under this XY" probe ABOVE every candidate surface. `ray_cast`
(simple `CastRay`) likewise ignores back faces and has been measured to
miss the static feature meshes at some points. `ensure_jolt_registered()`
installs `JPH::Trace`/`AssertFailed`
callbacks that print to stderr (Jolt's own defaults are silent and just
trigger a bare breakpoint). Broadphase/contact-constraint capacity and the
`TempAllocatorImpl` arena size scale with `max_bodies` rather than a fixed
constant.

Heightfield contacts: the constructor installs `PhysicsSystem::SetSimCollideBodyVsBody` (`collide_body_vs_body`); a body-vs-heightfield pair is collided against EVERY overlapping heightfield body (`heightfield_body_ids_`, sorted ascending) with `CollideWithAll`/`CollectFaces` into one `TerrainSeamWeldingCollector` (`JPH::InternalEdgeRemovingCollector`'s algorithm across all tiles, vertex tolerance 1 cm), which drops internal/seam-edge contacts; every other pair uses Jolt's default.

Prebuilt shapes (racing_game R2, merge d1f40ad, out/prebuilt_shapes_progress.md): `IRigidBackend::create_shape(ShapeDesc) const` / `World::create_shape` may be called from ANY thread (also during a step - it touches no backend member, Jolt registration is ordered by `ensure_jolt_registered()`) and return a ref-counted `ShapeHandle` (`JoltPreparedShape` holding a `RefConst<JPH::Shape>`, built by the same `make_shape`/`make_mesh_shape`); `create_body(desc, handle)` (sim thread between steps, `desc.shape` ignored) and `create_body(desc)` both end in one private `add_body`, so the body is bit-identical (`debug_body_shape_digest`, test-only); `Slot::prepared` keeps the shape alive until `free_slot`; a handle may outlive its World. Box/Sphere/Capsule/Cylinder/Mesh only - Heightfield throws `std::invalid_argument` (mutable content); a mesh handle needs a Static body. `World::create_body(desc, handle)` fires `on_static_geometry_changed` like the desc path. BodyIds stay in install order whatever order builders finish. Install cost of a 48000-triangle deck 22.4 ms -> 6.4 us. Tests `tests/unit/test_prebuilt_shapes.cpp` ([backend][world]: equivalence 1/10 workers, order independence, 36 concurrent builds (linux-tsan clean), suppression, debug_alloc with std::thread builders, lifetime; [.][diag] cost).

Phantom wheel casts (D0, merge of 651d8f8): Jolt's default GJK `CastShape` of a nearly axis-aligned cylinder against a very large thin box (the 1000 m test-rig ground) occasionally returns a phantom - fraction 0 / started penetrating with no real overlap, or a miss with the ground inside the sweep [M: 3 of 200000 near-level wheel casts; 0 on 64x64 heightfield tiles and 20 m-cell meshes]. `shape_cast` re-casts only such a SUSPICIOUS result (miss or fraction <= 0) with `mUseShrunkenShapeAndConvexRadius` and takes it only when it is a clean hit (fraction > 0); every other cast is bit-identical. A phantom fed a wheel full bump for one substep (Fz ~540 kN, hub +1300 rad/s, chassis roll 3.3 rad/s). vehicle_tractor_i4.json met one (hash moved). Tests tests/unit/test_dog_release_spike.cpp S1/S2/W1 [backend][vehicle] (sabotage: re-cast disabled -> all fail); hidden diag forces a 1->2 after an LC launch.

Kinematic bodies and doors (branch claude/kinematic-bodies): `IRigidBackend::motion_type`/`is_active`/`move_kinematic(id, target, dt)`/`stop_kinematic` (Jolt `MoveKinematic`: the body gets the velocity that reaches `target` at the end of the next `step`, so contacts see a real velocity; `stop_kinematic` zeroes it and deactivates the body so a door at rest is asleep; invalid/stale id or non-Kinematic body throws `std::invalid_argument`). `World::move_kinematic(id, pose)` is the thin call (sim thread, between steps); `World::create_door(body, DoorDesc)`/`set_door_command(fraction)`/`door_state`/`door_pose_at`/`body_is_active` run a DoorDriver per tick in `World::step` BEFORE the pipeline (sequential, sim thread, `doors_` empty = nothing runs, nothing hashed). Contact detection is `collide_shape_into` of the door's own `probe_shape` at the pose it is about to take (not a sweep: the next pose is one tick of travel away, speed*dt, so overlap at the target equals the sweep result without a cast's phantom-hit issues; not a Jolt contact listener: that reports after the push happened), counting only DYNAMIC bodies (static frames and other doors are ignored); a body heavier than `max_push_mass_kg` blocks at `unpushable_penetration_m` (1 mm) so a car or tractor is never shoved, a light one is pushed up to `max_penetration_m`; moving away from an overlap is always allowed (reopen), a blocked door retries every tick (resumes when the obstacle leaves). Door state (fraction, command, moving, blocked) is folded into `state_hash` only while a door exists. Kinematic bodies are not Static, so terrain suppression ignores them. Tests tests/unit/test_kinematic_door.cpp K1-K10 [kinematic].

`IRigidBackend::is_static` (PLAN.md M1/PHYS-021 - see "Terrain/static-feature
coincident-collision suppression" above): `body->IsStatic()` behind the
same `TryGetBody(lookup(id))` null-safety pattern as `get_mass`/
`get_inertia_world`.

`IRigidBackend::set_integrate_sub_threshold_rotation(BodyId, bool)`
(parking-slope creep fix D2, default off per body): opts a body in to
`JoltRigidBackend::step()` reapplying, single-threaded right after
`PhysicsSystem::Update`, the rotation Jolt's own `Body::AddRotationStep`
(`Jolt/Physics/Body/Body.inl`) silently drops when `|omega*dt_substep| <=
1.0e-6f` rad, via Jolt's own `Quat::sRotation` about the body's centre of
mass. `World::create_vehicle` opts the chassis body in; nothing else does.

`IRigidBackend::shape_cast_excluding`/`ray_cast_excluding` take a list of bodies a query ignores;
`set_disabled_collision_pairs` installs a sorted list of body pairs that never collide, a Jolt `GroupFilter`
at the broad phase (each listed body gets a group id on first use, the filter is created lazily because
`JPH::Ref` needs the allocator `ensure_jolt_registered()` installs). Both exist for hitches: a real drawbar
reaches under its tow vehicle.

`IRigidBackend::get_integration_params` (PHYS-033 I1 W1, `BodyIntegrationParams`
in `ps/backend/i_rigid_backend.h`): `inv_mass`, `inv_inertia_world` (at the
body's CURRENT rotation - Jolt itself uses the rotation at step START),
`gravity_factor`, `linear_damping`/`angular_damping`, `gyroscopic`
(`GetApplyGyroscopicForce()`) - same `TryGetBody`/null-safety pattern as
`get_mass`/`get_inertia_world` above. Read-only, no side effects. Consumed
by `VehicleInstance::step_tick`'s shadow chassis (see "Vehicle system"
above); `tests/unit/test_body_integration_params.cpp` cross-checks it
against the box-inertia formula.

`IRigidBackend::body_triangle_xy_bounds_into` (PHYS-021 Task B, load-time
only - allocates): flattens the body's shape into leaf `TransformedShape`s
(`CollectTransformedShapes`) and walks `GetTrianglesStart/Next` relative to
the body's COM, returning each triangle's world XY bounds (convex primitives
come back triangulated). Used by `TileManager::precompute_suppression` to
bin static geometry to tile keys.

`MeshShape`/`HeightfieldShape` (PLAN.md M1): `SurfaceMaterial :
JPH::PhysicsMaterial` carries one `SurfaceId`; `shared_surface_material_list()`
is a single 256-entry `JPH::PhysicsMaterialList` (slot 255 reserved for
`kInvalidSurfaceId`) reused by every heightfield shape, so
`update_heightfield_body` (the hot tile-streaming path) never builds a new
material list. `make_mesh_shape` instead builds its OWN small local
material list per mesh (`JPH::MeshShape` caps at 32 materials, unlike
`HeightFieldShape`'s 256) from only the distinct surface ids that mesh
actually uses. Jolt's native heightfield convention is Y-up
(`pos = offset + scale*(x, height, y)`); this backend keeps `mScale`
strictly POSITIVE on every axis (negating an axis to "flip" it also flips
triangle winding/normals, which desyncs `ray_cast`, which back-face-culls,
from `collide_shape_into`, which does not - a bug once caught by exactly
that asymmetry) and instead REVERSES which sample row lands at which Jolt
row index (`reverse_heightfield_rows_into`/
`reverse_heightfield_material_rows_into`), compensated by a constant
position offset on the wrapping `RotatedTranslatedShape`
(`heightfield_wrapper_position`, `kHeightfieldYupToZup` - a fixed +90°
about local X, written as an exact literal so the D7 transcendental grep
never has to see a runtime trig call for a fixed geometric constant).
`update_heightfield_body` (`IRigidBackend`) calls Jolt's own
`HeightFieldShape::SetHeights`/`SetMaterials` through this backend's own
`temp_allocator_` (safe to reuse: `terrain_tiles` runs strictly after
`backend_step` in `World`'s pipeline) with grow-once-never-shrink
float/uint8 scratch conversion buffers - zero-alloc once warmed up, same
pattern as `AbaScratch` elsewhere in this codebase. `sample_count` must be
EVEN (`HeightfieldShape`'s own doc comment, `ps/backend/shape_desc.h`):
Jolt rounds `mSampleCount` up to a multiple of `mBlockSize` (default 2) at
construction time, and `SetHeights`/`SetMaterials` require the caller's
arrays to match that exact (possibly-rounded) count.

`RigidBackendSettings` (`ps/backend/i_rigid_backend.h`, passed to
`create_jolt_backend`) exposes `JPH::PhysicsSettings` fields (`baumgarte`,
`num_velocity_steps`, `num_position_steps`, `penetration_slop`,
`speculative_contact_distance`, `max_penetration_distance`,
`linear_cast_threshold`, `linear_cast_max_penetration`) plus
`collision_steps` (`PhysicsSystem::Update`'s own `inCollisionSteps`
substep-count parameter, stored as `JoltRigidBackend::collision_steps_`
and applied in `step()`). `World` forwards its own same-named
`WorldConfig` fields into this struct in its constructor
(`core/src/world/world.cpp`). Every default matches Jolt's own
`PhysicsSettings`/`BodyCreationSettings` default, so a default-constructed
`WorldConfig`/`BodyDesc` reproduces pre-existing behaviour exactly.

`BodyDesc` additionally carries `motion_quality` (`ps::MotionQuality::
Discrete`/`LinearCast`, mapped to `JPH::EMotionQuality`),
`max_angular_velocity` (`MotionProperties::mMaxAngularVelocity`, default
`0.25*pi*60` rad/s) and `max_linear_velocity`
(`MotionProperties::mMaxLinearVelocity`, default 500 m/s - Jolt silently
clamps to this every step; see the P1 Part A report for how this
produced an apparent energy loss in a long free-fall scenario before it
was traced back to the clamp).

## Building, testing, running

```powershell
# from a VS 2022 developer shell (tools\ci.ps1 sets one up automatically)
cmake --preset debug
cmake --build --preset debug
ctest --preset debug --output-on-failure
.\out\build\debug\tools\ps_run\ps_run.exe data\scenarios\free_fall.json
.\out\build\debug\tools\sweep\sweep.exe data\scenarios\*.json --jobs 4
```

```bash
# inside WSL
cd /mnt/s/claude_code/physics_sim
cmake --preset linux-release
cmake --build --preset linux-release
ctest --preset linux-release --output-on-failure
```

## CI

Three levels - use the smallest that fits (also at the top of `tools/ci.ps1`):

```powershell
tools\ci.ps1 -Affected            # PER CHANGE: topics from the changed paths (vs HEAD), release leg only, unit + determinism tests only
tools\ci.ps1 -Affected -Base HEAD~3   # ... covering the last 3 commits too
tools\ci.ps1 -Topic terrain,vehicle   # ... topics picked by hand
tools\ci.ps1 -Topic drivetrain -WithValidation   # ... plus the topics' non-[long] validation tests
tools\ci.ps1 -Affected -PlanOnly  # print the decision only, build/test nothing
tools\ci.ps1                      # DEFAULT FAST: release + msvc build + linux-release unit/determinism + hash compare; per merge batch (< 3 min)
tools\ci.ps1 -Sanitizers          # default + debug, asan, linux-asan-ubsan, linux-tsan: batches touching threading/memory, and before -Full
tools\ci.ps1 -Full                # every leg, every test: before a phase counts done, after foundational changes
tools\ci.ps1 -SkipWsl             # Windows legs only
tools\ci.ps1 -SkipWindows         # WSL legs only
tools\ci.ps1 -Only debug,asan     # named legs only (matches preset names, plus msvc-release)
tools\ci.ps1 -NoOverlap           # old strictly sequential layout (Windows lane, then WSL lane)
```

**`-Affected` / `-Topic`** (`tools/ci.ps1`'s `$script:PathRules`,
`$script:ScenarioTopics`, `$script:SensitivePatterns`): changed paths
(`git diff --name-only <Base>` + untracked) map to test topics through an
explicit, ordered path table; test files map through their own Catch2
topic tags. Runs the `release` leg with `ctest -L '^(topics)$'` and
`-LE 'validation|long|walltime'` (unit + determinism tests only;
`-WithValidation` drops `validation` from the exclusion), the two
greps, the topics' `walltime` benches (alone), and the hash compare for the
topics' scenarios only (building just the Linux `ps_run` target for it).
Adds `linux-release` (unit + determinism) (+ `linux-tsan` for threading) only when the changed
core/tools lines touch unordered containers, sorting, libm/FP-contract
math or threading, or with `-Linux`. Foundational paths (core
`math`/`jobs`/`world`/`backend`, any `CMakeLists.txt`/`CMakePresets.json`,
`tools/ci.ps1`, `tools/tsan_suppressions.txt`, `tools/common`,
`tests/support`) run the default fast run and ask for `-Full`; **a path no
rule knows also falls back to the default fast run, never to nothing**;
doc-only changes (`*.md`, plus stray root `*.csv` / `backup_*/` files) run the greps only; `adapters/**` notes that
`adapters/godot/smoke_test.ps1` is the check. One narrow exception to the
"any `CMakeLists.txt` is foundational" rule:
`tests/{unit,validation,determinism}/CMakeLists.txt` gets its own
`cmake-test-registration` path-rule `Kind`, checked ahead of the generic
`CMakeLists.txt` rule - if the diff's added/removed lines are NOTHING BUT
bare `test_*.cpp`/`.h` source-list entries (`$script:CmakeTestLineAllow`,
registering a new test alongside adding its `.cpp` file), it contributes no
topics of its own instead of forcing the default fast run; the sibling test
file's own Catch2 tags (the ordinary `testfile` rule) decide what runs.
Any other change to that file - `target_link_libraries`, `add_executable(`,
compile definitions, `catch_discover_tests`, a plain comment edit, anything
outside that narrow allow-list - still falls back like every other
`CMakeLists.txt` edit. Every decision is printed with the path that caused
it. Adding a subsystem = add a topic tag to its tests, a `$script:PathRules`
row, and (if it has scenarios) `$script:ScenarioTopics`.

**Default/Full layout (overlapped).** Phase A: the Windows lane (`release`,
`msvc-release`, greps; `debug` and `asan` only with `-Sanitizers`/`-Full`)
runs in `ci.ps1` itself while the WSL lane runs in a child `powershell -File
tools\ci.ps1 -WslLaneOnly` (results handed back as JSON under `%TEMP%`);
inside the WSL lane, `linux-release` runs first, then (with `-Sanitizers`/
`-Full`) `linux-asan-ubsan` and `linux-tsan` build
one after the other and run their ctest side by side, `--parallel 12` each
(scripts written to `out/ci_tmp/`, logs to `~/ps_ci_<preset>.log`). Every
phase-A ctest excludes the `walltime` label. Phase B starts only after both
lanes have exited: each built leg's `walltime` tests, one leg at a time,
nothing else running. Then the cross-platform hash compare. The summary adds
a per-phase wall-clock table.

WSL lane step `linux-aarch64-check` (after `linux-release`, also `-Only linux-aarch64-check`): cross-builds the aarch64 preset and, with qemu, runs free_fall and reports its hash vs linux-release (informational; all 20 HashCheckScenarios matched x86 bit for bit on 2026-10-12, not a requirement); SKIP (passing) when the cross headers are absent, FAIL on a configure/build error.

Legs: clang-cl `debug`, `release`, `asan`; cl `msvc-release`; a grep that
fails the run if anything under `core/` includes a Godot/GDExtension header;
WSL `linux-release`, `linux-asan-ubsan`, `linux-tsan` (its test preset sets
`TSAN_OPTIONS` to load `tools/tsan_suppressions.txt` and to symbolize via
`/usr/bin/addr2line` - TSan's own search finds a dangling
`llvm-symbolizer-21` in WSL, and suppressions only match symbolized
frames; applies to a hand-run `ctest --preset linux-tsan` too). Prints a per-leg
PASS/FAIL summary with per-stage (configure/build/ctest) timing and a
per-leg + total wall-clock line; exits non-zero on any failure. Windows
ctest uses `--parallel 12`, `linux-release` 24. The PASS/FAIL lines exist
only in the final summary (cmake/build output is swallowed by
`Measure-Command`), so capture the whole run (`*>`) and let it finish.
The cross-platform hash compare runs `data/scenarios` only, none of which
loads `data/tracks/test_ground` (`terrain_multi_interest.json` covers
multi-interest-point streaming plus suppression on the small synthetic
spec instead). A stale build-dir cache that lost its optimisation flags
fails configure on purpose (top-level `CMakeLists.txt`; fix with
`cmake --preset <name> --fresh`).

**Test selection** (2026-09-27 cut, owner request "cut down stronger on the
amount of tests run frequently"). Default FAST: `release` runs every test
except `[long]`; `msvc-release` is BUILT only (compiler coverage, no ctest,
no phase-B walltime); `linux-release` runs `ps_unit_tests` +
`ps_determinism_tests` only (`-LE "validation|long"`; what Linux adds is
the compiler/libstdc++ and the hash compare, the physics is the same code);
the `debug`, `asan`, `linux-asan-ubsan` and `linux-tsan` legs do not run
(`-Sanitizers` adds them; `-Only <leg>` naming one runs it too). When they
run they take only `ps_unit_tests` + `ps_determinism_tests` +
`[sanitizer]`-tagged tests (`-LE "validation|long"` - the whole
`ps_validation_tests` suite is `[validation]`-tagged, and the `[sanitizer]`
short variants deliberately omit that tag so this one exclusion routes them
back in). `-Affected`/`-Topic`: the topics' unit + determinism tests only
(`-WithValidation` adds validation). `-Full`: no long/validation filter,
every test on every leg (mandatory before a phase counts done per PLAN.md
section 7). `walltime` is excluded from phase A in all modes and run in
phase B.

**Test tags / labels** (`tests/*/CMakeLists.txt` pass `ADD_TAGS_AS_LABELS`
to `catch_discover_tests`, so every Catch2 tag becomes a CTest `LABEL`
selectable via `ctest -L`/`-LE`; repeated `-L` are ANDed):
- **Topic** (every test has at least one; the benches get theirs via
  `add_test` `LABELS` in `tests/bench/CMakeLists.txt`): `[math]`, `[jobs]`,
  `[world]`, `[backend]` (Jolt-backed rigid bodies, shapes, the `[hello]`
  self-check), `[articulation]`, `[contact]`, `[terrain]`, `[vehicle]`,
  `[io]`, `[cycle]` (engine cycle model, pipes, generator), `[vvl]` (variable valve lift, garage P9), `[audio]` (ps_audio), `[machine]` (hitches between vehicles),
  `[determinism]`, `bench`. Multiple topics per test are normal.
- `walltime` (CTest label, benches only): set by `tests/bench/CMakeLists.txt`
  exactly where the bench ASSERTS timing on that build -
  `bench_force_elements` on every non-Debug build type,
  `bench_terrain_tiles` on every non-sanitizer build - together with
  `PROCESSORS 24`. Resulting set: `release`/`msvc-release`/`linux-release`
  both, `debug` `bench_terrain_tiles`, `asan` `bench_force_elements`, the
  WSL sanitizer legs none.
- `[long]`: a physics test that simulates tens of seconds (60 s energy-drift/
  momentum tests, 30-35 s mass-ratio/resting-contact tests) - cheap in a
  release build but 5-15x slower under a sanitizer, where it would blow the
  fast-CI budget. Never used on a test needing under about 2 s in release.
  Also every non-determinism test taking >= 5 s in release (2026-09-27: the
  two terrain seam-crossing cases, the compliant clutch lock-up, split-mu
  braking open and the hub-torque cross-check, the tractor MFWD drawbar pull,
  the PTO governor load sweep) - they set the release ctest's wall time;
  they run under `-Full` only.
- `[sanitizer]`: a short (1-3 s simulated) hand-written variant of a
  `[long]` Jolt/job-system-backed test (`test_joint_chain_energy.cpp`,
  `test_mass_ratio.cpp`, `test_resting_contact.cpp`), reusing the same
  thresholds (all three are steady-state/bounded-oscillation quantities, not
  ones that grow with run length - see each variant's comment) so the
  sanitizer legs still exercise the Jolt/job-system path without the full
  60 s cost. Pure-articulation `[long]` tests (energy, floating-base,
  four-bar, mass-ratio) got no `[sanitizer]` variant: articulation stepping
  never touches the job system (`tests/determinism/test_worker_count_hash.cpp`'s
  comment), so a short variant would add no TSan-relevant coverage beyond
  what the always-run white-box unit tests (`test_articulation_core.cpp`,
  `test_articulation_four_bar.cpp`) already give.
- `[heavy]` (`tests/determinism`): a test that spins up its own `JobSystem`
  with up to 24 workers; given CTest `PROCESSORS=24` so one ctest invocation
  never schedules it alongside another of its own tests (the two WSL
  sanitizer ctests do run side by side - that only slows these, their
  hashes do not depend on scheduling).

## Dev environment setup

```powershell
tools\setup_dev_env.ps1             # install everything (idempotent)
tools\setup_dev_env.ps1 -CheckOnly  # verify only, prints versions, exits non-zero if anything is missing
tools\setup_dev_env.ps1 -Aarch64Cross [-CheckOnly]  # optional: WSL aarch64 cross headers (libstdc++/libc6-dev-arm64-cross) + qemu-user, via wsl -u root
```
