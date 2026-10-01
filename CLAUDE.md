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
    ps_engine_audio/                  ps_audio A10 listening tool: `ps_engine_audio <engine.json> --wav out.wav --script events.json [--installation f] [--seconds S] [--seed N] [--map-workers N] [--engine-cache DIR]` or `--live` (miniaudio, FetchContent, tool-only); hosts ps::EngineRig + a ps_audio Voice (thin glue), own WAV writer; scripts in data/scripts/engine_audio/ (free_rev.json, stall.json)
    sweep/                            fans N scenarios out over local ps_run processes (--jobs N); target sweep
    plots/                            python telemetry tools, dev only: rpm_to_wav.py (stdlib-only crude 4-rotor synth from a t/engine_rpm CSV, firing 4*rpm/60 - the 787B lift-off preview; pulse shape/resonance are artistic, labelled so; optional --fm-reference REF.csv --fm-exaggerate G magnifies the rpm difference to a reference run, labelled EXAGGERATED, a listening aid only - default output unchanged)
    enginedata/                       python engine-data generators, dev only: a25a_mb_engine.py (PHYS-033 P4 I7c C17a fallback M-B: EPA NVFEL A25A xlsx -> data/engines/toyota_a25a_fks_sim.json; --sabotage {transpose_ve,reverse_ve_rows,kpa_as_pa} for the data test's corruption check); a25a_gen_engine.py (I7c C17a M-A: out/a25a/a25a_tables.json -> data/engines/toyota_a25a_fks_gen.json's cycle block, deterministic, same bytes on re-run)
    assetgen/                         Godot art-asset generator (Python; owned by a separate session, do not touch)
  core/
    include/ps/                       public API headers (no Jolt or Godot types); target ps_core (STATIC)
      version.h                       ps::version_string(), ps::jolt_backend_self_check()
      types.h                         ps::real = double; id_index_t/id_generation_t; SurfaceId (uint16_t) + kInvalidSurfaceId
      math/                           vec3.h, quat.h, mat3.h, pose.h (Pose/Motion/Wrench incl. Wrench::operator*(real)), simd.h (f64x4 skeleton)
      jobs/                           job_fn.h (JobFn), job_system.h (JobSystem, WaitGroup, parallel_for/parallel_batches), task_graph.h, debug_alloc.h
      world/                          ids.h (BodyId/JointId/ForceElementId/ArticulationId), frame.h (Frame, ILinkStateSource, ArticulationLinkRef), force_element.h (IForceElement), snapshot.h (WorldSnapshot incl. ArticulationSnapshot/contacts, SnapshotTripleBuffer), world.h (World, WorldConfig), ambient.h (AmbientSample{pressure_pa, temperature_k}, ISA sea-level default - PHYS-033 P4 I7b C9)
      backend/                        i_rigid_backend.h (IRigidBackend, BodyDesc/JointDesc, ShapeContactHit, ContactEvent, BodyIntegrationParams (PHYS-033 I1), create_jolt_backend()); shape_desc.h (ShapeDesc/BoxShape/SphereShape/CapsuleShape/MeshShape/HeightfieldShape - split out of i_rigid_backend.h so articulation_desc.h can use ShapeDesc without pulling in the backend's JointType)
      articulation/                   articulation_desc.h (ArticulationDesc incl. contact_strategy, RootDesc, LinkDesc incl. contact, JointType, JointLimits, VelocityActuator, LoopClosureDesc/LoopClosureType, ContactStrategy, ContactMaterial, LinkContactDesc) - descriptors only, no algorithm code
      terrain/                        terrain_source.h (ITerrainSource, TileKey, TileSample), terrain_config.h (TerrainConfig - pool sizing only, NOT grid resolution, see "Terrain tile system" below), procedural_terrain.h (load_procedural_terrain_source() factory)
      vehicle/                        public, additive descriptors/seams only (PLAN.md 5.1/P3, no algorithm code - see "Vehicle system" below): vehicle_desc.h (VehicleDesc/WheelDesc/SuspensionDesc/SteeringDesc - PHYS-033 I2 commit 5a: BrakeDesc/DriveDesc/WheelDesc::driven are gone, replaced by VehicleDesc::brake_system/powertrain below), curve1d.h (Curve1D - split out of vehicle_desc.h in I2 commit 5a so drivetrain/powertrain_desc.h can use it without a vehicle_desc.h <-> powertrain_desc.h include cycle), tyre_desc.h (TyreDesc, Pacejka MF5.2/6.1-subset coefficients, low_speed_damping_ratio/low_speed_damping_vx - PHYS-033 I1, rolling_resistance_v_eps - parking-slope creep fix D1), tyre_model.h (ITyreModel, TyreSlipInput/TyreForces, TyreForcesWithSlope, evaluate_with_slope(), create_pacejka_tyre_model(), SlipRelaxationState/step_slip_relaxation()/relaxation_step_axis() - PHYS-033 I2), wheel_telemetry.h (WheelTelemetry, returned by World::wheel_telemetry()), wheel_state.h (WheelState, returned by World::wheel_state() and carried in WorldSnapshot::vehicles - PLAN.md M1)
      drivetrain/                     public, additive descriptors only (PHYS-033 P4 I2 plan section 2, no algorithm code - see "Vehicle system" below): powertrain_desc.h (PortRef/ShaftDesc/EngineLimiterDesc/TorqueMapEngineDesc/GearPairDesc/DifferentialDesc/ComponentDesc/PowertrainDesc; I7a C3: ThrottleInput, TorqueMapEngineDesc::throttle_input, ThrottleControlDesc; I7a C6: FuelTankDesc (a ComponentDesc variant), FluidLinkDesc, PowertrainDesc::fluid; I7b C10: SimulatedEngineDesc (a ComponentDesc variant, an engine file's "kind": "simulated" - docs/data_formats/engine.md)), brake_system_desc.h (BrakeCornerDesc/HandbrakeDesc/BrakeSystemDesc), energy_ledger.h (PowertrainEnergy; I7a C5 adds the unhashed fuel sub-ledger, see "Drivetrain"; I7b C12 pumping_work/friction_work - cumulative, unhashed, outside balance_error, 0 from the graph engine), table2d.h (Table2D - PHYS-033 P4 I7a C1: header-only bilinear 2D lookup, row-major values[ix*y.size()+iy], clamped at every edge like Curve1D, dvalue_dx/dvalue_dy with Curve1D's slope convention (interior knot = left segment, 0 at/beyond an edge), validate(); for the I7a fuel map), engine_maps.h (PHYS-033 P4 I7c C16a: EngineMaps, MapQuality Full/Preview, EngineMapOptions{quality, pool, cache_dir (I7c C16b: opt-in .psmaps disk cache, empty = no disk access), generation_count_out test hook}, EngineMapGenTask, generate_engine_maps, engine_maps_content_hash, kGeneratorVersion 2 (ps_audio A5: kLumpedGeneratorVersion 1 for engines without pipe blocks - maps and cache keys bit-identical to v1 - and kPipeGeneratorVersion 3 (A8b: catalyst monolith + inflow fix; was 2); engine_map_generator_version(desc) picks one, the cache key and the load use it)), installation_desc.h (ps_audio A4: InstallationDesc - intake airbox/snorkel and the exhaust element chain (pipe, catalyst, silencer, branch, outlets) downstream of the engine's first flange; loaded from data/installations/*.json, physics_sim.installation/1, docs/data_formats/installation.md; engine_cycle_desc.h gained IntakeCycleDesc/ExhaustCycleDesc (runners, primaries, merge DAG), ResolutionCycleDesc and CycleModelDesc::{intake, exhaust, default_installation_path, installation, installation_path, resolution} - data only until A5), fuel_desc.h (FuelDesc/FuelPhase - PHYS-033 P4 I7a C4: name, phase (liquid only), lhv_j_kg, afr_stoich, density_kg_m3, source; loaded by ps::io::load_fuel_json)
      io/                             telemetry.h (TelemetryRegistry, CSV via IO thread), scenario.h (Scenario incl. ScenarioArticulation/ScenarioTerrain/ScenarioVehicle/ScenarioControlEvent, load_scenario, build_scenario), surface_table.h (SurfaceTable), test_ground_loader.h (load_test_ground_collision()), fuel_io.h (load_fuel_json() - physics_sim.fuel/1, PHYS-033 P4 I7a C4: every field required, phase must be "liquid", numbers > 0 and finite, unknown keys ignored), vehicle_io.h (load_tyre_json(), load_vehicle_json() - physics_sim.vehicle/2 as of PHYS-033 I2 commit 5a, see "Vehicle system" below; I7a C5: an engine file's "kind" dispatches - "torque_map" (or absent) loads, "simulated" is rejected until I7b; optional "fuel_map" (Table2D, g/s over rpm x theta_c) + "fuel" (relative path to a physics_sim.fuel/1 file); I7a C5d: "fuels" (list of relative path strings, the first is burnt until C6; "fuel" is shorthand for a one-entry list; object entries, both keys, an empty list, a duplicate file (std::filesystem::equivalent) and "fuel_map" with "fuels" rejected), "ignition" ("spark"/"compression", required on any fuelled engine - an ENGINE property, fuel files carry none) and optional "indicated_efficiency" in (0,1) (rejected with a fuel_map) - docs/data_formats/engine.md; ps_audio A4: a cycle block's intake/exhaust pipe pair (both or neither; mutually exclusive with runner_length_mm/runner_diameter_mm, which are then zero - A5 made CycleStepper pipe-aware: no lumped runner, its ports take constant per-step sources from the PipeNetwork port flux), resolution (CFL floor checked against literal copies of the pipe wave-speed/step constants), default_installation (required with pipes, rejected without); load_installation_json(); a vehicle's per-engine "installation" override, legal only on a simulated engine with pipe blocks)
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
      drivetrain/                     INTERNAL (ps::drivetrain, not a public header); PHYS-033 P4 I2 shaft-network powertrain, see "Vehicle system" below and the "Drivetrain" section: dense.h (SmallVec/SmallMat, Cholesky), box_lcp.h/.cpp (F1 box-LCP: block principal pivoting + Murty fallback), nullspace.h (build_nullspace), network.h/.cpp (NetworkBuilder, Assembly, ShaftNetwork), throttle_stage.h (ThrottleStage - PHYS-033 P4 I7a C3: the typed throttle-command stage, see "Drivetrain"), energy_store.h (IEnergyStore/FuelTank - PHYS-033 P4 I7a C6: header-only lumped store, content/capacity/withdraw/set_content/fold_state_hash; a future I9 battery implements the same interface), engine.h (IEngine - PHYS-033 P4 I7a C2: pre_solve/post_solve/idle_integrator/crank_angle/idle_rpm/fold_state_hash/reset - plus the engine's Sensors/TcuRequest/PreSolveResult (incl. theta_c, the post-limiter combustion fraction, published as EngineSoundState::load)/PostSolveResult at namespace level; Powertrain holds unique_ptr<IEngine>, EngineTorqueMap in engine_torque_map.h/.cpp and (I7b C11) SimulatedEngine in engine_simulated.h/.cpp - the mean-value engine model (throttle/manifold/VE/eta_i/spark/pumping/friction tables from a SimulatedEngineDesc; C12: built from a desc copy whose fuels = the burnt fuel, per-engine ambient-dependent SimulatedSteadyTables refilled in place (allocation-free, ~68 ms) when Sensors::ambient changes exactly - callers must quantise ambient; a throttle_control kind differing from its throttle_input throws; rotary still refused until I7e C13)), components.h/.cpp (IComponent, TorqueMapEngine, GearPair, Differential (I4 C4, was OpenDifferential), TransferCase (I4 C9), ParkLock (I4 C10), Pto/LoadDyno (I4 C11), DirectTorqueBrakes - I2 commit 5b), powertrain.h/.cpp (Powertrain - builds the node/rigid-row/friction-row graph from a vehicle's PowertrainDesc/BrakeSystemDesc/WheelDesc list, drives the per-substep pre_solve/solve/post_solve sequence, owns the PowertrainEnergy ledger - I2 commit 5b)
      engine_cycle/                   ps::engine_cycle (PHYS-033 P4 I7c C15a; ps_audio A1: the live-model files geometry/cycle_stepper/cycle_point/firing_order .cpp build as target ps_engine_cycle, their headers (plus gas.h, heat_transfer.h, valve_flow.h, wiebe.h, firing_order.h) are public under core/include/ps/engine_cycle/; map_generator.cpp and map_cache.cpp (map_cache.h stays internal here) build into ps_core because they need JobSystem and file IO; firing_order.h: validate_firing_order/validate_firing_angles/resolve_firing_angles_deg - explicit firing_angles_deg is indexed by CYLINDER number, not firing position; ps_audio A2: pipe_network.h/.cpp - PipeNetwork, the quasi-1D finite-volume pipe core shared by maps (A5) and audio (A7), no consumer yet: MUSCL-Hancock + HLLC with a well-balanced area source and a passive burned fraction, Fanning friction + Reynolds-analogy heat, one characteristic pressure-reservoir face for Open/Volume/Port ends (Port = the cylinder hand-off: set_port_state before step, port_flux after), end correction as a pipe extension, explicit conservative 0D volumes, orifices only between volumes, build-time CFL checks throwing "PipeNetwork: ..."; A8b: a runtime CFL check per pipe per step (max |u|+a) sub-cycles that pipe n = ceil(h*s/(cfl*dx)) times at h/n, n clamped to [2, kMaxPipeSubcycles = 8] [A] (silently saturates above 8), volumes/ports/WG see start-of-step state and the end faces the time-averaged flux, subcycled_pipe_steps() is an unhashed diagnostic, a network that never exceeds it is bit-identical - a supersonic intake backflow jet after a WOT hold (|u|+a ~1260 m/s vs the assumed 500) had blown up the live model; the open-end reservoir face matches p and u on INFLOW (Newton, <= 8 iterations) - the old interior-Riemann-invariant closure at the reservoir sound speed made a velocity deadband at u = 0 whenever pipe and ambient temperatures differed; 74 ns per cell-step with wall terms (~0.57 core for an I4's audio-grade pipes, so A9 SIMD matters); pipe_network_types.h holds the enums/constants shared with the waveguide; ps_audio A3: waveguide_network.h/.cpp - WaveguideNetwork, the linear downstream pipe model: segments = delay ring + 6-shelf Kirchhoff wall-loss IIR + Thiran allpass, Norton junctions with WDF compliance and a flow source, Closed/Open (Levine-Schwinger radiation, one-pole)/Anechoic ends, a Resistance 2-port linearised with the incremental R (catalyst/silencer loss), LaminarFriction as a series R lumped at Junction/Closed ends (a distortionless exp(-alpha*L) gain halves a Helmholtz neck's decay); 20.9 ns per sample per segment (9.6 lossless). pipe_network's PipeEndDesc::Kind::Waveguide couples an FV pipe end to a WG Junction: the WHOLE flow crosses as one more junction port of admittance A/(rho*a) on the MUSCL-Hancock half-step state (a running-mean split bypassed the WG impedance below 1/tau - rejected), p_ref = ambient + mean_pressure_drop - linearised_mean_drop restores the quasi-steady resistance drop; the WG is linear about a fixed mean, so the split sits at the exhaust flange and upstream of the throttle; WG state hashed only when a WG exists; ps_audio A5: cycle_engine.h/.cpp - CycleEngine, n CycleSteppers (pipe mode set_port_sources/step_phased; reservoir reference mode set_port_reservoirs) in A1 firing phases coupled to one PipeNetwork from make_engine_pipe_network_desc (intake: snorkel -> airbox -> throttle orifice -> plenum -> runners -> ports, no snorkel = throttle from ambient; exhaust: ports -> primaries -> merge junction volumes (sum of pipe areas x 0.1 m [AS]) -> flange, then a WG chain from the installation or an Open end); per step set_port_state -> network step -> constant port sources -flux/h -> RK4 per cylinder (mass closure <= 1e-13); flow-balance throttle trim on the cycle-mean plenum pressure once per cycle; 8 Full / 6 Preview settling cycles; one CycleEngine per grid point built inside the job, run() single-use and allocation-free; ~12 us per step at dx 50 mm, 0.22 s per Full point (12x20 Full 61.9 s on 1 worker / 7.2 s on 10); an unreachable p_im at a grid node throws; the loader resolves the installation (vehicle override or engine default) BEFORE generation; ps_audio A7 split cycle_engine into EngineCore (phased steppers, pipe network + layout, throttle Cd*A, port<->pipe exchange, radiating_ends()/radiating_outflow() built in the ctor, never read by the generator) shared by CycleEngine (generator policy) and ps_audio's LiveEngine, plus combustion_variation.h (header-only counter-hash cycle-to-cycle variation, audio only) and the CycleStepper live API set_speed/step_live/set_combustion_event/set_charge (audio only)): 0D single-cylinder cycle model between fixed intake/exhaust reservoirs - geometry.h/.cpp (slider-crank with pin offset), gas.h ((m, U, m_b) state, linear cv from NIST-JANAF fits, U referenced to 298.15 K, closed-form T), valve_flow.h (choked/unchoked/reverse, Psi linear above pressure ratio 0.99, gamma blended by burned fraction), heat_transfer.h (Woschni), wiebe.h (a 5, m 2), cycle_stepper.h/.cpp (fixed-step RK4 on ps::math, integer steps per cycle), cycle_point.h/.cpp (one operating point: h_full 1/(960*48) s, h_preview 1/(960*24) s, N 5/4 cycles - PROVISIONAL, RK4 stability edge 30-36 us at 1000 rpm / 30 kPa, larger valve CdA moves it down); descriptor ps/drivetrain/engine_cycle_desc.h (A1 adds firing_order/firing_angles_deg/banks/bank_angle_deg - loaded and validated, firing_order default 1-3-4-2 only for 4 cylinders, else required - firing_order/firing_angles_deg are read by the A5 CycleEngine and folded into the C16b cache key for pipe engines only (lumped keys unchanged); banks/bank_angle_deg are not generator inputs and not in the key; key and the H3 table must be extended together); ISA constants in core/include/ps/drivetrain/isa_air.h. No friction model in the cycle itself. map_generator.cpp (I7c C16a): EngineMapGenTask (run_point(i) writes slot i only, finish() assembles in index order) extracts ve/eta_i/pmep_excess (signed)/friction etc. from cycle points over the cycle block's grid_rpm x grid_p_im_kpa, friction = D13 FMEP polynomial over mean piston speed; called by the loader when an engine file has a "cycle" block and its maps lack any of the 7 keys (a complete override never runs it); the maps' content hash is folded into SimulatedEngine::fold_state_hash. Full 12x20 grid 2.2 s single-thread / 0.23 s on 10 workers, Preview 0.33 / 0.034 s. map_cache.h/.cpp (I7c C16b): the .psmaps disk cache - key FNV-1a 64 over the RESOLVED CycleModelDesc (incl. source) + FuelDesc + kGeneratorVersion + quality (not JSON text; every new generator input must join the key, guarded by a table-driven H3 test), file <cache_dir>/<key:016x>.psmaps = 40-byte little-endian header + raw doubles, content hash re-checked on load, any mismatch/corruption a silent miss (regenerate + rewrite), temp file per process+thread + rename, catch(...) only (TOOL-019); cache hit 0.2-0.4 ms vs 0.17-2.2 s generation
      aero/ machine/                  empty (.gitkeep) until P4+
  audio/                            target ps_audio (STATIC, ps_audio A7): links ps_engine_cycle (+ ps_detmath/ps_warnings/ps_options) only, NEVER ps_core; core/ never includes ps_audio/ (ci.ps1 include grep; the detmath grep also covers audio/)
    include/ps_audio/, src/         engine_input.h (EngineInput snapshot + flags ignition cut / fuel cut / DFCO), spsc.h (SpscQueue<T,N>, SpscFloatRing - acquire/release on the two indices), time_map.h/.cpp (TimeMap: type-2 rate PLL per arrival, read point ~1 interval behind, rate clamp [0.5, 1.02], hitch > 2 intervals holds, resume jumps without catch-up), halfband.h/.cpp (31-tap exactly symmetric half-band decimator, used only at 2 substeps), live_engine.h/.cpp (LiveEngine: EngineCore stepped per sample at h = 1/48000 on audio dx, crank integrated at audio rate, plenum loop (10 ms filter + multiplicative integral tau 50 ms, frozen at omega 0), per-IVC combustion events with seeded variation, fuel cut/DFCO at IVC, ignition cut at start of burn, radiating ends as a PROVISIONAL monopole p = rho/(4 pi) dq/dt at 1 m [Pa] until A8; probe_stability()), voice.h/.cpp (Voice: SPSC input queue, snapshot interpolation, pitch from omega only, crank-phase correction capped at 1 % of omega while locked, pause fade 50 ms, render(frames); render_to_buffer offline), runner.h/.cpp (Runner: one thread, 256-frame blocks, SPSC float ring, one mono channel per source); A8: source.h/.cpp (SourceKind ExhaustOutlet/IntakeMouth, SourceInfo with installation position, Calibration{pa_full_scale} default 200 Pa at 1 m [A], SourceBlock via the free function render_to_source_blocks - Voice::render keeps its interleaved signature), a Strouhal-scaled flow-noise layer in Voice (f = 0.2 u/D, seeded, level_coeff_pa default 0 = off), and FV open-end radiation on the live path (PipeEndDesc::radiation, the WaveguideNetwork's Levine-Schwinger one-pole reused; the generator never sets it). A8b fixed the sustained-WOT blow-up (runtime CFL sub-cycling), the F1 post-stop ring (catalyst laminar monolith resistance r_lin += 2*fRe*mu*L/(d_h^2*OFA), fRe 14.227 Shah & London [C], generic 400 cpsi/6.5 mil substrate [A], WgNodeDesc::monolith_*; plus the inflow deadband) and W2's radiation excess (inflow fix); V9 has no fitted coefficient. The minmod FV loses ~14 dB/m at 2 kHz at the default 32 mm exhaust dx (3 dB/m at 1 kHz) - part of the HF deficit is numerical. Scalar cost of the audio-grade fixture I4: 18.6-19.4 us per sample = 0.89-0.93 core (FV pipes 83 %, cylinders 16 %, WG 1 %) - live play needs A9.
  tests/
    support/                          test-only shared helpers (not part of ps_core)
      counter_force_element.h         deterministic counter-hash IForceElement used by determinism/bench tests
      catch_main.cpp                  custom Catch2 entry point installing headless CRT handlers; target ps_test_catch_main
      vehicle_test_helpers.h          PLAN.md P3: VehicleTestRig/build_vehicle_rig()/settle() - a World with a static ground box (flat or tilted) plus one vehicle built directly from a data/vehicles/*.json file; see "Vehicle system" below for the chassis-half-extents-vs-wheel-shape-cast constraint documented on build_vehicle_rig(). build_vehicle_rig_from_desc() (PHYS-033 I1) is the VehicleDesc-taking core build_vehicle_rig() wraps - takes an already-loaded (test-mutable) desc plus a tilt_axis (unit_y() default = longitudinal slope, unit_x() = cross slope) and tick_hz/substep_hz (for a fine-tick reference run). Both (and build_vehicle_rig()) also take a trailing ground_half_extent (PHYS-033 P4 I3 plan section 13, default 1000 matching every pre-I3 caller) - the accel/top-speed validation tests drive far enough (2000/4000 m) that the old hardcoded 1000 m half-extent would run the car off the edge. Also build_vehicle_rig_on_surface()/flat_surface_mesh() - same, but a flat MeshShape ground tagged with a named data/surfaces/surfaces.json SurfaceId instead of an untagged BoxShape, for tests comparing behaviour across surfaces (a plain BoxShape has no Jolt material and always resolves to kInvalidSurfaceId); build_vehicle_rig_on_surface() also gained trailing optional tick_hz/substep_hz params (PHYS-033 I1b, default 240/960 matching every existing call site) for a fine-tick reference run on a non-asphalt surface. flat_surface_mesh() is tessellated into <= 20 m cells (PHYS-047, 2026-09-26): the old 2-triangle mesh's ~2828 m diagonal ran right under the (0,0) spawn point and fed Jolt's large-triangle CastShape artefact into every wheel cast; pinned by test_vehicle_flat_surface_mesh
      powertrain_reference.h          PHYS-033 P4 I3 plan section 13: RefCar, an INDEPENDENT reference model built field-for-field from the public VehicleDesc (never hand-duplicated JSON constants) - engine curves, clutch, gearbox ratios/efficiencies/inertias, diff, driven/undriven hub inertias, the Pacejka tyre model, aero. ref_gear_loss_params() derives the locked-gear reflected-inertia/torque-gain chain (K_half, J_refl) from the engine/clutch/gearbox/diff node equations (plan section 13's own staged T_m->T_out->T_fd->per-hub derivation); ref_locked_gear_step() is one semi-implicit-Euler substep of the resulting ODE (test 5's "Engaged(k), Locked" case - test_powertrain_engine_braking.cpp). Deliberately simplified (no suspension/pitch dynamics, one-substep mesh-loss lag, event quantisation, lateral dynamics - plan's own "expected combined error <= 1%"), not a byte-exact replica of core/src/drivetrain's own implicit solver
      powertrain_multi_diff_desc.h    PHYS-033 P4 I4 C4: multi-differential test topologies built by rewiring car_sedan.json's VehicleDesc in the test only (no data file) - make_awd_desc(centre_eff) (open centre diff ratio 1 -> open front/rear axle diffs copied from car_sedan's own diff), make_parallel_4wd_desc() (gearbox.out shaft-joined to both axle carriers, no centre diff)
    unit/                             target ps_unit_tests: test_hello, test_math, test_engine_sound_glue (ps_audio A11, [audio]: the shared EngineSoundState -> EngineInput mapping), test_engine_rig_audio (ps_audio A10, tags [audio][drivetrain], R1-R6 [long] - each generates the pipe demo engine's maps, 20-90 s: R1 free rev to the limiter and back, R2 starter -> Running, R3 ignition off -> Off at once (not Stalled), R4 WAV bytes identical twice / seed changes them, R5 ledger closes from tick 1, R6 zero alloc; A8b B1 [long] WOT hold then close at the default audio dx stays bounded, max 22.4 Pa), test_audio_time_map (ps_audio A7: T1 time map, T2 SPSC, T3 half-band), test_audio_voice (ps_audio A7, tags [audio][cycle][pipe]: S1 stability probe 1500 rpm / 30 kPa rel 2.77e-3 -> 1 substep; V1 half/odd orders <= -112.6 dB rel. order 2; V11 seed/PCM/state pins, split render, pinned CycleEngine hash 0x3d50ccb4707a1c36 (the generator never applies the variation); V12a ramp under a 1.5 % clock mismatch tracks the firing frequency to 1.79e-5 away from the ramp corners; V12b 200 ms hitch; V12c 600 s at 0.3 % mismatch, lag within [0.42, 1.42] intervals; V12d crank correction cap 1e-2; V13 stop tail finite, -42.0 dB (bound -36 dB, A8b; exhaust tail tau 0.60 s); A8b V9 order-2 outlet pressure = the monopole rho*dq/dt/(4 pi r) within 1 % (0.99608; rho/4 pi sabotages fail), V10 with flow noise off; V11 pins pcm 0xe4e8cf0d04b01008 / state 0xa798da1cb19bfb03; tie vs CycleEngine within 1 % at 3 points (bound 3 %); Z1 zero alloc over 60 Runner blocks; P1 pause fade; R1 threaded Runner (TSan-able); [.][diag] variants), test_cycle_engine / test_engine_map_gen_v2 / test_cycle_engine_studies / test_engine_rig_wheelless (ps_audio A5, tags [drivetrain][cycle][pipe], fixture tests/support/cycle_pipe_fixtures.h - an [AS] generic I4 with runners, primaries and an installation: S1-S3 pipe-mode stepper, E1-E5 CycleEngine closure/trim/reservoir mode, V7 lumped maps bit-identical to v1 (pinned content hashes and keys), G1 pipe-engine map content hash 0x171173a11cd5cd05 (A8b; was 0xeced05eb5892fd4a), G2 [long] 1 vs 10 workers, G3 unreachable node, G4 version in the key, V8a runner sweep 0.25/0.35/0.50 m peak within +-15 % of the 1/L_eff estimate (tracks one tuning order), V8b primary sweep 1.2/1.8/2.4 m moves the residual minimum 4132/3153/2607 rpm, C1 h vs h/2 <= 0.25 %, C2 dx 50 vs 25 mm within 0.5 % [!mayfail] (-0.72 % at 1500 rpm / 30 kPa, owner accepted 50 mm 2026-09-28, P17 - revisit after A9), EngineRig: a wheel-less crank + LoadDyno Powertrain builds and closes its ledger; [.][diag] trim trace, studies, grid cost), test_spark_schedule (I7c C17a follow-up, X1 no schedule + delay 0 bit-identical maps, X2 runtime eta_i*eta_spark equals the cycle model at scheduled spark with a delay, X3 loader rejections incl. both cycle and ecu schedules given), test_waveguide_network (ps_audio A3, tags [drivetrain][cycle][pipe]: W1-W8 - closed-open/open-open/Helmholtz within 0.02 %, radiation |R| within 0.00226, Kirchhoff fit 0.29 % / mode decay 0.41 %, lossless energy +2.2e-16, pinned hash, resistance exact to 1e-12; A8b W9 catalyst monolith laminar resistance - and V6 expansion-chamber TL within 2.6e-3 dB below the first cross-mode; each with a recorded sabotage), test_pipe_waveguide (ps_audio A3: V4 FV vs WG on a 2->4 mm cone within 0.41 dB per third-octave to 1.5 kHz, I1-I8 face transparency/energy hand-over/quarter wave across the split/mean-flow closure/hybrid vs pure FV 0.19 dB/rejections/zero alloc/resistance mean drop/WG->FV reflection 3.1e-8 (the only check that catches a doubled face admittance); [.][diag] FV numerical dissipation - a sigma 0.05 m pulse at dx 8.5 mm loses 8/15/21 % in 1/2/3 ms), test_pipe_network (ps_audio A2, tags [drivetrain][cycle][pipe]: P1 Sod vs exact Riemann within bounds + no new extrema, P2 closed network mass/energy/burned drift <= 5e-15 over 20000 steps, P3 quiescent taper stays at exactly 0 m/s, P4 closed-open/open-open/Helmholtz resonances within 0.13 %, P5 port hand-off closure, P6 pinned CFL rejection, P7 zero alloc, P8 run-to-run identical; A8b: P9 a supersonic jet stays bounded (sub-cycling), P10 a ring decays identically with ambient +-0.01 K about the pipe temperature, W2 radiation decay 0.861x analytic (bound [0.75, 1.0], was 5.26x), P5 checked on a 50 ms tail mean; each with a recorded sabotage), test_engine_installation_io (ps_audio A4, tags [io][cycle][drivetrain]: engine + installation round trips incl. the vehicle-level override, an engine without pipe blocks bit-identical to pre-A4, every intake/exhaust/resolution/installation rejection message; fixtures built inline), test_engine_map_cache / test_engine_hot_swap (I7c C16b; A4 extended H3: every acoustic pipe/installation field moves the key, positions and source/path strings do not, topology names (interface/into/from) do, tags [drivetrain][cycle][io]: H1 hit bit-identical, H2/R5 corrupt/truncated/wrong-key files regenerated through the loader, H3 key covers every generator input, H4 atomic write, H5 swap visible exactly at the consuming tick (engine work differs; the state hash alone cannot time a swap), H6 1 vs 10 workers + zero alloc/free inside jobs on the swap tick, H7 rejections incl. unknown vehicle id), test_engine_map_generator (I7c C16a, tags [drivetrain][cycle]: M1 runtime reproduces the cycle net torque at nodes to 5e-16, eta/pump/friction off-node <= 1 %, air worst 9 %/mean 2 % (a real ~260 Hz intake-runner Helmholtz ripple, +-2.5 % at 22.5 kPa, sub-grid); M2 eta_i resampling exact at nodes; M3 1/4/10 workers and reverse order hash-identical; M4 content-hash coverage; M5 a complete override runs the generator 0 times; M6 maps hash folded into the state hash (fresh engines differing only in a far-corner VE node); M7 zero alloc; F4 Ecotec motored-friction fit pinned (a 66619.52 Pa, b 91.085, c 346.81 - biased low at high speed, the subtracted fixture pumping 65 kPa at 5000 rpm comes from its ASSUMED valvetrain); [.][diag] ripple, M8 timing and the review's runner-length/pumping-split cases), test_engine_cycle (I7c C15a, tag [cycle], fixture tests/support/cycle_fixtures.h - a generic ASSUMED 2 L I4 plus an LIVC variant: G1-G11 geometry/closed motoring/Otto limit/Wiebe/blowdown both directions/bookkeeping/step convergence/Woschni literals/cycle count/zero alloc/low-speed VE, each with a sabotage; [.][diag] G7/G9 tables, cost (165 ns/RHS), review h sweep and 1-deg valve-area probe), test_simulated_engine / test_simulated_engine_io (PHYS-033 P4 I7b C11/C10: SimulatedEngine white-box against tests/support/mvem_fixtures.h, SimulatedEngineDesc loader rejections), test_simulated_engine_powertrain (I7b C12, cases "C12 *": idle, ambient T3a-d (air/rev vs speed-density 2.2e-16, refill count and bit-identical ISA refill), T4c stage command, T8 WOT launch ledger/tank closure from tick 1 (tick 0 carries the unbooked idle seed, vault PHYS-049 F-T0), F9 burnt fuel, pairing, determinism 1 vs 10 workers + zero own alloc; plus "C12/C17a integration": shipped car_sedan_sim.json idles, WOT 1st->2nd, v > 18.5 m/s at 5 s, balance <= 5e-8 from tick 1), test_a25a_sim_engine_data (I7c C17a-MB: literal pins of toyota_a25a_fks_sim.json/car_sedan_sim.json, WOT anchors vs the EPA xlsx, round trip vs C7), test_fuel_tank (PHYS-033 P4 I7a C6, tags [drivetrain][fuel], fixtures tests/support/fixtures/vehicle_tank_sedan.json / _e10.json / vehicle_tank_fuel_map_sedan.json, cases named "C6 T*": T1 a never-empty tank bit-identical to the plain car every tick; T2 initial - m_tank == fuel_mass - fuel_external_added (worst 1.46e-12 kg); T3/T5 crude and map engines starve at WOT - the emptying substep books exactly the rest (tank exactly 0.0), stall time within 0.3 % + 2h of an integral of J/|m(omega)| over the loaded desc (measured +0.050 %/+0.035 %, idle T3b 0.26 s), dry cranking never fires, refuel + starter restarts; T4 tank selects the fuel, E10/E5 fuel ratio = literal LHV ratio to 1e-12; T6 set_fuel_tank_mass at the tick boundary, clamped, booked signed, T6b reset_vehicle does not refill; T7 every loader rejection message; T8 balance_error while starving (7.5e-16); T9 1 vs 10 workers, zero own allocations; T10 white-box pre_solve with supply +inf/0/half the demand, tau0 and d against literal curve knots in the WOT-slope, soft-cut and governor regimes - each missing s factor on d fails it (e.g. governor d 23.88 vs 0.19 at s = 0)), test_cited_engine_data (PHYS-033 P4 I7a C7: toyota_a25a_fks.json's WOT/motoring curves equal the cited EPA xlsx rows exactly, the fuel map equals its own literal grid at every node plus an aggregate check against every measured fired row (n=182, max 41.10 % on a Pareto-excluded ECU-enriched row, median 0.50 %), tier3_e10_cert.json's literal properties and an independent AFR re-derivation, car_sedan_fuel.json WOT launch without stall with the energy balance held), test_fuel_io (PHYS-033 P4 I7a C4: both shipped fuel files load, each required-field/format/phase rejection; C5c: ethanol and the four blends load and equal the test's own mixing-rule recomputation from gasoline.json + ethanol.json to 1e-6 - fails on a volume-for-mass-fraction mixup and on one corrupted digit; E5/E10/E85 density inside EN 228 / EN 15293 as sanity cross-checks only, no E25 range exists), test_engine_fuel_map (PHYS-033 P4 I7a C5, tags [drivetrain][fuel], fixture tests/support/fixtures/vehicle_fuel_map_sedan.json: fuel-map loader rejections, no-map engines bit-identical, indicated_work = h*omega_new*tau_comb incl. a DFCO-recovery case (the old tau_comb/tau0 split was singular as tau0 -> 0, a grid-search demo reads 2304.85 vs. 40.51), and two 960/960 Hz real-Powertrain cross-checks - fuel flow and the booked fuel_mass equal fuel_map(pre-solve rpm, theta_c) every substep through a free rev, DFCO and idle PI; fuel_mass/fuel_energy_in/engine_heat equal independent per-substep sums over a WOT launch - each shown to fail on a sabotaged build: post-solve rpm lookup 1.33 %, command instead of theta_c 100 %, a dropped booking line 100 %; I7a C5d crude no-map model: tests go through the production to_engine_desc resolver with literal expected constants - dyno-held integral fuel_mass == sum h*max(0,tau_comb*omega_old)/(0.325*LHV_E5) to 1e-12, settled World idle mdot = -m(rpm)*omega/(eta*LHV) within 2 % (the Powertrain's Coulomb row cancels c_k), exact zeros in DFCO/fuel cut/ignition cut/Stalled/Off, engine_heat >= 0 crank->overrun, tractor fuel_mass*0.48*42e6 == sum h*tau_comb*omega_old to 1e-12, World ratios fuels [E10,E5] = LHV_E5/LHV_E10, indicated_efficiency 0.40 = 0.325/0.40, ignition flipped = 0.325/0.48, state_hash identical with/without fuels, zero-alloc, 1 vs 10 workers; sabotages: fuels.back() ratio 0.979 vs 1.021, swapped defaults 1.477 vs 0.677, ignored override 1.0 vs 0.8125, gasoline for the tractor 4.5 %, net torque tau0 instead of tau_comb 21.5 % (idle flow 0), omega_new caught by the WOT free-rev crude test (~0.26 %) not by the dyno-held ones), test_throttle_control (PHYS-033 P4 I7a C3, tags [drivetrain][throttle]: P1 the engine input has no pedal field (static_assert); P2 an explicit electronic stage with a pedal_map flat over the tested pedal range leaves every engine result and both state hashes bit-identical over 1920 substeps (fails at substep 0 when the engine is fed the raw pedal); P3 pedal map at a dyno-held rpm, throttle_eff/torque demand to 1e-12; P4 tests/support/fixtures/vehicle_mechanical_throttle.json - linkage curve applied, floor/ceiling exactly 0/1 while the TCU asks for less, upshift cut zeroes theta_c; P6 tractor over vehicle_tractor_i4's control events compared exactly against throttle_eff/torque demand/governor setpoint/hand_throttle recorded on the pre-C3 code (ba0f486); car_sedan with an extra hand_throttle channel unaffected, snapshot hand_throttle 0; C5b, tag [tcu]: a pedal_map moves the auto-shift upshift point (target stays 1 at 4021.7 rpm, 2 with the TCU fed the raw pedal), and tractor.json's hand throttle reaches the auto-clutch launch law (engagement 0.2289 = the command's e_tgt vs. 0.2722 with the raw pedal)), test_table2d (PHYS-033 P4 I7a C1: Table2D exact at nodes, bilinear vs. hand values, clamping on every side/corner, slopes vs. central differences, zero slope outside, validate() rejections), test_jobs (incl. the queue-full inline-fallback cases), test_world_basic, test_world_snapshot (ArticulationSnapshot/contacts/vehicles additions), test_articulation_core (white-box, core/src on its include path), test_articulation_four_bar (white-box), test_articulation_parallel_alloc (white-box: System::step_all is one job per articulation, debug_alloc-armed), test_aba_fast_transform (white-box: fast vs. reference ABA congruence transform, 1e-12 relative), test_scenario_articulation, test_articulation_contact (Compliant/ProxyBody smoke tests on a single-DOF rail rig, incl. a ProxyBody-at-high-speed [!mayfail] case), test_mesh_heightfield_shapes (MeshShape/HeightfieldShape collision + surface ids), test_terrain_golden (ProceduralTerrainSource vs. terrain.py's golden points), test_terrain_tile_manager (white-box: seams, streaming, throttling, zero-alloc steady state), test_test_ground_loader (load_test_ground_collision + SurfaceTable), test_test_ground_edge_lengths ([.][diag], PHYS-047: read-only triangle edge-length/bad-cast report for data/tracks/test_ground - asphalt max edge 15.03 m, kerb 2.14, low_mu 5.59, dirt/grass one 174.64 m-edge triangle pair each with 0 bad casts over 3.3M probes per triangle), test_terrain_static_overlap (PHYS-021: a parked vehicle's wheels and a sliding probe crossing the test_ground straight's edge, plus a zero-gap sweep of both skidpad ring edges - the terrain/static-feature coincident-collision fix, see "Terrain tile system"), test_terrain_suppression_precompute (white-box, real test_ground data: load-time suppression precompute is bit-identical to the lazy path for every tile, worker-count invariant, and hit at runtime), test_terrain_tile_datum (white-box, R1: per-tile vertical datum for real-world 90-880 m ASL terrain - synthetic sources at +800 m/-350 m land probes and a dropped body within 1 cm, adjacent tiles with different datums stay seam-continuous, relief-overflow counting, plus a 1 m-spacing collision-resolution measurement at datum 0 and ~768/~896 m for flat/30% slope/5 mm step cases, both at sample points and cell centres), test_pacejka_tyre_model (PLAN.md P3: MF5.2-subset pure/combined-slip curves, relaxation ODE, rolling resistance, cross-checked against an independent re-derivation and exact combined-slip-degeneracy identities; plus every shipped tyre's longitudinal slide-to-peak shape - Fx(kappa=1)/peak, Fx(kappa=3)/peak and peak slip at fz0 and 0.75*fz0), test_vehicle_ackermann_symmetry (front wheel Ackermann steer angles are exact left/right mirror images at +-full lock, and each wheel turns toward the commanded direction - see "Vehicle system" below and vault PHYS-022), test_vehicle_parallel_alloc (PLAN.md P4 I0: a full World::step with a vehicle, debug_alloc-armed - PHYS-033 I1b commit C3 added a T3 negative-control case proving the Jolt-allocator hook is wired, and [!mayfail]-tagged the existing zero-alloc case once that hook revealed a pre-existing Jolt-internal allocation, see "Job system"; PHYS-033 P4 I4 C15: a tractor case with a viscous load_dyno on pto.out toggling awd/pto/diff_lock and park_lock after a stop - own count 0, external 620 = the known Jolt QuadTree gap), test_vehicle_io_validation (load_vehicle_json/load_tyre_json error paths: ackermann_fraction range, "format" field incl. /1 rejection, legacy driven/drive/brakes keys, differential efficiency != 1 - PHYS-033 I2 commit 5a; auto_clutch sync_time_s/max_sync_torque_nm must be > 0; auto_blip upshift_cut_release_slip_rpm/upshift_cut_max_hold_s > 0, upshift_torque_ramp_s >= 0), test_body_integration_params (PHYS-033 I1: IRigidBackend::get_integration_params vs. the box-inertia formula, damping/gravity-flag round-trip), test_wheel_contact_frame (PHYS-033 I1b, white-box, core/src on its include path: ContactFrame's normal/long_dir/lat_dir and the true ground contact patch, incl. bottomed/rim-contact/side-wall-fallback cases - see "Vehicle system"), test_sub_threshold_rotation (parking-slope creep fix D2: a free box with a small steady angular velocity does not rotate by default, does when opted in via IRigidBackend::set_integrate_sub_threshold_rotation - see "Jolt backend"), test_powertrain_engine (PHYS-033 P4 I3, white-box: EngineTorqueMap's torque blend against hand values and a central-finite-difference slope check, the Off/Stalled/Cranking/Running state machine's every transition, the idle PI's small/large-dip response, FuelCut limiter hysteresis at WOT in neutral, cranking equilibrium against the starter/Coulomb/smooth-friction balance; PHYS-033 P4 I4 C12 governor: theta_gov against hand values on a lever/rpm grid (the hand_throttle/pedal max() combination and the no-governor case moved to test_throttle_control in I7a C3), the governor dtau/domega term against a central finite difference inside the droop band (1e-6 relative) and absent when saturated, absent when the idle PI or the TCU floor wins strictly inside the band (the round-2 tautology guard - the idle-PI SECTION fails on the old `>=` code), a single governed inertia under constant load settling to the bisection steady speed), test_powertrain_gearbox (white-box: Gearbox's position<->state-index mapping, a full N->1st shift cycle through Selecting/Synchronising, dog release refused while loaded then releasing once torque drops, synchro time matching J_in*|domega|/capacity within 2 substeps, engaging reverse while rolling grinding then aborting to N, Sequential walking one position at a time toward a far target; plus Clutch: engagement = min(pedal curve, auto), capacities scale from it), test_powertrain_tcu (white-box: ManualTcu's counter differences/first-tick baseline/reset/clamped-pending, gear_request precedence over the counter path and the "none" sentinel, the auto-clutch launch law's e(rpm) and Launch->Locked->Launch cycle, the anti-stall panic release rate vs. the ordinary release_rate_per_s, blip/ignition-cut flags following shift phase, launching from Neutral below vs. above the speed threshold, the clutch held open while the gearbox engages from Neutral whatever the rpm, the upshift cut held through the post-shift close until speed-matched (released below upshift_cut_release_slip_rpm, never re-armed in the same shift, released after upshift_cut_max_hold_s, not extended with a manual clutch) and the pedal ceiling's ramp-in, auto-shift thresholds and hysteresis), test_powertrain_network (white-box: dense.h's Cholesky factor/solve against reference Gauss-Jordan and its PSD regularisation, build_nullspace against the plan's own car/tractor worked examples, two inertias locking through a friction row momentum-conservingly, a gear chain's power reproducing ratio*efficiency*torque with lagged mesh loss, an open differential splitting torque and holding a locked hub, a topology switch conserving momentum and booking the exact engagement loss, a friction row's ratio generalising its coupling (synchro row shape), clutch lock-up under a ramping capacity showing no spike - plan section 13.3(a)'s literal criterion on a test-only network, test_powertrain_clutch_lockup.cpp below is the whole-vehicle companion), test_powertrain_friction_lcp (white-box: box_lcp's block-principal-pivoting stage against brute force over batches 1/3/4/physical and a [long] 10^5-case sweep, the k=1 closed form and c=0 caller-level filtering, test_powertrain_control_channels (white-box, core/src on its include path, PHYS-033 P4 I4 C3: declared control-channel list unchanged for car_sedan/tractor, declare_control_channels duplicate-merge and empty-for-real-vehicle cases, a declared channel's default/set_control value arriving in ComponentSubstepCtx::channels exactly one tick late, debug_alloc-armed zero-alloc check with a declared channel present), test_powertrain_multi_diff (white-box, PHYS-033 P4 I4 C4: car_sedan's rigid-row order/Leaf mode/speedo/reset unchanged, AWD and parallel-4WD networks build with the expected recon modes/rows/speedo (4.06, not 8.12, for two axle diffs on one carrier), a lossy diff whose outputs carry other rigid couplings is rejected, reconstruct_mesh_torque_pair against the carrier's truth with a lock row/friction row/coupling on the outputs, nested (centre) reconstruction against truth and bit-identical across post_solve orders, multi-diff reset derivation order leaves-up), test_powertrain_topology (white-box, PHYS-033 P4 I4 C5: composed-state mixed-radix round trip and per-state row lists, no lockable diff -> s == g, lock pending above the mismatch threshold then engaging, engagement loss vs. an analytic projection (1e-12), lossless release, locked |w_L - w_R| <= 1e-12 over 2000 substeps, LeafPair vs. carrier truth, reset keeps the composed state, 4 locks = 128 states / 5 rejected, lockable centre diff rejected, debug_alloc-armed lock toggling while driving), test_powertrain_diff (white-box + whole-vehicle, PHYS-033 P4 I4 C6: spool |w_L - w_R| at rounding level over 2000 substeps lossy and lossless, viscous hub-torque difference = 2*c*|w_L - w_R| and diff_loss = sum h*c*v^2 with a balanced ledger, LeafPair truth for a locked leaf, Nested exact for a lossless locked/viscous centre, lossy non-open centre rejected, plain open bit-identical, debug_alloc-armed zero-alloc World stepping with a viscous diff; I4 C7: clutch_lsd saturation 2|lambda| = preload + f*T_axle (1e-9) with drive -> coast after a carrier-torque reversal, capacity lagging T_axle by exactly one substep, torsen T_high/T_low = TBR (1e-9), LeafPair truth with the row active, whole-Powertrain ledger (<= 1e-12) with diff_loss = sum -h*v*lambda, centre LSD/Torsen rejected, lockable LSD capacity 0 while locked, zero-alloc World stepping and a 1 vs 10 worker state_hash check for both kinds), test_powertrain_transfer_case (white-box + whole-vehicle, PHYS-033 P4 I4 C9: raw-network TransferCase slip/stuck/disengaged identities, tractor.json's rear/front diff ratios 20/15.30 with all four hubs driven, reset derivation order [rear diff, front_diff, transfer], speedo bit-identical 20.0/1.025 to the pre-C9 baseline), test_powertrain_park_lock (white-box + whole-vehicle, PHYS-033 P4 I4 C10: a request above max_engage_speed_rad_s stays pending while spinning down then engages at the threshold and holds, release is immediate, a torque above capacity slips at exactly the kinetic capacity with -h*v*lambda booked into brake_loss, tractor's park row sits between the transfer_case row and the brake rows (brakes shift by one) while car_sedan has none, debug_alloc-armed park_lock toggling (own 0), 1 vs 10 worker state_hash), test_powertrain_pto (white-box + whole-vehicle, PHYS-033 P4 I4 C11: Pto slip bounded by e*capacity and dissipative, engagement ramp at 1/engage_time_s clamped to [0,1], mode change refused while engaged or |omega_out| >= 1 rad/s, disengaged stub brake decaying omega_out per its exact implicit recurrence, efficiency < 1 booking a lagged loss into gear_loss, pto_clutch_loss = row slip loss + stub-brake loss; LoadDyno Brake/Viscous/Quadratic against their exact implicit updates; a load_dyno added to car_sedan in code inserting its row without shifting existing ones, tractor's pto row between the park_lock row and the brake rows; debug_alloc-armed pto toggling with a load_dyno on pto.out, 1 vs 10 worker state_hash with pto toggled and a mode change mid-run), test_powertrain_compliance (white-box + whole-vehicle, PHYS-033 P4 I4 C13: undamped two-inertia period vs 2*pi/omega_n (0.5 %) and the trapezoidal discrete frequency, KE+PE drift over 10^4 substeps (measured 4.7e-13), damped log decrement vs zeta (2 %), backward Euler exactly after a reset / a friction-row stick change / a topology switch, reset relaxes every compliant shaft, twist and the BE flag hashed while a Rigid vehicle's hash ignores stiffness, whole-Powertrain ledger closure per tick, debug_alloc-armed zero alloc, compliant sedan (clutch.out-gearbox.in k 600, c 2) launch at throttle 0.4/1.0 without stall, 1 vs 10 worker state_hash; the implicit_reaction guard: gearbox mesh torque closes gearbox.in's node balance per substep through throttle steps (7.1e-15 N*m; 7.063 N*m with the fix disabled) - a quasi-steady compliant-vs-rigid mesh-torque comparison passes either way, the twist rate is ~0 there; [.][diag] WOT launch speed vs stiffness)
    validation/                       target ps_validation_tests: test_car_sedan_pipes (ps_audio A11: car_sedan_pipes.json and car_sedan_pipes_rwd.json - 2026-09-29: a WOT free-rev lift never dips below idle, min 800.045 (the SimulatedEngine idle rework: DFCO dashpot at the no-load idle position, manifold-fill term, one-sided PI with the pedal released, idle_control.max_torque_nm authority cap - demo_i4_pipes.json 15 N*m [AS], creeping in 1st/2nd needs 9.8/10.4); settles at idle - last 2 s of 10 s range 0.653 rpm, mean 800.054, within +-10 rpm from 3.05 s (was a ~1040-1300 rpm hunt: demo_i4_pipes.json grid_rpm now starts at 800, closed-throttle leak 0.002, a 25 kPa grid_p_im node under the ~29.5 kPa idle point, the A25A sim engine's throttle curve shape and idle gains; data only, sabotage ranges in the file header) - free-revs to the 6800 rpm limiter, WOT 1st-gear launch to 12.2 m/s without stalling; the same checks run for car_sedan_pipes_rwd.json, launch 13.3 m/s), test_gen_engine_dyno (I7c C17a M-A, tags [drivetrain][validation]: the generated A25A on a dyno-held harness vs EPA NVFEL WOT rows - S1 torque [!mayfail]: with the measured spark schedule and the cited 40.3 deg burn (C17a second follow-up) +2.4/+2.9/+18.9/+16.9/+17.0/+11.1/+0.3 % at 2017..4954 rpm, 4 of 7 outside +-10 % (measured spark + 60 deg burn: -4.9..+11.2 %, only 2996 outside - the slow burn's eta deficit masked the air excess); per-term budget in the file header: trapped air +40..+49 N*m at 3000-4500 rpm from the [AS] quarter-wave 709 mm runner (no cited A25A runner length exists), eta now +0..+8 N*m (was -9..-19), F4 friction 7-8 N*m MORE lossy than the A25A's motoring; nothing tuned, D1; S2 WOT BSFC [!mayfail] (owner-confirmed) now within +-10 % at every point (-0.4..+7.4 %) - fuel and torque both follow the air excess, so the pass does not validate air; S4 pedal law 4.2e-7 of span; check 4 steady torque monotone in plate; [.][diag] RK4 convergence, runner ripple, load cost, budget, spark/motoring review diag), test_free_fall, test_pendulum, test_joint_chain_energy, test_box_on_slope, test_tunnelling, test_resting_contact, test_huge_forces, test_mass_ratio (one case [!mayfail]), test_articulation_pendulum, test_articulation_double_pendulum, test_articulation_energy, test_articulation_mass_ratio, test_articulation_floating_base, test_articulation_actuator, test_articulation_four_bar_energy (white-box, core/src on its include path), test_articulation_contact_measurements (Compliant/ProxyBody: a fixed-base arm resting on the ground, the same rig pressed with a known force, a rail rig pushing a dynamic crate, a floating-base vehicle drop, 5/20 m/s impacts; two [!mayfail] ProxyBody cases), test_terrain_seam_crossing (tile seams vs. a seamless single-heightfield baseline on a synthetic flat/slope/wavy ITerrainSource over the real TileManager pool: sliding sphere/box at 5/20/40 m/s peak |az| <= 2*seamless + 30 and <= 120 m/s^2, the original 256 m/64-sample x = -512 repro likewise; car_sedan.json at 5/20/40 m/s chassis |az| <= 2*seamless + 0.5 m/s^2, peak pitch acceleration <= 2*seamless + 0.5 rad/s^2, peak wheel Fz <= 1.05*seamless; [.][diag] per-case dump cases), test_vehicle_static (corner weights vs. hand calculation), test_vehicle_flat_surface_mesh (PHYS-047: the real wheel cast probed over flat_surface_mesh() near (0,0) - 0 of 160801 wrong vs. 31 for the old 2-triangle mesh, reproduced by a [.][diag] case), test_vehicle_parking_slope (15% slope, <1mm creep in 30s - [!mayfail] history through PHYS-033 I1b was chasing the wrong cause ("elastic hold" weakness); the real mechanism was two numerical defects, a standstill rolling-resistance sign(vx) chatter (TyreDesc::rolling_resistance_v_eps) plus Jolt's own dropped sub-threshold rotation (IRigidBackend::set_integrate_sub_threshold_rotation) - both fixed, [!mayfail] removed, measured drift 2.5e-6 m (1.03e-6 m as of car_sedan.json's PHYS-033 P4 I5 prep 60/40 split, 2026-09-25); a [.][diag] re-check case still runs the 240Hz/960Hz comparison), test_vehicle_step_steer (yaw-rate response does not grow after a step-steer input), test_vehicle_skidpad (PHYS-036 rewrite: speed is held by a tangential CoM force element with throttle fixed at 0 - free rolling, not a traction limit through the FWD drivetrain - walked up through a sequence of held-speed trials (a slow ramp across trials) gated on |mean(dv/dt)| < 0.02 m/s^2, curvature error < 0.5% and no steer/thrust saturation; compared against TyreLimitedSteadyStateSolver, an in-test quasi-static steady-circle solve using the vehicle's own real VehicleDesc/TyreDesc data and the real Pacejka tyre model (replacing the old per-axle peak-sum estimate, whose /2 halving bug - vault PHYS-018 - and missing Mz term made it unusable once Mz's sign was corrected, see PHYS-036) - PASSES, measured 8.8538 vs. estimate 8.7871 m/s^2 (0.76%; 8.6778 vs. 8.6024 m/s^2, 0.88%, as of car_sedan.json's PHYS-033 P4 I5 prep 60/40 split, 2026-09-25, still well under the 4% tolerance), [!mayfail] removed; does not assert which axle limits), test_vehicle_braking ([!mayfail]: 100-0 km/h distance vs. a load-transfer-corrected mu estimate - a per-axle ABS emulation fix (was front-only) closed the gap from 16% to 9%, still over the 3% tolerance; the residual is root-caused to closed-loop slip regulation against a locally unstable target under the wheel spin model's hard velocity clamp, not a harness oversight - see the test's own header comment and vault PHYS-019; ~10.4% as of car_sedan.json's PHYS-033 P4 I5 prep 60/40 split, 2026-09-25 - stop_distance=41.59 m vs. estimate 37.69 m, still [!mayfail], same root cause, not re-diagnosed), test_vehicle_accel_standing_start (full-throttle 0-50 km/h average acceleration from an ALREADY-SETTLED rest vs. a point-mass estimate from car_sedan.json's own torque curve, within 15%, driven-wheel slip bounded (no runaway wheelspin); "acceleration on dirt/grass lower than on asphalt" PASSES, [!mayfail] removed (PHYS-047, 2026-09-26): its "chaotic" behaviour was flat_surface_mesh()'s old diagonal seam under the spawn point corrupting wheel casts, not vehicle dynamics - bit-identical over 3 reruns (2 s speeds asphalt 4.714 / dirt 4.289 / grass 2.074 m/s); a third case ("...stays bounded and does not stall (tyre refit 5064b8b + 60/40 split)"): full throttle commanded before the suspension has settled, from t=0 - it used to drive the front tyres to the +-3.0 kappa clamp (documented, not asserted against); since the tyre refit plus car_sedan.json's 60/40 split it asserts max |kappa| < 0.8 over the run (measured 0.461) plus continuous forward progress - see the file's own header comment and out/handover_accel.md for the history (final_drive_ratio 10.0 -> 8.5 in I1b C8, replaced by the real gearbox in I3)), test_vehicle_steer_mirror (full-throttle-from-settled-rest steering response is left/right mirror-symmetric: yaw rate opposite sign but equal magnitude, forward speed gained equally in both directions - see "Vehicle system" below and vault PHYS-022), test_vehicle_standstill_wiggle (a car arriving at near-standstill with residual yaw rate and/or steering settles instead of oscillating - the live-demo skidpad "wiggle" bug; two driven-then-braked cases plus a clean pure-yaw-rate-kick isolation, all asserting decaying not growing/holding amplitude, a worker-count state_hash check for the low-speed code path, a [.][diag] "zeta sweep diagnostic" (PHYS-033 I1 commit E), and PHYS-033 spec test 10 - a yaw kick with zero low-speed damping is neutral and discretization-converged (240 vs. 960 Hz), the same with rolling resistance on, and a [!mayfail] check of the default-zeta decay rate against the section 4.2 modal estimate (measured off by ~14x, a known limitation of that single-mode estimate, not a code bug) - see "Vehicle system" below; Spec test 10.1's tail indices k=17-23 fail its 5% discretization-convergence check (~25% off) - pre-existing (PHYS-037, fails identically at be36545), root cause not diagnosed), test_vehicle_shadow_chassis (PHYS-033 I1 W1: shadow_dv_error/dw_error stay within 1e-4 - measured ~1e-6/1e-7 - for a settled car under step steer, a constant force element and an alternating one matching the predicted one-tick-lag estimate; a chassis forced into contact shows dv_error > 1e-2; a per-vehicle-file low-speed-damper stability bound gamma < 1, split into separate car_sedan/tractor cases by PHYS-033 I1b commit C7 (the patch-lever gamma: car_sedan 0.950 (0.956 as of the PHYS-033 P4 I5 prep 60/40 split, 2026-09-25 - still under 1), tractor 0.652, both still under 1)), test_vehicle_tractor_cross_slope (PHYS-033 I1 spec test 11: a tractor holds a 10% cross slope / 15% longitudinal slope on the handbrake alone - [!mayfail] history through PHYS-033 I1b (same wrong "elastic hold" diagnosis as test_vehicle_parking_slope.cpp) removed now the real cause (same two defects) is fixed - measured drift 3.9e-6 m / 1.3e-4 m; a [.][diag] A/B harness against the H0-H6 cross-slope hypotheses, confirming H0 (data/vehicles/tractor.json's inconsistent rear attachment z) as the root cause of the ORIGINAL owner-reported bug; PHYS-033 P4 I4 C10 park lock cases: park_lock alone in neutral on 15% longitudinal drifts 1.38e-4 m over 30 s while the same rig with park_lock = 0 rolls 519.6 m (negative control), park_lock plus handbrake on the 10% cross slope 4.45e-6 m), test_vehicle_contact_geometry (PHYS-033 I1b: whole-vehicle pins for the contact-patch/force-decomposition fixes - a pitched-at-rest chassis does not self-propel, steady-cornering/straight-braking load-transfer ratios match statics within the plan's own bands - plus a [.][diag] "contact geometry diagnostics" case (skidpad steady-state check, braking/accel traction diagnostics); "Straight braking" LTR_long 1.010 vs. 1.01 +- 0.12 passes, measured against the analytic static axle load m*g*|x_rear|/L (its old unsettled telemetry static sample caused the 1.125-1.149 excess reported from I1b C7 on); "Steady cornering" LTR_lat 1.061 passes), test_powertrain_engine_braking (PHYS-033 P4 I3 plan section 13.4/13.5, tests 5-6: engine braking in 4th from 100 km/h vs. tests/support/powertrain_reference.h's RefCar locked-gear ODE - fixed (commit eb920bf, "test 5: RefCar applies mesh losses direction-aware on overrun": the reference, not the sim, was wrong - RefCar applied the drive-direction mesh efficiency on overrun too, shrinking the modelled engine-braking torque), [!mayfail] removed, measured 0.05% (0.0496% as of car_sedan.json's PHYS-033 P4 I5 prep 60/40 split, 2026-09-25 - unaffected by the split, confirmed deterministic across 3 reruns), see the test's own header comment for the full term-by-term breakdown; plus "stall under load" (handbraked uphill clutch dump stalls the engine), "gentle idle-only launch" (creeps away without stalling), "auto-clutch anti-stall" (never stalls braking to a stop) - all three PASS), test_powertrain_clutch_lockup (plan section 13.3(b), the whole-vehicle companion to test_powertrain_network.cpp's test-only-network case: manual rolling-start clutch lock-up PASSES (unaffected by the PHYS-033 P4 I5 prep 60/40 split, 2026-09-25); auto-clutch launch from rest is [!mayfail] - halfshaft torque continues climbing ~12% above the pre-lock level for up to 0.5s after lock, specific to an abrupt full-throttle auto-clutch launch, not a general driveline bug; still [!mayfail] as of the 60/40 split, measured max_post/mean_steady ~2.23x on both driven halfshafts (626.0 vs. 280.4 Nm, bound 294.5 Nm) and ~1.79x on the clutch itself, same failure category, not re-diagnosed; PHYS-033 P4 I4 C14 compliant variant (car_sedan desc mutated: clutch.out-gearbox.in k 600 c 2, 960/960 Hz, the clutch locks 0.42 s into the pedal ramp): the rigid gates hold over the post-ramp window (clutch 21.80 vs. 22.89, halfshafts 143.5 vs. 150.7) and over ramp + window incl. the lock event (clutch 74.49 vs. 79.14, halfshafts 492.9 vs. 515.9); the post-lock mode 7.711 Hz (zero crossings of a fitted-baseline-detrended shaft torque, first 4 half-periods - zeta ~0.29 leaves only ~2 usable cycles; window/offset spread 2.3 %) is within 2.5 % of a THREE-mass reference - engine side J1 0.164 / hubs 0.0223 / vehicle mass 0.793 kg*m^2 reflected, joined by the tyre slip-relaxation spring k_t = K_x*r^2/sigma_x (804 N*m/rad at gearbox.in, K_x from the real tyre model at the post-lock window mean) in series with its K_x*r^2/V damper - vs. -72 %/-27 % for the two-mass hubs-only/rigid-m*r^2 references; asserted ratio checks sigma_x x2 (0.863 vs. 0.848), pkx1 x2 (1.161 vs. 1.161), chassis mass x2 (1.108 vs. 1.121); measured stays +1.3..+4.4 % above the three-mass model (chassis pitch suspected, not modelled); zeta 0.290 vs. 0.283 report-only), test_vehicle_787b_shuffle (2026-09-26: car_sedan_787b.json, a lift to throttle 0 in 4th after a steady pull - 1500 kg at WOT and 830 kg at throttle 0.5 (830 kg at WOT spins the passenger tyre onto the limiter in every gear): the lift-off shuffle, fitted as a damped sinusoid plus a quadratic trend over 0.2 s from the first extremum, at 23.87/25.47 Hz (halfshaft torque) and 23.88/24.80 Hz (engine rpm, +-236/+-81 rpm) vs. an n-mass spring/damper model built from the loaded VehicleDesc with the tyre as a Maxwell element (25.03/25.78 Hz), asserted within 7 % and zeta within 0.05; the same car forced Rigid shows no mode (amp/step 0.0004/0.0013, 0 extrema); damping comes from the tyre, not mesh efficiency (all eta = 1 moves f <= 0.13 Hz); [.][diag] cases write the lift-off CSVs rpm_to_wav.py reads; further [.][diag] cases (2026-09-26 ring-duration sweep): one-factor parameter sweep, best-case combinations, an aliasing verification (a c x0.1 input-shaft mode at 180.8 Hz aliases to 76.5 Hz at a 240 Hz tick), a lumped lash ODE, listening CSVs - base zeta 0.226 splits into tyre slip damper ~0.145, shaft dampers ~0.075, engine motoring slope ~0.006, mesh loss ~0; ~500 ms of ring (zeta*omega ~6.5 1/s) only with 830 kg, mu 1.7, 6th at 8500 rpm, Je 0.03, shaft c x0.1 and halfshaft k x0.5 stacked; lash 0-8 deg adds a clunk, no long ring), test_powertrain_energy_balance (plan section 13.6, test 7: the PowertrainEnergy ledger's own balance_error identity stays within a measured-with-margin 5e-8 relative per tick (max observed 1.485e-8, a brief relaxation-lag transient during hard braking with wheel slip - the same class of effect as tyre_loss's "steady sign" caveat) and within the literal <= 1% of (engine_out+starter_in) end-of-run acceptance bound - PASSES; PHYS-033 P4 I4 C15 tractor case: MFWD on, pto plus a dyno, diff_lock toggled while driving, braked to a stop - same per-tick and end-of-run bounds (end-of-run 2.85e-11 J vs. bound 1431.68 J), with awd_clutch_loss/pto_clutch_loss/external_absorbed checked > 0), test_powertrain_accel_topspeed (plan section 13.1/13.2: only test 2, top speed within 3% of a steady-state-per-gear bisection reference, is implemented - PASSES with a wide margin (car_sedan.json: gear 5, v_ref=56.79 m/s / 204.4 km/h, sim 0.24% off; v_ref=56.83 m/s / 204.6 km/h, sim 0.22% off as of the PHYS-033 P4 I5 prep 60/40 split, 2026-09-25 - top speed on flat ground is largely drag/gearing-limited, not split-sensitive); test 1 (0-100 km/h) is explicitly deferred, not forgotten - needs an event-driven Neutral/Engaged(k) x {Slipping,Locked} clutch state machine plus an independent re-derivation of the gearbox's own shift-phase timing, substantially more complex than test 5's locked-gear-only reference which itself needed two real attempts - see the file's own header comment), test_powertrain_auto_clutch_shifts (owner test drive 2026-09-25, auto-clutch around upshifts on car_sedan.json: selecting 2nd at walking pace (5 km/h) never stalls and pulls away on throttle - before the fix the rpm-blind Shift-mode reclose dragged the engine onto a ~350 rpm input shaft and stalled it; a part-throttle 1->2 upshift at 15 km/h locks within 1 s without wheelspin; the torque-following post-shift close (AutoClutchDesc::sync_time_s/max_sync_torque_nm): WOT 1->2 from a non-spinning 1st keeps the driven wheels below kappa 0.2 and locks within 2 s, WOT in 2nd from a steady 20 km/h does not spin, a full-throttle 1->2 at 15 km/h locks within 1 s, a part-throttle 2->3 from ~2500 rpm locks within 1 s; N->1 requested at WOT from rest (flat and 15% uphill) engages within 1.5 s and drives away - the Launch law used to close the clutch on a flaring engine while the gearbox was still synchronising 1st; the assist.auto_shift channel alone enables auto-shift (the TCU desc's enabled flags are only the assist.* channels' defaults) and WOT upshifts near up_rpm_at_full_throttle; the speed-matched upshift cut (AutoBlipDesc::upshift_cut_release_slip_rpm/upshift_cut_max_hold_s/upshift_torque_ramp_s): WOT 1->2 and 2->3 near the shift point, and both auto-shift WOT upshifts, hold the cut through the post-shift close so the car coasts (a < 0.3 m/s^2) from clutch-open to cut release, the torque hole (request -> pedal ceiling back at 1 and clutch torque >= engine torque demand) stays inside a per-case measured bound (1->2 0.78 s, 2->3 0.57 s), the clutch never carries more than torque demand + max_sync_torque_nm after the cut, and the clutch locks within 1.5 s; the clutch-torque ceiling it checks against is the close law's own max(0, T_demand + max_sync_torque_nm) - without the max(0, .) a motoring demand below -50 N*m made a fully open clutch read as an overshoot), test_powertrain_multi_diff_awd (PHYS-033 P4 I4 C4, tests/support/powertrain_multi_diff_desc.h's AWD at centre efficiency 1.0 and 0.99: WOT from rest drives all four wheels, per-tick balance_error <= 5e-8 relative and end-of-run <= 1%, front/rear hub-torque share 0.5 +- 0.02, max |kappa| < 0.5, speedo within 3% of ground speed at a light-throttle cruise; plus a 1 vs 10 worker state_hash equality case), test_reference_cars (baseline smoke for the three reference sedans car_sedan/_rwd/_awd: driven-wheel set, settled corner loads sum to m*g within 2%, a 3 s WOT launch in 1st from settled rest - no stall, speed > 7 m/s, per-tick balance_error <= 5e-8 relative, per-car measured-with-margin kappa bounds (FWD 2.02 / AWD 1.36 driven kappa is real wheelspin without traction control, RWD 0.10) - plus a 1 vs 10 worker state_hash equality case per car), test_powertrain_split_mu (PHYS-033 P4 I4 C8, plan Test 4, re-diagnosed 2026-09-26: car_sedan.json with its front diff mutated per kind, left wheels on asphalt / right on low_mu, on a LOCAL split ground tessellated into 20 m cells - the original 4-triangle +-1000 m ground gave wrong wheel shape casts near its 2000 m seam edge (276 of 28471 probes: phantom fraction-0 hits ~0.12 m above the plane, wrong-side surface ids), which was the whole of C8's "diff hop", viscous wrong-surface drift and locked-braking divergence; a "rig ground validity" cast-probe case pins it, a [.][diag] case reproduces 0/6/80/276 bad casts for half-extent 100/300/500/1000 m. PASS: open launch; locked launch slip/force and yaw (sum(-y*fx), I_z*dr/dt); open/locked braking Mz; clutch_lsd and torsen saturation identities at throttle 0.6 with limiter_active asserted false (at WOT the 60 ms fuel-cut cycle flips drive/coast - not the one-substep capacity lag, whose loop gain is <= 0.45/0.5); viscous 2*c*|dw|; the hub-torque cross-check at 960/960 Hz (one substep per tick, so tick-mean and last-substep torques agree) for all five kinds over [t0+0.2, t0+2], worst 3.6e-11 N*m. [!mayfail]: only the plan's literal telemetry Mz formula for the locked launch (-68.7 vs < -100 N*m) - genuine: lateral tyre forces cancel ~97% of the fx moment at steer 0, and the formula omits aligning moment and rolling resistance), test_powertrain_mfwd (PHYS-033 P4 I4 C9, tractor.json on a local 20 m-cell tessellated ground (ground-validity probe: 2501 casts, 0 wrong): MFWD engaged at gear 6 / 3 m/s PI-held shows the kinematic front lead computed from the desc (1.02025; kappa_F - kappa_R 0.0201545 vs. 0.0202455, +-0.003), clutch slip ~1e-12 and wind-up (fx_f 4422 N, fx_r -2389 N on asphalt); MFWD off (PHYS-033 P4 I4 C12 rewrite): awd_active_ticks == 0, and the front |fx| peak (817.95 N at t=0.346 s, a launch transient) is the front hub plus reflected front-driveline inertia - hub and driveline torque closures <= 1e-9 at 960/960 Hz, |fx_inertial| > 100 N, |fx/fx_inertial - 1| <= 1 - eta = 0.06 (the lagged 6 % mesh loss; measured 1.033), [.][diag] case prints both rates; engagement ramps both ways; drawbar pull (C12, OECD Code 2 style): gear 1, hand_throttle 1, pedal 0, settle at zero load until consecutive 1 s window mean speeds differ by < 0.1 % (60 s timeout), then a 100 N/s rearward ramp; limit = drawbar force at the argmax of the trailing-0.25 s-mean summed driven fx (traction saturation), the 0.5*v0 speed gate only stops the ramp (CHECK limit < stop); within +-25% of a static-Fz Pacejka estimate - off 1666.9 vs. 1564.6 N (+6.5 %, stop 1882.1 N), on 3141.9 vs. 3338.6 N (-5.9 %, stop 3352.9 N), ratio 1.885 (the C9 1 m/s speed-PI target was unreachable - gear 1 at 2400 rpm high idle is 0.613 m/s; a slip-threshold stop at WOT measured wheelspin onset, max kappa 0.76 at zero load, [.][diag] case); 1 vs 10 worker state_hash with awd on/off; debug_alloc-armed awd toggling (own 0, external = the known Jolt QuadTree gap); balance_error worst 2.22e-15), test_powertrain_pto (PHYS-033 P4 I4 C11, tractor.json in N with park_lock, a viscous load_dyno (c 5 N*m*s/rad, J 0.02) shafted to pto.out: 540 mode at throttle 0.35 and 1000 mode at throttle 1.0 after a clean disengage/mode switch, both on the C12 governor's droop line (1355.6 rpm toward n_nl 1392.5 / 2243.49 rpm toward high idle 2400; pre-C12 soft_cut 2213.7 / 2243.5 rpm), window drift of omega_out and engine rpm <= 1e-3 (measured 0), engine_rpm/pto_rpm = 3.61 / 1.95 to 1e-9, |pto_clutch_slip| < 1e-9 (~5e-12), external_absorbed rate = c*mean(omega_out^2) (7731.66 / 72578 W), the pto share of gear_loss = (1-eta)*|lambda|*omega_in to 1e-3 (309.27 / 2903 W), balance_error <= 5e-8 relative (worst 8.2e-16). Pre-C12 (ungoverned throttle) the 1000-mode point at throttle 0.35 was not steady (the engine lugged from ~947 to ~696 rpm over 60 s) - why the case commands WOT, not a longer settle), test_powertrain_pto_governor ([long], PHYS-033 P4 I4 C12: tractor.json's all-speed governor measured through the pto as a dynamometer - a Brake-mode load_dyno at 0/25/50/75/100/120 % of full-rack pto torque at full lever settles at 2368.02 / 2324.15 / 2281.60 / 2240.25 / 2200.00 / 1935.00 rpm, each within 7e-7 of an independent bisection of m(n) + theta_gov(n)*(w(n) - m(n)) = (2 - eta)*tau_dyno/ratio, cross-checked against PowertrainDebug::engine_torque_demand; the same load fraction gives the same crank rpm in 540 and 1000 mode (2281.6 at 50 %))
    determinism/                      target ps_determinism_tests: test_worker_count_hash (1/2/4/10/24 workers + 1000-run randomised-scheduling stress test; one case adds several articulations to the same scenario, one case mixes Compliant/ProxyBody contact-bearing rail rigs with plain chains, one case adds two vehicles built directly from a VehicleDesc with a mid-run step-steer input - migrated to the full PHYS-033 P4 I3 engine/clutch/gearbox/diff chain by I3 plan section 11, one case loads data/scenarios/terrain_multi_interest.json from PS_SOURCE_DIR - two interest points plus suppression, one case (I3 plan section 13.9) hands the same two-vehicle fixture off to assist.auto_shift=1 at full throttle - the fixture's gearbox has no synchro capacity (dog only), so the resulting auto-upshift genuinely grinds (measured grind_events=1, asserted as a scenario-sanity precondition since that field itself is sound-only/not hashed) - hashes still identical across every worker count; PHYS-033 P4 I4 C15: data/scenarios/vehicle_tractor_i4.json at 1/2/4/10/24 workers equal to the recorded 0xac386af7129b9ffe, plus a companion case asserting each control event's live effect (pto_engagement/pto_rpm, gear 3, awd_engagement 1, rear diff locked in [5,6) and released, stopped with park_lock_engaged and engine Running at t=10) - run_scenario_file now applies scenario.control_events like ps_run (it silently ignored them before))
    bench/                            target bench_force_elements (plain main, also CTest-registered): 2000-force-element worker scaling; target bench_contact_coupling (plain main, also CTest-registered): cost per tick per contact strategy, scaling with contact count; target bench_terrain_tiles (plain main, also CTest-registered): tile-streaming cost/tick against the real test_ground terrain spec, simulated 50 km drive; target ps_energy_study (plain main, NOT CTest-registered): Part A solver-settings energy-loss sweep plus the P1.5a Jolt-vs-articulation table and 10-link cost benchmark, energy_study.cpp
  data/
    vehicles/                         JSON ps::vehicle::VehicleDesc files (ps::io::load_vehicle_json - see "Vehicle system" below): car_sedan.json (FWD 60/40 reference), car_sedan_rwd.json (RWD 50/50 reference variant, same engine/gearbox/tyres/brakes), car_sedan_awd.json (AWD reference: the RWD file's 50/50 geometry, gearbox.out -> open centre diff (ratio 1, lossless) -> open front/rear axle diffs; needs I4 C4 multi-diff), car_sedan_x3.json (demo-only fun variant: car_sedan.json with engines/i4_2l_na_x3.json and a 900 N*m clutch; no test uses it), car_sedan_fuel.json (PHYS-033 P4 I7a C7: car_sedan.json with engines/toyota_a25a_fks.json plus a fuel_tank), tractor_fuel.json (I7a C7: tractor.json plus a fuel_tank, no diesel fuel map - owner-deferred), car_sedan_sim.json (PHYS-033 P4 I7c C17a-MB: car_sedan.json with engines/toyota_a25a_fks_sim.json (kind "simulated") and a 350 N*m clutch - PROVISIONAL, the morning-drivable simulated-engine car), car_sedan_gen.json (I7c C17a M-A: car_sedan_sim.json with engines/toyota_a25a_fks_gen.json), car_sedan_pipes.json (ps_audio A11: car_sedan_sim.json with engines/demo_i4_pipes.json [AS] - the car whose physics maps and live engine voice come from the same pipe cycle model; car_sedan's 300 N*m clutch; not the demo default), car_sedan_pipes_rwd.json (owner 2026-09-29: car_sedan_rwd.json with engines/demo_i4_pipes.json), car_sedan_787b.json (2026-09-26, owner request: car_sedan_rwd.json with engines/r26b_4rotor.json, a 1200 N*m clutch, sedan_6mt/4.06 reused (no citable 787B ratios), compliance compliant: clutch.out-gearbox.in k 21020 c 1.35 (input shaft) and gearbox.out-diff.carrier k 1530.3 c 1.30 = both halfshafts LUMPED through the final drive (C13 allows no compliant diff-output shaft - diff outputs have no inertia); shaft dimensions and zeta assumed, k = G*J_p/L; still the passenger 225/45R17 tyre), tractor.json (MFWD as of PHYS-033 P4 I4 C9: gearbox.out -> transfer_case in; rear -> rear diff (ratio 20, eff 0.94); front -> open front_diff (15.30) -> FL/FR; "awd" channel default 0 = RWD; C10: a park_lock pawl on the gearbox.out shaft, channel "park_lock"; C11: a pto on the engine.crank/clutch.in shaft, 540/1000 modes (ratios 3.61/1.95), channels "pto"/"pto_speed"; C12: the engine's governor adds channel "hand_throttle"; C15: the rear diff is lockable, channel "diff_lock") - geometry derived from data/models/*/*.rig.json's wheel pivots, plausible/published parameters with source noted per PLAN.md section 8. Both are physics_sim.vehicle/2 (PHYS-033 I2 commit 5a): each references an engines/*.json and a gearboxes/*.json file via its powertrain.components' "ref" fields (see "Vehicle system" below)
    tyres/                            JSON ps::vehicle::TyreDesc files (ps::io::load_tyre_json) referenced by data/vehicles/*.json: passenger_car_225_45r17.json, tractor_front.json, tractor_rear.json
    engines/                          JSON ps::drivetrain::TorqueMapEngineDesc files (PHYS-033 P4 I3 commit 6, ps::io::load_vehicle_json's component loader): i4_2l_na.json (car_sedan.json and its _rwd/_awd variants), i4_2l_na_x3.json (car_sedan_x3.json: WOT torque x3), r26b_4rotor.json (car_sedan_787b.json: Mazda R26B, WOT 4000-9000 rpm digitised from SAE 920309 Fig. 6 with the cited 608 N*m/515 kW points, below 4000 rpm a labelled linear extrapolation, fuel_cut 9000, inertia 0.06 derived from 13B rotor mass plus an assumed 0.05, idle 1500 assumed), diesel_i4_4l5_turbo.json (tractor.json; PHYS-033 P4 I4 C12: a "governor" block - all_speed, high_idle 2400, rated 2200, droop 0.0909, channel hand_throttle - and a fuel_cut 2650/100 overspeed limiter replacing the soft_cut) - WOT/motoring torque curves, idle/limiter/starter parameters; I7a C5d: each lists its fuels and ignition - both i4 [e5, e10] spark, r26b [e5] spark (a stand-in, its race fuel is no EN 228 blend), tractor [diesel] compression; toyota_a25a_fks.json (I7a C7: 2018 Toyota 2.5 L A25A-FKS from EPA NVFEL Report 26781 plus its Tier 3 test-data xlsx - WOT = per-speed-line measured torque envelope, motoring measured, fuel map fitted per speed line on Pareto-non-dominated fired rows only (the lambda 0.63-0.73 ECU protection-enrichment rows are excluded and labelled; that ECU behaviour is not modelled), interpolated down to the motoring point, 807-6600 rpm; limiter 6600 ASSUMED; fuel tier3_e10_cert), toyota_a25a_fks_sim.json (I7c C17a fallback M-B, kind "simulated": MVEM tables back-solved from the same EPA NVFEL A25A data by tools/enginedata/a25a_mb_engine.py - own motoring-row friction, measured spark as MBT; WOT reproduced to <= 3.6e-6 relative, part-load +30-60 N*m (no cam-phase axis); the dyno match is tautological and does NOT validate the I7c generator (vault PHYS-050); fuels [tier3_e10_cert, e10], spark, electronic throttle), toyota_a25a_fks_gen.json (I7c C17a M-A, kind "simulated" with a `cycle` block and NO maps - generated at load by C16a from cited geometry (SAE 2019-01-0249), EPA-measured cam, lambda and (C17a follow-up) spark schedules (Tier 1b, cam and spark median-3 smoothed along MAP; cycle.spark_advance_deg is copied into ecu.spark_advance_deg by the loader, both given is rejected; burn_duration_deg 40.3205 CITED (C17a second follow-up: MIT 2.61 lec. 9, modern fast-burn 10-90 % ~20 deg through the frozen Wiebe a 5 m 2; was 60 [AS]), ignition_delay_deg 3.8632 DERIVED (cited spark-to-10 % 15 deg minus the Wiebe's own 0-10 % span 11.14 deg); eta_i stored MBT-equivalent so runtime eta_i*eta_spark reproduces the cycle at nodes), frozen literature constants and [AS] sizing rules, never fitted to the dyno; tier table in its source; 2.96 s to generate on 1 thread, 0.32 s on 10, 7 ms from the C16b cache)
    gearboxes/                        JSON ps::drivetrain::GearboxDesc files (same loader): sedan_6mt.json (car_sedan.json, 6-speed manual), tractor_12f4r.json (tractor.json, 12F/4R) - forward/reverse ratios+efficiencies, synchro capacities, neutral_phase_s/grind_after_s/abort_after_s
    installations/                    physics_sim.installation/1 files (ps_audio A4 format, docs/data_formats/installation.md): demo_i4_exhaust.json (A10, [AS])
    scripts/engine_audio/             ps_engine_audio event scripts (A10): free_rev.json, stall.json
    fuels/                            JSON ps::drivetrain::FuelDesc files (ps::io::load_fuel_json, PHYS-033 P4 I7a C4): gasoline.json (Heywood 1988 Table 9.7: LHV 44.0 MJ/kg, A/F 14.5; EN 228 density range midpoint 747.5), diesel.json (Heywood Problem 4.12: LHV 42 MJ/kg, F/A 0.067; EN 590 temperate density range midpoint 830) - Heywood App. D Table D.4 has no usable text layer in the scans available; ethanol.json (anhydrous, Heywood Table 9.7 p. 477: LHV 26.8 MJ/kg, SG 0.794, A/F 9.0) and e5/e10/e25/e85.json (I7a C5c: DERIVED from gasoline.json + ethanol.json by ideal volume mixing, rho = v*rho_e + (1-v)*rho_g, LHV and A/F mass-weighted by w_e = v*rho_e/rho - docs/data_formats/fuel.md). No ignition field: spark vs compression is an engine property. tier3_e10_cert.json (I7a C7): the EPA Tier 3 E10 certification fuel as cited by NVFEL Report 26781
    aircraft/                         empty (.gitkeep) until P6+
    surfaces/surfaces.json            ps::io::SurfaceTable input: name -> {lambda_mu, crr} (shared, deterministic, alphabetically-assigned SurfaceId space)
    scenarios/                        JSON scenario files (ps::io::Scenario schema): free_fall.json, pendulum.json, joint_chain.json, articulation_chain.json, multi_articulation.json (several open chains of different sizes, a floating base, a four-bar loop closure - Windows/Linux hash-compare and worker-count determinism), contact_articulation.json (one Compliant and one ProxyBody rail rig - same hash-compare/determinism coverage for P1.5b contact coupling), terrain_tiles.json + terrain_spec_small.json (a small synthetic ITerrainSource + tile-manager scenario, distinct from data/tracks/test_ground - same hash-compare/worker-count-determinism coverage for the M1 tile system), terrain_multi_interest.json (same small spec with TWO interest points, a static pad triggering suppression and dynamic balls on the streamed terrain - guards the cross-platform tile-slot assignment order, vault PHYS-026), vehicle_step_steer.json (car_sedan.json on a flat static ground box, a step-steer control_event - ps_run-drivable vehicle scenario per PLAN.md P3 scope item 7; same hash-compare/worker-count-determinism coverage for the vehicle system, see tools/ci.ps1's `$script:HashCheckScenarios`), vehicle_powertrain.json (PHYS-033 P4 I3 plan section 13.9: car_sedan.json on a flat static ground box, control_events driving shift_up_count/throttle/brake/shift_down_count/a starter pulse over 10s - same hash-compare/worker-count-determinism coverage for the I3 drivetrain chain; state_hash=0x5380949669212c71 - updated PHYS-033 P4 I5 prep 2026-09-25 for car_sedan.json's 60/40 front/rear static split, was 0xf797fc7c7bd967d1), vehicle_tractor_i4.json (PHYS-033 P4 I4 C15: tractor.json on a flat static ground box, control_events hand_throttle 0.6, pto 1, shift_up_count 1/2/3, awd 1, diff_lock 1 then 0, brake to a stop, park_lock 1 over 10 s; state_hash=0xac386af7129b9ffe at 1/2/4/10/24 workers; ScenarioTopics vehicle+drivetrain), vehicle_fuel_tank.json (PHYS-033 P4 I7a C7: tests/support/fixtures/vehicle_fuel_tank_car.json - a 0.005 kg tank car launched at WOT in 1st - runs dry, the anti-stall release opens the clutch, the engine stalls within 10 s; state_hash=0xe545d5f57136415f; hash-compared, ScenarioTopics vehicle+drivetrain), vehicle_sim_engine.json (PHYS-033 P4 I7b C12/I7c C17a-MB: vehicle_powertrain.json's bodies and control events on ../vehicles/car_sedan_sim.json - the simulated engine through World; state_hash=0x063a507387ea89fd at 1 and 10 workers (2026-09-29 idle rework; was 0x67872eb9fb8b3396); hash-compared, ScenarioTopics vehicle+drivetrain), vehicle_gen_engine.json (I7c C17a M-A: the same events on ../vehicles/car_sedan_gen.json, the generated-map engine; state_hash=0x4030651924c4dbd2 at 1/2/4/10/24 workers (2026-09-29 idle rework; 0x5e88a765a7e748e4 at the C17a second follow-up, cited burn duration; 0xbc61f26e2d05379e with the measured spark schedule, 0xe2a2ebb8616b9f2f before); hash-compared, ScenarioTopics vehicle+drivetrain)
    models/                           Godot art assets (glb + rig.json); owned by a separate session, do not touch
    tracks/test_ground/               owned by a separate session (trackgen), READ-ONLY from ps_core: terrain.json/terrain_golden.json (ProceduralTerrainSource's golden reference), test_ground_collision.json/.bin (ps::io::load_test_ground_collision's input - see "Test-ground static collision" below for the .bin's byte-offset layout)
  adapters/
    godot/                            GDExtension adapter + demo project (P2/M1); built by its own adapters/godot/smoke_test.ps1 (own build dir out/build/godot-adapter) or with PS_BUILD_GODOT_ADAPTER=ON. src/fall_detector.h: Godot-free fall diagnostic called from PsSimulation's sim-thread post_step - prints ONE "ps_godot FALL DETECTED ... END FALL REPORT" block per event when a vehicle's chassis centre drops more than 2.0 m below the terrain source's own height at its XY (tick, sim time, chassis pose/velocity, tile key under the car, resident/starved/precomputed tile counts, static collision under the chassis, per-wheel contact/surface); smoke_test.ps1 fails if it fires during the --m1-autopilot drive
      src/vehicle_voice.h/.cpp        ps_audio A11: VehicleVoice::try_create builds a ps_audio Voice + Runner from the vehicle's OWN engine desc when it is a simulated engine with a cycle block and pipes (else nullptr, reason logged once; no separate sound engine), live dx = kLiveDxScale 2.0 x the engine's audio dx (scalar voice at the default dx measured 20.89 us/sample = 1.00 core, no margin; 2x = 13.78 us, 0.66 core) on a desc copy only; PsSimulation feeds it from post_step and exposes has_vehicle_voice/get_vehicle_voice_sample_rate_hz/get_vehicle_voice_channel_count/read_vehicle_voice_audio (divided by set_voice_full_scale_pa, default 15 Pa)/get_vehicle_voice_underrun_count (empty-ring reads); demo main.gd mixes the sources to mono into an AudioStreamGenerator, keeping only ENGINE_AUDIO_TARGET_S 0.05 s queued (filling its 0.2 s buffer to the brim put ~250 ms between an rpm change and the speaker), Runner ring 4 x 256 frames (21 ms), and prints Godot playback skips. Drive with sound: --m1-vehicle=car_sedan_pipes
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
- `ps_audio` (STATIC, `audio/`, ps_audio A7): the live engine voice; links
  `ps_engine_cycle` only, never `ps_core`. Guard `ps_audio_standalone`
  (`tests/unit/audio_standalone_main.cpp`, plain `main`, WHOLE_ARCHIVE
  ps_audio + ps_engine_cycle + ps_detmath without ps_core, labels
  `audio;cycle` - a JobSystem reference in ps_audio fails its link).
  `bench_audio_voice` (`tests/bench`, plain `main`, NOT CTest-registered):
  per-sample cost split cylinders/FV/WG of an audio-grade I4.
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
so an error-path test for those would crash in this leg only; Linux ASan
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
`World::set_control`/`get_control` channel values (global, not per-vehicle)
- also a no-op when `World` has no vehicles.

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
- `vehicle_instance.h`/`.cpp` (`VehicleInstance`) - `step_tick()` runs a
  per-substep SHADOW chassis (PHYS-033 I1 W1): a local pose/twist,
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
split, now `0xbab69b300eba41a3` at 1 and 10 workers, still worker-count
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
`IRigidBackend::shape_cast` has no way to exclude a specific body from the
query, and Jolt's `CastShape` reports an immediate fraction-0 "hit" against
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

## Adapter seams (`ps/world/snapshot.h`, `World::set_control`/`get_control`, PLAN.md P2)

`World::set_ambient(AmbientSample)`/`ambient()` (PHYS-033 P4 I7b C9): sim thread between steps,
applied at the next tick; plumbed via `VehicleTickEnv::ambient` -> `ComponentSubstepCtx::ambient` ->
engine `Sensors::ambient` (`Powertrain::last_engine_sensors_ambient()` is the white-box hook). The graph
engine ignores it (owner ruling Q10), `WorldConfig::air_density` (aero) is NOT derived from it, and it is
never hashed; `tests/unit/test_ambient.cpp` guards both.

`World::set_control(name, value)`/`get_control(name)`: a `std::string`-keyed
`real` map (`control_channels_`) - global, not per-vehicle (PLAN.md P3/M1's
one-car scope; multi-vehicle channel namespacing, e.g. `"veh0.steer"`, is
deferred). Consumed every tick by every vehicle created via
`World::create_vehicle` (`"steer"`/`"throttle"`/`"brake"`/`"handbrake"`,
see "Vehicle system" below) - the first consumer since this seam was added.

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
`EOverrideMassProperties::CalculateInertia`. Joints: `Fixed` and `Hinge`
(`JPH::FixedConstraint`/`HingeConstraint`, world-space anchors/axes computed
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
  `[io]`, `[cycle]` (engine cycle model, pipes, generator), `[audio]` (ps_audio),
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
```
