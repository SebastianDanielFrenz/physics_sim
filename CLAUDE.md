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
  tools/
    setup_dev_env.ps1                 installs/verifies the dev toolchain (-CheckOnly to only verify)
    ci.ps1                            CI at three levels: -Affected/-Topic (per change), default fast, -Full (see "CI" below)
    tsan_suppressions.txt             ThreadSanitizer suppressions, loaded only by the linux-tsan TEST preset's TSAN_OPTIONS (one entry: Jolt's debug-assert read in JPH::Mutex::try_lock)
    common/                           tools/tests-only helpers, never linked into ps_core; target ps_headless_env
      headless_env.h/.cpp             installs Windows headless-CRT handlers (no blocking assert/abort dialogs)
    ps_run/                           headless scenario runner (JSON scenario -> CSV, pure function); target ps_run
    sweep/                            fans N scenarios out over local ps_run processes (--jobs N); target sweep
    plots/                            (empty until needed) python telemetry plotting, dev only
    assetgen/                         Godot art-asset generator (Python; owned by a separate session, do not touch)
  core/
    include/ps/                       public API headers (no Jolt or Godot types); target ps_core (STATIC)
      version.h                       ps::version_string(), ps::jolt_backend_self_check()
      types.h                         ps::real = double; id_index_t/id_generation_t; SurfaceId (uint16_t) + kInvalidSurfaceId
      math/                           vec3.h, quat.h, mat3.h, pose.h (Pose/Motion/Wrench incl. Wrench::operator*(real)), simd.h (f64x4 skeleton)
      jobs/                           job_fn.h (JobFn), job_system.h (JobSystem, WaitGroup, parallel_for/parallel_batches), task_graph.h, debug_alloc.h
      world/                          ids.h (BodyId/JointId/ForceElementId/ArticulationId), frame.h (Frame, ILinkStateSource, ArticulationLinkRef), force_element.h (IForceElement), snapshot.h (WorldSnapshot incl. ArticulationSnapshot/contacts, SnapshotTripleBuffer), world.h (World, WorldConfig)
      backend/                        i_rigid_backend.h (IRigidBackend, BodyDesc/JointDesc, ShapeContactHit, ContactEvent, create_jolt_backend()); shape_desc.h (ShapeDesc/BoxShape/SphereShape/CapsuleShape/MeshShape/HeightfieldShape - split out of i_rigid_backend.h so articulation_desc.h can use ShapeDesc without pulling in the backend's JointType)
      articulation/                   articulation_desc.h (ArticulationDesc incl. contact_strategy, RootDesc, LinkDesc incl. contact, JointType, JointLimits, VelocityActuator, LoopClosureDesc/LoopClosureType, ContactStrategy, ContactMaterial, LinkContactDesc) - descriptors only, no algorithm code
      terrain/                        terrain_source.h (ITerrainSource, TileKey, TileSample), terrain_config.h (TerrainConfig - pool sizing only, NOT grid resolution, see "Terrain tile system" below), procedural_terrain.h (load_procedural_terrain_source() factory)
      vehicle/                        public, additive descriptors/seams only (PLAN.md 5.1/P3, no algorithm code - see "Vehicle system" below): vehicle_desc.h (VehicleDesc/WheelDesc/SuspensionDesc/SteeringDesc/BrakeDesc/DriveDesc/Curve1D), tyre_desc.h (TyreDesc, Pacejka MF5.2/6.1-subset coefficients), tyre_model.h (ITyreModel, TyreSlipInput/TyreForces, create_pacejka_tyre_model(), SlipRelaxationState/step_slip_relaxation()), wheel_telemetry.h (WheelTelemetry, returned by World::wheel_telemetry()), wheel_state.h (WheelState, returned by World::wheel_state() and carried in WorldSnapshot::vehicles - PLAN.md M1)
      io/                             telemetry.h (TelemetryRegistry, CSV via IO thread), scenario.h (Scenario incl. ScenarioArticulation/ScenarioTerrain/ScenarioVehicle/ScenarioControlEvent, load_scenario, build_scenario), surface_table.h (SurfaceTable), test_ground_loader.h (load_test_ground_collision()), vehicle_io.h (load_tyre_json(), load_vehicle_json())
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
      vehicle/                        INTERNAL (ps::vehicle, not a public header); see "Vehicle system" below: tyre_desc.h+tyre_model.h's implementation is pacejka_tyre_model.cpp, tir_parser.h/.cpp (.tir file parser), suspension.h (header-only spring/damper/bump-stop/ARB force functions), wheel_instance.h/.cpp (WheelInstance: contact measurement + tyre/suspension substep), vehicle_instance.h/.cpp (VehicleInstance: chassis extrapolation, Ackermann steering, brake/drive torque split), system.h/.cpp (System, mirrors articulation::System)
      drivetrain/ aero/ machine/       empty (.gitkeep) until P4+
  tests/
    support/                          test-only shared helpers (not part of ps_core)
      counter_force_element.h         deterministic counter-hash IForceElement used by determinism/bench tests
      catch_main.cpp                  custom Catch2 entry point installing headless CRT handlers; target ps_test_catch_main
      vehicle_test_helpers.h          PLAN.md P3: VehicleTestRig/build_vehicle_rig()/settle() - a World with a static ground box (flat or tilted) plus one vehicle built directly from a data/vehicles/*.json file; see "Vehicle system" below for the chassis-half-extents-vs-wheel-shape-cast constraint documented on build_vehicle_rig(). Also build_vehicle_rig_on_surface()/flat_surface_mesh() - same, but a flat MeshShape ground tagged with a named data/surfaces/surfaces.json SurfaceId instead of an untagged BoxShape, for tests comparing behaviour across surfaces (a plain BoxShape has no Jolt material and always resolves to kInvalidSurfaceId)
    unit/                             target ps_unit_tests: test_hello, test_math, test_jobs (incl. the queue-full inline-fallback cases), test_world_basic, test_world_snapshot (ArticulationSnapshot/contacts/vehicles additions), test_articulation_core (white-box, core/src on its include path), test_articulation_four_bar (white-box), test_articulation_parallel_alloc (white-box: System::step_all is one job per articulation, debug_alloc-armed), test_aba_fast_transform (white-box: fast vs. reference ABA congruence transform, 1e-12 relative), test_scenario_articulation, test_articulation_contact (Compliant/ProxyBody smoke tests on a single-DOF rail rig, incl. a ProxyBody-at-high-speed [!mayfail] case), test_mesh_heightfield_shapes (MeshShape/HeightfieldShape collision + surface ids), test_terrain_golden (ProceduralTerrainSource vs. terrain.py's golden points), test_terrain_tile_manager (white-box: seams, streaming, throttling, zero-alloc steady state), test_test_ground_loader (load_test_ground_collision + SurfaceTable), test_terrain_static_overlap (PHYS-021: a parked vehicle's wheels and a sliding probe crossing the test_ground straight's edge, plus a zero-gap sweep of both skidpad ring edges - the terrain/static-feature coincident-collision fix, see "Terrain tile system"), test_terrain_suppression_precompute (white-box, real test_ground data: load-time suppression precompute is bit-identical to the lazy path for every tile, worker-count invariant, and hit at runtime), test_pacejka_tyre_model (PLAN.md P3: MF5.2-subset pure/combined-slip curves, relaxation ODE, rolling resistance, cross-checked against an independent re-derivation and exact combined-slip-degeneracy identities), test_vehicle_ackermann_symmetry (front wheel Ackermann steer angles are exact left/right mirror images at +-full lock, and each wheel turns toward the commanded direction - see "Vehicle system" below and vault PHYS-022)
    validation/                       target ps_validation_tests: test_free_fall, test_pendulum, test_joint_chain_energy, test_box_on_slope, test_tunnelling, test_resting_contact, test_huge_forces, test_mass_ratio (one case [!mayfail]), test_articulation_pendulum, test_articulation_double_pendulum, test_articulation_energy, test_articulation_mass_ratio, test_articulation_floating_base, test_articulation_actuator, test_articulation_four_bar_energy (white-box, core/src on its include path), test_articulation_contact_measurements (Compliant/ProxyBody: a fixed-base arm resting on the ground, the same rig pressed with a known force, a rail rig pushing a dynamic crate, a floating-base vehicle drop, 5/20 m/s impacts; two [!mayfail] ProxyBody cases), test_vehicle_static (corner weights vs. hand calculation), test_vehicle_parking_slope (15% slope, <1mm creep in 30s), test_vehicle_step_steer (yaw-rate response does not grow after a step-steer input), test_vehicle_skidpad (steady lateral acceleration vs. a two-axle roll-stiffness-split tyre-limited estimate, within 0.4% - PASSES; an earlier estimate-formula bug that double-counted lateral load transfer put this 5.2% over the 3% tolerance, see the test's own header comment and vault PHYS-018), test_vehicle_braking ([!mayfail]: 100-0 km/h distance vs. a load-transfer-corrected mu estimate - a per-axle ABS emulation fix (was front-only) closed the gap from 16% to 9%, still over the 3% tolerance; the residual is root-caused to closed-loop slip regulation against a locally unstable target under the wheel spin model's hard velocity clamp, not a harness oversight - see the test's own header comment and vault PHYS-019), test_vehicle_accel_standing_start (full-throttle 0-50 km/h average acceleration from an ALREADY-SETTLED rest vs. a point-mass estimate from car_sedan.json's own torque curve, within 15%, driven-wheel slip bounded (no runaway wheelspin); acceleration on dirt/grass lower than on asphalt; a third case documents that full throttle commanded before the suspension has settled DOES drive the tyre past its peak (Fx/Fz~0.6 vs ~1.0) but recovers within the test's own window and never stalls the car - see the file's own header comment, vehicle_desc.h's DriveDesc comment, and out/handover_accel.md for the final_drive_ratio root cause and this no-clutch-model caveat), test_vehicle_steer_mirror (full-throttle-from-settled-rest steering response is left/right mirror-symmetric: yaw rate opposite sign but equal magnitude, forward speed gained equally in both directions - see "Vehicle system" below and vault PHYS-022), test_vehicle_standstill_wiggle (a car arriving at near-standstill with residual yaw rate and/or steering settles instead of oscillating - the live-demo skidpad "wiggle" bug; two driven-then-braked cases plus a clean pure-yaw-rate-kick isolation, all asserting decaying not growing/holding amplitude, plus a worker-count state_hash check for the new low-speed code path - see "Vehicle system" below)
    determinism/                      target ps_determinism_tests: test_worker_count_hash (1/2/4/10/24 workers + 1000-run randomised-scheduling stress test; one case adds several articulations to the same scenario, one case mixes Compliant/ProxyBody contact-bearing rail rigs with plain chains, one case adds two vehicles built directly from a VehicleDesc with a mid-run step-steer input, one case loads data/scenarios/terrain_multi_interest.json from PS_SOURCE_DIR - two interest points plus suppression)
    bench/                            target bench_force_elements (plain main, also CTest-registered): 2000-force-element worker scaling; target bench_contact_coupling (plain main, also CTest-registered): cost per tick per contact strategy, scaling with contact count; target bench_terrain_tiles (plain main, also CTest-registered): tile-streaming cost/tick against the real test_ground terrain spec, simulated 50 km drive; target ps_energy_study (plain main, NOT CTest-registered): Part A solver-settings energy-loss sweep plus the P1.5a Jolt-vs-articulation table and 10-link cost benchmark, energy_study.cpp
  data/
    vehicles/                         JSON ps::vehicle::VehicleDesc files (ps::io::load_vehicle_json - see "Vehicle system" below): car_sedan.json, tractor.json - geometry derived from data/models/*/*.rig.json's wheel pivots, plausible/published parameters with source noted per PLAN.md section 8
    tyres/                            JSON ps::vehicle::TyreDesc files (ps::io::load_tyre_json) referenced by data/vehicles/*.json: passenger_car_225_45r17.json, tractor_front.json, tractor_rear.json
    aircraft/                         empty (.gitkeep) until P6+
    surfaces/surfaces.json            ps::io::SurfaceTable input: name -> {lambda_mu, crr} (shared, deterministic, alphabetically-assigned SurfaceId space)
    scenarios/                        JSON scenario files (ps::io::Scenario schema): free_fall.json, pendulum.json, joint_chain.json, articulation_chain.json, multi_articulation.json (several open chains of different sizes, a floating base, a four-bar loop closure - Windows/Linux hash-compare and worker-count determinism), contact_articulation.json (one Compliant and one ProxyBody rail rig - same hash-compare/determinism coverage for P1.5b contact coupling), terrain_tiles.json + terrain_spec_small.json (a small synthetic ITerrainSource + tile-manager scenario, distinct from data/tracks/test_ground - same hash-compare/worker-count-determinism coverage for the M1 tile system), terrain_multi_interest.json (same small spec with TWO interest points, a static pad triggering suppression and dynamic balls on the streamed terrain - guards the cross-platform tile-slot assignment order, vault PHYS-026), vehicle_step_steer.json (car_sedan.json on a flat static ground box, a step-steer control_event - ps_run-drivable vehicle scenario per PLAN.md P3 scope item 7; same hash-compare/worker-count-determinism coverage for the vehicle system, see tools/ci.ps1's `$script:HashCheckScenarios`)
    models/                           Godot art assets (glb + rig.json); owned by a separate session, do not touch
    tracks/test_ground/               owned by a separate session (trackgen), READ-ONLY from ps_core: terrain.json/terrain_golden.json (ProceduralTerrainSource's golden reference), test_ground_collision.json/.bin (ps::io::load_test_ground_collision's input - see "Test-ground static collision" below for the .bin's byte-offset layout)
  adapters/
    godot/                            GDExtension adapter + demo project (P2/M1); built by its own adapters/godot/smoke_test.ps1 (own build dir out/build/godot-adapter) or with PS_BUILD_GODOT_ADAPTER=ON. src/fall_detector.h: Godot-free fall diagnostic called from PsSimulation's sim-thread post_step - prints ONE "ps_godot FALL DETECTED ... END FALL REPORT" block per event when a vehicle's chassis centre drops more than 2.0 m below the terrain source's own height at its XY (tick, sim time, chassis pose/velocity, tile key under the car, resident/starved/precomputed tile counts, static collision under the chassis, per-wheel contact/surface); smoke_test.ps1 fails if it fires during the --m1-autopilot drive
```

## Targets

- `ps_core` (STATIC library): public headers in `core/include/ps`, sources in
  `core/src`. Links JoltPhysics and nlohmann_json `PRIVATE`. No Jolt or
  Godot type appears in any public header.
- `ps_headless_env` (STATIC, `tools/common`): Windows-only headless CRT
  setup (`ps_tools::install_headless_crt_handlers()`); linked by every
  tool/test executable (`ps_run`, `sweep`, `bench_force_elements`,
  `ps_test_catch_main`), never by `ps_core`.
- `ps_test_catch_main` (STATIC, `tests/support`): custom Catch2 entry point
  linking `Catch2::Catch2` + `ps_headless_env`; linked by every Catch2 test
  executable instead of `Catch2::Catch2WithMain`.
- `ps_run` (executable): `tools/ps_run/main.cpp`. Links `ps_core`,
  `ps_headless_env`. CLI: `ps_run <scenario.json> [output.csv] [--workers N]`.
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
(`PS_DEBUG_ALLOC_BACKTRACE=<n>` env var, `execinfo.h`) for the first `n`
violations - a root-cause diagnostic, not part of `debug_alloc.h`'s public
contract.

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
time), and one set of six per-wheel channels (`vehicle<i>.<wheel
name>.{slip_ratio,slip_angle,fx,fy,fz,surface}`) per wheel of every vehicle
created via `World::create_vehicle`, registered up front in
`create_vehicle` itself (PLAN.md P3 scope item 7) so the CSV header is
complete before the first `record_row`. `World::state_hash()` folds
`IRigidBackend::compute_state_hash` (FNV-1a over every live body's
pose+velocity bytes, in ascending `BodyId` order), then
`articulation::System::fold_state_hash` (same FNV-1a fold, every live
articulation's q/qd and, if floating, root pose), then
`vehicle::System::fold_state_hash` (same fold, every live vehicle's
per-wheel compression/spin/relaxed-slip state - not otherwise observable
from the chassis body alone) - PLAN.md D7; a `World` with no
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
(re)assignment bookkeeping (`unordered_map` insert/erase) is the one
explicitly-accepted exception to strict zero-alloc, bounded by
`max_tile_fills_per_tick`/tick; the STEADY STATE (needed-tile-set
unchanged, the common case while driving) is checked allocation-free.

Ordering rule (PLAN.md D7, vault PHYS-026, stated on `tile_manager.h`'s
container block): nothing that decides simulation state, pool-slot
assignment or backend-call order may ITERATE an unordered container -
MSVC's and libstdc++'s iteration orders differ, and a different fill order
hands the same tile to a different pool `BodyId` (moves `state_hash` and
Jolt's contact-solve order). `interest_points_` is therefore a `std::map`
keyed by interest-point id (updating an existing id does not allocate);
`key_to_slot_`/`own_pool_body_ids_`/`suppression_cache_` stay
`unordered_*` because they are lookup-only; the precompute's tile-key set
is sorted by `TileKey::packed` before use.

`World` wiring (additive, `ps/world/world.h`): `set_terrain_source`
(constructs the `TileManager`, then runs the suppression precompute below),
`set_terrain_interest_point`/`remove_terrain_interest_point`,
`terrain_resident_tile_count`/`terrain_starved_tile_count`/
`terrain_precomputed_tile_count`; the `terrain_tiles` pipeline stage is a
no-op tick until `set_terrain_source` is called. `World::create_body`/
`destroy_body` (now out of line in `world.cpp`) call
`TileManager::on_static_geometry_changed` for a Static body once a terrain
source is set (drops every cached suppression result). `bench_terrain_tiles`
(`tests/bench/`) measures real cost against `data/tracks/test_ground`'s
terrain AND static collision (demo config: 512-tile pool, 16 fills/tick,
700 m radius): release drive ~3.8 ms/tick avg (worst ~5-6 ms), cold
teleport onto split_mu worst tick ~6.8 ms; `set_terrain_source` ~305 ms
release (~123 ms pool + ~185 ms precompute). Flags `--no-precompute` and
`--workers N` are for A/B measurement only.

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

Per-corner suspension + tyre + steering + brakes + a placeholder drive, on
an existing chassis rigid body. Entirely internal except the descriptors in
`ps/vehicle/{vehicle_desc.h,tyre_desc.h,tyre_model.h}` and `ps/vehicle/
{wheel_telemetry.h,wheel_state.h}`, plus the `World`/`ps/io/scenario.h`/
`ps/io/vehicle_io.h` seams below - mirrors `ps::articulation`'s D2/D10 public-descriptor-plus-
internal-implementation seam. No Jolt type appears in any public header.

- `tyre_desc.h`/`tyre_model.h` (public) + `pacejka_tyre_model.cpp`
  (internal) - Pacejka Magic Formula MF5.2/6.1-SUBSET: pure-slip
  `Fx0(kappa, Fz, gamma)`/`Fy0(alpha, Fz, gamma)` (sine-arctangent B/C/D/E
  form), combined-slip `Gxa`/`Gyk` cosine-weighted normalisation, a
  simplified `Mz0` (its own B/C/D/E curve, NOT the full pneumatic-trail x
  `Fy` decomposition) and `Mx` (reduced linear-in-camber form), rolling
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
  acceptance tests don't need one). `step()`: suspension force from that
  measured `x`/`xdot`, tyre force via `ITyreModel` at the measured
  slip/load/camber, wheel spin integration from drive/brake/tyre-Fx torque.
  `step()` also adds `lateral_standstill_damping_force` (a low-speed-only
  viscous term on `fy`) - the skidpad "wiggle" fix (owner-reported live-demo
  bug: a car settles into a persistent lateral/yaw oscillation after driving
  onto the skidpad and slowing to near-standstill). Root cause, measured via
  `test_vehicle_standstill_wiggle.cpp`'s pure-yaw-rate-kick isolation: the
  relaxation ODE's own decay rate (`step_slip_relaxation`'s
  `a = vx_mag/sigma`) is shared by the longitudinal and lateral slip
  channels and vanishes as a wheel's ROLLING speed -> 0; the longitudinal
  channel is saved by rolling resistance (a genuine drag along the same
  axis, nonzero at any speed - why `test_vehicle_parking_slope` already
  passed), but the lateral channel has no equivalent, so the
  `alpha_eq`/chassis-yaw-rate loop is a marginally-stable (`zeta -> 0`)
  continuous oscillator. Confirmed NUMERICAL (not physical): the same
  initial kick grows into a saturated ~5.4 Hz/~0.92 rad/s limit cycle at
  the normal 240 Hz tick but decays cleanly at 4x finer tick rate -
  `VehicleInstance::step_tick`'s one-tick-delayed wrench application
  (chassis motion frozen at tick-start across all 4 substeps) turns zero
  continuous damping margin negative. **This fix is a COMPENSATION for that
  delay, not a repair of it** - the delay itself is untouched. `c_lat =
  2*zeta_min*(fz0/g)*sqrt(pky1*g/sigma_y)` (fz0 the tyre's FIXED reference
  load, not live fz - see below) gives the CONTINUOUS system enough real
  damping margin that the delay can no longer tip it negative, faded to
  zero above 2 m/s TOTAL contact-patch speed (not just longitudinal - so a
  genuinely fast lateral slide/spin/push is never damped). `zeta_min` is
  sized against THIS SPECIFIC 240 Hz one-tick delay, not derived from tyre/
  carcass physics - if `step_tick`'s application delay is ever fixed
  properly (e.g. wrench applied per SUBSTEP against an updated chassis
  velocity instead of once per tick against a tick-start snapshot, or an
  implicit lateral-relaxation integration), THIS TERM MUST BE REVISITED,
  not left stacked on top; that delay fix is plausibly the better long-term
  repair but was judged too large in scope for this bug (touches the
  tick/substep pipeline every vehicle validation test is measured against,
  would very likely move every vehicle scenario's `state_hash`) - flagged
  as its own PLAN.md item rather than attempted here. Two corrections found
  only by measuring, not by the initial derivation alone: (1) the damper's
  own stability bound is NOT the single-wheel check it first looks like -
  all four wheels' damping torques sum onto the one shared YAW degree of
  freedom, and because a real chassis's mass sits closer to the CoM than
  "concentrated at the 4 wheels", the yaw mode is ~3x more sensitive to
  this damper than the lateral-translation mode; a value the single-wheel
  check called safe (`zeta_min=2.5`) made the wiggle WORSE than no fix at
  all. The resulting aggregate bound (`wheel_instance.cpp`'s own comment)
  puts `zeta_min=1.5` just past the ratio=1 line into the alternating-sign-
  decay band - NOT the same as "marginal": the explicit-Euler decay
  multiplier is smallest (fastest decay) right at ratio=1 and only becomes
  slow/dangerous approaching ratio=2 (predicted, and measured, to diverge
  at `zeta_min~=2.47`); at `zeta_min=1.5` the multiplier is -0.213, an
  alternating correction shrinking ~79% every 4.17 ms tick, unmeasurable at
  any observable timescale - confirmed empirically the cleanest of
  {0.7, 1.0, 1.5, 2.5} actually tried (0.7 and 1.0 left a measurable slow
  secondary regrowth bump over 12-24 s that this single-mode estimate does
  not itself predict - real evidence of at least one further mode the
  estimate does not capture; 1.5 decayed with no regrowth over 24 s; 2.5
  diverged, matching the model almost exactly). (2) `c_lat` must scale with
  the tyre's fixed `fz0`, not the live/instantaneous `fz`: an earlier
  version tied to live `fz` created a positive-feedback loop during
  hard-acceleration weight-transfer transients (measured `fz` spiking to
  ~11x nominal on a FWD standing start) that briefly spun the car out in
  `test_vehicle_accel_standing_start.cpp`'s asphalt case before this was
  caught and fixed. Separately confirmed (not fixed, out of
  scope): the tractor on `test_ground.json`'s `spawn_side_slope` (10% cross
  slope, handbrake) already slides materially with this term fully
  DISABLED (~18 m/10 s, A/B measured) - a pre-existing, previously-
  unvalidated gap (no CTest ever covered tractor lateral-slope holding),
  not caused by this fix; enabling the fix makes it only modestly worse
  (~21 m/10 s).
- `vehicle_instance.h`/`.cpp` (`VehicleInstance`) - `step_tick()` runs
  every substep within a 240 Hz tick (PLAN.md D3: chassis pose/motion
  extrapolated from tick-start via the quaternion exponential map, held
  across substeps), computing every wheel's contact (pass 1, so the
  anti-roll-bar's partner-compression read in pass 2 is fresh), then each
  wheel's suspension+tyre+spin substep (pass 2); applies the
  substep-AVERAGED wrench once per tick via `IRigidBackend::apply_wrench`
  directly (not through `IForceElement`, whose `evaluate()` is const/
  stateless by contract and cannot own wheel state) - mirrors
  `articulation::ContactCoupling`'s existing "apply directly to the other
  body from inside the substep stage" precedent. Ackermann steering
  (`ackermann_wheel_angle()`): per-wheel angle from turn radius `R =
  wheelbase/tan(delta_avg)` and each wheel's attachment point relative to
  the rear axle (both derived from wheel attachment points at construction,
  not stored in `SteeringDesc`), blended by `ackermann_fraction`; a rear-
  steered wheel is proportional/opposite-sign only (no speed-dependent
  phase reversal). A front wheel's angle is `atan(dx / (R - dy))`, NOT
  `atan2(dx, R - dy)` - `dx` (the wheelbase) is always positive, so
  `atan2` could never return a negative (right-lock) wheel angle; see
  `tests/unit/test_vehicle_ackermann_symmetry.cpp`,
  `tests/validation/test_vehicle_steer_mirror.cpp` and vault PHYS-022.
  Brake/drive torque are computed once per tick (not per substep) from
  each wheel's spin speed at tick start.
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
integration of `omega_ * dt` each substep, in `step()`) and
`last_steer_angle_` (the `steer_angle` `WheelSubstepInput` already carried
in, now also stored). Both are deliberately EXCLUDED from
`fold_state_hash()` - neither is ever read back by anything that feeds into
future dynamics (write-only as far as the simulation itself is concerned),
so excluding them cannot hide a worker-count-dependent divergence, and every
existing scenario's `state_hash` had to stay bit-identical (confirmed:
`vehicle_step_steer.json` still hashes to `0x1c2b775be0dc3339` at 1 and 10
workers, matching the value recorded before this field existed).

`World` additions (additive, `ps/world/world.h`): `create_vehicle`,
`vehicle_wheel_count`, `wheel_telemetry`, `wheel_name`, `wheel_state`.
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
`.tir` file).

## Adapter seams (`ps/world/snapshot.h`, `World::set_control`/`get_control`, PLAN.md P2)

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

Not yet done: the adapter itself (`adapters/godot/`, untouched/read-only
for this task) still carries its own duplicate `control_channels_` map
and snapshot type (`ps_simulation.h`/`triple_buffer.h`) and does not yet
consume these `World`/`WorldSnapshot` additions.

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

`IRigidBackend::is_static` (PLAN.md M1/PHYS-021 - see "Terrain/static-feature
coincident-collision suppression" above): `body->IsStatic()` behind the
same `TryGetBody(lookup(id))` null-safety pattern as `get_mass`/
`get_inertia_world`.

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
tools\ci.ps1 -Affected            # PER CHANGE: topics from the changed paths (vs HEAD), release leg only
tools\ci.ps1 -Affected -Base HEAD~3   # ... covering the last 3 commits too
tools\ci.ps1 -Topic terrain,vehicle   # ... topics picked by hand
tools\ci.ps1 -Affected -PlanOnly  # print the decision only, build/test nothing
tools\ci.ps1                      # DEFAULT FAST: every leg; batch boundaries / before reporting done (< 5 min)
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
topic tags. Runs the `release` leg with `ctest -L '^(topics)$'`, the two
greps, the topics' `walltime` benches (alone), and the hash compare for the
topics' scenarios only (building just the Linux `ps_run` target for it).
Adds `linux-release` (+ `linux-tsan` for threading) only when the changed
core/tools lines touch unordered containers, sorting, libm/FP-contract
math or threading, or with `-Linux`. Foundational paths (core
`math`/`jobs`/`world`/`backend`, any `CMakeLists.txt`/`CMakePresets.json`,
`tools/ci.ps1`, `tools/tsan_suppressions.txt`, `tools/common`,
`tests/support`) run the default fast run and ask for `-Full`; **a path no
rule knows also falls back to the default fast run, never to nothing**;
doc-only changes (`*.md`) run the greps only; `adapters/**` notes that
`adapters/godot/smoke_test.ps1` is the check. Every decision is printed with
the path that caused it. Adding a subsystem = add a topic tag to its tests,
a `$script:PathRules` row, and (if it has scenarios) `$script:ScenarioTopics`.

**Default/Full layout (overlapped).** Phase A: the Windows lane (`release`,
`debug`, `asan`, `msvc-release`, greps) runs in `ci.ps1` itself while the
WSL lane runs in a child `powershell -File tools\ci.ps1 -WslLaneOnly`
(results handed back as JSON under `%TEMP%`); inside the WSL lane,
`linux-release` runs first, then `linux-asan-ubsan` and `linux-tsan` build
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

**Test selection.** Default FAST: the `release`, `msvc-release` and
`linux-release` legs run every test except `[long]`-tagged ones; the
`debug`, `asan`, `linux-asan-ubsan` and `linux-tsan` legs run only
`ps_unit_tests` + `ps_determinism_tests` + `[sanitizer]`-tagged tests
(`-LE "validation|long"` - the whole `ps_validation_tests` suite is
`[validation]`-tagged, and the `[sanitizer]` short variants deliberately
omit that tag so this one exclusion routes them back in). `-Full`: no
long/validation filter, every test on every leg (mandatory before a phase
counts done per PLAN.md section 7). `walltime` is excluded from phase A in
both and run in phase B.

**Test tags / labels** (`tests/*/CMakeLists.txt` pass `ADD_TAGS_AS_LABELS`
to `catch_discover_tests`, so every Catch2 tag becomes a CTest `LABEL`
selectable via `ctest -L`/`-LE`; repeated `-L` are ANDed):
- **Topic** (every test has at least one; the benches get theirs via
  `add_test` `LABELS` in `tests/bench/CMakeLists.txt`): `[math]`, `[jobs]`,
  `[world]`, `[backend]` (Jolt-backed rigid bodies, shapes, the `[hello]`
  self-check), `[articulation]`, `[contact]`, `[terrain]`, `[vehicle]`,
  `[io]`, `[determinism]`, `bench`. Multiple topics per test are normal.
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
