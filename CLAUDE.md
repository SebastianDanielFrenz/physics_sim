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
    ci.ps1                            runs every CI leg (see below)
    common/                           tools/tests-only helpers, never linked into ps_core; target ps_headless_env
      headless_env.h/.cpp             installs Windows headless-CRT handlers (no blocking assert/abort dialogs)
    ps_run/                           headless scenario runner (JSON scenario -> CSV, pure function); target ps_run
    sweep/                            fans N scenarios out over local ps_run processes (--jobs N); target sweep
    plots/                            (empty until needed) python telemetry plotting, dev only
    assetgen/                         Godot art-asset generator (Python; owned by a separate session, do not touch)
  core/
    include/ps/                       public API headers (no Jolt or Godot types); target ps_core (STATIC)
      version.h                       ps::version_string(), ps::jolt_backend_self_check()
      types.h                         ps::real = double; id_index_t/id_generation_t
      math/                           vec3.h, quat.h, mat3.h, pose.h (Pose/Motion/Wrench incl. Wrench::operator*(real)), simd.h (f64x4 skeleton)
      jobs/                           job_fn.h (JobFn), job_system.h (JobSystem, WaitGroup, parallel_for/parallel_batches), task_graph.h, debug_alloc.h
      world/                          ids.h (BodyId/JointId/ForceElementId/ArticulationId), frame.h (Frame, ILinkStateSource, ArticulationLinkRef), force_element.h (IForceElement), snapshot.h (WorldSnapshot, SnapshotTripleBuffer), world.h (World, WorldConfig)
      backend/                        i_rigid_backend.h (IRigidBackend, BodyDesc/JointDesc, ShapeContactHit, ContactEvent, create_jolt_backend()); shape_desc.h (ShapeDesc/BoxShape/SphereShape/CapsuleShape - split out of i_rigid_backend.h so articulation_desc.h can use ShapeDesc without pulling in the backend's JointType)
      articulation/                   articulation_desc.h (ArticulationDesc incl. contact_strategy, RootDesc, LinkDesc incl. contact, JointType, JointLimits, VelocityActuator, LoopClosureDesc/LoopClosureType, ContactStrategy, ContactMaterial, LinkContactDesc) - descriptors only, no algorithm code
      io/                             telemetry.h (TelemetryRegistry, CSV via IO thread), scenario.h (Scenario incl. ScenarioArticulation, load_scenario, build_scenario)
    src/                              implementation
      version.cpp, jolt_proof.cpp
      math/math_check.cpp             compiles every math header once + static_asserts (math/ is otherwise header-only)
      jobs/
        job_system.cpp, task_graph.cpp, topology.cpp (physical_core_count()), debug_alloc.cpp (global operator new/delete override)
        jolt_job_system.h             INTERNAL: JoltJobSystemBridge (JPH::JobSystemWithBarrier over ps::jobs::JobSystem); only included from backend/jolt/
      world/world.cpp                 World's staged tick pipeline (built as a jobs::TaskGraph)
      backend/jolt/jolt_backend.cpp   the only P1 IRigidBackend implementation; every Jolt type is confined to this file + jolt_job_system.h
      articulation/                   INTERNAL (ps::articulation, not a public header); see "Articulation solver" below, incl. contact.h/.cpp (ContactCoupling) and contact_source.h (IContactSubstepSource)
      io/telemetry.cpp, io/scenario.cpp
      vehicle/ drivetrain/ aero/ machine/ terrain/   empty (.gitkeep) until P3+
  tests/
    support/                          test-only shared helpers (not part of ps_core)
      counter_force_element.h         deterministic counter-hash IForceElement used by determinism/bench tests
      catch_main.cpp                  custom Catch2 entry point installing headless CRT handlers; target ps_test_catch_main
    unit/                             target ps_unit_tests: test_hello, test_math, test_jobs (incl. the queue-full inline-fallback cases), test_world_basic, test_articulation_core (white-box, core/src on its include path), test_articulation_four_bar (white-box), test_articulation_parallel_alloc (white-box: System::step_all is one job per articulation, debug_alloc-armed), test_aba_fast_transform (white-box: fast vs. reference ABA congruence transform, 1e-12 relative), test_scenario_articulation, test_articulation_contact (Compliant/ProxyBody smoke tests on a single-DOF rail rig, incl. a ProxyBody-at-high-speed [!mayfail] case)
    validation/                       target ps_validation_tests: test_free_fall, test_pendulum, test_joint_chain_energy, test_box_on_slope, test_tunnelling, test_resting_contact, test_huge_forces, test_mass_ratio (one case [!mayfail]), test_articulation_pendulum, test_articulation_double_pendulum, test_articulation_energy, test_articulation_mass_ratio, test_articulation_floating_base, test_articulation_actuator, test_articulation_four_bar_energy (white-box, core/src on its include path), test_articulation_contact_measurements (Compliant/ProxyBody: a fixed-base arm resting on the ground, the same rig pressed with a known force, a rail rig pushing a dynamic crate, a floating-base vehicle drop, 5/20 m/s impacts; two [!mayfail] ProxyBody cases)
    determinism/                      target ps_determinism_tests: test_worker_count_hash (1/2/4/10/24 workers + 1000-run randomised-scheduling stress test; one case adds several articulations to the same scenario, one case mixes Compliant/ProxyBody contact-bearing rail rigs with plain chains)
    bench/                            target bench_force_elements (plain main, also CTest-registered): 2000-force-element worker scaling; target bench_contact_coupling (plain main, also CTest-registered): cost per tick per contact strategy, scaling with contact count; target ps_energy_study (plain main, NOT CTest-registered): Part A solver-settings energy-loss sweep plus the P1.5a Jolt-vs-articulation table and 10-link cost benchmark, energy_study.cpp
  data/
    vehicles/ tyres/ aircraft/        empty (.gitkeep) until P3+
    scenarios/                        JSON scenario files (ps::io::Scenario schema): free_fall.json, pendulum.json, joint_chain.json, articulation_chain.json, multi_articulation.json (several open chains of different sizes, a floating base, a four-bar loop closure - Windows/Linux hash-compare and worker-count determinism), contact_articulation.json (one Compliant and one ProxyBody rail rig - same hash-compare/determinism coverage for P1.5b contact coupling)
    models/                           Godot art assets (glb + rig.json); owned by a separate session, do not touch
  adapters/
    godot/                            empty (.gitkeep) until P2; built only when PS_BUILD_GODOT_ADAPTER=ON
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
  `test_aba_fast_transform.cpp`.
- `bench_force_elements` (executable, `tests/bench`): plain `main()`
  (not Catch2, so its measured numbers are always on stdout); links
  `ps_core`, `ps_headless_env`. Registered with CTest via `add_test()`.
- `bench_contact_coupling` (executable, `tests/bench`): plain `main()`;
  links `ps_core`, `ps_headless_env`. Registered with CTest via
  `add_test()` as a smoke check only (no scaling assertion, unlike
  `bench_force_elements`) - PLAN.md P1.5b measurement (f), cost per tick
  per contact strategy over 1/2/4/8/16 independent contact-bearing rigs.
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
  `bench_contact_coupling` and fetch Catch2.
- `PS_BUILD_GODOT_ADAPTER` (default `OFF`): fetch godot-cpp and build
  `adapters/godot`.

## Dependencies (FetchContent, pinned)

| Dependency   | Pin      | Notes |
|--------------|----------|-------|
| JoltPhysics  | `v5.6.0` | `DOUBLE_PRECISION ON`, `CROSS_PLATFORM_DETERMINISTIC ON`, `USE_AVX2 ON`, `USE_FMADD OFF`, GPU-compute options OFF, sample/test/viewer targets OFF |
| Catch2       | `v3.16.0`| only when `PS_BUILD_TESTS=ON` |
| nlohmann/json| `v3.12.0`| linked `PRIVATE` into `ps_core` (scenario JSON parsing, confined to `core/src/io/scenario.cpp`) |
| godot-cpp    | branch `4.5` | only when `PS_BUILD_GODOT_ADAPTER=ON` |

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
  -> terrain_tiles (stub) -> snapshot (publishes to SnapshotTripleBuffer, records one telemetry row)
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
own `Instance`'s state, PLAN.md D8's scatter/compute pattern).

Each stage records its own wall-clock duration to a
`stage.<name>.ms` telemetry channel; `World::step` also records one
`jobs.worker<N>.utilization` channel per worker (busy time / tick wall
time). `World::state_hash()` folds `IRigidBackend::compute_state_hash`
(FNV-1a over every live body's pose+velocity bytes, in ascending `BodyId`
order) and then `articulation::System::fold_state_hash` (same FNV-1a
fold, every live articulation's q/qd and, if floating, root pose) -
PLAN.md D7; a `World` with no articulations hashes bit-identically to
before this field existed.

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

## Jolt backend (`core/src/backend/jolt/jolt_backend.cpp`)

`JoltRigidBackend : IRigidBackend`. Two-layer broadphase/object-layer setup
(`NON_MOVING`/`MOVING`). Bodies: box/sphere/capsule shapes, mass via
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
(`ShapeContactHit`: body/point/normal/penetration). `ensure_jolt_registered()`
installs `JPH::Trace`/`AssertFailed`
callbacks that print to stderr (Jolt's own defaults are silent and just
trigger a bare breakpoint). Broadphase/contact-constraint capacity and the
`TempAllocatorImpl` arena size scale with `max_bodies` rather than a fixed
constant.

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

```powershell
tools\ci.ps1                      # every leg, FAST test selection (default)
tools\ci.ps1 -Full                # every leg, every test (pre-P1.5b behaviour)
tools\ci.ps1 -SkipWsl             # Windows legs only
tools\ci.ps1 -SkipWindows         # WSL legs only
tools\ci.ps1 -Only debug,asan     # named legs only (matches preset names, plus msvc-release)
```

Legs: clang-cl `debug`, `release`, `asan`; cl `msvc-release`; a grep that
fails the run if anything under `core/` includes a Godot/GDExtension header;
WSL `linux-release`, `linux-asan-ubsan`, `linux-tsan`. Prints a per-leg
PASS/FAIL summary with per-stage (configure/build/ctest) timing and a
per-leg + total wall-clock line; exits non-zero on any failure. Every
`ctest` invocation passes `--parallel` (12 on Windows, 24 in WSL - see
"Test tags" below for why heavy determinism tests are exempted from
oversubscription).

**Test selection (default = FAST, target < 5 min wall clock on the dev
machine).** `tools/ci.ps1` (no flag): the `release`, `msvc-release` and
`linux-release` legs run every test except `[long]`-tagged ones
(`ctest -LE long`); the `debug`, `asan`, `linux-asan-ubsan` and `linux-tsan`
legs run only `ps_unit_tests` + `ps_determinism_tests` + `[sanitizer]`-tagged
tests (`ctest -LE "validation|long"` - the whole `ps_validation_tests` suite
is `[validation]`-tagged, and the `[sanitizer]` short variants deliberately
omit that tag so this one exclusion routes them back in). `tools/ci.ps1
-Full`: no label filter, every test runs on every leg (the behaviour before
this split; mandatory before a phase counts done per PLAN.md section 7).

**Test tags** (`tests/*/CMakeLists.txt` pass `ADD_TAGS_AS_LABELS` to
`catch_discover_tests`, so every Catch2 tag becomes a CTest `LABEL`
selectable via `ctest -L`/`-LE`):
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
  with up to 24 workers; given CTest `PROCESSORS=24` so `ctest --parallel`
  never schedules it alongside another test and oversubscribes the
  machine's 24 logical threads.

## Dev environment setup

```powershell
tools\setup_dev_env.ps1             # install everything (idempotent)
tools\setup_dev_env.ps1 -CheckOnly  # verify only, prints versions, exits non-zero if anything is missing
```
