# physics_sim — implementation plan

Audience: Claude Opus 5, working as a coding agent in this repository.
Owner: Sebastian. Written 2026-09-21.

## 0. How to use this plan

- Work phase by phase (section 7). A phase is done only when every acceptance
  test listed for it passes in CI-style headless runs. Report failures with the
  output; never mark a phase done on "looks right".
- Decisions in section 3 are settled. Do not re-litigate them. If one turns out
  to be unworkable, stop, show the evidence, and ask Sebastian.
- Ask before: installing software, adding a dependency not listed here,
  changing a public interface of an already finished phase, pushing anywhere.
- Global rules from `~/.claude/CLAUDE.md` apply: search the knowledge vault
  before every phase (2–3 phrasings), and store findings / design decisions via
  the `store-knowledge` skill afterwards with `projects: physics_sim`. The
  repo's own `CLAUDE.md` stays a pure architecture map — no rationale there.
- You cannot feel how a vehicle drives. Everything is validated numerically
  against analytic results or published data. Subjective tuning is Sebastian's
  job, through the Godot test scenes.

## 1. Goal

A standalone, real-time, deterministic C++ physics simulation library for
ground vehicles, articulated machinery and aircraft. Two games will be built on
it later (a racing game and a farming simulator); they are **not** part of this
repo. Godot is only one possible front end and sits in a thin adapter layer.

In scope:
- Rigid multibody dynamics with joints and contacts.
- Realistic wheeled vehicles: tyres, suspension, drivetrain, brakes, steering.
- Articulated machines: trailers, hitches, three-point linkage, loader arms,
  PTO, hydraulics.
- Aerodynamics as a first-class, general system: any body or link can carry
  aero elements; forces act at their centre of pressure, so they produce
  torques on the vehicle. Must scale from "drag on a combine's unloading auger
  yaws the combine" up to "a fixed-wing aircraft flies and stalls correctly".
- Deformable terrain and soft-soil tyre interaction (needed by the farming sim).

Out of scope for now, but the architecture must not block it:
- Comprehensive body deformation (crash damage, bending structures, soft
  bodies). Section 6 lists the seams to keep open. Build none of it yet.

Not in scope at all: rendering, audio, UI, game logic, networking.

## 2. Machine and toolchain

Machine: Windows 11, AMD Ryzen 9 5900X (Zen 3: AVX2 + FMA, **no AVX-512**),
12 cores. On 2026-09-21 none of cmake, ninja, MSVC, clang, Godot were
installed; only git and the Store python stub.

Phase 0 installs (ask Sebastian first, then use winget):
- Visual Studio 2022 Build Tools (C++ workload, Windows SDK)
- LLVM (clang-cl), CMake, Ninja
- Godot 4.x latest stable (≥ 4.4), standard build (not .NET)
- A real Python 3 (for telemetry plots only; never a runtime dependency)

WSL (approved by Sebastian): WSL 2 with Ubuntu 26.04 is present, 24 threads,
~15 GB RAM, no build tools in it yet. Phase 0 installs clang, lld, cmake,
ninja, python3 there via apt. The Linux build of the core, tools and tests (not
the Godot adapter) is a required CI leg with presets `linux-release`,
`linux-asan-ubsan` and `linux-tsan`. Build inside the Linux filesystem
(`~/build/physics_sim`), not under `/mnt/s`, which is slow; the source can stay
on `S:`. Benefits beyond TSan: a second standard library and ABI keep the core
honestly portable, and determinism is additionally checked Windows-vs-Linux
(same hashes expected with Jolt's cross-platform mode and `/fp:precise` ≙
`-ffp-contract=off`; if they differ, find out why before relaxing anything).

Compiler: **clang-cl**, C++20, MSVC ABI.
Reason: MSVC x64 has no inline assembly at all; clang-cl has it, optimises
numeric code better, and stays link-compatible with godot-cpp built by MSVC.
Keep the code compiling with MSVC `cl` too (CI job), minus the asm paths.

Build: CMake + Ninja, presets `debug`, `release`, `relwithdebinfo`, `asan`.
Flags: `/arch:AVX2`, `/fp:precise` (never `/fp:fast`), FMA contraction off by
default and enabled explicitly per function where wanted — determinism depends
on it. Warnings as errors in our own code.

Dependencies, all via CMake FetchContent pinned to a commit/tag:
- JoltPhysics (MIT) — rigid-body + collision backend, see D2
- Catch2 v3 — tests and micro-benchmarks
- nlohmann/json — scenario and vehicle definition files
- godot-cpp — adapter only, branch matching the installed Godot version

## 3. Settled decisions

Record each as a vault ticket in Phase 0 (decision, alternatives, pro/contra,
reason) — the text below is the short form.

**D1 — Godot is a thin adapter; the sim owns the world.**
Godot's own physics is not used for any simulated object. The core has zero
Godot includes (enforced by a CI grep). Godot does rendering, input, audio, UI,
and hands static collision geometry to the sim. Rejected: Unreal (binary
assets, agent can't drive the editor, Chaos Vehicles too simple); driving
Godot rigid bodies from GDScript (ties the sim to one engine, no headless
tests, wrong step rates).
*Amended 2026-09-24 (Sebastian):* a port to Unreal Engine 5 stays possible, so
every piece of logic that is not rendering, input plumbing or audio-device
output lives in engine-neutral C++ libraries (no Godot or Unreal types), and
each game-engine adapter stays thin. This includes audio synthesis and
propagation (`ps_audio`, below), the engine cycle model, input mapping and
driver aids; the adapter only feeds samples to the engine's audio output,
reports listener and source poses, and supplies scene geometry through an
interface.

**D2 — Jolt is embedded as the rigid-body/collision backend, hidden behind our
own interfaces.** No Jolt type appears in any public header. Jolt does
broadphase, narrowphase, contact and joint solving, body integration. We own
everything that makes it a vehicle simulator: tyres, suspension forces,
drivetrain, aero, soil, hydraulics. Rejected: writing our own collision
detection and constraint solver first (months of work with no vehicle-specific
value; the backend seam keeps this possible later); using Jolt's built-in
`VehicleConstraint` (arcade tyre model, not extensible enough).
Build Jolt with `JPH_DOUBLE_PRECISION` (aircraft and large farm maps) and
`CROSS_PLATFORM_DETERMINISTIC`.
*Amended 2026-09-22 by D10:* joint chains of vehicles and machines are not
solved by Jolt's joints but by our own reduced-coordinate solver. Jolt keeps
collision detection, contacts and free bodies.

**D10 — Articulated chains in joint coordinates (Featherstone).** Decided by
Sebastian on 2026-09-22 after the P1 energy study (vault PHYS-003). Jolt, like
every maximal-coordinate solver, pushes joints back together after each step.
That discards energy (hinge chain at a 90° swing: −22 % in 60 s; finer steps
only reach −18 %), and it lets joints give under large mass ratios (20 t on a
10 kg link: 46 mm, and it goes unstable at 960 Hz). Our own solver works in
joint coordinates (Featherstone's articulated-body algorithm), so joints
cannot come apart by construction, and mass ratios don't matter. It runs in
the 960 Hz vehicle-island stage (D3/D8). Every link exposes a `Frame`, so
force elements attach unchanged. Closed loops (three-point linkage, hydraulic
cylinders, parallel-linkage loaders) get loop-closure constraints.
Rejected: replacing Jolt's whole solver with XPBD (little of D2 would remain,
and energy is only better, not exact); staying on Jolt with mitigations (not a
real fix). How articulations couple to Jolt contacts is settled by a measured
prototype comparison in P1.5, not here.

**D3 — Multi-rate fixed stepping.** Rigid-body tick 240 Hz. Vehicle internals
(wheel spin, tyre slip states, drivetrain, hydraulics) sub-step at 4× = 960 Hz
inside each tick, using the chassis state extrapolated within the tick, and
apply the accumulated impulse to the bodies. All rates configurable. No
wall-clock anywhere in the core. The adapter runs the sim on its own thread
and reads interpolated snapshots (lock-free triple buffer).

**D4 — Conventions.** SI units everywhere. Core frame is ISO 8855: X forward,
Y left, Z up, right-handed. The Godot adapter does the single basis change to
Godot's Y-up/−Z-forward; nothing else may convert. `ps::real` = `double` for
poses and world positions; hot per-wheel/per-element math may use `float` only
after a benchmark justifies it.

**D5 — Performance policy.** Order is: correct scalar code → benchmark →
data layout (SoA, batching per wheel/aero element) → AVX2/FMA intrinsics behind
`ps::simd` wrappers with a scalar fallback → assembly. Assembly (clang inline
asm, or NASM `.asm` files) is allowed only when a committed benchmark shows a
worthwhile gain over the intrinsics version, and every asm routine keeps a
scalar reference implementation that a test compares it against bit-for-bit
or within a stated tolerance.

**D6 — Data-driven.** Vehicles, tyres (`.tir` Pacejka files plus our JSON),
aero elements, and test scenarios are files, not code. A headless runner
executes scenarios and writes CSV telemetry.

**D7 — Determinism.** Same binary + same inputs → bit-identical state.
A replay test hashes the full world state every N ticks.
Transcendental functions (sin, cos, tan, atan2, asin, acos, exp, log, pow,
...) are never called from the platform libm in the core: platform libms round
differently (found in P1.5a: articulation hashes differed Windows vs Linux).
The core uses `ps::math` versions ported from musl libm (MIT, fdlibm-derived;
decided by Sebastian, 2026-09-22). `sqrt` is IEEE-exact and may stay. A CI grep
enforces this, and CI compares the Windows and Linux scenario hashes
automatically.

**D8 — Multithreaded by design, and still deterministic.** Everything that can
run in parallel does, from P1 on — it is not a late optimisation, because it
shapes the data layout and the interfaces. Results must not depend on the
number of threads or on scheduling (D7 stays in force).

- One work-stealing job system (`core/src/jobs/`) owns all worker threads.
  Jolt gets a `JPH::JobSystem` implementation that forwards to it, so there is
  a single pool, not two fighting for cores. Default worker count = physical
  cores − 2 (leave room for Godot's main and render threads); configurable,
  and `1` must work for debugging.
- Parallelism pattern is **scatter → compute → ordered gather**: jobs read
  immutable tick-start state, write only to their own pre-allocated,
  cache-line-aligned output slot, and a gather step applies the results in
  fixed ID order. Never sum floats in completion order. No locks, no shared
  mutable state, no allocation inside jobs on the hot path.
- What runs in parallel:
  - *Vehicle islands.* Vehicles only interact through the rigid-body backend
    at 240 Hz, so the 960 Hz internals run as one job per island. An island is
    a set of vehicles coupled below the backend level (tractor + trailer +
    implement sharing PTO/hydraulics). Islands are rebuilt when couplings
    change, and sorted by ID.
  - *Force elements.* Evaluated in batches (SoA, so the same batches feed the
    SIMD paths of D5), wrenches gathered in element-ID order.
  - *Scene queries.* Wheel shape casts and similar are batched and run in
    parallel while the backend is not stepping (Jolt queries are thread-safe
    for reading then).
  - *Backend step.* Jolt's own parallel broadphase/narrowphase/island solver on
    the shared pool.
  - *Terrain.* Soil deformation requests are collected, sorted by (tile, source
    ID), then applied one job per tile. Mesh/collision rebuilds of dirty tiles
    run as low-priority background jobs.
  - *Pipeline level.* Sim thread is decoupled from Godot (D3); telemetry and
    file IO go through a queue to an IO thread; asset/scenario loading is async.
- Randomness (gusts, turbulence, surface noise) uses counter-based generators
  keyed by (entity ID, tick, stream) — never per-thread generators.
- Granularity: do not create jobs for less than roughly 20 µs of work; batch
  instead. Be honest about limits: the sub-step chain of a *single* vehicle is
  sequential by nature, so parallel speed-up comes from many vehicles and many
  elements, not from one car.
- All parallel code goes through `ps::jobs` (`parallel_for`, `parallel_batches`,
  task graph with barriers). Nothing else in the core may create threads or
  name a threading runtime, so the backend stays replaceable.
- Rejected: a thread per subsystem (poor scaling, latency between stages);
  `std::execution::par` (no control over the pool); locking shared
  accumulators (non-deterministic summation order and contention). OpenMP and
  MPI were considered on Sebastian's request — see D9.
- ThreadSanitizer does not exist for Windows targets, so the WSL build (section
  2) carries the TSan job. On Windows use ASan plus a stress test that runs
  scenarios with randomised worker counts and job orders and compares hashes.

**D9 — OpenMP and MPI: evaluated, used only at the edges.**

*OpenMP.* Pro: mature, trivial `parallel for`, supported by clang on both
platforms, `omp simd` is a clean portable vectorisation hint. Contra for the
real-time core: (1) Jolt needs a job system handed to it, and an OpenMP team
next to it means two pools oversubscribing the same cores — inside a Godot
process that has its own threads too; (2) reductions and dynamic schedules sum
in thread-dependent order, which breaks D7 unless restricted to
`schedule(static)` without reductions — at which point little is left over
our own `parallel_for`; (3) no priorities or background jobs (terrain
rebuilds, IO) and no control over worker wake-up latency, which matters at a
4 ms tick; (4) on Windows it means shipping `libomp.dll`, and MSVC's own
OpenMP is stuck at 2.0.
Decision: no OpenMP *runtime* in the core. Allowed: `#pragma omp simd`
(compile with `-fopenmp-simd`, which needs no runtime) where it beats plain
auto-vectorisation and before reaching for intrinsics. Allowed: an optional
OpenMP backend for `ps::jobs` in headless tools if a benchmark ever shows it
ahead — the seam above makes that a contained experiment, not a rewrite.

*MPI.* It solves distributed memory across processes/machines. A real-time
game tick on one shared-memory machine gains nothing from it: message-passing
latency and process-level domain decomposition cost more than the whole 4 ms
budget saves, and a game cannot ship an MPI runtime. Decision: never in the
core or the adapter. Where it could fit is offline batch work — parameter
sweeps, tyre-model fitting, Monte-Carlo tuning, the validation suite — and
those runs are independent, so they do not need MPI either: `ps_run` is a pure
function (scenario file → CSV, no shared state), and `tools/sweep` fans
scenarios out over local processes (`--jobs N`). Because of that purity, any
launcher (MPI, GNU parallel, SSH to other machines) can distribute runs later
without touching the library. Revisit only if a future offline, high-
resolution deformation/FEM solver (P10) needs more memory than one machine.

## 4. Repository layout

```
physics_sim/
  CLAUDE.md               architecture map only
  PLAN.md                 this file
  CMakeLists.txt, CMakePresets.json
  core/
    include/ps/           public API (no Jolt, no Godot types)
    src/
      math/               vec, quat, mat, frames, simd wrappers
      jobs/               work-stealing job system, parallel_for, Jolt bridge
      world/              World, BodyId, Frame, stepping, snapshots
      backend/            IRigidBackend + jolt/ implementation
      force/              IForceElement pipeline (see section 5)
      vehicle/            suspension, tyre, steering, brakes, assembly
      drivetrain/         torque-node graph
      aero/               atmosphere, wind, surfaces, bluff bodies, propulsion
      machine/            hitches, linkages, hydraulics, PTO
      terrain/            heightfield, soil model, deformation
      io/                 json loading, .tir parser, telemetry
  tools/ps_run/           headless scenario runner (CLI → CSV)
  tools/sweep/            fan scenarios out over local processes (batch/validation)
  tools/plots/            python plotting of telemetry (dev only)
  tests/                  unit/, validation/, determinism/, bench/
  data/                   vehicles/, tyres/, aircraft/, scenarios/
  adapters/godot/         GDExtension (own CMake target) + demo Godot project
```

## 5. Core architecture

- `World` owns bodies, joints, force elements, terrain and the clock. Bodies
  and joints are handles (`BodyId`, `JointId`) into the backend.
- **`Frame`**: a transform-and-velocity provider. Today a frame is "a rigid
  body plus a local offset". Every force element attaches to a `Frame`, never
  directly to a body. This is the main seam for future deformation (section 6).
- **`IForceElement`**: `evaluate(state, dt) → wrenches on frames`. Tyres,
  springs, dampers, aero elements, hitch compliance, soil reactions are all
  force elements. Order per tick: freeze tick-start state → batched scene
  queries ∥ → sub-stepped vehicle islands ∥ → remaining force elements ∥ →
  ordered gather of wrenches → backend step ∥ → terrain tiles ∥ → snapshot
  (∥ = parallel stage, see D8). Stages are barriers; inside a stage jobs never
  see each other's output. Implementations of `IForceElement` must be const
  with respect to shared state and declare what they read.
- **`IRigidBackend`**: create/destroy bodies, shapes, joints; apply wrench;
  step; ray/shape casts; contact listener. Jolt implementation only, for now.
- Telemetry: any component can register named channels; runner writes CSV.
- A flat C ABI (`ps_c.h`) over the public API is a later nice-to-have; keep
  public headers free of templates-in-signatures so it stays easy.

### 5.1 Vehicle
- Suspension: per-corner geometry (start with double-wishbone/MacPherson as
  kinematic tables: camber, toe, track change vs. travel), spring, damper
  curves, bump stops, anti-roll bars; rigid and pendulum axles for tractors.
  Wheel contact via shape casts (cylinder), not single rays.
- Tyre: Pacejka Magic Formula (MF 5.2/6.1 subset) with combined slip, load
  sensitivity, camber, relaxation length, rolling resistance, aligning and
  overturning moment. Known pitfall: slip definitions blow up near standstill —
  implement a low-speed blend (damped stiffness model) and test parking-on-a-
  slope explicitly. Separate tyre-model interface so a brush/TMeasy or
  soft-soil model can replace it per surface.
- Drivetrain: graph of rotational nodes and torque components — engine (torque
  map, inertia, friction, limiter), clutch, torque converter, gearbox, shafts
  with compliance, open/LSD/locked/viscous differentials, transfer case, brakes,
  PTO branch, hydraulic pump load. Stiff couplings (clutch lock-up, locked
  diff) need implicit or constraint-based solving, not explicit springs.
- Steering incl. Ackermann, articulated steering (wheel loaders), rear steer.

### 5.2 Aerodynamics
- Atmosphere: ISA density/temperature/speed of sound vs. altitude. Wind field:
  constant + gust, turbulence (Dryden) later.
- Local flow per element = −(frame linear velocity + ω × r) + wind. Using the
  local velocity at each element gives aerodynamic damping for free.
- **Bluff elements** (boxes, cylinders, plates): direction-dependent drag
  area, centre of pressure offset. Used for vehicle bodies, cabs, implements,
  booms, the combine auger. Because the element sits on the arm's frame, its
  wrench goes into the arm link and reaches the chassis through the joint —
  swinging the arm out changes the yaw moment with no special code.
- **Lifting surfaces**: strip elements with CL/CD/CM over the full ±180° angle
  of attack (airfoil table pre-stall, flat-plate model post-stall, blended),
  finite-span correction, control-surface deflection, ground effect. An
  aircraft is a body plus a set of strips; stall, spin and adverse yaw must
  emerge from the strips, not be scripted.
- **Aero maps** for race cars: CL·A, CD·A and balance as functions of front/
  rear ride height, yaw and wing settings, applied as a wrench — the standard
  motorsport approach where strip theory is useless (underbody, diffuser).
- Propulsion: propeller from CT/CP vs. advance ratio, with torque reaction and
  propwash over downstream strips; simple jet thrust model.
- Later: wake/slipstream field for drafting.

### 5.3 Machines
Hitches (ball, drawbar, fifth wheel), three-point linkage with lift hydraulics
and draft/position control, loader arms, hydraulic cylinders as force-limited
actuators with a pressure/flow model, PTO shaft loads, implement draft forces
from soil.

### 5.4 Terrain and soil
Heightfield with material layers. Soft soil: Bekker–Wong pressure–sinkage plus
Janosi–Hanamoto shear, multi-pass effect, rut formation written back into the
heightfield (SCM-style; Project Chrono's SCM is the reference to check
against). Dirty regions are reported to the adapter for mesh updates.

## 6. Seams to keep open for future deformation

Build none of this now; just do not violate it.
- Force elements attach to `Frame`s (5.). Later a frame can be a node of a
  deformable structure instead of a rigid body.
- Mass properties are queried from the body each tick, never cached in
  vehicle/aero code.
- Collision shapes are referenced through our own handle so a shape can be
  swapped or re-meshed at runtime.
- `World` stepping has a named slot for additional solvers between force
  evaluation and backend step (a future node-beam / FEM / XPBD solver couples
  there through attachment constraints and contacts).
- Snapshots carry an optional per-body "deformation payload" blob so the
  adapter protocol does not need to change.

Seams for later terrain sources (P11, real-world geodata). Build these into
the terrain tiles from the start:
- `ITerrainSource`: fills a tile (heights, per-cell surface ids) for a tile
  key. The procedural `terrain.json` spec is one implementation; geodata will
  be another. Streaming, caching, collision and the adapter's render meshes
  sit above this interface and do not know the source.
- `IStaticStructureSource`: supplies static geometry per tile (roads,
  buildings, bridges) as engine-neutral meshes with a surface id per triangle.
  The test features (straights, ramps, circuit) use the same path.
- Render meshes for tiles and structures are built in the library as plain
  vertex/index buffers; adapters only upload them. That keeps every front end
  equal and never Godot-only.
- World frame of a scene: local east-north-up (x east, y north, z up), with an
  optional georeference (origin lat/lon/height plus projection) in the
  scenario. Vehicle body frames stay ISO 8855 (D4).
- The surface table is data (μ scale, rolling resistance, later soil
  parameters), shared by all sources.
- `IWorldShape` (for optional planet curvature, P11): answers `gravity(p)`,
  `up(p)`, `altitude(p)` and `local_frame(p)` for a world position. The flat
  implementation returns constant gravity and z as the altitude; a WGS84
  implementation comes with P11. From the terrain tiles on, new code never
  assumes that −z is down or that z is the altitude: gravity is applied per
  body/link from `gravity(p)`, and atmosphere, wind and suspension references
  use the local frame. Tile keys are opaque (not raw planar x/y indices), so a
  geodetic or cube-sphere tiling can replace the planar grid. The existing
  constant `WorldConfig::gravity` becomes the flat world shape's parameter.

## 7. Phases and acceptance tests

Each phase ends with: tests green, `CLAUDE.md` map updated, vault tickets
written, one git commit (or a few) with clear messages. No pushing.
CI has two levels (Sebastian, 2026-09-22): `tools/ci.ps1` is the fast
everyday run (target < 5 min; long physics tests only in the release legs,
short variants under the sanitizers). `tools/ci.ps1 -Full` runs everything in
every leg, and is mandatory before a phase counts as done.
Per-change testing (Sebastian, 2026-09-23: full CI after every change made
development infeasibly slow): during development run only the tests for the
topics a change touches (`tools/ci.ps1 -Affected`, which maps changed paths to
subsystem test labels and falls back to the full fast suite for foundational
or unmapped paths); the fast run at batch boundaries and before reporting work
as done; `-Full` before a phase counts as done or after foundational changes.

**P0 — Toolchain and skeleton.** Installs (after approval), git init, CMake
presets, FetchContent pins, empty targets, Catch2 hello test, CI script
`tools/ci.ps1` (configure, build clang-cl + cl, run tests, Godot-include grep,
then the WSL leg: release, ASan+UBSan, TSan).
D1–D8 are already in the vault as PHYS-001 (note: its D-numbering differs
slightly from this file; go by the headings) — extend it via `update-file`
rather than creating duplicates.

**P1 — Core world.** Math, job system with Jolt bridge (D8), `World`, `Frame`,
`IForceElement`, Jolt backend, staged parallel stepping, snapshots, telemetry,
`ps_run`, JSON scenarios.
Accept: state hash identical with 1, 2, 4, 10 and 24 workers and under the
randomised-scheduling stress test (1000 runs); TSan leg in WSL clean with no
suppressions in our own code; Windows and Linux hashes equal; `tools/sweep`
runs N scenarios over local processes; 2000 independent force elements
scale ≥ 6× from 1 to 10 workers; no allocation inside jobs (checked by a
debug allocator hook); free fall matches analytic to 1e-6 rel. over 10 s; pendulum period
within 0.1 %; energy drift of a frictionless joint chain < 0.5 % over 60 s;
box-on-slope sticks/slides at the right friction angle; determinism hash
identical across 3 runs and across debug/release of the same compiler.

**P1.5 — Articulation solver (D10).** Two steps.
*a) Core:* `ps::articulation` with a fixed or floating (6-DoF) root;
revolute, prismatic and fixed joints; joint forces/limits; ABA forward
dynamics (Featherstone, "Rigid Body Dynamics Algorithms", 2008, ch. 7), CRBA
and RNEA as cross-checks; loop-closure constraints; the integrator chosen by
measurement; one job per articulation in the vehicle-island stage; a `Frame`
per link.
Accept: ABA accelerations equal CRBA solve M⁻¹(τ−C) to 1e-9 rel.; single
pendulum at large amplitude matches the elliptic-integral period within 0.1 %;
double pendulum at small amplitude matches its two normal-mode frequencies
within 0.1 %; energy drift < 0.5 % over 60 s without damping (single link at
90° and 170°, 3-link chain at 90°); 2000:1 mass ratio in both orders is stable
with drift < 0.5 %; a floating base without gravity conserves momentum and
angular momentum to 1e-6 rel. over 60 s; four-bar linkage: closure error
< 0.1 mm, drift < 0.5 %, output angle matches closed-form kinematics to 1e-6
rad; a force-limited prismatic actuator stalls at its force limit; thread-count
determinism and Windows = Linux hashes; cost of a 10-link articulation per
960 Hz substep measured and reported.
*Decided 2026-09-23 (measured, see vault PHYS-012/PHYS-013):* the default is
**Compliant** — we compute contact forces ourselves (Hunt-Crossley plus
regularised Coulomb friction), evaluated at each RK4 stage's own state, with
Jolt as a pure collision query. Measured: settles within 0.01 cm gently and
1 cm from 9.6 m/s, matches the analytic equilibrium under a 500 N load to
2 mm, no tunnelling at 20 m/s, 10-100 µs per contact.
ProxyBody (Jolt solves the contacts through a driven proxy body) stays in the
code as a comparison and fallback, but is not the default: its one-tick delay
cannot follow a step load (a 500 N step sent the arm into a 688 m fall), it
leaves 11.5 cm penetration at a 20 m/s impact, and it needs `proxy_mass_scale`
set by hand whenever the contact link rigidly carries more mass behind it.
Both keep determinism across worker counts and Windows/Linux.
*b) Coupling prototype:* at least two strategies for how articulation links
interact with Jolt contacts (e.g. compliant contact force elements with Jolt
as a pure collision query, and proxy Jolt bodies with contact impulses read
back), measured on the same scenarios: arm bucket resting on the ground,
pushing a 500 kg Jolt crate, articulated vehicle on the ground. Metrics:
penetration, stability, energy, action = reaction, cost. Sebastian decides.

**P2 — Minimal Godot adapter.** GDExtension exposing: load scenario, step
thread, snapshot → Node3D transforms, input → sim controls, debug draw of
force vectors, static mesh/heightmap → sim collision. Demo project with a flat
test ground.
Front-end performance target (Sebastian, 2026-09-23): the games should run at
4K and 240 Hz on his machine (Ryzen 9 5900X, Radeon RX 7900 XT, 32 GB) with
decent 2026-era visuals. Native 4K240 is out of reach for that GPU, so the
realistic path is rendering at a lower internal resolution with upscaling
(Godot's FSR) plus driver-side frame generation, and an art style that avoids
the heaviest effects. What this repo owes that target: the adapter's per-frame
main-thread work must fit far inside a 4.2 ms frame — bulk transform updates,
instancing for repeated objects, never blocking on the sim, and no per-body
script work. Measure it: adapter main-thread time per frame with 30 vehicles
visible, reported as a telemetry channel, target < 1 ms.
Large worlds: the sim works in double precision (D2/D4), but standard Godot
renders in float32, which gives about 4 mm steps at 50 km, so the camera and
car jitter. The adapter therefore uses origin rebasing: Godot positions are
relative to a floating origin near the camera, moved in large steps; the
adapter's single ISO→Godot conversion applies the offset. A double-precision
custom Godot build is rejected for now (own engine build, godot-cpp must
match). Accept: no visible jitter at ±50 km (camera-relative error < 0.1 mm,
checked numerically).
The adapter streams the terrain tiles' render meshes from the same tiles the
sim uses (no second height source).
Gamepad support (Sebastian, 2026-09-22): Xbox One / Series controllers via
Godot's joypad input, also keyboard. Actions: steer (analog stick), throttle
and brake (separate analog triggers), handbrake, shift up/down, clutch,
camera, reset vehicle. All remappable; deadzone and steering curve
configurable; rumble output as a hook (fed from tyre slip/ABS later). Input
reaches the sim only as named control channels (e.g. `steer` −1…1,
`throttle` 0…1) in ISO/SI terms, never as Godot types.
Accept: P1 scenarios play back in Godot; the sim keeps 240 Hz with Godot at
any frame rate; grep proves core has no Godot include; headless
`godot --headless` smoke test passes; the input mapping (deadzone, curve,
trigger axes) is tested with synthetic input events.

**M1 — First test drive (after P3).** On the test ground (`data/tracks/
test_ground`, see below): one car, P3 tyres and suspension, a placeholder
drive of throttle → torque at the driven wheels from a torque curve (no gears,
no clutch — they come in P4), brakes, steering, on the Xbox controller in
Godot. Sebastian drives; subjective tuning is his.
Test ground (Sebastian, 2026-09-22: must be large, ±50 km = 100 × 100 km):
two parts.
- *Procedural terrain* over the full extent, described by a spec
  (`terrain.json`: seed, noise height function, flat zones, surface rules).
  The core evaluates it deterministically at runtime, in tiles streamed around
  active vehicles, as heightfield collision with per-cell surfaces. This is
  the first piece of the P8 terrain system (heightfield tiles, material
  layers, dirty regions), pulled forward; soil deformation stays in P8.
  Golden sample points from the Python reference evaluator must match the C++
  implementation bit for bit.
- *Test features* as exact static meshes on flat zones (z = 0): a 1 km
  asphalt strip, a 10 km top-speed straight, a skidpad (radius 40 m, for P3),
  grass and dirt surfaces, a split-μ strip (for P4), ramps at 5/10/15/30/50 %
  (15 % = the P3 parking test, steep ones for the tractor), a side slope, a
  small asphalt circuit, and an airstrip, 1500 × 30 m, for P6.
Surfaces carry physical parameters (μ scale, rolling resistance) as data with
the source noted; visually plain; a world-space grid shader in Godot as the
speed reference.
Terrain-tile acceptance (before M1): golden points match; tiles stream around
a vehicle driving 50 km without hitches in the 240 Hz tick (per-tick cost
measured); no seams between tiles (a wheel crossing a tile border sees
continuous height and normal); determinism across thread counts and
Windows/Linux as in P1.

**P3 — Chassis, suspension, tyres.** One passenger car and one tractor defined
in JSON, with published or plausible `.tir` data (note the source).
Accept: tyre model reproduces the `.tir` reference curves (Fx, Fy, Mz,
combined) within 1 %; static corner weights match hand calculation; steady-
state skidpad lateral g within 3 % of the tyre-limited estimate; 100–0 km/h
braking distance within 3 % of μ-based estimate incl. load transfer; car
parked on a 15 % slope does not creep (< 1 mm over 30 s); step-steer response
is stable, no oscillation growth.

**P4 — Drivetrain.** Carried over from P3 (2026-09-23): the 100-0 km/h braking
test still sits about 9 % above its μ-based estimate (43.8 → 41.1 m after the
test's brake control was fixed to regulate both axles). Root cause is
control-theoretic, not the tyre model: the peak-slip point is locally
unstable and the wheel model's brake torque acts as a hard velocity clamp, so
any fixed-rate slip loop oscillates around the peak and loses average grip.
A real ABS and brake-proportioning model belongs here; re-measure then.

Also carried over from P3 (2026-09-23), two findings from the near-standstill
lateral oscillation fix:

- *The tractor does not hold a cross slope.* On the 10 % cross slope
  (`spawn_side_slope`) it slides about 18 m in 10 s, measured with the
  standstill damping fix fully disabled, so this predates that fix and is
  not caused by it (the fix takes it to about 21 m). No CTest has ever
  covered tractor lateral slope holding — the parking-slope criterion above
  is car-only and purely longitudinal. Give the tractor its own slope
  acceptance test here and root-cause the sliding; the suspicion is the
  tractor's much longer relaxation lengths (0.30/0.38 m vs the car's 0.15 m)
  against the same lateral path, but that is untested.
- *The standstill damping coefficient compensates for a numerical defect,
  not a physical one.* It was demonstrated numerically, not physically: the
  same yaw kick that grows at the 240 Hz tick decays at 4× that rate. The
  cause is `VehicleInstance::step_tick` applying its substep-averaged wrench
  once per tick against a chassis velocity frozen at tick start, which turns
  the continuous system's zero damping margin negative at standstill. The
  shipped damper buys back enough continuous-time margin to stay stable
  despite that delay; it does not repair it. The real repair — applying the
  wrench per substep against an updated chassis velocity, or integrating the
  lateral relaxation implicitly — belongs in a drivetrain/solver pass here.
  Whoever does it must re-derive the damping coefficient rather than leave it
  stacked on top of a delay that no longer exists.

Engine model requirement (Sebastian, 2026-09-23): engines must eventually be
specified in a GUI (the GUI is not built yet). The engine is therefore
defined by data, editable at runtime, with no GUI dependency in core:
cylinder count (and possibly chamber shape), fuel type, bore and stroke per
cylinder, ignition-timing strategies, turbo- and supercharging, ambient
density (altitude, from the P5 atmosphere), rotary (Wankel) engines,
hydrogen combustion (added 2026-09-23), and an open-ended list beyond that — so the model must be extensible. The same
model must deliver both power (torque at the crank) and sound. engine-sim
(github.com/ange-yaghi/engine-sim) was assessed; decided 2026-09-23 (Sebastian,
vault PHYS-028): our own engine model, in three layers — a deterministic
mean-value model in core running every substep for every vehicle, its maps
generated from the engine data file by our own crank-angle-resolved cycle
model at load/edit time (this is where rotary, hydrogen, forced induction and
altitude come from), and our own sound synthesiser (in `ps_audio`, see
below; originally written as "in the adapter").
engine-sim is a reference and a source of example engines only: it fails
D7/D10/D3 and the P9 budget and covers little of the list above.
Refined 2026-09-24 by measurement (vault PHYS-038), on Sebastian's rule "live
if the performance allows it, otherwise lookup tables": a live crank-angle
model for every vehicle costs 1.4–2.3× the whole P9(a) budget, so power stays
on maps in core. Audio does not come from the maps: the same cycle model runs
live in `ps_audio` (hosted by the adapter), speed-slaved to the core (about 4 % of one core per
audible V8), with crank-angle tables from the map generator as a fallback. The
generator integrates with a fixed time step (the model is stiff), not a fixed
crank angle.

Tyre audio requirement (Sebastian, 2026-09-24): tyre sounds that depend on
compound, surface, slip angle, slip ratio, load and speed. Approach: parametric
synthesis in `ps_audio` driven by per-wheel physics state the core already
publishes (slip ratio/angle, load, surface, speed), plus a tyre compound id and
the sliding velocity at the patch; recorded or synthesised layers per surface
(rolling roar, squeal near and past the grip peak, scrub/skid, gravel crunch)
blended by those parameters, not fixed loops. The core only publishes state;
all audio lives in `ps_audio`.

Audio library `ps_audio` (Sebastian, 2026-09-24: no Godot-specific logic, a
UE5 port must stay possible): a separate engine-neutral C++ library target,
never linked into `ps_core` and never read by the simulation (so D7 and the
P9 budget are unaffected). Input: published `WorldSnapshot` state (engine
sound state, per-wheel tyre state, mount forces), listener pose, and scene
geometry through an interface. Output: PCM buffers per listener. It owns the
engine cycle model instance for audio, tyre synthesis, the interior/exterior
paths and propagation. Adapters (Godot now, possibly Unreal later) only pump
buffers to the audio device and forward poses and geometry. Unit-tested
headlessly like the core.
Not yet specified (gap noted 2026-09-28, to be designed before `ps_audio`
starts): how the live cycle model's gas flows become sound. The C15a model is
one cylinder between fixed reservoirs with a lumped runner, which cannot carry
pressure waves. Missing: all cylinders phased by firing order (and bank
angle); a 1D acoustic model of the intake and exhaust pipe networks (headers,
collector, silencers, tailpipe; e.g. waveguides) excited by the port mass
flows and radiating at the openings; the exhaust/intake geometry in the engine
data file; cycle-to-cycle combustion variation; mechanical noise; the audio
sample rate vs the cycle model's step (1/(960*48) s is ~46 kHz).
The pipe geometry is part of the ENGINE, not of audio (Sebastian,
2026-09-28): it shapes power too (wave tuning, scavenging, ram effect, back
pressure). One intake/exhaust description in the engine data file and one 1D
pipe model serve both the map generator (a kGeneratorVersion bump) and
`ps_audio`; today's maps lack these effects, one reason the I7c validation
excludes >= 5500 rpm.
Design: out/audio_plan.md (2026-09-28, increments A1-A12; vault PHYS-054).
Owner rulings on its open questions (Sebastian, 2026-09-28):
- Q1 split: the engine file owns throttle-body inlet through the first flange
  after the last engine-mounted part; an installation file referenced by the
  vehicle owns airbox/snorkel, catalyst and everything downstream, and the
  outlet positions; each engine names a `default_installation` (A25A: the
  EPA test-cell exhaust). Maps keyed by engine + installation acoustic
  content, never positions.
- Q2: one live voice (the player car), other audible cars on the crank-angle
  table fallback - unless the SIMD increment (A9) makes room for more live
  voices, then use it. A9 is a prerequisite for a V8/V12 player car.
- Q3: pipes first, but the owner's first listening test may come before the
  silencer (and catalyst) elements exist - only those may be missing, the
  rest of the pipe model (primaries, collector, tailpipe) must be there.
- Q4: engines with a torque map (tractor diesel) get an audio-only cycle
  block ([AS], no turbo until I7d).
- Q5: miniaudio (public domain / MIT-0), for `tools/ps_engine_audio --live`
  only.

Engine mounts, interior sound and propagation (Sebastian, 2026-09-24):
- Engine mounts: the engine block sits on mounts with stiffness and damping,
  so the crank reaction torque and firing pulses reach the chassis through
  them. Low frequencies pass through and audibly and visibly rock the car;
  higher ones are isolated. Physics side: model the block as its own degree of
  freedom (roll about the crank axis, later more) on mount spring-dampers
  between drivetrain and chassis, replacing the direct rotor-reaction torque
  of Q11 where a vehicle defines mounts; publish the mount forces. This also
  gives real engine rock in vehicle dynamics.
- Interior vs exterior: `ps_audio` renders two listener paths. Exterior:
  exhaust, intake, tyres and wind as sources with distance, direction and
  Doppler. Interior: structure-borne sound from the published mount forces
  (and suspension forces) through a cabin transfer model, plus airborne sound
  through firewall and glazing as filtered transmission.
- Propagation: later, geometric or ray-traced audio (reflections, occlusion,
  tunnels, walls) in `ps_audio`, with scene geometry supplied by the adapter
  through an interface; evaluate an existing open-source, engine-neutral
  spatial audio library before writing one.

Data format documentation and an engine editor (Sebastian, 2026-09-24): a
human-readable description of every data file format, kept in the repo next
to the data, and a GUI for creating engines — for now an out-of-game web page
that reads and writes the same JSON files the sim loads, with its fields
checked against what the loaders accept.

Electric drive requirement (Sebastian, 2026-09-23): a parallel system for
electric drives — battery electric, fuel cell, and other sources — alongside
the combustion engine, equally data-defined. Consequence for the design: the
drivetrain must not assume a combustion engine. Engines, electric machines,
energy stores (fuel tank, hydrogen tank, battery) and converters (fuel cell,
inverter) attach to shafts and energy buses through common interfaces, so
series and parallel hybrids fall out of composition rather than special
cases. The component set and interfaces are designed before either power
source is implemented.

Tanks (Sebastian, 2026-09-24): fluids and gases in tanks get no detailed
simulation; every store is a fill state plus a pressure given as data.
N2O tanks and nitrous injection for combustion engines are required too.

P4 design decided 2026-09-24 (vault PHYS-032 = decision record, PHYS-033 =
full specification with the owner's rulings, which implementers follow):
powertrain graph per vehicle in `ps::drivetrain`; reduced-coordinate,
linearly-implicit shaft network per substep; exact Coulomb friction
(box-LCP, capped pivoting); wheel spin moves into the network; the one-tick
wrench delay is repaired first (shadow chassis per substep) and the standstill
compensation replaced; no legacy data converter — existing data files are
rewritten and loaders reject outdated formats. Increments: I0 housekeeping,
I1 wrench-delay repair and tractor cross slope, I2 shaft network and friction,
I3 first drivable (torque-map engine, clutch, manual gearbox, open diff,
minimal drag), then I4 diffs/PTO, I5 ABS/TC, I6 ESC, I7 mean-value engine and
fuel tank, N2O, I8 map generator, I9 electric and hybrids, I10 automatics.

Status 2026-09-26: I0-I4 done. I4 (C1-C16) closed by `tools\ci.ps1 -Full`
(7 legs, all 11 cross-platform scenario hashes equal); the only failing test
on every leg is the pre-existing Spec test 10.1 (vault PHYS-037, fails
identically at be36545, root cause not diagnosed). Next: I7 (below).

Engine options and order (Sebastian, 2026-09-26): I7 moves up to come right
after I4 closes, before the tractor demo. Two generic engine types must exist
side by side, selected per engine data file:
- Graph engine: a lookup table mapping rpm to maximum torque (the existing
  torque-map engine), plus an optional fuel consumption map. Meant for users
  and developers who want to test and experiment without dealing with real
  combustion-engine engineering trade-offs.
- Simulated engine: the full physically simulated engine (PHYS-028/038: a
  mean-value runtime model in core fed by the cycle-model map generator).
A vehicle file using the graph engine without a fuel map must behave exactly
as before.

I7 scope and order (Sebastian, 2026-09-26, rulings on the I7 plan):
- One simulated-engine track: I7a graph-engine fuel map, fuel files and fuel
  tank; I7b mean-value runtime; I7c cycle-model map generator for NA petrol
  engines plus the first simulated engine; I7d turbo (wastegate, blow-off
  valve, intercooler) plus anti-lag; I7e rotary cycle model and a simulated
  R26B for the 787B; N2O after that. The former I8 generator is absorbed.
- The tractor demo starts after the whole I7 track, not after I7a.
  Superseded 2026-09-27 (night): the tractor demo moves back to run in
  parallel with I7b/I7c, at low priority - a car with a simulated engine by
  the morning takes precedence over the tractor being ready.
- I7c's first simulated engine (Sebastian, 2026-09-27): the Toyota 2.5 L
  A25A-FKS, whose EPA NVFEL test-cell data (C7) is the dyno reference; the
  held-out second engine is the Mazda 2.0 L SkyActiv-G (EPA benchmark data).
  Public-domain EPA/data.gov packages and open-access papers may be
  downloaded for it without asking each time.
- Overnight working mode (Sebastian, 2026-09-27): decisions that would need
  an owner ruling take the recommended option and are recorded here as
  PROVISIONAL for confirmation; reviewed, CI-green commits are merged.
  PROVISIONAL decisions taken overnight 2026-09-27/28, for owner review
  (details and alternatives: vault PHYS-033 and PHYS-048, out/i7c_plan.md
  section 2, out/handover_overnight.md):
  - I7a C7 (merged): the A25A fuel map is fitted on Pareto-non-dominated rows
    per speed line. This drops the lambda 0.63-0.73 ECU protection-enrichment
    rows, so the model does not reproduce the real car's low-speed WOT
    torque limitation. WOT = the per-speed-line measured torque envelope.
    Fuel is interpolated linearly down to the motoring point. The axis is
    extended to 6600 rpm, where the limiter is ASSUMED.
  - I7c D1: the frozen-literature bright line - the measured description
    and ECU schedules are inputs; measured torque, fuel, air and BSFC are
    never inputs; no calibration.
  - I7c D2: +-10 % is asserted only in the WOT window, 2000-5000 rpm (A25A)
    and 1500-4500 rpm (Mazda).
  - I7c D3: EGR is not modelled.
  - I7c D5: fixed-step RK4 at h = 1/(960*24) s (Preview 1/(960*12) s) with a
    fixed cycle count.
  - I7c D6: ps::math first, fast math later with a version bump.
  - I7c D7-D9: maps are generated at load, with an opt-in cache; a hot swap
    is a pointer exchange at begin_tick.
  - I7c D10: a single-cylinder generator between fixed reservoirs.
  - I7c D11: MBT-tracking spark; the knock limit comes in C17b.
  - I7c D12: the morning car uses measured-derived maps (M-B), labelled as
    not validating the generator.
  - I7c D13: friction is the measured motoring FMEP of a third engine (the
    2013 GM 2.5 L Ecotec EPA package) if usable, else Barnes-Moss.
  - I7c D14: Wiebe a = 5, m = 2.
  - C17-pre: bore/stroke are labelled [A] on four agreeing open-web sources,
    with no primary Toyota document read. Valve lift is a shape-fit, not a
    pixel trace.
- Throttle path: the engine never reads the pedal directly. A separate
  system passes the driver's input on to it - a mechanical linkage (pedal
  cable to the throttle bodies) or an ECU (drive-by-wire). The engine data
  file only declares whether its throttle takes that input mechanically or
  electronically.
- Turbo sound state (Sebastian, 2026-09-26): the sound system later needs
  the turbo's state to generate noises. I7d publishes it race-free in the
  powertrain snapshot (shaft speed, boost, wastegate, blow-off and anti-lag
  events with counters, surge), sound-only fields unhashed, no audio code in
  core.
- Taken as recommended: fuel map as mass flow over rpm x load; no existing
  data file edited in I7 (fuel goes into new variant files); chassis mass
  constant as fuel burns in I7a (superseded later, see "Fuel" below); engine type chosen by "kind": "simulated" plus
  an architecture field; generated maps never committed; the graph engine
  ignores altitude; refuelling only through a test/game hook.
- Cable throttle and the pedal law (Sebastian, 2026-09-26): an engine whose
  throttle input is mechanical is exempt from the 2026-09-25 torque-fraction
  pedal semantics below - the cable sets the throttle position directly.
  The simulated R26B for the 787B declares an electronic throttle.
- Compressor surge (Sebastian, 2026-09-26): I7d only flags surge and counts
  events for the sound system. A real surge model stays an optional later
  item, taken up only if the results turn out unsatisfactory.
- Generated A25A follow-up rulings (Sebastian, 2026-09-28; C17a M-A missed
  the WOT torque window, vault PHYS-055), each fixed BEFORE any re-run:
  - Runner: use a CITED A25A intake runner length if one can be found (Tier 1);
    otherwise keep the quarter-wave rule and S1's [!mayfail] at 3000-4500 rpm.
    The ps_audio 1D pipe model (A5/A6) replaces the lumped runner later. The
    0.25L runner that happens to match is rejected as a fit.
  - Spark: the measured EPA WOT/part-load spark schedule becomes a Tier 1b ECU
    input (like cam phase and lambda), plus a Tier 2 frozen literature
    flame-development (ignition) delay, since the model has none. This
    supersedes D11's "measured spark schedules are NOT used by the shipped
    runtime" for engines that have a measured schedule; engines without one
    keep mbt_tracking.
  - Friction: keep the F4 Ecotec correlation. A25A motoring data stays
    forbidden as an input (D1).
- Second follow-up rulings (Sebastian, 2026-09-28, after the measured spark
  schedule: the eta term turned -9..-19 N*m, vault PHYS-055), fixed BEFORE any
  re-run:
  - Burn duration: replace the generic 60 deg 0-100 % Wiebe with a CITED 10-90 %
    burn angle for modern fast-burn (high-tumble) SI engines at WOT / medium
    speed, converted through the frozen Wiebe shape (a 5, m 2: 10-90 % = 0.4959
    of the 0-100 % duration). Chosen from the literature before the model is
    run; one constant, no speed dependence unless the descriptor already has
    one. The ignition delay is re-derived by its existing rule (max(0, cited
    spark-to-10 % - the Wiebe's own 0-10 % angle)). No value is picked because
    it makes S1 pass.
  - S2 WOT BSFC stays [!mayfail] (confirmed): fuel follows air through the
    prescribed lambda, so it fails on the known runner air excess.
- Pipe cell size for maps (Sebastian, 2026-09-28, ps_audio A5 P17): keep
  power_dx 50 mm. Measured: VE at 50 vs 25 mm differs -0.72 % at 1500 rpm /
  30 kPa (+0.42 / -0.45 % at 3000/60 and 6000/90; 25 vs 12.5 mm -0.37 %),
  against the plan's 0.5 % [A] target. The C2 test stays [!mayfail] as an
  accepted, measured miss; revisit once A9 SIMD makes finer cells affordable.
- Sound before hook-up (Sebastian, 2026-09-29, after the first listening test
  of the A10 demo I4: "someone imitating engine sounds using a Red Bull can",
  about the same as engine-sim): refine the sound before any wider hook-up.
  A9 (SIMD) and A11 (Godot) wait behind a sound-realism pass that adds the
  missing sources and environment one at a time with an A/B render per step
  (measured on the first render: almost no energy above 2 kHz; a few pipe
  resonances at 125-500 Hz dominate). The A8b blow-up fix comes first (fine
  audio cells are unusable until then).
- Order (Sebastian, 2026-09-28): `ps_audio` (engine sound from the live cycle
  model, after its design pass closes the acoustic gap noted under "Audio
  library") is implemented BEFORE the engine design GUI below.
- Engine design GUI (Sebastian, 2026-09-28): once the "specs -> engine"
  workflow works (I7c C16a map generation plus C17a), build a GUI to
  specify and design an engine. A website is acceptable for now if that is
  much faster. Constraints already in force: the GUI edits the engine data
  file only (no GUI dependency in core, engine-neutral logic, D1 frozen
  literature for anything shipped). Proposed shape, to confirm when it
  starts: a local web page that writes the engine JSON and asks a small
  local tool (a command-line front end to generate_engine_maps at Preview
  quality) for the resulting torque, power and fuel curves; compiling core
  to WebAssembly is the alternative if a hosted page is wanted.
  Required view (Sebastian, 2026-09-28): while designing, the page shows an
  rpm vs torque / power chart (full-throttle curves over the engine's rpm
  range), refreshed as parameters change. Preview-quality generation
  (0.03-0.3 s on 10 workers) plus the C16b .psmaps cache should keep it
  interactive; each change regenerates the maps, not only the chart.
  Sound preview (Sebastian, 2026-09-28): the page also lets the designer
  hear the engine free-revving without a car - a throttle control drives an
  engine-only simulation (no vehicle, no clutch load) in real time, and
  `ps_audio`'s live cycle model (below: no samples, the sound comes from the
  engine's own geometry and combustion) renders it. This needs `ps_audio` and
  a real-time engine-only run mode; tools/plots/rpm_to_wav.py is only an
  offline placeholder with an artistic pulse shape, not this model.
- Fun engine (Sebastian, 2026-09-28): once development is far enough, a
  simulated Dodge Challenger SRT Demon 170 engine (6.2 L supercharged V8,
  E85) in a car of its own. Not before these exist: the I7c generator
  producing a multi-cylinder V8 from its description; a crank-driven
  positive-displacement supercharger (pulley ratio, bypass valve, drive
  power taken from the crank) - I7d as scoped covers turbochargers only,
  so this is an addition to I7d or right after it; knock-limited spark
  (I7c C17b) so the pump-fuel and E85 figures differ for a physical
  reason. Data rule unchanged: cited manufacturer figures (peak power and
  torque on E85 and on pump fuel, bore/stroke) are the only validation
  targets, never inputs; everything else labelled assumed. Open for then:
  the real car has an 8-speed torque-converter automatic, which the
  project does not have yet (stand-in: sedan-style manual with the
  auto-shift assist, labelled), and drag radials, for which no citable
  tyre data is known yet.
- Shift logic and the pedal (Sebastian, 2026-09-26): on a manual gearbox the
  auto-shift, auto-clutch and auto-blip are game driver assists for players
  who do not want to shift, not a gear controller; auto-shift is not a
  requirement (the existing assist stays). A future automatic gearbox's
  controller acts on the torque request (after the pedal map), not the raw
  pedal.
  The manual gearbox's assists switch to the same torque request (the
  throttle stage's command) in I7a, right after the fuel map (C5): sedans
  stay bit-identical, the tractor's auto-clutch starts to see its hand
  throttle, so vehicle_tractor_i4's hash is re-recorded in that commit.
- Fuel (Sebastian, 2026-09-26/27):
  - An engine without a fuel map still models fuel use, crudely: constant
    indicated efficiency (petrol 0.325, diesel 0.48, Heywood), so fuel flow =
    positive combustion power / (efficiency x heating value).
  - The fuel is referenced by a plain relative file path (moddability).
  - Ship ethanol blends E5, E10, E25 and E85, derived from the E0 gasoline and
    an ethanol file by a cited mixing rule. The existing petrol engines burn
    E5.
  - Each engine lists the fuels it accepts: the 2 l four-cylinders E5 and
    E10, the R26B E5, the tractor diesel. The fuel actually burnt comes from
    the tank.
  - A graph engine never makes different power on different fuels; the fuel
    only changes the bookkeeping. E85's extra power is a calibration effect
    (knock margin) and belongs to the simulated engine only.
  - Exception to "no existing data file edited in I7": the shipped engine
    files gain their fuels list. CSV headers may grow by the fuel channels.
  - Spark vs compression ignition is an ENGINE property (2026-09-27): any
    fuel autoignites if compressed enough; fuels differ only in ignition
    quality (octane/cetane). Fuel files carry no ignition field. An engine
    with a fuel model declares `ignition: "spark" | "compression"`, which
    picks the crude efficiency default. Octane/cetane numbers are added with
    the knock/autoignition model (I7c+), their first consumer.
  - Order: C5 fuel map, C5b assists on the torque request, C5c blend data,
    C5d fuels list plus crude model, then C6 tank, C7 cited data, C8 docs.
  - Fuel mass moves the chassis (2026-09-27, later - not part of C6/I7a):
    the tank's fuel mass will feed back into the chassis physics. C6 keeps
    the chassis mass constant and only tracks m_tank; the follow-up adds
    the coupling. Rulings (2026-09-27): the fuel is a POINT MASS at a tank
    position given in the vehicle file (additive field); the chassis mass
    properties are updated EVERY TICK by offsetting them with the fuel burnt
    (or refuelled) that tick - an O(1) point-mass update (mass, CoM shift,
    parallel-axis inertia term), no recomputation of the whole body. To keep
    rounding from accumulating, the offset is applied as dry chassis + the
    current m_tank point mass (same O(1) cost). Open for the plan: how the
    backend changes a body's mass/inertia and CoM at a tick boundary without
    allocating (Jolt's CoM comes from the shape), the shadow chassis and
    static suspension loads picking it up, hash impact (vehicles with a tank
    only). Sloshing is out of scope unless asked for.

Pedal semantics (Sebastian, 2026-09-25), set per vehicle by its transmission
type:
- Manual: the pedal requests 0-100 % of the engine's gross (indicated) torque
  available at the current rpm, before losses. Friction, pumping and engine
  braking are subtracted in full at every pedal position:
  `T = motoring(rpm) + pedal * (wot(rpm) - motoring(rpm))`.
  `EngineTorqueMap` has implemented exactly this since I3. The adapter adds no
  pedal curve (trigger exponent 1.0). Consequence: 30 % pedal at 3000 rpm
  gives 0.3 * (187 + 45) - 45 = 25 N*m net on car_sedan, about 13 % of WOT.
- Automatic, CVT or DCT (I10): the pedal is a power request. The
  transmission/engine controller chooses the gear (or ratio) and the engine
  torque to deliver that power, the way a real drive-by-wire powertrain
  interprets the pedal.
- Driving modes (at least for automatics, later): each vehicle file lists the
  modes that model offers (e.g. eco, comfort, sport, track), selectable at
  runtime through a control channel. Each mode maps pedal 0..1 to a raw power
  request 0..1 (fraction of peak engine power) with its own monotonic curve.
  Example: pedal 0.5 -> 0.35 in eco, 0.70 in sport. Every curve must satisfy
  f(0) = 0 and f(1) = 1, so flooring the pedal requests peak power in every
  mode; eco therefore rises steeply near full pedal. The loader validates the
  endpoints and monotonicity. A mode can later carry more than the pedal map
  (shift points, damping, steering weight).

Mass distribution (Sebastian, 2026-09-25): car_sedan moves from 50/50 to about
60/40 front/rear, typical for a FWD sedan. This is an interim data change.
Later, a vehicle's total mass, CoM and inertia should be computed from its
parts (engine, gearbox, body, fuel, occupants), as part of the parametric
vehicle editor, instead of being hand-entered.

Driver-assistance requirement (Sebastian, 2026-09-23): optional ABS,
traction control, ESP/ESC, and an open-ended list beyond that (e.g. launch
control, torque vectoring, hill hold). Each vehicle opts in per system via its
data file, and each can be switched on or off at runtime through a control
channel. Controllers are components like the powertrain's: they read only
what a real ECU could measure (wheel speeds, yaw rate, accelerations, steering
angle, pedal inputs) and act only through real actuators (per-wheel brake
pressure, torque requests to the power sources), never by writing simulation
state directly. They run inside the deterministic core at a fixed, stated rate.
This is also where the carried-over braking gap (the real ABS above) and the
M1 launch wheelspin (traction control) get closed.

Driver HUD (Sebastian, 2026-09-24, hand sketch): bottom right, a round
tachometer (rpm x1000 around the arc, redline zone, needle) with the gear in a
circle at its centre and the assist lamps around it (ABS, TC, ESP, LC = launch
control, HILL = hill hold; later systems add lamps the same way), and a
digital km/h readout beside it. A lamp shows three states: not fitted
(hidden), fitted but off, and on/intervening (lit, flashing while it acts).
One generic HUD, scaled per vehicle from its own data (idle, limiter and max
rpm from the engine, the gearbox's gear list), not a hand-made HUD per car.
The car stays centred in the chase view. A mini map is for later games, not
this testbed. Rendering lives in the adapter; the values come from the
engine-neutral PowertrainSnapshot and vehicle data.

Accept: 0–100 km/h and top speed within 3 % of a
point-mass calculation from the torque curve, gearing, drag; clutch lock-up
without torque spikes > 5 % above steady value; open vs. locked diff on split-μ
behaves correctly (wheel speeds, yaw moment sign); engine braking and stall
work; drivetrain energy balance closes within 1 %.

**P5 — Aerodynamics.** Atmosphere, wind, bluff elements, lifting strips, aero
maps, propeller.
Accept: terminal velocity of a bluff body matches analytic within 0.5 %;
**combine test** — combine at 40 km/h in a 60 km/h crosswind: yaw moment about
the chassis COM with the auger folded vs. extended matches the hand-computed
value within 1 %, and it arrives at the chassis through the arm joint; car top
speed drops/rises correctly with CD·A; downforce raises skidpad g by the
predicted amount; flat plate strip matches the coefficient table at all AoA.

**P6 — Aircraft validation.** A Cessna-172-class aircraft from public data.
Accept: trimmed cruise speed, stall speed (flaps up), best glide ratio and
climb rate each within 10 % of the handbook values; phugoid period within
15 % of the theoretical estimate; stalls with a nose drop and recovers; takes
off and lands on the P3 tyres/suspension. Hands-off stable in trimmed flight
for 120 s.

**P7 — Articulated machines.** Hitches, trailer, three-point linkage,
hydraulic cylinders, loader arm, PTO load.
Accept: tractor + trailer tracks correctly in steady circle (off-tracking vs.
geometric formula within 3 %); jack-knife does not explode numerically; loader
lifting a mass shifts axle loads per hand calculation within 2 %; hydraulic
cylinder stalls at the pressure-limited force; PTO load pulls engine rpm down
per the torque curve.

**P8 — Terrain and soil.** Open from the terrain tiles (2026-09-23): a single
tick shows a contact spike of about 159 m/s² when a body crosses a tile SEAM
(measured at x = −512; unrelated to the feature-edge overlap fixed in
`TileManager::suppress_covered_samples`). Heights and normals match across
seams to 1e-9, so this looks like a generic tiled-heightfield contact
artefact; root-cause it here, where terrain gets its own phase.
Accept: plate-sinkage test reproduces the Bekker
curve; drawbar pull vs. slip curve has the right shape and peak against a
published Wong example within 10 %; second pass in the same rut sinks less;
rut depth persists and is reported to the adapter; tractor performance
degrades plausibly from road → field → wet field.

**P9 — Performance pass.** Profile first (per-stage timings and worker
utilisation are telemetry channels from P1 on). Targets on the 5900X:
(a) 30 vehicles (4–8 wheels, full drivetrain, ~20 aero elements each) plus an
aircraft at 240/960 Hz using under 50 % of *one* core — single-thread
efficiency comes first; (b) 300 such vehicles in real time on 10 workers, with
≥ 7× speed-up over 1 worker and no stage where workers idle > 30 % on a
barrier. Apply D5 in order; commit the benchmarks that justify every intrinsic
or asm routine. Check false sharing and the 5900X's two-CCX L3 split when
placing per-worker data.
Accept: both targets met; all earlier validation tests and thread-count
determinism hashes unchanged; asm/intrinsic paths match scalar reference.
Carried over from the terrain tiles (2026-09-23): the tile manager's slot
bookkeeping (a hash map insert/erase when the visible tile set changes) is the
one accepted exception to "no allocation inside jobs"; it is bounded per tick
and the steady state while driving is allocation-free. Replace it with a fixed,
preallocated slot table so the rule holds without exception.
Carried over from P4 I1b (2026-09-24): with Jolt's allocator now hooked into
the debug allocation check, Jolt itself allocates inside `PhysicsSystem::Update`
(`QuadTree::UpdatePrepare`'s `new NodeID[mNumBodies]`, about 49 allocations per
100 ticks with a vehicle). Our own code is checked separately and must stay at
zero; Jolt's internal allocations need a fix here (e.g. an arena behind
`JPH::Allocate`, or broadphase settings that avoid the rebuild).
Carried over from P1.5a (Sebastian, 2026-09-22: accept 3× now, the rest here):
the articulation solver costs 13.5 µs per 960 Hz substep for 10 links (from
40.6 µs; the goal was ≤ 8 µs). Known remaining levers: single-axis rank-1
updates in ABA (U a single column, D a scalar), reusing kinematics across RK4
stages, and the gravity forward-kinematics pass per stage. The substep rate
stays at 960 Hz: at 480 Hz, a four-bar loop's energy drift is 0.85 % in 60 s
(limit 0.5 %).

Every phase P3–P8 additionally repeats the thread-count determinism test with
its new components included.

**P11 — Real-world terrain from open geodata (optional module, later).**
Requested by Sebastian on 2026-09-22: drive around real places. A separate,
optional library target `ps_geo` (CMake option `PS_BUILD_GEO`, default OFF).
The core never depends on it. It plugs in through the same interfaces as the
procedural terrain (see the seams below), so any front end gets it, not only
Godot: the adapter only uploads render buffers the library builds.
- Elevation: Copernicus DEM GLO-30 or SRTM/NASADEM (30 m) worldwide; where
  available, national 1 m lidar terrain models (e.g. the German states' open
  DGM1).
- Ground type: ESA WorldCover (10 m) and OSM landuse/natural/surface tags,
  mapped to our surface table (μ scale, rolling resistance) and later to soil
  parameters (SoilGrids, for P8).
- Structures: OSM roads (width from tags/lanes, surface from `surface=*`,
  draped onto and flattened into the terrain), buildings as extruded
  footprints (height from `height`/`building:levels`, or LoD1/LoD2 city models
  where openly available), bridges, water, forests as colliders/areas.
- Pipeline: an offline importer bakes a bounding box into our own tile format
  (heights, surface ids, road and building geometry, attributions), so runtime
  never parses OSM or GeoTIFF. Heavy geo libraries (GDAL/PROJ/osmium or Python
  equivalents) stay in the importer; which ones gets decided and approved when
  the phase starts.
- Georeference: the scenario names an origin (lat, lon, height) and a world
  shape:
  - *flat* (default): projection (UTM or local transverse Mercator),
    constant gravity along −z. Scale and curvature error at ±50 km are
    quantified and documented. Cheapest, and enough for most driving.
  - *curved* (optional, Sebastian 2026-09-22): the WGS84 ellipsoid. Positions
    are true 3D geometry in a local frame tangent at the origin (ECEF offset,
    double precision), so the ground falls away with distance (about 196 m at
    50 km) and the horizon is real. Gravity at each body and link points along
    the local ellipsoid normal (normal gravity per WGS84/Somigliana, with
    height dependence). "Up", altitude and heading are local quantities
    everywhere: atmosphere (P5) uses height above the ellipsoid, and
    suspension and aero use the local up. Terrain tiles lie on the ellipsoid
    (geodetic tile grid or cube-sphere quadtree, decided in the phase).
    Earth rotation effects (Coriolis, centrifugal) are an optional extra
    term, off by default, and the aircraft phase may use them.
  Accept (curved): gravity direction matches the ellipsoid normal to 1e-9 rad
  at sample points; a ball resting on the ground 50 km away stays at rest (no
  sliding from a mismatched "down"); a straight road drapes over the curvature
  without steps at tile borders; a flight or drive over 50 km shows the
  expected geometric horizon drop; results are deterministic and equal across
  platforms, as in P1.
- Licences: ODbL (OSM) requires attribution and share-alike for derived
  databases; Copernicus and WorldCover require attribution. The baked tiles
  carry their source and licence, and the adapter can display attributions.
Accept (defined in detail when scheduled): heights match the source DEM at
sample points; roads are drivable without steps at road/terrain seams; building
footprints match OSM within 0.1 m after projection; a 50 km drive streams
without hitches as in M1.

**P10 — Deformation (future, not scheduled).** Write a design proposal only
when Sebastian asks: candidate approaches (node-beam, XPBD, corotational FEM,
plastic hinge joints between rigid panels), coupling through the section 6
seams.

## 8. Working rules

- Small, reviewable commits. Commit messages end with the co-author line given
  in the session.
- Every model gets a short header comment naming its source (paper, book
  chapter, equation numbers). Use Pacejka "Tyre and Vehicle Dynamics", Wong
  "Theory of Ground Vehicles", Milliken "Race Car Vehicle Dynamics", Stevens &
  Lewis "Aircraft Control and Simulation", and Project Chrono as references.
  Do not copy code from GPL sources; Chrono is BSD-3, Jolt is MIT.
- Public data only for vehicle/aircraft parameters; note the source in the
  data file.
- When a validation test fails, fix the model, not the tolerance. Changing a
  tolerance needs a written reason in the test and a vault ticket.
- Keep `data/` files and telemetry channel names stable once a phase is done;
  the games will depend on them.
