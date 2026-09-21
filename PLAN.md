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

## 7. Phases and acceptance tests

Each phase ends with: tests green, `CLAUDE.md` map updated, vault tickets
written, one git commit (or a few) with clear messages. No pushing.

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

**P2 — Minimal Godot adapter.** GDExtension exposing: load scenario, step
thread, snapshot → Node3D transforms, input → sim controls, debug draw of
force vectors, static mesh/heightmap → sim collision. Demo project with a flat
test ground.
Accept: P1 scenarios play back in Godot; the sim keeps 240 Hz with Godot at
any frame rate; grep proves core has no Godot include; headless
`godot --headless` smoke test passes.

**P3 — Chassis, suspension, tyres.** One passenger car and one tractor defined
in JSON, with published or plausible `.tir` data (note the source).
Accept: tyre model reproduces the `.tir` reference curves (Fx, Fy, Mz,
combined) within 1 %; static corner weights match hand calculation; steady-
state skidpad lateral g within 3 % of the tyre-limited estimate; 100–0 km/h
braking distance within 3 % of μ-based estimate incl. load transfer; car
parked on a 15 % slope does not creep (< 1 mm over 30 s); step-steer response
is stable, no oscillation growth.

**P4 — Drivetrain.** Accept: 0–100 km/h and top speed within 3 % of a
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

**P8 — Terrain and soil.** Accept: plate-sinkage test reproduces the Bekker
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

Every phase P3–P8 additionally repeats the thread-count determinism test with
its new components included.

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
