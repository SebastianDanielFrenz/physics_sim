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
      backend/                        i_rigid_backend.h (IRigidBackend, BodyDesc/JointDesc, ShapeContactHit, ContactEvent, BodyIntegrationParams (PHYS-033 I1), create_jolt_backend()); shape_desc.h (ShapeDesc/BoxShape/SphereShape/CapsuleShape/MeshShape/HeightfieldShape - split out of i_rigid_backend.h so articulation_desc.h can use ShapeDesc without pulling in the backend's JointType)
      articulation/                   articulation_desc.h (ArticulationDesc incl. contact_strategy, RootDesc, LinkDesc incl. contact, JointType, JointLimits, VelocityActuator, LoopClosureDesc/LoopClosureType, ContactStrategy, ContactMaterial, LinkContactDesc) - descriptors only, no algorithm code
      terrain/                        terrain_source.h (ITerrainSource, TileKey, TileSample), terrain_config.h (TerrainConfig - pool sizing only, NOT grid resolution, see "Terrain tile system" below), procedural_terrain.h (load_procedural_terrain_source() factory)
      vehicle/                        public, additive descriptors/seams only (PLAN.md 5.1/P3, no algorithm code - see "Vehicle system" below): vehicle_desc.h (VehicleDesc/WheelDesc/SuspensionDesc/SteeringDesc - PHYS-033 I2 commit 5a: BrakeDesc/DriveDesc/WheelDesc::driven are gone, replaced by VehicleDesc::brake_system/powertrain below), curve1d.h (Curve1D - split out of vehicle_desc.h in I2 commit 5a so drivetrain/powertrain_desc.h can use it without a vehicle_desc.h <-> powertrain_desc.h include cycle), tyre_desc.h (TyreDesc, Pacejka MF5.2/6.1-subset coefficients, low_speed_damping_ratio/low_speed_damping_vx - PHYS-033 I1, rolling_resistance_v_eps - parking-slope creep fix D1), tyre_model.h (ITyreModel, TyreSlipInput/TyreForces, TyreForcesWithSlope, evaluate_with_slope(), create_pacejka_tyre_model(), SlipRelaxationState/step_slip_relaxation()/relaxation_step_axis() - PHYS-033 I2), wheel_telemetry.h (WheelTelemetry, returned by World::wheel_telemetry()), wheel_state.h (WheelState, returned by World::wheel_state() and carried in WorldSnapshot::vehicles - PLAN.md M1)
      drivetrain/                     public, additive descriptors only (PHYS-033 P4 I2 plan section 2, no algorithm code - see "Vehicle system" below): powertrain_desc.h (PortRef/ShaftDesc/EngineLimiterDesc/TorqueMapEngineDesc/GearPairDesc/DifferentialDesc/ComponentDesc/PowertrainDesc), brake_system_desc.h (BrakeCornerDesc/HandbrakeDesc/BrakeSystemDesc), energy_ledger.h (PowertrainEnergy)
      io/                             telemetry.h (TelemetryRegistry, CSV via IO thread), scenario.h (Scenario incl. ScenarioArticulation/ScenarioTerrain/ScenarioVehicle/ScenarioControlEvent, load_scenario, build_scenario), surface_table.h (SurfaceTable), test_ground_loader.h (load_test_ground_collision()), vehicle_io.h (load_tyre_json(), load_vehicle_json() - physics_sim.vehicle/2 as of PHYS-033 I2 commit 5a, see "Vehicle system" below)
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
      drivetrain/                     INTERNAL (ps::drivetrain, not a public header); PHYS-033 P4 I2 shaft-network powertrain, see "Vehicle system" below and the "Drivetrain" section: dense.h (SmallVec/SmallMat, Cholesky), box_lcp.h/.cpp (F1 box-LCP: block principal pivoting + Murty fallback), nullspace.h (build_nullspace), network.h/.cpp (NetworkBuilder, Assembly, ShaftNetwork), components.h/.cpp (IComponent, TorqueMapEngine, GearPair, OpenDifferential, DirectTorqueBrakes - I2 commit 5b), powertrain.h/.cpp (Powertrain - builds the node/rigid-row/friction-row graph from a vehicle's PowertrainDesc/BrakeSystemDesc/WheelDesc list, drives the per-substep pre_solve/solve/post_solve sequence, owns the PowertrainEnergy ledger - I2 commit 5b)
      aero/ machine/                  empty (.gitkeep) until P4+
  tests/
    support/                          test-only shared helpers (not part of ps_core)
      counter_force_element.h         deterministic counter-hash IForceElement used by determinism/bench tests
      catch_main.cpp                  custom Catch2 entry point installing headless CRT handlers; target ps_test_catch_main
      vehicle_test_helpers.h          PLAN.md P3: VehicleTestRig/build_vehicle_rig()/settle() - a World with a static ground box (flat or tilted) plus one vehicle built directly from a data/vehicles/*.json file; see "Vehicle system" below for the chassis-half-extents-vs-wheel-shape-cast constraint documented on build_vehicle_rig(). build_vehicle_rig_from_desc() (PHYS-033 I1) is the VehicleDesc-taking core build_vehicle_rig() wraps - takes an already-loaded (test-mutable) desc plus a tilt_axis (unit_y() default = longitudinal slope, unit_x() = cross slope) and tick_hz/substep_hz (for a fine-tick reference run). Both (and build_vehicle_rig()) also take a trailing ground_half_extent (PHYS-033 P4 I3 plan section 13, default 1000 matching every pre-I3 caller) - the accel/top-speed validation tests drive far enough (2000/4000 m) that the old hardcoded 1000 m half-extent would run the car off the edge. Also build_vehicle_rig_on_surface()/flat_surface_mesh() - same, but a flat MeshShape ground tagged with a named data/surfaces/surfaces.json SurfaceId instead of an untagged BoxShape, for tests comparing behaviour across surfaces (a plain BoxShape has no Jolt material and always resolves to kInvalidSurfaceId); build_vehicle_rig_on_surface() also gained trailing optional tick_hz/substep_hz params (PHYS-033 I1b, default 240/960 matching every existing call site) for a fine-tick reference run on a non-asphalt surface
      powertrain_reference.h          PHYS-033 P4 I3 plan section 13: RefCar, an INDEPENDENT reference model built field-for-field from the public VehicleDesc (never hand-duplicated JSON constants) - engine curves, clutch, gearbox ratios/efficiencies/inertias, diff, driven/undriven hub inertias, the Pacejka tyre model, aero. ref_gear_loss_params() derives the locked-gear reflected-inertia/torque-gain chain (K_half, J_refl) from the engine/clutch/gearbox/diff node equations (plan section 13's own staged T_m->T_out->T_fd->per-hub derivation); ref_locked_gear_step() is one semi-implicit-Euler substep of the resulting ODE (test 5's "Engaged(k), Locked" case - test_powertrain_engine_braking.cpp). Deliberately simplified (no suspension/pitch dynamics, one-substep mesh-loss lag, event quantisation, lateral dynamics - plan's own "expected combined error <= 1%"), not a byte-exact replica of core/src/drivetrain's own implicit solver
    unit/                             target ps_unit_tests: test_hello, test_math, test_jobs (incl. the queue-full inline-fallback cases), test_world_basic, test_world_snapshot (ArticulationSnapshot/contacts/vehicles additions), test_articulation_core (white-box, core/src on its include path), test_articulation_four_bar (white-box), test_articulation_parallel_alloc (white-box: System::step_all is one job per articulation, debug_alloc-armed), test_aba_fast_transform (white-box: fast vs. reference ABA congruence transform, 1e-12 relative), test_scenario_articulation, test_articulation_contact (Compliant/ProxyBody smoke tests on a single-DOF rail rig, incl. a ProxyBody-at-high-speed [!mayfail] case), test_mesh_heightfield_shapes (MeshShape/HeightfieldShape collision + surface ids), test_terrain_golden (ProceduralTerrainSource vs. terrain.py's golden points), test_terrain_tile_manager (white-box: seams, streaming, throttling, zero-alloc steady state), test_test_ground_loader (load_test_ground_collision + SurfaceTable), test_terrain_static_overlap (PHYS-021: a parked vehicle's wheels and a sliding probe crossing the test_ground straight's edge, plus a zero-gap sweep of both skidpad ring edges - the terrain/static-feature coincident-collision fix, see "Terrain tile system"), test_terrain_suppression_precompute (white-box, real test_ground data: load-time suppression precompute is bit-identical to the lazy path for every tile, worker-count invariant, and hit at runtime), test_pacejka_tyre_model (PLAN.md P3: MF5.2-subset pure/combined-slip curves, relaxation ODE, rolling resistance, cross-checked against an independent re-derivation and exact combined-slip-degeneracy identities; plus every shipped tyre's longitudinal slide-to-peak shape - Fx(kappa=1)/peak, Fx(kappa=3)/peak and peak slip at fz0 and 0.75*fz0), test_vehicle_ackermann_symmetry (front wheel Ackermann steer angles are exact left/right mirror images at +-full lock, and each wheel turns toward the commanded direction - see "Vehicle system" below and vault PHYS-022), test_vehicle_parallel_alloc (PLAN.md P4 I0: a full World::step with a vehicle, debug_alloc-armed - PHYS-033 I1b commit C3 added a T3 negative-control case proving the Jolt-allocator hook is wired, and [!mayfail]-tagged the existing zero-alloc case once that hook revealed a pre-existing Jolt-internal allocation, see "Job system"), test_vehicle_io_validation (load_vehicle_json/load_tyre_json error paths: ackermann_fraction range, "format" field incl. /1 rejection, legacy driven/drive/brakes keys, differential efficiency != 1 - PHYS-033 I2 commit 5a), test_body_integration_params (PHYS-033 I1: IRigidBackend::get_integration_params vs. the box-inertia formula, damping/gravity-flag round-trip), test_wheel_contact_frame (PHYS-033 I1b, white-box, core/src on its include path: ContactFrame's normal/long_dir/lat_dir and the true ground contact patch, incl. bottomed/rim-contact/side-wall-fallback cases - see "Vehicle system"), test_sub_threshold_rotation (parking-slope creep fix D2: a free box with a small steady angular velocity does not rotate by default, does when opted in via IRigidBackend::set_integrate_sub_threshold_rotation - see "Jolt backend"), test_powertrain_engine (PHYS-033 P4 I3, white-box: EngineTorqueMap's torque blend against hand values and a central-finite-difference slope check, the Off/Stalled/Cranking/Running state machine's every transition, the idle PI's small/large-dip response, FuelCut limiter hysteresis at WOT in neutral, cranking equilibrium against the starter/Coulomb/smooth-friction balance), test_powertrain_gearbox (white-box: Gearbox's position<->state-index mapping, a full N->1st shift cycle through Selecting/Synchronising, dog release refused while loaded then releasing once torque drops, synchro time matching J_in*|domega|/capacity within 2 substeps, engaging reverse while rolling grinding then aborting to N, Sequential walking one position at a time toward a far target; plus Clutch: engagement = min(pedal curve, auto), capacities scale from it), test_powertrain_tcu (white-box: ManualTcu's counter differences/first-tick baseline/reset/clamped-pending, gear_request precedence over the counter path and the "none" sentinel, the auto-clutch launch law's e(rpm) and Launch->Locked->Launch cycle, the anti-stall panic release rate vs. the ordinary release_rate_per_s, blip/ignition-cut flags following shift phase, launching from Neutral below vs. above the speed threshold, auto-shift thresholds and hysteresis), test_powertrain_network (white-box: dense.h's Cholesky factor/solve against reference Gauss-Jordan and its PSD regularisation, build_nullspace against the plan's own car/tractor worked examples, two inertias locking through a friction row momentum-conservingly, a gear chain's power reproducing ratio*efficiency*torque with lagged mesh loss, an open differential splitting torque and holding a locked hub, a topology switch conserving momentum and booking the exact engagement loss, a friction row's ratio generalising its coupling (synchro row shape), clutch lock-up under a ramping capacity showing no spike - plan section 13.3(a)'s literal criterion on a test-only network, test_powertrain_clutch_lockup.cpp below is the whole-vehicle companion), test_powertrain_friction_lcp (white-box: box_lcp's block-principal-pivoting stage against brute force over batches 1/3/4/physical and a [long] 10^5-case sweep, the k=1 closed form and c=0 caller-level filtering)
    validation/                       target ps_validation_tests: test_free_fall, test_pendulum, test_joint_chain_energy, test_box_on_slope, test_tunnelling, test_resting_contact, test_huge_forces, test_mass_ratio (one case [!mayfail]), test_articulation_pendulum, test_articulation_double_pendulum, test_articulation_energy, test_articulation_mass_ratio, test_articulation_floating_base, test_articulation_actuator, test_articulation_four_bar_energy (white-box, core/src on its include path), test_articulation_contact_measurements (Compliant/ProxyBody: a fixed-base arm resting on the ground, the same rig pressed with a known force, a rail rig pushing a dynamic crate, a floating-base vehicle drop, 5/20 m/s impacts; two [!mayfail] ProxyBody cases), test_vehicle_static (corner weights vs. hand calculation), test_vehicle_parking_slope (15% slope, <1mm creep in 30s - [!mayfail] history through PHYS-033 I1b was chasing the wrong cause ("elastic hold" weakness); the real mechanism was two numerical defects, a standstill rolling-resistance sign(vx) chatter (TyreDesc::rolling_resistance_v_eps) plus Jolt's own dropped sub-threshold rotation (IRigidBackend::set_integrate_sub_threshold_rotation) - both fixed, [!mayfail] removed, measured drift 2.5e-6 m; a [.][diag] re-check case still runs the 240Hz/960Hz comparison), test_vehicle_step_steer (yaw-rate response does not grow after a step-steer input), test_vehicle_skidpad (PHYS-036 rewrite: speed is held by a tangential CoM force element with throttle fixed at 0 - free rolling, not a traction limit through the FWD drivetrain - walked up through a sequence of held-speed trials (a slow ramp across trials) gated on |mean(dv/dt)| < 0.02 m/s^2, curvature error < 0.5% and no steer/thrust saturation; compared against TyreLimitedSteadyStateSolver, an in-test quasi-static steady-circle solve using the vehicle's own real VehicleDesc/TyreDesc data and the real Pacejka tyre model (replacing the old per-axle peak-sum estimate, whose /2 halving bug - vault PHYS-018 - and missing Mz term made it unusable once Mz's sign was corrected, see PHYS-036) - PASSES, measured 8.8538 vs. estimate 8.7871 m/s^2 (0.76%), [!mayfail] removed; does not assert which axle limits), test_vehicle_braking ([!mayfail]: 100-0 km/h distance vs. a load-transfer-corrected mu estimate - a per-axle ABS emulation fix (was front-only) closed the gap from 16% to 9%, still over the 3% tolerance; the residual is root-caused to closed-loop slip regulation against a locally unstable target under the wheel spin model's hard velocity clamp, not a harness oversight - see the test's own header comment and vault PHYS-019), test_vehicle_accel_standing_start (full-throttle 0-50 km/h average acceleration from an ALREADY-SETTLED rest vs. a point-mass estimate from car_sedan.json's own torque curve, within 15%, driven-wheel slip bounded (no runaway wheelspin); "acceleration on dirt/grass lower than on asphalt" is [!mayfail] as of the parking-slope creep fix (D1/D2) - CHAOTIC at HEAD independent of that fix (both front wheels pegged at the kappa=+-3 clamp with no clutch/traction-control model), its previous pass was luck, see the test's own header comment for the measured variant table; a third case documents that full throttle commanded before the suspension has settled DOES drive the tyre past its peak (Fx/Fz~0.6 vs ~1.0) but recovers within the test's own window and never stalls the car - see the file's own header comment, vehicle_desc.h's DriveDesc comment, and out/handover_accel.md for the final_drive_ratio root cause and this no-clutch-model caveat - final_drive_ratio lowered 10.0 -> 8.5 by PHYS-033 I1b commit C8, an interim fix for the SAME slip bound after the contact-patch lever fix unloaded the front axle enough to push max_abs_slip to 1.39; car_sedan.json's own "source" field has the arithmetic), test_vehicle_steer_mirror (full-throttle-from-settled-rest steering response is left/right mirror-symmetric: yaw rate opposite sign but equal magnitude, forward speed gained equally in both directions - see "Vehicle system" below and vault PHYS-022), test_vehicle_standstill_wiggle (a car arriving at near-standstill with residual yaw rate and/or steering settles instead of oscillating - the live-demo skidpad "wiggle" bug; two driven-then-braked cases plus a clean pure-yaw-rate-kick isolation, all asserting decaying not growing/holding amplitude, a worker-count state_hash check for the low-speed code path, a [.][diag] "zeta sweep diagnostic" (PHYS-033 I1 commit E), and PHYS-033 spec test 10 - a yaw kick with zero low-speed damping is neutral and discretization-converged (240 vs. 960 Hz), the same with rolling resistance on, and a [!mayfail] check of the default-zeta decay rate against the section 4.2 modal estimate (measured off by ~14x, a known limitation of that single-mode estimate, not a code bug) - see "Vehicle system" below), test_vehicle_shadow_chassis (PHYS-033 I1 W1: shadow_dv_error/dw_error stay within 1e-4 - measured ~1e-6/1e-7 - for a settled car under step steer, a constant force element and an alternating one matching the predicted one-tick-lag estimate; a chassis forced into contact shows dv_error > 1e-2; a per-vehicle-file low-speed-damper stability bound gamma < 1, split into separate car_sedan/tractor cases by PHYS-033 I1b commit C7 (the patch-lever gamma: car_sedan 0.950, tractor 0.652, both still under 1)), test_vehicle_tractor_cross_slope (PHYS-033 I1 spec test 11: a tractor holds a 10% cross slope / 15% longitudinal slope on the handbrake alone - [!mayfail] history through PHYS-033 I1b (same wrong "elastic hold" diagnosis as test_vehicle_parking_slope.cpp) removed now the real cause (same two defects) is fixed - measured drift 3.9e-6 m / 1.3e-4 m; a [.][diag] A/B harness against the H0-H6 cross-slope hypotheses, confirming H0 (data/vehicles/tractor.json's inconsistent rear attachment z) as the root cause of the ORIGINAL owner-reported bug), test_vehicle_contact_geometry (PHYS-033 I1b: whole-vehicle pins for the contact-patch/force-decomposition fixes - a pitched-at-rest chassis does not self-propel, steady-cornering/straight-braking load-transfer ratios match statics within the plan's own bands - plus a [.][diag] "contact geometry diagnostics" case (skidpad steady-state check, braking/accel traction diagnostics)), test_powertrain_engine_braking (PHYS-033 P4 I3 plan section 13.4/13.5, tests 5-6: engine braking in 4th from 100 km/h vs. tests/support/powertrain_reference.h's RefCar locked-gear ODE - [!mayfail], measured 5.63% off the 3% gate, two attempts made this session (baseline, then coupling rolling resistance into the driven hub's own torque balance - measured WORSE at 14.1%, reverted), root cause undiagnosed, see the test's own header comment; plus "stall under load" (handbraked uphill clutch dump stalls the engine), "gentle idle-only launch" (creeps away without stalling), "auto-clutch anti-stall" (never stalls braking to a stop) - all three PASS), test_powertrain_clutch_lockup (plan section 13.3(b), the whole-vehicle companion to test_powertrain_network.cpp's test-only-network case: manual rolling-start clutch lock-up PASSES; auto-clutch launch from rest is [!mayfail] - halfshaft torque continues climbing ~12% above the pre-lock level for up to 0.5s after lock, specific to an abrupt full-throttle auto-clutch launch, not a general driveline bug), test_powertrain_energy_balance (plan section 13.6, test 7: the PowertrainEnergy ledger's own balance_error identity stays within a measured-with-margin 5e-8 relative per tick (max observed 1.485e-8, a brief relaxation-lag transient during hard braking with wheel slip - the same class of effect as tyre_loss's "steady sign" caveat) and within the literal <= 1% of (engine_out+starter_in) end-of-run acceptance bound - PASSES), test_powertrain_accel_topspeed (plan section 13.1/13.2: only test 2, top speed within 3% of a steady-state-per-gear bisection reference, is implemented - PASSES with a wide margin (car_sedan.json: gear 5, v_ref=56.79 m/s / 204.4 km/h, sim 0.24% off); test 1 (0-100 km/h) is explicitly deferred, not forgotten - needs an event-driven Neutral/Engaged(k) x {Slipping,Locked} clutch state machine plus an independent re-derivation of the gearbox's own shift-phase timing, substantially more complex than test 5's locked-gear-only reference which itself needed two real attempts - see the file's own header comment), test_powertrain_auto_clutch_shifts (owner test drive 2026-09-25, auto-clutch around upshifts on car_sedan.json: selecting 2nd at walking pace (5 km/h) never stalls and pulls away on throttle - before the fix the rpm-blind Shift-mode reclose dragged the engine onto a ~350 rpm input shaft and stalled it; a part-throttle 1->2 upshift at 15 km/h locks within 1 s without wheelspin)
    determinism/                      target ps_determinism_tests: test_worker_count_hash (1/2/4/10/24 workers + 1000-run randomised-scheduling stress test; one case adds several articulations to the same scenario, one case mixes Compliant/ProxyBody contact-bearing rail rigs with plain chains, one case adds two vehicles built directly from a VehicleDesc with a mid-run step-steer input - migrated to the full PHYS-033 P4 I3 engine/clutch/gearbox/diff chain by I3 plan section 11, one case loads data/scenarios/terrain_multi_interest.json from PS_SOURCE_DIR - two interest points plus suppression, one case (I3 plan section 13.9) hands the same two-vehicle fixture off to assist.auto_shift=1 at full throttle - the fixture's gearbox has no synchro capacity (dog only), so the resulting auto-upshift genuinely grinds (measured grind_events=1, asserted as a scenario-sanity precondition since that field itself is sound-only/not hashed) - hashes still identical across every worker count)
    bench/                            target bench_force_elements (plain main, also CTest-registered): 2000-force-element worker scaling; target bench_contact_coupling (plain main, also CTest-registered): cost per tick per contact strategy, scaling with contact count; target bench_terrain_tiles (plain main, also CTest-registered): tile-streaming cost/tick against the real test_ground terrain spec, simulated 50 km drive; target ps_energy_study (plain main, NOT CTest-registered): Part A solver-settings energy-loss sweep plus the P1.5a Jolt-vs-articulation table and 10-link cost benchmark, energy_study.cpp
  data/
    vehicles/                         JSON ps::vehicle::VehicleDesc files (ps::io::load_vehicle_json - see "Vehicle system" below): car_sedan.json, tractor.json - geometry derived from data/models/*/*.rig.json's wheel pivots, plausible/published parameters with source noted per PLAN.md section 8. Both are physics_sim.vehicle/2 (PHYS-033 I2 commit 5a): each references an engines/*.json and a gearboxes/*.json file via its powertrain.components' "ref" fields (see "Vehicle system" below)
    tyres/                            JSON ps::vehicle::TyreDesc files (ps::io::load_tyre_json) referenced by data/vehicles/*.json: passenger_car_225_45r17.json, tractor_front.json, tractor_rear.json
    engines/                          JSON ps::drivetrain::TorqueMapEngineDesc files (PHYS-033 P4 I3 commit 6, ps::io::load_vehicle_json's component loader): i4_2l_na.json (car_sedan.json), diesel_i4_4l5_turbo.json (tractor.json) - WOT/motoring torque curves, idle/limiter/starter parameters
    gearboxes/                        JSON ps::drivetrain::GearboxDesc files (same loader): sedan_6mt.json (car_sedan.json, 6-speed manual), tractor_12f4r.json (tractor.json, 12F/4R) - forward/reverse ratios+efficiencies, synchro capacities, neutral_phase_s/grind_after_s/abort_after_s
    aircraft/                         empty (.gitkeep) until P6+
    surfaces/surfaces.json            ps::io::SurfaceTable input: name -> {lambda_mu, crr} (shared, deterministic, alphabetically-assigned SurfaceId space)
    scenarios/                        JSON scenario files (ps::io::Scenario schema): free_fall.json, pendulum.json, joint_chain.json, articulation_chain.json, multi_articulation.json (several open chains of different sizes, a floating base, a four-bar loop closure - Windows/Linux hash-compare and worker-count determinism), contact_articulation.json (one Compliant and one ProxyBody rail rig - same hash-compare/determinism coverage for P1.5b contact coupling), terrain_tiles.json + terrain_spec_small.json (a small synthetic ITerrainSource + tile-manager scenario, distinct from data/tracks/test_ground - same hash-compare/worker-count-determinism coverage for the M1 tile system), terrain_multi_interest.json (same small spec with TWO interest points, a static pad triggering suppression and dynamic balls on the streamed terrain - guards the cross-platform tile-slot assignment order, vault PHYS-026), vehicle_step_steer.json (car_sedan.json on a flat static ground box, a step-steer control_event - ps_run-drivable vehicle scenario per PLAN.md P3 scope item 7; same hash-compare/worker-count-determinism coverage for the vehicle system, see tools/ci.ps1's `$script:HashCheckScenarios`), vehicle_powertrain.json (PHYS-033 P4 I3 plan section 13.9: car_sedan.json on a flat static ground box, control_events driving shift_up_count/throttle/brake/shift_down_count/a starter pulse over 10s - same hash-compare/worker-count-determinism coverage for the I3 drivetrain chain; state_hash=0xa25f3a50fa10c5f8)
    models/                           Godot art assets (glb + rig.json); owned by a separate session, do not touch
    tracks/test_ground/               owned by a separate session (trackgen), READ-ONLY from ps_core: terrain.json/terrain_golden.json (ProceduralTerrainSource's golden reference), test_ground_collision.json/.bin (ps::io::load_test_ground_collision's input - see "Test-ground static collision" below for the .bin's byte-offset layout)
  adapters/
    godot/                            GDExtension adapter + demo project (P2/M1); built by its own adapters/godot/smoke_test.ps1 (own build dir out/build/godot-adapter) or with PS_BUILD_GODOT_ADAPTER=ON. src/fall_detector.h: Godot-free fall diagnostic called from PsSimulation's sim-thread post_step - prints ONE "ps_godot FALL DETECTED ... END FALL REPORT" block per event when a vehicle's chassis centre drops more than 2.0 m below the terrain source's own height at its XY (tick, sim time, chassis pose/velocity, tile key under the car, resident/starved/precomputed tile counts, static collision under the chassis, per-wheel contact/surface); smoke_test.ps1 fails if it fires during the --m1-autopilot drive
      godot_common.ps1                shared helper (dot-sourced by smoke_test.ps1 and the root drive.ps1, never run directly): Enter-VsDevShell, Find-GodotConsoleExe/Find-GodotExe, Invoke-AdapterCMakeConfigure/Build (return cmake's exit code; their own stdout is piped through Out-Host, not left in the function's return value - concatenating the two silently broke `-ne 0` exit-code checks once, see the file's own comment), Test-AdapterConfigured, Test-DllLocked (drive.ps1's locked-DLL-before-build check)
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
value, never hashed) - all registered up front in `create_vehicle` itself (PLAN.md P3 scope item
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
`vehicle_step_steer.json` still hashes to `0x1c2b775be0dc3339` at 1 and 10
workers, matching the value recorded before this field existed).

`World` additions (additive, `ps/world/world.h`): `create_vehicle`,
`vehicle_wheel_count`, `wheel_telemetry`, `wheel_name`, `wheel_state`,
`vehicle_shadow_dv_error`, `vehicle_shadow_dw_error` (PHYS-033 I1 W1, see
above), `wheel_omega`, `powertrain_energy` (PHYS-033 P4 I2 commit 5b, see
"Drivetrain" below).
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
  solver itself does not track.
- `components.h`/`.cpp` (`IComponent` and four concrete types, commit 5b) -
  `IComponent`: `pre_solve(ctx, network, assembly)` (write this substep's
  `tau0`/`d` terms), `post_solve(ctx, network, ledger)` (read back the
  solved omegas, accumulate this component's energy-ledger contribution),
  `fold_state_hash` (default no-op; only a component with its own
  persistent state beyond the network's node omegas overrides it).
  - `TorqueMapEngine` - `wot_torque_nm_vs_rpm` curve times `ctx.throttle`,
    clamped to `[idle_rpm, limiter.rpm]` and zeroed above the limiter;
    `d` (the linearised damping term the implicit solve needs) comes from
    the curve's own local slope, not a separate tuning constant.
  - `GearPair` - a fixed-ratio mesh, realised as one of `network.h`'s rigid
    rows; carries `lambda_mesh_prev_` (the previous substep's reconstructed
    mesh-loss multiplier, folded into the state hash - the one `IComponent`
    that overrides `fold_state_hash`) so a lossy mesh's dissipation is a
    lagged reconstruction from momentum balance rather than an extra
    implicit unknown.
  - `OpenDifferential` - stateless, no-op `pre_solve`/`post_solve`: an open
    differential's Willis constraint is itself one of `NetworkBuilder`'s
    rigid rows (built once, at graph-build time), so there is nothing left
    for the component to do at solve time.
  - `DirectTorqueBrakes` - one friction row per corner (`Row{row,
    corner_max_nm, handbrake_max_nm}`, service and handbrake capacities
    merged per hub at graph-build time) - the exact-Coulomb replacement for
    P3's hard brake-torque clamp; commit 5b's stop condition 5 tracks how
    often a row's capacity is actually hit (`friction_cap_hits`) versus the
    LCP falling back past stage 1 (`nonspd_hits`/`lcp_stage3_hits`, folded
    up from `box_lcp`'s three-stage fallback above).
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
  work, never repeated per tick/substep. Per substep,
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

`drivetrain::PowertrainEnergy` (`ps/drivetrain/energy_ledger.h`): the
mechanical-energy ledger a `Powertrain` accumulates every substep and
exposes per tick via `World::powertrain_energy` -
`engine_out`/`wheel_out`/`tyre_loss`/`brake_loss`/`gear_loss` (component
contributions), `integrator_loss` (the linearly-implicit Euler step's own
`1/2 * delta_omega^T * J * delta_omega` dissipation - not a physical loss
channel, an artifact of the integrator itself), `rotor_ke` (instantaneous
stored rotational kinetic energy, not cumulative), `balance_error` (see
`end_tick()` above). Surfaced as CSV telemetry per vehicle,
`vehicle<i>.energy.*` (eight channels) and `vehicle<i>.powertrain.*`
(`friction_cap_hits`/`nonspd_hits`/`lcp_stage3_hits`, three channels) -
see "World tick pipeline" above for exactly when these are registered/
recorded.

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
`OpenDifferential::lambda_mesh`/`left_node`/`right_node`/`hub_coef`) -
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
