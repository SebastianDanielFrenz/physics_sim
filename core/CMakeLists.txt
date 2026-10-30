# ps_detmath — the deterministic double-precision transcendental math port
# (ps/math/transcendental.h + its internal detmath/ implementation, PLAN.md
# D7, vault PHYS-009), split out of ps_core into its own leaf STATIC target
# for the SAME reason ps_engine_cycle is its own target below: found while
# verifying ps_engine_cycle_standalone (the coordinator review's link-time
# guard) - ps_engine_cycle's own "live" files (geometry.cpp/cycle_stepper.cpp/
# cycle_point.cpp) call ps::math::sincos/asin/exp/log/pow/cos (required by
# PLAN.md D7 - core/ code must never call the platform libm transcendentals
# directly), but that public API's IMPLEMENTATION used to be compiled only
# into ps_core (src/math/transcendental.cpp + detmath/*.cpp) - so even with
# map_generator.cpp/map_cache.cpp correctly moved out (below),
# ps_engine_cycle_standalone still failed to LINK, with unresolved
# ps::math::* symbols, whenever it actually ran a cycle point. Has zero
# dependencies of its own beyond <bit>/<cfloat>/<cstdint>/<cmath> (std::sqrt
# only, PLAN.md D7's own carve-out) - confirmed by reading every detmath/*.cpp
# #include line - so splitting it out risks nothing further. Both ps_core and
# ps_engine_cycle link it PRIVATE (single compiled copy, no duplicated
# symbols); math_check.cpp (a "compile every math/ header once" static_assert
# smoke file, not part of the deterministic-transcendental implementation
# itself) stays in ps_core, where it already exercises this header tree via
# ps_core's own public include path.
add_library(ps_detmath STATIC
  src/math/transcendental.cpp
  src/math/detmath/common.h
  src/math/detmath/rem_pio2.cpp
  src/math/detmath/trig_kernel.cpp
  src/math/detmath/trig.cpp
  src/math/detmath/inverse_trig.cpp
  src/math/detmath/exp_data.cpp
  src/math/detmath/exp.cpp
  src/math/detmath/log_data.cpp
  src/math/detmath/log.cpp
  src/math/detmath/pow_data.cpp
  src/math/detmath/pow.cpp
)

target_include_directories(ps_detmath PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(ps_detmath
  PRIVATE
    ps_warnings
    ps_options
)

# ps_engine_cycle — the 0D cycle model's "live" pieces only (the not-yet-
# public "stepper" a future ps_audio, out/audio_plan.md D-11, will also link
# for live engine sound) split out of ps_core into its own STATIC target
# (ps_audio A1). MUST NOT link ps_core and must never pull in anything that
# needs ps_core's own implementation (e.g. ps::jobs::JobSystem, whose .cpp
# lives only in ps_core) - a coordinator review of the first cut of this
# split found exactly that cycle (map_generator.cpp needs JobSystem, so
# ps_core -> ps_engine_cycle -> ps_core, which only "worked" because MSVC
# link/lld resolve archives in any order; GNU ld would not). The generator/
# disk-cache infrastructure (map_generator.cpp, map_cache.cpp - both need
# JobSystem and/or file IO) therefore stay OUT of this target and live in
# ps_core's own source list below instead; this target keeps only the
# geometry/cycle-stepper/cycle-point/firing-order files the live model
# itself needs. ps_core links this target PRIVATE (below); ps_audio will
# link it too, without ever linking ps_core - see
# tests/unit/engine_cycle_standalone_main.cpp (target
# ps_engine_cycle_standalone, just below ps_core's own target block) for the
# link-time guard proving this. Its public headers live under
# include/ps/engine_cycle/ - the SAME include/ tree ps_core's own
# target_include_directories(PUBLIC) already exposes, so nothing extra is
# needed for a ps_core consumer (ps_unit_tests, ps_run, ...) to see them.
# It links no third-party library (no Jolt, no nlohmann_json - JSON parsing
# stays in ps_core's io/) and must never #include a ps_core-INTERNAL header
# (core/src/<anything but engine_cycle>/*.h) - see ps/drivetrain/isa_air.h
# for the one dependency that had to be untangled to keep this true. It DOES
# link ps_detmath (above) - a leaf target with no dependencies of its own,
# not "ps_core's own implementation" in the sense this file's dependency
# rule is about.
find_package(Threads REQUIRED)

add_library(ps_engine_cycle STATIC
  src/engine_cycle/geometry.cpp
  src/engine_cycle/cycle_stepper.cpp
  src/engine_cycle/cycle_point.cpp
  src/engine_cycle/firing_order.cpp
  src/engine_cycle/pipe_network.cpp
  src/engine_cycle/waveguide_network.cpp
  src/engine_cycle/cycle_engine.cpp
  src/engine_cycle/step_team.cpp
  src/engine_cycle/knock.cpp
)

target_include_directories(ps_engine_cycle PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(ps_engine_cycle
  PRIVATE
    ps_detmath
    Threads::Threads
    ps_warnings
    ps_options
)

# ps_environment (weather plan W1, out/weather_plan.md K1): the engine-neutral
# environment library - clock, calendar, sun (later weather, surface
# condition, tyre-profile response). Like ps_engine_cycle it MUST NOT link
# ps_core (a game server links it without Jolt or the vehicle code); it links
# only ps_detmath (deterministic ps::math, PLAN.md D7), ps_warnings and
# ps_options. Public headers under include/ps/environment/ (the same include/
# tree ps_core exposes). Guard: ps_environment_standalone
# (tests/unit/CMakeLists.txt, plain main, linked WHOLE_ARCHIVE without ps_core).
add_library(ps_environment STATIC
  src/environment/climate_desc.cpp
  src/environment/solar_geometry.cpp
  src/environment/calendar.cpp
  src/environment/sun.cpp
  src/environment/environment.cpp
  src/environment/weather_desc.cpp
  src/environment/air_temperature.cpp
  src/environment/weather.cpp
  src/environment/weather_classes.cpp
  src/environment/atmosphere.cpp
  src/environment/wind.cpp
  src/environment/surface_condition.cpp
  src/environment/condition_grip.cpp
  src/environment/tyre_profile.cpp
)

target_include_directories(ps_environment PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(ps_environment
  PRIVATE
    ps_detmath
    ps_warnings
    ps_options
)

# ps_environment_io (W1): the JSON loaders (climate now; weather presets,
# surface conditions and tyre profiles in later increments) and the shared
# `extends` merge-patch helper. A sibling of ps_environment, NOT part of
# ps_core's io/, so a server needs neither Jolt nor the vehicle code.
# nlohmann_json PRIVATE like ps_core: a consumer calling the json-typed
# helpers links nlohmann_json itself.
add_library(ps_environment_io STATIC
  src/environment/io/json_extends.cpp
  src/environment/io/climate_io.cpp
  src/environment/io/weather_io.cpp
  src/environment/io/surface_conditions_io.cpp
  src/environment/io/tyre_profile_io.cpp
)

target_include_directories(ps_environment_io PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(ps_environment_io
  PUBLIC
    ps_environment
  PRIVATE
    nlohmann_json::nlohmann_json
    ps_warnings
    ps_options
)

# ps_engine_cycle_standalone (ps_audio A1 coordinator review's link-time
# guard: proves ps_engine_cycle links and runs WITHOUT ps_core) is defined
# in tests/unit/CMakeLists.txt, not here, even though it links only
# ps_engine_cycle from this file - core/ is add_subdirectory'd BEFORE the
# top-level CMakeLists.txt calls enable_testing()/include(CTest) (those run
# right before tests/* are added), so an add_test() call made from this
# file would register before CTest testing is enabled for this directory
# and be silently dropped. tests/unit/CMakeLists.txt is processed after
# enable_testing(), the same place every other CTest-registered
# executable in this project (bench_force_elements etc., in
# tests/bench/CMakeLists.txt) is defined despite also linking a target from
# core/. See tests/unit/engine_cycle_standalone_main.cpp for the guard's
# own implementation and header comment.

# ps_core — public API in include/ps, implementation in src/. Phase 0 only
# has the version placeholder and the Jolt link proof; math/, jobs/, world/,
# backend/, force/, vehicle/, drivetrain/, aero/, machine/, terrain/, io/
# are empty (.gitkeep) until P1+.

add_library(ps_core STATIC
  src/io/aero_io.cpp
  src/aero/aerodynamics.cpp
  src/aero/coefficient_map.cpp
  src/version.cpp
  src/jolt_proof.cpp

  src/math/math_check.cpp

  src/jobs/job_system.cpp
  src/jobs/task_graph.cpp
  src/jobs/topology.cpp
  src/jobs/debug_alloc.cpp

  src/backend/shape_handle.cpp
  src/backend/jolt/jolt_backend.cpp

  src/articulation/model.cpp
  src/articulation/rnea.cpp
  src/articulation/crba.cpp
  src/articulation/aba.cpp
  src/articulation/loop_closure.cpp
  src/articulation/instance.cpp
  src/articulation/system.cpp
  src/articulation/contact.cpp

  src/world/world.cpp

  src/vehicle/tyre_model.cpp
  src/vehicle/pacejka_tyre_model.cpp
  src/vehicle/tir_parser.cpp
  src/vehicle/wheel_instance.cpp
  src/vehicle/assist_unit.cpp
  src/vehicle/vehicle_instance.cpp
  src/vehicle/system.cpp
  src/io/vehicle_io.cpp
  src/io/turbo_stage_io.cpp
  src/io/fuel_io.cpp
  src/io/nitrous_io.cpp
  src/io/engine_thermal_io.cpp
  src/io/engine_parts.cpp
  src/io/electronics_io.cpp
  src/io/limit_policy_io.cpp
  src/io/knock_model_io.cpp

  src/drivetrain/box_lcp.cpp
  src/drivetrain/network.cpp
  src/drivetrain/components.cpp
  src/drivetrain/powertrain.cpp
  src/drivetrain/engine_torque_map.cpp
  src/drivetrain/engine_simulated.cpp
  src/drivetrain/gearbox.cpp
  src/drivetrain/manual_tcu.cpp
  src/drivetrain/auto_transmission.cpp
  src/drivetrain/engine_sound_state.cpp
  src/drivetrain/engine_rig.cpp
  src/drivetrain/engine_dyno.cpp
  src/drivetrain/electric_envelope.cpp
  src/drivetrain/powertrain_dyno.cpp
  src/drivetrain/limit_findings.cpp
  src/drivetrain/limit_remedies.cpp
  src/drivetrain/engine_rules.cpp
  src/drivetrain/turbo_meanline.cpp
  src/drivetrain/limit_policy.cpp
  src/drivetrain/engine_change.cpp
  src/drivetrain/nitrous_flow.cpp
  src/drivetrain/nitrous_system.cpp
  src/drivetrain/engine_thermal.cpp

  # Engine-map generator/disk-cache infrastructure (I7c C16a/C16b) - NOT
  # part of ps_engine_cycle above: both need ps::jobs::JobSystem (ps_core-
  # only implementation) and/or file IO, so they live here even though the
  # cycle-model math they drive (ps::engine_cycle::CycleStepper etc.) is in
  # ps_engine_cycle. See ps_engine_cycle's own header comment above.
  src/engine_cycle/map_generator.cpp
  src/engine_cycle/map_cache.cpp

  src/terrain/procedural_terrain_source.cpp
  src/terrain/procedural_terrain.cpp
  src/terrain/tile_manager.cpp

  src/io/telemetry.cpp
  src/io/scenario.cpp
  src/io/surface_table.cpp
  src/io/test_ground_loader.cpp
)

target_include_directories(ps_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

# Jolt is linked PRIVATELY: no Jolt type may leak through ps_core's public
# headers (PLAN.md D2). Consumers of ps_core never see Jolt. ps_engine_cycle
# is PRIVATE too, ps_audio A1 (out/audio_plan.md D-11) - vehicle_io.cpp uses
# ps_engine_cycle's public stepper API (via the map_generator.cpp/
# map_cache.cpp sources above, which live in THIS target, not
# ps_engine_cycle - see that target's own header comment for why); a
# ps_core consumer never links ps_engine_cycle directly for this. CMake
# still forwards ps_engine_cycle's own archive onto the final link line of
# anything that links ps_core (same mechanism Jolt already relies on for a
# PRIVATE static-lib dependency) since ps_core's own object files call into
# it (CycleStepper etc.).
target_link_libraries(ps_core
  PRIVATE
    Jolt
    nlohmann_json::nlohmann_json
    ps_warnings
    ps_options
    ps_engine_cycle
    ps_detmath
    # W1: nothing in ps_core calls these yet (W2 wires World in); linked now so
    # the dependency direction ps_core -> ps_environment is fixed from the start.
    ps_environment
    ps_environment_io
)
