# ps_core — public API in include/ps, implementation in src/. Phase 0 only
# has the version placeholder and the Jolt link proof; math/, jobs/, world/,
# backend/, force/, vehicle/, drivetrain/, aero/, machine/, terrain/, io/
# are empty (.gitkeep) until P1+.

add_library(ps_core STATIC
  src/version.cpp
  src/jolt_proof.cpp

  src/math/math_check.cpp
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

  src/jobs/job_system.cpp
  src/jobs/task_graph.cpp
  src/jobs/topology.cpp
  src/jobs/debug_alloc.cpp

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
  src/vehicle/vehicle_instance.cpp
  src/vehicle/system.cpp
  src/io/vehicle_io.cpp
  src/io/fuel_io.cpp

  src/drivetrain/box_lcp.cpp
  src/drivetrain/network.cpp
  src/drivetrain/components.cpp
  src/drivetrain/powertrain.cpp
  src/drivetrain/engine_torque_map.cpp
  src/drivetrain/engine_simulated.cpp
  src/drivetrain/gearbox.cpp
  src/drivetrain/manual_tcu.cpp
  src/engine_cycle/geometry.cpp
  src/engine_cycle/cycle_stepper.cpp
  src/engine_cycle/cycle_point.cpp
  src/engine_cycle/map_generator.cpp

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
# headers (PLAN.md D2). Consumers of ps_core never see Jolt.
target_link_libraries(ps_core
  PRIVATE
    Jolt
    nlohmann_json::nlohmann_json
    ps_warnings
    ps_options
)
