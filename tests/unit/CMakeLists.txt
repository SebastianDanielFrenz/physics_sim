# Phase 0 has a single smoke test (test_hello.cpp) proving ps_core builds,
# its version API works, and Jolt actually links. Real unit tests for math,
# world stepping, jobs etc. arrive in P1.

add_executable(ps_unit_tests
  test_hello.cpp
  test_math.cpp
  test_jobs.cpp
  test_world_basic.cpp
  test_body_integration_params.cpp
  test_world_snapshot.cpp
  test_test_ground_loader.cpp
  test_test_ground_edge_lengths.cpp
  test_articulation_core.cpp
  test_articulation_four_bar.cpp
  test_articulation_parallel_alloc.cpp
  test_aba_fast_transform.cpp
  test_scenario_articulation.cpp
  test_detmath.cpp
  test_articulation_hinge_guard.cpp
  test_articulation_contact.cpp
  test_mesh_heightfield_shapes.cpp
  test_terrain_golden.cpp
  test_terrain_tile_manager.cpp
  test_terrain_static_overlap.cpp
  test_terrain_suppression_precompute.cpp
  test_terrain_tile_datum.cpp
  test_pacejka_tyre_model.cpp
  test_vehicle_ackermann_symmetry.cpp
  test_vehicle_io_validation.cpp
  test_vehicle_parallel_alloc.cpp
  test_wheel_contact_frame.cpp
  test_powertrain_network.cpp
  test_powertrain_friction_lcp.cpp
  test_powertrain_engine.cpp
  test_powertrain_gearbox.cpp
  test_powertrain_tcu.cpp
  test_powertrain_control_channels.cpp
  test_powertrain_multi_diff.cpp
  test_powertrain_topology.cpp
  test_powertrain_diff.cpp
  test_powertrain_transfer_case.cpp
  test_powertrain_park_lock.cpp
  test_powertrain_pto.cpp
  test_powertrain_compliance.cpp
  test_sub_threshold_rotation.cpp
)

target_link_libraries(ps_unit_tests
  PRIVATE
    ps_core
    ps_test_catch_main
    ps_warnings
    ps_options
    # test_terrain_golden.cpp parses data/tracks/test_ground/terrain_golden.json
    # directly (nlohmann/json is linked PRIVATE into ps_core, so a test that
    # needs it too must link it itself rather than get it transitively).
    nlohmann_json::nlohmann_json
)

# test_articulation_core.cpp white-box tests core/src/articulation directly
# (internal spatial algebra / RNEA / CRBA / ABA - not part of ps_core's
# public API surface), so it needs core/src on its include path in
# addition to the public core/include ps_core already exposes.
target_include_directories(ps_unit_tests PRIVATE ${CMAKE_SOURCE_DIR}/core/src ${CMAKE_SOURCE_DIR}/tests/support)

# test_scenario_articulation.cpp needs the repo root as an absolute path
# baked in at configure time (see that file's own comment): the WSL/Linux
# presets build outside the source tree entirely (~/build/physics_sim),
# so no relative/CWD-based lookup of data/scenarios/articulation_chain.json
# can work on every preset.
target_compile_definitions(ps_unit_tests PRIVATE PS_SOURCE_DIR="${CMAKE_SOURCE_DIR}")

list(APPEND CMAKE_MODULE_PATH ${catch2_SOURCE_DIR}/extras)
include(Catch)

# ADD_TAGS_AS_LABELS: every Catch2 tag becomes a CTest LABEL, so tools/ci.ps1
# can select by tag via ctest -L/-LE (see CLAUDE.md "CI" and tests/validation/
# CMakeLists.txt for the fast/full split; unit tests are always fast, so no
# selection is needed here, but the label wiring is shared).
catch_discover_tests(ps_unit_tests ADD_TAGS_AS_LABELS)
