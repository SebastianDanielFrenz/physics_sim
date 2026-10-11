# Phase 0 has a single smoke test (test_hello.cpp) proving ps_core builds,
# its version API works, and Jolt actually links. Real unit tests for math,
# world stepping, jobs etc. arrive in P1.

add_executable(ps_unit_tests
  test_traction_control.cpp
  test_adaptive_pressure.cpp
  test_live_engine_ambient.cpp
  test_aerodynamics.cpp
  test_aero_coefficient_map.cpp
  test_hello.cpp
  test_math.cpp
  test_jobs.cpp
  test_job_background_lane.cpp
  test_world_basic.cpp
  test_environment_world.cpp
  test_environment_surface_world.cpp
  test_world_atmosphere.cpp
  test_world_weather_override.cpp
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
  test_heightfield_spare.cpp
  test_prebuilt_shapes.cpp
  test_terrain_golden.cpp
  test_terrain_tile_manager.cpp
  test_terrain_static_overlap.cpp
  test_terrain_suppression_precompute.cpp
  test_terrain_tile_datum.cpp
  test_terrain_prefetch.cpp
  test_pacejka_tyre_model.cpp
  test_tyre_rolling_resistance.cpp
  test_vehicle_ackermann_symmetry.cpp
  test_vehicle_io_validation.cpp
  test_vehicle_parallel_alloc.cpp
  test_wheel_contact_frame.cpp
  test_powertrain_network.cpp
  test_powertrain_friction_lcp.cpp
  test_powertrain_engine.cpp
  test_powertrain_gearbox.cpp
  test_powertrain_tcu.cpp
  test_powertrain_reverse_launch.cpp
  test_hyper_idle_recovery.cpp
  test_powertrain_control_channels.cpp
  test_powertrain_multi_diff.cpp
  test_powertrain_topology.cpp
  test_powertrain_diff.cpp
  test_powertrain_transfer_case.cpp
  test_powertrain_park_lock.cpp
  test_powertrain_pto.cpp
  test_powertrain_compliance.cpp
  test_sub_threshold_rotation.cpp
  test_table2d.cpp
  test_fuel_io.cpp
  test_engine_thermal_io.cpp
  test_engine_thermal.cpp
  test_engine_thermal_powertrain.cpp
  test_engine_thermal_coupling.cpp
  test_engine_thermal_scenarios.cpp
  test_n2o_flow.cpp
  test_n2o_system.cpp
  test_n2o_controller.cpp
  test_n2o_engine.cpp
  test_n2o_turbo.cpp
  test_throttle_control.cpp
  test_engine_fuel_map.cpp
  test_fuel_tank.cpp
  test_ambient.cpp
  test_cited_engine_data.cpp
  test_simulated_engine_io.cpp
  test_simulated_engine.cpp
  test_turbo_pair.cpp
  test_turbo_bov.cpp
  test_turbo_surge.cpp
  test_turbo_intercooler.cpp
  test_turbo_io.cpp
  test_turbo_stage_maps.cpp
  test_turbo_vehicle.cpp
  test_turbo_compressor_flow.cpp
  test_turbo_anti_lag.cpp
  test_turbo_exhaust_thermal.cpp
  test_turbo_als_cut.cpp
  test_turbo_wastegate_split.cpp
  test_simulated_engine_powertrain.cpp
  test_a25a_sim_engine_data.cpp
  test_engine_cycle.cpp
  test_cycle_engine.cpp
  test_cycle_engine_studies.cpp
  test_engine_rig_wheelless.cpp
  test_pipe_network.cpp
  test_pipe_network_simd.cpp
  test_waveguide_network.cpp
  test_pipe_waveguide.cpp
  test_engine_map_generator.cpp
  test_engine_map_cache.cpp
  test_engine_map_gen_v2.cpp
  test_engine_map_control.cpp
  test_engine_dyno.cpp
  test_limit_findings.cpp
  test_limit_remedies.cpp
  test_spark_schedule.cpp
  test_engine_installation_io.cpp
  test_engine_hot_swap.cpp
  test_firing_order.cpp
  test_audio_time_map.cpp
  test_audio_voice.cpp
  test_audio_bank.cpp
  ${CMAKE_SOURCE_DIR}/tools/common/engine_bank_generator.cpp
  test_native_audio_output.cpp
  test_voice_threads.cpp
  test_engine_sound_glue.cpp
  test_engine_rig_audio.cpp
)

target_link_libraries(ps_unit_tests
  PRIVATE
    ps_core
    ps_audio
    ps_test_catch_main
    ps_warnings
    ps_options
    # test_terrain_golden.cpp parses data/tracks/test_ground/terrain_golden.json
    # directly (nlohmann/json is linked PRIVATE into ps_core, so a test that
    # needs it too must link it itself rather than get it transitively).
    nlohmann_json::nlohmann_json
    # test_engine_sound_glue.cpp (ps_audio A11) includes tools/common/
    # engine_sound_glue.h; ps_headless_env PUBLICly exposes tools/common as
    # an include dir (see tools/common/CMakeLists.txt) - linked only for
    # that include path, its own CRT-handler code is unused here.
    ps_headless_env
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

# ps_engine_cycle_standalone — link-time guard (ps_audio A1 coordinator
# review): a plain main() (NOT Catch2 - every Catch2 executable in this
# project assumes ps_core via ps_test_catch_main) that links ONLY
# ps_engine_cycle + ps_warnings + ps_options (defined in core/CMakeLists.txt;
# explicitly NOT ps_core, NOT ps_headless_env) and runs one real cycle point
# via the public stepper API (tests/unit/engine_cycle_standalone_main.cpp).
# If map_generator.cpp/map_cache.cpp (or anything else needing ps_core's own
# implementation, e.g. ps::jobs::JobSystem) ever moves back into
# ps_engine_cycle's source list, this target fails to LINK (an unresolved
# external symbol), catching the cycle before it reaches ps_core - see that
# file's own header comment for the sabotage that verified this. Defined
# here rather than in core/CMakeLists.txt (even though it only links a
# core/ target) because add_subdirectory(core) runs before the top-level
# CMakeLists.txt calls enable_testing()/include(CTest); an add_test() call
# made before that point is silently dropped.
#
# $<LINK_LIBRARY:WHOLE_ARCHIVE,ps_engine_cycle> (not a plain
# target_link_libraries entry): a static archive only pulls in the .o
# members that resolve a symbol something ELSE already references
# (per-object extraction, PLAN.md/the coordinator review's own point) -
# engine_cycle_standalone_main.cpp calls only into cycle_stepper.cpp/
# cycle_point.cpp, so a reintroduced map_generator.cpp would sit in the
# archive UNREFERENCED and never get pulled in, silently passing this guard
# despite needing ps::jobs::JobSystem. Whole-archive linking forces every
# object file in ps_engine_cycle's archive to be linked, so any such
# unresolved symbol surfaces regardless of whether this file's own code
# happens to call it - portable across MSVC (/WHOLEARCHIVE:) and
# GNU-ld/lld (--whole-archive), both used by this project's presets.
add_executable(ps_engine_cycle_standalone
  engine_cycle_standalone_main.cpp
)

target_link_libraries(ps_engine_cycle_standalone
  PRIVATE
    $<LINK_LIBRARY:WHOLE_ARCHIVE,ps_engine_cycle>
    ps_warnings
    ps_options
)

# tests/support/cycle_fixtures.h is header-only and has no ps_core
# dependency of its own (confirmed when this guard was written) - safe to
# put on this target's include path without pulling in ps_core.
target_include_directories(ps_engine_cycle_standalone PRIVATE ${CMAKE_SOURCE_DIR}/tests/support)

add_test(NAME ps_engine_cycle_standalone COMMAND ps_engine_cycle_standalone)
set_tests_properties(ps_engine_cycle_standalone PROPERTIES LABELS "cycle")

# ps_audio_standalone - ps_audio A7's link-time guard, the same pattern as
# ps_engine_cycle_standalone above: a plain main() linking ps_audio
# WHOLE-ARCHIVE (every object pulled in, referenced or not) plus
# ps_engine_cycle, never ps_core. It renders a short Runner block on the pipe
# I4 fixture (tests/unit/audio_standalone_main.cpp).
add_executable(ps_audio_standalone
  audio_standalone_main.cpp
)

target_link_libraries(ps_audio_standalone
  PRIVATE
    $<LINK_LIBRARY:WHOLE_ARCHIVE,ps_audio>
    ps_engine_cycle
    ps_warnings
    ps_options
)

target_include_directories(ps_audio_standalone PRIVATE ${CMAKE_SOURCE_DIR}/tests/support)

add_test(NAME ps_audio_standalone COMMAND ps_audio_standalone)
set_tests_properties(ps_audio_standalone PROPERTIES LABELS "audio;cycle")

# ps_environment_tests (weather plan W1): the Catch2 tests of ps_environment /
# ps_environment_io. A separate executable that does NOT link ps_core (it links
# only ps_test_catch_main's own dependencies, which are core-free apart from
# the CRT handlers) so the environment library's independence stays visible in
# the link line; the standalone guard below checks it for the libraries
# themselves. Tests carry the [environment] tag, so CTest labels select them.
add_executable(ps_environment_tests
  test_environment_calendar.cpp
  test_environment_sun.cpp
  test_environment_io.cpp
  test_environment_object.cpp
  test_environment_weather.cpp
  test_environment_weather_classes.cpp
  test_environment_weather_io.cpp
  test_environment_atmosphere.cpp
  test_environment_wind.cpp
  test_environment_day_share.cpp
  test_environment_season_override.cpp
  test_environment_surface.cpp
  test_environment_surface_classes.cpp
)

target_link_libraries(ps_environment_tests
  PRIVATE
    ps_environment
    ps_environment_io
    ps_test_catch_main
    ps_warnings
    ps_options
    nlohmann_json::nlohmann_json
)

target_include_directories(ps_environment_tests PRIVATE ${CMAKE_SOURCE_DIR}/tests/support)
target_compile_definitions(ps_environment_tests PRIVATE PS_SOURCE_DIR="${CMAKE_SOURCE_DIR}")

catch_discover_tests(ps_environment_tests ADD_TAGS_AS_LABELS)

# ps_environment_standalone - link-time guard like ps_engine_cycle_standalone:
# a plain main() linking ps_environment and ps_environment_io WHOLE-ARCHIVE
# without ps_core (tests/unit/environment_standalone_main.cpp).
add_executable(ps_environment_standalone
  environment_standalone_main.cpp
)

target_link_libraries(ps_environment_standalone
  PRIVATE
    $<LINK_LIBRARY:WHOLE_ARCHIVE,ps_environment>
    $<LINK_LIBRARY:WHOLE_ARCHIVE,ps_environment_io>
    nlohmann_json::nlohmann_json
    ps_warnings
    ps_options
)

target_include_directories(ps_environment_standalone PRIVATE ${CMAKE_SOURCE_DIR}/tests/support)
target_compile_definitions(ps_environment_standalone PRIVATE PS_SOURCE_DIR="${CMAKE_SOURCE_DIR}")

add_test(NAME ps_environment_standalone COMMAND ps_environment_standalone)
set_tests_properties(ps_environment_standalone PROPERTIES LABELS "environment")
