# tests/validation — physics acceptance tests (PLAN.md P1: free fall,
# pendulum period, joint-chain energy drift, box-on-slope friction).

add_executable(ps_validation_tests
  test_free_fall.cpp
  test_pendulum.cpp
  test_joint_chain_energy.cpp
  test_box_on_slope.cpp
  test_tunnelling.cpp
  test_resting_contact.cpp
  test_huge_forces.cpp
  test_mass_ratio.cpp
  test_articulation_pendulum.cpp
  test_articulation_double_pendulum.cpp
  test_articulation_energy.cpp
  test_articulation_mass_ratio.cpp
  test_articulation_floating_base.cpp
  test_articulation_actuator.cpp
  test_articulation_four_bar_energy.cpp
  test_articulation_contact_measurements.cpp
  test_vehicle_static.cpp
  test_vehicle_parking_slope.cpp
  test_vehicle_skidpad.cpp
  test_vehicle_braking.cpp
  test_vehicle_step_steer.cpp
  test_vehicle_accel_standing_start.cpp
  test_vehicle_steer_mirror.cpp
  test_vehicle_standstill_wiggle.cpp
  test_vehicle_tractor_cross_slope.cpp
  test_vehicle_shadow_chassis.cpp
)

target_link_libraries(ps_validation_tests
  PRIVATE
    ps_core
    ps_test_catch_main
    ps_warnings
    ps_options
)

# test_articulation_four_bar_energy.cpp white-box-seeds its initial
# condition via core/src/articulation (internal) - see
# tests/unit/CMakeLists.txt's identical comment. tests/support: the
# vehicle validation tests share vehicle_test_helpers.h (build a World +
# chassis + vehicle from a data/vehicles/*.json path).
target_include_directories(ps_validation_tests PRIVATE ${CMAKE_SOURCE_DIR}/core/src ${CMAKE_SOURCE_DIR}/tests/support)

# The vehicle validation tests load data/vehicles/*.json and
# data/surfaces/surfaces.json by an absolute, source-tree-relative path
# (same reasoning as tests/unit/test_scenario_articulation.cpp's identical
# PS_SOURCE_DIR use - the WSL/Linux presets build outside the source tree).
target_compile_definitions(ps_validation_tests PRIVATE PS_SOURCE_DIR="${CMAKE_SOURCE_DIR}")

# ADD_TAGS_AS_LABELS: every Catch2 tag (e.g. [long], [sanitizer]) becomes a
# CTest LABEL. tools/ci.ps1's default (fast) run selects tests via ctest -L/
# -LE against these labels:
#   - release legs (release, msvc-release, linux-release): "-LE long"
#     (everything except [long]).
#   - debug/asan/linux-asan-ubsan/linux-tsan legs: "-LE validation|long"
#     (skips the whole validation suite except the short [sanitizer]
#     variants, which are new test cases that deliberately do NOT carry the
#     [validation] tag so this one exclusion regex routes them correctly -
#     see e.g. test_joint_chain_energy.cpp's short variant).
# tools/ci.ps1 -Full applies no label filter (every test, every leg, as
# before this split). See CLAUDE.md "CI" for the full rationale.
catch_discover_tests(ps_validation_tests ADD_TAGS_AS_LABELS)
