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
# tests/unit/CMakeLists.txt's identical comment.
target_include_directories(ps_validation_tests PRIVATE ${CMAKE_SOURCE_DIR}/core/src)

catch_discover_tests(ps_validation_tests)
