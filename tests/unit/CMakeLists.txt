# Phase 0 has a single smoke test (test_hello.cpp) proving ps_core builds,
# its version API works, and Jolt actually links. Real unit tests for math,
# world stepping, jobs etc. arrive in P1.

add_executable(ps_unit_tests
  test_hello.cpp
  test_math.cpp
  test_jobs.cpp
  test_world_basic.cpp
  test_articulation_core.cpp
  test_articulation_four_bar.cpp
)

target_link_libraries(ps_unit_tests
  PRIVATE
    ps_core
    ps_test_catch_main
    ps_warnings
    ps_options
)

# test_articulation_core.cpp white-box tests core/src/articulation directly
# (internal spatial algebra / RNEA / CRBA / ABA - not part of ps_core's
# public API surface), so it needs core/src on its include path in
# addition to the public core/include ps_core already exposes.
target_include_directories(ps_unit_tests PRIVATE ${CMAKE_SOURCE_DIR}/core/src)

list(APPEND CMAKE_MODULE_PATH ${catch2_SOURCE_DIR}/extras)
include(Catch)
catch_discover_tests(ps_unit_tests)
