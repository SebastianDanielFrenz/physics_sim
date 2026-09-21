# Phase 0 has a single smoke test (test_hello.cpp) proving ps_core builds,
# its version API works, and Jolt actually links. Real unit tests for math,
# world stepping, jobs etc. arrive in P1.

add_executable(ps_unit_tests
  test_hello.cpp
)

target_link_libraries(ps_unit_tests
  PRIVATE
    ps_core
    Catch2::Catch2WithMain
    ps_warnings
    ps_options
)

list(APPEND CMAKE_MODULE_PATH ${catch2_SOURCE_DIR}/extras)
include(Catch)
catch_discover_tests(ps_unit_tests)
