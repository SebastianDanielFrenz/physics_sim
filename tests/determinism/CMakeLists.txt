# tests/determinism — PLAN.md D7/D8 acceptance: state hash independent of
# worker count and scheduling.

add_executable(ps_determinism_tests
  test_worker_count_hash.cpp
)

target_include_directories(ps_determinism_tests PRIVATE ${CMAKE_SOURCE_DIR}/tests/support)

target_link_libraries(ps_determinism_tests
  PRIVATE
    ps_core
    ps_test_catch_main
    ps_warnings
    ps_options
)

catch_discover_tests(ps_determinism_tests)
