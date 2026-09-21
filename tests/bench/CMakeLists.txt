# tests/bench — PLAN.md P1 acceptance: 2000 independent force elements
# scale >= 6x from 1 to 10 workers. Plain executable (not Catch2) so its
# measured numbers are always on stdout; run on the release preset for a
# meaningful pass/fail (see bench_force_elements.cpp's header comment).

add_executable(bench_force_elements bench_force_elements.cpp)

target_include_directories(bench_force_elements PRIVATE ${CMAKE_SOURCE_DIR}/tests/support)

target_link_libraries(bench_force_elements
  PRIVATE
    ps_core
    ps_headless_env
    ps_warnings
    ps_options
)

add_test(NAME bench_force_elements COMMAND bench_force_elements)
