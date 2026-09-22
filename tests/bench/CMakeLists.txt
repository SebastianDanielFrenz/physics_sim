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

# This is a wall-clock scaling benchmark (asserts >= 6x speedup 1->10
# workers from busy_ns measurements) - it went from a comfortable pass in
# isolation to an outright FAIL under tools/ci.ps1 -Full once ctest started
# passing --parallel: CPU contention from sibling tests scheduled alongside
# it corrupts its own timing measurements. Measured on this machine: 9.08x
# isolated vs. 1.59x under a Windows --parallel 12 run and 4.54x under a
# WSL --parallel 24 run, both well under the 6x bound, both on an otherwise
# fully green run (every other test - including every [long]/[heavy]/
# [sanitizer] one - passed). PROCESSORS is set above the highest --parallel
# level tools/ci.ps1 uses (WSL's 24) so ctest's scheduler always runs this
# test completely alone, regardless of leg.
set_tests_properties(bench_force_elements PROPERTIES PROCESSORS 24)

# bench_contact_coupling - PLAN.md P1.5b measurement (f): cost per tick per
# strategy, scaling with contact count (bench_contact_coupling.cpp's header
# comment). Registered as a smoke check (must run to completion without
# crashing) like bench_force_elements, but does not itself assert a scaling
# bound - there is no PLAN.md acceptance number for this measurement.
add_executable(bench_contact_coupling bench_contact_coupling.cpp)

target_include_directories(bench_contact_coupling PRIVATE ${CMAKE_SOURCE_DIR}/tests/support)

target_link_libraries(bench_contact_coupling
  PRIVATE
    ps_core
    ps_headless_env
    ps_warnings
    ps_options
)

add_test(NAME bench_contact_coupling COMMAND bench_contact_coupling)

# ps_energy_study - Part A energy-loss measurement study (see
# tests/bench/energy_study.cpp's header comment). Deliberately NOT
# registered with add_test/CTest: it is a measurement tool whose table feeds
# a written report, not a pass/fail acceptance test.
add_executable(ps_energy_study energy_study.cpp)

target_include_directories(ps_energy_study PRIVATE ${CMAKE_SOURCE_DIR}/tests/support)

target_link_libraries(ps_energy_study
  PRIVATE
    ps_core
    ps_headless_env
    ps_warnings
    ps_options
)
