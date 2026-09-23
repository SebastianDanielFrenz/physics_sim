# tests/bench — PLAN.md P1 acceptance: 2000 independent force elements
# scale >= 6x from 1 to 10 workers. Plain executable (not Catch2) so its
# measured numbers are always on stdout; run on the release preset for a
# meaningful pass/fail (see bench_force_elements.cpp's header comment).

# Which builds ASSERT wall-clock numbers (single source of truth for both
# the benches' own #if and the ctest labels below):
# - bench_force_elements: every non-Debug build type (where NDEBUG is set);
# - bench_terrain_tiles:  every non-sanitizer build (PS_SANITIZER_BUILD OFF).
# A bench that asserts timing on this build gets LABELS walltime and
# PROCESSORS 24. tools/ci.ps1 excludes "walltime" from its parallel phase
# (where Windows and WSL legs overlap) and runs those tests afterwards in a
# serial phase with nothing else running, one leg at a time. PROCESSORS 24
# additionally keeps a hand-run `ctest --parallel` from scheduling anything
# beside them. A bench that only REPORTS timing on this build (sanitizer or
# Debug) is an ordinary test: no label, default PROCESSORS, so it no longer
# serialises the sanitizer legs for a number nobody checks.
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  set(PS_BENCH_FORCE_ASSERTS_TIMING 0)
else()
  set(PS_BENCH_FORCE_ASSERTS_TIMING 1)
endif()
if(PS_SANITIZER_BUILD)
  set(PS_BENCH_TERRAIN_ASSERTS_TIMING 0)
else()
  set(PS_BENCH_TERRAIN_ASSERTS_TIMING 1)
endif()
function(ps_mark_walltime_bench test_name asserts_timing)
  if(asserts_timing)
    set_tests_properties(${test_name} PROPERTIES LABELS walltime PROCESSORS 24)
  endif()
endfunction()

add_executable(bench_force_elements bench_force_elements.cpp)

target_include_directories(bench_force_elements PRIVATE ${CMAKE_SOURCE_DIR}/tests/support)

target_link_libraries(bench_force_elements
  PRIVATE
    ps_core
    ps_headless_env
    ps_warnings
    ps_options
)

target_compile_definitions(bench_force_elements PRIVATE PS_BENCH_ASSERT_TIMING=${PS_BENCH_FORCE_ASSERTS_TIMING})

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
# test completely alone - on every build where it asserts (see ps_mark_walltime_bench above).
ps_mark_walltime_bench(bench_force_elements ${PS_BENCH_FORCE_ASSERTS_TIMING})

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

# bench_terrain_tiles - PLAN.md M1 terrain-tile acceptance: per-tick/worst-
# tick cost while streaming tiles around a moving interest point, using the
# real data/tracks/test_ground/terrain.json. Registered as a smoke check
# (see bench_terrain_tiles.cpp's header comment for the bound it asserts).
add_executable(bench_terrain_tiles bench_terrain_tiles.cpp)

target_include_directories(bench_terrain_tiles PRIVATE ${CMAKE_SOURCE_DIR}/tests/support)

target_link_libraries(bench_terrain_tiles
  PRIVATE
    ps_core
    ps_headless_env
    ps_warnings
    ps_options
)

# Same reason as tests/unit/CMakeLists.txt's PS_SOURCE_DIR: this bench loads
# data/tracks/test_ground/terrain.json by an absolute path baked in at
# configure time (the WSL/Linux presets build entirely outside the source
# tree, so no relative/CWD-based lookup works on every preset).
target_compile_definitions(bench_terrain_tiles PRIVATE PS_SOURCE_DIR="${CMAKE_SOURCE_DIR}")

# Sanitizer bench policy: on a PS_SANITIZER_BUILD (set explicitly by the
# asan / linux-asan-ubsan / linux-tsan presets, top-level CMakeLists.txt)
# this bench still runs in full - real ASan/UBSan/TSan coverage of the
# streaming + suppression path - but only REPORTS its wall-clock numbers;
# the tick-time bounds are asserted on every other build. Non-timing
# checks (no starved tiles) are asserted everywhere. Always defined (0/1)
# so the source can #error if the definition is ever missing.
target_compile_definitions(bench_terrain_tiles PRIVATE PS_SANITIZER_BUILD=$<BOOL:${PS_SANITIZER_BUILD}>)

add_test(NAME bench_terrain_tiles COMMAND bench_terrain_tiles)

# Same root cause and same fix as bench_force_elements above: this is a
# wall-clock timing bound (its own 250 ms smoke bound), and tools/ci.ps1
# -Full's WSL legs (ctest --parallel 24) schedule this bench alongside
# many other tests, whose CPU contention corrupts the tick-time
# measurement - found via -Full: worst tick 338.999 ms (linux-asan-ubsan)
# / 529.608 ms (linux-tsan) under contention vs. 55.8 ms / 90.0 ms when run
# alone immediately afterward on the same machine, both comfortably under
# the 250 ms bound. PROCESSORS above the highest --parallel level
# tools/ci.ps1 uses (WSL's 24) makes ctest's scheduler always run this test
# completely alone wherever it asserts - not a loosened threshold, the
# measurement itself needs to run uncontended to mean anything.
ps_mark_walltime_bench(bench_terrain_tiles ${PS_BENCH_TERRAIN_ASSERTS_TIMING})

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
