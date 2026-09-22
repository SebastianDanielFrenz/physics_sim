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

# Two of the three test cases here spin up their own JobSystem with up to
# 24 workers (the full physical+logical thread count of the dev machine);
# tagged [heavy] and given CTest PROCESSORS=24 so ctest --parallel N's
# scheduler never runs one of them alongside anything else (avoids
# oversubscription - see tools/ci.ps1's comment on this). The stress test
# (up to 8 workers, sub-second) is left at the CTest default of PROCESSORS=1;
# its own worker-count contention window is brief enough not to matter.
catch_discover_tests(ps_determinism_tests TEST_SPEC "~[heavy]" ADD_TAGS_AS_LABELS)
catch_discover_tests(ps_determinism_tests TEST_SPEC "[heavy]" ADD_TAGS_AS_LABELS PROPERTIES PROCESSORS 24)
