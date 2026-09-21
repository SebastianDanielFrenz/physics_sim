# ps_core — public API in include/ps, implementation in src/. Phase 0 only
# has the version placeholder and the Jolt link proof; math/, jobs/, world/,
# backend/, force/, vehicle/, drivetrain/, aero/, machine/, terrain/, io/
# are empty (.gitkeep) until P1+.

add_library(ps_core STATIC
  src/version.cpp
  src/jolt_proof.cpp

  src/math/math_check.cpp

  src/jobs/job_system.cpp
  src/jobs/task_graph.cpp
  src/jobs/topology.cpp
  src/jobs/debug_alloc.cpp

  src/backend/jolt/jolt_backend.cpp

  src/world/world.cpp

  src/io/telemetry.cpp
  src/io/scenario.cpp
)

target_include_directories(ps_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

# Jolt is linked PRIVATELY: no Jolt type may leak through ps_core's public
# headers (PLAN.md D2). Consumers of ps_core never see Jolt.
target_link_libraries(ps_core
  PRIVATE
    Jolt
    nlohmann_json::nlohmann_json
    ps_warnings
    ps_options
)
