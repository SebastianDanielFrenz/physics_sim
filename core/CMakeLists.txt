# ps_core — public API in include/ps, implementation in src/. Phase 0 only
# has the version placeholder and the Jolt link proof; math/, jobs/, world/,
# backend/, force/, vehicle/, drivetrain/, aero/, machine/, terrain/, io/
# are empty (.gitkeep) until P1+.

add_library(ps_core STATIC
  src/version.cpp
  src/jolt_proof.cpp
)

target_include_directories(ps_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

# Jolt is linked PRIVATELY: no Jolt type may leak through ps_core's public
# headers (PLAN.md D2). Consumers of ps_core never see Jolt.
target_link_libraries(ps_core
  PRIVATE
    Jolt
    ps_warnings
    ps_options
)
