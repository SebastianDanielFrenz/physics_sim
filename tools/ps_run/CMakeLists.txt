# ps_run — headless scenario runner (CLI -> CSV). Placeholder for Phase 0;
# scenario loading and stepping arrive in P1 (PLAN.md section 4).

add_executable(ps_run main.cpp)

target_link_libraries(ps_run
  PRIVATE
    ps_core
    ps_warnings
    ps_options
)
