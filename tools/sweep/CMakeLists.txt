# tools/sweep — fans N scenarios out over local processes (PLAN.md D9);
# depends only on the standard library and platform process APIs (never
# ps_core - see main.cpp), so it stays a thin launcher around ps_run.

add_executable(sweep main.cpp)

target_link_libraries(sweep
  PRIVATE
    ps_headless_env
    ps_warnings
    ps_options
)

add_dependencies(sweep ps_run)
