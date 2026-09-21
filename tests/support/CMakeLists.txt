# tests/support — shared test-only helpers: counter_force_element.h
# (header-only, included directly via target_include_directories by
# tests/determinism and tests/bench) and a custom Catch2 main that installs
# the headless CRT handlers (see catch_main.cpp / tools/common/headless_env.h).

add_library(ps_test_catch_main STATIC catch_main.cpp)

target_link_libraries(ps_test_catch_main
  PUBLIC
    Catch2::Catch2
    ps_headless_env
  PRIVATE
    ps_warnings
    ps_options
)
