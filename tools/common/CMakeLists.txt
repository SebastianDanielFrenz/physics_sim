# tools/common — small shared helpers for tools/tests executables (never
# linked into ps_core). See headless_env.h for why this exists.

add_library(ps_headless_env STATIC headless_env.cpp)

target_include_directories(ps_headless_env PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(ps_headless_env
  PRIVATE
    ps_warnings
    ps_options
)
