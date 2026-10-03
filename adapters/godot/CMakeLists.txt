# adapters/godot/CMakeLists.txt — GDExtension adapter (PLAN.md P2), gated
# by PS_BUILD_GODOT_ADAPTER (top-level CMakeLists.txt). godot-cpp is fetched
# by the top-level file, pinned to branch 4.5 - see that file's comment and
# this adapter's report for why no branch/tag matching Godot 4.7 exists
# upstream yet (checked again for this phase: still true as of the date in
# this comment's commit).
#
# Two things get built here: the GDExtension shared library itself
# (ps_godot_adapter) and a small set of plain-C++ (no test framework, no
# Godot) test executables for the numeric cores that must be unit-testable
# with synthetic input (PLAN.md P2 acceptance) - registered with CTest via
# add_test() directly, since tests/ is the other agent's lane right now.

set(PS_GODOT_ADAPTER_BIN_DIR "${CMAKE_CURRENT_SOURCE_DIR}/demo/bin")

add_library(ps_godot_adapter SHARED
  ${CMAKE_SOURCE_DIR}/tools/common/engine_bank_generator.cpp
  src/register_types.cpp
  src/frame_convert_core.cpp
  src/input_map_core.cpp
  src/input_map.cpp
  src/haptics_core.cpp
  src/ps_simulation.cpp
  src/ps_body_node.cpp
  src/debug_draw.cpp
  src/collision_import.cpp
  src/spatial_audio.cpp
  src/vehicle_voice.cpp
)

target_include_directories(ps_godot_adapter PRIVATE src)
# ps_audio A11: tools/common/engine_sound_glue.h (ps_tools::make_engine_input)
# is the one shared EngineSoundState->EngineInput mapping between
# tools/ps_engine_audio and this adapter (core's CLAUDE.md: neither ps_core
# nor ps_audio may include the other, so the glue lives outside both). Only
# the include path is needed here, not the ps_headless_env TARGET - that
# target's own .cpp installs Windows headless-CRT/abort handlers, meant for
# standalone tool/test executables, not a Godot-hosted GDExtension DLL.
target_include_directories(ps_godot_adapter PRIVATE ${CMAKE_SOURCE_DIR}/tools/common)

# godot-cpp's own CMakeLists.txt (branch 4.5) defines the target as
# "godot-cpp"; newer godot-cpp versions also alias it to "godot::cpp", but
# 4.5 does not, so link the unaliased name directly.
# NOT linked against ps_warnings, unlike every other target in this repo -
# confirmed by actually building this target, not assumed. godot-cpp
# (branch 4.5) is header-heavy (inline/template bindings: core/memory.hpp's
# placement-new overloads, classes/wrapped.hpp's virtual-notification
# stubs, ...), so its own unused-parameter-laden headers get compiled as
# part of OUR translation units, not just godot-cpp's own target (which
# PLAN.md's "warnings-as-errors ... never [applied] to third-party targets"
# already exempts) - ps_warnings' /W4 /WX then fails on THIRD-PARTY code
# merely included by ours. Tried appending -Wno-unused-parameter as an
# extra ps_godot_adapter compile option first: clang resolves -W flags in
# command-line order, and CMake always emits a target's own
# target_compile_options before the flags propagated from a linked
# INTERFACE library, so ps_warnings' later /W4 unconditionally re-enabled
# the warning regardless of where -Wno-unused-parameter was added - the fix
# would need every godot_cpp include wrapped in its own
# push/ignored/pop pragma block, which is more churn than this phase
# warrants. ps_options (architecture/FP flags) is still linked below.
target_link_libraries(ps_godot_adapter
  PRIVATE
    ps_core
    ps_audio
    godot-cpp
    nlohmann_json::nlohmann_json
    ps_options
)

if(WIN32)
  target_link_libraries(ps_godot_adapter PRIVATE ole32)
endif()

set_target_properties(ps_godot_adapter PROPERTIES
  OUTPUT_NAME "libps_godot"
  RUNTIME_OUTPUT_DIRECTORY "${PS_GODOT_ADAPTER_BIN_DIR}"
  RUNTIME_OUTPUT_DIRECTORY_DEBUG "${PS_GODOT_ADAPTER_BIN_DIR}"
  RUNTIME_OUTPUT_DIRECTORY_RELEASE "${PS_GODOT_ADAPTER_BIN_DIR}"
  RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO "${PS_GODOT_ADAPTER_BIN_DIR}"
  LIBRARY_OUTPUT_DIRECTORY "${PS_GODOT_ADAPTER_BIN_DIR}"
  LIBRARY_OUTPUT_DIRECTORY_DEBUG "${PS_GODOT_ADAPTER_BIN_DIR}"
  LIBRARY_OUTPUT_DIRECTORY_RELEASE "${PS_GODOT_ADAPTER_BIN_DIR}"
  LIBRARY_OUTPUT_DIRECTORY_RELWITHDEBINFO "${PS_GODOT_ADAPTER_BIN_DIR}"
)

# ---------------------------------------------------------------------------
# Plain-C++ test executables (PLAN.md P2: "a plain C++ test in the adapter's
# own test target, not in tests/"). No Catch2, no godot-cpp - only the
# Godot-free numeric cores (frame_convert_core, origin_rebase,
# input_map_core) plus ps_core for ps::Vec3/Quat. Each returns its failure
# count as the process exit code, so add_test()'s pass/fail is exactly that.
# ---------------------------------------------------------------------------

add_executable(ps_godot_test_input_map tests/test_input_map.cpp src/input_map_core.cpp)
target_include_directories(ps_godot_test_input_map PRIVATE src)
target_link_libraries(ps_godot_test_input_map PRIVATE ps_warnings ps_options)

add_executable(ps_godot_test_origin_rebase tests/test_origin_rebase.cpp src/frame_convert_core.cpp)
target_include_directories(ps_godot_test_origin_rebase PRIVATE src)
target_link_libraries(ps_godot_test_origin_rebase PRIVATE ps_core ps_warnings ps_options)

# Render-time interpolation (render_interp.h, PLAN.md D3 judder fix) -
# header-only and Godot-free like origin_rebase.h, so this needs only
# ps_core for ps::Pose/Quat/slerp - no extra .cpp to compile alongside it.
add_executable(ps_godot_test_render_interp tests/test_render_interp.cpp)
target_include_directories(ps_godot_test_render_interp PRIVATE src)
target_link_libraries(ps_godot_test_render_interp PRIVATE ps_core ps_warnings ps_options)

# Multi-stage gamepad rumble core (haptics_core.h/.cpp) - Godot-free like
# input_map_core.h, so this stays a plain-C++ test with no godot-cpp/ps_core
# dependency, same reasoning as ps_godot_test_input_map above.
add_executable(ps_godot_test_haptics tests/test_haptics.cpp src/haptics_core.cpp)
target_include_directories(ps_godot_test_haptics PRIVATE src)
target_link_libraries(ps_godot_test_haptics PRIVATE ps_warnings ps_options)

# Registered unconditionally (independent of PS_BUILD_TESTS, which governs
# the other agent's tests/ suite only): include(CTest) calls enable_testing()
# itself if it has not already run, so these register whenever
# PS_BUILD_GODOT_ADAPTER is ON regardless of PS_BUILD_TESTS.
include(CTest)
add_test(NAME ps_godot_test_input_map COMMAND ps_godot_test_input_map)
add_test(NAME ps_godot_test_origin_rebase COMMAND ps_godot_test_origin_rebase)
add_test(NAME ps_godot_test_haptics COMMAND ps_godot_test_haptics)
add_test(NAME ps_godot_test_render_interp COMMAND ps_godot_test_render_interp)

target_include_directories(ps_godot_adapter PRIVATE ${CMAKE_SOURCE_DIR}/core/src ${CMAKE_SOURCE_DIR}/tools/common)

target_link_libraries(ps_godot_adapter PRIVATE nlohmann_json::nlohmann_json)
