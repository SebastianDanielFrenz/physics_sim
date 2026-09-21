cmake_minimum_required(VERSION 3.25)

# CMP0091: let CMAKE_MSVC_RUNTIME_LIBRARY select the CRT (NEW behaviour is
# the default for cmake_minimum_required >= 3.15, kept explicit here). Both
# this policy and the runtime-library selection itself must be set BEFORE
# project() - CMake's internal "does this compiler work" ABI-detection
# try-compile (triggered by project()/enable_language) already reads this
# variable. Set too late (even by the start of the next line after
# project()), that internal check silently falls back to a debug dynamic
# CRT (/MDd) no matter what CMAKE_BUILD_TYPE was requested; combined with
# -fsanitize=address (the "asan" preset) that fails outright, since
# AddressSanitizer's Windows runtime does not support the debug CRT
# ("invalid argument '-MDd' not allowed with '-fsanitize=address'") -
# discovered by actually running the asan preset, not by inspection.
# Fixing the ordering alone was not enough, though: CMake's internal
# try_compile() calls (including this ABI check) default to the "Debug"
# configuration regardless of the project's own CMAKE_BUILD_TYPE, so
# $<$<CONFIG:Debug>:Debug> still picked the debug CRT even after moving
# this above project(). The "asan" preset additionally sets
# CMAKE_TRY_COMPILE_CONFIGURATION to RelWithDebInfo (its own build type) in
# CMakePresets.json to close that gap.
# Setting this unconditionally (not gated on `if(MSVC)`, since MSVC is not
# yet defined before project()) is harmless on non-Windows, where the
# variable is simply ignored.
if(POLICY CMP0091)
  cmake_policy(SET CMP0091 NEW)
endif()
set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")

# clang-cl + AddressSanitizer runtime, linked explicitly (Windows "asan"
# preset only). Finding, again verified by actually running the preset, not
# assumed: clang-cl auto-injects the ASan runtime import libraries only when
# IT performs the final link itself. CMake's Ninja generator for clang-cl
# instead invokes lld-link.exe/link.exe DIRECTLY as the link step and passes
# "-fsanitize=address" straight through to it - a compiler flag the linker
# does not understand ("lld-link: warning: ignoring unknown argument
# '-fsanitize=address'"), so the ASan runtime never gets linked in and the
# build fails with undefined __asan_* symbols. This must be set before
# project() because CMake's own internal "does this compiler work" ABI
# check hits the exact same link step. CMAKE_CXX_COMPILER_ID is not known
# yet at this point (it is only set by enable_language, inside project()),
# so this matches on the CMAKE_CXX_COMPILER cache value's name instead -
# both it and CMAKE_CXX_FLAGS are already populated from the preset's
# cacheVariables before any CMakeLists.txt code runs.
if(CMAKE_CXX_COMPILER MATCHES "clang-cl" AND CMAKE_CXX_FLAGS MATCHES "-fsanitize=address")
  execute_process(
    COMMAND "${CMAKE_CXX_COMPILER}" -print-resource-dir
    OUTPUT_VARIABLE PS_CLANG_RESOURCE_DIR
    OUTPUT_STRIP_TRAILING_WHITESPACE
  )
  set(PS_CLANG_ASAN_LIB_DIR "${PS_CLANG_RESOURCE_DIR}/lib/windows")
  # Mirrors exactly what `clang-cl -fsanitize=address -MT ... -###` itself
  # generates for the static (/MT) CRT we use (CMAKE_MSVC_RUNTIME_LIBRARY
  # above) - both the plain lib reference and the /WHOLEARCHIVE: form, same
  # as clang-cl's own generated link line.
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /LIBPATH:\"${PS_CLANG_ASAN_LIB_DIR}\" /WHOLEARCHIVE:clang_rt.asan-x86_64.lib \"${PS_CLANG_ASAN_LIB_DIR}/clang_rt.asan-x86_64.lib\" /WHOLEARCHIVE:clang_rt.asan_cxx-x86_64.lib \"${PS_CLANG_ASAN_LIB_DIR}/clang_rt.asan_cxx-x86_64.lib\" /INFERASANLIBS:NO")
endif()

# /OPT:NOICF (Identical COMDAT Folding) is disabled for the "asan" preset
# via CMakePresets.json, not here - it belongs with the rest of that
# preset's linker flags. Documented here because the reason is non-obvious:
# without it, the linker was observed folding two byte-identical read-only
# globals from DIFFERENT translation units (an empty string literal in
# tests/unit/test_hello.cpp and one inside Catch2's own
# catch_console_colour.cpp) so close together that ASan's cross-TU global
# redzones did not separate them, producing a false-positive
# "global-buffer-overflow" crash the moment Catch2 printed a test case name
# - 100% inside third-party code, reproduced with the trivial placeholder
# test and absent on every other leg (including linux-asan-ubsan with the
# same Catch2 version), so not a physics_sim bug. /OPT:NOICF fixed it,
# confirmed by relinking and rerunning the test binary by hand before
# changing the preset. The preset also sets ASAN_OPTIONS=
# detect_odr_violation=0: without it Catch2's own static libs still trip an
# unrelated, known ODR-violation false positive on Windows (a wide- and a
# narrow-char "d" string literal registered as if they were the same
# global) even after /OPT:NOICF.

project(physics_sim LANGUAGES CXX)

# --------------------------------------------------------------------------
# Phase 0 ("Toolchain and skeleton"). This file wires up dependencies and
# empty placeholder targets only. Real simulation code (math, jobs, world,
# vehicle, aero, ...) starts in P1. See PLAN.md and the repo's CLAUDE.md.
# --------------------------------------------------------------------------

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(PS_BUILD_TESTS "Build the Catch2 unit test suite" ON)
option(PS_BUILD_GODOT_ADAPTER "Build the Godot GDExtension adapter (P2)" OFF)

# CMAKE_MSVC_RUNTIME_LIBRARY is set above, before project() - see the
# comment there. Repeated here only as a note: every target in the build -
# ours and Jolt's - ends up linking the same CRT this way. Jolt's own
# Build/CMakeLists.txt sets USE_STATIC_MSVC_RUNTIME_LIBRARY internally too,
# but only in its own directory scope, which does not propagate back up to
# targets added from our own CMakeLists; relying on that would risk a
# static/dynamic CRT mismatch at link time.

# FMA-contraction-off, applied globally (not just to our own targets) so
# that JoltPhysics itself - compiled by its own Build/CMakeLists.txt, which
# we do not own - is built with the same determinism guarantee (D7). Setting
# this before FetchContent_MakeAvailable(JoltPhysics) below means Jolt's own
# script inherits it as the starting value of CMAKE_CXX_FLAGS and appends to
# it, rather than us trying to patch Jolt's build after the fact.
#
# Finding (verified empirically, not assumed): clang-cl 19.1.5's own
# narrower argument parser does not recognise the plain clang spelling
# "-ffp-contract=off" - it is silently ignored ("unknown argument ignored in
# clang-cl"), which under -WX/-Werror becomes a hard error and, more
# importantly, when it is NOT an error just means the flag has no effect at
# all. The fix is the "/clang:" passthrough prefix, which forwards the flag
# to the underlying clang cc1 frontend unchanged: "/clang:-ffp-contract=off".
# This also means JoltPhysics' own Build/CMakeLists.txt has a gap: its
# CROSS_PLATFORM_DETERMINISTIC handling only sets /fp:precise for real MSVC
# (cl.exe) and -ffp-contract=off for non-Windows Clang/GCC, but sets nothing
# at all for clang-cl (CMAKE_CXX_COMPILER_ID "Clang" under the MSVC
# frontend) - so without this project-wide flag, Jolt built by clang-cl
# would silently keep FMA contraction on despite CROSS_PLATFORM_DETERMINISTIC.
if(MSVC)
  if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /clang:-ffp-contract=off")
  endif()
  # cl (CMAKE_CXX_COMPILER_ID "MSVC"): no flag needed, see ps_options below.
else()
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ffp-contract=off")
endif()

include(FetchContent)

# ===========================================================================
# Dependencies (PLAN.md section 2), each pinned to a release tag.
# ===========================================================================

# --- JoltPhysics (MIT) — rigid-body/collision backend, hidden behind our
# own IRigidBackend from P1 on (PLAN.md D2). Pinned to the latest stable
# release tag as of 2026-09-21.
set(DOUBLE_PRECISION ON CACHE BOOL "" FORCE)               # D2: needed for aircraft / large farm maps
set(CROSS_PLATFORM_DETERMINISTIC ON CACHE BOOL "" FORCE)   # D2/D7: also forces /fp:precise and -ffp-contract=off, and disables USE_FMADD, inside Jolt's own build
set(USE_AVX2 ON CACHE BOOL "" FORCE)                       # matches PLAN.md section 2 (already Jolt's default; explicit for documentation)
set(USE_FMADD OFF CACHE BOOL "" FORCE)                     # explicit; CROSS_PLATFORM_DETERMINISTIC already forces this off internally
set(USE_AVX512 OFF CACHE BOOL "" FORCE)                    # 5900X (Zen 3) has no AVX-512
set(ENABLE_ALL_WARNINGS OFF CACHE BOOL "" FORCE)           # third-party code: warnings-as-errors is our own targets' business only, not Jolt's (see ps_warnings below); Jolt's own /Wall /WX default is not something we want breaking CI on a newer compiler
set(INTERPROCEDURAL_OPTIMIZATION OFF CACHE BOOL "" FORCE)  # LTO is a P9 performance-pass concern; keep P0 builds fast and simple
set(ENABLE_INSTALL OFF CACHE BOOL "" FORCE)                # we vendor Jolt via FetchContent, never install it
set(TARGET_UNIT_TESTS OFF CACHE BOOL "" FORCE)
set(TARGET_HELLO_WORLD OFF CACHE BOOL "" FORCE)
set(TARGET_PERFORMANCE_TEST OFF CACHE BOOL "" FORCE)
set(TARGET_SAMPLES OFF CACHE BOOL "" FORCE)
set(TARGET_VIEWER OFF CACHE BOOL "" FORCE)
set(JPH_USE_DX12 OFF CACHE BOOL "" FORCE)        # GPU compute (cloth etc.) - unused by physics_sim, not a listed dependency
set(JPH_USE_VK OFF CACHE BOOL "" FORCE)          # ditto; also avoids a Vulkan SDK requirement this machine does not have
set(JPH_USE_MTL OFF CACHE BOOL "" FORCE)         # macOS only
set(JPH_USE_CPU_COMPUTE OFF CACHE BOOL "" FORCE) # CPU fallback for the same unused GPU compute path

FetchContent_Declare(
  JoltPhysics
  GIT_REPOSITORY https://github.com/jrouwe/JoltPhysics.git
  GIT_TAG        v5.6.0
  SOURCE_SUBDIR  Build
)
FetchContent_MakeAvailable(JoltPhysics)

# --- nlohmann/json (MIT) — scenario and vehicle definition files (D6).
FetchContent_Declare(
  nlohmann_json
  GIT_REPOSITORY https://github.com/nlohmann/json.git
  GIT_TAG        v3.12.0
)
FetchContent_MakeAvailable(nlohmann_json)

# --- Catch2 v3 (BSL-1.0) — tests and micro-benchmarks.
if(PS_BUILD_TESTS)
  FetchContent_Declare(
    Catch2
    GIT_REPOSITORY https://github.com/catchorg/Catch2.git
    GIT_TAG        v3.16.0
  )
  FetchContent_MakeAvailable(Catch2)
endif()

# --- godot-cpp — adapter only (P2). Only fetched when explicitly requested;
# default OFF in P0. No branch matching Godot 4.7 exists yet upstream (only
# up to "4.5" as of 2026-09-21); see CLAUDE.md for the deviation note.
if(PS_BUILD_GODOT_ADAPTER)
  FetchContent_Declare(
    godot-cpp
    GIT_REPOSITORY https://github.com/godotengine/godot-cpp.git
    GIT_TAG        4.5
  )
  FetchContent_MakeAvailable(godot-cpp)
endif()

# ===========================================================================
# Shared compile-option INTERFACE targets for OUR OWN targets only.
# Third-party targets (Jolt, Catch2, nlohmann_json, godot-cpp) manage their
# own flags and are never linked against these.
# ===========================================================================

# ps_warnings: warnings-as-errors for our own code only (PLAN.md section 2).
add_library(ps_warnings INTERFACE)
if(MSVC)
  target_compile_options(ps_warnings INTERFACE /W4 /WX)
else()
  target_compile_options(ps_warnings INTERFACE -Wall -Wextra -Wpedantic -Werror)
endif()

# ps_options: architecture and floating-point flags (PLAN.md section 2/D5).
# /arch:AVX2 /fp:precise, FMA contraction off. Determinism (D7) depends on
# never letting the compiler silently fuse a*b+c into an FMA instruction
# outside of code that asks for it explicitly.
#
# Compiler-by-compiler finding (documented here because the plan asked us to
# verify it, not just assume it):
#  - cl (MSVC):      /fp:precise is the default and NEVER contracts FMA on
#                     its own; contraction only happens under /fp:fast. There
#                     is no cl equivalent of -ffp-contract=off because none
#                     is needed - /fp:precise already is that setting.
#  - clang-cl/clang:  /fp:precise does not exist as a contraction switch the
#                     way it does for cl; clang defaults to
#                     -ffp-contract=fast, so it must be turned off
#                     explicitly with -ffp-contract=off (accepted directly
#                     by clang-cl, no /clang: prefix needed).
# Note: FMA-contraction-off is already applied to CMAKE_CXX_FLAGS above
# (project-wide, so it also reaches JoltPhysics' own compilation); it is not
# repeated here to avoid passing the /clang: passthrough flag twice.
add_library(ps_options INTERFACE)
if(MSVC)
  # /EHsc: standard C++ exception handling with proper stack unwinding.
  # Neither cl nor clang-cl enable this by default; without it, <chrono>/
  # <ostream> (pulled in by Catch2) trip MSVC warning C4530 ("C++ exception
  # handler used, but unwind semantics are not enabled"), which /WX (via
  # ps_warnings) then turns into a hard error - found by actually building
  # the msvc-release leg, not by inspection.
  target_compile_options(ps_options INTERFACE /arch:AVX2 /fp:precise /EHsc)
else()
  target_compile_options(ps_options INTERFACE -mavx2 -mfma)
endif()

# ===========================================================================
# Placeholder targets (PLAN.md section 4 layout).
# ===========================================================================

add_subdirectory(core)
add_subdirectory(tools/ps_run)

if(PS_BUILD_TESTS)
  include(CTest)
  enable_testing()
  add_subdirectory(tests/unit)
endif()

if(PS_BUILD_GODOT_ADAPTER)
  add_subdirectory(adapters/godot)
endif()
