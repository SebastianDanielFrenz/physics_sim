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
# Target architecture (native Linux ARM64 / Steam Frame support). Every
# x86-only flag below (/arch:AVX2, -mavx2 -mfma, Jolt's USE_AVX2 & co.) is
# gated on PS_TARGET_X86 so an aarch64 target never sees them; on x86 the
# gate is TRUE and every compile line is byte-identical to before.
#
# The TARGET, not the host, decides: CMAKE_SYSTEM_PROCESSOR is the target when
# a toolchain file sets it (cmake/toolchains/aarch64-linux-gnu-clang.cmake),
# but a plain `--target=` given through CMAKE_CXX_COMPILER_TARGET or
# CMAKE_CXX_FLAGS does not change it, and MSVC/clang-cl report their target in
# CMAKE_CXX_COMPILER_ARCHITECTURE_ID. First hit wins, most specific first.
# Undetectable (empty) counts as x86 so nothing changes for a build that never
# set any of them.
# --------------------------------------------------------------------------
set(PS_TARGET_ARCH "")
if(CMAKE_CXX_COMPILER_ARCHITECTURE_ID)
  set(PS_TARGET_ARCH "${CMAKE_CXX_COMPILER_ARCHITECTURE_ID}")
elseif(CMAKE_CXX_COMPILER_TARGET)
  string(REGEX MATCH "^[^-]+" PS_TARGET_ARCH "${CMAKE_CXX_COMPILER_TARGET}")
elseif(CMAKE_CXX_FLAGS MATCHES "--target=([A-Za-z0-9_]+)")
  set(PS_TARGET_ARCH "${CMAKE_MATCH_1}")
elseif(CMAKE_SYSTEM_PROCESSOR)
  set(PS_TARGET_ARCH "${CMAKE_SYSTEM_PROCESSOR}")
endif()
string(TOLOWER "${PS_TARGET_ARCH}" PS_TARGET_ARCH)
if(PS_TARGET_ARCH STREQUAL "" OR PS_TARGET_ARCH MATCHES "^(x86_64|amd64|x64|x86|i[3-6]86)$")
  set(PS_TARGET_X86 TRUE)
else()
  set(PS_TARGET_X86 FALSE)
endif()
if(PS_TARGET_ARCH MATCHES "^(aarch64|arm64|arm64ec)$")
  set(PS_TARGET_AARCH64 TRUE)
else()
  set(PS_TARGET_AARCH64 FALSE)
endif()
message(STATUS "physics_sim target architecture: '${PS_TARGET_ARCH}' (x86: ${PS_TARGET_X86}, aarch64: ${PS_TARGET_AARCH64})")

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
# Set ON explicitly by every sanitizer preset (asan, linux-asan-ubsan,
# linux-tsan in CMakePresets.json). Wall-clock benches (tests/bench) still
# run fully on such a build but report their timing without asserting it.
option(PS_SANITIZER_BUILD "This build instruments code with a sanitizer (ASan/UBSan/TSan)" OFF)
if(NOT PS_SANITIZER_BUILD AND CMAKE_CXX_FLAGS MATCHES "-fsanitize=")
  message(WARNING "CMAKE_CXX_FLAGS contains -fsanitize= but PS_SANITIZER_BUILD is OFF - "
                  "wall-clock bench bounds will be asserted on an instrumented build. "
                  "Set PS_SANITIZER_BUILD=ON in the preset.")
endif()

# An optimised build type must actually carry an optimisation flag. No preset
# sets CMAKE_CXX_FLAGS_<CONFIG>; CMake seeds them ("/O2 /Ob2 /DNDEBUG",
# "-O3 -DNDEBUG", ...) the FIRST time a build dir is configured and never
# again. A cache that once got empty values (e.g. a manual
# -DCMAKE_CXX_FLAGS_RELEASE=) keeps them forever, and `cmake --preset` does
# not reset them. Found 2026-09-23: out/build/release had
# CMAKE_CXX_FLAGS and CMAKE_CXX_FLAGS_RELEASE both empty, so the Windows
# "release" CI leg built all of our own code (and Catch2) with no /O2 at all
# (Jolt sets its own), while a fresh configure of the same preset gives the
# normal defaults. Fail loudly rather than silently test an unoptimised
# "release".
if(CMAKE_BUILD_TYPE MATCHES "^(Release|RelWithDebInfo|MinSizeRel)$")
  string(TOUPPER "${CMAKE_BUILD_TYPE}" _ps_build_type_upper)
  set(_ps_effective_flags " ${CMAKE_CXX_FLAGS} ${CMAKE_CXX_FLAGS_${_ps_build_type_upper}} ")
  if(NOT _ps_effective_flags MATCHES " [-/]O[123sxz]")
    message(FATAL_ERROR
      "CMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE} but neither CMAKE_CXX_FLAGS ('${CMAKE_CXX_FLAGS}') nor "
      "CMAKE_CXX_FLAGS_${_ps_build_type_upper} ('${CMAKE_CXX_FLAGS_${_ps_build_type_upper}}') contains an "
      "optimisation flag - this build dir's cache is stale or was overridden. Reconfigure it from scratch: "
      "cmake --preset <name> --fresh")
  endif()
endif()

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
  else()
    # Real cl.exe (CMAKE_CXX_COMPILER_ID "MSVC", the msvc-release preset):
    # /arch:AVX2 alone (as set on ps_options, below - our own 3 targets
    # only) is not enough here. Jolt's Core.h auto-detects its own
    # JPH_USE_AVX/AVX2/SSE4.x/F16C/LZCNT feature macros purely from
    # compiler-predefined macros (__AVX2__ etc., the GCC/Clang convention -
    # confirmed by reading Jolt/Core/Core.h). clang-cl predefines these
    # under /arch:AVX2 (it is Clang underneath), so Jolt's own build
    # picks the right ABI there automatically; real cl.exe does not
    # predefine them from /arch:AVX2 at all. Left alone, that means Jolt's
    # *own* compiled .lib silently uses the narrower (pre-AVX) calling
    # convention for a few double-precision APIs (Math/MathTypes.h:
    # DVec3Arg is "const DVec3" - by value - only #ifdef JPH_USE_AVX, else
    # "const DVec3&"), while our own code - compiled with /arch:AVX2 via
    # ps_options - disagrees, since /arch:AVX2 alone was already enough to
    # flip OUR translation units' JPH_USE_AVX (this MSVC toolset, 14.44,
    # does predefine __AVX2__ under /arch:AVX2 after all - verified via
    # dumpbin, not assumed). Net effect without this line: a hard
    # `LNK2019 unresolved external symbol` for exactly the DVec3Arg-taking
    # overloads (BodyInterface::SetPositionAndRotation,
    # NarrowPhaseQuery::CastShape, *Shape::GetSubmergedVolume), reproduced
    # and diagnosed via dumpbin /symbols on both objects before writing
    # this fix. Setting this project-wide (not just on ps_options) means
    # Jolt's own CMakeLists.txt picks it up as part of its inherited
    # CMAKE_CXX_FLAGS baseline too, the same mechanism already used for
    # the FMA-contraction flag below.
    # x86 only (a Windows-on-ARM64 cl has no /arch:AVX2).
    if(PS_TARGET_X86)
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /arch:AVX2")
    endif()
  endif()
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
if(PS_TARGET_X86)
  set(USE_AVX2 ON CACHE BOOL "" FORCE)                     # matches PLAN.md section 2 (already Jolt's default; explicit for documentation)
  set(USE_FMADD OFF CACHE BOOL "" FORCE)                   # explicit; CROSS_PLATFORM_DETERMINISTIC already forces this off internally
  set(USE_AVX512 OFF CACHE BOOL "" FORCE)                  # 5900X (Zen 3) has no AVX-512
else()
  # Non-x86 target (aarch64): Jolt's Jolt.cmake adds no instruction-set flag
  # and defines no JPH_USE_* macro there (ARM64 branch "uses no special
  # commandline flags"; its Core.h then picks NEON from the compiler's
  # predefined __aarch64__/__ARM_NEON). Force every x86 feature OFF anyway so
  # a stale cache or a preset cannot ask for them. FMA contraction stays off
  # (CROSS_PLATFORM_DETERMINISTIC, -ffp-contract=off on every non-MSVC target).
  foreach(_ps_x86_opt USE_AVX512 USE_AVX2 USE_AVX USE_SSE4_1 USE_SSE4_2 USE_LZCNT USE_TZCNT USE_F16C USE_FMADD)
    set(${_ps_x86_opt} OFF CACHE BOOL "" FORCE)
  endforeach()
endif()
set(ENABLE_ALL_WARNINGS OFF CACHE BOOL "" FORCE)           # third-party code: warnings-as-errors is our own targets' business only, not Jolt's (see ps_warnings below); Jolt's own /Wall /WX default is not something we want breaking CI on a newer compiler
# Jolt defaults both of these OFF (its own CMakeLists.txt), trading a
# little codegen for RTTI/exception machinery it does not use internally.
# Our own code is compiled with RTTI and exceptions ON (neither is turned
# off anywhere in this file), and core/src/backend/jolt/jolt_job_system.h
# derives ps::jobs::JoltJobSystemBridge from JPH::JobSystemWithBarrier -
# with the two mismatched, that derived class's own RTTI needs "typeinfo
# for JPH::JobSystemWithBarrier", which a Jolt built with RTTI off never
# emits anywhere. This is a hard `ld.lld: undefined symbol` on the WSL/
# Linux legs (Itanium ABI requires a derived class's RTTI to reference
# every base's typeinfo, unconditionally); it was not caught by the
# Windows legs at all, where the COFF linker did not seem to enforce the
# same requirement here - found by actually running the Linux legs, not by
# inspection, and worth being explicit about since it is exactly the kind
# of Windows-vs-Linux difference D7 says to run down rather than relax.
set(CPP_RTTI_ENABLED ON CACHE BOOL "" FORCE)
set(CPP_EXCEPTIONS_ENABLED ON CACHE BOOL "" FORCE)
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

  # Catch2's own library must be built with C++ exceptions ON, or its
  # exception translator (what REQUIRE_THROWS_WITH/_MATCHES and "unexpected
  # exception" reporting use) is compiled as a stub that aborts with
  # "Attempted to translate active exception under
  # CATCH_CONFIG_DISABLE_EXCEPTIONS!". Catch2 decides per TU from
  # _CPPUNWIND/__cpp_exceptions. For clang-cl that means /EHsc must be on
  # its command line, and nothing guarantees it: /EHsc lives only in
  # CMake's default CMAKE_CXX_FLAGS, which any preset that sets
  # CMAKE_CXX_FLAGS itself replaces (debug: /Z7; asan: -fsanitize=address),
  # and ps_options' /EHsc reaches only this project's own targets. Added
  # PRIVATE to exactly the two Catch2 targets, so it cannot leak into any
  # other third-party code (Jolt strips /EHsc from its own flags anyway,
  # CPP_EXCEPTIONS_ENABLED OFF). Real cl.exe gets the same flag
  # harmlessly. Rationale: vault TOOL-019.
  if(MSVC)
    target_compile_options(Catch2 PRIVATE /EHsc)
    target_compile_options(Catch2WithMain PRIVATE /EHsc)
  endif()
  # clang-cl 19.1.5 + -fsanitize=address + optimisation (the asan preset is
  # RelWithDebInfo): ASan's STACK-variable instrumentation breaks the catch
  # parameter of a function that catches by reference or by value. The
  # MSVC EH runtime writes the exception object's address into the frame
  # slot its EH tables name, but ASan has moved that variable into its own
  # instrumented stack layout, so the handler reads garbage
  # (access-violation at 0xffffffffffffffff). Reproduced with a 15-line
  # two-file program and no project code: fine at /Od, fine without ASan,
  # fine with -mllvm -asan-stack=0, still broken with use-after-return
  # instrumentation off. It is decided purely on the CATCHING side.
  # Catch2's exception translator catches std::exception by reference, so
  # every REQUIRE_THROWS_WITH/_MATCHES would crash in the asan leg. Turn
  # stack instrumentation off for the two Catch2 targets only: they are
  # still ASan-built (heap/global checks and the STL annotation ABI match,
  # so they still link against ASan objects), and our own code keeps full
  # stack instrumentation. Linux ASan (Itanium EH) is unaffected. Vault
  # TOOL-019.
  if(MSVC AND CMAKE_CXX_COMPILER_ID STREQUAL "Clang" AND CMAKE_CXX_FLAGS MATCHES "-fsanitize=address")
    target_compile_options(Catch2 PRIVATE "SHELL:-mllvm -asan-stack=0")
    target_compile_options(Catch2WithMain PRIVATE "SHELL:-mllvm -asan-stack=0")
  endif()
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
  target_compile_options(ps_warnings INTERFACE -Wall -Wextra -Wpedantic -Werror
    # nlohmann/json v3.12.0's serializer.hpp has
    # `JSON_ASSERT(byte < utf8d.size())` where `byte` is a `std::uint8_t`
    # (max 255) against the fixed constant 400 - always true, flagged by
    # clang's -Wtautological-constant-out-of-range-compare (part of -Wall).
    # JSON_ASSERT is a no-op under NDEBUG (release builds never hit this),
    # so it only appears in a from-scratch DEBUG-preset build of any .cpp
    # that includes <nlohmann/json.hpp> (io/scenario.cpp, io/vehicle_io.cpp
    # etc., PLAN.md D6) - found doing exactly that for PLAN.md P3's vehicle
    # JSON loading. Third-party header code, not fixable on our side;
    # disabled for this one diagnostic only, everywhere ps_warnings is
    # linked (our own code stays under every other -Wall/-Wextra check).
    -Wno-tautological-constant-out-of-range-compare)
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
  if(PS_TARGET_X86)
    target_compile_options(ps_options INTERFACE /arch:AVX2 /fp:precise /EHsc)
  else()
    # Windows-on-ARM64: /arch:AVX2 is x86-only.
    target_compile_options(ps_options INTERFACE /fp:precise /EHsc)
  endif()
elseif(PS_TARGET_X86)
  target_compile_options(ps_options INTERFACE -mavx2 -mfma)
endif()
# Any non-MSVC, non-x86 target (aarch64-linux-gnu): ps_options adds nothing.
# FMA contraction is already off via the global -ffp-contract=off above
# (clang defaults to contraction ON for aarch64, so this flag matters there);
# NEON/FMA availability is the baseline of armv8-a, no -march flag needed.

# ===========================================================================
# Placeholder targets (PLAN.md section 4 layout).
# ===========================================================================

add_subdirectory(tools/common)
add_subdirectory(core)
add_subdirectory(audio)
add_subdirectory(engine_tools)
add_subdirectory(tools/ps_run)
add_subdirectory(tools/sweep)
add_subdirectory(tools/ps_engine_audio)

if(PS_BUILD_TESTS)
  include(CTest)
  enable_testing()
  add_subdirectory(tests/support)
  add_subdirectory(tests/unit)
  add_subdirectory(tests/validation)
  add_subdirectory(tests/determinism)
  add_subdirectory(tests/bench)
endif()

if(PS_BUILD_GODOT_ADAPTER)
  add_subdirectory(adapters/godot)
endif()
