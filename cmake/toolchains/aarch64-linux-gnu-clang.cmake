# CMake toolchain file: clang cross-compiling to native Linux ARM64
# (aarch64-linux-gnu), e.g. for the Steam Frame headset. Used by the
# `linux-aarch64-check` preset (CMakePresets.json) and by hand:
#
#   cmake -S . -B ~/build/physics_sim/aarch64 \
#         -DCMAKE_TOOLCHAIN_FILE=cmake/toolchains/aarch64-linux-gnu-clang.cmake \
#         -DCMAKE_BUILD_TYPE=Release -DPS_BUILD_TESTS=OFF -G Ninja
#
# Host requirements (WSL Ubuntu: `tools\setup_dev_env.ps1 -Aarch64Cross`):
# clang + lld, the arm64 libstdc++/glibc headers
# (libstdc++-<N>-dev-arm64-cross, libc6-dev-arm64-cross, N = the GCC major
# clang uses natively) and, only to RUN the result, qemu-user.
#
# Sysroot: by default clang finds /usr/aarch64-linux-gnu and
# /usr/lib/gcc-cross/aarch64-linux-gnu/<N> (the Debian/Ubuntu cross packages).
# For another sysroot (e.g. the Steam Runtime 4 ARM64 SDK) set the environment
# variable PS_AARCH64_SYSROOT to its root before configuring; it becomes
# CMAKE_SYSROOT and the emulator's -L path. Optional PS_AARCH64_GCC_TOOLCHAIN
# points clang at a GCC installation prefix (--gcc-toolchain=) when the
# sysroot's own libstdc++ is not discovered.
#
# Why these choices:
# - CMAKE_SYSTEM_PROCESSOR + the *_COMPILER_TARGET triple are both set: the top
#   level CMakeLists.txt derives PS_TARGET_X86 from them (the TARGET, not the
#   host, decides whether /arch:AVX2 / -mavx2 / Jolt USE_AVX2 apply), and
#   Jolt.cmake keys its "ARM64 uses no special flags" branch on
#   CMAKE_SYSTEM_PROCESSOR == aarch64.
# - -ffp-contract=off is added by the top level for every non-MSVC target;
#   clang defaults to contraction ON on aarch64, so it matters here.
# - lld: the linker the Linux presets use anyway (-fuse-ld=lld).

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

set(PS_AARCH64_TRIPLE aarch64-linux-gnu)
set(CMAKE_C_COMPILER clang)
set(CMAKE_CXX_COMPILER clang++)
set(CMAKE_C_COMPILER_TARGET ${PS_AARCH64_TRIPLE})
set(CMAKE_CXX_COMPILER_TARGET ${PS_AARCH64_TRIPLE})

if(DEFINED ENV{PS_AARCH64_SYSROOT} AND NOT "$ENV{PS_AARCH64_SYSROOT}" STREQUAL "")
  set(CMAKE_SYSROOT "$ENV{PS_AARCH64_SYSROOT}")
  set(PS_AARCH64_RUN_ROOT "$ENV{PS_AARCH64_SYSROOT}")
else()
  set(PS_AARCH64_RUN_ROOT "/usr/aarch64-linux-gnu")
endif()
if(DEFINED ENV{PS_AARCH64_GCC_TOOLCHAIN} AND NOT "$ENV{PS_AARCH64_GCC_TOOLCHAIN}" STREQUAL "")
  set(CMAKE_CXX_FLAGS_INIT "--gcc-toolchain=$ENV{PS_AARCH64_GCC_TOOLCHAIN}")
  set(CMAKE_C_FLAGS_INIT "--gcc-toolchain=$ENV{PS_AARCH64_GCC_TOOLCHAIN}")
endif()

set(CMAKE_EXE_LINKER_FLAGS_INIT "-fuse-ld=lld")
set(CMAKE_SHARED_LINKER_FLAGS_INIT "-fuse-ld=lld")
set(CMAKE_MODULE_LINKER_FLAGS_INIT "-fuse-ld=lld")

# Programs from the host, libraries/headers from the target sysroot only.
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

# Let ctest / add_test run target binaries under qemu-user when it is present
# (no binfmt registration needed). Absent qemu: cross builds still work, the
# results just cannot be run on this host.
find_program(PS_QEMU_AARCH64 qemu-aarch64)
if(PS_QEMU_AARCH64)
  set(CMAKE_CROSSCOMPILING_EMULATOR "${PS_QEMU_AARCH64};-L;${PS_AARCH64_RUN_ROOT}")
endif()
