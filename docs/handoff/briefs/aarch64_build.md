# Brief: native Linux ARM64 (aarch64) build support - physics_sim

You are a coding agent (Sonnet) on the C++20 physics engine S:\claude_code\physics_sim (Windows, CMake, Ninja,
clang-cl; WSL for Linux legs). You work in your own git worktree, created by the harness. Base it on master d53cf54 or
later. The coordinator (Claude Opus) reviews, runs CI and merges.

READ FIRST: C:\Users\sebas\AppData\Local\Temp\claude\S--claude-code-physics-sim\094bd4b1-17d1-4c89-9e37-5d1204e63823\scratchpad\briefs\physics_sim_context.md
(standing rules section only; the engine history is irrelevant here). Then CMakeLists.txt lines ~140-210 (global FP flags,
Jolt options) and ~330-370 (ps_options), core/include/ps/math/simd.h, core/include/ps/engine_cycle/step_team.h,
core/src/engine_cycle/step_team.cpp, CMakePresets.json (linux-* presets), tools/ci.ps1 (how the WSL lane runs).

## Why
racing_game (a separate repo that pins physics_sim) will run natively on the Steam Frame headset: Linux aarch64. The
owner approved it. Bit-identity x86 vs ARM is NOT required now (single player on the headset).

## Scope
1. `ps_options` (CMakeLists.txt ~363-365): `/arch:AVX2` and `-mavx2 -mfma` only when the TARGET is x86-64. The check
   must work natively and under a cross toolchain file / `--target=aarch64-linux-gnu` (CMAKE_SYSTEM_PROCESSOR from the
   toolchain file; consider also CMAKE_CXX_COMPILER_TARGET). Same for the global `/arch:AVX2` on the clang-cl path
   (~183) - it is Windows-only today; keep it as is unless it would misfire. Keep `-ffp-contract=off` on every
   non-MSVC target incl. aarch64 (clang defaults to contraction ON on aarch64). Note: `-mfma` on x86 stays as it is.
2. Jolt options (~199-205): USE_AVX2 / USE_SSE4_1 / USE_SSE4_2 / USE_F16C / USE_LZCNT / USE_TZCNT / USE_AVX etc. only
   set on x86; on aarch64 leave them unset/OFF so Jolt picks NEON itself. Check Jolt v5.6.0's own CMake
   (out/build/release/_deps/joltphysics-src/Build/CMakeLists.txt) for what it does with these on ARM, and say so.
   CROSS_PLATFORM_DETERMINISTIC and DOUBLE_PRECISION stay ON everywhere.
3. `core/include/ps/math/simd.h`: it already has a PS_SIMD_SCALAR path; make sure the AVX2 path is selected only on
   x86 with __AVX2__ and `<immintrin.h>` is never included on ARM. NEON is optional - scalar is fine; say which.
4. `step_team.h/.cpp`: `<immintrin.h>` / `_mm_pause()` only on x86; aarch64: `__asm__ __volatile__("yield")` (or
   `__builtin_arm_yield` / `__yield` per compiler) with std::this_thread::yield as the generic fallback. Grep the whole
   repo (core/, audio/, engine_tools/, tools/, tests/; NOT adapters/ - out of scope, but list any x86-only spot you see
   there) for other x86-only intrinsics/headers/asm (`__rdtsc`, `cpuid`, `_BitScan`, `__builtin_ia32`, `<x86intrin.h>`,
   SSE types) and fix them the same way.
5. **Cross-compile guard.** First check what WSL has: `clang --version`, whether `clang --target=aarch64-linux-gnu`
   finds aarch64 libstdc++ headers (packages like `libstdc++-<v>-dev-arm64-cross`, `g++-aarch64-linux-gnu`). Installs only
   via tools/setup_dev_env.ps1: if a package is needed, add it there (idempotent, with -CheckOnly support) and try running
   it; if it needs a password / sudo you cannot give, stop there and report the exact command for the owner.
   Then add a check that suits the repo: preferably a CMake toolchain file (e.g. cmake/toolchains/aarch64-linux-gnu-clang.cmake,
   sysroot optional via an env var for the later Steam Runtime 4 ARM64 SDK) plus a `linux-aarch64-check` configure
   preset (binary dir under ~/build/physics_sim/, never /mnt/s) that builds ps_core (and if it works ps_engine_cycle,
   ps_audio, ps_environment, ps_run) - compile and link the static libs; running needs qemu, optional: if qemu-user is
   available, running ps_run on a scenario and reporting the hash vs x86 is a bonus (report whether it matches). If no
   aarch64 C++ headers can be had, fall back to a script doing `clang++ --target=aarch64-linux-gnu -fsyntax-only` over
   core/src with the repo's flags. Wire it into tools/ci.ps1 as an extra WSL-lane step that reports SKIP (not FAIL) when
   the toolchain is absent - you MAY edit tools/ci.ps1 for this one step (keep the change small; `-Only` should be able
   to name it). Add a PathRules note only if needed.

## Hard constraints
- x86 byte-identical: every x86 compile line for our targets and Jolt must stay the same. Prove it: compare
  `compile_commands.json` (or `ninja -t commands`) for the release preset and the linux-release preset before/after -
  zero diff in flags. Then build release and run ps_run on every scenario in tools/ci.ps1
  $script:HashCheckScenarios at 1 and 10 workers: unchanged hashes (the coordinator runs the full CI).
- No behaviour change anywhere on x86; no change to any test, hash, golden.
- No edits to CLAUDE.md (propose lines), no vault writes, no merge/push.

## Rules
- Build lock: before ANY cmake/ctest/ninja run (Windows or WSL), create S:\claude_code\physics_sim\out\build.lock
  (mkdir is atomic; if it exists, retry every 30 s; another agent (P9) shares it); owner.txt = "sonnet aarch64". Remove
  it after each burst ONLY if owner.txt still holds your string. Keep bursts short.
- Configure the Windows release preset in your worktree with
  -DFETCHCONTENT_SOURCE_DIR_JOLTPHYSICS=S:/claude_code/physics_sim/out/build/release/_deps/joltphysics-src and the same
  for _CATCH2, _NLOHMANN_JSON and _MINIAUDIO (check the exact _deps names). VS 2022 env via vcvars64.bat like
  tools/ci.ps1. CMAKE_BUILD_PARALLEL_LEVEL=12. Build msvc-release once before the final commit.
- Commits: message in a file, `git commit -F file`, last line "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>".
- Never launch Godot. Read-only: data/, adapters/, the racing_game and geo2map repos.
- Progress file out/aarch64_progress.md in your worktree (git-ignored): what changed, the x86 flag-diff proof, hashes,
  the toolchain state, proposed CLAUDE.md lines, open points.
- Final message (short): branch + commits, what changed, x86 identity proof, aarch64 check result (and hash vs x86 if
  you could run it), anything the owner must install.
