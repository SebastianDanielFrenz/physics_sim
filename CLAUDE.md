# physics_sim — architecture map

Pure map of what exists and how to build/test/run it. Rationale and design
decisions live in the knowledge vault (`S:\claude_knowledge_base`), not here.
Full design/phase plan: `PLAN.md`.

## Layout

```
physics_sim/
  CMakeLists.txt, CMakePresets.json   top-level build config
  README.md                           setup pointer
  CLAUDE.md                           this file
  PLAN.md                             design and phase plan
  tools/
    setup_dev_env.ps1                 installs/verifies the dev toolchain (-CheckOnly to only verify)
    ci.ps1                            runs every CI leg (see below)
    ps_run/                           headless scenario runner (CLI -> CSV); target ps_run
    sweep/                            (empty until P1) fan scenarios out over local processes
    plots/                            (empty until P1) python telemetry plotting, dev only
  core/
    include/ps/                       public API headers (no Jolt or Godot types)
      version.h                       ps::version_string(), ps::jolt_backend_self_check()
    src/                              implementation; target ps_core (STATIC)
      version.cpp
      jolt_proof.cpp                  proves ps_core links and runs JoltPhysics
      math/ jobs/ world/ backend/ backend/jolt/ force/ vehicle/ drivetrain/
      aero/ machine/ terrain/ io/     empty (.gitkeep) until P1+
  tests/
    unit/                             Catch2 target ps_unit_tests; test_hello.cpp
    validation/ determinism/ bench/   empty (.gitkeep) until P1+
  data/
    vehicles/ tyres/ aircraft/ scenarios/   empty (.gitkeep) until P3+
  adapters/
    godot/                            empty (.gitkeep) until P2; built only when PS_BUILD_GODOT_ADAPTER=ON
```

## Targets

- `ps_core` (STATIC library): public headers in `core/include/ps`, sources in
  `core/src`. Links JoltPhysics `PRIVATE`. No Jolt or Godot type appears in
  any public header.
- `ps_run` (executable): `tools/ps_run/main.cpp`. Links `ps_core`.
- `ps_unit_tests` (executable): `tests/unit`. Links `ps_core` and
  `Catch2::Catch2WithMain`. Registered with CTest via `catch_discover_tests`.
- `ps_warnings` (INTERFACE): warnings-as-errors, linked into `ps_core`,
  `ps_run`, `ps_unit_tests` only - never into third-party targets.
- `ps_options` (INTERFACE): `/arch:AVX2` / `-mavx2 -mfma`, `/fp:precise`,
  `/EHsc` (MSVC family); same three targets only.
- `adapters/godot` (P2, not yet built): GDExtension adapter, gated by
  `PS_BUILD_GODOT_ADAPTER` (default `OFF`).

## CMake options

- `PS_BUILD_TESTS` (default `ON`): build `ps_unit_tests` and fetch Catch2.
- `PS_BUILD_GODOT_ADAPTER` (default `OFF`): fetch godot-cpp and build
  `adapters/godot`.

## Dependencies (FetchContent, pinned)

| Dependency   | Pin      | Notes |
|--------------|----------|-------|
| JoltPhysics  | `v5.6.0` | `DOUBLE_PRECISION ON`, `CROSS_PLATFORM_DETERMINISTIC ON`, `USE_AVX2 ON`, `USE_FMADD OFF`, GPU-compute options OFF, sample/test/viewer targets OFF |
| Catch2       | `v3.16.0`| only when `PS_BUILD_TESTS=ON` |
| nlohmann/json| `v3.12.0`| |
| godot-cpp    | branch `4.5` | only when `PS_BUILD_GODOT_ADAPTER=ON` |

## CMake presets

Windows (Ninja; run from a VS 2022 Build Tools developer environment -
`tools/ci.ps1` enters it automatically via `vcvars64.bat`):

| Preset | Compiler | Build type |
|---|---|---|
| `debug` | clang-cl | Debug |
| `release` | clang-cl | Release |
| `relwithdebinfo` | clang-cl | RelWithDebInfo |
| `asan` | clang-cl | RelWithDebInfo + AddressSanitizer |
| `msvc-debug` | cl | Debug |
| `msvc-release` | cl | Release |

Linux/WSL (Ninja, clang + lld; binary dir `~/build/physics_sim/<preset>`,
never under `/mnt/s`):

| Preset | Build type |
|---|---|
| `linux-release` | Release |
| `linux-asan-ubsan` | Debug + AddressSanitizer + UndefinedBehaviorSanitizer |
| `linux-tsan` | Debug + ThreadSanitizer |

## Building, testing, running

```powershell
# from a VS 2022 developer shell (tools\ci.ps1 sets one up automatically)
cmake --preset debug
cmake --build --preset debug
ctest --preset debug --output-on-failure
.\out\build\debug\tools\ps_run\ps_run.exe
```

```bash
# inside WSL
cd /mnt/s/claude_code/physics_sim
cmake --preset linux-release
cmake --build --preset linux-release
ctest --preset linux-release --output-on-failure
```

## CI

```powershell
tools\ci.ps1                      # every leg
tools\ci.ps1 -SkipWsl             # Windows legs only
tools\ci.ps1 -SkipWindows         # WSL legs only
tools\ci.ps1 -Only debug,asan     # named legs only (matches preset names, plus msvc-release)
```

Legs: clang-cl `debug`, `release`, `asan`; cl `msvc-release`; a grep that
fails the run if anything under `core/` includes a Godot/GDExtension header;
WSL `linux-release`, `linux-asan-ubsan`, `linux-tsan`. Prints a per-leg
PASS/FAIL summary; exits non-zero on any failure.

## Dev environment setup

```powershell
tools\setup_dev_env.ps1             # install everything (idempotent)
tools\setup_dev_env.ps1 -CheckOnly  # verify only, prints versions, exits non-zero if anything is missing
```
