# physics_sim

A standalone, real-time, deterministic C++ physics simulation library for
ground vehicles, articulated machinery and aircraft. See `PLAN.md` for the
full design and phased implementation plan, and `CLAUDE.md` for the
architecture map (directories, targets, presets, build/test/CI commands).

## Setup

New machine? Run:

```powershell
tools\setup_dev_env.ps1
```

It installs the Windows toolchain (VS 2022 Build Tools with clang-cl, CMake,
Ninja, Godot, Python, Blender) via winget and the WSL Ubuntu build toolchain
via apt.
It is idempotent - safe to re-run.

To only verify the toolchain without installing anything:

```powershell
tools\setup_dev_env.ps1 -CheckOnly
```

This prints the version of every required tool (Windows and WSL) and exits
non-zero if anything is missing.
