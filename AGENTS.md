# Shared Godot runtime coordination

Read [RUNTIME_LOCKS.md](RUNTIME_LOCKS.md) before rebuilding or replacing `adapters/godot/demo/bin/libps_godot.dll`. Inspect `out/runtime_locks/godot_*.json` for running demo/probe ownership, model, heartbeat and expected close time. Verify the actual process; records may be stale and not every DLL loader creates one. Do not terminate another session's process without user authorization. Unknown metadata is explicit and must not be guessed.

The existing project architecture and build map is [CLAUDE.md](CLAUDE.md).

# Long sanitizer scheduling

Owner preference (2026-10-01): run extremely long sanitizer suites around
05:00 Europe/Berlin, rather than blocking interactive daytime work. Run focused,
change-specific checks during the day. Queue remaining long checks in
out/terrain_prefetch_overnight_pending.md for the Overnight physics sanitizers
thread automation, and report pending checks honestly; do not claim full CI
passed before it completes. An explicit later owner request takes precedence.
Coordinate CI ownership before starting an overnight run.
