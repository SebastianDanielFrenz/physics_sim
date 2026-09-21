// ps/version.h — public version API.
//
// Placeholder for Phase 0 ("Toolchain and skeleton"). Proves that the
// core library builds, links against its dependencies (Jolt in particular)
// and is callable from another target. No simulation logic lives here yet.
//
// Public headers under core/include/ps must never expose a Jolt or Godot
// type (see PLAN.md D1, D2).

#pragma once

namespace ps {

// Returns the ps_core library version as "MAJOR.MINOR.PATCH".
const char* version_string();

// Touches the Jolt backend once (registers its default allocator) and
// reports whether it initialised correctly. This exists solely so a test
// can prove the Jolt dependency actually links and runs; it is not part of
// the eventual public API and carries no Jolt types across this header.
bool jolt_backend_self_check();

} // namespace ps
