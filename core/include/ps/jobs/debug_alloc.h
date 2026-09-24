// ps/jobs/debug_alloc.h — detects heap allocation performed from inside a
// job body (PLAN.md D8: "no allocation inside jobs on the hot path").
//
// JobSystem marks a thread_local flag for the duration of every job body it
// runs (see job_system.cpp); global operator new/delete (debug_alloc.cpp)
// checks that flag and, when armed, counts a violation instead of crashing
// the process - this lets a test both (a) run real workloads and assert
// zero violations, and (b) deliberately allocate inside a job and assert
// the hook actually caught it, in the same process.

#pragma once

#include <cstdint>

namespace ps::jobs::debug_alloc {

// Enables/disables violation counting. Disarmed by default so ordinary
// allocation (startup, test harness, non-job code) is never penalised.
void set_armed(bool armed);
bool is_armed();

std::uint64_t violation_count();
void reset();

// PHYS-033 I1b (D-D): for a third-party allocator routed AROUND the global
// operator new/delete this file overrides below (Jolt's own JPH::Allocate/
// Reallocate/AlignedAllocate, wired up from jolt_backend.cpp's
// JoltGlobalRegistration) - counted under the SAME armed/in-job/re-entrancy
// rules as note_allocation() above, and folded into violation_count() too
// (so every EXISTING zero-alloc test now also sees a Jolt allocation it
// could not see before), plus this separate counter for attribution (the T3
// negative control proves the hook is actually wired, independent of
// operator new).
void note_external_allocation();
std::uint64_t external_violation_count();

// PHYS-033 P4 I2 pre-step P1: violation_count() folds together OUR code's
// own allocations (operator new, via note_allocation()) and Jolt-internal
// ones (note_external_allocation()) - I1b's own test_vehicle_parallel_alloc.cpp
// had to mark its whole strict guarantee [!mayfail] once the external route
// was wired up, because Jolt's PhysicsSystem::Update itself allocates
// (QuadTree::UpdatePrepare, ~49 per 100 ticks) for reasons outside this
// codebase's control (PLAN.md P9). That blinded the test to regressions in
// OUR OWN code for the whole vehicle job.
//
// own_violation_count() counts ONLY the non-external violations (i.e. hits
// through this file's own operator new/delete overrides) - a test can
// therefore assert own_violation_count() == 0 strictly (gating our code)
// while still reporting external_violation_count() (Jolt's) ungated. The
// two counters are independent tallies (not violation_count() minus
// external_violation_count()) so each stays meaningful even if one is
// reset independently.
std::uint64_t own_violation_count();

// False under an ASan/TSan build: those sanitizers install their own global
// operator new/delete, so this hook's own overrides (debug_alloc.cpp) are
// compiled out there to avoid a duplicate-symbol link error, and armed()
// becomes a harmless no-op that never counts a violation. Callers (tests)
// should skip assertions that depend on the hook actually seeing an
// allocation when this is false, rather than get a misleading pass/fail.
bool hook_available();

// Called by JobSystem around every job body invocation.
void enter_job();
void exit_job();
bool in_job();

} // namespace ps::jobs::debug_alloc
