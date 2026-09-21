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
