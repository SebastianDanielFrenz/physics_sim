// ps/jobs/debug_alloc.h implementation, plus the global operator new/delete
// overrides that make the hook actually observe allocations. See that
// header for the rationale (never abort; count instead, so a test can prove
// both "clean" and "caught" without crashing).
//
// These overrides apply process-wide to any binary linking ps_core (there is
// exactly one definition, here). That is intentional: it is the only way to
// see allocations made by any code path a job body might call into,
// including third-party code.

#include "ps/jobs/debug_alloc.h"

#include <atomic>
#include <cstdlib>
#include <new>

// ASan and TSan each install their own global operator new/delete as part
// of their runtime instrumentation; defining ours too is a hard link error
// (duplicate symbol), not just redundant, discovered by actually running
// the asan/tsan CI legs (they never got this far in earlier, smaller CI
// runs that failed on unrelated errors first). __has_feature is the
// standard portable way to ask Clang whether a given sanitizer is active
// for this translation unit; MSVC's /fsanitize=address does not go through
// this macro at all (different compiler, checked separately below).
#if defined(__has_feature)
#if __has_feature(address_sanitizer) || __has_feature(thread_sanitizer)
#define PS_DEBUG_ALLOC_HOOK_DISABLED 1
#endif
#endif
#if defined(__SANITIZE_ADDRESS__) || defined(__SANITIZE_THREAD__)
#define PS_DEBUG_ALLOC_HOOK_DISABLED 1
#endif

namespace ps::jobs::debug_alloc {
namespace {
std::atomic<bool> g_armed{false};
std::atomic<std::uint64_t> g_violations{0};
thread_local int g_job_depth = 0;

// Re-entrancy guard: the violation bookkeeping below (atomics only) does
// not allocate, but stay defensive against any future change here calling
// into something that does, which would otherwise recurse into operator new.
thread_local bool g_in_hook = false;

[[maybe_unused]] void note_allocation() {
    if (g_job_depth > 0 && g_armed.load(std::memory_order_relaxed) && !g_in_hook) {
        g_in_hook = true;
        g_violations.fetch_add(1, std::memory_order_relaxed);
        g_in_hook = false;
    }
}
} // namespace

void set_armed(bool armed) { g_armed.store(armed, std::memory_order_relaxed); }
bool is_armed() { return g_armed.load(std::memory_order_relaxed); }
std::uint64_t violation_count() { return g_violations.load(std::memory_order_relaxed); }
void reset() { g_violations.store(0, std::memory_order_relaxed); }

void enter_job() { ++g_job_depth; }
void exit_job() { --g_job_depth; }
bool in_job() { return g_job_depth > 0; }

#if defined(PS_DEBUG_ALLOC_HOOK_DISABLED)
bool hook_available() { return false; }
#else
bool hook_available() { return true; }
#endif

} // namespace ps::jobs::debug_alloc

#if !defined(PS_DEBUG_ALLOC_HOOK_DISABLED)

namespace {
void* alloc_impl(std::size_t size) {
    ps::jobs::debug_alloc::note_allocation();
    if (size == 0) size = 1;
    void* p = std::malloc(size);
    if (!p) throw std::bad_alloc();
    return p;
}

void* alloc_aligned_impl(std::size_t size, std::align_val_t align) {
    ps::jobs::debug_alloc::note_allocation();
    if (size == 0) size = 1;
    const std::size_t a = static_cast<std::size_t>(align) < sizeof(void*) ? sizeof(void*) : static_cast<std::size_t>(align);
#if defined(_MSC_VER)
    void* p = _aligned_malloc(size, a);
#else
    void* p = std::aligned_alloc(a, (size + a - 1) / a * a);
#endif
    if (!p) throw std::bad_alloc();
    return p;
}

void free_impl(void* p) { std::free(p); }
void free_aligned_impl(void* p) {
#if defined(_MSC_VER)
    _aligned_free(p);
#else
    std::free(p);
#endif
}
} // namespace

void* operator new(std::size_t size) { return alloc_impl(size); }
void* operator new[](std::size_t size) { return alloc_impl(size); }
void operator delete(void* p) noexcept { free_impl(p); }
void operator delete[](void* p) noexcept { free_impl(p); }
void operator delete(void* p, std::size_t) noexcept { free_impl(p); }
void operator delete[](void* p, std::size_t) noexcept { free_impl(p); }

void* operator new(std::size_t size, std::align_val_t align) { return alloc_aligned_impl(size, align); }
void* operator new[](std::size_t size, std::align_val_t align) { return alloc_aligned_impl(size, align); }
void operator delete(void* p, std::align_val_t) noexcept { free_aligned_impl(p); }
void operator delete[](void* p, std::align_val_t) noexcept { free_aligned_impl(p); }
void operator delete(void* p, std::size_t, std::align_val_t) noexcept { free_aligned_impl(p); }
void operator delete[](void* p, std::size_t, std::align_val_t) noexcept { free_aligned_impl(p); }

#endif // !PS_DEBUG_ALLOC_HOOK_DISABLED
