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

// PHYS-009 Step 3 root-cause investigation only: an opt-in backtrace dump
// for the first few violations, gated by the PS_DEBUG_ALLOC_BACKTRACE env
// var so normal test runs (armed but not tracing) pay nothing extra. Linux
// only (execinfo.h); temporary diagnostic, not part of the shipped hook's
// public contract (debug_alloc.h declares none of this). Only meaningful
// when the hook itself is actually compiled in (never under ASan/TSan).
#if defined(__linux__) && !defined(PS_DEBUG_ALLOC_HOOK_DISABLED)
#include <cstdio>
#include <execinfo.h>
#include <unistd.h>
#define PS_DEBUG_ALLOC_HAVE_BACKTRACE 1
#define PS_DEBUG_ALLOC_BACKTRACE_LINUX 1
#elif defined(_WIN32) && !defined(PS_DEBUG_ALLOC_HOOK_DISABLED)
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#include <dbghelp.h>
#include <cstdio>
#pragma comment(lib, "dbghelp.lib")
#define PS_DEBUG_ALLOC_HAVE_BACKTRACE 1
#define PS_DEBUG_ALLOC_BACKTRACE_WINDOWS 1
#endif

namespace ps::jobs::debug_alloc {
namespace {
std::atomic<bool> g_armed{false};
std::atomic<std::uint64_t> g_violations{0};
std::atomic<std::uint64_t> g_external_violations{0};
std::atomic<std::uint64_t> g_own_violations{0}; // P4 I2 pre-step P1: violations NOT tagged external
std::atomic<std::uint64_t> g_frees_in_job{0}; // C16b review: own operator delete while armed + in a job
thread_local int g_job_depth = 0;

// Re-entrancy guard: the violation bookkeeping below (atomics only) does
// not allocate, but stay defensive against any future change here calling
// into something that does, which would otherwise recurse into operator new.
thread_local bool g_in_hook = false;

#if defined(PS_DEBUG_ALLOC_HAVE_BACKTRACE)
// Lazily read once per process (env var never changes mid-run); avoids a
// getenv() call - itself allocation-free, but still pointless overhead - on
// every single allocation in the overwhelmingly common case this is unset.
int backtrace_budget() {
    static const int budget = [] {
#if defined(_WIN32)
        // getenv() is flagged deprecated (-Werror) by the ucrt headers on
        // this toolchain; _dupenv_s is the suggested replacement.
        char* v = nullptr;
        std::size_t len = 0;
        if (_dupenv_s(&v, &len, "PS_DEBUG_ALLOC_BACKTRACE") != 0 || !v) return 0;
        const int result = (*v) ? std::atoi(v) : 0;
        std::free(v);
        return result;
#else
        const char* v = std::getenv("PS_DEBUG_ALLOC_BACKTRACE");
        return (v && *v) ? std::atoi(v) : 0;
#endif
    }();
    return budget;
}
std::atomic<int> g_backtrace_dumped{0};

void dump_backtrace(int violation_number) {
    char header[64];
    const int hlen = std::snprintf(header, sizeof(header), "--- debug_alloc violation #%d ---\n", violation_number);
#if defined(PS_DEBUG_ALLOC_BACKTRACE_LINUX)
    void* frames[32];
    const int n = ::backtrace(frames, 32);
    if (hlen > 0) (void)::write(STDERR_FILENO, header, static_cast<std::size_t>(hlen));
    ::backtrace_symbols_fd(frames, n, STDERR_FILENO);
#elif defined(PS_DEBUG_ALLOC_BACKTRACE_WINDOWS)
    // SymInitialize/SymFromAddr (dbghelp.h): resolves symbol names directly
    // from the process's own PDB (present on a Debug build) without an
    // external addr2line-equivalent tool. Not thread-safe against another
    // concurrent SymXxx call, but this whole path only runs under an
    // explicitly-armed, single-scenario diagnostic run (PS_DEBUG_ALLOC_BACKTRACE),
    // never in normal armed-but-not-tracing test runs.
    static bool sym_ready = [] {
        ::SymSetOptions(SYMOPT_LOAD_LINES | SYMOPT_UNDNAME | SYMOPT_DEFERRED_LOADS);
        if (::SymInitialize(::GetCurrentProcess(), nullptr, TRUE) != FALSE) return true;
        std::fprintf(stderr, "debug_alloc: SymInitialize failed, GetLastError=%lu\n", ::GetLastError());
        return false;
    }();
    if (hlen > 0) std::fputs(header, stderr);
    void* frames[32];
    const USHORT n = ::CaptureStackBackTrace(0, 32, frames, nullptr);
    if (sym_ready) {
        alignas(SYMBOL_INFO) char sym_buf[sizeof(SYMBOL_INFO) + 256];
        auto* sym = reinterpret_cast<SYMBOL_INFO*>(sym_buf);
        sym->SizeOfStruct = sizeof(SYMBOL_INFO);
        sym->MaxNameLen = 256;
        for (USHORT i = 0; i < n; ++i) {
            const DWORD64 addr = reinterpret_cast<DWORD64>(frames[i]);
            DWORD64 displacement = 0;
            const char* name = "?";
            DWORD last_err = 0;
            if (::SymFromAddr(::GetCurrentProcess(), addr, &displacement, sym)) {
                name = sym->Name;
            } else {
                last_err = ::GetLastError();
            }
            DWORD line_disp = 0;
            IMAGEHLP_LINE64 line{};
            line.SizeOfStruct = sizeof(line);
            const bool have_line = ::SymGetLineFromAddr64(::GetCurrentProcess(), addr, &line_disp, &line) != FALSE;
            if (have_line) {
                std::fprintf(stderr, "  %s (%s:%lu)\n", name, line.FileName, line.LineNumber);
            } else if (last_err != 0) {
                static const DWORD64 base = reinterpret_cast<DWORD64>(::GetModuleHandleW(nullptr));
                std::fprintf(stderr, "  0x%p rva=0x%llx ?  [SymFromAddr err=%lu]\n", frames[i],
                              static_cast<unsigned long long>(addr - base), last_err);
            } else {
                std::fprintf(stderr, "  %s (+0x%llx)\n", name, static_cast<unsigned long long>(displacement));
            }
        }
    } else {
        for (USHORT i = 0; i < n; ++i) std::fprintf(stderr, "  0x%p\n", frames[i]);
    }
#endif
}
#endif

// Shared by note_allocation() (our own global operator new/delete overrides,
// below) and the public note_external_allocation() (PHYS-033 I1b D-D: a
// third-party allocator routed around operator new entirely, e.g. Jolt's own
// JPH::Allocate/Reallocate/AlignedAllocate once jolt_backend.cpp's
// JoltGlobalRegistration wires them through this - see debug_alloc.h).
// `external` also bumps g_external_violations, for attribution and the T3
// negative control that proves the Jolt route is actually wired.
void note_violation(bool external) {
    if (g_job_depth > 0 && g_armed.load(std::memory_order_relaxed) && !g_in_hook) {
        g_in_hook = true;
        g_violations.fetch_add(1, std::memory_order_relaxed);
        if (external) g_external_violations.fetch_add(1, std::memory_order_relaxed);
        else g_own_violations.fetch_add(1, std::memory_order_relaxed);
#if defined(PS_DEBUG_ALLOC_HAVE_BACKTRACE)
        const int budget = backtrace_budget();
        if (budget > 0) {
            const int number = g_backtrace_dumped.fetch_add(1, std::memory_order_relaxed) + 1;
            if (number <= budget) dump_backtrace(number);
        }
#endif
        g_in_hook = false;
    }
}

[[maybe_unused]] void note_allocation() { note_violation(/*external=*/false); }

// PHYS-033 P4 I7c C16b review: a free through our own operator delete while
// armed inside a job. Counted separately from every allocation counter (so
// no existing zero-alloc assertion changes meaning); a null delete is not a
// free.
[[maybe_unused]] void note_free(const void* p) {
    if (p != nullptr && g_job_depth > 0 && g_armed.load(std::memory_order_relaxed) && !g_in_hook) {
        g_frees_in_job.fetch_add(1, std::memory_order_relaxed);
    }
}
} // namespace

void set_armed(bool armed) { g_armed.store(armed, std::memory_order_relaxed); }
bool is_armed() { return g_armed.load(std::memory_order_relaxed); }
std::uint64_t violation_count() { return g_violations.load(std::memory_order_relaxed); }
std::uint64_t external_violation_count() { return g_external_violations.load(std::memory_order_relaxed); }
std::uint64_t own_violation_count() { return g_own_violations.load(std::memory_order_relaxed); }
void reset() {
    g_violations.store(0, std::memory_order_relaxed);
    g_external_violations.store(0, std::memory_order_relaxed);
    g_own_violations.store(0, std::memory_order_relaxed);
    g_frees_in_job.store(0, std::memory_order_relaxed);
}
std::uint64_t own_free_in_job_count() { return g_frees_in_job.load(std::memory_order_relaxed); }
void note_external_allocation() { note_violation(/*external=*/true); }

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

void free_impl(void* p) {
    ps::jobs::debug_alloc::note_free(p);
    std::free(p);
}
void free_aligned_impl(void* p) {
    ps::jobs::debug_alloc::note_free(p);
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
