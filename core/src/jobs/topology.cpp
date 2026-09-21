// Physical core count, best effort per platform. Used only for JobSystem's
// default worker count (PLAN.md D8: physical cores - 2). Not exposed as a
// public dependency of anything - callers that care use
// ps::jobs::physical_core_count() (ps/jobs/job_system.h), which never
// leaks a platform type.

#include "ps/jobs/job_system.h"

#include <algorithm>
#include <thread>

#if defined(_WIN32)
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#elif defined(__linux__)
#include <cstdio>
#include <set>
#include <utility>
#endif

namespace ps::jobs {

#if defined(_WIN32)

int physical_core_count() {
    DWORD len = 0;
    GetLogicalProcessorInformationEx(RelationProcessorCore, nullptr, &len);
    if (len == 0) {
        const unsigned hc = std::thread::hardware_concurrency();
        return std::max(1u, hc >= 2 ? hc / 2 : hc);
    }
    std::vector<std::byte> buf(len);
    auto* info = reinterpret_cast<SYSTEM_LOGICAL_PROCESSOR_INFORMATION_EX*>(buf.data());
    if (!GetLogicalProcessorInformationEx(RelationProcessorCore, info, &len)) {
        const unsigned hc = std::thread::hardware_concurrency();
        return std::max(1u, hc >= 2 ? hc / 2 : hc);
    }
    int cores = 0;
    std::byte* p = buf.data();
    std::byte* end = buf.data() + len;
    while (p < end) {
        auto* rec = reinterpret_cast<SYSTEM_LOGICAL_PROCESSOR_INFORMATION_EX*>(p);
        if (rec->Relationship == RelationProcessorCore) ++cores;
        p += rec->Size;
    }
    return std::max(1, cores);
}

#elif defined(__linux__)

int physical_core_count() {
    // Count distinct (physical_id, core_id) pairs from /proc/cpuinfo.
    std::FILE* f = std::fopen("/proc/cpuinfo", "r");
    if (!f) {
        const unsigned hc = std::thread::hardware_concurrency();
        return static_cast<int>(std::max(1u, hc >= 2 ? hc / 2 : hc));
    }
    std::set<std::pair<int, int>> cores;
    int physical_id = 0;
    int core_id = -1;
    char line[256];
    while (std::fgets(line, sizeof(line), f)) {
        int v;
        if (std::sscanf(line, "physical id : %d", &v) == 1) {
            physical_id = v;
        } else if (std::sscanf(line, "core id : %d", &v) == 1) {
            core_id = v;
            cores.insert({physical_id, core_id});
        }
    }
    std::fclose(f);
    if (cores.empty()) {
        const unsigned hc = std::thread::hardware_concurrency();
        return static_cast<int>(std::max(1u, hc >= 2 ? hc / 2 : hc));
    }
    return static_cast<int>(cores.size());
}

#else

int physical_core_count() {
    const unsigned hc = std::thread::hardware_concurrency();
    return static_cast<int>(std::max(1u, hc >= 2 ? hc / 2 : hc));
}

#endif

} // namespace ps::jobs
