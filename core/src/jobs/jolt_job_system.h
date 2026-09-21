// core/src/jobs/jolt_job_system.h — JPH::JobSystem bridge (PLAN.md D8).
//
// INTERNAL header (lives under core/src/, not core/include/ps): it includes
// Jolt headers and is only ever included by core/src/backend/jolt/*.cpp, so
// no Jolt type leaks into any public ps/ header (PLAN.md D2).
//
// Forwards every Jolt job onto ps::jobs::JobSystem, our one work-stealing
// pool, so Jolt's own parallel broadphase/narrowphase/island solving shares
// the exact same threads as force-element/vehicle-island jobs instead of
// running a second, competing pool.

#pragma once

#include "ps/jobs/job_system.h"

#include <Jolt/Jolt.h>
#include <Jolt/Core/FixedSizeFreeList.h>
#include <Jolt/Core/JobSystemWithBarrier.h>

#include <cstdio>

namespace ps::jobs {

// NOTE: this class derives (transitively, via JPH::JobSystemWithBarrier)
// from JPH::JobSystem, whose injected-class-name "JobSystem" would
// otherwise shadow ps::jobs::JobSystem for unqualified lookups inside this
// class body - every reference to OUR job system below is therefore
// spelled out as ps::jobs::JobSystem, never bare "JobSystem".
class JoltJobSystemBridge final : public JPH::JobSystemWithBarrier {
public:
    explicit JoltJobSystemBridge(ps::jobs::JobSystem& pool, JPH::uint max_jobs = 4096, JPH::uint max_barriers = 8)
        : JPH::JobSystemWithBarrier(max_barriers), pool_(pool) {
        jobs_.Init(max_jobs, max_jobs);
    }

    int GetMaxConcurrency() const override { return pool_.worker_count() + 1; }

    JobHandle CreateJob(const char* name, JPH::ColorArg color, const JobFunction& fn, JPH::uint32 num_dependencies) override {
        JPH::uint32 index;
        for (;;) {
            index = jobs_.ConstructObject(name, color, this, fn, num_dependencies);
            if (index != FreeList::cInvalidObjectIndex) break;
            // Pool exhausted (should not happen at P1 scenario sizes with
            // max_jobs above): help drain in-flight work and retry, rather
            // than allocating - mirrors JobSystemThreadPool's own approach.
            std::this_thread::yield();
        }
        Job* job = &jobs_.Get(index);
        JobHandle handle(job);
        if (num_dependencies == 0) QueueJob(job);
        return handle;
    }

protected:
    void QueueJob(Job* job) override {
        job->AddRef();
        pool_.submit([job]() {
            job->Execute();
            job->Release();
        });
    }

    void QueueJobs(Job** jobs, JPH::uint num_jobs) override {
        for (JPH::uint i = 0; i < num_jobs; ++i) QueueJob(jobs[i]);
    }

    void FreeJob(Job* job) override { jobs_.DestructObject(job); }

private:
    using FreeList = JPH::FixedSizeFreeList<Job>;
    FreeList jobs_;
    ps::jobs::JobSystem& pool_;
};

} // namespace ps::jobs
