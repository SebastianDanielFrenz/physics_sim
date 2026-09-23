// core/src/articulation/system.h — owns every ps::World articulation
// instance (internal; PLAN.md D10/P1.5a). Implements ps::ILinkStateSource
// so Frame can query link poses/velocities without World exposing any
// articulation internals through a public header beyond ArticulationDesc
// and the Frame seam (ps/world/frame.h).

#pragma once

#include "instance.h"
#include "ps/articulation/articulation_desc.h"
#include "ps/jobs/job_system.h"
#include "ps/world/frame.h"
#include "ps/world/ids.h"

#include <memory>
#include <vector>

namespace ps::articulation {

class System : public ps::ILinkStateSource {
public:
    ArticulationId create(const ArticulationDesc& desc);

    [[nodiscard]] Instance& at(ArticulationId id) { return *instances_[id.index]; }
    [[nodiscard]] const Instance& at(ArticulationId id) const { return *instances_[id.index]; }

    [[nodiscard]] ps::Frame link_frame(ArticulationId id, int link_index, ps::Pose local_offset = ps::Pose::identity()) const {
        return ps::Frame(*this, ArticulationLinkRef{id.index, link_index}, local_offset);
    }

    void clear_external_wrenches();
    void accumulate_external_wrench(ArticulationLinkRef ref, const ps::Pose& frame_world_pose, const ps::Wrench& wrench);
    // PLAN.md D8/P1.5a ("one job per articulation in the vehicle-island
    // stage"), delivered in PHYS-009 Step 1: instances_ is already in
    // ArticulationId order (ids are assigned sequentially by create(), one
    // per push_back below), so a plain parallel_for over it is "sorted by
    // ArticulationId" for free. Each job touches only its own Instance's
    // state - no shared mutable state between articulations, so no ordered
    // gather step is needed the way force_elements/stage_gather need one
    // (World::stage_vehicle_islands, world.cpp): stepping instance i can
    // run concurrently with, before, or after instance j with an
    // identical result, so the scatter+compute stage IS the whole
    // operation. `grain` == 0 selects an automatic chunk size (same
    // formula World::stage_force_elements uses) that keeps each job's
    // work comfortably above D8's ~20us-per-job floor by batching several
    // small articulations into one job rather than submitting one job per
    // instance regardless of size.
    void step_all(jobs::JobSystem& job_system, const InstanceConfig& cfg, real tick_dt, std::size_t grain = 0);

    [[nodiscard]] std::uint64_t fold_state_hash(std::uint64_t seed) const;

    [[nodiscard]] Pose link_world_pose(ArticulationLinkRef ref) const override {
        return instances_[ref.articulation_index]->link_world_pose(ref.link_index);
    }
    [[nodiscard]] Motion link_world_motion(ArticulationLinkRef ref) const override {
        return instances_[ref.articulation_index]->link_world_motion(ref.link_index);
    }

    [[nodiscard]] bool empty() const { return instances_.empty(); }
    // Number of live articulations - instances_ is indexed 0..count()-1 in
    // ArticulationId order (create()'s own comment above), so this lets a
    // caller enumerate every articulation by id without any other System
    // internals (PLAN.md P2 adapter seam: World::stage_snapshot() below).
    [[nodiscard]] std::size_t count() const { return instances_.size(); }

private:
    std::vector<std::unique_ptr<Instance>> instances_;
};

} // namespace ps::articulation
