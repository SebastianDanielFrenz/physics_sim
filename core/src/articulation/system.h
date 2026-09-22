// core/src/articulation/system.h — owns every ps::World articulation
// instance (internal; PLAN.md D10/P1.5a). Implements ps::ILinkStateSource
// so Frame can query link poses/velocities without World exposing any
// articulation internals through a public header beyond ArticulationDesc
// and the Frame seam (ps/world/frame.h).

#pragma once

#include "instance.h"
#include "ps/articulation/articulation_desc.h"
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
    void step_all(const InstanceConfig& cfg, real tick_dt);

    [[nodiscard]] std::uint64_t fold_state_hash(std::uint64_t seed) const;

    [[nodiscard]] Pose link_world_pose(ArticulationLinkRef ref) const override {
        return instances_[ref.articulation_index]->link_world_pose(ref.link_index);
    }
    [[nodiscard]] Motion link_world_motion(ArticulationLinkRef ref) const override {
        return instances_[ref.articulation_index]->link_world_motion(ref.link_index);
    }

    [[nodiscard]] bool empty() const { return instances_.empty(); }

private:
    std::vector<std::unique_ptr<Instance>> instances_;
};

} // namespace ps::articulation
