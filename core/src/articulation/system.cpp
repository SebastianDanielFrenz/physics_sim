#include "system.h"

namespace ps::articulation {

ArticulationId System::create(const ArticulationDesc& desc) {
    const Model model = build_model(desc);
    instances_.push_back(std::make_unique<Instance>(model, desc));
    return ArticulationId{static_cast<id_index_t>(instances_.size() - 1), 1};
}

void System::clear_external_wrenches() {
    for (auto& inst : instances_) inst->clear_external_wrenches();
}

void System::accumulate_external_wrench(ArticulationLinkRef ref, const ps::Pose& frame_world_pose, const ps::Wrench& wrench) {
    Instance& inst = *instances_[ref.articulation_index];
    // link_index == -1 means "the root itself" as a frame target; the root
    // is body 0 only when floating (see model.h) - a fixed root has no
    // body to receive a wrench (it has zero DoF by construction), so that
    // case is a documented no-op rather than undefined behaviour.
    const int body_index = inst.body_index_for_link(ref.link_index);
    if (body_index < 0) return; // fixed root has no body to carry a wrench
    inst.accumulate_external_wrench(body_index, frame_world_pose, wrench);
}

void System::step_all(const InstanceConfig& cfg, real tick_dt) {
    for (auto& inst : instances_) inst->step_tick(cfg, tick_dt);
}

std::uint64_t System::fold_state_hash(std::uint64_t seed) const {
    std::uint64_t h = seed;
    for (const auto& inst : instances_) h = inst->fold_state_hash(h);
    return h;
}

} // namespace ps::articulation
