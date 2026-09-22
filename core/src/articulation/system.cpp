#include "system.h"

#include <algorithm>

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

void System::step_all(jobs::JobSystem& job_system, const InstanceConfig& cfg, real tick_dt, std::size_t grain) {
    if (instances_.empty()) return;
    const std::size_t n = instances_.size();
    const int workers = std::max(1, job_system.worker_count());
    // Same auto-grain formula as World::stage_force_elements (world.cpp):
    // spreads n items over roughly 4 chunks per worker, at least 1 item per
    // chunk, so small articulations batch together instead of one job each
    // (PLAN.md D8: "do not create jobs for less than ~20us of work").
    const std::size_t auto_grain = std::max<std::size_t>(1, (n + static_cast<std::size_t>(workers) * 4 - 1) / (static_cast<std::size_t>(workers) * 4));
    const std::size_t g = grain != 0 ? grain : auto_grain;

    job_system.parallel_for(n, g, [this, &cfg, tick_dt](std::size_t begin, std::size_t end) {
        // Scatter -> compute (PLAN.md D8): each iteration only ever reads
        // or writes instances_[i]'s own Instance object (its own
        // preallocated scratch, instance.h) - safe to run concurrently
        // with any other i.
        for (std::size_t i = begin; i < end; ++i) instances_[i]->step_tick(cfg, tick_dt);
    });
}

std::uint64_t System::fold_state_hash(std::uint64_t seed) const {
    std::uint64_t h = seed;
    for (const auto& inst : instances_) h = inst->fold_state_hash(h);
    return h;
}

} // namespace ps::articulation
