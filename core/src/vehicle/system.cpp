#include "system.h"

#include <algorithm>

namespace ps::vehicle {

VehicleId System::create(const VehicleDesc& desc, BodyId chassis, const io::SurfaceTable* surfaces,
                          const ControlChannelIndices& control_indices) {
    instances_.push_back(std::make_unique<VehicleInstance>(desc, chassis, surfaces, control_indices));
    aero_wakes_.resize(instances_.size());
    const auto vehicle_index = static_cast<std::int32_t>(instances_.size() - 1);
    if (vehicle_for_body_index_.size() <= chassis.index) vehicle_for_body_index_.resize(chassis.index + 1, -1);
    vehicle_for_body_index_[chassis.index] = vehicle_index;
    return VehicleId{static_cast<id_index_t>(instances_.size() - 1), 1};
}

void System::step_all(IRigidBackend& backend, jobs::JobSystem& job_system, const std::vector<real>& channel_values,
                       const VehicleTickEnv& env, real tick_dt, real substep_rate_hz) {
    if (instances_.empty()) return;
    const std::size_t n = instances_.size();
    for(std::size_t i=0;i<n;++i) {
        const auto body=instances_[i]->chassis();
        aero_wakes_[i]={body,backend.get_pose(body).position,backend.get_motion(body).linear,instances_[i]->aero_wake()};
    }
    VehicleTickEnv tick_env=env; tick_env.wakes=aero_wakes_;

    const int workers = std::max(1, job_system.worker_count());
    const std::size_t grain = std::max<std::size_t>(1, (n + static_cast<std::size_t>(workers) * 4 - 1) / (static_cast<std::size_t>(workers) * 4));

    job_system.parallel_for(n, grain, [this, &backend, &channel_values, &tick_env, tick_dt, substep_rate_hz](std::size_t begin, std::size_t end) {
        for (std::size_t i = begin; i < end; ++i) {
            instances_[i]->step_tick(backend, channel_values, tick_env, tick_dt, substep_rate_hz);
        }
    });
}

void System::apply_pending_engine_maps_swaps() {
    for (const auto& inst : instances_) inst->apply_pending_engine_maps_swap();
}

void System::note_body_wrench(BodyId body, const Wrench& w) {
    if (body.index >= vehicle_for_body_index_.size()) return;
    const std::int32_t vi = vehicle_for_body_index_[body.index];
    if (vi < 0) return;
    VehicleInstance& inst = *instances_[static_cast<std::size_t>(vi)];
    if (inst.chassis() == body) inst.add_external_wrench(w);
}

std::uint64_t System::fold_state_hash(std::uint64_t seed) const {
    std::uint64_t h = seed;
    for (const auto& inst : instances_) h = inst->fold_state_hash(h);
    return h;
}

} // namespace ps::vehicle
