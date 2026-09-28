#include "system.h"

#include <algorithm>

namespace ps::vehicle {

VehicleId System::create(const VehicleDesc& desc, BodyId chassis, const io::SurfaceTable* surfaces,
                          const ControlChannelIndices& control_indices) {
    instances_.push_back(std::make_unique<VehicleInstance>(desc, chassis, surfaces, control_indices));
    const auto vehicle_index = static_cast<std::int32_t>(instances_.size() - 1);
    if (vehicle_for_body_index_.size() <= chassis.index) vehicle_for_body_index_.resize(chassis.index + 1, -1);
    vehicle_for_body_index_[chassis.index] = vehicle_index;
    return VehicleId{static_cast<id_index_t>(instances_.size() - 1), 1};
}

void System::step_all(IRigidBackend& backend, jobs::JobSystem& job_system, const std::vector<real>& channel_values,
                       const VehicleTickEnv& env, real tick_dt, real substep_rate_hz) {
    if (instances_.empty()) return;
    // PHYS-033 P4 I7c C16b (D9): apply any queued engine-maps hot swap
    // BEFORE this tick's per-vehicle jobs are dispatched below -
    // VehicleInstance::step_tick (the job body) is where Powertrain::
    // begin_tick's OWN pending-apply pattern (e.g. the fuel-tank precedent)
    // runs, but a maps swap additionally destroys the OLD maps object, and
    // debug_alloc's zero-alloc guarantee (tested by H6) only holds outside a
    // job - so this one queue-drain runs sequentially, here, on the thread
    // that calls step_all itself (never a job).
    for (const auto& inst : instances_) inst->apply_pending_engine_maps_swap();
    const std::size_t n = instances_.size();
    const int workers = std::max(1, job_system.worker_count());
    const std::size_t grain = std::max<std::size_t>(1, (n + static_cast<std::size_t>(workers) * 4 - 1) / (static_cast<std::size_t>(workers) * 4));

    job_system.parallel_for(n, grain, [this, &backend, &channel_values, &env, tick_dt, substep_rate_hz](std::size_t begin, std::size_t end) {
        for (std::size_t i = begin; i < end; ++i) {
            instances_[i]->step_tick(backend, channel_values, env, tick_dt, substep_rate_hz);
        }
    });
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
