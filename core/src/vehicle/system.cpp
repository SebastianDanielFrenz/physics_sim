#include "system.h"

#include <algorithm>

namespace ps::vehicle {

VehicleId System::create(const VehicleDesc& desc, BodyId chassis, const io::SurfaceTable* surfaces) {
    instances_.push_back(std::make_unique<VehicleInstance>(desc, chassis, surfaces));
    return VehicleId{static_cast<id_index_t>(instances_.size() - 1), 1};
}

void System::step_all(IRigidBackend& backend, jobs::JobSystem& job_system, const ControlInputs& controls,
                       real tick_dt, real substep_rate_hz) {
    if (instances_.empty()) return;
    const std::size_t n = instances_.size();
    const int workers = std::max(1, job_system.worker_count());
    const std::size_t grain = std::max<std::size_t>(1, (n + static_cast<std::size_t>(workers) * 4 - 1) / (static_cast<std::size_t>(workers) * 4));

    job_system.parallel_for(n, grain, [this, &backend, &controls, tick_dt, substep_rate_hz](std::size_t begin, std::size_t end) {
        for (std::size_t i = begin; i < end; ++i) {
            instances_[i]->step_tick(backend, controls, tick_dt, substep_rate_hz);
        }
    });
}

std::uint64_t System::fold_state_hash(std::uint64_t seed) const {
    std::uint64_t h = seed;
    for (const auto& inst : instances_) h = inst->fold_state_hash(h);
    return h;
}

} // namespace ps::vehicle
