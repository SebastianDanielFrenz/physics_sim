// core/src/vehicle/system.h — owns every ps::World vehicle instance
// (internal; PLAN.md 5.1/P3). Mirrors core/src/articulation/system.h's
// pattern for the same reason: vehicle wheel/suspension/tyre state is
// integrated at substep rate exactly like an articulation instance, and
// IForceElement::evaluate is const/stateless by contract (D8) so it cannot
// own that state (out/handover_p3.md decision 3).

#pragma once

#include "vehicle_instance.h"

#include "ps/io/surface_table.h"
#include "ps/jobs/job_system.h"
#include "ps/vehicle/vehicle_desc.h"
#include "ps/world/ids.h"

#include <cstdint>
#include <memory>
#include <vector>

namespace ps::vehicle {

class System {
public:
    VehicleId create(const VehicleDesc& desc, BodyId chassis, const io::SurfaceTable* surfaces);

    // One job per vehicle (PLAN.md D8), safe unparallelised-vs-shared-state
    // because each vehicle owns a distinct chassis body - no two jobs ever
    // call IRigidBackend::apply_wrench on the same body (unlike
    // articulation::System::step_all's Compliant-contact case, which needs
    // a sequential pass first for exactly that reason).
    void step_all(IRigidBackend& backend, jobs::JobSystem& job_system, const ControlInputs& controls,
                  real tick_dt, real substep_rate_hz);

    [[nodiscard]] bool empty() const { return instances_.empty(); }
    [[nodiscard]] std::size_t count() const { return instances_.size(); }
    [[nodiscard]] VehicleInstance& at(VehicleId id) { return *instances_[id.index]; }
    [[nodiscard]] const VehicleInstance& at(VehicleId id) const { return *instances_[id.index]; }

    // PLAN.md D7: folds every wheel's compression/spin/relaxed-slip state
    // into World::state_hash(), same reasoning as
    // articulation::System::fold_state_hash - the chassis body's own pose/
    // motion is already covered by IRigidBackend::compute_state_hash, but a
    // wheel's internal state (not otherwise observable) must be covered
    // too, or a worker-count/scheduling difference that only perturbed it
    // (with no chassis-visible effect yet) would go undetected.
    [[nodiscard]] std::uint64_t fold_state_hash(std::uint64_t seed) const;

private:
    std::vector<std::unique_ptr<VehicleInstance>> instances_;
};

} // namespace ps::vehicle
