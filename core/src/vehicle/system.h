// core/src/vehicle/system.h — owns every ps::World vehicle instance
// (internal; PLAN.md 5.1/P3). Mirrors core/src/articulation/system.h's
// pattern for the same reason: vehicle wheel/suspension/tyre state is
// integrated at substep rate exactly like an articulation instance, and
// IForceElement::evaluate is const/stateless by contract (D8) so it cannot
// own that state (out/handover_p3.md decision 3).

#pragma once

#include "vehicle_instance.h"

#include "ps/environment/vehicle_air.h"
#include "ps/io/surface_table.h"
#include "ps/jobs/job_system.h"
#include "ps/machine/coupling.h"
#include "ps/vehicle/vehicle_desc.h"
#include "ps/world/ids.h"

#include <cstdint>
#include <memory>
#include <span>
#include <vector>

namespace ps::vehicle {

class System {
public:
    VehicleId create(const VehicleDesc& desc, BodyId chassis, const io::SurfaceTable* surfaces,
                      const ControlChannelIndices& control_indices);

    // One job per vehicle (PLAN.md D8), safe unparallelised-vs-shared-state
    // because each vehicle owns a distinct chassis body - no two jobs ever
    // call IRigidBackend::apply_wrench on the same body (unlike
    // articulation::System::step_all's Compliant-contact case, which needs
    // a sequential pass first for exactly that reason).
    //
    // PLAN.md P4 I0 (PHYS-033 section 8, Q4): `channel_values` is World's
    // flat control_channel_values_ array; each VehicleInstance reads it at
    // its own pre-resolved indices (ControlChannelIndices, set at create())
    // - no string lookup and no per-vehicle vector to build happens here.
    // PHYS-033 I1 W1: `env` (gravity) is forwarded to every instance's
    // shadow chassis unchanged.
    //
    // Weather plan W4: `air` (empty = none) is one environment::VehicleAir per
    // vehicle, in vehicle order, filled by World::stage_freeze_tick_start_state;
    // an entry with valid == true replaces that vehicle's ambient sample, air
    // density and wind in `env` (the others keep the World-wide values).
    void step_all(IRigidBackend& backend, jobs::JobSystem& job_system, const std::vector<real>& channel_values,
                  const VehicleTickEnv& env, real tick_dt, real substep_rate_hz,
                  std::span<const environment::VehicleAir> air = {});

    // Attachment spike A0: couples two vehicles at their sockets. Setup time. Vehicles joined by couplings form an
    // island: one job that steps them substep by substep, the couplings solved in id order (one pass, exact for one
    // coupling, Gauss-Seidel along a chain). A vehicle outside any coupling takes the plain step_tick path, and a
    // World with no coupling never reaches any of this.
    HitchId create_coupling(const machine::CouplingDesc& desc);
    [[nodiscard]] bool contains(HitchId id) const { return id.generation == 1 && id.index < couplings_.size(); }
    [[nodiscard]] const machine::CouplingState& coupling_state(HitchId id) const { return couplings_[id.index].state; }

    // Called by World right before the backend steps, sequential: see VehicleInstance::align_backend_with_shadow.
    // Nothing happens to a vehicle outside a coupling.
    void pre_backend_step(IRigidBackend& backend);

    [[nodiscard]] bool empty() const { return instances_.empty(); }
    // PHYS-033 P4 I7c C16b (D9): applies every vehicle's queued engine-maps
    // hot swap and drops the old maps. Called by World::step BEFORE the tick
    // pipeline runs - TaskGraph::run submits every stage, the sequential
    // ones included, as a job (task_graph.cpp), so nothing inside step_all is
    // "outside a job". C16b review fix: this used to run at the top of
    // step_all, i.e. inside the vehicle_islands_substep job.
    void apply_pending_engine_maps_swaps();
    [[nodiscard]] std::size_t count() const { return instances_.size(); }
    // True iff id names a vehicle this System created. Vehicles are never
    // destroyed, so every issued id has generation 1 (create()).
    [[nodiscard]] bool contains(VehicleId id) const { return id.generation == 1 && id.index < instances_.size(); }
    [[nodiscard]] VehicleInstance& at(VehicleId id) { return *instances_[id.index]; }
    [[nodiscard]] const VehicleInstance& at(VehicleId id) const { return *instances_[id.index]; }

    // PHYS-033 I1 W1 (section 3.4): called from World::stage_gather (the
    // `else` branch, i.e. a force element attached to a plain body frame,
    // not an articulation link), sequential in fixed element order - never
    // concurrently for the same body, so no lock is needed here. Looks
    // `body` up in vehicle_for_body_index_ (built in create(), grown to
    // body.index + 1) and forwards to that vehicle's
    // VehicleInstance::add_external_wrench if it is in fact this vehicle's
    // own chassis (generation-checked) - a no-op for any body that is not a
    // vehicle chassis.
    void note_body_wrench(BodyId body, const Wrench& w);

    // PLAN.md D7: folds every wheel's compression/spin/relaxed-slip state
    // into World::state_hash(), same reasoning as
    // articulation::System::fold_state_hash - the chassis body's own pose/
    // motion is already covered by IRigidBackend::compute_state_hash, but a
    // wheel's internal state (not otherwise observable) must be covered
    // too, or a worker-count/scheduling difference that only perturbed it
    // (with no chassis-visible effect yet) would go undetected.
    [[nodiscard]] std::uint64_t fold_state_hash(std::uint64_t seed) const;

private:
    struct Coupling {
        machine::CouplingDesc desc;
        std::size_t towing = 0, towed = 0; // indices into instances_
        machine::CouplingState state;
    };
    // Sorted by lowest vehicle index; vehicles and couplings ascending. Built at setup, read only during the tick.
    struct Island {
        std::vector<std::size_t> vehicles;
        std::vector<std::size_t> couplings;
    };

    void rebuild_islands();
    void step_island(Island& island, IRigidBackend& backend, const std::vector<real>& channel_values, const VehicleTickEnv& env,
                     real tick_dt, real substep_rate_hz, std::span<const environment::VehicleAir> air);
    void solve_coupling(Coupling& c, real h);

    std::vector<std::unique_ptr<VehicleInstance>> instances_;
    std::vector<Coupling> couplings_;
    std::vector<Island> islands_;       // only with couplings: every vehicle in exactly one island
    std::vector<VehicleTickWork> tick_work_; // one per vehicle, sized at setup; each entry touched by its island's job only
    std::vector<aero::WakeSource> aero_wakes_; // sized at create; immutable during vehicle jobs
    // PHYS-033 I1 W1: index == BodyId::index -> that vehicle's index into
    // instances_, or -1 if that body is not a vehicle chassis. Built/grown
    // only in create() (sim thread, setup time); note_body_wrench's lookup
    // is then a plain array read, no allocation, no string/map lookup.
    std::vector<std::int32_t> vehicle_for_body_index_;
};

} // namespace ps::vehicle
