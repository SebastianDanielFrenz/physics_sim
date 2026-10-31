// core/src/vehicle/system.h — owns every ps::World vehicle instance
// (internal; PLAN.md 5.1/P3). Mirrors core/src/articulation/system.h's
// pattern for the same reason: vehicle wheel/suspension/tyre state is
// integrated at substep rate exactly like an articulation instance, and
// IForceElement::evaluate is const/stateless by contract (D8) so it cannot
// own that state (out/handover_p3.md decision 3).

#pragma once

#include <stdexcept>
#include <string>

#include "vehicle_instance.h"

#include "ps/environment/vehicle_air.h"
#include "ps/io/surface_table.h"
#include "ps/jobs/job_system.h"
#include "ps/machine/coupling.h"
#include "ps/machine/hitch_limits.h"
#include "ps/vehicle/vehicle_desc.h"
#include "ps/world/ids.h"

#include "../machine/coupling_solver.h"
#include "../machine/tool_runtime.h"

#include <array>
#include <cstdint>
#include <memory>
#include <span>
#include <string_view>
#include <utility>
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

    // Couples two vehicles at their sockets, sim thread between steps. Vehicles joined by couplings form an
    // island: one job that steps them substep by substep, the couplings solved in id order (one pass, exact for one
    // coupling, coupling_passes Gauss-Seidel sweeps along a chain). A vehicle outside any coupling takes the plain
    // step_tick path, and a World with no coupling never reaches any of this.
    // Each change rebuilds the islands, outside the tick. A towed vehicle has one tow vehicle, a tow vehicle any number
    // of towed ones, and no chain closes a loop.
    [[nodiscard]] machine::AttachCheck check_attach(const machine::CouplingDesc& desc, const IRigidBackend& backend) const;
    HitchId attach(const machine::CouplingDesc& desc, IRigidBackend& backend, real substep_rate_hz); // std::invalid_argument
    void detach(HitchId id, IRigidBackend& backend);
    void set_tow_brake_channel(HitchId id, std::size_t channel_index) { couplings_[id.index].brake_channel = channel_index; }
    void set_lift_channels(HitchId id, std::size_t position, std::size_t mode, std::size_t draft);
    // The coupling two named hitch points make; `problem` says why they do not mate, and then the desc means nothing.
    [[nodiscard]] machine::CouplingDesc mate_points(VehicleId towing, std::string_view towing_point, VehicleId towed, std::string_view towed_point,
                                                    std::string& problem) const;
    [[nodiscard]] machine::HitchLoads hitch_loads(HitchId id, const IRigidBackend& backend) const;
    // The towing point a live coupling sits on; nullptr for one built by hand.
    [[nodiscard]] const machine::HitchPointDesc* towing_point(HitchId id) const;
    [[nodiscard]] bool contains(HitchId id) const { return id.index < couplings_.size() && couplings_[id.index].alive && couplings_[id.index].generation == id.generation; }
    [[nodiscard]] bool chassis_is_coupled(BodyId chassis) const;
    void set_coupling_passes(int passes) { coupling_passes_ = passes; }

    // A PTO shaft across a live hitch, from the towing vehicle's pto.out into the towed vehicle's first PTO tool. Its
    // load goes on the tractor's own shaft network; detaching the hitch pulls the shaft off with it.
    [[nodiscard]] machine::PtoCheck check_pto(HitchId id) const;
    void connect_pto(HitchId id); // std::invalid_argument
    void disconnect_pto(HitchId id);
    [[nodiscard]] bool pto_connected(HitchId id) const { return couplings_[id.index].pto_tool != kNoTool; }

    // Tools. The soil source is read from jobs, concurrently, and may be replaced between steps; without one no tool
    // has soil under it. The vehicles of a World with a tool step through the island path.
    void set_soil_source(std::shared_ptr<const machine::ISoilSource> soil) { soil_ = std::move(soil); }
    [[nodiscard]] std::size_t tool_count(VehicleId id) const { return tools_[id.index].size(); }
    [[nodiscard]] const machine::ToolDesc& tool_desc(VehicleId id, std::size_t tool) const { return tools_[id.index].at(tool).desc(); }
    [[nodiscard]] const machine::ToolState& tool_state(VehicleId id, std::size_t tool) const { return tools_[id.index].at(tool).state(); }
    void set_tool_gauge_depth(VehicleId id, std::size_t tool, real depth_m) { tools_[id.index].at(tool).set_gauge_depth(depth_m); }
    void set_tool_material_flow(VehicleId id, std::size_t tool, real kg_s) { tools_[id.index].at(tool).set_material_flow(kg_s); }
    void set_tool_enabled(VehicleId id, std::size_t tool, bool on) {
        machine::ToolRuntime& t = tools_[id.index].at(tool);
        if (t.desc().kind != machine::ToolKind::Applicator) throw std::invalid_argument("set_tool_enabled: tool '" + t.desc().name + "' is not an applicator");
        t.set_enabled(on);
    }
    // Appends every buffered quad to `out` in (tick, vehicle, tool) order. Between steps; allocates.
    void drain_work(std::vector<machine::WorkQuad>& out);
    [[nodiscard]] std::uint64_t work_dropped() const;
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
    // True iff id names a vehicle this System created and has not retired.
    // Every issued id has generation 1 (create()): a slot is never reused.
    [[nodiscard]] bool contains(VehicleId id) const { return id.generation == 1 && id.index < instances_.size() && retired_[id.index] == 0; }
    // Takes a vehicle out of the simulation and detaches its couplings. Its instance stays, frozen, so that every other
    // vehicle keeps its index, channels and telemetry columns. The chassis body stays a plain body, the caller's to
    // destroy.
    void retire(VehicleId id, IRigidBackend& backend);
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
        bool alive = false;
        id_generation_t generation = 1;
        machine::CouplingState state;
        // A latch ramps the rows' target from what attach measured down to zero: position and rate both match at each
        // end, so nothing jumps, and the force is what the ramp's acceleration takes. The linear part lives in the
        // towing chassis' axes because the pair may turn meanwhile.
        int latch_left = 0, latch_total = 0; // substeps
        Vec3 latch_error_local{}, latch_rate_local{};
        std::array<real, 3> latch_angle{}, latch_angle_rate{}; // locked axes
        std::size_t brake_channel = 0; // a tow_brake channel link: resolved in the towing vehicle's scope
        real brake_state = 0.0;
        // The lift, on a linkage. Target and sensed draft carry over ticks; the rest is rebuilt each tick.
        std::size_t lift_position_channel = 0, lift_mode_channel = 0, lift_draft_channel = 0;
        real lift_target = 0.0, lift_target_rate = 0.0;
        real lift_sensed_n = 0.0;
        machine::LiftCommand lift_command{};
        machine::LinkageSolve linkage_last{};
        std::size_t pto_tool = kNoTool; // the towed vehicle's tool on the shaft
        // This substep's wrench so far: a later pass replaces it, it does not add to it.
        Vec3 force{}, torque_towing{}, torque_towed{};
        machine::SocketSolve last{};
    };
    // Sorted by lowest vehicle index; vehicles and couplings ascending. Built at setup, read only during the tick.
    struct Island {
        std::vector<std::size_t> vehicles;
        std::vector<std::size_t> couplings;
        std::vector<BodyId> chassis; // of vehicles: a tool looking for the ground does not see them
    };
    static constexpr std::size_t kNoTool = static_cast<std::size_t>(-1);

    void rebuild_islands();
    void apply_collision_pairs(IRigidBackend& backend);
    void kill_coupling(Coupling& c);
    void update_tow_brakes(const Island& island, const IRigidBackend& backend, const std::vector<real>& channel_values, real tick_dt);
    void update_lifts(const Island& island, const std::vector<real>& channel_values, real tick_dt);
    void update_pto_shafts(const Island& island);
    [[nodiscard]] machine::LatchTarget latch_target(const Coupling& c, const VehicleTickWork& towing, real h) const;
    void step_island(Island& island, IRigidBackend& backend, const std::vector<real>& channel_values, const VehicleTickEnv& env,
                     real tick_dt, real substep_rate_hz, std::span<const environment::VehicleAir> air);
    void solve_coupling(Coupling& c, real h);

    std::vector<std::unique_ptr<VehicleInstance>> instances_;
    std::vector<Coupling> couplings_;  // slots: a dead one is reused with the next generation
    std::size_t alive_couplings_ = 0;
    std::vector<unsigned char> retired_; // one per instance
    std::vector<real> tow_brake_demand_; // one per instance; each entry touched by its island's job only
    int coupling_passes_ = 2;
    // One entry per instance, empty for a vehicle without a machine. tool_vehicles_ counts the ones with tools.
    std::vector<std::vector<machine::ToolRuntime>> tools_;
    std::size_t tool_vehicles_ = 0;
    std::shared_ptr<const machine::ISoilSource> soil_;
    std::uint64_t work_tick_ = 0;
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
