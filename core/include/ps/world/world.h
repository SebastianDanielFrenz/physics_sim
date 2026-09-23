// ps/world/world.h — World (PLAN.md section 5).
//
// Owns bodies/joints (via IRigidBackend), force elements, the job system,
// snapshot buffer and telemetry registry, and drives the staged tick
// pipeline. No Jolt or Godot type appears here (PLAN.md D1/D2).

#pragma once

#include "ps/articulation/articulation_desc.h"
#include "ps/backend/i_rigid_backend.h"
#include "ps/io/telemetry.h"
#include "ps/jobs/job_system.h"
#include "ps/jobs/task_graph.h"
#include "ps/terrain/terrain_config.h"
#include "ps/terrain/terrain_source.h"
#include "ps/world/force_element.h"
#include "ps/world/ids.h"
#include "ps/world/snapshot.h"

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

// Forward-declared only: the articulation runtime (PLAN.md D10/P1.5a) is
// internal (core/src/articulation/system.h), never included from a public
// header - World owns it through this incomplete-type pointer exactly the
// way it owns IRigidBackend's Jolt implementation (PLAN.md D2's seam).
namespace ps::articulation { class System; class ContactCoupling; } // ContactCoupling: PLAN.md P1.5b, core/src/articulation/contact.h
// Forward-declared only: PLAN.md M1's tile system (core/src/terrain/tile_manager.h)
// is internal for the same reason - World owns it via this incomplete-type
// pointer, same pattern as articulation::System above.
namespace ps::terrain { class TileManager; }

namespace ps {

struct WorldConfig {
    real tick_rate_hz = 240.0;      // PLAN.md D3: rigid-body tick.
    real substep_rate_hz = 960.0;   // documented stub until P3 (vehicles).
    Vec3 gravity = Vec3{0.0, 0.0, -9.81};
    int job_workers = 0;            // 0 = auto (physical cores - 2, PLAN.md D8)
    // 0 = auto (ps::jobs::JobSystem::kDefaultQueueCapacity, 4096 - every real
    // workload in this codebase). A test may set a small explicit value to
    // force the queue-full inline-execution fallback deliberately (P1.5b
    // review item; see ps::jobs::JobSystem::queue_full_fallback_count).
    std::size_t job_queue_capacity = 0;
    std::size_t max_bodies = 4096;
    // Chunk size for the force-element parallel_for stage; 0 = auto
    // (spreads elements evenly over the worker count).
    std::size_t force_element_grain = 0;
    // Chunk size for the vehicle-island (articulation) parallel_for stage
    // (PLAN.md D8/P1.5a, PHYS-009 Step 1); 0 = auto, same formula as
    // force_element_grain.
    std::size_t articulation_grain = 0;

    // Jolt solver settings, exposed for the Part A energy-loss study
    // (PLAN.md). Defaults equal Jolt's own defaults (see
    // ps::RigidBackendSettings, i_rigid_backend.h, for the exact
    // PhysicsSettings field each one maps to) - unless a caller sets one of
    // these, behaviour is unchanged from before this field existed.
    real baumgarte = 0.2;
    unsigned num_velocity_steps = 10;
    unsigned num_position_steps = 2;
    int collision_steps = 1;
    real penetration_slop = 0.02;
    real speculative_contact_distance = 0.02;
    real max_penetration_distance = 0.2;
    real linear_cast_threshold = 0.75;
    real linear_cast_max_penetration = 0.25;
};

class World {
public:
    explicit World(const WorldConfig& config = {});
    ~World();
    World(const World&) = delete;
    World& operator=(const World&) = delete;

    BodyId create_body(const BodyDesc& desc) { return backend_->create_body(desc); }
    void destroy_body(BodyId id) { backend_->destroy_body(id); }
    JointId create_joint(const JointDesc& desc) { return backend_->create_joint(desc); }
    void destroy_joint(JointId id) { backend_->destroy_joint(id); }

    [[nodiscard]] Pose get_pose(BodyId id) const { return backend_->get_pose(id); }
    [[nodiscard]] Motion get_motion(BodyId id) const { return backend_->get_motion(id); }
    [[nodiscard]] real get_mass(BodyId id) const { return backend_->get_mass(id); }

    // PLAN.md D10/P1.5a: our own reduced-coordinate (Featherstone) solver
    // for articulated chains. Runs in the vehicle-island stage,
    // sub-stepped at WorldConfig::substep_rate_hz inside each tick (D3/D8);
    // see PLAN.md section 5 and P1.5a's scope for the model. Additive to
    // every existing World method; a World that never calls this behaves
    // exactly as before.
    ArticulationId create_articulation(const articulation::ArticulationDesc& desc);
    [[nodiscard]] real joint_position(ArticulationId id, int link_index) const;
    [[nodiscard]] real joint_velocity(ArticulationId id, int link_index) const;
    // Direct joint force/torque actuation (PLAN.md P1.5a scope item 2);
    // persists until changed, applied every substep.
    void set_joint_force(ArticulationId id, int link_index, real tau);
    // A Frame anchored to link_index (-1 == the articulation's own root -
    // meaningful only for a floating root, PLAN.md P1.5a scope item 2).
    // Force elements attach to this exactly like to a body Frame; their
    // wrenches reach the articulation as external spatial forces.
    [[nodiscard]] Frame link_frame(ArticulationId id, int link_index, Pose local_offset = Pose::identity()) const;
    [[nodiscard]] Pose root_pose(ArticulationId id) const { return link_frame(id, -1).world_pose(*backend_); }
    [[nodiscard]] Motion root_motion(ArticulationId id) const { return link_frame(id, -1).world_motion(*backend_); }

    // PLAN.md M1/section 6: the terrain-tile system. Installing a source
    // (re)creates the tile pool per `config`; may only be called once per
    // World (not a hot-path operation - setup time, like create_body).
    // Interest points (one per vehicle/camera) drive which tiles stream in;
    // radius_m is in metres. Wired into the "terrain_tiles" pipeline stage
    // (World::stage_terrain_tiles) - a no-op tick if no source was set,
    // exactly as before this feature existed.
    void set_terrain_source(std::shared_ptr<const terrain::ITerrainSource> source, const terrain::TerrainConfig& config = {});
    void set_terrain_interest_point(std::uint64_t id, const Vec3& world_xy, real stream_radius_m);
    void remove_terrain_interest_point(std::uint64_t id);
    [[nodiscard]] std::size_t terrain_resident_tile_count() const;
    [[nodiscard]] std::size_t terrain_starved_tile_count() const;

    // PLAN.md P2 adapter report / this task's step 3: named control
    // channels (e.g. "steer" -1..1, "throttle" 0..1), SI/ISO, so input can
    // reach vehicle code (P3+) without any Godot (or other front-end) type
    // crossing into the core. A plain seam today - nothing in P1/M1 reads
    // these yet; a future drivetrain/vehicle force element does.
    void set_control(const std::string& channel, real value) { control_channels_[channel] = value; }
    [[nodiscard]] real get_control(const std::string& channel) const {
        const auto it = control_channels_.find(channel);
        return it == control_channels_.end() ? real(0.0) : it->second;
    }

    // Registration order is the element's fixed evaluation/gather id
    // (PLAN.md D8: deterministic ordered gather). Ownership stays with the
    // caller, who must keep the element alive (and remove it first if it
    // will be destroyed before the World).
    ForceElementId add_force_element(const IForceElement* element);
    void remove_force_element(ForceElementId id);

    // Advances exactly one fixed tick (WorldConfig::tick_rate_hz) through
    // the staged pipeline (PLAN.md section 5): freeze tick-start state ->
    // scene queries -> vehicle islands -> force elements -> ordered gather
    // -> [solver seam] -> backend step -> terrain -> snapshot.
    void step();

    [[nodiscard]] std::uint64_t tick() const { return tick_; }
    [[nodiscard]] real dt() const { return dt_; }
    [[nodiscard]] real sim_time() const { return sim_time_; }

    // Deterministic hash of the full world state (PLAN.md D7): same binary
    // + same inputs -> bit-identical value regardless of worker count or
    // scheduling.
    [[nodiscard]] std::uint64_t state_hash() const;

    [[nodiscard]] jobs::JobSystem& job_system() { return *job_system_; }
    [[nodiscard]] IRigidBackend& backend() { return *backend_; }
    [[nodiscard]] const IRigidBackend& backend() const { return *backend_; }

    [[nodiscard]] SnapshotTripleBuffer& snapshots() { return *snapshots_; }
    [[nodiscard]] io::TelemetryRegistry& telemetry() { return *telemetry_; }

    // Named seam for a future solver stage between force evaluation and the
    // backend step (PLAN.md section 6). No-op unless set; a later phase can
    // install a callback here without changing the pipeline shape.
    using BetweenForcesAndBackendStepHook = std::function<void(World&)>;
    void set_between_forces_and_backend_step_hook(BetweenForcesAndBackendStepHook hook) { between_hook_ = std::move(hook); }

    [[nodiscard]] const jobs::TaskGraph& pipeline() const { return *pipeline_; }

private:
    void build_pipeline();
    void stage_vehicle_islands();
    void stage_force_elements();
    void stage_gather();
    void stage_contact_coupling_pre();  // PLAN.md P1.5b; between solver_seam and backend_step
    void stage_backend_step();
    void stage_contact_coupling_post(); // PLAN.md P1.5b; between backend_step and terrain_tiles
    void stage_terrain_tiles();         // PLAN.md M1; no-op until set_terrain_source is called
    void stage_snapshot();

    WorldConfig config_;
    real dt_;
    std::uint64_t tick_ = 0;
    real sim_time_ = 0.0;

    std::unique_ptr<jobs::JobSystem> job_system_;
    std::unique_ptr<IRigidBackend> backend_;
    std::unique_ptr<jobs::TaskGraph> pipeline_;
    std::unique_ptr<articulation::System> articulation_system_; // PLAN.md D10/P1.5a; forward-declared above
    std::unique_ptr<terrain::TileManager> terrain_manager_;      // PLAN.md M1; forward-declared above; null until set_terrain_source
    std::unordered_map<std::string, real> control_channels_;     // PLAN.md P2/M1 step 3
    // PLAN.md P1.5b; index i pairs with ArticulationId{i, 1} (articulations
    // are never destroyed, so index == id.index always); nullptr == that
    // articulation's contact_strategy was None. Declared after backend_
    // (above) so it is destroyed before backend_ (reverse declaration
    // order) - ContactCoupling's destructor calls backend_.destroy_body()
    // on any ProxyBody strategy bodies it created.
    std::vector<std::unique_ptr<articulation::ContactCoupling>> articulation_contacts_;

    std::vector<const IForceElement*> force_elements_; // index == ForceElementId.index; nullptr == removed
    std::vector<Wrench> force_element_wrenches_;        // scatter slots, parallel to force_elements_

    BetweenForcesAndBackendStepHook between_hook_;

    std::unique_ptr<SnapshotTripleBuffer> snapshots_;
    std::unique_ptr<io::TelemetryRegistry> telemetry_;

    // Per-stage timing + worker-utilisation telemetry channels (P1 scope).
    // Wall-clock reads here are profiling/observability only - they never
    // feed back into simulation state or control dt (PLAN.md's "no wall
    // clock in the core" targets the deterministic stepping logic itself).
    std::vector<std::size_t> stage_timing_channels_;
    std::vector<std::size_t> worker_utilization_channels_;
    std::size_t queue_full_fallback_channel_ = 0; // P1.5b review item
    std::vector<double> last_stage_ms_;
    std::vector<ContactEvent> last_contact_events_;
    // PHYS-009 Step 3: reused every tick by stage_snapshot() instead of
    // taking IRigidBackend::live_body_ids()'s by-value return - see that
    // method's comment (i_rigid_backend.h) for why a fresh vector there
    // allocated inside a job every single tick.
    std::vector<BodyId> live_body_ids_scratch_;

public:
    [[nodiscard]] const std::vector<ContactEvent>& last_contact_events() const { return last_contact_events_; }
};

} // namespace ps
