// ps/world/world.h — World (PLAN.md section 5).
//
// Owns bodies/joints (via IRigidBackend), force elements, the job system,
// snapshot buffer and telemetry registry, and drives the staged tick
// pipeline. No Jolt or Godot type appears here (PLAN.md D1/D2).

#pragma once

#include "ps/backend/i_rigid_backend.h"
#include "ps/io/telemetry.h"
#include "ps/jobs/job_system.h"
#include "ps/jobs/task_graph.h"
#include "ps/world/force_element.h"
#include "ps/world/ids.h"
#include "ps/world/snapshot.h"

#include <cstdint>
#include <functional>
#include <memory>
#include <vector>

namespace ps {

struct WorldConfig {
    real tick_rate_hz = 240.0;      // PLAN.md D3: rigid-body tick.
    real substep_rate_hz = 960.0;   // documented stub until P3 (vehicles).
    Vec3 gravity = Vec3{0.0, 0.0, -9.81};
    int job_workers = 0;            // 0 = auto (physical cores - 2, PLAN.md D8)
    std::size_t max_bodies = 4096;
    // Chunk size for the force-element parallel_for stage; 0 = auto
    // (spreads elements evenly over the worker count).
    std::size_t force_element_grain = 0;
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
    void stage_force_elements();
    void stage_gather();
    void stage_backend_step();
    void stage_snapshot();

    WorldConfig config_;
    real dt_;
    std::uint64_t tick_ = 0;
    real sim_time_ = 0.0;

    std::unique_ptr<jobs::JobSystem> job_system_;
    std::unique_ptr<IRigidBackend> backend_;
    std::unique_ptr<jobs::TaskGraph> pipeline_;

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
    std::vector<double> last_stage_ms_;
    std::vector<ContactEvent> last_contact_events_;

public:
    [[nodiscard]] const std::vector<ContactEvent>& last_contact_events() const { return last_contact_events_; }
};

} // namespace ps
