// core/src/machine/tool_runtime.h — one tool in the tick: reads the ground at the tick's start,
// puts its forces on the shadow chassis each substep, writes its swept quad at the tick's end.
// Owned by vehicle::System, stepped inside the vehicle's island job: nothing here allocates
// after construction.

#pragma once

#include "ps/backend/i_rigid_backend.h"
#include "ps/machine/soil_source.h"
#include "ps/machine/tool.h"
#include "ps/machine/work.h"
#include "ps/math/pose.h"

#include <cstdint>
#include <span>
#include <vector>

namespace ps::machine {

class ToolRuntime {
public:
    ToolRuntime(const ToolDesc& desc, VehicleId vehicle, std::uint32_t index, std::size_t work_capacity);

    // The soil and the ground under the edge for this tick.
    void tick_begin(const Pose& pose, const IRigidBackend& backend, std::span<const BodyId> excluded, const ISoilSource* soil);
    // A rotor's load for this tick, from last tick's work and this tick's shaft speed (rad/s at the tractor's shaft).
    [[nodiscard]] real pto_load_nm(real shaft_omega);
    // Force and torque about the centre of mass, world axes.
    void substep(const Pose& pose, const Motion& motion, Vec3& force, Vec3& torque);
    void tick_end(const Pose& pose, std::uint64_t tick);
    void disconnect_shaft() { shaft_fraction_ = 0.0; state_.pto_speed_fraction = 0.0; state_.pto_torque_nm = 0.0; }

    [[nodiscard]] const ToolDesc& desc() const { return desc_; }
    [[nodiscard]] const ToolState& state() const { return state_; }
    // Applicator only (a soil tool has no switch): the owner of the vehicle checks the kind first.
    void set_enabled(bool on) { enabled_ = on; state_.enabled = on; }
    [[nodiscard]] bool enabled() const { return enabled_; }
    void set_gauge_depth(real depth_m) { gauge_depth_ = depth_m; state_.gauge_depth_m = depth_m; }
    [[nodiscard]] real shaft_omega_nominal() const;

    // Moves the buffered quads to the end of `out`, oldest first.
    void drain(std::vector<WorkQuad>& out);
    [[nodiscard]] std::uint64_t dropped() const { return dropped_; }
    [[nodiscard]] std::uint64_t fold_state_hash(std::uint64_t seed) const;

private:
    void push(const WorkQuad& quad);

    ToolDesc desc_;
    VehicleId vehicle_;
    std::uint32_t index_ = 0;
    Vec3 mid_local_{};

    bool have_soil_ = false, have_ground_ = false;
    SoilSample soil_{};
    real ground_z_ = 0.0;
    real gauge_depth_ = 0.0;
    bool enabled_ = false; // applicator only

    // Over the tick's substeps.
    int substeps_ = 0;
    real depth_sum_ = 0.0, speed_sum_ = 0.0, draft_sum_ = 0.0, vertical_sum_ = 0.0;

    // The edge at the end of the last tick; only meaningful with have_edge_.
    bool have_edge_ = false;
    WorkPoint left_{}, right_{};
    real shaft_fraction_ = 0.0; // of nominal; 0 without a shaft
    bool in_soil_ = false;      // last tick: deep enough in ground it can work

    ToolState state_{};

    std::vector<WorkQuad> ring_;
    std::size_t head_ = 0, count_ = 0;
    std::uint64_t dropped_ = 0;
};

} // namespace ps::machine
