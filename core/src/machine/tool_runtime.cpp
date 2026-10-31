#include "tool_runtime.h"

#include <algorithm>
#include <cmath>

namespace ps::machine {

namespace {

constexpr real kTwoPi = 6.283185307179586476925286766559;
constexpr real kMsToKmh = 3.6;
// How far above the edge the ground is looked for, and below it. Implements work on a body that is at most a few metres
// from the ground; the ray starts above any slope the edge can have sunk under.
constexpr real kGroundProbeUp = 3.0;
constexpr real kGroundProbeLength = 8.0;
constexpr real kGroundProbeNudge = 0.01;
constexpr real kMinQuadArea = 1e-6; // m2

template <typename T>
[[nodiscard]] std::uint64_t fold(std::uint64_t h, const T& value) {
    const auto* bytes = reinterpret_cast<const unsigned char*>(&value);
    for (std::size_t i = 0; i < sizeof(value); ++i) {
        h ^= bytes[i];
        h *= 1099511628211ULL;
    }
    return h;
}

} // namespace

ToolRuntime::ToolRuntime(const ToolDesc& desc, VehicleId vehicle, std::uint32_t index, std::size_t work_capacity)
    : desc_(desc), vehicle_(vehicle), index_(index), mid_local_((desc.edge_left_local + desc.edge_right_local) * real(0.5)), gauge_depth_(desc.depth_nominal_m), ring_(work_capacity) {
    state_.gauge_depth_m = gauge_depth_;
}

real ToolRuntime::shaft_omega_nominal() const { return desc_.pto.nominal_rpm * kTwoPi / 60.0; }

void ToolRuntime::tick_begin(const Pose& pose, const IRigidBackend& backend, std::span<const BodyId> excluded, const ISoilSource* soil) {
    const Vec3 mid = pose.transform_point(mid_local_);
    have_soil_ = soil != nullptr;
    soil_ = have_soil_ ? soil->sample(mid.x, mid.y) : SoilSample{};
    const Vec3 down = Vec3::unit_z() * real(-1.0);
    RayCastHit hit = backend.ray_cast_excluding(mid + Vec3::unit_z() * kGroundProbeUp, down, kGroundProbeLength, excluded);
    // A ray can slip through exactly on the edge between two triangles; beside it, it cannot.
    if (!hit.hit) hit = backend.ray_cast_excluding(mid + Vec3{kGroundProbeNudge, kGroundProbeNudge, kGroundProbeUp}, down, kGroundProbeLength, excluded);
    have_ground_ = hit.hit;
    if (hit.hit) ground_z_ = hit.point.z;
    substeps_ = 0;
    depth_sum_ = speed_sum_ = draft_sum_ = vertical_sum_ = 0.0;
}

real ToolRuntime::pto_load_nm(real shaft_omega) {
    if (!desc_.pto.present) return 0.0;
    const real nominal = shaft_omega_nominal();
    shaft_fraction_ = std::abs(shaft_omega) / nominal;
    state_.pto_speed_fraction = shaft_fraction_;
    // The rotor's own losses grow with its speed like its power falls with the load: a torque. The soil is a power: what
    // goes through per second is the forward speed's, so a slowing rotor bites deeper and the torque rises.
    const real idle = tool_pto_power_w(desc_, 0.0, 0.0) / nominal;
    const real floor = nominal * desc_.pto.min_speed_fraction;
    const real soil_power = in_soil_ ? tool_pto_power_w(desc_, state_.speed_m_s, state_.depth_m) - tool_pto_power_w(desc_, 0.0, 0.0) : real(0.0);
    state_.pto_torque_nm = idle + soil_power / std::max(std::abs(shaft_omega), floor);
    return state_.pto_torque_nm;
}

void ToolRuntime::substep(const Pose& pose, const Motion& motion, Vec3& force, Vec3& torque) {
    force = Vec3::zero();
    torque = Vec3::zero();
    ++substeps_;
    if (!have_soil_ || !have_ground_) return;

    const Vec3 mid = pose.transform_point(mid_local_);
    const real depth = ground_z_ - mid.z;
    Vec3 forward = pose.orientation.rotate(Vec3::unit_x());
    forward.z = 0.0;
    const real forward_length = forward.length();
    if (!(forward_length > 1e-6)) return;
    forward = forward / forward_length;
    const Vec3 lateral{-forward.y, forward.x, 0.0};
    const Vec3 offset = mid - pose.position;
    const Vec3 velocity = motion.linear + cross(motion.angular, offset);
    const real speed = dot(velocity, forward);
    speed_sum_ += speed;
    if (depth <= 0.0) return;

    const ToolSoilDesc& s = desc_.soil;
    const real creep = desc_.draft.creep_speed_m_s;
    const real along = std::clamp(speed / creep, real(-1.0), real(1.0));
    const real across = std::clamp(dot(velocity, lateral) / creep, real(-1.0), real(1.0));
    const real width = desc_.width_m;

    real draft = 0.0, up = 0.0;
    if (soil_.workable) {
        const real d = tool_draft_n(desc_, soil_, speed * kMsToKmh, depth);
        draft = d * along;
        // Soil pushes back, it does not hold on: damping acts going down, and only the draft's own suction pulls.
        up = std::max(s.bearing_n_m2 * width * depth + s.gauge_n_m2 * width * std::max(depth - gauge_depth_, real(0.0)) - s.damping_n_s_m2 * width * velocity.z, real(0.0));
        up -= s.suction_ratio * d * std::abs(along);
    } else {
        up = std::max(s.hard_n_m2 * width * depth - s.damping_n_s_m2 * width * velocity.z, real(0.0));
        draft = s.hard_friction * up * along;
    }
    const real side = soil_.workable ? desc_.draft.lateral_ratio * tool_draft_n(desc_, soil_, speed * kMsToKmh, depth) * across : s.hard_friction * up * across;

    force = forward * -draft + lateral * -side + Vec3::unit_z() * up;
    // At half the worked depth.
    torque = cross(offset + Vec3::unit_z() * (0.5 * depth), force);

    depth_sum_ += depth;
    draft_sum_ += draft;
    vertical_sum_ += up;
}

void ToolRuntime::tick_end(const Pose& pose, std::uint64_t tick) {
    const real n = static_cast<real>(std::max(substeps_, 1));
    state_.depth_m = depth_sum_ / n;
    state_.speed_m_s = speed_sum_ / n;
    state_.draft_n = draft_sum_ / n;
    state_.vertical_n = vertical_sum_ / n;
    state_.quality = tool_quality(desc_, state_.speed_m_s * kMsToKmh);

    const bool shaft_ok = !desc_.pto.present || shaft_fraction_ >= desc_.pto.min_speed_fraction;
    in_soil_ = have_soil_ && soil_.workable && state_.depth_m >= desc_.depth_min_m;
    const bool working = in_soil_ && shaft_ok;
    state_.working = working;

    const Vec3 l = pose.transform_point(desc_.edge_left_local);
    const Vec3 r = pose.transform_point(desc_.edge_right_local);
    const WorkPoint left{l.x, l.y}, right{r.x, r.y};
    if (working && have_edge_) {
        WorkQuad q;
        q.tick = tick;
        q.vehicle = vehicle_;
        q.tool = index_;
        q.left_start = left_;
        q.right_start = right_;
        q.right_end = right;
        q.left_end = left;
        q.depth_m = state_.depth_m;
        q.speed_m_s = state_.speed_m_s;
        q.quality = state_.quality;
        // Standing still sweeps nothing: the strip goes on from where it stopped.
        const real area = work_area(q);
        if (area < kMinQuadArea) return;
        state_.worked_area_m2 += area;
        push(q);
    }
    left_ = left;
    right_ = right;
    have_edge_ = working;
}

void ToolRuntime::push(const WorkQuad& quad) {
    if (count_ == ring_.size()) {
        ++dropped_;
        return;
    }
    ring_[(head_ + count_) % ring_.size()] = quad;
    ++count_;
}

void ToolRuntime::drain(std::vector<WorkQuad>& out) {
    for (; count_ > 0; --count_) {
        out.push_back(ring_[head_]);
        head_ = (head_ + 1) % ring_.size();
    }
}

std::uint64_t ToolRuntime::fold_state_hash(std::uint64_t seed) const {
    std::uint64_t h = seed;
    h = fold(h, gauge_depth_);
    h = fold(h, have_edge_);
    h = fold(h, in_soil_);
    h = fold(h, state_.worked_area_m2);
    h = fold(h, state_.depth_m);
    h = fold(h, state_.speed_m_s);
    return h;
}

} // namespace ps::machine
