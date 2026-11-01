#include "system.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <numeric>
#include <stdexcept>
#include <string>

namespace ps::vehicle {

namespace {

constexpr real kRadToDeg = 180.0 / 3.14159265358979323846;

[[nodiscard]] machine::SocketBody socket_body_now(const IRigidBackend& backend, BodyId chassis, const Vec3& socket_local, const Quat& frame_local) {
    const Pose pose = backend.get_pose(chassis);
    const Motion motion = backend.get_motion(chassis);
    machine::SocketBody b;
    b.socket_offset = pose.orientation.rotate(socket_local);
    b.socket_position = pose.position + b.socket_offset;
    b.hitch_frame = pose.orientation * frame_local;
    b.linear = motion.linear;
    b.angular = motion.angular;
    return b;
}

[[nodiscard]] machine::LinkageBody linkage_body_now(const IRigidBackend& backend, BodyId chassis, const Quat& frame_local) {
    const Pose pose = backend.get_pose(chassis);
    const Motion motion = backend.get_motion(chassis);
    machine::LinkageBody b;
    b.position = pose.position;
    b.orientation = pose.orientation;
    b.hitch_frame = pose.orientation * frame_local;
    b.linear = motion.linear;
    b.angular = motion.angular;
    return b;
}

// How many quads a tool can hold between two drains: a second and a bit at 240 Hz would be 300; a farm that drains once a
// game minute at time scale 1 needs more.
constexpr std::size_t kWorkQuadCapacity = 4096;

template <typename... Args>
[[nodiscard]] std::string format(const char* pattern, Args... args) {
    char text[200];
    std::snprintf(text, sizeof(text), pattern, args...);
    return text;
}

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

VehicleId System::create(const VehicleDesc& desc, BodyId chassis, const io::SurfaceTable* surfaces,
                          const ControlChannelIndices& control_indices) {
    instances_.push_back(std::make_unique<VehicleInstance>(desc, chassis, surfaces, control_indices));
    aero_wakes_.resize(instances_.size());
    tick_work_.resize(instances_.size());
    retired_.resize(instances_.size(), 0);
    tow_brake_demand_.resize(instances_.size(), 0.0);
    tools_.emplace_back();
    if (desc.machine) {
        machine::validate_machine(*desc.machine);
        const VehicleId id{static_cast<id_index_t>(instances_.size() - 1), 1};
        std::vector<machine::ToolRuntime>& list = tools_.back();
        list.reserve(desc.machine->tools.size());
        for (std::size_t t = 0; t < desc.machine->tools.size(); ++t) list.emplace_back(desc.machine->tools[t], id, static_cast<std::uint32_t>(t), kWorkQuadCapacity);
        ++tool_vehicles_;
    }
    const auto vehicle_index = static_cast<std::int32_t>(instances_.size() - 1);
    if (vehicle_for_body_index_.size() <= chassis.index) vehicle_for_body_index_.resize(chassis.index + 1, -1);
    vehicle_for_body_index_[chassis.index] = vehicle_index;
    if (alive_couplings_ > 0 || tool_vehicles_ > 0) rebuild_islands();
    return VehicleId{static_cast<id_index_t>(instances_.size() - 1), 1};
}

machine::AttachCheck System::check_attach(const machine::CouplingDesc& desc, const IRigidBackend& backend) const {
    machine::AttachCheck check;
    const auto refuse = [&check](std::string message) {
        if (check.message.empty()) check.message = "attach: " + std::move(message);
    };
    if (!contains(desc.towing) || !contains(desc.towed)) {
        refuse("unknown vehicle");
        return check;
    }
    if (desc.towing == desc.towed) {
        refuse("a vehicle cannot tow itself");
        return check;
    }
    for (const Coupling& c : couplings_) {
        if (c.alive && c.towed == desc.towed.index) refuse(format("vehicle %u is already towed", static_cast<unsigned>(desc.towed.index)));
    }
    // Walks up the tow vehicles; a chain has no loop, so this ends.
    std::size_t top = desc.towing.index;
    for (bool moved = true; moved;) {
        moved = false;
        for (const Coupling& c : couplings_) {
            if (c.alive && c.towed == top) {
                top = c.towing;
                moved = true;
                break;
            }
        }
    }
    if (top == desc.towed.index) refuse("the chain would close a loop");
    for (const Coupling& c : couplings_) {
        if (c.alive && c.towing == desc.towing.index && !desc.towing_point.empty() && c.desc.towing_point == desc.towing_point) {
            refuse(format("'%s' of vehicle %u is in use", desc.towing_point.c_str(), static_cast<unsigned>(desc.towing.index)));
        }
    }

    const machine::HitchClassDesc& hitch = desc.hitch;
    if (desc.linkage) {
        const machine::LinkageBody a = linkage_body_now(backend, instances_[desc.towing.index]->chassis(), desc.frame_towing_local);
        const machine::LinkageBody b = linkage_body_now(backend, instances_[desc.towed.index]->chassis(), desc.frame_towed_local);
        const machine::LinkageMeasure m = machine::measure_linkage(a, b, *desc.linkage);
        check.error_m = m.error.length();
        check.speed_m_s = m.error_rate.length();
        check.angle_rad = m.angle;
        const char* name = hitch.name.c_str();
        if (check.error_m > hitch.max_latch_error_m) refuse(format("links %.3f m off their pins, class '%s' latches within %.3f m", check.error_m, name, hitch.max_latch_error_m));
        if (check.speed_m_s > hitch.max_latch_speed_m_s) refuse(format("pins closing at %.3f m/s, class '%s' latches below %.3f m/s", check.speed_m_s, name, hitch.max_latch_speed_m_s));
        for (const std::size_t k : {std::size_t{0}, std::size_t{2}}) {
            if (std::abs(m.angle[k]) > hitch.max_latch_angle_rad) refuse(format("%s %.2f deg, class '%s' latches within %.2f deg", k == 0 ? "yaw" : "roll", m.angle[k] * kRadToDeg, name, hitch.max_latch_angle_rad * kRadToDeg));
        }
        if (m.lift_angle > desc.linkage->max_angle_rad || m.lift_angle < desc.linkage->min_angle_rad) {
            refuse(format("the pins need the lower links at %.2f deg, the stops are %.2f..%.2f deg", m.lift_angle * kRadToDeg, desc.linkage->min_angle_rad * kRadToDeg, desc.linkage->max_angle_rad * kRadToDeg));
        }
        check.ok = check.message.empty();
        return check;
    }
    const machine::SocketBody a = socket_body_now(backend, instances_[desc.towing.index]->chassis(), desc.socket_towing_local, desc.frame_towing_local);
    const machine::SocketBody b = socket_body_now(backend, instances_[desc.towed.index]->chassis(), desc.socket_towed_local, desc.frame_towed_local);
    const machine::HitchMeasure m = machine::measure_hitch(a, b, hitch);
    check.error_m = m.error.length();
    check.speed_m_s = m.error_rate.length();
    check.angle_rad = m.angle;
    const char* name = hitch.name.c_str();
    if (check.error_m > hitch.max_latch_error_m) refuse(format("sockets %.3f m apart, class '%s' latches within %.3f m", check.error_m, name, hitch.max_latch_error_m));
    if (check.speed_m_s > hitch.max_latch_speed_m_s) refuse(format("sockets closing at %.3f m/s, class '%s' latches below %.3f m/s", check.speed_m_s, name, hitch.max_latch_speed_m_s));
    const std::array<const machine::HitchAxis*, 3> axes = {&hitch.yaw, &hitch.pitch, &hitch.roll};
    const std::array<const char*, 3> axis_names = {"yaw", "pitch", "roll"};
    for (std::size_t k = 0; k < 3; ++k) {
        const real angle = m.angle[k];
        if (axes[k]->mode == machine::HitchAxisMode::Locked && std::abs(angle) > hitch.max_latch_angle_rad) {
            refuse(format("%s %.2f deg, class '%s' latches within %.2f deg", axis_names[k], angle * kRadToDeg, name, hitch.max_latch_angle_rad * kRadToDeg));
        }
        if (axes[k]->mode == machine::HitchAxisMode::Limited && (angle > axes[k]->max_rad || angle < axes[k]->min_rad)) {
            refuse(format("%s %.2f deg lies outside class '%s' limits %.2f..%.2f deg", axis_names[k], angle * kRadToDeg, name, axes[k]->min_rad * kRadToDeg, axes[k]->max_rad * kRadToDeg));
        }
    }
    check.ok = check.message.empty();
    return check;
}

HitchId System::attach(const machine::CouplingDesc& desc, IRigidBackend& backend, real substep_rate_hz) {
    machine::validate_hitch_class(desc.hitch);
    const machine::AttachCheck check = check_attach(desc, backend);
    if (!check.ok) throw std::invalid_argument(check.message);

    std::size_t slot = couplings_.size();
    for (std::size_t i = 0; i < couplings_.size(); ++i) {
        if (!couplings_[i].alive) {
            slot = i;
            break;
        }
    }
    if (slot == couplings_.size()) couplings_.emplace_back();
    Coupling& c = couplings_[slot];
    const id_generation_t generation = c.generation;
    c = Coupling{};
    c.generation = generation;
    c.alive = true;
    c.desc = desc;
    c.towing = desc.towing.index;
    c.towed = desc.towed.index;

    const BodyId towing = instances_[c.towing]->chassis();
    c.latch_total = c.latch_left = std::max(1, static_cast<int>(desc.hitch.latch_time_s * substep_rate_hz + 0.5));
    if (desc.linkage) {
        // A linkage's rows are link lengths, not a vector: their latch stays as measured whatever the pair does. The lift
        // takes over the height the implement has.
        const machine::LinkageBody a = linkage_body_now(backend, towing, desc.frame_towing_local);
        const machine::LinkageBody b = linkage_body_now(backend, instances_[c.towed]->chassis(), desc.frame_towed_local);
        const machine::LinkageMeasure m = machine::measure_linkage(a, b, *desc.linkage);
        c.latch_error_local = m.error;
        c.latch_rate_local = m.error_rate;
        for (const std::size_t k : {std::size_t{0}, std::size_t{2}}) {
            c.latch_angle[k] = m.angle[k];
            c.latch_angle_rate[k] = m.angle_rate[k];
        }
        const machine::LinkageGeometry& g = *desc.linkage;
        c.lift_target = m.lift_angle;
        c.state.linkage.angle_rad = m.lift_angle;
        c.state.linkage.target_rad = m.lift_angle;
        c.state.linkage.position = (m.lift_angle - g.min_angle_rad) / (g.max_angle_rad - g.min_angle_rad);
    } else {
        const machine::SocketBody a = socket_body_now(backend, towing, desc.socket_towing_local, desc.frame_towing_local);
        const machine::SocketBody b = socket_body_now(backend, instances_[c.towed]->chassis(), desc.socket_towed_local, desc.frame_towed_local);
        const machine::HitchMeasure m = machine::measure_hitch(a, b, desc.hitch);
        const Quat towing_rotation = backend.get_pose(towing).orientation;
        c.latch_error_local = towing_rotation.conjugate().rotate(m.error);
        c.latch_rate_local = towing_rotation.conjugate().rotate(m.error_rate - cross(a.angular, m.error));
        const std::array<const machine::HitchAxis*, 3> axes = {&desc.hitch.yaw, &desc.hitch.pitch, &desc.hitch.roll};
        for (std::size_t k = 0; k < 3; ++k) {
            if (axes[k]->mode != machine::HitchAxisMode::Locked) continue;
            c.latch_angle[k] = m.angle[k];
            c.latch_angle_rate[k] = m.angle_rate[k];
        }
    }
    c.state.latch_progress = 0.0;
    ++alive_couplings_;
    rebuild_islands();
    apply_collision_pairs(backend);
    return HitchId{static_cast<id_index_t>(slot), generation};
}

machine::CouplingDesc System::mate_points(VehicleId towing, std::string_view towing_point, VehicleId towed, std::string_view towed_point, std::string& problem) const {
    machine::CouplingDesc d;
    if (!contains(towing) || !contains(towed)) {
        problem = "unknown vehicle";
        return d;
    }
    const machine::HitchPointDesc* a = machine::find_hitch_point(instances_[towing.index]->hitch_points(), towing_point);
    const machine::HitchPointDesc* b = machine::find_hitch_point(instances_[towed.index]->hitch_points(), towed_point);
    const std::string a_name(towing_point), b_name(towed_point);
    if (a == nullptr) problem = "vehicle " + std::to_string(towing.index) + " has no hitch point '" + a_name + "'";
    else if (b == nullptr) problem = "vehicle " + std::to_string(towed.index) + " has no hitch point '" + b_name + "'";
    else if (a->role != machine::HitchRole::Towing) problem = "'" + a_name + "' is not a towing point";
    else if (b->role != machine::HitchRole::Towed) problem = "'" + b_name + "' is not a towed point";
    else if (a->hitch.name != b->hitch.name) problem = "'" + a_name + "' takes class '" + a->hitch.name + "', '" + b_name + "' class '" + b->hitch.name + "'";
    else if (a->linkage && !b->three_point) problem = "'" + a_name + "' is a linkage, '" + b_name + "' has no three pins";
    if (!problem.empty()) return d;
    d.towing = towing;
    d.towed = towed;
    d.socket_towing_local = a->position_local;
    d.socket_towed_local = b->position_local;
    d.hitch = a->hitch;
    d.towing_point = a_name;
    d.towed_point = b_name;
    if (a->linkage) {
        const machine::LinkageDesc& lift = *a->linkage;
        machine::LinkageGeometry g;
        g.lower_pivot_local = lift.lower_pivot_local;
        g.top_pivot_local = lift.top_pivot_local;
        g.lower_pin_local = b->lower_pin_local;
        g.top_pin_local = b->top_pin_local;
        const Vec3 lower = a->lower_pin_local - lift.lower_pivot_local;
        g.lower_length_m = lower.length();
        g.top_length_m = (a->top_pin_local - lift.top_pivot_local).length();
        g.reference_angle_rad = machine::linkage_angle(lower);
        g.min_angle_rad = lift.min_angle_rad;
        g.max_angle_rad = lift.max_angle_rad;
        g.max_lift_torque_nm = lift.max_lift_force_n * g.lower_length_m;
        d.linkage = g;
        d.linkage_lift = lift;
    }
    return d;
}

machine::HitchLoads System::hitch_loads(HitchId id, const IRigidBackend& backend) const {
    const Coupling& c = couplings_[id.index];
    machine::HitchLoads loads;
    loads.tongue_load_n = -c.state.mean_force_on_towing_world.z;
    loads.lift_force_n = c.state.linkage.lift_force_n;
    loads.lift_saturated = c.state.linkage.saturated;
    std::vector<std::size_t> behind{c.towed};
    while (!behind.empty()) {
        const std::size_t v = behind.back();
        behind.pop_back();
        loads.towed_mass_kg += backend.get_mass(instances_[v]->chassis());
        for (const Coupling& next : couplings_) {
            if (next.alive && next.towing == v) behind.push_back(next.towed);
        }
    }
    return loads;
}

const machine::HitchPointDesc* System::towing_point(HitchId id) const {
    const Coupling& c = couplings_[id.index];
    return machine::find_hitch_point(instances_[c.towing]->hitch_points(), c.desc.towing_point);
}

void System::kill_coupling(Coupling& c) {
    if (c.pto_tool != kNoTool) {
        instances_[c.towing]->set_pto_load({});
        tools_[c.towed][c.pto_tool].disconnect_shaft();
        c.pto_tool = kNoTool;
    }
    c.alive = false;
    tow_brake_demand_[c.towed] = 0.0;
    ++c.generation;
    --alive_couplings_;
}

void System::detach(HitchId id, IRigidBackend& backend) {
    if (!contains(id)) throw std::invalid_argument("detach: unknown hitch");
    kill_coupling(couplings_[id.index]);
    rebuild_islands();
    apply_collision_pairs(backend);
}

void System::retire(VehicleId id, IRigidBackend& backend) {
    if (!contains(id)) throw std::invalid_argument("destroy_vehicle: unknown vehicle");
    for (Coupling& c : couplings_) {
        if (c.alive && (c.towing == id.index || c.towed == id.index)) kill_coupling(c);
    }
    retired_[id.index] = 1;
    if (!tools_[id.index].empty()) --tool_vehicles_;
    const BodyId chassis = instances_[id.index]->chassis();
    if (chassis.index < vehicle_for_body_index_.size()) vehicle_for_body_index_[chassis.index] = -1;
    rebuild_islands();
    apply_collision_pairs(backend);
}

bool System::chassis_is_coupled(BodyId chassis) const {
    return std::any_of(couplings_.begin(), couplings_.end(), [&](const Coupling& c) {
        return c.alive && (instances_[c.towing]->chassis() == chassis || instances_[c.towed]->chassis() == chassis);
    });
}

void System::rebuild_islands() {
    // Union-find over vehicle indices, always keeping the lower index as the root, so an island's root is its lowest
    // vehicle and walking the vehicles in ascending order gives the same islands in the same order on every run.
    std::vector<std::size_t> parent(instances_.size());
    std::iota(parent.begin(), parent.end(), std::size_t{0});
    const auto find = [&parent](std::size_t i) {
        while (parent[i] != i) i = parent[i] = parent[parent[i]];
        return i;
    };
    for (const Coupling& c : couplings_) {
        if (!c.alive) continue;
        const std::size_t a = find(c.towing), b = find(c.towed);
        parent[std::max(a, b)] = std::min(a, b);
    }
    islands_.clear();
    std::vector<std::size_t> island_of_root(instances_.size(), instances_.size());
    for (std::size_t i = 0; i < instances_.size(); ++i) {
        if (retired_[i] != 0) continue;
        const std::size_t root = find(i);
        if (island_of_root[root] == instances_.size()) {
            island_of_root[root] = islands_.size();
            islands_.emplace_back();
        }
        islands_[island_of_root[root]].vehicles.push_back(i);
    }
    for (std::size_t i = 0; i < couplings_.size(); ++i) {
        if (couplings_[i].alive) islands_[island_of_root[find(couplings_[i].towing)]].couplings.push_back(i);
    }

    for (Island& island : islands_) {
        for (const std::size_t i : island.vehicles) island.chassis.push_back(instances_[i]->chassis());
        for (const std::size_t i : island.vehicles) instances_[i]->set_cast_exclusions(island.couplings.empty() ? std::vector<BodyId>{} : island.chassis);
    }
}

void System::apply_collision_pairs(IRigidBackend& backend) {
    // Only directly coupled chassis pass through each other: a tractor still hits its second trailer.
    std::vector<std::pair<BodyId, BodyId>> pairs;
    for (const Coupling& c : couplings_) {
        if (c.alive) pairs.emplace_back(instances_[c.towing]->chassis(), instances_[c.towed]->chassis());
    }
    backend.set_disabled_collision_pairs(pairs);
}

machine::LatchTarget System::latch_target(const Coupling& c, const VehicleTickWork& towing, real h) const {
    machine::LatchTarget t;
    if (c.latch_left == 0) return t;
    // A cubic through (position, rate) at the start and (0, 0) at the end, over the latch time T.
    const real duration = static_cast<real>(c.latch_total) * h;
    const real u = 1.0 - static_cast<real>(c.latch_left) / static_cast<real>(c.latch_total);
    const real position_gain = (2.0 * u - 3.0) * u * u + 1.0;
    const real rate_gain = u * (u - 1.0) * (u - 1.0) * duration;
    const real position_rate = 6.0 * u * (u - 1.0) / duration;
    const real rate_rate = (3.0 * u - 1.0) * (u - 1.0);
    const Vec3 local = c.latch_error_local * position_gain + c.latch_rate_local * rate_gain;
    const Vec3 local_rate = c.latch_error_local * position_rate + c.latch_rate_local * rate_rate;
    if (c.desc.linkage) {
        t.error = local;
        t.error_rate = local_rate;
    } else {
        t.error = towing.pose_s.orientation.rotate(local);
        t.error_rate = cross(towing.twist_s.angular, t.error) + towing.pose_s.orientation.rotate(local_rate);
    }
    for (std::size_t k = 0; k < 3; ++k) {
        t.angle[k] = c.latch_angle[k] * position_gain + c.latch_angle_rate[k] * rate_gain;
        t.angle_rate[k] = c.latch_angle[k] * position_rate + c.latch_angle_rate[k] * rate_rate;
    }
    return t;
}

void System::update_tow_brakes(const Island& island, const IRigidBackend& backend, const std::vector<real>& channel_values, real tick_dt) {
    // In coupling order: in a chain whose rear link comes first the demand reaches it a tick later, the same every run.
    for (const std::size_t ci : island.couplings) {
        Coupling& c = couplings_[ci];
        const std::optional<machine::TowBrakeDesc>& link = instances_[c.towed]->tow_brake();
        if (!link) continue;
        real demand = 0.0;
        if (link->kind == machine::TowBrakeKind::Channel) {
            demand = link->gain * std::max(channel_values[c.brake_channel], tow_brake_demand_[c.towing]);
        } else {
            const BodyId chassis = instances_[c.towed]->chassis();
            const Vec3 forward = backend.get_pose(chassis).orientation.rotate(Vec3::unit_x());
            if (dot(backend.get_motion(chassis).linear, forward) >= -link->reverse_release_speed_m_s) {
                const real push = dot(c.state.mean_force_on_towing_world, forward);
                demand = (push - link->deadband_n) / (link->full_force_n - link->deadband_n);
            }
        }
        c.brake_state += (std::clamp(demand, real(0.0), real(1.0)) - c.brake_state) * tick_dt / (link->lag_s + tick_dt);
        c.state.tow_brake = c.brake_state;
        tow_brake_demand_[c.towed] = c.brake_state;
    }
}

void System::set_lift_channels(HitchId id, std::size_t position, std::size_t mode, std::size_t draft) {
    Coupling& c = couplings_[id.index];
    c.lift_position_channel = position;
    c.lift_mode_channel = mode;
    c.lift_draft_channel = draft;
}

void System::update_lifts(const Island& island, const std::vector<real>& channel_values, real tick_dt) {
    for (const std::size_t ci : island.couplings) {
        Coupling& c = couplings_[ci];
        if (!c.desc.linkage) continue;
        const machine::LinkageGeometry& g = *c.desc.linkage;
        const machine::LinkageDesc& lift = c.desc.linkage_lift;
        machine::LinkageState& state = c.state.linkage;
        const real span = g.max_angle_rad - g.min_angle_rad;
        const real lever_angle = g.min_angle_rad + std::clamp(channel_values[c.lift_position_channel], real(0.0), real(1.0)) * span;
        const real stroke_rate = span / lift.full_stroke_s;
        const int mode = std::clamp(static_cast<int>(channel_values[c.lift_mode_channel] + 0.5), 0, 2);
        state.mode = static_cast<machine::LiftMode>(mode);

        c.lift_sensed_n += (state.lower_tension_n - c.lift_sensed_n) * tick_dt / (lift.draft_filter_s + tick_dt);
        state.sensed_draft_n = c.lift_sensed_n;

        real goal = lever_angle;
        if (state.mode == machine::LiftMode::Float) {
            c.lift_target = state.angle_rad;
            c.lift_target_rate = 0.0;
            c.lift_command = machine::LiftCommand{false, c.lift_target, 0.0};
            state.target_rad = c.lift_target;
            continue;
        }
        if (state.mode == machine::LiftMode::Draft) {
            const real setpoint = std::max(channel_values[c.lift_draft_channel], real(1.0));
            const real error = (c.lift_sensed_n - setpoint) / setpoint;
            const real rate = std::abs(error) <= lift.draft_band ? real(0.0) : std::clamp(lift.draft_gain_per_s * error, real(-1.0), real(1.0)) * stroke_rate;
            goal = std::clamp(c.lift_target + rate * tick_dt, lever_angle, g.max_angle_rad);
        }
        const real step = std::clamp(goal - c.lift_target, -stroke_rate * tick_dt, stroke_rate * tick_dt);
        c.lift_target += step;
        c.lift_target_rate = step / tick_dt;
        // The ground holds the implement above where the lift would have it: the held height must not run away below
        // it, or raising the lever would first have to wind all that back.
        if (state.angle_rad - c.lift_target > lift.follow_slack_rad) {
            c.lift_target = state.angle_rad - lift.follow_slack_rad;
            c.lift_target_rate = 0.0;
        }
        c.lift_command = machine::LiftCommand{true, c.lift_target, c.lift_target_rate};
        state.target_rad = c.lift_target;
    }
}

void System::update_pto_shafts(const Island& island) {
    // A self-propelled machine: the tool and the PTO are on one vehicle, no hitch between them.
    for (const std::size_t i : island.vehicles) {
        if (!instances_[i]->has_pto()) continue;
        for (machine::ToolRuntime& tool : tools_[i]) {
            if (!tool.desc().pto.present) continue;
            const real torque = tool.pto_load_nm(instances_[i]->pto_out_omega());
            instances_[i]->set_pto_load({tool.desc().pto.inertia_kgm2, torque});
            break;
        }
    }
    for (const std::size_t ci : island.couplings) {
        const Coupling& c = couplings_[ci];
        if (c.pto_tool == kNoTool) continue;
        machine::ToolRuntime& tool = tools_[c.towed][c.pto_tool];
        const real torque = tool.pto_load_nm(instances_[c.towing]->pto_out_omega());
        instances_[c.towing]->set_pto_load({tool.desc().pto.inertia_kgm2, torque});
    }
}

machine::PtoCheck System::check_pto(HitchId id) const {
    machine::PtoCheck check;
    if (!contains(id)) {
        check.message = "pto: not a live hitch";
        return check;
    }
    const Coupling& c = couplings_[id.index];
    const std::vector<machine::ToolRuntime>& tools = tools_[c.towed];
    const auto tool = std::find_if(tools.begin(), tools.end(), [](const machine::ToolRuntime& t) { return t.desc().pto.present; });
    const std::vector<machine::ToolRuntime>& own = tools_[c.towing];
    if (!instances_[c.towing]->has_pto()) check.message = "pto: the towing vehicle has no PTO";
    else if (std::any_of(own.begin(), own.end(), [](const machine::ToolRuntime& t) { return t.desc().pto.present; })) check.message = "pto: the towing vehicle's shaft turns its own tool";
    else if (tool == tools.end()) check.message = "pto: the towed vehicle has no tool on a shaft";
    else if (c.pto_tool != kNoTool) check.message = "pto: this hitch's shaft is on";
    else if (!instances_[c.towing]->pto_offers(tool->desc().pto.nominal_rpm)) check.message = format("pto: the tractor has no %.0f rpm mode", tool->desc().pto.nominal_rpm);
    check.ok = check.message.empty();
    return check;
}

void System::connect_pto(HitchId id) {
    const machine::PtoCheck check = check_pto(id);
    if (!check.ok) throw std::invalid_argument(check.message);
    Coupling& c = couplings_[id.index];
    const std::vector<machine::ToolRuntime>& tools = tools_[c.towed];
    for (std::size_t t = 0; t < tools.size(); ++t) {
        if (tools[t].desc().pto.present) {
            c.pto_tool = t;
            break;
        }
    }
    instances_[c.towing]->set_pto_load({tools[c.pto_tool].desc().pto.inertia_kgm2, 0.0});
}

void System::disconnect_pto(HitchId id) {
    Coupling& c = couplings_[id.index];
    if (c.pto_tool == kNoTool) return;
    instances_[c.towing]->set_pto_load({});
    tools_[c.towed][c.pto_tool].disconnect_shaft();
    c.pto_tool = kNoTool;
}

void System::drain_work(std::vector<machine::WorkQuad>& out) {
    const std::size_t first = out.size();
    for (std::vector<machine::ToolRuntime>& list : tools_) {
        for (machine::ToolRuntime& tool : list) tool.drain(out);
    }
    std::stable_sort(out.begin() + static_cast<std::ptrdiff_t>(first), out.end(), [](const machine::WorkQuad& a, const machine::WorkQuad& b) {
        if (a.tick != b.tick) return a.tick < b.tick;
        if (a.vehicle.index != b.vehicle.index) return a.vehicle.index < b.vehicle.index;
        return a.tool < b.tool;
    });
}

std::uint64_t System::work_dropped() const {
    std::uint64_t dropped = 0;
    for (const std::vector<machine::ToolRuntime>& list : tools_) {
        for (const machine::ToolRuntime& tool : list) dropped += tool.dropped();
    }
    return dropped;
}

void System::solve_coupling(Coupling& c, real h) {
    VehicleTickWork& wa = tick_work_[c.towing];
    VehicleTickWork& wb = tick_work_[c.towed];
    // Every other force of this substep, this coupling's own from an earlier pass excepted.
    const auto socket_body = [h](const VehicleTickWork& w, const Vec3& socket_local, const Quat& frame_local, const Vec3& own_force, const Vec3& own_torque) {
        machine::SocketBody b;
        b.socket_offset = w.pose_s.orientation.rotate(socket_local);
        b.socket_position = w.pose_s.position + b.socket_offset;
        b.hitch_frame = w.pose_s.orientation * frame_local;
        b.linear = w.twist_s.linear + ((w.force_s - own_force) * w.ip.inv_mass + w.a_ext) * h;
        b.angular = w.twist_s.angular + (w.ip.inv_inertia_world * (w.torque_s - own_torque) + w.alpha_ext) * h;
        b.inv_mass = w.ip.inv_mass;
        b.inv_inertia_world = w.ip.inv_inertia_world;
        return b;
    };
    if (c.desc.linkage) {
        const auto linkage_body = [h](const VehicleTickWork& w, const Quat& frame_local, const Vec3& own_force, const Vec3& own_torque) {
            machine::LinkageBody b;
            b.position = w.pose_s.position;
            b.orientation = w.pose_s.orientation;
            b.hitch_frame = w.pose_s.orientation * frame_local;
            b.linear = w.twist_s.linear + ((w.force_s - own_force) * w.ip.inv_mass + w.a_ext) * h;
            b.angular = w.twist_s.angular + (w.ip.inv_inertia_world * (w.torque_s - own_torque) + w.alpha_ext) * h;
            b.inv_mass = w.ip.inv_mass;
            b.inv_inertia_world = w.ip.inv_inertia_world;
            return b;
        };
        const machine::LinkageBody a = linkage_body(wa, c.desc.frame_towing_local, c.force, c.torque_towing);
        const machine::LinkageBody b = linkage_body(wb, c.desc.frame_towed_local, -c.force, c.torque_towed);
        c.linkage_last = machine::solve_linkage(a, b, *c.desc.linkage, c.desc.hitch, h, c.lift_command, latch_target(c, wa, h));
        c.last = c.linkage_last.wrench;
    } else {
        const machine::SocketBody a = socket_body(wa, c.desc.socket_towing_local, c.desc.frame_towing_local, c.force, c.torque_towing);
        const machine::SocketBody b = socket_body(wb, c.desc.socket_towed_local, c.desc.frame_towed_local, -c.force, c.torque_towed);
        c.last = machine::solve_socket_coupling(a, b, c.desc.hitch, h, latch_target(c, wa, h));
    }
    VehicleInstance::substep_add_wrench(wa, c.last.force_on_a - c.force, c.last.torque_on_a - c.torque_towing);
    VehicleInstance::substep_add_wrench(wb, c.force - c.last.force_on_a, c.last.torque_on_b - c.torque_towed);
    c.force = c.last.force_on_a;
    c.torque_towing = c.last.torque_on_a;
    c.torque_towed = c.last.torque_on_b;
}

void System::step_island(Island& island, IRigidBackend& backend, const std::vector<real>& channel_values, const VehicleTickEnv& env,
                         real tick_dt, real substep_rate_hz, std::span<const environment::VehicleAir> air) {
    update_tow_brakes(island, backend, channel_values, tick_dt);
    update_lifts(island, channel_values, tick_dt);
    for (const std::size_t i : island.vehicles) {
        VehicleTickEnv e = env;
        e.brake_link = tow_brake_demand_[i];
        if (i < air.size() && air[i].valid) {
            const environment::VehicleAir& a = air[i];
            e.ambient = AmbientSample{a.pressure_pa, a.temperature_k, a.vapour_pressure_pa};
            e.air_density = a.density_kg_m3;
            e.wind_world = a.wind_world;
        }
        tick_work_[i] = VehicleTickWork{};
        instances_[i]->tick_begin(tick_work_[i], backend, channel_values, e, tick_dt, substep_rate_hz);
        for (machine::ToolRuntime& tool : tools_[i]) tool.tick_begin(tick_work_[i].pose_s, backend, island.chassis, soil_.get());
    }
    update_pto_shafts(island);
    for (const std::size_t ci : island.couplings) {
        Coupling& c = couplings_[ci];
        c.state.mean_force_on_towing_world = Vec3::zero();
        c.state.max_error_m = 0.0;
        c.state.linkage.lift_force_n = c.state.linkage.lower_tension_n = c.state.linkage.top_force_n = 0.0;
        c.state.linkage.saturated = false;
    }
    const std::size_t num_substeps = tick_work_[island.vehicles.front()].num_substeps;
    const real h = tick_work_[island.vehicles.front()].substep_dt;
    for (std::size_t s = 0; s < num_substeps; ++s) {
        for (const std::size_t i : island.vehicles) {
            VehicleTickWork& tw = tick_work_[i];
            instances_[i]->substep_forces(tw, s, backend);
            for (machine::ToolRuntime& tool : tools_[i]) {
                Vec3 force, torque;
                tool.substep(tw.pose_s, tw.twist_s, force, torque);
                VehicleInstance::substep_add_wrench(tw, force, torque);
            }
        }
        for (const std::size_t ci : island.couplings) {
            Coupling& c = couplings_[ci];
            c.force = c.torque_towing = c.torque_towed = Vec3::zero();
            // The lift's target moves along the tick instead of in a step.
            if (c.desc.linkage) c.lift_command.target_rad = c.lift_target - c.lift_target_rate * h * static_cast<real>(num_substeps - 1 - s);
        }
        const int passes = island.couplings.size() > 1 ? coupling_passes_ : 1;
        for (int pass = 0; pass < passes; ++pass) {
            for (const std::size_t ci : island.couplings) solve_coupling(couplings_[ci], h);
        }
        for (const std::size_t ci : island.couplings) {
            Coupling& c = couplings_[ci];
            c.state.mean_force_on_towing_world += c.last.force_on_a;
            if (c.desc.linkage) {
                const machine::LinkageSolve& l = c.linkage_last;
                machine::LinkageState& state = c.state.linkage;
                state.angle_rad = l.lift_angle;
                state.lift_force_n += l.lift_torque_nm / c.desc.linkage->lower_length_m;
                state.lower_tension_n += l.lower_tension_n;
                state.top_force_n += l.top_compression_n;
                state.saturated = state.saturated || l.saturated;
            }
            c.state.error_world = c.last.error;
            c.state.max_error_m = std::max(c.state.max_error_m, c.last.error.length());
            c.state.yaw_rad = c.last.angle[0];
            c.state.pitch_rad = c.last.angle[1];
            c.state.roll_rad = c.last.angle[2];
            c.state.work_j += c.last.work_j;
            if (c.latch_left > 0) --c.latch_left;
            c.state.latch_progress = 1.0 - static_cast<real>(c.latch_left) / static_cast<real>(c.latch_total);
        }
        for (const std::size_t i : island.vehicles) VehicleInstance::substep_advance(tick_work_[i]);
    }
    for (const std::size_t ci : island.couplings) {
        Coupling& c = couplings_[ci];
        const real per_substep = 1.0 / static_cast<real>(num_substeps);
        c.state.mean_force_on_towing_world /= static_cast<real>(num_substeps);
        if (c.desc.linkage) {
            machine::LinkageState& state = c.state.linkage;
            const machine::LinkageGeometry& g = *c.desc.linkage;
            state.lift_force_n *= per_substep;
            state.lower_tension_n *= per_substep;
            state.top_force_n *= per_substep;
            state.position = (state.angle_rad - g.min_angle_rad) / (g.max_angle_rad - g.min_angle_rad);
        }
    }
    for (const std::size_t i : island.vehicles) {
        for (machine::ToolRuntime& tool : tools_[i]) tool.tick_end(tick_work_[i].pose_s, work_tick_);
        instances_[i]->tick_end(tick_work_[i], backend);
    }
}

void System::step_all(IRigidBackend& backend, jobs::JobSystem& job_system, const std::vector<real>& channel_values,
                       const VehicleTickEnv& env, real tick_dt, real substep_rate_hz,
                       std::span<const environment::VehicleAir> air) {
    if (instances_.empty()) return;
    const std::size_t n = instances_.size();
    ++work_tick_;
    for(std::size_t i=0;i<n;++i) {
        if(retired_[i]!=0) {
            aero_wakes_[i]=aero::WakeSource{};
            aero_wakes_[i].desc.length_m=0;
            continue;
        }
        const auto body=instances_[i]->chassis();
        aero_wakes_[i]={body,backend.get_pose(body).position,backend.get_motion(body).linear,instances_[i]->aero_wake()};
    }
    VehicleTickEnv tick_env=env; tick_env.wakes=aero_wakes_;

    const int workers = std::max(1, job_system.worker_count());
    if (alive_couplings_ > 0 || tool_vehicles_ > 0) {
        // One job per island; each job touches only its own vehicles, couplings and tick_work_ entries.
        const std::size_t count = islands_.size();
        const std::size_t island_grain = std::max<std::size_t>(1, (count + static_cast<std::size_t>(workers) * 4 - 1) / (static_cast<std::size_t>(workers) * 4));
        job_system.parallel_for(count, island_grain, [this, &backend, &channel_values, &tick_env, air, tick_dt, substep_rate_hz](std::size_t begin, std::size_t end) {
            for (std::size_t i = begin; i < end; ++i) step_island(islands_[i], backend, channel_values, tick_env, tick_dt, substep_rate_hz, air);
        });
        return;
    }
    const std::size_t grain = std::max<std::size_t>(1, (n + static_cast<std::size_t>(workers) * 4 - 1) / (static_cast<std::size_t>(workers) * 4));

    job_system.parallel_for(n, grain, [this, &backend, &channel_values, &tick_env, air, tick_dt, substep_rate_hz](std::size_t begin, std::size_t end) {
        for (std::size_t i = begin; i < end; ++i) {
            if (retired_[i] != 0) continue;
            if (i < air.size() && air[i].valid) {
                // W4: this vehicle's own atmosphere and wind (a stack copy, no allocation).
                const environment::VehicleAir& a = air[i];
                VehicleTickEnv e = tick_env;
                e.ambient = AmbientSample{a.pressure_pa, a.temperature_k, a.vapour_pressure_pa};
                e.air_density = a.density_kg_m3;
                e.wind_world = a.wind_world;
                instances_[i]->step_tick(backend, channel_values, e, tick_dt, substep_rate_hz);
            } else {
                instances_[i]->step_tick(backend, channel_values, tick_env, tick_dt, substep_rate_hz);
            }
        }
    });
}

void System::pre_backend_step(IRigidBackend& backend) {
    for (const Island& island : islands_) {
        if (island.couplings.empty()) continue;
        for (const std::size_t i : island.vehicles) instances_[i]->align_backend_with_shadow(tick_work_[i], backend);
    }
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
    for (std::size_t i = 0; i < instances_.size(); ++i) {
        if (retired_[i] == 0) h = instances_[i]->fold_state_hash(h);
    }
    // Only a live coupling folds in: a World without one hashes exactly as before. The solve itself carries nothing
    // over a tick but the latch; a tow brake carries its lag, and an overrun brake reads last tick's hitch force.
    for (std::size_t i = 0; i < couplings_.size(); ++i) {
        const Coupling& c = couplings_[i];
        if (!c.alive) continue;
        h = fold(h, i);
        h = fold(h, c.towing);
        h = fold(h, c.towed);
        h = fold(h, c.latch_left);
        h = fold(h, c.latch_total);
        h = fold(h, c.latch_error_local);
        h = fold(h, c.latch_rate_local);
        h = fold(h, c.latch_angle);
        h = fold(h, c.latch_angle_rate);
        const std::optional<machine::TowBrakeDesc>& link = instances_[c.towed]->tow_brake();
        if (link) h = fold(h, c.brake_state);
        if (link && link->kind == machine::TowBrakeKind::Overrun) h = fold(h, c.state.mean_force_on_towing_world);
        if (c.desc.linkage) {
            h = fold(h, c.lift_target);
            h = fold(h, c.lift_sensed_n);
            h = fold(h, c.state.linkage.lower_tension_n);
            h = fold(h, c.state.linkage.angle_rad);
        }
        if (c.pto_tool != kNoTool) h = fold(h, c.pto_tool);
    }
    // Likewise only a vehicle with tools.
    for (const std::vector<machine::ToolRuntime>& list : tools_) {
        for (const machine::ToolRuntime& tool : list) h = tool.fold_state_hash(h);
    }
    return h;
}

} // namespace ps::vehicle
