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
    const auto vehicle_index = static_cast<std::int32_t>(instances_.size() - 1);
    if (vehicle_for_body_index_.size() <= chassis.index) vehicle_for_body_index_.resize(chassis.index + 1, -1);
    vehicle_for_body_index_[chassis.index] = vehicle_index;
    if (alive_couplings_ > 0) rebuild_islands();
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
    const machine::SocketBody a = socket_body_now(backend, towing, desc.socket_towing_local, desc.frame_towing_local);
    const machine::SocketBody b = socket_body_now(backend, instances_[c.towed]->chassis(), desc.socket_towed_local, desc.frame_towed_local);
    const machine::HitchMeasure m = machine::measure_hitch(a, b, desc.hitch);
    const Quat towing_rotation = backend.get_pose(towing).orientation;
    c.latch_total = c.latch_left = std::max(1, static_cast<int>(desc.hitch.latch_time_s * substep_rate_hz + 0.5));
    c.latch_error_local = towing_rotation.conjugate().rotate(m.error);
    c.latch_rate_local = towing_rotation.conjugate().rotate(m.error_rate - cross(a.angular, m.error));
    const std::array<const machine::HitchAxis*, 3> axes = {&desc.hitch.yaw, &desc.hitch.pitch, &desc.hitch.roll};
    for (std::size_t k = 0; k < 3; ++k) {
        if (axes[k]->mode != machine::HitchAxisMode::Locked) continue;
        c.latch_angle[k] = m.angle[k];
        c.latch_angle_rate[k] = m.angle_rate[k];
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
    if (!problem.empty()) return d;
    d.towing = towing;
    d.towed = towed;
    d.socket_towing_local = a->position_local;
    d.socket_towed_local = b->position_local;
    d.hitch = a->hitch;
    d.towing_point = a_name;
    d.towed_point = b_name;
    return d;
}

machine::HitchLoads System::hitch_loads(HitchId id, const IRigidBackend& backend) const {
    const Coupling& c = couplings_[id.index];
    machine::HitchLoads loads;
    loads.tongue_load_n = -c.state.mean_force_on_towing_world.z;
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

    for (const Island& island : islands_) {
        std::vector<BodyId> chassis;
        if (!island.couplings.empty()) {
            for (const std::size_t i : island.vehicles) chassis.push_back(instances_[i]->chassis());
        }
        for (const std::size_t i : island.vehicles) instances_[i]->set_cast_exclusions(chassis);
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
    t.error = towing.pose_s.orientation.rotate(local);
    t.error_rate = cross(towing.twist_s.angular, t.error) + towing.pose_s.orientation.rotate(local_rate);
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
    const machine::SocketBody a = socket_body(wa, c.desc.socket_towing_local, c.desc.frame_towing_local, c.force, c.torque_towing);
    const machine::SocketBody b = socket_body(wb, c.desc.socket_towed_local, c.desc.frame_towed_local, -c.force, c.torque_towed);
    c.last = machine::solve_socket_coupling(a, b, c.desc.hitch, h, latch_target(c, wa, h));
    VehicleInstance::substep_add_wrench(wa, c.last.force_on_a - c.force, c.last.torque_on_a - c.torque_towing);
    VehicleInstance::substep_add_wrench(wb, c.force - c.last.force_on_a, c.last.torque_on_b - c.torque_towed);
    c.force = c.last.force_on_a;
    c.torque_towing = c.last.torque_on_a;
    c.torque_towed = c.last.torque_on_b;
}

void System::step_island(Island& island, IRigidBackend& backend, const std::vector<real>& channel_values, const VehicleTickEnv& env,
                         real tick_dt, real substep_rate_hz, std::span<const environment::VehicleAir> air) {
    update_tow_brakes(island, backend, channel_values, tick_dt);
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
    }
    for (const std::size_t ci : island.couplings) {
        couplings_[ci].state.mean_force_on_towing_world = Vec3::zero();
        couplings_[ci].state.max_error_m = 0.0;
    }
    const std::size_t num_substeps = tick_work_[island.vehicles.front()].num_substeps;
    const real h = tick_work_[island.vehicles.front()].substep_dt;
    for (std::size_t s = 0; s < num_substeps; ++s) {
        for (const std::size_t i : island.vehicles) instances_[i]->substep_forces(tick_work_[i], s, backend);
        for (const std::size_t ci : island.couplings) {
            Coupling& c = couplings_[ci];
            c.force = c.torque_towing = c.torque_towed = Vec3::zero();
        }
        const int passes = island.couplings.size() > 1 ? coupling_passes_ : 1;
        for (int pass = 0; pass < passes; ++pass) {
            for (const std::size_t ci : island.couplings) solve_coupling(couplings_[ci], h);
        }
        for (const std::size_t ci : island.couplings) {
            Coupling& c = couplings_[ci];
            c.state.mean_force_on_towing_world += c.last.force_on_a;
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
    for (const std::size_t ci : island.couplings) couplings_[ci].state.mean_force_on_towing_world /= static_cast<real>(num_substeps);
    for (const std::size_t i : island.vehicles) instances_[i]->tick_end(tick_work_[i], backend);
}

void System::step_all(IRigidBackend& backend, jobs::JobSystem& job_system, const std::vector<real>& channel_values,
                       const VehicleTickEnv& env, real tick_dt, real substep_rate_hz,
                       std::span<const environment::VehicleAir> air) {
    if (instances_.empty()) return;
    const std::size_t n = instances_.size();
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
    if (alive_couplings_ > 0) {
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
    }
    return h;
}

} // namespace ps::vehicle
