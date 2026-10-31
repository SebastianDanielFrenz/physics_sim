#include "system.h"

#include <algorithm>
#include <numeric>
#include <stdexcept>

namespace ps::vehicle {

VehicleId System::create(const VehicleDesc& desc, BodyId chassis, const io::SurfaceTable* surfaces,
                          const ControlChannelIndices& control_indices) {
    instances_.push_back(std::make_unique<VehicleInstance>(desc, chassis, surfaces, control_indices));
    aero_wakes_.resize(instances_.size());
    tick_work_.resize(instances_.size());
    const auto vehicle_index = static_cast<std::int32_t>(instances_.size() - 1);
    if (vehicle_for_body_index_.size() <= chassis.index) vehicle_for_body_index_.resize(chassis.index + 1, -1);
    vehicle_for_body_index_[chassis.index] = vehicle_index;
    if (!couplings_.empty()) rebuild_islands();
    return VehicleId{static_cast<id_index_t>(instances_.size() - 1), 1};
}

HitchId System::create_coupling(const machine::CouplingDesc& desc, IRigidBackend& backend) {
    if (!contains(desc.towing) || !contains(desc.towed)) throw std::invalid_argument("create_coupling: unknown vehicle");
    if (desc.towing == desc.towed) throw std::invalid_argument("create_coupling: a vehicle cannot tow itself");
    machine::validate_hitch_class(desc.hitch);
    Coupling c;
    c.desc = desc;
    c.towing = desc.towing.index;
    c.towed = desc.towed.index;
    couplings_.push_back(c);
    rebuild_islands();
    apply_collision_pairs(backend);
    return HitchId{static_cast<id_index_t>(couplings_.size() - 1), 1};
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
        const std::size_t a = find(c.towing), b = find(c.towed);
        parent[std::max(a, b)] = std::min(a, b);
    }
    islands_.clear();
    std::vector<std::size_t> island_of_root(instances_.size(), instances_.size());
    for (std::size_t i = 0; i < instances_.size(); ++i) {
        const std::size_t root = find(i);
        if (island_of_root[root] == instances_.size()) {
            island_of_root[root] = islands_.size();
            islands_.emplace_back();
        }
        islands_[island_of_root[root]].vehicles.push_back(i);
    }
    for (std::size_t i = 0; i < couplings_.size(); ++i) islands_[island_of_root[find(couplings_[i].towing)]].couplings.push_back(i);

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
    for (const Coupling& c : couplings_) pairs.emplace_back(instances_[c.towing]->chassis(), instances_[c.towed]->chassis());
    backend.set_disabled_collision_pairs(pairs);
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
    c.last = machine::solve_socket_coupling(a, b, c.desc.hitch, h);
    VehicleInstance::substep_add_wrench(wa, c.last.force_on_a - c.force, c.last.torque_on_a - c.torque_towing);
    VehicleInstance::substep_add_wrench(wb, c.force - c.last.force_on_a, c.last.torque_on_b - c.torque_towed);
    c.force = c.last.force_on_a;
    c.torque_towing = c.last.torque_on_a;
    c.torque_towed = c.last.torque_on_b;
}

void System::step_island(Island& island, IRigidBackend& backend, const std::vector<real>& channel_values, const VehicleTickEnv& env,
                         real tick_dt, real substep_rate_hz, std::span<const environment::VehicleAir> air) {
    for (const std::size_t i : island.vehicles) {
        VehicleTickEnv e = env;
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
        const auto body=instances_[i]->chassis();
        aero_wakes_[i]={body,backend.get_pose(body).position,backend.get_motion(body).linear,instances_[i]->aero_wake()};
    }
    VehicleTickEnv tick_env=env; tick_env.wakes=aero_wakes_;

    const int workers = std::max(1, job_system.worker_count());
    if (!couplings_.empty()) {
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
    for (const auto& inst : instances_) h = inst->fold_state_hash(h);
    return h;
}

} // namespace ps::vehicle
