#include "powertrain.h"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <stdexcept>
#include <unordered_map>
#include <variant>

namespace ps::drivetrain {
namespace {

real sign(real x) { return x > real(0.0) ? real(1.0) : (x < real(0.0) ? real(-1.0) : real(0.0)); }

constexpr real kPi = 3.14159265358979323846;
// rpm -> rad/s (inverse of engine_torque_map.cpp's own kRadPerSecToRpm).
real rpm_to_rad_per_s(real rpm) { return rpm * (real(2.0) * kPi) / real(60.0); }

// Build-time-only union-find over PORT indices (plan section 4.1 step 3),
// same "smaller root wins" rule as network.cpp's own (unrelated) NodeId
// union-find - ordinary std::vector allocation is fine here (Powertrain's
// constructor runs on the sim thread, outside any job, PLAN.md D8).
class UnionFind {
public:
    explicit UnionFind(int n) : parent_(static_cast<std::size_t>(n)) { std::iota(parent_.begin(), parent_.end(), 0); }
    int find(int x) {
        while (parent_[static_cast<std::size_t>(x)] != x) {
            parent_[static_cast<std::size_t>(x)] = parent_[static_cast<std::size_t>(parent_[static_cast<std::size_t>(x)])];
            x = parent_[static_cast<std::size_t>(x)];
        }
        return x;
    }
    void unite(int a, int b) {
        const int ra = find(a), rb = find(b);
        if (ra == rb) return;
        if (ra < rb) parent_[static_cast<std::size_t>(rb)] = ra;
        else parent_[static_cast<std::size_t>(ra)] = rb;
    }

private:
    std::vector<int> parent_;
};

std::string component_port_key(const std::string& id, const std::string& port) { return id + "." + port; }
std::string wheel_port_key(const std::string& name) { return "wheel:" + name; }

// EngineTorqueMap/Clutch/Gearbox live in core/src/drivetrain (framework-
// agnostic classes, commits 2-4) with their OWN Desc types, deliberately
// named differently from the public plan section 4.1 descriptors (that
// file's own top comment). Converts field-for-field; every field name below
// matches 1:1 (commit 6 renames were confined to the public header only).
EngineTorqueMap::Desc to_engine_desc(const TorqueMapEngineDesc& e) {
    EngineTorqueMap::Desc d;
    d.wot_torque_nm_vs_rpm = e.wot_torque_nm_vs_rpm;
    d.motoring_torque_nm_vs_rpm = e.motoring_torque_nm_vs_rpm;
    d.inertia_kgm2 = e.inertia_kgm2;
    d.breakaway_torque_nm = e.breakaway_torque_nm;
    d.idle_rpm = e.idle_rpm;
    d.stall_rpm = e.stall_rpm;
    d.fire_rpm = e.fire_rpm;
    d.start_complete_rpm = e.start_complete_rpm;
    d.dfco_enable_rpm = e.dfco_enable_rpm;
    d.dfco_reenable_rpm = e.dfco_reenable_rpm;
    d.idle.kp_per_rpm = e.idle.kp_per_rpm;
    d.idle.ki_per_rpm_s = e.idle.ki_per_rpm_s;
    d.idle.max_throttle = e.idle.max_throttle;
    d.limiter.type = e.limiter.type == LimiterType::FuelCut ? EngineTorqueMap::LimiterType::FuelCut : EngineTorqueMap::LimiterType::SoftCut;
    d.limiter.rpm = e.limiter.rpm;
    d.limiter.hysteresis_rpm = e.limiter.hysteresis_rpm;
    d.limiter.band_rpm = e.limiter.band_rpm;
    d.starter.stall_torque_nm = e.starter.stall_torque_nm;
    d.starter.free_speed_rpm = e.starter.free_speed_rpm;
    d.starter.interlock = e.starter.interlock == StarterDesc::Interlock::None ? EngineTorqueMap::StarterInterlock::None
                                                                               : EngineTorqueMap::StarterInterlock::ClutchOrNeutral;
    d.initial_state = e.initial_state == EngineInitialState::Running ? EngineTorqueMap::InitialState::Running : EngineTorqueMap::InitialState::Off;
    return d;
}

Clutch::Desc to_clutch_desc(const ClutchDesc& c) {
    Clutch::Desc d;
    d.capacity_nm = c.capacity_nm;
    d.static_factor = c.static_factor;
    d.engagement_vs_pedal = c.engagement_vs_pedal;
    d.in_inertia_kgm2 = c.in_inertia_kgm2;
    d.out_inertia_kgm2 = c.out_inertia_kgm2;
    return d;
}

Gearbox::Desc to_gearbox_desc(const GearboxDesc& g) {
    Gearbox::Desc d;
    d.type = g.type == GearboxType::Manual ? GearboxKind::Manual : GearboxKind::Sequential;
    d.forward_ratios = g.forward_ratios;
    d.reverse_ratios = g.reverse_ratios;
    d.forward_efficiency = g.forward_efficiency;
    d.reverse_efficiency = g.reverse_efficiency;
    d.input_inertia_kgm2 = g.input_inertia_kgm2;
    d.output_inertia_kgm2 = g.output_inertia_kgm2;
    d.synchro.forward_capacity_nm = g.synchro.forward_capacity_nm;
    d.synchro.reverse_capacity_nm = g.synchro.reverse_capacity_nm;
    d.synchro.static_factor = g.synchro.static_factor;
    d.synchro.max_dog_mismatch_rad_s = g.synchro.max_dog_mismatch_rad_s;
    d.neutral_phase_s = g.neutral_phase_s;
    d.dog_release_torque_nm = g.dog_release_torque_nm;
    d.grind_after_s = g.grind_after_s;
    d.abort_after_s = g.abort_after_s;
    d.initial_gear = g.initial_gear;
    return d;
}

} // namespace

Powertrain::Powertrain(const PowertrainDesc& pt, const BrakeSystemDesc& brakes, const std::vector<vehicle::WheelDesc>& wheels,
                        const std::vector<ControllerDesc>& controllers, const std::string& vehicle_name) {
    rotor_reaction_ = pt.rotor_reaction;

    // --- Phase A: port table (plan section 4.1 step 1 / 5.1) --------------
    // Walk components in array order, each type's ports in FIXED order
    // (engine: crank; clutch: in, out; gearbox: in, out; differential:
    // carrier, left, right; gear_pair: in, out), then one hub port per
    // wheel in VehicleDesc::wheels order. port_axis_local tracks each
    // port's OWN rotor-reaction axis (plan section 6.7) - zero for a wheel
    // hub port (handled separately by VehicleInstance) and for any
    // component port whose desc leaves its axis field at the zero default.
    std::vector<std::string> port_key;
    std::vector<real> port_inertia;
    std::vector<Vec3> port_axis_local;
    std::unordered_map<std::string, int> port_index_of;

    auto add_port = [&](const std::string& key, real inertia, Vec3 axis_local) -> int {
        if (port_index_of.count(key)) {
            throw std::runtime_error("vehicle '" + vehicle_name + "': powertrain port '" + key + "' is declared more than once");
        }
        const int idx = static_cast<int>(port_key.size());
        port_index_of.emplace(key, idx);
        port_key.push_back(key);
        port_inertia.push_back(inertia);
        port_axis_local.push_back(axis_local);
        return idx;
    };

    std::unordered_map<std::string, int> component_index_of; // component id -> index into pt.components
    int engine_component_index = -1, clutch_component_index = -1, gearbox_component_index = -1;
    for (std::size_t ci = 0; ci < pt.components.size(); ++ci) {
        const ComponentDesc& c = pt.components[ci];
        if (c.id.empty()) throw std::runtime_error("vehicle '" + vehicle_name + "': a powertrain component has an empty id");
        if (!component_index_of.emplace(c.id, static_cast<int>(ci)).second) {
            throw std::runtime_error("vehicle '" + vehicle_name + "': powertrain component id '" + c.id + "' is not unique");
        }
        if (const auto* e = std::get_if<TorqueMapEngineDesc>(&c.params)) {
            if (engine_component_index >= 0) {
                throw std::runtime_error("vehicle '" + vehicle_name + "': more than one engine component - I3 supports at most one");
            }
            engine_component_index = static_cast<int>(ci);
            add_port(component_port_key(c.id, "crank"), e->inertia_kgm2, e->crank_axis_local);
        } else if (const auto* cl = std::get_if<ClutchDesc>(&c.params)) {
            if (clutch_component_index >= 0) {
                throw std::runtime_error("vehicle '" + vehicle_name + "': more than one clutch component - I3 supports at most one");
            }
            clutch_component_index = static_cast<int>(ci);
            add_port(component_port_key(c.id, "in"), cl->in_inertia_kgm2, cl->axis_local);
            add_port(component_port_key(c.id, "out"), cl->out_inertia_kgm2, cl->axis_local);
        } else if (const auto* g = std::get_if<GearboxDesc>(&c.params)) {
            if (gearbox_component_index >= 0) {
                throw std::runtime_error("vehicle '" + vehicle_name + "': more than one gearbox component - I3 supports at most one");
            }
            gearbox_component_index = static_cast<int>(ci);
            add_port(component_port_key(c.id, "in"), g->input_inertia_kgm2, g->input_axis_local);
            add_port(component_port_key(c.id, "out"), g->output_inertia_kgm2, g->output_axis_local);
        } else if (const auto* gp = std::get_if<GearPairDesc>(&c.params)) {
            add_port(component_port_key(c.id, "in"), gp->inertia_in_kgm2, gp->in_axis_local);
            add_port(component_port_key(c.id, "out"), gp->inertia_out_kgm2, gp->out_axis_local);
        } else if (const auto* d = std::get_if<DifferentialDesc>(&c.params)) {
            add_port(component_port_key(c.id, "carrier"), d->carrier_inertia_kgm2, d->carrier_axis_local);
            add_port(component_port_key(c.id, "left"), real(0.0), Vec3::zero());
            add_port(component_port_key(c.id, "right"), real(0.0), Vec3::zero());
        }
    }

    std::vector<int> wheel_port_index(wheels.size(), -1);
    for (std::size_t wi = 0; wi < wheels.size(); ++wi) {
        if (wheels[wi].wheel_inertia <= real(0.0)) {
            throw std::runtime_error("vehicle '" + vehicle_name + "': wheel '" + wheels[wi].name +
                                      "' has non-positive wheel_inertia (needed as the drivetrain hub's own inertia)");
        }
        wheel_port_index[wi] = add_port(wheel_port_key(wheels[wi].name), wheels[wi].wheel_inertia, Vec3::zero());
    }

    const int port_count = static_cast<int>(port_key.size());

    // --- Phase B: shafts -> union-find over ports, with validation --------
    UnionFind uf(std::max(port_count, 1));
    std::vector<std::uint8_t> port_in_any_shaft(static_cast<std::size_t>(port_count), 0);

    auto resolve_port = [&](const drivetrain::PortRef& ref) -> int {
        const std::string key = ref.is_wheel ? wheel_port_key(ref.component) : component_port_key(ref.component, ref.port);
        const auto it = port_index_of.find(key);
        if (it == port_index_of.end()) {
            throw std::runtime_error("vehicle '" + vehicle_name + "': powertrain shaft references unknown port '" + key + "'");
        }
        return it->second;
    };

    for (const ShaftDesc& shaft : pt.shafts) {
        if (shaft.stiffness != real(0.0)) {
            throw std::runtime_error("vehicle '" + vehicle_name + "': powertrain shaft has non-zero stiffness (I4 adds compliant shafts)");
        }
        std::vector<int> ports_here;
        ports_here.reserve(shaft.ports.size());
        for (const drivetrain::PortRef& ref : shaft.ports) {
            const int idx = resolve_port(ref);
            if (port_in_any_shaft[static_cast<std::size_t>(idx)]) {
                throw std::runtime_error("vehicle '" + vehicle_name + "': powertrain port '" + port_key[static_cast<std::size_t>(idx)] +
                                          "' is connected by more than one shaft, or twice in one shaft");
            }
            port_in_any_shaft[static_cast<std::size_t>(idx)] = 1;
            ports_here.push_back(idx);
        }
        for (std::size_t i = 1; i < ports_here.size(); ++i) uf.unite(ports_here[0], ports_here[i]);
    }

    for (int p = 0; p < port_count; ++p) {
        const bool is_wheel_port = port_key[static_cast<std::size_t>(p)].rfind("wheel:", 0) == 0;
        if (!is_wheel_port && !port_in_any_shaft[static_cast<std::size_t>(p)]) {
            throw std::runtime_error("vehicle '" + vehicle_name + "': powertrain port '" + port_key[static_cast<std::size_t>(p)] +
                                      "' is not connected by any shaft");
        }
    }

    // --- Phase D: per-root summed inertia ----------------------------------
    std::unordered_map<int, real> root_inertia;
    for (int p = 0; p < port_count; ++p) {
        const int root = uf.find(p);
        root_inertia[root] += port_inertia[static_cast<std::size_t>(p)];
    }

    // --- Phase E: number nodes by walking ports in port-index order -------
    NetworkBuilder builder;
    std::unordered_map<int, NodeId> node_of_root;
    std::vector<NodeId> node_of_port(static_cast<std::size_t>(port_count));
    for (int p = 0; p < port_count; ++p) {
        const int root = uf.find(p);
        auto it = node_of_root.find(root);
        if (it == node_of_root.end()) {
            const NodeId nid = builder.add_node(root_inertia[root]);
            it = node_of_root.emplace(root, nid).first;
        }
        node_of_port[static_cast<std::size_t>(p)] = it->second;
    }

    auto node_for_key = [&](const std::string& key) -> NodeId {
        const auto it = port_index_of.find(key);
        if (it == port_index_of.end()) {
            throw std::runtime_error("vehicle '" + vehicle_name + "': internal error resolving powertrain port '" + key + "'");
        }
        return node_of_port[static_cast<std::size_t>(it->second)];
    };

    // --- Phase F: topology states (plan section 5.2) -----------------------
    // Sorted order R_m..R_1, N, F_1..F_n (Gearbox::state_index_for_position's
    // own convention: index = reverse_ratios.size() + position). No gearbox
    // -> exactly one state (I2's shape).
    const GearboxDesc* gearbox_desc = gearbox_component_index >= 0 ? std::get_if<GearboxDesc>(&pt.components[static_cast<std::size_t>(gearbox_component_index)].params) : nullptr;
    const int num_states = gearbox_desc ? static_cast<int>(gearbox_desc->reverse_ratios.size()) + 1 + static_cast<int>(gearbox_desc->forward_ratios.size()) : 1;
    // Built here (not Phase K below) so the state loop just below can call
    // ratio_for_position directly instead of constructing a throwaway
    // Gearbox per state.
    if (gearbox_desc) gearbox_ = std::make_unique<Gearbox>(to_gearbox_desc(*gearbox_desc));

    std::unordered_map<std::uint16_t, int> rigid_row_touch_count; // NodeId::index -> how many rigid-row TERMS touch it (across every state)
    auto register_rigid_row = [&](int state, RigidRow row) {
        for (const auto& [node_index, coef] : row.terms) {
            (void)coef;
            ++rigid_row_touch_count[static_cast<std::uint16_t>(node_index)];
        }
        builder.add_rigid_row(state, std::move(row));
    };

    NodeId diff_carrier_n{}, diff_left_n{}, diff_right_n{};
    bool has_diff = false;
    real diff_ratio = real(1.0);
    for (const ComponentDesc& c : pt.components) {
        if (const auto* d = std::get_if<DifferentialDesc>(&c.params)) {
            if (has_diff) throw std::runtime_error("vehicle '" + vehicle_name + "': more than one differential component - I3 supports at most one");
            has_diff = true;
            diff_ratio = d->ratio;
            diff_carrier_n = node_for_key(component_port_key(c.id, "carrier"));
            diff_left_n = node_for_key(component_port_key(c.id, "left"));
            diff_right_n = node_for_key(component_port_key(c.id, "right"));
        }
    }

    NodeId gearbox_in_n{}, gearbox_out_n{};
    if (gearbox_desc) {
        gearbox_in_n = node_for_key(component_port_key(pt.components[static_cast<std::size_t>(gearbox_component_index)].id, "in"));
        gearbox_out_n = node_for_key(component_port_key(pt.components[static_cast<std::size_t>(gearbox_component_index)].id, "out"));
    }

    for (int state = 0; state < num_states; ++state) {
        if (has_diff) {
            RigidRow row;
            row.terms.emplace_back(diff_carrier_n.index, real(1.0));
            row.terms.emplace_back(diff_left_n.index, -diff_ratio / real(2.0));
            row.terms.emplace_back(diff_right_n.index, -diff_ratio / real(2.0));
            register_rigid_row(state, std::move(row));
        }
        if (gearbox_desc) {
            const int position = state - static_cast<int>(gearbox_desc->reverse_ratios.size());
            if (position != 0) {
                const real ratio = gearbox_->ratio_for_position(position);
                RigidRow row;
                row.terms.emplace_back(gearbox_in_n.index, real(1.0));
                row.terms.emplace_back(gearbox_out_n.index, -ratio);
                register_rigid_row(state, std::move(row));
            }
        }
    }

    // gear_pair: replicated across EVERY state (plan section 5.1/5.2 - its
    // own rigid coupling must hold regardless of the gearbox's position),
    // even though no I3 data ships one alongside a multi-state gearbox.
    for (const ComponentDesc& c : pt.components) {
        if (const auto* g = std::get_if<GearPairDesc>(&c.params)) {
            const NodeId in_n = node_for_key(component_port_key(c.id, "in"));
            const NodeId out_n = node_for_key(component_port_key(c.id, "out"));
            for (int state = 0; state < num_states; ++state) {
                RigidRow row;
                row.terms.emplace_back(in_n.index, real(1.0));
                row.terms.emplace_back(out_n.index, -g->ratio);
                register_rigid_row(state, std::move(row));
            }
        }
    }

    // gear_pair efficiency < 1 needs a single rigid coupling at its input
    // node (plan section 5.4's reconstruction-node rule) - touched exactly
    // `num_states` times by ITS OWN row above if nothing else shares that
    // node (gearbox/differential/another gear_pair never do in any shipped
    // I3 graph).
    for (const ComponentDesc& c : pt.components) {
        const auto* g = std::get_if<GearPairDesc>(&c.params);
        if (g == nullptr || g->efficiency >= real(1.0)) continue;
        const NodeId in_n = node_for_key(component_port_key(c.id, "in"));
        const int touches = rigid_row_touch_count[in_n.index];
        if (touches != num_states) {
            throw std::runtime_error("vehicle '" + vehicle_name + "': gear_pair '" + c.id +
                                      "' has efficiency < 1 but its input node carries another rigid row - "
                                      "gear efficiency needs a single rigid coupling at its input node");
        }
    }

    // --- Phase G: friction rows (clutch, engine-to-ground, gearbox synchro) ---
    FrictionRowId clutch_row{}, engine_row{}, synchro_row{};
    if (clutch_component_index >= 0) {
        const std::string& cid = pt.components[static_cast<std::size_t>(clutch_component_index)].id;
        clutch_in_ = node_for_key(component_port_key(cid, "in"));
        clutch_out_ = node_for_key(component_port_key(cid, "out"));
        clutch_row = builder.add_friction_row(clutch_in_, clutch_out_);
    }
    if (engine_component_index >= 0) {
        const std::string& eid = pt.components[static_cast<std::size_t>(engine_component_index)].id;
        engine_crank_ = node_for_key(component_port_key(eid, "crank"));
        engine_row = builder.add_friction_row(engine_crank_, kGround);
    }
    if (gearbox_desc) {
        gearbox_in_ = gearbox_in_n;
        gearbox_out_ = gearbox_out_n;
        synchro_row = builder.add_friction_row(gearbox_in_, gearbox_out_);
    }

    // --- Phase H: brake friction rows --------------------------------------
    std::unordered_map<std::string, std::size_t> wheel_index_of_name;
    for (std::size_t wi = 0; wi < wheels.size(); ++wi) wheel_index_of_name.emplace(wheels[wi].name, wi);

    std::vector<std::string> braked_wheels_order;
    std::unordered_map<std::string, real> corner_max_of;
    std::unordered_map<std::string, real> hand_max_of;
    for (const BrakeCornerDesc& corner : brakes.corners) {
        if (!wheel_index_of_name.count(corner.wheel)) {
            throw std::runtime_error("vehicle '" + vehicle_name + "': brake_system corner references unknown wheel '" + corner.wheel + "'");
        }
        if (corner_max_of.count(corner.wheel)) {
            throw std::runtime_error("vehicle '" + vehicle_name + "': brake_system has more than one corner for wheel '" + corner.wheel + "'");
        }
        corner_max_of[corner.wheel] = corner.max_torque_nm;
        braked_wheels_order.push_back(corner.wheel);
    }
    for (const std::string& wn : brakes.handbrake.wheels) {
        if (!wheel_index_of_name.count(wn)) {
            throw std::runtime_error("vehicle '" + vehicle_name + "': brake_system.handbrake references unknown wheel '" + wn + "'");
        }
        hand_max_of[wn] = brakes.handbrake.max_torque_nm;
        if (!corner_max_of.count(wn) && std::find(braked_wheels_order.begin(), braked_wheels_order.end(), wn) == braked_wheels_order.end()) {
            braked_wheels_order.push_back(wn);
        }
    }

    std::vector<DirectTorqueBrakes::Row> brake_rows;
    brake_rows.reserve(braked_wheels_order.size());
    for (const std::string& wn : braked_wheels_order) {
        const std::size_t wi = wheel_index_of_name[wn];
        const FrictionRowId rid = builder.add_friction_row(node_of_port[static_cast<std::size_t>(wheel_port_index[wi])]);
        DirectTorqueBrakes::Row row;
        row.row = rid;
        const auto cit = corner_max_of.find(wn);
        row.corner_max_nm = cit == corner_max_of.end() ? real(0.0) : cit->second;
        const auto hit = hand_max_of.find(wn);
        row.handbrake_max_nm = hit == hand_max_of.end() ? real(0.0) : hit->second;
        brake_rows.push_back(row);
    }

    // --- Phase I: build the network (validates every cluster's reduced mass
    // matrix, for every state) -----------------------------------------------
    network_ = builder.build(vehicle_name);

    // --- Phase J: hub nodes / driven flags ----------------------------------
    hub_nodes_.resize(wheels.size());
    driven_.resize(wheels.size());
    for (std::size_t wi = 0; wi < wheels.size(); ++wi) {
        const int p = wheel_port_index[wi];
        hub_nodes_[wi] = node_of_port[static_cast<std::size_t>(p)];
        driven_[wi] = port_in_any_shaft[static_cast<std::size_t>(p)];
    }
    // Mean radius of the driven wheels - speedo (plan section 7.6) and the
    // launch-vs-shift speed proxy fed to the TCU (manual_tcu.h's
    // Sensors::approx_speed_mps) both need it; computed once here since this
    // is the only place Powertrain sees WheelDesc::wheel_radius.
    {
        real radius_sum = real(0.0);
        int driven_count = 0;
        for (std::size_t wi = 0; wi < wheels.size(); ++wi) {
            if (driven_[wi] != 0) {
                radius_sum += wheels[wi].wheel_radius;
                ++driven_count;
            }
        }
        driven_wheel_mean_radius_ = driven_count > 0 ? radius_sum / static_cast<real>(driven_count) : real(0.0);
    }
    omega_old_.assign(static_cast<std::size_t>(network_.node_count()), real(0.0));
    topo_switch_scratch_.assign(static_cast<std::size_t>(network_.node_count()), real(0.0));

    // --- Phase K: engine/clutch/gearbox typed members -----------------------
    if (engine_component_index >= 0) {
        const auto& e = std::get<TorqueMapEngineDesc>(pt.components[static_cast<std::size_t>(engine_component_index)].params);
        engine_ = std::make_unique<EngineTorqueMap>(to_engine_desc(e));
        engine_friction_row_ = engine_row;
        // Bootstrap fix: EngineTorqueMap::pre_solve's own state machine can
        // only ENTER Running via a bump-start (omega already > 0, see plan
        // section 6.1.3) or a completed crank - it never spins the crank up
        // from a true rest state on its own (combustion requires omega_old >
        // 0, and the Coulomb/motoring term's sign(omega_old) is exactly 0 at
        // rest too, so tau0 is identically 0 forever). A vehicle constructed
        // with EngineInitialState::Running represents "key already turned,
        // engine already idling" - so the crank's network state must be
        // seeded to idle speed directly here, once, at construction, not left
        // for the physics to bootstrap from omega=0.
        if (engine_->desc().initial_state == EngineTorqueMap::InitialState::Running) {
            network_.set_omega(engine_crank_, rpm_to_rad_per_s(engine_->desc().idle_rpm));
        }
    }
    if (clutch_component_index >= 0) {
        const auto& c = std::get<ClutchDesc>(pt.components[static_cast<std::size_t>(clutch_component_index)].params);
        clutch_ = std::make_unique<Clutch>(to_clutch_desc(c));
        clutch_row_ = clutch_row;
    }
    if (gearbox_desc) {
        gearbox_synchro_row_ = synchro_row; // gearbox_ itself was already constructed above (Phase F), before the state loop
    }
    diff_ratio_ = diff_ratio;
    has_diff_ = has_diff;
    if (has_diff_) {
        diff_carrier_ = diff_carrier_n;
        diff_left_ = diff_left_n;
        diff_right_ = diff_right_n;
    }

    // --- Phase L: controllers (plan section 4.2/9.1) -------------------------
    if (controllers.size() > 1) {
        throw std::runtime_error("vehicle '" + vehicle_name + "': at most one controller (manual_tcu) is supported in I3");
    }
    for (const ControllerDesc& cd : controllers) {
        const auto* tcu_desc = std::get_if<ManualTcuDesc>(&cd.params);
        if (tcu_desc == nullptr) continue; // unreachable (ControllerParams has only ManualTcuDesc in I3)
        auto check_ref = [&](const std::string& id, const char* expect_type, auto&& type_matches) {
            const auto it = component_index_of.find(id);
            if (it == component_index_of.end()) {
                throw std::runtime_error("vehicle '" + vehicle_name + "': controller '" + cd.id + "' references unknown component '" + id + "'");
            }
            if (!type_matches(pt.components[static_cast<std::size_t>(it->second)].params)) {
                throw std::runtime_error("vehicle '" + vehicle_name + "': controller '" + cd.id + "' component '" + id +
                                          "' is not a " + expect_type);
            }
        };
        check_ref(tcu_desc->engine, "engine", [](const ComponentParams& p) { return std::holds_alternative<TorqueMapEngineDesc>(p); });
        check_ref(tcu_desc->clutch, "clutch", [](const ComponentParams& p) { return std::holds_alternative<ClutchDesc>(p); });
        check_ref(tcu_desc->gearbox, "gearbox", [](const ComponentParams& p) { return std::holds_alternative<GearboxDesc>(p); });

        ManualTcu::Desc md;
        md.auto_clutch = tcu_desc->auto_clutch;
        md.auto_blip = tcu_desc->auto_blip;
        md.auto_shift = tcu_desc->auto_shift;
        md.max_pending_steps = tcu_desc->max_pending_steps;
        tcu_ = std::make_unique<ManualTcu>(md);
    }

    // --- Phase M: rotor ports (plan section 6.7) -----------------------------
    for (int p = 0; p < port_count; ++p) {
        const Vec3& axis = port_axis_local[static_cast<std::size_t>(p)];
        if (axis.length_squared() <= real(0.0)) continue; // skip zero-axis ports (hubs included - always zero here)
        rotor_ports_.push_back(RotorPort{node_of_port[static_cast<std::size_t>(p)], port_inertia[static_cast<std::size_t>(p)], axis});
    }
    rotor_port_omega_old_.assign(rotor_ports_.size(), real(0.0));
    rotor_port_alpha_.assign(rotor_ports_.size(), real(0.0));

    // --- Phase N: components_ (gear_pair/differential in array order, brakes
    // last) - engine/clutch/gearbox are NOT here (typed members instead, see
    // this file's own top comment). -------------------------------------------
    components_.reserve(pt.components.size() + 1);
    for (const ComponentDesc& c : pt.components) {
        if (const auto* g = std::get_if<GearPairDesc>(&c.params)) {
            components_.push_back(std::make_unique<GearPair>(node_for_key(component_port_key(c.id, "in")),
                                                               node_for_key(component_port_key(c.id, "out")), *g));
        } else if (const auto* d = std::get_if<DifferentialDesc>(&c.params)) {
            auto diff = std::make_unique<OpenDifferential>(node_for_key(component_port_key(c.id, "carrier")),
                                                             node_for_key(component_port_key(c.id, "left")),
                                                             node_for_key(component_port_key(c.id, "right")), *d);
            diff_ = diff.get(); // PHYS-033 P4 I3 commit 7: powertrain_debug's diff/hub torque reads (powertrain.h)
            components_.push_back(std::move(diff));
        }
    }
    components_.push_back(std::make_unique<DirectTorqueBrakes>(std::move(brake_rows), brakes.static_factor));
}

void Powertrain::begin_substep() {
    assembly_.begin_substep(network_.node_count());
    assembly_.begin_substep_rows(network_.friction_row_count());
}

void Powertrain::pre_solve_components(const ComponentSubstepCtx& ctx_in) {
    ComponentSubstepCtx ctx = ctx_in;

    // --- plan section 7.2 step 3: controllers -------------------------------
    if (gearbox_) {
        if (tcu_) {
            ManualTcu::Sensors s;
            s.engine_omega = engine_ ? network_.omega(engine_crank_) : real(0.0);
            s.gearbox_in_omega = network_.omega(gearbox_in_);
            s.gearbox_out_omega = network_.omega(gearbox_out_);
            s.approx_speed_mps = approx_speed_mps();
            s.throttle_pedal = ctx.throttle;
            s.gear_request = ctx.gear_request;
            s.shift_up_count = ctx.shift_up_count;
            s.shift_down_count = ctx.shift_down_count;
            s.assist_auto_clutch = ctx.assist_auto_clutch;
            s.assist_auto_blip = ctx.assist_auto_blip;
            s.assist_auto_shift = ctx.assist_auto_shift;
            s.gearbox_current = gearbox_->current_position();
            s.gearbox_target = gearbox_->target();
            s.gearbox_phase = gearbox_->phase();
            s.synchro_target_ratio = last_gearbox_result_.synchro_target_ratio;
            s.min_gear = -static_cast<int>(gearbox_->desc().reverse_ratios.size());
            s.max_gear = static_cast<int>(gearbox_->desc().forward_ratios.size());
            const ManualTcu::Outputs out = tcu_->step(s, ctx.h, ctx.first_substep_of_tick);
            gearbox_->set_target(out.gearbox_target);
            ctx.e_auto = out.e_auto;
            // Engine reads LAST substep's TCU request (lagged - plan section
            // 7.2's deliberate order independence); the cache is then
            // overwritten with THIS substep's for next time.
            ctx.tcu_throttle_floor = tcu_prev_request_.throttle_floor;
            ctx.tcu_ignition_cut = tcu_prev_request_.ignition_cut;
            tcu_prev_request_.throttle_floor = out.throttle_floor;
            tcu_prev_request_.ignition_cut = out.ignition_cut;
        }

        Gearbox::StepInput gin;
        gin.omega1 = network_.omega(gearbox_in_);
        gin.omega2 = network_.omega(gearbox_out_);
        gin.lambda_mesh_prev = gearbox_lambda_mesh_prev_;
        gin.h = ctx.h;
        last_gearbox_result_ = gearbox_->phase_step(gin);
        ctx.gearbox_in_neutral = (last_gearbox_result_.current == 0);

        // --- step 3b: a possible topology-state switch (plan section 5.3) ---
        if (last_gearbox_result_.state_index != network_.topology_state()) {
            ledger_.engagement_loss += apply_topology_switch(last_gearbox_result_.state_index, ctx.h);
        }
    }

    if (clutch_) {
        last_clutch_result_ = clutch_->pre_solve(ctx.clutch_pedal, ctx.e_auto);
        ctx.clutch_engaged_fraction = last_clutch_result_.engagement;
        assembly_.set_friction_capacity(clutch_row_, last_clutch_result_.cap_static, last_clutch_result_.cap_kinetic);
    }

    // --- step 4: engine ------------------------------------------------------
    if (engine_) {
        EngineTorqueMap::Sensors s;
        s.omega = network_.omega(engine_crank_);
        s.pedal_throttle = ctx.throttle;
        s.ignition = ctx.ignition;
        s.starter = ctx.starter;
        s.clutch_engaged_fraction = ctx.clutch_engaged_fraction;
        s.gearbox_in_neutral = ctx.gearbox_in_neutral;
        const EngineTorqueMap::TcuRequest req{ctx.tcu_throttle_floor, ctx.tcu_ignition_cut};
        last_engine_result_ = engine_->pre_solve(s, req, ctx.h);
        const auto& pre = last_engine_result_;
        assembly_.torque(engine_crank_, pre.tau0, pre.d);
        assembly_.set_friction_capacity(engine_friction_row_, pre.friction_cap_static, pre.friction_cap_kinetic);
    }

    // --- step 4: gearbox mesh loss + synchro row (plan section 5.4/5.5) -----
    if (gearbox_) {
        gearbox_omega_old_in_ = network_.omega(gearbox_in_);
        const int cur = gearbox_->current_position();
        const real eff = cur == 0 ? real(1.0) : gearbox_->efficiency_for_position(cur);
        if (cur != 0 && eff < real(1.0)) {
            gearbox_own_loss_tau_ = -(real(1.0) - eff) * std::abs(gearbox_lambda_mesh_prev_) * sign(gearbox_omega_old_in_);
            assembly_.torque(gearbox_in_, gearbox_own_loss_tau_, real(0.0));
        } else {
            gearbox_own_loss_tau_ = real(0.0);
        }
        assembly_.set_friction_capacity(gearbox_synchro_row_, last_gearbox_result_.synchro_cap_static, last_gearbox_result_.synchro_cap_kinetic);
        if (last_gearbox_result_.synchro_target_ratio != real(0.0)) {
            assembly_.set_friction_row_ratio(gearbox_synchro_row_, last_gearbox_result_.synchro_target_ratio);
        }
    }

    for (auto& c : components_) c->pre_solve(ctx, network_, assembly_);
}

real Powertrain::apply_topology_switch(int new_state, real h) {
    const int n = network_.node_count();
    for (int i = 0; i < n; ++i) topo_switch_scratch_[static_cast<std::size_t>(i)] = network_.omega(NodeId{static_cast<std::uint16_t>(i)});

    network_.set_topology_state(new_state);
    projection_assembly_.begin_substep(n);
    projection_assembly_.begin_substep_rows(network_.friction_row_count());
    // Zero tau0/d/friction-caps: algebraically exactly the momentum-
    // conserving projection T_newT*J*T_new*nu = T_newT*J*w_old for ANY h
    // (see this method's declaration comment, powertrain.h).
    network_.solve(h, projection_assembly_);

    real loss = 0.0;
    for (int i = 0; i < n; ++i) {
        const NodeId nid{static_cast<std::uint16_t>(i)};
        const real dw = topo_switch_scratch_[static_cast<std::size_t>(i)] - network_.omega(nid);
        loss += real(0.5) * network_.node_inertia(nid) * dw * dw;
    }
    return loss;
}

void Powertrain::solve(real h) {
    const int n = network_.node_count();
    for (int i = 0; i < n; ++i) omega_old_[static_cast<std::size_t>(i)] = network_.omega(NodeId{static_cast<std::uint16_t>(i)});

    if (rotor_reaction_) {
        for (std::size_t i = 0; i < rotor_ports_.size(); ++i) rotor_port_omega_old_[i] = network_.omega(rotor_ports_[i].node);
    }

    network_.solve(h, assembly_);

    if (rotor_reaction_) {
        for (std::size_t i = 0; i < rotor_ports_.size(); ++i) {
            rotor_port_alpha_[i] = (network_.omega(rotor_ports_[i].node) - rotor_port_omega_old_[i]) / h;
        }
    }

    real integrator_loss_substep = real(0.0);
    real rotor_ke = real(0.0);
    for (int i = 0; i < n; ++i) {
        const NodeId nid{static_cast<std::uint16_t>(i)};
        const real J = network_.node_inertia(nid);
        const real w_old = omega_old_[static_cast<std::size_t>(i)];
        const real w_new = network_.omega(nid);
        const real dw = w_new - w_old;
        integrator_loss_substep += real(0.5) * J * dw * dw;
        rotor_ke += real(0.5) * J * w_new * w_new;
    }
    ledger_.integrator_loss += integrator_loss_substep;
    ledger_.rotor_ke = rotor_ke;
}

void Powertrain::post_solve_components(const ComponentSubstepCtx& ctx) {
    if (engine_) {
        const real omega_new = network_.omega(engine_crank_);
        const auto post = engine_->post_solve(omega_new, ctx.h);
        ledger_.engine_out += post.engine_out_delta;
        ledger_.starter_in += post.starter_in_delta;
        const real v = network_.row_velocity(engine_friction_row_);
        const real lambda = network_.row_lambda(engine_friction_row_);
        ledger_.engine_friction_loss += -ctx.h * v * lambda;
    }
    if (clutch_) {
        const real v = network_.row_velocity(clutch_row_);
        const real lambda = network_.row_lambda(clutch_row_);
        ledger_.clutch_loss += Clutch::clutch_loss_delta(ctx.h, v, lambda);
    }
    if (gearbox_) {
        const int cur = gearbox_->current_position();
        const real eff = cur == 0 ? real(1.0) : gearbox_->efficiency_for_position(cur);
        if (cur == 0 || eff >= real(1.0)) {
            gearbox_lambda_mesh_prev_ = real(0.0);
        } else {
            const real omega_new_in = network_.omega(gearbox_in_);
            gearbox_lambda_mesh_prev_ =
                reconstruct_mesh_torque(gearbox_in_, real(1.0), gearbox_own_loss_tau_, network_, assembly_, gearbox_omega_old_in_, ctx.h);
            ledger_.gear_loss += -ctx.h * omega_new_in * gearbox_own_loss_tau_;
        }
        const real v_syn = network_.row_velocity(gearbox_synchro_row_);
        const real lambda_syn = network_.row_lambda(gearbox_synchro_row_);
        ledger_.synchro_loss += -ctx.h * v_syn * lambda_syn;
    }
    for (auto& c : components_) c->post_solve(ctx, network_, assembly_, ledger_);
}

void Powertrain::end_tick() {
    const PowertrainEnergy L = ledger_;
    const PowertrainEnergy& L0 = ledger_tick_start_;
    ledger_.balance_error = (L.rotor_ke - L0.rotor_ke) - (L.engine_out - L0.engine_out) - (L.starter_in - L0.starter_in) -
                             (L.external_reset - L0.external_reset) + (L.wheel_out - L0.wheel_out) +
                             (L.tyre_loss - L0.tyre_loss) + (L.brake_loss - L0.brake_loss) + (L.gear_loss - L0.gear_loss) +
                             (L.clutch_loss - L0.clutch_loss) + (L.synchro_loss - L0.synchro_loss) +
                             (L.engine_friction_loss - L0.engine_friction_loss) + (L.engagement_loss - L0.engagement_loss) +
                             (L.integrator_loss - L0.integrator_loss);
}

void Powertrain::reset(const vehicle::VehicleResetOptions& opts, const std::vector<real>& hub_speeds) {
    // 1. Snapshot total rotational KE BEFORE any change (signed delta booked
    //    as ledger_.external_reset below - unlike apply_topology_switch's
    //    always-non-negative engagement_loss, a reset is an arbitrary
    //    teleport, not a physical inelastic engagement, so it can ADD energy
    //    too, e.g. spinning the hubs up to match a fast-moving chassis).
    const int n = network_.node_count();
    real ke_before = real(0.0);
    for (int i = 0; i < n; ++i) {
        const NodeId nid{static_cast<std::uint16_t>(i)};
        const real w = network_.omega(nid);
        ke_before += real(0.5) * network_.node_inertia(nid) * w * w;
    }

    // 2. Resolve the target gear and force the gearbox/topology-state
    //    directly (reset_to: current==target==gear, phase Idle, no shift
    //    sequence, no grind event; set_topology_state: no projection loss).
    int gear = opts.gear;
    if (gearbox_) {
        if (gear == vehicle::VehicleResetOptions::kKeepGear) gear = gearbox_->current_position();
        gear = std::clamp(gear, -static_cast<int>(gearbox_->desc().reverse_ratios.size()),
                           static_cast<int>(gearbox_->desc().forward_ratios.size()));
        gearbox_->reset_to(gear);
        network_.set_topology_state(gearbox_->state_index_for_position(gear));
    }

    // 3. Hub omegas - directly as given (VehicleInstance already resolved
    //    hubs_roll_with_chassis vs. 0 per wheel; Powertrain never touches
    //    the chassis itself).
    for (std::size_t i = 0; i < hub_nodes_.size() && i < hub_speeds.size(); ++i) {
        network_.set_omega(hub_nodes_[i], hub_speeds[i]);
    }

    // 4. Differential carrier (== gearbox.out, shaft-merged - plan section
    //    4.1's port graph): matches the diff's own rigid row exactly
    //    (carrier = diff_ratio_/2 * (left + right), this file's constructor)
    //    via the average of every DRIVEN wheel's hub speed (order-independent,
    //    so it does not matter which driven wheel is "left" vs "right").
    if (has_diff_) {
        real sum = real(0.0);
        int count = 0;
        for (std::size_t i = 0; i < hub_nodes_.size() && i < hub_speeds.size(); ++i) {
            if (driven_[i] != 0) {
                sum += hub_speeds[i];
                ++count;
            }
        }
        const real driven_avg = count > 0 ? sum / static_cast<real>(count) : real(0.0);
        network_.set_omega(diff_carrier_, diff_ratio_ * driven_avg);
    }

    // 5. Engine crank (== clutch.in, shaft-merged): idle when Off or not yet
    //    running; when clutch_locked and in gear, the driveline's own speed
    //    through the gear ratio, magnitude-floored at idle (spec's own
    //    wording) so a reset into a low driveline speed cannot leave the
    //    engine below idle.
    const real idle_omega = engine_ ? rpm_to_rad_per_s(engine_->desc().idle_rpm) : real(0.0);
    const real gearbox_out_omega_now = gearbox_ ? network_.omega(gearbox_out_) : real(0.0);
    const real driveline_in_omega = (gearbox_ && gear != 0) ? gearbox_->ratio_for_position(gear) * gearbox_out_omega_now : real(0.0);
    real engine_omega_target = real(0.0);
    if (engine_ && opts.engine_state != EngineState::Off) {
        if (opts.clutch_locked && gear != 0) {
            const real mag = std::abs(driveline_in_omega);
            engine_omega_target =
                mag >= idle_omega ? driveline_in_omega : (driveline_in_omega >= real(0.0) ? idle_omega : -idle_omega);
        } else {
            engine_omega_target = idle_omega;
        }
    }
    if (engine_) {
        network_.set_omega(engine_crank_, engine_omega_target);
        engine_->reset(opts.engine_state == EngineState::Off ? EngineTorqueMap::EngineState::Off
                                                               : EngineTorqueMap::EngineState::Running);
    }

    // 6. Clutch.out (== gearbox.in, shaft-merged): in gear, ALWAYS the
    //    driveline's own value (the gear ratio row is rigid regardless of
    //    clutch engagement - the clutch only sits between engine crank and
    //    this node); in Neutral, matches engine crank if clutch_locked
    //    (mirrors the Launch law's own "In gearbox N: e_tgt = 1" rule),
    //    else 0 (freewheeling at rest).
    if (gearbox_) {
        const real clutch_out_target =
            gear != 0 ? driveline_in_omega : (opts.clutch_locked ? engine_omega_target : real(0.0));
        network_.set_omega(clutch_out_, clutch_out_target);
    }
    if (tcu_) tcu_->reset(opts.clutch_locked);

    // 7. Clear every friction row's stick flags/warm-start set (no shift, no
    //    stale LCP active-set guess carried across the teleport).
    network_.clear_friction_row_state();

    // 8. Book the signed rotational-KE delta.
    real ke_after = real(0.0);
    for (int i = 0; i < n; ++i) {
        const NodeId nid{static_cast<std::uint16_t>(i)};
        const real w = network_.omega(nid);
        ke_after += real(0.5) * network_.node_inertia(nid) * w * w;
    }
    ledger_.external_reset += ke_after - ke_before;
}

real Powertrain::gearbox_output_ratio_magnitude() const { return std::abs(diff_ratio_); }

real Powertrain::clutch_slip() const {
    if (!clutch_) return real(0.0);
    return network_.omega(clutch_in_) - network_.omega(clutch_out_);
}

std::uint64_t Powertrain::fold_state_hash(std::uint64_t seed) const {
    std::uint64_t h = network_.fold_state_hash(seed);
    for (const auto& c : components_) h = c->fold_state_hash(h);
    if (engine_) h = engine_->fold_state_hash(h);
    if (gearbox_) h = gearbox_->fold_state_hash(h);
    if (tcu_) h = tcu_->fold_state_hash(h);
    if (gearbox_) {
        const auto* bytes = reinterpret_cast<const unsigned char*>(&gearbox_lambda_mesh_prev_);
        for (std::size_t i = 0; i < sizeof(gearbox_lambda_mesh_prev_); ++i) {
            h ^= bytes[i];
            h *= 1099511628211ULL;
        }
    }
    return h;
}

} // namespace ps::drivetrain
