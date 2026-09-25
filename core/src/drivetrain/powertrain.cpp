#include "powertrain.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <numeric>
#include <stdexcept>
#include <unordered_map>
#include <variant>

namespace ps::drivetrain {
namespace {

real sign(real x) { return x > real(0.0) ? real(1.0) : (x < real(0.0) ? real(-1.0) : real(0.0)); }

// PHYS-033 P4 I4 C6: lowercase names matching the loader's own accepted JSON
// "kind" strings (vehicle_io.cpp) - used only in the reconstruction-mode
// rejection message below, for kind != Open (Open never reaches it).
const char* diff_kind_name(DifferentialKind k) {
    switch (k) {
    case DifferentialKind::Open: return "open";
    case DifferentialKind::Locked: return "locked";
    case DifferentialKind::Viscous: return "viscous";
    case DifferentialKind::ClutchLsd: return "clutch_lsd"; // I4 C7
    case DifferentialKind::Torsen: return "torsen";        // I4 C7
    }
    return "unknown";
}

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

std::vector<ChannelDecl> Powertrain::declare_control_channels(const PowertrainDesc& pt, const std::vector<ChannelDecl>& test_only_extra) {
    std::vector<ChannelDecl> out;
    auto add = [&](const ChannelDecl& decl) {
        for (const ChannelDecl& existing : out) {
            if (existing.name == decl.name) return; // duplicate name: first occurrence's default wins
        }
        out.push_back(decl);
    };
    // Array order preserved. I4 C5: every lockable differential declares its
    // lock channel (default 0 = unlocked). Nothing else declares one yet.
    for (const ComponentDesc& c : pt.components) {
        if (const auto* d = std::get_if<DifferentialDesc>(&c.params)) {
            if (d->lockable) add(ChannelDecl{d->lock.channel, real(0.0)});
        }
    }
    for (const ChannelDecl& decl : test_only_extra) add(decl);
    return out;
}

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
    //
    // PHYS-033 P4 I4 C5 (plan section 2.6): COMPOSED states. The network's
    // state index is s = g + n_g * sum_i lock_i*2^i - g the gearbox state
    // above, n_g its state count (1 without a gearbox), i over LOCKABLE
    // diffs in component order. No lockable diff -> s == g, and every loop
    // below emits exactly the pre-C5 row list (the sedan's hashes).
    const GearboxDesc* gearbox_desc = gearbox_component_index >= 0 ? std::get_if<GearboxDesc>(&pt.components[static_cast<std::size_t>(gearbox_component_index)].params) : nullptr;
    const int num_gear_states = gearbox_desc ? static_cast<int>(gearbox_desc->reverse_ratios.size()) + 1 + static_cast<int>(gearbox_desc->forward_ratios.size()) : 1;
    int lockable_count = 0;
    for (const ComponentDesc& c : pt.components) {
        if (const auto* d = std::get_if<DifferentialDesc>(&c.params)) lockable_count += d->lockable ? 1 : 0;
    }
    // Validated here, before any row is built, with the composition spelled
    // out (NetworkBuilder::build's own kMaxStates throw is generic).
    {
        long long composed = num_gear_states;
        for (int i = 0; i < lockable_count && composed <= kMaxStates; ++i) composed *= 2;
        if (composed > kMaxStates) {
            throw std::runtime_error("vehicle '" + vehicle_name + "': powertrain has " + std::to_string(num_gear_states) +
                                      " gearbox states x 2^" + std::to_string(lockable_count) +
                                      " diff-lock combinations - more than kMaxStates (" + std::to_string(kMaxStates) +
                                      ") composed topology states");
        }
    }
    const int num_states = num_gear_states << lockable_count;
    gear_state_count_ = num_gear_states;
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
        build_rigid_rows_.emplace_back(state, row); // white-box record (tests), build time only
        builder.add_rigid_row(state, std::move(row));
    };

    // PHYS-033 P4 I4 C4 (plan section 2.5): every differential, in
    // pt.components array order - I3's "at most one" limit is gone.
    struct DiffBuild {
        std::string id;
        real ratio = 1.0;
        real efficiency = 1.0;
        bool lockable = false;
        DifferentialKind kind = DifferentialKind::Open; // I4 C6
        real viscous_nm_s_rad = 0.0;                    // I4 C6, kind == Viscous only
        CouplingId coupling_id{};                        // I4 C6, kind == Viscous only (see has_viscous_coupling_)
        NodeId carrier, left, right;
    };
    std::vector<DiffBuild> diff_builds;
    for (const ComponentDesc& c : pt.components) {
        if (const auto* d = std::get_if<DifferentialDesc>(&c.params)) {
            DiffBuild b;
            b.id = c.id;
            b.ratio = d->ratio;
            b.efficiency = d->efficiency;
            b.lockable = d->lockable;
            b.kind = d->kind;
            b.viscous_nm_s_rad = d->viscous_nm_s_rad;
            b.carrier = node_for_key(component_port_key(c.id, "carrier"));
            b.left = node_for_key(component_port_key(c.id, "left"));
            b.right = node_for_key(component_port_key(c.id, "right"));
            // I4 C6: viscous diffs get an implicit L-R coupling ALONGSIDE
            // their own (open) rigid row below - declared once here (build
            // time), parameterised every substep by Differential::pre_solve.
            if (b.kind == DifferentialKind::Viscous) b.coupling_id = builder.add_coupling(b.left, b.right);
            diff_builds.push_back(std::move(b));
        }
    }

    NodeId gearbox_in_n{}, gearbox_out_n{};
    if (gearbox_desc) {
        gearbox_in_n = node_for_key(component_port_key(pt.components[static_cast<std::size_t>(gearbox_component_index)].id, "in"));
        gearbox_out_n = node_for_key(component_port_key(pt.components[static_cast<std::size_t>(gearbox_component_index)].id, "out"));
    }

    for (int state = 0; state < num_states; ++state) {
        // Every diff row, in component order, BEFORE the gearbox row (plan
        // section 2.5) - a single-diff vehicle's per-state row list is
        // exactly I3's [diff, gearbox].
        for (const DiffBuild& db : diff_builds) {
            RigidRow row;
            row.terms.emplace_back(db.carrier.index, real(1.0));
            row.terms.emplace_back(db.left.index, -db.ratio / real(2.0));
            row.terms.emplace_back(db.right.index, -db.ratio / real(2.0));
            register_rigid_row(state, std::move(row));
        }
        if (gearbox_desc) {
            const int g = state % num_gear_states; // == state without a lockable diff
            const int position = g - static_cast<int>(gearbox_desc->reverse_ratios.size());
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

    // PHYS-033 P4 I4 C6 (plan section 3.1): kind == Locked (spool) rows
    // omega_L - omega_R = 0, UNCONDITIONAL - every topology state, not gated
    // by any composed-state bit (a Locked diff spends no bit; it is not a
    // "lockable" diff, it is always locked). Ordered here, BEFORE C5's own
    // per-state lock rows below (both live in the "lock rows" section, after
    // every diff/gearbox/gear_pair row) - a stable, documented order: kind-
    // Locked rows first, then C5's lockable-diff bit rows. Same
    // rigid_row_touch_count exclusion as C5's rows below, for the same
    // reason (a Locked diff's own row reaction must read as "the only rigid
    // coupling on its L/R" for the leaf-pattern check further down).
    for (int state = 0; state < num_states; ++state) {
        for (const DiffBuild& db : diff_builds) {
            if (db.kind != DifferentialKind::Locked) continue;
            RigidRow row;
            row.terms.emplace_back(db.left.index, real(1.0));
            row.terms.emplace_back(db.right.index, real(-1.0));
            build_rigid_rows_.emplace_back(state, row);
            builder.add_rigid_row(state, std::move(row));
        }
    }

    // PHYS-033 P4 I4 C5 (plan section 2.6): lock rows omega_L - omega_R = 0,
    // LAST in each state's list and only in states whose bit is set. Always
    // L-R (no carrier lock). Deliberately NOT counted in
    // rigid_row_touch_count: the reconstruction-mode rule below must discount
    // a diff's OWN lock row (its reaction cancels in the pair sum), so a
    // lockable diff's L/R still read as "touched only by its own diff row".
    // A lock row only ever touches a lockable diff's L/R, which the leaf rule
    // below requires to carry nothing else rigid - so no OTHER diff's mode
    // can depend on these uncounted terms.
    for (int state = 0; state < num_states; ++state) {
        const int bits = state / num_gear_states;
        int lock_i = 0;
        for (const DiffBuild& db : diff_builds) {
            if (!db.lockable) continue;
            if (((bits >> lock_i) & 1) != 0) {
                RigidRow row;
                row.terms.emplace_back(db.left.index, real(1.0));
                row.terms.emplace_back(db.right.index, real(-1.0));
                build_rigid_rows_.emplace_back(state, row);
                builder.add_rigid_row(state, std::move(row));
            }
            ++lock_i;
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

    // PHYS-033 P4 I4 C4 (plan section 2.7): each diff's row-multiplier
    // reconstruction mode (components.h's Differential::ReconMode). Rigid-
    // row term counts per node are summed over EVERY state, so "carries no
    // other rigid row" == "touched exactly num_states times" (its own row,
    // present in every state).
    //  - Leaf: left touched only by this diff's own row (I3's path, literal).
    //  - Nested: L and R are touched only by this row plus the rows of child
    //    diffs whose CARRIER they are (coef +1.0 there), >= 1 child, every
    //    child itself resolvable - an open centre diff feeding axle diffs.
    //  - None: anything else; rejected when efficiency < 1.
    constexpr int kModeNone = static_cast<int>(Differential::ReconMode::None);
    std::vector<int> diff_mode(diff_builds.size(), -1); // -1 unresolved, else static_cast<int>(Differential::ReconMode)
    std::vector<std::vector<int>> diff_children_left(diff_builds.size()), diff_children_right(diff_builds.size());
    {
        auto touches = [&](NodeId n) -> int {
            const auto it = rigid_row_touch_count.find(n.index);
            return it == rigid_row_touch_count.end() ? 0 : it->second;
        };
        std::vector<std::uint8_t> visiting(diff_builds.size(), 0);
        std::function<int(std::size_t)> resolve = [&](std::size_t di) -> int {
            if (diff_mode[di] >= 0) return diff_mode[di];
            if (visiting[di]) {
                throw std::runtime_error("vehicle '" + vehicle_name + "': differential '" + diff_builds[di].id +
                                          "' is part of a cycle of differentials feeding each other's carriers");
            }
            visiting[di] = 1;
            const DiffBuild& db = diff_builds[di];
            int mode = kModeNone;
            // I4 C7: clutch_lsd/torsen take their capacity from T_axle =
            // |ratio*lambda_row| (components.cpp), which is only available
            // as an exact pair reconstruction on a LEAF diff - so both are
            // leaf-only, lossy or not (plan section 3.2: centre LSD/Torsen
            // is not supported in I4).
            const bool leaf_only_kind = db.kind == DifferentialKind::ClutchLsd || db.kind == DifferentialKind::Torsen;
            if (db.lockable || leaf_only_kind) {
                // I4 C5 (plan section 2.7 / owner ruling Q3): a lockable diff
                // must be a LEAF - L and R each touched only by its own diff
                // row (its own lock row is uncounted, see above). LeafPair,
                // not Leaf: the left-only balance would include the lock
                // reaction while locked; the pair sum cancels it. I4 C7: the
                // same for clutch_lsd/torsen - their L-R friction row's
                // +-mu reaction cancels in the pair sum exactly like a lock
                // row's (friction rows never count as rigid touches).
                if (db.left == db.right || touches(db.left) != num_states || touches(db.right) != num_states) {
                    throw std::runtime_error("vehicle '" + vehicle_name + "': differential '" + db.id + "' of kind " +
                                              (db.lockable ? "lockable " : "") + diff_kind_name(db.kind) +
                                              " must drive nodes with no other rigid coupling "
                                              "(centre LSD/Torsen is not supported in I4)");
                }
                mode = static_cast<int>(Differential::ReconMode::LeafPair);
            } else if (db.left != db.right && touches(db.left) == num_states) {
                // I4 C6: a Locked/Viscous diff's own L-R mechanism (the
                // unconditional lock row, or the viscous coupling) is an
                // "other rigid coupling"-shaped reaction on L/R exactly like
                // C5's lockable case above - LeafPair, not Leaf, so the pair
                // sum cancels it. A plain Open diff keeps Leaf (I3-literal,
                // unchanged).
                mode = db.kind == DifferentialKind::Open ? static_cast<int>(Differential::ReconMode::Leaf)
                                                          : static_cast<int>(Differential::ReconMode::LeafPair);
            } else if (db.left != db.right) {
                std::vector<int> cl, cr;
                for (std::size_t dj = 0; dj < diff_builds.size(); ++dj) {
                    if (dj == di) continue;
                    if (diff_builds[dj].carrier == db.left) cl.push_back(static_cast<int>(dj));
                    if (diff_builds[dj].carrier == db.right) cr.push_back(static_cast<int>(dj));
                }
                const bool counts_ok = touches(db.left) == num_states * (1 + static_cast<int>(cl.size())) &&
                                       touches(db.right) == num_states * (1 + static_cast<int>(cr.size()));
                if (counts_ok && !(cl.empty() && cr.empty())) {
                    // I4 C6: a lossy centre (Nested-pattern) Locked/Viscous
                    // diff is rejected outright, same wording as C5's own
                    // lockable-diff rejection above - a LOSSLESS Locked/
                    // Viscous diff MAY be a centre diff (the L-R reaction
                    // still cancels in reconstruct_mesh_torque_pair's sum),
                    // so only the lossy case is refused here.
                    if (db.kind != DifferentialKind::Open && db.efficiency < real(1.0)) {
                        throw std::runtime_error("vehicle '" + vehicle_name + "': differential '" + db.id +
                                                  "' of kind " + diff_kind_name(db.kind) +
                                                  " must drive nodes with no other rigid coupling (centre LSD/Torsen is "
                                                  "not supported in I4)");
                    }
                    bool children_ok = true;
                    for (int dj : cl) children_ok = children_ok && resolve(static_cast<std::size_t>(dj)) != kModeNone;
                    for (int dj : cr) children_ok = children_ok && resolve(static_cast<std::size_t>(dj)) != kModeNone;
                    if (children_ok) {
                        mode = static_cast<int>(Differential::ReconMode::Nested);
                        diff_children_left[di] = std::move(cl);
                        diff_children_right[di] = std::move(cr);
                    }
                }
            }
            if (mode == kModeNone && db.efficiency < real(1.0)) {
                // I4 C6: a Locked/Viscous diff that reaches here (neither the
                // leaf nor the Nested-candidate pattern matched at all, e.g.
                // its L/R also touch an unrelated rigid row) gets the same
                // kind-specific message as the two cases above; a plain Open
                // diff keeps the exact pre-C6 wording (tested verbatim).
                if (db.kind != DifferentialKind::Open) {
                    throw std::runtime_error("vehicle '" + vehicle_name + "': differential '" + db.id +
                                              "' of kind " + diff_kind_name(db.kind) +
                                              " must drive nodes with no other rigid coupling (centre LSD/Torsen is not "
                                              "supported in I4)");
                }
                throw std::runtime_error("vehicle '" + vehicle_name + "': differential '" + db.id +
                                          "' has efficiency < 1 but its output nodes carry rigid couplings other than child "
                                          "differentials' carriers - mesh-loss reconstruction is not available there");
            }
            visiting[di] = 0;
            diff_mode[di] = mode;
            return mode;
        };
        for (std::size_t di = 0; di < diff_builds.size(); ++di) (void)resolve(di);
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

    // --- Phase H2 (PHYS-033 P4 I4 C7, plan section 3.2): one L-R friction
    // row per clutch_lsd/torsen diff (a = L, b = R, ratio_b = 1), component
    // order, present in every topology state (friction rows are never
    // per-state). Added AFTER every clutch/engine/synchro/brake row so every
    // pre-C7 vehicle's friction-row ids - and with them its hashed row_stuck_
    // /row_set_ byte order - are unchanged.
    std::vector<int> diff_lsd_row(diff_builds.size(), -1); // FrictionRowId::index, or -1
    for (std::size_t di = 0; di < diff_builds.size(); ++di) {
        const DiffBuild& db = diff_builds[di];
        if (db.kind != DifferentialKind::ClutchLsd && db.kind != DifferentialKind::Torsen) continue;
        diff_lsd_row[di] = builder.add_friction_row(db.left, db.right).index;
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
    diff_ratio_ = diff_builds.empty() ? real(1.0) : diff_builds.front().ratio;

    // PHYS-033 P4 I4 C4 (plan section 2.5): speedo path. Walk the rigid
    // kinematic tree DOWN from gearbox.out computing K(n) = omega(n)/v under
    // straight-line no-slip rolling (v = ground speed):
    //  - a node holding wheel hubs: K = 1/r (mean radius of the hubs there);
    //  - a diff whose carrier is n: K = ratio*((K(L)+K(R))/2), the diff's own
    //    row; each branch's path ratio is multiplied by `ratio`;
    //  - a gear_pair whose `in` is n: K = ratio*K(out), path ratio *ratio;
    //  - several of these at one node (e.g. two axle diffs sharing a rigid
    //    carrier shaft) are each a complete, kinematically consistent
    //    estimate of the same K: their mean is used, never their sum.
    // Rule: if every reached hub-leaf's path ratio is EXACTLY equal (R) and
    // every reached hub's radius is exactly equal, speedo_ratio_ = |R| and
    // speedo_radius_ = the arithmetic mean of the reached hubs' radii in
    // wheel-index order - I3's own expression for a single diff (bit-
    // identical: same summation order as driven_wheel_mean_radius_) and
    // exact for an open centre diff (ratio 1) over two equal axle diffs
    // (1.0*4.06 on every path, K = 4.06/r). Otherwise the exact no-slip
    // form: speedo_ratio_ = 1, speedo_radius_ = 1/|K(gearbox.out)|. No
    // gearbox, or no hub reached: I3's |first diff ratio| (or 1) and the
    // driven mean radius. (A transfer case - I4 C9 - restricts the walk to
    // its rear output, plan section 2.5.)
    speedo_ratio_ = std::abs(diff_ratio_);
    speedo_radius_ = driven_wheel_mean_radius_;
    if (gearbox_desc) {
        std::vector<NodeId> gp_in, gp_out;
        std::vector<real> gp_ratio;
        for (const ComponentDesc& c : pt.components) {
            if (const auto* g = std::get_if<GearPairDesc>(&c.params)) {
                gp_in.push_back(node_for_key(component_port_key(c.id, "in")));
                gp_out.push_back(node_for_key(component_port_key(c.id, "out")));
                gp_ratio.push_back(g->ratio);
            }
        }
        std::vector<std::uint8_t> reached(hub_nodes_.size(), 0);
        std::vector<real> leaf_path_ratios;
        struct KResult {
            bool ok = false;
            real k = 0.0;
        };
        constexpr int kMaxSpeedoDepth = 32; // guards against a malformed cyclic graph
        std::function<KResult(NodeId, real, int)> k_of = [&](NodeId n, real path_ratio, int depth) -> KResult {
            if (depth > kMaxSpeedoDepth) return {};
            real est_sum = real(0.0);
            int est_count = 0;
            real r_sum = real(0.0);
            int r_count = 0;
            for (std::size_t wi = 0; wi < hub_nodes_.size(); ++wi) {
                if (hub_nodes_[wi] != n) continue;
                r_sum += wheels[wi].wheel_radius;
                ++r_count;
                reached[wi] = 1;
            }
            if (r_count > 0) {
                est_sum += real(1.0) / (r_sum / static_cast<real>(r_count));
                ++est_count;
                leaf_path_ratios.push_back(path_ratio);
            }
            for (const DiffBuild& db : diff_builds) {
                if (db.carrier != n) continue;
                const KResult kl = k_of(db.left, path_ratio * db.ratio, depth + 1);
                const KResult kr = k_of(db.right, path_ratio * db.ratio, depth + 1);
                if (kl.ok && kr.ok) {
                    est_sum += db.ratio * ((kl.k + kr.k) / real(2.0));
                    ++est_count;
                }
            }
            for (std::size_t gi = 0; gi < gp_in.size(); ++gi) {
                if (gp_in[gi] != n) continue;
                const KResult ko = k_of(gp_out[gi], path_ratio * gp_ratio[gi], depth + 1);
                if (ko.ok) {
                    est_sum += gp_ratio[gi] * ko.k;
                    ++est_count;
                }
            }
            if (est_count == 0) return {};
            return {true, est_sum / static_cast<real>(est_count)};
        };
        const KResult k_root = k_of(gearbox_out_n, real(1.0), 0);

        if (k_root.ok && !leaf_path_ratios.empty()) {
            bool same_ratio = true;
            for (real pr : leaf_path_ratios) same_ratio = same_ratio && pr == leaf_path_ratios.front();
            bool same_radius = true;
            real first_radius = real(-1.0);
            real radius_sum = real(0.0);
            int reached_count = 0;
            for (std::size_t wi = 0; wi < hub_nodes_.size(); ++wi) {
                if (reached[wi] == 0) continue;
                if (first_radius < real(0.0)) first_radius = wheels[wi].wheel_radius;
                same_radius = same_radius && wheels[wi].wheel_radius == first_radius;
                radius_sum += wheels[wi].wheel_radius;
                ++reached_count;
            }
            if (same_ratio && same_radius) {
                speedo_ratio_ = std::abs(leaf_path_ratios.front());
                speedo_radius_ = radius_sum / static_cast<real>(reached_count);
            } else if (k_root.k != real(0.0)) {
                speedo_ratio_ = real(1.0);
                speedo_radius_ = real(1.0) / std::abs(k_root.k);
            }
        }
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
    // PHYS-033 P4 I4 C4 (plan section 2.5): reset derivations, leaves up.
    // Each diff's carrier = ratio*((w_L+w_R)/2) and each gear_pair's in =
    // ratio*out, ordered so every source node is final before a derivation
    // reads it (repeated passes in component order: an entry is ready once
    // no still-pending entry targets one of its sources). A malformed
    // cyclic graph stops making progress; the rest is appended in component
    // order rather than throwing (reset stays best-effort there).
    {
        std::vector<ResetDerivation> pending;
        for (const ComponentDesc& c : pt.components) {
            if (const auto* d = std::get_if<DifferentialDesc>(&c.params)) {
                ResetDerivation r;
                r.is_diff = true;
                r.target = node_for_key(component_port_key(c.id, "carrier"));
                r.a = node_for_key(component_port_key(c.id, "left"));
                r.b = node_for_key(component_port_key(c.id, "right"));
                r.ratio = d->ratio;
                pending.push_back(r);
            } else if (const auto* g = std::get_if<GearPairDesc>(&c.params)) {
                ResetDerivation r;
                r.is_diff = false;
                r.target = node_for_key(component_port_key(c.id, "in"));
                r.a = node_for_key(component_port_key(c.id, "out"));
                r.b = r.a;
                r.ratio = g->ratio;
                pending.push_back(r);
            }
        }
        std::vector<std::uint8_t> done(pending.size(), 0);
        std::size_t remaining = pending.size();
        while (remaining > 0) {
            bool progress = false;
            for (std::size_t i = 0; i < pending.size(); ++i) {
                if (done[i]) continue;
                bool ready = true;
                for (std::size_t j = 0; j < pending.size() && ready; ++j) {
                    if (j == i || done[j]) continue;
                    if (pending[j].target == pending[i].a || pending[j].target == pending[i].b) ready = false;
                }
                if (!ready) continue;
                reset_derivations_.push_back(pending[i]);
                done[i] = 1;
                --remaining;
                progress = true;
            }
            if (!progress) {
                for (std::size_t i = 0; i < pending.size(); ++i) {
                    if (!done[i]) reset_derivations_.push_back(pending[i]);
                }
                break;
            }
        }
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
        // Torque-following close inputs (AutoClutchDesc::sync_time_s): the
        // engine-side clutch node's inertia and the clutch's capacity at
        // engagement 1 (check_ref above guarantees both components exist).
        if (clutch_) {
            md.engine_inertia = network_.node_inertia(clutch_in_);
            md.clutch_capacity_nm = clutch_->desc().capacity_nm;
        }
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
            auto diff = std::make_unique<Differential>(node_for_key(component_port_key(c.id, "carrier")),
                                                         node_for_key(component_port_key(c.id, "left")),
                                                         node_for_key(component_port_key(c.id, "right")), *d);
            diffs_.push_back(diff.get()); // powertrain_debug's diff/hub torque reads (powertrain.h), component order
            components_.push_back(std::move(diff));
        }
    }
    // Reconstruction modes resolved in Phase F (diffs_ is in the same
    // component order as diff_builds).
    for (std::size_t di = 0; di < diffs_.size(); ++di) {
        std::vector<const Differential*> cl, cr;
        for (int dj : diff_children_left[di]) cl.push_back(diffs_[static_cast<std::size_t>(dj)]);
        for (int dj : diff_children_right[di]) cr.push_back(diffs_[static_cast<std::size_t>(dj)]);
        diffs_[di]->configure_reconstruction(static_cast<Differential::ReconMode>(diff_mode[di]), std::move(cl), std::move(cr));
        // PHYS-033 P4 I4 C6: diff_builds is in the same component order as
        // diffs_ (both built by iterating pt.components once - Phase A/N),
        // so diff_builds[di]'s coupling_id (Phase A, kind == Viscous only)
        // wires straight onto diffs_[di].
        if (diff_builds[di].kind == DifferentialKind::Viscous) diffs_[di]->set_viscous_coupling(diff_builds[di].coupling_id);
        // PHYS-033 P4 I4 C7: same component-order pairing for Phase H2's
        // clutch_lsd/torsen friction rows.
        if (diff_lsd_row[di] >= 0) diffs_[di]->set_lsd_row(FrictionRowId{static_cast<std::uint16_t>(diff_lsd_row[di])});
    }
    // PHYS-033 P4 I4 C5: one DiffLock per lockable diff, bit i in component
    // order (the same order the lock-row loop above used). channel_slot is
    // the lock channel's index in declare_control_channels(pt) - the order
    // World::create_vehicle resolves into ComponentSubstepCtx::channels (any
    // test_only_extra channels are appended AFTER, so these slots are stable).
    {
        const std::vector<ChannelDecl> decls = declare_control_channels(pt);
        std::size_t di = 0;
        for (const ComponentDesc& c : pt.components) {
            const auto* d = std::get_if<DifferentialDesc>(&c.params);
            if (d == nullptr) continue;
            if (d->lockable) {
                DiffLock lk;
                lk.diff_index = di;
                lk.left = diffs_[di]->left_node();
                lk.right = diffs_[di]->right_node();
                lk.max_mismatch = d->lock.max_engage_mismatch_rad_s;
                for (std::size_t k = 0; k < decls.size(); ++k) {
                    if (decls[k].name == d->lock.channel) lk.channel_slot = static_cast<int>(k);
                }
                // PHYS-033 P4 I4 C7 (plan section 2.6, "a lockable LSD row
                // gets capacity 0 while locked"): the diff reads its own bit
                // (== locks_.size() before this push, the lock-row loop's
                // bit order) straight from the network's composed state.
                if (diffs_[di]->has_lsd_row()) diffs_[di]->set_lock_bit(static_cast<int>(locks_.size()), gear_state_count_);
                locks_.push_back(lk);
            } else if (d->kind == DifferentialKind::Locked) {
                // PHYS-033 P4 I4 C6: kind == Locked - unconditional, not a
                // locks_/lock_pending_ entry (no bit, no channel; see
                // always_locked_diffs_'s own comment in powertrain.h).
                always_locked_diffs_.emplace_back(diffs_[di]->left_node(), diffs_[di]->right_node());
            }
            ++di;
        }
        lock_pending_.assign(locks_.size(), 0);
    }
    components_.push_back(std::make_unique<DirectTorqueBrakes>(std::move(brake_rows), brakes.static_factor));
}

void Powertrain::begin_substep() {
    assembly_.begin_substep(network_.node_count());
    assembly_.begin_substep_rows(network_.friction_row_count());
    assembly_.begin_substep_couplings(network_.coupling_count());
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
            // Engine's demanded net crank torque (limiter ignored), from its
            // previous substep - EngineTorqueMap::PreSolveResult::torque_demand.
            if (engine_) s.engine_torque_nm = last_engine_result_.torque_demand;
            const ManualTcu::Outputs out = tcu_->step(s, ctx.h, ctx.first_substep_of_tick);
            gearbox_->set_target(out.gearbox_target);
            ctx.e_auto = out.e_auto;
            // Engine reads LAST substep's TCU request (lagged - plan section
            // 7.2's deliberate order independence); the cache is then
            // overwritten with THIS substep's for next time.
            ctx.tcu_throttle_floor = tcu_prev_request_.throttle_floor;
            ctx.tcu_ignition_cut = tcu_prev_request_.ignition_cut;
            ctx.tcu_throttle_ceiling = tcu_prev_request_.throttle_ceiling;
            tcu_prev_request_.throttle_floor = out.throttle_floor;
            tcu_prev_request_.ignition_cut = out.ignition_cut;
            tcu_prev_request_.throttle_ceiling = out.throttle_ceiling;
        }

        Gearbox::StepInput gin;
        gin.omega1 = network_.omega(gearbox_in_);
        gin.omega2 = network_.omega(gearbox_out_);
        gin.lambda_mesh_prev = gearbox_lambda_mesh_prev_;
        gin.h = ctx.h;
        last_gearbox_result_ = gearbox_->phase_step(gin);
        ctx.gearbox_in_neutral = (last_gearbox_result_.current == 0);
    }

    // --- step 3b: a possible topology-state switch (plan section 5.3; I4 C5
    // section 2.6: the COMPOSED s = g + n_g*lock_bits, lock requests decided
    // after phase_step). One projection covers a simultaneous gear change and
    // lock engage. No gearbox and no lockable diff: s == 0 always, never a
    // switch (the pre-C5 code did not compare at all there). ---
    {
        const int g = gearbox_ ? last_gearbox_result_.state_index : 0;
        const int s = g + gear_state_count_ * update_lock_bits(ctx);
        if (s != network_.topology_state()) ledger_.engagement_loss += apply_topology_switch(s, ctx.h);
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
        const EngineTorqueMap::TcuRequest req{ctx.tcu_throttle_floor, ctx.tcu_ignition_cut, ctx.tcu_throttle_ceiling};
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

int Powertrain::update_lock_bits(const ComponentSubstepCtx& ctx) {
    int bits = network_.topology_state() / gear_state_count_; // 0 without a lockable diff
    for (std::size_t i = 0; i < locks_.size(); ++i) {
        const DiffLock& lk = locks_[i];
        const real cmd = (ctx.channels != nullptr && lk.channel_slot >= 0 && lk.channel_slot < ctx.channel_count)
                             ? ctx.channels[lk.channel_slot]
                             : real(0.0);
        const int bit = 1 << i;
        if (cmd >= real(0.5)) {
            if ((bits & bit) == 0) {
                // Engage only near speed match (omega_old, before this
                // substep's solve); otherwise pending, retried next substep.
                const real mismatch = std::abs(network_.omega(lk.left) - network_.omega(lk.right));
                if (mismatch <= lk.max_mismatch) {
                    bits |= bit;
                    lock_pending_[i] = 0;
                } else {
                    lock_pending_[i] = 1;
                }
            } else {
                lock_pending_[i] = 0;
            }
        } else {
            bits &= ~bit; // release: lossless (dropping a row projects omega onto itself)
            lock_pending_[i] = 0;
        }
    }
    return bits;
}

real Powertrain::apply_topology_switch(int new_state, real h) {
    const int n = network_.node_count();
    for (int i = 0; i < n; ++i) topo_switch_scratch_[static_cast<std::size_t>(i)] = network_.omega(NodeId{static_cast<std::uint16_t>(i)});

    network_.set_topology_state(new_state);
    projection_assembly_.begin_substep(n);
    projection_assembly_.begin_substep_rows(network_.friction_row_count());
    projection_assembly_.begin_substep_couplings(network_.coupling_count()); // c = 0: couplings skipped, pure projection
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
                             (L.integrator_loss - L0.integrator_loss) + (L.diff_loss - L0.diff_loss);
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
    //    I4 C5: the COMPOSED state - the current lock bits are kept (a reset
    //    is not a lock request; the next substep's channel read decides).
    const int lock_bits = network_.topology_state() / gear_state_count_;
    int gear = opts.gear;
    if (gearbox_) {
        if (gear == vehicle::VehicleResetOptions::kKeepGear) gear = gearbox_->current_position();
        gear = std::clamp(gear, -static_cast<int>(gearbox_->desc().reverse_ratios.size()),
                           static_cast<int>(gearbox_->desc().forward_ratios.size()));
        gearbox_->reset_to(gear);
        network_.set_topology_state(gearbox_->state_index_for_position(gear) + gear_state_count_ * lock_bits);
    }
    std::fill(lock_pending_.begin(), lock_pending_.end(), std::uint8_t{0});

    // 3. Hub omegas - directly as given (VehicleInstance already resolved
    //    hubs_roll_with_chassis vs. 0 per wheel; Powertrain never touches
    //    the chassis itself).
    for (std::size_t i = 0; i < hub_nodes_.size() && i < hub_speeds.size(); ++i) {
        network_.set_omega(hub_nodes_[i], hub_speeds[i]);
    }
    //    I4 C5: an ENGAGED lock's L/R take their mean, so the reset state
    //    satisfies the lock row exactly (before the carrier derivations
    //    below read them).
    for (std::size_t i = 0; i < locks_.size(); ++i) {
        if (((lock_bits >> i) & 1) == 0) continue;
        const real w = (network_.omega(locks_[i].left) + network_.omega(locks_[i].right)) / real(2.0);
        network_.set_omega(locks_[i].left, w);
        network_.set_omega(locks_[i].right, w);
    }
    //    I4 C6: a kind == Locked diff's row is UNCONDITIONAL (every topology
    //    state, no bit) - so, unlike the lock_bits-gated loop above, this one
    //    always runs, for the same reason (the reset state must satisfy the
    //    row exactly before the carrier derivations below read L/R).
    for (const auto& [left, right] : always_locked_diffs_) {
        const real w = (network_.omega(left) + network_.omega(right)) / real(2.0);
        network_.set_omega(left, w);
        network_.set_omega(right, w);
    }

    // 4. Derived nodes, leaves up (PHYS-033 P4 I4 C4, plan section 2.5 -
    //    reset_derivations_, ordered at construction): every diff carrier
    //    (== gearbox.out for the outermost one, shaft-merged) from its own
    //    two outputs, matching its rigid row exactly, and every gear_pair
    //    input from its output. A nested (centre) diff's outputs are its
    //    children's carriers, derived earlier in the list.
    for (const ResetDerivation& r : reset_derivations_) {
        if (r.is_diff) {
            const real w_l = network_.omega(r.a);
            const real w_r = network_.omega(r.b);
            network_.set_omega(r.target, r.ratio * ((w_l + w_r) / real(2.0)));
        } else {
            network_.set_omega(r.target, r.ratio * network_.omega(r.a));
        }
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
    tcu_prev_request_ = {};

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

real Powertrain::gearbox_output_ratio_magnitude() const { return speedo_ratio_; }

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
