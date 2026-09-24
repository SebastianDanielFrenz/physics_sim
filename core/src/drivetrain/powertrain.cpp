#include "powertrain.h"

#include <algorithm>
#include <numeric>
#include <stdexcept>
#include <unordered_map>
#include <variant>

namespace ps::drivetrain {
namespace {

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

} // namespace

Powertrain::Powertrain(const PowertrainDesc& pt, const BrakeSystemDesc& brakes, const std::vector<vehicle::WheelDesc>& wheels,
                        const std::string& vehicle_name) {
    // --- Phase A: port table (plan section 4.1 step 1) --------------------
    // Walk components in array order, each type's ports in FIXED order
    // (engine: crank; gear_pair: in, out; differential: carrier, left,
    // right), then one hub port per wheel in VehicleDesc::wheels order.
    std::vector<std::string> port_key;   // port index -> its lookup key (component_port_key or wheel_port_key)
    std::vector<real> port_inertia;      // port index -> its own inertia contribution (plan step 5)
    std::unordered_map<std::string, int> port_index_of;

    auto add_port = [&](const std::string& key, real inertia) -> int {
        if (port_index_of.count(key)) {
            throw std::runtime_error("vehicle '" + vehicle_name + "': powertrain port '" + key + "' is declared more than once");
        }
        const int idx = static_cast<int>(port_key.size());
        port_index_of.emplace(key, idx);
        port_key.push_back(key);
        port_inertia.push_back(inertia);
        return idx;
    };

    std::unordered_map<std::string, int> component_index_of; // component id -> index into pt.components
    for (std::size_t ci = 0; ci < pt.components.size(); ++ci) {
        const ComponentDesc& c = pt.components[ci];
        if (c.id.empty()) throw std::runtime_error("vehicle '" + vehicle_name + "': a powertrain component has an empty id");
        if (!component_index_of.emplace(c.id, static_cast<int>(ci)).second) {
            throw std::runtime_error("vehicle '" + vehicle_name + "': powertrain component id '" + c.id + "' is not unique");
        }
        if (const auto* e = std::get_if<TorqueMapEngineDesc>(&c.params)) {
            add_port(component_port_key(c.id, "crank"), e->inertia_kgm2);
        } else if (const auto* g = std::get_if<GearPairDesc>(&c.params)) {
            add_port(component_port_key(c.id, "in"), g->inertia_in_kgm2);
            add_port(component_port_key(c.id, "out"), g->inertia_out_kgm2);
        } else if (std::get_if<DifferentialDesc>(&c.params)) {
            add_port(component_port_key(c.id, "carrier"), real(0.0));
            add_port(component_port_key(c.id, "left"), real(0.0));
            add_port(component_port_key(c.id, "right"), real(0.0));
        }
    }

    std::vector<int> wheel_port_index(wheels.size(), -1);
    for (std::size_t wi = 0; wi < wheels.size(); ++wi) {
        if (wheels[wi].wheel_inertia <= real(0.0)) {
            throw std::runtime_error("vehicle '" + vehicle_name + "': wheel '" + wheels[wi].name +
                                      "' has non-positive wheel_inertia (needed as the drivetrain hub's own inertia)");
        }
        wheel_port_index[wi] = add_port(wheel_port_key(wheels[wi].name), wheels[wi].wheel_inertia);
    }

    const int port_count = static_cast<int>(port_key.size());

    // --- Phase B: shafts -> union-find over ports (step 3), with plan
    // section 4.1 step 2's validation -----------------------------------
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
            throw std::runtime_error("vehicle '" + vehicle_name + "': powertrain shaft has non-zero stiffness (I2 requires rigid shafts)");
        }
        std::vector<int> ports_here;
        ports_here.reserve(shaft.ports.size());
        for (const drivetrain::PortRef& ref : shaft.ports) {
            const int idx = resolve_port(ref);
            if (port_in_any_shaft[static_cast<std::size_t>(idx)]) {
                throw std::runtime_error("vehicle '" + vehicle_name + "': powertrain port '" + port_key[static_cast<std::size_t>(idx)] +
                                          "' is connected by more than one shaft, or twice in one shaft");
            }
            // Also reject a port repeated within THIS shaft's own list before
            // it is marked "in any shaft" below (both checks share the same
            // flag: setting it immediately after the check above makes a
            // same-shaft repeat fail on its second occurrence exactly like a
            // cross-shaft repeat would).
            port_in_any_shaft[static_cast<std::size_t>(idx)] = 1;
            ports_here.push_back(idx);
        }
        for (std::size_t i = 1; i < ports_here.size(); ++i) uf.unite(ports_here[0], ports_here[i]);
    }

    // Every non-wheel (component) port must be connected (step 2's "every
    // component port that appears in no shaft" error); wheel hubs may be
    // free.
    for (int p = 0; p < port_count; ++p) {
        const bool is_wheel_port = port_key[static_cast<std::size_t>(p)].rfind("wheel:", 0) == 0;
        if (!is_wheel_port && !port_in_any_shaft[static_cast<std::size_t>(p)]) {
            throw std::runtime_error("vehicle '" + vehicle_name + "': powertrain port '" + port_key[static_cast<std::size_t>(p)] +
                                      "' is not connected by any shaft");
        }
    }

    // --- Phase D: per-root summed inertia (step 5) -------------------------
    std::unordered_map<int, real> root_inertia;
    for (int p = 0; p < port_count; ++p) {
        const int root = uf.find(p);
        root_inertia[root] += port_inertia[static_cast<std::size_t>(p)];
    }

    // --- Phase E: number nodes by walking ports in port-index order (step 4) ---
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

    // --- Phase F/G: rigid rows (plan section 4.2) + the lossy-gear
    // single-rigid-row-at-input validation (section 5.4) --------------------
    std::unordered_map<std::uint16_t, int> rigid_row_touch_count; // NodeId::index -> how many rigid-row TERMS touch it
    auto register_rigid_row = [&](RigidRow row) {
        for (const auto& [node_index, coef] : row.terms) {
            (void)coef;
            ++rigid_row_touch_count[static_cast<std::uint16_t>(node_index)];
        }
        builder.add_rigid_row(0, std::move(row));
    };

    for (const ComponentDesc& c : pt.components) {
        if (const auto* g = std::get_if<GearPairDesc>(&c.params)) {
            const NodeId in_n = node_for_key(component_port_key(c.id, "in"));
            const NodeId out_n = node_for_key(component_port_key(c.id, "out"));
            RigidRow row;
            row.terms.emplace_back(in_n.index, real(1.0));
            row.terms.emplace_back(out_n.index, -g->ratio);
            register_rigid_row(std::move(row));
        } else if (const auto* d = std::get_if<DifferentialDesc>(&c.params)) {
            const NodeId carrier_n = node_for_key(component_port_key(c.id, "carrier"));
            const NodeId left_n = node_for_key(component_port_key(c.id, "left"));
            const NodeId right_n = node_for_key(component_port_key(c.id, "right"));
            RigidRow row;
            row.terms.emplace_back(carrier_n.index, real(1.0));
            row.terms.emplace_back(left_n.index, -d->ratio / real(2.0));
            row.terms.emplace_back(right_n.index, -d->ratio / real(2.0));
            register_rigid_row(std::move(row));
        }
    }

    for (const ComponentDesc& c : pt.components) {
        const auto* g = std::get_if<GearPairDesc>(&c.params);
        if (g == nullptr || g->efficiency >= real(1.0)) continue;
        const NodeId in_n = node_for_key(component_port_key(c.id, "in"));
        const int touches = rigid_row_touch_count[in_n.index];
        if (touches != 1) {
            throw std::runtime_error("vehicle '" + vehicle_name + "': gear_pair '" + c.id +
                                      "' has efficiency < 1 but its input node carries another rigid row - I2 limitation: "
                                      "gear efficiency needs a single rigid coupling at its input node");
        }
    }

    // --- Phase H: brake friction rows (plan section 4.2/6) -----------------
    std::unordered_map<std::string, std::size_t> wheel_index_of_name;
    for (std::size_t wi = 0; wi < wheels.size(); ++wi) wheel_index_of_name.emplace(wheels[wi].name, wi);

    std::vector<std::string> braked_wheels_order; // dedup, brake_system.corners order first, then handbrake-only
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
    // matrix, plan section 4.3) ---------------------------------------------
    network_ = builder.build(vehicle_name);

    // --- Phase J: hub nodes / driven flags ----------------------------------
    hub_nodes_.resize(wheels.size());
    driven_.resize(wheels.size());
    for (std::size_t wi = 0; wi < wheels.size(); ++wi) {
        const int p = wheel_port_index[wi];
        hub_nodes_[wi] = node_of_port[static_cast<std::size_t>(p)];
        driven_[wi] = port_in_any_shaft[static_cast<std::size_t>(p)];
    }
    omega_old_.assign(static_cast<std::size_t>(network_.node_count()), real(0.0));

    // --- Phase K: components, in PowertrainDesc::components array order,
    // brakes appended last (plan section 7's "components in ARRAY order" -
    // DirectTorqueBrakes is not part of that array, so appending it after
    // is the only order that still makes every array-order component run
    // before it deterministically; brakes/engine/gear/diff never share a
    // node, so this ordering has no numeric effect either way). ------------
    components_.reserve(pt.components.size() + 1);
    for (const ComponentDesc& c : pt.components) {
        if (const auto* e = std::get_if<TorqueMapEngineDesc>(&c.params)) {
            components_.push_back(std::make_unique<TorqueMapEngine>(node_for_key(component_port_key(c.id, "crank")), *e));
        } else if (const auto* g = std::get_if<GearPairDesc>(&c.params)) {
            components_.push_back(std::make_unique<GearPair>(node_for_key(component_port_key(c.id, "in")),
                                                               node_for_key(component_port_key(c.id, "out")), *g));
        } else if (const auto* d = std::get_if<DifferentialDesc>(&c.params)) {
            components_.push_back(std::make_unique<OpenDifferential>(node_for_key(component_port_key(c.id, "carrier")),
                                                                       node_for_key(component_port_key(c.id, "left")),
                                                                       node_for_key(component_port_key(c.id, "right")), *d));
        }
    }
    components_.push_back(std::make_unique<DirectTorqueBrakes>(std::move(brake_rows), brakes.static_factor));
}

void Powertrain::begin_substep() {
    assembly_.begin_substep(network_.node_count());
    assembly_.begin_substep_rows(network_.friction_row_count());
}

void Powertrain::pre_solve_components(const ComponentSubstepCtx& ctx) {
    for (auto& c : components_) c->pre_solve(ctx, network_, assembly_);
}

void Powertrain::solve(real h) {
    const int n = network_.node_count();
    for (int i = 0; i < n; ++i) omega_old_[static_cast<std::size_t>(i)] = network_.omega(NodeId{static_cast<std::uint16_t>(i)});

    network_.solve(h, assembly_);

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
    for (auto& c : components_) c->post_solve(ctx, network_, assembly_, ledger_);
}

void Powertrain::end_tick() {
    const PowertrainEnergy L = ledger_;
    const PowertrainEnergy& L0 = ledger_tick_start_;
    // PHYS-033 I3 plan section 7.3 extends I2's identity with the new loss
    // terms (clutch/synchro/engine-friction/engagement) and the new input
    // terms (starter_in, external_reset) - every I2 graph never books any of
    // these (no clutch/gearbox/engine component exists yet), so every new
    // delta below is exactly 0.0 and this is bit-identical to I2's formula.
    ledger_.balance_error = (L.rotor_ke - L0.rotor_ke) - (L.engine_out - L0.engine_out) - (L.starter_in - L0.starter_in) -
                             (L.external_reset - L0.external_reset) + (L.wheel_out - L0.wheel_out) +
                             (L.tyre_loss - L0.tyre_loss) + (L.brake_loss - L0.brake_loss) + (L.gear_loss - L0.gear_loss) +
                             (L.clutch_loss - L0.clutch_loss) + (L.synchro_loss - L0.synchro_loss) +
                             (L.engine_friction_loss - L0.engine_friction_loss) + (L.engagement_loss - L0.engagement_loss) +
                             (L.integrator_loss - L0.integrator_loss);
}

std::uint64_t Powertrain::fold_state_hash(std::uint64_t seed) const {
    std::uint64_t h = network_.fold_state_hash(seed);
    for (const auto& c : components_) h = c->fold_state_hash(h);
    return h;
}

} // namespace ps::drivetrain
