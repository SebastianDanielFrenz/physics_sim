#include "powertrain.h"
#include "ps/drivetrain/hybrid_split.h"
#include "hybrid_charge.h"

#include "engine_params_util.h"
#include "engine_simulated.h"
#include "ps/drivetrain/limiter_range.h"

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

} // namespace

// EngineTorqueMap/Clutch/Gearbox live in core/src/drivetrain (framework-
// agnostic classes, commits 2-4) with their OWN Desc types, deliberately
// named differently from the public plan section 4.1 descriptors (that
// file's own top comment). Converts field-for-field; every field name below
// matches 1:1 (commit 6 renames were confined to the public header only).
//
// PHYS-033 P4 I7a C5d review fix (B2): moved out of the anonymous namespace
// above (and declared in powertrain.h) so tests can call this, the REAL
// production resolver, instead of a hand-duplicated copy that cannot catch a
// bug introduced here.
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
    // I4 C12: the governor's channel string is resolved to a slot index
    // separately (Powertrain::declare_control_channels/Phase K below), so
    // only the three numeric fields are copied here.
    d.has_governor = e.has_governor;
    d.governor.high_idle_rpm = e.governor.high_idle_rpm;
    d.governor.rated_rpm = e.governor.rated_rpm;
    d.governor.droop = e.governor.droop;
    // PHYS-033 P4 I7a C5: optional fuel map, copied field-for-field (Table2D/
    // FuelDesc are both plain value types - the same copy-in pattern as
    // wot_torque_nm_vs_rpm/motoring_torque_nm_vs_rpm above).
    d.has_fuel_map = e.has_fuel_map;
    d.fuel = e.fuel;
    d.fuel_map = e.fuel_map;
    // PHYS-033 P4 I7a C5d: has_fuel_model / the crude no-map path's active
    // fuel and resolved eta_i. Resolution happens HERE (not in
    // EngineTorqueMap itself) so the hot path never needs
    // TorqueMapEngineDesc::ignition/fuels/indicated_efficiency at all - see
    // EngineTorqueMap::Desc's own comment.
    d.has_fuel_model = has_fuel_model(e);
    d.rolloff = rpm_rolloff(e); // G1: disarmed while the limiter lies inside the curves' data
    if (!e.has_fuel_map && !e.fuels.empty()) {
        d.fuel = e.fuels.front(); // crude path: fuels[0] is the ACTIVE fuel (order matters)
        // Heywood, John B., Internal Combustion Engine Fundamentals,
        // McGraw-Hill, 1988 - two DIFFERENT kinds of operating point, an
        // acknowledged inconsistency between the two defaults, not hidden:
        //   SI: Sec. 15.2, p.827, full load: "indicated fuel conversion
        //   efficiency increases by about 10 percent from 0.31 to 0.34 over
        //   the speed range 1000 to 4000 rev/min." 0.325 is the DERIVED
        //   midpoint of [0.31, 0.34], not a direct quote.
        //   CI: Sec. 15.5.1 "Load and Speed", p.859, at the minimum-bsfc
        //   operating point (220 g/kWh, 1600 rev/min, 580 kPa bmep, medium-
        //   swirl NA DI diesel): "The gross indicated fuel conversion
        //   efficiency would be about 48 percent."
        // Both are single-point defaults for a model this crude, not a
        // load/speed map; indicated_efficiency (loader-validated to (0,1))
        // overrides either when the file sets it.
        //
        // Sabotage demonstrations run during the B2/B3/N7 review pass
        // (each: edit here, rebuild, run the affected test, capture the
        // failing numbers, then revert exactly - see out/c5d_progress.md):
        //  - fuels.back() instead of fuels.front(): wrong "active" fuel
        //    whenever fuels.size() > 1.
        //  - swapping kEtaSparkDefault/kEtaCompressionDefault's values:
        //    every spark engine gets the diesel default and vice versa.
        //  - unconditionally using the ignition default (ignoring an
        //    explicit indicated_efficiency override): a file-level eta_i
        //    override becomes silently unreachable.
        constexpr real kEtaSparkDefault = real(0.325);
        constexpr real kEtaCompressionDefault = real(0.48);
        d.eta_i = e.indicated_efficiency > real(0.0)
                      ? e.indicated_efficiency
                      : (e.ignition == EngineIgnition::Compression ? kEtaCompressionDefault : kEtaSparkDefault);
    }
    return d;
}

EngineTorqueMap::Desc to_engine_desc(const TorqueMapEngineDesc& e, const FuelTankDesc* tank) {
    EngineTorqueMap::Desc d = to_engine_desc(e);
    // PHYS-033 P4 I7a C6: with a tank the engine burns the TANK's fuel -
    // for the crude path this replaces fuels[0]; for a map engine it is the
    // map's own one fuel (fuels == {fuel}, enforced by the loader's path
    // check and the Powertrain constructor's value check).
    if (tank != nullptr && has_fuel_model(e)) d.fuel = tank->fuel;
    return d;
}

namespace {

// PHYS-033 P4 I7a C6: value identity of two loaded fuel files (FuelDesc has
// no path; the loader already compared the files themselves with
// std::filesystem::equivalent - this is the Powertrain constructor's
// backstop for a code-built VehicleDesc).
bool same_fuel(const FuelDesc& a, const FuelDesc& b) {
    return a.name == b.name && a.phase == b.phase && a.lhv_j_kg == b.lhv_j_kg && a.afr_stoich == b.afr_stoich &&
           a.density_kg_m3 == b.density_kg_m3 && a.has_knock == b.has_knock &&
           (!a.has_knock || (a.knock.model == b.knock.model && a.knock.ron == b.knock.ron && a.knock.mon == b.knock.mon));
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
    // Automatic gearbox plan A1: an Automatic/DualClutch box keeps a Gearbox object only for its ratio/efficiency
    // lookups and the current position (synced from the AutoTransmission every substep, never phase-stepped).
    d.type = g.type == GearboxType::Sequential ? GearboxKind::Sequential : GearboxKind::Manual;
    d.forward_ratios = g.forward_ratios;
    d.reverse_ratios = g.reverse_ratios;
    d.forward_efficiency = g.forward_efficiency;
    d.reverse_efficiency = g.reverse_efficiency;
    d.input_inertia_kgm2 = g.input_inertia_kgm2;
    d.output_inertia_kgm2 = g.output_inertia_kgm2;
    d.synchro.forward_capacity_nm = g.synchro.forward_capacity_nm;
    d.synchro.reverse_capacity_nm = g.synchro.reverse_capacity_nm;
    if (is_clutch_to_clutch(g.type)) {
        d.synchro.forward_capacity_nm.assign(g.forward_ratios.size(), real(0.0));
        d.synchro.reverse_capacity_nm.assign(g.reverse_ratios.size(), real(0.0));
    }
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
        if (const auto* e = std::get_if<TorqueMapEngineDesc>(&c.params)) {
            // I4 C12: the governor's hand-throttle channel (default 0) -
            // car_sedan/no-governor vehicles declare nothing extra here;
            // tractor.json's channel list gains "hand_throttle".
            if (e->has_governor) add(ChannelDecl{e->governor.channel, real(0.0)});
        } else if (const auto* se = std::get_if<SimulatedEngineDesc>(&c.params)) {
            // I7d D5: the turbo configuration's anti-lag switch (default = its
            // default_enabled); nothing without an anti_lag block.
            if (se->turbo_pair && se->turbo_pair->anti_lag)
                add(ChannelDecl{se->turbo_pair->anti_lag->channel, se->turbo_pair->anti_lag->default_enabled});
        } else if (const auto* bo = std::get_if<N2oBottleDesc>(&c.params)) {
            // N2O plan N1: the bottle heater's switch (default = its default_on); nothing without a heater.
            if (bo->has_heater) add(ChannelDecl{bo->heater.channel, bo->heater.default_on});
        } else if (const auto* kit = std::get_if<NitrousKitDesc>(&c.params)) {
            // N2O plan N1: the controller's arm switch (a plain 0/1 level, default = arm_default, 0 = disarmed)
            // and, with a purge, the purge switch (default 0). Declared ONLY for a vehicle with a kit.
            add(ChannelDecl{kit->controller.arm_channel, kit->controller.arm_default});
            if (kit->controller.has_purge) add(ChannelDecl{kit->controller.purge.channel, real(0.0)});
        } else if (const auto* d = std::get_if<DifferentialDesc>(&c.params)) {
            if (d->lockable) add(ChannelDecl{d->lock.channel, real(0.0)});
        } else if (const auto* tc = std::get_if<TransferCaseDesc>(&c.params)) {
            // I4 C9 (plan section 3.3): the MFWD transfer case's own engagement
            // channel (default 0 = disengaged) - no shipped vehicle declared
            // one before this, so tractor.json's channel list gains "awd" and
            // car_sedan.json's stays empty (it has no transfer_case).
            add(ChannelDecl{tc->channel, real(0.0)});
        } else if (const auto* pl = std::get_if<ParkLockDesc>(&c.params)) {
            // I4 C10 (plan section 3.4): the park pawl's own request channel
            // (default 0 = released) - tractor.json's channel list gains
            // "park_lock"; car_sedan.json's stays unchanged (no park_lock).
            add(ChannelDecl{pl->channel, real(0.0)});
        } else if (const auto* pto = std::get_if<PtoDesc>(&c.params)) {
            // I4 C11 (plan section 3.5): the pto's own engagement channel
            // (default 0 = disengaged) and mode-select channel (default 540
            // rpm) - tractor.json's channel list gains "pto"/"pto_speed";
            // car_sedan.json's stays unchanged (no pto).
            add(ChannelDecl{pto->channel, real(0.0)});
            add(ChannelDecl{pto->speed_channel, real(540.0)});
        } else if (const auto* dyno = std::get_if<LoadDynoDesc>(&c.params)) {
            // I4 C11: a load_dyno's scale channel is OPTIONAL (empty name =
            // fixed scale 1, no channel declared at all) - test/bench only,
            // no shipped vehicle has one.
            if (!dyno->scale_channel.empty()) add(ChannelDecl{dyno->scale_channel, real(1.0)});
        } else if (const auto* tcd = std::get_if<ThrottleControlDesc>(&c.params)) {
            // I7a C3: an EXPLICIT throttle_control component's own hand-lever
            // channel (default 0), if it names one - the IMPLICIT default
            // stage's hand lever (an engine's own governor channel, when it
            // has one) is already declared by the TorqueMapEngineDesc branch
            // above, so no separate declaration is needed for that case.
            if (!tcd->hand_lever_channel.empty()) add(ChannelDecl{tcd->hand_lever_channel, real(0.0)});
        } else if (const auto* motor = std::get_if<ElectricMotorDesc>(&c.params)) {
            add(ChannelDecl{motor->ready_channel,real(1)});
            for(const auto* name:{&motor->throttle_channel,&motor->regen_channel,&motor->direction_channel})
                if(!name->empty())add(ChannelDecl{*name,real(0)});
        } else if(std::get_if<HybridControlDesc>(&c.params)) {
            add(ChannelDecl{"assist.drive_mode",real(1)});
            add(ChannelDecl{"electric.ready",real(1)});
        } else if (const auto* cell = std::get_if<FuelCellDesc>(&c.params)) {
            add(ChannelDecl{cell->ready_channel,real(1)});
        } else if (const auto* gb = std::get_if<GearboxDesc>(&c.params)) {
            // Automatic gearbox plan A1: the D/S mode switch of an Automatic/DualClutch box (default 0 = D).
            if (is_clutch_to_clutch(gb->type)) add(ChannelDecl{kTransmissionModeChannel, real(0.0)});
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
    int transfer_case_component_index = -1; // I4 C9
    int park_lock_component_index = -1;     // I4 C10
    int pto_component_index = -1;           // I4 C11
    int throttle_control_component_index = -1; // I7a C3
    std::string pto_out_port_key;           // I4 C11: exempt from the "not connected" check below
    std::string turbine_port_key;           // automatic gearbox plan A1: the converter turbine (internal, no shaft)
    for (std::size_t ci = 0; ci < pt.components.size(); ++ci) {
        const ComponentDesc& c = pt.components[ci];
        if (c.id.empty()) throw std::runtime_error("vehicle '" + vehicle_name + "': a powertrain component has an empty id");
        if (!component_index_of.emplace(c.id, static_cast<int>(ci)).second) {
            throw std::runtime_error("vehicle '" + vehicle_name + "': powertrain component id '" + c.id + "' is not unique");
        }
        if (const auto* motor = std::get_if<ElectricMotorDesc>(&c.params)) {
            add_port(component_port_key(c.id,"shaft"),motor->inertia_kgm2,motor->axis_local);
        } else if (const auto* e = std::get_if<TorqueMapEngineDesc>(&c.params)) {
            if (engine_component_index >= 0) {
                throw std::runtime_error("vehicle '" + vehicle_name + "': more than one engine component - I3 supports at most one");
            }
            engine_component_index = static_cast<int>(ci);
            add_port(component_port_key(c.id, "crank"), e->inertia_kgm2, e->crank_axis_local);
        } else if (const auto* se = std::get_if<SimulatedEngineDesc>(&c.params)) {
            // PHYS-033 P4 I7b C12: the mean-value engine takes the same single
            // engine slot and the same "crank" port as the graph engine.
            if (engine_component_index >= 0) {
                throw std::runtime_error("vehicle '" + vehicle_name + "': more than one engine component - I3 supports at most one");
            }
            engine_component_index = static_cast<int>(ci);
            add_port(component_port_key(c.id, "crank"), se->inertia_kgm2, se->crank_axis_local);
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
            if ((g->type == GearboxType::Automatic || g->type == GearboxType::Cvt) && g->converter) { // CVT plan C1: a converter CVT too
                // Automatic gearbox plan A1: `in` is the converter's pump; the turbine is an internal port carrying
                // the turbine and the gear set's input inertia (not connected by a shaft - exempt below).
                add_port(component_port_key(c.id, "in"), g->converter->pump_inertia_kgm2, g->input_axis_local);
                add_port(component_port_key(c.id, "out"), g->output_inertia_kgm2, g->output_axis_local);
                turbine_port_key = component_port_key(c.id, "turbine");
                add_port(turbine_port_key, g->converter->turbine_inertia_kgm2 + g->input_inertia_kgm2, g->input_axis_local);
            } else {
                add_port(component_port_key(c.id, "in"), g->input_inertia_kgm2, g->input_axis_local);
                add_port(component_port_key(c.id, "out"), g->output_inertia_kgm2, g->output_axis_local);
            }
        } else if (const auto* gp = std::get_if<GearPairDesc>(&c.params)) {
            add_port(component_port_key(c.id, "in"), gp->inertia_in_kgm2, gp->in_axis_local);
            add_port(component_port_key(c.id, "out"), gp->inertia_out_kgm2, gp->out_axis_local);
        } else if (const auto* d = std::get_if<DifferentialDesc>(&c.params)) {
            add_port(component_port_key(c.id, "carrier"), d->carrier_inertia_kgm2, d->carrier_axis_local);
            add_port(component_port_key(c.id, "left"), real(0.0), Vec3::zero());
            add_port(component_port_key(c.id, "right"), real(0.0), Vec3::zero());
        } else if (const auto* tc = std::get_if<TransferCaseDesc>(&c.params)) {
            // I4 C9 (plan section 3.3): one transfer_case per vehicle, like
            // engine/clutch/gearbox above (not gear_pair/differential, which
            // may repeat) - a second one would need per-instance channel/hash
            // disambiguation the plan never specifies.
            if (transfer_case_component_index >= 0) {
                throw std::runtime_error("vehicle '" + vehicle_name + "': more than one transfer_case component - I4 C9 supports at most one");
            }
            transfer_case_component_index = static_cast<int>(ci);
            // `rear` gets zero inertia (a pure kinematic passthrough port,
            // mirroring a differential's zero-inertia left/right ports) -
            // `in`/`front` share axis_local (one physical shaft, plan section 6.7).
            add_port(component_port_key(c.id, "in"), tc->in_inertia_kgm2, tc->axis_local);
            add_port(component_port_key(c.id, "rear"), real(0.0), Vec3::zero());
            add_port(component_port_key(c.id, "front"), tc->front_inertia_kgm2, tc->axis_local);
        } else if (std::get_if<ParkLockDesc>(&c.params)) {
            // I4 C10 (plan section 3.4): at most one park_lock per vehicle -
            // same reasoning as transfer_case above (no per-instance channel/
            // hash disambiguation for a second one).
            if (park_lock_component_index >= 0) {
                throw std::runtime_error("vehicle '" + vehicle_name + "': more than one park_lock component - I4 C10 supports at most one");
            }
            park_lock_component_index = static_cast<int>(ci);
            // A pure friction-row node, no rotor-reaction axis of its own
            // (it never carries a physical inertia beyond whatever port it
            // shares its shaft with).
            add_port(component_port_key(c.id, "shaft"), real(0.0), Vec3::zero());
        } else if (const auto* pto = std::get_if<PtoDesc>(&c.params)) {
            // I4 C11 (plan section 3.5): at most one pto per vehicle - same
            // reasoning as transfer_case/park_lock above (no per-instance
            // channel/hash disambiguation for a second one). `in` carries no
            // inertia of its own (a pure connection point on the crank
            // shaft, like a differential's left/right ports); `out` carries
            // `out_inertia_kgm2` and is EXEMPT from the "not connected by
            // any shaft" check below - it may stand alone (disengaged stub
            // brake only) or be shafted to an external load (e.g. a test's
            // load_dyno).
            if (pto_component_index >= 0) {
                throw std::runtime_error("vehicle '" + vehicle_name + "': more than one pto component - I4 C11 supports at most one");
            }
            pto_component_index = static_cast<int>(ci);
            add_port(component_port_key(c.id, "in"), real(0.0), pto->axis_local);
            const int pto_out_idx = add_port(component_port_key(c.id, "out"), pto->out_inertia_kgm2, pto->axis_local);
            pto_out_port_key = port_key[static_cast<std::size_t>(pto_out_idx)];
        } else if (const auto* dyno = std::get_if<LoadDynoDesc>(&c.params)) {
            // I4 C11: test/bench only, any number per vehicle. Its `shaft`
            // port gets NO connectivity exemption (unlike pto.out) - a
            // load_dyno with nothing wired to it would be pointless and is
            // almost certainly a test typo.
            add_port(component_port_key(c.id, "shaft"), dyno->inertia_kgm2, dyno->axis_local);
        } else if (std::get_if<ThrottleControlDesc>(&c.params)) {
            // I7a C3 (plan section 3.4): at most one throttle_control per
            // vehicle - a pure control-signal seam (throttle_stage.h), no
            // shaft-network port/node of its own, so nothing to add_port()
            // here. Phase K below resolves it against engine_component_index.
            if (throttle_control_component_index >= 0) {
                throw std::runtime_error("vehicle '" + vehicle_name + "': more than one throttle_control component - I7a C3 supports at most one");
            }
            throttle_control_component_index = static_cast<int>(ci);
        } else if (std::get_if<FuelTankDesc>(&c.params)) {
            // PHYS-033 P4 I7a C6: an energy store, no shaft port/node/row -
            // wired to the engine through PowertrainDesc::fluid below.
        } else if (std::get_if<N2oBottleDesc>(&c.params) || std::get_if<NitrousKitDesc>(&c.params)) {
            // N2O plan N1: no shaft port/node/row; the NitrousSystem is a typed member
            // built below from the fluid-linked bottle + kit.
        }
    }
    if (throttle_control_component_index >= 0 && engine_component_index < 0) {
        throw std::runtime_error("vehicle '" + vehicle_name + "': throttle_control component requires an engine component");
    }

    // PHYS-033 P4 I7a C6: fluid links. The loader rejects every malformed
    // link with its own pinned message; this is the backstop for a
    // code-built VehicleDesc. At most one tank feeds the (single) engine, and
    // every tank must feed it.
    int fuel_tank_component_index = -1;
    int n2o_bottle_component_index = -1, nitrous_kit_component_index = -1; // N2O plan N1
    bool kit_feeds_engine = false;
    for (const FluidLinkDesc& link : pt.fluid) {
        const auto from_it = link.from.is_wheel ? component_index_of.end() : component_index_of.find(link.from.component);
        const auto to_it = link.to.is_wheel ? component_index_of.end() : component_index_of.find(link.to.component);
        const ComponentParams* from_params =
            from_it == component_index_of.end() ? nullptr : &pt.components[static_cast<std::size_t>(from_it->second)].params;
        const ComponentParams* to_params = to_it == component_index_of.end() ? nullptr : &pt.components[static_cast<std::size_t>(to_it->second)].params;
        const bool from_bottle = from_params && std::holds_alternative<N2oBottleDesc>(*from_params);
        const bool from_kit = from_params && std::holds_alternative<NitrousKitDesc>(*from_params);
        if (from_bottle) {
            // N2O plan N1: "<bottle>.out" -> "<kit>.n2o"
            if (link.from.port != "out" || !to_params || !std::holds_alternative<NitrousKitDesc>(*to_params) || link.to.port != "n2o") {
                throw std::runtime_error("vehicle '" + vehicle_name + "': fluid link '" + link.from.component + "." + link.from.port + "' -> '" +
                                          link.to.component + "." + link.to.port + "': an n2o_bottle's \"out\" port feeds a nitrous_kit's \"n2o\" port");
            }
            if (nitrous_kit_component_index >= 0 || n2o_bottle_component_index >= 0) {
                throw std::runtime_error("vehicle '" + vehicle_name + "': nitrous_kit '" + link.to.component + "' n2o port is fed by more than one fluid link");
            }
            n2o_bottle_component_index = from_it->second;
            nitrous_kit_component_index = to_it->second;
            continue;
        }
        if (from_kit) {
            // N2O plan N1: "<kit>.out" -> "<engine>.nitrous"
            if (link.from.port != "out" || !to_params || link.to.port != "nitrous" ||
                !(std::holds_alternative<TorqueMapEngineDesc>(*to_params) || std::holds_alternative<SimulatedEngineDesc>(*to_params))) {
                throw std::runtime_error("vehicle '" + vehicle_name + "': fluid link '" + link.from.component + "." + link.from.port + "' -> '" +
                                          link.to.component + "." + link.to.port + "': a nitrous_kit's \"out\" port feeds an engine's \"nitrous\" port");
            }
            if (kit_feeds_engine) {
                throw std::runtime_error("vehicle '" + vehicle_name + "': engine '" + link.to.component +
                                          "' nitrous port is fed by more than one fluid link");
            }
            kit_feeds_engine = true;
            continue;
        }
        if (link.from.is_wheel || from_it == component_index_of.end() || link.from.port != "out" ||
            !std::holds_alternative<FuelTankDesc>(pt.components[static_cast<std::size_t>(from_it->second)].params)) {
            throw std::runtime_error("vehicle '" + vehicle_name + "': fluid link source '" + link.from.component + "." + link.from.port +
                                      "' is not a fuel_tank's \"out\" port");
        }
        if (link.to.is_wheel || to_it == component_index_of.end() || link.to.port != "fuel" ||
            !(std::holds_alternative<TorqueMapEngineDesc>(pt.components[static_cast<std::size_t>(to_it->second)].params) ||
              std::holds_alternative<SimulatedEngineDesc>(pt.components[static_cast<std::size_t>(to_it->second)].params))) {
            throw std::runtime_error("vehicle '" + vehicle_name + "': fluid link target '" + link.to.component + "." + link.to.port +
                                      "' is not an engine's \"fuel\" port");
        }
        if (fuel_tank_component_index >= 0) {
            throw std::runtime_error("vehicle '" + vehicle_name + "': engine '" + link.to.component +
                                      "' fuel port is fed by more than one fluid link");
        }
        fuel_tank_component_index = from_it->second;
    }
    for (std::size_t ci = 0; ci < pt.components.size(); ++ci) {
        if (std::holds_alternative<FuelTankDesc>(pt.components[ci].params) && static_cast<int>(ci) != fuel_tank_component_index) {
            throw std::runtime_error("vehicle '" + vehicle_name + "': fuel_tank '" + pt.components[ci].id + "' has no consumer");
        }
        if (std::holds_alternative<N2oBottleDesc>(pt.components[ci].params) && static_cast<int>(ci) != n2o_bottle_component_index) {
            throw std::runtime_error("vehicle '" + vehicle_name + "': n2o_bottle '" + pt.components[ci].id + "' is not linked to a nitrous_kit");
        }
        if (std::holds_alternative<NitrousKitDesc>(pt.components[ci].params)) {
            if (static_cast<int>(ci) != nitrous_kit_component_index) {
                throw std::runtime_error("vehicle '" + vehicle_name + "': nitrous_kit '" + pt.components[ci].id + "' has no n2o_bottle");
            }
            if (!kit_feeds_engine) {
                throw std::runtime_error("vehicle '" + vehicle_name + "': nitrous_kit '" + pt.components[ci].id + "' has no consumer");
            }
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

    // PHYS-033 P4 I4 C13: compliant shafts. Validated here as well as in the
    // loader (vehicle_io.cpp) because a test may mutate a loaded desc in
    // code. Under Rigid, stiffness/damping are ignored (every shaft merges
    // its ports exactly as before); under Compliant, a shaft with stiffness
    // > 0 must list exactly 2 ports, which are NOT merged - they become two
    // nodes joined by a spring-damper (compliant_pending below, turned into
    // a network coupling in Phase I).
    struct CompliantPending {
        int port_a;
        int port_b;
        real k;
        real c;
        int shaft_index;
    };
    std::vector<CompliantPending> compliant_pending;
    const bool compliance_on = pt.compliance == drivetrain::Compliance::Compliant;
    for (std::size_t si = 0; si < pt.shafts.size(); ++si) {
        const ShaftDesc& shaft = pt.shafts[si];
        const std::string sctx = "vehicle '" + vehicle_name + "': powertrain.shafts[" + std::to_string(si) + "]";
        if (!(shaft.stiffness >= real(0.0))) throw std::runtime_error(sctx + ": stiffness must be >= 0");
        if (!(shaft.damping >= real(0.0))) throw std::runtime_error(sctx + ": damping must be >= 0");
        if (shaft.damping > real(0.0) && shaft.stiffness == real(0.0)) {
            throw std::runtime_error(sctx + ": damping > 0 requires stiffness > 0 (a pure damper shaft is not supported)");
        }
        const bool is_compliant = compliance_on && shaft.stiffness > real(0.0);
        if (is_compliant && shaft.ports.size() != 2) {
            throw std::runtime_error(sctx + ": a compliant shaft (stiffness > 0) must list exactly 2 ports, got " +
                                     std::to_string(shaft.ports.size()));
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
        if (is_compliant) {
            compliant_pending.push_back(CompliantPending{ports_here[0], ports_here[1], shaft.stiffness, shaft.damping, static_cast<int>(si)});
            continue; // not merged: two nodes, coupled in Phase I
        }
        for (std::size_t i = 1; i < ports_here.size(); ++i) uf.unite(ports_here[0], ports_here[i]);
    }
    if (compliance_on && compliant_pending.empty()) {
        throw std::runtime_error("vehicle '" + vehicle_name + "': powertrain.compliance is \"compliant\" but no shaft has stiffness > 0");
    }
    if (compliant_pending.size() > kMaxCompliantShafts) {
        throw std::runtime_error("vehicle '" + vehicle_name + "': at most " + std::to_string(kMaxCompliantShafts) +
                                 " compliant shafts are supported, got " + std::to_string(compliant_pending.size()));
    }

    for (int p = 0; p < port_count; ++p) {
        const bool is_wheel_port = port_key[static_cast<std::size_t>(p)].rfind("wheel:", 0) == 0;
        // I4 C11 (plan section 3.5): the pto's own `out` port is exempt - it
        // may stand alone (disengaged stub brake only, no external load).
        const bool is_pto_out_port = !pto_out_port_key.empty() && port_key[static_cast<std::size_t>(p)] == pto_out_port_key;
        const bool is_turbine_port = !turbine_port_key.empty() && port_key[static_cast<std::size_t>(p)] == turbine_port_key;
        if (!is_wheel_port && !is_pto_out_port && !is_turbine_port && !port_in_any_shaft[static_cast<std::size_t>(p)]) {
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
    const bool c2c_gearbox = gearbox_desc && is_clutch_to_clutch(gearbox_desc->type);
    // Automatic gearbox plan A1: a clutch-to-clutch box has ONE topology state (its gears are friction rows).
    const int num_gear_states = (gearbox_desc && !c2c_gearbox) ? static_cast<int>(gearbox_desc->reverse_ratios.size()) + 1 + static_cast<int>(gearbox_desc->forward_ratios.size()) : 1;
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
    if (gearbox_desc) {
        gearbox_ = std::make_unique<Gearbox>(to_gearbox_desc(*gearbox_desc));
        // Transmission plan T0: the sound-side data (tooth form, tooth counts, fallback mesh order) - never hashed.
        gearbox_sound_.gear_type = gearbox_desc->gear_type;
        gearbox_sound_.forward_teeth = gearbox_desc->forward_teeth;
        gearbox_sound_.reverse_teeth = gearbox_desc->reverse_teeth;
        gearbox_sound_.input_pair_teeth = gearbox_desc->input_pair_teeth;
        gearbox_sound_.mesh_order = gearbox_desc->mesh_order;
    }

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
        gearbox_in_n = node_for_key(turbine_port_key.empty() ? component_port_key(pt.components[static_cast<std::size_t>(gearbox_component_index)].id, "in")
                                                             : turbine_port_key);
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
        if (gearbox_desc && !c2c_gearbox) {
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

    // transfer_case (I4 C9, plan section 3.3): the `rear` rigid row,
    // omega(in) = rear_ratio*omega(rear), replicated across EVERY state -
    // exactly gear_pair's own pattern above (a mechanical rigid coupling that
    // holds regardless of the gearbox's position/any diff lock bit). Only
    // `rear` is a rigid row; `front` is the friction row built in Phase G below.
    for (const ComponentDesc& c : pt.components) {
        if (const auto* tc = std::get_if<TransferCaseDesc>(&c.params)) {
            const NodeId in_n = node_for_key(component_port_key(c.id, "in"));
            const NodeId rear_n = node_for_key(component_port_key(c.id, "rear"));
            for (int state = 0; state < num_states; ++state) {
                RigidRow row;
                row.terms.emplace_back(in_n.index, real(1.0));
                row.terms.emplace_back(rear_n.index, -tc->rear_ratio);
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
        if (c2c_gearbox) {
            // Automatic gearbox plan A1: the second shift clutch (row B; row A is the synchro slot above), then the
            // torque converter's lock-up clutch (pump -> turbine) and its implicit slip coupling. Only a clutch-to-
            // clutch box adds these, so every Manual/Sequential vehicle's row ids are unchanged.
            shift_row_b_ = builder.add_friction_row(gearbox_in_, gearbox_out_);
            if (!turbine_port_key.empty()) {
                converter_pump_ = node_for_key(component_port_key(pt.components[static_cast<std::size_t>(gearbox_component_index)].id, "in"));
                lockup_row_ = builder.add_friction_row(converter_pump_, gearbox_in_);
                converter_coupling_ = builder.add_coupling(converter_pump_, gearbox_in_);
                converter_.emplace(*gearbox_desc->converter);
            }
        }
    }
    // I4 C9 (plan section 3.3): the transfer_case's own `front` friction row,
    // appended AFTER clutch/engine/synchro above so no pre-existing vehicle's
    // friction-row ids shift (car_sedan.json has no transfer_case at all).
    if (transfer_case_component_index >= 0) {
        const std::string& tcid = pt.components[static_cast<std::size_t>(transfer_case_component_index)].id;
        const NodeId tc_in_n = node_for_key(component_port_key(tcid, "in"));
        const NodeId tc_front_n = node_for_key(component_port_key(tcid, "front"));
        transfer_row_ = builder.add_friction_row(tc_in_n, tc_front_n);
    }
    // I4 C10 (plan section 3.4): the park pawl's own friction row (shaft to
    // GROUND), appended AFTER the transfer_case row above so no pre-existing
    // vehicle's friction-row ids shift (car_sedan.json has no park_lock at
    // all; tractor.json has no hash gate yet to preserve).
    if (park_lock_component_index >= 0) {
        const std::string& plid = pt.components[static_cast<std::size_t>(park_lock_component_index)].id;
        const NodeId pl_shaft_n = node_for_key(component_port_key(plid, "shaft"));
        park_lock_row_ = builder.add_friction_row(pl_shaft_n, kGround);
    }
    // I4 C11 (plan section 3.5): the pto's own friction row (engine/crank-
    // side `in`, PTO-side `out`), appended AFTER park_lock above so no
    // pre-existing vehicle's friction-row ids shift (car_sedan.json has no
    // pto at all; tractor.json gains this row at I4 C11).
    if (pto_component_index >= 0) {
        const std::string& pid = pt.components[static_cast<std::size_t>(pto_component_index)].id;
        const NodeId pto_in_n = node_for_key(component_port_key(pid, "in"));
        const NodeId pto_out_n = node_for_key(component_port_key(pid, "out"));
        pto_row_ = builder.add_friction_row(pto_in_n, pto_out_n);
    }
    // I4 C11: each load_dyno component in Brake mode gets its own friction
    // row to GROUND, appended in component order right after the pto row
    // above - test/bench only, so this never fires for any shipped vehicle
    // as of I4 C11 (a test rig adds a load_dyno to a mutated VehicleDesc in
    // code, never via a data file).
    std::unordered_map<std::string, int> dyno_brake_row_of_id; // component id -> FrictionRowId::index, Brake mode only
    for (const ComponentDesc& c : pt.components) {
        const auto* dyno = std::get_if<LoadDynoDesc>(&c.params);
        if (dyno == nullptr || dyno->mode != LoadDynoMode::Brake) continue;
        const NodeId shaft_n = node_for_key(component_port_key(c.id, "shaft"));
        dyno_brake_row_of_id[c.id] = builder.add_friction_row(shaft_n, kGround).index;
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
        row.wheel = wi;
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

    // --- Phase H3 (PHYS-033 P4 I4 C13): one implicit coupling per compliant
    // shaft, shafts[] order, AFTER every C6 viscous-diff coupling (so no
    // existing coupling id moves). A Rigid vehicle adds none.
    compliant_.reserve(compliant_pending.size());
    for (const CompliantPending& cp : compliant_pending) {
        const std::string sctx = "vehicle '" + vehicle_name + "': powertrain.shafts[" + std::to_string(cp.shaft_index) + "]";
        const NodeId na = node_of_port[static_cast<std::size_t>(cp.port_a)];
        const NodeId nb = node_of_port[static_cast<std::size_t>(cp.port_b)];
        if (na == nb) {
            throw std::runtime_error(sctx + ": compliant shaft's two ports '" + port_key[static_cast<std::size_t>(cp.port_a)] + "' and '" +
                                     port_key[static_cast<std::size_t>(cp.port_b)] +
                                     "' are already rigidly joined by other shafts (a compliant shaft must join two distinct nodes)");
        }
        for (const int pp : {cp.port_a, cp.port_b}) {
            if (!(root_inertia[uf.find(pp)] > real(0.0))) {
                throw std::runtime_error(sctx + ": compliant shaft endpoint '" + port_key[static_cast<std::size_t>(pp)] +
                                         "' has zero inertia (both ends of a compliant shaft need inertia > 0 - give the component's port an "
                                         "inertia, or make that shaft rigid)");
            }
        }
        CompliantShaft cs;
        cs.a = na;
        cs.b = nb;
        cs.k = cp.k;
        cs.c = cp.c;
        cs.shaft_index = cp.shaft_index;
        cs.coupling = builder.add_coupling(na, nb);
        compliant_.push_back(cs);
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
        // I4 C9 (plan section 2.5): a transfer_case restricts the speedo walk
        // to its `rear` output only - `front` (MFWD) never contributes a K
        // estimate, so adding the front axle cannot change the tractor's
        // speedo (bit-identical before/after, confirmed by the test).
        std::vector<NodeId> tc_in, tc_rear;
        std::vector<real> tc_rear_ratio;
        for (const ComponentDesc& c : pt.components) {
            if (const auto* g = std::get_if<GearPairDesc>(&c.params)) {
                gp_in.push_back(node_for_key(component_port_key(c.id, "in")));
                gp_out.push_back(node_for_key(component_port_key(c.id, "out")));
                gp_ratio.push_back(g->ratio);
            } else if (const auto* tc = std::get_if<TransferCaseDesc>(&c.params)) {
                tc_in.push_back(node_for_key(component_port_key(c.id, "in")));
                tc_rear.push_back(node_for_key(component_port_key(c.id, "rear")));
                tc_rear_ratio.push_back(tc->rear_ratio);
            }
        }
        std::vector<std::uint8_t> reached(hub_nodes_.size(), 0);
        std::vector<real> leaf_path_ratios;
        struct KResult {
            bool ok = false;
            real k = 0.0;
        };
        constexpr int kMaxSpeedoDepth = 32; // guards against a malformed cyclic graph
        std::vector<NodeId> speedo_stack; // C13: current walk path, compliant-shaft crossings only
        std::function<KResult(NodeId, real, int)> k_of = [&](NodeId n, real path_ratio, int depth) -> KResult {
            if (depth > kMaxSpeedoDepth) return {};
            if (!compliant_.empty()) speedo_stack.push_back(n); // C13, popped below
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
            // I4 C9: a transfer_case's `in` recurses ONLY into `rear` (never
            // `front`) - see this block's own top comment.
            for (std::size_t ti = 0; ti < tc_in.size(); ++ti) {
                if (tc_in[ti] != n) continue;
                const KResult kr = k_of(tc_rear[ti], path_ratio * tc_rear_ratio[ti], depth + 1);
                if (kr.ok) {
                    est_sum += tc_rear_ratio[ti] * kr.k;
                    ++est_count;
                }
            }
            // PHYS-033 P4 I4 C13: a compliant shaft is kinematically a ratio-1
            // shaft at steady no-slip rolling (constant twist), so the walk
            // crosses it like a rigid join - in either port order, but never
            // back onto a node already on the current walk path (a compliant
            // shaft has no "down" direction of its own). No compliant shaft
            // -> this loop is empty and the walk is literally unchanged.
            if (!compliant_.empty()) {
                for (const CompliantShaft& cs : compliant_) {
                    NodeId other{};
                    if (cs.a == n) other = cs.b;
                    else if (cs.b == n) other = cs.a;
                    else continue;
                    if (std::find(speedo_stack.begin(), speedo_stack.end(), other) != speedo_stack.end()) continue;
                    const KResult ko = k_of(other, path_ratio, depth + 1);
                    if (ko.ok) {
                        est_sum += ko.k;
                        ++est_count;
                    }
                }
                speedo_stack.pop_back();
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
    if (!compliant_.empty()) compliant_stuck_before_.assign(static_cast<std::size_t>(network_.friction_row_count()), 0);

    // --- Phase K: engine/clutch/gearbox typed members -----------------------
    if (engine_component_index >= 0) {
        // PHYS-033 P4 I7b C12: exactly one of tm/sim is set - the graph
        // engine (EngineTorqueMap) or the mean-value engine (SimulatedEngine).
        const ComponentParams& e_params = pt.components[static_cast<std::size_t>(engine_component_index)].params;
        const auto* tm = std::get_if<TorqueMapEngineDesc>(&e_params);
        const auto* sim = std::get_if<SimulatedEngineDesc>(&e_params);
        const std::vector<FuelDesc>& e_fuels = tm ? tm->fuels : sim->fuels;
        const bool e_has_fuel_model = tm ? has_fuel_model(*tm) : has_fuel_model(*sim);
        const ThrottleInput e_throttle_input = tm ? tm->throttle_input : sim->throttle_input;
        const bool e_has_governor = tm && tm->has_governor; // the mean-value engine has no governor (I7b)
        const std::string& e_id = pt.components[static_cast<std::size_t>(engine_component_index)].id;
        // PHYS-033 P4 I7a C6: the tank feeding this engine, if any.
        const FuelTankDesc* tank_desc = nullptr;
        if (fuel_tank_component_index >= 0) {
            const ComponentDesc& tc = pt.components[static_cast<std::size_t>(fuel_tank_component_index)];
            tank_desc = &std::get<FuelTankDesc>(tc.params);
            if (!e_has_fuel_model) {
                throw std::runtime_error("vehicle '" + vehicle_name + "': engine '" + e_id + "' has no fuel model - fuel_tank '" + tc.id +
                                          "' cannot feed it");
            }
            if (std::none_of(e_fuels.begin(), e_fuels.end(), [&](const FuelDesc& f) { return same_fuel(f, tank_desc->fuel); })) {
                throw std::runtime_error("vehicle '" + vehicle_name + "': tank fuel '" + tank_desc->fuel.name + "' is not in engine '" +
                                          e_id + "''s fuels list");
            }
            if (!(std::isfinite(tank_desc->capacity_kg) && tank_desc->capacity_kg > real(0.0)) ||
                !(std::isfinite(tank_desc->initial_kg) && tank_desc->initial_kg >= real(0.0)) ||
                tank_desc->initial_kg > tank_desc->capacity_kg) {
                throw std::runtime_error("vehicle '" + vehicle_name + "': fuel_tank '" + tc.id +
                                          "' needs finite capacity_kg > 0 and 0 <= initial_kg <= capacity_kg");
            }
            fuel_tank_ = std::make_unique<FuelTank>(tank_desc->fuel, tank_desc->capacity_kg, tank_desc->initial_kg);
        }
        EngineTorqueMap* tm_engine_ptr = nullptr;
        if (tm) {
            EngineTorqueMap::Desc tm_desc = to_engine_desc(*tm, tank_desc);
            if (pt.thermal.has_value()) tm_desc.thermal_friction_fraction = pt.thermal->engine.motoring_friction_fraction;
            auto tm_engine = std::make_unique<EngineTorqueMap>(std::move(tm_desc));
            tm_engine_ptr = tm_engine.get(); // pedal plan P1: the law is set once the throttle stage kind is known
            engine_ = std::move(tm_engine);
        } else {
            // PHYS-033 P4 I7b C12: SimulatedEngine::build_maps takes LHV/AFR
            // from fuels[0], so the engine is built from a copy whose fuels
            // list holds ONLY the fuel actually burnt - the tank's (already
            // checked to be in the list) or, without a tank, fuels[0] (the
            // same rule as the graph engine's to_engine_desc). Review R7.
            //
            // N1 (vault PHYS-049/050 follow-ups): the production loader
            // (core/src/io/vehicle_io.cpp) already rejects an empty "fuels"
            // list for a simulated engine, so this can only be reached by a
            // CODE-BUILT SimulatedEngineDesc (tests, or a future non-JSON
            // caller) that skips that check. Without a tank, sim->fuels
            // must be non-empty or sim->fuels.front() below is UB.
            if (!tank_desc && sim->fuels.empty()) {
                throw std::runtime_error("vehicle '" + vehicle_name + "': simulated engine '" + e_id +
                                          "' has an empty fuels list and no fuel_tank - nothing to burn");
            }
            SimulatedEngineDesc burnt = *sim;
            burnt.fuels.assign(1, tank_desc ? tank_desc->fuel : sim->fuels.front());
            // Engine garage P7: a generated knock limit has one table per listed fuel - use the burnt fuel's (the
            // first match in the list, the same match the tank check above made). Without per-fuel tables a no-op.
            burnt_fuel_index_ = 0;
            if (tank_desc) {
                for (std::size_t fi = 0; fi < sim->fuels.size(); ++fi) {
                    if (same_fuel(sim->fuels[fi], tank_desc->fuel)) {
                        burnt_fuel_index_ = fi;
                        break;
                    }
                }
            }
            select_knock_fuel(burnt.maps, burnt_fuel_index_);
            for (auto& pm : burnt.cam_profile_maps) select_knock_fuel(pm, burnt_fuel_index_); // engine garage P9
            for (auto& pm : burnt.lift_node_maps) select_knock_fuel(pm, burnt_fuel_index_);   // engine garage P10
            auto sim_engine = std::make_unique<SimulatedEngine>(burnt);
            sim_engine_ptr_ = sim_engine.get(); // PHYS-033 P4 I7c C16b
            // I7d D5: resolve the anti-lag channel's slot like the other declared channels.
            if (burnt.turbo_pair && burnt.turbo_pair->anti_lag) {
                anti_lag_channel_name_ = burnt.turbo_pair->anti_lag->channel;
                const std::vector<ChannelDecl> al_decls = declare_control_channels(pt);
                for (std::size_t i = 0; i < al_decls.size(); ++i) {
                    if (al_decls[i].name == anti_lag_channel_name_) {
                        anti_lag_channel_slot_ = static_cast<int>(i);
                        break;
                    }
                }
                anti_lag_channel_live_ = anti_lag_channel_slot_ >= 0 && pt.anti_lag_fitted; // P8: no switch fitted = default_enabled only
            }
            simulated_engine_desc_ = std::move(burnt);
            engine_ = std::move(sim_engine);
        }
        engine_component_id_ = e_id; // PHYS-033 P4 I7c C16b
        engine_friction_row_ = engine_row;
        engine_has_fuel_model_ = e_has_fuel_model; // PHYS-033 P4 I7a C5, broadened C5d: gates fuel/heat booking below

        // N2O plan N1: the Powertrain-owned nitrous system from the fluid-linked bottle + kit. The kit's fuel jet
        // is sized with the density of the fuel the engine burns (the tank's, else fuels[0]).
        if (nitrous_kit_component_index >= 0) {
            if (!tank_desc && e_fuels.empty()) {
                throw std::runtime_error("vehicle '" + vehicle_name + "': nitrous_kit needs an engine with a fuel (its jet density)");
            }
            const FuelDesc& burnt_fuel = tank_desc ? tank_desc->fuel : e_fuels.front();
            const N2oBottleDesc& bottle_desc = std::get<N2oBottleDesc>(pt.components[static_cast<std::size_t>(n2o_bottle_component_index)].params);
            const NitrousKitDesc& kit_desc = std::get<NitrousKitDesc>(pt.components[static_cast<std::size_t>(nitrous_kit_component_index)].params);
            // N2O plan N4 backstop (the vehicle loader rejects the same): the heater, arm and purge switches are three
            // different channels; declared channels merge by name, so a clash would silently make two of them one.
            if (bottle_desc.has_heater && bottle_desc.heater.channel == kit_desc.controller.arm_channel) {
                throw std::runtime_error("vehicle '" + vehicle_name + "': the bottle heater channel '" + bottle_desc.heater.channel +
                                          "' is also the nitrous controller's arm channel");
            }
            if (kit_desc.controller.has_purge && bottle_desc.has_heater && bottle_desc.heater.channel == kit_desc.controller.purge.channel) {
                throw std::runtime_error("vehicle '" + vehicle_name + "': the bottle heater channel '" + bottle_desc.heater.channel +
                                          "' is also the nitrous controller's purge channel");
            }
            if (kit_desc.controller.has_purge && kit_desc.controller.purge.channel == kit_desc.controller.arm_channel) {
                throw std::runtime_error("vehicle '" + vehicle_name + "': the nitrous controller's purge channel '" + kit_desc.controller.purge.channel +
                                          "' is also its arm channel");
            }
            const std::vector<ChannelDecl> n2o_decls = declare_control_channels(pt);
            auto slot_of = [&](const std::string& name) {
                for (std::size_t i = 0; i < n2o_decls.size(); ++i) {
                    if (n2o_decls[i].name == name) return static_cast<int>(i);
                }
                return -1;
            };
            nitrous_ = std::make_unique<NitrousSystem>(bottle_desc, kit_desc, burnt_fuel.density_kg_m3, slot_of(kit_desc.controller.arm_channel),
                                                       bottle_desc.has_heater ? slot_of(bottle_desc.heater.channel) : -1,
                                                       kit_desc.controller.has_purge ? slot_of(kit_desc.controller.purge.channel) : -1);
        }

        // Engine thermal plan T1: the lumped engine/oil temperature model. Heat comes from the engine's fuel
        // sub-ledger path, so an engine without a fuel model cannot feed it.
        if (pt.thermal.has_value()) {
            if (!e_has_fuel_model) {
                throw std::runtime_error("vehicle '" + vehicle_name + "': a cooling_system needs an engine with a fuel model (a fuel_map or a fuels list)");
            }
            thermal_ = std::make_unique<EngineThermal>(*pt.thermal);
            thermal_coulomb_share_ = tm ? thermal_->motoring_friction_fraction() : real(1.0);
        }

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
        if ((tm ? tm->initial_state : sim->initial_state) == EngineInitialState::Running) {
            network_.set_omega(engine_crank_, rpm_to_rad_per_s(tm ? tm->idle_rpm : sim->idle_rpm));
        }
        // F-T0 (vault PHYS-049/050 follow-ups): the seed above teleports the
        // crank (and, through any rigid row already wired to it, other nodes)
        // straight to idle speed with no substep and no tracked energy flow.
        // ledger_.rotor_ke defaults to 0 (PowertrainEnergy's all-zero default
        // member init) until pre_solve_components' own per-substep loop
        // OVERWRITES it (an assignment, not an accumulation) on tick 0's
        // first substep - so begin_tick()'s tick-0 snapshot
        // (ledger_tick_start_ = ledger_) captures the phantom zero, and
        // end_tick()'s balance_error sees the full seeded KE appear with no
        // matching engine_out/starter_in to explain it (measured ~561 J
        // car_sedan.json / ~526 J car_sedan_sim.json). Seeding rotor_ke here
        // too, from the SAME all-nodes formula pre_solve_components uses,
        // makes the seed part of construction's initial condition instead of
        // an unbooked flow on the first tick. Every node is summed (not just
        // engine_crank_) so this stays correct if a future seed ever touches
        // more than the crank; every other node's omega is still 0 here, so
        // this is a no-op for them today.
        {
            real seeded_rotor_ke = real(0.0);
            for (int i = 0; i < network_.node_count(); ++i) {
                const NodeId nid{static_cast<std::uint16_t>(i)};
                const real w = network_.omega(nid);
                seeded_rotor_ke += real(0.5) * network_.node_inertia(nid) * w * w;
            }
            ledger_.rotor_ke = seeded_rotor_ke;
        }
        // --- PHYS-033 P4 I7a C3: the throttle-command stage --------------
        // An explicit throttle_control component (validated above: at most
        // one, and only when there is an engine) configures the stage;
        // otherwise the IMPLICIT default takes the engine's own declared
        // throttle_input, no pedal_map/linkage_curve, and a hand lever
        // exactly when the engine has a governor (its own channel) - the
        // same hand-lever channel the pre-C3 code resolved directly below
        // (engine_governor_channel_slot_, now folded into throttle_stage_).
        ThrottleInput throttle_kind = e_throttle_input;
        bool throttle_has_curve = false;
        vehicle::Curve1D throttle_curve;
        std::string hand_lever_channel;
        if (throttle_control_component_index >= 0) {
            const auto& tcd = std::get<ThrottleControlDesc>(pt.components[static_cast<std::size_t>(throttle_control_component_index)].params);
            if (tcd.engine != e_id) {
                throw std::runtime_error("vehicle '" + vehicle_name + "': throttle_control component '" +
                                          pt.components[static_cast<std::size_t>(throttle_control_component_index)].id +
                                          "' names engine '" + tcd.engine + "', which is not this vehicle's engine ('" + e_id + "')");
            }
            // PHYS-033 P4 I7b C12 pairing backstop (the loader already
            // rejects a mismatch; this catches a code-built desc): the
            // mean-value engine reads the command as a plate position
            // (mechanical) or a torque request (electronic) by its OWN
            // throttle_input, so a stage of the other kind would silently
            // reinterpret the pedal. The graph engine keeps its pre-C12
            // behaviour (no backstop) so no existing code-built desc changes.
            if (sim && tcd.kind != e_throttle_input) {
                throw std::runtime_error("vehicle '" + vehicle_name + "': throttle_control component '" +
                                          pt.components[static_cast<std::size_t>(throttle_control_component_index)].id +
                                          "' kind does not match simulated engine '" + e_id + "''s throttle_input");
            }
            throttle_kind = tcd.kind;
            throttle_has_curve = tcd.has_curve;
            throttle_curve = tcd.curve;
            // PHYS-033 P4 I7a C3 review: an explicit stage that does not name
            // its own hand_lever_channel still falls back to a governed
            // engine's governor.channel (the loader rejects any OTHER
            // explicit channel on a governed engine - vehicle_io.cpp - so
            // this is the only case left: no channel named at all). Without
            // this, naming an explicit electronic/mechanical stage on the
            // tractor's governed diesel would silently drop its hand lever.
            hand_lever_channel = tcd.hand_lever_channel;
            if (hand_lever_channel.empty() && e_has_governor) {
                hand_lever_channel = tm->governor.channel;
            }
        } else if (e_has_governor) {
            hand_lever_channel = tm->governor.channel;
        }
        int hand_lever_channel_slot = -1;
        if (!hand_lever_channel.empty()) {
            const std::vector<ChannelDecl> hand_lever_decls = declare_control_channels(pt);
            for (std::size_t i = 0; i < hand_lever_decls.size(); ++i) {
                if (hand_lever_decls[i].name == hand_lever_channel) {
                    hand_lever_channel_slot = static_cast<int>(i);
                    break;
                }
            }
        }
        // Pedal plan P1 (out/transmission_pedal_plan.md 2.1): the drive-mode pedal bend acts only through an
        // automatic, DCT or CVT transmission; decided here from the gearbox kind (a gearbox swap is a respawn).
        const bool drive_bend = gearbox_desc != nullptr && gearbox_takes_drive_bend(gearbox_desc->type);
        throttle_stage_.configure(throttle_kind, throttle_has_curve, throttle_curve, hand_lever_channel_slot, drive_bend);
        // Pedal plan P1 (2.2): the manual pedal law on every engine with an ECU - an electronic stage on an engine
        // without a governor. The simulated engine decides it from its own throttle_input (the stage kind matches it).
        if (tm_engine_ptr != nullptr) tm_engine_ptr->set_manual_pedal_law(throttle_kind == ThrottleInput::Electronic && !e_has_governor);
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
            } else if (const auto* tc = std::get_if<TransferCaseDesc>(&c.params)) {
                // I4 C9: transfer.in = rear_ratio*omega(rear) - the rigid row's
                // own relation, same is_diff=false shape as a gear_pair's
                // target=ratio*a (mirrors that branch exactly).
                ResetDerivation r;
                r.is_diff = false;
                r.target = node_for_key(component_port_key(c.id, "in"));
                r.a = node_for_key(component_port_key(c.id, "rear"));
                r.b = r.a;
                r.ratio = tc->rear_ratio;
                pending.push_back(r);
            }
        }
        // PHYS-033 P4 I4 C13: a compliant shaft reachable from the hubs
        // through diff/gear_pair/transfer-case(in-rear) derivation edges and
        // other compliant shafts (e.g. a compliant propshaft gearbox.out ->
        // diff.carrier) is derived like a ratio-1 gear_pair: target = the
        // endpoint FARTHER from the hubs (BFS hop count), source = the nearer
        // one, so it slots into the same leaves-up ordering and the reset
        // state has zero twist rate across it. Rigid vehicle: skipped.
        if (!compliant_.empty()) {
            const std::size_t node_n = static_cast<std::size_t>(network_.node_count());
            std::vector<int> dist(node_n, -1);
            std::vector<NodeId> queue;
            for (const NodeId hn : hub_nodes_) {
                if (dist[hn.index] < 0) {
                    dist[hn.index] = 0;
                    queue.push_back(hn);
                }
            }
            std::vector<std::pair<NodeId, NodeId>> edges;
            for (const ResetDerivation& r : pending) {
                edges.emplace_back(r.target, r.a);
                if (r.is_diff) edges.emplace_back(r.target, r.b);
            }
            for (const CompliantShaft& cs : compliant_) edges.emplace_back(cs.a, cs.b);
            for (std::size_t qi = 0; qi < queue.size(); ++qi) {
                const NodeId u = queue[qi];
                for (const auto& [x, y] : edges) {
                    NodeId v{};
                    if (x == u) v = y;
                    else if (y == u) v = x;
                    else continue;
                    if (dist[v.index] >= 0) continue;
                    dist[v.index] = dist[u.index] + 1;
                    queue.push_back(v);
                }
            }
            for (const CompliantShaft& cs : compliant_) {
                const int da = dist[cs.a.index];
                const int db = dist[cs.b.index];
                if (da < 0 || db < 0 || da == db) continue;
                ResetDerivation r;
                r.is_diff = false;
                r.target = da > db ? cs.a : cs.b;
                r.a = da > db ? cs.b : cs.a;
                r.b = r.a;
                r.ratio = real(1.0);
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

    // PHYS-033 P4 I4 C13: per compliant shaft, what reset() does after its
    // own steps 4-6. "Owned" = a node reset() already sets: every hub, every
    // derivation target (incl. the compliant ones above), the engine crank,
    // clutch.out and gearbox.in (step 6 sets both). Exactly one end owned:
    // the other end takes its speed; neither: b takes a's; both: nothing.
    if (!compliant_.empty()) {
        std::vector<std::uint8_t> owned(static_cast<std::size_t>(network_.node_count()), 0);
        for (const NodeId hn : hub_nodes_) owned[hn.index] = 1;
        for (const ResetDerivation& r : reset_derivations_) owned[r.target.index] = 1;
        if (engine_) owned[engine_crank_.index] = 1;
        if (gearbox_) {
            if (clutch_) owned[clutch_out_.index] = 1;
            owned[gearbox_in_.index] = 1;
        }
        for (CompliantShaft& cs : compliant_) {
            const bool oa = owned[cs.a.index] != 0;
            const bool ob = owned[cs.b.index] != 0;
            cs.reset_mode = (oa && ob) ? std::uint8_t{0} : (ob ? std::uint8_t{2} : std::uint8_t{1});
        }
    }

    // --- Phase L: controllers (plan section 4.2/9.1) -------------------------
    {
        // At most one manual_tcu and (engine garage P8) at most one launch_control.
        int n_tcu = 0, n_launch = 0, n_auto = 0;
        for (const ControllerDesc& cd : controllers) {
            if (std::holds_alternative<ManualTcuDesc>(cd.params)) ++n_tcu;
            else if (std::holds_alternative<LaunchControlDesc>(cd.params)) ++n_launch;
            else ++n_auto;
        }
        if (n_tcu > 1) {
            throw std::runtime_error("vehicle '" + vehicle_name + "': at most one controller (manual_tcu) is supported in I3");
        }
        // Automatic gearbox plan A1: one transmission controller - manual_tcu for a Manual/Sequential box,
        // automatic_tcu (required) for an Automatic/DualClutch box; no driver clutch with a clutch-to-clutch box.
        if (n_tcu + n_auto > 1) {
            throw std::runtime_error("vehicle '" + vehicle_name + "': at most one transmission controller (manual_tcu or automatic_tcu)");
        }
        if (c2c_gearbox) {
            if (n_tcu > 0) {
                throw std::runtime_error("vehicle '" + vehicle_name + "': a manual_tcu cannot drive an automatic or dual_clutch gearbox - use automatic_tcu");
            }
            if (n_auto == 0) {
                throw std::runtime_error("vehicle '" + vehicle_name + "': an automatic or dual_clutch gearbox needs an automatic_tcu controller");
            }
            if (clutch_component_index >= 0) {
                throw std::runtime_error("vehicle '" + vehicle_name + "': an automatic or dual_clutch gearbox has no separate clutch component");
            }
        } else if (n_auto > 0) {
            throw std::runtime_error("vehicle '" + vehicle_name + "': an automatic_tcu needs an automatic or dual_clutch gearbox");
        }
        if (n_launch > 1) {
            throw std::runtime_error("vehicle '" + vehicle_name + "': at most one launch_control controller is supported");
        }
    }
    anti_lag_fitted_ = pt.anti_lag_fitted;
    for (const ControllerDesc& cd : controllers) {
        if (const auto* lc = std::get_if<LaunchControlDesc>(&cd.params)) {
            // Engine garage P8 (A16): the second limit lies strictly between the engine's idle and its own limiter.
            real idle = real(0.0), limit = real(0.0);
            bool have_engine = false;
            for (const ComponentDesc& c : pt.components) {
                if (const auto* te = std::get_if<TorqueMapEngineDesc>(&c.params)) {
                    idle = te->idle_rpm; limit = te->limiter.rpm; have_engine = true; break;
                }
                if (const auto* se = std::get_if<SimulatedEngineDesc>(&c.params)) {
                    idle = se->idle_rpm; limit = se->limiter.rpm; have_engine = true; break;
                }
            }
            if (!have_engine) {
                throw std::runtime_error("vehicle '" + vehicle_name + "': launch_control controller '" + cd.id + "' needs an engine component");
            }
            // G1: an rpm at or above max_limiter_fraction of the limiter (a low garage limiter) is held there instead of
            // being rejected (the loader warns); below that the part's own rpm runs unchanged.
            const LaunchRpmFit fit = fit_launch_rpm(lc->rpm, idle, limit, lc->max_limiter_fraction);
            if (!(lc->rpm > idle) || !(limit > idle) || !(fit.rpm > idle) || !(fit.rpm < limit) || !(lc->max_speed_mps > real(0.0)) ||
                !std::isfinite(lc->rpm) || !std::isfinite(lc->max_speed_mps) || !(lc->max_limiter_fraction > real(0.0)) ||
                !(lc->max_limiter_fraction < real(1.0))) {
                throw std::runtime_error("vehicle '" + vehicle_name + "': launch_control controller '" + cd.id +
                                          "': rpm must lie between the engine's idle and its own limiter, max_speed_mps must be > 0 and "
                                          "max_limiter_fraction in (0, 1)");
            }
            LaunchControlDesc fitted = *lc;
            fitted.rpm = fit.rpm;
            launch_.emplace(fitted);
            continue;
        }
        if (const auto* at = std::get_if<AutomaticTcuDesc>(&cd.params)) {
            const auto git = component_index_of.find(at->gearbox);
            if (git == component_index_of.end() || git->second != gearbox_component_index) {
                throw std::runtime_error("vehicle '" + vehicle_name + "': controller '" + cd.id + "' gearbox '" + at->gearbox +
                                          "' is not this vehicle's gearbox component");
            }
            const auto eit = component_index_of.find(at->engine);
            if (eit == component_index_of.end() || eit->second != engine_component_index) {
                throw std::runtime_error("vehicle '" + vehicle_name + "': controller '" + cd.id + "' engine '" + at->engine +
                                          "' is not this vehicle's engine component");
            }
            AutoTransmission::Desc ad;
            ad.dual_clutch = gearbox_desc->type == GearboxType::DualClutch;
            // CVT plan C1: a cvt gearbox and the automatic_tcu's cvt block come together.
            const bool is_cvt = gearbox_desc->type == GearboxType::Cvt;
            if (is_cvt != at->cvt.has_value()) {
                throw std::runtime_error("vehicle '" + vehicle_name + "': controller '" + cd.id +
                                          (is_cvt ? "': a cvt gearbox needs the automatic_tcu's 'cvt' block (its ratio law)"
                                                  : "': the automatic_tcu's 'cvt' block is for a cvt gearbox only"));
            }
            if (is_cvt) {
                if (!gearbox_desc->cvt) throw std::runtime_error("vehicle '" + vehicle_name + "': a cvt gearbox needs its 'cvt' (variator) block");
                ad.cvt = true;
                ad.cvt_desc = *gearbox_desc->cvt;
            }
            ad.forward_ratios = gearbox_desc->forward_ratios;
            ad.reverse_ratios = gearbox_desc->reverse_ratios;
            ad.clutch = gearbox_desc->shift_clutch;
            ad.tcu = *at;
            ad.initial_gear = gearbox_desc->initial_gear;
            if (gearbox_desc->converter) {
                ad.has_lockup = true;
                ad.lockup_capacity_nm = gearbox_desc->converter->lockup_capacity_nm;
                ad.lockup_static_factor = gearbox_desc->converter->lockup_static_factor;
                ad.lockup_engage_time_s = gearbox_desc->converter->lockup_engage_time_s;
                ad.lockup_release_time_s = gearbox_desc->converter->lockup_release_time_s;
            }
            auto_ = std::make_unique<AutoTransmission>(std::move(ad));
            const std::vector<ChannelDecl> decls = declare_control_channels(pt);
            for (std::size_t i = 0; i < decls.size(); ++i) {
                if (decls[i].name == kTransmissionModeChannel) {
                    mode_channel_slot_ = i;
                    has_mode_channel_ = true;
                    break;
                }
            }
            continue;
        }
        const auto* tcu_desc = std::get_if<ManualTcuDesc>(&cd.params);
        if (tcu_desc == nullptr) continue;
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
        check_ref(tcu_desc->engine, "engine", [](const ComponentParams& p) {
            return std::holds_alternative<TorqueMapEngineDesc>(p) || std::holds_alternative<SimulatedEngineDesc>(p);
        });
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
        // G1: the auto-shift cap against the engine's effective limiter (check_ref above guarantees the engine exists).
        md.shift_rpm_cap = tcu_desc->auto_shift.max_limiter_fraction *
                           effective_limiter_rpm(pt.components[static_cast<std::size_t>(component_index_of.at(tcu_desc->engine))].params);
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
    std::unordered_map<std::string,Battery*> battery_of;
    for(const auto& c:pt.components) if(const auto* b=std::get_if<BatteryDesc>(&c.params)) {
        auto store=std::make_unique<Battery>(*b);battery_of[c.id]=store.get();batteries_.push_back(std::move(store));
    }

    for (const ComponentDesc& c : pt.components) {
        if (const auto* cell = std::get_if<FuelCellDesc>(&c.params)) {
            const auto store=battery_of.find(cell->battery);
            if(store==battery_of.end())throw std::runtime_error("fuel cell battery reference is missing: "+cell->battery);
            const auto decls=declare_control_channels(pt);int ready=-1;
            for(std::size_t i=0;i<decls.size();++i)if(decls[i].name==cell->ready_channel)ready=static_cast<int>(i);
            auto stack=std::make_unique<FuelCell>(*cell,*store->second,ready);
            fuel_cells_.push_back(stack.get());components_.push_back(std::move(stack));
        }
    }
    for(const auto& c:pt.components)if(const auto* control=std::get_if<HybridControlDesc>(&c.params)) {
        if(hybrid_)throw std::invalid_argument("only one hybrid supervisor is supported");
        if(!engine_||!gearbox_||(!clutch_&&!auto_))throw std::invalid_argument("hybrid supervisor requires an ICE and disconnecting gearbox");
        if(!std::isfinite(control->charge_power_w)||control->charge_power_w<=0||!std::isfinite(control->requested_power_w)||control->requested_power_w<=0||!std::isfinite(control->low_soc)||control->low_soc<0||
           !std::isfinite(control->charge_target_soc)||control->charge_target_soc>1||control->low_soc>=control->charge_target_soc||
           !std::isfinite(control->charge_rpm)||control->charge_rpm<=0||!std::isfinite(control->generator_fraction)||control->generator_fraction<0||control->generator_fraction>1||
           !std::isfinite(control->stop_delay_s)||control->stop_delay_s<0||!std::isfinite(control->crank_timeout_s)||control->crank_timeout_s<=0)
            throw std::invalid_argument("invalid hybrid controller calibration");
        hybrid_=*control;
        const auto decls=declare_control_channels(pt);
        for(std::size_t i=0;i<decls.size();++i){if(decls[i].name=="assist.drive_mode")hybrid_mode_slot_=static_cast<int>(i);if(decls[i].name=="electric.ready")hybrid_ready_slot_=static_cast<int>(i);}
    }
    for (const ComponentDesc& c : pt.components) {
        if (const auto* motor = std::get_if<ElectricMotorDesc>(&c.params)) {
            const auto store=battery_of.find(motor->battery);
            if(store==battery_of.end())throw std::runtime_error("electric motor battery reference is missing or is not a battery: "+motor->battery);
            const auto decls=declare_control_channels(pt);
            const auto slot=[&](const std::string& name){for(std::size_t i=0;i<decls.size();++i)if(decls[i].name==name)return static_cast<int>(i);return -1;};
            auto machine=std::make_unique<ElectricMotor>(node_for_key(component_port_key(c.id,"shaft")),*motor,*store->second,
                slot(motor->throttle_channel),slot(motor->regen_channel),slot(motor->direction_channel),slot(motor->ready_channel));
            electric_motors_.push_back(machine.get());components_.push_back(std::move(machine));
        } else if (const auto* g = std::get_if<GearPairDesc>(&c.params)) {
            components_.push_back(std::make_unique<GearPair>(node_for_key(component_port_key(c.id, "in")),
                                                               node_for_key(component_port_key(c.id, "out")), *g));
        } else if (const auto* d = std::get_if<DifferentialDesc>(&c.params)) {
            auto diff = std::make_unique<Differential>(node_for_key(component_port_key(c.id, "carrier")),
                                                         node_for_key(component_port_key(c.id, "left")),
                                                         node_for_key(component_port_key(c.id, "right")), *d);
            diffs_.push_back(diff.get()); // powertrain_debug's diff/hub torque reads (powertrain.h), component order
            components_.push_back(std::move(diff));
        } else if (const auto* tc = std::get_if<TransferCaseDesc>(&c.params)) {
            // I4 C9: channel_slot resolved from declare_control_channels(pt)'s
            // own array-order/dedup rules (same mechanism as the diff-lock
            // block just below) - transfer_row_ was already assigned in
            // Phase G above (the front friction row, appended after clutch/
            // engine/synchro so no pre-existing vehicle's row ids shift).
            const std::vector<ChannelDecl> tc_decls = declare_control_channels(pt);
            int channel_slot = -1;
            for (std::size_t i = 0; i < tc_decls.size(); ++i) {
                if (tc_decls[i].name == tc->channel) {
                    channel_slot = static_cast<int>(i);
                    break;
                }
            }
            auto transfer = std::make_unique<TransferCase>(node_for_key(component_port_key(c.id, "in")),
                                                             node_for_key(component_port_key(c.id, "front")), transfer_row_, *tc,
                                                             channel_slot);
            transfer_case_ = transfer.get(); // non-owning alias, mirrors diffs_'s pattern
            components_.push_back(std::move(transfer));
        } else if (const auto* pl = std::get_if<ParkLockDesc>(&c.params)) {
            // I4 C10: channel_slot resolved the same way as transfer_case
            // above - park_lock_row_ was already assigned in Phase G.
            const std::vector<ChannelDecl> pl_decls = declare_control_channels(pt);
            int channel_slot = -1;
            for (std::size_t i = 0; i < pl_decls.size(); ++i) {
                if (pl_decls[i].name == pl->channel) {
                    channel_slot = static_cast<int>(i);
                    break;
                }
            }
            auto park = std::make_unique<ParkLock>(node_for_key(component_port_key(c.id, "shaft")), park_lock_row_, *pl, channel_slot);
            park_lock_ = park.get(); // non-owning alias, mirrors transfer_case_'s pattern
            components_.push_back(std::move(park));
        } else if (const auto* pto = std::get_if<PtoDesc>(&c.params)) {
            // I4 C11: channel_slot/speed_channel_slot resolved the same way
            // as transfer_case/park_lock above - pto_row_ was already
            // assigned in Phase G.
            const std::vector<ChannelDecl> pto_decls = declare_control_channels(pt);
            int channel_slot = -1, speed_channel_slot = -1;
            for (std::size_t i = 0; i < pto_decls.size(); ++i) {
                if (pto_decls[i].name == pto->channel) channel_slot = static_cast<int>(i);
                if (pto_decls[i].name == pto->speed_channel) speed_channel_slot = static_cast<int>(i);
            }
            const NodeId pto_in_n = node_for_key(component_port_key(c.id, "in"));
            const NodeId pto_out_n = node_for_key(component_port_key(c.id, "out"));
            auto pto_comp = std::make_unique<Pto>(pto_in_n, pto_out_n, pto_row_, *pto, channel_slot, speed_channel_slot,
                                                   network_.node_inertia(pto_out_n));
            pto_ = pto_comp.get(); // non-owning alias, mirrors transfer_case_/park_lock_'s pattern
            components_.push_back(std::move(pto_comp));
        } else if (const auto* dyno = std::get_if<LoadDynoDesc>(&c.params)) {
            // I4 C11: test/bench only. scale_channel_slot resolved the same
            // way, but only when the desc actually names a channel (empty =
            // fixed scale 1, no channel declared at all - declare_control_
            // channels never emits one for this dyno in that case).
            int scale_channel_slot = -1;
            if (!dyno->scale_channel.empty()) {
                const std::vector<ChannelDecl> dyno_decls = declare_control_channels(pt);
                for (std::size_t i = 0; i < dyno_decls.size(); ++i) {
                    if (dyno_decls[i].name == dyno->scale_channel) {
                        scale_channel_slot = static_cast<int>(i);
                        break;
                    }
                }
            }
            const NodeId shaft_n = node_for_key(component_port_key(c.id, "shaft"));
            FrictionRowId row{};
            if (dyno->mode == LoadDynoMode::Brake) {
                row = FrictionRowId{static_cast<std::uint16_t>(dyno_brake_row_of_id.at(c.id))};
            }
            auto dyno_comp = std::make_unique<LoadDyno>(shaft_n, row, *dyno, scale_channel_slot);
            load_dynos_.push_back(dyno_comp.get());
            components_.push_back(std::move(dyno_comp));
        }
    }
    if(hybrid_) {
        bool generator=false,traction=false;
        for(const auto& c:pt.components)if(const auto* m=std::get_if<ElectricMotorDesc>(&c.params)) {
            const auto node=node_for_key(component_port_key(c.id,"shaft"));
            if(m->crank_assist){if(node!=engine_crank_)throw std::invalid_argument("hybrid generator must share the engine crankshaft");generator=true;}
            else if(node!=engine_crank_)traction=true;
        }
        if(!generator||!traction)throw std::invalid_argument("hybrid charging requires separate traction motor and crank generator");
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
    // --- Brake blending (regeneration first, docs/electric_drive.md): for every motor that regenerates from the brake
    // pedal, walk the shaft graph from its node down to the wheel hubs it brakes, through gear pairs and differentials
    // (nominal split, mesh efficiencies in the regeneration direction), giving the rim torque per unit of motor-shaft
    // torque of each hub. A motor with no such path (behind a gearbox, a transfer case, ...) stays outside the scheme.
    regen_cfg_ = brakes.regen_blend;
    regen_corner_max_.assign(wheels.size(), real(0.0));
    regen_wheel_nm_.assign(wheels.size(), real(0.0));
    for (const auto& r : brake_rows) regen_corner_max_[r.wheel] = r.corner_max_nm;
    if (brakes.regen_blend.enabled && !electric_motors_.empty()) {
        struct GearEdge { NodeId in, out; real ratio, eff; };
        struct DiffEdge { NodeId carrier, left, right; real ratio, eff; };
        std::vector<GearEdge> gear_edges;
        std::vector<DiffEdge> diff_edges;
        for (const ComponentDesc& c : pt.components) {
            if (const auto* g = std::get_if<GearPairDesc>(&c.params)) {
                gear_edges.push_back({node_for_key(component_port_key(c.id, "in")), node_for_key(component_port_key(c.id, "out")),
                                      std::abs(g->ratio), g->efficiency});
            } else if (const auto* d = std::get_if<DifferentialDesc>(&c.params)) {
                diff_edges.push_back({node_for_key(component_port_key(c.id, "carrier")), node_for_key(component_port_key(c.id, "left")),
                                      node_for_key(component_port_key(c.id, "right")), std::abs(d->ratio), d->efficiency});
            }
        }
        const std::size_t node_total = static_cast<std::size_t>(network_.node_count());
        std::vector<std::vector<std::size_t>> wheels_of_node(node_total);
        for (std::size_t wi = 0; wi < wheels.size(); ++wi) wheels_of_node[hub_nodes_[wi].index].push_back(wi);
        // Transfer case (rigid rear output, friction front output) and the typed gearbox/clutch: only downstream edges.
        struct TcEdge { NodeId in, rear, front; real rear_ratio, front_ratio; };
        std::vector<TcEdge> tc_edges;
        for (const ComponentDesc& c : pt.components)
            if (const auto* t = std::get_if<TransferCaseDesc>(&c.params))
                tc_edges.push_back({node_for_key(component_port_key(c.id, "in")), node_for_key(component_port_key(c.id, "rear")),
                                    node_for_key(component_port_key(c.id, "front")), std::abs(t->rear_ratio), std::abs(t->front_ratio)});
        const bool gearbox_edge = gearbox_ && !auto_;
        const bool clutch_edge = clutch_ != nullptr;
        std::size_t motor_i = 0;
        for (const ComponentDesc& c : pt.components) {
            const auto* md = std::get_if<ElectricMotorDesc>(&c.params);
            if (md == nullptr) continue;
            ElectricMotor* motor = electric_motors_[motor_i++];
            if (md->crank_assist || !md->regen_channel.empty()) continue; // a generator / externally commanded regeneration
            RegenMap map;
            map.motor = motor;
            map.nodes.push_back({-1, 0u, RegenEdge::Fixed, real(1.0), {}});
            std::vector<NodeId> node_of{node_for_key(component_port_key(c.id, "shaft"))};
            std::vector<char> visited(node_total, 0);
            visited[node_of[0].index] = 1;
            std::vector<int> stack{0};
            while (!stack.empty()) {
                const int cur = stack.back();
                stack.pop_back();
                const NodeId here = node_of[static_cast<std::size_t>(cur)];
                const auto& hub_wheels = wheels_of_node[here.index];
                if (!hub_wheels.empty()) { // a hub is a leaf
                    map.nodes[static_cast<std::size_t>(cur)].wheels = hub_wheels;
                    continue;
                }
                unsigned branches = 0;
                const auto add = [&](NodeId to, RegenEdge kind, real factor, unsigned group) {
                    visited[to.index] = 1;
                    map.nodes.push_back({cur, group, kind, factor, {}});
                    node_of.push_back(to);
                    stack.push_back(static_cast<int>(map.nodes.size()) - 1);
                    if (kind != RegenEdge::Fixed) map.dynamic = true;
                };
                for (const GearEdge& g : gear_edges) {
                    if (g.in == here && !visited[g.out.index]) add(g.out, RegenEdge::Fixed, g.ratio / g.eff, branches++);
                    else if (g.out == here && !visited[g.in.index] && g.ratio > real(0.0)) add(g.in, RegenEdge::Fixed, real(1.0) / (g.ratio * g.eff), branches++);
                }
                for (const DiffEdge& d : diff_edges) {
                    if (d.carrier != here) continue;
                    bool any = false;
                    for (NodeId side : {d.left, d.right})
                        if (!visited[side.index]) { add(side, RegenEdge::Fixed, d.ratio * real(0.5) / d.eff, branches); any = true; }
                    if (any) ++branches;
                }
                if (gearbox_edge && gearbox_in_ == here && !visited[gearbox_out_.index]) add(gearbox_out_, RegenEdge::Gearbox, real(1.0), branches++);
                if (clutch_edge && clutch_in_ == here && !visited[clutch_out_.index]) add(clutch_out_, RegenEdge::Clutch, real(1.0), branches++);
                for (const TcEdge& t : tc_edges) {
                    if (t.in != here) continue;
                    if (!visited[t.rear.index]) add(t.rear, RegenEdge::Fixed, t.rear_ratio, branches++);
                    if (!visited[t.front.index]) add(t.front, RegenEdge::TransferFront, t.front_ratio, branches++);
                }
            }
            for (const RegenNode& n : map.nodes)
                for (std::size_t wi : n.wheels) {
                    bool known = false;
                    for (const RegenHub& h : map.hubs) known = known || h.wheel == wi;
                    if (!known) map.hubs.push_back({wi, real(0.0), wheels[wi].wheel_radius});
                }
            if (map.hubs.empty()) continue; // no rim reachable at all: outside the scheme
            std::sort(map.hubs.begin(), map.hubs.end(), [](const RegenHub& a, const RegenHub& b) { return a.wheel < b.wheel; });
            map.factor.assign(map.nodes.size(), real(0.0));
            map.group_mask.assign(map.nodes.size(), 0u);
            map.open.assign(map.nodes.size(), 0);
            motor->enable_blend(brakes.regen_blend, map.dynamic);
            regen_maps_.push_back(std::move(map));
        }
    }
    auto brakes_component = std::make_unique<DirectTorqueBrakes>(std::move(brake_rows), brakes.static_factor);
    brakes_ = brakes_component.get();
    components_.push_back(std::move(brakes_component));
    // B mode (docs/electric_drive.md): available when blending runs, a motor reaches the rims and the car has no gearbox
    // (pure EV) or is a hybrid (its Electric mode). Everything below is sized here so the substep path allocates nothing.
    b_capable_ = brakes.regen_blend.enabled && brakes.regen_blend.b_mode.enabled && !regen_maps_.empty() && (!gearbox_ || hybrid_.has_value());
    if (b_capable_) {
        const std::size_t nmap = regen_maps_.size();
        b_fraction_.assign(nmap, real(1.0));
        b_env_w_.assign(nmap, real(0.0));
        b_regen_w_.assign(nmap, real(0.0));
        b_regen_nm_.assign(nmap, real(0.0));
        b_scale_d_.assign(batteries_.size(), real(1.0));
        b_scale_r_.assign(batteries_.size(), real(1.0));
        b_battery_idx_.assign(nmap, 0);
        for (std::size_t k = 0; k < nmap; ++k)
            for (std::size_t bi = 0; bi < batteries_.size(); ++bi)
                if (&regen_maps_[k].motor->battery() == batteries_[bi].get()) b_battery_idx_[k] = bi;
    }
    if (hybrid_) {
        for (const ComponentDesc& c : pt.components) {
            const real lim = effective_limiter_rpm(c.params);
            if (lim > real(0.0)) { engine_limiter_rpm_ = lim; break; }
        }
        refresh_hybrid_ice_power();
    }
}

// Derived full-pedal demand of a hybrid: the ICE's own full-load peak power, swept over its envelope at build and again
// whenever the envelope is rebuilt (never per substep). Authored requested_power_w -> 0 (unused).
void Powertrain::refresh_hybrid_ice_power() {
    hybrid_ice_full_w_ = real(0.0);
    if (!hybrid_ || !hybrid_->requested_power_derived || !engine_) return;
    const real idle = engine_->idle_rpm();
    const real top = engine_limiter_rpm_ > idle ? engine_limiter_rpm_ : idle * real(2.0);
    constexpr int kSteps = 64;
    real best = real(0.0);
    for (int i = 0; i <= kSteps; ++i) {
        const real rpm = idle + (top - idle) * static_cast<real>(i) / static_cast<real>(kSteps);
        const real w = rpm * real(2.0) * real(3.14159265358979323846) / real(60.0);
        best = std::max(best, engine_->request_envelope(rpm).f_nm * w);
    }
    hybrid_ice_full_w_ = best;
}

// B mode selector: the absolute gear_request code kGearRequestB (edge-triggered, so a game that keeps sending its selector
// does not re-enter B after Electric mode ended it) and the shift counters (D -> up -> B, B -> down -> D). `allowed`: the
// position exists right now (a pure EV; a hybrid in Electric mode). In Electric mode the counters are consumed here, so the
// gearbox's controller never sees them (the supervisor keeps the gearbox where it wants it).
void Powertrain::b_update_selector(const ComponentSubstepCtx& ctx_in, bool allowed) {
    if (!b_capable_ && !hybrid_) return;
    if (!b_counters_init_) {
        b_prev_up_ = ctx_in.shift_up_count;
        b_prev_down_ = ctx_in.shift_down_count;
        b_counters_init_ = true;
    }
    const real du = ctx_in.shift_up_count - b_prev_up_;
    const real dd = ctx_in.shift_down_count - b_prev_down_;
    b_prev_up_ = ctx_in.shift_up_count;
    b_prev_down_ = ctx_in.shift_down_count;
    if (hybrid_ && hybrid_electric_active_) {
        swallow_up_ += du;
        swallow_down_ += dd;
    }
    if (!b_capable_) return;
    const real dir = hybrid_ ? hybrid_selector_ : electric_direction_;
    bool b = b_selected_;
    if (ctx_in.gear_request != b_prev_request_) {
        b_prev_request_ = ctx_in.gear_request;
        if (ctx_in.gear_request != kGearRequestNone) b = ctx_in.gear_request == kGearRequestB;
    }
    if (allowed && dir > real(0.0)) {
        if (du > real(0.5) && !b) b = true;
        else if (dd > real(0.5) && b) b = false;
    }
    if (!allowed || !(dir > real(0.0))) b = false;
    if (b != b_selected_) {
        b_selected_ = b;
        if (!b) std::fill(b_fraction_.begin(), b_fraction_.end(), real(1.0));
    }
}

// B mode, per substep after prepare_regen: the system power law P = -R + pedal (R + P_max) (ps/drivetrain/b_mode.h) turned
// into each wheel-path motor's drive command or regeneration torque, the wheel-slip guard, and the pads' auto hold.
void Powertrain::prepare_b(const ComponentSubstepCtx& ctx, real pedal) {
    if (!b_capable_) return;
    const BModeDesc& bd = regen_cfg_.b_mode;
    const real dir = hybrid_ ? hybrid_selector_ : electric_direction_;
    const bool active = b_selected_ && dir > real(0.0) && ctx.h > real(0.0);
    if (!active) {
        if (b_feed_on_) {
            for (RegenMap& m : regen_maps_) m.motor->set_b_feed(false, real(0.0), real(0.0));
            b_feed_on_ = false;
        }
        b_hold_active_ = false;
        if (b_hold_level_ > real(0.0))
            b_hold_level_ = bd.hold_release_s > real(0.0) ? std::max(real(0.0), b_hold_level_ - ctx.h / bd.hold_release_s) : real(0.0);
        brakes_->set_hold_level(b_hold_level_);
        b_regen_cap_w_ = b_drive_cap_w_ = b_power_cmd_w_ = b_zero_pedal_ = real(0.0);
        return;
    }
    b_feed_on_ = true;
    const std::size_t n = regen_maps_.size();
    for (std::size_t k = 0; k < n; ++k) {
        const ElectricMotor* mo = regen_maps_[k].motor;
        const real om = std::abs(network_.omega(mo->shaft_node()));
        const bool part = mo->ready;
        b_env_w_[k] = part ? mo->drive_envelope_torque(om) * std::max(om, real(1.0)) : real(0.0);
        const real tr = part ? mo->b_regen_capacity_nm(om) : real(0.0);
        b_regen_nm_[k] = tr; // capacity torque before the pack's scale
        b_regen_w_[k] = tr * om;
    }
    for (std::size_t bi = 0; bi < batteries_.size(); ++bi) {
        const Battery& bat = *batteries_[bi];
        real sum_e = real(0.0), sum_r = real(0.0), eff = real(1.0);
        bool first = true;
        for (std::size_t k = 0; k < n; ++k)
            if (b_battery_idx_[k] == bi) {
                sum_e += b_env_w_[k];
                sum_r += b_regen_w_[k];
                if (first) { eff = regen_maps_[k].motor->regen_efficiency(); first = false; }
            }
        real supply_w = battery_discharge_budget_w(bat.desc, bat.soc(), ctx.h);
        for (const FuelCell* fc : fuel_cells_)
            if (&fc->battery() == &bat && fc->hydrogen_kg > real(0.0)) supply_w += fc->rated_power();
        b_scale_d_[bi] = sum_e > real(0.0) ? std::min(real(1.0), supply_w * eff / sum_e) : real(1.0);
        const real accept_w = std::max(real(0.0), battery_charge_budget_j(bat.desc, bat.content(), ctx.h)) / ctx.h;
        b_scale_r_[bi] = sum_r > real(0.0) ? std::min(real(1.0), accept_w / eff / sum_r) : real(1.0);
    }
    real p_max = real(0.0), regen_w = real(0.0);
    for (std::size_t k = 0; k < n; ++k) {
        p_max += b_env_w_[k] * b_scale_d_[b_battery_idx_[k]];
        regen_w += b_regen_w_[k] * b_scale_r_[b_battery_idx_[k]];
    }
    const real p = std::clamp(pedal, real(0.0), real(1.0));
    const real p_cmd = b_system_power(regen_w, p_max, p);
    const real zero_pedal = ps::drivetrain::b_zero_power_pedal(regen_w, p_max);
    const real ceiling = std::clamp(ctx.traction_throttle_ceiling, real(0.0), real(1.0));
    const real tc_scale = (p > real(0.0) && ceiling < p) ? ceiling / p : real(1.0);
    for (std::size_t k = 0; k < n; ++k) {
        RegenMap& m = regen_maps_[k];
        const std::size_t bi = b_battery_idx_[k];
        real slip = real(0.0);
        if (p_cmd >= real(0.0)) {
            const real c = p_max > real(0.0) ? std::clamp(p_cmd * b_scale_d_[bi] / p_max, real(0.0), real(1.0)) : real(0.0);
            // TC ceiling (PR #37): a fraction of the torque available now, so it scales the drive; B regeneration below is not.
            m.motor->set_b_feed(true, c, real(0.0), tc_scale);
        } else {
            const real f = regen_w > real(0.0) ? std::min(real(1.0), -p_cmd / regen_w) : real(0.0);
            // Wheel-slip guard (the TC regulator's structure, braking direction): the worst slip of the wheels this motor
            // reaches, from the contact-patch sensors; the regeneration is withdrawn above the target and ramps back.
            if (ctx.traction_wheels != nullptr) {
                for (const RegenHub& h : m.hubs) {
                    if (!(h.share > real(1e-9)) || static_cast<int>(h.wheel) >= ctx.traction_wheel_count) continue;
                    const vehicle::TractionWheelState& w = ctx.traction_wheels[h.wheel];
                    if (!w.contact || !(w.normal_load_n > real(1.0)) || !std::isfinite(w.normal_load_n) ||
                        !std::isfinite(w.ground_speed_m_s) || !std::isfinite(w.peripheral_speed_m_s)) continue;
                    const real sgn = std::abs(w.ground_speed_m_s) > real(0.1) ? (w.ground_speed_m_s < real(0.0) ? real(-1.0) : real(1.0))
                                                                              : (w.peripheral_speed_m_s < real(0.0) ? real(-1.0) : real(1.0));
                    slip = std::max(slip, sgn * (w.ground_speed_m_s - w.peripheral_speed_m_s) /
                                              std::max(bd.slip_floor_mps, std::abs(w.ground_speed_m_s)));
                }
            }
            m.motor->set_b_feed(true, real(0.0), b_regen_nm_[k] * b_scale_r_[bi] * f * b_fraction_[k]);
        }
        b_fraction_[k] = b_slip_fraction_step(bd, b_fraction_[k], slip, ctx.h);
    }
    // Auto hold: below the regeneration fade the motors cannot hold the car, so with the pedal at or below the zero-power
    // point the pads do, until the pedal rises above it by the margin; they release over hold_release_s.
    if (!bd.auto_hold || p > zero_pedal + (b_hold_active_ ? bd.hold_pedal_margin : real(0.0))) b_hold_active_ = false;
    else if (regen_fade_min_ < real(1.0)) b_hold_active_ = true;
    // While the regeneration fades the pads take over its share (hold_blend_level x the fade, no jerk); once it is gone (the car
    // is about to stop) they rise to the full hold.
    const real target = !b_hold_active_ ? real(0.0)
                        : regen_fade_min_ > real(0.0) ? (real(1.0) - regen_fade_min_) * bd.hold_blend_level : real(1.0);
    if (target > b_hold_level_) b_hold_level_ = std::min(target, b_hold_level_ + bd.hold_rise_per_s * ctx.h);
    else b_hold_level_ = bd.hold_release_s > real(0.0) ? std::max(target, b_hold_level_ - ctx.h / bd.hold_release_s) : target;
    brakes_->set_hold_level(b_hold_level_);
    b_regen_cap_w_ = regen_w;
    b_drive_cap_w_ = p_max;
    b_power_cmd_w_ = p_cmd;
    b_zero_pedal_ = zero_pedal;
}

// Brake blending, step 1 (before any component's pre_solve): evaluate each blended motor's path tree over the elements
// that are closed THIS substep (engaged gear and its ratio, closed clutch, engaged transfer-case output), then hand the
// motor the rim brake request in its own terms, the speed fade and whether ABS modulates a wheel it drives.
void Powertrain::prepare_regen(const ComponentSubstepCtx& ctx) {
    const real pedal = std::clamp(ctx.brake, real(0.0), real(1.0));
    const RegenBlendDesc& cfg = regen_cfg_;
    // The gearbox path is closed only with a gear fully engaged and no shift pending: a requested shift opens it at once
    // (the regeneration must be gone for the mesh torque to fall below the dog-release threshold).
    const bool gear_closed = gearbox_ && !auto_ && last_gearbox_result_.phase == Gearbox::Phase::Idle &&
                             last_gearbox_result_.current != 0 && last_gearbox_result_.target == last_gearbox_result_.current;
    real gear_factor = real(1.0);
    if (gear_closed) {
        const real ratio = std::abs(gearbox_->ratio_for_position(last_gearbox_result_.current));
        const real eff = gearbox_->efficiency_for_position(last_gearbox_result_.current);
        gear_factor = ratio / eff;
    }
    const bool clutch_closed = clutch_ && last_clutch_result_.engagement >= cfg.path_min_engagement;
    const bool front_closed = transfer_case_ && transfer_case_->engagement() >= cfg.path_min_engagement;
    regen_fade_min_ = real(1.0);
    for (RegenMap& m : regen_maps_) {
        const std::size_t n = m.nodes.size();
        for (RegenHub& h : m.hubs) h.share = real(0.0);
        // pass 1: which nodes are reachable over open edges; which branch groups of a parent have an open child
        m.open[0] = 1;
        std::fill(m.group_mask.begin(), m.group_mask.end(), 0u);
        for (std::size_t i = 1; i < n; ++i) {
            const RegenNode& nd = m.nodes[i];
            bool edge_open = true;
            switch (nd.kind) {
                case RegenEdge::Fixed: break;
                case RegenEdge::Gearbox: edge_open = gear_closed; break;
                case RegenEdge::Clutch: edge_open = clutch_closed; break;
                case RegenEdge::TransferFront: edge_open = front_closed; break;
            }
            m.open[i] = (m.open[static_cast<std::size_t>(nd.parent)] && edge_open) ? 1 : 0;
            if (m.open[i]) m.group_mask[static_cast<std::size_t>(nd.parent)] |= (1u << nd.group);
        }
        // pass 2: torque at each open node per unit motor torque; the rim share per hub; the clutch's torque limit
        m.factor[0] = real(1.0);
        real cap_limit = std::numeric_limits<real>::infinity();
        for (std::size_t i = 0; i < n; ++i) {
            const RegenNode& nd = m.nodes[i];
            if (i > 0) {
                if (!m.open[i]) { m.factor[i] = real(0.0); continue; }
                unsigned groups = 0;
                for (unsigned msk = m.group_mask[static_cast<std::size_t>(nd.parent)]; msk != 0; msk &= msk - 1) ++groups;
                const real edge = nd.kind == RegenEdge::Gearbox ? gear_factor : nd.factor;
                m.factor[i] = m.factor[static_cast<std::size_t>(nd.parent)] * edge * (real(1.0) / static_cast<real>(groups));
                if (nd.kind == RegenEdge::Clutch && m.factor[i] > real(0.0))
                    cap_limit = std::min(cap_limit, cfg.clutch_capacity_margin * last_clutch_result_.cap_kinetic / m.factor[i]);
            }
            if (!nd.wheels.empty() && m.open[i]) {
                const real share = m.factor[i] / static_cast<real>(nd.wheels.size());
                for (std::size_t wi : nd.wheels)
                    for (RegenHub& h : m.hubs) if (h.wheel == wi) h.share += share;
            }
        }
        real request = cap_limit, speed = real(0.0);
        std::size_t active = 0;
        bool abs_acts = false;
        for (const RegenHub& h : m.hubs) {
            if (!(h.share > real(1e-9)) || !std::isfinite(h.share)) continue;
            ++active;
            request = std::min(request, regen_corner_max_[h.wheel] * pedal / h.share);
            speed += std::abs(network_.omega(hub_nodes_[h.wheel])) * h.radius;
            if (ctx.wheel_abs_scale != nullptr && ctx.wheel_abs_scale[h.wheel] < real(0.995)) abs_acts = true;
        }
        if (active == 0) { m.motor->set_blend_feed(real(0.0), real(0.0), false, ctx.h, false); continue; }
        speed /= static_cast<real>(active);
        const real fade = std::clamp((speed - cfg.fade_end_speed_mps) / (cfg.fade_start_speed_mps - cfg.fade_end_speed_mps), real(0.0), real(1.0));
        regen_fade_min_ = std::min(regen_fade_min_, fade);
        m.motor->set_blend_feed(request, fade, abs_acts, ctx.h, true);
    }
}

// Step 2 (after the motors' pre_solve, right before the brake rows'): the rim torque the motors supply per wheel, which
// the pads then leave out of their capacity; and the snapshot sums.
void Powertrain::reflect_regen(const ComponentSubstepCtx& ctx) {
    std::fill(regen_wheel_nm_.begin(), regen_wheel_nm_.end(), real(0.0));
    for (const RegenMap& m : regen_maps_)
        for (const RegenHub& h : m.hubs) regen_wheel_nm_[h.wheel] += h.share * m.motor->regen_brake_pred_nm;
    real regen = real(0.0), pad = real(0.0);
    for (std::size_t w = 0; w < regen_wheel_nm_.size(); ++w) {
        const real service = ctx.wheel_brake != nullptr ? ctx.wheel_brake[w] : ctx.brake;
        const real request = regen_corner_max_[w] * service;
        const real taken = std::min(regen_wheel_nm_[w], std::max(real(0.0), request));
        regen += taken;
        pad += std::max(real(0.0), request) - taken;
    }
    blend_regen_nm_ = regen;
    blend_pad_nm_ = pad;
    brakes_->set_regen_reflected(regen_wheel_nm_.data());
}

void Powertrain::begin_substep() {
    assembly_.begin_substep(network_.node_count());
    assembly_.begin_substep_rows(network_.friction_row_count());
    assembly_.begin_substep_couplings(network_.coupling_count());
    if (!compliant_.empty()) assembly_.begin_substep_implicit(network_.node_count());
}

real Powertrain::hybrid_generator_power(real rpm,real fraction) const {
    real total=0;
    for(const auto& battery:batteries_) {
        real mechanical=0,electrical=0;
        for(const auto* m:electric_motors_)if(m->is_crank_machine() && &m->battery()==battery.get()) {
            const auto& d=m->desc();
            const real p=std::min(d.regen_torque_nm*rpm_to_rad_per_s(rpm)*fraction,
                d.regen_power_w>0?d.regen_power_w:d.max_power_w);
            mechanical+=p;electrical+=p*d.efficiency;
        }
        total+=mechanical*std::min(real(1),battery->desc.max_charge_w/std::max(real(1),electrical));
    }
    return total;
}

void Powertrain::plan_hybrid_charge() {
    const auto plan=select_hybrid_charge_point(hybrid_->charge_power_w,engine_->idle_rpm()*real(1.4),
        engine_limiter_rpm_*real(.95),hybrid_->charge_optimize_efficiency,
        [&](real rpm){
            for(const auto* m:electric_motors_)if(m->is_crank_machine()&&rpm>=m->desc().max_rpm)return real(0);
            return std::min(hybrid_generator_power(rpm,1),engine_->request_envelope(rpm).f_nm*rpm_to_rad_per_s(rpm)/real(1.1));},
        [&](real rpm,real power){return engine_->charging_fuel_g_s(rpm,power/rpm_to_rad_per_s(rpm));});
    hybrid_charge_rpm_=plan.rpm;hybrid_charge_power_w_=plan.power_w;
}

int Powertrain::hybrid_fit_gear(const ComponentSubstepCtx& ctx) const {
    if (!gearbox_) return 0;
    constexpr real kRpm = real(60) / (real(2) * real(3.14159265358979323846));
    const real th = hybrid_driver_command_;
    if (auto_) {
        if (auto_->desc().cvt) return 0;
        AutoTransmission::Sensors fs;
        fs.output_omega = network_.omega(gearbox_out_);
        fs.mode_channel = last_mode_channel_;
        fs.up_rpm_at_zero_throttle = ctx.drive_up_rpm_zero;
        fs.up_rpm_at_full_throttle = ctx.drive_up_rpm_full;
        fs.down_rpm_at_zero_throttle = ctx.drive_down_rpm_zero;
        fs.down_rpm_at_full_throttle = ctx.drive_down_rpm_full;
        return auto_->fit_gear(fs, th);
    }
    if (!tcu_) return 0;
    const auto pick = [](real override_value, real own) { return std::isfinite(override_value) ? override_value : own; };
    const AutoShiftDesc& as = tcu_->desc().auto_shift;
    real up_z = pick(ctx.drive_up_rpm_zero, as.up_rpm_at_zero_throttle);
    real up_f = pick(ctx.drive_up_rpm_full, as.up_rpm_at_full_throttle);
    const real k = shift_point_scale(up_z, up_f, tcu_->desc().shift_rpm_cap);
    if (k != real(1.0)) {
        up_z *= k;
        up_f *= k;
    }
    const real up = up_z + (up_f - up_z) * th;
    const int n = static_cast<int>(gearbox_->desc().forward_ratios.size());
    const real w_out = network_.omega(gearbox_out_);
    if (n < 1 || w_out * gearbox_->ratio_for_position(1) <= real(0.0)) return n < 1 ? 0 : 1;
    for (int g = 1; g < n; ++g)
        if (std::abs(gearbox_->ratio_for_position(g) * w_out) * kRpm <= up) return g;
    return n;
}

void Powertrain::pre_solve_components(const ComponentSubstepCtx& ctx_in) {
    ComponentSubstepCtx ctx = ctx_in;
    real b_pedal = ctx_in.throttle; // B mode: the driver's pedal (a hybrid: the electric share of the split)
    if(hybrid_) {
        const auto channel=[&](int slot,real fallback){return ctx.channels&&slot>=0&&slot<ctx.channel_count?ctx.channels[slot]:fallback;};
        const bool ready=channel(hybrid_ready_slot_,1)>.5;
        const int mode_raw=static_cast<int>(std::clamp(std::round(channel(hybrid_mode_slot_,1)),real(0),real(4)));
        int mode=mode_raw;
        // Electric mode (docs/electric_drive.md): ICE off and disconnected, traction motors only. A pack at or below low_soc
        // leaves it for Normal (latched until the channel moves off Electric; the car writes the part's default back).
        hybrid_electric_active_=false;
        if(mode_raw==4) {
            if(!ctx_in.electric_mode_allowed||hybrid_electric_lost_)mode=1;
            else if(battery_soc()<=hybrid_->low_soc){hybrid_electric_lost_=true;hybrid_electric_reason_=1;hybrid_electric_leave_pending_=true;mode=1;}
            else {mode=4;hybrid_electric_active_=true;hybrid_electric_reason_=0;}
        } else hybrid_electric_lost_=false;
        if(ctx_in.gear_request!=kGearRequestNone)hybrid_selector_=ctx_in.gear_request<0?-1:(ctx_in.gear_request>0?1:0);
        b_update_selector(ctx_in,hybrid_electric_active_);
        if(swallow_up_!=real(0)||swallow_down_!=real(0)){ctx.shift_up_count=ctx_in.shift_up_count-swallow_up_;ctx.shift_down_count=ctx_in.shift_down_count-swallow_down_;}
        real pack=0,motors=0,stack=0;
        for(const auto& b:batteries_)if(b->soc()>0)pack+=b->desc.max_discharge_w;
        for(const auto* c:fuel_cells_)if(c->hydrogen_kg>0)stack+=c->rated_power();
        for(const auto* m:electric_motors_)if(!m->is_crank_machine())motors+=m->rated_power();
        hybrid_available_w_=std::min(motors,pack+stack);
        // The driver's command (pedal curve and drive-mode bend applied, BEFORE the TC/LC ceilings): the power request, the
        // ICE/electric split and the shift schedule all read it. A TC ceiling is applied once, downstream, by the engine
        // stage (min with the ICE command) and the motors (min with their pedal) - not folded into the request, which
        // amplified it through the split (pedal 0.75 -> ICE 0.5) and pulled the shift schedule to part-throttle points.
        hybrid_driver_command_=std::clamp(throttle_stage_.evaluate(ctx_in).command,real(0),real(1));
        const bool running=last_engine_result_.state==EngineState::Running;
        // The split itself is the public pure function (ps/drivetrain/hybrid_split.h; the charge latch is updated in it).
        HybridSplitInput split_in;
        split_in.driver_command=hybrid_driver_command_;
        split_in.available_w=hybrid_available_w_;
        split_in.drive_mode=mode;
        split_in.ice_full_power_w=hybrid_->requested_power_derived?hybrid_ice_full_w_:real(0);
        split_in.battery_soc=battery_soc();
        split_in.charge_latched=hybrid_charge_latched_;
        split_in.ready=ready;
        split_in.selector=hybrid_selector_;
        const HybridSplit split=hybrid_power_split(*hybrid_,split_in);
        hybrid_charge_latched_=split.charge_latched;
        const bool traction=split.engine_coupled;
        const bool want=split.engine_wanted;
        hybrid_idle_s_=want?0:hybrid_idle_s_+ctx.h;
        hybrid_ice_requested_=!hybrid_electric_active_&&(want||(ready&&running&&hybrid_idle_s_<hybrid_->stop_delay_s));
        hybrid_crank_s_=(!hybrid_ice_requested_||running)?0:hybrid_crank_s_+ctx.h;
        ctx.ignition=hybrid_ice_requested_?1:0;
        ctx.starter=hybrid_ice_requested_&&!running&&hybrid_crank_s_<hybrid_->crank_timeout_s?1:0;
        const bool hybrid_was_disconnected=hybrid_disconnect_;
        hybrid_disconnect_=!traction||!running;
        hybrid_requested_w_=split.requested_w;
        b_pedal=split.electric_command;
        ctx.electric_throttle=split.electric_command;
        ctx.electric_charge=0;
        ctx.throttle=split.engine_command;
        ctx.throttle_is_command=true;
        if(hybrid_charge_latched_&&running&&hybrid_disconnect_&&!hybrid_electric_active_) {
            const real rpm=std::abs(network_.omega(engine_crank_))*60/(2*3.14159265358979323846);
            const real taper=std::clamp((hybrid_->charge_target_soc-battery_soc())/real(.02),real(0),real(1));
            hybrid_charge_plan_s_-=ctx.h;
            if(hybrid_charge_plan_s_<=0){plan_hybrid_charge();hybrid_charge_plan_s_=real(.5);}
            const real omega=std::max(real(1),rpm_to_rad_per_s(rpm));
            // Unload below target speed so the engine can accelerate into its feasible band.
            const real ramp=std::clamp((rpm-engine_->idle_rpm())/std::max(real(1),hybrid_charge_rpm_-engine_->idle_rpm()),real(0),real(1));
            const real power=hybrid_charge_power_w_*taper*ramp;
            real lo=0,hi=1;
            for(int i=0;i<24;++i){const real mid=(lo+hi)*real(.5);if(hybrid_generator_power(rpm,mid)<power)lo=mid;else hi=mid;}
            ctx.electric_charge=power>0?hi:0;
            const auto env=engine_->request_envelope(rpm);
            const real ff=std::clamp((power/omega-env.z_nm)/std::max(real(1),env.f_nm-env.z_nm),real(0),real(1));
            const real error=hybrid_charge_rpm_-rpm;
            const real raw=ff+real(.0003)*error+hybrid_charge_integral_;
            if((raw>0&&raw<1)||(raw<=0&&error>0)||(raw>=1&&error<0))
                hybrid_charge_integral_=std::clamp(hybrid_charge_integral_+real(.00015)*error*ctx.h,real(-.5),real(.5));
            ctx.throttle=std::clamp(raw,real(0),real(1));
        } else {
            hybrid_charge_plan_s_=hybrid_charge_rpm_=hybrid_charge_power_w_=hybrid_charge_integral_=0;
        }
        // ICE connect while rolling: engaging a gear on an idling engine at road speed dragged the engine up through the
        // clutch (or shifted through every gear). Engage the gear the schedule fits, keep the clutch open and rev the
        // engine toward that gear's input speed (same proportional law as the charge mode), then close within the clutch
        // law's lock window - or after connect_timeout_s. Never opens at standstill (the fit gear's input speed is 0).
        int hybrid_fit=0;
        if(hybrid_disconnect_||!gearbox_||!engine_||(auto_&&dyno_gear_hold_!=kNoDynoGearHold))hybrid_connect_s_=0; // dyno hold: no rev-match, the gear is the dyno's
        else if(hybrid_selector_>0){
            hybrid_fit=hybrid_fit_gear(ctx);
            if(hybrid_fit>0){
                constexpr real kRpm=real(60)/(real(2)*real(3.14159265358979323846));
                const real fit_ratio=auto_?auto_->ratio(hybrid_fit):gearbox_->ratio_for_position(hybrid_fit);
                const real target=std::abs(fit_ratio*network_.omega(gearbox_out_))*kRpm;
                const real rpm=std::abs(network_.omega(engine_crank_))*kRpm;
                const real window=auto_?auto_->desc().tcu.launch_lock_slip_rpm:(tcu_?tcu_->desc().auto_clutch.lock_slip_rpm:real(80));
                if(hybrid_connect_s_>0){
                    hybrid_connect_s_+=ctx.h;
                    if(target-rpm<=window||hybrid_connect_s_>=hybrid_->connect_timeout_s)hybrid_connect_s_=0;
                } else if(hybrid_was_disconnected&&hybrid_->connect_timeout_s>0&&target-rpm>window)hybrid_connect_s_=ctx.h;
                if(hybrid_connect_s_>0)ctx.throttle=std::clamp(real(.45)+real(.0006)*(target-rpm),real(0),real(1));
            } else hybrid_connect_s_=0;
        } else hybrid_connect_s_=0;
        ctx.assist_auto_clutch=true;ctx.assist_auto_shift=true;
        if(hybrid_disconnect_)ctx.gear_request=0;
        else if(auto_)ctx.gear_request=hybrid_selector_<0?-1:real(99);
        else {
            // Manual-TCU gearbox: select 1st only from a settled neutral (D chosen from N, or traction resuming after the
            // disconnect). A shift passes through position 0 while Releasing/Selecting; asking for 1st then reverted every
            // upshift (and, as a valid request, discarded the driver's shift_up_count presses) - the car never left 1st.
            const bool settled_neutral=gearbox_->current_position()==0&&gearbox_->phase()==Gearbox::Phase::Idle&&gearbox_->target()==0;
            ctx.gear_request=hybrid_selector_<0?-1:(settled_neutral?real(hybrid_fit>0?hybrid_fit:1):kGearRequestNone);
        }
    }

    // Powertrain dyno (powertrain_dyno.cpp) only: hold the dyno's gear (a hybrid supervisor's disconnect keeps its neutral).
    // dyno_gear_hold_ is kNoDynoGearHold in every car, so this is never taken at runtime and nothing here is hashed.
    // An automatic / DCT / CVT gearbox takes the same request and holds it inside AutoTransmission (dyno hold).
    if (dyno_gear_hold_ != kNoDynoGearHold && gearbox_) {
        ctx.assist_auto_shift = false;
        if (!(hybrid_ && hybrid_disconnect_)) ctx.gear_request = static_cast<real>(dyno_gear_hold_);
    }

    // --- plan section 7.2 step 3: controllers -------------------------------
    if (gearbox_) {
        if (tcu_) {
            // Engine reads LAST substep's TCU request (lagged - plan section
            // 7.2's deliberate order independence); the cache is then
            // overwritten with THIS substep's for next time. This assignment
            // depends only on tcu_prev_request_ (set last substep), never on
            // `out` below, so it is safe to read here, BEFORE tcu_->step().
            ctx.tcu_throttle_floor = tcu_prev_request_.throttle_floor;
            ctx.tcu_ignition_cut = tcu_prev_request_.ignition_cut;
            ctx.tcu_throttle_ceiling = tcu_prev_request_.throttle_ceiling;

            // Assists I2 (PHYS-074 D-LS1/D-LS3): at crawl speed traction control acts through the auto-clutch.
            // In 1st at idle the rim of car_hyper turns 3.49 m/s [D], so a clutch locked below ~3 m/s forces
            // wheelspin and an engine-torque cut can only remove it by dragging the locked engine below anti-stall
            // (the Part A limit cycle). While the auto-clutch is in Launch or Locked mode with the ground-speed
            // input rpm below 1.25 x anti_stall_rpm [AS], the engine keeps the driver's command (no TC ceiling, no
            // TC ignition cut, and the launch law sees that command) and the TC fraction caps the clutch's
            // capacity instead: cap = f * T_d + (1 - f) * T_floor, T_floor = max(0, min_fraction * T_d), T_d the
            // engine's demand at the driver's command (previous substep, like every engine -> TCU exchange). The
            // clutch slips instead of pulling the engine down. Everywhere else (Shift mode, faster, manual clutch,
            // TC not intervening, TC Off - ctx.traction_torque_fraction stays 1 then) the engine-ceiling path is
            // unchanged. TCU mode = last substep's (lagged).
            const int cur_gear = gearbox_->current_position();
            const real ground_to_in = (cur_gear != 0 && speedo_radius_ > real(0.0))
                                          ? gearbox_->ratio_for_position(cur_gear) * speedo_ratio_ / speedo_radius_
                                          : real(0.0);
            tc_via_clutch_ = false;
            clutch_capacity_cap_tc_ = std::numeric_limits<real>::infinity();
            if (engine_ && ctx.assist_auto_clutch && ctx.traction_torque_fraction < real(1.0) && cur_gear != 0 &&
                tcu_->desc().clutch_capacity_nm > real(0.0)) {
                // Crawl band in BOTH clutch modes: a Launch-mode-only gate without it kept a slipping clutch with the
                // engine on the limiter up to 12 m/s in the hyper TC test (measured, PROVISIONAL decision 5).
                constexpr real kTcClutchCrawlBand = real(1.25); // [AS] plan B2, x anti_stall_rpm
                const real ground_in_rpm = ctx.ground_speed_mps * ground_to_in * (real(60.0) / (real(2.0) * real(3.14159265358979323846)));
                const bool crawl = ground_in_rpm < kTcClutchCrawlBand * tcu_->desc().auto_clutch.anti_stall_rpm; // NaN: false
                const ManualTcu::ClutchMode mode = tcu_->clutch_mode();
                tc_via_clutch_ = crawl && (mode == ManualTcu::ClutchMode::Launch || mode == ManualTcu::ClutchMode::Locked);
            }
            if (tc_via_clutch_) {
                const real f = ctx.traction_torque_fraction;
                const real t_d = last_engine_result_.torque_demand;
                const real t_floor = std::max(real(0.0), ctx.traction_floor_fraction * t_d);
                clutch_capacity_cap_tc_ = std::max(real(0.0), f * t_d + (real(1.0) - f) * t_floor);
                ctx.traction_throttle_ceiling = real(1.0);
                ctx.traction_ignition_cut = false;
            }

            // PHYS-033 P4 I7a C5b (owner ruling 2026-09-26): the throttle-
            // command stage is evaluated HERE, before the TCU, so the TCU's
            // launch law / auto-shift thresholds / auto-blip act on the
            // stage's torque REQUEST (pedal_map/hand-lever applied), not the
            // raw pedal. This is a pure reorder of a stateless computation:
            // the stage has no state of its own and reads only ctx.throttle,
            // ctx.channels (hand lever) and ctx.tcu_throttle_floor/
            // tcu_throttle_ceiling - all already available above (the lagged
            // TCU limits assigned just now) - never anything the TCU writes
            // this same substep. Moving this earlier does not change its
            // result (see throttle_stage.h's own header comment for what the
            // stage depends on).
            if (engine_) {
                last_throttle_command_ = throttle_stage_.evaluate(ctx, last_throttle_hand_lever_raw_);
            }

            ManualTcu::Sensors s;
            s.engine_omega = engine_ ? network_.omega(engine_crank_) : real(0.0);
            s.gearbox_in_omega = network_.omega(gearbox_in_);
            s.gearbox_out_omega = network_.omega(gearbox_out_);
            s.approx_speed_mps = approx_speed_mps();
            // PHYS-033 P4 I7a C5b: the torque REQUEST (throttle_stage_'s
            // command - pedal_map/hand-lever already applied), not the raw
            // pedal - see Sensors::throttle_request's own comment
            // (manual_tcu.h). Without an engine there is no throttle stage
            // to speak of; fall back to the raw pedal exactly as before.
            s.throttle_request = engine_ ? last_throttle_command_.command : ctx.throttle;
            // Pedal P1: the gear choice reads the driver's request, before the TC ceiling below (V8c: the ceiling
            // at the launch-control hand-over made auto-shift take 2nd at 7.4 m/s and lug at 1430 rpm).
            s.shift_throttle_request = hybrid_ ? hybrid_driver_command_ : s.throttle_request; // hybrid: the driver's pedal, not the ICE share
            // TC changes the available ECU request: launch clutch must not
            // wait for full-pedal launch RPM after TC has limited that request.
            // Assists I2 (D-LS3): not while TC acts through the clutch (ctx.traction_throttle_ceiling is 1 then).
            if (ctx.traction_throttle_ceiling < real(1)) s.throttle_request = std::min(s.throttle_request,ctx.traction_throttle_ceiling);
            s.ground_speed_mps = ctx.ground_speed_mps; // assists I2 (D-LS2), NaN with TC Off
            s.ground_speed_to_input_omega = ground_to_in;
            s.clutch_capacity_cap_nm = clutch_capacity_cap_tc_; // assists I2 (D-LS1)
            s.up_rpm_at_zero_throttle = ctx.drive_up_rpm_zero; // assists I7: the drive mode's auto-shift / close-time overrides (NaN = the car's)
            s.up_rpm_at_full_throttle = ctx.drive_up_rpm_full;
            s.down_rpm_at_zero_throttle = ctx.drive_down_rpm_zero;
            s.down_rpm_at_full_throttle = ctx.drive_down_rpm_full;
            s.close_time_s = ctx.drive_close_time_s;
            // Assists I4 (PHYS-074 D-LC1): the unit's launch control owns the clutch while Armed (held open) and
            // Launch (a torque-following capacity, ramped 0 -> 1 over the unit's clutch_ramp_s). VehicleInstance
            // suspends traction control in those phases (ctx.traction_torque_fraction stays 1), so the I2 cap above
            // is +inf then; the min() keeps the precedence explicit and bit-identical when neither is active.
            //   Launch cap = ramp * max(0, sign(slip) * T_engine + T_sync): the TCU's own torque-following close law
            //   (ManualTcu::sync_close_target) - the clutch carries what the engine makes plus a bounded
            //   synchronising torque that pulls the held-rpm engine onto the input shaft, so the wheels never see
            //   more than T_engine + max_sync_torque_nm through the clutch. T_engine is the engine's demand at the
            //   throttle command INCLUDING the launch ceiling (previous substep), so the slip PI's cut shows up in
            //   the cap as well.
            s.defer_auto_shift = hybrid_connect_s_ > real(0.0);
            if (ctx.launch_hold_clutch || s.defer_auto_shift) {
                s.clutch_capacity_cap_nm = real(0.0);
            } else if (ctx.launch_clutch_ramp >= real(0.0) && engine_) {
                const real slip = s.engine_omega - s.gearbox_in_omega;
                const real dir = slip >= real(0.0) ? real(1.0) : real(-1.0);
                const AutoClutchDesc& ac = tcu_->desc().auto_clutch;
                const real t_sync = std::min(tcu_->desc().engine_inertia * std::abs(slip) / ac.sync_time_s, ac.max_sync_torque_nm);
                const real t = std::max(real(0.0), dir * last_engine_result_.torque_demand + t_sync);
                s.clutch_capacity_cap_nm = std::min(s.clutch_capacity_cap_nm, std::min(ctx.launch_clutch_ramp, real(1.0)) * t);
            }
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
            tcu_prev_request_.throttle_floor = out.throttle_floor;
            tcu_prev_request_.ignition_cut = out.ignition_cut;
            tcu_prev_request_.throttle_ceiling = out.throttle_ceiling;
        } else if (auto_) {
            // Automatic gearbox plan A1: the automatic TCU and the clutch-to-clutch shift machine. Same lagged
            // engine limits as the manual TCU (the torque reduction of a power-on upshift is a throttle ceiling).
            ctx.tcu_throttle_floor = tcu_prev_request_.throttle_floor;
            ctx.tcu_ignition_cut = tcu_prev_request_.ignition_cut;
            ctx.tcu_throttle_ceiling = tcu_prev_request_.throttle_ceiling;
            // DCT/clutch-CVT crawl: preserve idle combustion and meter the
            // traction-control torque through clutch slip, as for the AMT.
            tc_via_clutch_ = false;
            clutch_capacity_cap_tc_ = std::numeric_limits<real>::infinity();
            const int gear = auto_->current();
            if (engine_ && auto_->slip_launch() && gear != 0 && ctx.traction_torque_fraction < real(1.0) && speedo_radius_ > real(0.0)) {
                const real ground_rpm = std::abs(ctx.ground_speed_mps * auto_->ratio(gear) * speedo_ratio_ / speedo_radius_) * real(60.0) / (real(2.0) * real(3.14159265358979323846));
                if (ground_rpm < real(1.25) * auto_->desc().tcu.launch_anti_stall_rpm) {
                    const real f = ctx.traction_torque_fraction;
                    const real demand = std::max(real(0.0), last_engine_result_.torque_demand);
                    const real floor = std::max(real(0.0), ctx.traction_floor_fraction * demand);
                    clutch_capacity_cap_tc_ = f * demand + (real(1.0) - f) * floor;
                    tc_via_clutch_ = true;
                    ctx.traction_throttle_ceiling = real(1.0);
                    ctx.traction_ignition_cut = false;
                }
            }
            if (engine_) last_throttle_command_ = throttle_stage_.evaluate(ctx, last_throttle_hand_lever_raw_);
            AutoTransmission::Sensors as;
            as.clutch_capacity_cap_nm = clutch_capacity_cap_tc_;
            as.engine_omega = engine_ ? network_.omega(engine_crank_) : real(0.0);
            as.input_omega = network_.omega(gearbox_in_);
            as.output_omega = network_.omega(gearbox_out_);
            as.throttle = engine_ ? last_throttle_command_.command : ctx.throttle;
            // Hybrid: the schedule reads the driver's pedal command (before the TC ceiling), as in every other car. It used to
            // read requested / (motor + ICE envelope power), a fraction of the whole system's capability: ~0.3 at full pedal
            // on the hypercar, i.e. the part-throttle upshift points (4100 rpm instead of ~8000 in Racing).
            if(hybrid_)as.throttle=hybrid_driver_command_;
            as.up_rpm_at_zero_throttle = ctx.drive_up_rpm_zero;
            as.up_rpm_at_full_throttle = ctx.drive_up_rpm_full;
            as.down_rpm_at_zero_throttle = ctx.drive_down_rpm_zero;
            as.down_rpm_at_full_throttle = ctx.drive_down_rpm_full;
            as.close_time_s = ctx.drive_close_time_s;
            as.gear_request = ctx.gear_request;
            as.shift_up_count = ctx.shift_up_count;
            as.shift_down_count = ctx.shift_down_count;
            as.mode_channel = (has_mode_channel_ && ctx.channels != nullptr && static_cast<int>(mode_channel_slot_) < ctx.channel_count)
                                  ? ctx.channels[mode_channel_slot_]
                                  : real(0.0);
            last_mode_channel_ = as.mode_channel;
            as.input_torque_prev = std::abs(network_.row_lambda(gearbox_synchro_row_) + network_.row_lambda(shift_row_b_));
            if (engine_) as.engine_torque_demand = last_engine_result_.torque_demand;
            // DCT launch control (the assist unit's launch_control, VehicleInstance arms it only on a DualClutch): Armed
            // holds the launch clutch open; Launch gives it the manual TCU's torque-following cap,
            //   ramp * max(0, sign(slip) * T_engine + T_sync),  T_sync = min(kLcSyncNmPerRadS * |slip|, kLcSyncFraction * C),
            // so the clutch carries what the held-rpm engine makes plus a bounded synchronising torque.
            as.launch_hold_open = ctx.launch_hold_clutch;
            as.rev_match_hold = hybrid_connect_s_ > real(0.0);
            if (ctx.launch_clutch_ramp >= real(0.0) && engine_) {
                constexpr real kLcSyncNmPerRadS = real(0.5), kLcSyncFraction = real(0.2); // [AS]
                const real slip = as.input_omega - auto_->ratio(auto_->current()) * as.output_omega;
                const real dir = slip >= real(0.0) ? real(1.0) : real(-1.0);
                const real C = auto_->desc().clutch.capacity_nm;
                const real t_sync = std::min(kLcSyncNmPerRadS * std::abs(slip), kLcSyncFraction * C);
                const real t = std::max(real(0.0), dir * last_engine_result_.torque_demand + t_sync);
                as.launch_cap_nm = std::min(ctx.launch_clutch_ramp, real(1.0)) * t;
            }
            as.first_substep = ctx.first_substep_of_tick;
            as.h = ctx.h;
            last_auto_out_ = auto_->step(as);
            gearbox_->reset_to(last_auto_out_.current); // ratio/efficiency lookups and current_position() for readers
            tcu_prev_request_.throttle_floor = real(0.0);
            tcu_prev_request_.ignition_cut = false;
            tcu_prev_request_.throttle_ceiling = last_auto_out_.throttle_ceiling;
        }

        if (auto_) {
            last_gearbox_result_ = Gearbox::StepResult{};
            last_gearbox_result_.state_index = 0; // a clutch-to-clutch box has one topology state
            last_gearbox_result_.current = last_auto_out_.current;
            last_gearbox_result_.target = last_auto_out_.target;
            ctx.gearbox_in_neutral = (last_auto_out_.current == 0);
            // The engine's start interlock and stall logic: an Automatic couples through its converter (always
            // "engaged": the starter works only in N), a DualClutch through its launch clutch.
            ctx.clutch_engaged_fraction = last_auto_out_.launch_engagement;
        } else {
            Gearbox::StepInput gin;
            gin.omega1 = network_.omega(gearbox_in_);
            gin.omega2 = network_.omega(gearbox_out_);
            gin.lambda_mesh_prev = gearbox_lambda_mesh_prev_;
            gin.h = ctx.h;
            last_gearbox_result_ = gearbox_->phase_step(gin);
            ctx.gearbox_in_neutral = (last_gearbox_result_.current == 0);
        }
    }

    // --- step 3b: a possible topology-state switch (plan section 5.3; I4 C5
    // section 2.6: the COMPOSED s = g + n_g*lock_bits, lock requests decided
    // after phase_step). One projection covers a simultaneous gear change and
    // lock engage. No gearbox and no lockable diff: s == 0 always, never a
    // switch (the pre-C5 code did not compare at all there). ---
    {
        const int g = gearbox_ ? last_gearbox_result_.state_index : 0;
        const int s = g + gear_state_count_ * update_lock_bits(ctx);
        if (s != network_.topology_state()) {
            ledger_.engagement_loss += apply_topology_switch(s, ctx.h);
            topo_switched_this_substep_ = true; // C13: makes the NEXT substep backward Euler (read by solve())
        }
    }

    if (clutch_) {
        last_clutch_result_ = clutch_->pre_solve(ctx.clutch_pedal, ctx.e_auto);
        ctx.clutch_engaged_fraction = last_clutch_result_.engagement;
        assembly_.set_friction_capacity(clutch_row_, last_clutch_result_.cap_static, last_clutch_result_.cap_kinetic);
    }

    // --- step 4: throttle-command stage + engine (I7a C3: the stage sits
    // strictly before the engine's own pre_solve, plan section 7.2 step 4;
    // I7a C5b: already evaluated above, ahead of the TCU, whenever a
    // gearbox_+tcu_ pair exists - only a vehicle with an engine but no
    // manual transmission at all (none shipped, but structurally possible)
    // still needs it evaluated here) ------------------------------------
    if (engine_) {
        if (!(gearbox_ && (tcu_ || auto_))) {
            last_throttle_command_ = throttle_stage_.evaluate(ctx, last_throttle_hand_lever_raw_);
        }
        // A disconnected engine produces no tyre torque. Keep wheel interventions on the
        // traction motors, but remove their throttle and ignition cuts from the ICE.
        if(hybrid_&&hybrid_disconnect_) {
            auto engine_ctx=ctx;
            engine_ctx.traction_throttle_ceiling=engine_ctx.tcu_throttle_ceiling=engine_ctx.launch_throttle_ceiling=1;
            engine_ctx.tcu_throttle_floor=0;
            last_throttle_command_=throttle_stage_.evaluate(engine_ctx,last_throttle_hand_lever_raw_);
        }
        const ThrottleCommand& throttle_cmd = last_throttle_command_;

        EngineTorqueMap::Sensors s;
        s.omega = network_.omega(engine_crank_);
        s.ignition = ctx.ignition;
        s.starter = ctx.starter;
        s.clutch_engaged_fraction = ctx.clutch_engaged_fraction;
        s.gearbox_in_neutral = ctx.gearbox_in_neutral;
        if (fuel_tank_) s.fuel_available_kg = fuel_tank_->content(); // PHYS-033 P4 I7a C6: else +infinity (never starves)
        s.ambient = ctx.ambient; // PHYS-033 P4 I7b C9; EngineTorqueMap ignores it
        if (thermal_) { // engine thermal plan T1: last tick's scales; defaults 1.0 without thermal
            s.friction_scale = thermal_->plant_friction_scale();
            s.ecu_friction_scale = thermal_->ecu_friction_scale();
            s.friction_reference_rpm = thermal_->hydrodynamic_reference_rpm();
            s.oil_temperature_c = thermal_->oil_temperature_k() - real(273.15); // engine garage P9: cam switching gate
        }
        if (anti_lag_channel_live_ && ctx.channels != nullptr && anti_lag_channel_slot_ < ctx.channel_count)
            s.anti_lag_channel = ctx.channels[anti_lag_channel_slot_]; // I7d D5; else NaN = default_enabled
        last_engine_sensors_ambient_ = ctx.ambient; // test-only white-box hook
        if (nitrous_) {
            // N2O plan N1: the nitrous system decides this substep's flows BEFORE the engine's pre_solve
            // (previous substep's engine flags, the throttle stage's command). The engine ignores the block in N1.
            NitrousInputs ni;
            ni.h = ctx.h;
            auto channel = [&](int slot, real fallback) {
                return (ctx.channels != nullptr && slot >= 0 && slot < ctx.channel_count) ? ctx.channels[slot] : fallback;
            };
            ni.arm = channel(nitrous_->arm_slot(), nitrous_->kit().controller.arm_default);
            if (!ctx.nitrous_arm_allowed) ni.arm = real(0.0); // assists I7: the effective drive mode forbids arming
            ni.heater = channel(nitrous_->heater_slot(), nitrous_->bottle().has_heater ? nitrous_->bottle().heater.default_on : real(0.0));
            ni.purge = channel(nitrous_->purge_slot(), real(0.0));
            ni.throttle_command = throttle_cmd.command;
            ni.omega = s.omega;
            ni.has_gearbox = gearbox_ != nullptr;
            ni.gear = gearbox_ ? gearbox_->current_position() : 0;
            ni.speed_mps = approx_speed_mps();
            ni.engine_running = last_engine_result_.state == EngineState::Running;
            ni.ignition_cut = ctx.tcu_ignition_cut || ctx.traction_ignition_cut;
            ni.p_intake_pa = engine_->intake_pressure_pa();
            ni.p_ambient_pa = ctx.ambient.pressure_pa;
            nitrous_->step(ni);
            s.nitrous = nitrous_->engine_input();
        }
        EngineTorqueMap::TcuRequest req{!(hybrid_&&hybrid_disconnect_) && (ctx.tcu_ignition_cut || ctx.traction_ignition_cut)};
        req.hold_boost_control = !(hybrid_&&hybrid_disconnect_) && (ctx.tcu_ignition_cut || ctx.tcu_throttle_ceiling < real(1.0)); // Boost B0: shift in progress
        req.traction_cut = !(hybrid_&&hybrid_disconnect_) && ctx.traction_ignition_cut && !ctx.tcu_ignition_cut; // pedal P1 (b), engine.h TcuRequest
        if (!(hybrid_&&hybrid_disconnect_) && ctx.launch_limiter_rpm > real(0.0)) req.limiter_rpm = ctx.launch_limiter_rpm; // assists I4: the unit's launch control (Armed)
        if (launch_) { // engine garage P8 (A16): the second rev limit, through the engine's own limiter
            LaunchControl::Inputs li;
            li.switch_on = ctx.assist_launch_control;
            li.speed_mps = approx_speed_mps();
            li.has_clutch = clutch_ != nullptr;
            li.clutch_engagement = ctx.clutch_engaged_fraction;
            req.limiter_rpm = launch_->step(li);
        }
        // TC crawl rev hold: while traction control caps the auto-clutch instead of the engine (tc_via_clutch_), the
        // engine keeps the driver's command, so without a limit the slipping clutch lets it run to the limiter. Hold
        // it at the launch law's full-engagement rpm for this command (the rpm a normal auto-clutch launch slips at),
        // through the engine's own limiter like launch control.
        if (tc_via_clutch_ && tcu_) {
            const auto& ac = tcu_->desc().auto_clutch;
            const real cmd = std::clamp(last_throttle_command_.command, real(0.0), real(1.0));
            const real hold = ac.full_rpm_min + cmd * (ac.full_rpm_max - ac.full_rpm_min);
            req.limiter_rpm = req.limiter_rpm > real(0.0) ? std::min(req.limiter_rpm, hold) : hold;
        }
        if (tc_via_clutch_ && auto_) {
            const auto& t = auto_->desc().tcu;
            const real cmd = std::clamp(last_throttle_command_.command, real(0.0), real(1.0));
            const real hold = t.launch_full_rpm_min + cmd * (t.launch_full_rpm_max - t.launch_full_rpm_min);
            req.limiter_rpm = req.limiter_rpm > real(0.0) ? std::min(req.limiter_rpm, hold) : hold;
        }
        last_engine_result_ = engine_->pre_solve(s, throttle_cmd, req, ctx.h);
        const auto& pre = last_engine_result_;
        if (nitrous_) nitrous_->observe(pre.limiter_active || pre.dfco_active, pre.fuel_supply_fraction < real(1.0));
        assembly_.torque(engine_crank_, pre.tau0, pre.d);
        assembly_.set_friction_capacity(engine_friction_row_, pre.friction_cap_static, pre.friction_cap_kinetic);
    }

    // --- step 4: gearbox mesh loss + synchro row (plan section 5.4/5.5) -----
    if (gearbox_) {
        gearbox_omega_old_in_ = network_.omega(gearbox_in_);
        const int cur = gearbox_->current_position();
        real eff = cur == 0 ? real(1.0) : gearbox_->efficiency_for_position(cur);
        const CvtDesc* cvt = (auto_ && auto_->desc().cvt) ? &auto_->desc().cvt_desc : nullptr;
        if (cvt && cur > 0) eff = cvt->efficiency_vs_ratio.evaluate(last_auto_out_.cvt_ratio); // CVT plan C1
        if (cur != 0 && eff < real(1.0)) {
            gearbox_own_loss_tau_ = -(real(1.0) - eff) * std::abs(gearbox_lambda_mesh_prev_) * sign(gearbox_omega_old_in_);
        } else {
            gearbox_own_loss_tau_ = real(0.0);
        }
        if (cvt) {
            // CVT plan C1: the hydraulic pump's drag on the input, faded in over the first 10 rad/s [AS].
            const real drag = cvt->pump_drag_nm + cvt->pump_drag_nm_per_kn * last_auto_out_.clamp_force_n * real(1e-3);
            gearbox_own_loss_tau_ -= drag * std::clamp(gearbox_omega_old_in_ / real(10.0), real(-1.0), real(1.0));
        }
        if (gearbox_own_loss_tau_ != real(0.0)) assembly_.torque(gearbox_in_, gearbox_own_loss_tau_, real(0.0));
        if (auto_) {
            // Automatic gearbox plan A1: the two shift clutches (row A in the synchro slot, row B), the lock-up
            // clutch and the converter.
            const AutoTransmission::RowCmd& ra = last_auto_out_.row[0];
            const AutoTransmission::RowCmd& rb = last_auto_out_.row[1];
            assembly_.set_friction_row_ratio(gearbox_synchro_row_, ra.ratio);
            assembly_.set_friction_capacity(gearbox_synchro_row_, ra.cap_static, ra.cap_kinetic);
            assembly_.set_friction_row_ratio(shift_row_b_, rb.ratio);
            assembly_.set_friction_capacity(shift_row_b_, rb.cap_static, rb.cap_kinetic);
            if (converter_) {
                assembly_.set_friction_capacity(lockup_row_, last_auto_out_.lockup_cap_static, last_auto_out_.lockup_cap_kinetic);
                const real wp = network_.omega(converter_pump_);
                const real wt = network_.omega(gearbox_in_);
                const TorqueConverter::Result cr = converter_->evaluate(wp, wt);
                conv_tau_p_ = cr.tau_pump;
                conv_tau_t_ = cr.tau_turbine;
                conv_c_ = cr.slip_slope;
                conv_v_old_ = wp - wt;
                conv_sr_ = cr.speed_ratio;
                // The pump torque's own growth with pump speed at a fixed speed ratio (|T_p| ~ w^2), less the part the
                // slip coupling already carries, as an implicit damping on the pump node.
                conv_d_pump_ = std::abs(wp) > real(1e-6)
                                   ? std::max(real(0.0), real(2.0) * std::abs(cr.tau_pump) / std::abs(wp) - conv_c_ * std::abs(cr.speed_ratio))
                                   : real(0.0);
                if (cr.overrun) conv_d_pump_ = real(0.0);
                assembly_.torque(converter_pump_, conv_tau_p_ + conv_c_ * conv_v_old_, conv_d_pump_);
                assembly_.torque(gearbox_in_, conv_tau_t_ - conv_c_ * conv_v_old_, real(0.0));
                assembly_.set_coupling(converter_coupling_, real(1.0), real(1.0), conv_c_);
                conv_wp_old_ = wp;
            }
        } else {
            assembly_.set_friction_capacity(gearbox_synchro_row_, last_gearbox_result_.synchro_cap_static, last_gearbox_result_.synchro_cap_kinetic);
            if (last_gearbox_result_.synchro_target_ratio != real(0.0)) {
                assembly_.set_friction_row_ratio(gearbox_synchro_row_, last_gearbox_result_.synchro_target_ratio);
            }
        }
    }

    if(!electric_motors_.empty()) {
        if(hybrid_)electric_direction_=hybrid_selector_;
        else if(gearbox_)electric_direction_=static_cast<real>(gearbox_->current_position());
        else if(ctx.gear_request!=kGearRequestNone)electric_direction_=ctx.gear_request<0?-1:(ctx.gear_request>0?1:0);
        if(!hybrid_&&!gearbox_)b_update_selector(ctx_in,true);
        ctx.electric_direction=electric_direction_;
    }
    for(auto& b:batteries_) b->begin_substep(ctx.h);
    if (!regen_maps_.empty()) { prepare_regen(ctx); prepare_b(ctx, b_pedal); }
    // Discharge allocation: every motor that will drive this substep registers its request with its pack first, so a
    // limited pack shares its budget in proportion (Battery::reserve_discharge; docs/electric_drive.md).
    for(const ElectricMotor* m:electric_motors_) m->battery().add_drive_claim(m->drive_claim_j(ctx,network_));
    for (auto& c : components_) {
        if (!regen_maps_.empty() && c.get() == brakes_) reflect_regen(ctx); // every motor has run: the pads take the rest
        c->pre_solve(ctx, network_, assembly_);
    }

    // --- PHYS-033 P4 I4 C13: compliant shafts (see CompliantShaft in
    // powertrain.h). After any topology switch above, so v_old is the
    // post-projection twist rate the solve actually starts from. ---
    if (!compliant_.empty()) {
        be_this_ = be_next_;
        const real h = ctx.h;
        for (CompliantShaft& cs : compliant_) {
            cs.v_old = network_.omega(cs.a) - network_.omega(cs.b);
            if (be_this_) {
                cs.c_impl = cs.c + cs.k * h;
                cs.tau_expl = cs.k * cs.theta;
            } else {
                cs.c_impl = cs.c / real(2.0) + cs.k * h / real(4.0);
                cs.tau_expl = cs.k * cs.theta + cs.c_impl * cs.v_old;
            }
            assembly_.set_coupling(cs.coupling, real(1.0), real(1.0), cs.c_impl);
            assembly_.torque(cs.a, -cs.tau_expl, real(0.0));
            assembly_.torque(cs.b, cs.tau_expl, real(0.0));
        }
    }
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

    if (!compliant_.empty()) {
        for (int r = 0; r < network_.friction_row_count(); ++r) {
            compliant_stuck_before_[static_cast<std::size_t>(r)] =
                network_.row_stuck(FrictionRowId{static_cast<std::uint16_t>(r)}) ? std::uint8_t{1} : std::uint8_t{0};
        }
    }

    network_.solve(h, assembly_);

    if (rotor_reaction_) {
        for (std::size_t i = 0; i < rotor_ports_.size(); ++i) {
            rotor_port_alpha_[i] = (network_.omega(rotor_ports_[i].node) - rotor_port_omega_old_[i]) / h;
        }
    }

    // PHYS-033 P4 I4 C13: advance each compliant shaft's twist, book its
    // ledger terms, expose its implicit coupling reaction to the post_solve
    // reconstructions, and decide the next substep's scheme.
    real shaft_residual = real(0.0);
    if (!compliant_.empty()) {
        real pe = real(0.0);
        for (CompliantShaft& cs : compliant_) {
            const real v_new = network_.coupling_velocity(cs.coupling);
            const real theta_old = cs.theta;
            const real v_d = be_this_ ? v_new : (cs.v_old + v_new) / real(2.0);
            cs.theta = theta_old + h * v_d;
            const real tau_s = cs.tau_expl + cs.c_impl * v_new;
            cs.tau_last = tau_s;
            const real d_pe = real(0.5) * cs.k * (cs.theta * cs.theta - theta_old * theta_old);
            const real d_damp = h * cs.c * v_d * v_d;
            ledger_.shaft_damping_loss += d_damp;
            // The shaft's work on the network this substep is -h*tau_s*v_new;
            // whatever of it is not dPE + damping is the scheme's own residual.
            shaft_residual += h * tau_s * v_new - d_pe - d_damp;
            pe += real(0.5) * cs.k * cs.theta * cs.theta;
            assembly_.add_implicit_reaction(cs.a, -cs.c_impl * v_new);
            assembly_.add_implicit_reaction(cs.b, cs.c_impl * v_new);
        }
        ledger_.shaft_pe = pe;
        bool stick_changed = false;
        for (int r = 0; r < network_.friction_row_count() && !stick_changed; ++r) {
            const std::uint8_t now = network_.row_stuck(FrictionRowId{static_cast<std::uint16_t>(r)}) ? std::uint8_t{1} : std::uint8_t{0};
            stick_changed = now != compliant_stuck_before_[static_cast<std::size_t>(r)];
        }
        be_next_ = topo_switched_this_substep_ || stick_changed;
        topo_switched_this_substep_ = false;
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
    if (!compliant_.empty()) integrator_loss_substep += shaft_residual;
    ledger_.integrator_loss += integrator_loss_substep;
    ledger_.rotor_ke = rotor_ke;
}

void Powertrain::post_solve_components(const ComponentSubstepCtx& ctx) {
    if (engine_) {
        const real omega_new = network_.omega(engine_crank_);
        const auto post = engine_->post_solve(omega_new, ctx.h);
        ledger_.engine_out += post.engine_out_delta;
        ledger_.starter_in += post.starter_in_delta;
        // PHYS-033 P4 I7a C5 (owner ruling, post-review): indicated_work is
        // combustion's own mechanical work this substep (see
        // PostSolveResult::indicated_work_delta's own comment) and is
        // booked UNCONDITIONALLY - an engine without a fuel map still fires
        // combustion mechanically and this quantity is meaningful for it
        // too (N10 review fix: it now DOES feed the crude no-map fuel
        // estimate shipped in C5d, engine_torque_map.cpp's has_fuel_model
        // branch - indicated_work itself is booked the same way regardless
        // of which fuel path, if any, exists). Only the mass/LHV-derived
        // terms below - which need an actual fuel model to convert a mass
        // into energy - are gated on engine_has_fuel_model_ (post.
        // fuel_mass_delta/fuel_energy_in_delta are exactly 0.0 from any
        // engine without one - map or crude, PHYS-033 P4 I7a C5d).
        ledger_.indicated_work += post.indicated_work_delta;
        // PHYS-033 P4 I7b C12: the mean-value engine's pumping/friction split
        // of engine_out (0 from the graph engine) - diagnostic, outside
        // balance_error.
        ledger_.pumping_work += post.pumping_work_delta;
        ledger_.friction_work += post.friction_work_delta;
        if (engine_has_fuel_model_) {
            ledger_.fuel_mass += post.fuel_mass_delta;
            ledger_.fuel_energy_in += post.fuel_energy_in_delta;
            // N2O plan N2 (section 4): engine_heat = fuel_energy_in + n2o_energy_in - indicated_work; the N2O
            // term is added only when nonzero so every N2O-free substep keeps the old floating-point order.
            ledger_.kit_fuel_mass += post.kit_fuel_mass_delta;
            const real heat_delta = post.n2o_energy_in_delta != real(0.0)
                                        ? (post.fuel_energy_in_delta + post.n2o_energy_in_delta) - post.indicated_work_delta
                                        : post.fuel_energy_in_delta - post.indicated_work_delta;
            ledger_.engine_heat += heat_delta;
            if (heat_delta < real(0.0)) ++ledger_.fuel_efficiency_violations;
            // PHYS-033 P4 I7a C6: the tank loses exactly the booked mass
            // (the engine never books more than the tank held at pre_solve).
            if (fuel_tank_) fuel_tank_->withdraw(post.fuel_mass_delta);
        }
        const real v = network_.row_velocity(engine_friction_row_);
        const real lambda = network_.row_lambda(engine_friction_row_);
        const real coulomb_delta = -ctx.h * v * lambda;
        ledger_.engine_friction_loss += coulomb_delta;
        if (thermal_) {
            // Engine thermal plan T1 (4.4): released fuel energy and friction heat (engine's smooth friction + the
            // Coulomb row's loss, the latter times the share that is friction rather than pumping).
            thermal_->accumulate(post.released_heat_delta, post.friction_heat_delta + thermal_coulomb_share_ * coulomb_delta, ctx.h);
        }
        if (nitrous_) {
            // N2O plan N1/N2: the bottle loses what was delivered/vented (n2o_mass is the bottle's truth); the
            // engine reports the burnt share (post.n2o_mass_delta - post.n2o_unburnt_delta, 0 from the graph
            // engine, so every kg is pass-through there) and the decomposition heat it released.
            const NitrousBooking nb = nitrous_->commit();
            const real burnt = post.n2o_mass_delta - post.n2o_unburnt_delta;
            ledger_.n2o_mass += nb.n2o_kg;
            ledger_.n2o_unburnt += std::max(real(0.0), nb.n2o_kg - burnt);
            ledger_.n2o_energy_in += post.n2o_energy_in_delta;
            ledger_.n2o_purged += nb.purge_kg;
        }
    }
    if (clutch_) {
        const real v = network_.row_velocity(clutch_row_);
        const real lambda = network_.row_lambda(clutch_row_);
        ledger_.clutch_loss += Clutch::clutch_loss_delta(ctx.h, v, lambda);
    }
    if (gearbox_) {
        const int cur = gearbox_->current_position();
        const real eff = cur == 0 ? real(1.0) : gearbox_->efficiency_for_position(cur);
        if (auto_) {
            // Automatic gearbox plan A1: the own loss torque applied in pre_solve is booked here; the lagged mesh
            // torque is set from the shift rows below (no node-balance reconstruction).
            ledger_.gear_loss += -ctx.h * network_.omega(gearbox_in_) * gearbox_own_loss_tau_;
        } else if (cur == 0 || eff >= real(1.0)) {
            gearbox_lambda_mesh_prev_ = real(0.0);
            // Transmission plan T0 (sound only, never hashed): the transmitted torque of a lossless engaged gear is the
            // same node-balance reconstruction with no own loss term; neutral carries none.
            gearbox_mesh_load_nm_ = cur == 0 ? real(0.0)
                                             : std::abs(reconstruct_mesh_torque(gearbox_in_, real(1.0), real(0.0), network_, assembly_,
                                                                                gearbox_omega_old_in_, ctx.h));
        } else {
            const real omega_new_in = network_.omega(gearbox_in_);
            gearbox_lambda_mesh_prev_ =
                reconstruct_mesh_torque(gearbox_in_, real(1.0), gearbox_own_loss_tau_, network_, assembly_, gearbox_omega_old_in_, ctx.h);
            gearbox_mesh_load_nm_ = std::abs(gearbox_lambda_mesh_prev_);
            ledger_.gear_loss += -ctx.h * omega_new_in * gearbox_own_loss_tau_;
        }
        if (auto_) {
            // Automatic gearbox plan A1: the shift clutches' slip goes to clutch_loss; the mesh torque for the
            // lagged gear loss is what the two rows carried.
            const real la = network_.row_lambda(gearbox_synchro_row_), lb = network_.row_lambda(shift_row_b_);
            ledger_.clutch_loss += -ctx.h * network_.row_velocity(gearbox_synchro_row_) * la;
            ledger_.clutch_loss += -ctx.h * network_.row_velocity(shift_row_b_) * lb;
            gearbox_lambda_mesh_prev_ = cur == 0 ? real(0.0) : la + lb;
            gearbox_mesh_load_nm_ = std::abs(la + lb);
            if (converter_) {
                const real wp = network_.omega(converter_pump_);
                const real wt = network_.omega(gearbox_in_);
                const real v_new = network_.coupling_velocity(converter_coupling_);
                const real f_p = conv_tau_p_ + conv_c_ * conv_v_old_ - conv_c_ * v_new - conv_d_pump_ * (wp - conv_wp_old_);
                const real f_t = conv_tau_t_ - conv_c_ * conv_v_old_ + conv_c_ * v_new;
                ledger_.converter_loss += -ctx.h * (f_p * wp + f_t * wt);
                ledger_.converter_loss += -ctx.h * network_.row_velocity(lockup_row_) * network_.row_lambda(lockup_row_);
            }
        } else {
            const real v_syn = network_.row_velocity(gearbox_synchro_row_);
            const real lambda_syn = network_.row_lambda(gearbox_synchro_row_);
            ledger_.synchro_loss += -ctx.h * v_syn * lambda_syn;
        }
    }
    for (auto& c : components_) c->post_solve(ctx, network_, assembly_, ledger_);
}

void Powertrain::end_tick() {
    const PowertrainEnergy L = ledger_;
    const PowertrainEnergy& L0 = ledger_tick_start_;
    ledger_.balance_error = (L.rotor_ke - L0.rotor_ke) - (L.electric_out - L0.electric_out) - (L.engine_out - L0.engine_out) - (L.starter_in - L0.starter_in) -
                             (L.external_reset - L0.external_reset) + (L.wheel_out - L0.wheel_out) +
                             (L.tyre_loss - L0.tyre_loss) + (L.brake_loss - L0.brake_loss) + (L.gear_loss - L0.gear_loss) +
                             (L.clutch_loss - L0.clutch_loss) + (L.synchro_loss - L0.synchro_loss) +
                             (L.engine_friction_loss - L0.engine_friction_loss) + (L.engagement_loss - L0.engagement_loss) +
                             (L.integrator_loss - L0.integrator_loss) + (L.diff_loss - L0.diff_loss) +
                             (L.awd_clutch_loss - L0.awd_clutch_loss) + (L.pto_clutch_loss - L0.pto_clutch_loss) +
                             (L.external_absorbed - L0.external_absorbed);
    if (converter_) ledger_.balance_error += L.converter_loss - L0.converter_loss; // automatic gearbox plan A1
    if (!compliant_.empty()) {
        ledger_.balance_error += (L.shaft_pe - L0.shaft_pe) + (L.shaft_damping_loss - L0.shaft_damping_loss);
    }
    if (thermal_) {
        // Engine thermal plan T1: one BE step over this tick's accumulated heat, then copy the thermal sub-ledger
        // (cumulative, unhashed, outside balance_error) into PowertrainEnergy.
        const real rpm = std::abs(network_.omega(engine_crank_)) * (real(60.0) / (real(2.0) * real(3.14159265358979323846)));
        thermal_->step(rpm);
        const EngineThermalLedger& tl = thermal_->ledger();
        ledger_.thermal_wall_in = tl.wall_in;
        ledger_.thermal_friction_in = tl.friction_in;
        ledger_.thermal_radiator_out = tl.radiator_out;
        ledger_.thermal_block_air_out = tl.block_air_out;
        ledger_.thermal_oil_air_out = tl.oil_air_out;
        ledger_.thermal_heater_out = tl.heater_out;
        ledger_.thermal_stored = tl.stored;
        ledger_.thermal_balance_error = tl.balance_error;
    }
}

void Powertrain::request_thermal_state(const EngineThermalState& st) {
    if (!thermal_) throw std::runtime_error("set_engine_thermal_state: this vehicle has no cooling_system");
    if (!std::isfinite(st.coolant_c) || !std::isfinite(st.oil_c)) {
        throw std::runtime_error("set_engine_thermal_state: coolant_c and oil_c must be finite");
    }
    thermal_->request_state(st);
}

void Powertrain::request_cooling_capacity(const CoolingCapacity& c) {
    if (!thermal_) throw std::runtime_error("set_engine_cooling_capacity: this vehicle has no cooling_system");
    const auto ok = [](real v) { return std::isfinite(v) && v >= 0.0 && v <= 1e3; };
    if (!ok(c.radiator_ua_scale) || !ok(c.fan_flow_scale)) {
        throw std::runtime_error("set_engine_cooling_capacity: factors must be finite and in [0, 1e3]");
    }
    thermal_->request_cooling_capacity(c);
}

void Powertrain::request_engine_wear(const EngineWear& w) {
    if (!thermal_) throw std::runtime_error("set_engine_wear: this vehicle has no cooling_system (the friction scale lives in the thermal model)");
    if (!(std::isfinite(w.friction_scale) && w.friction_scale >= 0.0 && w.friction_scale <= 1e3)) {
        throw std::runtime_error("set_engine_wear: friction_scale must be finite and in [0, 1e3]");
    }
    thermal_->request_engine_wear(w);
}

void Powertrain::reset(const vehicle::VehicleResetOptions& opts, const std::vector<real>& hub_speeds) {
    for (RegenMap& m : regen_maps_) m.motor->reset_blend();
    blend_regen_nm_ = blend_pad_nm_ = real(0.0);
    b_selected_ = b_hold_active_ = b_feed_on_ = false;
    b_hold_level_ = real(0.0);
    std::fill(b_fraction_.begin(), b_fraction_.end(), real(1.0));
    b_counters_init_ = false;
    b_prev_request_ = kGearRequestNone;
    if (brakes_ != nullptr) brakes_->set_hold_level(real(0.0));
    hybrid_connect_s_ = real(0.0);
    hybrid_charge_plan_s_=hybrid_charge_rpm_=hybrid_charge_power_w_=hybrid_charge_integral_=0;
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
        // Automatic/DCT/CVT ratios use friction rows in one gearbox topology state.
        // Manual position indices (N follows reverse) would select an invalid state or a diff-lock bit here.
        const int gear_state = auto_ ? 0 : gearbox_->state_index_for_position(gear);
        network_.set_topology_state(gear_state + gear_state_count_ * lock_bits);
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
    const real idle_omega = engine_ ? rpm_to_rad_per_s(engine_->idle_rpm()) : real(0.0);
    const real gearbox_out_omega_now = gearbox_ ? network_.omega(gearbox_out_) : real(0.0);
    // A held CVT ratio (powertrain dyno) is the variator's, not a virtual step's.
    const real driveline_ratio = (gearbox_ && gear > 0 && auto_ && auto_->dyno_hold() && auto_->desc().cvt) ? auto_->ratio(gear)
                                                                                                             : (gearbox_ && gear != 0 ? gearbox_->ratio_for_position(gear) : real(0.0));
    const real driveline_in_omega = (gearbox_ && gear != 0) ? driveline_ratio * gearbox_out_omega_now : real(0.0);
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

    // I4 C11 (plan section 3.5): pto.out - "stuck/e==1: crank/ratio; else
    // leave it" (spec's own wording; pto.in is used here rather than
    // engine_crank_ directly - they are the SAME node whenever the pto sits
    // on the crank shaft as the spec intends, and reading pto.in's own
    // omega keeps this correct without assuming an engine exists at all).
    // Every other node this function touches is set unconditionally;
    // pto.out is deliberately left untouched in the "else" branch, so a
    // disengaged pto's out node simply retains whatever omega it already
    // had across the reset (its own disengaged stub brake decays it on the
    // very next substep regardless).
    if (pto_ && (network_.row_stuck(pto_row_) || pto_->engagement() >= real(1.0))) {
        network_.set_omega(pto_->out_node(), network_.omega(pto_->in_node()) / pto_->current_ratio());
    }

    // 6. Clutch.out (== gearbox.in, shaft-merged): in gear, ALWAYS the
    //    driveline's own value (the gear ratio row is rigid regardless of
    //    clutch engagement - the clutch only sits between engine crank and
    //    this node); in Neutral, matches engine crank if clutch_locked
    //    (mirrors the Launch law's own "In gearbox N: e_tgt = 1" rule),
    //    else 0 (freewheeling at rest).
    if (gearbox_ && auto_) {
        // Automatic gearbox plan A1: no clutch. An Automatic's turbine takes the driveline's speed in gear and
        // the engine's in N (the converter is then coupled); a DualClutch's input IS the crank, set above.
        if (converter_ && gearbox_in_ != engine_crank_) {
            network_.set_omega(gearbox_in_, gear != 0 ? driveline_in_omega : engine_omega_target);
        }
        auto_->reset(gear, opts.clutch_locked);
        last_auto_out_ = {};
    } else if (gearbox_) {
        const real clutch_out_target =
            gear != 0 ? driveline_in_omega : (opts.clutch_locked ? engine_omega_target : real(0.0));
        network_.set_omega(clutch_out_, clutch_out_target);
        // C13: with a compliant clutch.out-gearbox.in shaft they are two
        // nodes - both take the same value (the shaft starts relaxed).
        if (gearbox_in_ != clutch_out_) network_.set_omega(gearbox_in_, clutch_out_target);
    }
    if (tcu_) tcu_->reset(opts.clutch_locked);
    if (launch_) launch_->reset();
    tcu_prev_request_ = {};

    // 6b. PHYS-033 P4 I4 C13: compliant shafts not already set at both ends
    //     (reset_mode, built at construction), then zero twist and force the
    //     next substep to backward Euler (a reset is a mode change).
    real pe_before = real(0.0);
    if (!compliant_.empty()) {
        for (CompliantShaft& cs : compliant_) {
            if (cs.reset_mode == 1) network_.set_omega(cs.b, network_.omega(cs.a));
            else if (cs.reset_mode == 2) network_.set_omega(cs.a, network_.omega(cs.b));
            pe_before += real(0.5) * cs.k * cs.theta * cs.theta;
            cs.theta = real(0.0);
            cs.tau_last = real(0.0);
        }
        be_next_ = true;
        topo_switched_this_substep_ = false;
    }


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
    if (!compliant_.empty()) {
        // C13: the dropped twist energy is part of the same signed teleport
        // (ledger_.shaft_pe itself, like rotor_ke, is only rewritten by the
        // next solve()).
        ledger_.external_reset += (ke_after - ke_before) - pe_before;
    } else {
        ledger_.external_reset += ke_after - ke_before;
    }
}

real Powertrain::gearbox_output_ratio_magnitude() const { return speedo_ratio_; }

real Powertrain::clutch_slip() const {
    if (!clutch_) return real(0.0);
    return network_.omega(clutch_in_) - network_.omega(clutch_out_);
}

std::uint64_t Powertrain::fold_state_hash(std::uint64_t seed) const {
    std::uint64_t h = network_.fold_state_hash(seed);
    for(const auto& b:batteries_)h=b->fold_state_hash(h);
    if(hybrid_charge_rpm_>0){for(real v:{hybrid_charge_plan_s_,hybrid_charge_rpm_,hybrid_charge_power_w_,hybrid_charge_integral_}){const auto* b=reinterpret_cast<const unsigned char*>(&v);for(std::size_t i=0;i<sizeof(v);++i){h^=b[i];h*=1099511628211ULL;}}}
    if(hybrid_){for(real v:{real(hybrid_charge_latched_),hybrid_selector_,hybrid_idle_s_,hybrid_crank_s_}){const auto* bytes=reinterpret_cast<const unsigned char*>(&v);for(std::size_t i=0;i<sizeof(v);++i){h^=bytes[i];h*=1099511628211ULL;}}
        if(hybrid_connect_s_>0){const real v=hybrid_connect_s_;const auto* bytes=reinterpret_cast<const unsigned char*>(&v);for(std::size_t i=0;i<sizeof(v);++i){h^=bytes[i];h*=1099511628211ULL;}}}
    if(!electric_motors_.empty()) {
        const auto* b=reinterpret_cast<const unsigned char*>(&electric_direction_);
        for(std::size_t i=0;i<sizeof(electric_direction_);++i){h^=b[i];h*=1099511628211ULL;}
    }
    // B mode / hybrid Electric mode: hashed only while non-default, so every other car (and a B-capable car in D) hashes as before.
    if(b_selected_||b_hold_active_||b_hold_level_>0||hybrid_electric_lost_){
        const auto fold=[&](real v){const auto* bytes=reinterpret_cast<const unsigned char*>(&v);for(std::size_t i=0;i<sizeof(v);++i){h^=bytes[i];h*=1099511628211ULL;}};
        fold(real(0x42));fold(real(b_selected_));fold(real(b_hold_active_));fold(b_hold_level_);fold(real(hybrid_electric_lost_));
        for(real f:b_fraction_)fold(f);
    }
    for (const auto& c : components_) h = c->fold_state_hash(h);
    if (engine_) h = engine_->fold_state_hash(h);
    if (gearbox_) h = gearbox_->fold_state_hash(h);
    if (tcu_) h = tcu_->fold_state_hash(h);
    if (launch_) h = launch_->fold_state_hash(h); // engine garage P8: hashed only when the component exists
    if (auto_) h = auto_->fold_state_hash(h); // automatic gearbox plan A1: only for an Automatic/DualClutch
    if (gearbox_) {
        const auto* bytes = reinterpret_cast<const unsigned char*>(&gearbox_lambda_mesh_prev_);
        for (std::size_t i = 0; i < sizeof(gearbox_lambda_mesh_prev_); ++i) {
            h ^= bytes[i];
            h *= 1099511628211ULL;
        }
    }
    // PHYS-033 P4 I4 C13: each compliant shaft's twist, then the pending
    // backward-Euler flag - only for a vehicle that has compliant shafts.
    if (!compliant_.empty()) {
        for (const CompliantShaft& cs : compliant_) {
            const auto* bytes = reinterpret_cast<const unsigned char*>(&cs.theta);
            for (std::size_t i = 0; i < sizeof(cs.theta); ++i) {
                h ^= bytes[i];
                h *= 1099511628211ULL;
            }
        }
        h ^= static_cast<unsigned char>(be_next_ ? 1 : 0);
        h *= 1099511628211ULL;
    }
    // PHYS-033 P4 I7a C6: the tank's fuel mass - only for a vehicle that has
    // a tank (every pre-C6 vehicle hashes exactly as before). The cumulative
    // fuel ledger counters stay unhashed.
    if (fuel_tank_) h = fuel_tank_->fold_state_hash(h);
    if (nitrous_) h = nitrous_->fold_state_hash(h); // N2O plan N1: appended after every existing fold, only with a kit
    if (thermal_) h = thermal_->fold_state_hash(h); // engine thermal plan T1: T_e, T_o, T_wax, T_play, fan_on; only with a cooling_system
    return h;
}

void Powertrain::request_n2o_bottle_mass(real kg) {
    if (!nitrous_) throw std::runtime_error("set_n2o_bottle_mass: this vehicle has no n2o_bottle");
    if (std::isnan(kg)) throw std::runtime_error("set_n2o_bottle_mass: the requested mass is NaN");
    n2o_bottle_set_value_ = kg;
    n2o_bottle_set_pending_ = true;
}

void Powertrain::request_fuel_tank_mass(real kg) {
    if (!fuel_tank_) throw std::runtime_error("set_fuel_tank_mass: this vehicle has no fuel_tank");
    if (std::isnan(kg)) throw std::runtime_error("set_fuel_tank_mass: the requested mass is NaN");
    fuel_tank_set_value_ = kg;
    fuel_tank_set_pending_ = true;
}

// PHYS-033 P4 I7c C16b -----------------------------------------------------

PreparedTurboEnvelope Powertrain::prepare_turbo_envelope(std::string_view component_id,const TurboPairDesc& desc) const {
    if(!sim_engine_ptr_ || component_id!=engine_component_id_)
        throw std::runtime_error("prepare_turbo_envelope: unknown simulated engine");
    return sim_engine_ptr_->prepare_turbo_envelope(desc);
}

void Powertrain::replace_turbo_configuration(std::string_view component_id,const TurboPairDesc& desc,const PreparedTurboEnvelope* prepared) {
    if(!sim_engine_ptr_ || component_id!=engine_component_id_)
        throw std::runtime_error("replace_turbo_configuration: unknown simulated engine");
    sim_engine_ptr_->replace_turbo_configuration(desc,prepared);
    simulated_engine_desc_->turbo_pair=desc;
    refresh_hybrid_ice_power(); // the full-load envelope was rebuilt for the new hardware
    // I7d D5: channels are declared at build time only; a swapped-in anti_lag block naming that channel
    // keeps reading it, any other falls back to its default_enabled (Sensors::anti_lag_channel NaN).
    anti_lag_channel_live_ = anti_lag_fitted_ && anti_lag_channel_slot_ >= 0 && desc.anti_lag && desc.anti_lag->channel == anti_lag_channel_name_;
}

void Powertrain::request_replace_engine_maps(std::string_view component_id, EngineMapsPtr new_engine_maps) {
    if (!engine_ || component_id != engine_component_id_) {
        throw std::runtime_error("replace_engine_maps: unknown component id '" + std::string(component_id) + "'");
    }
    if (sim_engine_ptr_ == nullptr || !simulated_engine_desc_.has_value()) {
        throw std::runtime_error("replace_engine_maps: component '" + std::string(component_id) + "' is not a simulated engine");
    }
    if (!new_engine_maps) throw std::runtime_error("replace_engine_maps: null maps");
    hybrid_ice_dirty_ = hybrid_.has_value();
    // Only .maps ever differs from the desc this engine was originally built
    // from (architecture/geometry/ECU/limiter/idle/starter/etc. are fixed
    // for this engine's lifetime - a hot swap replaces MAPS, not the
    // engine); SimulatedEngine::build_maps needs the full desc regardless
    // (it reads geometry/ECU/limiter fields too, not just the map curves).
    SimulatedEngineDesc updated = *simulated_engine_desc_;
    set_engine_maps(updated, *new_engine_maps, "replace_engine_maps"); // P9: every cam profile's set together
    select_knock_fuel(updated.maps, burnt_fuel_index_); // P7: the burnt fuel's knock table (no-op without per-fuel tables)
    for (auto& pm : updated.cam_profile_maps) select_knock_fuel(pm, burnt_fuel_index_);
    for (auto& pm : updated.lift_node_maps) select_knock_fuel(pm, burnt_fuel_index_); // P10: every lift node's set
    // ALLOCATES (build_maps constructs a fresh SimulatedEngineMaps with its
    // own derived tables) - safe here: this runs on the caller's thread,
    // between World::step() calls, never inside a job (see this method's
    // header-comment contract).
    sim_engine_ptr_->request_replace_maps(SimulatedEngine::build_maps(updated));
}

void Powertrain::apply_pending_engine_maps_swap() {
    if (sim_engine_ptr_ != nullptr) sim_engine_ptr_->apply_pending_maps_swap(); // returned old maps drop here
    if (hybrid_ice_dirty_) { hybrid_ice_dirty_ = false; refresh_hybrid_ice_power(); }
}

std::uint64_t Powertrain::simulated_engine_maps_content_hash() const {
    return sim_engine_ptr_ != nullptr ? sim_engine_ptr_->maps().content_hash : 0;
}

std::uint64_t Powertrain::simulated_engine_steady_rebuild_count() const {
    return sim_engine_ptr_ != nullptr ? sim_engine_ptr_->steady_rebuild_count() : 0;
}

real Powertrain::transmission_input_power_w() const {
    if (!engine_) return real(0.0);
    real reaction = clutch_torque();
    if (auto_) {
        if (converter_) {
            // Same solved pump reaction as the converter's energy ledger,
            // plus the mechanical lock-up path in parallel with the fluid.
            const real wp = network_.omega(converter_pump_);
            const real slip = network_.coupling_velocity(converter_coupling_);
            reaction = conv_tau_p_ + conv_c_ * conv_v_old_ - conv_c_ * slip
                       - conv_d_pump_ * (wp - conv_wp_old_) + network_.row_lambda(lockup_row_);
        } else {
            reaction = network_.row_lambda(gearbox_synchro_row_) + network_.row_lambda(shift_row_b_);
        }
    }
    return -reaction * engine_omega();
}

void Powertrain::fill_gearbox_sound(DrivelineSoundState& out) const {
    out.gear_type = static_cast<std::uint8_t>(gearbox_sound_.gear_type);
    out.mesh_frequency_hz[0] = real(0.0);
    out.mesh_frequency_hz[1] = real(0.0);
    out.mesh_load_nm = real(0.0);
    if (!gearbox_) return;
    constexpr real kTwoPiRad = real(2.0) * real(3.14159265358979323846);
    const real w_in = std::abs(network_.omega(gearbox_in_));
    const GearTeeth& pair = gearbox_sound_.input_pair_teeth;
    // Input pair (constant mesh, turns whenever the input shaft turns): teeth of the input-shaft gear, else the fallback order.
    if (pair.drive > 0) out.mesh_frequency_hz[0] = w_in * static_cast<real>(pair.drive) / kTwoPiRad;
    else if (gearbox_sound_.mesh_order > real(0.0)) out.mesh_frequency_hz[0] = w_in * gearbox_sound_.mesh_order / kTwoPiRad;
    const int pos = gearbox_->current_position();
    if (pos == 0) return; // neutral: no gear mesh carries a load, nothing meshes at the gear's frequency
    // Active gear: the shaft that drives it is the layshaft when an input pair exists (w_in * z_a / z_b), else the input shaft.
    const std::vector<GearTeeth>& list = pos > 0 ? gearbox_sound_.forward_teeth : gearbox_sound_.reverse_teeth;
    const std::size_t idx = static_cast<std::size_t>(pos > 0 ? pos - 1 : -pos - 1);
    if (idx < list.size() && list[idx].drive > 0) {
        real w_drive_shaft = w_in;
        if (pair.drive > 0 && pair.driven > 0) w_drive_shaft = w_in * static_cast<real>(pair.drive) / static_cast<real>(pair.driven);
        out.mesh_frequency_hz[1] = w_drive_shaft * static_cast<real>(list[idx].drive) / kTwoPiRad;
    } else if (gearbox_sound_.mesh_order > real(0.0)) {
        out.mesh_frequency_hz[1] = w_in * gearbox_sound_.mesh_order / kTwoPiRad;
    }
    out.mesh_load_nm = gearbox_mesh_load_nm_;
}

} // namespace ps::drivetrain

