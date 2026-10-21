// ps/drivetrain/powertrain_dyno.h - the whole-powertrain chassis dyno (powertrain dyno, 2026-10-19).
#include "ps/drivetrain/powertrain_dyno.h"

#include "electric_drive.h"
#include "engine_params_util.h"
#include "engine_simulated.h"
#include "powertrain.h"
#include "ps/drivetrain/hybrid_split.h"
#include "ps/vehicle/vehicle_reset.h"

#include <algorithm>
#include <cmath>
#include <exception>
#include <stdexcept>
#include <string>

namespace ps::drivetrain {

namespace {

constexpr real kTickHz = 240.0;
constexpr int kSubstepsPerTick = 4; // the car's 960 Hz substep
constexpr real kH = real(1.0) / (kTickHz * kSubstepsPerTick);
constexpr real kTwoPi = 6.283185307179586;
// The speed hold on every hub: torque = -D (omega_new - target) [N*m*s/rad]. A 5 kN*m hub torque leaves the hub 5e-3 rad/s
// ahead of its target; D is ~500x a hub's J/h, so the reduced matrix stays well conditioned. [D]
constexpr real kHoldDamping = 1.0e6;
constexpr real kPowerFloorW = 100.0; // relative settle test floor [AS]

bool cancelled(const std::atomic<bool>* cancel) { return cancel != nullptr && cancel->load(std::memory_order_relaxed); }

void validate(const PowertrainDynoSpec& s) {
    if (s.points.empty()) throw std::invalid_argument("run_powertrain_dyno: points is empty");
    for (real p : s.points)
        if (!std::isfinite(p) || !(p > 0)) throw std::invalid_argument("run_powertrain_dyno: every point must be finite and > 0");
    if (!std::isfinite(s.pedal) || s.pedal < 0 || s.pedal > 1) throw std::invalid_argument("run_powertrain_dyno: pedal must be in [0, 1]");
    if (!std::isnan(s.battery_soc) && !(s.battery_soc >= 0 && s.battery_soc <= 1))
        throw std::invalid_argument("run_powertrain_dyno: battery_soc must be in [0, 1] (or NaN = the packs' initial_soc)");
    if (!std::isnan(s.hydrogen_fraction) && !(s.hydrogen_fraction >= 0 && s.hydrogen_fraction <= 1))
        throw std::invalid_argument("run_powertrain_dyno: hydrogen_fraction must be in [0, 1] (or NaN = the initial fill)");
    if (s.drive_mode < 0 || s.drive_mode > 3) throw std::invalid_argument("run_powertrain_dyno: drive_mode must be 0..3");
    if (!(s.settle_window_s > 0) || !(s.max_settle_s >= s.settle_window_s) || !(s.min_settle_s >= 0) || !(s.settle_power_rel_per_s >= 0) ||
        !(s.settle_boost_bar_per_s >= 0))
        throw std::invalid_argument("run_powertrain_dyno: invalid settle parameters");
}

// The descriptor with `maps` applied to its engine (run_dyno_sweep's rule).
vehicle::VehicleDesc resolve(const vehicle::VehicleDesc& in, const EngineMapsPtr& maps) {
    vehicle::VehicleDesc d = in;
    if (maps) {
        bool done = false;
        for (ComponentDesc& c : d.powertrain.components) {
            if (std::holds_alternative<SimulatedEngineDesc>(c.params) || std::holds_alternative<TorqueMapEngineDesc>(c.params)) {
                c.params = with_engine_maps(c.params, maps, "run_powertrain_dyno");
                done = true;
                break;
            }
        }
        if (!done) throw std::invalid_argument("run_powertrain_dyno: maps were given but the vehicle has no engine");
    }
    return d;
}

real held_soc(const BatteryDesc& b, real soc) { return std::isnan(soc) ? b.initial_soc : soc; }

// The electric potential the hybrid supervisor computes (powertrain.cpp), from the descriptors at the spec's state.
real hybrid_available_w(const PowertrainDesc& pt, const PowertrainDynoSpec& spec) {
    real pack = 0, stack = 0, motors = 0;
    for (const ComponentDesc& c : pt.components) {
        if (const auto* b = std::get_if<BatteryDesc>(&c.params)) { if (held_soc(*b, spec.battery_soc) > 0) pack += b->max_discharge_w; }
        else if (const auto* f = std::get_if<FuelCellDesc>(&c.params)) {
            const real kg = std::isnan(spec.hydrogen_fraction) ? f->initial_hydrogen_kg : spec.hydrogen_fraction * f->hydrogen_capacity_kg;
            if (kg > 0) stack += f->max_power_w;
        } else if (const auto* m = std::get_if<ElectricMotorDesc>(&c.params)) { if (!m->crank_assist) motors += m->max_power_w; }
    }
    return std::min(motors, pack + stack);
}

real energy_weighted_soc(const PowertrainDesc& pt, real soc) {
    real j = 0, cap = 0;
    for (const ComponentDesc& c : pt.components)
        if (const auto* b = std::get_if<BatteryDesc>(&c.params)) {
            j += b->capacity_kwh * 3600000 * held_soc(*b, soc);
            cap += b->capacity_kwh * 3600000;
        }
    return cap > 0 ? j / cap : 0;
}

struct Prepared {
    vehicle::VehicleDesc desc;
    std::vector<real> channels; // declare_control_channels order, spec overrides applied
    int gear = 0;               // the gear held (0 without a gearbox)
    int reset_gear = 0;         // the position the Powertrain is reset to / asked for: gear, but D (1) for a CVT's forward steps
    real cvt_ratio = 0.0;       // CVT: the held variator ratio (0 = not a CVT / reverse)
    bool has_gearbox = false, has_engine = false, has_hybrid = false;
    real kinematic = 1.0;       // axis omega (rad/s; RoadSpeed: m/s) per 1 m/s of road speed
    real driven_radius = 0.0;
    vehicle::VehicleResetOptions reset;
    HybridControlDesc hybrid{};
};

Powertrain build(const Prepared& p) {
    return Powertrain(p.desc.powertrain, p.desc.brake_system, p.desc.wheels, p.desc.controllers,
                      p.desc.name.empty() ? std::string("powertrain dyno") : p.desc.name);
}

// The gear hold (manual / sequential: the request; automatic / DCT / CVT: AutoTransmission's dyno hold).
void hold_gear(Powertrain& pt, const Prepared& p) {
    pt.set_dyno_gear_hold(p.has_gearbox ? p.reset_gear : Powertrain::kNoDynoGearHold, p.cvt_ratio);
}

std::vector<real> hub_targets(const Prepared& p, real v) {
    std::vector<real> out(p.desc.wheels.size(), real(0));
    for (std::size_t i = 0; i < out.size(); ++i) {
        const real r = p.desc.wheels[i].wheel_radius;
        out[i] = r > 0 ? v / r : real(0);
    }
    return out;
}

Prepared prepare(const vehicle::VehicleDesc& vehicle, const EngineMapsPtr& maps, const PowertrainDynoSpec& spec) {
    Prepared p;
    p.desc = resolve(vehicle, maps);
    const std::vector<ChannelDecl> decls = Powertrain::declare_control_channels(p.desc.powertrain);
    p.channels.resize(decls.size());
    for (std::size_t i = 0; i < decls.size(); ++i) {
        p.channels[i] = decls[i].default_value;
        if (decls[i].name == "assist.drive_mode") p.channels[i] = static_cast<real>(spec.drive_mode);
        for (const auto& [name, value] : spec.channels)
            if (decls[i].name == name) p.channels[i] = value;
    }
    for (const ComponentDesc& c : p.desc.powertrain.components)
        if (const auto* h = std::get_if<HybridControlDesc>(&c.params)) {
            p.has_hybrid = true;
            p.hybrid = *h;
        }

    // The kinematic probe: every hub rolling at 1 m/s in the held gear; the axis shaft's speed is the ratio.
    Powertrain probe = build(p);
    p.has_gearbox = probe.has_gearbox();
    p.has_engine = probe.has_engine();
    p.driven_radius = probe.driven_wheel_mean_radius();
    if (p.has_gearbox) {
        const int fwd = static_cast<int>(probe.gearbox().desc().forward_ratios.size());
        const int rev = static_cast<int>(probe.gearbox().desc().reverse_ratios.size());
        if (spec.gear > fwd || spec.gear < -rev) throw std::invalid_argument("run_powertrain_dyno: gear " + std::to_string(spec.gear) + " does not exist");
        p.gear = spec.gear;
        p.reset_gear = spec.gear;
        const bool cvt = probe.has_auto() && probe.auto_trans().desc().cvt;
        if (spec.cvt_ratio != 0 && !(cvt && spec.gear > 0))
            throw std::invalid_argument("run_powertrain_dyno: cvt_ratio needs a CVT gearbox in a forward gear");
        if (cvt && spec.gear > 0) {
            // A CVT has one forward gear (D); `gear` k picks its virtual step k's ratio unless cvt_ratio is given.
            const CvtDesc& cd = probe.auto_trans().desc().cvt_desc;
            p.reset_gear = 1;
            p.cvt_ratio = spec.cvt_ratio != 0 ? spec.cvt_ratio : probe.gearbox().desc().forward_ratios[static_cast<std::size_t>(spec.gear - 1)];
            if (!std::isfinite(p.cvt_ratio) || p.cvt_ratio < cd.ratio_high || p.cvt_ratio > cd.ratio_low)
                throw std::invalid_argument("run_powertrain_dyno: the CVT ratio " + std::to_string(p.cvt_ratio) + " is outside the variator's range");
        }
    }
    if (!p.has_gearbox && (spec.axis == PowertrainDynoAxis::EngineRpm || spec.axis == PowertrainDynoAxis::GearboxOutputRpm))
        throw std::invalid_argument("run_powertrain_dyno: the EngineRpm and GearboxOutputRpm axes need a gearbox");
    if (spec.axis == PowertrainDynoAxis::EngineRpm && p.gear == 0)
        throw std::invalid_argument("run_powertrain_dyno: the EngineRpm axis needs a gear (not neutral)");
    if (spec.axis == PowertrainDynoAxis::MotorRpm &&
        (spec.motor_index < 0 || static_cast<std::size_t>(spec.motor_index) >= probe.electric_motors().size()))
        throw std::invalid_argument("run_powertrain_dyno: motor_index " + std::to_string(spec.motor_index) + " is out of range");
    {
        vehicle::VehicleResetOptions ro;
        ro.gear = p.has_gearbox ? p.reset_gear : vehicle::VehicleResetOptions::kKeepGear;
        // A DualClutch's gear-set input IS the crank, and an Off engine rests at 0: the probe's engine runs there.
        ro.engine_state = probe.has_auto() && probe.has_engine() ? EngineState::Running : EngineState::Off;
        ro.clutch_locked = true;
        if (probe.has_auto()) hold_gear(probe, p);
        // A DualClutch's input is the crank, which a reset floors at idle: probe at a road speed whose input speed is far above
        // idle in every gear (a manual / AT input shaft is not floored, so those keep 1 m/s, bit-identical to before).
        const real probe_v = probe.has_auto() ? real(100.0) : real(1.0);
        probe.reset(ro, hub_targets(p, probe_v));
        real k = 1.0;
        switch (spec.axis) {
        case PowertrainDynoAxis::EngineRpm: k = probe.gearbox_in_omega(); break;
        case PowertrainDynoAxis::GearboxOutputRpm: k = probe.gearbox_out_omega(); break;
        case PowertrainDynoAxis::MotorRpm:
            k = probe.node_omega(probe.electric_motors()[static_cast<std::size_t>(spec.motor_index)]->shaft_node());
            break;
        case PowertrainDynoAxis::RoadSpeed: k = probe_v; break;
        }
        if (!(std::abs(k) > real(1e-9)))
            throw std::invalid_argument("run_powertrain_dyno: the axis shaft does not turn with the wheels in this gear");
        p.kinematic = std::abs(k) / probe_v;
    }

    // Initial state: the engine running iff the car would run it (a hybrid: the split's start request; an ICE: always), in
    // gear iff the engine is coupled (a hybrid in Eco starts in neutral, as its supervisor would put it).
    p.reset.clutch_locked = true;
    p.reset.engine_state = p.has_engine ? EngineState::Running : EngineState::Off;
    p.reset.gear = p.has_gearbox ? p.reset_gear : vehicle::VehicleResetOptions::kKeepGear;
    if (p.has_hybrid) {
        const HybridSplit s = hybrid_full_pedal_split(p.hybrid, hybrid_available_w(p.desc.powertrain, spec), spec.pedal, spec.drive_mode,
                                                      energy_weighted_soc(p.desc.powertrain, spec.battery_soc));
        p.reset.engine_state = s.engine_wanted ? EngineState::Running : EngineState::Off;
        if (!s.engine_coupled) p.reset.gear = 0;
    }
    return p;
}

// Window accumulators (means over one settle window).
struct Window {
    long n = 0;
    real system_power = 0, wheel_torque = 0, engine_torque = 0, engine_power = 0, electric_power = 0, battery_power = 0,
         fuel_cell_power = 0, boost = 0, engine_command = 0, engine_rpm = 0, output_rpm = 0;
    std::vector<real> m_rpm, m_torque, m_power, m_electrical;
    void clear(std::size_t motors) {
        n = 0;
        system_power = wheel_torque = engine_torque = engine_power = electric_power = battery_power = fuel_cell_power = boost = engine_command =
            engine_rpm = output_rpm = 0;
        m_rpm.assign(motors, 0);
        m_torque.assign(motors, 0);
        m_power.assign(motors, 0);
        m_electrical.assign(motors, 0);
    }
    [[nodiscard]] real mean(real s) const { return n > 0 ? s / static_cast<real>(n) : real(0); }
};

PowertrainDynoPoint run_point(const Prepared& p, const PowertrainDynoSpec& spec, real axis_value) {
    Powertrain pt = build(p);
    hold_gear(pt, p);
    pt.dyno_hold_energy_state(spec.battery_soc, spec.hydrogen_fraction);

    const bool rpm_axis = spec.axis != PowertrainDynoAxis::RoadSpeed;
    const real axis_omega = rpm_axis ? axis_value * kTwoPi / real(60.0) : axis_value;
    real v = axis_omega / p.kinematic;
    if (p.gear < 0) v = -v;
    const std::vector<real> target = hub_targets(p, v);
    pt.reset(p.reset, target);

    std::vector<real> channels = p.channels;
    ComponentSubstepCtx ctx;
    ctx.h = kH;
    ctx.throttle = spec.pedal;
    ctx.brake = 0;
    ctx.gear_request = p.has_gearbox ? static_cast<real>(p.reset_gear) : real(1); // EV: D
    if (p.has_hybrid) ctx.gear_request = p.gear < 0 ? real(-1) : real(1);         // the driver's D / R lever
    ctx.ignition = 1;
    ctx.starter = 0;
    ctx.ambient = spec.ambient;
    ctx.channels = channels.empty() ? nullptr : channels.data();
    ctx.channel_count = static_cast<int>(channels.size());

    const std::size_t nm = pt.electric_motors().size();
    const std::size_t nw = target.size();
    std::vector<real> w_old(nw, 0);

    const long window_ticks = std::max(1L, static_cast<long>(std::floor(spec.settle_window_s * kTickHz + real(0.5))));
    const real window_s = static_cast<real>(window_ticks) / kTickHz;
    const long max_windows = std::max(1L, static_cast<long>(std::floor(spec.max_settle_s / window_s + real(1e-9))));
    const long min_windows = std::max(3L, static_cast<long>(std::ceil(spec.min_settle_s / window_s - real(1e-9))));
    const bool turbo = p.has_engine && pt.has_engine() && pt.simulated_engine() != nullptr && pt.simulated_engine()->turbo() != nullptr;

    Window w;
    real prev_power = std::numeric_limits<real>::quiet_NaN(), prev_boost = prev_power;
    int passing = 0;
    bool converged = false;
    long k = 0;
    for (; k < max_windows; ++k) {
        w.clear(nm);
        for (long t = 0; t < window_ticks; ++t) {
            pt.dyno_hold_energy_state(spec.battery_soc, spec.hydrogen_fraction);
            pt.begin_tick();
            for (int s = 0; s < kSubstepsPerTick; ++s) {
                ctx.first_substep_of_tick = (s == 0);
                pt.begin_substep();
                pt.pre_solve_components(ctx);
                for (std::size_t i = 0; i < nw; ++i) {
                    w_old[i] = pt.hub_omega(i);
                    pt.add_hub_terms(i, -kHoldDamping * (w_old[i] - target[i]), kHoldDamping);
                }
                const real crank_old = pt.engine_omega();
                pt.solve(kH);
                pt.post_solve_components(ctx);
                real power = 0, torque = 0;
                for (std::size_t i = 0; i < nw; ++i) {
                    const real w_new = pt.hub_omega(i);
                    const real t_sys = kHoldDamping * (w_new - target[i]); // what the powertrain puts into the hub
                    torque += t_sys;
                    power += t_sys * w_new;
                }
                w.system_power += power;
                w.wheel_torque += torque;
                if (pt.has_engine()) {
                    const auto& r = pt.last_engine_result();
                    const real crank = pt.engine_omega();
                    const real net = r.tau0 - r.d * (crank - crank_old) + pt.engine_friction_torque();
                    w.engine_torque += net;
                    w.engine_power += net * crank;
                    w.boost += r.boost_bar;
                    w.engine_command += pt.last_throttle_command().command;
                    w.engine_rpm += std::abs(crank) * real(60.0) / kTwoPi;
                }
                if (p.has_gearbox) w.output_rpm += std::abs(pt.gearbox_out_omega()) * real(60.0) / kTwoPi;
                for (std::size_t m = 0; m < nm; ++m) {
                    const ElectricMotor& em = *pt.electric_motors()[m];
                    w.m_rpm[m] += em.rpm;
                    w.m_torque[m] += em.torque_nm;
                    w.m_power[m] += em.power_w;
                    w.m_electrical[m] += em.battery_power_w;
                }
                w.electric_power += pt.electric_power_w();
                w.battery_power += pt.battery_power_w();
                w.fuel_cell_power += pt.fuel_cell_power_w();
                ++w.n;
            }
            pt.end_tick();
        }
        const real power = w.mean(w.system_power);
        const real boost = w.mean(w.boost);
        if (k > 0) {
            const bool power_ok = std::abs(power - prev_power) / window_s < spec.settle_power_rel_per_s * std::max(std::abs(power), kPowerFloorW);
            const bool boost_ok = !turbo || std::abs(boost - prev_boost) / window_s < spec.settle_boost_bar_per_s;
            passing = (power_ok && boost_ok) ? passing + 1 : 0;
            if (passing >= 2 && k + 1 >= min_windows) {
                converged = true;
                ++k;
                break;
            }
        }
        prev_power = power;
        prev_boost = boost;
    }
    if (k > max_windows) k = max_windows;

    PowertrainDynoPoint out;
    out.axis_value = axis_value;
    out.road_speed_mps = v;
    out.converged = converged;
    out.settle_s = static_cast<real>(k) * window_s;
    out.system_power_w = w.mean(w.system_power);
    out.system_wheel_torque_nm = w.mean(w.wheel_torque);
    const real ref_omega = rpm_axis ? axis_omega : (p.driven_radius > 0 ? std::abs(v) / p.driven_radius : real(0));
    out.system_torque_nm = ref_omega > 0 ? out.system_power_w / ref_omega : real(0);
    out.engine_rpm = w.mean(w.engine_rpm);
    out.gearbox_output_rpm = w.mean(w.output_rpm);
    out.engine_torque_nm = w.mean(w.engine_torque);
    out.engine_power_w = w.mean(w.engine_power);
    out.engine_command = w.mean(w.engine_command);
    out.boost_bar = w.mean(w.boost);
    out.engine_running = pt.has_engine() && pt.last_engine_result().state == EngineState::Running;
    out.gear = pt.has_gearbox() ? pt.gearbox().current_position() : 0;
    out.electric_power_w = w.mean(w.electric_power);
    out.battery_power_w = w.mean(w.battery_power);
    out.fuel_cell_power_w = w.mean(w.fuel_cell_power);
    out.motors.resize(nm);
    for (std::size_t m = 0; m < nm; ++m) {
        const ElectricMotor& em = *pt.electric_motors()[m];
        PowertrainDynoMotor& o = out.motors[m];
        o.rpm = w.mean(w.m_rpm[m]);
        o.torque_nm = w.mean(w.m_torque[m]);
        o.power_w = w.mean(w.m_power[m]);
        o.electrical_w = w.mean(w.m_electrical[m]);
        o.limit = em.drive_limit;
        o.crank_machine = em.is_crank_machine();
        if (!o.crank_machine && (o.limit == MotorLimit::BatteryDischarge || o.limit == MotorLimit::Empty)) out.battery_limited = true;
    }
    if (pt.has_hybrid_control()) {
        out.hybrid_requested_w = pt.hybrid_requested_power();
        out.hybrid_available_w = pt.hybrid_available_power();
        out.hybrid_engine_coupled = pt.hybrid_engine_coupled();
        out.hybrid_charging = pt.hybrid_charging();
    }
    return out;
}

} // namespace

PowertrainDynoResult run_powertrain_dyno(const vehicle::VehicleDesc& vehicle, EngineMapsPtr maps, const PowertrainDynoSpec& spec,
                                         jobs::JobSystem* pool, const std::atomic<bool>* cancel) {
    validate(spec);
    if (cancelled(cancel)) throw GenerationCancelled();
    const Prepared prepared = prepare(vehicle, maps, spec);

    PowertrainDynoResult result;
    const std::size_t n = spec.points.size();
    result.points.assign(n, PowertrainDynoPoint{});
    std::vector<std::exception_ptr> errors(n);
    std::atomic<bool> skipped{false};
    const auto run_one = [&](std::size_t i) {
        if (cancelled(cancel)) {
            skipped.store(true, std::memory_order_relaxed);
            return;
        }
        try {
            result.points[i] = run_point(prepared, spec, spec.points[i]);
        } catch (...) {
            errors[i] = std::current_exception();
        }
    };
    if (pool == nullptr) {
        for (std::size_t i = 0; i < n; ++i) {
            run_one(i);
            if (skipped.load(std::memory_order_relaxed)) break;
        }
    } else {
        pool->parallel_batches(n, run_one);
    }
    if (skipped.load(std::memory_order_relaxed)) throw GenerationCancelled();
    for (const std::exception_ptr& e : errors)
        if (e) std::rethrow_exception(e);
    return result;
}

} // namespace ps::drivetrain
