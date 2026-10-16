// core/src/drivetrain/engine_thermal.cpp - see engine_thermal.h.

#include "engine_thermal.h"

#include "ps/math/transcendental.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace ps::drivetrain {
namespace {

constexpr real kLn10 = 2.302585092994046;
constexpr real kGasConstantAir = 287.05;   // J/(kg K) [C]
constexpr real kCpAir = 1006.0;            // J/(kg K) [C standard value]
constexpr real kOilExpansion = 6.5e-4;     // 1/K, oil density temperature coefficient [AS]
constexpr real kOilDensityRefK = 288.75;   // 15.6 C, the data sheet's reference temperature [C]
constexpr real kGridStepK = 5.0;           // k curve grid [D]
constexpr real kGridLowK = 233.15;         // -40 C
constexpr real kGridHighK = 423.15;        // 150 C

real log10_(real x) { return ps::math::log(x) / kLn10; }

} // namespace

EngineThermal::EngineThermal(const EngineThermalConfig& cfg) : cfg_(cfg) {
    const EngineThermalDesc& e = cfg_.engine;
    const CoolingSystemDesc& c = cfg_.cooling;
    if (!(e.engine_capacity_j_k > 0.0) || !(e.oil_capacity_j_k > 0.0)) {
        throw std::runtime_error("EngineThermal: lump capacities must be > 0");
    }
    if (!(e.oil.kv100_mm2_s > 0.0 && e.oil.kv40_mm2_s > e.oil.kv100_mm2_s)) {
        throw std::runtime_error("EngineThermal: oil requires kv40 > kv100 > 0");
    }
    if (!(e.oil.hydrodynamic_reference_rpm > 0.0)) {
        throw std::runtime_error("EngineThermal: oil hydrodynamic_reference_rpm must be > 0");
    }
    if (!(e.oil_coolant_time_constant_s > 0.0) || !(e.thermostat.time_constant_s > 0.0)) {
        throw std::runtime_error("EngineThermal: time constants must be > 0");
    }
    if (!(e.thermostat.full_open_k > e.thermostat.open_k)) {
        throw std::runtime_error("EngineThermal: thermostat requires open < full_open");
    }
    if (!(c.radiator.frontal_area_m2 > 0.0) || !(c.radiator.ua_ref_w_k > 0.0) || !(c.radiator.air_mass_flow_ref_kg_s > 0.0)) {
        throw std::runtime_error("EngineThermal: radiator requires positive area, UA and reference air flow");
    }
    if (!(c.fan.on_k > c.fan.off_k)) throw std::runtime_error("EngineThermal: fan requires off < on");
    if (c.gauge_x_k_y.x.size() < 2 || c.gauge_x_k_y.x.size() != c.gauge_x_k_y.y.size()) {
        throw std::runtime_error("EngineThermal: the gauge curve needs at least 2 matching points");
    }

    // Walther (ASTM D341): log10(log10(nu + 0.7)) = A - B*log10(T_K), through KV40 and KV100.
    const real t40 = 313.15, t100 = 373.15;
    const real c1 = log10_(log10_(e.oil.kv40_mm2_s + 0.7));
    const real c2 = log10_(log10_(e.oil.kv100_mm2_s + 0.7));
    walther_b_ = (c1 - c2) / (log10_(t100) - log10_(t40));
    walther_a_ = c1 + walther_b_ * log10_(t40);
    mu_ref_ = dynamic_viscosity(e.oil.reference_temperature_k);

    // k curve on a 5 K grid anchored at T_ref (T_ref itself is a knot), from -40 C to 150 C.
    const real t_ref = e.oil.reference_temperature_k;
    int n_below = 0;
    while (t_ref - kGridStepK * static_cast<real>(n_below) > kGridLowK) ++n_below;
    int n_above = 1;
    while (t_ref + kGridStepK * static_cast<real>(n_above) < kGridHighK) ++n_above;
    for (int i = -n_below; i <= n_above; ++i) {
        const real t = t_ref + kGridStepK * static_cast<real>(i);
        k_curve_.x.push_back(t);
        k_curve_.y.push_back(friction_scale_exact(t));
    }

    // Warm start (the vehicle may switch to a cold soak at its first tick).
    t_e_ = e.thermostat.open_k + 0.25 * (e.thermostat.full_open_k - e.thermostat.open_k);
    t_o_ = e.oil.reference_temperature_k;
    t_wax_ = t_e_;
    t_play_ = t_e_;
    fan_on_ = false;
    ambient_start_pending_ = (cfg_.start == ThermalStartKind::Ambient);
    reset_baseline();
    rebuild_scales();
    snapshot_.has_thermal = true;
    snapshot_.coolant_c = t_e_ - kCelsiusToKelvin;
    snapshot_.oil_c = t_o_ - kCelsiusToKelvin;
    snapshot_.gauge = c.gauge_x_k_y.evaluate(t_e_);
    snapshot_.friction_scale = plant_k_;
}

real EngineThermal::kinematic_viscosity_mm2_s(real t_k) const {
    // log10(log10(nu + 0.7)) = A - B*log10(T)  =>  nu = 10^(10^(A - B*log10 T)) - 0.7
    const real w = ps::math::exp(kLn10 * (walther_a_ - walther_b_ * log10_(t_k)));
    return ps::math::exp(kLn10 * w) - 0.7;
}

real EngineThermal::dynamic_viscosity(real t_k) const {
    const real rho = cfg_.engine.oil.density_15c_kg_m3 * (1.0 - kOilExpansion * (t_k - kOilDensityRefK));
    return kinematic_viscosity_mm2_s(t_k) * rho;
}

real EngineThermal::friction_scale_exact(real t_k) const {
    const EngineThermalDesc::Oil& o = cfg_.engine.oil;
    const real ratio = std::sqrt(dynamic_viscosity(t_k) / mu_ref_);
    return std::max(o.friction_scale_min, 1.0 + o.hydrodynamic_fraction * (ratio - 1.0));
}

void EngineThermal::thermostat_update(const EngineThermalDesc::Thermostat& th, real dt, real t_engine, real& t_wax, real& t_play,
                                      real& opening) {
    const real a_wax = dt / th.time_constant_s;
    t_wax = (t_wax + a_wax * t_engine) / (1.0 + a_wax);
    const real half = 0.5 * th.hysteresis_k;
    t_play = std::clamp(t_play, t_wax - half, t_wax + half);
    opening = std::clamp((t_play - th.open_k) / (th.full_open_k - th.open_k), 0.0, 1.0);
}

real EngineThermal::radiator_effectiveness(real ntu, real cr) {
    if (!(ntu > 0.0)) return 0.0;
    if (cr < 1e-9) return 1.0 - ps::math::exp(-ntu);
    // Cross-flow, both fluids unmixed (Incropera & DeWitt Table 11.3 [C]).
    return 1.0 - ps::math::exp((1.0 / cr) * ps::math::pow(ntu, 0.22) * (ps::math::exp(-cr * ps::math::pow(ntu, 0.78)) - 1.0));
}

real EngineThermal::radiator_conductance(real air_mass_flow_kg_s, real coolant_mass_flow_kg_s) const {
    if (!(air_mass_flow_kg_s > 0.0) || !(coolant_mass_flow_kg_s > 0.0)) return 0.0;
    const CoolingSystemDesc::Radiator& r = cfg_.cooling.radiator;
    const real c_a = air_mass_flow_kg_s * kCpAir;
    const real c_c = coolant_mass_flow_kg_s * cfg_.engine.coolant_cp_j_kg_k;
    const real ua = r.ua_ref_w_k * ps::math::pow(air_mass_flow_kg_s / r.air_mass_flow_ref_kg_s, r.ua_exponent) *
                    capacity_.radiator_ua_scale;
    const real c_min = std::min(c_a, c_c);
    const real c_max = std::max(c_a, c_c);
    const real cr = c_min / c_max;
    const real ntu = ua / c_min;
    return radiator_effectiveness(ntu, cr) * c_min;
}

void EngineThermal::rebuild_scales() {
    if (override_k_.has_value()) {
        plant_k_ = *override_k_;
        ecu_k_ = *override_k_;
        return;
    }
    plant_k_ = k_curve_.evaluate(t_o_);
    if (wear_.friction_scale != 1.0) plant_k_ *= wear_.friction_scale; // worn internals (EngineWear); the ECU's k is not
    // The ECU's oil temperature reading is quantised to 1 K relative to T_ref, so a warm start (oil exactly T_ref)
    // and every temperature within half a kelvin of it give exactly the reference scale 1.0.
    const real t_ref = cfg_.engine.oil.reference_temperature_k;
    const real t_q = t_ref + std::floor((t_o_ - t_ref) + 0.5);
    ecu_k_ = k_curve_.evaluate(t_q);
}

void EngineThermal::clear_friction_scale_override_for_test() {
    override_k_.reset();
    rebuild_scales();
}

void EngineThermal::reset_baseline() {
    t_e0_ = t_e_;
    t_o0_ = t_o_;
    in_since_ = 0.0;
    out_since_ = 0.0;
    balance_carry_ = ledger_.balance_error;
    ledger_.stored = 0.0;
}

void EngineThermal::set_state(const EngineThermalState& s) {
    t_e_ = s.coolant_c + kCelsiusToKelvin;
    t_o_ = s.oil_c + kCelsiusToKelvin;
    t_wax_ = t_e_;
    t_play_ = t_e_;
    fan_on_ = false;
    reset_baseline();
    rebuild_scales();
    snapshot_.coolant_c = s.coolant_c;
    snapshot_.oil_c = s.oil_c;
    snapshot_.fan_on = false;
    snapshot_.fan_power_w = 0.0;
    snapshot_.gauge = cfg_.cooling.gauge_x_k_y.evaluate(t_e_);
    snapshot_.friction_scale = plant_k_;
}

void EngineThermal::begin_tick() {
    if (ambient_start_pending_) {
        ambient_start_pending_ = false;
        const real ta = env_.ambient_k;
        set_state(EngineThermalState{ta - kCelsiusToKelvin, ta - kCelsiusToKelvin});
    }
    if (has_pending_) {
        has_pending_ = false;
        set_state(pending_);
    }
    if (has_pending_capacity_) {
        has_pending_capacity_ = false;
        capacity_ = pending_capacity_;
    }
    if (has_pending_wear_) {
        has_pending_wear_ = false;
        wear_ = pending_wear_;
        rebuild_scales();
        snapshot_.friction_scale = plant_k_;
        snapshot_.wear_friction_scale = wear_.friction_scale;
    }
}

void EngineThermal::step(real engine_rpm) {
    if (!(dt_acc_ > 0.0)) {
        released_ = 0.0;
        friction_ = 0.0;
        dt_acc_ = 0.0;
        return;
    }
    const EngineThermalDesc& e = cfg_.engine;
    const CoolingSystemDesc& c = cfg_.cooling;
    const real dt = dt_acc_;
    const real t_a = env_.ambient_k;

    // 1. Thermostat (wax lag, play, opening) from the OLD engine temperature.
    real wax_new = t_wax_, play_new = t_play_, x = 0.0;
    thermostat_update(e.thermostat, dt, t_e_, wax_new, play_new, x);

    // 2. Fan hysteresis from the old engine temperature.
    fan_on_ = fan_update(c.fan, fan_on_, t_e_);

    // 3. Conductances.
    const real ram = std::max(0.0, env_.ram_speed_mps);
    const real v_face = std::max(c.radiator.ram_coefficient * ram, fan_on_ ? c.fan.face_velocity_m_s * capacity_.fan_flow_scale : 0.0);
    const real rho_air = env_.ambient_pressure_pa / (kGasConstantAir * t_a);
    const real m_air = rho_air * c.radiator.frontal_area_m2 * v_face;
    const real pump_l_min = e.pump_l_min_per_krpm * (std::max(0.0, engine_rpm) / 1000.0);
    const real m_coolant = x * pump_l_min / 60000.0 * e.coolant_density_kg_m3;
    const real g_rad = radiator_conductance(m_air, m_coolant);
    const real g_ba = e.block_air_w_k + e.block_air_w_k_per_m_s * ram;
    const real g_oa = e.oil_air_w_k + (e.oil_air_w_k_per_m_s + c.oil_cooler_air_w_k_per_m_s) * ram;
    const real g_oc = e.oil_capacity_j_k / e.oil_coolant_time_constant_s;

    // 4. Backward-Euler closed form.
    const real e_wall = e.wall_heat_fraction * released_;
    const real e_fric_e = (1.0 - e.friction_to_oil_fraction) * friction_;
    const real e_fric_o = e.friction_to_oil_fraction * friction_;
    const real a11 = e.engine_capacity_j_k + dt * (g_rad + g_ba + g_oc);
    const real a12 = -dt * g_oc;
    const real a21 = a12;
    const real a22 = e.oil_capacity_j_k + dt * (g_oa + g_oc);
    const real b1 = e.engine_capacity_j_k * t_e_ + e_wall + e_fric_e + dt * (g_rad + g_ba) * t_a;
    const real b2 = e.oil_capacity_j_k * t_o_ + e_fric_o + dt * g_oa * t_a;
    const real det = a11 * a22 - a12 * a21;
    const real te_new = (b1 * a22 - a12 * b2) / det;
    const real to_new = (a11 * b2 - a21 * b1) / det;

    // 5. Ledger at the new temperatures.
    const real out_rad = dt * g_rad * (te_new - t_a);
    const real out_block = dt * g_ba * (te_new - t_a);
    const real out_oil = dt * g_oa * (to_new - t_a);
    const real in_step = e_wall + e_fric_e + e_fric_o;
    ledger_.wall_in += e_wall;
    ledger_.friction_in += friction_;
    ledger_.radiator_out += out_rad;
    ledger_.block_air_out += out_block;
    ledger_.oil_air_out += out_oil;
    ledger_.abs_flow_sum += std::abs(e_wall) + std::abs(friction_) + std::abs(out_rad) + std::abs(out_block) + std::abs(out_oil);
    in_since_ += in_step;
    out_since_ += out_rad + out_block + out_oil;

    t_wax_ = wax_new;
    t_play_ = play_new;
    t_e_ = te_new;
    t_o_ = to_new;
    ledger_.stored = e.engine_capacity_j_k * (t_e_ - t_e0_) + e.oil_capacity_j_k * (t_o_ - t_o0_);
    ledger_.balance_error = balance_carry_ + (ledger_.stored - (in_since_ - out_since_));

    // 6. Snapshot.
    snapshot_.has_thermal = true;
    snapshot_.coolant_c = t_e_ - kCelsiusToKelvin;
    snapshot_.oil_c = t_o_ - kCelsiusToKelvin;
    snapshot_.thermostat_opening = x;
    snapshot_.fan_on = fan_on_;
    // A damaged fan (fan_flow_scale < 1) turns slower. Fan affinity laws: flow ~ speed, power ~ speed^3, so the electrical
    // power it draws goes with the cube of the flow factor. Skipped at exactly 1 (bit for bit the loaded power_w).
    const real fs = capacity_.fan_flow_scale;
    snapshot_.fan_power_w = fan_on_ ? (fs == 1.0 ? c.fan.power_w : c.fan.power_w * fs * fs * fs) : 0.0;
    snapshot_.radiator_ua_scale = capacity_.radiator_ua_scale;
    snapshot_.fan_flow_scale = capacity_.fan_flow_scale;
    snapshot_.wear_friction_scale = wear_.friction_scale;
    snapshot_.radiator_heat_w = g_rad * (t_e_ - t_a);
    snapshot_.wall_heat_w = e_wall / dt;
    snapshot_.friction_heat_w = friction_ / dt;
    snapshot_.gauge = c.gauge_x_k_y.evaluate(t_e_);
    snapshot_.warning_bits = 0;
    if (t_e_ >= c.warnings.overheat_k) snapshot_.warning_bits |= kThermalWarningOverheat;
    if (t_e_ < c.warnings.cold_k) snapshot_.warning_bits |= kThermalWarningCold;

    // 7. Next tick's friction scales.
    rebuild_scales();
    snapshot_.friction_scale = plant_k_;

    released_ = 0.0;
    friction_ = 0.0;
    dt_acc_ = 0.0;
}

std::uint64_t EngineThermal::fold_state_hash(std::uint64_t seed) const {
    std::uint64_t h = seed;
    const auto fold = [&h](const void* p, std::size_t n) {
        const auto* bytes = static_cast<const unsigned char*>(p);
        for (std::size_t i = 0; i < n; ++i) {
            h ^= bytes[i];
            h *= 1099511628211ULL;
        }
    };
    fold(&t_e_, sizeof(t_e_));
    fold(&t_o_, sizeof(t_o_));
    fold(&t_wax_, sizeof(t_wax_));
    fold(&t_play_, sizeof(t_play_));
    const std::uint8_t fan = fan_on_ ? 1 : 0;
    fold(&fan, sizeof(fan));
    // The cooling capacity is an input like the controls, but it persists; fold it once it is not the default so a
    // damaged run hashes differently, while every run at (1, 1) keeps its previous hash.
    if (capacity_.radiator_ua_scale != 1.0 || capacity_.fan_flow_scale != 1.0) {
        fold(&capacity_.radiator_ua_scale, sizeof(capacity_.radiator_ua_scale));
        fold(&capacity_.fan_flow_scale, sizeof(capacity_.fan_flow_scale));
    }
    if (wear_.friction_scale != 1.0) fold(&wear_.friction_scale, sizeof(wear_.friction_scale)); // same rule as the capacity
    return h;
}

} // namespace ps::drivetrain
