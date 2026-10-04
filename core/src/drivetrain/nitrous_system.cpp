// drivetrain/nitrous_system.cpp - N2O plan N1 section 3 (see nitrous_system.h).
#include "nitrous_system.h"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace ps::drivetrain {

namespace {

constexpr real kPi = 3.14159265358979323846;
constexpr real kRadPerSecToRpm = real(60.0) / (real(2.0) * kPi);
constexpr real kBarToPa = real(1.0e5);
constexpr real kMmToM = real(1.0e-3);

real circle_area_m2(real d_mm) {
    const real d = d_mm * kMmToM;
    return real(0.25) * kPi * d * d;
}

std::uint64_t fold_bytes(std::uint64_t h, const void* p, std::size_t n) {
    const auto* b = static_cast<const unsigned char*>(p);
    for (std::size_t i = 0; i < n; ++i) {
        h ^= b[i];
        h *= 1099511628211ULL;
    }
    return h;
}

} // namespace

NitrousSystem::NitrousSystem(const N2oBottleDesc& bottle, const NitrousKitDesc& kit, real fuel_density_kg_m3, int arm_slot, int heater_slot,
                             int purge_slot)
    : bottle_(bottle),
      kit_(kit),
      tables_(std::make_unique<OxidizerTables>(bottle.oxidizer)),
      fuel_density_kg_m3_(fuel_density_kg_m3),
      arm_slot_(arm_slot),
      heater_slot_(heater_slot),
      purge_slot_(purge_slot),
      m_bottle_(std::clamp(bottle.initial_kg, real(0.0), bottle.capacity_kg)) {
    nitrous_area_m2_ = circle_area_m2(kit_.nitrous_jet_diameter_mm);
    fuel_area_m2_ = circle_area_m2(kit_.fuel_jet_diameter_mm);
    purge_area_m2_ = kit_.controller.has_purge ? circle_area_m2(kit_.controller.purge.jet_diameter_mm) : real(0.0);
    const bool heater_default_on = bottle_.has_heater && bottle_.heater.default_on >= real(0.5);
    status_.bottle_p_pa = bottle_pressure_pa(m_bottle_, heater_default_on);
    const SaturatedLiquid liq = tables_->liquid_at_pressure(status_.bottle_p_pa);
    status_.bottle_t_k = liq.t_k;
    status_.h_in_j_kg = liq.h_l_j_kg;
}

real NitrousSystem::bottle_pressure_pa(real fill_kg, bool heater_on) const {
    if (heater_on && bottle_.has_heater) {
        if (bottle_.heater.set_point_is_temperature) return tables_->saturation_pressure_pa(bottle_.heater.set_point_k);
        return bottle_.heater.set_point_gauge_bar * kBarToPa + kIsaAirPressurePa;
    }
    switch (bottle_.pressure.mode) {
    case N2oPressureMode::Constant: return bottle_.pressure.gauge_bar * kBarToPa + kIsaAirPressurePa;
    case N2oPressureMode::VsFill: {
        const real fraction = bottle_.capacity_kg > real(0.0) ? std::clamp(fill_kg / bottle_.capacity_kg, real(0.0), real(1.0)) : real(0.0);
        return bottle_.pressure.gauge_vs_fill.evaluate(fraction) * kBarToPa + kIsaAirPressurePa;
    }
    case N2oPressureMode::Temperature: return tables_->saturation_pressure_pa(bottle_.pressure.temperature_k);
    }
    return kIsaAirPressurePa;
}

const NitrousStatus& NitrousSystem::step(const NitrousInputs& in) {
    const NitrousControllerDesc& c = kit_.controller;
    NitrousStatus s;

    // Bottle state (section 3 step 2): a pure function of the fill state and the heater channel.
    const bool heater_on = bottle_.has_heater && in.heater >= real(0.5);
    s.bottle_p_pa = bottle_pressure_pa(m_bottle_, heater_on);
    const SaturatedLiquid liq = tables_->liquid_at_pressure(s.bottle_p_pa);
    s.bottle_t_k = liq.t_k;
    s.h_in_j_kg = liq.h_l_j_kg;
    const real gauge_bar = (s.bottle_p_pa - kIsaAirPressurePa) * real(1.0e-5);

    // Request (step 3).
    const bool armed = in.arm >= real(0.5);
    const bool wot = in.throttle_command >= c.min_throttle_command;
    if (!wot) latch_ = static_cast<std::uint8_t>(NitrousSafetyCut::None); // a cut holds until the command drops below the switch
    const real rpm = std::abs(in.omega) * kRadPerSecToRpm;
    const bool rpm_ok = rpm >= c.min_rpm && rpm <= c.max_rpm;
    const bool gear_ok = !in.has_gearbox || (in.gear >= c.min_gear && in.gear <= c.max_gear);
    const bool speed_ok = in.speed_mps >= c.min_speed_mps;
    const bool base_request = armed && wot && rpm_ok && gear_ok && speed_ok && in.engine_running && !in.ignition_cut && !prev_engine_cut_;

    // Safety (step 4), each latching.
    if (base_request && latch_ == static_cast<std::uint8_t>(NitrousSafetyCut::None)) {
        if (c.safety.min_bottle_gauge_bar > real(0.0) && gauge_bar < c.safety.min_bottle_gauge_bar) {
            latch_ = static_cast<std::uint8_t>(NitrousSafetyCut::BottlePressure);
        } else if (c.safety.cut_on_fuel_starve && prev_starving_) {
            latch_ = static_cast<std::uint8_t>(NitrousSafetyCut::FuelStarve);
        } else if (c.safety.max_active_s > real(0.0) && t_active_ > c.safety.max_active_s) {
            latch_ = static_cast<std::uint8_t>(NitrousSafetyCut::DutyTime);
        }
    }
    s.safety_cut = static_cast<NitrousSafetyCut>(latch_);
    const bool open = base_request && latch_ == static_cast<std::uint8_t>(NitrousSafetyCut::None);
    s.solenoid_open = open;
    if (open && !solenoid_open_) ++activations_;

    // Progressive duty (step 5): from the time the solenoid has been open so far.
    real frac = real(0.0);
    if (open) {
        const NitrousProgressiveDesc& pg = c.progressive;
        frac = pg.start_fraction;
        if (t_active_ >= pg.delay_s) {
            frac = pg.ramp_s > real(0.0)
                       ? std::clamp(pg.start_fraction + (real(1.0) - pg.start_fraction) * (t_active_ - pg.delay_s) / pg.ramp_s, real(0.0), real(1.0))
                       : real(1.0);
        }
    }
    s.flowing = open && m_bottle_ > real(0.0);
    s.flow_fraction = s.flowing ? frac : real(0.0);

    // Flows (step 6).
    real deliver_n = real(0.0), mdot_kf = real(0.0);
    pending_empties_ = false;
    if (s.flowing && in.h > real(0.0)) {
        const real count = real(kit_.nozzle_count);
        const real g = kit_.flow_model == NitrousFlowModel::Hem ? tables_->g_hem(s.bottle_p_pa) : tables_->g_dyer(s.bottle_p_pa, in.p_intake_pa);
        const real mdot_n_demand = frac * count * kit_.nitrous_jet_cd * nitrous_area_m2_ * g;
        const real demand_kg = mdot_n_demand * in.h;
        if (demand_kg >= m_bottle_) {
            deliver_n = m_bottle_; // the emptying substep delivers exactly the rest
            pending_empties_ = true;
        } else {
            deliver_n = demand_kg;
        }
        const real p_fuel_abs =
            (kit_.fuel_reference == NitrousFuelReference::Manifold ? in.p_intake_pa : in.p_ambient_pa) + kit_.fuel_pressure_gauge_bar * kBarToPa;
        const real dp = std::max(real(0.0), p_fuel_abs - in.p_intake_pa);
        const real mdot_kf_demand = frac * count * kit_.fuel_jet_cd * fuel_area_m2_ * std::sqrt(real(2.0) * fuel_density_kg_m3_ * dp);
        // The two solenoids pulse together: an emptying bottle cuts the fuel in proportion.
        mdot_kf = demand_kg > real(0.0) ? mdot_kf_demand * (deliver_n / demand_kg) : real(0.0);
        s.retard_deg = c.retard_is_curve ? c.retard_vs_fraction.evaluate(frac) : c.spark_retard_deg;
    }
    pending_n_kg_ = deliver_n;

    // Purge (step 7), independent of the request, after the injection took its share.
    real deliver_p = real(0.0);
    const bool purge_on = c.has_purge && in.purge >= real(0.5) && m_bottle_ > real(0.0);
    if (purge_on && in.h > real(0.0)) {
        const real rest = m_bottle_ - deliver_n; // identical expression to commit() so the last substep ends at exactly 0.0
        const real want = c.purge.cd * purge_area_m2_ * tables_->g_hem(s.bottle_p_pa) * in.h;
        deliver_p = std::max(real(0.0), std::min(want, rest));
    }
    pending_p_kg_ = deliver_p;
    s.purge_active = purge_on;
    if (purge_on && !purge_prev_on_) ++purge_events_;
    purge_prev_on_ = purge_on;

    if (in.h > real(0.0)) {
        s.mdot_n_kg_s = deliver_n / in.h;
        s.mdot_purge_kg_s = deliver_p / in.h;
    }
    s.mdot_kf_kg_s = mdot_kf;

    // State advance: the active time counts while the solenoid is open, resets when it closes.
    solenoid_open_ = open;
    t_active_ = open ? t_active_ + in.h : real(0.0);

    status_ = s;
    return status_;
}

NitrousInput NitrousSystem::engine_input() const {
    NitrousInput n;
    if (!status_.flowing || status_.mdot_n_kg_s <= real(0.0)) return n;
    n.mdot_n_kg_s = status_.mdot_n_kg_s;
    n.mdot_kf_kg_s = status_.mdot_kf_kg_s;
    n.h_in_j_kg = status_.h_in_j_kg;
    n.retard_deg = status_.retard_deg;
    n.injection = kit_.injection == NitrousInjection::DirectPort ? std::uint8_t(1) : std::uint8_t(0);
    n.enrichment = 0;
    n.active = true;
    return n;
}

NitrousBooking NitrousSystem::commit() {
    NitrousBooking b;
    b.n2o_kg = pending_n_kg_;
    b.purge_kg = pending_p_kg_;
    // (m - n) - p: when n or n + p is the whole rest the result is exactly 0.0.
    m_bottle_ = std::max(real(0.0), (m_bottle_ - pending_n_kg_) - pending_p_kg_);
    pending_n_kg_ = real(0.0);
    pending_p_kg_ = real(0.0);
    pending_empties_ = false;
    return b;
}

real NitrousSystem::set_bottle_kg(real kg) {
    const real old = m_bottle_;
    m_bottle_ = std::clamp(kg, real(0.0), bottle_.capacity_kg);
    return m_bottle_ - old;
}

std::uint64_t NitrousSystem::fold_state_hash(std::uint64_t seed) const {
    std::uint64_t h = seed;
    const std::uint64_t tables_hash = tables_->content_hash();
    h = fold_bytes(h, &tables_hash, sizeof(tables_hash));
    h = fold_bytes(h, &m_bottle_, sizeof(m_bottle_));
    h = fold_bytes(h, &t_active_, sizeof(t_active_));
    const std::uint8_t flags =
        static_cast<std::uint8_t>((solenoid_open_ ? 1u : 0u) | (prev_engine_cut_ ? 2u : 0u) | (prev_starving_ ? 4u : 0u));
    h = fold_bytes(h, &flags, sizeof(flags));
    h = fold_bytes(h, &latch_, sizeof(latch_));
    return h;
}

} // namespace ps::drivetrain
