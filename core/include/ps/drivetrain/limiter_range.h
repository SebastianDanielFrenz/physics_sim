// ps/drivetrain/limiter_range.h - G1 limiter-range robustness (docs/data_formats/engine.md "Beyond the authored rpm range").
//
// A garage may move an engine's rev limiter anywhere between 2000 and 20000 rpm. Three things read absolute rpm numbers that
// were authored for the stock limiter; this header holds the rules that keep them sane at any limiter, as pure functions of
// the descriptors (no state, no allocation), shared by the loader (its warnings), the runtime and the dyno:
//
//   1. rpm_rolloff: past the highest rpm the engine's own data reaches, the fired part of the torque rolls off linearly to
//      zero over rpm_rolloff_band_fraction of that rpm (instead of the data being held flat to any speed). Armed only when
//      the engine can fire above its data (fired_top_rpm > authored_top_rpm); disarmed it is exactly the old behaviour.
//   2. fit_launch_rpm: a launch control rpm at or above max_limiter_fraction of the limiter is clamped there (a load
//      warning, not an error).
//   3. shift_point_scale: auto-shift points whose upshift lies above max_limiter_fraction of the effective limiter are
//      scaled down together (up and down, so the hysteresis and its ratio to the gear steps are kept).
//
// Every number used here is a field of a descriptor (a data file key) with an [AS] default; see engine.md / electronics.md.
#pragma once

#include "ps/drivetrain/powertrain_desc.h"
#include "ps/types.h"

#include <algorithm>
#include <variant>
#include <vector>

namespace ps::drivetrain {

// The fired-torque multiplier past the engine's data. factor(rpm) = 1 up to start_rpm, (end - rpm) / (end - start) inside
// the band, 0 at and above end_rpm. A default-constructed (disarmed) roll-off is 1 everywhere.
struct RpmRolloff {
    real start_rpm = 0.0; // the authored top (n_e)
    real end_rpm = 0.0;   // n_e * (1 + band fraction) (n_z); 0 = disarmed
    [[nodiscard]] bool armed() const { return end_rpm > start_rpm; }
    // true where factor() < 1 (callers branch on this so that every rpm inside the data runs the old expressions exactly).
    [[nodiscard]] bool active(real rpm) const { return armed() && rpm > start_rpm; }
    [[nodiscard]] real factor(real rpm) const {
        if (!active(rpm)) return real(1.0);
        if (rpm >= end_rpm) return real(0.0);
        return (end_rpm - rpm) / (end_rpm - start_rpm);
    }
    // d factor / d rpm (0 outside the open band, like a piecewise-linear curve's slope outside its segment).
    [[nodiscard]] real slope_per_rpm(real rpm) const {
        return (armed() && rpm > start_rpm && rpm < end_rpm) ? real(-1.0) / (end_rpm - start_rpm) : real(0.0);
    }
};

namespace detail {
inline real top_of(const std::vector<real>& x, real acc) { return x.empty() ? acc : std::max(acc, x.back()); }
inline real maps_top(const SimulatedEngineMapsDesc& m, real acc) {
    acc = top_of(m.ve.x, acc);
    acc = top_of(m.eta_i.x, acc);
    acc = top_of(m.theta_mbt_deg.x, acc);
    acc = top_of(m.friction_torque_nm.x, acc);
    if (m.has_pmep_excess) acc = top_of(m.pmep_excess_pa.x, acc);
    if (m.has_theta_kl) acc = top_of(m.theta_kl_deg.x, acc);
    acc = top_of(m.wot_p_im_pa.x, acc);
    return acc;
}
} // namespace detail

// The highest rpm the engine's OWN data reaches (n_e): torque map = the WOT, motoring and fuel-map rpm axes; simulated = the
// rpm axes of every engine map (ve, eta_i, MBT, friction, pmep excess, knock limit, WOT plenum rows) of every cam profile /
// lift node, and the cycle block's grid while the maps are still pending. The ECU part's tables are control, not engine data,
// and do not count. 0 for an engine without any rpm axis.
[[nodiscard]] inline real authored_top_rpm(const TorqueMapEngineDesc& e) {
    real top = detail::top_of(e.wot_torque_nm_vs_rpm.x, real(0.0));
    top = detail::top_of(e.motoring_torque_nm_vs_rpm.x, top);
    if (e.has_fuel_map) top = detail::top_of(e.fuel_map.x, top);
    return top;
}
[[nodiscard]] inline real authored_top_rpm(const SimulatedEngineDesc& e) {
    real top = detail::maps_top(e.maps, real(0.0));
    for (const SimulatedEngineMapsDesc& m : e.cam_profile_maps) top = detail::maps_top(m, top);
    for (const SimulatedEngineMapsDesc& m : e.lift_node_maps) top = detail::maps_top(m, top);
    if (e.maps_pending && e.cycle) top = detail::top_of(e.cycle->grid_rpm, top);
    return top;
}

// The highest rpm the engine fires at: the limiter (a FuelCut latches at it, a SoftCut fires nothing at it), or a
// governor's high idle when that is lower (an all-speed governor cuts the fuel there).
[[nodiscard]] inline real fired_top_rpm(const TorqueMapEngineDesc& e) {
    return e.has_governor ? std::min(e.limiter.rpm, e.governor.high_idle_rpm) : e.limiter.rpm;
}
[[nodiscard]] inline real fired_top_rpm(const SimulatedEngineDesc& e) { return e.limiter.rpm; }

namespace detail {
inline RpmRolloff make_rolloff(real authored_top, real fired_top, real band_fraction) {
    RpmRolloff r;
    if (authored_top > real(0.0) && fired_top > authored_top && band_fraction > real(0.0)) {
        r.start_rpm = authored_top;
        r.end_rpm = authored_top * (real(1.0) + band_fraction);
    }
    return r;
}
} // namespace detail

// Armed iff the engine fires above its data (fired_top_rpm > authored_top_rpm). An engine whose limiter lies inside its data
// never fires beyond it, so its roll-off stays disarmed and every result is the pre-G1 one bit for bit.
[[nodiscard]] inline RpmRolloff rpm_rolloff(const TorqueMapEngineDesc& e) {
    return detail::make_rolloff(authored_top_rpm(e), fired_top_rpm(e), e.rpm_rolloff_band_fraction);
}
[[nodiscard]] inline RpmRolloff rpm_rolloff(const SimulatedEngineDesc& e) {
    return detail::make_rolloff(authored_top_rpm(e), fired_top_rpm(e), e.rpm_rolloff_band_fraction);
}

// The rpm the engine can actually be driven to under power: the FuelCut limiter, the start of a SoftCut's taper, a
// governor's high idle, or the end of an armed roll-off (no fired torque beyond it) - whichever is lowest.
template <class Engine>
[[nodiscard]] inline real effective_limiter_rpm(const Engine& e) {
    real lim = e.limiter.type == LimiterType::SoftCut ? e.limiter.rpm - e.limiter.band_rpm : e.limiter.rpm;
    lim = std::min(lim, fired_top_rpm(e));
    const RpmRolloff r = rpm_rolloff(e);
    if (r.armed()) lim = std::min(lim, r.end_rpm);
    return lim;
}

// The same three for a component that may be an engine (std::nullopt-like 0 / disarmed for anything else).
[[nodiscard]] inline real effective_limiter_rpm(const ComponentParams& p) {
    if (const auto* t = std::get_if<TorqueMapEngineDesc>(&p)) return effective_limiter_rpm(*t);
    if (const auto* s = std::get_if<SimulatedEngineDesc>(&p)) return effective_limiter_rpm(*s);
    return real(0.0);
}

// --- launch control -------------------------------------------------------------------------------------------------
// [AS] default of LaunchControlDesc / LaunchAssistDesc::max_limiter_fraction: a launch rpm is held at most at 80 % of the
// limiter. Shipped launch rpms sit at 0.55-0.62 of their engine's limiter; 0.8 leaves room for the limiter's own fuel-cut
// hysteresis / soft-cut band below the second limit and is still a high launch.
inline constexpr real kDefaultLaunchLimiterFraction = 0.8;

struct LaunchRpmFit {
    real rpm = 0.0;       // the rpm the launch control uses
    bool clamped = false; // true: `requested` was at or above max_limiter_fraction * limiter
    real cap = 0.0;       // max_limiter_fraction * limiter
};

// requested: the part's launch rpm (absolute, or a fraction already resolved against the limiter). A requested rpm below
// cap is returned unchanged (bit for bit). At or above it the rpm becomes the cap; if the cap itself is not above idle (a
// limiter barely above idle) the midpoint between idle and the limiter. The caller still rejects requested <= idle (a data
// error that no limiter causes) and limiter <= idle.
[[nodiscard]] inline LaunchRpmFit fit_launch_rpm(real requested, real idle_rpm, real limiter_rpm, real max_limiter_fraction) {
    LaunchRpmFit f;
    f.cap = max_limiter_fraction * limiter_rpm;
    f.rpm = requested;
    if (requested < f.cap) return f;
    f.clamped = true;
    f.rpm = f.cap > idle_rpm ? f.cap : real(0.5) * (idle_rpm + limiter_rpm);
    return f;
}

// --- auto-shift ------------------------------------------------------------------------------------------------------
// [AS] default of AutoShiftDesc::max_limiter_fraction: an upshift point may lie at most at 98.5 % of the effective limiter.
// The highest shipped ratio is car_hyper_n2o's Sport mode, 8100 / 8250 = 0.982.
inline constexpr real kDefaultShiftLimiterFraction = 0.985;

// The factor every auto-shift point (both upshift and both downshift rpms) is multiplied by: 1 when the higher upshift
// point lies at or below `cap_rpm` (or cap_rpm <= 0 = no cap), else cap_rpm / max(up_zero, up_full). One factor for all
// four keeps each down/up ratio, hence the hysteresis relative to the gear steps the points were authored for.
[[nodiscard]] inline real shift_point_scale(real up_rpm_at_zero_throttle, real up_rpm_at_full_throttle, real cap_rpm) {
    const real up = std::max(up_rpm_at_zero_throttle, up_rpm_at_full_throttle);
    if (!(cap_rpm > real(0.0)) || !(up > cap_rpm)) return real(1.0);
    return cap_rpm / up;
}

} // namespace ps::drivetrain
