// core/src/environment/condition_grip.cpp - see ps/environment/condition_grip.h.

#include "ps/environment/condition_grip.h"

#include "ps/math/transcendental.h"

#include <algorithm>

namespace ps::environment {

namespace {
constexpr real kDegPerSpeed = 3.6;                 // km/h per m/s
constexpr real kVRefKmh = 60.0;                    // PIARC IFI reference speed [C]
constexpr real kVRefMps = kVRefKmh / kDegPerSpeed; // 16.667 m/s
} // namespace

void ConditionGrip::reset() {
    *this = ConditionGrip{};
}

void ConditionGrip::build_static(const ConditionGrid& grid, const TyreConditionFactors& profile) {
    reset();
    profile_ = profile;
    count_ = std::min(grid.class_count(), kMaxSurfaceClasses);
    for (int i = 0; i < count_; ++i) {
        stateful_[i] = grid.has_state(i);
        vegetated_[i] = grid.kind(i) == SurfaceKind::Vegetated;
        friction_[i] = grid.column(i).friction;
        if (!stateful_[i]) continue;
        // PIARC International Friction Index speed constant, metric MPD = MTD.
        const real sp_kmh = 14.2 + 89.7 * grid.column(i).mtd_mm;   // [C PIARC 1995]
        const real sp_mps = sp_kmh / kDegPerSpeed;
        for (int j = 0; j < kSpeedSamples; ++j) {
            const real v = kSpeedMaxMps * static_cast<real>(j) / static_cast<real>(kSpeedSamples - 1);
            real mu = friction_[i].wet * math::exp((kVRefMps - v) / sp_mps);
            if (mu > friction_[i].damp) mu = friction_[i].damp;   // plan 5.2 step 3 clamp
            table_[i][j] = mu;
        }
    }
    valid_ = count_ > 0;
}

void ConditionGrip::update_state(const ConditionGrid& grid, real t_env) {
    if (!valid_) return;
    const real w = grid.weight_at(t_env);
    const int n = std::min(grid.class_count(), kMaxSurfaceClasses);
    for (int i = 0; i < n; ++i) {
        w_wet_[i] = 0.0;
        w_grass_[i] = 0.0;
        if (!stateful_[i]) continue;
        const SurfaceClassState& a = grid.prev(i);
        const SurfaceClassState& b = grid.next(i);
        const auto lerp = [w](real x, real y) { return x + (y - x) * w; };
        const real water = lerp(a.water_mm, b.water_mm);
        const real canopy = lerp(a.canopy_water_mm, b.canopy_water_mm);
        const real theta = lerp(a.soil_moisture, b.soil_moisture);
        const ColumnModel& col = grid.column(i);
        real ww = 0.0;
        if (col.kind == SurfaceKind::Sealed) {
            ww = col.w_tex_mm > 0.0 ? water / col.w_tex_mm : (water > 0.0 ? 1.0 : 0.0);
        } else if (col.kind == SurfaceKind::Soil || col.kind == SurfaceKind::Vegetated) {
            // Wet-earth level from the soil moisture (the mud index form, plan
            // 4.6; the mud-specific grip curve is W9).
            if (col.soil_theta_sat > col.soil_theta_fc)
                ww = (theta - col.soil_theta_fc) / (col.soil_theta_sat - col.soil_theta_fc);
            if (water > 0.0 && ww < 1.0) ww = 1.0;   // ponded
        }
        w_wet_[i] = std::clamp(ww, real(0.0), real(1.0));
        if (vegetated_[i]) {
            const real cw = col.canopy_capacity_mm > 0.0 ? canopy / col.canopy_capacity_mm : 0.0;
            w_grass_[i] = std::clamp(std::max(cw, w_wet_[i]), real(0.0), real(1.0));
        }
    }
}

real ConditionGrip::water_weight(int surface_id) const {
    if (!valid_ || surface_id < 0 || surface_id >= count_) return 0.0;
    return w_wet_[surface_id];
}

real ConditionGrip::factor(int surface_id, real speed_m_s) const { return factor(surface_id, profile_, speed_m_s); }

real ConditionGrip::factor(int surface_id, const TyreConditionFactors& p, real speed_m_s) const {
    if (!valid_ || surface_id < 0 || surface_id >= count_ || !stateful_[surface_id]) return 1.0;
    const real v = std::clamp(speed_m_s, real(0.0), kSpeedMaxMps);
    const int j = std::clamp(static_cast<int>(v / kSpeedMaxMps * (kSpeedSamples - 1) + 0.5), 0, kSpeedSamples - 1);
    const real mu_wet = table_[surface_id][j];
    if (vegetated_[surface_id]) {
        const real g = w_grass_[surface_id];
        return (1.0 - g) * p.dry + g * friction_[surface_id].wet_grass * p.wet_grass;
    }
    const real ww = w_wet_[surface_id];
    return (1.0 - ww) * p.dry + ww * mu_wet * p.wet;
}

} // namespace ps::environment
