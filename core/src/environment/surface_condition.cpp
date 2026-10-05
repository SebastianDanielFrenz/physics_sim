// ps/environment/surface_condition.h and surface_condition_desc.h
// implementation (weather plan 4.1-4.3, 4.7; W5).

#include "ps/environment/surface_condition.h"

#include "ps/environment/air_temperature.h"

#include "ps/math/transcendental.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace ps::environment {

// ---------------------------------------------------------------- descs

const char* surface_kind_name(SurfaceKind k) {
    switch (k) {
    case SurfaceKind::Static: return "static";
    case SurfaceKind::Sealed: return "sealed";
    case SurfaceKind::Soil: return "soil";
    case SurfaceKind::Vegetated: return "vegetated";
    }
    return "?";
}

const SurfaceClassDesc* SurfaceConditionsDesc::find(const std::string& name) const {
    for (const SurfaceClassDesc& c : classes)
        if (c.name == name) return &c;
    return nullptr;
}

namespace {

bool positive_finite(real v) { return std::isfinite(v) && v > 0.0; }

std::string layer_error(const ThermalLayerDesc& l, const std::string& where) {
    if (!positive_finite(l.depth_m)) return where + ".depth_m must be > 0";
    if (!positive_finite(l.k_w_mk)) return where + ".k_w_mk must be > 0";
    if (!positive_finite(l.rho_c_j_m3k)) return where + ".rho_c_j_m3k must be > 0";
    return {};
}

// Splits one slab of thickness L into geometrically growing cells starting at
// `first`; returns the cell count (0 = too many) and the scaled spacings.
int split_slab(real L, real first, std::array<real, kMaxColumnNodes>& sp) {
    int m = 0;
    real s = 0.0;
    real d = first;
    while (s < L) {
        if (m == kMaxColumnNodes) return 0;
        sp[static_cast<std::size_t>(m++)] = d;
        s += d;
        d *= kColumnCellGrowth;
    }
    // Drop an overshooting last cell when it lies mostly below the slab, then
    // rescale so the cells end exactly on the slab boundary.
    if (m > 1 && (s - L) > 0.5 * sp[static_cast<std::size_t>(m - 1)]) {
        s -= sp[static_cast<std::size_t>(m - 1)];
        --m;
    }
    const real scale = L / s;
    for (int j = 0; j < m; ++j) sp[static_cast<std::size_t>(j)] *= scale;
    return m;
}

// W6: the per-kind hydraulic fields (plan 4.4, 7.4). Returns an error string
// instead of throwing so the validation path can share it.
std::string hydraulic_error(const SurfaceClassDesc& d) {
    if (!(d.friction.damp > 0.0 && d.friction.damp <= 2.0)) return "friction.damp must be in (0, 2]";
    if (!(d.friction.wet > 0.0 && d.friction.wet <= 2.0)) return "friction.wet must be in (0, 2]";
    if (!(d.friction.wet_grass > 0.0 && d.friction.wet_grass <= 2.0)) return "friction.wet_grass must be in (0, 2]";
    switch (d.kind) {
    case SurfaceKind::Static:
        break;
    case SurfaceKind::Sealed:
        if (!(d.mtd_mm > 0.0)) return "a sealed class needs mtd_mm > 0";
        if (d.texture_storage_mm == 0.0 || d.texture_storage_mm < -1.0) return "texture_storage_mm must be > 0 or omitted";
        if (!(d.drainage_path_m > 0.0)) return "a sealed class needs drainage.path_m > 0";
        if (!(d.drainage_slope_m_m > 0.0)) return "a sealed class needs drainage.slope > 0";
        if (!(d.soil.ksat_mm_h == 0.0 && d.soil.theta_fc == 0.0 && d.soil.theta_sat == 0.0))
            return "a sealed class takes no soil fields";
        if (d.canopy_capacity_mm != 0.0) return "a sealed class takes no canopy field";
        break;
    case SurfaceKind::Soil:
        if (!(d.soil.ksat_mm_h > 0.0)) return "a soil class needs soil.ksat_mm_h > 0";
        if (!(d.soil.theta_fc > 0.0 && d.soil.theta_sat > d.soil.theta_fc && d.soil.theta_sat <= 1.0))
            return "a soil class needs 0 < soil.theta_fc < soil.theta_sat <= 1";
        if (d.canopy_capacity_mm != 0.0) return "a soil class takes no canopy field";
        if (d.mtd_mm != 0.0 || d.texture_storage_mm >= 0.0) return "a soil class takes no mtd/texture storage";
        break;
    case SurfaceKind::Vegetated:
        if (!(d.soil.ksat_mm_h > 0.0)) return "a vegetated class needs soil.ksat_mm_h > 0";
        if (!(d.soil.theta_fc > 0.0 && d.soil.theta_sat > d.soil.theta_fc && d.soil.theta_sat <= 1.0))
            return "a vegetated class needs 0 < soil.theta_fc < soil.theta_sat <= 1";
        if (!(d.canopy_capacity_mm > 0.0)) return "a vegetated class needs canopy.capacity_mm > 0";
        if (d.mtd_mm != 0.0 || d.texture_storage_mm >= 0.0) return "a vegetated class takes no mtd/texture storage";
        break;
    }
    return {};
}

// Builds the column; returns an error string instead of throwing so the
// validation path can share it.
std::string build_column_impl(const SurfaceClassDesc& d, ColumnModel& c) {
    if (d.kind == SurfaceKind::Static) return "a static class has no column";
    if (!(d.albedo >= 0.0 && d.albedo < 1.0)) return "albedo must be in [0, 1)";
    if (!(d.emissivity > 0.0 && d.emissivity <= 1.0)) return "emissivity must be in (0, 1]";
    std::string e = layer_error(d.layer, "layer");
    if (!e.empty()) return e;
    e = layer_error(d.deep, "deep");
    if (!e.empty()) return e;
    if (!(d.layer.depth_m + d.deep.depth_m <= kMaxColumnDepthM))
        return "layer.depth_m + deep.depth_m must be <= " + std::to_string(kMaxColumnDepthM) + " m";
    e = hydraulic_error(d);
    if (!e.empty()) return e;

    c = ColumnModel{};
    c.albedo = d.albedo;
    c.emissivity = d.emissivity;
    c.kind = d.kind;
    c.mtd_mm = d.mtd_mm;
    c.w_tex_mm = d.texture_storage_mm >= 0.0 ? d.texture_storage_mm : d.mtd_mm * kTexturePorosity;
    c.drainage_path_m = d.drainage_path_m;
    c.drainage_slope_m_m = d.drainage_slope_m_m;
    c.canopy_capacity_mm = d.canopy_capacity_mm;
    c.soil_ksat_mm_h = d.soil.ksat_mm_h;
    c.soil_theta_fc = d.soil.theta_fc;
    c.soil_theta_sat = d.soil.theta_sat;
    c.soil_top_depth_m = kSoilTopDepthM;
    c.friction = d.friction;
    // Cells: thickness, conductivity, heat capacity.
    std::array<real, kMaxColumnNodes> cell_dz{}, cell_k{}, cell_rc{};
    int cells = 0;
    real first = kColumnTopCellM;
    real z_top = 0.0;
    const ThermalLayerDesc* slabs[2] = {&d.layer, &d.deep};
    for (const ThermalLayerDesc* slab : slabs) {
        std::array<real, kMaxColumnNodes> sp{};
        const int m = split_slab(slab->depth_m, first, sp);
        if (m == 0 || cells + m > kMaxColumnNodes - 1)
            return "the column needs more than " + std::to_string(kMaxColumnNodes) +
                   " nodes (a slab too deep for the fixed cell growth)";
        for (int j = 0; j < m; ++j) {
            cell_dz[static_cast<std::size_t>(cells)] = sp[static_cast<std::size_t>(j)];
            cell_k[static_cast<std::size_t>(cells)] = slab->k_w_mk;
            cell_rc[static_cast<std::size_t>(cells)] = slab->rho_c_j_m3k;
            ++cells;
        }
        z_top += slab->depth_m;
        first = sp[static_cast<std::size_t>(m - 1)] * kColumnCellGrowth;
    }
    c.nodes = cells + 1;
    c.z_m[0] = 0.0;
    for (int j = 0; j < cells; ++j)
        c.z_m[static_cast<std::size_t>(j + 1)] = c.z_m[static_cast<std::size_t>(j)] + cell_dz[static_cast<std::size_t>(j)];
    c.z_m[static_cast<std::size_t>(cells)] = z_top;   // exact bottom (no rounding drift)
    for (int i = 0; i < c.nodes; ++i) {
        real cap = 0.0;
        if (i > 0) cap += 0.5 * cell_rc[static_cast<std::size_t>(i - 1)] * cell_dz[static_cast<std::size_t>(i - 1)];
        if (i < cells) cap += 0.5 * cell_rc[static_cast<std::size_t>(i)] * cell_dz[static_cast<std::size_t>(i)];
        c.cap_j_m2k[static_cast<std::size_t>(i)] = cap;
        if (i < cells)
            c.cond_w_m2k[static_cast<std::size_t>(i)] = cell_k[static_cast<std::size_t>(i)] / cell_dz[static_cast<std::size_t>(i)];
    }
    return {};
}

} // namespace

std::string surface_conditions_validation_error(const SurfaceConditionsDesc& d) {
    if (d.classes.size() > static_cast<std::size_t>(kMaxSurfaceClasses) * 4)
        return "surface_conditions: too many classes";
    for (std::size_t i = 0; i < d.classes.size(); ++i) {
        const SurfaceClassDesc& c = d.classes[i];
        const std::string where = "surface_conditions: surfaces." + (c.name.empty() ? std::string("<empty>") : c.name);
        if (c.name.empty()) return "surface_conditions: a class name is empty";
        if (i > 0 && !(d.classes[i - 1].name < c.name))
            return "surface_conditions: classes must be sorted by name and unique (" + c.name + ")";
        if (c.kind == SurfaceKind::Static) continue;
        ColumnModel col;
        const std::string e = build_column_impl(c, col);
        if (!e.empty()) return where + ": " + e;
    }
    return {};
}

void validate_surface_conditions(const SurfaceConditionsDesc& d) {
    const std::string e = surface_conditions_validation_error(d);
    if (!e.empty()) throw std::invalid_argument(e);
}

std::string condition_grid_validation_error(const ConditionGridDesc& g) {
    if (!(std::isfinite(g.dt_env_s) && g.dt_env_s > 0.0 && g.dt_env_s <= 3600.0))
        return "condition: dt_env_s must be in (0, 3600]";
    if (!(std::isfinite(g.spinup_s) && g.spinup_s >= g.dt_env_s && g.spinup_s <= 366.0 * 86400.0))
        return "condition: spinup_s must be in [dt_env_s, 366 days]";
    if (!(std::isfinite(g.max_fast_forward_s) && g.max_fast_forward_s >= g.dt_env_s &&
          g.max_fast_forward_s <= 366.0 * 86400.0))
        return "condition: max_fast_forward_s must be in [dt_env_s, 366 days]";
    return {};
}

ColumnModel build_column(const SurfaceClassDesc& desc) {
    ColumnModel c;
    const std::string e = build_column_impl(desc, c);
    if (!e.empty()) throw std::invalid_argument("surface_conditions: surfaces." + desc.name + ": " + e);
    return c;
}

// ---------------------------------------------------------------- physics

real sky_emissivity(real vapour_pressure_hpa, real cloud_cover) {
    // Geiger 1959 (via Solaimanian & Kennedy 1993): eps = G - J 10^(-P p),
    // G 0.77, J 0.28, P 0.074, p in mmHg. 10^x = exp(x ln 10).
    real p_mmhg = vapour_pressure_hpa * 0.750061683;   // 1 hPa = 0.750061683 mmHg [C definition]
    if (!(p_mmhg > 0.0)) p_mmhg = 0.0;
    const real ln10 = 2.302585092994045684;
    const real eps_clear = 0.77 - 0.28 * math::exp(-0.074 * p_mmhg * ln10);
    real n = cloud_cover;
    if (!(n > 0.0)) n = 0.0;
    if (n > 1.0) n = 1.0;
    return n + (1.0 - n) * eps_clear;   // Crawford & Duchon 1999 [M]
}

real convection_coefficient_w_m2k(real surface_temp_k, real air_temp_k, real wind_m_s) {
    real u = wind_m_s;
    if (!(u > 0.0)) u = 0.0;
    const real tm = 0.5 * (surface_temp_k + air_temp_k);
    real forced = 0.0;
    if (u > 0.0) forced = 0.00144 * math::pow(tm, 0.3) * math::pow(u, 0.7);
    real free = 0.0;
    const real dt = surface_temp_k - air_temp_k;
    if (dt > 0.0) free = 0.00097 * math::pow(dt, 0.3);
    return 698.24 * (forced + free);
}

real vapour_density_kg_m3(real vapour_pressure_hpa, real temp_k) {
    // Ideal gas: rho_v = e M_w / (R T) with e in Pa:
    // rho_v = e_pa * 0.018016 / (8.314462618 * T).
    const real e_pa = vapour_pressure_hpa * 100.0;
    if (!(e_pa > 0.0) || !(temp_k > 0.0)) return 0.0;
    return e_pa * 0.018016 / (8.314462618 * temp_k);
}

real gallaway_wfd_mm(real mtd_mm, real path_m, real intensity_mm_h, real slope_m_m) {
    if (!(intensity_mm_h > 0.0) || !(mtd_mm > 0.0) || !(path_m > 0.0) || !(slope_m_m > 0.0)) return 0.0;
    const real wfd = 0.01485 * math::pow(mtd_mm, 0.11) * math::pow(path_m, 0.43) *
                         math::pow(intensity_mm_h, 0.59) / math::pow(slope_m_m, 0.42) -
                     mtd_mm;
    return wfd > 0.0 ? wfd : 0.0;
}

VapourExchange surface_vapour_exchange(const ColumnModel& col, const SurfaceForcing& f, const SurfaceClassState& in,
                                       real dt) {
    VapourExchange x;
    if (!(dt > 0.0)) return x;
    const real t0 = in.temp_k[0];
    const real hc = convection_coefficient_w_m2k(t0, f.air_temp_k, f.wind_m_s);
    // Dalton bulk formula via the Lewis analogy (Le^(2/3) = 1 [AS]): the mass
    // transfer coefficient h_m = h_c / (rho cp) [m/s]; the Vehrencamp h_c
    // supplies the wind function (plan 4.4 [D]).
    const real h_mass = hc / (kAirDensityKgM3 * kAirCpJkgK);
    const real rho_v_air = vapour_density_kg_m3(f.vapour_pressure_hpa, f.air_temp_k);
    const real rho_v_sat = vapour_density_kg_m3(saturation_vapour_pressure_hpa(t0 - kCelsiusToKelvin), t0);
    const real e_pot = h_mass * (rho_v_sat - rho_v_air);   // kg/(m2 s); > 0 evaporates, < 0 condenses
    if (e_pot < 0.0) {
        // W6 models liquid only: below freezing there is no liquid dew (frost is W8).
        if (t0 > kCelsiusToKelvin) x.dew = std::min(-e_pot, kMaxCondensationKgM2S);
        return x;
    }
    // Evaporation draws the free water first, then the canopy store, then the
    // soil surface (scaled by theta / theta_fc, plan 4.4); each limited by the
    // water that is there at the start of the step.
    real rem = e_pot;
    x.from_water = std::min(rem, in.water_mm / dt);
    rem -= x.from_water;
    if (col.kind == SurfaceKind::Vegetated) {
        x.from_canopy = std::min(rem, in.canopy_water_mm / dt);
        rem -= x.from_canopy;
    }
    if ((col.kind == SurfaceKind::Soil || col.kind == SurfaceKind::Vegetated) && col.soil_theta_fc > 0.0) {
        const real scale = std::min(in.soil_moisture / col.soil_theta_fc, 1.0);
        x.from_soil = std::min(rem * scale, in.soil_moisture * col.soil_top_depth_m * 1000.0 / dt);
    }
    return x;
}

real surface_vapour_flux_kg_m2_s(const ColumnModel& col, const SurfaceForcing& f, const SurfaceClassState& in,
                                 real dt) {
    return surface_vapour_exchange(col, f, in, dt).net();
}

void initial_column_state(const ColumnModel& column, const InitialProfile& p, SurfaceClassState& out) {
    out = SurfaceClassState{};
    const int n = column.nodes;
    const real depth = column.z_m[static_cast<std::size_t>(n - 1)];
    for (int i = 0; i < n; ++i) {
        const real f = column.z_m[static_cast<std::size_t>(i)] / depth;
        out.temp_k[static_cast<std::size_t>(i)] = p.surface_temp_k + (p.bottom_temp_k - p.surface_temp_k) * f;
    }
    out.temp_k[static_cast<std::size_t>(n - 1)] = p.bottom_temp_k;
    // W6: soil and turf start at field capacity (a state with no gravity water and no
    // ponding, unscaled evaporation) [AS]; the spin-up's weather moves it from there.
    if (column.kind == SurfaceKind::Soil || column.kind == SurfaceKind::Vegetated) out.soil_moisture = column.soil_theta_fc;
}

void step_column(const ColumnModel& col, const SurfaceForcing& f, real dt, const SurfaceClassState& in,
                 SurfaceClassState& out) {
    const int n = col.nodes;
    const int m = n - 1;   // unknowns 0..n-2; node n-1 is the Dirichlet bottom
    const real t0 = in.temp_k[0];
    const real ta = f.air_temp_k;
    const real eps = col.emissivity;
    const real ta2 = ta * ta;
    const real t02 = t0 * t0;
    const real l_sky = sky_emissivity(f.vapour_pressure_hpa, f.cloud_cover) * kStefanBoltzmann * ta2 * ta2;
    const real hc = convection_coefficient_w_m2k(t0, ta, f.wind_m_s);
    // W6: the latent flux is evaluated at the OLD state and folded into the
    // energy balance as a lagged constant (plan 4.4), so the solve stays one
    // linear system and the energy balance stays closed.
    const VapourExchange ex = surface_vapour_exchange(col, f, in, dt);
    const real evap_kg_m2_s = ex.net();
    const real L_v = kLatentHeatVaporisation - kLatentHeatSlope * (t0 - kCelsiusToKelvin);
    const real q_latent = L_v * evap_kg_m2_s;   // W/m2 out of the surface
    // q(T) ~ A - B' T with T^4 linearised about t0.
    const real a_flux =
        (1.0 - col.albedo) * f.ghi_w_m2 + eps * l_sky + 3.0 * eps * kStefanBoltzmann * t02 * t02 + hc * ta - q_latent;
    const real b_flux = 4.0 * eps * kStefanBoltzmann * t02 * t0 + hc;

    // Tridiagonal: lo[i] T[i-1] + di[i] T[i] + up[i] T[i+1] = rhs[i].
    std::array<real, kMaxColumnNodes> cp{}, dp{};
    const real inv_dt = 1.0 / dt;
    for (int i = 0; i < m; ++i) {
        const std::size_t u = static_cast<std::size_t>(i);
        const real cap = col.cap_j_m2k[u] * inv_dt;
        const real k_up = i > 0 ? col.cond_w_m2k[u - 1] : 0.0;
        const real k_dn = col.cond_w_m2k[u];
        real di = cap + k_up + k_dn;
        real rhs = cap * in.temp_k[u];
        if (i == 0) {
            di += b_flux;
            rhs += a_flux;
        }
        const real lo = -k_up;
        real up = -k_dn;
        if (i == m - 1) {
            rhs += k_dn * f.bottom_temp_k;   // the Dirichlet node moves to the right-hand side
            up = 0.0;
        }
        // Thomas forward sweep.
        if (i == 0) {
            cp[u] = up / di;
            dp[u] = rhs / di;
        } else {
            const real denom = di - lo * cp[u - 1];
            cp[u] = up / denom;
            dp[u] = (rhs - lo * dp[u - 1]) / denom;
        }
    }
    // Back substitution (writes `out` last, so out may alias in: in.temp_k
    // was read completely above).
    std::array<real, kMaxColumnNodes> t{};
    t[static_cast<std::size_t>(m - 1)] = dp[static_cast<std::size_t>(m - 1)];
    for (int i = m - 2; i >= 0; --i) {
        const std::size_t u = static_cast<std::size_t>(i);
        t[u] = dp[u] - cp[u] * t[u + 1];
    }
    t[static_cast<std::size_t>(n - 1)] = f.bottom_temp_k;

    // --- W6 mass update (plan 4.4): rain, interception, infiltration,
    // evaporation/dew, drainage. W in mm; 1 kg/m2 == 1 mm. The evaporation and
    // dew amounts are exactly the ones whose latent heat the energy balance
    // above used (ex), so mass and energy stay consistent.
    real W = in.water_mm;
    real C = in.canopy_water_mm;
    real theta = in.soil_moisture;
    const bool soil_like = col.kind == SurfaceKind::Soil || col.kind == SurfaceKind::Vegetated;
    const real layer_mm = soil_like ? col.soil_top_depth_m * 1000.0 : 0.0;   // mm of water per unit volumetric theta
    real input = f.precip_mm_h / 3600.0 * dt;   // liquid water this step

    if (col.kind == SurfaceKind::Vegetated && input > 0.0) {
        const real room = col.canopy_capacity_mm - C;
        if (room > 0.0) {
            const real take = input < room ? input : room;
            C += take;
            input -= take;
        }
    }
    if (soil_like && input > 0.0) {
        const real infil_cap = col.soil_ksat_mm_h / 3600.0 * dt;
        real pore = (col.soil_theta_sat - theta) * layer_mm;
        if (pore < 0.0) pore = 0.0;
        real take = input < infil_cap ? input : infil_cap;
        if (take > pore) take = pore;
        if (take > 0.0) {
            theta += take / layer_mm;
            input -= take;
        }
    }
    W += input;

    W -= ex.from_water * dt;
    C -= ex.from_canopy * dt;
    if (soil_like) theta -= ex.from_soil * dt / layer_mm;
    if (ex.dew > 0.0) {
        // Dew wets the leaves first (vegetated), the rest the surface film.
        real dew = ex.dew * dt;
        if (col.kind == SurfaceKind::Vegetated) {
            const real room = col.canopy_capacity_mm - C;
            const real take = room > 0.0 ? (dew < room ? dew : room) : 0.0;
            C += take;
            dew -= take;
        }
        W += dew;
    }

    // Drainage (plan 4.4): sealed water above the texture storage relaxes
    // toward the Gallaway steady film with tau = WFD / R (the fixed point of
    // W - W_tex under the input R is R tau = WFD exactly); water below the
    // storage does not drain. Soil/vegetated drain gravity water above field
    // capacity and ponding.
    real tau = in.drain_tau_s;
    if (col.kind == SurfaceKind::Sealed) {
        const real R = f.precip_mm_h;
        if (R >= kDrainRainMinMmH) {
            const real wfd = gallaway_wfd_mm(col.mtd_mm, col.drainage_path_m, R, col.drainage_slope_m_m);
            tau = wfd > 0.0 ? wfd / (R / 3600.0) : 0.0;
        }
        if (W > col.w_tex_mm) {
            if (tau > 0.0) W = col.w_tex_mm + (W - col.w_tex_mm) / (1.0 + dt / tau);   // backward Euler
            else W = col.w_tex_mm;
        }
    } else if (soil_like) {
        const real excess = theta - col.soil_theta_fc;
        if (excess > 0.0) theta = col.soil_theta_fc + excess / (1.0 + dt / kSoilTauFcS);
        if (W > 0.0) W = W / (1.0 + dt / kPondTauS);
    }
    if (W < 0.0) W = 0.0;
    if (C < 0.0) C = 0.0;
    if (theta < 0.0) theta = 0.0;
    if (soil_like && theta > col.soil_theta_sat) theta = col.soil_theta_sat;

    out.water_mm = W;
    out.canopy_water_mm = C;
    out.soil_moisture = theta;
    out.drain_tau_s = tau;
    out.ice_mm = in.ice_mm;
    out.frost_mm = in.frost_mm;
    out.snow_mm = in.snow_mm;
    out.snow_density_kg_m3 = in.snow_density_kg_m3;
    for (int i = 0; i < n; ++i) out.temp_k[static_cast<std::size_t>(i)] = t[static_cast<std::size_t>(i)];
    for (int i = n; i < kMaxColumnNodes; ++i) out.temp_k[static_cast<std::size_t>(i)] = 0.0;
}

// ---------------------------------------------------------------- grid

ConditionGrid::ConditionGrid(const SurfaceConditionsDesc& desc, const std::vector<std::string>& surface_names,
                             const ConditionGridDesc& grid)
    : grid_(grid) {
    validate_surface_conditions(desc);
    const std::string ge = condition_grid_validation_error(grid);
    if (!ge.empty()) throw std::invalid_argument(ge);

    if (surface_names.empty()) {
        for (const SurfaceClassDesc& c : desc.classes) names_.push_back(c.name);
    } else {
        names_ = surface_names;
    }
    if (names_.size() > static_cast<std::size_t>(kMaxSurfaceClasses))
        throw std::invalid_argument("surface conditions: more than " + std::to_string(kMaxSurfaceClasses) +
                                    " surface classes");
    for (std::size_t i = 0; i < names_.size(); ++i) {
        if (names_[i].empty()) throw std::invalid_argument("surface conditions: an empty surface name");
        for (std::size_t j = 0; j < i; ++j)
            if (names_[j] == names_[i])
                throw std::invalid_argument("surface conditions: duplicate surface name '" + names_[i] + "'");
    }
    count_ = static_cast<int>(names_.size());
    for (const std::string& name : names_) {
        const SurfaceClassDesc* c = desc.find(name);
        if (!c) {
            unmapped_.push_back(name);
            kinds_.push_back(SurfaceKind::Static);
            columns_.push_back(ColumnModel{});
            continue;
        }
        kinds_.push_back(c->kind);
        columns_.push_back(c->kind == SurfaceKind::Static ? ColumnModel{} : build_column(*c));
    }
    for (const SurfaceClassDesc& c : desc.classes)
        if (std::find(names_.begin(), names_.end(), c.name) == names_.end()) unused_.push_back(c.name);

    spinup_steps_ = static_cast<std::int64_t>(std::ceil(grid_.spinup_s / grid_.dt_env_s));
    max_ff_steps_ = static_cast<std::int64_t>(std::floor(grid_.max_fast_forward_s / grid_.dt_env_s));
    if (max_ff_steps_ < 1) max_ff_steps_ = 1;
}

std::int64_t ConditionGrid::grid_index_at(real t_env) const {
    return static_cast<std::int64_t>(std::floor(t_env / grid_.dt_env_s));
}

void ConditionGrid::step_all(const ISurfaceForcingSource& src) {
    // S[k+1] -> S[k+2] with the forcing at (k + 3/2) dt.
    const real dt = grid_.dt_env_s;
    const SurfaceForcing f = src.forcing_at((static_cast<real>(k_) + 1.5) * dt);
    for (int i = 0; i < count_; ++i) {
        const std::size_t u = static_cast<std::size_t>(i);
        if (kinds_[u] == SurfaceKind::Static) continue;
        prev_[u] = next_[u];
        step_column(columns_[u], f, dt, prev_[u], next_[u]);
    }
    ++k_;
}

std::int64_t ConditionGrid::spin_up(std::int64_t k_target, const ISurfaceForcingSource& src) {
    const real dt = grid_.dt_env_s;
    const std::int64_t k0 = k_target - spinup_steps_;
    const InitialProfile p = src.initial_at(static_cast<real>(k0) * dt);
    const SurfaceForcing f = src.forcing_at((static_cast<real>(k0) + 0.5) * dt);
    for (int i = 0; i < count_; ++i) {
        const std::size_t u = static_cast<std::size_t>(i);
        prev_[u] = SurfaceClassState{};
        next_[u] = SurfaceClassState{};
        if (kinds_[u] == SurfaceKind::Static) continue;
        initial_column_state(columns_[u], p, prev_[u]);
        step_column(columns_[u], f, dt, prev_[u], next_[u]);
    }
    k_ = k0;
    initialised_ = true;
    std::int64_t steps = 1;
    while (k_ < k_target) {
        step_all(src);
        ++steps;
    }
    return steps;
}

std::int64_t ConditionGrid::advance_to(real t_env, const ISurfaceForcingSource& src) {
    const std::int64_t kt = grid_index_at(t_env);
    if (!initialised_) return spin_up(kt, src);
    if (kt == k_) return 0;
    if (kt > k_ && kt - k_ <= max_ff_steps_) {
        std::int64_t steps = 0;
        while (k_ < kt) {
            step_all(src);
            ++steps;
        }
        return steps;
    }
    return spin_up(kt, src);
}

real ConditionGrid::weight_at(real t_env) const {
    const real dt = grid_.dt_env_s;
    real w = (t_env - static_cast<real>(k_) * dt) / dt;
    if (!(w > 0.0)) w = 0.0;
    if (w > 1.0) w = 1.0;
    return w;
}

real ConditionGrid::surface_temp_k(int i, real t_env) const {
    const std::size_t u = static_cast<std::size_t>(i);
    if (kinds_[u] == SurfaceKind::Static || !initialised_) return 0.0;
    const real w = weight_at(t_env);
    return prev_[u].temp_k[0] + (next_[u].temp_k[0] - prev_[u].temp_k[0]) * w;
}

ConditionCheckpoint ConditionGrid::checkpoint() const {
    ConditionCheckpoint c;
    c.valid = initialised_;
    c.grid_index = k_;
    c.class_count = count_;
    c.dt_env_s = grid_.dt_env_s;
    c.prev = prev_;
    c.next = next_;
    return c;
}

void ConditionGrid::restore(const ConditionCheckpoint& c) {
    if (!c.valid) throw std::invalid_argument("ConditionGrid::restore: the checkpoint is not valid");
    if (c.class_count != count_)
        throw std::invalid_argument("ConditionGrid::restore: class count " + std::to_string(c.class_count) +
                                    " does not match " + std::to_string(count_));
    if (c.dt_env_s != grid_.dt_env_s)
        throw std::invalid_argument("ConditionGrid::restore: dt_env_s does not match");
    k_ = c.grid_index;
    prev_ = c.prev;
    next_ = c.next;
    initialised_ = true;
}

namespace {
void fold_bytes(std::uint64_t& h, const void* p, std::size_t n) {
    const auto* b = static_cast<const unsigned char*>(p);
    for (std::size_t i = 0; i < n; ++i) {
        h ^= b[i];
        h *= 1099511628211ULL;
    }
}
template <class T>
void fold_value(std::uint64_t& h, const T& v) { fold_bytes(h, &v, sizeof(T)); }

void fold_state(std::uint64_t& h, const SurfaceClassState& s, int nodes) {
    for (int i = 0; i < nodes; ++i) fold_value(h, s.temp_k[static_cast<std::size_t>(i)]);
    fold_value(h, s.water_mm);
    fold_value(h, s.ice_mm);
    fold_value(h, s.frost_mm);
    fold_value(h, s.snow_mm);
    fold_value(h, s.snow_density_kg_m3);
    fold_value(h, s.soil_moisture);
    fold_value(h, s.canopy_water_mm);
    fold_value(h, s.drain_tau_s);
}
} // namespace

std::uint64_t ConditionGrid::fold_hash(std::uint64_t h) const {
    fold_value(h, static_cast<std::int32_t>(initialised_ ? 1 : 0));
    fold_value(h, k_);
    fold_value(h, static_cast<std::int32_t>(count_));
    for (int i = 0; i < count_; ++i) {
        const std::size_t u = static_cast<std::size_t>(i);
        fold_value(h, static_cast<std::int32_t>(kinds_[u]));
        if (kinds_[u] == SurfaceKind::Static) continue;
        fold_state(h, prev_[u], columns_[u].nodes);
        fold_state(h, next_[u], columns_[u].nodes);
    }
    return h;
}

} // namespace ps::environment
