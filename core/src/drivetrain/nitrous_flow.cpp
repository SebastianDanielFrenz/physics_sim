#include "nitrous_flow.h"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace ps::drivetrain {
namespace {

constexpr std::uint64_t kFnvOffset = 14695981039346656037ULL;
constexpr std::uint64_t kFnvPrime = 1099511628211ULL;

void fold_real(std::uint64_t& h, real v) {
    const auto* bytes = reinterpret_cast<const unsigned char*>(&v);
    for (std::size_t i = 0; i < sizeof(v); ++i) {
        h ^= bytes[i];
        h *= kFnvPrime;
    }
}

void fold_vector(std::uint64_t& h, const std::vector<real>& v) {
    for (real x : v) fold_real(h, x);
}

// Linear interpolation of ys over ascending xs; clamps at both ends. No
// allocation; binary search (std::upper_bound) so the cost does not grow
// with the table.
real interp(const std::vector<real>& xs, const std::vector<real>& ys, real x) {
    if (x <= xs.front()) return ys.front();
    if (x >= xs.back()) return ys.back();
    const auto it = std::upper_bound(xs.begin(), xs.end(), x);
    const std::size_t hi = static_cast<std::size_t>(it - xs.begin());
    const std::size_t lo = hi - 1;
    const real t = (x - xs[lo]) / (xs[hi] - xs[lo]);
    return ys[lo] + t * (ys[hi] - ys[lo]);
}

} // namespace

OxidizerTables::OxidizerTables(const OxidizerDesc& desc) {
    const auto& s = desc.saturation;
    const std::size_t n = s.t_k.size();
    molar_mass_kg_mol_ = desc.molar_mass_g_mol * real(1e-3);
    // kJ/mol -> J/mol -> J/kg: dHf [kJ/mol] * 1000 / (M [g/mol] * 1e-3)
    decomposition_j_per_kg_ = desc.formation_enthalpy_kj_mol * real(1000.0) / molar_mass_kg_mol_;
    oxygen_mass_fraction_ = desc.oxygen_mass_fraction;

    t_k_ = s.t_k;
    p_pa_.resize(n);
    rho_l_ = s.rho_liquid;
    rho_v_ = s.rho_vapour;
    h_l_.resize(n);
    h_v_.resize(n);
    s_l_.resize(n);
    s_v_.resize(n);
    for (std::size_t i = 0; i < n; ++i) {
        p_pa_[i] = s.p_bar[i] * real(1.0e5);
        h_l_[i] = s.h_liquid_kj_kg[i] * real(1000.0);
        h_v_[i] = s.h_vapour_kj_kg[i] * real(1000.0);
        s_l_[i] = s.s_liquid_j_gk[i] * real(1000.0);
        s_v_[i] = s.s_vapour_j_gk[i] * real(1000.0);
    }
    gas_t_k_ = desc.gas_1bar.t_k;
    gas_h_.resize(desc.gas_1bar.h_kj_kg.size());
    for (std::size_t i = 0; i < gas_h_.size(); ++i) gas_h_[i] = desc.gas_1bar.h_kj_kg[i] * real(1000.0);

    // HEM critical flux per bottle row (see the header comment).
    g_hem_.assign(n, real(0.0));
    choke_pa_.assign(n, real(0.0));
    for (std::size_t i = 0; i < n; ++i) {
        const real s1 = s_l_[i];
        const real h1 = h_l_[i];
        real best = real(0.0);
        real best_p2 = p_pa_[i];
        for (std::size_t j = 0; j < i; ++j) {
            const real x2 = (s1 - s_l_[j]) / (s_v_[j] - s_l_[j]);
            if (!(x2 >= real(0.0) && x2 <= real(1.0))) continue;
            const real h2 = h_l_[j] + x2 * (h_v_[j] - h_l_[j]);
            const real dh = h1 - h2;
            if (!(dh > real(0.0))) continue;
            const real rho2 = real(1.0) / ((real(1.0) - x2) / rho_l_[j] + x2 / rho_v_[j]);
            const real g = rho2 * std::sqrt(real(2.0) * dh);
            if (g > best) {
                best = g;
                best_p2 = p_pa_[j];
            }
        }
        g_hem_[i] = best;
        choke_pa_[i] = best_p2;
    }

    std::uint64_t h = kFnvOffset;
    fold_real(h, molar_mass_kg_mol_);
    fold_real(h, decomposition_j_per_kg_);
    fold_real(h, oxygen_mass_fraction_);
    fold_vector(h, t_k_);
    fold_vector(h, p_pa_);
    fold_vector(h, rho_l_);
    fold_vector(h, rho_v_);
    fold_vector(h, h_l_);
    fold_vector(h, h_v_);
    fold_vector(h, s_l_);
    fold_vector(h, s_v_);
    fold_vector(h, g_hem_);
    fold_vector(h, choke_pa_);
    fold_vector(h, gas_t_k_);
    fold_vector(h, gas_h_);
    content_hash_ = h;
}

SaturatedLiquid OxidizerTables::liquid_at_pressure(real p_pa) const {
    SaturatedLiquid r;
    r.p_pa = std::clamp(p_pa, p_pa_.front(), p_pa_.back());
    r.t_k = interp(p_pa_, t_k_, r.p_pa);
    r.rho_l = interp(p_pa_, rho_l_, r.p_pa);
    r.h_l_j_kg = interp(p_pa_, h_l_, r.p_pa);
    r.s_l_j_kgk = interp(p_pa_, s_l_, r.p_pa);
    return r;
}

SaturatedLiquid OxidizerTables::liquid_at_temperature(real t_k) const {
    SaturatedLiquid r;
    r.t_k = std::clamp(t_k, t_k_.front(), t_k_.back());
    r.p_pa = interp(t_k_, p_pa_, r.t_k);
    r.rho_l = interp(t_k_, rho_l_, r.t_k);
    r.h_l_j_kg = interp(t_k_, h_l_, r.t_k);
    r.s_l_j_kgk = interp(t_k_, s_l_, r.t_k);
    return r;
}

real OxidizerTables::saturation_pressure_pa(real t_k) const { return interp(t_k_, p_pa_, t_k); }

real OxidizerTables::gas_enthalpy_j_kg(real t_k) const { return interp(gas_t_k_, gas_h_, t_k); }

real OxidizerTables::g_hem(real p_bottle_pa) const { return interp(p_pa_, g_hem_, p_bottle_pa); }

real OxidizerTables::choke_pressure_pa(real p_bottle_pa) const { return interp(p_pa_, choke_pa_, p_bottle_pa); }

real OxidizerTables::g_spi(real p1_pa, real p_down_pa) const {
    const real dp = p1_pa - p_down_pa;
    if (!(dp > real(0.0))) return real(0.0);
    return std::sqrt(real(2.0) * interp(p_pa_, rho_l_, p1_pa) * dp);
}

real OxidizerTables::g_dyer(real p1_pa, real p_down_pa) const {
    return real(0.5) * g_spi(p1_pa, p_down_pa) + real(0.5) * g_hem(p1_pa);
}

} // namespace ps::drivetrain
