// drivetrain/nitrous_flow.h - INTERNAL (N2O plan N0, section 2.2). The N2O
// property lookups and the two-phase jet mass-flux tables, built ONCE at
// load from an OxidizerDesc (ps::io::load_oxidizer_json) and immutable after:
// every runtime accessor is const, allocation-free and uses only + - * / and
// sqrt plus binary-search table interpolation (PLAN.md D7/D8: no libm
// transcendentals, no unordered iteration).
//
// Units inside this class are SI: pressure Pa (absolute), temperature K,
// density kg/m^3, enthalpy J/kg, entropy J/(kg*K), mass flux kg/(m^2*s).
//
// Flow models (docs/data_formats/oxidizer.md, plan decision D5):
//  - HEM (default): the critical (choked) mass flux of the SATURATED LIQUID at
//    bottle pressure through an isentropic homogeneous-equilibrium expansion.
//    For each saturation row used as the bottle state (s1 = s_l, h1 = h_l) the
//    downstream pressure is swept down the saturation table (rows below the
//    bottle row) and the flux rho2*sqrt(2*(h1 - h2)) is maximised, with the
//    two-phase state at p2 from the isentropic quality
//    x2 = (s1 - s_l(p2))/(s_v(p2) - s_l(p2)) and the homogeneous density
//    1/rho2 = (1 - x2)/rho_l + x2/rho_v. The result is tabulated over the
//    bottle pressure (the rows' pressures); a runtime query interpolates
//    linearly and clamps at both ends. Manifold pressures (<= a few bar) are
//    far below the choke pressure (about 44-47 bar for a 900-1000 psig
//    bottle), so the jet is choked and no downstream-pressure axis exists.
//  - Dyer NHNE: G = 1/(1+k) * G_SPI + k/(1+k) * G_HEM with k = 1 for a
//    SATURATED inlet (k = sqrt((p1 - p2)/(p_v - p2)) with p_v = p1), i.e.
//    0.5 * sqrt(2*rho_l*(p1 - p_down)) + 0.5 * G_HEM. The SPI term is the
//    one runtime sqrt and uses the current downstream pressure.
#pragma once

#include "ps/drivetrain/oxidizer_desc.h"
#include "ps/types.h"

#include <cstdint>
#include <vector>

namespace ps::drivetrain {

// Saturated-liquid state at a bottle pressure or temperature.
struct SaturatedLiquid {
    real t_k = 0.0;
    real p_pa = 0.0;
    real rho_l = 0.0;       // kg/m^3
    real h_l_j_kg = 0.0;    // J/kg
    real s_l_j_kgk = 0.0;   // J/(kg*K)
};

class OxidizerTables {
public:
    // Load time: converts to SI and builds the HEM tables (O(n^2) in the
    // saturation row count, a few hundred microseconds).
    explicit OxidizerTables(const OxidizerDesc& desc);

    [[nodiscard]] real molar_mass_kg_mol() const { return molar_mass_kg_mol_; }
    // Heat released per kg of N2O by 2 N2O -> 2 N2 + O2 (J/kg) = dHf / M.
    [[nodiscard]] real decomposition_j_per_kg() const { return decomposition_j_per_kg_; }
    [[nodiscard]] real oxygen_mass_fraction() const { return oxygen_mass_fraction_; }

    // Saturation range of the table (Pa / K). A bottle state must lie inside it.
    [[nodiscard]] real p_min_pa() const { return p_pa_.front(); }
    [[nodiscard]] real p_max_pa() const { return p_pa_.back(); }
    [[nodiscard]] real t_min_k() const { return t_k_.front(); }
    [[nodiscard]] real t_max_k() const { return t_k_.back(); }
    [[nodiscard]] std::size_t row_count() const { return t_k_.size(); }

    // Saturated liquid at an absolute pressure / temperature (linear in the
    // table's own axis, clamped at both ends).
    [[nodiscard]] SaturatedLiquid liquid_at_pressure(real p_pa) const;
    [[nodiscard]] SaturatedLiquid liquid_at_temperature(real t_k) const;
    // p_sat(T), Pa.
    [[nodiscard]] real saturation_pressure_pa(real t_k) const;

    // Gas enthalpy on the 1 bar isobar (J/kg), linear in T, clamped.
    [[nodiscard]] real gas_enthalpy_j_kg(real t_k) const;

    // HEM critical mass flux (kg/(m^2*s)) and the choke (throat) pressure
    // (Pa abs) for a saturated-liquid bottle at p_bottle_pa.
    [[nodiscard]] real g_hem(real p_bottle_pa) const;
    [[nodiscard]] real choke_pressure_pa(real p_bottle_pa) const;
    // Single-phase incompressible flux sqrt(2*rho_l*(p1 - p_down)), rho_l at p1; 0 when p_down >= p1.
    [[nodiscard]] real g_spi(real p1_pa, real p_down_pa) const;
    // Dyer NHNE flux for a saturated inlet: 0.5*G_SPI + 0.5*G_HEM.
    [[nodiscard]] real g_dyer(real p1_pa, real p_down_pa) const;

    // FNV-1a over every table value, in a fixed order - folded once into the
    // state hash of a vehicle that has a nitrous kit (like maps_->content_hash).
    [[nodiscard]] std::uint64_t content_hash() const { return content_hash_; }

private:
    real molar_mass_kg_mol_ = 0.0;
    real decomposition_j_per_kg_ = 0.0;
    real oxygen_mass_fraction_ = 0.0;

    // Saturation columns (SI), ascending in T and p.
    std::vector<real> t_k_, p_pa_, rho_l_, rho_v_, h_l_, h_v_, s_l_, s_v_;
    // Per bottle row: the HEM flux and the throat pressure.
    std::vector<real> g_hem_, choke_pa_;
    std::vector<real> gas_t_k_, gas_h_;
    std::uint64_t content_hash_ = 0;
};

} // namespace ps::drivetrain
