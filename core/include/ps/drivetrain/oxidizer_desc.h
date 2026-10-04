// ps/drivetrain/oxidizer_desc.h - public, additive: oxidizer fluid properties
// for the nitrous-oxide system (N2O plan N0). Pure data, no algorithm code
// (same seam as FuelDesc/fuel_desc.h): a standalone descriptor loaded from
// data/oxidizers/*.json via ps::io::load_oxidizer_json
// (ps/io/nitrous_io.h, core/src/io/nitrous_io.cpp). The derived flow tables
// (HEM/Dyer, saturation lookups) are internal: core/src/drivetrain/nitrous_flow.h.
//
// Units are the file's own (bar, K, kg/m^3, kJ/kg, J/(g*K)); the internal
// tables convert to SI once at load. Saturation arrays are indexed by one
// common row index and ascend in temperature (and therefore in pressure).

#pragma once

#include "ps/types.h"

#include <string>
#include <vector>

namespace ps::drivetrain {

struct OxidizerSaturationTable {
    std::vector<real> t_k;            // strictly ascending
    std::vector<real> p_bar;          // saturation pressure, absolute, strictly ascending
    std::vector<real> rho_liquid;     // kg/m^3
    std::vector<real> rho_vapour;     // kg/m^3
    std::vector<real> h_liquid_kj_kg; // same reference state as h_vapour
    std::vector<real> h_vapour_kj_kg;
    std::vector<real> s_liquid_j_gk;  // J/(g*K), same reference state as s_vapour
    std::vector<real> s_vapour_j_gk;
};

// Gas enthalpy on the 1 bar isobar (the cooling model's gas side).
struct OxidizerGasIsobar {
    std::vector<real> t_k;     // strictly ascending, at least 2 rows
    std::vector<real> h_kj_kg; // same reference state as the saturation table
};

struct OxidizerDesc {
    std::string name;                     // informational
    real molar_mass_g_mol = 0.0;          // > 0
    real formation_enthalpy_kj_mol = 0.0; // gas, 298 K; N2O decomposition releases this per mole
    real oxygen_mass_fraction = 0.0;      // O2 mass fraction after decomposition, in (0, 1)
    OxidizerSaturationTable saturation;
    OxidizerGasIsobar gas_1bar;
    std::string source;                   // required, non-empty (PLAN.md section 8)
};

} // namespace ps::drivetrain
