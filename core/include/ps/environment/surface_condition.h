// ps/environment/surface_condition.h - the surface condition core (weather
// plan 4.1-4.3, 4.7; W5): per-class state on a fixed environment-time grid and
// the road/ground surface temperature energy balance.
//
// Thermal model (W5 decisions D1-D5, docs/data_formats/surface_conditions.md):
// per class a 1D vertex-centred finite-volume conduction column (node 0 is
// the surface, geometric cell growth from kColumnTopCellM, the top/deep slab
// boundary is a node, Dirichlet at the bottom = the climate's per-day
// ground_temp_c), stepped by backward Euler with a tridiagonal (Thomas)
// solve. The surface node receives
//     q = (1 - albedo) G + eps eps_sky sigma T_a^4 - eps sigma T_s^4 + h_c (T_a - T_s)
// with T_s^4 linearised about the old surface temperature and h_c (Vehrencamp
// 1953, as used by Solaimanian & Kennedy 1993) evaluated at the old state, so
// one step is one linear solve [D: unconditionally stable, deterministic].
// Sky emissivity: Geiger/Angstrom clear-sky form (Solaimanian & Kennedy 1993)
// raised toward 1 with cloud cover, N + (1 - N) eps_clear (Crawford & Duchon
// 1999 [M]).
//
// Grid (plan 4.2): states S[k] at t = k dt. The grid holds S[k] and S[k+1]
// with k = floor(t_env / dt); consumers interpolate linearly at t_env. The
// step k -> k+1 uses the forcing at the midpoint (k + 1/2) dt. Jumps (plan
// 4.7): a forward move of up to max_fast_forward_s is stepped; a larger one, a
// backward one and the first advance spin up from the initial profile at
// k - ceil(spinup_s / dt). Every path is a pure function of (descs, forcing,
// target k), so fast-forward is bit-identical to stepping.
//
// Allocation-free after construction (fixed-size arrays only); not thread safe.

#pragma once

#include "ps/environment/surface_condition_desc.h"
#include "ps/types.h"

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace ps::environment {

inline constexpr real kStefanBoltzmann = 5.670374419e-8;   // W/(m2 K4) [C CODATA 2018, exact]
inline constexpr real kCelsiusToKelvin = 273.15;

// The column of one class, built once from its desc.
struct ColumnModel {
    int nodes = 0;                                   // >= 2; node nodes-1 is the Dirichlet bottom
    std::array<real, kMaxColumnNodes> z_m{};         // node depth below the surface
    std::array<real, kMaxColumnNodes> cap_j_m2k{};   // heat capacity of the node's control volume per m2
    std::array<real, kMaxColumnNodes> cond_w_m2k{};  // conductance between node i and i+1 (k / dz)
    real albedo = 0.0;
    real emissivity = 0.0;
};

// Throws std::invalid_argument when the column needs more than kMaxColumnNodes
// nodes, the total depth exceeds kMaxColumnDepthM, or a value is not positive.
ColumnModel build_column(const SurfaceClassDesc& desc);

// What drives one step (all classes share it): the step midpoint's sun,
// weather and deep-ground temperature.
struct SurfaceForcing {
    real ghi_w_m2 = 0.0;            // global horizontal irradiance incl. clouds
    real air_temp_k = 288.15;
    real wind_m_s = 0.0;            // 10 m mean wind
    real vapour_pressure_hpa = 10.0;
    real cloud_cover = 0.0;         // [0, 1]
    real bottom_temp_k = 283.15;    // Dirichlet at the column bottom
};

// Initial profile at the start of a spin-up: linear from surface to bottom.
struct InitialProfile {
    real surface_temp_k = 288.15;
    real bottom_temp_k = 283.15;
};

// Per-class state. temp_k is the column (node 0 = surface). The W6-W9 fields
// are placeholders, sized now so the checkpoint/hash layout does not change
// shape later; W5 keeps them at exactly 0 (plan 4.1 names in brackets).
struct SurfaceClassState {
    std::array<real, kMaxColumnNodes> temp_k{};
    real water_mm = 0.0;            // W
    real ice_mm = 0.0;              // I (water equivalent)
    real frost_mm = 0.0;            // F (water equivalent)
    real snow_mm = 0.0;             // S (water equivalent)
    real snow_density_kg_m3 = 0.0;  // rho_s
    real soil_moisture = 0.0;       // theta
    real canopy_water_mm = 0.0;     // C
};

// Clear-sky emissivity from the vapour pressure (Geiger 1959 via Solaimanian &
// Kennedy 1993: 0.77 - 0.28 * 10^(-0.074 p), p in mmHg) and the cloud raise.
real sky_emissivity(real vapour_pressure_hpa, real cloud_cover);
// Convection coefficient, Vehrencamp 1953 via Solaimanian & Kennedy 1993:
// 698.24 [0.00144 T_m^0.3 U^0.7 + 0.00097 (T_s - T_a)^0.3], T_m the mean of
// surface and air in K; the free-convection term only while T_s > T_a [AS].
real convection_coefficient_w_m2k(real surface_temp_k, real air_temp_k, real wind_m_s);

// One backward-Euler step of length dt. `out` may alias `in`. Allocation-free.
void step_column(const ColumnModel& column, const SurfaceForcing& forcing, real dt, const SurfaceClassState& in,
                 SurfaceClassState& out);
// The initial (linear) state of a column.
void initial_column_state(const ColumnModel& column, const InitialProfile& p, SurfaceClassState& out);

// The grid's view of its surroundings (the Environment implements it; tests
// can drive a grid with explicit forcing).
class ISurfaceForcingSource {
public:
    virtual ~ISurfaceForcingSource() = default;
    virtual SurfaceForcing forcing_at(real t_env) const = 0;
    virtual InitialProfile initial_at(real t_env) const = 0;
};

// Bit-exact snapshot of the grid (plan 11.2). Plain data, fixed size.
struct ConditionCheckpoint {
    bool valid = false;
    std::int64_t grid_index = 0;    // k: prev = S[k], next = S[k+1]
    std::int32_t class_count = 0;
    real dt_env_s = 0.0;
    std::array<SurfaceClassState, kMaxSurfaceClasses> prev{};
    std::array<SurfaceClassState, kMaxSurfaceClasses> next{};
};

class ConditionGrid {
public:
    // `surface_names`: index = SurfaceId. Empty = the desc's classes in name
    // order. A name without an entry is a static class (listed in
    // unmapped_names()); an entry no name uses is listed in unused_entries().
    // Throws std::invalid_argument (bad desc/grid, > kMaxSurfaceClasses names,
    // duplicate or empty names).
    ConditionGrid(const SurfaceConditionsDesc& desc, const std::vector<std::string>& surface_names,
                  const ConditionGridDesc& grid);

    int class_count() const { return count_; }
    const std::string& class_name(int i) const { return names_[static_cast<std::size_t>(i)]; }
    SurfaceKind kind(int i) const { return kinds_[static_cast<std::size_t>(i)]; }
    bool has_state(int i) const { return kinds_[static_cast<std::size_t>(i)] != SurfaceKind::Static; }
    const ColumnModel& column(int i) const { return columns_[static_cast<std::size_t>(i)]; }
    const std::vector<std::string>& unmapped_names() const { return unmapped_; }
    const std::vector<std::string>& unused_entries() const { return unused_; }
    const ConditionGridDesc& grid_desc() const { return grid_; }
    std::int64_t spinup_steps() const { return spinup_steps_; }
    std::int64_t max_fast_forward_steps() const { return max_ff_steps_; }

    // floor(t / dt) as a grid index.
    std::int64_t grid_index_at(real t_env) const;
    bool initialised() const { return initialised_; }
    std::int64_t grid_index() const { return k_; }
    const SurfaceClassState& prev(int i) const { return prev_[static_cast<std::size_t>(i)]; }
    const SurfaceClassState& next(int i) const { return next_[static_cast<std::size_t>(i)]; }

    // Moves the grid so that k == grid_index_at(t_env) (see the file header).
    // Returns the number of steps taken (spin-up included). Allocation-free.
    std::int64_t advance_to(real t_env, const ISurfaceForcingSource& src);
    // Reinitialises at k_target - spinup_steps() and steps to k_target.
    std::int64_t spin_up(std::int64_t k_target, const ISurfaceForcingSource& src);
    // Forgets the state (the next advance_to spins up).
    void invalidate() { initialised_ = false; }

    // Surface temperature of class i interpolated at t_env (clamped to the
    // [k dt, (k+1) dt] interval the grid holds). 0 for a static class.
    real surface_temp_k(int i, real t_env) const;
    // Interpolation weight of S[k+1] at t_env, [0, 1].
    real weight_at(real t_env) const;

    ConditionCheckpoint checkpoint() const;
    // Throws std::invalid_argument when the checkpoint does not fit (class
    // count, dt, not valid); the state is unchanged then.
    void restore(const ConditionCheckpoint& c);

    // FNV-1a fold: k, class count, then per stateful class in SurfaceId order
    // S[k] and S[k+1] (the column's nodes and the placeholders).
    std::uint64_t fold_hash(std::uint64_t h) const;

private:
    void step_all(const ISurfaceForcingSource& src);   // S[k+1] -> S[k+2], k++

    int count_ = 0;
    std::vector<std::string> names_;
    std::vector<SurfaceKind> kinds_;
    std::vector<ColumnModel> columns_;
    std::vector<std::string> unmapped_;
    std::vector<std::string> unused_;
    ConditionGridDesc grid_;
    std::int64_t spinup_steps_ = 0;
    std::int64_t max_ff_steps_ = 0;

    bool initialised_ = false;
    std::int64_t k_ = 0;
    std::array<SurfaceClassState, kMaxSurfaceClasses> prev_{};
    std::array<SurfaceClassState, kMaxSurfaceClasses> next_{};
};

} // namespace ps::environment
