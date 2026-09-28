// engine_cycle/pipe_network.h - the quasi-1D finite-volume pipe core (ps_audio
// A2, out/audio_plan.md decision D-2 and section 2.2). Shared by the map
// generator (A5, power-grade cells) and ps_audio (A7, audio-grade cells); part
// of target ps_engine_cycle, never links ps_core. Nothing consumes it yet.
//
// Model (per pipe, per unit length, A(x) the local cross-section):
//   d(rho A)/dt   + d(rho u A)/dx         = 0
//   d(rho u A)/dt + d((rho u^2 + p) A)/dx = p dA/dx - tau_w * pi*D
//   d(E A)/dt     + d(u (E + p) A)/dx     = q_w * pi*D
//   d(rho x A)/dt + d(rho u x A)/dx       = 0          (burned mass fraction x)
// E = rho*(e + u^2/2), e = the two-group sensible internal energy of gas.h
// (cv = a + b*T per group, referenced to 298.15 K), p = rho*R_mix*T. The same
// energy reference as the 0D control volumes, so a volume and a pipe exchange
// enthalpy without a conversion.
//
// Numerics (E. F. Toro, Riemann Solvers and Numerical Methods for Fluid
// Dynamics, 3rd ed., Springer 2009 [A: sections]):
// - MUSCL-Hancock (ch. 14.4, primitive-variable half step) with a minmod limiter
//   on (rho, u, p, x); the half step includes the area term. Zero slope in each
//   pipe's first and last cell (PROVISIONAL, out/audio_a2_progress.md P3).
// - HLLC flux (ch. 10.4) with Davis wave-speed estimates (ch. 10.5.1).
// - Area source p*dA/dx written in a well-balanced form: a quiescent uniform
//   state stays exactly at rest in a tapered pipe.
// - A face falls back to first order (zero slope, no half step) when the
//   reconstructed state would have rho <= 0 or p <= 0.
// - Wall friction: Fanning factor max(16/Re, 0.0791 Re^-1/4) (laminar Poiseuille;
//   Blasius 1913), Sutherland viscosity of air (F. M. White, Viscous Fluid Flow,
//   3rd ed. 2006 [A: table 1-2], mu0 1.716e-5 Pa s, T0 273 K, S 111 K).
//   Optional wall heat transfer by the Reynolds analogy St = f/2 (Reynolds 1874)
//   to a fixed wall temperature. Both evaluated from the start-of-step state.
// - Explicit, one fixed step h for the whole network (the cycle model's h; no
//   adaptive step). Only + - * / and std::sqrt in the cell loop; ps::math::pow
//   only at boundary faces and orifices (PLAN.md D7).
//
// Elements:
// - Pipe: length, linear diameter taper, target cell size, worst-case wave speed
//   (|u| + a) for the CFL check, two ends.
// - Pipe ends: Closed (reflecting wall), Open (the network ambient; optional
//   end correction), Volume (a network-owned 0D (m, U, m_b) volume: plenum,
//   junction, collector, silencer chamber), Port (an EXTERNALLY owned 0D volume,
//   the interface A5 couples to the cylinder model's RK4 port: the owner sets
//   the port state before each step and applies port_flux() to its own state
//   after it - a conservative hand-off). Open/Volume/Port faces use one
//   characteristic "pressure reservoir" solver at full pipe area: outflow at the
//   reservoir pressure along the outgoing isentropic characteristic, inflow
//   isentropic from the reservoir's stagnation state, sonic limit both ways.
// - End correction: an Open/Volume/Port end may carry an unflanged (0.6133 r,
//   H. Levine, J. Schwinger, Phys. Rev. 73, 383 (1948)) or flanged (0.8216 r,
//   Lord Rayleigh / A. N. Norris, I. C. Sheng, J. Sound Vib. 135 (1989) 85
//   [A: page]) end correction. It is modelled as a straight extension of the
//   pipe's end diameter, so the pipe's computational length is L + corrections
//   (effective_length_m()); PROVISIONAL, progress file P2.
// - Orifice: a quasi-steady compressible orifice (valve_flow.h's Psi) between
//   two volumes or a volume and the ambient - the throttle from the airbox into
//   the plenum (plan section 2.2). Explicit.
// - Volumes are adiabatic, stagnant and updated explicitly with the same face
//   fluxes the pipes use (mass, energy and burned mass are conserved to
//   rounding; a volume absorbs the momentum of a stream entering it).
//
// Build-time checks (std::invalid_argument, messages pinned by tests):
// - pipe CFL: h * max_wave_speed / dx <= cfl_limit (default 0.8, plan section 4);
// - volume/port coupling: h * sum(A_end * max_wave_speed) / V <= cfl_limit;
// - orifice: h * CdA * S * sqrt(1.4) * max_wave_speed * (1/V_a + 1/V_b) <=
//   cfl_limit, S the regularised Psi slope at pressure equality.
//
// Allocation-free after construction; no unordered containers; no mutable
// statics; plain double, scalar code over structure-of-arrays cell storage
// (SIMD is A9).

#pragma once

#include "ps/drivetrain/engine_cycle_desc.h"
#include "ps/engine_cycle/gas.h"
#include "ps/engine_cycle/pipe_network_types.h"
#include "ps/engine_cycle/valve_flow.h"
#include "ps/types.h"

#include <cstdint>
#include <string>
#include <vector>

namespace ps::engine_cycle {

// kUnflangedEndCorrection, kFlangedEndCorrection, kDefaultPipeCflLimit,
// EndCorrection and GasStateDesc live in pipe_network_types.h (shared with
// waveguide_network.h since ps_audio A3).

struct PipeEndDesc {
    enum class Kind : std::uint8_t { Closed, Open, Volume, Port };
    Kind kind = Kind::Closed;
    int index = -1;  // volume index (Volume) or port index (Port); unused otherwise
    EndCorrection end_correction = EndCorrection::None;  // rejected on a Closed end
};

enum class WallFriction : std::uint8_t { None, LaminarBlasius };

struct PipeWallDesc {
    WallFriction friction = WallFriction::LaminarBlasius;
    bool heat_transfer = false;      // Reynolds analogy to wall_temperature_k
    real wall_temperature_k = 300.0;
};

struct PipeDesc {
    std::string name;
    real length_m = 0.0;
    real d_start_m = 0.0;  // diameter at the start end
    real d_end_m = 0.0;    // diameter at the far end (linear taper in diameter)
    real dx_target_m = 0.0;  // cells: n = max(1, floor(L_eff / dx_target)), so dx >= dx_target
    real max_wave_speed_m_s = 900.0;  // worst-case |u| + a for the CFL check (exhaust ~900, intake ~500 [A])
    PipeEndDesc start;
    PipeEndDesc end;
    PipeWallDesc wall;
    GasStateDesc initial;  // uniform initial state, at rest
};

struct VolumeDesc {
    std::string name;
    real volume_m3 = 0.0;
    GasStateDesc initial;
};

// An externally owned 0D volume (A5: the cylinder model's intake/exhaust port).
// Its volume is used only for the build-time coupling check.
struct PortDesc {
    std::string name;
    real volume_m3 = 0.0;
};

struct OrificeDesc {
    std::string name;
    int volume_a = -1;  // must be a volume
    int volume_b = -1;  // a volume, or -1 for the network ambient
    real cd_area_m2 = 0.0;
    real max_wave_speed_m_s = 500.0;  // worst-case sound speed on either side, for the check
};

struct PipeNetworkDesc {
    drivetrain::GasPropertiesDesc gas;
    GasStateDesc ambient;  // every Open end and ambient-side orifice
    std::vector<PipeDesc> pipes;
    std::vector<VolumeDesc> volumes;
    std::vector<PortDesc> ports;
    std::vector<OrificeDesc> orifices;
    real cfl_limit = kDefaultPipeCflLimit;
};

// State of an externally owned port volume at the start of a step (stagnant).
struct PortState {
    real p_pa = 101325.0;
    real t_k = 293.15;
    real x_b = 0.0;
};

// What crossed a port face during the last step, positive from the port INTO
// the pipe. The owner subtracts these from its port volume (energy is total
// enthalpy flux: the stream's kinetic energy is part of it).
struct PortFlux {
    real mass_kg = 0.0;
    real energy_j = 0.0;
    real burned_kg = 0.0;
};

// Primitive state of one pipe cell.
struct PipeCellState {
    real rho = 0.0;
    real u = 0.0;
    real p = 0.0;
    real t = 0.0;
    real x_b = 0.0;
    real a = 0.0;  // sound speed
};

// Cumulative exchange with the outside since construction, positive INTO the
// network: global closure is total_mass() - initial = ambient + source + port.
struct PipeNetworkExchange {
    real ambient_mass_kg = 0.0, ambient_energy_j = 0.0, ambient_burned_kg = 0.0;
    real source_mass_kg = 0.0, source_energy_j = 0.0, source_burned_kg = 0.0;
    real port_mass_kg = 0.0, port_energy_j = 0.0, port_burned_kg = 0.0;
};

class PipeNetwork {
public:
    // Throws std::invalid_argument on an invalid description or a failed
    // stability check (messages start with "PipeNetwork: ").
    PipeNetwork(const PipeNetworkDesc& desc, real h);

    // One explicit step of h. Allocation-free.
    void step();

    [[nodiscard]] real h() const { return h_; }
    [[nodiscard]] std::uint64_t steps() const { return steps_; }
    [[nodiscard]] const GasModel& gas() const { return gas_; }

    // --- Pipes -----------------------------------------------------------
    [[nodiscard]] int pipe_count() const { return static_cast<int>(pipes_.size()); }
    [[nodiscard]] int cell_count(int pipe) const { return pipes_[pipe].n; }
    [[nodiscard]] int total_cell_count() const { return static_cast<int>(r_.size()); }
    [[nodiscard]] real cell_dx(int pipe) const { return pipes_[pipe].dx; }
    [[nodiscard]] real effective_length_m(int pipe) const { return pipes_[pipe].l_eff; }
    // Computational length added in front of the physical start / after its end.
    [[nodiscard]] real start_extension_m(int pipe) const { return pipes_[pipe].ext_start; }
    [[nodiscard]] real end_extension_m(int pipe) const { return pipes_[pipe].ext_end; }
    // Cell centre in computational coordinates [0, effective_length_m].
    [[nodiscard]] real cell_center_m(int pipe, int i) const { return (i + 0.5) * pipes_[pipe].dx; }
    [[nodiscard]] real cell_volume_m3(int pipe, int i) const { return vol_[pipes_[pipe].c0 + i]; }
    [[nodiscard]] PipeCellState cell(int pipe, int i) const;
    // Overwrite one cell (initial conditions, tests). rho from p/(R T).
    void set_cell(int pipe, int i, real p_pa, real t_k, real u_m_s, real x_b);

    // --- Volumes and ports -------------------------------------------------
    [[nodiscard]] int volume_count() const { return static_cast<int>(vols_.size()); }
    [[nodiscard]] CvState volume_state(int v) const;
    void set_volume(int v, const GasStateDesc& s);
    // A constant external source into a network volume, applied every step
    // until changed: mass rate [kg/s] carrying total enthalpy h_total [J/kg]
    // and burned fraction x_b. Negative mass rate removes gas at the volume's
    // own enthalpy and composition (h_total, x_b ignored).
    void set_volume_source(int v, real mdot_kg_s, real h_total_j_kg, real x_b);

    [[nodiscard]] int port_count() const { return static_cast<int>(ports_.size()); }
    void set_port_state(int port, const PortState& s);
    [[nodiscard]] const PortFlux& port_flux(int port) const { return ports_[port].flux; }

    // --- Totals, determinism -------------------------------------------------
    [[nodiscard]] real total_mass() const;         // pipes + network volumes (not ports)
    [[nodiscard]] real total_energy() const;       // sum E*V + volume U (gas.h reference)
    [[nodiscard]] real total_burned_mass() const;
    [[nodiscard]] const PipeNetworkExchange& exchange() const { return exch_; }
    // Cells that fell back to first order in the last step (diagnostic).
    [[nodiscard]] int last_step_fallback_count() const { return fallback_count_; }
    // FNV-1a fold of every cell's and volume's conserved state bytes, in order.
    [[nodiscard]] std::uint64_t fold_state_hash(std::uint64_t h) const;

private:
    struct PipeRt {
        int c0 = 0;   // first cell in the global cell arrays
        int n = 0;    // cells
        int f0 = 0;   // first face in the global face arrays (n + 1 faces)
        real dx = 0.0;
        real inv_dx = 0.0;
        real l_eff = 0.0;
        real ext_start = 0.0;
        real ext_end = 0.0;
        PipeEndDesc start;
        PipeEndDesc end;
        PipeWallDesc wall;
    };
    struct VolumeRt {
        real v = 0.0;
        real m = 0.0, u = 0.0, mb = 0.0;  // conserved (U: gas.h sensible energy)
        real src_mdot = 0.0, src_h = 0.0, src_x = 0.0;
        // Start-of-step reservoir view.
        real p = 0.0, t = 0.0, x = 0.0, r = 0.0, gamma = 1.4, a = 0.0, h_static = 0.0;
    };
    struct PortRt {
        real p = 101325.0, t = 293.15, x = 0.0;
        real r = 0.0, gamma = 1.4, a = 0.0, h_static = 0.0;
        PortFlux flux;
        int pipe = -1;
        bool at_start = true;
    };
    struct OrificeRt {
        int a = -1, b = -1;
        real cd_area = 0.0;
    };

    // Reservoir view used by the boundary-face solver.
    struct Reservoir {
        real p, t, x, r, gamma, a, h_static;
    };

    void refresh_reservoir(VolumeRt& v) const;
    void refresh_port(PortRt& p) const;
    [[nodiscard]] Reservoir ambient_reservoir() const;

    real h_ = 0.0;
    std::uint64_t steps_ = 0;
    GasModel gas_;
    PsiPair psi_;
    Reservoir ambient_{};
    std::vector<PipeRt> pipes_;
    std::vector<VolumeRt> vols_;
    std::vector<PortRt> ports_;
    std::vector<OrificeRt> orifices_;
    PipeNetworkExchange exch_;
    int fallback_count_ = 0;

    // Cells (structure of arrays): conserved densities per unit volume.
    std::vector<real> r_, ru_, e_, rx_;
    std::vector<real> vol_, inv_vol_, diam_, area_src_;  // area_src_ = (A+ - A-)/(dx * A_mean)
    // Faces (n + 1 per pipe).
    std::vector<real> af_;
    // Scratch (per cell): primitives, gamma, half-step face states.
    std::vector<real> w_r_, w_u_, w_p_, w_x_, w_g_, ps_;
    std::vector<real> l_r_, l_u_, l_p_, l_x_;  // left face state of each cell (at i - 1/2)
    std::vector<real> rr_, ru2_, rp_, rx2_;    // right face state of each cell (at i + 1/2)
    // Scratch (per face): flux per unit area.
    std::vector<real> fm_, fmom_, fe_, fx_;
};

} // namespace ps::engine_cycle
