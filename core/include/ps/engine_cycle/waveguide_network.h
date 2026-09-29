// engine_cycle/waveguide_network.h - the linear digital waveguide network for
// the small-amplitude downstream part of an intake or exhaust (ps_audio A3,
// out/audio_plan.md decision D-2 and section 2.2). Part of target
// ps_engine_cycle, never links ps_core. Used standalone and as the downstream
// part of a PipeNetwork (pipe_network.h, PipeEndDesc::Kind::Waveguide): the
// finite-volume pipes carry the large-amplitude upstream part, the waveguide
// the tailpipe side.
//
// Model: linear plane-wave acoustics about a fixed mean state per segment
// (p, T, burned fraction, mean velocity u). Pressure p' = p+ + p-, volume
// velocity Q = Y (p+ - p-), Y = A / (rho c). References: J. O. Smith III,
// "Physical Audio Signal Processing", CCRMA online book
// (https://ccrma.stanford.edu/~jos/pasp/) for the waveguide/junction form and
// the Thiran allpass; A. Fettweis, "Wave digital filters: theory and
// practice", Proc. IEEE 74 (1986) 270 [A: page] for the compliance port.
//
// Segments (constant area, one per straight piece; an area change is a
// junction between two segments). Each segment carries two directions, each a
// chain
//     departing wave -> integer delay line (N >= 1 samples)
//                    -> wall-loss filter (cascade of first-order shelves)
//                    -> first-order Thiran allpass (fractional delay 0.5..1.5)
//                    -> arriving wave.
// The total DC delay of a direction is L_eff / (c +- u) * fs, where L_eff is
// the length plus the end corrections of its two ends (a straight extension,
// like the finite-volume pipes: unflanged 0.6133 r, flanged 0.8216 r), minus
// the DC group delay of an open end's radiation filter on the direction that
// arrives there (that filter models only the reflection magnitude). The wall
// loss filter is minimum phase and its phase is NOT compensated: it stands for
// the dispersion that goes with the wall loss (the Kirchhoff propagation
// constant exp(-(1+j) beta sqrt(omega) L) is itself minimum phase).
//
// Wall loss (per segment):
// - Kirchhoff visco-thermal boundary-layer loss (G. Kirchhoff 1868; A. D.
//   Pierce, Acoustics, 1989 [A: section 10-8]):
//     alpha(f) = sqrt(pi f nu) (1 + (gamma - 1)/sqrt(Pr)) / (r c)   [Np/m],
//   nu = mu/rho with Sutherland's mu (same constants as pipe_network.cpp),
//   Pr = kWaveguidePrandtl [A]. exp(-alpha(f) L) is fitted at build time
//   (ps::math only) by a cascade of kWgLossSections first-order shelves with
//   unit DC gain, fixed log-spaced pole corners over [kWgLossCornerLowHz,
//   min(kWgLossCornerHighHz, 0.45 fs)] and Levenberg-Marquardt-fitted zero
//   corners, relative error weighting over [kWgLossFitLowHz, min(kWgLossFitHighHz,
//   0.4 fs)] (loss_fit_max_rel_error() reports the achieved fit).
// - LaminarFriction: the small-amplitude limit of pipe_network's wall friction
//   (Poiseuille, tau_w = 8 mu u / D), a SERIES resistance per unit length
//   R' = 32 mu / (D^2 A) [Pa s/m^4] (travelling-wave attenuation
//   alpha = R' / (2 Z) = 16 mu / (rho c D^2) Np/m). The segment's R' L is
//   lumped in series with its ports at its Junction/Closed ends (half each, or
//   all of it at the one such end; a segment with neither is rejected), so the
//   loss acts on the volume flow only, like the FV's momentum source. A plain
//   wave gain exp(-alpha L) would be a distortionless line (half series, half
//   shunt): same attenuation at high frequency, but half the damping of a
//   Helmholtz neck's mass flow (measured: 21 vs 30 1/s on the V4 tube). Exists
//   so the waveguide is the linear limit of the finite-volume equations (V4).
//   Not part of mean_pressure_drop() (linear: a DC flow through the
//   waveguide already drops R' L Q).
// - None.
//
// Nodes:
// - Junction: N-port pressure junction (pressure continuity, volume-flow
//   balance): p_J = (2 sum Y_i p_i+ + 2 G_C s_C + Q_ext) / (sum Y_i + G_C),
//   outgoing p_i- = p_J - p_i+. An optional compliance volume V is a wave
//   digital capacitor (bilinear, lossless): G_C = 2 V / (h rho c^2), its
//   stored wave s_C. Q_ext is an external volume inflow for one step
//   (set_node_inflow: a test source, or the finite-volume interface port).
// - Closed: a rigid end (a one-port junction without a volume; R = +1). Also
//   accepts set_node_inflow (a volume-velocity source at a closed end).
// - Open: R = -1 (ideal pressure release) at the end-corrected length, or,
//   with radiation on, the Levine-Schwinger low-ka reflection magnitude
//   |R| ~ 1 - K (ka)^2 (unflanged K = 1/2, H. Levine, J. Schwinger, Phys. Rev.
//   73, 383 (1948); flanged K = 1, the baffled-piston radiation resistance,
//   L. E. Kinsler et al., Fundamentals of Acoustics, 4th ed. 2000 [A:
//   section]) as a one-pole lowpass whose DC curvature matches that form; its
//   DC group delay is taken off the arriving direction's delay line. The
//   flange kind is the attached segment end's end correction (required).
// - Anechoic: R = 0 (absorbs); set_node_wave injects a departing wave (a test
//   source); what arrives is readable (node_arriving_wave).
// - Resistance (quasi-steady loss element, ps_audio A3 part 3: catalyst
//   substrate, silencer baffle): a 2-port series resistance. Mean pressure
//   drop dp = r_lin u + K rho u |u| / 2 at the element's reference area, u
//   from the mean mass flow (set_mean_mass_flow, times flow_fraction); the
//   fluctuation sees the linearised resistance R = (r_lin + K rho |u|) / A
//   (acoustic, Pa s/m^3): Q = 2 (p1+ - p2+) / (R + Z1 + Z2), Z = 1/Y.
//   r_lin includes a monolith's laminar channel friction when the node
//   carries one (WgNodeDesc::monolith_*, ps_audio A8b) - the part of a
//   catalyst's resistance that does not vanish at zero mean flow.
//
// Timing: per step every direction first produces its arriving wave from the
// delay line written in earlier steps (begin_step), then every node scatters
// (finish_step) and the departing waves enter the delay lines. N >= 1 breaks
// every algebraic loop. step() = begin_step() + finish_step().
//
// Build-time checks (std::invalid_argument, messages start with
// "WaveguideNetwork: "): topology (node degree per kind), positive geometry
// and states, |u| < c, and every direction's integer delay N >= 1 (total delay
// >= 1.5 samples after the filter delays are taken off; message pinned by a
// test).
//
// Deterministic: ps::math only (and only at build time), no unordered
// containers, no mutable statics, allocation-free after construction, plain
// double scalar code (SIMD is A9).

#pragma once

#include "ps/drivetrain/engine_cycle_desc.h"
#include "ps/engine_cycle/gas.h"
#include "ps/engine_cycle/pipe_network_types.h"
#include "ps/types.h"

#include <cstdint>
#include <string>
#include <vector>

namespace ps::engine_cycle {

inline constexpr real kWaveguidePrandtl = 0.71;     // air near 300 K [A]; used for burned gas too
inline constexpr int kWgLossSections = 6;           // first-order shelves per Kirchhoff loss filter
// Pole corners, log-spaced. Measured (out/audio_a3_progress.md): 3 shelves with
// corners on the 40..6000 Hz fit band fit to 12 % max relative error, 6 shelves
// with corners over 10..10000 Hz to 0.3 %.
inline constexpr real kWgLossCornerLowHz = 10.0;
inline constexpr real kWgLossCornerHighHz = 10000.0;
inline constexpr real kWgLossFitLowHz = 40.0;       // Kirchhoff fit band (relative error weighting)
inline constexpr real kWgLossFitHighHz = 6000.0;
inline constexpr real kWgMinDelaySamples = 1.5;     // integer part N >= 1 plus the allpass fraction >= 0.5

enum class WgLoss : std::uint8_t { None, Kirchhoff, LaminarFriction };

struct WgNodeDesc {
    enum class Kind : std::uint8_t { Junction, Closed, Open, Anechoic, Resistance };
    Kind kind = Kind::Junction;
    std::string name;
    // Junction: compliance volume (0 = a pure pressure junction) and its mean state.
    real volume_m3 = 0.0;
    GasStateDesc volume_gas;
    // Open: Levine-Schwinger reflection filter (true) or an ideal release R = -1.
    bool radiation = true;
    // Resistance: dp = r_linear * u + k_loss * rho u|u|/2 at area_m2 (0 = the
    // smaller attached segment's area); u = flow_fraction * mdot_mean / (rho A).
    real k_loss = 0.0;
    real r_linear_pa_s_m = 0.0;
    real area_m2 = 0.0;
    real flow_fraction = 1.0;
    // ps_audio A8b: a monolith substrate's laminar channel friction, added to
    // r_linear at build time from the attached (inlet-side) segment's
    // viscosity mu: r += 2 fRe mu L / (d_h^2 OFA) [Pa per m/s at area_m2] -
    // fully developed laminar flow in each channel at u_ch = u / OFA,
    // dp/L = 2 fRe mu u_ch / d_h^2 (fRe = Fanning f * Re). 0 length = none.
    real monolith_length_m = 0.0;
    real monolith_d_h_m = 0.0;         // channel hydraulic diameter
    real monolith_open_frontal_area = 0.0;  // open / frontal area, (0, 1]
    real monolith_f_re = 0.0;          // Fanning f * Re of the channel shape
};

struct WgSegmentDesc {
    std::string name;
    real length_m = 0.0;
    real diameter_m = 0.0;
    int node_a = -1;  // the segment runs a -> b (+x)
    int node_b = -1;
    EndCorrection end_a = EndCorrection::None;  // end corrections (length extensions)
    EndCorrection end_b = EndCorrection::None;
    WgLoss loss = WgLoss::Kirchhoff;
    GasStateDesc gas;              // mean state: rho, c, mu
    real mean_velocity_m_s = 0.0;  // mean flow a -> b
};

struct WaveguideNetworkDesc {
    drivetrain::GasPropertiesDesc gas;
    std::vector<WgNodeDesc> nodes;
    std::vector<WgSegmentDesc> segments;
};

// Norton view of a junction for an external port (the finite-volume
// interface): with an external port of admittance Y_e sending wave w into the
// junction, p_J = (s + 2 Y_e w) / (y_sum + Y_e).
struct WgNorton {
    real s = 0.0;      // 2 sum Y_i p_i+ (+ 2 G_C s_C)
    real y_sum = 0.0;  // sum Y_i (+ G_C)
};

class WaveguideNetwork {
public:
    WaveguideNetwork() = default;  // empty network (step is a no-op)
    // Throws std::invalid_argument ("WaveguideNetwork: ...").
    WaveguideNetwork(const WaveguideNetworkDesc& desc, real h);

    // One step: begin_step() then finish_step(). Allocation-free.
    void step();
    // Arriving waves of this step from the delay lines (no node work).
    void begin_step();
    // Node scattering with this step's external inflows/injected waves, then
    // the departing waves enter the delay lines. Clears the per-step inputs.
    void finish_step();

    [[nodiscard]] bool empty() const { return segs_.empty(); }
    [[nodiscard]] real h() const { return h_; }
    [[nodiscard]] int node_count() const { return static_cast<int>(nodes_.size()); }
    [[nodiscard]] int segment_count() const { return static_cast<int>(segs_.size()); }

    // --- Inputs for the current step (between begin_step and finish_step, or
    // before step()) ---------------------------------------------------------
    // External volume inflow [m^3/s] into a Junction or Closed node.
    void set_node_inflow(int node, real q_m3_s);
    // Departing wave [Pa] injected by an Anechoic node.
    void set_node_wave(int node, real p_pa);
    // Mean mass flow [kg/s] through the Resistance elements (positive a -> b of
    // the downstream path), held until changed.
    void set_mean_mass_flow(real mdot_kg_s) { mdot_mean_ = mdot_kg_s; }

    // Norton view of a Junction node after begin_step().
    [[nodiscard]] WgNorton junction_norton(int node) const;

    // --- Observables (valid after finish_step) ----------------------------
    // Acoustic pressure [Pa] at the node in the last step (junction pressure,
    // or p+ + p- at a one-port end).
    [[nodiscard]] real node_pressure(int node) const { return nodes_[node].p; }
    // Volume velocity [m^3/s] leaving the network through an Open node.
    [[nodiscard]] real node_outflow(int node) const { return nodes_[node].q_out; }
    // Wave [Pa] that arrived at a one-port node in the last step.
    [[nodiscard]] real node_arriving_wave(int node) const { return nodes_[node].arriving; }
    // Sum of the mean pressure drops of all Resistance elements [Pa] (series
    // assumption, PROVISIONAL out/audio_a3_progress.md).
    [[nodiscard]] real mean_pressure_drop() const;
    // What the linearised elements drop for the same mean flow carried as a DC
    // volume flow through the waveguide: sum (r_lin + K rho |u|) u [Pa].
    // mean_pressure_drop() - linearised_mean_drop() = -sum K rho u|u|/2 is the
    // correction a caller feeding the whole flow adds to its reference pressure.
    [[nodiscard]] real linearised_mean_drop() const;

    // Wave energy in flight [J]: h * sum over directions Y (delay-line samples^2
    // + allpass state^2/(1 - a^2)) + h * sum G_C s_C^2. Exactly conserved by a
    // lossless network (no wall loss, no radiation, no resistance, no Anechoic
    // node); loss- and radiation-filter states are not included.
    [[nodiscard]] real wave_energy() const;

    // --- Build results (tests, diagnostics) --------------------------------
    [[nodiscard]] real segment_admittance(int seg) const { return segs_[seg].y; }
    [[nodiscard]] real segment_sound_speed(int seg) const { return segs_[seg].c; }
    [[nodiscard]] real segment_density(int seg) const { return segs_[seg].rho; }
    [[nodiscard]] real segment_effective_length(int seg) const { return segs_[seg].l_eff; }
    // Total delay in samples of direction dir (0: a -> b, 1: b -> a) incl. the
    // allpass fraction; and its integer delay-line length.
    [[nodiscard]] real direction_delay_samples(int seg, int dir) const { return dirs_[2 * seg + dir].delay; }
    [[nodiscard]] int direction_line_length(int seg, int dir) const { return dirs_[2 * seg + dir].n; }
    // Per-pass wall attenuation target alpha(f) * L [Np] and the fitted filter's
    // actual per-pass attenuation -ln|H(f)| [Np] of a segment.
    [[nodiscard]] real wall_attenuation_target(int seg, real f_hz) const;
    [[nodiscard]] real wall_attenuation_filter(int seg, real f_hz) const;
    [[nodiscard]] real loss_fit_max_rel_error(int seg) const { return segs_[seg].fit_err; }
    // Radiation filter pole of an Open node (0 when ideal) and its DC group delay [samples].
    [[nodiscard]] real radiation_pole(int node) const { return nodes_[node].rad_p; }

    // FNV-1a fold of every delay-line sample and filter state, in order.
    [[nodiscard]] std::uint64_t fold_state_hash(std::uint64_t h) const;

private:
    struct Shelf {
        real b0 = 1.0, b1 = 0.0, a1 = 0.0;  // H = (b0 + b1 z^-1) / (1 + a1 z^-1)
    };
    struct Dir {
        int from = -1;     // node the wave departs from
        int to = -1;       // node it arrives at
        int buf0 = 0;      // first sample in buf_
        int n = 1;         // integer delay-line length
        int w = 0;         // ring index: oldest sample (read, then overwritten)
        int sh0 = 0;       // first shelf in shelves_/shelf_state_
        int nsh = 0;       // shelves (0 = no loss)
        real delay = 0.0;  // total delay [samples]
        real ap_a = 0.0;   // Thiran coefficient
        real ap_s = 0.0;   // allpass state (transposed direct form II)
        real arriving = 0.0;
        real departing = 0.0;
    };
    struct Seg {
        real area = 0.0, y = 0.0, z = 0.0, rho = 0.0, c = 0.0, mu = 0.0, gamma = 1.4;
        real length = 0.0, l_eff = 0.0, alpha0 = 0.0, alpha_lam = 0.0;
        WgLoss loss = WgLoss::None;
        real fit_err = 0.0;
    };
    struct Port {
        int arrive = -1;  // direction arriving at the node through this port
        int depart = -1;  // direction departing from the node through this port
        int seg = -1;
        real y = 0.0;  // port admittance 1/(Z + r) seen by the junction
        real r = 0.0;  // lumped series resistance (LaminarFriction) [Pa s/m^3]
    };
    struct Node {
        WgNodeDesc::Kind kind = WgNodeDesc::Kind::Junction;
        int p0 = 0, np = 0;   // ports in ports_
        real g_c = 0.0;       // compliance port conductance
        real s_c = 0.0;       // compliance stored wave
        real y_sum = 0.0;     // sum of port admittances + g_c
        real inflow = 0.0;    // per-step external volume inflow
        real wave_in = 0.0;   // per-step injected wave (Anechoic)
        real rad_p = 0.0;     // radiation one-pole pole (Open)
        real rad_s = 0.0;     // radiation one-pole state
        // Resistance element
        real k_loss = 0.0, r_lin = 0.0, area = 0.0, frac = 1.0, rho = 0.0;
        // Observables
        real p = 0.0, q_out = 0.0, arriving = 0.0;
    };

    real h_ = 0.0;
    GasModel gas_;
    real mdot_mean_ = 0.0;
    std::vector<Seg> segs_;
    std::vector<Dir> dirs_;
    std::vector<Node> nodes_;
    std::vector<Port> ports_;
    std::vector<Shelf> shelves_;
    std::vector<real> shelf_state_;
    std::vector<real> buf_;
};

} // namespace ps::engine_cycle
