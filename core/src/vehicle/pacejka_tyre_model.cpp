// core/src/vehicle/pacejka_tyre_model.cpp — Pacejka Magic Formula (MF 5.2/
// 6.1 subset), PLAN.md 5.1/P3.
//
// Source: Pacejka, "Tyre and Vehicle Dynamics", 3rd ed. (Butterworth-
// Heinemann, 2012), ch. 4 ("Semi-Empirical Tyre Models"), specifically the
// "Similarity method" / MF5.2 pure- and combined-slip equations (eq.
// 4.E10-4.E20 for Fx0, 4.E21-4.E39 for Fy0, 4.E49-4.E58 for the combined-
// slip weighting functions Gxa/Gyk). Implemented here as a SUBSET:
//   - Fx0/Fy0 pure slip: full standard sine/arctangent Magic Formula shape
//     (B, C, D, E, SH, SV) as in the book.
//   - Combined-slip Gxa/Gyk: the standard cosine-weighted form (eq.
//     4.E49/4.E56), normalised at zero cross-slip so Gxa(alpha=0)=1 and
//     Gyk(kappa=0)=1 exactly (tested in test_pacejka_tyre_model.cpp).
//   - Mz0: NOT the book's full pneumatic-trail x Fy0 + residual-torque
//     decomposition (eq. 4.E70-4.E78, which needs its own combined-slip
//     coupling and camber residual terms). Simplified here to its OWN
//     Magic-Formula-shaped B/C/D/E curve in alpha (same functional family,
//     its own coefficients qbz*/qcz1/qdz*/qez*/qhz*) - a deliberate,
//     documented reduction (see tyre_desc.h's header comment and
//     out/handover_p3.md's "deferred" list).
//   - Mx: the book's simplified linear-in-camber-and-lateral-force case
//     (a reduced form of eq. 4.E69), not the full expression.
//   - Rolling resistance: eq. 4.E70's shape (qsy1 + qsy2*Fx/Fz0 +
//     qsy3*|V/V0| + qsy4*(V/V0)^4), returned as out.crr_effective via the surface table
//     (see step_rolling_resistance below) rather than folded into Fx here -
//     the caller (core/src/vehicle/wheel_instance.cpp) applies it as an
//     explicit drag force at the contact patch, matching
//     data/surfaces/surfaces.json's own documented convention
//     ("F_roll = crr * N").
//
// Every transcendental call goes through ps::math (PLAN.md D7 - platform
// libm rounds differently between Windows and Linux, found in P1.5a).

#include "ps/vehicle/tyre_model.h"

#include "ps/math/transcendental.h"

#include <algorithm> // std::max, std::clamp
#include <stdexcept> // derive_rolling_resistance
#include <string>
#include <cmath>     // std::abs only (exact, not a rounding-sensitive transcendental)
#include <utility>   // std::move

namespace ps::vehicle {
namespace {

real sign(real x) { return x > 0.0 ? 1.0 : (x < 0.0 ? -1.0 : 0.0); }

// Standard Magic Formula sine/arctangent shape (Pacejka eq. 4.E10-ish core
// term), shared by Fx0/Fy0/Mz0's own curves: y = D*sin(C*atan(B*x -
// E*(B*x - atan(B*x)))).
real magic_formula_core(real B, real C, real D, real E, real x) {
    const real Bx = B * x;
    return D * ps::math::sin(C * ps::math::atan(Bx - E * (Bx - ps::math::atan(Bx))));
}

class PacejkaTyreModel final : public ITyreModel {
public:
    explicit PacejkaTyreModel(TyreDesc desc) : desc_(std::move(desc)) {}

    [[nodiscard]] const TyreDesc& desc() const override { return desc_; }

    // PHYS-033 P4 I2 plan section 5.7: analytic dFx/dkappa at the combined-
    // slip operating point. Calls evaluate() unchanged for `.forces` (so
    // its bits stay identical - test 13e's own bit-identity check), then
    // computes the derivative separately by recomputing the same
    // intermediates evaluate() itself uses.
    [[nodiscard]] TyreForcesWithSlope evaluate_with_slope(const TyreSlipInput& in) const override {
        TyreForcesWithSlope out;
        out.forces = evaluate(in);

        const TyreDesc& p = desc_;
        const real fz = std::max(in.fz, real(0.0));
        if (fz <= 0.0) return out; // airborne: slope 0 (forces are already all-zero)

        const real dfz = (fz - p.fz0) / p.fz0;
        const real gamma = in.gamma;
        const real lambda_mux_eff = p.lambda_mux * in.lambda_mu;

        // --- Fx0 part (evaluate()'s own Bx/Cx/Dx/Ex/SHx, Ex's sign(x)
        // branch treated as a CONSTANT at this operating point, exactly as
        // evaluate() itself does via sign()). ---
        const real Cx = p.pcx1;
        const real Dx = fz * (p.pdx1 + p.pdx2 * dfz) * (real(1.0) - p.pdx3 * gamma * gamma) * lambda_mux_eff;
        const real SHx = p.phx1 + p.phx2 * dfz;
        const real x = in.kappa + SHx;
        const real Ex = (p.pex1 + p.pex2 * dfz + p.pex3 * dfz * dfz) * (real(1.0) - p.pex4 * sign(x));
        const real Kx = fz * (p.pkx1 + p.pkx2 * dfz) * ps::math::exp(p.pkx3 * dfz);
        const real Bx = safe_div(Kx, Cx * Dx);

        const real u = Bx * x;
        const real phi = u - Ex * (u - ps::math::atan(u));
        const real dFx0_dkappa =
            Dx * Cx * ps::math::cos(Cx * ps::math::atan(phi)) * Bx * (real(1.0) - Ex + Ex / (real(1.0) + u * u)) / (real(1.0) + phi * phi);
        const real SVx = fz * (p.pvx1 + p.pvx2 * dfz) * lambda_mux_eff;
        const real Fx0 = magic_formula_core(Bx, Cx, Dx, Ex, x) + SVx;

        // --- Gxa part (evaluate()'s own Bxa/Cxa, a1 = alpha + rhx1). ---
        const real a1 = in.alpha + p.rhx1;
        const real z = p.rbx2 * in.kappa;
        const real one_plus_z2 = real(1.0) + z * z;
        const real Bxa = p.rbx1 * ps::math::cos(ps::math::atan(z));
        // cos(atan(z)) = 1/sqrt(1+z^2) (algebraic form, std::sqrt per D7).
        const real dBxa_dkappa = -p.rbx1 * p.rbx2 * z / (one_plus_z2 * std::sqrt(one_plus_z2));
        const real Cxa = p.rcx1;
        const real Bxa_a1 = Bxa * a1;
        const real Bxa_rhx1 = Bxa * p.rhx1;
        const real N = ps::math::cos(Cxa * ps::math::atan(Bxa_a1));
        const real dN_dBxa = -ps::math::sin(Cxa * ps::math::atan(Bxa_a1)) * Cxa * a1 / (real(1.0) + Bxa_a1 * Bxa_a1);
        const real Dn = ps::math::cos(Cxa * ps::math::atan(Bxa_rhx1));
        const real dDn_dBxa = -ps::math::sin(Cxa * ps::math::atan(Bxa_rhx1)) * Cxa * p.rhx1 / (real(1.0) + Bxa_rhx1 * Bxa_rhx1);

        real dGxa_dkappa = 0.0;
        real Gxa = 0.0;
        constexpr real kEps = 1e-9;
        if (std::abs(Dn) >= kEps) {
            Gxa = N / Dn;
            dGxa_dkappa = (dN_dBxa * Dn - N * dDn_dBxa) / (Dn * Dn) * dBxa_dkappa;
        }
        // (Gxa == 0 here mirrors evaluate()'s safe_div returning 0 when
        // |Dn| < eps; dGxa_dkappa stays 0 too - "mirrors evaluate's
        // safe_div returning 0", plan section 5.7.)

        out.dfx_dkappa = Gxa * dFx0_dkappa + Fx0 * dGxa_dkappa;
        return out;
    }

    [[nodiscard]] TyreForces evaluate(const TyreSlipInput& in) const override {
        const TyreDesc& p = desc_;
        const real fz = std::max(in.fz, real(0.0));
        if (fz <= 0.0) return TyreForces{}; // airborne wheel: no tyre force at all

        const real dfz = (fz - p.fz0) / p.fz0;
        const real gamma = in.gamma;
        const real lambda_mux_eff = p.lambda_mux * in.lambda_mu;
        const real lambda_muy_eff = p.lambda_muy * in.lambda_mu;

        // --- Fx0(kappa, Fz, gamma), eq. 4.E10-4.E18 ---
        const real Cx = p.pcx1;
        const real Dx = fz * (p.pdx1 + p.pdx2 * dfz) * (real(1.0) - p.pdx3 * gamma * gamma) * lambda_mux_eff;
        const real SHx = p.phx1 + p.phx2 * dfz;
        const real kappa_x = in.kappa + SHx;
        const real Ex = (p.pex1 + p.pex2 * dfz + p.pex3 * dfz * dfz) * (real(1.0) - p.pex4 * sign(kappa_x));
        const real Kx = fz * (p.pkx1 + p.pkx2 * dfz) * ps::math::exp(p.pkx3 * dfz);
        const real Bx = safe_div(Kx, Cx * Dx);
        const real SVx = fz * (p.pvx1 + p.pvx2 * dfz) * lambda_mux_eff;
        const real fx0 = magic_formula_core(Bx, Cx, Dx, Ex, kappa_x) + SVx;

        // --- Fy0(alpha, Fz, gamma), eq. 4.E21-4.E39 (simplified Ky, see
        // tyre_desc.h's header comment) ---
        const real Cy = p.pcy1;
        const real Dy = fz * (p.pdy1 + p.pdy2 * dfz) * (real(1.0) - p.pdy3 * gamma * gamma) * lambda_muy_eff;
        const real SHy = p.phy1 + p.phy2 * dfz + p.phy3 * gamma;
        const real alpha_y = in.alpha + SHy;
        const real Ey = (p.pey1 + p.pey2 * dfz) * (real(1.0) - (p.pey3 + p.pey4 * gamma) * sign(alpha_y));
        const real Ky = fz * (p.pky1 + p.pky2 * dfz);
        const real By = safe_div(Ky, Cy * Dy);
        const real SVy = fz * (p.pvy1 + p.pvy2 * dfz) * lambda_muy_eff + fz * (p.pvy3 + p.pvy4 * dfz) * gamma;
        const real fy0 = magic_formula_core(By, Cy, Dy, Ey, alpha_y) + SVy;

        // --- combined-slip weighting, eq. 4.E49/4.E56 ---
        const real Bxa = p.rbx1 * ps::math::cos(ps::math::atan(p.rbx2 * in.kappa));
        const real Cxa = p.rcx1;
        const real Gxa = safe_div(ps::math::cos(Cxa * ps::math::atan(Bxa * (in.alpha + p.rhx1))),
                                   ps::math::cos(Cxa * ps::math::atan(Bxa * p.rhx1)));

        const real Byk = p.rby1 * ps::math::cos(ps::math::atan(p.rby2 * (in.alpha - p.rby3)));
        const real Cyk = p.rcy1;
        const real kappa_s = in.kappa + p.rhy1 + p.rhy2 * dfz;
        const real Gyk = safe_div(ps::math::cos(Cyk * ps::math::atan(Byk * kappa_s)),
                                   ps::math::cos(Cyk * ps::math::atan(Byk * p.rhy1)));
        const real SVyk = fz * (p.rvy1 + p.rvy2 * dfz) * ps::math::cos(ps::math::atan(p.rvy3 * in.alpha)) *
                           ps::math::sin(p.rvy4 * ps::math::atan(p.rvy5 * in.kappa) + p.rvy6);

        TyreForces out;
        out.fx_pure = fx0;
        out.fy_pure = fy0;
        out.fx = Gxa * fx0;
        out.fy = Gyk * fy0 + SVyk;

        // --- Mz0(alpha, Fz, gamma) - simplified own curve (see this
        // file's header comment). PHYS-036: the formula itself is
        // unchanged from Fx0/Fy0's shape (same magic_formula_core), and
        // deliberately carries no extra sign flip here - Pacejka's own
        // convention (Mz0 = -t*Fy0, trail t > 0) is realised entirely
        // through qdz1/qdz2's SIGN (opposite to pdy1/pdy2) in the shipped
        // data (tyre_desc.h's own field comment, data/tyres/*.json's
        // "aligning" source notes), the same way qbz1's magnitude alone
        // (not a formula change) sets a realistic peak slip angle. ---
        const real Cz = p.qcz1;
        const real Dz = fz * p.unloaded_radius * (p.qdz1 + p.qdz2 * dfz);
        const real Bz = p.qbz1 + p.qbz2 * dfz + p.qbz3 * gamma;
        const real Ez = p.qez1 + p.qez2 * dfz;
        const real SHz = p.qhz1 + p.qhz2 * dfz + p.qhz3 * gamma;
        out.mz = magic_formula_core(Bz, Cz, Dz, Ez, in.alpha + SHz);

        // --- Mx(Fz, gamma, Fy) - simplified linear form (reduced eq. 4.E69) ---
        out.mx = fz * p.unloaded_radius * (p.qsx1 - p.qsx2 * gamma + p.qsx3 * (out.fy / p.fz0));

        return out;
    }

private:
    static real safe_div(real num, real den) {
        constexpr real kEps = 1e-9;
        if (std::abs(den) < kEps) return 0.0;
        return num / den;
    }

    TyreDesc desc_;
};

} // namespace

std::unique_ptr<ITyreModel> create_pacejka_tyre_model(const TyreDesc& desc) {
    return std::make_unique<PacejkaTyreModel>(desc);
}

RelaxStep relaxation_step_axis(real x, real slip_velocity, real v_mag, real sigma, real dt) {
    // PHYS-033 P4 I2 plan section 5.6: same expressions, same branch, same
    // threshold as the (now-delegating) step_slip_relaxation above.
    const real a = v_mag / sigma;
    const real b = slip_velocity / sigma;
    constexpr real kTiny = 1e-6;
    if (a < kTiny) return {x + b * dt, dt / sigma};
    const real decay = ps::math::exp(-a * dt);
    return {x * decay + (b / a) * (real(1.0) - decay), (real(1.0) - decay) / v_mag};
}

real tyre_rolling_speed_term(const TyreDesc& desc, real vx) {
    if (desc.rolling_resistance_speed_law == RollingResistanceSpeedLaw::Quadratic) {
        const real x = vx / desc.rolling_resistance_reference_speed_m_s;
        return desc.rolling_resistance_quadratic * x * x;
    }
    // qsy3 == 0 adds an exact +0.0: bit-identical to the old qsy4-only term.
    return desc.qsy4 * ps::math::pow(vx / desc.v0, real(4.0)) + desc.qsy3 * std::abs(vx / desc.v0);
}

void select_rolling_resistance_speed_law(TyreDesc& d, RollingResistanceSpeedLaw law, real fallback) {
    if (law == RollingResistanceSpeedLaw::FourthPower) {
        d.rolling_resistance_speed_law = law;
        d.rolling_resistance_quadratic = 0.0;
        d.rolling_resistance_reference_speed_m_s = 0.0;
        return;
    }
    if (law != RollingResistanceSpeedLaw::Quadratic || !std::isfinite(fallback) || fallback < 0.0)
        throw std::invalid_argument("rolling_resistance: invalid law/reference speed");
    const auto& a = d.rolling_resistance_anchors;
    const bool authored = a.rated_speed_kmh != 0.0;
    real ref = authored ? a.rated_speed_kmh / real(3.6) : (fallback > 0.0 ? fallback : d.v0);
    real rise;
    if (authored) {
        if (!std::isfinite(a.crr_base) || a.crr_base <= 0.0 || !std::isfinite(a.factor_at_rated) || a.factor_at_rated < 1.0)
            throw std::invalid_argument("rolling_resistance: invalid rated anchor");
        rise = a.crr_base * (a.factor_at_rated - real(1.0));
    } else {
        if (!std::isfinite(d.v0) || d.v0 <= 0.0 || !std::isfinite(d.qsy3) || d.qsy3 < 0.0 || !std::isfinite(d.qsy4) || d.qsy4 < 0.0)
            throw std::invalid_argument("rolling_resistance: invalid QSY calibration");
        const real x = ref / d.v0;
        rise = d.qsy4 * ps::math::pow(x, real(4.0)) + d.qsy3 * std::abs(x);
    }
    if (!std::isfinite(ref) || ref <= 0.0 || !std::isfinite(rise) || rise < 0.0 || !std::isfinite(d.qsy1) || d.qsy1 < 0.0)
        throw std::invalid_argument("rolling_resistance: invalid quadratic calibration");
    d.rolling_resistance_reference_speed_m_s = ref;
    d.rolling_resistance_quadratic = rise;
    d.rolling_resistance_speed_law = law;
}

std::uint64_t fold_rolling_resistance_hash(const TyreDesc& d, std::uint64_t seed) {
    if (d.rolling_resistance_speed_law == RollingResistanceSpeedLaw::FourthPower) return seed;
    auto h = seed;
    const auto fold = [&](const auto& value) {
        const auto* bytes = reinterpret_cast<const unsigned char*>(&value);
        for (std::size_t i = 0; i < sizeof(value); ++i) { h ^= bytes[i]; h *= 1099511628211ULL; }
    };
    const std::uint64_t tag = 0x52524c4157000001ULL;
    fold(tag);
    fold(d.rolling_resistance_speed_law);
    fold(d.rolling_resistance_quadratic);
    fold(d.rolling_resistance_reference_speed_m_s);
    fold(d.qsy1); fold(d.qsy2); fold(d.fz0); fold(d.rolling_resistance_v_eps);
    const auto& a = d.rolling_resistance_anchors;
    fold(a.rated_speed_kmh); fold(a.crr_base); fold(a.fraction); fold(a.factor); fold(a.factor_at_rated);
    return h;
}

RollingResistanceCoefficients derive_rolling_resistance(const RollingResistanceAnchors& a, real v0) {
    if (!std::isfinite(a.rated_speed_kmh) || !std::isfinite(a.crr_base) || !std::isfinite(a.fraction) ||
        !std::isfinite(a.factor) || !std::isfinite(a.factor_at_rated) || !std::isfinite(v0))
        throw std::invalid_argument("rolling_resistance: anchors and v0 must be finite");
    const real f = a.fraction;
    if (!(a.rated_speed_kmh > real(0.0))) throw std::invalid_argument("rolling_resistance: rated_speed_kmh must be > 0");
    if (!(a.crr_base > real(0.0))) throw std::invalid_argument("rolling_resistance: crr_base must be > 0");
    if (!(v0 > real(0.0))) throw std::invalid_argument("rolling_resistance: v0 must be > 0");
    if (!(f > real(0.0) && f < real(1.0))) throw std::invalid_argument("rolling_resistance: rise_at_fraction.fraction must be in (0, 1)");
    if (!(a.factor >= real(1.0))) throw std::invalid_argument("rolling_resistance: rise_at_fraction.factor must be >= 1");
    if (!(a.factor_at_rated > a.factor)) throw std::invalid_argument("rolling_resistance: factor_at_rated must be > rise_at_fraction.factor");

    const real f4 = f * f * f * f;
    const real rise_rated = a.factor_at_rated - real(1.0);
    const real rise_f = a.factor - real(1.0);
    real b = (rise_rated * f - rise_f) / (f - f4);
    real aa = rise_rated - b;
    // Exact anchors on the edge (A or B == 0) can land a rounding error on
    // the wrong side of zero; accept that, reject a real violation.
    const real tol = real(1e-12) * rise_rated;
    if (b < -tol || aa < -tol) {
        throw std::invalid_argument("rolling_resistance: the anchors need a negative speed coefficient ((factor-1)/(factor_at_rated-1) = " +
                                    std::to_string(rise_f / rise_rated) + " must lie in [fraction^4, fraction] = [" + std::to_string(f4) + ", " +
                                    std::to_string(f) + "])");
    }
    b = std::max(b, real(0.0));
    aa = std::max(aa, real(0.0));

    const real v_rated = a.rated_speed_kmh / real(3.6);
    const real x_r = v_rated / v0;
    RollingResistanceCoefficients out;
    out.qsy1 = a.crr_base;
    out.qsy3 = aa * a.crr_base / x_r;
    out.qsy4 = b * a.crr_base / (x_r * x_r * x_r * x_r);
    return out;
}

void step_slip_relaxation(SlipRelaxationState& state, real slip_velocity_x, real slip_velocity_y,
                           real vx_mag, real relaxation_length_x, real relaxation_length_y, real dt) {
    // d(kappa_eq)/dt = (slip_velocity_x - vx_mag*kappa_eq) / sigma_kappa,
    // exact exponential solution over one substep (out/handover_p3.md
    // decision 2) - delegates to relaxation_step_axis (PHYS-033 P4 I2 plan
    // section 5.6) so the two are bit-for-bit identical BY CONSTRUCTION
    // (same code path), not merely by matching expressions independently.
    state.kappa_eq = relaxation_step_axis(state.kappa_eq, slip_velocity_x, vx_mag, relaxation_length_x, dt).x_unclamped;
    state.alpha_eq = relaxation_step_axis(state.alpha_eq, slip_velocity_y, vx_mag, relaxation_length_y, dt).x_unclamped;

    // Safety clamp, well past any physical peak/saturation slip (typically
    // within +-0.3 for kappa, +-0.5 rad for alpha): the Magic Formula's
    // coefficients are only fitted/meaningful within a bounded slip range,
    // and at true standstill (vx_mag ~ 0, this function's exact-integration
    // branch above) an extended period of wheel spin with no chassis
    // motion - e.g. a locked-then-reversing brake command - would otherwise
    // integrate kappa_eq/alpha_eq without bound (found running PLAN.md P3's
    // braking validation test: an unclamped kappa_eq reached -7.9 and kept
    // growing). This does not change any in-range behaviour.
    state.kappa_eq = std::clamp(state.kappa_eq, real(-3.0), real(3.0));
    state.alpha_eq = std::clamp(state.alpha_eq, real(-1.6), real(1.6));
}

} // namespace ps::vehicle
