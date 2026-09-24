// Unit tests for ps::vehicle's Pacejka Magic Formula tyre model (PLAN.md
// P3 acceptance: "the tyre model reproduces its own .tir/JSON reference
// curves (Fx, Fy, Mz, combined) within 1%").
//
// Two independent checks:
//  1. A from-scratch re-derivation of the PURE-slip Fx0/Fy0 formulas
//     (written directly against TyreDesc's fields here, not sharing code
//     with pacejka_tyre_model.cpp) is compared against
//     ITyreModel::evaluate()'s fx_pure/fy_pure outputs - catches a
//     transcription bug in the production implementation, the same way
//     test_aba_fast_transform.cpp cross-checks two nominally-equal
//     implementations of the same math.
//  2. Mathematical identities of the combined-slip weighting that must
//     hold for ANY parameter set, checked to near machine precision:
//     Gxa(alpha=0) == 1 (so Fx(kappa, 0) == Fx0(kappa)) and Gyk(kappa=0)
//     == 1 (so Fy(0, alpha) == Fy0(alpha), given SVyk(kappa=0) == 0 for
//     the default parameters below).

#include "ps/io/vehicle_io.h"
#include "ps/math/transcendental.h"
#include "ps/vehicle/tyre_model.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <string>

#ifndef PS_SOURCE_DIR
#error "PS_SOURCE_DIR must be defined by CMake (tests/unit/CMakeLists.txt)"
#endif

using Catch::Approx;
using namespace ps;
using namespace ps::vehicle;

namespace {

// Independent re-derivation of Fx0(kappa, Fz, gamma) - see this file's top
// comment. Deliberately restructured (different local variable order/
// naming) from pacejka_tyre_model.cpp's own magic_formula_core helper.
real reference_fx0(const TyreDesc& p, real kappa, real fz, real gamma) {
    const real dfz = (fz - p.fz0) / p.fz0;
    const real C = p.pcx1;
    const real D = fz * (p.pdx1 + p.pdx2 * dfz) * (1.0 - p.pdx3 * gamma * gamma) * p.lambda_mux;
    const real SH = p.phx1 + p.phx2 * dfz;
    const real x = kappa + SH;
    const real E = (p.pex1 + p.pex2 * dfz + p.pex3 * dfz * dfz) * (1.0 - p.pex4 * (x >= 0 ? 1.0 : -1.0));
    const real K = fz * (p.pkx1 + p.pkx2 * dfz) * ps::math::exp(p.pkx3 * dfz);
    const real B = K / (C * D);
    const real SV = fz * (p.pvx1 + p.pvx2 * dfz) * p.lambda_mux;
    const real Bx = B * x;
    return D * ps::math::sin(C * ps::math::atan(Bx - E * (Bx - ps::math::atan(Bx)))) + SV;
}

real reference_fy0(const TyreDesc& p, real alpha, real fz, real gamma) {
    const real dfz = (fz - p.fz0) / p.fz0;
    const real C = p.pcy1;
    const real D = fz * (p.pdy1 + p.pdy2 * dfz) * (1.0 - p.pdy3 * gamma * gamma) * p.lambda_muy;
    const real SH = p.phy1 + p.phy2 * dfz + p.phy3 * gamma;
    const real y = alpha + SH;
    const real E = (p.pey1 + p.pey2 * dfz) * (1.0 - (p.pey3 + p.pey4 * gamma) * (y >= 0 ? 1.0 : -1.0));
    const real K = fz * (p.pky1 + p.pky2 * dfz);
    const real B = K / (C * D);
    const real SV = fz * (p.pvy1 + p.pvy2 * dfz) * p.lambda_muy + fz * (p.pvy3 + p.pvy4 * dfz) * gamma;
    const real By = B * y;
    return D * ps::math::sin(C * ps::math::atan(By - E * (By - ps::math::atan(By)))) + SV;
}

} // namespace

TEST_CASE("Pacejka pure-slip Fx0 matches an independent re-derivation", "[vehicle][tyre]") {
    TyreDesc desc; // defaults from tyre_desc.h - a plausible mid-size passenger tyre
    auto model = create_pacejka_tyre_model(desc);

    for (real kappa : {-0.30, -0.10, -0.02, 0.0, 0.02, 0.08, 0.20, 0.35}) {
        for (real fz : {2000.0, 4000.0, 6000.0}) {
            TyreSlipInput in;
            in.kappa = kappa;
            in.alpha = 0.0; // isolates Fx0 (Gxa(alpha=0) == 1, checked separately below)
            in.fz = fz;
            in.gamma = 0.0;
            in.vx = 20.0;
            const TyreForces f = model->evaluate(in);
            const real expected = reference_fx0(desc, kappa, fz, 0.0);
            const real tol = std::max(1.0, std::abs(expected) * 0.01); // 1% relative, PLAN.md P3 acceptance
            CHECK(f.fx_pure == Approx(expected).margin(tol));
            CHECK(f.fx == Approx(expected).margin(tol)); // Gxa(alpha=0) == 1
        }
    }
}

TEST_CASE("Pacejka pure-slip Fy0 matches an independent re-derivation", "[vehicle][tyre]") {
    TyreDesc desc;
    auto model = create_pacejka_tyre_model(desc);

    for (real alpha : {-0.20, -0.08, -0.01, 0.0, 0.01, 0.05, 0.12, 0.20}) {
        for (real fz : {2000.0, 4000.0, 6000.0}) {
            for (real gamma : {-0.05, 0.0, 0.05}) {
                TyreSlipInput in;
                in.kappa = 0.0; // isolates Fy0 (Gyk(kappa=0) == 1 and SVyk(kappa=0) == 0 for these defaults)
                in.alpha = alpha;
                in.fz = fz;
                in.gamma = gamma;
                in.vx = 20.0;
                const TyreForces f = model->evaluate(in);
                const real expected = reference_fy0(desc, alpha, fz, gamma);
                const real tol = std::max(1.0, std::abs(expected) * 0.01);
                CHECK(f.fy_pure == Approx(expected).margin(tol));
                CHECK(f.fy == Approx(expected).margin(tol));
            }
        }
    }
}

TEST_CASE("Pacejka combined slip reduces to pure slip at zero cross-slip", "[vehicle][tyre]") {
    TyreDesc desc;
    auto model = create_pacejka_tyre_model(desc);

    // Fx(kappa, alpha=0) == Fx0(kappa) to near machine precision - a
    // mathematical identity of the Gxa normalisation, not an empirical
    // tolerance.
    for (real kappa : {-0.25, -0.05, 0.05, 0.25}) {
        TyreSlipInput in;
        in.kappa = kappa;
        in.alpha = 0.0;
        in.fz = 4000.0;
        in.vx = 20.0;
        const TyreForces f = model->evaluate(in);
        CHECK(f.fx == Approx(f.fx_pure).margin(1e-9 * std::max(1.0, std::abs(f.fx_pure))));
    }

    // Fy(kappa=0, alpha) == Fy0(alpha) (Gyk normalisation + SVyk(kappa=0)==0
    // for these defaults).
    for (real alpha : {-0.15, -0.03, 0.03, 0.15}) {
        TyreSlipInput in;
        in.kappa = 0.0;
        in.alpha = alpha;
        in.fz = 4000.0;
        in.vx = 20.0;
        const TyreForces f = model->evaluate(in);
        CHECK(f.fy == Approx(f.fy_pure).margin(1e-9 * std::max(1.0, std::abs(f.fy_pure))));
    }
}

TEST_CASE("Pacejka combined slip behaves qualitatively (friction ellipse)", "[vehicle][tyre]") {
    // PLAN.md P3 acceptance mentions "combined" curves explicitly: at a
    // fixed, sizeable longitudinal slip, adding lateral slip must not
    // INCREASE the longitudinal force (the friction-ellipse effect) - a
    // qualitative property every combined-slip weighting function must
    // have, checked here rather than against a specific numeric curve.
    TyreDesc desc;
    auto model = create_pacejka_tyre_model(desc);

    TyreSlipInput in;
    in.kappa = 0.10;
    in.fz = 4000.0;
    in.vx = 20.0;

    in.alpha = 0.0;
    const real fx_no_alpha = model->evaluate(in).fx;
    in.alpha = 0.15;
    const real fx_with_alpha = model->evaluate(in).fx;
    CHECK(std::abs(fx_with_alpha) <= std::abs(fx_no_alpha) + 1e-6);
}

TEST_CASE("Slip relaxation degenerates to plain integration at standstill", "[vehicle][tyre]") {
    // out/handover_p3.md decision 2: at vx_mag == 0 the ODE must reduce to
    // kappa_eq/alpha_eq += slip_velocity*dt exactly (the "damped stiffness"
    // standstill model), never a divide-by-zero or a frozen state that
    // ignores a genuine slip velocity.
    SlipRelaxationState s;
    step_slip_relaxation(s, /*slip_velocity_x=*/0.02, /*slip_velocity_y=*/-0.01, /*vx_mag=*/0.0,
                          /*sigma_x=*/0.1, /*sigma_y=*/0.15, /*dt=*/1.0 / 960.0);
    CHECK(s.kappa_eq == Approx(0.02 / 0.1 * (1.0 / 960.0)).margin(1e-12));
    CHECK(s.alpha_eq == Approx(-0.01 / 0.15 * (1.0 / 960.0)).margin(1e-12));

    // Zero slip velocity at standstill: state must HOLD (not decay to
    // zero) - this is exactly what lets a parked car's tyre keep resisting
    // gravity indefinitely (PLAN.md P3's parking-on-a-slope test).
    SlipRelaxationState s2;
    s2.kappa_eq = 0.0037;
    s2.alpha_eq = -0.0021;
    for (int i = 0; i < 1000; ++i) {
        step_slip_relaxation(s2, 0.0, 0.0, 0.0, 0.1, 0.15, 1.0 / 960.0);
    }
    CHECK(s2.kappa_eq == Approx(0.0037).margin(1e-15));
    CHECK(s2.alpha_eq == Approx(-0.0021).margin(1e-15));
}

TEST_CASE("Slip relaxation converges to the steady-state slip ratio at speed", "[vehicle][tyre]") {
    // At a constant vx_mag and slip velocity, kappa_eq must settle to
    // slip_velocity_x / vx_mag (the usual quasi-steady slip ratio
    // definition) after several relaxation-length time constants.
    SlipRelaxationState s;
    const real vx = 20.0;
    const real slip_velocity_x = 1.6; // -> steady-state kappa = 0.08
    const real sigma = 0.10;
    const real dt = 1.0 / 960.0;
    for (int i = 0; i < 20000; ++i) step_slip_relaxation(s, slip_velocity_x, 0.0, vx, sigma, sigma, dt);
    CHECK(s.kappa_eq == Approx(slip_velocity_x / vx).margin(1e-6));
}

// --- Case (h): relaxation_step_axis matches step_slip_relaxation per axis ---
TEST_CASE("relaxation_step_axis matches step_slip_relaxation bit-exactly, including the a<1e-6 branch",
          "[vehicle][tyre]") {
    for (real vx_mag : {0.0, 1e-8, 0.5, 20.0}) {
        for (real slip_velocity : {-3.0, -0.01, 0.0, 0.01, 3.0}) {
            SlipRelaxationState s;
            s.kappa_eq = 0.037;
            s.alpha_eq = -0.021;
            const real dt = 1.0 / 960.0;
            const real sigma_x = 0.10, sigma_y = 0.15;

            SlipRelaxationState reference = s;
            step_slip_relaxation(reference, slip_velocity, slip_velocity, vx_mag, sigma_x, sigma_y, dt);

            const RelaxStep rx = relaxation_step_axis(s.kappa_eq, slip_velocity, vx_mag, sigma_x, dt);
            const RelaxStep ry = relaxation_step_axis(s.alpha_eq, slip_velocity, vx_mag, sigma_y, dt);
            const real expected_kappa = std::clamp(rx.x_unclamped, real(-3.0), real(3.0));
            const real expected_alpha = std::clamp(ry.x_unclamped, real(-1.6), real(1.6));
            CHECK(reference.kappa_eq == expected_kappa);
            CHECK(reference.alpha_eq == expected_alpha);
        }
    }
}

namespace {
// Case (e)/(e2) fixture: every shipped tyre file, loaded exactly as
// VehicleInstance would (ps::io::load_tyre_json).
std::vector<std::string> shipped_tyre_paths() {
    const std::string dir = std::string(PS_SOURCE_DIR) + "/data/tyres/";
    return {dir + "passenger_car_225_45r17.json", dir + "tractor_front.json", dir + "tractor_rear.json"};
}
} // namespace

// --- Case (e): evaluate_with_slope analytic vs central FD, all 3 tyre files ---
TEST_CASE("evaluate_with_slope: analytic dFx/dkappa matches central FD on a grid (all shipped tyres)",
          "[vehicle][tyre]") {
    for (const std::string& path : shipped_tyre_paths()) {
        const TyreDesc desc = ps::io::load_tyre_json(path);
        auto model = create_pacejka_tyre_model(desc);
        INFO("tyre=" << path);

        for (real kappa = -1.0; kappa <= 1.0001; kappa += 0.05) {
            for (real alpha : {0.0, 0.05, -0.05, 0.2, -0.2, 0.5, -0.5}) {
                for (real fz_mult : {0.5, 1.0, 2.0}) {
                    for (real lambda_mu : {0.4, 1.0}) {
                        TyreSlipInput in;
                        in.kappa = kappa;
                        in.alpha = alpha;
                        in.fz = fz_mult * desc.fz0;
                        in.gamma = 0.0;
                        in.vx = 20.0;
                        in.lambda_mu = lambda_mu;

                        const TyreForcesWithSlope ws = model->evaluate_with_slope(in);
                        const TyreForces direct = model->evaluate(in);
                        CHECK(ws.forces.fx == direct.fx);
                        CHECK(ws.forces.fy == direct.fy);

                        constexpr real h = 1e-6;
                        TyreSlipInput plus = in;
                        plus.kappa += h;
                        TyreSlipInput minus = in;
                        minus.kappa -= h;
                        const real fd = (model->evaluate(plus).fx - model->evaluate(minus).fx) / (2.0 * h);

                        const real tol = 1e-6 * std::max(real(1.0), std::abs(fd));
                        INFO("kappa=" << kappa << " alpha=" << alpha << " fz_mult=" << fz_mult << " lambda_mu=" << lambda_mu);
                        CHECK(std::abs(ws.dfx_dkappa - fd) <= tol);
                    }
                }
            }
        }

        // Sanity: at kappa=0, alpha=0, the slope equals Kx exactly (all
        // three shipped tyre files have phx1=phx2=0, so SHx=0 and this
        // operating point is exactly x=0 - see pacejka_tyre_model.cpp's
        // evaluate_with_slope for the derivation).
        TyreSlipInput in0;
        in0.kappa = 0.0;
        in0.alpha = 0.0;
        in0.fz = desc.fz0;
        in0.gamma = 0.0;
        in0.vx = 20.0;
        const real dfz = 0.0;
        const real Kx = desc.fz0 * (desc.pkx1 + desc.pkx2 * dfz) * ps::math::exp(desc.pkx3 * dfz);
        const TyreForcesWithSlope ws0 = model->evaluate_with_slope(in0);
        CHECK(ws0.dfx_dkappa == Approx(Kx).epsilon(1e-12));
    }
}

// --- Case (e2): implicit tyre linearisation consistency ---
TEST_CASE("evaluate_with_slope: linearised Fx matches O(dkappa^2) against the true curve", "[vehicle][tyre]") {
    const TyreDesc desc = ps::io::load_tyre_json(shipped_tyre_paths()[0]);
    auto model = create_pacejka_tyre_model(desc);

    TyreSlipInput in;
    in.kappa = 0.05;
    in.alpha = 0.0;
    in.fz = desc.fz0;
    in.gamma = 0.0;
    in.vx = 10.0;

    const TyreForcesWithSlope ws = model->evaluate_with_slope(in);
    const real kappa0 = in.kappa;
    const real fx0 = ws.forces.fx;
    const real slope = ws.dfx_dkappa;

    // Central second difference for F'' at kappa0 (independent of the
    // analytic slope above).
    constexpr real h = 1e-4;
    TyreSlipInput p2 = in, m2 = in;
    p2.kappa += h;
    m2.kappa -= h;
    const real f_plus = model->evaluate(p2).fx;
    const real f_minus = model->evaluate(m2).fx;
    const real f_second = (f_plus - 2.0 * fx0 + f_minus) / (h * h);

    for (real dk : {0.01, -0.01, 0.02, -0.02}) {
        TyreSlipInput probe = in;
        probe.kappa = kappa0 + dk;
        const real f_true = model->evaluate(probe).fx;
        const real fx_lin = fx0 + slope * dk;
        const real actual_error = f_true - fx_lin;
        const real predicted_error = 0.5 * f_second * dk * dk;
        if (std::abs(predicted_error) > 1e-6) {
            const real ratio = actual_error / predicted_error;
            INFO("dk=" << dk << " actual=" << actual_error << " predicted=" << predicted_error);
            CHECK(ratio == Approx(1.0).margin(0.10)); // within 10%, plan section 11.2(e2)
        }
    }
}

// --- PHYS-036: self-aligning moment Mz0 sign/shape ---
//
// Before this fix, Mz0 carried the SAME sign as Fy0 (an aligning moment
// that steers further into a turn instead of resisting it) and peaked at
// about 21.7 deg (passenger) / 39.6 deg (tractor) of slip, both well past
// any plausible real tyre. The fix is entirely in data/tyres/*.json's
// "aligning" block (qdz1/qdz2 sign flipped so Mz0 = -t*Fy0 per Pacejka's
// own convention; qbz1 raised so the curve peaks at a realistic slip
// angle) - pacejka_tyre_model.cpp's formula itself is unchanged. These
// tests exercise every shipped tyre file so a future data edit that
// reintroduces the wrong sign or an unrealistic peak location is caught
// here, not just by the vehicle-level skidpad test.
constexpr real kDegToRad = 0.017453292519943295;

TEST_CASE("Pacejka self-aligning moment opposes lateral force for small slip angles (all shipped tyres)",
          "[vehicle][tyre]") {
    for (const std::string& path : shipped_tyre_paths()) {
        const TyreDesc desc = ps::io::load_tyre_json(path);
        auto model = create_pacejka_tyre_model(desc);
        INFO("tyre=" << path);

        for (real fz_mult : {0.5, 1.0, 1.5}) {
            for (real alpha_deg : {0.5, 1.0, 2.0, -0.5, -1.0, -2.0}) {
                TyreSlipInput in;
                in.kappa = 0.0;
                in.alpha = alpha_deg * kDegToRad;
                in.fz = fz_mult * desc.fz0;
                in.gamma = 0.0;
                in.vx = 20.0;
                const TyreForces f = model->evaluate(in);
                INFO("fz_mult=" << fz_mult << " alpha_deg=" << alpha_deg << " fy=" << f.fy << " mz=" << f.mz);
                // An aligning (restoring) moment: opposite sign to Fy, for
                // both signs of slip angle.
                CHECK(f.fy != 0.0);
                CHECK((f.fy > 0.0) != (f.mz > 0.0));
                CHECK(f.mz != 0.0);
            }
        }
    }
}

TEST_CASE("Pacejka self-aligning moment peaks within a realistic slip-angle range (all shipped tyres)",
          "[vehicle][tyre]") {
    for (const std::string& path : shipped_tyre_paths()) {
        const TyreDesc desc = ps::io::load_tyre_json(path);
        auto model = create_pacejka_tyre_model(desc);
        INFO("tyre=" << path);

        for (real fz_mult : {0.3, 1.0, 2.0}) {
            real peak_mz = 0.0;
            real peak_alpha_deg = 0.0;
            for (int i = 1; i <= 1800; ++i) { // 0.05 deg steps, up to 90 deg
                const real alpha_deg = 0.05 * static_cast<real>(i);
                TyreSlipInput in;
                in.kappa = 0.0;
                in.alpha = alpha_deg * kDegToRad;
                in.fz = fz_mult * desc.fz0;
                in.gamma = 0.0;
                in.vx = 20.0;
                const real mz = std::abs(model->evaluate(in).mz);
                if (mz > peak_mz) {
                    peak_mz = mz;
                    peak_alpha_deg = alpha_deg;
                }
            }
            INFO("fz_mult=" << fz_mult << " peak_alpha_deg=" << peak_alpha_deg << " peak_mz=" << peak_mz);
            // Realistic passenger/agricultural self-aligning-torque peak
            // range (Pacejka ch. 4; Milliken ch. 2's qualitative trail-vs-
            // slip-angle curves): well before Fy0's own ~8-10 deg peak.
            CHECK(peak_alpha_deg > 2.0);
            CHECK(peak_alpha_deg < 6.0);
        }
    }
}

TEST_CASE("Pacejka self-aligning moment stays bounded and same-signed past its peak (all shipped tyres)",
          "[vehicle][tyre]") {
    // This project's Mz0 is a single simplified B/C/D/E curve (this file's
    // top comment / pacejka_tyre_model.cpp's header comment), not the full
    // pneumatic-trail decomposition - it does NOT decay back to zero at
    // extreme slip the way a real tyre's Mz eventually does. The documented
    // behaviour is a plateau near its own peak magnitude, same sign as the
    // peak, never a runaway growth or a sign flip.
    for (const std::string& path : shipped_tyre_paths()) {
        const TyreDesc desc = ps::io::load_tyre_json(path);
        auto model = create_pacejka_tyre_model(desc);
        INFO("tyre=" << path);

        auto eval_mz = [&](real alpha_deg) {
            TyreSlipInput in;
            in.kappa = 0.0;
            in.alpha = alpha_deg * kDegToRad;
            in.fz = desc.fz0;
            in.gamma = 0.0;
            in.vx = 20.0;
            return model->evaluate(in).mz;
        };

        real peak_mz = 0.0;
        for (int i = 1; i <= 200; ++i) { // 0.05 deg steps up to 10 deg brackets the peak
            const real mz = std::abs(eval_mz(0.05 * static_cast<real>(i)));
            peak_mz = std::max(peak_mz, mz);
        }
        const real peak_sign = eval_mz(4.0) > 0.0 ? real(1.0) : real(-1.0);

        for (real alpha_deg : {20.0, 45.0, 80.0}) {
            const real mz = eval_mz(alpha_deg);
            INFO("alpha_deg=" << alpha_deg << " mz=" << mz << " peak_mz=" << peak_mz);
            CHECK(std::abs(mz) <= peak_mz * 1.02);
            CHECK(((mz > 0.0) == (peak_sign > 0.0)));
        }
    }
}
