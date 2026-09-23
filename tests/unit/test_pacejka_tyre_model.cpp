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

#include "ps/math/transcendental.h"
#include "ps/vehicle/tyre_model.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>

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
