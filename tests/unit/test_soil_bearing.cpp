// tests/unit/test_soil_bearing.cpp - soft soil under a wheel (ps/machine/soil_bearing.h, ISoilSource::bearing, WheelInstance::pre_solve).
//
// SB1 the law against a hand computation of Bekker/Wong's closed forms; SB2 no bearing = all zero, unlimited thrust;
// SB3 a source with no bearing (the default) or an invalid one leaves a car's state hash bit-identical;
// SB4 a car on soft ground sinks, is slower than on firm ground, and a heavier one is slower still / sinks deeper.
//
// Sabotage (2026-11-11): in WheelInstance::pre_solve drop the `pre_soil_resistance_` term and the lambda_mu cap -> SB4 fails (soft = firm speed, no slowdown);
// in soil_bearing.h make the resistance exponent n instead of n+1 -> SB1 fails.

#include "ps/machine/soil_bearing.h"
#include "ps/machine/soil_source.h"
#include "vehicle_test_helpers.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <memory>

using namespace ps;
using Catch::Approx;
using machine::SoilBearing;

namespace {

// Wong's example-class soft clay / sandy loam numbers ([AS] here, the law is what is tested): kc 5.27e3, kphi 1.515e6, n 0.5 (Wong, Table 2.? sandy loam
// range); c 1.7 kPa, phi 29 deg.
SoilBearing soft() {
    SoilBearing b;
    b.valid = true;
    b.kc = 5.27e3;
    b.kphi = 1.515e6;
    b.n = 0.7;
    b.cohesion_pa = 1700.0;
    b.phi_rad = 29.0 * 3.14159265358979323846 / 180.0;
    return b;
}

class SoftEverywhere final : public machine::ISoilSource {
public:
    explicit SoftEverywhere(SoilBearing b) : b_(b) {}
    [[nodiscard]] machine::SoilSample sample(real, real) const noexcept override { return {}; }
    [[nodiscard]] SoilBearing bearing(real, real) const noexcept override { return b_; }
private:
    SoilBearing b_;
};

class NoBearing final : public machine::ISoilSource {
public:
    [[nodiscard]] machine::SoilSample sample(real, real) const noexcept override { return {}; }
};

struct Run { std::uint64_t hash; real speed; real sinkage; };

Run drive(real mass, std::shared_ptr<const machine::ISoilSource> soil, real seconds = 6.0) {
    auto rig = ps_test::build_vehicle_rig(std::string(PS_SOURCE_DIR) + "/data/vehicles/car_sedan.json", mass, Vec3{2.0, 0.4, 0.15}, 0.6);
    if (soil) rig.world->set_soil_source(std::move(soil));
    ps_test::settle(*rig.world, 1.0);
    rig.world->set_control("gear_request", 1.0);
    rig.world->set_control("throttle", 0.7);
    real sink = 0.0;
    const int ticks = static_cast<int>(seconds / rig.world->dt());
    for (int i = 0; i < ticks; ++i) {
        rig.world->step();
        sink = std::max(sink, rig.world->wheel_soil_sinkage(rig.vehicle, 0));
    }
    return {rig.world->state_hash(), rig.world->get_motion(rig.chassis).linear.length(), sink};
}

} // namespace

TEST_CASE("SB1 wheel_soil_response equals the closed forms", "[soil_bearing][vehicle]") {
    const SoilBearing s = soft();
    const real fz = 5000.0, b = 0.3, r = 0.5;
    const auto resp = machine::wheel_soil_response(s, fz, b, r);
    REQUIRE_FALSE(resp.bottomed);
    // the sinkage carries exactly the load: p * b * l = Fz
    const real mod = s.kc / b + s.kphi;
    const real z = resp.sinkage_m;
    const real l = std::sqrt(2.0 * r * z - z * z);
    CHECK(mod * std::pow(z, s.n) * b * l == Approx(fz).epsilon(1e-9));
    CHECK(resp.patch_length_m == Approx(l).epsilon(1e-12));
    CHECK(resp.resistance_n == Approx(b * mod * std::pow(z, s.n + 1.0) / (s.n + 1.0)).epsilon(1e-9));
    CHECK(resp.thrust_limit_n == Approx(b * l * s.cohesion_pa + fz * std::tan(s.phi_rad)).epsilon(1e-9));
    // more load sinks deeper; a wider tyre sinks less
    CHECK(machine::wheel_soil_response(s, 2.0 * fz, b, r).sinkage_m > z);
    CHECK(machine::wheel_soil_response(s, fz, 2.0 * b, r).sinkage_m < z);
    // a load the soil cannot carry: down to the hub
    const auto sunk = machine::wheel_soil_response(s, 5.0e6, b, r);
    CHECK(sunk.bottomed);
    CHECK(sunk.sinkage_m == Approx(r));
}

TEST_CASE("SB2 no bearing or no load gives nothing", "[soil_bearing][vehicle]") {
    const auto none = machine::wheel_soil_response(SoilBearing{}, 5000.0, 0.3, 0.5);
    CHECK(none.sinkage_m == 0.0);
    CHECK(none.resistance_n == 0.0);
    CHECK(none.thrust_limit_n > 1.0e20);
    const auto light = machine::wheel_soil_response(soft(), 0.0, 0.3, 0.5);
    CHECK(light.sinkage_m == 0.0);
}

TEST_CASE("SB3 a source without bearing leaves the car bit-identical", "[soil_bearing][vehicle]") {
    const Run base = drive(1500.0, nullptr, 3.0);
    const Run none = drive(1500.0, std::make_shared<NoBearing>(), 3.0);
    const Run invalid = drive(1500.0, std::make_shared<SoftEverywhere>(SoilBearing{}), 3.0);
    CHECK(none.hash == base.hash);
    CHECK(invalid.hash == base.hash);
    CHECK(base.sinkage == 0.0);
}

TEST_CASE("SB4 a car on soft ground sinks and slows, a heavier one more", "[soil_bearing][vehicle]") {
    auto soil = std::make_shared<SoftEverywhere>(soft());
    const Run firm = drive(1500.0, nullptr);
    const Run light = drive(1500.0, soil);
    const Run heavy = drive(3000.0, soil);
    INFO("firm " << firm.speed << " m/s, light " << light.speed << " m/s sink " << light.sinkage << ", heavy " << heavy.speed << " m/s sink " << heavy.sinkage);
    CHECK(light.sinkage > 0.01);
    CHECK(heavy.sinkage > light.sinkage);
    CHECK(light.speed < 0.9 * firm.speed);
    CHECK(heavy.speed < light.speed);
}
