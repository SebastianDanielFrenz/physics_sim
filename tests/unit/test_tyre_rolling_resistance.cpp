// tests/unit/test_tyre_rolling_resistance.cpp -- tyre rolling resistance with a derived speed term
// (2026-10-06, out/tyre_crr_progress.md): MF 6.1's qsy3*|v/v0| term, plus an optional tyre-file
// "rolling_resistance" block that derives qsy1/qsy3/qsy4 in closed form from two anchors.
//
//   crr(v) = qsy1 * (1 + A*(v/V_r) + B*(v/V_r)^4),  crr(f*V_r) = factor*crr_base, crr(V_r) = factor_at_rated*crr_base
//   B = ((F-1)*f - (factor-1)) / (f - f^4),  A = (F-1) - B           (F = factor_at_rated)
//
// R1 exactness at both anchors (1e-12 relative) incl. the A == 0 and B == 0 edges, R2 every loader rejection with
// its message, R3 files without the block / without qsy3 are untouched (qsy3 0, speed term bit-identical to the old
// qsy4 formula), R4 the shipped tyre files' anchors, R5 raw qsy3 is legal.

#include "ps/io/vehicle_io.h"
#include "ps/world/world.h"
#include <array>
#include "ps/math/transcendental.h"
#include "ps/vehicle/tyre_model.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <string>

using namespace ps;
using Catch::Matchers::ContainsSubstring;

namespace {

struct TempTyre {
    std::filesystem::path path;
    explicit TempTyre(const std::string& body) {
        path = std::filesystem::temp_directory_path() / ("ps_test_tyre_crr_" + std::to_string(reinterpret_cast<std::uintptr_t>(this)) + ".json");
        std::ofstream out(path);
        out << R"({"format": "physics_sim.tyre/1", "name": "crr_fixture", "source": "test fixture", "v0": 16.7,)" << body << "}";
    }
    ~TempTyre() {
        std::error_code ec;
        std::filesystem::remove(path, ec);
    }
    [[nodiscard]] vehicle::TyreDesc load() const { return io::load_tyre_json(path.string()); }
};

std::string block(const std::string& rated, const std::string& base, const std::string& frac, const std::string& factor, const std::string& at_rated,
                  const std::string& extra = "") {
    return R"("rolling_resistance": {"rated_speed_kmh": )" + rated + R"(, "crr_base": )" + base + R"(, "rise_at_fraction": {"fraction": )" + frac +
           R"(, "factor": )" + factor + "}, \"factor_at_rated\": " + at_rated + extra + "}";
}

real crr_of(const vehicle::TyreDesc& d, real v) { return d.qsy1 + vehicle::tyre_rolling_speed_term(d, v); }

void check_anchors(const vehicle::RollingResistanceAnchors& a, real v0) {
    const auto c = vehicle::derive_rolling_resistance(a, v0);
    vehicle::TyreDesc d;
    d.v0 = v0;
    d.qsy1 = c.qsy1;
    d.qsy3 = c.qsy3;
    d.qsy4 = c.qsy4;
    const real vr = a.rated_speed_kmh / real(3.6);
    CHECK(crr_of(d, a.fraction * vr) == Catch::Approx(a.factor * a.crr_base).epsilon(1e-12).margin(0));
    CHECK(crr_of(d, vr) == Catch::Approx(a.factor_at_rated * a.crr_base).epsilon(1e-12).margin(0));
    CHECK(crr_of(d, 0.0) == a.crr_base);
}

} // namespace

TEST_CASE("R1 the closed-form derivation is exact at both anchors", "[tyre][vehicle][unit]") {
    SECTION("coordinator anchors 0.8 / 1.5 / 2.0 at several rated speeds and v0") {
        for (const real vr : {real(65.0), real(270.0), real(500.0)}) {
            check_anchors({vr, 0.01, 0.8, 1.5, 2.0}, 16.7);
            check_anchors({vr, 0.022, 0.8, 1.5, 2.0}, 8.3);
        }
        const auto c = vehicle::derive_rolling_resistance({500.0, 0.01, 0.8, 1.5, 2.0}, 16.7);
        const real x_r = (500.0 / 3.6) / 16.7;
        const real b_exact = (1.0 * 0.8 - 0.5) / (0.8 - std::pow(0.8, 4.0)); // 0.768443, A = 1 - B = 0.231557
        CHECK(c.qsy3 * x_r / c.qsy1 == Catch::Approx(1.0 - b_exact).epsilon(1e-12));
        CHECK(c.qsy4 * std::pow(x_r, 4.0) / c.qsy1 == Catch::Approx(b_exact).epsilon(1e-12));
    }
    SECTION("other fractions and factors") {
        check_anchors({300.0, 0.012, 0.5, 1.2, 1.9}, 16.7);
        check_anchors({240.0, 0.015, 0.9, 3.0, 3.5}, 16.7);
    }
    SECTION("edge B == 0: purely linear speed term") {
        // factor-1 = (F-1)*f  ->  B = 0, A = F-1; exactly representable
        const auto c = vehicle::derive_rolling_resistance({270.0, 0.01, 0.5, 1.5, 2.0}, 16.7);
        CHECK(c.qsy4 == 0.0);
        CHECK(c.qsy3 > 0.0);
        check_anchors({270.0, 0.01, 0.5, 1.5, 2.0}, 16.7);
    }
    SECTION("edge A == 0: purely quartic speed term") {
        // factor-1 = (F-1)*f^4 -> A = 0, B = F-1; f = 0.5 -> f^4 = 0.0625
        const auto c = vehicle::derive_rolling_resistance({270.0, 0.01, 0.5, 1.0625, 2.0}, 16.7);
        CHECK(c.qsy3 == 0.0);
        CHECK(c.qsy4 > 0.0);
        check_anchors({270.0, 0.01, 0.5, 1.0625, 2.0}, 16.7);
    }
}

TEST_CASE("R2 the loader rejects bad rolling_resistance blocks, each with its message", "[tyre][vehicle][io][unit]") {
    const auto expect_throw = [](const std::string& body, const char* message) {
        TempTyre f(body);
        REQUIRE_THROWS_AS(f.load(), std::runtime_error);
        CHECK_THROWS_WITH(f.load(), ContainsSubstring(message));
    };
    SECTION("accepts a valid block") {
        TempTyre f(block("270", "0.01", "0.8", "1.5", "2.0"));
        const auto d = f.load();
        CHECK(d.qsy1 == 0.01);
        CHECK(d.qsy3 > 0.0);
        CHECK(d.qsy4 > 0.0);
    }
    SECTION("negative speed coefficient") {
        // factor-1 / (F-1) = 0.9 > fraction 0.8: B < 0 ... and the other side: 0.3 < f^4 = 0.4096: A < 0
        expect_throw(block("270", "0.01", "0.8", "1.9", "2.0"), "negative speed coefficient");
        expect_throw(block("270", "0.01", "0.8", "1.3", "2.0"), "negative speed coefficient");
    }
    SECTION("explicit qsy1/qsy3/qsy4 next to the block") {
        expect_throw(R"("qsy3": 0.001,)" + block("270", "0.01", "0.8", "1.5", "2.0"), "explicit 'qsy3'");
        expect_throw(R"("qsy4": 0.0002,)" + block("270", "0.01", "0.8", "1.5", "2.0"), "explicit 'qsy4'");
        expect_throw(R"("qsy1": 0.01,)" + block("270", "0.01", "0.8", "1.5", "2.0"), "explicit 'qsy1'");
    }
    SECTION("fraction outside (0,1)") {
        expect_throw(block("270", "0.01", "0", "1.5", "2.0"), "fraction must be in (0, 1)");
        expect_throw(block("270", "0.01", "1.0", "1.5", "2.0"), "fraction must be in (0, 1)");
        expect_throw(block("270", "0.01", "1.2", "1.5", "2.0"), "fraction must be in (0, 1)");
    }
    SECTION("factor below 1") { expect_throw(block("270", "0.01", "0.8", "0.9", "2.0"), "factor must be >= 1"); }
    SECTION("factor_at_rated not above factor") {
        expect_throw(block("270", "0.01", "0.8", "1.5", "1.5"), "factor_at_rated must be > ");
        expect_throw(block("270", "0.01", "0.8", "1.5", "1.2"), "factor_at_rated must be > ");
    }
    SECTION("crr_base and rated_speed_kmh must be positive") {
        expect_throw(block("270", "0", "0.8", "1.5", "2.0"), "crr_base must be > 0");
        expect_throw(block("270", "-0.01", "0.8", "1.5", "2.0"), "crr_base must be > 0");
        expect_throw(block("0", "0.01", "0.8", "1.5", "2.0"), "rated_speed_kmh must be > 0");
        expect_throw(block("-270", "0.01", "0.8", "1.5", "2.0"), "rated_speed_kmh must be > 0");
    }
    SECTION("unknown keys") {
        expect_throw(block("270", "0.01", "0.8", "1.5", "2.0", R"(, "bogus": 1)"), "unknown key 'bogus'");
        expect_throw(R"("rolling_resistance": {"rated_speed_kmh": 270, "crr_base": 0.01, "rise_at_fraction": {"fraction": 0.8, "factor": 1.5, "x": 1}, "factor_at_rated": 2.0})",
                     "rise_at_fraction: unknown key 'x'");
    }
    SECTION("missing keys") {
        expect_throw(R"("rolling_resistance": {"rated_speed_kmh": 270, "crr_base": 0.01, "factor_at_rated": 2.0})", "missing required object 'rise_at_fraction'");
        expect_throw(R"("rolling_resistance": {"crr_base": 0.01, "rise_at_fraction": {"fraction": 0.8, "factor": 1.5}, "factor_at_rated": 2.0})",
                     "missing required number 'rated_speed_kmh'");
    }
}

TEST_CASE("R3 a tyre without the block and without qsy3 is untouched", "[tyre][vehicle][io][unit]") {
    TempTyre f(R"("qsy1": 0.012, "qsy2": 0.0, "qsy4": 0.0003)");
    const auto d = f.load();
    CHECK(d.qsy1 == 0.012);
    CHECK(d.qsy3 == 0.0);
    CHECK(d.qsy4 == 0.0003);
    // The speed term is bit-identical to the pre-qsy3 formula qsy4*(v/v0)^4.
    for (const real v : {0.0, 1.0, 16.7, 55.5, 102.8, -30.0}) {
        const real old_term = d.qsy4 * math::pow(v / d.v0, real(4.0));
        CHECK(vehicle::tyre_rolling_speed_term(d, v) == old_term);
    }
    // Defaults: a tyre that names nothing keeps the old defaults.
    TempTyre g("\"qsy2\": 0.0");
    const auto dd = g.load();
    CHECK(dd.qsy1 == 0.01);
    CHECK(dd.qsy3 == 0.0);
    CHECK(dd.qsy4 == 0.0002);
}

TEST_CASE("R5 raw qsy3 stays legal and is symmetric in the speed sign", "[tyre][vehicle][io][unit]") {
    TempTyre f(R"("qsy1": 0.01, "qsy3": 0.002, "qsy4": 0.0)");
    const auto d = f.load();
    CHECK(d.qsy3 == 0.002);
    CHECK(vehicle::tyre_rolling_speed_term(d, 33.4) == Catch::Approx(0.002 * 2.0).epsilon(1e-12));
    CHECK(vehicle::tyre_rolling_speed_term(d, -33.4) == vehicle::tyre_rolling_speed_term(d, 33.4));
}

TEST_CASE("R4 the shipped tyre files carry the documented anchors", "[tyre][vehicle][io][unit]") {
    struct Row {
        const char* file;
        real rated_kmh, base;
    };
    const Row rows[] = {{"hyper_front_265_35r20", 500.0, 0.01}, {"hyper_rear_325_30r21", 500.0, 0.01}, {"passenger_car_225_45r17", 270.0, 0.010},
                        {"tractor_front", 65.0, 0.020},        {"tractor_rear", 65.0, 0.022}};
    for (const Row& r : rows) {
        INFO(r.file);
        const auto d = io::load_tyre_json(std::string(PS_SOURCE_DIR) + "/data/tyres/" + r.file + ".json");
        const real vr = r.rated_kmh / real(3.6);
        CHECK(d.qsy1 == r.base);
        CHECK(crr_of(d, 0.8 * vr) == Catch::Approx(1.5 * r.base).epsilon(1e-12));
        CHECK(crr_of(d, vr) == Catch::Approx(2.0 * r.base).epsilon(1e-12));
    }
    // The hypercar at 370 km/h: ~1.40 x base (was 30x with the raw qsy4 0.0002).
    const auto h = io::load_tyre_json(std::string(PS_SOURCE_DIR) + "/data/tyres/hyper_rear_325_30r21.json");
    CHECK(crr_of(h, 370.0 / 3.6) / h.qsy1 == Catch::Approx(1.40).margin(0.005));
}


TEST_CASE("Rolling selector matches rating and reversibly preserves MF coefficients", "[tyre][rolling_selector][unit]") {
    auto d = io::load_tyre_json(std::string(PS_SOURCE_DIR) + "/data/tyres/hyper_rear_325_30r21.json");
    const auto original = d;
    const real vr = d.rolling_resistance_anchors.rated_speed_kmh / 3.6;
    REQUIRE(vr > 0.0);
    const auto legacy = vehicle::tyre_rolling_speed_term(d, 70.0);
    const std::uint64_t seed = 14695981039346656037ULL;
    CHECK(vehicle::fold_rolling_resistance_hash(d, seed) == seed);
    vehicle::select_rolling_resistance_speed_law(d, vehicle::RollingResistanceSpeedLaw::Quadratic);
    CHECK(crr_of(d, vr) == Catch::Approx(original.rolling_resistance_anchors.crr_base * original.rolling_resistance_anchors.factor_at_rated));
    CHECK(vehicle::tyre_rolling_speed_term(d, 0.0) == 0.0);
    CHECK(vehicle::tyre_rolling_speed_term(d, vr * 0.5) == Catch::Approx(vehicle::tyre_rolling_speed_term(d, vr) * 0.25));
    CHECK(vehicle::tyre_rolling_speed_term(d, -vr) == vehicle::tyre_rolling_speed_term(d, vr));
    CHECK(d.qsy1 == original.qsy1);
    CHECK(d.qsy3 == original.qsy3);
    CHECK(d.qsy4 == original.qsy4);
    const auto hash = vehicle::fold_rolling_resistance_hash(d, seed);
    CHECK(hash != seed);
    vehicle::select_rolling_resistance_speed_law(d, vehicle::RollingResistanceSpeedLaw::Quadratic);
    CHECK(vehicle::fold_rolling_resistance_hash(d, seed) == hash);
    auto altered = d; altered.rolling_resistance_quadratic += 0.001;
    CHECK(vehicle::fold_rolling_resistance_hash(altered, seed) != hash);
    altered = d; altered.rolling_resistance_reference_speed_m_s += 1.0;
    CHECK(vehicle::fold_rolling_resistance_hash(altered, seed) != hash);
    vehicle::select_rolling_resistance_speed_law(d, vehicle::RollingResistanceSpeedLaw::FourthPower);
    CHECK(vehicle::tyre_rolling_speed_term(d, 70.0) == legacy);
    CHECK(vehicle::fold_rolling_resistance_hash(d, seed) == seed);
    vehicle::select_rolling_resistance_speed_law(d, vehicle::RollingResistanceSpeedLaw::Quadratic);
    CHECK(vehicle::fold_rolling_resistance_hash(d, seed) == hash);
}

TEST_CASE("Rolling selector calibrates raw QSY and rejects invalid configuration atomically", "[tyre][rolling_selector][unit]") {
    vehicle::TyreDesc d;
    d.qsy3 = 0.001;
    const auto legacy = vehicle::tyre_rolling_speed_term(d, d.v0);
    vehicle::select_rolling_resistance_speed_law(d, vehicle::RollingResistanceSpeedLaw::Quadratic);
    CHECK(vehicle::tyre_rolling_speed_term(d, d.v0) == legacy);
    vehicle::select_rolling_resistance_speed_law(d, vehicle::RollingResistanceSpeedLaw::FourthPower);
    const auto custom = vehicle::tyre_rolling_speed_term(d, 50.0);
    vehicle::select_rolling_resistance_speed_law(d, vehicle::RollingResistanceSpeedLaw::Quadratic, 50.0);
    CHECK(vehicle::tyre_rolling_speed_term(d, 50.0) == custom);
    const auto hash = vehicle::fold_rolling_resistance_hash(d, 1);
    CHECK_THROWS_AS(vehicle::select_rolling_resistance_speed_law(d, vehicle::RollingResistanceSpeedLaw::Quadratic, -1.0), std::invalid_argument);
    CHECK(vehicle::fold_rolling_resistance_hash(d, 1) == hash);
    d.qsy4 = -0.1;
    CHECK_THROWS_AS(vehicle::select_rolling_resistance_speed_law(d, vehicle::RollingResistanceSpeedLaw::Quadratic), std::invalid_argument);
    d.qsy4 = 0.0002; d.v0 = std::numeric_limits<real>::infinity();
    CHECK_THROWS_AS(vehicle::select_rolling_resistance_speed_law(d, vehicle::RollingResistanceSpeedLaw::Quadratic), std::invalid_argument);
    TempTyre t(R"("qsy3": 0.001, "rolling_resistance_speed_law": "quadratic")");
    CHECK(t.load().rolling_resistance_speed_law == vehicle::RollingResistanceSpeedLaw::Quadratic);
    TempTyre bad(R"("rolling_resistance_speed_law": "cubic")");
    CHECK_THROWS(bad.load());
    TempTyre wrong(R"("rolling_resistance_speed_law": 3)");
    CHECK_THROWS(wrong.load());
}

TEST_CASE("Rolling law enters vehicle hash before contact and remains worker deterministic", "[tyre][rolling_selector][determinism][unit]") {
    const auto run = [](vehicle::RollingResistanceSpeedLaw law, int workers, bool restore) {
        WorldConfig cfg; cfg.job_workers = workers; cfg.gravity = {};
        World world(cfg);
        vehicle::VehicleDesc car; car.name = "rolling_hash";
        vehicle::WheelDesc wheel; wheel.name = "test_wheel";
        if (restore) vehicle::select_rolling_resistance_speed_law(wheel.tyre, vehicle::RollingResistanceSpeedLaw::Quadratic);
        vehicle::select_rolling_resistance_speed_law(wheel.tyre, law);
        car.wheels.push_back(wheel);
        BodyDesc body; body.mass = 1000; body.motion = BodyMotionType::Dynamic;
        body.shape = BoxShape{{1,.5,.2}}; body.allow_sleep = false;
        const auto b = world.create_body(body);
        world.create_vehicle(car, b);
        std::array<std::uint64_t, 8> hashes{};
        for (auto& h : hashes) { world.step(); h = world.state_hash(); }
        return hashes;
    };
    using Law = vehicle::RollingResistanceSpeedLaw;
    const auto legacy = run(Law::FourthPower, 1, false);
    CHECK(legacy == run(Law::FourthPower, 1, true));
    const auto quadratic = run(Law::Quadratic, 1, false);
    CHECK(quadratic != legacy);
    CHECK(quadratic == run(Law::Quadratic, 4, false));
}
