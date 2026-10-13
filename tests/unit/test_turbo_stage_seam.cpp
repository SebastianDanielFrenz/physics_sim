// Stage seam S0 (2026-10-13): the turbo stage map's coverage boundary. Outside a stage map's certified coverage (flow
// beyond a speed line's choke edge, speed outside the lines with data) TurboPair used to blend into the authored
// head/work law, which under-states head and Euler work 2.5-4x near the choke edge (boost B0 item 3); it now reads the
// map's own continuation (TurboStageMaps::compressor_extended). Fixtures: tests/support/fixtures/assists/v12/
// (owner setup, racing_game large.json) and tests/support/fixtures/turbo_seam/ (racing_game standard.json, stock 1.8 bar).
//
// Bands (all [AS], set after the exploration runs - see the S0 progress notes): frozen rotor, 2.5k rpm samples,
//  (a) at a frozen gate and a fixed flow, charge and compressor power never fall with rotor speed; with a speed-density
//      engine (pump mode) charge, flow and compressor power never fall;
//  (b) fixed flow: the compressor power and the turbine power rise by at most 6 dN/N between adjacent samples (Euler work
//      of a backswept impeller: d lnW / d lnN = (2 - x)/(1 - x) <= 5 for x <= 0.75 of the zero-work flow coefficient,
//      +20 %); pump mode: the gauge charge rises by at most 6 dN/N relative (same bound through head ~ work);
//  (c) with the live boost PID the turbine power reverses direction at most once over the sweep.
// Recorded sabotages (2026-10-13): the pre-S0 blend (authored law for the uncovered weight) fails the sweep case on all four
// sweeps (owner fixed 5, owner pump 18, standard fixed 2, standard pump 13 violations; standard charge 1.659 -> 1.569 bar
// at 100-105k, owner compressor power +56 % for a 2.5 % speed step) and the live case (standard: 3 turbine-power
// reversals); work beyond the choke edge halved instead of held fails the line case, the continuity case (38 / 1 / 1
// jumps on the hand / owner / standard maps) and all four sweeps; the coverage check without the per-turbo division of
// the engine flow flags the covering map. A constant-flow blend between speed lines (measured before the choke-aligned
// rule) gave the standard pump charge 1.775 / 1.864 / 2.241 bar at 110 / 115 / 120k (a +20 % step, band 13.6 %).
#include "drivetrain/turbo_pair.h"

#include "mvem_fixtures.h"
#include "powertrain_multi_diff_desc.h" // ps_test::load_sedan_desc()

#include "ps/drivetrain/engine_rules.h"
#include "ps/drivetrain/powertrain_desc.h"
#include "ps/drivetrain/turbo_maps.h"
#include "ps/drivetrain/turbo_stage_maps.h"
#include "ps/io/surface_table.h"
#include "ps/io/vehicle_io.h"
#include "ps/jobs/debug_alloc.h"
#include "ps/vehicle/vehicle_reset.h"
#include "ps/world/world.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <string>
#include <variant>
#include <vector>

using namespace ps;
using ps::drivetrain::TurboPair;
using ps::drivetrain::TurboPairDesc;
using ps::drivetrain::TurboStageMaps;

namespace {

std::string env_or(const char* name, const std::string& fallback) {
#if defined(_MSC_VER) || defined(_WIN32)
    char* buf = nullptr;
    std::size_t len = 0;
    if (_dupenv_s(&buf, &len, name) != 0 || buf == nullptr) return fallback;
    std::string out(buf);
    std::free(buf);
    return out.empty() ? fallback : out;
#else
    const char* v = std::getenv(name);
    return (v && *v) ? std::string(v) : fallback;
#endif
}
real env_num(const char* name, real fallback) {
    const std::string s = env_or(name, "");
    return s.empty() ? fallback : std::stod(s);
}

const char* kOwnerCfg = "tests/support/fixtures/assists/v12/turbo_config_v12.json";
const char* kStandardCfg = "tests/support/fixtures/turbo_seam/turbo_config_standard.json";
// Speed-density pump constants, air = k * charge_pa / charge_k [kg/s per Pa/K]: VE 1 [AS] x 5.066e-3 m3 (the hyper's
// 92 x 95.25 mm V8, [D]) x n/120 / 287.05, at 7500 rpm (owner) and 6000 rpm (standard).
constexpr real kOwnerPump = 1.103e-3;
constexpr real kStandardPump = 8.824e-4;
constexpr real kExhOverAir = 1.0625; // [AS] exhaust/air mass ratio at lambda ~1
constexpr real kPortK = 1150.0;      // [AS] exhaust port temperature

// The pair from a configuration fixture with the exhaust wall's thermal mass switched off (the wall settles over many
// seconds and would make the turbine power depend on how long a sweep point ran).
TurboPairDesc load_pair(const char* rel) {
    TurboPairDesc d = io::load_turbo_configuration_json(std::string(PS_SOURCE_DIR) + "/" + rel);
    d.exhaust_wall_gas_conductance_w_k = 0.0;
    return d;
}

struct SweepPoint {
    real rpm = 0.0, charge_bar = 0.0, air_kg_s = 0.0, comp_kw = 0.0, turb_kw = 0.0, exhaust_bar = 0.0;
    bool comp_stage = false, extended = false, fallback = false, turb_stage = false;
};

// Frozen-rotor sweep: for each rotor speed a fresh pair, the rotor energy reset to that speed before every step, the
// wastegate frozen at `gate` (hold_control) so the PID cannot switch the turbine path, plate 1, ISA ambient, 1 s at 960 Hz.
// pump_k_per_pa_k == 0: fixed air/exhaust flows (the B0 diagnostic). > 0: a speed-density engine at fixed speed with a wide-
// open throttle - air = pump_k * charge_pa / charge_k from the previous step, exhaust = air * exhaust_over_air.
std::vector<SweepPoint> frozen_sweep(const TurboPairDesc& td, real air, real exh_over_air, real port_k, real pump_k,
                                     real rpm_lo, real rpm_hi, real rpm_step, real gate) {
    std::vector<SweepPoint> out;
    const real h = 1.0 / 960.0;
    for (real rpm = rpm_lo; rpm <= rpm_hi + 1e-9; rpm += rpm_step) {
        TurboPair tp(td);
        tp.reset(101325.0, 288.15);
        const bool hold = gate >= 0.0; // gate < 0: the live boost PID (the B0 diagnostic's behaviour)
        tp.wastegate = hold ? gate : 0.0;
        const real omega = rpm * 6.2831853071795864769 / 60.0;
        const real e0 = 0.5 * td.rotor_inertia_kgm2 * omega * omega;
        real a = air;
        for (int i = 0; i < 960; ++i) {
            if (pump_k > 0.0) a = pump_k * tp.charge_pa / tp.charge_k;
            tp.energy_j = e0;
            tp.step(h, 101325.0, 288.15, a, a * exh_over_air, port_k, 1.0, 0.0, nullptr, hold);
        }
        SweepPoint p;
        p.rpm = rpm;
        p.charge_bar = (tp.charge_pa - 101325.0) / 1e5;
        p.air_kg_s = a;
        p.comp_kw = tp.compressor_w / 1000.0;
        p.turb_kw = tp.turbine_w / 1000.0;
        p.exhaust_bar = tp.exhaust_pa / 1e5;
        p.comp_stage = tp.compressor_stage_used;
        p.extended = tp.compressor_stage_extended;
        p.fallback = tp.stage_map_fallback;
        p.turb_stage = tp.turbine_stage_used;
        out.push_back(p);
    }
    return out;
}

void print_sweep(const char* tag, const std::vector<SweepPoint>& s) {
    std::printf("%s rpm charge_bar air_kg_s comp_kw turb_kw exh_barabs comp_stage extended fallback turb_stage\n", tag);
    for (const auto& p : s)
        std::printf("%s %7.0f %8.4f %7.4f %8.2f %8.2f %7.4f %d %d %d %d\n", tag, p.rpm, p.charge_bar, p.air_kg_s, p.comp_kw,
                    p.turb_kw, p.exhaust_bar, int(p.comp_stage), int(p.extended), int(p.fallback), int(p.turb_stage));
}

constexpr real kStepBand = 6.0; // [AS] d ln(power) / d ln(N) bound, see the file header
constexpr real kMonoTol = 1e-9; // relative tolerance of the monotonicity checks

bool non_decreasing(real a, real b) { return b >= a - kMonoTol * std::max(real(1), std::abs(a)); }

// Band (a), plus (b) for the selected mode. Returns the number of violations (each one also reported through CHECK).
int check_sweep(const char* tag, const std::vector<SweepPoint>& s, bool pump) {
    int bad = 0;
    for (std::size_t k = 0; k + 1 < s.size(); ++k) {
        const SweepPoint& p = s[k];
        const SweepPoint& q = s[k + 1];
        const real dn = (q.rpm - p.rpm) / p.rpm;
        const real band = kStepBand * dn;
        const bool charge_ok = non_decreasing(p.charge_bar, q.charge_bar);
        const bool comp_ok = non_decreasing(p.comp_kw, q.comp_kw);
        const bool air_ok = !pump || non_decreasing(p.air_kg_s, q.air_kg_s);
        bool step_ok = true;
        if (!pump) {
            step_ok = q.comp_kw - p.comp_kw <= band * p.comp_kw &&
                      std::abs(q.turb_kw - p.turb_kw) <= band * std::max(std::abs(p.turb_kw), std::abs(q.turb_kw));
        } else if (p.charge_bar > 0.0) {
            step_ok = q.charge_bar - p.charge_bar <= band * p.charge_bar;
        }
        if (!(charge_ok && comp_ok && air_ok && step_ok)) {
            ++bad;
            UNSCOPED_INFO(tag << " " << p.rpm << " -> " << q.rpm << " rpm: charge " << p.charge_bar << " -> " << q.charge_bar
                              << " bar, compressor " << p.comp_kw << " -> " << q.comp_kw << " kW, turbine " << p.turb_kw
                              << " -> " << q.turb_kw << " kW, air " << p.air_kg_s << " -> " << q.air_kg_s);
            CHECK(charge_ok);
            CHECK(comp_ok);
            CHECK(air_ok);
            CHECK(step_ok);
        }
    }
    return bad;
}

int turbine_reversals(const std::vector<SweepPoint>& s) {
    int reversals = 0, last_sign = 0;
    for (std::size_t k = 0; k + 1 < s.size(); ++k) {
        const real d = s[k + 1].turb_kw - s[k].turb_kw;
        if (std::abs(d) <= 1e-9 * std::max(real(1), std::abs(s[k].turb_kw))) continue;
        const int sign = d > 0.0 ? 1 : -1;
        if (last_sign != 0 && sign != last_sign) ++reversals;
        last_sign = sign;
    }
    return reversals;
}

// A small hand-built map: lines 0 (no data), 50k (valid flows 0..0.2), 100k (0..0.3), 150k (0..0.5 = the axis end).
TurboStageMaps hand_map() {
    TurboStageMaps m;
    const std::vector<real> sp{0.0, 50000.0, 100000.0, 150000.0};
    const std::vector<real> fl{0.0, 0.1, 0.2, 0.3, 0.4, 0.5};
    const int edge[4] = {-1, 2, 3, 5};
    m.compressor_pr.x = m.compressor_work_j_kg.x = sp;
    m.compressor_pr.y = m.compressor_work_j_kg.y = fl;
    m.compressor_pr.values.assign(sp.size() * fl.size(), 1.0);
    m.compressor_work_j_kg.values.assign(sp.size() * fl.size(), 0.0);
    m.compressor_valid.assign(sp.size() * fl.size(), false);
    for (std::size_t i = 0; i < sp.size(); ++i)
        for (std::size_t j = 0; j < fl.size(); ++j) {
            const real r = sp[i] / 150000.0, f = fl[j] / 0.5;
            m.compressor_pr.values[i * fl.size() + j] = 1.0 + 2.0 * r * r * (1.0 - 0.6 * f * f);
            m.compressor_work_j_kg.values[i * fl.size() + j] = 9e4 * r * r * (1.0 + 0.2 * fl[j]);
            m.compressor_valid[i * fl.size() + j] = int(j) <= edge[i];
        }
    return m;
}

// --- vehicle rig (test_turbo_vehicle.cpp's pattern): car_sedan with the mvem electronic engine and a turbo pair -------
vehicle::VehicleDesc turbo_sedan(const TurboPairDesc& pair) {
    vehicle::VehicleDesc d = ps_test::load_sedan_desc();
    drivetrain::SimulatedEngineDesc eng = ps::test::mvem::electronic_engine();
    eng.turbo_pair = pair;
    for (auto& c : d.powertrain.components) {
        if (const auto* tm = std::get_if<drivetrain::TorqueMapEngineDesc>(&c.params)) {
            eng.crank_axis_local = tm->crank_axis_local;
            c.params = eng;
            break;
        }
    }
    return d;
}

struct VehicleRun {
    std::uint64_t hash = 0;
    int extended_ticks = 0;
};

VehicleRun run_vehicle(const TurboPairDesc& pair, int workers) {
    WorldConfig cfg;
    cfg.job_workers = workers;
    cfg.tick_rate_hz = 240.0;
    cfg.substep_rate_hz = 960.0;
    World w(cfg);
    BodyDesc ground;
    ground.shape = BoxShape{Vec3{1000.0, 1000.0, 0.5}};
    ground.motion = BodyMotionType::Static;
    ground.friction = 1.0;
    ground.pose.position = Vec3{0.0, 0.0, -0.5};
    w.create_body(ground);
    BodyDesc chassis;
    chassis.shape = BoxShape{Vec3{2.0, 0.4, 0.15}};
    chassis.motion = BodyMotionType::Dynamic;
    chassis.mass = 1500.0;
    chassis.friction = 0.5;
    chassis.allow_sleep = false;
    chassis.pose.position = Vec3{0.0, 0.0, 0.6};
    const BodyId body = w.create_body(chassis);
    w.set_surface_table(std::make_shared<io::SurfaceTable>(std::string(PS_SOURCE_DIR) + "/data/surfaces/surfaces.json"));
    const VehicleId v = w.create_vehicle(turbo_sedan(pair), body);
    w.set_control("assist.auto_clutch", 1.0);
    w.set_control("assist.auto_blip", 0.0);
    w.set_control("assist.auto_shift", 0.0);
    w.set_control("clutch", 0.0);
    w.set_control("throttle", 0.0);
    vehicle::VehicleResetOptions o;
    o.gear = 1;
    o.engine_state = drivetrain::EngineState::Running;
    o.clutch_locked = false;
    w.reset_vehicle(v, o);
    for (int t = 0; t < 120; ++t) w.step();
    namespace da = ps::jobs::debug_alloc;
    const bool arm = da::hook_available();
    if (arm) {
        da::reset();
        da::set_armed(true);
    }
    VehicleRun out;
    w.set_control("throttle", 1.0);
    for (int t = 0; t < 720; ++t) {
        w.step();
        out.extended_ticks += int(w.powertrain_state(v).engines.at(0).turbo_compressor_stage_extended);
    }
    if (arm) {
        da::set_armed(false);
        WARN("S0 vehicle debug_alloc (" << workers << " workers): own " << da::own_violation_count() << ", external (Jolt) "
                                        << da::external_violation_count());
        CHECK(da::own_violation_count() == 0);
    }
    out.hash = w.state_hash();
    return out;
}

} // namespace

TEST_CASE("S0 extension follows a speed line past its choke edge", "[turbo_stage][drivetrain]") {
    const TurboStageMaps m = hand_map();
    const auto pr = [&](std::size_t i, std::size_t j) { return m.compressor_pr.at(i, j); };
    const auto wk = [&](std::size_t i, std::size_t j) { return m.compressor_work_j_kg.at(i, j); };
    // On the 50k line: the edge value at its last valid knot, linear to PR 1 at the first invalid knot, 1 beyond;
    // work held at the edge.
    const auto e = m.compressor_extended(50000.0, 0.2);
    CHECK(std::abs(e.pressure_ratio - pr(1, 2)) <= 1e-12);
    const auto half = m.compressor_extended(50000.0, 0.25);
    CHECK(std::abs(half.pressure_ratio - 0.5 * (pr(1, 2) + 1.0)) <= 1e-12);
    CHECK(std::abs(half.pressure_slope - (1.0 - pr(1, 2)) / 0.1) <= 1e-9);
    CHECK(half.beyond_choke);
    CHECK(std::abs(m.compressor_extended(50000.0, 0.3).pressure_ratio - 1.0) <= 1e-12);
    CHECK(std::abs(m.compressor_extended(50000.0, 0.45).pressure_ratio - 1.0) <= 1e-12);
    for (real f : {0.2, 0.25, 0.3, 0.45}) CHECK(std::abs(m.compressor_extended(50000.0, f).work - wk(1, 2)) <= 1e-9);
    // A line valid to the axis end gets one more knot at the last spacing.
    CHECK(std::abs(m.compressor_extended(150000.0, 0.55).pressure_ratio - 0.5 * (pr(3, 5) + 1.0)) <= 1e-12);
    CHECK(std::abs(m.compressor_extended(150000.0, 0.7).pressure_ratio - 1.0) <= 1e-12);
    // Above the highest line: affinity (flow f/r, PR^kappa - 1 and work scale r^2).
    {
        const real r = 180000.0 / 150000.0, k = m.air_kappa;
        const auto a = m.compressor_extended(180000.0, 0.24);
        const real pr_line = 1.0 + 2.0 * (1.0 - 0.6 * (0.2 / 0.5) * (0.2 / 0.5)); // the 150k line at 0.2
        CHECK(std::abs(a.pressure_ratio - std::pow(1.0 + (std::pow(pr_line, k) - 1.0) * r * r, 1.0 / k)) <= 1e-9);
        CHECK(std::abs(a.work - wk(3, 2) * r * r) <= 1e-6);
        CHECK(a.speed_outside);
    }
    // Below the lowest line with data (the 0 rpm line has none): affinity from the 50k line; a resting wheel makes nothing.
    {
        const auto a = m.compressor_extended(25000.0, 0.05);
        CHECK(std::abs(a.work - wk(1, 1) * 0.25) <= 1e-6);
        const auto z = m.compressor_extended(0.0, 0.05);
        CHECK(z.pressure_ratio == 1.0);
        CHECK(z.work == 0.0);
    }
}

TEST_CASE("S0 extension equals the covered map and is continuous and monotone in speed", "[turbo_stage][drivetrain]") {
    for (const char* cfg : {"hand", kOwnerCfg, kStandardCfg}) {
        INFO(cfg);
        TurboPairDesc td;
        const TurboStageMaps* mp = nullptr;
        const TurboStageMaps hand = hand_map();
        if (std::string(cfg) == "hand") mp = &hand;
        else {
            td = load_pair(cfg);
            REQUIRE(td.maps);
            REQUIRE(td.maps->stages);
            mp = td.maps->stages.get();
        }
        const TurboStageMaps& m = *mp;
        const real n_top = m.compressor_pr.x.back() * 1.2, f_top = m.compressor_pr.y.back() * 1.3;
        // (1) Inside coverage the extension is the covered bilinear value: one probe per cell centre.
        int covered = 0, mismatched = 0;
        for (std::size_t i = 0; i + 1 < m.compressor_pr.x.size(); ++i)
            for (std::size_t j = 0; j + 1 < m.compressor_pr.y.size(); ++j) {
                const real n = 0.5 * (m.compressor_pr.x[i] + m.compressor_pr.x[i + 1]);
                const real f = 0.5 * (m.compressor_pr.y[j] + m.compressor_pr.y[j + 1]);
                const auto s = m.compressor_sample(n, f);
                if (s.weight < 1.0) continue;
                ++covered;
                const auto x = m.compressor_extended(n, f);
                if (std::abs(x.pressure_ratio - s.pressure_ratio) > 1e-12 * s.pressure_ratio ||
                    std::abs(x.work - s.work) > 1e-12 * std::max(real(1), s.work) ||
                    std::abs(x.pressure_slope - s.pressure_slope) > 1e-9 * std::max(real(1), std::abs(s.pressure_slope)))
                    ++mismatched;
            }
        CHECK(covered > 0);
        CHECK(mismatched == 0);
        // (2) Continuity: fine sweeps along flow and speed; every step whose change exceeds 1e-3 (PR, or work relative to
        // the map's largest work) is bisected 30 times toward its larger half - a continuous function shrinks to ~0
        // there, a jump (the pre-S0 blend at the mask boundary, O(0.1) in PR) keeps its size.
        real max_w = 1.0;
        for (real v : m.compressor_work_j_kg.values) max_w = std::max(max_w, v);
        const auto gap = [&](real n0, real f0, real n1, real f1) {
            const auto a = m.compressor_extended(n0, f0), b = m.compressor_extended(n1, f1);
            return std::max(std::abs(b.pressure_ratio - a.pressure_ratio), std::abs(b.work - a.work) / max_w);
        };
        const auto jump = [&](real n0, real f0, real n1, real f1) {
            if (gap(n0, f0, n1, f1) <= 1e-3) return false;
            for (int d = 0; d < 30; ++d) {
                const real nm = 0.5 * (n0 + n1), fm = 0.5 * (f0 + f1);
                if (gap(n0, f0, nm, fm) >= gap(nm, fm, n1, f1)) { n1 = nm; f1 = fm; } else { n0 = nm; f0 = fm; }
            }
            return gap(n0, f0, n1, f1) > 1e-6;
        };
        const real df = f_top / 1000.0, dn = n_top / 1000.0;
        int jumps = 0, falls = 0;
        for (int a = 1; a <= 20; ++a) {
            const real n = n_top * a / 20.0 + 7.0; // off the speed knots
            for (int b = 1; b <= 1000; ++b) jumps += int(jump(n, df * (b - 1), n, df * b));
        }
        for (int b = 1; b <= 20; ++b) {
            const real f = f_top * b / 20.0 + 1e-4; // off the flow knots
            auto prev = m.compressor_extended(0.0, f);
            for (int a = 1; a <= 1000; ++a) {
                const auto cur = m.compressor_extended(dn * a, f);
                jumps += int(jump(dn * (a - 1), f, dn * a, f));
                // (3) More blade speed at a fixed flow never lowers head or work (the maps' lines are ordered).
                if (!non_decreasing(prev.pressure_ratio, cur.pressure_ratio) || !non_decreasing(prev.work, cur.work)) ++falls;
                prev = cur;
            }
        }
        CHECK(jumps == 0);
        CHECK(falls == 0);
    }
}

TEST_CASE("S0 frozen-rotor sweeps are monotone and inside the step bands", "[turbo_stage][drivetrain]") {
    const TurboPairDesc owner = load_pair(kOwnerCfg), standard = load_pair(kStandardCfg);
    const auto owner_fixed = frozen_sweep(owner, 1.6, kExhOverAir, kPortK, 0.0, 60000.0, 160000.0, 2500.0, 0.0);
    const auto owner_pump = frozen_sweep(owner, 0.0, kExhOverAir, kPortK, kOwnerPump, 60000.0, 160000.0, 2500.0, 0.0);
    const auto std_fixed = frozen_sweep(standard, 0.77, kExhOverAir, kPortK, 0.0, 60000.0, 160000.0, 2500.0, 0.0);
    const auto std_pump = frozen_sweep(standard, 0.0, kExhOverAir, kPortK, kStandardPump, 60000.0, 160000.0, 2500.0, 0.0);
    // The sweeps must cross the coverage boundary, or they test nothing.
    for (const auto* s : {&owner_fixed, &owner_pump, &std_fixed, &std_pump}) {
        int ext = 0, cov = 0;
        for (const auto& p : *s) (p.extended ? ext : cov) += 1;
        CHECK(ext > 0);
        CHECK(cov > 0);
    }
    CHECK(check_sweep("owner fixed 1.6 kg/s", owner_fixed, false) == 0);
    CHECK(check_sweep("owner pump", owner_pump, true) == 0);
    CHECK(check_sweep("standard fixed 0.77 kg/s", std_fixed, false) == 0);
    CHECK(check_sweep("standard pump", std_pump, true) == 0);
}

TEST_CASE("S0 live boost control: turbine power reverses at most once over a frozen-rotor sweep", "[turbo_stage][drivetrain]") {
    const TurboPairDesc owner = load_pair(kOwnerCfg), standard = load_pair(kStandardCfg);
    const auto o = frozen_sweep(owner, 1.6, kExhOverAir, kPortK, 0.0, 60000.0, 160000.0, 2500.0, -1.0);
    const auto s = frozen_sweep(standard, 0.77, kExhOverAir, kPortK, 0.0, 60000.0, 160000.0, 2500.0, -1.0);
    INFO("owner reversals " << turbine_reversals(o) << ", standard reversals " << turbine_reversals(s));
    CHECK(turbine_reversals(o) <= 1);
    CHECK(turbine_reversals(s) <= 1);
}

TEST_CASE("S0 extended compressor allocates nothing", "[turbo_stage][drivetrain]") {
    namespace da = ps::jobs::debug_alloc;
    if (!da::hook_available()) {
        WARN("debug_alloc hook unavailable in this build - allocation check skipped");
        return;
    }
    const TurboPairDesc owner = load_pair(kOwnerCfg);
    TurboPair tp(owner);
    tp.reset(101325.0, 288.15);
    da::reset();
    da::set_armed(true);
    da::enter_job();
    int extended = 0;
    for (int i = 0; i < 960; ++i) {
        tp.step(1.0 / 960.0, 101325.0, 288.15, 1.6, 1.7, kPortK, 1.0);
        extended += int(tp.compressor_stage_extended);
    }
    da::exit_job();
    da::set_armed(false);
    CHECK(extended > 0);
    CHECK(da::own_violation_count() == 0);
}

TEST_CASE("S0 a vehicle on a stage-map turbo is worker-count invariant and allocation-free", "[turbo_stage][drivetrain][vehicle][determinism]") {
    const TurboPairDesc owner = load_pair(kOwnerCfg);
    const VehicleRun a = run_vehicle(owner, 1);
    const VehicleRun b = run_vehicle(owner, 10);
    CHECK(a.extended_ticks > 0);
    CHECK(a.extended_ticks == b.extended_ticks);
    CHECK(a.hash == b.hash);
}

// Hidden diagnostic. Env: PS_S0_CFG owner|standard (default owner), PS_S0_AIR (pair air kg/s, fixed mode), PS_S0_PUMP
// (speed-density constant kg/s per Pa/K; > 0 selects the pump mode), PS_S0_EXH_OVER_AIR (1.0625), PS_S0_PORT_K (1150),
// PS_S0_GATE (0 = closed, < 0 = live PID), PS_S0_RPM_LO/HI/STEP (60000/160000/2500).
TEST_CASE("S0 frozen sweep diag", "[.][diag][stage_seam]") {
    const std::string cfg = env_or("PS_S0_CFG", "owner");
    const TurboPairDesc td = load_pair(cfg == "standard" ? kStandardCfg : kOwnerCfg);
    const auto s = frozen_sweep(td, env_num("PS_S0_AIR", 1.6), env_num("PS_S0_EXH_OVER_AIR", kExhOverAir),
                                env_num("PS_S0_PORT_K", kPortK), env_num("PS_S0_PUMP", 0.0), env_num("PS_S0_RPM_LO", 60000.0),
                                env_num("PS_S0_RPM_HI", 160000.0), env_num("PS_S0_RPM_STEP", 2500.0), env_num("PS_S0_GATE", 0.0));
    print_sweep("S0", s);
    std::printf("S0 turbine reversals %d\n", turbine_reversals(s));
    SUCCEED();
}

// --- Part 2: the coverage check (BuildRule::TurboStageMapCoverage, EngineMapOptions::warnings_out) ---------------------
namespace {

const char* kOwnerVehicle = "tests/support/fixtures/assists/v12/vehicle_v12.json";

// A validate-only load (no map generation, milliseconds) of the owner's vehicle; its engine and the load warnings.
drivetrain::SimulatedEngineDesc owner_engine(std::vector<std::string>* warnings) {
    io::VehicleLoadOptions o;
    o.maps.generate = false;
    o.maps.warnings_out = warnings;
    const vehicle::VehicleDesc v = io::load_vehicle_json(std::string(PS_SOURCE_DIR) + "/" + kOwnerVehicle, o);
    for (const auto& c : v.powertrain.components)
        if (const auto* e = std::get_if<drivetrain::SimulatedEngineDesc>(&c.params)) return *e;
    FAIL("no simulated engine in " << kOwnerVehicle);
    return {};
}

// The engine with another turbo configuration fitted (target in bar gauge).
drivetrain::SimulatedEngineDesc with_turbo(drivetrain::SimulatedEngineDesc e, const char* cfg, real target_bar) {
    e.turbo_pair = io::load_turbo_configuration_json(std::string(PS_SOURCE_DIR) + "/" + cfg);
    e.turbo_pair->target_boost_pa = target_bar * 1e5;
    return e;
}

// The same map with every speed line above `keep_rpm` emptied (no valid knot left on it).
drivetrain::SimulatedEngineDesc truncated(drivetrain::SimulatedEngineDesc e, real keep_rpm) {
    auto pm = std::make_shared<drivetrain::TurboPerformanceMaps>(*e.turbo_pair->maps);
    auto st = std::make_shared<TurboStageMaps>(*pm->stages);
    const std::size_t nf = st->compressor_pr.y.size();
    for (std::size_t i = 0; i < st->compressor_pr.x.size(); ++i)
        if (st->compressor_pr.x[i] > keep_rpm)
            for (std::size_t j = 0; j < nf; ++j) st->compressor_valid[i * nf + j] = false;
    pm->stages = st;
    e.turbo_pair->maps = pm;
    return e;
}

// The TurboStageMapCoverage issues of check_buildable (the same as check_turbo_stage_coverage's).
std::vector<drivetrain::BuildIssue> coverage(const drivetrain::SimulatedEngineDesc& e) {
    std::vector<drivetrain::BuildIssue> out;
    for (const auto& b : drivetrain::check_buildable(e))
        if (b.rule == drivetrain::BuildRule::TurboStageMapCoverage) out.push_back(b);
    CHECK(drivetrain::check_turbo_stage_coverage(e).size() == out.size());
    return out;
}

} // namespace

TEST_CASE("S0 coverage check: owner setup and truncated map fire; covering map does not","[turbo_stage][drivetrain][io]") {
    std::vector<std::string> warnings;
    const auto owner = owner_engine(&warnings);
    // The owner's 15.48 bar on the large map: far above its highest speed line.
    const auto o = coverage(owner);
    REQUIRE(o.size() == 1);
    CHECK(o[0].severity == drivetrain::LimitSeverity::Limiting);
    CHECK(o[0].slot == "forced_induction");
    CHECK(o[0].unit == drivetrain::LimitUnit::Fraction);
    CHECK(o[0].value < 1.0);
    REQUIRE(warnings.size() == 1);
    CHECK(warnings[0].find("Limiting") != std::string::npos);
    CHECK(warnings[0].find("turbo stage map") != std::string::npos);
    // The same engine on the large map at its racing_game baseline 1.8 bar is covered ...
    const auto covering = with_turbo(owner, kOwnerCfg, 1.8);
    CHECK(coverage(covering).empty());
    // ... and is not once the map's speed lines above 90k are gone.
    const auto cut = coverage(truncated(covering, 90000.0));
    REQUIRE(cut.size() == 1);
    CHECK(cut[0].severity == drivetrain::LimitSeverity::Limiting);
    // A turbo without a stage map is never checked.
    CHECK(coverage(with_turbo(owner, "data/turbo_configurations/hyper_twin_default.json", 1.8)).empty());
}

TEST_CASE("S0 coverage check diag", "[.][diag][stage_seam]") {
    const auto e = owner_engine(nullptr);
    for (const char* cfg : {kOwnerCfg, kStandardCfg})
        for (real bar : {0.6, 1.0, 1.4, 1.8, 2.5}) {
            const auto issues = drivetrain::check_turbo_stage_coverage(with_turbo(e, cfg, bar));
            const std::string text =
                issues.empty() ? std::string("covered") : std::string(drivetrain::to_string(issues[0].severity)) + " " + issues[0].message;
            std::printf("S0 coverage %s %.1f bar: %s\n", cfg, bar, text.c_str());
        }
    SUCCEED();
}
