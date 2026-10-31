// tests/unit/test_machine_tools.cpp — ps::machine work tools: the draft law and parts
// on their own, then a tractor working soil with a mounted cultivator, plough and rotary
// harrow, and the swept work a farm reads.
//
// Sabotages, run one at a time against "[machine]~[diag]~Coupling*" on 2026-10-31, each with
// the case that caught it:
//   no speed-squared term; moisture dropped; tilled ignored on A    -> draft: the D497 form
//   draft growing past the tines; the part's A 46 -> 56             -> draft: the D497 form
//   draft not applied as a force; half of it as force; no suction   -> a cultivator in soil; an undersized tractor
//   quality falling above the minimum                               -> quality; a cultivator in soil
//   no soil load on the rotor; the engine not feeling it            -> the rotary harrow
//   draft control reversed                                          -> draft control
//   a road counts as worked                                         -> a tool in ground that cannot be worked
//   the first tick writing a quad                                   -> the first tick in the ground
//   standing still writes quads; dropped quads uncounted;
//   worked area not added up                                        -> work: a tool sweeps its width
//   drain unsorted by tool                                          -> work from two tools
//   a bow tie as a plain quad; an unweighted ledger depth           -> work_coverage
//   the sweep forgetting the rotor; tool power never graded         -> the shop's figure
//   sources needing no tag                                          -> machine parts
//   combine_header.json's kind soil (G3 harvest chain, 2026-10-31)  -> applicator parts: the loader's rules
//   an allocation in the substep; a result that depends on the
//   worker                                                          -> work is the same on 1 and 10 workers
//   an unconditional fold in vehicle::System's state hash           -> ps_determinism_tests' pinned scenario hashes

#include "ps/drivetrain/energy_ledger.h"
#include "ps/drivetrain/limit_findings.h"
#include "ps/io/machine_io.h"
#include "ps/jobs/debug_alloc.h"
#include "ps/machine/coupling.h"
#include "ps/machine/tool.h"
#include "ps/machine/work.h"
#include "machine_test_rig.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <map>
#include <random>
#include <string>
#include <vector>

using namespace ps;
using Catch::Approx;
using ps_test::build_mount;
using ps_test::field_soil;
using ps_test::MountCfg;
using ps_test::MountRig;
using ps_test::UniformSoil;

namespace {

constexpr real kTwoPi = 6.283185307179586;

const io::MachineSockets kSockets = {{"socket_work_L", Vec3{-1.55, 1.5, -0.6}}, {"socket_work_R", Vec3{-1.55, -1.5, -0.6}}};

machine::ToolDesc part_tool(const std::string& name) {
    return io::load_machine_json(std::string(PS_SOURCE_DIR) + "/data/machines/" + name + ".json", kSockets).tools[0];
}

std::string scratch_file(const std::string& name, const std::string& text) {
    const std::filesystem::path dir = std::filesystem::temp_directory_path() / "ps_machine_tools_test";
    std::filesystem::create_directories(dir);
    const std::filesystem::path path = dir / name;
    std::ofstream(path) << text;
    return path.string();
}

struct Steady {
    real speed = 0.0, draft = 0.0, depth = 0.0, hitch_x = 0.0, hitch_z = 0.0, engine_kw = 0.0, rpm = 0.0, thrust = 0.0, kappa = 0.0, quality = 0.0;
};

// Means over the last `window` seconds of a run.
Steady working_run(MountRig& r, real seconds, real window) {
    Steady s;
    const int ticks = static_cast<int>(seconds * 240.0);
    const int from = ticks - static_cast<int>(window * 240.0);
    real energy0 = 0.0;
    for (int i = 0; i < ticks; ++i) {
        if (i == from) energy0 = r.world->powertrain_energy(r.tractor).engine_out;
        r.world->step();
        if (i < from) continue;
        const machine::ToolState& t = r.world->tool_state(r.implement, 0);
        const machine::CouplingState& c = r.world->coupling_state(r.hitch);
        s.speed += r.world->get_motion(r.tractor_body).linear.x;
        s.draft += t.draft_n;
        s.depth += t.depth_m;
        s.quality += t.quality;
        s.hitch_x += c.mean_force_on_towing_world.x;
        s.hitch_z += c.mean_force_on_towing_world.z;
        s.rpm += r.world->powertrain_state(r.tractor).engine_rpm;
        for (std::size_t w = 0; w < 4; ++w) s.thrust += r.world->wheel_telemetry(r.tractor, w).fx;
        s.kappa += 0.5 * (std::abs(r.world->wheel_telemetry(r.tractor, 2).slip_ratio) + std::abs(r.world->wheel_telemetry(r.tractor, 3).slip_ratio));
    }
    const real n = ticks - from;
    s.speed /= n;
    s.draft /= n;
    s.depth /= n;
    s.quality /= n;
    s.hitch_x /= n;
    s.hitch_z /= n;
    s.rpm /= n;
    s.thrust /= n;
    s.kappa /= n;
    s.engine_kw = (r.world->powertrain_energy(r.tractor).engine_out - energy0) / (n / 240.0) * 1e-3;
    return s;
}

// Lever for a lower link angle, on tractor.json's stops.
real lever_for(real angle_rad) { return (angle_rad + 20.0 * kTwoPi / 360.0) / (45.0 * kTwoPi / 360.0); }

constexpr real kWorkingAngle = -0.15;

MountCfg on_dirt() {
    MountCfg cfg;
    cfg.ground = "dirt";
    return cfg;
}

// The lift holding the tool at a depth. The lever must hold it: let right down the edge goes under the links'
// instant centre and the soil's push jacks up the tractor's rear, as in float.
MountRig worked_rig(const MountCfg& cfg, int gear, real awd = 0.0, real link_angle = kWorkingAngle, const machine::SoilSample& soil = field_soil()) {
    MountRig r = build_mount(cfg);
    r.world->set_soil_source(std::make_shared<UniformSoil>(soil));
    r.run(0.6);
    r.set("lift", lever_for(link_angle));
    r.set("gear_request", gear);
    r.set("awd", awd);
    r.set("throttle", 1.0);
    return r;
}

} // namespace

TEST_CASE("draft: the D497 form against the hand calculation", "[machine]") {
    const machine::ToolDesc cultivator = part_tool("cultivator_3m");
    const machine::ToolDesc disc = part_tool("disc_harrow_3m");
    const machine::ToolDesc plough = part_tool("plough_2m");
    const machine::SoilSample medium = field_soil(machine::SoilTexture::Medium);

    SECTION("per tool per cm of depth: F (A + B S) tools depth") {
        CHECK(machine::tool_draft_n(cultivator, medium, 8.0, 0.10) == Approx(0.85 * (46.0 + 2.8 * 8.0) * 15.0 * 10.0).epsilon(1e-12));
    }
    SECTION("per metre per cm") {
        CHECK(machine::tool_draft_n(disc, medium, 8.0, 0.10) == Approx(0.88 * (309.0 + 16.0 * 8.0) * 3.0 * 10.0).epsilon(1e-12));
    }
    SECTION("the plough's speed-squared term") {
        CHECK(machine::tool_draft_n(plough, medium, 7.0, 0.20) == Approx(0.70 * (652.0 + 5.1 * 49.0) * 3.0 * 20.0).epsilon(1e-12));
    }
    SECTION("texture: fine soil pulls more than coarse") {
        const real fine = machine::tool_draft_n(cultivator, field_soil(machine::SoilTexture::Fine), 8.0, 0.10);
        const real coarse = machine::tool_draft_n(cultivator, field_soil(machine::SoilTexture::Coarse), 8.0, 0.10);
        CHECK(fine / coarse == Approx(1.0 / 0.65).epsilon(1e-12));
    }
    SECTION("tilled soil falls to the secondary line, half-way half-way") {
        machine::SoilSample soil = medium;
        const real primary = machine::tool_draft_n(cultivator, soil, 8.0, 0.10);
        soil.tilled = 1.0;
        const real secondary = machine::tool_draft_n(cultivator, soil, 8.0, 0.10);
        soil.tilled = 0.5;
        CHECK(machine::tool_draft_n(cultivator, soil, 8.0, 0.10) == Approx(0.5 * (primary + secondary)).epsilon(1e-12));
        CHECK(secondary < 0.75 * primary);
    }
    SECTION("wet soil and compaction raise it") {
        machine::SoilSample soil = medium;
        const real base = machine::tool_draft_n(cultivator, soil, 8.0, 0.10);
        soil.wetness = 1.3;
        CHECK(machine::tool_draft_n(cultivator, soil, 8.0, 0.10) == Approx(1.4 * base).epsilon(1e-12));
        soil.wetness = 0.5;
        soil.compaction = 1.0;
        CHECK(machine::tool_draft_n(cultivator, soil, 8.0, 0.10) == Approx(1.3 * base).epsilon(1e-12));
    }
    SECTION("past the tines' length draft stops growing") {
        CHECK(machine::tool_draft_n(cultivator, medium, 8.0, 0.5) == machine::tool_draft_n(cultivator, medium, 8.0, cultivator.depth_max_m));
    }
}

TEST_CASE("quality: slow costs nothing, fast costs quality down to the floor", "[machine]") {
    const machine::ToolDesc t = part_tool("cultivator_3m");
    CHECK(machine::tool_quality(t, 1.0) == 1.0);
    CHECK(machine::tool_quality(t, t.speed_max_km_h) == 1.0);
    CHECK(machine::tool_quality(t, t.speed_max_km_h + 2.0) == Approx(1.0 - 2.0 * t.overspeed_quality_per_km_h));
    CHECK(machine::tool_quality(t, t.speed_max_km_h + 100.0) == t.quality_floor);
}

TEST_CASE("machine parts: extends, sockets, and what the loader refuses", "[machine][io]") {
    const std::string base = std::string(PS_SOURCE_DIR) + "/data/machines/cultivator_3m.json";

    SECTION("the width is the sockets' distance") {
        const machine::MachineDesc m = io::load_machine_json(base, kSockets);
        CHECK(m.tools[0].width_m == Approx(3.0));
        CHECK(m.tools[0].operation == "cultivator");
        CHECK(machine::tool_draft_width(m.tools[0]) == Approx(15.0));
    }
    SECTION("no such socket") {
        CHECK_THROWS_WITH(io::load_machine_json(base, {}), Catch::Matchers::ContainsSubstring("no socket 'socket_work_L'"));
    }
    SECTION("extends overrides a field and keeps the rest") {
        const std::string path = scratch_file("light.json", "{ \"extends\": \"" + std::filesystem::path(base).generic_string() + "\", \"name\": \"light_cultivator\" }");
        CHECK(io::load_machine_json(path, kSockets).name == "light_cultivator");
    }
    SECTION("an unknown key") {
        const std::string path = scratch_file("typo.json", "{ \"extends\": \"" + std::filesystem::path(base).generic_string() + "\", \"tooles\": [] }");
        CHECK_THROWS_WITH(io::load_machine_json(path, kSockets), Catch::Matchers::ContainsSubstring("unknown field 'tooles'"));
    }
    SECTION("the wrong format") {
        const std::string path = scratch_file("format.json", "{ \"format\": \"physics_sim.machine/2\", \"name\": \"x\", \"source\": \"x\", \"tools\": [] }");
        CHECK_THROWS_WITH(io::load_machine_json(path, kSockets), Catch::Matchers::ContainsSubstring("physics_sim.machine/1"));
    }
    SECTION("a block without a source tag") {
        std::ifstream in(base);
        std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
        const std::size_t at = text.find("\"source\": \"[AS] D497");
        REQUIRE(at != std::string::npos);
        text.replace(at, 15, "\"source\": \"D497");
        CHECK_THROWS_WITH(io::load_machine_json(scratch_file("tag.json", text), kSockets), Catch::Matchers::ContainsSubstring("must start with [C], [D], [M] or [AS]"));
    }
    SECTION("a vehicle file's machine block needs its sockets") {
        const std::string path = scratch_file("v.json", std::string(R"({ "format": "physics_sim.vehicle/2", "name": "x", "source": "x", "wheels": [], "machine": { "ref": ")") +
                                                            std::filesystem::path(base).generic_string() + R"(" } })");
        CHECK_THROWS_WITH(io::load_vehicle_json(path), Catch::Matchers::ContainsSubstring("'sockets' must give the model's sockets"));
    }
}

TEST_CASE("the shop's figure: a sweep over the tool's speeds, soils and efficiencies", "[machine]") {
    const machine::ToolPowerSpec spec = io::load_tool_power_json(std::string(PS_SOURCE_DIR) + "/data/machines/power_estimate.json");
    const machine::ToolDesc cultivator = part_tool("cultivator_3m");
    const machine::ToolPowerSweep sweep = machine::tool_power_sweep(cultivator, spec);
    REQUIRE(sweep.points.size() == 5 * spec.soils.size());

    // The worst corner: fine soil at the top speed and the nominal depth, wetness 0.8 keeps the factor at 1.
    const real draft = 1.0 * (46.0 + 2.8 * 12.0) * 15.0 * 10.0;
    const real drawbar_kw = draft * (12.0 / 3.6) * 1e-3;
    const real needed = drawbar_kw / 0.78 / 0.90 * 1.15;
    CHECK(sweep.needed_kw == Approx(needed).epsilon(1e-9));
    printf("cultivator 3 m: needs about %.1f kW (worst draft %.0f N, %.1f kW at the drawbar)\n", sweep.needed_kw, draft, drawbar_kw);

    const machine::ToolDesc rotary = part_tool("rotary_harrow_3m");
    const machine::ToolPowerSweep rotary_sweep = machine::tool_power_sweep(rotary, spec);
    // Its rotor is in the figure: the top speed in the worst soil, drawbar and shaft on one engine.
    const real rotary_speed = rotary.speed_max_km_h, rotary_depth = rotary.depth_nominal_m;
    const real rotary_draft = 1.0 * (108.0 + 5.6 * rotary_speed) * 3.0 * (rotary_depth * 100.0);
    const real rotor_kw = 3.0 + 1.0 * 3.0 + 0.15e3 * 3.0 * rotary_depth * (rotary_speed / 3.6);
    CHECK(rotary_sweep.needed_kw == Approx((rotary_draft * (rotary_speed / 3.6) * 1e-3 / 0.78 + rotor_kw) / 0.90 * 1.15).epsilon(1e-9));

    SECTION("against a tractor, as findings") {
        using drivetrain::LimitKind;
        using drivetrain::LimitSeverity;
        const auto kind_of = [](const std::vector<drivetrain::LimitFinding>& v, LimitKind k) { return std::find_if(v.begin(), v.end(), [k](const auto& f) { return f.kind == k; }); };
        const auto fits = machine::check_tool_limits(cultivator, sweep, {.engine_kw = 1000.0, .pto_torque_nm = 0.0});
        CHECK(kind_of(fits, LimitKind::ToolPower) == fits.end());
        const auto small = machine::check_tool_limits(cultivator, sweep, {.engine_kw = sweep.needed_kw / 1.5, .pto_torque_nm = 0.0});
        REQUIRE(kind_of(small, LimitKind::ToolPower) != small.end());
        CHECK(kind_of(small, LimitKind::ToolPower)->severity == LimitSeverity::Exceeded);
        CHECK(kind_of(small, LimitKind::ToolPower)->unit == drivetrain::LimitUnit::Kilowatt);
        const auto weak_pto = machine::check_tool_limits(rotary, rotary_sweep, {.engine_kw = 1000.0, .pto_torque_nm = 50.0});
        REQUIRE(kind_of(weak_pto, LimitKind::PtoShaftTorque) != weak_pto.end());
        CHECK(std::string(to_string(LimitKind::ToolPower)) == "ToolPower");
        CHECK(std::string(to_string(LimitKind::PtoShaftTorque)) == "PtoShaftTorque");
        CHECK(std::string(to_string(LimitKind::LiftCapacity)) == "LiftCapacity");
    }
}

TEST_CASE("a cultivator in soil: its draft is a real force at the hitch and the tractor pays for it", "[machine][vehicle]") {
    MountRig r = worked_rig(on_dirt(), 6);
    const Steady s = working_run(r, 9.0, 3.0);
    const machine::ToolDesc& tool = r.world->tool_desc(r.implement, 0);
    const real km_h = s.speed * 3.6;
    const real law = machine::tool_draft_n(tool, field_soil(), km_h, s.depth);
    const real drawbar_kw = s.draft * s.speed * 1e-3;
    printf("3 m cultivator, medium soil, untilled: %.2f km/h at %.1f cm: draft %.0f N (law %.0f N), hitch %.0f N, thrust %.0f N, drawbar %.1f kW, engine %.1f kW at %.0f rpm, slip %.3f\n", km_h, s.depth * 100.0, s.draft, law,
           -s.hitch_x, s.thrust, drawbar_kw, s.engine_kw, s.rpm, s.kappa);
    CHECK(km_h > 6.0);
    CHECK(s.depth > 0.05);
    CHECK(s.draft == Approx(law).epsilon(0.02));
    // Newton through the hitch rows: the tool's draft is what the tractor feels.
    CHECK(-s.hitch_x == Approx(s.draft).epsilon(0.03));
    // And the wheels pay it, on top of their own rolling resistance.
    CHECK(s.thrust > s.draft);
    CHECK(s.engine_kw > drawbar_kw);
    // The soil's push less the tool's suction, shallower than the roller.
    MountRig shallow = worked_rig(on_dirt(), 6, 0.0, -0.12);
    const Steady d = working_run(shallow, 6.0, 2.0);
    REQUIRE(d.depth < 0.95 * tool.depth_nominal_m);
    CHECK(shallow.world->tool_state(shallow.implement, 0).vertical_n == Approx(tool.soil.bearing_n_m2 * tool.width_m * d.depth - tool.soil.suction_ratio * d.draft).epsilon(0.05));
    CHECK(r.world->tool_state(r.implement, 0).working);
    CHECK(r.world->tool_state(r.implement, 0).quality == 1.0);

    // Without the soil the same tractor at the same governor setting takes no such power.
    MountRig idle = worked_rig(on_dirt(), 6, 0.0, 0.3);
    const Steady free = working_run(idle, 9.0, 3.0);
    printf("lifted out: %.2f km/h, engine %.1f kW\n", free.speed * 3.6, free.engine_kw);
    CHECK(free.engine_kw < s.engine_kw - 0.5 * drawbar_kw);
    CHECK(idle.world->tool_state(idle.implement, 0).depth_m == 0.0);
}

constexpr real kPloughAngle = -0.13;

TEST_CASE("an undersized tractor lugs, slips or stalls by itself", "[machine][vehicle]") {
    const auto plough = [](real torque_scale, real mass, const char* ground) {
        MountCfg cfg;
        cfg.fixture = "mounted_plough";
        cfg.tractor_mass = mass;
        cfg.ground = ground;
        cfg.edit_tractor = [torque_scale](vehicle::VehicleDesc& d) {
            for (drivetrain::ComponentDesc& c : d.powertrain.components) {
                if (auto* e = std::get_if<drivetrain::TorqueMapEngineDesc>(&c.params)) {
                    for (real& y : e->wot_torque_nm_vs_rpm.y) y *= torque_scale;
                }
            }
        };
        return cfg;
    };
    const auto report = [](const char* name, const Steady& s, const MountRig& r) {
        printf("%-30s %.2f km/h, %.1f cm, draft %.0f N, engine %.1f kW at %.0f rpm, slip %.3f, state %d\n", name, s.speed * 3.6, s.depth * 100.0, s.draft, s.engine_kw, s.rpm, s.kappa,
               static_cast<int>(r.world->powertrain_state(r.tractor).engine_state));
    };

    // Good grip, so the engine is the only limit.
    MountRig strong = worked_rig(plough(1.0, 6000.0, "asphalt"), 5, 0.0, kPloughAngle);
    const Steady s = working_run(strong, 10.0, 3.0);
    report("100 kW, 6 t, good grip", s, strong);
    CHECK(s.depth > 0.2);
    CHECK(s.rpm > 2000.0);
    CHECK(s.speed * 3.6 > 5.0);
    CHECK(s.kappa < 0.1);
    CHECK(s.draft == Approx(machine::tool_draft_n(strong.world->tool_desc(strong.implement, 0), field_soil(), s.speed * 3.6, s.depth)).epsilon(0.02));

    MountRig weak = worked_rig(plough(0.25, 6000.0, "asphalt"), 5, 0.0, kPloughAngle);
    const Steady w = working_run(weak, 10.0, 3.0);
    report("25 kW, 6 t, good grip", w, weak);
    CHECK(w.rpm < 1600.0);
    CHECK(w.speed < 0.6 * s.speed);
    CHECK(w.engine_kw < 0.7 * s.engine_kw);
    CHECK(weak.world->powertrain_state(weak.tractor).engine_state == drivetrain::EngineState::Running);

    // Dirt, and little on the rear axle.
    MountRig light = worked_rig(plough(1.0, 2500.0, "dirt"), 5, 0.0, kPloughAngle);
    const Steady l = working_run(light, 10.0, 3.0);
    report("100 kW, 2.5 t, dirt", l, light);
    CHECK(l.kappa > 0.5);
    CHECK(l.speed < 0.5 * s.speed);
    CHECK(l.rpm > 2000.0); // the engine has plenty, the ground does not

    // Without the auto-clutch to open under it, a small engine dies. Rolling first, then the plough goes in.
    MountRig tiny = worked_rig(plough(0.12, 6000.0, "asphalt"), 5, 0.0, 0.3);
    tiny.run(6.0);
    CHECK(tiny.world->get_motion(tiny.tractor_body).linear.x > 1.0);
    tiny.set("assist.auto_clutch", 0.0);
    tiny.set("lift", lever_for(kPloughAngle));
    tiny.run(6.0);
    report("12 kW, no auto-clutch", working_run(tiny, 0.1, 0.1), tiny);
    CHECK(tiny.world->powertrain_state(tiny.tractor).engine_state == drivetrain::EngineState::Stalled);
}

TEST_CASE("above its speed range a tool's work is worth less, below it only time is lost", "[machine][vehicle]") {
    MountRig fast = worked_rig(on_dirt(), 9);
    const Steady f = working_run(fast, 9.0, 2.0);
    printf("fast: %.2f km/h, quality %.3f, draft %.0f N\n", f.speed * 3.6, f.quality, f.draft);
    CHECK(f.speed * 3.6 > 12.0);
    CHECK(f.quality < 1.0);

    MountRig slow = worked_rig(on_dirt(), 3);
    const Steady l = working_run(slow, 6.0, 2.0);
    printf("slow: %.2f km/h, quality %.3f, draft %.0f N\n", l.speed * 3.6, l.quality, l.draft);
    CHECK(l.speed * 3.6 < 5.0);
    CHECK(l.quality == 1.0);
    CHECK(slow.world->tool_state(slow.implement, 0).working);

    std::vector<machine::WorkQuad> quads;
    fast.world->drain_work(quads);
    REQUIRE_FALSE(quads.empty());
    CHECK(quads.back().quality < 1.0);
}

TEST_CASE("draft control holds the pull at its setting by raising the tool", "[machine][vehicle]") {
    MountRig r = worked_rig(on_dirt(), 6);
    r.run(4.0);
    const real open = r.world->tool_state(r.implement, 0).draft_n;
    const real setting = 0.7 * open;
    r.set("lift_draft", setting);
    r.set("lift_mode", 1.0);
    r.run(8.0);
    const machine::CouplingState& c = r.world->coupling_state(r.hitch);
    printf("draft control: open %.0f N, set %.0f N, sensed %.0f N, tool %.0f N at %.1f cm\n", open, setting, c.linkage.sensed_draft_n, r.world->tool_state(r.implement, 0).draft_n, r.world->tool_state(r.implement, 0).depth_m * 100.0);
    CHECK(c.linkage.mode == machine::LiftMode::Draft);
    CHECK(c.linkage.sensed_draft_n == Approx(setting).epsilon(0.15));
    CHECK(r.world->tool_state(r.implement, 0).draft_n < 0.9 * open);
}

TEST_CASE("work: a tool sweeps its width times the distance, tick by tick, in order", "[machine][vehicle]") {
    MountRig r = worked_rig(on_dirt(), 5);
    r.run(4.0);
    std::vector<machine::WorkQuad> quads;
    r.world->drain_work(quads);
    quads.clear();
    const machine::ToolDesc& tool = r.world->tool_desc(r.implement, 0);
    const real area0 = r.world->tool_state(r.implement, 0).worked_area_m2;
    const real x0 = r.world->get_pose(r.implement_body).position.x;
    r.run(3.0);
    r.world->drain_work(quads);
    const real moved = r.world->get_pose(r.implement_body).position.x - x0;

    REQUIRE(quads.size() == 3 * 240);
    real area = 0.0;
    for (std::size_t i = 0; i < quads.size(); ++i) {
        area += machine::work_area(quads[i]);
        if (i > 0) {
            REQUIRE(quads[i].tick == quads[i - 1].tick + 1);
            // Each strip starts where the last ended: nothing between the ticks is lost.
            REQUIRE(quads[i].left_start.x == quads[i - 1].left_end.x);
            REQUIRE(quads[i].right_start.y == quads[i - 1].right_end.y);
        }
    }
    CHECK(area == Approx(tool.width_m * moved).epsilon(0.002));
    CHECK(r.world->tool_state(r.implement, 0).worked_area_m2 - area0 == Approx(area).epsilon(1e-9));
    CHECK(quads.front().vehicle == r.implement);
    CHECK(quads.front().depth_m > 0.05);

    SECTION("nothing is worked lifted out") {
        r.set("lift", 0.9);
        r.run(3.0);
        quads.clear();
        r.world->drain_work(quads);
        CHECK(quads.size() < 3 * 240);
        quads.clear();
        r.run(1.0);
        r.world->drain_work(quads);
        CHECK(quads.empty());
        CHECK_FALSE(r.world->tool_state(r.implement, 0).working);

        // Back in the ground the strip starts where the tool is, not where it came out.
        r.set("lift", lever_for(kWorkingAngle));
        r.run(3.0);
        quads.clear();
        r.world->drain_work(quads);
        REQUIRE_FALSE(quads.empty());
        real largest = 0.0;
        for (const machine::WorkQuad& q : quads) largest = std::max(largest, machine::work_area(q));
        CHECK(largest < tool.width_m * 3.0 / 240.0);
    }
    SECTION("a road: the tool scrapes on it, sweeps nothing and does not sink") {
        machine::SoilSample road;
        road.workable = false;
        r.world->set_soil_source(std::make_shared<UniformSoil>(road));
        r.run(2.0);
        quads.clear();
        r.world->drain_work(quads);
        r.run(1.0);
        quads.clear();
        r.world->drain_work(quads);
        CHECK(quads.empty());
        CHECK(r.world->tool_state(r.implement, 0).depth_m < 0.02);
        CHECK(r.world->tool_state(r.implement, 0).draft_n > 0.0);
    }
    SECTION("standing still sweeps nothing") {
        r.set("throttle", 0.0);
        r.set("brake", 1.0);
        r.run(4.0);
        quads.clear();
        r.world->drain_work(quads);
        CHECK(r.world->get_motion(r.tractor_body).linear.length() < 0.01);
        quads.clear();
        r.run(1.0);
        r.world->drain_work(quads);
        CHECK(quads.empty());
    }
    SECTION("a full buffer drops the newest and counts them") {
        r.run(4100.0 / 240.0);
        CHECK(r.world->work_dropped() > 0);
        quads.clear();
        r.world->drain_work(quads);
        CHECK(quads.size() == 4096);
    }
}

TEST_CASE("a tool in ground that cannot be worked works nothing, however deep it is", "[machine][vehicle]") {
    // A road soft enough for the edge to be below its surface at the working link angle: only `workable` says no.
    MountCfg cfg = on_dirt();
    cfg.edit_implement = [](vehicle::VehicleDesc& d) { d.machine->tools[0].soil.hard_n_m2 = d.machine->tools[0].soil.bearing_n_m2; };
    machine::SoilSample road;
    road.workable = false;
    MountRig r = worked_rig(cfg, 5, 0.0, kWorkingAngle, road);
    r.run(3.0);
    const machine::ToolState& tool = r.world->tool_state(r.implement, 0);
    CHECK(r.world->get_motion(r.tractor_body).linear.x > 0.5);
    CHECK(tool.depth_m > r.world->tool_desc(r.implement, 0).depth_min_m);
    CHECK_FALSE(tool.working);
    std::vector<machine::WorkQuad> quads;
    r.world->drain_work(quads);
    CHECK(quads.empty());
    CHECK(tool.worked_area_m2 == 0.0);
}

TEST_CASE("the first tick in the ground starts a strip and writes none", "[machine][vehicle]") {
    // There is no edge to sweep from yet: a quad here would run from the world's origin.
    MountCfg cfg = on_dirt();
    cfg.edit_implement = [](vehicle::VehicleDesc& d) {
        d.machine->tools[0].edge_left_local.z -= 0.2;
        d.machine->tools[0].edge_right_local.z -= 0.2;
    };
    MountRig r = build_mount(cfg);
    r.world->set_soil_source(std::make_shared<UniformSoil>(field_soil()));
    r.world->step();
    const machine::ToolState& tool = r.world->tool_state(r.implement, 0);
    CHECK(tool.depth_m > r.world->tool_desc(r.implement, 0).depth_min_m);
    CHECK(tool.working);
    std::vector<machine::WorkQuad> quads;
    r.world->drain_work(quads);
    CHECK(quads.empty());
    CHECK(tool.worked_area_m2 == 0.0);
}

TEST_CASE("work from two tools drains ordered by tick, vehicle and tool", "[machine][vehicle]") {
    MountCfg cfg = on_dirt();
    cfg.edit_implement = [](vehicle::VehicleDesc& d) {
        // The one width split in two.
        machine::ToolDesc& first = d.machine->tools[0];
        machine::ToolDesc second = first;
        second.name = "second";
        first.edge_right_local.y = 0.0;
        second.edge_left_local.y = 0.0;
        first.width_m = second.width_m = 0.5 * first.width_m;
        d.machine->tools.push_back(second);
    };
    MountRig r = worked_rig(cfg, 5);
    r.run(4.0);
    std::vector<machine::WorkQuad> quads;
    r.world->drain_work(quads);
    REQUIRE(quads.size() > 200);
    CHECK(std::count_if(quads.begin(), quads.end(), [](const machine::WorkQuad& q) { return q.tool == 1; }) > 100);
    for (std::size_t i = 1; i < quads.size(); ++i) {
        const machine::WorkQuad& a = quads[i - 1];
        const machine::WorkQuad& b = quads[i];
        REQUIRE((a.tick < b.tick || (a.tick == b.tick && a.tool < b.tool)));
    }
}

TEST_CASE("work_coverage: a quad's area per cell, conserved however it lies", "[machine]") {
    machine::WorkGrid grid;
    grid.x0 = -10.0;
    grid.y0 = -10.0;
    grid.cell_m = 0.5;
    grid.nx = 40;
    grid.ny = 40;
    const auto quad = [](real x0, real x1, real y_left, real y_right) {
        machine::WorkQuad q;
        q.left_start = {x0, y_left};
        q.right_start = {x0, y_right};
        q.right_end = {x1, y_right};
        q.left_end = {x1, y_left};
        return q;
    };
    const auto covered_m2 = [&grid](const std::vector<machine::CellCoverage>& v) {
        real sum = 0.0;
        for (const machine::CellCoverage& c : v) sum += c.coverage * grid.cell_m * grid.cell_m;
        return sum;
    };

    SECTION("exactly one cell") {
        std::vector<machine::CellCoverage> out;
        machine::work_coverage(quad(0.0, 0.5, 0.5, 0.0), grid, out);
        REQUIRE(out.size() == 1);
        CHECK(out[0].cell == 20 + 40 * 20);
        CHECK(out[0].coverage == Approx(1.0));
    }
    SECTION("a thin strip across two cells") {
        std::vector<machine::CellCoverage> out;
        machine::work_coverage(quad(0.25, 0.75, 0.3, 0.2), grid, out);
        REQUIRE(out.size() == 2);
        CHECK(out[0].cell < out[1].cell);
        CHECK(out[0].coverage == Approx(0.25 * 0.1 / 0.25));
        CHECK(out[1].coverage == Approx(0.25 * 0.1 / 0.25));
    }
    SECTION("what lies outside the grid is dropped, the area still knows it") {
        const machine::WorkQuad q = quad(-11.0, -9.0, 0.5, 0.0);
        std::vector<machine::CellCoverage> out;
        machine::work_coverage(q, grid, out);
        CHECK(covered_m2(out) == Approx(0.5).epsilon(1e-12));
        CHECK(machine::work_area(q) == Approx(1.0).epsilon(1e-12));
    }
    SECTION("a pivoting edge crosses itself: both of its triangles count") {
        machine::WorkQuad q;
        q.left_start = {0.0, 1.0};
        q.right_start = {0.0, -1.0};
        q.right_end = {1.0, 1.0};
        q.left_end = {1.0, -1.0};
        std::vector<machine::CellCoverage> out;
        machine::work_coverage(q, grid, out);
        CHECK(machine::work_area(q) == Approx(1.0).epsilon(1e-12));
        CHECK(covered_m2(out) == Approx(1.0).epsilon(1e-12));
    }
    SECTION("random quads: the cells add up to the quad") {
        std::mt19937_64 rng(11);
        std::uniform_real_distribution<real> u(-1.0, 1.0);
        for (int i = 0; i < 2000; ++i) {
            machine::WorkQuad q;
            const real cx = 3.0 * u(rng), cy = 3.0 * u(rng), heading = 3.2 * u(rng), half = 0.2 + 1.5 * (u(rng) + 1.0);
            const real step = 0.05 + 0.1 * (u(rng) + 1.0), turn = 0.3 * u(rng);
            const auto edge = [](real x, real y, real a, real side) { return machine::WorkPoint{x - side * std::sin(a), y + side * std::cos(a)}; };
            q.left_start = edge(cx, cy, heading, half);
            q.right_start = edge(cx, cy, heading, -half);
            const real ex = cx + step * std::cos(heading), ey = cy + step * std::sin(heading);
            q.left_end = edge(ex, ey, heading + turn, half);
            q.right_end = edge(ex, ey, heading + turn, -half);
            std::vector<machine::CellCoverage> out;
            machine::work_coverage(q, grid, out);
            REQUIRE(covered_m2(out) == Approx(machine::work_area(q)).epsilon(1e-9));
            for (std::size_t k = 1; k < out.size(); ++k) REQUIRE(out[k - 1].cell < out[k].cell);
            for (const machine::CellCoverage& c : out) REQUIRE(c.coverage <= 1.0 + 1e-9);
        }
    }
    SECTION("the ledger gathers a pass into one coverage per cell, weighted means, sorted") {
        machine::WorkLedger ledger(grid);
        VehicleId v{};
        v.index = 3;
        v.generation = 1;
        // Four strips of different widths filling one cell, the last, a tenth of it, shallower and worse.
        const real edges[5] = {0.0, 0.2, 0.4, 0.45, 0.5};
        for (int i = 0; i < 4; ++i) {
            machine::WorkQuad q = quad(edges[i], edges[i + 1], 0.5, 0.0);
            q.vehicle = v;
            q.tool = 1;
            q.depth_m = i < 3 ? 0.1 : 0.06;
            q.quality = i < 3 ? 1.0 : 0.6;
            ledger.add(q);
        }
        CHECK_FALSE(ledger.empty());
        const std::vector<machine::WorkCell> cells = ledger.take();
        REQUIRE(cells.size() == 1);
        CHECK(cells[0].coverage == Approx(1.0));
        CHECK(cells[0].depth_m == Approx(0.096));
        CHECK(cells[0].quality == Approx(0.96));
        CHECK(cells[0].tool == 1);
        CHECK(ledger.empty());
    }
}

TEST_CASE("the rotary harrow: PTO power from the soil going through it", "[machine][vehicle]") {
    MountRig r = worked_rig(on_dirt().rotary(), 5);
    r.set("pto_speed", 1000.0);
    r.set("pto", 1.0);
    r.world->connect_pto(r.hitch);
    const Steady s = working_run(r, 10.0, 3.0);
    const machine::ToolState& t = r.world->tool_state(r.implement, 0);
    const machine::ToolDesc& tool = r.world->tool_desc(r.implement, 0);
    const drivetrain::PowertrainSnapshot& p = r.world->powertrain_state(r.tractor);
    const real omega = p.pto_rpm * kTwoPi / 60.0;
    const real rotor_kw = t.pto_torque_nm * omega * 1e-3;
    const real expected_kw = machine::tool_pto_power_w(tool, s.speed, s.depth) * 1e-3;
    printf("rotary 3 m: %.2f km/h at %.1f cm, shaft %.0f rpm, %.1f N*m, rotor %.1f kW (law %.1f kW), draft %.0f N, engine %.1f kW at %.0f rpm\n", s.speed * 3.6, s.depth * 100.0, p.pto_rpm, t.pto_torque_nm, rotor_kw, expected_kw, s.draft,
           s.engine_kw, s.rpm);
    CHECK(t.working);
    CHECK(rotor_kw == Approx(expected_kw).epsilon(0.05));
    CHECK(s.engine_kw > rotor_kw + s.draft * s.speed * 1e-3);

    r.world->disconnect_pto(r.hitch);
    r.run(1.0);
    CHECK_FALSE(r.world->tool_state(r.implement, 0).working);
}

TEST_CASE("work is the same on 1 and 10 workers, and the tick allocates nothing", "[machine][vehicle]") {
    const auto scenario = [](int workers, bool measure) {
        MountCfg cfg = on_dirt().rotary();
        cfg.workers = workers;
        MountRig r = worked_rig(cfg, 5);
        r.set("pto_speed", 1000.0);
        r.set("pto", 1.0);
        r.world->connect_pto(r.hitch);
        r.run(2.0);
        if (measure && jobs::debug_alloc::hook_available()) {
            jobs::debug_alloc::reset();
            jobs::debug_alloc::set_armed(true);
        }
        r.run(1.0);
        if (measure && jobs::debug_alloc::hook_available()) {
            jobs::debug_alloc::set_armed(false);
            CHECK(jobs::debug_alloc::own_violation_count() == 0);
        }
        std::vector<machine::WorkQuad> quads;
        r.world->drain_work(quads);
        std::uint64_t h = r.world->state_hash();
        for (const machine::WorkQuad& q : quads) {
            for (const real v : {q.left_end.x, q.left_end.y, q.right_end.x, q.right_end.y, q.depth_m}) {
                h ^= std::hash<real>{}(v);
                h *= 1099511628211ULL;
            }
        }
        return h;
    };
    const std::uint64_t one = scenario(1, false);
    CHECK(one == scenario(10, false));
    (void)scenario(4, true);
}

// ---------------------------------------------------------------------------------------------------------------------------
// Applicators: tools that put something on the ground without touching it (machine.md "kind").
//
// Sabotages, run one at a time against "[applicator]" on 2026-10-31, each with the case that caught it (a first run let three
// survive - the switch not hashed, a draft left in, an unconditional fold - and the cases below were tightened until they failed):
//   the switch ignored (works while off)                         -> an applicator works only while switched on
//   the soil source's workable ignored                           -> an applicator works only while switched on
//   working at |speed| > 0 (reversing counts)                   -> an applicator works only while switched on
//   the draft form applied to an applicator                      -> an applicator has no draft ...
//   the enabled flag folded for every tool                       -> a soil tool's state hash is the one it had
//   the enabled flag never folded                                -> the switch is state for an applicator only
//   a soil tool accepted by set_tool_enabled                     -> the switch is state for an applicator only
//   quality unchanged above the speed range                      -> an applicator takes the speed range's quality
//   draft/soil/depth keys accepted on an applicator              -> applicator parts: the loader's rules
// ---------------------------------------------------------------------------------------------------------------------------

namespace {

MountCfg sprayer_cfg() {
    MountCfg cfg = on_dirt();
    cfg.fixture = "mounted_sprayer";
    cfg.implement_mass = 900.0;
    return cfg;
}

MountRig sprayer_rig(int gear, real awd = 0.0, const machine::SoilSample& soil = field_soil()) {
    MountRig r = build_mount(sprayer_cfg());
    r.world->set_soil_source(std::make_shared<UniformSoil>(soil));
    r.run(0.6);
    r.set("lift", 0.5);
    r.set("gear_request", gear);
    r.set("awd", awd);
    r.set("throttle", 1.0);
    return r;
}

machine::SoilSample unworkable_soil() {
    machine::SoilSample s = field_soil();
    s.workable = false;
    return s;
}

} // namespace

TEST_CASE("an applicator works only while switched on, over workable ground, moving forward", "[machine][vehicle][applicator]") {
    MountRig r = sprayer_rig(5);
    const machine::ToolDesc& tool = r.world->tool_desc(r.implement, 0);
    REQUIRE(tool.kind == machine::ToolKind::Applicator);
    CHECK_FALSE(r.world->tool_state(r.implement, 0).enabled);
    std::vector<machine::WorkQuad> quads;

    SECTION("off: it drives, it writes nothing") {
        r.run(4.0);
        CHECK(r.world->get_motion(r.tractor_body).linear.x > 1.0);
        r.world->drain_work(quads);
        CHECK(quads.empty());
        CHECK_FALSE(r.world->tool_state(r.implement, 0).working);
        CHECK(r.world->tool_state(r.implement, 0).worked_area_m2 == 0.0);
    }

    SECTION("on: the boom's width times the distance, at depth 0") {
        r.run(3.0);
        r.world->set_tool_enabled(r.implement, 0, true);
        CHECK(r.world->tool_state(r.implement, 0).enabled);
        r.run(0.5); // the first tick writes no quad
        r.world->drain_work(quads);
        quads.clear();
        const real x0 = r.world->get_pose(r.implement_body).position.x;
        r.run(3.0);
        r.world->drain_work(quads);
        const real moved = r.world->get_pose(r.implement_body).position.x - x0;
        REQUIRE(quads.size() == 3 * 240);
        real area = 0.0;
        for (const machine::WorkQuad& q : quads) {
            area += machine::work_area(q);
            CHECK(q.depth_m == 0.0);
            CHECK(q.quality > 0.0);
        }
        CHECK(tool.width_m == Approx(6.0));
        CHECK(area == Approx(tool.width_m * moved).epsilon(0.003));
        CHECK(r.world->tool_state(r.implement, 0).working);
        CHECK(quads.front().vehicle == r.implement);

        // Off again: nothing more is written.
        r.world->set_tool_enabled(r.implement, 0, false);
        r.run(0.1);
        quads.clear();
        r.world->drain_work(quads);
        quads.clear();
        r.run(2.0);
        r.world->drain_work(quads);
        CHECK(quads.empty());
    }

    SECTION("on, over ground that cannot be worked: nothing") {
        r.world->set_soil_source(std::make_shared<UniformSoil>(unworkable_soil()));
        r.world->set_tool_enabled(r.implement, 0, true);
        r.run(4.0);
        CHECK(r.world->get_motion(r.tractor_body).linear.x > 1.0);
        r.world->drain_work(quads);
        CHECK(quads.empty());
        CHECK_FALSE(r.world->tool_state(r.implement, 0).working);
    }

    SECTION("on, standing: only the settling of the springs, centimetres") {
        r.set("throttle", 0.0);
        r.set("brake", 1.0);
        r.world->set_tool_enabled(r.implement, 0, true);
        r.run(3.0);
        // A standing machine rocks on its springs for a moment, at well under the farm's 0.05 m/s "moving" bound.
        CHECK(r.world->tool_state(r.implement, 0).worked_area_m2 < 0.3);
        CHECK(std::abs(r.world->get_motion(r.tractor_body).linear.x) < 0.01);
        quads.clear();
        r.world->drain_work(quads);
        quads.clear();
        r.run(3.0);
        r.world->drain_work(quads);
        CHECK(quads.empty());
    }

    SECTION("on, reversing: nothing") {
        r.set("gear_request", -1);
        r.world->set_tool_enabled(r.implement, 0, true);
        r.run(4.0);
        REQUIRE(r.world->get_motion(r.tractor_body).linear.x < -0.3);
        quads.clear();
        r.world->drain_work(quads);
        quads.clear();
        r.run(2.0);
        REQUIRE(r.world->get_motion(r.tractor_body).linear.x < -0.3);
        r.world->drain_work(quads);
        CHECK(quads.empty());
        CHECK_FALSE(r.world->tool_state(r.implement, 0).working);
    }
}

TEST_CASE("an applicator has no draft, no soil force and no depth: the tractor does not feel it", "[machine][vehicle][applicator]") {
    // The same tractor with the same tool hung, switched off and then on: the pull at the hitch is the machine's weight and
    // nothing else, whatever the switch says.
    const auto run = [](bool on) {
        MountRig r = sprayer_rig(5);
        r.world->set_tool_enabled(r.implement, 0, on);
        r.run(5.0);
        const machine::ToolState& t = r.world->tool_state(r.implement, 0);
        CHECK(t.draft_n == 0.0);
        CHECK(t.vertical_n == 0.0);
        CHECK(t.depth_m == 0.0);
        return std::pair<real, real>{r.world->get_motion(r.tractor_body).linear.x, r.world->coupling_state(r.hitch).mean_force_on_towing_world.x};
    };
    const auto off = run(false);
    const auto on = run(true);
    CHECK(on.first == Approx(off.first).margin(1e-6));
    CHECK(on.second == Approx(off.second).margin(1e-6));
    // Even a tool that still carries a soil tool's draft numbers has none as an applicator.
    machine::ToolDesc carries = part_tool("cultivator_3m");
    REQUIRE(machine::tool_draft_n(carries, field_soil(), 8.0, 0.1) > 100.0);
    carries.kind = machine::ToolKind::Applicator;
    CHECK(machine::tool_draft_n(carries, field_soil(), 8.0, 0.1) == 0.0);
}

TEST_CASE("a soil tool's state hash is the one it had before applicators existed", "[machine][vehicle][applicator]") {
    // Recorded 2026-10-31 from the guarded fold, which for a soil tool is no fold at all: this is what the cultivator rig hashed to without
    // it (the fold is guarded by the tool's kind). Same on 1 and 10 workers.
    const auto hash = [](int workers) {
        MountCfg cfg = on_dirt();
        cfg.workers = workers;
        MountRig r = worked_rig(cfg, 5);
        r.run(2.0);
        return r.world->state_hash();
    };
    const std::uint64_t one = hash(1);
    CHECK(one == hash(10));
    CHECK(one == 0x48633e13e74d9aa5ULL);
}

TEST_CASE("an applicator takes the speed range's quality", "[machine][vehicle][applicator]") {
    const machine::ToolDesc tool = io::load_machine_json(std::string(PS_SOURCE_DIR) + "/data/machines/sprayer_boom.json", {{"socket_work_L", Vec3{0, 3, 0}}, {"socket_work_R", Vec3{0, -3, 0}}}).tools[0];
    CHECK(machine::tool_quality(tool, tool.speed_max_km_h) == Approx(1.0));
    CHECK(machine::tool_quality(tool, tool.speed_max_km_h + 2.0) == Approx(1.0 - 2.0 * tool.overspeed_quality_per_km_h));
    CHECK(machine::tool_quality(tool, 100.0) == Approx(tool.quality_floor));
    // And in the running tool: a gear faster than the boom's top speed writes a lower quality.
    MountRig r = sprayer_rig(9);
    r.world->set_tool_enabled(r.implement, 0, true);
    r.run(12.0);
    const machine::ToolState& t = r.world->tool_state(r.implement, 0);
    REQUIRE(t.speed_m_s * 3.6 > tool.speed_max_km_h + 1.0);
    CHECK(t.quality < 1.0);
    CHECK(t.quality == Approx(machine::tool_quality(tool, t.speed_m_s * 3.6)).epsilon(0.05));
}

TEST_CASE("the switch is state for an applicator only", "[machine][vehicle][applicator]") {
    // Over ground that cannot be worked the switch changes nothing else, so the hash moves only if the switch itself is folded.
    const auto hash = [](bool toggle) {
        MountRig r = build_mount(sprayer_cfg());
        r.world->set_soil_source(std::make_shared<UniformSoil>(unworkable_soil()));
        r.run(1.0);
        if (toggle) r.world->set_tool_enabled(r.implement, 0, true);
        r.run(0.05);
        return r.world->state_hash();
    };
    CHECK(hash(true) != hash(false));
    CHECK(hash(true) == hash(true));

    // A soil tool has no switch: refused, loudly.
    MountRig r = build_mount(on_dirt());
    CHECK_THROWS_AS(r.world->set_tool_enabled(r.implement, 0, true), std::invalid_argument);
    CHECK_FALSE(r.world->tool_state(r.implement, 0).enabled);
}

TEST_CASE("applicator parts: the loader's rules", "[machine][io][applicator]") {
    const io::MachineSockets sockets = {{"socket_work_L", Vec3{0, 3, 0}}, {"socket_work_R", Vec3{0, -3, 0}}};
    for (const char* name : {"sprayer_boom", "spreader_broadcast", "slurry_splash", "combine_header"}) {
        const machine::MachineDesc m = io::load_machine_json(std::string(PS_SOURCE_DIR) + "/data/machines/" + name + ".json", sockets);
        REQUIRE(m.tools.size() == 1);
        CHECK(m.tools[0].kind == machine::ToolKind::Applicator);
        CHECK(m.tools[0].width_m == Approx(6.0));
        CHECK(m.tools[0].depth_max_m == 0.0);
    }
    // Every other shipped part is still a soil tool.
    CHECK(part_tool("cultivator_3m").kind == machine::ToolKind::Soil);
    CHECK(part_tool("plough_2m").kind == machine::ToolKind::Soil);

    const auto text = [](const std::string& range_extra, const std::string& tool_extra, const std::string& kind) {
        return std::string(R"({"format":"physics_sim.machine/1","name":"x","source":"x","tools":[{"name":"t","kind":")") + kind +
               R"(","operation":"sprayer","edge_left":"socket_work_L","edge_right":"socket_work_R",)" + tool_extra +
               R"("range":{"source":"[AS] x","speed_min_km_h":4,"speed_max_km_h":12,"overspeed_quality_per_km_h":0.1,"quality_floor":0.3)" + range_extra + "}}]}";
    };
    CHECK_NOTHROW(io::load_machine_json(scratch_file("app_ok.json", text("", "", "applicator")), sockets));
    CHECK_THROWS_WITH(io::load_machine_json(scratch_file("app_depth.json", text(R"(,"depth_min_m":0.05)", "", "applicator")), sockets), Catch::Matchers::ContainsSubstring("depth_min_m"));
    CHECK_THROWS_WITH(io::load_machine_json(scratch_file("app_draft.json", text("", R"("draft":{},)", "applicator")), sockets), Catch::Matchers::ContainsSubstring("an applicator has no 'draft' block"));
    CHECK_THROWS_WITH(io::load_machine_json(scratch_file("app_pto.json", text("", R"("pto":{},)", "applicator")), sockets), Catch::Matchers::ContainsSubstring("an applicator has no 'pto' block"));
    CHECK_THROWS_WITH(io::load_machine_json(scratch_file("app_kind.json", text("", "", "sprinkler")), sockets), Catch::Matchers::ContainsSubstring("'kind' must be"));
    // A soil tool still needs all of its blocks.
    CHECK_THROWS(io::load_machine_json(scratch_file("soil_bare.json", text("", "", "soil")), sockets));
}

TEST_CASE("Ledger diag: where engine_out goes, tool lifted and in ground", "[.][diag][machine][vehicle]") {
    for (int mode = 0; mode < 2; ++mode) {
        MountRig r = worked_rig(on_dirt(), 6, 0.0, mode == 0 ? 0.3 : kWorkingAngle);
        const int ticks = 9 * 240, from = 6 * 240;
        drivetrain::PowertrainEnergy e0;
        real ke0 = 0, hx = 0, v = 0;
        for (int i = 0; i < ticks; ++i) {
            if (i == from) e0 = r.world->powertrain_energy(r.tractor);
            r.world->step();
            if (i >= from) {
                hx += r.world->coupling_state(r.hitch).mean_force_on_towing_world.x;
                v += r.world->get_motion(r.tractor_body).linear.x;
            }
        }
        const auto e1 = r.world->powertrain_energy(r.tractor);
        const real T = (ticks - from) / 240.0;
        const real n = ticks - from;
        auto kw = [&](real a, real b) { return (b - a) / T * 1e-3; };
        printf("== %s: v %.3f m/s, hitch x %.0f N (%.2f kW)\n", mode == 0 ? "lifted" : "in ground", v / n, hx / n, -hx / n * v / n * 1e-3);
        printf("engine_out %.2f starter %.2f wheel_out %.2f tyre %.2f brake %.2f gear %.2f diff %.2f awd %.2f pto %.2f ext %.2f integ %.2f clutch %.2f synchro %.2f engfric %.2f engage %.2f dKE %.2f balerr(last) %.2e\n",
               kw(e0.engine_out, e1.engine_out), kw(e0.starter_in, e1.starter_in), kw(e0.wheel_out, e1.wheel_out), kw(e0.tyre_loss, e1.tyre_loss), kw(e0.brake_loss, e1.brake_loss), kw(e0.gear_loss, e1.gear_loss),
               kw(e0.diff_loss, e1.diff_loss), kw(e0.awd_clutch_loss, e1.awd_clutch_loss), kw(e0.pto_clutch_loss, e1.pto_clutch_loss), kw(e0.external_absorbed, e1.external_absorbed),
               kw(e0.integrator_loss, e1.integrator_loss), kw(e0.clutch_loss, e1.clutch_loss), kw(e0.synchro_loss, e1.synchro_loss), kw(e0.engine_friction_loss, e1.engine_friction_loss),
               kw(e0.engagement_loss, e1.engagement_loss), kw(e0.rotor_ke, e1.rotor_ke), e1.balance_error);
        printf("engine pumping %.2f friction %.2f indicated %.2f\n", kw(e0.pumping_work, e1.pumping_work), kw(e0.friction_work, e1.friction_work), kw(e0.indicated_work, e1.indicated_work));
        (void)ke0;
    }
}
