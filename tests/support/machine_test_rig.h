// tests/support/machine_test_rig.h — a tractor and a mounted implement on a flat
// ground, for tests/unit/test_machine_linkage.cpp and test_machine_tools.cpp.
//
// The implement comes from tests/support/fixtures/machines: its centre of mass 1.2 m
// behind the lower pins, its working edge 0.6 m under them, which puts the edge on the
// ground with the lower links at the hitch point's own pose. Masses are the BodyDesc's,
// as everywhere: a machine part never carries one.

#pragma once

#include "ps/backend/i_rigid_backend.h"
#include "ps/io/surface_table.h"
#include "ps/io/vehicle_io.h"
#include "ps/machine/soil_source.h"
#include "ps/world/world.h"
#include "vehicle_test_helpers.h"

#include <functional>
#include <memory>
#include <string>
#include <utility>

namespace ps_test {

class UniformSoil final : public ps::machine::ISoilSource {
public:
    explicit UniformSoil(const ps::machine::SoilSample& sample) : sample_(sample) {}
    [[nodiscard]] ps::machine::SoilSample sample(ps::real, ps::real) const noexcept override { return sample_; }

private:
    ps::machine::SoilSample sample_;
};

[[nodiscard]] inline ps::machine::SoilSample field_soil(ps::machine::SoilTexture texture = ps::machine::SoilTexture::Medium) {
    ps::machine::SoilSample s;
    s.workable = true;
    s.texture = texture;
    s.wetness = 0.5;
    return s;
}

struct MountCfg {
    ps::real implement_mass = 1100.0;
    ps::real tractor_mass = 6000.0;
    std::string ground = "asphalt"; // a surfaces.json name
    std::string fixture = "mounted_cultivator";
    int workers = 1;
    bool attach = true;
    bool bare_tractor = false; // no implement at all
    std::function<void(ps::vehicle::VehicleDesc&)> edit_tractor;
    std::function<void(ps::vehicle::VehicleDesc&)> edit_implement;

    [[nodiscard]] MountCfg rotary() && {
        fixture = "mounted_rotary_harrow";
        implement_mass = 1400.0;
        return std::move(*this);
    }
};

struct MountRig {
    std::unique_ptr<ps::World> world;
    std::shared_ptr<ps::io::SurfaceTable> surfaces;
    ps::BodyId tractor_body{}, implement_body{};
    ps::VehicleId tractor{}, implement{};
    ps::HitchId hitch{};

    void set(const char* channel, ps::real value) const { world->set_control(world->vehicle_channel(tractor, channel), value); }
    void run(ps::real seconds) const {
        const int ticks = static_cast<int>(seconds * 240.0 + 0.5);
        for (int i = 0; i < ticks; ++i) world->step();
    }
    [[nodiscard]] ps::real axle_load(bool front) const {
        ps::real sum = 0.0;
        for (int i = front ? 0 : 2; i < (front ? 2 : 4); ++i) sum += world->wheel_telemetry(tractor, static_cast<std::size_t>(i)).fz;
        return sum;
    }
};

[[nodiscard]] inline MountRig build_mount(const MountCfg& cfg = {}) {
    using namespace ps;
    MountRig r;
    WorldConfig wc;
    wc.tick_rate_hz = 240.0;
    wc.substep_rate_hz = 960.0;
    wc.job_workers = cfg.workers;
    r.world = std::make_unique<World>(wc);
    r.surfaces = std::make_shared<io::SurfaceTable>(std::string(PS_SOURCE_DIR) + "/data/surfaces/surfaces.json");
    r.world->set_surface_table(r.surfaces);

    BodyDesc ground;
    ground.motion = BodyMotionType::Static;
    ground.friction = 1.0;
    r.world->create_body(ground, r.world->create_shape(flat_surface_mesh(1000.0, r.surfaces->id_for(cfg.ground))));

    vehicle::VehicleDesc tractor = io::load_vehicle_json(std::string(PS_SOURCE_DIR) + "/data/vehicles/tractor.json");
    if (cfg.edit_tractor) cfg.edit_tractor(tractor);
    BodyDesc a;
    a.shape = BoxShape{Vec3{1.2, 0.35, 0.9}};
    a.mass = cfg.tractor_mass;
    a.friction = 0.5;
    a.pose.position = Vec3{0.0, 0.0, 1.28};
    r.tractor_body = r.world->create_body(a);
    r.tractor = r.world->create_vehicle(tractor, r.tractor_body);
    for (const char* name : {"assist.auto_shift"}) r.world->set_control(name, 0.0);
    if (cfg.bare_tractor) return r;

    vehicle::VehicleDesc implement = io::load_vehicle_json(std::string(PS_SOURCE_DIR) + "/tests/support/fixtures/machines/" + cfg.fixture + ".json");
    if (cfg.edit_implement) cfg.edit_implement(implement);
    BodyDesc b;
    b.shape = BoxShape{Vec3{0.4, 1.0, 0.15}};
    b.mass = cfg.implement_mass;
    b.friction = 0.5;
    b.pose.position = a.pose.position + Vec3{-2.04, 0.0, -0.68} - Vec3{1.2, 0.0, 0.0};
    r.implement_body = r.world->create_body(b);
    r.implement = r.world->create_vehicle(implement, r.implement_body);
    if (cfg.attach) r.hitch = r.world->attach(r.tractor, "socket_3pt", r.implement, "socket_3pt");
    return r;
}

} // namespace ps_test
