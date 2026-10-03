// One-shot native timing probe; not a CI wall-clock gate.
#include "ps/world/world.h"
#include "ps/io/vehicle_io.h"
#include "ps/io/test_ground_loader.h"
#include "ps/io/surface_table.h"
#include "ps/math/transcendental.h"
#include <chrono>
#include <iostream>
#include <string>
#include <vector>
using namespace ps;
namespace {
template<class Fn> double time_us(int n,Fn&& fn) {
    const auto start=std::chrono::steady_clock::now(); for(int i=0;i<n;++i) fn();
    return std::chrono::duration<double,std::micro>(std::chrono::steady_clock::now()-start).count()/n;
}
void run(int workers,bool aero_enabled,bool live_ambient,const vehicle::VehicleDesc& initial,const std::string& csv) {
    WorldConfig cfg; cfg.job_workers=workers; cfg.gravity={}; World world(cfg);
    // 25 full-resolution tiles at the reported coordinate, each with nonflat samples.
    for(int y=-2;y<=2;++y) for(int x=-2;x<=2;++x) {
        HeightfieldShape h; h.sample_count=256; h.sample_spacing=1;
        h.heights.resize(256*256);
        for(int iy=0;iy<256;++iy) for(int ix=0;ix<256;++ix)
            h.heights[std::size_t(iy)*256+std::size_t(ix)]=251.0+.05*ps::math::sin(real(ix)*.1)*ps::math::cos(real(iy)*.1);
        BodyDesc b; b.motion=BodyMotionType::Static; b.shape=std::move(h);
        b.pose.position={-3100+255*real(x),-3690+255*real(y),0}; world.create_body(b);
    }
    BodyDesc b; b.motion=BodyMotionType::Dynamic; b.mass=1500; b.allow_sleep=false;
    b.shape=BoxShape{{1,.4,.1}}; b.pose.position={-2955.3,-3562.8,251.5}; const auto body=world.create_body(b);
    vehicle::VehicleDesc d=initial;
    if(d.name.empty()) {
        d.name="probe"; aero::SurfaceDesc surface; surface.name="floor"; surface.cl0=-.5;
        surface.position_local={.1,0,-.39}; surface.ground.range_m=.65; surface.ground.gain=.55; d.aero.surfaces.push_back(surface);
    }
    if(!aero_enabled) d.aero={};
    const auto vehicle=world.create_vehicle(d,body);
    int hits=0; auto ray=[&] { const auto hit=world.backend().ray_cast_excluding({-2955.2,-3562.8,251.11},{0,0,-1},.65,body); hits+=hit.hit?1:0; };
    const auto ray_us=time_us(5000,ray);
    if(!csv.empty()) world.telemetry().start_csv(csv);
    for(int i=0;i<20;++i) world.step();
    int tick=0;
    const auto step_us=time_us(20,[&] {
        // Microscopic vertical motion at altitude must not regenerate a full control grid.
        if(live_ambient) world.set_ambient({98000+real(tick)*.001,286.5+real(tick)*.00001});
        ++tick; world.backend().set_motion(body,{}); world.step();
    });
    world.telemetry().stop();
    std::cout<<"workers="<<workers<<" aero="<<aero_enabled<<" live_ambient="<<live_ambient<<" ray_us="<<ray_us<<" hits="<<hits<<" tick_us="<<step_us
             <<" q="<<world.vehicle_aero_telemetry(vehicle).body_dynamic_pressure_pa<<'\n';
}
}
int main(int argc,char** argv) {
    const std::string vehicle_path=argc>1?argv[1]:"";
    vehicle::VehicleDesc d; if(!vehicle_path.empty()) d=io::load_vehicle_json(vehicle_path);
    for(int workers:{1,4}) for(bool enabled:{false,true}) for(bool ambient:{false,true}) run(workers,enabled,ambient,d,"");
}
