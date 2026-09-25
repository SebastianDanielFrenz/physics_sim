// tests/support/powertrain_multi_diff_desc.h — PHYS-033 P4 I4 C4 (plan
// section 2.5/2.7): multi-differential test topologies built by rewiring
// data/vehicles/car_sedan.json's powertrain IN THE TEST ONLY (no data file):
//  - make_awd_desc: gearbox.out -> open centre diff (ratio 1) -> front and
//    rear open axle diffs (4.06, efficiency 0.97 like car_sedan's own diff)
//    -> the four wheels. The centre diff's outputs are the axle diffs'
//    carriers (a NON-leaf diff, reconstructed by the pair path).
//  - make_parallel_4wd_desc: gearbox.out shaft-joined to BOTH axle diffs'
//    carriers directly (a rigid "part-time 4WD" - two leaf diffs sharing
//    one carrier node, no centre diff).
// Every other field (wheels, brakes, engine/clutch/gearbox refs, TCU,
// tyres) is car_sedan.json's own.

#pragma once

#include "ps/drivetrain/powertrain_desc.h"
#include "ps/io/vehicle_io.h"
#include "ps/vehicle/vehicle_desc.h"

#include <stdexcept>
#include <string>
#include <variant>

namespace ps_test {

inline ps::drivetrain::PortRef comp_port(const std::string& c, const std::string& p) { return ps::drivetrain::PortRef{c, p, false}; }
inline ps::drivetrain::PortRef wheel_port(const std::string& w) { return ps::drivetrain::PortRef{w, "", true}; }

inline ps::drivetrain::ShaftDesc shaft(ps::drivetrain::PortRef a, ps::drivetrain::PortRef b) {
    ps::drivetrain::ShaftDesc s;
    s.ports = {std::move(a), std::move(b)};
    return s;
}

// car_sedan.json's own "diff" component, removed from `desc.powertrain` and
// returned (template for the new diffs: kind, ratio, efficiency, carrier
// inertia/axis). Also drops every shaft touching a diff port or a wheel.
inline ps::drivetrain::DifferentialDesc strip_sedan_diff(ps::vehicle::VehicleDesc& desc) {
    ps::drivetrain::DifferentialDesc tmpl;
    bool found = false;
    auto& comps = desc.powertrain.components;
    for (auto it = comps.begin(); it != comps.end(); ++it) {
        if (const auto* d = std::get_if<ps::drivetrain::DifferentialDesc>(&it->params)) {
            tmpl = *d;
            comps.erase(it);
            found = true;
            break;
        }
    }
    if (!found) throw std::runtime_error("car_sedan.json has no differential component");
    auto& shafts = desc.powertrain.shafts;
    std::vector<ps::drivetrain::ShaftDesc> kept;
    for (const auto& s : shafts) {
        bool drop = false;
        for (const auto& p : s.ports) drop = drop || p.is_wheel || p.component == "diff";
        if (!drop) kept.push_back(s);
    }
    shafts = std::move(kept);
    return tmpl;
}

inline ps::vehicle::VehicleDesc load_sedan_desc() {
    return ps::io::load_vehicle_json(std::string(PS_SOURCE_DIR) + "/data/vehicles/car_sedan.json");
}

// centre_efficiency: 1.0 (lossless centre, lambda_mesh_prev_ stays 0) or
// < 1 (the nested pair reconstruction then drives the hashed mesh loss).
inline ps::vehicle::VehicleDesc make_awd_desc(ps::real centre_efficiency = 1.0) {
    ps::vehicle::VehicleDesc desc = load_sedan_desc();
    const ps::drivetrain::DifferentialDesc axle = strip_sedan_diff(desc);
    ps::drivetrain::DifferentialDesc centre = axle;
    centre.ratio = 1.0;
    centre.efficiency = centre_efficiency;
    centre.carrier_inertia_kgm2 = 0.01;
    auto& comps = desc.powertrain.components;
    comps.push_back(ps::drivetrain::ComponentDesc{"centre", centre});
    comps.push_back(ps::drivetrain::ComponentDesc{"front", axle});
    comps.push_back(ps::drivetrain::ComponentDesc{"rear", axle});
    auto& shafts = desc.powertrain.shafts;
    shafts.push_back(shaft(comp_port("gearbox", "out"), comp_port("centre", "carrier")));
    shafts.push_back(shaft(comp_port("centre", "left"), comp_port("front", "carrier")));
    shafts.push_back(shaft(comp_port("centre", "right"), comp_port("rear", "carrier")));
    shafts.push_back(shaft(comp_port("front", "left"), wheel_port("FL")));
    shafts.push_back(shaft(comp_port("front", "right"), wheel_port("FR")));
    shafts.push_back(shaft(comp_port("rear", "left"), wheel_port("RL")));
    shafts.push_back(shaft(comp_port("rear", "right"), wheel_port("RR")));
    desc.name = "car_sedan_awd_test";
    return desc;
}

inline ps::vehicle::VehicleDesc make_parallel_4wd_desc() {
    ps::vehicle::VehicleDesc desc = load_sedan_desc();
    const ps::drivetrain::DifferentialDesc axle = strip_sedan_diff(desc);
    auto& comps = desc.powertrain.components;
    comps.push_back(ps::drivetrain::ComponentDesc{"front", axle});
    comps.push_back(ps::drivetrain::ComponentDesc{"rear", axle});
    auto& shafts = desc.powertrain.shafts;
    ps::drivetrain::ShaftDesc carrier_shaft;
    carrier_shaft.ports = {comp_port("gearbox", "out"), comp_port("front", "carrier"), comp_port("rear", "carrier")};
    shafts.push_back(carrier_shaft);
    shafts.push_back(shaft(comp_port("front", "left"), wheel_port("FL")));
    shafts.push_back(shaft(comp_port("front", "right"), wheel_port("FR")));
    shafts.push_back(shaft(comp_port("rear", "left"), wheel_port("RL")));
    shafts.push_back(shaft(comp_port("rear", "right"), wheel_port("RR")));
    desc.name = "car_sedan_parallel_4wd_test";
    return desc;
}

} // namespace ps_test
