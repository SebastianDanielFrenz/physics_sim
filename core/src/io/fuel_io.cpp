#include "ps/io/fuel_io.h"

#include <nlohmann/json.hpp>

#include <cmath>
#include <fstream>
#include <stdexcept>

namespace ps::io {
namespace {

using json = nlohmann::json;
using ps::drivetrain::FuelDesc;
using ps::drivetrain::FuelPhase;

json read_json_file(const std::string& path) {
    std::ifstream in(path);
    if (!in) throw std::runtime_error("cannot open file: " + path);
    json j;
    try {
        in >> j;
    } catch (const nlohmann::json::parse_error& e) {
        throw std::runtime_error("malformed JSON in " + path + ": " + e.what());
    }
    return j;
}

// PHYS-033 P4 I3 plan section 9.1 (owner ruling Q7: NO legacy support -
// missing, unknown or outdated is a hard error, never a silent default).
// Duplicated from vehicle_io.cpp's own require_format rather than shared
// across a translation-unit boundary (each io/*.cpp is self-contained -
// see vehicle_io.cpp's require_vehicle_format comment for why a shared
// try/catch-based helper is avoided under the asan preset).
void require_format(const json& j, const std::string& kind, int major, const std::string& path) {
    const std::string expected = "physics_sim." + kind + "/" + std::to_string(major);
    if (!j.contains("format") || !j.at("format").is_string()) {
        throw std::runtime_error(path + ": missing required 'format' field (expected \"" + expected + "\")");
    }
    const std::string actual = j.at("format").get<std::string>();
    if (actual != expected) {
        throw std::runtime_error(path + ": unsupported 'format' \"" + actual + "\" (expected \"" + expected + "\")");
    }
}

FuelDesc read_fuel_object(const json& j) {
    FuelDesc d;
    d.name = j.value("name", std::string());
    if (d.name.empty()) throw std::runtime_error("fuel: 'name' is required and must be non-empty");

    const std::string phase = j.value("phase", std::string());
    if (phase != "liquid") {
        throw std::runtime_error("fuel '" + d.name + "': phase \"" + phase +
                                  "\" is not yet supported (only \"liquid\" is implemented)");
    }
    d.phase = FuelPhase::Liquid;

    d.lhv_j_kg = j.value("lhv_j_kg", d.lhv_j_kg);
    if (!(d.lhv_j_kg > real(0.0)) || !std::isfinite(d.lhv_j_kg)) {
        throw std::runtime_error("fuel '" + d.name + "': lhv_j_kg must be > 0 and finite, got " +
                                  std::to_string(d.lhv_j_kg));
    }

    d.afr_stoich = j.value("afr_stoich", d.afr_stoich);
    if (!(d.afr_stoich > real(0.0)) || !std::isfinite(d.afr_stoich)) {
        throw std::runtime_error("fuel '" + d.name + "': afr_stoich must be > 0 and finite, got " +
                                  std::to_string(d.afr_stoich));
    }

    d.density_kg_m3 = j.value("density_kg_m3", d.density_kg_m3);
    if (!(d.density_kg_m3 > real(0.0)) || !std::isfinite(d.density_kg_m3)) {
        throw std::runtime_error("fuel '" + d.name + "': density_kg_m3 must be > 0 and finite, got " +
                                  std::to_string(d.density_kg_m3));
    }

    d.source = j.value("source", std::string());
    if (d.source.empty()) throw std::runtime_error("fuel '" + d.name + "': 'source' is required (PLAN.md section 8)");

    return d;
}

} // namespace

FuelDesc load_fuel_json(const std::string& path) {
    const json j = read_json_file(path);
    require_format(j, "fuel", 1, path);
    return read_fuel_object(j);
}

} // namespace ps::io
