#include "ps/io/chassis_hull_io.h"

#include <nlohmann/json.hpp>

#include <cmath>
#include <fstream>
#include <stdexcept>

namespace ps::io {
namespace {

using json = nlohmann::json;

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

} // namespace

ConvexCompoundShape load_chassis_hull_json(const std::string& path) {
    const json j = read_json_file(path);
    const std::string expected = "physics_sim.chassis_hull/1";
    if (!j.contains("format") || !j.at("format").is_string() || j.at("format").get<std::string>() != expected) {
        throw std::runtime_error(path + ": 'format' must be \"" + expected + "\"");
    }
    if (!j.contains("source") || !j.at("source").is_string() || j.at("source").get<std::string>().empty()) {
        throw std::runtime_error(path + ": 'source' is required (PLAN.md section 8)");
    }
    const real radius = j.value("convex_radius_m", real(0.05));
    if (!(radius >= real(0.0)) || !std::isfinite(radius)) throw std::runtime_error(path + ": convex_radius_m must be >= 0");
    if (!j.contains("pieces") || !j.at("pieces").is_array() || j.at("pieces").empty()) {
        throw std::runtime_error(path + ": 'pieces' must be a non-empty array");
    }

    ConvexCompoundShape shape;
    for (const json& piece : j.at("pieces")) {
        if (!piece.is_array() || piece.size() < 4) throw std::runtime_error(path + ": every piece needs at least 4 points");
        ConvexHullShape hull;
        hull.max_convex_radius = radius;
        hull.points.reserve(piece.size());
        for (const json& p : piece) {
            if (!p.is_array() || p.size() != 3) throw std::runtime_error(path + ": a point is [x, y, z]");
            const Vec3 v{p[0].get<real>(), p[1].get<real>(), p[2].get<real>()};
            if (!std::isfinite(v.x) || !std::isfinite(v.y) || !std::isfinite(v.z)) {
                throw std::runtime_error(path + ": non-finite point");
            }
            hull.points.push_back(v);
        }
        shape.pieces.push_back(std::move(hull));
    }
    return shape;
}

} // namespace ps::io
