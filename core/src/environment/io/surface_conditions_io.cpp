// core/src/environment/io/surface_conditions_io.cpp - see
// ps/environment/surface_conditions_io.h and docs/data_formats/surface_conditions.md.

#include "ps/environment/surface_conditions_io.h"

#include "ps/environment/json_extends.h"
#include "object_reader.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <stdexcept>
#include <string>

namespace ps::io {

using nlohmann::json;
using namespace ps::environment;
using ps::io::detail::ObjectReader;

namespace {

constexpr const char* kFormat = "physics_sim.surface_conditions/1";

std::string source_text(const json* src) {
    if (!src) return {};
    return src->is_string() ? src->get<std::string>() : src->dump();
}

ThermalLayerDesc read_layer(ObjectReader& parent, const char* key, const std::string& ctx, const std::string& origin) {
    const json& j = parent.object(key);
    ObjectReader r(j, ctx + "." + key, origin);
    ThermalLayerDesc l;
    l.depth_m = r.number("depth_m", true, 0.0);
    l.k_w_mk = r.number("k_w_mk", true, 0.0);
    l.rho_c_j_m3k = r.number("rho_c_j_m3k", true, 0.0);
    r.skip("source");
    r.finish();
    return l;
}

SoilDesc read_soil(ObjectReader& parent, const std::string& ctx, const std::string& origin) {
    const json& j = parent.object("soil");
    ObjectReader r(j, ctx + ".soil", origin);
    SoilDesc s;
    s.ksat_mm_h = r.number("ksat_mm_h", true, 0.0);
    s.theta_fc = r.number("theta_fc", true, 0.0);
    s.theta_sat = r.number("theta_sat", true, 0.0);
    r.skip("source");
    r.finish();
    return s;
}

SurfaceFrictionDesc read_friction(ObjectReader& parent, const std::string& ctx, const std::string& origin) {
    const json& j = parent.object("friction");
    ObjectReader r(j, ctx + ".friction", origin);
    SurfaceFrictionDesc f;
    f.damp = r.number("damp", false, 0.85);
    f.wet = r.number("wet", false, 0.71);
    f.wet_grass = r.number("wet_grass", false, 0.67);
    // W8/W9 conditions: a clear error, not a silent drop.
    for (const char* k : {"slush", "snow_loose", "snow_packed", "ice", "wet_ice", "frost", "black_ice", "mud"})
        r.reserve_for_later(k, "W8/W9 (snow, ice, mud grip)", kFormat);
    r.skip("source");
    r.finish();
    return f;
}

SurfaceClassDesc read_class(const std::string& name, const json& j, const std::string& origin) {
    const std::string ctx = "surfaces." + name;
    ObjectReader r(j, ctx, origin);
    SurfaceClassDesc c;
    c.name = name;
    const std::string kind = r.string("kind", true, "");
    if (kind == "static") c.kind = SurfaceKind::Static;
    else if (kind == "sealed") c.kind = SurfaceKind::Sealed;
    else if (kind == "soil") c.kind = SurfaceKind::Soil;
    else if (kind == "vegetated") c.kind = SurfaceKind::Vegetated;
    else r.fail("\"kind\" must be \"sealed\", \"soil\", \"vegetated\" or \"static\" (got \"" + kind + "\")");

    // Keys of later increments (weather plan 7.4): a clear error, not a silent drop.
    r.reserve_for_later("crr", "W8 (condition rolling resistance)", kFormat);
    r.reserve_for_later("abrasiveness", "W10 (tyre wear)", kFormat);
    r.reserve_for_later("maintenance", "a later increment (winter maintenance)", kFormat);

    if (c.kind == SurfaceKind::Static) {
        for (const char* k : {"albedo", "emissivity", "layer", "deep", "mtd_mm", "texture_storage_mm", "drainage",
                              "soil", "canopy", "friction"})
            if (r.has(k)) r.fail(std::string("a static class takes no field (\"") + k + "\")");
    } else {
        c.albedo = r.number("albedo", true, 0.0);
        c.emissivity = r.number("emissivity", true, 0.0);
        c.layer = read_layer(r, "layer", ctx, origin);
        c.deep = read_layer(r, "deep", ctx, origin);
        if (c.kind == SurfaceKind::Sealed) {
            c.mtd_mm = r.number("mtd_mm", true, 0.0);
            c.texture_storage_mm = r.number("texture_storage_mm", false, -1.0);
            const json& dj = r.object("drainage");
            ObjectReader dr(dj, ctx + ".drainage", origin);
            c.drainage_path_m = dr.number("path_m", true, 0.0);
            c.drainage_slope_m_m = dr.number("slope", true, 0.0);
            dr.skip("source");
            dr.finish();
        } else {
            c.soil = read_soil(r, ctx, origin);
            if (c.kind == SurfaceKind::Vegetated) {
                const json& cj = r.object("canopy");
                ObjectReader cr(cj, ctx + ".canopy", origin);
                c.canopy_capacity_mm = cr.number("capacity_mm", true, 0.0);
                cr.skip("source");
                cr.finish();
            }
        }
        if (r.has("friction")) c.friction = read_friction(r, ctx, origin);
    }
    c.source = source_text(r.child("source"));
    r.finish();
    return c;
}

} // namespace

SurfaceConditionsDesc parse_surface_conditions_json(const json& root, const std::string& origin) {
    ObjectReader r(root, "", origin);
    const std::string format = r.string("format", true, "");
    if (format != kFormat) r.fail(std::string("\"format\" must be \"") + kFormat + "\" (got \"" + format + "\")");
    r.skip("extends");
    SurfaceConditionsDesc d;
    const json& surfaces = r.object("surfaces");
    // nlohmann's default object is a std::map: iteration is in name order,
    // which is the order the desc requires.
    for (auto it = surfaces.begin(); it != surfaces.end(); ++it) {
        if (it.value().is_null()) continue;   // removed by an `extends` override (RFC 7396)
        if (it.key().empty()) r.fail("a surface name is empty");
        d.classes.push_back(read_class(it.key(), it.value(), origin));
    }
    if (d.classes.empty()) r.fail("\"surfaces\" must name at least one surface");
    d.source = source_text(r.child("source"));
    r.finish();
    const std::string problem = surface_conditions_validation_error(d);
    if (!problem.empty()) throw std::runtime_error(origin + ": " + problem);
    return d;
}

SurfaceConditionsDesc load_surface_conditions_json(const std::filesystem::path& path) {
    const json root = load_json_with_extends(path);
    return parse_surface_conditions_json(root, path.string());
}

} // namespace ps::io
