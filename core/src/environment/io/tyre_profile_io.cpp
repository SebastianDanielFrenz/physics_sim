// core/src/environment/io/tyre_profile_io.cpp - see
// ps/environment/tyre_profile_io.h and docs/data_formats/tyre_profile.md.

#include "ps/environment/tyre_profile_io.h"

#include "ps/environment/json_extends.h"
#include "object_reader.h"

#include <nlohmann/json.hpp>

#include <stdexcept>
#include <string>

namespace ps::io {

using nlohmann::json;
using namespace ps::environment;
using ps::io::detail::ObjectReader;

namespace {

constexpr const char* kFormat = "physics_sim.tyre_profile/1";

std::string source_text(const json* src) {
    if (!src) return {};
    return src->is_string() ? src->get<std::string>() : src->dump();
}

// W9/W10 blocks are accepted as null (the plan's `| null`), an object is
// reserved until its increment.
void reserve_block_or_null(ObjectReader& r, const char* key, const char* increment) {
    const json* c = r.child(key);
    if (c && !c->is_null())
        r.fail(std::string("\"") + key + "\" is part of " + kFormat + " but is not implemented before " + increment +
               "; use null");
}

} // namespace

TyreProfileDesc parse_tyre_profile_json(const json& root, const std::string& origin) {
    ObjectReader r(root, "", origin);
    const std::string format = r.string("format", true, "");
    if (format != kFormat) r.fail(std::string("\"format\" must be \"") + kFormat + "\" (got \"" + format + "\")");
    r.skip("extends");
    TyreProfileDesc d;
    const std::string cls = r.string("class", true, "");
    if (!parse_tyre_class(cls, d.tyre_class))
        r.fail("\"class\" must be one of slick, summer, all_season, winter, winter_studded, offroad_at, mud_terrain, "
               "agricultural_lug (got \"" + cls + "\")");
    d.tread_new_mm = r.number("tread_new_mm", true, 0.0);
    d.tread_min_mm = r.number("tread_min_mm", true, 0.0);
    d.groove_void_ratio = r.number("groove_void_ratio", true, 0.0);
    d.siping = r.string("siping", false, "none");
    d.inflation_pressure_kpa = r.number("inflation_pressure_kpa", true, 0.0);

    {
        const json& cf = r.object("condition_factors");
        ObjectReader fr(cf, "condition_factors", origin);
        d.condition_factors.dry = fr.number("dry", false, 1.0);
        d.condition_factors.damp = fr.number("damp", false, 1.0);
        d.condition_factors.wet = fr.number("wet", false, 1.0);
        d.condition_factors.wet_grass = fr.number("wet_grass", false, 1.0);
        for (const char* k : {"slush", "snow_loose", "snow_packed", "ice", "wet_ice", "frost", "black_ice", "mud"})
            fr.reserve_for_later(k, "W8/W9 (snow, ice, mud grip)", kFormat);
        fr.skip("source");
        fr.finish();
    }
    {
        const json& te = r.object("tread_exponents");
        ObjectReader tr(te, "tread_exponents", origin);
        d.tread_exponents.wet = tr.number("wet", false, 0.5);
        d.tread_exponents.snow_packed = tr.number("snow_packed", false, 1.0);
        tr.skip("source");
        tr.finish();
    }
    reserve_block_or_null(r, "thermal", "W9 (tyre thermal)");
    reserve_block_or_null(r, "wear", "W10 (tyre wear)");
    d.source = source_text(r.child("source"));
    r.finish();

    const std::string problem = tyre_profile_validation_error(d);
    if (!problem.empty()) throw std::runtime_error(origin + ": " + problem);
    return d;
}

TyreProfileDesc load_tyre_profile_json(const std::filesystem::path& path) {
    const json root = load_json_with_extends(path);
    TyreProfileDesc d = parse_tyre_profile_json(root, path.string());
    d.name = path.stem().string();
    return d;
}

} // namespace ps::io
