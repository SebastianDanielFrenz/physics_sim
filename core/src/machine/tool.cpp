#include "ps/machine/tool.h"

#include "ps/machine/hitch_limits.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace ps::machine {

namespace {

[[noreturn]] void reject(const MachineDesc& m, const ToolDesc& t, const std::string& what) {
    throw std::invalid_argument("machine '" + m.name + "': tool '" + t.name + "': " + what);
}

void check_positive(const MachineDesc& m, const ToolDesc& t, real v, const char* name) {
    if (!std::isfinite(v) || !(v > 0.0)) reject(m, t, std::string(name) + " must be finite and > 0");
}

void check_not_negative(const MachineDesc& m, const ToolDesc& t, real v, const char* name) {
    if (!std::isfinite(v) || v < 0.0) reject(m, t, std::string(name) + " must be finite and >= 0");
}

void validate_efficiency(const MachineDesc& m, const ToolDesc& t) {
    for (const real e : {t.efficiency_draft, t.efficiency_shaft}) {
        if (!std::isfinite(e) || !(e > 0.0) || e > 2.0) reject(m, t, "efficiency must lie in (0, 2]");
    }
}

void validate_pto(const MachineDesc& m, const ToolDesc& t) {
    if (!t.pto.present) return;
    check_positive(m, t, t.pto.nominal_rpm, "pto.nominal_rpm");
    check_positive(m, t, t.pto.inertia_kgm2, "pto.inertia_kgm2");
    check_not_negative(m, t, t.pto.no_load_kw, "pto.no_load_kw");
    check_not_negative(m, t, t.pto.kw_per_m, "pto.kw_per_m");
    check_not_negative(m, t, t.pto.specific_energy_mj_m3, "pto.specific_energy_mj_m3");
    check_not_negative(m, t, t.pto.specific_energy_kj_kg, "pto.specific_energy_kj_kg");
    if (!(t.pto.min_speed_fraction > 0.0 && t.pto.min_speed_fraction < 1.0)) reject(m, t, "pto.min_speed_fraction must lie in (0, 1)");
}

} // namespace

void validate_machine(const MachineDesc& m) {
    if (m.name.empty()) throw std::invalid_argument("machine: a name is required");
    if (m.tools.empty()) throw std::invalid_argument("machine '" + m.name + "': at least one tool is required");
    for (std::size_t i = 0; i < m.tools.size(); ++i) {
        const ToolDesc& t = m.tools[i];
        for (std::size_t j = 0; j < i; ++j) {
            if (m.tools[j].name == t.name) reject(m, t, "the name is used twice");
        }
        if (t.operation.empty()) reject(m, t, "operation is required");
        check_positive(m, t, t.width_m, "the edge's width");
        if (t.kind == ToolKind::Applicator) {
            // An applicator touches nothing: speed and quality are all its range says.
            if (t.depth_min_m != 0.0 || t.depth_nominal_m != 0.0 || t.depth_max_m != 0.0) reject(m, t, "an applicator has no depth");
            check_positive(m, t, t.speed_min_km_h, "speed.min_km_h");
            if (!(t.speed_max_km_h > t.speed_min_km_h)) reject(m, t, "speed must keep min < max");
            check_not_negative(m, t, t.overspeed_quality_per_km_h, "quality.overspeed_loss_per_km_h");
            if (!(t.quality_floor >= 0.0 && t.quality_floor <= 1.0)) reject(m, t, "quality.floor must lie in [0, 1]");
            validate_efficiency(m, t);
            validate_pto(m, t);
            continue;
        }
        check_positive(m, t, t.depth_min_m, "depth.min_m");
        if (!(t.depth_nominal_m >= t.depth_min_m) || !(t.depth_max_m >= t.depth_nominal_m)) reject(m, t, "depth must keep min <= nominal <= max");
        check_positive(m, t, t.speed_min_km_h, "speed.min_km_h");
        if (!(t.speed_max_km_h > t.speed_min_km_h)) reject(m, t, "speed must keep min < max");
        check_not_negative(m, t, t.overspeed_quality_per_km_h, "quality.overspeed_loss_per_km_h");
        if (!(t.quality_floor >= 0.0 && t.quality_floor <= 1.0)) reject(m, t, "quality.floor must lie in [0, 1]");

        const ToolDraftDesc& d = t.draft;
        if (d.unit == DraftWidthUnit::Tool) check_positive(m, t, d.tool_spacing_m, "draft.tool_spacing_m");
        for (const real f : d.texture_factor) check_positive(m, t, f, "draft.texture_factor");
        if (d.moisture_factor.x.empty() || d.moisture_factor.x.size() != d.moisture_factor.y.size()) reject(m, t, "draft.moisture_factor needs x and y of the same length");
        for (std::size_t k = 1; k < d.moisture_factor.x.size(); ++k) {
            if (!(d.moisture_factor.x[k] > d.moisture_factor.x[k - 1])) reject(m, t, "draft.moisture_factor x must rise");
        }
        check_not_negative(m, t, d.compaction_gain, "draft.compaction_gain");
        check_not_negative(m, t, d.lateral_ratio, "draft.lateral_ratio");
        check_positive(m, t, d.creep_speed_m_s, "draft.creep_speed_m_s");

        const ToolSoilDesc& s = t.soil;
        check_not_negative(m, t, s.suction_ratio, "soil.suction_ratio");
        check_positive(m, t, s.bearing_n_m2, "soil.bearing_n_m2");
        check_not_negative(m, t, s.gauge_n_m2, "soil.gauge_n_m2");
        check_positive(m, t, s.hard_n_m2, "soil.hard_n_m2");
        check_not_negative(m, t, s.hard_friction, "soil.hard_friction");
        check_not_negative(m, t, s.damping_n_s_m2, "soil.damping_n_s_m2");

        validate_efficiency(m, t);
        validate_pto(m, t);
    }
}

real tool_draft_width(const ToolDesc& tool) {
    return tool.draft.unit == DraftWidthUnit::Tool ? tool.width_m / tool.draft.tool_spacing_m : tool.width_m;
}

real tool_draft_n(const ToolDesc& tool, const SoilSample& soil, real speed_km_h, real depth_m) {
    if (tool.kind == ToolKind::Applicator) return 0.0;
    const ToolDraftDesc& d = tool.draft;
    const real tilled = std::clamp(soil.tilled, real(0.0), real(1.0));
    const real a = d.primary.a + (d.secondary.a - d.primary.a) * tilled;
    const real b = d.primary.b + (d.secondary.b - d.primary.b) * tilled;
    const real c = d.primary.c + (d.secondary.c - d.primary.c) * tilled;
    const real s = std::abs(speed_km_h);
    const real depth_cm = std::clamp(depth_m, real(0.0), tool.depth_max_m) * 100.0;
    const real t = d.depth_term ? depth_cm : (depth_cm > 0.0 ? real(1.0) : real(0.0));
    const real texture = d.texture_factor[static_cast<std::size_t>(soil.texture)];
    const real moisture = d.moisture_factor.evaluate(soil.wetness);
    const real compaction = 1.0 + d.compaction_gain * std::clamp(soil.compaction, real(0.0), real(1.0));
    return tool.efficiency_draft * texture * (a + (b + c * s) * s) * tool_draft_width(tool) * t * moisture * compaction;
}

real tool_quality(const ToolDesc& tool, real speed_km_h) {
    const real over = std::abs(speed_km_h) - tool.speed_max_km_h;
    if (over <= 0.0) return 1.0;
    return std::max(tool.quality_floor, 1.0 - tool.overspeed_quality_per_km_h * over);
}

real tool_pto_power_w(const ToolDesc& tool, real speed_m_s, real depth_m, real material_kg_s) {
    if (!tool.pto.present) return 0.0;
    const real soil_m3_s = tool.width_m * std::max(depth_m, real(0.0)) * std::abs(speed_m_s);
    return tool.efficiency_shaft * (1e3 * (tool.pto.no_load_kw + tool.pto.kw_per_m * tool.width_m) + 1e6 * tool.pto.specific_energy_mj_m3 * soil_m3_s + 1e3 * tool.pto.specific_energy_kj_kg * std::max(material_kg_s, real(0.0)));
}

ToolPowerSweep tool_power_sweep(const ToolDesc& tool, const ToolPowerSpec& spec, int speed_steps, real material_kg_m2) {
    if (speed_steps < 2) throw std::invalid_argument("tool_power_sweep: at least two speeds");
    if (spec.soils.empty()) throw std::invalid_argument("tool_power_sweep: the spec has no soils");
    if (!(spec.tractive_ratio > 0.0) || !(spec.pto_share > 0.0)) throw std::invalid_argument("tool_power_sweep: the spec's efficiencies must be > 0");
    ToolPowerSweep out;
    out.points.reserve(static_cast<std::size_t>(speed_steps) * spec.soils.size());
    real worst = 0.0;
    for (int i = 0; i < speed_steps; ++i) {
        const real speed = tool.speed_min_km_h + (tool.speed_max_km_h - tool.speed_min_km_h) * static_cast<real>(i) / static_cast<real>(speed_steps - 1);
        for (const SoilSample& soil : spec.soils) {
            ToolPowerPoint p;
            p.speed_km_h = speed;
            p.soil = soil;
            p.draft_n = tool_draft_n(tool, soil, speed, tool.depth_nominal_m);
            p.drawbar_kw = p.draft_n * (speed / 3.6) * 1e-3;
            p.pto_kw = tool_pto_power_w(tool, speed / 3.6, tool.depth_nominal_m, material_kg_m2 * tool.width_m * speed / 3.6) * 1e-3;
            p.engine_kw = (p.drawbar_kw / spec.tractive_ratio + p.pto_kw) / spec.pto_share;
            worst = std::max(worst, p.engine_kw);
            out.points.push_back(p);
        }
    }
    out.needed_kw = worst * spec.reserve_factor;
    return out;
}

std::vector<drivetrain::LimitFinding> check_tool_limits(const ToolDesc& tool, const ToolPowerSweep& sweep, const ToolTractorFit& tractor, const drivetrain::LimitPolicy* policy) {
    using namespace drivetrain;
    const LimitPolicy& pol = resolve_limit_policy(policy);
    std::vector<LimitFinding> out;
    const auto grade = [&](LimitKind kind, real value, real rating, LimitUnit unit, const char* field) {
        if (rating <= 0.0) return;
        const real share = value / rating;
        LimitFinding f;
        if (share > pol.hitch_exceeded_share) {
            f.severity = LimitSeverity::Exceeded;
        } else if (share > pol.hitch_limiting_share) {
            f.severity = LimitSeverity::Limiting;
        } else if (share > pol.hitch_near_share) {
            f.severity = LimitSeverity::Near;
        } else {
            return;
        }
        f.kind = kind;
        f.value = value;
        f.limit = rating;
        f.unit = unit;
        f.part = PartRef{tool.name, "", field};
        out.push_back(f);
    };
    grade(LimitKind::ToolPower, sweep.needed_kw, tractor.engine_kw, LimitUnit::Kilowatt, "engine");
    if (tool.pto.present) {
        const real omega = tool.pto.nominal_rpm * 6.283185307179586476925286766559 / 60.0;
        real worst_kw = 0.0;
        for (const ToolPowerPoint& p : sweep.points) worst_kw = std::max(worst_kw, p.pto_kw);
        grade(LimitKind::PtoShaftTorque, worst_kw * 1e3 / omega, tractor.pto_torque_nm, LimitUnit::NewtonMetre, "pto");
    }
    return out;
}

} // namespace ps::machine
