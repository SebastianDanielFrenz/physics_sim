// core/src/environment/weather_desc.cpp - meaning checks of the W3 weather
// descriptors (ps/environment/weather_desc.h): the climate's regime /
// transition / model blocks and weather presets. Each check RETURNS the first
// problem as a string (the loaders prefix the file name without catching an
// exception object - clang-cl ASan caveat, CLAUDE.md).

#include "weather_validation.h"

#include "ps/environment/weather_desc.h"

#include <cmath>
#include <set>
#include <stdexcept>
#include <string>

namespace ps::environment {

namespace {

std::string err(const std::string& prefix, const std::string& what) { return prefix + ": " + what; }

bool finite(real v) { return std::isfinite(v); }

std::string lognormal_error(const LogNormalDesc& l, const std::string& p, const std::string& where) {
    if (!finite(l.median) || !(l.median > 0.0)) return err(p, where + ".median must be > 0 and finite");
    if (!finite(l.sigma_log) || l.sigma_log < 0.0 || l.sigma_log > 3.0)
        return err(p, where + ".sigma_log must be in [0, 3]");
    return {};
}

std::string range_error(const RangeDesc& r, const std::string& p, const std::string& where, real lo, real hi) {
    if (!finite(r.lo) || !finite(r.hi)) return err(p, where + " must be finite");
    if (r.lo > r.hi) return err(p, where + " must have lo <= hi");
    if (r.lo < lo || r.hi > hi)
        return err(p, where + " must lie in [" + std::to_string(lo) + ", " + std::to_string(hi) + "]");
    return {};
}

std::string regime_error(const RegimeDesc& r, const std::string& p, const std::string& w) {
    std::string e;
    if (!finite(r.duration.min_h) || r.duration.min_h < kMinSegmentDurationH)
        return err(p, w + ".duration_h.min must be >= 0.1 (h)");
    if (!finite(r.duration.median_h) || !(r.duration.median_h > 0.0))
        return err(p, w + ".duration_h.median must be > 0");
    if (!finite(r.duration.sigma_log) || r.duration.sigma_log < 0.0 || r.duration.sigma_log > 3.0)
        return err(p, w + ".duration_h.sigma_log must be in [0, 3]");
    if (!(e = range_error(r.cloud_cover, p, w + ".cloud_cover", 0.0, 1.0)).empty()) return e;
    if (r.precip_mm_h && !(e = lognormal_error(*r.precip_mm_h, p, w + ".precip_mm_h")).empty()) return e;
    if (r.convective) {
        const ConvectiveDesc& c = *r.convective;
        if (!(e = lognormal_error(c.cell_duration_min, p, w + ".convective.cell_duration_min")).empty()) return e;
        if (!(e = lognormal_error(c.gap_min, p, w + ".convective.gap_min")).empty()) return e;
        if (!(e = lognormal_error(c.peak_mm_h, p, w + ".convective.peak_mm_h")).empty()) return e;
        if (!finite(c.rise_fraction) || !(c.rise_fraction > 0.0) || !(c.rise_fraction < 1.0))
            return err(p, w + ".convective.rise_fraction must be in (0, 1)");
        if (!finite(c.gust_factor) || c.gust_factor < 1.0) return err(p, w + ".convective.gust_factor must be >= 1");
        if (c.hail) {
            const HailDesc& h = *c.hail;
            if (!finite(h.probability) || h.probability < 0.0 || h.probability > 1.0)
                return err(p, w + ".convective.hail.probability must be in [0, 1]");
            if (!(e = lognormal_error(h.diameter_mm, p, w + ".convective.hail.diameter_mm")).empty()) return e;
            if (!finite(h.min_shape) || !(h.min_shape > 0.0) || h.min_shape > 1.0)
                return err(p, w + ".convective.hail.min_shape must be in (0, 1]");
        }
    }
    if (!finite(r.wind_mean_m_s.k) || !(r.wind_mean_m_s.k > 0.0)) return err(p, w + ".wind_mean_m_s.weibull_k must be > 0");
    if (!finite(r.wind_mean_m_s.c) || r.wind_mean_m_s.c < 0.0) return err(p, w + ".wind_mean_m_s.weibull_c must be >= 0");
    if (!finite(r.wind_dir_deg.mean_deg) || r.wind_dir_deg.mean_deg < 0.0 || r.wind_dir_deg.mean_deg > 360.0)
        return err(p, w + ".wind_dir_deg.mean must be in [0, 360]");
    if (!finite(r.wind_dir_deg.spread_deg) || r.wind_dir_deg.spread_deg < 0.0 || r.wind_dir_deg.spread_deg > 180.0)
        return err(p, w + ".wind_dir_deg.spread must be in [0, 180]");
    if (!finite(r.gust_factor) || r.gust_factor < 1.0) return err(p, w + ".gust_factor must be >= 1");
    if (!(e = range_error(r.temp_offset_k, p, w + ".temp_offset_k", -50.0, 50.0)).empty()) return e;
    if (!(e = range_error(r.rh, p, w + ".rh", 0.0, 1.0)).empty()) return e;
    if (!(e = range_error(r.pressure_offset_hpa, p, w + ".pressure_offset_hpa", -200.0, 200.0)).empty()) return e;
    if (!finite(r.lightning_per_min) || r.lightning_per_min < 0.0 || r.lightning_per_min > kMaxLightningPerMin)
        return err(p, w + ".lightning_per_min must be in [0, 30]");
    return {};
}

} // namespace

namespace detail {

std::string climate_weather_validation_error(const ClimateDesc& d) {
    const std::string p = "climate";
    std::string e;
    bool any = false, all = true, any_lightning = false;
    for (const SeasonDesc& s : d.seasons) {
        if (s.weather) any = true; else all = false;
    }
    if (any && !all) return err(p, "regimes/transitions/initial_regime must be given on every season or on none");

    if (d.lags) {
        const WeatherLagsDesc& l = *d.lags;
        const real v[] = {l.cloud_s, l.precip_on_s, l.precip_off_s, l.wind_s, l.temp_offset_s, l.rh_s, l.pressure_s};
        for (real x : v)
            if (!finite(x) || !(x > 0.0)) return err(p, "lags_s: every time constant must be > 0 and finite");
    }
    if (d.precip_phase) {
        const PrecipPhaseDesc& ph = *d.precip_phase;
        if (!finite(ph.snow_below_c) || !finite(ph.rain_above_c)) return err(p, "precip_phase thresholds must be finite");
        if (!(ph.rain_above_c > ph.snow_below_c)) return err(p, "precip_phase.rain_above_c must be > snow_below_c");
        if (!finite(ph.min_rate_mm_h) || !(ph.min_rate_mm_h > 0.0)) return err(p, "precip_phase.min_rate_mm_h must be > 0");
    }
    if (d.humidity) {
        const HumidityDesc& h = *d.humidity;
        if (!finite(h.precip_rh_floor) || h.precip_rh_floor < 0.0 || h.precip_rh_floor > 1.0)
            return err(p, "humidity.precip_rh_floor must be in [0, 1]");
        if (!finite(h.precip_full_mm_h) || !(h.precip_full_mm_h > 0.0))
            return err(p, "humidity.precip_full_mm_h must be > 0");
    }
    if (d.diurnal_temperature) {
        const DiurnalTempDesc& t = *d.diurnal_temperature;
        if (!finite(t.a_h) || !(t.a_h > 0.0)) return err(p, "diurnal_temperature.a_h must be > 0");
        if (!finite(t.b) || !(t.b > 0.0)) return err(p, "diurnal_temperature.b must be > 0");
        if (!finite(t.c_h)) return err(p, "diurnal_temperature.c_h must be finite");
        if (!(t.a_h > -0.5 * t.c_h))
            return err(p, "diurnal_temperature: a_h must be > -c_h/2 (the day sine must still be positive at sunset)");
        if (!finite(t.reference_cycle_s) || !(t.reference_cycle_s > 0.0))
            return err(p, "diurnal_temperature.reference_cycle_s must be > 0");
        if (!finite(t.cloud_range_reduction) || t.cloud_range_reduction < 0.0 || t.cloud_range_reduction > 1.0)
            return err(p, "diurnal_temperature.cloud_range_reduction must be in [0, 1]");
    }
    if (d.lightning) {
        const LightningDesc& l = *d.lightning;
        if (!finite(l.distance_min_km) || l.distance_min_km < 0.0) return err(p, "lightning.distance_km min must be >= 0");
        if (!finite(l.distance_max_km) || !(l.distance_max_km > 0.0) || l.distance_max_km < l.distance_min_km)
            return err(p, "lightning.distance_km must be [min, max] with max > 0 and max >= min");
    }

    if (d.gusts) {
        const std::string ge = gust_desc_validation_error(*d.gusts);
        if (!ge.empty()) return err(p, ge);
    }

    if (!any) return {};

    if (!d.lags) return err(p, "lags_s is required when the seasons carry regimes");
    if (!d.precip_phase) return err(p, "precip_phase is required when the seasons carry regimes");
    if (!d.humidity) return err(p, "humidity is required when the seasons carry regimes");
    if (!d.diurnal_temperature) return err(p, "diurnal_temperature is required when the seasons carry regimes");

    for (std::size_t i = 0; i < d.seasons.size(); ++i) {
        const SeasonDesc& s = d.seasons[i];
        const std::string ws = "seasons[" + std::to_string(i) + "]";
        if (!s.climate) return err(p, ws + " carries regimes but no climate scalars (temp_mean_c ...)");
        const SeasonWeatherDesc& w = *s.weather;
        if (w.regimes.empty()) return err(p, ws + ".regimes must not be empty");
        std::set<std::string> names;
        for (const NamedRegime& nr : w.regimes) {
            if (nr.name.empty()) return err(p, ws + ".regimes has an empty name");
            if (!names.insert(nr.name).second) return err(p, ws + ".regimes." + nr.name + " is defined twice");
            if (!(e = regime_error(nr.regime, p, ws + ".regimes." + nr.name)).empty()) return e;
            if (nr.regime.lightning_per_min > 0.0) any_lightning = true;
            if (nr.regime.precip_form == PrecipForm::FreezingRain && s.climate->temp_mean_c > 5.0)
                return err(p, ws + ".regimes." + nr.name +
                                  ": precip_form freezing_rain in a season with temp_mean_c above +5 degC");
        }
        std::set<std::string> rows;
        for (const TransitionRowDesc& row : w.transitions) {
            const std::string wr = ws + ".transitions." + row.from;
            if (!names.count(row.from)) return err(p, wr + ": \"" + row.from + "\" is not a regime of this season");
            if (!rows.insert(row.from).second) return err(p, wr + " is given twice");
            if (row.to.empty()) return err(p, wr + " must not be empty");
            std::set<std::string> tos;
            real sum = 0.0;
            for (const TransitionDesc& t : row.to) {
                if (!names.count(t.to)) return err(p, wr + " names \"" + t.to + "\", not a regime of this season");
                if (!tos.insert(t.to).second) return err(p, wr + " names \"" + t.to + "\" twice");
                if (!finite(t.p) || t.p < 0.0 || t.p > 1.0) return err(p, wr + "." + t.to + " must be in [0, 1]");
                sum += t.p;
            }
            if (std::fabs(sum - 1.0) > 1e-9) return err(p, wr + " must sum to 1 (within 1e-9)");
        }
        for (const std::string& n : names)
            if (!rows.count(n)) return err(p, ws + ".transitions has no row for regime \"" + n + "\"");
        if (!names.count(w.initial_regime))
            return err(p, ws + ".initial_regime \"" + w.initial_regime + "\" is not a regime of this season");
    }
    if (any_lightning && !d.lightning) return err(p, "lightning is required when a regime has lightning_per_min > 0");
    return {};
}

} // namespace detail

std::string gust_desc_validation_error(const GustDesc& g) {
    if (!finite(g.roughness_length_m) || !(g.roughness_length_m > 0.0) || g.roughness_length_m > 10.0)
        return "gusts.roughness_length_m must be in (0, 10] m";
    if (!finite(g.reference_height_m) || !(g.reference_height_m > 2.0 * g.roughness_length_m) || g.reference_height_m > 60.0)
        return "gusts.reference_height_m must be in (2 z0, 60] m (the Kaimal length scale 0.7 z holds to 60 m)";
    if (!finite(g.reference_gust_factor) || !(g.reference_gust_factor > 1.0) || g.reference_gust_factor > 4.0)
        return "gusts.reference_gust_factor must be in (1, 4]";
    if (!finite(g.f_min_hz) || !(g.f_min_hz >= 1e-5) || g.f_min_hz > 1.0)
        return "gusts.f_min_hz must be in [1e-5, 1] Hz";
    if (g.bands_per_octave < 2 || g.bands_per_octave > 8)
        return "gusts.bands_per_octave must be in [2, 8] (32 components)";
    if (!finite(g.window_s) || g.window_s < 60.0 || g.window_s > 86400.0)
        return "gusts.window_s must be in [60, 86400] s";
    if (!finite(g.blend_s) || g.blend_s < 0.0 || g.blend_s > 0.5 * g.window_s)
        return "gusts.blend_s must be in [0, window_s / 2]";
    if (!finite(g.min_advection_speed_m_s) || !(g.min_advection_speed_m_s > 0.0) || g.min_advection_speed_m_s > 10.0)
        return "gusts.min_advection_speed_m_s must be in (0, 10] m/s";
    return {};
}

std::string weather_preset_validation_error(const WeatherPresetDesc& w) {
    const std::string p = "weather_preset";
    if (!finite(w.cloud_cover) || w.cloud_cover < 0.0 || w.cloud_cover > 1.0) return err(p, "cloud_cover must be in [0, 1]");
    if (!finite(w.precip_mm_h) || w.precip_mm_h < 0.0) return err(p, "precip_mm_h must be >= 0");
    if (w.precip_type == PresetPrecipType::Hail) {
        if (!finite(w.hail_diameter_mm) || !(w.hail_diameter_mm > 0.0))
            return err(p, "hail_diameter_mm must be > 0 for precip_type hail");
        if (!(w.precip_mm_h > 0.0)) return err(p, "precip_type hail needs precip_mm_h > 0");
    } else if (w.hail_diameter_mm != 0.0) {
        return err(p, "hail_diameter_mm is only allowed with precip_type hail");
    }
    if (!finite(w.wind_mean_m_s) || w.wind_mean_m_s < 0.0) return err(p, "wind_mean_m_s must be >= 0");
    if (!finite(w.wind_dir_deg) || w.wind_dir_deg < 0.0 || w.wind_dir_deg > 360.0) return err(p, "wind_dir_deg must be in [0, 360]");
    if (!finite(w.gust_factor) || w.gust_factor < 1.0) return err(p, "gust_factor must be >= 1");
    if (!finite(w.temp_offset_k) || w.temp_offset_k < -50.0 || w.temp_offset_k > 50.0)
        return err(p, "temp_offset_k must be in [-50, 50]");
    if (!finite(w.temp_override_c) || w.temp_override_c < -90.0 || w.temp_override_c > 60.0)
        return err(p, "temp_override_c must be in [-90, 60]");
    if (!finite(w.rh) || w.rh < 0.0 || w.rh > 1.0) return err(p, "rh must be in [0, 1]");
    if (!finite(w.pressure_offset_hpa) || w.pressure_offset_hpa < -200.0 || w.pressure_offset_hpa > 200.0)
        return err(p, "pressure_offset_hpa must be in [-200, 200]");
    if (!finite(w.lightning_per_min) || w.lightning_per_min < 0.0 || w.lightning_per_min > kMaxLightningPerMin)
        return err(p, "lightning_per_min must be in [0, 30]");
    return {};
}

void validate_weather_preset(const WeatherPresetDesc& p) {
    const std::string e = weather_preset_validation_error(p);
    if (!e.empty()) throw std::invalid_argument(e);
}

} // namespace ps::environment
