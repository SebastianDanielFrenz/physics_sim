// core/src/environment/air_temperature.cpp - see ps/environment/air_temperature.h.

#include "ps/environment/air_temperature.h"

#include "ps/environment/climate_desc.h"
#include "ps/math/transcendental.h"

#include <stdexcept>

namespace ps::environment {

namespace {

struct DayShape {
    bool degenerate = false;
    real start = 0.0;   // env time of the day's midnight start
    real t_n = 0.0;     // time of the minimum
    real t_s = 0.0;     // sunset
    real y = 0.0;       // day length
    real a = 0.0;       // scaled lag of the maximum
    real t_min = 0.0;   // T_n
    real t_max = 0.0;   // T_x
    real mean = 0.0;
};

real clampr(real v, real lo, real hi) { return v < lo ? lo : (v > hi ? hi : v); }

DayShape shape_of(const DayValues& dv, real start, const DiurnalTempDesc& p, real k) {
    if (!dv.has_climate) throw std::invalid_argument("parton_logan_air_temp_c: the calendar has no climate scalars");
    DayShape s;
    s.start = start;
    s.mean = dv.climate.temp_mean_c;
    const real half = 0.5 * k * dv.climate.temp_range_k;
    s.t_min = s.mean - half;
    s.t_max = s.mean + half;
    s.degenerate = !(dv.day_s > 0.0) || !(dv.night_s > 0.0);
    if (s.degenerate) return s;
    const real scale = dv.length_s() / p.reference_cycle_s;
    s.a = p.a_h * 3600.0 * scale;
    const real c = clampr(p.c_h * 3600.0 * scale, -0.25 * dv.night_s, 0.25 * dv.day_s);
    const real sunrise = start + 0.5 * dv.night_s;
    s.t_n = sunrise + c;
    s.t_s = sunrise + dv.day_s;
    s.y = dv.day_s;
    return s;
}

real day_formula(const DayShape& d, real t) {
    return d.t_min + (d.t_max - d.t_min) * ps::math::sin(environment::kPi * (t - d.t_n) / (d.y + 2.0 * d.a));
}

real night_formula(const DayShape& d, const DayShape& next, real b, real t) {
    const real t_s = day_formula(d, d.t_s);
    const real zn = next.t_n - d.t_s;
    const real eb = ps::math::exp(-b);
    const real en = ps::math::exp(-b * (t - d.t_s) / zn);
    return (next.t_min - t_s * eb + (t_s - next.t_min) * en) / (1.0 - eb);
}

} // namespace

real parton_logan_air_temp_c(const Calendar& cal, const DiurnalTempDesc& p, real t, real cloud_cover) {
    const CalendarSample s = cal.sample(t);
    const int n = cal.days_per_year();
    const int d = s.day_of_year;
    const real k = 1.0 - p.cloud_range_reduction * clampr(cloud_cover, 0.0, 1.0);
    const real start = t - s.time_of_day_s;
    const DayValues& dv = cal.day(d);
    const DayShape cur = shape_of(dv, start, p, k);
    if (cur.degenerate) return cur.mean;

    if (t < cur.t_n) {
        const DayValues& pv = cal.day((d - 1 + n) % n);
        const DayShape prev = shape_of(pv, start - pv.length_s(), p, k);
        if (prev.degenerate) return cur.mean;
        return night_formula(prev, cur, p.b, t);
    }
    if (t < cur.t_s) return day_formula(cur, t);
    const DayValues& nv = cal.day((d + 1) % n);
    const DayShape next = shape_of(nv, start + dv.length_s(), p, k);
    if (next.degenerate) return cur.mean;
    return night_formula(cur, next, p.b, t);
}

DiurnalTimes parton_logan_times(const Calendar& cal, const DiurnalTempDesc& p, real t) {
    const CalendarSample s = cal.sample(t);
    const real start = t - s.time_of_day_s;
    const DayValues& dv = cal.day(s.day_of_year);
    const DayShape cur = shape_of(dv, start, p, 1.0);
    DiurnalTimes r;
    r.sunrise = start + 0.5 * dv.night_s;
    r.sunset = cur.t_s;
    r.t_min = cur.t_n;
    r.t_max = cur.t_n + 0.5 * (cur.y + 2.0 * cur.a);
    return r;
}

real saturation_vapour_pressure_hpa(real temp_c) {
    return 6.1094 * ps::math::exp(17.625 * temp_c / (temp_c + 243.04));
}

real dew_point_c(real temp_c, real rh) {
    const real r = rh < 1e-6 ? 1e-6 : (rh > 1.0 ? 1.0 : rh);
    const real g = ps::math::log(r) + 17.625 * temp_c / (temp_c + 243.04);
    return 243.04 * g / (17.625 - g);
}

} // namespace ps::environment
