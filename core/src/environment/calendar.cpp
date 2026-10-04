// core/src/environment/calendar.cpp - see ps/environment/calendar.h for the model.

#include "ps/environment/calendar.h"

#include "ps/environment/solar_geometry.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace ps::environment {

namespace {

// Length in days of a season's real day-of-year span (1-based inclusive;
// start > end wraps over the new year, a year counted as 365 days).
real span_length_days(const SeasonDesc& s) {
    if (s.doy_end >= s.doy_start) return static_cast<real>(s.doy_end - s.doy_start + 1);
    return static_cast<real>(365 - s.doy_start + 1 + s.doy_end);
}

struct ResolvedAuthored {
    bool valid = false;
    SunAuthoredDesc value;
};

ResolvedAuthored resolve_authored(const ClimateDesc& d, const SeasonDesc& s) {
    ResolvedAuthored r;
    SunAuthoredDesc a;
    bool have[4] = {false, false, false, false};
    if (d.sun_authored) {
        a = *d.sun_authored;
        have[0] = have[1] = have[2] = have[3] = true;
    }
    if (s.sun_authored.e_max_deg) { a.e_max_deg = *s.sun_authored.e_max_deg; have[0] = true; }
    if (s.sun_authored.e_night_deg) { a.e_night_deg = *s.sun_authored.e_night_deg; have[1] = true; }
    if (s.sun_authored.az_rise_deg) { a.az_rise_deg = *s.sun_authored.az_rise_deg; have[2] = true; }
    if (s.sun_authored.az_set_deg) { a.az_set_deg = *s.sun_authored.az_set_deg; have[3] = true; }
    r.valid = have[0] && have[1] && have[2] && have[3];
    r.value = a;
    return r;
}

// a + (b - a) w: exact for w = 0 and for a == b (the form (1 - w) a + w b is
// not, and a constant parameter must stay constant to the last bit).
real lerp(real a, real b, real w) { return a + (b - a) * w; }

} // namespace

Calendar::Calendar(const ClimateDesc& desc, CalendarOptions options) {
    validate_climate(desc);

    const int n = static_cast<int>(desc.seasons.size());
    season_names_.reserve(static_cast<std::size_t>(n));
    season_start_.reserve(static_cast<std::size_t>(n));
    season_days_.reserve(static_cast<std::size_t>(n));
    int total = 0;
    for (const SeasonDesc& s : desc.seasons) {
        season_names_.push_back(s.name);
        season_start_.push_back(total);
        season_days_.push_back(s.days);
        total += s.days;
    }

    if (options.mode == CalendarMode::FixedSeason && (options.fixed_season < 0 || options.fixed_season >= n))
        throw std::invalid_argument("Calendar: fixed_season out of range");
    if (options.mode == CalendarMode::FixedDay && (options.fixed_day < 0 || options.fixed_day >= total))
        throw std::invalid_argument("Calendar: fixed_day out of range");

    const real lat_rad = desc.location.latitude_deg * kDegToRad;
    const real cycle_s = desc.calendar.cycle_s;

    std::vector<ResolvedAuthored> authored(static_cast<std::size_t>(n));
    std::vector<real> mid(static_cast<std::size_t>(n));
    for (int i = 0; i < n; ++i) {
        authored[static_cast<std::size_t>(i)] = resolve_authored(desc, desc.seasons[static_cast<std::size_t>(i)]);
        // Season midpoint in day-CENTRE coordinates (day d has centre d + 0.5).
        mid[static_cast<std::size_t>(i)] = static_cast<real>(season_start_[static_cast<std::size_t>(i)]) +
                                           static_cast<real>(desc.seasons[static_cast<std::size_t>(i)].days) / 2.0;
    }

    // Real day of year (continuous, days since 1 Jan 00:00) at day coordinate x
    // inside season s.
    auto y_of = [&](int s, real x) -> real {
        const SeasonDesc& sd = desc.seasons[static_cast<std::size_t>(s)];
        if (!sd.has_doy_span) return 0.0;
        return static_cast<real>(sd.doy_start - 1) +
               (x - static_cast<real>(season_start_[static_cast<std::size_t>(s)])) /
                   static_cast<real>(sd.days) * span_length_days(sd);
    };
    auto y_mid_of = [&](int s) -> real {
        const SeasonDesc& sd = desc.seasons[static_cast<std::size_t>(s)];
        if (!sd.has_doy_span) return 0.0;
        return static_cast<real>(sd.doy_start - 1) + span_length_days(sd) / 2.0;
    };
    // (day_s, night_s) of season s's day/night form at real day of year y_noon.
    auto day_night_of = [&](int s, real y_noon) -> std::pair<real, real> {
        const DayNightDesc& dn = desc.seasons[static_cast<std::size_t>(s)].day_night;
        switch (dn.kind) {
        case DayNightKind::Explicit:
            return {dn.day_s, dn.night_s};
        case DayNightKind::CycleFraction:
            return {dn.cycle_s * dn.day_fraction, dn.cycle_s * (1.0 - dn.day_fraction)};
        case DayNightKind::Astronomical:
            break;
        }
        const real h0 = sunrise_hour_angle_rad(lat_rad, declination_spencer_rad(y_noon));
        const real day = cycle_s * (h0 / kPi);
        return {day, cycle_s - day};
    };

    days_.assign(static_cast<std::size_t>(total), DayValues{});

    // ---- Dynamic table: every game day interpolated between season midpoints.
    int day_index = 0;
    for (int s = 0; s < n; ++s) {
        for (int k = 0; k < season_days_[static_cast<std::size_t>(s)]; ++k, ++day_index) {
            DayValues& dv = days_[static_cast<std::size_t>(day_index)];
            dv.season = s;
            dv.day_in_season = k;

            const real x = static_cast<real>(day_index);
            const real xc = x + 0.5;
            dv.y_start = y_of(s, x);
            dv.y_end = y_of(s, x + 1.0);
            const real y_noon = y_of(s, xc);

            int left = 0, right = 0;
            real w = 0.0;
            if (n > 1) {
                int j = 0;
                while (j < n && mid[static_cast<std::size_t>(j)] <= xc) ++j;
                real m_left, m_right;
                if (j == 0) {
                    left = n - 1;
                    right = 0;
                    m_left = mid[static_cast<std::size_t>(n - 1)] - static_cast<real>(total);
                    m_right = mid[0];
                } else if (j == n) {
                    left = n - 1;
                    right = 0;
                    m_left = mid[static_cast<std::size_t>(n - 1)];
                    m_right = mid[0] + static_cast<real>(total);
                } else {
                    left = j - 1;
                    right = j;
                    m_left = mid[static_cast<std::size_t>(left)];
                    m_right = mid[static_cast<std::size_t>(right)];
                }
                w = (xc - m_left) / (m_right - m_left);
            }

            const auto dl = day_night_of(left, y_noon);
            const auto dr = day_night_of(right, y_noon);
            dv.day_s = lerp(dl.first, dr.first, w);
            dv.night_s = lerp(dl.second, dr.second, w);

            const SeasonDesc& sl = desc.seasons[static_cast<std::size_t>(left)];
            const SeasonDesc& sr = desc.seasons[static_cast<std::size_t>(right)];
            if (sl.climate && sr.climate) {
                dv.has_climate = true;
                dv.climate.temp_mean_c = lerp(sl.climate->temp_mean_c, sr.climate->temp_mean_c, w);
                dv.climate.temp_range_k = lerp(sl.climate->temp_range_k, sr.climate->temp_range_k, w);
                dv.climate.rh_mean = lerp(sl.climate->rh_mean, sr.climate->rh_mean, w);
                dv.climate.pressure_msl_hpa = lerp(sl.climate->pressure_msl_hpa, sr.climate->pressure_msl_hpa, w);
                dv.climate.ground_temp_c = lerp(sl.climate->ground_temp_c, sr.climate->ground_temp_c, w);
            }
            const ResolvedAuthored& al = authored[static_cast<std::size_t>(left)];
            const ResolvedAuthored& ar = authored[static_cast<std::size_t>(right)];
            if (al.valid && ar.valid) {
                dv.has_authored = true;
                dv.authored.e_max_deg = lerp(al.value.e_max_deg, ar.value.e_max_deg, w);
                dv.authored.e_night_deg = lerp(al.value.e_night_deg, ar.value.e_night_deg, w);
                dv.authored.az_rise_deg = lerp(al.value.az_rise_deg, ar.value.az_rise_deg, w);
                dv.authored.az_set_deg = lerp(al.value.az_set_deg, ar.value.az_set_deg, w);
            }
        }
    }

    // ---- Fixed modes: the calendar still counts days, every day carries one
    // set of values and a constant day of year.
    if (options.mode == CalendarMode::FixedSeason) {
        const int s = options.fixed_season;
        const real y_mid = y_mid_of(s);
        const auto dn = day_night_of(s, y_mid);
        const SeasonDesc& sd = desc.seasons[static_cast<std::size_t>(s)];
        for (DayValues& dv : days_) {
            dv.day_s = dn.first;
            dv.night_s = dn.second;
            dv.y_start = y_mid;
            dv.y_end = y_mid;
            dv.has_climate = sd.climate.has_value();
            if (sd.climate) dv.climate = *sd.climate;
            dv.has_authored = authored[static_cast<std::size_t>(s)].valid;
            if (dv.has_authored) dv.authored = authored[static_cast<std::size_t>(s)].value;
        }
    } else if (options.mode == CalendarMode::FixedDay) {
        const DayValues ref = days_[static_cast<std::size_t>(options.fixed_day)];
        const real y_noon = 0.5 * (ref.y_start + ref.y_end);
        for (DayValues& dv : days_) {
            const int season = dv.season;
            const int day_in_season = dv.day_in_season;
            dv = ref;
            dv.season = season;
            dv.day_in_season = day_in_season;
            dv.y_start = y_noon;
            dv.y_end = y_noon;
        }
    }

    // ---- Prefix sums over one year.
    day_start_.assign(static_cast<std::size_t>(total) + 1, 0.0);
    for (int d = 0; d < total; ++d)
        day_start_[static_cast<std::size_t>(d) + 1] = day_start_[static_cast<std::size_t>(d)] + days_[static_cast<std::size_t>(d)].length_s();
    year_length_s_ = day_start_[static_cast<std::size_t>(total)];

    // ---- Epoch.
    int epoch_season = 0;
    if (!desc.calendar.epoch.season.empty()) epoch_season = season_index(desc.calendar.epoch.season);
    const int epoch_day = season_start_[static_cast<std::size_t>(epoch_season)] + desc.calendar.epoch.day;
    const real epoch_len = days_[static_cast<std::size_t>(epoch_day)].length_s();
    if (!(desc.calendar.epoch.time_of_day_s < epoch_len))
        throw std::invalid_argument("Calendar: calendar.epoch.time_of_day_s is beyond the length of the epoch day");
    epoch_year_ = desc.calendar.epoch.year;
    epoch_offset_s_ = day_start_[static_cast<std::size_t>(epoch_day)] + desc.calendar.epoch.time_of_day_s;
}

int Calendar::season_index(const std::string& name) const {
    for (std::size_t i = 0; i < season_names_.size(); ++i)
        if (season_names_[i] == name) return static_cast<int>(i);
    return -1;
}

CalendarSample Calendar::sample(real t_env) const {
    const real total = t_env + epoch_offset_s_;
    real years = std::floor(total / year_length_s_);
    real r = total - years * year_length_s_;
    if (r < 0.0) { r += year_length_s_; years -= 1.0; }
    if (r >= year_length_s_) { r -= year_length_s_; years += 1.0; }

    const std::size_t n = days_.size();
    std::size_t idx = static_cast<std::size_t>(std::upper_bound(day_start_.begin(), day_start_.end(), r) - day_start_.begin());
    idx = idx == 0 ? 0 : idx - 1;
    if (idx >= n) idx = n - 1;

    const DayValues& dv = days_[idx];
    CalendarSample s;
    s.year = epoch_year_ + static_cast<std::int64_t>(years);
    s.day_of_year = static_cast<int>(idx);
    s.season = dv.season;
    s.day_in_season = dv.day_in_season;
    s.time_of_day_s = r - day_start_[idx];
    s.day_s = dv.day_s;
    s.night_s = dv.night_s;
    const real len = dv.length_s();
    s.day_fraction = s.time_of_day_s / len;

    const real half_night = 0.5 * dv.night_s;
    const real tod = s.time_of_day_s;
    if (tod < half_night) {
        s.phase = DayPhase::NightBefore;
        s.phase_u = 0.5 + tod / dv.night_s;
    } else if (tod < half_night + dv.day_s) {
        s.phase = DayPhase::Day;
        s.phase_u = (tod - half_night) / dv.day_s;
    } else if (dv.night_s > 0.0) {
        s.phase = DayPhase::NightAfter;
        s.phase_u = (tod - half_night - dv.day_s) / dv.night_s;
    } else {
        // Polar day (no night): a rounding-edge tod at the very end of the day.
        s.phase = DayPhase::Day;
        s.phase_u = 1.0;
    }
    return s;
}

real Calendar::env_time(std::int64_t year, int season, int day_in_season, real time_of_day_s) const {
    if (season < 0 || season >= season_count())
        throw std::invalid_argument("Calendar::env_time: season out of range");
    if (day_in_season < 0 || day_in_season >= season_days_[static_cast<std::size_t>(season)])
        throw std::invalid_argument("Calendar::env_time: day out of range for the season");
    const int idx = season_start_[static_cast<std::size_t>(season)] + day_in_season;
    if (!(time_of_day_s >= 0.0) || time_of_day_s > days_[static_cast<std::size_t>(idx)].length_s())
        throw std::invalid_argument("Calendar::env_time: time of day outside the day");
    return static_cast<real>(year - epoch_year_) * year_length_s_ + day_start_[static_cast<std::size_t>(idx)] +
           time_of_day_s - epoch_offset_s_;
}

} // namespace ps::environment
