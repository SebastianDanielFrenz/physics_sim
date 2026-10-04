// ps/environment/calendar.h - seasons, per-day day/night lengths and the
// t_env -> (year, day, phase) mapping (weather plan 2.2).
//
// Model
//  - A year is the concatenation of the climate's seasons; season i has
//    `days` game days; N_y = sum(days). Each game day is a cycle of
//    night/2, day, night/2 (midnight is the middle of the night): sunrise at
//    night/2, sunset at night/2 + day.
//  - Season values are MID-SEASON values. Per-day values (day_s, night_s, the
//    climate scalars, the authored sun parameters) are interpolated linearly in
//    day index between season midpoints (cyclic over the year), so day length
//    and climate change smoothly across season boundaries [AS - a step at the
//    boundary would be a visible jump at midnight]. The midpoint of season i is
//    start_i + days_i/2 in day-CENTRE coordinates (day d has centre d+0.5), so
//    an ODD day count has its midpoint on a day centre and that day carries the
//    authored value exactly.
//  - Exception, documented: a season whose day/night form is ASTRONOMICAL
//    computes its day length per day directly from that day's real day of year
//    (interpolating between season-mid values would put the solstice error at
//    ~1 h); the same interpolation weights then blend it with a neighbour of
//    another form.
//  - The real day of year the game day stands for is a continuous piecewise-
//    linear function y(x) of the day coordinate x (days since the year start):
//    y = (doy_start - 1) + (x - season_start)/days * span_length, in days since
//    1 January 00:00. The sun reads the declination at y(x) continuously through
//    the day.
//  - Fixed-season / fixed-day modes: the calendar still counts days but every
//    day carries one season's mid values (or one chosen day's values), and y is
//    constant.
//
// Allocation-free after construction: sample() is a binary search over the
// precomputed prefix sums of one year (O(log N_y)).

#pragma once

#include "ps/environment/climate_desc.h"
#include "ps/types.h"

#include <cstdint>
#include <string>
#include <vector>

namespace ps::environment {

struct DayValues {
    real day_s = 0.0;
    real night_s = 0.0;
    real y_start = 0.0;   // continuous day of year (days since 1 Jan 00:00) at this game day's start
    real y_end = 0.0;     // ... at its end
    int season = 0;       // natural season of this game day
    int day_in_season = 0;
    bool has_climate = false;
    SeasonClimateDesc climate;   // valid iff has_climate
    bool has_authored = false;
    SunAuthoredDesc authored;    // valid iff has_authored (degrees)

    real length_s() const { return day_s + night_s; }
};

enum class CalendarMode : std::uint8_t { Dynamic, FixedSeason, FixedDay };

struct CalendarOptions {
    CalendarMode mode = CalendarMode::Dynamic;
    int fixed_season = 0;   // FixedSeason: season index
    int fixed_day = 0;      // FixedDay: day index within the year, [0, N_y)
};

enum class DayPhase : std::uint8_t { NightBefore, Day, NightAfter };

struct CalendarSample {
    std::int64_t year = 0;        // epoch.year + whole years since the epoch year start
    int day_of_year = 0;          // game day index within the year, [0, N_y)
    int season = 0;
    int day_in_season = 0;
    real time_of_day_s = 0.0;     // since the day's start (midnight), [0, length)
    real day_s = 0.0;
    real night_s = 0.0;
    DayPhase phase = DayPhase::Day;
    // Phase coordinate: Day: u in [0,1) from sunrise to sunset. Night: v in
    // [0,1) from sunset to the next sunrise (a day's first night half has
    // v in [0.5, 1), its last half v in [0, 0.5)).
    real phase_u = 0.0;
    real day_fraction = 0.0;      // time_of_day_s / length, [0,1)

    bool is_day() const { return phase == DayPhase::Day; }
};

class Calendar {
public:
    // Throws std::invalid_argument (via validate_climate) on a bad descriptor,
    // on a bad option index, or on an epoch time of day beyond its day.
    explicit Calendar(const ClimateDesc& desc, CalendarOptions options = {});

    int days_per_year() const { return static_cast<int>(days_.size()); }
    real year_length_s() const { return year_length_s_; }
    int season_count() const { return static_cast<int>(season_names_.size()); }
    const std::string& season_name(int i) const { return season_names_[static_cast<std::size_t>(i)]; }
    int season_start_day(int i) const { return season_start_[static_cast<std::size_t>(i)]; }
    int season_days(int i) const { return season_days_[static_cast<std::size_t>(i)]; }

    const DayValues& day(int day_of_year) const { return days_[static_cast<std::size_t>(day_of_year)]; }
    // Start of a game day in seconds since the year start.
    real day_start_in_year_s(int day_of_year) const { return day_start_[static_cast<std::size_t>(day_of_year)]; }

    CalendarSample sample(real t_env) const;

    // t_env of (year, season, day within season, time of day). The inverse of
    // sample() for in-range arguments. Throws std::invalid_argument for an
    // out-of-range season/day or a time of day outside [0, day length].
    real env_time(std::int64_t year, int season, int day_in_season, real time_of_day_s) const;

    // t_env = 0 sits at this many seconds after the start of epoch.year.
    real epoch_offset_s() const { return epoch_offset_s_; }
    std::int64_t epoch_year() const { return epoch_year_; }

    // Sunrise / sunset as seconds since that day's start.
    real sunrise_tod_s(int day_of_year) const { return days_[static_cast<std::size_t>(day_of_year)].night_s * 0.5; }
    real sunset_tod_s(int day_of_year) const {
        const DayValues& d = days_[static_cast<std::size_t>(day_of_year)];
        return d.night_s * 0.5 + d.day_s;
    }

    int season_index(const std::string& name) const;  // -1 when unknown

private:
    std::vector<DayValues> days_;
    std::vector<real> day_start_;       // N_y + 1 entries, last = year length
    std::vector<std::string> season_names_;
    std::vector<int> season_start_;
    std::vector<int> season_days_;
    real year_length_s_ = 0.0;
    real epoch_offset_s_ = 0.0;
    std::int64_t epoch_year_ = 0;
};

} // namespace ps::environment
