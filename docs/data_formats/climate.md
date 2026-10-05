# `physics_sim.climate/1` - climate files (weather plan W1 + W3 + W4)

Loader: `ps::io::load_climate_json(path)` (`core/include/ps/environment/climate_io.h`,
library `ps_environment_io`, which does not link `ps_core`). Descriptor:
`ps/environment/climate_desc.h`. Model: `ps_environment` (`Calendar`, `Sun`,
`EnvClock`). Example: `data/climates/test_fixed_summer.json`.

W1 implements location, sun, calendar, seasons (day/night lengths, mid-season
climate scalars, per-season authored sun override) and `extends`. W3 adds the
weather statistics: per-season `regimes`, `transitions`, `initial_regime` and
the top-level `lags_s`, `precip_phase`, `humidity`, `diurnal_temperature`,
`lightning` blocks (section "Weather (W3)" below; model:
`ps/environment/weather.h`, example `data/climates/test_weather.json`).
`gusts` (W4, section "Gusts (W4)") configures the turbulence synthesis; `condition` stays
RESERVED: a file that contains it is rejected with "... is not implemented before W5".
Like every ps loader, any other unknown key is rejected.

## Top level

| Key | Type | Notes |
|---|---|---|
| `format` | string, required | exactly `"physics_sim.climate/1"` |
| `extends` | string, optional | relative path of a parent file, see below |
| `location` | object, required | `latitude_deg` [-90, 90] (required), `longitude_deg` [-180, 180], `elevation_m` (both optional, default 0; the clock is local mean solar time; from W4 `elevation_m` is the altitude of the World z = 0 plane above sea level, which the ISA atmosphere lifts from - see "Atmosphere and wind (W4)") |
| `sun` | object, optional | `mode`: `"astronomical"` (default) or `"authored"`; `authored`: `{e_max_deg, e_night_deg, az_rise_deg, az_set_deg}` (degrees). Authored mode needs a complete block globally or per season |
| `calendar` | object, optional | `cycle_s` > 0 (default 86400, the global day+night length used by the astronomical day/night form); `epoch` `{year, season, day, time_of_day_s}` = what `t_env = 0` means (season name, empty/absent = first season; `day` within the season; `time_of_day_s` since that day's midnight, >= 0) |
| `seasons` | array, required, non-empty | see below |
| `source` | string or object | free-form provenance; only cited / DERIVED / ASSUMED statements belong here |
| `lags_s` | object, W3 | first-order lag time constants in seconds, all > 0 and required: `cloud`, `precip_on` (rising rate), `precip_off` (falling rate), `wind` (speed and direction), `temp_offset`, `rh`, `pressure` |
| `precip_phase` | object, W3 | `snow_below_c` (T <= this: snow), `rain_above_c` (T >= this: rain, > `snow_below_c`; sleet between), `min_rate_mm_h` > 0 (below it the published type is none) |
| `humidity` | object, W3 | `precip_rh_floor` [0,1], `precip_full_mm_h` > 0: during precipitation `rh += (max(rh, floor) - rh) * clamp(P / precip_full_mm_h, 0, 1)` |
| `diurnal_temperature` | object, W3 | Parton-Logan: `a_h` > 0 (lag of the maximum), `b` > 0 (night decay), `c_h` (lag of the minimum after sunrise), `reference_cycle_s` > 0 (a_h and c_h scale with day+night / this), `cloud_range_reduction` [0,1] (range x (1 - this x N)); `a_h > -c_h/2` |
| `lightning` | object, W3 | `distance_km: [min, max]`, 0 <= min <= max, max > 0: strike distance from the weather cell centre, area-uniform |
| `gusts` | object, W4 | turbulence synthesis, see "Gusts (W4)"; every member optional. Needs the weather statistics only for the weather, not for loading |

## Seasons

A year is the concatenation of its seasons; each has `days` game days (integer
>= 1, total <= 100000). Names are unique and non-empty.

| Key | Notes |
|---|---|
| `name`, `days` | required |
| `doy_span` | `[first, last]` real day of year, 1-based inclusive, each in [1, 366]; `first > last` wraps over the new year (northern winter `[335, 59]`). The real year is 365 days. **Required for every season** when the sun is astronomical or any season uses the astronomical day/night form |
| `day_night` | required, exactly ONE of: `{day_s, night_s}` (both > 0), `{cycle_s, day_fraction}` (cycle_s > 0, fraction in (0,1)), `{astronomical: true}` (day length from latitude and the real day of year, total = calendar `cycle_s`) |
| `temp_mean_c`, `temp_range_k` (>= 0), `rh_mean` [0,1], `pressure_msl_hpa` (> 0), `ground_temp_c` | mid-season climate scalars, carried and interpolated per day (used from W3). All five on EVERY season or on none |
| `sun_authored` | per-season override of the authored block; each member optional, unset members inherit the global block |
| `regimes`, `transitions`, `initial_regime` | W3 weather statistics, all three together, on EVERY season or on none - see "Weather (W3)" |

## Calendar model

* Each game day is `night/2`, `day`, `night/2`: sunrise at `night/2`, sunset at
  `night/2 + day`. Day/night split of a day is phase `NightBefore`, `Day`,
  `NightAfter` with `u` in [0,1) inside each.
* Per-day values (day and night length, climate scalars, authored sun
  parameters) are interpolated linearly between season midpoints (cyclic over
  the year), in day-centre coordinates (midpoint = start + days/2). A season
  with an odd number of days carries its exact authored values on its middle
  day.
* The real day of year used by the sun and the astronomical day length is the
  continuous `y` (days since 1 Jan 00:00; noon of 1 Jan = 0.5), piecewise
  linear per season over its `doy_span`.
* Modes (`CalendarOptions`): `Dynamic` (default), `FixedSeason` (every day
  carries that season's mid values and a constant `y`), `FixedDay`.
* `sample(t_env)` is O(log years-length) (prefix sums + binary search), no
  per-tick accumulation. `EnvClock::env_time(tick)` is the closed form
  `anchor_env_s + scale * (tick - anchor_tick) / tick_hz`; `scale = 0` freezes
  time.

## Sun model

Both forms of owner question Q1 are implemented.

* **Astronomical-warped (default).** Declination from Spencer's Fourier series
  [C Spencer 1971; coefficients as in NOAA GML "General Solar Position
  Calculations"], `gamma = 2 pi / 365 (y - 0.5)`. Sunrise hour angle `H0` at
  zenith 90.833 deg. The hour angle is warped piecewise linearly onto the
  game's day/night lengths: day `H = -H0 + 2 H0 u`, night
  `H = H0 + (2 pi - 2 H0) v`. With the real day length and `cycle_s = 86400`
  the warp is the identity (real local MEAN solar time; the equation of time
  and the longitude are not modelled). Elevation `sin e = sin(lat) sin(d) +
  cos(lat) cos(d) cos(H)`; azimuth from north, clockwise.
* **Authored.** Day elevation `e_max sin(pi u)`, night `-e_night sin(pi v)`;
  azimuth linear from `az_rise` to `az_set` through south (north for southern
  latitudes), continuing around at night.
* Polar days (real `H0` degenerate while the game day is not): fall back to the
  authored curve (global/season parameters, else derived from the real upper
  and lower culmination). A day with zero-length day or night (only the
  astronomical day/night form) uses the unwarped real hour angle.
* All trigonometry through `ps::math` (deterministic); `clear_sky_ghi_w_m2`
  and `ghi_w_m2` are plain closed forms.

### Accuracy (measured, E3)

Reference: NOAA's own solar-calculator JavaScript (Meeus), run unmodified.

* Solstices: declination within 0.02 deg, elevation 0.02 deg, azimuth 0.015
  deg, day length 0.23 min.
* Equinoxes (and wherever the declination changes fast): the Spencer series is
  about one day late and cannot follow the 4-year leap phase; declination error
  up to 0.43 deg on 2026-03-20/09-23 (0.53 deg over 2024-2028 on 30 sampled
  dates), elevation 0.43 deg, azimuth 0.28 deg, day length 4.1 min. The plan's
  "Spencer ~0.1 deg" holds only away from the equinoxes.

### Clear-sky irradiance

Haurwitz `G = 1098 cos z exp(-0.059 / cos z)` [C Haurwitz 1945]; the constants
were checked against the pvlib reference implementation (secondary source,
read 2026-10-05); the 1945 paper itself was not accessed, so the constant
0.059 (some texts print 0.057) is PROVISIONAL. Cloud factor
`1 - 0.75 N^3.4` [C Kasten & Czeplak 1980].

## Weather (W3)

Model and determinism contract: `ps/environment/weather.h` (`WeatherTimeline`).
A climate with regimes needs the climate scalars, `lags_s`, `precip_phase`,
`humidity` and `diurnal_temperature`; `lightning` too when any regime has
`lightning_per_min` > 0.

```
"regimes": {
  "rain": {
    "duration_h": {"min": 1, "median": 5, "sigma_log": 0.6},   // h: max(min, median * exp(sigma * n)); min >= 0.1
    "cloud_cover": [0.9, 1.0],                                 // uniform [lo, hi] in [0, 1]
    "precip_mm_h": {"median": 1.5, "sigma_log": 0.6},           // stratiform target (lognormal) or null
    "precip_form": "auto",                                      // optional: auto | drizzle | freezing_rain
    "convective": null,                                         // optional, see below
    "wind_mean_m_s": {"weibull_k": 2.0, "weibull_c": 5.5},      // 10-min mean at 10 m
    "wind_dir_deg": {"mean": 240, "spread": 60},                // "from"; mean + spread * (2u - 1)
    "gust_factor": 1.6,                                         // >= 1 (nominal gust = mean x factor)
    "temp_offset_k": [-2, -1], "rh": [0.85, 0.95], "pressure_offset_hpa": [-10, -4],
    "lightning_per_min": 0                                      // optional [0, 30]; convective: only in cells
  }, ... },
"transitions": {"rain": {"clear": 0.2, "cloudy": 0.5, "rain": 0.3}, ...},
"initial_regime": "cloudy"
```

`convective`: `{cell_duration_min, gap_min, peak_mm_h}` (each `{median,
sigma_log}`), `rise_fraction` (0,1), `gust_factor` >= 1 (at the cell peak),
optional `hail: {probability [0,1], diameter_mm {median, sigma_log},
min_shape (0,1]}`. Cells follow each other gap - cell - gap ... from the segment
start; a cell that would end after its segment is not started. The shape is a
triangle peaking at `rise_fraction` of the cell.

Every regime has a transition row; rows name only regimes of the same season
and sum to 1 within 1e-9 (a self-transition is allowed: the regime continues
with a new segment). When the chain enters a season in which the current regime
is not defined, the next regime is drawn from the row of that season's
`initial_regime`. Segment 0 starts at `t_env = 0` with the season's
`initial_regime`. Lognormal draws use an Irwin-Hall n = 4 normal bounded to
+-3.46 sigma, so the mean of `median * exp(sigma n)` is
`median * [sinh(x)/x]^4`, `x = sigma sqrt(3)/2`.

Starting statistics are ASSUMED [AS] until fitted to DWD data (owner question
Q11); a transition probability or duration is a data change.

## Atmosphere and wind (W4)

An `Environment` created with `weather` on (scenario `"weather": "dynamic"` or
`{"preset": file}`; `ps::environment::EnvironmentWeather`) provides the
atmosphere of the World (`ps/environment/atmosphere.h`, `wind.h`,
`vehicle_air.h`):

* **Sea-level reference.** The weather sample's air temperature and reduced
  pressure are the STATION values at `location.elevation_m` (+ the scenario's
  `altitude_offset_m`). They are lifted to sea level on the ISA lapse (6.5 K/km)
  to give T0, p0 = the published mean-sea-level pressure; the vapour mole
  fraction (from the relative humidity) is kept and capped at the Magnus
  saturation pressure.
* **Per vehicle.** Altitude of a vehicle = `location.elevation_m +
  altitude_offset_m + chassis z`. `atmosphere_at` gives T (ISA lapse, isothermal
  above 11 km, altitude clamped to [-5000, 20000] m), p (barometric, [C] ISO
  2533), vapour pressure (constant mixing ratio) and the humid-air density
  `(p - e)/(R_d T) + e/(R_v T)` (ideal gas mixture, `R_v = 461.5228`). These
  replace the World's `AmbientSample`, `air_density` and `wind_world` for that
  vehicle (engine ambient, aero) once per tick, in the `freeze_tick_start_state`
  stage. `World::set_ambient` and `World::set_aero_environment` throw
  `std::logic_error` while an environment WITH weather is installed (the weather
  is the single source of truth; accepted again after `remove_environment`).
  An environment without weather leaves both unchanged.
* **Mean wind.** The weather's 10-min mean wind speed and "from" direction,
  rotated by `north_yaw_deg`, as a world-frame vector blowing TOWARD
  `-(sin(from) east + cos(from) north)`. No height profile: the 10 m value is
  used at any altitude [AS].
* Vehicles: `VehicleSnapshot::air` (valid flag, altitude, pressure, temperature,
  vapour pressure, density, mean wind, gust, wind); the weather group of
  `EnvironmentSnapshot` carries the weather-time outside values for a HUD.
  None of this is hashed except the weather timeline fold, the altitude offset
  and the gust flag, and only while an environment with weather is installed.

## Gusts (W4)

```
"gusts": {
  "roughness_length_m": 0.03,      // z0 in (0, 10]: open grass [C Davenport-Wieringa]
  "reference_height_m": 10.0,      // z in (2 z0, 60]: mean-wind height and Kaimal scales
  "reference_gust_factor": 1.35,   // G_ref in (1, 4], see below
  "f_min_hz": 0.0016667,           // lowest band edge (the 10-min mean cutoff) [1e-5, 1]
  "bands_per_octave": 2,           // [2, 8]: 32 bands span 32 / n octaves
  "window_s": 600.0,               // [60, 86400] sim s: phase re-seed and advection window
  "blend_s": 30.0,                 // [0, window_s / 2]: crossfade at the start of a window
  "min_advection_speed_m_s": 0.5   // (0, 10]: floor of the Taylor advection speed
}
```

Absent = no gusts (the mean wind only); every member is optional. Unknown keys
and out-of-range values are load errors naming the key. Model:
`GustField` (`ps/environment/wind.h`), Shinozuka's spectral representation:

* Three axes (u along the mean wind, v across it, w vertical), 32 sinusoid
  lines each, one line per frequency band, uniform random frequency inside the
  band and random phase (SplitMix64 counter hash of the scenario seed, window
  index, axis, line).
* Kaimal spectrum, band variance `sigma^2 [C(f_lo) - C(f_hi)]` with
  `C(f) = (1 + 6 f L / U)^(-2/3)` (closed-form band integral of `S(f) = 4 sigma^2
  (L/U) / (1 + 6 f L / U)^(5/3)`), `L_u : L_v : L_w = 8.1 : 2.7 : 0.66` times
  `0.7 z` [C IEC 61400-1 and Kaimal et al. 1972, via a secondary source: the standard text was not re-read]; `sigma_v : sigma_w = 0.8 :
  0.5` times `sigma_u` [C IEC 61400-1].
* Intensity: `sigma_u / U = I (G - 1)/(G_ref - 1)` with the WMO-No. 8 intensity
  `I = 2.2 kappa / ln(z / z0)` [M] (0.1515 at z 10 m, z0 0.03 m) and `G` the
  gust factor of the current weather regime or preset.
* Frozen turbulence (Taylor): a point at `x` sees the field at `t - (x . e)/U_adv`
  (`e` the mean wind direction, `U_adv` the mean speed at the window start,
  floored at `min_advection_speed_m_s`); two vehicles in one gust field see
  one coherent field. There is no lateral decorrelation (documented limit).
* Gust time is simulation time (tick / tick_hz), the weather runs on environment
  time (weather plan Q6, PROVISIONAL): a scaled clock does not speed the gusts up.
* Every `window_s` of sim time the phases are re-drawn; across `blend_s` the new
  and old window are blended with cos/sin weights (variance preserving).

`reference_gust_factor`: the default 1.35 is what the synthesised field
MEASURES as its 3-s/10-min gust factor at the WMO intensity (1.346, sd 0.041
over 200 records, U 10 m/s; U 5: 1.358, U 20: 1.324). The plan cites about 1.66
for open land (Harper et al. 2010, secondary source); a Gaussian field reaches it
at `sigma_u / U` about 0.29, so a published gust factor of 1.66 here is
honoured by scaling the intensity (measured 1.652, sd 0.077, 200 records).
PROVISIONAL: the calibration holds for U about 10 m/s only (the 3-s averaging
path grows with U).

## `extends`

`"extends": "<relative path>"` (path relative to the file naming it): the
file's object is applied OVER its parent as a JSON Merge Patch [C RFC 7396] -
objects merge recursively, `null` deletes a member, anything else (including
arrays) replaces. Chains resolve bottom-up (deepest parent first, the file the
user named last). A cycle, a non-string/empty `extends`, an unreadable parent or
a chain deeper than 8 parents is an error naming the file.

## Rejections (all are `std::runtime_error`, message prefixed with the file)

Wrong or missing `format`; unknown key anywhere; the reserved W5 key `condition`; latitude
or longitude out of range; empty `seasons`; `days` < 1 or non-integer;
duplicate or empty season name; `doy_span` not two integers in [1, 366];
astronomical sun or day length without a `doy_span`; `day_s`/`night_s` <= 0;
more than one (or no) day/night form; `day_fraction` outside (0,1);
`cycle_s` <= 0; negative `temp_range_k`, `rh_mean` outside [0,1],
`pressure_msl_hpa` <= 0; climate scalars on some seasons only; unknown epoch
season, epoch day outside the season, negative `time_of_day_s`; unknown sun
mode; authored mode without a complete block; authored elevation out of range.

W3 (weather): regimes/transitions/initial_regime on some seasons only; empty
`regimes`; a transition row for an unknown regime, naming an unknown regime or
one twice, a probability outside [0,1] or not a number, a row not summing to 1
(1e-9), a regime without a row; unknown `initial_regime`; `duration_h.min` <
0.1, median <= 0, sigma outside [0, 3] (also every lognormal's sigma);
`cloud_cover`/`rh` outside [0,1], a range with lo > hi or not two numbers;
`precip_mm_h` missing (use null for none); unknown `precip_form`;
`freezing_rain` in a season with `temp_mean_c` above +5 degC [AS guard];
`weibull_k` <= 0, `weibull_c` < 0; `wind_dir_deg.mean` outside [0,360], spread
outside [0,180]; `gust_factor` < 1; `temp_offset_k` outside [-50,50],
`pressure_offset_hpa` outside [-200,200]; `lightning_per_min` outside [0,30];
`rise_fraction` outside (0,1); hail probability outside [0,1], `min_shape`
outside (0,1]; a lag <= 0; `rain_above_c` <= `snow_below_c`; `min_rate_mm_h`
<= 0; `precip_rh_floor` outside [0,1]; `precip_full_mm_h` <= 0; `a_h`, `b`,
`reference_cycle_s` <= 0, `a_h <= -c_h/2`, `cloud_range_reduction` outside
[0,1]; a bad lightning range; regimes without the climate scalars, `lags_s`,
`precip_phase`, `humidity`, `diurnal_temperature`, or (with lightning) the
`lightning` block.

W4 (gusts): `gusts` not an object; unknown `gusts.*` key; `roughness_length_m`
outside (0, 10]; `reference_height_m` outside (2 z0, 60]; `reference_gust_factor`
outside (1, 4]; `f_min_hz` outside [1e-5, 1]; `bands_per_octave` outside [2, 8];
`window_s` outside [60, 86400]; `blend_s` outside [0, window_s/2];
`min_advection_speed_m_s` outside (0, 10].
