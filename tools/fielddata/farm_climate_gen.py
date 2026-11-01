"""Generates data/climates/farm_de_home.json (physics_sim.climate/1) for the farming
field model (ps_field, F1a). Dev tool, stdlib only; the generated file is committed.

A real-year climate for the home region (Rhein-Main, 50.15 N): twelve monthly "seasons"
whose lengths are the real ones (365 game days, one game day = one real day, so the sun
warp is the identity), starting on 1 December. Season values are MID-season values
(the calendar interpolates linearly between season midpoints), so each month carries
its own monthly normal. (F1c fix A: the former four-season version interpolated
February to 4.8 C against a 3.1 C normal and August to 16.8 C against 20.0 C, so winter
evaporation was too high and summer too low, and the profile drained almost nothing.)

Temperature, precipitation, humidity and the cloud factor come from the DWD station
Frankfurt/Main (1420) daily record, 1991-2020 (see the SEASONS comment); the regime
statistics are the weather plan's test regimes (data/climates/test_weather.json) with the
precipitation intensity of the rain regime solved here so that the long-run mean
precipitation per month matches the target (semi-Markov stationary mix).
Usage: python tools/fielddata/farm_climate_gen.py [out.json]
"""
import json
import math
import sys

# name, days, doy span (1-based, from 1 December), T mean, daily T range K (regime-mix input), rh, pressure hPa,
# ground T, target mm/month, target mean cloud factor G/G_clear, regime RH target.
# T mean, rain, rh: DWD station Frankfurt/Main (1420) daily record, 1991-2020 monthly means, computed from the
# produkt_klima_tag file [C] (annual 11.1 C, 598 mm). Cloud factor: Angstrom-Prescott from the same record's
# sunshine duration, (0.25 + 0.5 n/N)/0.75 with N the astronomical day length at 50.15 N [D]. temp_range_k =
# 1.7 x the record's mean daily range (Tx - Tn): the regime offsets and the diurnal model's cloud reduction
# bring the realised range back down (factor [AS], kept from the four-season version). ground_temp_c = monthly
# T + 1.0 in Dec-Feb, + 0.5 in Mar-May and Oct-Nov, - 0.3 in Jun-Sep [AS]. pressure_msl_hpa 1016/1017 [AS].
SEASONS = [
    ("dec", 31, [335, 365], 3.1, 8.7, 0.85, 1017.0, 4.1, 54.5, 0.44, 0.85),
    ("jan", 31, [1, 31], 2.3, 9.2, 0.82, 1017.0, 3.3, 44.0, 0.47, 0.82),
    ("feb", 28, [32, 59], 3.1, 11.9, 0.78, 1017.0, 4.1, 38.6, 0.52, 0.78),
    ("mar", 31, [60, 90], 6.8, 15.5, 0.71, 1016.0, 7.3, 38.7, 0.59, 0.71),
    ("apr", 30, [91, 120], 11.1, 18.9, 0.64, 1016.0, 11.6, 36.6, 0.65, 0.64),
    ("may", 31, [121, 151], 15.1, 18.9, 0.66, 1016.0, 15.6, 60.4, 0.64, 0.66),
    ("jun", 30, [152, 181], 18.5, 19.0, 0.65, 1015.0, 18.2, 55.4, 0.65, 0.65),
    ("jul", 31, [182, 212], 20.5, 19.2, 0.64, 1015.0, 20.2, 63.5, 0.65, 0.64),
    ("aug", 31, [213, 243], 20.0, 19.2, 0.67, 1015.0, 19.7, 61.4, 0.67, 0.67),
    ("sep", 30, [244, 273], 15.5, 17.3, 0.74, 1016.0, 15.2, 47.7, 0.63, 0.74),
    ("oct", 31, [274, 304], 10.7, 13.8, 0.82, 1016.0, 11.2, 50.4, 0.55, 0.82),
    ("nov", 30, [305, 334], 6.1, 9.7, 0.85, 1017.0, 6.6, 47.3, 0.46, 0.85),
]


def regimes(season):
    r = {
        "clear": {"duration_h": {"min": 2, "median": 10, "sigma_log": 0.5}, "cloud_cover": [0.0, 0.25],
                  "precip_mm_h": None, "wind_mean_m_s": {"weibull_k": 2.0, "weibull_c": 3.5},
                  "wind_dir_deg": {"mean": 240.0, "spread": 70.0}, "gust_factor": 1.4,
                  "temp_offset_k": [0.5, 2.0], "rh": [0.45, 0.65], "pressure_offset_hpa": [3, 8]},
        "cloudy": {"duration_h": {"min": 2, "median": 8, "sigma_log": 0.5}, "cloud_cover": [0.6, 0.95],
                   "precip_mm_h": None, "wind_mean_m_s": {"weibull_k": 2.0, "weibull_c": 4.5},
                   "wind_dir_deg": {"mean": 240.0, "spread": 60.0}, "gust_factor": 1.5,
                   "temp_offset_k": [-1.0, 0.0], "rh": [0.65, 0.85], "pressure_offset_hpa": [-3, 3]},
        "rain": {"duration_h": {"min": 1, "median": 5, "sigma_log": 0.6}, "cloud_cover": [0.9, 1.0],
                 "precip_mm_h": {"median": 1.5, "sigma_log": 0.6},
                 "wind_mean_m_s": {"weibull_k": 2.0, "weibull_c": 5.5},
                 "wind_dir_deg": {"mean": 240.0, "spread": 60.0}, "gust_factor": 1.6,
                 "temp_offset_k": [-2.0, -1.0], "rh": [0.85, 0.95], "pressure_offset_hpa": [-10, -4]},
    }
    tr = {"clear": {"clear": 0.5, "cloudy": 0.4, "rain": 0.1},
          "cloudy": {"clear": 0.3, "cloudy": 0.4, "rain": 0.3},
          "rain": {"clear": 0.2, "cloudy": 0.5, "rain": 0.3}}
    if season in ("may", "jun", "jul", "aug", "sep"):
        r["thunderstorm"] = {
            "duration_h": {"min": 1, "median": 3, "sigma_log": 0.4}, "cloud_cover": [0.7, 1.0],
            "precip_mm_h": {"median": 0.3, "sigma_log": 0.5},
            "wind_mean_m_s": {"weibull_k": 2.0, "weibull_c": 4.0},
            "wind_dir_deg": {"mean": 240.0, "spread": 60.0}, "gust_factor": 1.8,
            "temp_offset_k": [-3.0, -1.0], "rh": [0.75, 0.9], "pressure_offset_hpa": [-5, -1],
            "convective": {"cell_duration_min": {"median": 30, "sigma_log": 0.3},
                           "gap_min": {"median": 45, "sigma_log": 0.5},
                           "peak_mm_h": {"median": 25, "sigma_log": 0.7}, "rise_fraction": 0.3,
                           "gust_factor": 2.0,
                           "hail": {"probability": 0.05, "diameter_mm": {"median": 12, "sigma_log": 0.4},
                                    "min_shape": 0.6}},
            "lightning_per_min": 3.0}
        tr = {"clear": {"clear": 0.5, "cloudy": 0.3, "rain": 0.05, "thunderstorm": 0.15},
              "cloudy": {"clear": 0.35, "cloudy": 0.35, "rain": 0.2, "thunderstorm": 0.1},
              "rain": {"clear": 0.3, "cloudy": 0.5, "rain": 0.2},
              "thunderstorm": {"clear": 0.5, "cloudy": 0.4, "rain": 0.1}}
    return r, tr


def mean_logn(median, sigma):
    return median * math.exp(sigma * sigma / 2.0)


def stationary(names, tr):
    p = {n: 1.0 / len(names) for n in names}
    for _ in range(2000):
        q = {n: 0.0 for n in names}
        for a in names:
            for b, w in tr[a].items():
                q[b] += p[a] * w
        p = q
    return p


def mm_per_month(reg, tr):
    names = list(reg)
    pi = stationary(names, tr)

    def dur(n):
        return mean_logn(reg[n]["duration_h"]["median"], reg[n]["duration_h"]["sigma_log"])

    tot_t = sum(pi[n] * dur(n) for n in names)
    rate = 0.0
    for n in names:
        pr = reg[n]["precip_mm_h"]
        if not pr:
            continue
        frac = pi[n] * dur(n) / tot_t
        if "convective" in reg[n]:
            c = reg[n]["convective"]
            d = mean_logn(c["cell_duration_min"]["median"], c["cell_duration_min"]["sigma_log"])
            g = mean_logn(c["gap_min"]["median"], c["gap_min"]["sigma_log"])
            peak = mean_logn(c["peak_mm_h"]["median"], c["peak_mm_h"]["sigma_log"])
            rate += frac * (mean_logn(pr["median"], pr["sigma_log"]) + d / (d + g) * 0.5 * peak)
        else:
            rate += frac * mean_logn(pr["median"], pr["sigma_log"])
    return rate * 24.0 * 30.4


def mean_factor(reg, tr, c):
    """Mean of 1 - 0.75 N^3.4 (Kasten & Czeplak) over the regime mix with every cloud range pushed toward 1 by c."""
    names = list(reg)
    pi = stationary(names, tr)
    dur = {n: mean_logn(reg[n]["duration_h"]["median"], reg[n]["duration_h"]["sigma_log"]) for n in names}
    tot = sum(pi[n] * dur[n] for n in names)
    f = 0.0
    for n in names:
        a, b = reg[n]["cloud_cover"]
        a, b = a + c * (1 - a), b + c * (1 - b)
        e = 1.0 - 0.75 * ((b ** 4.4 - a ** 4.4) / (4.4 * (b - a)) if b > a + 1e-9 else a ** 3.4)
        f += pi[n] * dur[n] / tot * e
    return f


def build():
    seasons = []
    for name, days, span, tmean, rng, rh, pres, gt, target, cloud_target, rh_target in SEASONS:
        reg, tr = regimes(name)
        lo, hi = 0.0, 1.0
        for _ in range(60):
            mid = 0.5 * (lo + hi)
            if mean_factor(reg, tr, mid) > cloud_target:
                lo = mid
            else:
                hi = mid
        c = 0.5 * (lo + hi)
        sys.stderr.write("%s: cloud shift %.3f -> mean factor %.3f (target %.2f)\n" % (name, c, mean_factor(reg, tr, c), cloud_target))
        for n in reg:
            a, b = reg[n]["cloud_cover"]
            reg[n]["cloud_cover"] = [round(a + c * (1 - a), 3), round(b + c * (1 - b), 3)]
        # humidity: the mean of the regime RH ranges over the regime mix is pushed to the season's RH target
        pi = stationary(list(reg), tr)
        dur = {n: mean_logn(reg[n]["duration_h"]["median"], reg[n]["duration_h"]["sigma_log"]) for n in reg}
        tot = sum(pi[n] * dur[n] for n in reg)
        cur = sum(pi[n] * dur[n] / tot * 0.5 * sum(reg[n]["rh"]) for n in reg)
        k = (rh_target - cur) / (1.0 - cur) if rh_target > cur else (rh_target - cur) / cur
        for n in reg:
            reg[n]["rh"] = [round(min(1.0, max(0.05, v + (k * (1 - v) if k > 0 else k * v))), 3) for v in reg[n]["rh"]]
        sys.stderr.write("%s: mean regime RH %.3f -> target %.2f\n" % (name, cur, rh_target))
        scale = target / mm_per_month(reg, tr)
        # the long-run mean is linear in every intensity median, so one factor scales them all
        reg["rain"]["precip_mm_h"]["median"] = round(reg["rain"]["precip_mm_h"]["median"] * scale, 3)
        if "thunderstorm" in reg:
            ts = reg["thunderstorm"]
            ts["precip_mm_h"]["median"] = round(ts["precip_mm_h"]["median"] * scale, 3)
            ts["convective"]["peak_mm_h"]["median"] = round(ts["convective"]["peak_mm_h"]["median"] * scale, 3)
        sys.stderr.write("%s: estimated %.1f mm/month (target %.1f)\n" % (name, mm_per_month(reg, tr), target))
        seasons.append({"name": name, "days": days, "doy_span": span, "day_night": {"astronomical": True},
                        "temp_mean_c": tmean, "temp_range_k": rng, "rh_mean": rh, "pressure_msl_hpa": pres,
                        "ground_temp_c": gt, "regimes": reg, "transitions": tr, "initial_regime": "cloudy"})
    return {
        "format": "physics_sim.climate/1",
        "location": {"latitude_deg": 50.14988, "longitude_deg": 8.45474, "elevation_m": 150},
        "sun": {"mode": "astronomical"},
        "calendar": {"epoch": {"year": 0, "season": "dec", "day": 0, "time_of_day_s": 0}, "cycle_s": 86400},
        "seasons": seasons,
        "lags_s": {"cloud": 1200, "precip_on": 300, "precip_off": 600, "wind": 900, "temp_offset": 3600,
                   "rh": 1800, "pressure": 3600},
        "precip_phase": {"snow_below_c": 0.0, "rain_above_c": 2.0, "min_rate_mm_h": 0.01},
        "humidity": {"precip_rh_floor": 0.95, "precip_full_mm_h": 0.5},
        "diurnal_temperature": {"a_h": 1.86, "b": 2.2, "c_h": -0.17, "reference_cycle_s": 86400,
                                "cloud_range_reduction": 0.6},
        "lightning": {"distance_km": [0.5, 15.0]},
        "source": {
            "purpose": "climate of the farming field model (ps_field F1a): a real 365-day year starting 1 December for the home region.",
            "location": "latitude/longitude: the racing_game home-region spawn point as in test_fixed_summer.json; elevation 150 m ASSUMED.",
            "seasons": "twelve calendar months (31/31/28/31/30/31/30/31/31/30/31/30 days from 1 December = 365); doy_span 1-based. Season values sit at the month midpoint, so they are the monthly normals.",
            "scalars": "temp_mean_c, rh_mean, precipitation target and cloud factor target: DWD station Frankfurt/Main (1420) daily record 1991-2020 monthly means [C], cloud factor via Angstrom-Prescott from sunshine duration [D]; temp_range_k 1.7 x the mean daily range, ground_temp_c, pressure [AS]. Regime offsets shift the realised means; the measured annual statistics are reported by the ps_field tests.",
            "regimes": "[AS] the regimes, transitions and durations of test_weather.json (the weather plan's assumed regime statistics); only the rain regime's precipitation median is solved by tools/fielddata/farm_climate_gen.py so that the long-run mean precipitation matches 45/48/62/50 mm per month (the monthly means of the Frankfurt/Main record, 598 mm per year [C]).",
            "rest": "lags_s, precip_phase, humidity, diurnal_temperature, lightning: as test_weather.json (see its source notes)."
        }}


if __name__ == "__main__":
    out = sys.argv[1] if len(sys.argv) > 1 else "data/climates/farm_de_home.json"
    with open(out, "w") as f:
        json.dump(build(), f, indent=2)
        f.write("\n")
