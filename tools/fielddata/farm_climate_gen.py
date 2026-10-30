"""Generates data/climates/farm_de_home.json (physics_sim.climate/1) for the farming
field model (ps_field, F1a). Dev tool, stdlib only; the generated file is committed.

A real-year climate for the home region (Rhein-Main, 50.15 N): four seasons whose
lengths are the real ones (365 game days, one game day = one real day, so the sun
warp is the identity), starting on 1 December. Season values are MID-season values
(the calendar interpolates linearly between season midpoints), so each is set to
the monthly normal of the middle month: winter = January, spring = April, summer =
July, autumn = October.

Every number is ASSUMED ([AS]) from memory of the DWD 1991-2020 normals for the
Rhein-Main area (not re-read in this session); the regime statistics are the
weather plan's test regimes (data/climates/test_weather.json) with the
precipitation intensity of the rain regime solved here so that the long-run mean
precipitation per season matches the target (semi-Markov stationary mix).
Usage: python tools/fielddata/farm_climate_gen.py [out.json]
"""
import json
import math
import sys

# name, days, doy span, T mean, range K, rh, pressure hPa, ground T, target mm/month of the mid month [AS],
# target mean cloud factor G/G_clear of the mid month [AS]
SEASONS = [
    ("winter", 90, [335, 59], 1.8, 7.9, 0.86, 1017.0, 3.5, 45.0, 0.45, 0.87),
    ("spring", 92, [60, 151], 10.3, 19.0, 0.72, 1016.0, 10.0, 48.0, 0.64, 0.74),
    ("summer", 92, [152, 243], 19.6, 19.5, 0.67, 1015.0, 19.0, 62.0, 0.64, 0.70),
    ("autumn", 91, [244, 334], 10.0, 14.8, 0.82, 1016.0, 11.0, 50.0, 0.57, 0.82),
]
# The cloud factor targets are monthly global-radiation normals (recalled approximate DWD values for
# Rhein-Main in kWh/m2: Jan 21, Apr 120, Jul 165, Oct 55 [AS]) divided by the Haurwitz clear-sky daily
# total at 50.15 N for the middle of the month (5.3, 22.7, 30.1, 11.2 MJ/m2 per day).


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
    if season == "summer":
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
        "calendar": {"epoch": {"year": 0, "season": "winter", "day": 0, "time_of_day_s": 0}, "cycle_s": 86400},
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
            "seasons": "days 90/92/92/91 = the real meteorological seasons (Dec-Feb 90 days in a non-leap year, Mar-May 92, Jun-Aug 92, Sep-Nov 91); doy_span as test_fixed_summer.json. Season values sit at the season midpoint, so they are set to the January/April/July/October normals.",
            "scalars": "[AS] temp_mean_c 1.8/10.3/19.6/10.0, temp_range_k, rh_mean, ground_temp_c: approximate DWD 1991-2020 normals for the Rhein-Main area recalled from memory, NOT re-read from the DWD files. Regime offsets shift the realised means; the measured annual statistics are reported by the ps_field tests.",
            "regimes": "[AS] the regimes, transitions and durations of test_weather.json (the weather plan's assumed regime statistics); only the rain regime's precipitation median is solved by tools/fielddata/farm_climate_gen.py so that the long-run mean precipitation matches 45/48/62/50 mm per month (winter/spring/summer/autumn, [AS] recalled Rhein-Main normals of roughly 620 mm per year).",
            "rest": "lags_s, precip_phase, humidity, diurnal_temperature, lightning: as test_weather.json (see its source notes)."
        }}


if __name__ == "__main__":
    out = sys.argv[1] if len(sys.argv) > 1 else "data/climates/farm_de_home.json"
    with open(out, "w") as f:
        json.dump(build(), f, indent=2)
        f.write("\n")
