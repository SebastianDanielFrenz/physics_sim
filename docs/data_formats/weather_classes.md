# `physics_sim.weather_classes/1` - weather class thresholds (weather plan W3)

Loader: `ps::io::load_weather_classes_json(path)` / `parse_weather_classes_json`
(`ps/environment/weather_io.h`). Descriptor and classifier:
`ps::environment::WeatherClassesDesc`, `classify(desc, sample)`
(`ps/environment/weather_classes.h`). File:
`data/weather_classes/wmo_dwd_torro.json` (WMO-No. 8, DWD Warnkriterien,
TORRO - the citations are in its `source` block).

Every threshold is data, so a different national convention or the owner's
answer to Q17 (cloudburst) is a data change. A scale is

```
{"thresholds": [t1, t2, ...], "rule": "ge" | "gt" | ["ge", "gt", ...]}
```

thresholds strictly ascending, finite, >= 0; `rule` one entry per threshold (a
single string applies to all). The level of a value is the number of
consecutive thresholds it passes from the bottom - `ge`: `v >= t`, `gt`:
`v > t` - so each boundary value belongs to exactly one class
(`tests/unit/test_environment_weather_classes.cpp`, T3).

| Key | Thresholds | Meaning of the level |
|---|---|---|
| `drizzle_mm_h` | 2 | 0 light, 1 moderate, 2 heavy (WMO) |
| `rain_mm_h` | 3 | 0 light .. 3 violent (WMO); also sleet, freezing rain and hail [AS] |
| `snow_mm_h` | 2 | 0 light, 1 moderate, 2 heavy (water equivalent, WMO) |
| `cloudburst_mm_h` | number > 0 | instantaneous rate >= this is a cloudburst (Q17, default 100 [AS]) |
| `dwd_rain_1h_mm`, `dwd_rain_6h_mm` | 3 each | 1 Starkregen, 2 heftiger, 3 extrem heftiger (DWD); applied to the trailing 1 h / 6 h accumulation, the larger level wins |
| `dwd_rain_unwetter_level` | integer [1, 3] | rain level from which a warning is an Unwetter |
| `beaufort_m_s` | 12 | Beaufort force 0 .. 12 of the 10-min mean wind |
| `dwd_gust_km_h` | 6 | 1 Windboeen .. 6 extreme Orkanboeen (DWD), on the nominal gust (mean x gust factor, km/h) until W4 synthesises gusts |
| `dwd_gust_unwetter_level` | integer [1, 6] | gust level from which a warning is an Unwetter |
| `torro_hail_mm` | 11 | TORRO size code = level - 1 (-1 = below code 0 or no hail) |
| `source` | - | provenance (string or object) |

`classify()` returns: the sample's precipitation type, the intensity on the
type's scale (`None`..`Violent`), `cloudburst`, `dwd_rain_level`, `beaufort`,
`dwd_gust_level`, `torro_hail_code`, `thunderstorm` (lightning rate > 0) and
`unwetter`.

Rejections: wrong `format`; unknown key; a missing scale or `rule`; a wrong
threshold count; thresholds not ascending, negative or non-finite; a rule
other than `ge`/`gt` or a rule array of another length; `cloudburst_mm_h` <= 0;
an Unwetter level outside its range.
