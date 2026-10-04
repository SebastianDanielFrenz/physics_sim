# `physics_sim.weather_preset/1` - weather presets (weather plan W3)

Loader: `ps::io::load_weather_preset_json(path)` / `parse_weather_preset_json`
(`core/include/ps/environment/weather_io.h`, library `ps_environment_io`).
Descriptor: `ps::environment::WeatherPresetDesc` (`ps/environment/weather_desc.h`).
Files: `data/weather_presets/*.json` - clear, overcast, drizzle, light_rain,
rain, heavy_rain, cloudburst, thunderstorm, storm, snow, freezing_rain, fog.

A preset is one regime with fixed values. It is used two ways
(`ps/environment/weather.h`):

- **Preset mode** (`WeatherOptions::mode = Preset`): the preset is the base
  weather instead of the regime chain - constant except for the diurnal air
  temperature (Parton-Logan of the climate, plus the offset).
- **Override** (`WeatherTimeline::add_override`, plan 3.6): the preset is the
  target of a window `[t_start, t_end)` with a smoothstep ramp of `ramp_s` in
  and out; the timeline keeps running underneath and is restored bit-exactly
  once the ramp-out ends.

| Key | Required | Notes |
|---|---|---|
| `format` | yes | `"physics_sim.weather_preset/1"` |
| `extends` | no | merge-patch parent, as for climate files |
| `regime` | yes | label, non-empty; matched against the climate's regime names (an unknown label is allowed, `WeatherSample::regime` is then -1) |
| `cloud_cover` | yes | [0, 1] |
| `precip_mm_h` | yes | >= 0, liquid-equivalent rate |
| `precip_type` | yes | `auto` (by air temperature: `precip_phase`), `rain`, `snow`, `sleet`, `freezing_rain`, `drizzle`, `hail` |
| `hail_diameter_mm` | with `hail` only | > 0 (maximum stone diameter) |
| `wind_mean_m_s` | yes | >= 0, 10-min mean at 10 m |
| `wind_dir_deg` | yes | [0, 360], "from" |
| `gust_factor` | yes | >= 1 |
| `temp_offset_k` | no (default 0) | [-50, 50], added to the Parton-Logan temperature |
| `temp_override_c` | no | [-90, 60], pins the air temperature; at most one of the two temperature keys |
| `rh` | yes | [0, 1] (raised during precipitation by the climate's `humidity` rule) |
| `pressure_offset_hpa` | no (default 0) | [-200, 200] on the season's sea-level pressure |
| `lightning_per_min` | no (default 0) | [0, 30]; needs the climate's `lightning` block |
| `source` | no | provenance |

The shipped presets are ASSUMED game presets: each intensity is chosen inside
the class it is named after (`weather_classes.md`); their `source` field says
which class. `tests/unit/test_environment_weather_io.cpp` checks that every one
classifies as named.

Rejections: wrong `format`; unknown key; a missing required key; unknown
`precip_type`; `hail` without `hail_diameter_mm` or the diameter without
`hail`; both temperature keys; any value outside the ranges above.
