# `physics_sim.tyre_profile/1` - tyre profiles for the condition grip (weather plan W6)

Loader: `ps::io::load_tyre_profile_json(path)` (`core/include/ps/environment/tyre_profile_io.h`,
library `ps_environment_io`, no `ps_core`). Descriptor:
`ps/environment/tyre_profile_desc.h` (`TyreProfileDesc`, `TyreConditionFactors`,
`TyreTreadExponents`, `TyreClass`). Consumers: `ps/environment/condition_grip.h`
(`ConditionGrip`) and the environment's `default_tyre_profile`. Shipped profiles:
`data/tyre_profiles/*.json`.

This is the W6 tyre model (weather plan 5.1, 5.2): a small per-condition friction
multiplier relative to the reference summer tyre, used with the surface condition
to shape a wheel's `lambda_mu`. It is NOT the `physics_sim.tyre/1` Magic-Formula
`TyreDesc` of [`tyre.md`](tyre.md) (the `.tyre` files of a vehicle): that stays the
per-wheel force model, this describes the tyre's response to the surface state.

W6 uses `condition_factors` (dry/damp/wet/wet_grass), the tread limits and the
inflation pressure. `tread_exponents` is carried for W7 (tread-depth factor), and
`thermal`/`wear` are W9/W10: the loader accepts `null` for them and rejects an
object by name until then.

## File

```json
{
  "format": "physics_sim.tyre_profile/1",
  "class": "summer",
  "tread_new_mm": 8.0,
  "tread_min_mm": 1.6,
  "groove_void_ratio": 0.30,
  "siping": "low",
  "inflation_pressure_kpa": 220.0,
  "condition_factors": { "dry": 1.0, "damp": 1.0, "wet": 1.0, "wet_grass": 1.0 },
  "tread_exponents": { "wet": 0.5, "snow_packed": 1.0 },
  "thermal": null,
  "wear": null,
  "source": { "condition_factors": "[AS] ..." }
}
```

| Key | Type | Notes |
|---|---|---|
| `format` | string, required | exactly `"physics_sim.tyre_profile/1"` |
| `class` | string, required | one of `slick`, `summer`, `all_season`, `winter`, `winter_studded`, `offroad_at`, `mud_terrain`, `agricultural_lug` |
| `tread_new_mm` | real, required | new tread depth, > 0 |
| `tread_min_mm` | real, required | legal minimum, in [0, `tread_new_mm`] |
| `groove_void_ratio` | real, required | groove void fraction, in [0, 1] (inclusive) |
| `siping` | string, optional | `none` (default), `low` or `high` |
| `inflation_pressure_kpa` | real, required | reference pressure, > 0 |
| `condition_factors` | object, required | `dry`, `damp`, `wet`, `wet_grass`, each > 0 and finite, default 1.0; relative to the reference summer tyre. The W8/W9 keys (`slush`, `snow_loose`, `snow_packed`, `ice`, `wet_ice`, `frost`, `black_ice`, `mud`) are reserved |
| `tread_exponents` | object, optional | `wet` (default 0.5), `snow_packed` (default 1.0), each > 0; W7 reads them |
| `thermal` | `null`, optional | reserved for W9; an object is rejected |
| `wear` | `null`, optional | reserved for W10; an object is rejected |
| `source` | string or object | provenance; mark every number [C] cited, [D] derived, [M] from memory of a named source, [AS] assumed |

Every object (top level, `condition_factors`, `tread_exponents`) also accepts an
optional `source`.

## How W6 uses it (condition grip, plan 5.2)

`ConditionGrip` combines a tyre profile's factors with a class's wetness. For a
sealed class with wet weight `w` and speed `v` (km/h), the wet friction is
`table(v) = min(wet * exp((v_ref - v) / Sp), damp)` with `v_ref` = 60 km/h and
`Sp = 14.2 + 89.7 mtd` km/h [C PIARC 1995], and the factor is
`f = (1 - w) dry + w table(v) wet`. A vegetated class uses
`f = (1 - g) dry + g wet_grass * wet_grass_tyre` with the grass weight `g` and no
speed table. A static class or no environment gives exactly 1.

## Shipped profiles (`data/tyre_profiles/`)

| File | class | tread new/min mm | void | siping | kPa | dry/damp/wet/wet_grass |
|---|---|---|---|---|---|---|
| `slick.json` | slick | 4.0 / 1.6 | 0.05 | none | 220 | 1.15 / 0.75 / 0.50 / 0.50 |
| `summer_generic.json` | summer | 8.0 / 1.6 | 0.30 | low | 220 | 1.0 / 1.0 / 1.0 / 1.0 |
| `all_season.json` | all_season | 8.5 / 1.6 | 0.32 | low | 220 | 0.97 / 1.0 / 1.0 / 1.0 |
| `winter.json` | winter | 9.0 / 1.6 | 0.35 | high | 230 | 0.95 / 1.05 / 1.0 / 1.05 |
| `offroad_at.json` | offroad_at | 12.0 / 2.0 | 0.40 | low | 240 | 0.95 / 1.0 / 1.05 / 1.15 |
| `agricultural_lug.json` | agricultural_lug | 25.0 / 5.0 | 0.55 | none | 160 | 0.90 / 1.0 / 1.10 / 1.25 |

`summer_generic` is the reference (all factors 1.0) and the environment default.
The values are `[AS]` (assumed) with their provenance in each file's `source`.

## Vehicle wiring

A vehicle may name a profile per wheel (`wheels[i].tyre_profile`, see
[`vehicle.md`](vehicle.md)); W6 resolves every wheel through the environment's
`default_tyre_profile` and documents the per-wheel lookup as deferred to W7.

## Validation (measured, `tests/unit/test_environment_surface.cpp`, W6-G5)

The six shipped profiles load with their expected class and wet/wet_grass factors,
and the ten rejection rows fire: wrong `format`, unknown `class`, unknown
top-level key, `tread_min_mm` above `tread_new_mm`, `groove_void_ratio` above 1,
bad `siping`, zero `inflation_pressure_kpa`, a reserved `condition_factors` key,
and an object (not `null`) for `thermal` / `wear`.

## Rejections (`std::runtime_error`, message prefixed with the file)

Wrong or missing `format`; unknown key anywhere; unknown `class`; missing
`tread_new_mm`/`tread_min_mm`/`groove_void_ratio`/`inflation_pressure_kpa`;
`tread_new_mm` <= 0; `tread_min_mm` outside [0, `tread_new_mm`];
`groove_void_ratio` outside [0, 1]; `siping` not `none`/`low`/`high`; a
`condition_factors` value not > 0; a `tread_exponents` value not > 0; an object
for `thermal` ("... not implemented before W9 (tyre thermal); use null") or
`wear` (W10); a reserved `condition_factors` key ("... not implemented before
W8/W9 (snow, ice, mud grip)").
