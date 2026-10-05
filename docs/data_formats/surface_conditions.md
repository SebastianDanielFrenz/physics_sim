# `physics_sim.surface_conditions/1` - surface condition classes (weather plan W5/W6)

Loader: `ps::io::load_surface_conditions_json(path)` (`core/include/ps/environment/surface_conditions_io.h`,
library `ps_environment_io`, no `ps_core`). Descriptor: `ps/environment/surface_condition_desc.h`
(`SurfaceConditionsDesc`, `SurfaceClassDesc`, `ThermalLayerDesc`, `SoilDesc`, `SurfaceFrictionDesc`).
Model: `ps/environment/surface_condition.h` (`build_column`, `step_column`, `ConditionGrid`) and the
friction factor `ps/environment/condition_grip.h` (`ConditionGrip`). Example:
`data/surface_conditions/default.json` (the classes of `data/surfaces/surfaces.json`).

W5 implements the THERMAL part: one surface temperature state per surface class,
driven by the environment's sun and weather. W6 adds the WATER part: a film depth
`W` above the texture, drainage, evaporation/dew, soil infiltration/ponding and a
vegetated canopy store, plus the dry/damp/wet/wet_grass friction levels the
condition grip turns into a `lambda_mu` factor (see [`tyre_profile.md`](tyre_profile.md)).
Ice, frost, snow and mud (W8-W9) still exist as zero placeholders in the state and
the snapshot; their file keys stay reserved (see "Rejections").

## File

```json
{
  "format": "physics_sim.surface_conditions/1",
  "extends": "optional/parent.json",
  "surfaces": {
    "asphalt": {
      "kind": "sealed",
      "albedo": 0.10,
      "emissivity": 0.90,
      "layer": { "depth_m": 0.20, "k_w_mk": 1.38, "rho_c_j_m3k": 2.0e6 },
      "deep":  { "depth_m": 1.80, "k_w_mk": 1.0,  "rho_c_j_m3k": 2.2e6 },
      "mtd_mm": 0.8,
      "drainage": { "path_m": 7.3, "slope": 0.025 },
      "friction": { "damp": 0.85, "wet": 0.71, "wet_grass": 0.67 },
      "source": { "albedo": "[C] ..." }
    },
    "grass": {
      "kind": "vegetated",
      "albedo": 0.22, "emissivity": 0.95,
      "layer": { "depth_m": 0.10, "k_w_mk": 0.6, "rho_c_j_m3k": 1.6e6 },
      "deep":  { "depth_m": 0.90, "k_w_mk": 1.2, "rho_c_j_m3k": 2.0e6 },
      "soil": { "ksat_mm_h": 15.0, "theta_fc": 0.25, "theta_sat": 0.45 },
      "canopy": { "capacity_mm": 0.5 }
    },
    "low_mu": { "kind": "static" }
  },
  "source": "free-form provenance"
}
```

| Key | Type | Notes |
|---|---|---|
| `format` | string, required | exactly `"physics_sim.surface_conditions/1"` |
| `extends` | string, optional | parent file, RFC 7396 merge patch like the climate files; a `null` surface entry removes an inherited class |
| `surfaces` | object, required, non-empty | class name -> class object; the name is the surface table's name |
| `source` | string or object | provenance; mark every number [C] cited, [D] derived, [M] from memory of a named source, [AS] assumed |

Class object:

| Key | Type | Notes |
|---|---|---|
| `kind` | string, required | `sealed` (asphalt, concrete), `soil`, `vegetated`, `static` (no state: an ice rink, an indoor floor) |
| `albedo` | real in [0, 1), required unless static | shortwave reflectance |
| `emissivity` | real in (0, 1], required unless static | longwave emissivity (also its absorptivity) |
| `layer` | object, required unless static | the top slab: `depth_m` > 0, `k_w_mk` > 0 (conductivity W/(m K)), `rho_c_j_m3k` > 0 (volumetric heat capacity J/(m3 K)), optional `source` |
| `deep` | object, required unless static | the slab below, same keys; `layer.depth_m + deep.depth_m` <= 10 m is the column depth |
| `mtd_mm` | real > 0, required for `sealed` | mean texture depth; the texture storage `W_tex = texture_storage_mm` when given, else `0.5 * mtd_mm` |
| `texture_storage_mm` | real > 0, optional for `sealed` | explicit texture storage (omit to derive it from `mtd_mm`) |
| `drainage` | object, required for `sealed` | `path_m` > 0 (drainage path length), `slope` > 0 (m/m); feeds the Gallaway steady film |
| `soil` | object, required for `soil`/`vegetated` | `ksat_mm_h` > 0 (saturated conductivity), `theta_fc` and `theta_sat` volumetric, `0 < theta_fc < theta_sat <= 1` |
| `canopy` | object, required for `vegetated` | `capacity_mm` > 0 (interception store) |
| `friction` | object, optional for non-static | `damp`/`wet`/`wet_grass` in (0, 2], relative to the class's own dry level (defaults 0.85 / 0.71 / 0.67); the W8/W9 condition keys stay reserved |
| `source` | string or object | provenance |

A static class takes none of the fields above.

## Class list (which `SurfaceId` is which class)

The grid's classes are the World's surface table (`data/surfaces/surfaces.json`,
`SurfaceId` = alphabetical order): index `i` is the table's name `i`. A table name
without an entry here is a static class (listed by `ConditionGrid::unmapped_names()`);
an entry no table name uses is ignored (`unused_entries()`). At most 32 classes.
`World::install_environment` rejects an environment whose class names differ from
the world's table (order included). Without a table (a code-built Environment)
the file's classes are used in name order.

## Model

Per stateful class a one-dimensional vertex-centred finite-volume conduction
column: node 0 on the surface, a 5 mm top cell, cells growing by 1.3 per node,
the `layer`/`deep` boundary is a node, at most 32 nodes, the bottom node held at the climate's per-day `ground_temp_c` (Dirichlet).
Backward Euler in time with a Thomas solve (a dense Gaussian elimination is the
test reference).

Surface energy balance at node 0 (W/m2, positive into the surface):

- shortwave `(1 - albedo) * GHI`, GHI from the sun elevation and the weather's
  cloud cover (Haurwitz clear sky, Kasten-Czeplak cloud factor - climate.md);
- longwave in `emissivity * eps_sky * sigma * T_air^4`, clear-sky emissivity
  `0.77 - 0.28 * 10^(-0.074 p)` (p = vapour pressure in mm Hg; Geiger, as used by
  Solaimanian & Kennedy 1993 [C]), raised by cloud to `N + (1 - N) eps_clear`
  [M Crawford & Duchon 1999];
- longwave out `emissivity * sigma * T_s^4`, linearised about the previous
  surface temperature;
- convection `h_c (T_air - T_s)`, `h_c = 698.24 [0.00144 T_m^0.3 U^0.7 + 0.00097 (T_s - T_air)^0.3]`
  (W/(m2 K), T_m the mean of surface and air in K, U the wind in m/s; Vehrencamp
  1953 via Solaimanian & Kennedy 1993 [C]); the free-convection term only while
  the surface is warmer than the air [AS]; `h_c` is taken at the previous state.

### Water part (W6)

One step updates the mass after the thermal solve, in this order: rain input ->
canopy interception (vegetated) -> soil infiltration (soil/vegetated) -> the rest
is added to the film `W` -> evaporation or dew -> drainage.

- **Rain**: `precip_mm_h / 3600 * dt` of liquid water (snow is W8; the
  environment passes 0 for a snow sample).
- **Canopy** (vegetated): fills up to `canopy.capacity_mm` first.
- **Infiltration** (soil/vegetated): at most `ksat_mm_h / 3600 * dt` and at most
  the pore space `(theta_sat - theta) * layer.depth_m * 1000` mm (theta is volumetric, so
  no further porosity factor; the top soil layer is the `layer` slab, 0.1 m in the shipped
  file [AS]); the rest stays on the surface. A soil class starts at `theta_fc` [AS].
- **Evaporation/dew**: the Dalton bulk flux `E = h_m (rho_v,sat(T_s) - rho_v,air)`
  with `h_m = h_c / (rho cp)` (Lewis analogy, Le = 1 [AS], `rho cp` = 1.2 * 1005),
  `e_sat` the Magnus form `6.1094 exp(17.625 Tc / (Tc + 243.04))` hPa [C Bolton 1980, the
  same function the weather's dew point uses],
  `rho_v = e_pa * 0.018016 / (8.314462618 T)`. Positive `E` evaporates (drawn from
  `W`, then canopy, then the soil surface scaled by `theta/theta_fc`), limited by
  the available water over one step; negative `E` is dew, capped at 0.36 mm/h and
  zeroed below 0 C (liquid only; frost is W8). The latent heat `L_v E` (with
  `L_v = 2.501e6 - 2370 (T_s - 273.15)`) is folded into the surface energy balance
  as a lagged constant, so the step stays one linear solve.
- **Drainage** (`sealed`): the Gallaway et al. 1979 (FHWA-RD-79-31) steady film
  above the texture, metric form
  `WFD = 0.01485 mtd^0.11 path^0.43 I^0.59 / slope^0.42 - mtd` (mm, clamped at 0),
  with `I` the rain in mm/h, capped at 150 mm/h [AS, owner Q17: >= 100] inside the
  formula (the tau below still uses the real rain). While it rains, `tau = WFD / (I/3600)` and the water
  above the texture storage relaxes to `WFD` by backward Euler
  (`W <- W_tex + (W - W_tex) / (1 + a)`, `a = dt/tau`, only while `W > W_tex`; water
  below the storage never drains, it only evaporates); the fixed point under the rain
  input is `R tau = WFD`, exact. Once the rain stops (`I < 0.1 mm/h`) `tau` is frozen
  (the state carries `drain_tau_s`) and free water above `W_tex` drains at that rate
  (`tau = 0`: down to `W_tex` in one step).
- **Drainage** (soil/vegetated): gravity water above `theta_fc` relaxes with a
  3-day [AS] time constant and any ponded `W` with a 1 h [AS] constant.

The published wet weight of a class (the condition grip and the snapshot's
`condition`): sealed `W / W_tex`; soil/vegetated
`(theta - theta_fc) / (theta_sat - theta_fc)`, at least `min(W / 1 mm, 1)` while
ponded (a ramp over 1 mm [AS], not a switch); the vegetated grass weight is
`max(canopy/capacity, wet weight)`. Dew on turf wets the canopy first.

## Grid (climate `condition` block)

The state lives on a grid of the environment time `t`: index `k = floor(t / dt_env_s)`,
the grid holds `S[k]` and `S[k+1]`; a step `S[k+1] -> S[k+2]` uses the weather and
sun at `(k + 1.5) dt`. The published surface temperature is the linear
interpolation between the two at `t`. A forward move of up to `max_fast_forward_s`
steps through every grid point (bit-identical to having run there tick by tick);
the first start, a longer forward jump and any backward jump spin up: a linear
initial profile (the day's `temp_mean_c` at the surface, `ground_temp_c` at the
bottom) at `k - spinup_s / dt_env_s`, stepped to `k`. Defaults and ranges:
[`climate.md`](climate.md) "Surface condition grid (W5)". Stepping runs inside the
World's `freeze_tick_start_state` stage and allocates nothing.

Checkpoint/restore (`Environment::checkpoint_conditions` / `restore_conditions`):
`k` plus both states, fixed-size plain data, restored bit-exactly. Rejected: a
checkpoint that lies ahead of the clock, one more than `max_fast_forward_s` behind
it, and one whose class count or `dt_env_s` differs.

Hash: while an environment with surface conditions is installed, `k`, the class
count and kinds and every stateful class's `S[k]` and `S[k+1]` (nodes, water,
ice/frost/snow placeholders, soil moisture, canopy water and the frozen drainage
`drain_tau_s`) are folded into `World::state_hash()`; without surfaces the hash is
unchanged. The snapshot (`EnvironmentSnapshot::surfaces`, see
[`scenario.md`](scenario.md)) is not hashed.

## Condition grip (W6)

`ps/environment/condition_grip.h` (`ConditionGrip`) turns each class's state into
the `lambda_mu` multiplier a wheel contact uses; `SurfaceTable::lambda_mu` stays
the absolute dry level and the factor is relative to it (plan 5.2). Built once per
grid (one `ps::math::exp` per class and speed sample), updated per tick, no
allocation. Per class: a wet speed table
`table(v) = min(wet * exp((v_ref - v) / Sp), damp)` with `Sp = 14.2 + 89.7 mtd`
km/h [C PIARC 1995] and `v_ref` = 60 km/h, for sealed classes only (a soil/vegetated
class has no MTD: its wet level is flat in speed, the mud curve is W9), interpolated
linearly between the 33 samples and evaluated at the contact-patch speed; non-vegetated
`f = (1 - w) dry_tyre + w table(v) wet_tyre`, vegetated
`f = (1 - g) dry_tyre + g wet_grass * wet_grass_tyre`; `w`/`g` are the wet/grass
weights above and `*_tyre` the tyre profile's per-condition factors
([`tyre_profile.md`](tyre_profile.md)). A static class, an out-of-range id or no
environment gives exactly 1. W7 adds the tread-depth factor, W8 aquaplaning.

## Validation (measured, `tests/unit/test_environment_surface.cpp`)

- SHRP (FHWA-RD-97-103 eq. 3) surface maximum on a clear summer day at 50 N:
  42.40 C vs 41.37 C (+1.03 K; band 3 K).
- Kallas 1966 (HRR 150, College Park MD, June 30 1964): diurnal range 36.69 vs
  36.11 K (+1.6 %), time of maximum 14.88 vs 14.62 h, 12 in hourly rms 1.11 K;
  the night minimum is 20.5 vs 25.0 C (-4.5 K, an open `[!mayfail]` with its
  measurement in the test).
- dt 60 s vs 15 s: 0.033 K.
- W6-G1: a sealed steady film equals `W_tex + WFD` to 1e-9 (the Gallaway fixed
  point) and the frozen `drain_tau_s` is `WFD / (I/3600)`.
- W6-G2: the flux equals an independent hand evaluation (h_c 17.17 W/(m2 K): 1.12 mm/h at
  T_s = T_a = 300 K, 5 m/s, 5 hPa), drying removes exactly the integrated flux to 1e-9 and a
  0.4 mm film dries in 27 min (600 W/m2 sun, 5 hPa) / 55 min (100 W/m2, 14 hPa).
- W6-G8: the latent heat enters the surface node (dense reference, 1010 W/m2 at 1.507 mm/h).
- W6-G9: soil/turf interception, infiltration limit, ponding, drain and the mass closure.
- W6-G7: the grip wet weight, the PIARC speed table, soil flat in speed and the fallbacks.
- W6-G3 (World, `test_environment_surface_world.cpp`): wet vs dry threshold braking, car_sedan
  on a tagged asphalt mesh, no ABS, 0.8 s pedal ramp: distance ratio 1.40 from 20 m/s
  (Wong 1.14-1.8, PIARC bracket 1.26-1.62) and 1.86 from 30 m/s.
- W6-G3 (World level, `test_environment_surface_world.cpp`): wet vs dry asphalt
  braking distance ratio inside the Wong band [1.14, 1.8].

## Rejections (`std::runtime_error`, message prefixed with the file)

Wrong or missing `format`; unknown key anywhere; `surfaces` missing, not an
object or empty; an empty class name; an unknown `kind`; a field on a static
class; a missing required field; albedo outside [0, 1), emissivity outside
(0, 1]; a layer value <= 0 or not finite; a column deeper than 10 m or needing
more than 32 nodes; a `sealed` class without `mtd_mm`/`drainage` or with
`drainage.path_m`/`slope` <= 0; `texture_storage_mm` <= 0; a `soil`/`vegetated`
class without valid `soil` fields or (vegetated) without `canopy`; `mtd`/texture
storage on a soil class; a friction level outside (0, 2]; an `extends` error.
Keys of later increments are rejected by name with "is part of
physics_sim.surface_conditions/1 but is not implemented before ...": `crr` (W8),
`abrasiveness` (W10), `maintenance` (later) and the `friction` condition keys
`slush`, `snow_loose`, `snow_packed`, `ice`, `wet_ice`, `frost`, `black_ice`,
`mud` (W8/W9).
