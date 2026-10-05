# `physics_sim.surface_conditions/1` - surface condition classes (weather plan W5)

Loader: `ps::io::load_surface_conditions_json(path)` (`core/include/ps/environment/surface_conditions_io.h`,
library `ps_environment_io`, no `ps_core`). Descriptor: `ps/environment/surface_condition_desc.h`
(`SurfaceConditionsDesc`, `SurfaceClassDesc`, `ThermalLayerDesc`). Model:
`ps/environment/surface_condition.h` (`build_column`, `step_column`, `ConditionGrid`). Example:
`data/surface_conditions/default.json` (the classes of `data/surfaces/surfaces.json`).

W5 implements the THERMAL part only: one surface temperature state per surface
class, driven by the environment's sun and weather. Water film, ice, frost, snow,
soil moisture and canopy water (W6-W8) exist as zero placeholders in the state
and the snapshot; their file keys are reserved (see "Rejections"). Nothing in the
vehicle physics reads the state yet (tyres consume it from W6 on).

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
      "source": { "albedo": "[C] ..." }
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
| `source` | string or object | provenance |

A static class takes none of the thermal fields.

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

No latent heat (evaporation, melt) in W5: grass and soil run warmer than real in
sunshine until W6.

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
count and kinds and every stateful class's `S[k]` and `S[k+1]` (nodes and
placeholders) are folded into `World::state_hash()`; without surfaces the hash is
unchanged. The snapshot (`EnvironmentSnapshot::surfaces`, see
[`scenario.md`](scenario.md)) is not hashed.

## Validation (measured, `tests/unit/test_environment_surface.cpp`)

- SHRP (FHWA-RD-97-103 eq. 3) surface maximum on a clear summer day at 50 N:
  42.40 C vs 41.37 C (+1.03 K; band 3 K).
- Kallas 1966 (HRR 150, College Park MD, June 30 1964): diurnal range 36.69 vs
  36.11 K (+1.6 %), time of maximum 14.88 vs 14.62 h, 12 in hourly rms 1.11 K;
  the night minimum is 20.5 vs 25.0 C (-4.5 K, an open `[!mayfail]` with its
  measurement in the test).
- dt 60 s vs 15 s: 0.033 K.

## Rejections (`std::runtime_error`, message prefixed with the file)

Wrong or missing `format`; unknown key anywhere; `surfaces` missing, not an
object or empty; an empty class name; an unknown `kind`; a thermal field on a
static class; a missing thermal field on a stateful class; albedo outside
[0, 1), emissivity outside (0, 1]; a layer value <= 0 or not finite; a column
deeper than 10 m or needing more than 32 nodes; an `extends` error. Keys of later
increments are rejected by name with "is part of physics_sim.surface_conditions/1
but is not implemented before ...": `mtd_mm`, `drainage`, `texture_storage_mm`
(W6 water film), `canopy` (W6), `friction` (W6 condition grip), `crr` (W8),
`soil` (W9 mud), `abrasiveness` (W10 tyre wear), `maintenance` (later).
