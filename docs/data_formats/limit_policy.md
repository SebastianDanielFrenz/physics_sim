# Limit policy format — `physics_sim.limit_policy/1`

A limit policy holds every number that decides **what the player is told or refused** in the engine garage — and nothing a
generator or loader builds from. Owner decision 2026-10-13: *refusal limits and advice thresholds are configuration values,
not code*. The garage warns at Near / Limiting and refuses only at Exceeded; that refusal rule is itself part of the policy
(`refuse_at`).

Shipped: [`data/limit_policies/default.json`](../../data/limit_policies/default.json) — the compiled defaults, entry by entry,
each with its source text. `tests/unit/test_limit_policy.cpp` asserts the file equals the compiled defaults (value and source
of every entry), so the file is the readable copy of the numbers in `core/src/drivetrain/limit_thresholds.h` and
`engine_rules_table.h` (which stay the single place the defaults live in code).

## What it changes — and what it never can

| Class | Meaning | Where it lives |
|---|---|---|
| **POLICY** (61 entries) | decides only the severity of a finding, a build issue, the size of a remedy margin or the refusal | `LimitPolicy` field of the same name, a key of this file |
| **NOT POLICY** (13 entries) | a mathematical constant, or something a generator / loader / grid helper BUILDS from, or anything in a cache key or state hash | stays `constexpr` in `engine_rules_table.h` |

The NOT-POLICY entries are: `valve_layouts` (the list of production layouts), `valve_row_chord_fraction`,
`pack_density_n2` / `n3` / `n4` / `n5` (circle-packing densities), `boost_grid_margin_bar`, `boost_grid_ambient_bar`,
`boost_grid_step_pa`, `boost_grid_min_last_step_pa`, `boost_grid_ambient_node_gap_pa`, `na_grid_top_fraction_of_ambient` (the
grid helpers `boost_grid_p_im` / `ambient_grid_p_im` that decide which maps a generator builds) and `relative_tolerance`
(comparison slack). A policy value **cannot** reach `engine_map_cache_key`, `turbo_table_key`, a `.psmaps` content hash, a
`DynoResult::points` value or a `World` state hash — those entries are not in the struct, and the test guards the API
(`LP6`).

Known limits of the config (not entries, reported): the dyno still has a few constants in code that are not in the threshold
tables — the boost-target sampling plate floor (0.10), the rotor `f > 1.0` Exceeded line, the surge-ratio 1.0 line and the
numeric slack `kEps`; they were hard-coded before L0 and stay so.

## Using a policy

```cpp
#include "ps/io/limit_policy_io.h"

LimitPolicy policy = ps::io::load_limit_policy_json("my_policy.json", "user_layer.json"); // user layer optional
DynoSweepSpec spec;  spec.policy = &policy;          // run_dyno_sweep, and through RemedyRequest::spec solve_remedy
auto findings = check_driveline_limits(dyno, powertrain, clutch_file, gearbox_file, &policy);
auto issues   = check_buildable(engine, &policy);    // also check_gearbox_rating(.., &policy), has_blocking_issue(issues, &policy)
bool refuse   = has_refusing_finding(dyno.limits, &policy);
if (policy.warns(f.severity)) { /* show f */ }       // the garage's advice filter
```

A null pointer (the default of every consumer) is `default_limit_policy()`, the compiled defaults — **bit-identical to the
behaviour before the policy existed** (every existing finding and rule result is unchanged). The pointee must outlive the
call. The engine never drops a finding because of `warn_at` / `refuse_at`: the findings of a sweep are the same for every
policy of the two rules, and `warns()` / `refuses()` / `has_refusing_finding()` / `has_blocking_issue()` are the predicates a
garage UI applies. `LimitPolicy` is a plain value (61 doubles and two enums, no strings): copy it, change a field, pass its
address. Nothing here allocates per tick; no policy value enters a World.

When a policy overrides a number, the `source_label` of a build issue says so (`[policy] <name> = <value> from a limit policy
(default ...; default source: ...)`) instead of repeating the default's citation, which the override no longer has.

## File format

```json
{
  "format": "physics_sim.limit_policy/1",
  "extends": "default.json",
  "refuse_at": "limiting",
  "warn_at": "near",
  "turbo": { "gate_near_opening": 0.90, "rotor_near_fraction": { "value": 0.85, "source": "[AS] my engine file" } },
  "knock": { "knock_near_retard_deg": 0.5 }
}
```

| Key | Meaning |
|---|---|
| `format` | required: `"physics_sim.limit_policy/1"` (checked on the merged result; a user layer may omit it) |
| `source` | optional free text for the reader |
| `extends` | optional path (relative to the file) of a base policy; RFC 7396 merge, bottom-up, depth 8, cycles rejected (`ps::io::load_json_with_extends`) |
| `refuse_at` | severity from which the garage REFUSES: `"exceeded"` (default), `"limiting"`, `"never"` |
| `warn_at` | severity from which a finding is worth telling: `"near"` (default), `"limiting"`, `"exceeded"`, `"never"` |
| group objects | `turbo`, `limiter`, `piston_speed`, `map_grid`, `driveline`, `nitrous`, `knock`, `valves`, `rod`, `compression`, `grid_coverage`, `cam`, `lift`, `gearbox`, `hitch` — each holds entries |
| entry | a number, or `{"value": number, "source": "[C]|[D]|[AS]|[M] text"}` |

A file may be **partial**: every entry it omits keeps the compiled default (a derived file is usually `extends` + the few
entries it changes). The **user layer** (`load_limit_policy_json(path, user_path)`) is merged last, over the whole chain, and
may extend a file of its own; a `null` entry in a user layer deletes the entry from the merge, so the **compiled default**
applies (not the chain's value).

Rejected with the file path and the key (`<path>: <group>/<key>: <what>`; a user layer is named `user layer` or by its path):
an unknown top-level key, group or entry; a value that is not a number; an entry object with a key other than `value` /
`source` or without `value`; a `source` without a `[C]`/`[D]`/`[AS]`/`[M]` tag; a bad `refuse_at` / `warn_at`; a missing
`format`; a value outside its domain (every number is finite and > 0, fractions are <= 1, margins and ratios >= 1) and a
broken ordering (e.g. `gate_near_opening` must be below `gate_full_opening`, `piston_speed_near_mps` below
`piston_speed_limiting_mps`, `kit_lambda_target` at most `kit_lambda_near`). `validate_limit_policy(policy)` gives the same
check for a code-built policy; consumers do not call it (a test may probe with extremes).

## Entries (defaults = `data/limit_policies/default.json`)

Every default carries a tag: **[C]** cited, **[D]** derived, **[AS]** assumed, **[M]** measured. Most are `[AS]` — they are
design thresholds, not physical constants, which is why they are configuration.


### `turbo`

Read by: run_dyno_sweep (turbo window findings: WastegateSaturated, BoostAbove/BelowTarget, TurboRotorCap, CompressorSurge, CompressorChoke); solve_remedy.

| Key | Default | Source of the default |
|---|---|---|
| `gate_near_opening` | 0.92 | [AS] N7 wastegate-sizing margin (design section 18): opening <= 0.92 |
| `gate_full_opening` | 0.98 | [AS] 2 % below the actuator's asymptote |
| `gate_saturated_window_fraction` | 0.5 | [AS] half of the settled window |
| `boost_band_near` | 0.02 | [AS] >> the 20 Pa ripple measured on the hyper (PHYS-058) |
| `boost_band_exceeded` | 0.05 | [AS] 5 % of the target boost |
| `rotor_near_fraction` | 0.9 | [AS] 10 % below the table edge |
| `rotor_limiting_fraction` | 0.97 | [AS] 3 % below the table edge: within the turbine's reach in one more step of load |
| `surge_near_flow_ratio` | 1.1 | [AS] twice TurboPair's 1.05 exit hysteresis |
| `surge_min_window_fraction` | 0.02 | [AS] 2 % of the window (~10 substeps at 960 Hz x 0.5 s) |
| `choke_near_flow_fraction` | 0.9 | [AS] 10 % below the flow capacity |
| `choke_limiting_flow_fraction` | 1 | [D] the head table's normalised flow = 1 (turbo_pair.h) |
| `wastegate_remedy_margin` | 1.1 | [AS] N7b: the bisected area plus 10 % |

### `limiter`

Read by: run_dyno_sweep (RevLimiter, FuelCut).

| Key | Default | Source of the default |
|---|---|---|
| `limiter_near_fraction` | 0.97 | [AS] 3 % below the cut rpm |
| `soft_cut_near_bands` | 2 | [AS] two bands below the zero-fuel rpm |

### `piston_speed`

Read by: run_dyno_sweep (MeanPistonSpeed finding) and check_buildable (MeanPistonSpeed rule).

| Key | Default | Source of the default |
|---|---|---|
| `piston_speed_near_mps` | 23 | [AS] just above the A25A's 22.4 m/s [D] |
| `piston_speed_limiting_mps` | 25 | [AS] F1 2.4 L V8 era peak ~25 m/s, recalled |
| `piston_speed_exceeded_mps` | 28 | [AS] 12 % above the race-engine ceiling |

### `map_grid`

Read by: run_dyno_sweep (MapGridEdge).

| Key | Default | Source of the default |
|---|---|---|
| `grid_edge_min_window_fraction` | 0.05 | [AS] 5 % of the window |
| `grid_edge_near_span_fraction` | 0.02 | [AS] 2 % of the axis span |

### `driveline`

Read by: check_driveline_limits (ClutchCapacity).

| Key | Default | Source of the default |
|---|---|---|
| `driveline_torque_margin` | 1.3 | [AS] design section 18 (owner-visible 'x1.3') |

### `nitrous`

Read by: run_dyno_sweep with a kit (KitFuelLean, N2OBottleDuration, N2OBottlePressureDrop and their Closed remedies).

| Key | Default | Source of the default |
|---|---|---|
| `kit_lambda_near` | 0.9 | [AS] recalled tuning-guide ceiling for a safe wet shot (~12.5:1 gasoline = 0.85, plus margin) |
| `kit_lambda_limiting` | 1 | [AS] stoichiometric: no fuel margin left under the shot |
| `kit_lambda_exceeded` | 1.1 | [AS] 10 % lean of stoichiometric: the shot burns the engine |
| `kit_lambda_target` | 0.82 | [AS] middle of the recalled 0.78-0.85 wet-shot range |
| `bottle_duration_target_s` | 10 | [AS] one drag-strip pass, ~6-12 s recalled |
| `bottle_duration_near_factor` | 1.1 | [AS] Near below target x 1.1: the same 10 % as the Closed remedy reserve, so a bottle sized to the remedy m... |
| `bottle_reserve_factor` | 1.1 | [AS] ~10 % of the liquid stays below the pickup |
| `bottle_flow_retention_near` | 0.9 | [AS] a 10 % flow loss over one pass is visible on the power |
| `bottle_flow_retention_limiting` | 0.8 | [AS] 20 % flow loss over one pass |

### `knock`

Read by: run_dyno_sweep (Knock).

| Key | Default | Source of the default |
|---|---|---|
| `knock_near_retard_deg` | 0.05 | [D] the knock search's own 0.05 deg bisection tolerance (knock.h) |
| `knock_limiting_retard_deg` | 5 | [AS] ~1-3 % torque on the generated eta_spark curves |

### `valves`

Read by: check_buildable (ValveDiameterBand, ExhaustIntakeRatio, ValvePackingArea).

| Key | Default | Source of the default |
|---|---|---|
| `intake_d_over_bore_1v_min` | 0.34 | [AS] Heywood 1988 sec. 6.3 recalled (2-valve heads ~0.4-0.5 of bore), widened by 15 % |
| `intake_d_over_bore_1v_max` | 0.52 | [AS] Heywood 1988 sec. 6.3 recalled; just above the Ford Kent 0.475 (recalled) |
| `intake_d_over_bore_2v_min` | 0.28 | [AS] 20 % below the A25A-FKS 0.36 (repo [AS], Heywood Table 6.2 recalled 0.35-0.37) |
| `intake_d_over_bore_2v_max` | 0.44 | [AS] widened over the hyper 0.413 [M] and the Honda K20A-like 0.407 (recalled) |
| `intake_d_over_bore_3v_min` | 0.22 | [AS] 5-valve heads (3 intake) recalled ~0.28-0.33, widened |
| `intake_d_over_bore_3v_max` | 0.34 | [AS] 5-valve heads recalled ~0.28-0.33, widened |
| `exhaust_over_intake_min` | 0.78 | [AS] design section 6.2 (production range 0.78-0.90) |
| `exhaust_over_intake_max` | 0.9 | [AS] design section 6.2; the hyper (34/38 = 0.895) is the closest shipped engine |
| `pack_share_of_ideal` | 0.95 | [AS] the hyper packs 0.89 of the ideal 4-circle density [M]; 0.95 leaves it 6 % of room and still refuses v... |

### `rod`

Read by: check_buildable (RodRatio).

| Key | Default | Source of the default |
|---|---|---|
| `rod_over_crank_radius_min` | 3 | [AS] design section 6.3; Heywood sec. 2.1 recalls R = 3-4 for small and medium engines. The Toyota A25A-FKS... |
| `rod_over_crank_radius_max` | 4.2 | [AS] design section 6.3 |

### `compression`

Read by: check_buildable (CompressionRatio).

| Key | Default | Source of the default |
|---|---|---|
| `cr_na_min` | 8 | [AS] production gasoline engines since the 1980s run 8-14 |
| `cr_na_max` | 14.5 | [AS] Mazda SkyActiv-G 14.0 (recalled); the A25A-FKS runs 13.0 [C] SAE 2019-01-0249 via the repo data |
| `cr_forced_min` | 7 | [AS] low-compression turbo engines (recalled 7.5-8.5) |
| `cr_forced_max` | 10.5 | [AS] design section 6.3: no knock model, so forced induction is limited to <= 10.5 |

### `grid_coverage`

Read by: check_buildable (LimiterAboveRpmGrid, BoostAbovePressureGrid).

| Key | Default | Source of the default |
|---|---|---|
| `grid_overshoot_limiting` | 0.05 | [AS] 5 % past the last node: clamped numbers start to matter (the shipped demo I4: limiter 6800 over a 6500... |
| `grid_overshoot_exceeded` | 0.15 | [AS] 15 % past the last node |

### `cam`

Read by: check_buildable (CamSwitchWindow, CamProfileOrder, CamLiftOverValve).

| Key | Default | Source of the default |
|---|---|---|
| `cam_switch_over_limiter_min` | 0.45 | [AS] Honda VTEC 0.60-0.72 of the limiter (B16A, K20A DC5/FD2, F20C; sources in the comment above), widened |
| `cam_switch_over_limiter_max` | 0.85 | [AS] Honda VTEC 0.60-0.72 of the limiter, widened; above it the high cam serves a sliver of the band |
| `cam_switch_hysteresis_min_rpm` | 50 | [AS] below ~50 rpm a WOT pull's speed ripple toggles the solenoid (no OEM hysteresis found) |
| `cam_switch_hysteresis_max_rpm` | 1000 | [AS] beyond ~1000 rpm the engine runs the wrong cam for a large part of a lift (no OEM hysteresis found) |
| `cam_peak_lift_over_valve_d_max` | 0.4 | [AS] 0.35 seen on an aftermarket K20 race cam (junauto.co.jp), widened; beyond it more lift adds no flow ar... |

### `lift`

Read by: check_buildable (LiftFamilyOrder, LiftMinimumBand).

| Key | Default | Source of the default |
|---|---|---|
| `lift_min_node_peak_min_mm` | 0.1 | [AS] below ~0.1 mm the valve barely leaves its seat (BMW's second-generation minimum is 0.18 mm, fenderbend... |
| `lift_min_node_peak_max_fraction_of_full` | 0.25 | [AS] BMW N52 0.18 of 9.9 mm = 0.018 (fenderbender.com; mymotorlist.com); above a quarter of full lift the t... |

### `gearbox`

Read by: check_buildable / check_gearbox_rating (GearboxRating), check_driveline_limits (GearboxInputTorque and its Closed remedy).

| Key | Default | Source of the default |
|---|---|---|
| `gearbox_rating_over_limiting` | 0.05 | [AS] transmission plan 7.1: 5 % over the rating; a gearbox is rated with margin, so the first few percent o... |
| `gearbox_rating_over_exceeded` | 0.15 | [AS] transmission plan 7.1 / owner 2026-10-13 (Q7): more than 15 % over the rating is refused |
| `gearbox_rating_remedy_margin` | 1.1 | [AS] transmission plan 7.1: required rating = peak x 1.1 |

### `hitch`

Read by: check_hitch_limits, World::hitch_limits (TowedMass, TongueLoad). The share of a towing point's rating at which a finding is Near, Limiting, Exceeded.

| Entry | Default | Source |
|---|---|---|
| `hitch_near_share` | 0.9 | [AS] the last 10 % below a rating |
| `hitch_limiting_share` | 1 | [D] the rating itself |
| `hitch_exceeded_share` | 1.15 | [AS] the gearbox rating's 15 % over |

## Tests

`tests/unit/test_limit_policy.cpp` (`[limit_policy][drivetrain][io]`):

- `LP1` default.json equals the compiled defaults, value and source of all 61 entries; the 13 NOT-POLICY names are in no file.
- `LP2a` / `LP2b` / `LP2c` every one of the 61 entries is reachable: set to an extreme through a policy it changes a finding,
  build issue or remedy of at least one consumer; an entry no consumer reads fails, naming it. `LP2b` is `[long]` (hyper
  sweeps, one wastegate solve).
- `LP3` `refuse_at` / `warn_at` matrices, `has_blocking_issue`, `has_refusing_finding`, findings independent of both.
- `LP4` / `LP5` extends, user layer, null deletes, partial files; every rejection.
- `LP6` a policy does not move a map key, a turbo key, a dyno point or a World state hash.
