# Gearbox format — `physics_sim.gearbox/1`

Files: `data/gearboxes/*.json`, **or** inline inside a vehicle file's
`powertrain.components[]` entry with `"type": "gearbox"` (same fields, same
validation — see `vehicle.md`'s ["Components"](vehicle.md#components)
section). Loader: `read_gearbox_fields` (`core/src/io/vehicle_io.cpp`).
Public descriptor: `ps::drivetrain::GearboxDesc`
(`core/include/ps/drivetrain/powertrain_desc.h`).

A gearbox is a list of forward and reverse ratios (indexed 1..N and
-1..-M), each with a mesh efficiency and an optional synchro capacity, plus
timing constants for the shift itself (neutral-phase dwell, grind/abort
thresholds).

## Fields

| Field | Type | Req./opt. | Default | Validation | Meaning |
|---|---|---|---|---|---|
| `format` | string | required (standalone file only) | — | must equal `"physics_sim.gearbox/1"` | see README |
| `type` | string | optional | `"manual"` | `"manual"`, `"sequential"`, `"automatic"`, `"dual_clutch"` or `"cvt"`, else `"<ctx>: gearbox 'type' must be \"manual\", \"sequential\", \"automatic\", \"dual_clutch\" or \"cvt\", got \"<type>\""` | selector behaviour (manual = H-pattern with neutral between every gear; sequential = up/down through an ordered list, e.g. a tractor range-and-gear box flattened into one sequence; automatic = torque-converter planetary box, see [Automatic and dual-clutch gearboxes](#automatic-and-dual-clutch-gearboxes); dual_clutch = two wet clutches on odd/even shafts, same section; cvt = push-belt variator, see [CVT gearboxes](#cvt-gearboxes)) |
| `forward_ratios` | array of real | **required** (rejected on a `cvt`, which fills it from `cvt.steps`) | — | must be present (`"<ctx>: 'forward_ratios' is required"`); every ratio `> 0`; **strictly decreasing**, 1st gear first (`"<ctx>.forward_ratios: must be strictly decreasing (1st first)"`) | `omega_in = ratio * omega_out` per forward gear, 1st listed first |
| `reverse_ratios` | array of real | optional | empty | every ratio `< 0` (`"<ctx>.reverse_ratios: every ratio must be < 0"`) | reverse ratios, R1 (slowest) first |
| `forward_efficiency` | array of real | **required** (rejected on a `cvt`, filled from `cvt.efficiency_vs_ratio` at each step) | — | must be present (`"<ctx>: 'forward_efficiency' is required"`); **same length** as `forward_ratios` (`"<ctx>: forward_efficiency must have the same length as forward_ratios"`); every value in `(0, 1]` (`"<ctx>.forward_efficiency: every value must be in (0, 1]"`) | mesh efficiency per forward gear |
| `reverse_efficiency` | array of real | optional (required if `reverse_ratios` non-empty) | empty | **same length** as `reverse_ratios` (`"<ctx>: reverse_efficiency must have the same length as reverse_ratios"`); every value in `(0, 1]` | mesh efficiency per reverse gear |
| `input_inertia_kgm2` | real (kg·m²) | optional | `0` | none | inertia at the `in` port |
| `output_inertia_kgm2` | real (kg·m²) | optional | `0` | none | inertia at the `out` port |
| `input_axis_local` | Vec3 | optional | `[0,0,0]` | zero or unit length, else `"<ctx>.input_axis_local must be zero or a unit vector, got length <len>"` | rotor-reaction axis for `in` |
| `output_axis_local` | Vec3 | optional | `[0,0,0]` | zero or unit length, same rule | rotor-reaction axis for `out` |
| `synchro` | object, see [below](#synchro) | optional | zero capacity every gear (dog-only) | see below | per-gear synchro capacity and shift-quality parameters |
| `neutral_phase_s` | real (s) | optional | `0.06` | none | minimum dwell time in neutral (selector travel through neutral) before a new gear may begin synchronising |
| `dog_release_torque_nm` | real (N·m) | optional | `60.0` | none | `\|mesh torque at input\|` must drop below this before a gear can be pulled out |
| `grind_after_s` | real (s) | optional | `0.25` | none | synchronising for longer than this produces a "grind" event |
| `abort_after_s` | real (s) | optional | `1.0` | none | synchronising for longer than this gives up and returns to neutral |
| `initial_gear` | int | optional | `0` | must be within `[-reverse_ratios.size(), forward_ratios.size()]`, else `"<ctx>: initial_gear is out of range"` | starting position; `0` = neutral, `k > 0` = forward gear k, `-k` = reverse gear Rk |
| `rated_input_torque_nm` | real (N·m) | optional | `0` (= unrated, never checked) | number `> 0`, else `"<ctx>: rated_input_torque_nm must be a number > 0"` | the largest steady input torque the box is built for (transmission plan T0). Compared with the engine's peak net torque (boosted, incl. a nitrous kit when the sweep has one): see [Rating check](#rating-check) |
| `gear_type` | string | optional | `"helical"` | `"helical"` or `"straight_cut"`, else `"<ctx>: gear_type must be \"helical\" or \"straight_cut\""` | tooth form; sound only (straight-cut = gear whine), published as `DrivelineSoundState::gear_type` (0/1). Never hashed |
| `teeth` | object, see [below](#teeth) | optional | no tooth data | see below | tooth counts for the mesh frequencies. Sound only |
| `mesh_order` | real | optional | `0` (unset) | number `> 0`, else `"<ctx>: mesh_order must be a number > 0"` | fallback mesh order (teeth per input revolution) used where no tooth counts exist |

A gearbox **part file** (`ref`) may carry `"extends": "<path>"` (relative to
the file; RFC 7396 merge patch, chains up to depth 8, cycles rejected — the
same rule as the engine-family parts). `extends` inside an inline gearbox
component is rejected (`"'extends' is only available in a gearbox part file
(referenced with 'ref')"`); a vehicle can swap a gearbox through the user
layer (`VehicleLoadOptions::part_overrides["gearbox"]`).

### `teeth`

| Key | Type | Validation | Meaning |
|---|---|---|---|
| `forward` | array of `{drive, driven}` | same length as `forward_ratios` (`"teeth.forward must have the same length as forward_ratios"`) | tooth counts of each forward gear's pair on the layshaft/output side; `drive` is the gear on the driving shaft |
| `reverse` | array of `{drive, driven}` | same length as `reverse_ratios` | same for reverse |
| `input_pair` | `{drive, driven}` | both keys required (`"a tooth pair needs"`) | the constant-mesh pair from the input shaft to the layshaft |

Counts are integers `>= 1` (`"tooth counts must be integers >= 1"`); unknown
keys are rejected (`"teeth: unknown key '<key>'"`). The counts are **not**
cross-checked against the ratios.

Mesh frequencies (`DrivelineSoundState::mesh_frequency_hz[2]`, 0 where
unknown): `[0]` = `|w_in| * input_pair.drive / (2 pi)`; `[1]` = the engaged
gear, `|w_layshaft| * forward/reverse_teeth[gear].drive / (2 pi)` with
`w_layshaft = w_in * input_pair.drive / input_pair.driven` (`= w_in` without
an input pair); `0` in neutral. Without tooth counts `mesh_order` stands in:
`|w_in| * mesh_order / (2 pi)`. `DrivelineSoundState::mesh_load_nm` is the
torque through the engaged mesh (`|` node-balance reaction at the input `|`,
0 in neutral), computed for lossless boxes as well. None of these fields is
hashed.

### Rating check

`check_buildable` rule `GearboxRating` (`BuildRule` 16) and the dyno finding
`LimitKind::GearboxInputTorque` (8) grade `peak / rated - 1` when positive:
`> 0` Near, `> 5 %` Limiting, `> 15 %` Exceeded (all `[AS]`, `rules::
kGearboxOverLimiting/Exceeded`). Only Exceeded blocks the garage. The Closed
remedy asks for `peak` (minimum) and `1.1 * peak` (margin, `kGearboxRemedyMargin`)
at `/rated_input_torque_nm`. The peak of a torque-map engine is
`peak_net_torque_nm` (the WOT curve maximum up to the limiter); a simulated
engine's peak comes from the dyno sweep, so boost and nitrous count.

### `synchro`

`SynchroDesc`.

| Field | Type | Req./opt. | Default | Validation | Meaning |
|---|---|---|---|---|---|
| `forward_capacity_nm` | array of real (N·m, input-shaft-referred) | optional | zero for every forward gear (dog-only, no synchro) | if non-empty, must have the **same length** as `forward_ratios`, else `"<ctx>: synchro.forward_capacity_nm must have the same length as forward_ratios"` | per-forward-gear cone-synchro torque capacity; `0` = that gear has no synchro (dog only) |
| `reverse_capacity_nm` | array of real | optional | zero for every reverse gear | same-length rule vs. `reverse_ratios`, else `"<ctx>: synchro.reverse_capacity_nm must have the same length as reverse_ratios"` | per-reverse-gear synchro capacity |
| `static_factor` | real | optional | `1.0` | none | static synchro capacity = `static_factor * kinetic` |
| `max_dog_mismatch_rad_s` | real (rad/s) | optional | `5.0` | none | maximum input-shaft speed mismatch the dog clutch will engage at without a grind |

An **empty** `forward_capacity_nm`/`reverse_capacity_nm` array is
automatically padded to zeros matching the ratio-array length (`"0 entries
meaning dog only is fine, an EMPTY array is not"` per the loader's own
comment) — you never need to spell out an all-zero array by hand.

## Automatic and dual-clutch gearboxes

Automatic gearbox plan A1. A gearbox of type `"automatic"` or `"dual_clutch"` has no synchros and no dog clutches:
two **shift clutches** (row A and row B, friction rows from the gear set's input to its output whose ratio is the gear
each holds) do every gear change clutch-to-clutch. The vehicle has no separate `clutch` component; the engine crank is
shafted straight to the gearbox `in` port. An `automatic_tcu` controller (see vehicle.md "Controllers") drives the
selector, the shift schedule and (automatic) the lock-up clutch or (dual_clutch) the launch on clutch slip.

`synchro`, `neutral_phase_s`, `dog_release_torque_nm`, `grind_after_s` and `abort_after_s` are ignored by these types.
`initial_gear` must be 0 (`"<ctx>: a clutch-to-clutch gearbox starts in N (initial_gear 0)"` - the box starts in
N, the TCU selects D/R). A manual or sequential gearbox with `shift_clutch` or `converter` is rejected (`"<ctx>:
'shift_clutch' and 'converter' are for \"automatic\" and \"dual_clutch\" gearboxes only"`).

Shift sequence (both types): **Engage** (N -> D/R, the first clutch ramps in over `engage_time_s`), **Preselect**
(dual_clutch only, a gear not already waiting on the idle shaft takes `preselect_time_s`), **Torque phase** (the
off-going clutch ramps out while the on-coming ramps in over `torque_phase_s`, each held at
`C_h = min(C, 1.3 max(|input torque|, 0.15 C))`), **Inertia phase** (the on-coming clutch pulls the input shaft to the
new gear's speed; in a power-on upshift its capacity is `min(C, 1.5 C_h)` and the engine throttle is capped at the TCU's
`upshift_torque_fraction`; ends when `|slip| < sync_slip_rad_s` or after `max_inertia_phase_s`), **Closing** (the
on-coming clutch ramps to full capacity). A dual_clutch box preselects the next gear on the idle shaft while it holds a
gear: `cur + 1` when the input rpm is above the midpoint of the current up/down lines, else `cur - 1`.

### `shift_clutch`

Required for both types (`"<ctx>: 'shift_clutch' is required for this gearbox type"`); unknown keys rejected
(`"<ctx>.shift_clutch: unknown key '<key>'"`).

| Field | Type | Default | Validation | Meaning |
|---|---|---|---|---|
| `capacity_nm` | real (N·m, input-referred) | **required** | `> 0` | kinetic capacity of each shift clutch at engagement 1 |
| `static_factor` | real | `1.15` | `>= 1` | static capacity = `static_factor * kinetic` |
| `torque_phase_s` | real (s) | `0.15` | `> 0` | handover duration (off-going out, on-coming in) |
| `max_inertia_phase_s` | real (s) | `0.6` | `> 0` | the inertia phase ends after this long at the latest |
| `sync_slip_rad_s` | real (rad/s) | `3.0` | `> 0` | on-coming slip below this ends the inertia phase |
| `engage_time_s` | real (s) | `0.35` | `> 0` | N -> D / N -> R engagement ramp |
| `preselect_time_s` | real (s) | `0` | `>= 0`; dual_clutch only (`"preselect_time_s is for dual_clutch gearboxes only"`) | time to preselect a gear not already waiting on the idle shaft; 0 = always preselected |

### `converter`

Required for `"automatic"` (`"<ctx>: 'converter' is required for an automatic gearbox"`), rejected for `"dual_clutch"`
(`"<ctx>: a dual_clutch gearbox has no 'converter'"`). Unknown keys rejected.

| Field | Type | Default | Validation | Meaning |
|---|---|---|---|---|
| `k_factor` | Curve1D `{x, y}` | **required** | x = speed ratio SR starting at 0, strictly increasing, below 1; every K `> 0` | capacity factor K(SR) in rpm/sqrt(N·m): pump torque = (pump rpm / K)². Between the last point and SR 1 the capacity falls linearly to 0 (the coupling point) |
| `torque_ratio` | Curve1D `{x, y}` | **required** | x starts at 0 within [0, 1]; every TR `>= 1`; non-increasing; `TR(SR) * SR <= 1` (no efficiency above 1) | turbine torque / pump torque |
| `pump_inertia_kgm2` | real | `0` | `> 0` | inertia of the `in` port (replaces `input_inertia_kgm2` there) |
| `turbine_inertia_kgm2` | real | `0` | `>= 0`; turbine + `input_inertia_kgm2` must be `> 0` | inertia of the internal turbine node (with `input_inertia_kgm2`) |
| `lockup_capacity_nm` | real (N·m) | **required** | `> 0` | kinetic capacity of the lock-up clutch (pump -> turbine) |
| `lockup_static_factor` | real | `1.1` | `>= 1` | static factor |
| `lockup_engage_time_s` / `lockup_release_time_s` | real (s) | `0.6` / `0.15` | `> 0` | lock-up ramp times |

Model: forward flow (|pump| >= |turbine|) pump torque `-sign * C(SR) * rpm_pump²`, turbine torque `TR(SR)` times that;
overrun (turbine faster) the same law with pump and turbine swapped and TR 1. The Powertrain applies the explicit
torques plus an implicit pump-turbine coupling of slope `|dT/dw_turbine|`, so the converter is stable at any substep.
The converter's slip loss and the shift/lock-up clutch slip losses are booked as `PowertrainEnergy::converter_loss`
(converter and lock-up clutch) and `clutch_loss` (the shift clutches), both in `balance_error`.

Ports: `in` (pump; the engine crank) and `out`, as for a manual box. The turbine is an internal node with no port.

Shipped data (all [AS]): `data/gearboxes/sedan_8at.json` (8-speed automatic, 4.70-0.67, rated 400 N·m, stall about
2400 rpm against the sedan engine) and `data/gearboxes/sedan_7dct.json` (7-speed dual clutch, 3.6-0.65, 350 N·m
clutches, handover 0.06 s, preselect 0.12 s). Test cars: `tests/support/fixtures/vehicles/car_sedan_at.json`,
`car_sedan_dct.json`; tests `tests/validation/test_automatic_gearbox.cpp`, `tests/unit/test_automatic_gearbox_io.cpp`
(`[drivetrain][automatic]`).


## CVT gearboxes

CVT plan C1. A gearbox of type `"cvt"` is a push-belt continuously variable transmission: a forward/reverse clutch
set (`shift_clutch`, required as for the clutch-to-clutch types) and, optionally, a torque `converter` ahead of a
variator whose ratio is a state. It is a clutch-to-clutch kind for the powertrain wiring: no `clutch` component, the
engine crank shafted straight to `in`, an `automatic_tcu` with a [`cvt` ratio law](vehicle.md#automatic_tcu) required.
The variator ratio is the ratio of the one friction row from the gear set's input to its output; its capacity is
`min(forward clutch capacity, belt capacity)`. D is one gear (gear 1), plus N and R.

`forward_ratios` / `forward_efficiency` are rejected (`"<ctx>: a cvt gearbox has no 'forward_ratios'/'forward_efficiency'
(the variator's cvt.steps and efficiency_vs_ratio replace them)"`); the loader fills them from `cvt.steps` and the
efficiency at each step, for position readers. Exactly one reverse ratio (`"<ctx>: a cvt gearbox has exactly one reverse
ratio"`; the value includes the variator's launch ratio); no `preselect_time_s`. A `cvt` block on another type is
rejected (`"<ctx>: 'cvt' is for \"cvt\" gearboxes only"`).

**Launch.** With a `converter` the box launches through it and its lock-up clutch (lock-up policy of the TCU,
`lockup.min_gear` counts D as gear 1). Without one the forward clutch slips like a dual-clutch launch clutch (the TCU's
`launch` and `creep_torque_nm`); the variator is held at `ratio_low` until that clutch has locked, and the clamping
force is raised to the clutch's full static capacity while it slips under throttle (so the lock-up never slips the belt).

### `cvt`

Unknown keys rejected (`"<ctx>.cvt: unknown key '<key>'"`); `ratio_low`, `ratio_high`, `steps`, `efficiency_vs_ratio`
and `belt` are required (`"<ctx>.cvt: '<key>' is required"`).

| Field | Type | Default | Validation | Meaning |
|---|---|---|---|---|
| `ratio_low` / `ratio_high` | real | **required** | `ratio_low > ratio_high > 0` | launch (largest) and overdrive ratios, `omega_in = r * omega_out` |
| `ratio_rate_per_s` | real (ln r / s) | `1.0` | `> 0` | fastest ratio change in the continuous mode |
| `step_rate_per_s` | real (ln r / s) | `4.0` | `> 0` | fastest ratio change between the virtual steps (stepped mode) |
| `steps` | array of real | **required** | at least two, strictly decreasing (1st first), within `[ratio_high, ratio_low]` | the virtual steps of the stepped mode (S / paddles) |
| `efficiency_vs_ratio` | Curve1D `{x, y}` | **required** | x ascending, every y in `(0, 1]` | variator mechanical efficiency at the current ratio (applied as the lagged mesh loss, booked in `gear_loss`) |
| `pump_drag_nm`, `pump_drag_nm_per_kn` | real | `0`, `0` | `>= 0` | hydraulic pump drag on the input: `pump_drag_nm + pump_drag_nm_per_kn * F / 1000`, faded in over the first 10 rad/s of input speed (`gear_loss`) |
| `belt` | object | **required** | see below | the belt and its clamping force |

`belt` (unknown keys rejected): `friction_coefficient` (`0.09`, `> 0`), `sheave_half_angle_deg` (`11`, in (0, 45)),
`pulley_radius_sum_m` (`0.2`, `> 0`), `clamp_force_min_n` / `clamp_force_max_n` (`5000` / `40000`, `0 < min <= max`),
`clamp_safety_factor` (`1.3`, `>= 1`), `clamp_time_constant_s` (`0.05`, `> 0`), `static_factor` (`1.1`, `>= 1`).

Model: drive pulley running radius `R_p = pulley_radius_sum_m / (1 + r)`; belt capacity at the input
`C_belt = 2 mu F R_p / cos(beta)`. The clamping force `F` lags (`clamp_time_constant_s`) toward
`clamp_safety_factor * max(|input torque|, |engine torque demand|) / (2 mu R_p / cos(beta))`, clamped to
`[clamp_force_min_n, clamp_force_max_n]`. A torque above the belt's capacity slips the row (static capacity
`static_factor * C_belt`); the slip loss is booked in `clutch_loss`. Ratio, virtual step, step timer and clamping force
are hashed only on a cvt.

Shipped data (all [AS], `source` field in each file): `data/gearboxes/sedan_cvt.json` (2.63-0.38, converter from
`sedan_8at.json`, 450 N·m forward clutch) and `data/gearboxes/sedan_cvt_clutch.json` (same variator, 300 N·m wet
starting clutch, no converter). Test cars `tests/support/fixtures/vehicles/car_sedan_cvt.json`,
`car_sedan_cvt_clutch.json`; tests `tests/validation/test_cvt_gearbox.cpp`, `tests/unit/test_cvt_gearbox_io.cpp`
(`[cvt]`).

## Ports

Fixed names, same for every gearbox: `<id>.in`, `<id>.out`.

## Minimal complete example

A simple 5-speed + reverse manual box:

```json
{
  "format": "physics_sim.gearbox/1",
  "name": "doc_example_5mt",
  "type": "manual",
  "source": "Documentation example, plausible 5-speed manual, not a real gearbox.",
  "forward_ratios":     [3.60, 2.10, 1.45, 1.05, 0.82],
  "forward_efficiency": [0.96, 0.97, 0.975, 0.98, 0.98],
  "reverse_ratios":     [-3.40],
  "reverse_efficiency": [0.95],
  "input_inertia_kgm2": 0.010,
  "output_inertia_kgm2": 0.015,
  "input_axis_local": [0, 1, 0],
  "output_axis_local": [0, -1, 0],
  "synchro": { "forward_capacity_nm": [80, 70, 60, 50, 50], "reverse_capacity_nm": [0],
               "static_factor": 1.0, "max_dog_mismatch_rad_s": 5.0 },
  "neutral_phase_s": 0.06,
  "dog_release_torque_nm": 60.0,
  "grind_after_s": 0.25,
  "abort_after_s": 1.0,
  "initial_gear": 0
}
```

## Common mistakes

- **`forward_ratios` not strictly decreasing.** Ratios must be listed 1st
  gear first, each one smaller than the last — a common transcription
  error is listing them in ascending numeric order or including a typo'd
  duplicate.
- **`forward_efficiency`/`reverse_efficiency` length mismatch** against the
  corresponding ratio array — every ratio needs exactly one efficiency
  value, even if it's a boring `1.0`.
- **`synchro.forward_capacity_nm`/`reverse_capacity_nm` sized to the wrong
  array.** These are matched against `forward_ratios`/`reverse_ratios`
  respectively, in the same order — a length mismatch throws immediately;
  leaving the field out entirely is safer than getting the length wrong.
- **`initial_gear` out of range** — remember it is an index, not a count:
  a 5-speed + 1-reverse box accepts `-1..5` (plus `0` for neutral).
- **`type: "sequential"` used with normal H-pattern-style
  `neutral_phase_s`/`dog_release_torque_nm` values copied from a manual
  box** — not a loader error, but the tractor gearbox example
  (`data/gearboxes/tractor_12f4r.json`) uses larger
  `neutral_phase_s`/`grind_after_s`/`abort_after_s` values deliberately;
  read its `source` note before assuming manual-box defaults transfer.
