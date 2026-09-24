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
| `type` | string | optional | `"manual"` | must be `"manual"` or `"sequential"`, else `"<ctx>: gearbox 'type' must be \"manual\" or \"sequential\", got \"<type>\""` | selector behaviour (manual = H-pattern with neutral between every gear; sequential = up/down through an ordered list, e.g. a tractor range-and-gear box flattened into one sequence) |
| `forward_ratios` | array of real | **required** | — | must be present (`"<ctx>: 'forward_ratios' is required"`); every ratio `> 0`; **strictly decreasing**, 1st gear first (`"<ctx>.forward_ratios: must be strictly decreasing (1st first)"`) | `omega_in = ratio * omega_out` per forward gear, 1st listed first |
| `reverse_ratios` | array of real | optional | empty | every ratio `< 0` (`"<ctx>.reverse_ratios: every ratio must be < 0"`) | reverse ratios, R1 (slowest) first |
| `forward_efficiency` | array of real | **required** | — | must be present (`"<ctx>: 'forward_efficiency' is required"`); **same length** as `forward_ratios` (`"<ctx>: forward_efficiency must have the same length as forward_ratios"`); every value in `(0, 1]` (`"<ctx>.forward_efficiency: every value must be in (0, 1]"`) | mesh efficiency per forward gear |
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
