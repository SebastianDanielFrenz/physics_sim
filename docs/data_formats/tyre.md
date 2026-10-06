# Tyre format — `physics_sim.tyre/1`

Files: `data/tyres/*.json`, **or** inline inside a wheel's `"tyre"` field in
a vehicle file, **or** a standard MF `.tir` parameter file. Loaders:
`ps::io::load_tyre_json` (standalone file, `core/src/io/vehicle_io.cpp`),
`read_tyre_object` (shared by the standalone and inline paths),
`ps::vehicle::parse_tir_file` (`.tir`, `core/src/vehicle/tir_parser.h`/
`.cpp`, only the subset of keys `TyreDesc` uses — unrecognised `.tir` keys
are silently ignored, not an error). Public descriptor:
`ps::vehicle::TyreDesc` (`core/include/ps/vehicle/tyre_desc.h`).

Coefficient names follow Pacejka, *Tyre and Vehicle Dynamics*, 3rd ed.,
ch. 4's own `P*`/`Q*`/`R*` naming (Magic Formula MF5.2/6.1 — a **subset**,
not the full equation set; see `tyre_desc.h`'s and
`core/src/vehicle/pacejka_tyre_model.cpp`'s header comments for exactly
which terms are simplified). This struct assigns no meaning to any specific
numeric value on its own — every shipped file is "published or plausible"
data (PLAN.md section 8), never a measured/proprietary `.tir`; see README's
"source" convention.

## How a wheel's `"tyre"` field resolves

```jsonc
"tyre": "../tyres/passenger_car_225_45r17.json"   // standalone JSON, format-checked on its own
"tyre": "../tyres/some_tyre.tir"                  // detected by literal ".tir" suffix, parsed by parse_tir_file
"tyre": { "name": "...", "source": "...", ... }   // inline object, same fields as below, no own "format" field
```

A path is resolved relative to the vehicle file's own directory (README's
path-resolution rule). If the resolved path ends in `.tir` (case-sensitive,
literal 4-character suffix check), it is parsed as a `.tir` file; otherwise
it is read as JSON and must carry `"format": "physics_sim.tyre/1"`.

## Fields

Every numeric field below is optional with the shown default **except**
`name`/`source` and the two low-speed-damping fields, which are validated.
Unless noted, there is no range check — an out-of-range Pacejka coefficient
is not rejected, it just produces an unrealistic force curve.

| Field | Type | Req./opt. | Default | Validation | Meaning |
|---|---|---|---|---|---|
| `format` | string | required (standalone file only) | — | must equal `"physics_sim.tyre/1"` | see README |
| `name` | string | optional | `""` | none | tyre name |
| `source` | string | **required** | — | must be non-empty, else `"tyre '<name>': 'source' is required (PLAN.md section 8)"` | provenance note (README's "source" convention) |
| `unloaded_radius` | real (m) | optional | `0.31` | none | R0, unloaded tyre radius |
| `width` | real (m) | optional | `0.205` | none | informational + `CylinderShape` width for the wheel's contact query |
| `rim_radius` | real (m) | optional | `0.2286` | none | informational only, not modelled |
| `fz0` | real (N) | optional | `4000` | none | nominal (reference) vertical load |
| `vertical_stiffness` | real (N/m) | optional | `200000` | none | informational only — `Fz` comes from the suspension model, not tyre carcass deflection, in this phase's simplified contact model |
| `v0` | real (m/s) | optional | `16.7` (~60 km/h) | none | reference speed for the `qsy3`/`qsy4` speed rolling-resistance terms |
| `qsy1` | real | optional | `0.01` | none | baseline rolling-resistance coefficient (multiplied by the surface's own `crr`) |
| `qsy2` | real | optional | `0` | none | `Fx`-dependent rolling-resistance term |
| `qsy3` | real | optional | `0` | none | MF 6.1 linear speed term, `qsy3*abs(vx/v0)`; `0` (the default) leaves every file that never names it bit-identical |
| `qsy4` | real | optional | `0.0002` | none | speed⁴ rolling-resistance term coefficient |
| `rolling_resistance` | object | optional | — | see "Derived rolling resistance" below | derives `qsy1`, `qsy3` and `qsy4` in closed form from two speed anchors; cannot be combined with an explicit `qsy1`/`qsy3`/`qsy4` |
| `rolling_resistance_v_eps` | real (m/s) | optional | `0.05` | **must be > 0**, else `"tyre '<name>': rolling_resistance_v_eps must be > 0, got <value>"` | half-width of the linear ramp rolling resistance uses near `vx = 0` (regularised Coulomb drag — see the field's own header comment for why a hard `sign(vx)` caused a parked-car creep bug) |
| `relaxation_length_x` | real (m) | optional | `0.10` | none | longitudinal slip relaxation length (Pacejka ch. 7 transient model) |
| `relaxation_length_y` | real (m) | optional | `0.15` | none | lateral slip relaxation length |
| `lambda_mux` / `lambda_muy` | real | optional | `1.0` / `1.0` | none | friction-scale multipliers, combined with the surface table's `lambda_mu` |
| `low_speed_damping_ratio` | real | optional | `1.5` | **must be in `[0, 2]`**, else `"tyre '<name>': low_speed_damping_ratio must be in [0, 2], got <value>"` | dimensionless viscous damping ratio for the low-speed lateral carcass-damping term; `0` disables the term entirely |
| `low_speed_damping_vx` | real (m/s) | optional | `2.0` | **must be > 0**, else `"tyre '<name>': low_speed_damping_vx must be > 0, got <value>"` | the low-speed damping term fades to 0 at this TOTAL contact-patch speed |
| `longitudinal.*` | object of reals | optional | see below | none | pure-slip `Fx0(kappa, Fz, gamma)` coefficients |
| `lateral.*` | object of reals | optional | see below | none | pure-slip `Fy0(alpha, Fz, gamma)` coefficients |
| `combined.*` | object of reals | optional | see below | none | combined-slip weighting (`Gxa` reduces `Fx0` for nonzero slip angle, `Gyk` reduces `Fy0` for nonzero slip ratio) |
| `aligning.*` | object of reals | optional | see below | none | self-aligning moment `Mz0(alpha, Fz, gamma)` — a simplified own B/C/D/E curve, not the full pneumatic-trail × `Fy` decomposition |
| `overturning.*` | object of reals | optional | see below | none | overturning moment `Mx(Fz, gamma, Fy)` — a reduced linear-in-camber form |

Each of `longitudinal`/`lateral`/`combined`/`aligning`/`overturning` is an
**optional** nested object; any key you omit keeps `TyreDesc`'s own
compiled-in default for that coefficient (listed in `tyre_desc.h`, and
matching `data/tyres/passenger_car_225_45r17.json`'s values, which is where
the defaults originated). The full coefficient list per group:

| Group | Keys |
|---|---|
| `longitudinal` | `pcx1`, `pdx1`, `pdx2`, `pdx3`, `pex1`, `pex2`, `pex3`, `pex4`, `pkx1`, `pkx2`, `pkx3`, `phx1`, `phx2`, `pvx1`, `pvx2` |
| `lateral` | `pcy1`, `pdy1`, `pdy2`, `pdy3`, `pey1`, `pey2`, `pey3`, `pey4`, `pky1`, `pky2`, `phy1`, `phy2`, `phy3`, `pvy1`, `pvy2`, `pvy3`, `pvy4` |
| `combined` | `rbx1`, `rbx2`, `rcx1`, `rhx1`, `rby1`, `rby2`, `rby3`, `rcy1`, `rhy1`, `rhy2`, `rvy1`, `rvy2`, `rvy3`, `rvy4`, `rvy5`, `rvy6` |
| `aligning` | `qbz1`, `qbz2`, `qbz3`, `qcz1`, `qdz1`, `qdz2`, `qez1`, `qez2`, `qhz1`, `qhz2`, `qhz3` |
| `overturning` | `qsx1`, `qsx2`, `qsx3` |

**Sign convention for `aligning.qdz1`/`qdz2`:** these must carry the
**opposite** sign to `lateral.pdy1`/`pdy2` so that `Mz0 = -t*Fy0` with trail
`t > 0` (Pacejka's own convention) — an aligning moment that *resists*
turning, not one with the same sign as `Fy0` that steers further into the
turn. See `data/tyres/passenger_car_225_45r17.json`'s own `source` note
(PHYS-036) for the concrete before/after values; the loader does **not**
check this sign relationship, it is purely a physical-plausibility
convention.

## `.tir` files

`parse_tir_file` reads a standard MF `.tir` parameter file's `[SECTION]`
blocks and picks out only the key names `TyreDesc` above actually models
(via a key-name-to-field table in `tir_parser.cpp`); every other key in a
real `.tir` (full scaling coefficients, structural/gyroscopic properties)
is silently ignored. `source` is set to `"Parsed from <path>"` if the file
supplies no source note of its own (a `.tir` has no standard "cite your
source" key). **Project convention: never commit a real/proprietary `.tir`
file to this repo** (PLAN.md section 8) — `data/tyres/*.json` is this
project's own always-committable format instead. This is a repository
convention, not something the parser itself enforces (see README's
"Discrepancies found").

## Minimal complete example

```json
{
  "format": "physics_sim.tyre/1",
  "name": "doc_example_tyre",
  "source": "Documentation example, plausible MF5.2 magnitudes, not a measured tyre.",
  "unloaded_radius": 0.3,
  "width": 0.2,
  "rim_radius": 0.2,
  "fz0": 4000.0,
  "vertical_stiffness": 200000.0,
  "v0": 16.7,
  "qsy1": 0.01,
  "qsy2": 0.0,
  "qsy4": 0.0002,
  "relaxation_length_x": 0.10,
  "relaxation_length_y": 0.15,
  "low_speed_damping_ratio": 1.5,
  "low_speed_damping_vx": 2.0,
  "lambda_mux": 1.0,
  "lambda_muy": 1.0,
  "longitudinal": { "pcx1": 1.65, "pdx1": 1.05, "pdx2": -0.10, "pkx1": 18.0 },
  "lateral":      { "pcy1": 1.35, "pdy1": 1.0,  "pdy2": -0.12, "pky1": 16.0 },
  "aligning":     { "qbz1": 43.4, "qcz1": 1.15, "qdz1": -0.10, "qdz2": 0.02, "qez1": -1.0 }
}
```

(Fields omitted here, e.g. `combined`/`overturning`/most of `longitudinal`'s
and `lateral`'s coefficients, fall back to `TyreDesc`'s own defaults — a
real tyre file should specify all of them explicitly, as the shipped
`data/tyres/*.json` files do.)

## Common mistakes

- **Missing `source`.** Throws `"tyre '<name>': 'source' is required
  (PLAN.md section 8)"`, whether the tyre is a standalone file or an inline
  object.
- **`rolling_resistance_v_eps <= 0`.** Throws immediately — this value must
  be strictly positive because it's a ramp half-width, not a threshold that
  can be "zero to disable".
- **`low_speed_damping_ratio` outside `[0, 2]`**, or
  **`low_speed_damping_vx <= 0`** — both are hard errors with the exact
  bound in the message.
- **`aligning.qdz1`/`qdz2` given the same sign as `lateral.pdy1`/`pdy2`.**
  Not a loader error, but produces an aligning moment that steers *into*
  the turn instead of resisting it — see the sign-convention note above.
- **Confusing a standalone tyre file's own `"format"` check with the
  enclosing vehicle's.** A wheel's `"tyre": "../tyres/foo.json"` reference
  is independently format-checked (`physics_sim.tyre/1`); an *inline* tyre
  object has no `"format"` key at all and must not have one.

## Derived rolling resistance (`rolling_resistance` block)

The wheel applies a rolling-resistance drag `crr * Fz` at the contact patch with

```
crr = qsy1 + surface crr + qsy2*|Fx|/fz0 + qsy3*|vx/v0| + qsy4*(vx/v0)^4
```

(MF 6.1 eq. 4.E70's shape; `vx` is the contact-patch ground speed). Hand-tuning `qsy3`/`qsy4` is not authorable for
a tyre like "this one is for 600 km/h", so a tyre file may instead give two anchors of the rise relative to the
base coefficient and let the loader solve for the coefficients:

```json
"rolling_resistance": {
  "rated_speed_kmh": 270,
  "crr_base": 0.010,
  "rise_at_fraction": { "fraction": 0.8, "factor": 1.5 },
  "factor_at_rated": 2.0
}
```

`crr_base` becomes `qsy1`. The conditions are `crr(fraction*V_r) = factor*crr_base` and `crr(V_r) =
factor_at_rated*crr_base`, `V_r = rated_speed_kmh/3.6`. With `x_r = V_r/v0`, `A = qsy3*x_r/qsy1`, `B =
qsy4*x_r^4/qsy1` and `f = fraction`:

```
(factor - 1)          = A*f + B*f^4
(factor_at_rated - 1) = A   + B
B = ((factor_at_rated - 1)*f - (factor - 1)) / (f - f^4),   A = (factor_at_rated - 1) - B
qsy3 = A*qsy1/x_r,   qsy4 = B*qsy1/x_r^4
```

Rejected (`std::runtime_error`, message prefixed `tyre '<name>': rolling_resistance: ...`): `A < 0` or `B < 0` (i.e.
`(factor-1)/(factor_at_rated-1)` outside `[f^4, f]`); an explicit `qsy1`, `qsy3` or `qsy4` next to the block;
`fraction` outside `(0,1)`; `factor < 1`; `factor_at_rated <= factor`; `crr_base <= 0`; `rated_speed_kmh <= 0`; a
missing key or an unknown key (in the block or in `rise_at_fraction`). Raw `qsy3`/`qsy4` stay legal for measured data.
`rolling_resistance_v_eps` (the standstill ramp) is unchanged. The derivation is `ps::vehicle::derive_rolling_resistance`
(`ps/vehicle/tyre_model.h`); the speed law the wheel and every test reference share is `tyre_rolling_speed_term`.

Shipped anchors (all `fraction` 0.8, `factor` 1.5, `factor_at_rated` 2.0 [AS], giving A 0.2316, B 0.7684): passenger
225/45R17 `rated_speed_kmh` 270 (ISO 4000-1 / ETRTO speed symbol W), hypercar tyres 500 [AS], tractor tyres 65
(agricultural symbol D, above the tractor's 47.8 km/h limiter top speed).
