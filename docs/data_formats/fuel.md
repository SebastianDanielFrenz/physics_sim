# Fuel format — `physics_sim.fuel/1`

Files: `data/fuels/*.json`. Loader: `ps::io::load_fuel_json`
(`core/src/io/fuel_io.cpp`). Public descriptor: `ps::drivetrain::FuelDesc`
(`core/include/ps/drivetrain/fuel_desc.h`) — lives in `ps::drivetrain`
because its planned consumers (an engine descriptor and a `fuel_tank`
component) are both there, not because a fuel is itself a drivetrain
concept.

A fuel file carries only bulk thermochemical/physical properties — lower
heating value, stoichiometric air/fuel ratio, liquid density — not a
combustion model. Every numeric value must be traceable to a specific,
readable source (publication, standard, table or problem, page/edition);
see each shipped file's own `"source"` field for the exact citation used
(PLAN.md section 8's "cite your source" convention, same as
`data/tyres/*.json`/`data/vehicles/*.json`).

## Fields

| Field | Type | Req./opt. | Default | Validation | Meaning |
|---|---|---|---|---|---|
| `format` | string | required | — | must equal `"physics_sim.fuel/1"` | see README |
| `name` | string | **required** | — | must be non-empty | fuel name, e.g. `"gasoline"`, `"diesel"` |
| `phase` | string | **required** | — | must equal `"liquid"`, else `"phase \"<value>\" is not yet supported"` | phase of matter the fuel is modelled as — only `"liquid"` is implemented (a future `"gaseous"` phase, e.g. CNG/LPG/hydrogen, needs its own pressure/temperature-dependent density convention and is not modelled yet) |
| `lhv_j_kg` | real (J/kg) | **required** | — | must be `> 0` and finite | lower heating value |
| `afr_stoich` | real | **required** | — | must be `> 0` and finite | stoichiometric air/fuel mass ratio |
| `density_kg_m3` | real (kg/m³) | **required** | — | must be `> 0` and finite | liquid density at the cited reference condition |
| `source` | string | **required** | — | must be non-empty | provenance note (README's "source" convention) — every numeric value's exact citation |

Unlike `tyre.md`/`engine.md`'s mix of optional-with-defaults fields, every
field in `physics_sim.fuel/1` is required — there is no plausible default
for a fuel's own heating value or density, unlike (say) a tyre's Pacejka
coefficients, so the loader accepts no partial file.

## Minimal complete example

```json
{
  "format": "physics_sim.fuel/1",
  "name": "gasoline",
  "phase": "liquid",
  "lhv_j_kg": 44000000.0,
  "afr_stoich": 14.5,
  "density_kg_m3": 747.5,
  "source": "Heywood, Internal Combustion Engine Fundamentals, Table 9.7, 'Gasoline' column; density EN 228:2012 midpoint of 720.0-775.0 kg/m^3."
}
```

(`data/fuels/gasoline.json`/`diesel.json` carry the full, precise citation
text in their own `"source"` fields — see those files directly rather than
this shortened example.)

## Common mistakes

- **Missing or non-`"liquid"` `phase`.** Throws `"fuel '<name>': phase
  \"<value>\" is not yet supported (only \"liquid\" is implemented)"` —
  there is no silent fallback to liquid.
- **`lhv_j_kg`/`afr_stoich`/`density_kg_m3` <= 0, non-finite, or omitted.**
  Each is a hard error naming the offending field and the value read; there
  is no compiled-in default for any of them (unlike most of `tyre.md`'s
  fields).
- **Missing `name` or `source`.** Both are required non-empty strings, same
  as `TyreDesc`/`VehicleDesc`'s own `source` convention.
- **Inventing or "typical"-izing a number.** Every shipped
  `data/fuels/*.json` file's values must come from a source that can be
  read and cited precisely (publication/table/problem, page, edition) — a
  plausible-sounding round number with no citation is not acceptable here,
  unlike some of `tyre.md`'s "published or plausible" Pacejka coefficients.
