# Oxidizer format — `physics_sim.oxidizer/1`

Files: `data/oxidizers/*.json` (shipped: `nitrous_oxide.json`). Loader:
`ps::io::load_oxidizer_json` (`core/src/io/nitrous_io.cpp`, declared in
`ps/io/nitrous_io.h`). Public descriptor: `ps::drivetrain::OxidizerDesc`
(`core/include/ps/drivetrain/oxidizer_desc.h`). Generator:
`tools/oxidizerdata/nist_n2o.py` (stdlib only; reads the saved NIST downloads
under `out/sources/`, `--fetch` downloads them; deterministic, the same raw
files give the same bytes).

An oxidizer file carries the fluid properties the nitrous-oxide system needs:
the saturation line (for the bottle state and the two-phase jet), the gas
enthalpy at 1 bar (for the evaporative-cooling balance) and the
decomposition data. It carries no hardware: bottles, kits and controllers are
separate part files (`nitrous.md`).

## Fields

| Field | Type | Req./opt. | Validation | Meaning |
|---|---|---|---|---|
| `format` | string | required | must equal `"physics_sim.oxidizer/1"` | |
| `name` | string | required | non-empty | informational |
| `molar_mass_g_mol` | real | required | `> 0`, finite | 44.0128 for N2O |
| `formation_enthalpy_kj_mol` | real | required | finite | gas, 298 K; `2 N2O -> 2 N2 + O2` releases this per mole (82.05 kJ/mol = 1.8643 MJ/kg) |
| `oxygen_mass_fraction` | real | required | in `(0, 1)` **and** within `1e-4` of `0.5*31.998/molar_mass_g_mol` | O2 mass fraction after decomposition; the cross-check makes a typo fail at load |
| `saturation` | object | required | see below | the saturation line, columns of equal length |
| `gas_1bar` | object | required | `t_k` strictly ascending (>= 2 rows), `h_kj_kg` same length and strictly ascending | gas enthalpy on the 1 bar isobar |
| `source` | string | required | non-empty | provenance (query URLs, fetch date, SHA-256 of every raw file) |

`saturation` keys (all required, no others accepted, >= 3 rows each): `t_k`
(K), `p_bar` (absolute), `rho_liquid`, `rho_vapour` (kg/m3),
`h_liquid_kj_kg`, `h_vapour_kj_kg`, `s_liquid_j_gk`, `s_vapour_j_gk`
(J/(g*K)). `t_k` and `p_bar` strictly ascending; densities positive; every
row needs `s_vapour > s_liquid` and `h_vapour > h_liquid`. One reference state
for h and s throughout (NIST `DEF`). Unknown keys anywhere are rejected.

## Shipped data

`data/oxidizers/nitrous_oxide.json`: NIST Chemistry WebBook (SRD 69, fluid
properties, Lemmon and Span 2006 equation of state): saturation 183-309 K in
1 K steps plus rows at 293.13, 303.37, 305.41, 307.36 and 309.5 K (the last is
within 0.05 K of the critical point, 72.4 bar), the 1 bar isobar 200-400 K in
10 K steps, `dHf` 82.05 kJ/mol (Chase 1998) and M 44.0128 g/mol.
`oxygen_mass_fraction` 0.3635 is DERIVED (0.5 * 31.998 / 44.0128).

## Flow tables built from it (internal: `core/src/drivetrain/nitrous_flow.h`)

`OxidizerTables` converts to SI once and builds, at load:

* **HEM critical mass flux** of the saturated liquid at bottle pressure.
  Each saturation row is used as the bottle state (`s1 = s_l`, `h1 = h_l`);
  the downstream pressure is swept down the table (all lower rows); the
  flux `rho2*sqrt(2*(h1 - h2))` is maximised with the isentropic quality
  `x2 = (s1 - s_l(p2))/(s_v(p2) - s_l(p2))`, `h2 = h_l + x2*(h_v - h_l)` and the
  homogeneous density `1/rho2 = (1 - x2)/rho_l + x2/rho_v`. The result is a
  table over the bottle pressure, interpolated linearly and clamped. The jet
  is choked at any manifold pressure, so HEM has no downstream-pressure axis.
  The choke (throat) pressure is the argmax row's pressure, i.e. a table-row
  value with about 1 K (1.5-2 bar) resolution, not refined.
* **Dyer NHNE** (selectable per kit): `0.5*sqrt(2*rho_l*(p1 - p_down)) +
  0.5*G_HEM` (kappa = 1 for a saturated inlet). The SPI term is the one runtime
  sqrt.
* Spot values (the shipped file, linear interpolation in pressure; psig to
  absolute with `(psig + 14.696)*0.0689476` bar):

  | bottle | p1 [bar abs] | T_sat [K] | rho_l | SPI flux to 1 bar | G_HEM | choke p2 |
  |---|---|---|---|---|---|---|
  | 900 psig | 63.07 | 303.14 | 688.2 | 92 426 | 32 294 kg/m2s | 43.8 bar |
  | 950 psig | 66.51 | 305.59 | 651.0 | 92 356 | 32 749 | 45.9 |
  | 1000 psig | 69.96 | 307.92 | 597.0 | 90 738 | 32 771 | 48.0 |

  HEM is nearly flat over 900-1000 psig, while kit makers say real flow rises
  with pressure (the cause is probably liquid subcooling in the feed line,
  which the model does not capture). A known limitation.

Runtime accessors use only `+ - * /`, `sqrt` and binary-search table
interpolation and allocate nothing; `OxidizerTables::content_hash()` (FNV-1a over
every table value) is folded once into the state hash of a vehicle that has a
kit.
