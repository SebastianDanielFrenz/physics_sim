"""F1d part 7: the three 30-year field scenarios (python stdlib only).

Run from the repo root:  python tools/fielddata/f1d_scenarios_gen.py
Idempotent: the three files are rewritten with the same bytes on every run.

  wheat_continuous_30y.json            winter wheat every year, fertilised
  rotation_rape_wheat_barley_30y.json  winter rapeseed -> winter wheat -> winter barley, fertilised (a non-cereal break every third year)
  wheat_unfertilised_30y.json          winter wheat every year, Broadbalk-style: no N, P or K, chalk kept

All three run the nutrient, management, pathogen and soil-organic-carbon models on the deep loess (the sandy loam is in the
soil list so a test can put it in some cells), straw returned, no weeds, no fungicide. The harvest year k of the label
is the k-th harvest: a winter crop is sown in the autumn of year k-1, so the file runs 31 game years (year 0 holds the first sowing).
The fertiliser plan per crop is the one the F1d part 4 rotation tests use (tests/unit/test_field_pathogens.cpp, P8): DAP and MOP
at sowing, three calcium ammonium nitrate dressings in spring, chalk 4 t/ha every fourth year.
"""
import json
import os

ROOT = os.path.normpath(os.path.join(os.path.dirname(__file__), '..', '..'))
OUT = os.path.join(ROOT, 'data', 'field_scenarios')

PLANS = {
    'winter_wheat': dict(dap=0.15, mop=0.17, sow=288, harvest=212, n=[(95, 60), (130, 50), (160, 40)]),
    'winter_barley': dict(dap=0.15, mop=0.17, sow=275, harvest=196, n=[(60, 50), (95, 60), (125, 40)]),
    'winter_rapeseed': dict(dap=0.16, mop=0.17, sow=235, harvest=208, n=[(50, 80), (80, 60), (110, 40)]),
}


def scenario(seq, fertilised=True):
    names = sorted(set(seq), key=seq.index)
    L = len(seq)
    ops = [{"op": "apply", "material": "lime_carbonate_90", "method": "incorporated", "rate_t_ha": 4.0, "doy": 250, "every_years": 4}]
    for i, nme in enumerate(seq):
        p = PLANS[nme]
        hy = i + 1
        sy = hy - 1
        kw = {"first_year": sy, "every_years": L}
        if fertilised:
            ops.append({"op": "apply", "material": "dap_18_46", "method": "broadcast", "rate_t_ha": p['dap'], "doy": 262, **kw})
            ops.append({"op": "apply", "material": "mop_60", "method": "broadcast", "rate_t_ha": p['mop'], "doy": 262, **kw})
        ops.append({"op": "sow", "crop": nme, "doy": p['sow'], **kw})
        if fertilised:
            for doy, kg in p['n']:
                ops.append({"op": "apply", "material": "can_27", "method": "broadcast", "rate_n_kg_ha": kg, "doy": doy,
                            "first_year": hy, "every_years": L})
        ops.append({"op": "harvest", "doy": p['harvest'], "first_year": hy, "every_years": L, "straw_removed": 0.0})
    return {
        "format": "physics_sim.field_scenario/1",
        "climate": "../climates/farm_de_home.json",
        "seed": 21,
        "years": 31,
        "nutrients": True,
        "management": True,
        "pathogens": True,
        "soc": True,
        "soils": ["../soils/silt_loam_loess.json", "../soils/sandy_loam.json"],
        "crops": ["../crops/%s.json" % n for n in names],
        "materials": ["../field_materials/%s.json" % m for m in ('can_27', 'dap_18_46', 'mop_60', 'lime_carbonate_90')],
        "methods": "../field_materials/methods.json",
        "grid": {"nx": 2, "ny": 2, "cell_size_m": 4.0},
        "initial_depletion_fraction": 0.2,
        "co2_ppm": 400.0,
        "operations": ops,
    }


FILES = {
    'wheat_continuous_30y.json': scenario(['winter_wheat']),
    'rotation_rape_wheat_barley_30y.json': scenario(['winter_rapeseed', 'winter_wheat', 'winter_barley']),
    'wheat_unfertilised_30y.json': scenario(['winter_wheat'], fertilised=False),
}

for name, obj in FILES.items():
    with open(os.path.join(OUT, name), 'w', encoding='utf-8', newline='\n') as f:
        f.write(json.dumps(obj, indent=2) + '\n')
print('ok')
