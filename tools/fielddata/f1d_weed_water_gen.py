"""F1d part 6: adds the "water" block to the shipped weed groups (python stdlib only).

Run from the repo root:  python tools/fielddata/f1d_weed_water_gen.py
Idempotent: the block is replaced, the rest of each file is kept (indent 2, line endings kept).
"""
import json
import os

ROOT = os.path.normpath(os.path.join(os.path.dirname(__file__), '..', '..'))

KC = {
    'black_grass': 1.0, 'cleavers': 1.0, 'couch_grass': 1.0, 'volunteer_wheat': 1.1,
}

SOURCE = {
    'kc_tr': ("[AS] the basal transpiration coefficient of the weed canopy at full cover: the FAO-56 Kcb of a closed green grass or broadleaf stand is "
              "about 1.0 (Allen et al. 1998, table 17, mid-season values 0.9-1.15; recalled). Volunteer wheat takes the 1.1 of the wheat crop file."),
    'p_depletion': "[AS] 0.5: share of the root-zone TAW that can be used before the group is water stressed, the FAO-56 default for crops of about this rooting (table 22 lists 0.45-0.6 for cereals and grasses; recalled).",
    'cover_per_g_m2': ("[AS] canopy cover = 1 - exp(-k x biomass): leaf area about 0.015 m2 per g of dry matter (specific leaf area ~25 m2/kg, 60 % leaf) "
                       "times an extinction coefficient 0.5 gives k = 0.0075 m2/g, i.e. 100 g/m2 of weeds cover 53 % of the soil."),
}


def write(path, obj, crlf):
    txt = json.dumps(obj, indent=2, ensure_ascii=False)
    if crlf:
        txt = txt.replace('\n', '\r\n')
    with open(path, 'w', encoding='utf-8', newline='') as f:
        f.write(txt)


for name, kc in KC.items():
    path = os.path.join(ROOT, 'data', 'weeds', name + '.json')
    raw = open(path, encoding='utf-8', newline='').read()
    crlf = '\r\n' in raw
    j = json.loads(raw)
    top = j.pop('source', None)
    j['water'] = {'kc_tr': kc, 'p_depletion': 0.5, 'cover_per_g_m2': 0.0075, 'source': dict(SOURCE)}
    if top is not None:
        j['source'] = top
    write(os.path.join(ROOT, 'data', 'weeds', name + '.json'), j, crlf)
print('ok')
