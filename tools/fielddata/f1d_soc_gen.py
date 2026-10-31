"""F1d part 5: adds the "organic_matter" block to the shipped soils (python stdlib only).

Run from the repo root:  python tools/fielddata/f1d_soc_gen.py [rate_per_day_20c]
Idempotent: the block is replaced, the rest of each soil file is kept (indent 2, line endings kept).
The default rate is the one the model was calibrated to (see the source text of rate_per_day_20c).
"""
import json
import os
import sys

ROOT = os.path.normpath(os.path.join(os.path.dirname(__file__), '..', '..'))

RATE = float(sys.argv[1]) if len(sys.argv) > 1 else 0.000042
RATE_NOTE = sys.argv[2] if len(sys.argv) > 2 else ('Measured: a bare fallow run of 11 years on the home climate with the nutrient rate 1.15e-4 decays the humus at 0.01655 per year, '
                                                    'so the rate is 1.15e-4 x 0.00605 / 0.01655 = 4.2e-5; the same run with 4.2e-5 gives 0.006046 per year.')

SOURCE = {
    'layer_depth_m': "[AS] the plough layer; the organic N of the nutrient block (org_n_kg_ha) is the N of this layer.",
    'bulk_density_t_m3': "[AS] dry bulk density of a loamy arable topsoil, 1.3 t/m3; with 0.3 m it gives the 3900 t/ha of dry soil that the org_n_kg_ha note of the nutrient block uses.",
    'humus_cn': "[AS] C:N of soil organic matter is close to 10 in cultivated mineral topsoils (recalled, not checked against a source: textbook value; Batjes 1996 is recalled as 10-12 for 0-30 cm).",
    'rate_per_day_20c': ("[D] first-order decay of the whole humus pool at 20 degC, optimum moisture and pH: the ICBM old-pool rate k2 = 0.00605 per year "
                         "(Andren and Kaetterer 1997, Ecol. Appl. 7:1226, as shipped in the SoilR package defaults), divided by the mean of the model's "
                         "temperature x moisture x pH factors on the home climate (measured on a bare fallow run of this model with ps_field_run) "
                         "so that the pool decays at k2 per year in the mean. Not tuned to a yield or to a long-term experiment. " + RATE_NOTE),
    'humification': ("[C] ICBM humification coefficient h = 0.125: the share of the decomposed young-pool (residue) carbon that is passed to the old pool "
                     "(Andren and Kaetterer 1997; SoilR examples use 0.125 for crop residues, 0.25 for manure). The carbon of a residue that is not humified is "
                     "respired as the biomass turns over."),
    'awc_per_soc_pct': ("[C] plant-available water capacity rises by 1.16 mm per 100 mm of soil per 1 % soil organic carbon, i.e. 0.0116 m3/m3 per 1 % "
                        "(Minasny and McBratney 2018, 'Limited effect of organic matter on soil available water capacity', Eur. J. Soil Sci. 69:39-47); "
                        "all of it is put on theta_fc."),
}


def write(path, obj, crlf):
    txt = json.dumps(obj, indent=2, ensure_ascii=False)
    if crlf:
        txt = txt.replace('\n', '\r\n')
    with open(path, 'w', encoding='utf-8', newline='') as f:
        f.write(txt)


for name in ('silt_loam_loess', 'sandy_loam'):
    path = os.path.join(ROOT, 'data', 'soils', name + '.json')
    raw = open(path, encoding='utf-8', newline='').read()
    crlf = '\r\n' in raw
    j = json.loads(raw)
    j['organic_matter'] = {
        'layer_depth_m': 0.3, 'bulk_density_t_m3': 1.3, 'humus_cn': 10.0, 'rate_per_day_20c': RATE,
        'humification': 0.125, 'awc_per_soc_pct': 0.0116, 'source': SOURCE,
    }
    write(path, j, crlf)
print('ok', RATE)
