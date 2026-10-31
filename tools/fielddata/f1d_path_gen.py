"""F1d part 4: adds the "pathogen" block to the shipped crops and writes the cereal fungicide (python stdlib only).

Run from the repo root:  python tools/fielddata/f1d_path_gen.py
Idempotent: the blocks are replaced, the rest of each crop file is kept byte for byte (indent 2, line endings kept).
"""
import json
import os

ROOT = os.path.normpath(os.path.join(os.path.dirname(__file__), '..', '..'))

CEREAL_SRC = {
    'agent': 'text',
    'loss_max': ("[M] fitted with build_per_season, suppression_* and the half-lives to three published margins: second wheat 10-15 % "
                 "below first wheat and 10-20 % in second and third wheat (AHDB take-all guidance; Farmers Weekly 'How to make second "
                 "wheat profitable'), the peak of take-all in the 3rd to 5th cropping year and the later decline to 80-90 % of the early "
                 "yields (Rothamsted take-all decline as summarised on Wikipedia 'Take-all'; not checked against the Rothamsted "
                 "primary source). Measured in the field model as yield ratios against the same run without pathogens (home loess, 4 "
                 "wheats / 1 rapeseed / wheat cycle for the break, test P8a): first wheat after a one-year break 0.967, continuous wheat "
                 "second 0.847 (12.4 % below the break wheat), third 0.823 (15 %), worst year 3, years 11-16 mean 0.872 (10 % below)."),
    'build_per_season': "[M] fitted together with loss_max (see there): 1 - exp(-0.7 X) of the free inoculum gap is closed by a season with X = (gdd of the season)/season_gdd, about 1.2 for autumn-sown wheat.",
    'season_gdd': "[D] development units from emergence to maturity of data/crops/%s.json (the daily gdd the model accumulates, clamped Tmin/Tmax).",
    'decay_half_life_days': ("[AS] half-life of the inoculum without a host, 110 d: a year without a host leaves a tenth of it "
                             "(AHDB: a break crop cuts take-all risk strongly and two years nearly clear it; no numeric half-life was found)."),
    'suppression_max': "[M] fitted with loss_max: the antagonists of take-all decline remove at most 90 % of the loss (Wikipedia 'Take-all' summary: yields return to 80-90 % of the early level).",
    'suppression_gain': "[M] fitted with loss_max: S closes 1 - exp(-0.4 P x) of its gap per season, so decline needs several seasons at a high inoculum (peak in year 3-5).",
    'suppression_half_life_days': ("[AS] half-life of the antagonist level, 1800 d: it outlasts the usual one-year break (AHDB: decline is lost only "
                                   "after a long break; no numeric half-life was found)."),
    'residue_source_per_day': "[AS] inoculum built per day, at full residue, from infected stubble and straw lying on the surface; ploughing it under removes the source.",
    'residue_ref_c_kg_ha': "[AS] surface residue carbon (loose straw + stubble) at which the source is complete: about 3 t/ha of dry matter.",
}

CROPS = {
    'winter_wheat': dict(agent='take-all (Gaeumannomyces tritici) and other cereal root and foot diseases', loss_max=0.66, build_per_season=0.7,
                         season_gdd=2400.0, decay_half_life_days=110.0, suppression_max=0.9, suppression_gain=0.4,
                         suppression_half_life_days=1800.0, residue_source_per_day=0.0002, residue_ref_c_kg_ha=1500.0, kind='cereal'),
    'winter_barley': dict(agent='take-all (Gaeumannomyces tritici) and other cereal root and foot diseases', loss_max=0.66, build_per_season=0.7,
                          season_gdd=2300.0, decay_half_life_days=110.0, suppression_max=0.9, suppression_gain=0.4,
                          suppression_half_life_days=1800.0, residue_source_per_day=0.0002, residue_ref_c_kg_ha=1500.0, kind='cereal'),
    'winter_rapeseed': dict(agent='clubroot, Sclerotinia and Verticillium (long-lived resting bodies)', loss_max=0.5, build_per_season=0.8,
                            season_gdd=3300.0, decay_half_life_days=700.0, suppression_max=0.0, suppression_gain=0.0,
                            suppression_half_life_days=1800.0, residue_source_per_day=0.0005, residue_ref_c_kg_ha=1500.0, kind='rape'),
    'grain_maize': dict(agent='Fusarium ear and stalk rot and maize pests that overwinter in the residue', loss_max=0.15, build_per_season=0.6,
                        season_gdd=1450.0, decay_half_life_days=400.0, suppression_max=0.0, suppression_gain=0.0,
                        suppression_half_life_days=1800.0, residue_source_per_day=0.002, residue_ref_c_kg_ha=1500.0, kind='maize'),
    'sugar_beet': dict(agent='beet cyst nematode, Rhizoctonia and rhizomania (long-lived resting stages)', loss_max=0.25, build_per_season=1.0,
                       season_gdd=2100.0, decay_half_life_days=700.0, suppression_max=0.0, suppression_gain=0.0,
                       suppression_half_life_days=1800.0, residue_source_per_day=0.0005, residue_ref_c_kg_ha=1500.0, kind='beet'),
}

RAPE_SRC = {
    'loss_max': ("[M] fitted with build_per_season and decay_half_life_days to one published margin: a 4-year break (rapeseed every fifth "
                 "year) yields 8-15 % more than rapeseed every third year (topagrar 'Rapsanbau in Deutschland: weite Fruchtfolge', which in "
                 "another passage gives only 4-7 %); the fit gives +12.5 % (test P8b). The loss of rapeseed after rapeseed (0.43) is not "
                 "published as a number anywhere that was found."),
    'build_per_season': "[M] fitted with loss_max (see there).",
    'decay_half_life_days': ("[M] fitted with loss_max (see there); consistent with clubroot and Verticillium resting spores that stay active "
                             "for years (Regierungspraesidium Giessen: 10 to 20 years; topagrar: infestation falls noticeably only after a six-year break)."),
    'suppression_max': "[AS] no suppression modelled for this family (0).",
    'suppression_gain': "[AS] no suppression modelled for this family (0).",
    'suppression_half_life_days': "[AS] unused while suppression_max is 0; equal across the crops of a family.",
    'residue_source_per_day': "[AS] small: the resting bodies of this family live in the soil more than on the surface residue.",
    'residue_ref_c_kg_ha': "[AS] surface residue carbon at which the source is complete.",
}

OTHER_SRC = {
    'loss_max': "[AS] not found as a published number for this crop; plausible order of the self-incompatibility yield penalty named in rotation guidance.",
    'build_per_season': "[AS] see loss_max.",
    'decay_half_life_days': "[AS] see loss_max; rotation guidance recommends 3-4 years between host crops for the long-lived pathogens of beet and about one year for maize residue diseases.",
    'suppression_max': "[AS] no suppression modelled for this family (0).",
    'suppression_gain': "[AS] no suppression modelled for this family (0).",
    'suppression_half_life_days': "[AS] unused while suppression_max is 0.",
    'residue_source_per_day': "[AS] see loss_max.",
    'residue_ref_c_kg_ha': "[AS] surface residue carbon at which the source is complete.",
}


def block(name, p):
    src = dict(CEREAL_SRC if p['kind'] == 'cereal' else RAPE_SRC if p['kind'] == 'rape' else OTHER_SRC)
    src.pop('agent', None)
    b = {'agent': p['agent']}
    for k in ('loss_max', 'build_per_season', 'season_gdd', 'decay_half_life_days', 'suppression_max', 'suppression_gain',
              'suppression_half_life_days', 'residue_source_per_day', 'residue_ref_c_kg_ha'):
        b[k] = p[k]
    s = {}
    for k in b:
        if k == 'agent':
            continue
        t = src.get(k)
        if k == 'season_gdd':
            t = ("[D] development units (daily gdd, clamped Tmin/Tmax) from emergence to maturity along the same thermal rule as data/crops/%s.json's "
                 "stages, rounded; the build per day is proportional to the day's gdd over this number." % name)
        if k in ('loss_max', 'build_per_season') and p['kind'] != 'cereal' and p['kind'] != 'rape':
            t = OTHER_SRC[k]
        s[k] = t
    b['source'] = s
    return b


def write(path, obj, crlf):
    txt = json.dumps(obj, indent=2, ensure_ascii=False)
    if crlf:
        txt = txt.replace('\n', '\r\n')
    with open(path, 'w', encoding='utf-8', newline='') as f:
        f.write(txt)


for name, p in CROPS.items():
    path = os.path.join(ROOT, 'data', 'crops', name + '.json')
    raw = open(path, encoding='utf-8', newline='').read()
    crlf = '\r\n' in raw
    j = json.loads(raw)
    j['pathogen'] = block(name, p)
    # keep "source" last
    top = j.pop('source')
    j['source'] = top
    write(path, j, crlf)

fung = {
    'format': 'physics_sim.herbicide/1',
    'name': 'cereal_fungicide',
    'moa': 'group_fungicide_demethylation_inhibitor',
    'label_rate': 1.0,
    'targets': {},
    'resistance': {'ed50_factor': 1.0, 'source': {'ed50_factor': '[AS] resistance of fungi is not modelled; the factor must be >= 1.'}},
    'residual': {'half_life_days': 0.0, 'germ_kill': 0.0,
                 'source': {'half_life_days': '[AS] no soil residual; the protection decays with the fungicide block.',
                            'germ_kill': '[AS] a fungicide does not act on weed seedlings.'}},
    'crops': {'default': {'max_bbch': 99.0, 'damage_in': 0.0, 'damage_out': 0.0,
                          'source': {'max_bbch': '[AS] applied at any stage.', 'damage_in': '[AS] no crop damage at the label rate.',
                                     'damage_out': '[AS] no crop damage at the label rate.'}}},
    'damage': {'dose_exponent': 1.5, 'source': {'dose_exponent': '[AS] unused: the product does no crop damage.'}},
    'fungicide': {
        'efficacy_max': 0.6, 'dose_half': 0.3, 'half_life_days': 200.0, 'family': 'cereal',
        'source': {
            'efficacy_max': ("[AS] largest share of the take-all loss a seed treatment or foliar programme buys back; AHDB advises a seed "
                             "treatment for third wheat and calls take-all control by chemistry partial; no numeric efficacy was found. "
                             "Always below 1: a fungicide never buys back the whole loss."),
            'dose_half': '[AS] dose relative to the label rate that gives half of efficacy_max (saturation dose/(dose + dose_half)); at the label rate 0.77 of efficacy_max (0.46).',
            'half_life_days': '[AS] the protection of one application fades with this half-life, 200 d: a seed or early-autumn treatment protects through the root-infection period.'}},
    'source': 'F1d part 4: a partial-protection fungicide for the pathogen model. The spray path is the herbicide one (operation "spray": {"product", "dose"}).'
}
write(os.path.join(ROOT, 'data', 'herbicides', 'cereal_fungicide.json'), fung, False)
print('ok')
