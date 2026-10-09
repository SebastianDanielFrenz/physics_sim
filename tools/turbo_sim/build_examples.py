"""Generate reproducible preliminary standard/compact/large turbo examples."""
import argparse
import copy
import json
from pathlib import Path
import math
from meanline import Generator, atomic_json

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir",type=Path,default=Path("out/turbo_sim/examples"))
    parser.add_argument("--cache",type=Path,default=Path("out/turbo_point_cache"))
    args=parser.parse_args()
    base=json.loads(Path(__file__).with_name("standard_geometry.json").read_text())
    root=Path(__file__).resolve().parents[2]
    hw_base=json.loads((root/"data/turbos/hyper_turbo_approx.json").read_text())
    config_base=json.loads((root/"data/turbo_configurations/hyper_twin_default.json").read_text())
    for name,scale in (("compact",.8),("standard",1),("large",1.2)):
        profile=copy.deepcopy(base)
        profile["name"]=name+"_meanline_preliminary"
        for key in ("inducer_mm","exducer_mm","hub_mm","exit_width_mm"):
            profile["compressor"][key]*=scale
        profile["turbine"]["diameter_mm"]*=scale
        profile["turbine"]["effective_throat_mm2"]*=scale**2
        profile["provenance"]["size_variant"]=f"DERIVED: all lengths scaled by {scale}; areas by scale squared. Not a separate manufacturer's turbo."
        g=Generator(profile,args.cache)
        n=[240000/scale*i/8 for i in range(9)]
        # Extend turbine coverage beyond its zero-torque speed for any PR.
        # Torque interpolation is linear in corrected RPM; this terminal
        # line avoids abandoning physical braking for the authored fallback.
        jet_max=math.sqrt(2*g.exhaust.cp*1000*g.t.nozzle_efficiency)
        terminal=2*jet_max/(g.t.diameter_mm*.0005)*60/math.tau
        if terminal>n[-1]:
            n.append(terminal)
        w=[.9*scale**2*i/12 for i in range(13)]
        a=(g.exhaust.gamma-1)/g.exhaust.gamma
        chi=(1-40**(-a))**.5
        er=[(1-(chi*i/8)**2)**(-1/a) for i in range(9)]
        result=g.adaptive_generate(n,w,er,max_passes=5,max_speed_lines=129,max_flow_points=257,workers=3)
        if result["sampling"]["budget_limited"]:
            raise RuntimeError(name+": interpolation refinement did not meet its sampling target")
        directory=args.output_dir/name
        atomic_json(directory/"geometry.json",profile)
        atomic_json(directory/"stage_maps.json",result)
        hardware=copy.deepcopy(hw_base)
        hardware["name"]=profile["name"]
        hardware["source"]=profile["source"]+" Runtime has explicit authored fallback outside generated coverage."
        hardware["performance_map"]="stage_maps.json"
        hardware["rotor_inertia_kgm2"]*=scale**5
        hardware["reference_rpm"]/=scale
        for key in ("max_mass_flow_kg_s","turbine_area_m2","wastegate_area_m2","shaft_loss_w_at_reference"):
            hardware[key]*=scale**2
        atomic_json(directory/"hardware.json",hardware)
        config=copy.deepcopy(config_base)
        config["name"]=profile["name"]+"_twin"
        config["turbo"]="hardware.json"
        config["source"]="Experimental meanline turbo installation. See stage_maps.json provenance and coverage."
        atomic_json(directory/"configuration.json",config)
        print(name, "shape",len(result["corrected_rpm"]),len(result["compressor_corrected_flow_kg_s"]),len(result["turbine_expansion_ratio"]),"audit",result["sampling"]["audit"])

if __name__=="__main__":
    main()
