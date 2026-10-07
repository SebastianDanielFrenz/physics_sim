"""Focused car_hyper exported topology, flow selection and rig regression check.

Run python tools/assetgen/hyper_topology_check.py [--report out.json]. Stdlib.
Groups materials by node before welding; material boundaries are not holes.
Edge closure is a visual validation, NOT proof of intersection-free CFD solids.
"""
import argparse
from collections import Counter, defaultdict
import hashlib, json, math, os, subprocess
from pathlib import Path
from validate import read_glb, accessor, iso
ROOT=Path(__file__).resolve().parents[2]

def audit(triangles,tolerance=1e-6):
    edges=Counter();directed=Counter();faces=Counter();bad=0
    for a,b,c in triangles:
        keys=[tuple(round(x/tolerance) for x in p) for p in (a,b,c)]
        u=[b[i]-a[i] for i in range(3)];v=[c[i]-a[i] for i in range(3)]
        n=(u[1]*v[2]-u[2]*v[1],u[2]*v[0]-u[0]*v[2],u[0]*v[1]-u[1]*v[0])
        bad+=len(set(keys))<3 or sum(x*x for x in n)<1e-24
        faces[tuple(sorted(keys))]+=1
        for i in range(3):
            x,y=keys[i],keys[(i+1)%3];edges[tuple(sorted((x,y)))]+=1;directed[x,y]+=1
    return dict(triangles=len(triangles),boundary_edges=sum(n==1 for n in edges.values()),
        nonmanifold_edges=sum(n>2 for n in edges.values()),degenerate_triangles=bad,
        duplicate_faces=sum(n-1 for n in faces.values()),
        inconsistent_winding_edges=sum(n==2 and (directed[x,y]!=1 or directed[y,x]!=1) for (x,y),n in edges.items()))

def check():
    folder=ROOT/'data/models/car_hyper';path=folder/'car_hyper.glb'
    gltf,blob,_=read_glb(path);nodes=gltf['nodes'];parents={child:i for i,n in enumerate(nodes) for child in n.get('children',[])}
    def translation(i):
        local=nodes[i].get('translation',[0,0,0]);assert 'matrix' not in nodes[i] and 'rotation' not in nodes[i] and 'scale' not in nodes[i]
        parent=translation(parents[i]) if i in parents else (0,0,0)
        return tuple(a+b for a,b in zip(local,parent))
    parts=defaultdict(list);primitives={}
    for i,node in enumerate(nodes):
        if 'mesh' not in node:continue
        offset=translation(i)
        for primitive in gltf['meshes'][node['mesh']]['primitives']:
            points=[iso(tuple(x+y for x,y in zip(p,offset))) for p in accessor(gltf,blob,primitive['attributes']['POSITION'])]
            indices=accessor(gltf,blob,primitive['indices'])
            triangles=[tuple(points[j] for j in indices[k:k+3]) for k in range(0,len(indices),3)]
            material=gltf['materials'][primitive['material']]['name'];key=(node['name'],material)
            assert key not in primitives;primitives[key]=triangles;parts[node['name']].extend(triangles)
    reports={name:audit(triangles) for name,triangles in parts.items()};combined=audit([t for ts in parts.values() for t in ts])
    for name,result in {**reports,'combined':combined}.items():
        assert all(result[k]==0 for k in ('boundary_edges','nonmanifold_edges','degenerate_triangles','duplicate_faces','inconsistent_winding_edges')),(name,result)
    selection=json.loads((folder/'car_hyper.flow.json').read_text(encoding='utf-8'))
    assert selection['source_glb_sha256']==hashlib.sha256(path.read_bytes()).hexdigest()
    assert selection['cfd_ready'] is False and selection['status']=='visual_surface_selection_not_fluid_solid'
    selected=0;used=defaultdict(set)
    for row in selection['ranges']:
        key=(row['node'],row['material']);a=row['first_triangle'];b=a+row['triangle_count']
        assert 0<=a<b<=len(primitives[key]);assert not(used[key]&set(range(a,b)));used[key].update(range(a,b));selected+=b-a
        assert not row['material'].startswith('cabin_') and row['material'] not in ('interior_trim','interior','engine','engine_red')
    assert selected<combined['triangles']
    # Source selection is checked against FINAL indices, not merely bounds/hash.
    from assetgen.vehicles.car_hyper import build
    model=build();assert model.flow_selection['ranges']==selection['ranges']
    rebuilt,_=model._build_glb();assert hashlib.sha256(rebuilt).hexdigest()==selection['source_glb_sha256']
    rig=json.loads((folder/'car_hyper.rig.json').read_text(encoding='utf-8'))
    before=json.loads(subprocess.check_output(['git','-c','safe.directory='+ROOT.as_posix(),'-C',str(ROOT),'show','HEAD:data/models/car_hyper/car_hyper.rig.json']))
    for field in ('nodes','sockets'):
        old={n['name']:n for n in before[field]}
        assert {n['name'] for n in rig[field]}==set(old)
        for node in rig[field]:
            for key in ('parent','pivot','position','joint'):
                assert node.get(key)==old[node['name']].get(key),(node['name'],key)
    return dict(format='physics_sim.hyper_visual_topology/1',source_glb_sha256=selection['source_glb_sha256'],
        combined=combined,nodes=reports,flow_selected_triangles=selected,cfd_ready=False,
        note='Closed visual components; intersections/occluded faces/flow physics not certified.')
if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--report');args=parser.parse_args()
    report=check()
    if args.report:Path(args.report).write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({k:v for k,v in report.items() if k!='nodes'},indent=2))
