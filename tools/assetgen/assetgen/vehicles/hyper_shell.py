"""Hypercar-only material returns. Never cap an opening across its mouth.

Solidify selected source skin to an inner skin with stitched perimeter walls.
Position welding is for adjacency only; preserves outer normals/material seams.
Each moving part owns its returns. A small inset leaves physical panel clearance.
This is visual shell preparation, not boolean union or CFD certification.
"""
from collections import defaultdict
from .. import mesh as M
from ..vec import add, sub, mul, norm, cross, length

def key(p):
    return tuple(round(v*1e8) for v in p)

def clean_degenerate(mb):
    removed=0
    for p in mb.prims.values():
        indices=[]
        for i in range(0,len(p.idx),3):
            tri=p.idx[i:i+3];a,b,c=[p.pos[j] for j in tri]
            # Same threshold as independent glTF audit, including float-export welding.
            weld=[tuple(round(v*1e6) for v in q) for q in (a,b,c)]
            if len(set(weld))<3 or length(cross(sub(b,a),sub(c,a)))<1e-12:
                removed+=1;continue
            indices.extend(tri)
        p.idx=indices
    return removed

def solidify(mb, part, sources, thickness=.018, inset=0, back_material='interior_trim'):
    """Sources must be one logical material shell, before decorative meshes are added."""
    positions={}; normals=defaultdict(lambda:(0,0,0));edges=defaultdict(list)
    for material,p in sources:
        for v,n in zip(p.pos,p.nrm):
            k=key(v);positions.setdefault(k,v);normals[k]=add(normals[k],n)
        for i in range(0,len(p.idx),3):
            tri=p.idx[i:i+3]
            for a,b in zip(tri,tri[1:]+tri[:1]):
                ka,kb=key(p.pos[a]),key(p.pos[b]);edges[tuple(sorted((ka,kb)))].append((ka,kb))
    if any(len(v)>2 for v in edges.values()):
        raise ValueError('hyper shell source has nonmanifold edges')
    directions={k:norm(n) for k,n in normals.items()}
    outer={k:sub(p,mul(directions[k],inset)) for k,p in positions.items()}
    inner={k:sub(p,mul(directions[k],inset+thickness)) for k,p in positions.items()}
    for material,p in sources:
        dst=mb.prim(part,material if material=='glass' else back_material)
        # Capture first, as destination can share a primitive with source in callers.
        pos=list(p.pos);nrms=list(p.nrm);uv=list(p.uv);idx=list(p.idx)
        p.pos=[outer[key(v)] for v in pos]
        base=len(dst.pos)
        dst.pos.extend(inner[key(v)] for v in pos);dst.nrm.extend(mul(n,-1) for n in nrms);dst.uv.extend(uv)
        for i in range(0,len(idx),3):
            a,b,c=idx[i:i+3];dst.idx.extend((base+a,base+c,base+b))
    for incidences in edges.values():
        if len(incidences)!=1:continue
        a,b=incidences[0]
        # Outer boundary direction reversed; inner face winds opposite to outer face.
        mb.polygon([outer[b],outer[a],inner[a],inner[b]],back_material,part)

def skin_part(mb,part,thickness=.018,inset=0,back_material='interior_trim'):
    # Explicit part rather than mutation of mesh-builder global behavior.
    solidify(mb,part,[(mat,p) for (prt,mat),p in list(mb.prims.items()) if prt==part],thickness,inset,back_material)
