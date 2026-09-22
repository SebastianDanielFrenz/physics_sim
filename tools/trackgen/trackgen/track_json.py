"""Assembles test_ground.json: physical surface table, spawn points, named
test zones and the circuit centerline -- everything a sim/adapter needs that
isn't baked into the mesh. See data/tracks/README.md for the schema summary.
"""
import math

from . import layout as L


def _pose(x, y, z, heading_rad, pitch_deg=0.0):
    return {"position": [round(x, 3), round(y, 3), round(z, 3)],
            "yaw_deg": round(math.degrees(heading_rad), 2), "pitch_deg": pitch_deg}


def _spawn(id_, area, vehicle, sample, note, lateral=0.0, pitch_deg=0.0):
    x, y, z = sample.point(lateral)
    return {"id": id_, "area": area, "vehicle": vehicle, **_pose(x, y, z, sample.heading, pitch_deg),
            "note": note}


def _spawns(samples):
    out = [
        {"id": "spawn_main", "area": "main_disc", "vehicle": "any",
         **_pose(0.0, 0.0, 0.0, 0.0),
         "note": "Main spawn, centre of the terrain's flat disc (z=0 by the terrain spec), facing +x."},
    ]
    st = samples["straight"]
    for veh in ("car", "tractor"):
        out.append(_spawn(f"spawn_straight_{veh}", "straight", veh, st[0], "Straight start line, lane centre."))
    sk = samples["skidpad"]
    edge = sk[0]  # +x point of the ring
    for veh in ("car", "tractor"):
        out.append(_spawn(f"spawn_skidpad_{veh}", "skidpad", veh, edge,
                           "On the skidpad centreline, tangent heading.", lateral=0.0))
    sp = samples["split_mu"]
    for veh in ("car", "tractor"):
        out.append(_spawn(f"spawn_split_mu_{veh}", "split_mu", veh, sp[0], "Split-mu strip start."))
    for rid, rs in samples["ramps"].items():
        r = next(rr for rr in L.RAMPS if rr["id"] == rid)
        foot_s = sum(r["s_flat0"]) / 2.0
        plateau_s = sum(r["s_plateau"]) / 2.0
        foot = min(rs, key=lambda s: abs(s.s - foot_s))
        plateau = min(rs, key=lambda s: abs(s.s - plateau_s))
        veh = "tractor" if r["grade"] >= 0.30 else "car"
        out.append(_spawn(f"spawn_{rid}_foot", rid, veh, foot, f"{rid}: flat approach, grade {r['grade']:.0%}."))
        out.append(_spawn(f"spawn_{rid}_plateau", rid, veh, plateau, f"{rid}: flat plateau at the top."))
    ss = samples["side_slope"]
    mid = min(ss, key=lambda s: abs(s.s - (L.SIDE_SLOPE["apron"] + L.SIDE_SLOPE["core_len"] / 2)))
    out.append(_spawn("spawn_side_slope", "side_slope", "tractor", mid, "Centre of the 10% cross slope."))
    out.append({"id": "spawn_grass", "area": "grass_patch", "vehicle": "any",
                **_pose(*L.GRASS_PATCH["center"], 0.0, 0.0), "note": "Grass patch centre."})
    out.append({"id": "spawn_dirt", "area": "dirt_patch", "vehicle": "any",
                **_pose(*L.DIRT_PATCH["center"], 0.0, 0.0), "note": "Dirt patch centre."})
    cc = samples["circuit"]
    for veh in ("car", "tractor"):
        out.append(_spawn(f"spawn_circuit_{veh}", "circuit", veh, cc[0], "Circuit start/finish line."))
    ls = samples["long_straight"]
    out.append(_spawn("spawn_long_straight_car", "long_straight", "car", ls[0],
                       "Long (10 km) top-speed straight, start line."))
    air = samples["airstrip"]
    out.append(_spawn("spawn_airstrip", "airstrip", "aircraft", air[0], "Airstrip threshold (runway 01)."))
    return out


def _zones(samples, segs):
    st = samples["straight"]
    zones = {
        "straight_start_line": {"s": 0.0, **_pose(st[0].x, st[0].y, st[0].z, st[0].heading)},
        "straight_finish_line": {"s": L.STRAIGHT["length"],
                                  **_pose(st[-1].x, st[-1].y, st[-1].z, st[-1].heading)},
        "straight_braking_zone": {
            "s_start": L.STRAIGHT["length"] - 300.0, "s_end": L.STRAIGHT["length"],
            "note": "Reserved for 100-0 braking: accelerate on s < s_start, stop by s_end.",
        },
        "skidpad": {"center": [round(c, 3) for c in L.SKIDPAD["center"]] + [0.0],
                    "radius": L.SKIDPAD["radius"], "width": L.SKIDPAD["width"]},
        "split_mu_start": {
            **_pose(samples["split_mu"][0].x, samples["split_mu"][0].y, samples["split_mu"][0].z,
                    samples["split_mu"][0].heading),
            "note": "y>0 half = asphalt, y<0 half = low_mu (see test_ground_collision surfaces).",
        },
        "circuit_start_finish": {
            **_pose(samples["circuit"][0].x, samples["circuit"][0].y, samples["circuit"][0].z,
                    samples["circuit"][0].heading),
            "direction": "counter-clockwise (increasing s turns left on average; net turn +360 deg)",
            "length": round(samples["circuit"][-1].s, 2),
            "min_radius": L.min_circuit_radius(segs),
        },
    }
    for rid, rs in samples["ramps"].items():
        r = next(rr for rr in L.RAMPS if rr["id"] == rid)
        foot = rs[0]
        up0 = min(rs, key=lambda s: abs(s.s - r["s_up"][0]))
        plateau_mid = min(rs, key=lambda s: abs(s.s - sum(r["s_plateau"]) / 2.0))
        zones[f"{rid}_foot"] = {**_pose(up0.x, up0.y, up0.z, up0.heading), "grade": r["grade"]}
        zones[f"{rid}_plateau"] = {**_pose(plateau_mid.x, plateau_mid.y, plateau_mid.z, plateau_mid.heading),
                                    "grade": r["grade"], "rise": round(r["rise"], 3)}

    ls = samples["long_straight"]
    zones["long_straight_start_line"] = {**_pose(ls[0].x, ls[0].y, ls[0].z, ls[0].heading)}
    zones["long_straight_finish_line"] = {**_pose(ls[-1].x, ls[-1].y, ls[-1].z, ls[-1].heading)}
    zones["long_straight_braking_zone"] = {
        "s_start": L.LONG_STRAIGHT["length"] - 3000.0, "s_end": L.LONG_STRAIGHT["length"],
        "note": "Reserved for braking from top speed: accelerate on s < s_start, stop by s_end.",
    }
    air = samples["airstrip"]
    a0 = air[0]
    thr1 = min(air, key=lambda s: abs(s.s - L.AIRSTRIP["overrun"]))
    thr2 = min(air, key=lambda s: abs(s.s - (L.AIRSTRIP["length"] - L.AIRSTRIP["overrun"])))
    zones["airstrip_threshold_01"] = {**_pose(thr1.x, thr1.y, thr1.z, thr1.heading),
                                       "note": "Runway 01 threshold (after the first overrun)."}
    zones["airstrip_threshold_19"] = {**_pose(thr2.x, thr2.y, thr2.z, thr2.heading),
                                       "note": "Runway 19 threshold (before the second overrun)."}
    zones["airstrip_approach"] = {
        "note": "Corridor flat zone extends >= 2 km beyond each paved end for an obstacle-free "
                "approach/departure (see terrain.json flat_zones.zones['airstrip_corridor']).",
        "approach_length_m": L.AIRSTRIP_APPROACH,
        "start": [round(a0.x, 3), round(a0.y, 3), round(a0.z, 3)],
    }
    return zones


def _centerline(samples_full, decimate=2):
    out = []
    for i, sm in enumerate(samples_full):
        if i % decimate == 0 or i == len(samples_full) - 1:
            out.append({"s": round(sm.s, 2), "x": round(sm.x, 3), "y": round(sm.y, 3), "z": round(sm.z, 3),
                        "heading_deg": round(math.degrees(sm.heading), 2), "curvature": round(sm.curvature, 5)})
    return out


def build_json(samples, glb_stats, col_stats):
    circuit_full = samples["circuit"]  # samples_full: includes the closing duplicate sample
    return {
        "format": "physics_sim.track/1",
        "id": "test_ground",
        "title": "M1 test ground",
        "description": "Procedural, deterministic test ground for the P2/M1 first test drive. This file "
                       "covers the meshed static features (straight, skidpad, grass/dirt patches, a "
                       "split-mu strip, ramps at 5/10/15/30/50%, a side slope, a small circuit, a 10 km "
                       "top-speed straight and an airstrip); the surrounding 100 x 100 km world is "
                       "procedural terrain, see terrain.json. Every feature here sits on one of "
                       "terrain.json's flat zones, so none of it depends on the noise height function. "
                       "See PLAN.md section 'M1 -- First test drive' and data/tracks/README.md.",
        "frame": "World frame: local east-north-up (x=east, y=north, z=up), metres, radians "
                 "(yaw_deg/pitch_deg in degrees for readability) -- algebraically the same as a "
                 "vehicle's own ISO 8855 frame (x forward, y left, z up); the world is oriented so the "
                 "main spawn's forward is due east. Origin (0,0,0) is the main spawn point, at the "
                 "centre of terrain.json's main flat disc.",
        "world": {"half_extent_m": 50_000.0, "terrain_spec": "terrain.json", "terrain_golden": "terrain_golden.json"},
        "glb": "test_ground.glb",
        "collision": {"bin": "test_ground_collision.bin", "index": "test_ground_collision.json"},
        "generated": {"glb_triangles": glb_stats["triangles"], "glb_bytes": glb_stats["bytes"],
                      "collision_triangles": col_stats["total_triangles"], "collision_bytes": col_stats["bytes"]},
        "surfaces_file": "data/surfaces/surfaces.json",
        "spawns": _spawns(samples),
        "zones": _zones(samples, samples["circuit_segments"]),
        "circuit_centerline": _centerline(circuit_full),
    }
