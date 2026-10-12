"""Engine garage P9: writes data/cams/honda_k20a_dc5_vtec.json, the two-profile (VTEC) cam part of
data/engines/honda_k20a_dc5.json. Deterministic (same bytes on re-run), stdlib only.

Every lift number is ASSUMED: no OEM, SAE or service-manual K20A cam lift/duration/timing was found by web reading
(2026-10-12 search; only aftermarket cam cards exist - see the cam file's source text). The shapes are
lift(x) = L * sin(pi * x / D)^2 over the 0-lift duration D (smooth zero-velocity ends, a generic stand-in for a real
polydyne lobe). Centre lines fixed (VTC not modelled): intake 105 deg after gas-exchange TDC, exhaust 110 deg before it.

    python tools/enginedata/k20a_vtec_cam.py
"""
import json
import math
import os

STEP = 10.0  # deg crank between knots

# (id, intake (duration deg, peak mm), exhaust (duration deg, peak mm)) - all [AS]
PROFILES = [
    ("low", (230.0, 8.0), (230.0, 7.5)),
    ("high", (290.0, 11.5), (280.0, 10.5)),
]
INTAKE_CENTRE = 105.0          # deg after gas-exchange TDC (0)
EXHAUST_CENTRE = 720.0 - 110.0  # deg (110 before gas-exchange TDC)


def lift_curve(duration, peak_mm, centre):
    n = int(round(duration / STEP))
    xs, ys = [], []
    for i in range(n + 1):
        x = duration * i / n
        y = 0.0 if i in (0, n) else peak_mm * math.sin(math.pi * x / duration) ** 2
        xs.append(round(x, 4))
        ys.append(round(y, 4))
    return {"lift_mm_vs_deg": {"x": xs, "y": ys}, "phase_ref_deg": round(centre - duration / 2.0, 4)}


def main():
    root = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
    out = {
        "format": "physics_sim.cam/1",
        "name": "honda_k20a_dc5_vtec",
        "source": (
            "Engine garage P9: the two VTEC cam profiles of engines/honda_k20a_dc5.json, written by "
            "tools/enginedata/k20a_vtec_cam.py. Honda states that the K20A Type R's VTEC switches lift and timing on both "
            "the intake and the exhaust cam [C] honda.co.jp Integra fact book 2001.07 p.07 "
            "(https://www.honda.co.jp/factbook/auto/INTEGRA/200107/07.html). Every lift number is [AS]: no OEM, SAE or "
            "service-manual lift/duration/timing figure for any VTEC engine was found (only aftermarket cam cards, e.g. JUN "
            "K20 Type 0 high lobe 304 deg / 12.2 mm, https://junauto.co.jp/products/camshaft/k20.en.html, an upper bound "
            "for a race cam). Shapes L*sin^2(pi*x/D) over the 0-lift duration D; low intake 230 deg / 8.0 mm, high intake "
            "290 deg / 11.5 mm, low exhaust 230 deg / 7.5 mm, high exhaust 280 deg / 10.5 mm [AS]; fixed centre lines "
            "intake 105 deg after and exhaust 110 deg before gas-exchange TDC [AS] (the K20A's 50 deg intake VTC "
            "[C] hondanews.com 2002 RSX powertrain press kit is not modelled)."
        ),
        "fits": {
            "family": "honda_k20a",
            "intake_valves": 2,
            "exhaust_valves": 2,
            "intake_valve_d_mm": [35, 35],
            "exhaust_valve_d_mm": [30, 30],
            "source": "[D] the valve heads of engines/honda_k20a_dc5.json's cycle block",
        },
        "profiles": [],
    }
    for pid, (idur, ipk), (edur, epk) in PROFILES:
        out["profiles"].append({
            "id": pid,
            "intake_lift": lift_curve(idur, ipk, INTAKE_CENTRE),
            "exhaust_lift": lift_curve(edur, epk, EXHAUST_CENTRE),
        })
    path = os.path.join(root, "data", "cams", "honda_k20a_dc5_vtec.json")
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        json.dump(out, f, indent=2)
        f.write("\n")
    print("wrote", path)


if __name__ == "__main__":
    main()
