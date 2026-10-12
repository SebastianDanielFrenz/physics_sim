"""Engine garage P10: writes data/cams/bmw_n52b30_valvetronic.json, the Valvetronic intake lift family (and the fixed
exhaust lift) of data/engines/bmw_n52b30.json. Deterministic (same bytes on re-run), stdlib only.

Cited (web pages read 2026-10-12, no downloads):
  - intake lift range 0.18 .. 9.9 mm, exhaust lift 9.7 mm, "phase 255/263" (read as the intake / exhaust opening
    durations in crank degrees at full lift): https://mymotorlist.com/engines/bmw/n52b30/ (a secondary page; the same
    numbers are attributed to a reprinted BMW 2006 330i data chart).
  - minimum lift 0.18 mm on the second Valvetronic generation, full stroke of the eccentric shaft in 300 ms, at part
    load the intake valve opens later and closes sooner: https://www.fenderbender.com/running-a-shop/operations/article/
    33028956/how-to-repair-bmw-valvetronic-systems (Scot Manna, 2017).
ASSUMED (no OEM lift-versus-eccentric-angle or duration-versus-lift curve was found; the owner should supply BMW's
training manual ST055 / Flierl & Klueting SAE 2000-01-1227):
  - nodes u = 0, 0.15, 0.3, 0.5, 0.75, 1 of the normalised eccentric-shaft position;
  - peak lift L(u) = 0.18 + 9.72 u^2 mm (0.18 mm at u = 0 and 9.9 mm at u = 1 are the cited end points, the quadratic
    in between is assumed - the eccentric mechanism gives a lift that grows slowly near minimum);
  - 0-lift duration D(u) = 120 + 135 u deg crank (255 deg at u = 1 is the cited figure, 120 deg at minimum is assumed);
  - shapes L*sin^2(pi*x/D) over the duration (smooth zero-velocity ends, a generic stand-in for a real lobe);
  - fixed centre lines: intake 105 deg after and exhaust 110 deg before gas-exchange TDC (the double-VANOS phasing is
    not modelled) - a shorter event about a fixed centre opens later and closes sooner, the cited part-load behaviour.

    python tools/enginedata/n52_valvetronic_cam.py
"""
import json
import math
import os

STEP = 10.0  # deg crank between knots (approximately: the knot count is even so the peak is a knot; at least 8 knots)

NODES_U = [0.0, 0.15, 0.3, 0.5, 0.75, 1.0]  # [AS]
LIFT_MIN_MM, LIFT_MAX_MM = 0.18, 9.9         # [C]
DUR_MIN_DEG, DUR_MAX_DEG = 120.0, 255.0      # [AS] / [C]
EXHAUST = (263.0, 9.7)                        # [C] duration deg, peak mm
INTAKE_CENTRE = 105.0                         # [AS] deg after gas-exchange TDC (0)
EXHAUST_CENTRE = 720.0 - 110.0                # [AS] deg (110 before gas-exchange TDC)


def lift_curve(duration, peak_mm, centre):
    n = max(8, 2 * int(round(duration / (2.0 * STEP))))  # even: a knot sits on the peak
    xs, ys = [], []
    for i in range(n + 1):
        x = duration * i / n
        y = 0.0 if i in (0, n) else peak_mm * math.sin(math.pi * x / duration) ** 2
        xs.append(round(x, 4))
        ys.append(round(y, 5))
    return {"lift_mm_vs_deg": {"x": xs, "y": ys}, "phase_ref_deg": round(centre - duration / 2.0, 4)}


def main():
    root = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
    out = {
        "format": "physics_sim.cam/1",
        "name": "bmw_n52b30_valvetronic",
        "source": (
            "Engine garage P10: the Valvetronic intake lift family and the fixed exhaust lift of engines/bmw_n52b30.json, "
            "written by tools/enginedata/n52_valvetronic_cam.py. [C] intake lift 0.18-9.9 mm, exhaust lift 9.7 mm, "
            "'phase 255/263' (read as the full-lift intake/exhaust durations, crank deg) - https://mymotorlist.com/engines/"
            "bmw/n52b30/ (secondary page); minimum lift 0.18 mm, full stroke in 300 ms, part-load intake opens later and "
            "closes sooner - https://www.fenderbender.com/running-a-shop/operations/article/33028956/how-to-repair-bmw-"
            "valvetronic-systems (Scot Manna 2017). [AS] nodes u = 0, 0.15, 0.3, 0.5, 0.75, 1; peak lift 0.18 + 9.72 u^2 "
            "mm; 0-lift duration 120 + 135 u deg; L*sin^2(pi*x/D) shapes; fixed centre lines intake 105 deg after / exhaust "
            "110 deg before gas-exchange TDC (double VANOS not modelled). No OEM lift curve was available (owner: BMW "
            "ST055 training manual, Flierl & Klueting SAE 2000-01-1227)."
        ),
        "fits": {
            "family": "bmw_n52",
            "intake_valves": 2,
            "exhaust_valves": 2,
            "intake_valve_d_mm": [34.2, 34.2],
            "exhaust_valve_d_mm": [29.0, 29.0],
            "source": "[D] the valve heads of engines/bmw_n52b30.json's cycle block",
        },
        "intake_lift_family": [],
        "exhaust_lift": lift_curve(EXHAUST[0], EXHAUST[1], EXHAUST_CENTRE),
    }
    for u in NODES_U:
        peak = LIFT_MIN_MM + (LIFT_MAX_MM - LIFT_MIN_MM) * u * u
        dur = DUR_MIN_DEG + (DUR_MAX_DEG - DUR_MIN_DEG) * u
        out["intake_lift_family"].append({"u": u, "intake_lift": lift_curve(dur, round(peak, 6), INTAKE_CENTRE)})
    path = os.path.join(root, "data", "cams", "bmw_n52b30_valvetronic.json")
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        json.dump(out, f, indent=2)
        f.write("\n")
    print("wrote", path)


if __name__ == "__main__":
    main()
