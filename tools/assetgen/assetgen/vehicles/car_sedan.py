"""b) Decent car: a mid-size 4-door sedan (generic, class-typical dimensions).

Length 4.55 m, width 1.82 m (body, no mirrors), height 1.45 m,
wheelbase 2.70 m, track 1.56 m, tyres 225/45 R17 (r = 0.317 m).
Interactive: 4 corners (travel, front steer, spin), 4 doors, hood, trunk lid,
steering wheel, 2 wipers, lamps (head, tail, brake, indicators, reverse).
"""
import math

from ..scene import Model
from ..carbody import CarBody, Panel
from ..parts import standard_materials, corner, WheelSpec, steering_wheel, seat, mirror
from .. import mesh as M
from ..vec import Curve, norm

WB, TRACK = 2.70, 1.56
XF, XR = WB / 2, -WB / 2
TYRE = WheelSpec(radius=0.317, width=0.225, rim_radius=0.216, tyre="road", rim="alloy5", spokes=5,
                 rim_color=(0.72, 0.73, 0.75), caliper_color=(0.25, 0.25, 0.27))


def build():
    m = Model("car_sedan", "Sedan (decent car)",
              "Mid-size 4-door sedan, 4.55 m, 225/45 R17, front-engine layout.",
              "Ground plane below the wheel centres, x = 0 midway between the axles, y = 0 on the centreline.")
    standard_materials(m)
    m.mat("paint", color=(1, 1, 1), tex=("paint", {"color": (0.08, 0.16, 0.32), "metallic": 0.7, "rough": 0.22, "seed": 2}),
          tile=0.8, double=True, role="paint")
    m.mat("seat", color=(1, 1, 1), tex=("fabric", {"color": (0.13, 0.13, 0.14)}), tile=0.2)

    R = TYRE.radius
    y_out, y_in = TRACK / 2 + TYRE.width / 2, TRACK / 2 - TYRE.width / 2
    wheels = [dict(x=XF, z=R, r=R, y_out=y_out, y_in=y_in), dict(x=XR, z=R, r=R, y_out=y_out, y_in=y_in)]

    body = CarBody(
        m, x_rear=-2.25, x_front=2.30,
        W=[(-2.25, 0.86), (-1.9, 0.895), (-1.2, 0.91), (1.3, 0.91), (1.9, 0.895), (2.30, 0.84)],
        zb=[(-2.25, 0.30), (-1.95, 0.24), (-1.0, 0.17), (1.0, 0.17), (1.9, 0.22), (2.30, 0.26)],
        zt=[(-2.25, 0.93), (-1.9, 1.01), (-1.55, 1.03), (-1.0, 1.0), (0.95, 0.95), (1.6, 0.87), (2.1, 0.80),
            (2.30, 0.72)],
        exp=[(-2.25, 4.5), (-1.8, 5.5), (1.8, 5.5), (2.30, 4.0)],
        end_r=(0.14, 0.18), wheels=wheels,
        green=dict(x0=-1.58, x1=0.97,
                   zroof=[(-1.58, 1.03), (-1.2, 1.28), (-0.85, 1.41), (-0.3, 1.45), (0.15, 1.425), (0.55, 1.2),
                          (0.97, 0.955)],
                   wb=[(-1.58, 0.80), (-1.2, 0.85), (0.6, 0.85), (0.97, 0.82)],
                   wr=[(-1.58, 0.62), (-0.9, 0.64), (0.2, 0.64), (0.97, 0.66)],
                   r=0.09, crown=0.04))

    W = body.W

    # ---- opening panels
    doors = [("door_FL", 1, -0.08, 0.93), ("door_RL", 1, -0.97, -0.10),
             ("door_FR", -1, -0.08, 0.93), ("door_RR", -1, -0.97, -0.10)]

    def door_inside(side, x0, x1):
        def f(c, n):
            return (x0 - 0.012 < c[0] < x1 + 0.012 and c[1] * side > 0 and n[1] * side > 0.3 and n[2] < 0.75
                    and c[2] > 0.20 and not body.in_arch(c))
        return f

    def door_border(side, x0, x1):
        def f(c, n):
            return c[0] < x0 + 0.008 or c[0] > x1 - 0.008 or c[2] < 0.235
        return f

    for name, side, x0, x1 in doors:
        body.panels.append(Panel(name, door_inside(side, x0, x1), door_border(side, x0, x1)))

    body.panels.append(Panel(
        "hood", lambda c, n: c[0] > 0.99 and body.ls > 0.60 and n[2] > 0.3,
        lambda c, n: c[0] < 1.015 or body.ls < 0.625 or n[2] < 0.5, inner_skin=False))
    body.panels.append(Panel(
        "trunk", lambda c, n: -2.18 < c[0] < -1.56 and body.ls > 0.62 and n[2] > 0.3,
        lambda c, n: c[0] > -1.585 or body.ls < 0.645 or n[2] < 0.55,
        inner_skin=False))

    def lower(c, n, b):
        x, y, z = c
        ay = abs(y)
        if n[2] < -0.6:
            return "underbody"
        if x < -1.9 and z < 0.36 and n[0] < -0.25:
            return "plastic_black"
        if x > 1.9 and z < 0.3 and n[0] > 0.3:
            return "plastic_black"
        return Ellipsis

    def green(c, n, b):
        x, y, z = c
        seg, pos = b.gseg, b.gpos
        if seg == "roof":  # windshield / roof / rear window, with a black ceramic border
            if x > 0.2:
                return "trim_black" if pos < 0.12 or x < 0.26 or x > 0.9 else "glass"
            if x < -0.9:
                return "trim_black" if pos < 0.12 or x > -0.96 or x < -1.5 else "glass"
            return "paint"
        if seg == "side":
            side = 1 if y > 0 else -1
            for name, s, x0, x1 in doors:
                if s == side and x0 < x < x1:
                    if 0.17 < pos < 0.9 and x0 + 0.03 < x < x1 - 0.03:
                        return (name, "glass")
                    return (name, "trim_black")
            return "trim_black" if x > -1.05 else "paint"
        return "paint"

    body.build(lower, green)

    # ---- lamps, grille, plate: patches draped onto the body surface
    rr = M.rrect
    body.patch(rr(0.64, 0.17, 0.04, 4, 0.0, 0.535), "grille", "front", skirt_mat="chrome", offset=0.002)
    body.patch(rr(1.0, 0.09, 0.04, 4, 0.0, 0.33), "grille", "front", offset=0.002)
    for s in (1, -1):
        ind = "light_indicator_left" if s > 0 else "light_indicator_right"
        body.patch(rr(0.38, 0.1, 0.035, 4, s * 0.6, 0.655), "light_head", "front", skirt_mat="chrome")
        body.patch(M.circle(0.032, 16, s * 0.5, 0.655), "chrome", "front", offset=0.007, skirt=0)
        body.patch(M.circle(0.026, 16, s * 0.64, 0.655), "chrome", "front", offset=0.007, skirt=0)
        body.patch(rr(0.07, 0.07, 0.02, 3, s * 0.755, 0.655), ind, "front", offset=0.006, skirt=0)
        body.patch(rr(0.16, 0.05, 0.02, 3, s * 0.62, 0.33), "trim_black", "front", offset=0.004)  # fog lamp bezel
        # rear cluster
        body.patch(rr(0.40, 0.13, 0.03, 4, s * 0.6, 0.83), "light_tail", "rear", skirt_mat="trim_black")
        body.patch(rr(0.28, 0.045, 0.015, 3, s * 0.62, 0.86), "light_brake", "rear", offset=0.006, skirt=0)
        body.patch(rr(0.07, 0.1, 0.02, 3, s * 0.755, 0.83), ind, "rear", offset=0.006, skirt=0)
        body.patch(rr(0.07, 0.045, 0.015, 3, s * 0.45, 0.805), "light_reverse", "rear", offset=0.006, skirt=0)
    body.patch(rr(0.52, 0.11, 0.01, 2, 0.0, 0.66), "plate", "rear", skirt_mat="trim_black")

    # ---- corners
    for key, x, y, steer in (("FL", XF, TRACK / 2, True), ("FR", XF, -TRACK / 2, True),
                             ("RL", XR, TRACK / 2, False), ("RR", XR, -TRACK / 2, False)):
        corner(m, key, (x, y, R), TYRE, steer=steer, steer_limits=(-36, 36), travel=(-0.09, 0.09))

    # ---- panel pivots: create nodes (geometry already tagged with their names)
    for name, side, x0, x1 in doors:
        hx = x1
        m.node(name, None, (hx, side * (W(hx) - 0.02), 0.62),
               m.revolute((0, 0, -side), 0, 70, "door opening angle"))
        # door handle
        hx2 = x0 + 0.12
        M.rounded_box(m.mb, (hx2, side * (W(hx2) + 0.004), 0.86), (0.16, 0.025, 0.03), 0.01, "chrome", name)
    m.node("hood", None, (1.0, 0.0, body.zt(1.0)), m.revolute((0, -1, 0), 0, 60, "hood opening angle"))
    m.node("trunk", None, (-1.57, 0.0, body.zt(-1.57)), m.revolute((0, 1, 0), 0, 70, "trunk lid opening angle"))

    # mirrors on the front doors
    for name, side in (("door_FL", 1), ("door_FR", -1)):
        mirror(m, name, (0.8, side * (W(0.8) - 0.03), 1.01), side, size=(0.09, 0.16, 0.1), arm=0.02)

    # ---- interior (static, on the body)
    mb = m.mb
    M.box(mb, (-0.3, 0, 0.27), (2.3, 1.55, 0.04), "interior")                   # floor
    M.rounded_box(mb, (0.82, 0, 0.82), (0.45, 1.6, 0.26), 0.06, "interior")     # dashboard
    M.box(mb, (0.95, 0, 0.62), (0.2, 1.5, 0.4), "interior")                     # firewall
    M.rounded_box(mb, (0.2, 0, 0.42), (1.0, 0.22, 0.28), 0.05, "interior")      # centre console
    for yy in (0.38, -0.38):
        seat(m, "", (-0.25, yy, 0.52), width=0.5, recline=0.32)
    seat(m, "", (-1.1, 0.0, 0.5), width=1.3, recline=0.28, headrest=False)
    M.box(mb, (-1.52, 0, 0.99), (0.25, 1.5, 0.03), "interior")                  # parcel shelf
    steering_wheel(m, "steering_wheel", None, (0.38, 0.38, 0.86), (0.9, 0.0, -0.44), radius=0.185)

    # engine block under the hood
    M.rounded_box(mb, (1.6, 0, 0.62), (0.55, 0.75, 0.38), 0.05, "engine")
    M.rounded_box(mb, (1.6, 0, 0.8), (0.45, 0.55, 0.06), 0.02, "plastic_black")

    # wipers: rest along the cowl pointing +y, axis = windshield normal (+ sweeps up)
    ws_n = norm((0.5, 0.0, 0.87))
    for name, py, L in (("wiper_L", 0.02, 0.58), ("wiper_R", -0.55, 0.52)):
        p = (0.93, py, 0.975)
        m.node(name, None, p, m.revolute(ws_n, 0, 95, "wiper sweep"))
        M.box(mb, (p[0] - 0.01, py + L / 2, p[2] + 0.012), (0.018, L, 0.012), "trim_black", name)
        M.box(mb, (p[0] - 0.02, py + L / 2 + 0.03, p[2] + 0.004), (0.012, L * 0.85, 0.01), "plastic_black", name)
        M.cylinder(mb, p, (p[0] + 0.01, p[1], p[2] + 0.025), 0.018, "trim_black", name, count=10)

    # exhaust
    M.cylinder(mb, (-2.0, 0.55, 0.28), (-2.28, 0.55, 0.28), 0.035, "chrome", count=14)

    # sockets
    m.socket("socket_driver_eye", None, (-0.35, 0.38, 1.18), "driver eye point (camera)")
    m.socket("socket_tow_rear", None, (-2.22, 0.0, 0.38), "rear towing eye")
    m.socket("socket_tow_front", None, (2.28, 0.25, 0.32), "front towing eye")
    m.notes.append("Generic mid-size sedan; dimensions are class-typical, not a specific production car.")
    return m
