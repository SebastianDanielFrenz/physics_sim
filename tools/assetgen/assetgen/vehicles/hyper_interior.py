"""Fitted two-seat cockpit for Sculpted A; all coordinates are ISO metres."""
import math
from .. import mesh as M
from ..parts import seat, steering_wheel

def build_cabin(m, body):
    mb=m.mb
    m.mat('cabin_leather',color=(.15,.16,.18),roughness=.78,double=True)
    m.mat('cabin_suede',color=(.20,.21,.23),roughness=.95,double=True)
    m.mat('cabin_carpet',color=(.10,.11,.12),roughness=1.0,double=True)
    m.mat('cabin_metal',color=(.58,.60,.64),metallic=.85,roughness=.3)
    m.mat('cabin_stitch',color=(.64,.64,.60),roughness=.8)
    m.mat('display_black',color=(.018,.025,.03),roughness=.24)
    m.mat('display_white',color=(.75,.92,1),roughness=.3,emissive=(.3,.65,.8))
    # Closed tub, full-height front/rear firewalls and fixed rear quarter trim.
    M.rounded_box(mb,(-.10,0,.185),(1.90,1.38,.06),.02,'cabin_carpet')
    M.rounded_box(mb,(-.98,0,.575),(.075,1.42,.80),.02,'cabin_suede')
    M.rounded_box(mb,(.87,0,.475),(.08,1.42,.59),.015,'cabin_carpet')
    for side in (1,-1):
        M.rounded_box(mb,(-.79,side*.67,.52),(.38,.10,.66),.025,'cabin_leather')
        # Enclosed front wheel-well backs, behind the footwell/firewall.
        M.rounded_box(mb,(.77,side*.63,.43),(.22,.18,.48),.025,'cabin_carpet')
        M.rounded_box(mb,(-.13,side*.65,.24),(1.72,.13,.16),.035,'cabin_leather')
        name='door_L' if side>0 else 'door_R'
        # The door card is its own closed loft, attached to the butterfly door.
        # Top follows the belt; card is inset from the body, under side glass.
        rows=[]
        for x in (-.57,-.42,0,.45,.73,.87):
            top=min(.80,body.zt(x)-.035)
            outer=.72 if x<.55 else .68
            rows.append([(x,side*.625,.24),(x,side*outer,.24),
                         (x,side*outer,top),(x,side*.625,top)])
        M.loft(mb,rows,'cabin_leather',name)
        M.rounded_box(mb,(-.05,side*.60,.51),(.75,.095,.065),.025,'cabin_suede',name)
        M.rounded_box(mb,(.34,side*.603,.65),(.17,.015,.035),.01,'cabin_metal',name)
        # Speaker grille, armrest buttons, stitching and lower pocket.
        M.rounded_box(mb,(-.06,side*.606,.34),(.58,.02,.10),.025,'trim_black',name)
        for k in range(12):
            M.box(mb,(.60,side*.620,.30+k*.012),(.105,.007,.004),'trim_black',name)
        for x in (.04,.10):
            M.rounded_box(mb,(x,side*.595,.548),(.035,.025,.012),.004,'trim_black',name)
        M.tube(mb,[(-.42,side*.616,.70),(-.1,side*.616,.725),(.35,side*.616,.72)],
               .0015,'cabin_stitch',name,count=6)
    # Inner roof only. Side windows remain clear; ceramic pillars get inner trim.
    rows=[]
    for k in range(33):
        x=-.62+.92*k/32
        width=body.green['wr'](x)-.04
        zr=body.green['zroof'](x)
        rows.append([(x,width*(2*j/16-1),zr+.035*(1-(2*j/16-1)**2)-.028)
                     for j in range(17)])
    # Six-millimetre headliner with closed perimeter returns, not a floating sheet.
    M.loft(mb,[row+[(x,y,z+.006) for x,y,z in reversed(row)] for row in rows],'cabin_suede')
    for side in (1,-1):
        M.tube(mb,[(.91,side*.60,.76),(.68,side*.54,.92),(.36,side*.49,1.075)],
               .019,'cabin_suede',count=10)
        M.tube(mb,[(-.60,side*.56,.83),(-.64,side*.48,1.01)],.022,'cabin_suede',count=10)
    # Sculpted padded dash. Broad end caps hide wheel housings from both seats.
    M.rounded_box(mb,(.65,0,.645),(.42,1.34,.14),.055,'cabin_suede')
    M.rounded_box(mb,(.75,0,.715),(.24,1.27,.055),.022,'cabin_leather')
    M.rounded_box(mb,(.47,0,.575),(.10,1.27,.08),.025,'cabin_leather')
    M.tube(mb,[(.446,-.59,.627),(.44,0,.625),(.446,.59,.627)],.004,'cabin_metal',count=8)
    # Fitted digital cluster surround; legacy analogue binnacle removed.
    M.rounded_box(mb,(.569,.33,.790),(.052,.43,.180),.018,'cabin_leather')
    M.rounded_box(mb,(.546,.33,.790),(.008,.410,.166),.010,'trim_black')
    # Centre infotainment screen, climate controls and narrow console.
    M.rounded_box(mb,(.428,-.045,.623),(.025,.24,.13),.018,'trim_black')
    M.box(mb,(.412,-.045,.623),(.003,.207,.094),'display_black')
    for k in range(4):
        M.box(mb,(.409,-.12+k*.05,.635),(.002,.026,.003),'display_white')
    for yy in (-.57,-.29,.60):
        M.rounded_box(mb,(.414,yy,.655),(.018,.12,.04),.01,'trim_black')
        for k in range(4):
            M.box(mb,(.402,yy-.042+k*.028,.655),(.004,.003,.027),'cabin_metal')
    M.rounded_box(mb,(.05,0,.33),(1.35,.24,.25),.045,'cabin_leather')
    M.rounded_box(mb,(.09,0,.462),(.72,.18,.025),.025,'carbon')
    M.tube(mb,[(.14,0,.47),(.14,0,.54)],.010,'cabin_metal',count=12)
    M.rounded_box(mb,(.14,0,.548),(.075,.045,.035),.012,'cabin_leather')
    for yy in (-.065,.065):
        M.cylinder(mb,(.34,yy,.469),(.34,yy,.484),.019,'cabin_metal',count=20)
    for yy in (.33,-.33):
        seat(m,'',(-.28,yy,.27),width=.48,recline=.55,height=.10)
    steering_wheel(m,'steering_wheel',None,(.30,.33,.62),(.95,0,-.3),radius=.17,lock_deg=360)
    # Driver footwell pedals and passenger footrest.
    for yy in (.23,.33,.43):
        M.rounded_box(mb,(.68,yy,.255),(.028,.06,.09),.006,'cabin_metal')
    M.rounded_box(mb,(.67,-.33,.25),(.08,.34,.08),.018,'cabin_carpet')
