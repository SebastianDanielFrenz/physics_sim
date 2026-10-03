extends SceneTree
const Tyre = preload("res://scripts/tyre_sound_model.gd")
const Turbo = preload("res://scripts/turbo_sound_model.gd")
const Player = preload("res://scripts/vehicle_surface_audio.gd")
var failures := 0
func require(condition: bool, label: String) -> void:
    if not condition:
        push_error(label)
        failures += 1
func energy(model: RefCounted) -> float:
    var total := 0.0
    for i in range(24000):
        var v: float = model.sample(1.0 / 24000.0)
        require(is_finite(v), "nonfinite sample")
        total += v * v
    return total / 24000.0
func tyre(speed: float, slip: float, surface: String, wet: float, load_n: float = 4000.0) -> RefCounted:
    var t := Tyre.new(42)
    t.update(speed, speed + 10.0 * slip, slip, 0.25 * slip, load_n, 2500.0, 2000.0, surface, wet)
    return t
func _initialize() -> void:
    require(energy(tyre(0,0,"asphalt",0)) == 0.0, "stationary rolling silence")
    require(energy(tyre(30,0.5,"asphalt",0,0)) == 0.0, "unloaded wheel silence")
    var slow := energy(tyre(5,0,"asphalt",0))
    var fast := energy(tyre(30,0,"asphalt",0))
    var drift := energy(tyre(30,0.8,"asphalt",0))
    require(fast > slow * 2.0, "rolling energy grows with speed")
    require(drift > fast * 2.0, "drift energy grows with slip")
    require(energy(tyre(30,0,"asphalt",1)) == fast, "rolling baseline preserved in wet conditions")
    require(energy(tyre(30,0,"dirt",0)) == fast, "explicit generic surface fallback")
    var braking := Tyre.new(42)
    braking.update(40,38,0.05,0,4000,-2500,0,"asphalt",0)
    require(braking.slide == 0.0, "ordinary braking has no squeal even at highway speed")
    require(energy(braking) == energy(tyre(40,0,"asphalt",0)), "ordinary braking preserves rolling-only audio")
    var onset := Tyre.new(42)
    onset.update(30,26.7,0.11,0,4000,-4000,0,"asphalt",0)
    var locked := Tyre.new(42)
    locked.update(30,0,1,0,4000,-4000,0,"asphalt",0)
    require(onset.slide > 0.0 and onset.slide < locked.slide * 0.1, "squeal onset gradual")
    require(locked.slide > 0.0, "locked braking wheel squeals")
    var creep := Tyre.new(42)
    creep.update(0.1,0,1,0,4000,-1000,0,"asphalt",0)
    require(creep.slide == 0.0, "low-speed slip-ratio spike stays quiet")
    var burnout := Tyre.new(42)
    burnout.update(0,40,1,0,4000,3000,0,"asphalt",0)
    require(energy(burnout) > 0.0, "stationary burnout audible")
    var a := tyre(20,0.5,"grass",0.5)
    var b := tyre(20,0.5,"grass",0.5)
    for i in range(1000):
        require(a.sample(1.0/24000) == b.sample(1.0/24000), "seed reproducibility")
    var cabin := Player.new()
    cabin.cabin_low.append(0.0)
    require(cabin._filter_tyre(0.25,0,1.0/24000) == 0.25, "exterior tyre signal unchanged")
    cabin.set_cabin_listener(true,0.01)
    require(cabin.cabin_mix > 0.0 and cabin.cabin_mix < 0.1, "camera transition fades")
    cabin.set_cabin_listener(true,2.0)
    var cabin_energy := 0.0
    for i in range(24000):
        var filtered: float = cabin._filter_tyre(sin(TAU*4000*i/24000.0),0,1.0/24000)
        cabin_energy += filtered * filtered / 24000.0
    require(cabin_energy < 0.002, "cabin attenuates high-frequency tyre sound")
    cabin.set_cabin_listener(false,2.0)
    require(cabin.cabin_mix < 0.0001, "exterior mix restored")
    cabin.free()
    var turbo := Turbo.new(7)
    for i in range(120):
        turbo.update(1.0/60,6500,1,70,true)
    require(turbo.spool > 0.5, "spool cue builds under load")
    turbo.update(1.0/60,6500,0,0,true)
    require(turbo.release > 0.1, "lift-off release cue")
    for i in range(600):
        turbo.update(1.0/60,0,0,0,false)
    require(turbo.spool < 0.001 and turbo.release < 0.001, "shutdown decay")
    var measured := Turbo.new(7)
    measured.update(1.0,1000,0,0,false,130000,1.8)
    require(not measured.estimated and measured.spool > 0.9, "real telemetry overrides cue")
    print("surface/turbo audio checks: ", failures, " failures; slow=",slow," fast=",fast," drift=",drift)
    quit(0 if failures == 0 else 1)