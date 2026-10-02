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
    require(energy(tyre(30,0,"asphalt",1)) > fast, "wet road adds water layer")
    require(energy(tyre(30,0,"dirt",0)) != fast, "surface changes noise")
    var burnout := Tyre.new(42)
    burnout.update(0,40,1,0,4000,3000,0,"asphalt",0)
    require(energy(burnout) > 0.0, "stationary burnout audible")
    var a := tyre(20,0.5,"grass",0.5)
    var b := tyre(20,0.5,"grass",0.5)
    for i in range(1000):
        require(a.sample(1.0/24000) == b.sample(1.0/24000), "seed reproducibility")
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