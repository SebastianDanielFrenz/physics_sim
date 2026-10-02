extends RefCounted
# Sound envelope ONLY. No simulated turbo RPM/boost is available. Never export
# these proxies to physics telemetry or the dashboard as measured quantities.
var rng := RandomNumberGenerator.new()
var spool := 0.0
var release := 0.0
var phase := 0.0
var phase2 := 0.0
var low := 0.0
var previous_throttle := 0.0
var estimated := true

func _init(seed_value: int = 100) -> void:
    rng.seed = seed_value

func update(dt: float, engine_rpm: float, throttle: float, fuel_g_s: float,
        running: bool, turbo_rpm: float = -1.0, boost_bar: float = -1.0) -> void:
    estimated = turbo_rpm < 0.0
    var target := clampf(turbo_rpm / 130000.0, 0.0, 1.2) if not estimated else         clampf(engine_rpm / 7500.0, 0.0, 1.1) * sqrt(clampf(throttle, 0.0, 1.0))
    if estimated and (not running or fuel_g_s <= 0.0):
        target = 0.0
    spool = lerpf(spool, target, 1.0 - exp(-dt / (0.32 if target > spool else 0.65)))
    # A real boost input wins if supplied later; today this is only an audio cue.
    var pressure_cue := clampf(boost_bar / 1.8, 0.0, 1.5) if boost_bar >= 0.0 else spool * spool
    if previous_throttle - throttle > 0.15 and pressure_cue > 0.04:
        release = maxf(release, pressure_cue)
    previous_throttle = throttle
    release *= exp(-dt / 0.18)

func sample(dt: float) -> float:
    var noise := rng.randf_range(-1.0, 1.0)
    low += (1.0 - exp(-TAU * 1300.0 * dt)) * (noise - low)
    phase = fmod(phase + (700.0 + 2600.0 * spool) * dt, 1.0)
    phase2 = fmod(phase2 + (730.0 + 2730.0 * spool) * dt, 1.0)
    # Twin-tone beating plus filtered broadband flow and pressure-release hiss.
    var whine := (sin(TAU * phase) + sin(TAU * phase2)) * 0.005 * spool * spool
    return whine + (noise - low) * (0.012 * spool * spool + 0.06 * release)
