extends RefCounted
# Authored procedural sound, not an extra tyre/weather simulation.
const PROFILES = {
    "asphalt": [1.0, 1.0, 1100.0, 0.0],
    "kerb": [1.65, 0.75, 1350.0, 0.15],
    "dirt": [2.0, 0.12, 650.0, 0.7],
    "grass": [1.1, 0.05, 450.0, 0.35],
    "field": [1.7, 0.03, 380.0, 0.65],
    "low_mu": [0.55, 0.2, 1500.0, 0.0]
}
var rng := RandomNumberGenerator.new()
var roll := 0.0
var slide := 0.0
var water := 0.0
var tone_gain := 0.0
var frequency := 1000.0
var roll_cutoff := 850.0
var low := 0.0
var phase := 0.0
var grain := 0.0
var contact := false

func _init(seed_value: int = 1) -> void:
    rng.seed = seed_value

func update(speed: float, tread_speed: float, slip_ratio: float, slip_angle: float,
        load_n: float, fx: float, fy: float, surface: String, wetness: float) -> void:
    var profile: Array = PROFILES.get(surface, PROFILES.asphalt)
    var wet := clampf(wetness, 0.0, 1.0)
    var v := absf(speed)
    contact = load_n > 20.0
    var load_scale := sqrt(clampf(load_n / 4000.0, 0.0, 2.0)) if contact else 0.0
    var longitudinal := maxf(v, absf(tread_speed)) * minf(absf(slip_ratio), 1.0)
    var lateral := v * absf(sin(slip_angle))
    # Slip work proxy from real forces/slips, not an invented friction force.
    var work := absf(fx) * longitudinal + absf(fy) * lateral
    var scrub := clampf(sqrt(work / 15000.0), 0.0, 1.8)
    if longitudinal + lateral < 0.05:
        scrub = 0.0
    roll = 0.014 * float(profile[0]) * pow(minf(v / 30.0, 3.0), 0.7) * load_scale
    slide = 0.035 * scrub * load_scale
    water = 0.027 * wet * pow(minf(v / 30.0, 3.0), 0.8) * load_scale
    tone_gain = float(profile[1]) * (1.0 - wet * 0.7)
    frequency = clampf(float(profile[2]) + 10.0 * v + 35.0 * sqrt(longitudinal + lateral), 200.0, 5000.0)
    grain = float(profile[3])
    roll_cutoff = clampf(250.0 + v * 32.0 + grain * 450.0, 200.0, 5000.0)

func sample(dt: float) -> float:
    var noise := rng.randf_range(-1.0, 1.0)
    low += (1.0 - exp(-TAU * roll_cutoff * dt)) * (noise - low)
    phase = fmod(phase + frequency * dt, 1.0)
    var rough := low + grain * (noise - low)
    var skid := 0.65 * sin(TAU * phase) * tone_gain + 0.7 * (noise - low)
    var value := rough * roll + skid * slide + (noise - low) * water
    return clampf(value, -0.4, 0.4) if contact else 0.0
