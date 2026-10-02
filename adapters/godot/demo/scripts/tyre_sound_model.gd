extends RefCounted
# Authored procedural sound, not an extra tyre/weather simulation.
static var squeal_pcm := PackedByteArray()
var squeal_position := 0
var squeal_level := 0.0
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
var wetness_audio := 0.0

func _init(seed_value: int = 1) -> void:
    rng.seed = seed_value
    if squeal_pcm.is_empty():
        var wav := AudioStreamWAV.load_from_file("res://audio/tyres/asphalt_squeal.wav")
        if wav != null:
            squeal_pcm = wav.data
    squeal_position = seed_value % maxi(1, squeal_pcm.size() / 2 - 1200)

func update(speed: float, tread_speed: float, slip_ratio: float, slip_angle: float,
        load_n: float, fx: float, fy: float, surface: String, wetness: float) -> void:
    # All surfaces share the accepted asphalt baseline until usable recordings exist.
    var profile: Array = [1.0, 1.0, 1100.0, 0.0]
    var wet := clampf(wetness, 0.0, 1.0)
    wetness_audio = wet
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
    water = 0.0 # Rejected synthetic spray is muted; wetness has no distinct sample yet.
    tone_gain = float(profile[1]) * (1.0 - wet * 0.7)
    frequency = clampf(float(profile[2]) + 10.0 * v + 35.0 * sqrt(longitudinal + lateral), 200.0, 5000.0)
    grain = float(profile[3])
    roll_cutoff = clampf(250.0 + v * 32.0 + grain * 450.0, 200.0, 5000.0)

func sample(dt: float) -> float:
    var noise := rng.randf_range(-1.0, 1.0)
    low += (1.0 - exp(-TAU * roll_cutoff * dt)) * (noise - low)
    phase = fmod(phase + frequency * dt, 1.0)
    var rough := low + grain * (noise - low)
    squeal_level += (slide * (1.0 - wetness_audio * 0.7) - squeal_level) * (1.0 - exp(-dt / 0.04))
    var skid := _squeal_sample()
    var value := rough * roll + skid * squeal_level + (noise - low) * water
    return clampf(value, -0.4, 0.4) if contact else 0.0

func _squeal_sample() -> float:
    var count := squeal_pcm.size() / 2
    const CROSSFADE := 1200 # 50 ms overlap, independent of frame update timing.
    if count <= CROSSFADE:
        return 0.0
    var value := float(squeal_pcm.decode_s16(squeal_position * 2)) / 32768.0
    if squeal_position >= count - CROSSFADE:
        var head := squeal_position - (count - CROSSFADE)
        var blend := float(head) / CROSSFADE
        value = lerpf(value, float(squeal_pcm.decode_s16(head * 2)) / 32768.0, blend)
    squeal_position += 1
    if squeal_position >= count:
        squeal_position = CROSSFADE
    return value
