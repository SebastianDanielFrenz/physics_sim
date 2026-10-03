extends Node3D
const TyreModel = preload("res://scripts/tyre_sound_model.gd")
const TurboModel = preload("res://scripts/turbo_sound_model.gd")
const RATE := 24000.0
const TARGET_S := 0.05
var simulation: Node
var vehicle_name := "car"
var chassis_name := "chassis"
var enable_tyres := true
var enable_turbos := false
var wetness := 0.0
var surface_wetness: Dictionary = {}
var voices: Array = []
var players: Array[AudioStreamPlayer3D] = []
var capacity: Array[int] = []
var gains: Array[float] = []
var tyre_count := 0
var skipped := 0
var cabin_mix := 0.0
var cabin_low: Array[float] = []
const CABIN_TYRE_GAIN := 0.12589254 # -18 dB before low-pass filtering.
const CABIN_CUTOFF_HZ := 900.0

func _listener_in_cabin() -> bool:
    var main := get_parent()
    if main == null:
        return false
    if main.get("vr_active") == true:
        return true
    var cockpit := main.get("cockpit_camera") as Camera3D
    return cockpit != null and cockpit.current

func set_cabin_listener(inside: bool, delta: float) -> void:
    cabin_mix = lerpf(cabin_mix, 1.0 if inside else 0.0, 1.0 - exp(-delta / 0.12))

func _filter_tyre(raw: float, wheel: int, dt: float) -> float:
    cabin_low[wheel] += (raw - cabin_low[wheel]) * (1.0 - exp(-TAU * CABIN_CUTOFF_HZ * dt))
    return lerpf(raw, cabin_low[wheel] * CABIN_TYRE_GAIN, cabin_mix)

func set_surface_wetness(surface: String, amount: float) -> void:
    surface_wetness[surface] = clampf(amount, 0.0, 1.0)

func _ready() -> void:
    # Synthetic turbo cues rejected in audition; keep muted until a usable sample exists.
    enable_turbos = false
    tyre_count = simulation.get_vehicle_wheel_count(vehicle_name) if enable_tyres else 0
    for wheel in range(tyre_count):
        var attachment: Vector3 = simulation.get_wheel_attachment_local(vehicle_name, wheel)
        attachment.z -= simulation.get_wheel_radius(vehicle_name, wheel)
        _add_voice(TyreModel.new(1234 + wheel * 997), attachment)
    if enable_turbos:
        _add_voice(TurboModel.new(9847), Vector3(-0.7, 0.0, 0.15))

func _add_voice(model: RefCounted, point: Vector3) -> void:
    var stream := AudioStreamGenerator.new()
    stream.mix_rate = RATE
    stream.buffer_length = 0.15
    var player := AudioStreamPlayer3D.new()
    player.stream = stream
    player.position = point
    player.unit_size = 5.0
    player.max_distance = 100.0
    player.volume_db = -3.0
    add_child(player)
    player.play()
    voices.append(model)
    players.append(player)
    capacity.append(0)
    gains.append(0.0)
    cabin_low.append(0.0)

func _process(delta: float) -> void:
    if simulation == null:
        return
    set_cabin_listener(_listener_in_cabin(), delta)
    global_transform = simulation.get_body_transform(chassis_name)
    var speed: float = simulation.get_vehicle_ground_speed_mps(vehicle_name)
    for wheel in range(tyre_count):
        var surface: String = simulation.get_wheel_surface_name(vehicle_name, wheel)
        var radius: float = simulation.get_wheel_radius(vehicle_name, wheel)
        voices[wheel].update(speed, simulation.get_wheel_omega(vehicle_name, wheel) * radius,
            simulation.get_wheel_slip_ratio(vehicle_name, wheel), simulation.get_wheel_slip_angle(vehicle_name, wheel),
            simulation.get_wheel_load_n(vehicle_name, wheel), simulation.get_wheel_fx(vehicle_name, wheel),
            simulation.get_wheel_fy(vehicle_name, wheel), surface, float(surface_wetness.get(surface, wetness)))
    if enable_turbos:
        var pt: Dictionary = simulation.get_vehicle_powertrain(vehicle_name)
        voices[tyre_count].update(delta, float(pt.get("rpm", 0.0)), simulation.get_control("throttle"),
            float(pt.get("fuel_flow_g_s", 0.0)), pt.get("engine_state", "off") == "running",
            float(pt.get("turbo_rpm", -1.0)), float(pt.get("boost_bar", -1.0)))
    for i in range(players.size()):
        var playback := players[i].get_stream_playback() as AudioStreamGeneratorPlayback
        if playback == null:
            continue
        skipped = maxi(skipped, playback.get_skips())
        var available := playback.get_frames_available()
        capacity[i] = maxi(capacity[i], available)
        var frames := mini(available, maxi(0, int(RATE * TARGET_S) - (capacity[i] - available)))
        var buffer := PackedVector2Array()
        buffer.resize(frames)
        for frame in range(frames):
            var raw: float = voices[i].sample(1.0 / RATE)
            if i < tyre_count:
                raw = _filter_tyre(raw, i, 1.0 / RATE)
            # Short output slew suppresses contact/surface update discontinuities.
            gains[i] += (raw - gains[i]) * 0.65
            buffer[frame] = Vector2(gains[i], gains[i])
        playback.push_buffer(buffer)

func shutdown() -> void:
    set_process(false)
    for player in players:
        player.stop()
        player.stream = null
    players.clear()
    voices.clear()
func _exit_tree() -> void:
    shutdown()
