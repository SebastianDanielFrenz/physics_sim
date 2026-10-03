extends Control
var simulation: Node
var vehicle_name: String
var speed := 0.0
var rpm := 0.0
var drive_kw := 0.0
var boost_bar := -1.0
var gear := 0
var park_engaged := false
var capacity := 0.0
var density := 0.0
var fuel := 0.0
var initial_fuel := -1.0
var distance := 0.0
var flow_lph := 0.0
var elapsed := 0.0
var font := ThemeDB.fallback_font

func _ready() -> void:
	var info: Dictionary = simulation.get_vehicle_gauge_info(vehicle_name)
	capacity = info.get("fuel_capacity_kg",0.0)
	density = info.get("fuel_density_kg_m3",0.0)

func _process(delta: float) -> void:
	var ground_speed: float = simulation.get_vehicle_ground_speed_mps(vehicle_name)
	distance += maxf(ground_speed,0.0)*delta
	elapsed += delta
	if elapsed < 0.05:
		return
	var dt := elapsed
	elapsed = 0.0
	var pt: Dictionary = simulation.get_vehicle_powertrain(vehicle_name)
	speed = ground_speed*3.6
	rpm = pt.get("rpm",0.0)
	drive_kw = pt.get("drive_power_kw",0.0)
	boost_bar = pt.get("boost_bar",-1.0)
	gear = int(pt.get("gear",0))
	park_engaged = bool(pt.get("park_lock_engaged",false))
	fuel = pt.get("fuel_mass_kg",0.0)
	if capacity>0.0 and (initial_fuel<0.0 or fuel>initial_fuel+0.01):
		initial_fuel = fuel
		distance = 0.0
	if density>0.0:
		var flow: float = pt.get("fuel_flow_g_s",0.0)*3600.0/density
		flow_lph = lerpf(flow_lph,flow,1.0-exp(-dt/0.5))
	queue_redraw()

func text(value: String, at: Vector2, size: int, color := Color.WHITE) -> void:
	at.x -= font.get_string_size(value,HORIZONTAL_ALIGNMENT_LEFT,-1,size).x/2.0
	draw_string(font,at,value,HORIZONTAL_ALIGNMENT_LEFT,-1,size,color)

func dial(center: Vector2,value: float,maximum: float,color: Color,is_rpm: bool) -> void:
	var a := deg_to_rad(135.0)
	var b := deg_to_rad(405.0)
	draw_arc(center,125,a,b,100,Color(0.12,0.20,0.27),12,true)
	draw_arc(center,125,a,a+(b-a)*clampf(value/maximum,0,1),100,color,12,true)
	if is_rpm:
		draw_arc(center,125,a+(b-a)*8.0/9.0,b,20,Color(1,0.25,0.16),12,true)
	for k in range(10):
		var angle := a+(b-a)*k/9.0
		var direction := Vector2(cos(angle),sin(angle))
		draw_line(center+direction*105,center+direction*114,Color(0.65,0.75,0.85),2,true)
		text(str(k) if is_rpm else str(k*40),center+direction*86+Vector2(0,6),18,Color(0.65,0.75,0.85))

func small_dial(center: Vector2, value: String, title: String, fraction: float) -> void:
	draw_arc(center,45,deg_to_rad(135),deg_to_rad(405),48,Color(0.15,0.27,0.34),5,true)
	if fraction>=0:
		draw_arc(center,45,deg_to_rad(135),deg_to_rad(135)+deg_to_rad(270)*clampf(fraction,0,1),48,Color(0.6,0.85,1),5,true)
	text(value,center+Vector2(0,3),28)
	text(title,center+Vector2(0,25),15,Color(0.5,0.7,0.82))

func _draw() -> void:
	draw_rect(Rect2(0,0,1024,400),Color(0.008,0.016,0.026))
	var blue := Color(0.5,0.7,0.82)
	var has_fuel := capacity>0 and density>0
	var fraction := clampf(fuel/capacity,0,1) if has_fuel else 0.0
	text("SCULPTED A",Vector2(503,29),18,blue)
	# The sketch: vertical fuel strip, two dominant dials and stacked auxiliaries.
	text("FUEL",Vector2(48,59),18,blue)
	draw_rect(Rect2(38,78,20,180),Color(0.15,0.22,0.28))
	draw_rect(Rect2(38,78+180*(1-fraction),20,180*fraction),Color(1,0.55,0.15) if fraction<0.15 else Color(0.3,0.8,1))
	text("F",Vector2(75,93),16,blue)
	text("E",Vector2(75,259),16,blue)
	text("%.0f%%" % (fraction*100) if has_fuel else "--",Vector2(48,286),20)
	dial(Vector2(245,170),speed,360,Color(0.25,0.7,1),false)
	dial(Vector2(735,170),rpm,9000,Color(0.2,0.95,0.75),true)
	text("%.0f" % speed,Vector2(245,180),68)
	text("km/h",Vector2(245,214),22,blue)
	text("%.0f" % rpm,Vector2(735,180),58)
	text("RPM",Vector2(735,214),22,blue)
	text("P" if park_engaged else ("R" if gear<0 else ("N" if gear==0 else str(gear))),Vector2(490,174),90)
	text("GEAR",Vector2(490,207),18,blue)
	small_dial(Vector2(935,109),"%.0f" % drive_kw,"DRIVE kW",maxf(drive_kw,0)/650.0)
	small_dial(Vector2(935,241),"%.1f" % boost_bar if boost_bar>=0 else "--","BOOST bar",boost_bar/3.0 if boost_bar>=0 else -1)
	# Dynamic information strip, reserved for selectable trip pages later.
	draw_rect(Rect2(102,311,903,78),Color(0.025,0.045,0.061))
	text("TRIP",Vector2(181,338),16,blue)
	text("%.1f km" % (distance/1000),Vector2(181,369),24)
	var instant := "%.1f L/100km" % (flow_lph*100/speed) if speed>=5 else "%.1f L/h" % flow_lph
	text("INSTANT",Vector2(362,338),16,blue)
	text(instant if has_fuel else "--",Vector2(362,369),24)
	var used := maxf(initial_fuel-fuel,0)*1000/density if has_fuel else 0.0
	var average := used*100000/distance if distance>=100 else 0.0
	text("AVERAGE",Vector2(562,338),16,blue)
	text("%.1f L/100km" % average if average>0 else "--",Vector2(562,369),24)
	text("RANGE",Vector2(753,338),16,blue)
	text("%.0f km" % (fuel*1000/density*100/average) if average>0 else "--",Vector2(753,369),24)
	text("REMAINING",Vector2(922,338),16,blue)
	text("%.1f L" % (fuel*1000/density) if has_fuel else "--",Vector2(922,369),24)
