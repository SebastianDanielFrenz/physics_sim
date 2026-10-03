extends Node3D
var simulation: Node
var vehicle_name: String
func _ready() -> void:
	var viewport := SubViewport.new()
	viewport.size = Vector2i(1024,400)
	viewport.disable_3d = true
	viewport.render_target_update_mode = SubViewport.UPDATE_ALWAYS
	add_child(viewport)
	var cluster := Control.new()
	cluster.set_script(load("res://scripts/digital_cluster.gd"))
	cluster.set("simulation",simulation)
	cluster.set("vehicle_name",vehicle_name)
	cluster.size = Vector2(1024,400)
	viewport.add_child(cluster)
	var panel := MeshInstance3D.new()
	var quad := QuadMesh.new()
	quad.size = Vector2(0.40,0.15625)
	panel.mesh = quad
	panel.position = Vector3(0.33,0.790,0.540)
	panel.rotation.y = PI
	var material := StandardMaterial3D.new()
	material.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	material.albedo_texture = viewport.get_texture()
	material.texture_filter = BaseMaterial3D.TEXTURE_FILTER_LINEAR
	panel.material_override = material
	add_child(panel)
