// adapters/godot/demo/shaders/speed_grid.gdshader — world-space 1 m / 10 m
// grid as a speed/distance reference (PLAN.md P2: "a world-space grid
// shader in Godot as the speed reference"; data/tracks/README.md: "The
// 1 m/10 m speed-and-distance grid is not a mesh texture ... it is left to
// a world-space shader in the Godot adapter").
//
// Uses VERTEX (object/world space before the origin-rebase-relative camera
// transform) so the grid is stable in world space rather than sliding with
// the floating origin - since the origin only rebases in large (>~500 m)
// steps and this ground plane is itself repositioned relative to the
// origin like everything else the adapter draws, the grid lines still line
// up across a rebase (both move together).

shader_type spatial;
render_mode unshaded, cull_disabled;

uniform float minor_spacing_m = 1.0;
uniform float major_spacing_m = 10.0;
uniform float line_width_px = 1.5;
uniform vec4 base_color : source_color = vec4(0.35, 0.42, 0.38, 1.0);
uniform vec4 minor_line_color : source_color = vec4(0.55, 0.6, 0.58, 1.0);
uniform vec4 major_line_color : source_color = vec4(0.85, 0.85, 0.3, 1.0);

varying vec3 world_pos;

void vertex() {
	world_pos = (MODEL_MATRIX * vec4(VERTEX, 1.0)).xyz;
}

float grid_line(vec2 coord, float spacing, float width) {
	vec2 g = abs(fract(coord / spacing - 0.5) - 0.5) * spacing;
	vec2 d = fwidth(coord);
	vec2 aa = max(d * width, vec2(0.0001));
	vec2 line2 = smoothstep(aa, vec2(0.0), g);
	return max(line2.x, line2.y);
}

void fragment() {
	// World frame here is Godot axes (x right, y up, z back); the ground
	// plane lies in the world XZ plane, matching the sim's own flat ground
	// (z=0 in sim/world-ENU axes - see frame_convert_core.h for the basis
	// map, and collision_import.cpp's flat-plane loader for why this plane
	// sits at sim z=0).
	vec2 coord = world_pos.xz;
	float minor = grid_line(coord, minor_spacing_m, line_width_px);
	float major = grid_line(coord, major_spacing_m, line_width_px * 1.6);

	vec3 col = base_color.rgb;
	col = mix(col, minor_line_color.rgb, minor);
	col = mix(col, major_line_color.rgb, major);

	ALBEDO = col;
}
