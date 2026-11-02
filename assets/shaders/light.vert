#version 420 core

// one instanced light volume : a unit sphere for a point light, or a unit cone
// for a spot. the cone's apex is mapped to the light centre and its base ring to
// the reach at the outer angle, so the mesh is the light's real footprint

layout (location = 0) in vec3 vertex_position;

layout (location = 1) in vec4 instance_pos_radius;   // xyz centre, w radius
layout (location = 2) in vec4 instance_color_power;  // rgb colour, a intensity
layout (location = 3) in vec4 instance_dir_shape;    // xyz axis, w 0 point / 1 cone
layout (location = 4) in vec4 instance_params;       // outer radians, outer cos, inner cos

uniform mat4 proj_view;

out vec3 light_center;
out vec3 light_color;
out vec3 light_dir;
out float light_radius;
out float light_power;
out float light_outer_cos;
out float light_inner_cos;
out float light_shape;

void main()
{
	float shape = instance_dir_shape.w;
	vec3 dir = instance_dir_shape.xyz;
	if (dot(dir, dir) < 1e-6) dir = vec3(0.0, -1.0, 0.0);
	else                      dir = normalize(dir);

	vec3 center = instance_pos_radius.xyz;
	float radius = instance_pos_radius.w;

	vec3 world;
	if (shape > 0.5)
	{
		// the cone mesh is centred on Y : base at -0.5, apex at +0.5. map the
		// apex to the centre and the base to centre + dir * radius
		vec3 helper = abs(dir.y) < 0.95 ? vec3(0.0, 1.0, 0.0) : vec3(1.0, 0.0, 0.0);
		vec3 right = normalize(cross(helper, dir));
		vec3 up = cross(dir, right);
		float tan_outer = tan(instance_params.x);
		float along = 0.5 - vertex_position.y;
		world = center + dir * (along * radius)
		      + (right * vertex_position.x + up * vertex_position.z) * (tan_outer * radius * 1.02);
	}
	else
	{
		// a little larger than the reach : the smooth falloff then ends inside
		// the mesh instead of on its tessellated silhouette
		world = center + vertex_position * (radius * 1.10);
	}

	light_center = center;
	light_color = instance_color_power.rgb;
	light_power = instance_color_power.a;
	light_dir = dir;
	light_radius = radius;
	light_outer_cos = instance_params.y;
	light_inner_cos = instance_params.z;
	light_shape = shape;

	gl_Position = proj_view * vec4(world, 1.0);
}
