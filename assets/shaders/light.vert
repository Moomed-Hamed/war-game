#version 420 core

// one instanced light volume : a unit sphere at the light centre. the mesh is a
// little larger than the reach, so the smooth falloff ends inside the geometry
// instead of on its tessellated silhouette

layout (location = 0) in vec3 vertex_position;

layout (location = 1) in vec4 instance_pos_radius;   // xyz centre, w radius
layout (location = 2) in vec4 instance_color_power;  // rgb colour, a intensity

uniform mat4 proj_view;

out vec3 light_center;
out vec3 light_color;
out float light_radius;
out float light_power;

void main()
{
	vec3 center = instance_pos_radius.xyz;
	float radius = instance_pos_radius.w;

	light_center = center;
	light_color = instance_color_power.rgb;
	light_power = instance_color_power.a;
	light_radius = radius;

	gl_Position = proj_view * vec4(center + vertex_position * (radius * 1.10), 1.0);
}
