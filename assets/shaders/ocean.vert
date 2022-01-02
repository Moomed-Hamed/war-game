#version 420 core

struct VS_OUT
{
	vec3 frag_pos;
	vec2 tex_coords;
	vec3 color;
};

layout (location = 0) in vec3 position;
layout (location = 1) in vec3 normal;
layout (location = 2) in vec3 world_position;
layout (location = 3) in vec3 color;

uniform mat4 proj_view;

layout (binding = 0) uniform sampler2D heightmap;

out VS_OUT vs_out;

void main()
{
	float scale = 20;
	vec3 p = position + world_position;
	vec3 d = texture(heightmap, p.xz / scale).rgb; d.y /= 1;
	p += d;

	vs_out.tex_coords = p.xz / scale;
	vs_out.frag_pos = p;
	vs_out.color = color;

	gl_Position = proj_view * vec4(vs_out.frag_pos, 1.0);
	//gl_Position = proj_view * vec4((vs_out.frag_pos / 8) + vec3(0,.5,0), 1.0);
}