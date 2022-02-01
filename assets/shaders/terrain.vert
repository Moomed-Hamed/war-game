#version 420 core

struct VS_OUT
{
	vec3 frag_pos;
	vec2 tex_coord;
};

layout (location = 0) in vec3 position;
layout (location = 1) in vec3 normal;

uniform mat4 proj_view;

layout (binding = 2) uniform sampler2D heightmap;

out VS_OUT vs_out;

void main()
{
	vec2 tex_coord = position.zx / 256; // not sure why xz has to be reversed
	float h = texture(heightmap, tex_coord).r;

	vs_out.frag_pos  = position + vec3(0, h, 0);
	vs_out.tex_coord = tex_coord;

	gl_Position = proj_view * vec4(vs_out.frag_pos, 1.0);
}