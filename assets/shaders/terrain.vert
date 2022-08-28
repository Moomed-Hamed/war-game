#version 420 core

struct VS_OUT
{
	vec3 frag_pos;
	vec3 tex_coord;
};

layout (location = 0) in vec3 position;
layout (location = 1) in vec3 normal;

uniform mat4 proj_view;

layout (binding = 2) uniform sampler2D heightmap;

out VS_OUT vs_out;
out float texture_scale;

void main()
{
	texture_scale = 256;
	vec2 tex_coord = position.xz / texture_scale; // not sure why xz has to be reversed
	
	vs_out.tex_coord.z = texture(heightmap, tex_coord).r;
	vs_out.frag_pos  = position + vec3(0, vs_out.tex_coord.z, 0);
	vs_out.tex_coord.xy = tex_coord.xy;

	gl_Position = proj_view * vec4(vs_out.frag_pos, 1.0);
}