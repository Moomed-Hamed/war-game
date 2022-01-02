#version 330 core

struct VS_OUT
{
	vec2 frag_pos;
	vec3 color;
};

in VS_OUT vs_out;

layout (location = 0) out vec4 frag_position;
layout (location = 1) out vec4 frag_normal;
layout (location = 2) out vec4 frag_albedo;

void main()
{
	frag_position = vec4(vs_out.frag_pos, 0, 0);
	frag_normal   = vec4(0, 0, 1, 0);
	frag_albedo   = vec4(vs_out.color   , 0);
}