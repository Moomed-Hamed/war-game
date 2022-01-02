#version 330 core

struct VS_OUT
{
	vec3 normal;
	vec3 frag_pos;
	vec3 color;
};

in VS_OUT vs_out;

layout (location = 0) out vec4 frag_position;
layout (location = 1) out vec4 frag_normal;
layout (location = 2) out vec4 frag_albedo;

void main()
{
	frag_position = vec4(vs_out.frag_pos,.9); // metalness
	frag_normal   = vec4(vs_out.normal  ,.9); // roughness
	frag_albedo   = vec4(vs_out.color   ,.1); // ambient occlusion
}
