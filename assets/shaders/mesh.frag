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

uniform vec3 material = vec3(.3, .8, .05);

void main()
{
	frag_position = vec4(vs_out.frag_pos, material.x); // metalness
	frag_normal   = vec4(vs_out.normal  , material.y); // roughness
	frag_albedo   = vec4(vs_out.color   , material.z); // ambient occlusion
}
