#version 420 core

struct VS_OUT
{
	vec3 frag_pos;
	vec2 tex_coord;
	vec3 color;
};

in VS_OUT vs_out;

layout (location = 0) out vec4 frag_position;
layout (location = 1) out vec4 frag_normal;
layout (location = 2) out vec4 frag_albedo;

layout (binding = 3) uniform sampler2D normals;

void main()
{
	vec3 normal = (texture(normals, vs_out.tex_coord).rbg * 2) - 1;

	frag_position = vec4(vs_out.frag_pos,.3); // metalness
	frag_normal   = vec4(normal         ,.9); // roughness
	frag_albedo   = vec4(vs_out.color   ,.1); // ambient occlusion
}