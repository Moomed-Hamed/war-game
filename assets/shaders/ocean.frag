#version 420 core

struct VS_OUT
{
	vec3 frag_pos;
	vec2 tex_coords;
	vec3 color;
};

in VS_OUT vs_out;

layout (location = 0) out vec4 frag_position;
layout (location = 1) out vec4 frag_normal;
layout (location = 2) out vec4 frag_albedo;

layout (binding = 0) uniform sampler2D heightmap; // w component is foam
layout (binding = 1) uniform sampler2D normalmap;

void main()
{
	vec3 normal = texture(normalmap, vs_out.tex_coords).rgb;
	float foam  = texture(heightmap, vs_out.tex_coords).w;

	vec3 color = vs_out.color + vec3(foam);

	frag_position = vec4(vs_out.frag_pos,.0); // metalness
	frag_normal   = vec4(normal         ,.65 + foam); // roughness
	frag_albedo   = vec4(color          ,.05); // ambient occlusion
}