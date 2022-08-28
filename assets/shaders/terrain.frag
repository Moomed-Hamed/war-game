#version 420 core

struct VS_OUT
{
	vec3 frag_pos;
	vec3 tex_coord;
};

in VS_OUT vs_out;
in float texture_scale;

layout (location = 0) out vec4 frag_position;
layout (location = 1) out vec4 frag_normal;
layout (location = 2) out vec4 frag_albedo;

layout (binding = 3) uniform sampler2D grass_normals;
layout (binding = 4) uniform sampler2D grass_albedos;
layout (binding = 5) uniform sampler2D grass_mat;

layout (binding = 6) uniform sampler2D dirt_normals;
layout (binding = 7) uniform sampler2D dirt_albedos;
layout (binding = 8) uniform sampler2D dirt_mat;

void main()
{
	vec2 tex = vs_out.tex_coord.xy * texture_scale / 1.5;

	vec3 normal = (texture(grass_normals, tex).rbg * 2) - 1;
	vec3 albedo =  texture(grass_albedos, tex).rgb;
	vec3 mat    =  texture(grass_mat    , tex).rgb;

	if(vs_out.tex_coord.z < .25) // stone texture; this is a hack
	{
		normal = (texture(dirt_normals, tex).rbg * 2) - 1;
		albedo =  texture(dirt_albedos , tex).rgb;
		mat    =  texture(dirt_mat     , tex).rgb;
	}

	frag_position = vec4(vs_out.frag_pos, mat.x); // metalness
	frag_normal   = vec4(normal         , mat.y); // roughness
	frag_albedo   = vec4(albedo         , mat.z); // ambient occlusion
}