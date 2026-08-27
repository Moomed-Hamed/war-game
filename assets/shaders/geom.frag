#version 420 core

struct VS_OUT
{
	vec3 normal;         // normal vector
	vec3 world_position; // position of this vertex in world space
	vec2 uv;
};

in VS_OUT vs_out;

layout (location = 0) out vec4 pixel_position;
layout (location = 1) out vec4 pixel_normal;
layout (location = 2) out vec4 pixel_albedo;

layout (binding = 0) uniform sampler2D texture_data;
//layout (binding = 1) uniform sampler2D material_data;

void main()
{
	vec3 plastic_mat = vec3(1, .5, 1);
	vec3 iron_mat    = vec3(1, .5, 1);
	vec3 gold_mat    = vec3(1, .5, 1);

	vec3 plastic_color = vec3(0.9, 0.1, 0.1); // red plastic
	vec3 iron_color = vec3(0.560, 0.570, 0.580);
	vec3 gold_color = vec3(1.000, 0.766, 0.336);

	vec3 material = texture(texture_data, vs_out.uv * 4.f).rgb;
	vec3 color    = texture(texture_data, vs_out.uv * 4.f).rgb;

	pixel_position = vec4(vs_out.world_position, material.r); // metalness
	pixel_normal   = vec4(vs_out.normal, material.g);         // roughness
	pixel_albedo   = vec4(color, material.b);                 // ambient occlusion
}
