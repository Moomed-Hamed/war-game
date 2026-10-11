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

// material : comes in flat from the instance (geom.vert / anim.vert)
flat in vec4 mat_a; // albedo rgb + metallic
flat in vec4 mat_b; // roughness, ao, texture-albedo flag

void main()
{
	// albedo is display space, the lighting pass decodes the g-buffer. the flag picks
	// between the texture tinted by the material and the material colour alone
	vec3 tex  = texture(texture_data, vs_out.uv * 4.f).rgb;
	vec3 base = mix(mat_a.rgb, tex * mat_a.rgb, mat_b.b);

	pixel_position = vec4(vs_out.world_position, mat_a.a); // metalness
	pixel_normal   = vec4(vs_out.normal, mat_b.r);         // roughness
	pixel_albedo   = vec4(base, mat_b.g);                  // ambient occlusion
}
