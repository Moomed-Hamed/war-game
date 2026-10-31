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

// water : persistent foam field + short-chop normal cascade
uniform vec3  foam_color;
uniform float foam_roughness;
uniform sampler2D water_normal_map;
uniform float water_detail;
uniform float water_detail_tiles;

// material : comes in flat from the instance (geom.vert / anim.vert)
flat in vec4 mat_a; // albedo rgb + metallic
flat in vec4 mat_b; // roughness, ao, texture-albedo flag, material mode

void main()
{
	vec3 base = mat_a.rgb;
	vec3 normal = normalize(vs_out.normal);
	float roughness = mat_b.r;

	// mode 1 : water. the texture is a per-pixel foam field, not a tile mask
	if (mat_b.a > .5)
	{
		// short chop : two scroll-free scales of the simulated normal cascade
		vec2 uv = vs_out.uv;
		vec2 d1 = texture(water_normal_map, uv * water_detail_tiles).rg * 2.0 - 1.0;
		vec2 d2 = texture(water_normal_map, uv.yx * (water_detail_tiles * 1.63)
		                  + vec2(.37, .11)).rg * 2.0 - 1.0;

		normal = normalize(normal + vec3(d1.x + d2.x, 0.0, d1.y + d2.y) * water_detail);

		float foam = texture(texture_data, uv).r;
		foam = smoothstep(.12, .88, foam);

		base = mix(mat_a.rgb, foam_color, foam);
		roughness = mix(mat_b.r, foam_roughness, foam);
	}
	else
	{
		// albedo is display space, the lighting pass decodes the g-buffer. the flag picks
		// between the texture tinted by the material and the material colour alone
		vec3 tex = texture(texture_data, vs_out.uv * 4.f).rgb;
		base = mix(mat_a.rgb, tex * mat_a.rgb, mat_b.b);
	}

	pixel_position = vec4(vs_out.world_position, mat_a.a); // metalness
	pixel_normal   = vec4(normal, roughness);
	pixel_albedo   = vec4(base, mat_b.g);                  // ambient occlusion
}
