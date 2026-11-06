#version 420 core

// local light volume : the sphere only covers the pixels the light can reach, so
// the deferred frame is shaded additively inside the volume and skipped outside

layout (binding = 0) uniform sampler2D positions; // rgb world pos, a metallic
layout (binding = 1) uniform sampler2D normals;   // rgb world normal, a roughness
layout (binding = 2) uniform sampler2D albedo;    // rgb base colour, a ao

uniform vec3 view_pos;
uniform ivec2 screen_size;

in vec3 light_center;
in vec3 light_color;
in float light_radius;
in float light_power;

layout (location = 0) out vec4 pixel_color;

const float PI = 3.14159265359;

float distribution_ggx(float NdotH, float roughness)
{
	float a2 = roughness * roughness;
	a2 *= a2;
	float d = NdotH * NdotH * (a2 - 1.0) + 1.0;
	return a2 / max(PI * d * d, 1e-6);
}

float geometry_smith(float NdotV, float NdotL, float roughness)
{
	float k = pow(roughness + 1.0, 2.0) / 8.0;
	float gv = NdotV / (NdotV * (1.0 - k) + k);
	float gl = NdotL / (NdotL * (1.0 - k) + k);
	return gv * gl;
}

vec3 fresnel_schlick(float cosTheta, vec3 F0)
{
	float c = clamp(cosTheta, 0.0, 1.0);
	return F0 + (1.0 - F0) * pow(1.0 - c, 5.0);
}

void main()
{
	ivec2 pixel = ivec2(gl_FragCoord.xy);
	if (pixel.x < 0 || pixel.y < 0 || pixel.x >= screen_size.x || pixel.y >= screen_size.y)
		discard;

	vec4 pos_tex  = texelFetch(positions, pixel, 0);
	vec4 norm_tex = texelFetch(normals,   pixel, 0);

	if (dot(norm_tex.rgb, norm_tex.rgb) < 1e-8) // sky : no surface to light
		discard;

	vec3  P = pos_tex.rgb;
	float d = distance(P, light_center);
	if (d >= light_radius)
		discard;

	vec4 alb_tex = texelFetch(albedo, pixel, 0);

	// soft window : the light fades over the outer part of its reach and reaches
	// zero with a flat slope, so the volume edge does not read as a ring
	float x = clamp(d / light_radius, 0.0, 1.0);
	float window = 1.0 - smoothstep(0.55, 1.0, x);
	float attenuation = window * window / (1.0 + d * d * 0.35);

	vec3 N = normalize(norm_tex.rgb);
	vec3 V = normalize(view_pos - P);
	vec3 L = normalize(light_center - P);

	vec3 base      = pow(alb_tex.rgb, vec3(2.2));
	float metallic = clamp(pos_tex.a, 0.0, 1.0);
	float roughness = clamp(norm_tex.a, 0.045, 1.0);

	float NdotV = max(dot(N, V), 1e-4);
	float NdotL = max(dot(N, L), 0.0);
	if (NdotL <= 0.0)
		discard;

	vec3  H  = normalize(V + L);
	float NDF = distribution_ggx(max(dot(N, H), 0.0), roughness);
	float G   = geometry_smith(NdotV, NdotL, roughness);
	vec3  F0  = mix(vec3(0.04), base, metallic);
	vec3  F   = fresnel_schlick(max(dot(H, V), 0.0), F0);

	vec3 specular = NDF * G * F / (4.0 * NdotV * NdotL + 1e-6);
	vec3 kD = (1.0 - F) * (1.0 - metallic);

	vec3 radiance = light_color * light_power * attenuation;
	vec3 direct = (kD * base / PI + specular) * radiance * NdotL;

	pixel_color = vec4(direct, 1.0);
}
