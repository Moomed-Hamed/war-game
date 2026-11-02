#version 420 core

// forward water : the opaque frame is copied to scene_color, then the surface
// refracts it, absorbs it with depth, reflects the sky and adds foam. drawn after
// the deferred lights, so the water is lit by the same sun and environment

struct VS_OUT
{
	vec3 normal;
	vec3 world_position;
	vec2 uv;
};

in VS_OUT vs_out;
flat in vec4 mat_a; // water colour rgb + mean surface height
flat in vec4 mat_b; // roughness, ao, open-sea flag, 1

layout (binding = 0) uniform sampler2D foam_map;
layout (binding = 1) uniform sampler2D scene_color;
layout (binding = 2) uniform sampler2D scene_depth;
layout (binding = 3) uniform sampler2D water_normal_map;
layout (binding = 4) uniform sampler2D foam_pattern;
layout (binding = 5) uniform sampler2D scene_normal;

layout (location = 0) out vec4 pixel_color;

uniform vec3  view_pos;
uniform vec2  screen_size;
uniform float near_plane;
uniform float far_plane;

uniform vec3  sun_direction;
uniform vec3  sun_color;
uniform float sun_power;

uniform vec3  moon_direction;
uniform vec3  moon_color;
uniform float moon_power;
uniform float moon_disk;

uniform vec3 zenith;
uniform vec3 haze;
uniform vec3 glow;
uniform vec3 ground;
uniform vec3 ambient_up;
uniform vec3 ambient_side;
uniform vec3 ambient_down;

uniform float fog_density;

uniform vec3  foam_color;
uniform float foam_roughness;
uniform float water_detail;
uniform float water_sss;
uniform float water_time;

const float PI = 3.14159265359;

// ---- environment : the same sky the opaque pass uses ----------------------
vec3 environment(vec3 dir)
{
	dir = normalize(dir);
	float h = dir.y;
	vec3 L_sun = -sun_direction;

	vec2 flat_dir = normalize(vec2(dir.x, dir.z) + vec2(1e-4, 0.0));
	vec2 flat_sun = normalize(vec2(L_sun.x, L_sun.z) + vec2(1e-4, 0.0));
	float azimuth = max(dot(flat_dir, flat_sun), 0.0);

	vec3 horizon = mix(haze, glow, pow(azimuth, 3.0) * 0.85);
	vec3 col = (h > 0.0) ? mix(horizon, zenith, pow(h, 0.55))
	                     : mix(horizon, ground, pow(-h, 0.45));

	float d = max(dot(dir, L_sun), 0.0);
	float sun_patch = smoothstep(0.99930, 0.99975, d);
	vec3 sky = col + sun_color * (sun_patch * 26.0 + pow(d, 24.0) * 0.35
	                            + pow(d, 4.0) * 0.05);

	vec3 L_moon = -moon_direction;
	float dm = max(dot(dir, L_moon), 0.0);
	float moon_patch = smoothstep(0.99955, 0.99982, dm);
	sky += moon_color * (moon_patch * 22.0 + pow(dm, 96.0) * 0.30) * moon_disk;
	return sky;
}

vec3 ambient_irradiance(vec3 N)
{
	return (N.y > 0.0) ? mix(ambient_side, ambient_up,   smoothstep(0.0, 1.0,  N.y))
	                   : mix(ambient_side, ambient_down, smoothstep(0.0, 1.0, -N.y));
}

// ---- brdf -----------------------------------------------------------------
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

// ---- screen-space helpers -------------------------------------------------
float linear_depth(float d)
{
	float z = d * 2.0 - 1.0;
	return (2.0 * near_plane * far_plane) / (far_plane + near_plane - z * (far_plane - near_plane));
}

void main()
{
	vec2 screen_uv = gl_FragCoord.xy / screen_size;

	vec3 V = normalize(view_pos - vs_out.world_position);
	float water_level = mat_a.a;
	float dist = length(vs_out.world_position - view_pos);

	// short chop : three world-space scales of the 16 m cascade. the fine layers
	// fade with distance so the horizon does not shimmer
	vec2 detail_uv = vs_out.world_position.xz * (1.0 / 16.0);
	vec2 detail_uv2 = vs_out.world_position.xz * (1.0 / 5.7)
	                + vec2(water_time * .004, -water_time * .003);
	vec2 d1 = texture(water_normal_map, detail_uv).rg * 2.0 - 1.0;
	vec2 d2 = texture(water_normal_map, detail_uv.yx * 1.63 + vec2(.37, .11)).rg * 2.0 - 1.0;
	vec2 d3 = texture(water_normal_map, detail_uv2.yx * 1.37 + vec2(.19, .53)).rg * 2.0 - 1.0;

	float detail_fade = 1.0 - smoothstep(80.0, 250.0, dist);
	vec3 detail = vec3(d1.x + d2.x + d3.x * .55, 0.0,
	                   d1.y + d2.y + d3.y * .55) * water_detail * detail_fade;
	vec3 N = normalize(vs_out.normal + detail);

	// glitter : a tiny high-frequency normal just for the sun/moon highlight. the
	// lace mask is already in memory, so it makes a cheap sparkle field
	float glitter_fade = 1.0 - smoothstep(18.0, 110.0, dist);
	vec2 g_uv = vs_out.world_position.xz / 2.7 + vec2(water_time * .020, water_time * .011);
	float gx = texture(foam_pattern, g_uv).r - 0.5;
	float gz = texture(foam_pattern, g_uv.yx + vec2(.31, .17)).r - 0.5;
	vec3 Ng = normalize(N + vec3(gx, 0.0, gz) * (0.20 * water_detail * glitter_fade));

	float water_z = linear_depth(gl_FragCoord.z);
	float scene_z = linear_depth(texture(scene_depth, screen_uv).r);
	float thickness = clamp(scene_z - water_z, 0.0, 40.0);

	// refraction : bend the screen lookup by the surface normal, then fall back to
	// the unshifted pixel if the shifted one belongs to a nearer surface
	vec2 refr_uv = screen_uv + N.xz * (0.025 / max(water_z, 0.5));
	refr_uv = clamp(refr_uv, vec2(0.001), vec2(0.999));
	float refr_z = linear_depth(texture(scene_depth, refr_uv).r);
	if (refr_z < water_z) refr_uv = screen_uv;

	vec3 background = texture(scene_color, refr_uv).rgb;

	// absorption : red dies first, then green, so the depth reads as water rather
	// than a tinted window. the scattered part is lit by the same rig.
	vec3 deep = pow(mat_a.rgb, vec3(2.2)) * 6.0; // volume colour, not a surface albedo
	vec3 transmittance = exp(-thickness * vec3(0.34, 0.10, 0.065));
	vec3 scattered = deep * (ambient_irradiance(vec3(0.0, 1.0, 0.0)) * 1.4
	                       + sun_color * sun_power * 0.25);
	vec3 beneath = background * transmittance + scattered * (1.0 - transmittance);

	// fresnel : more sky at grazing angles
	float NdotV = max(dot(N, V), 1e-3);
	float fresnel = 0.02 + 0.98 * pow(1.0 - NdotV, 5.0);
	vec3 reflection = environment(reflect(-V, N));
	vec3 color = mix(beneath, reflection, fresnel);

	// sun highlight : a tight GGX lobe, with the roughness the preset asks for
	vec3 L = -sun_direction;
	float NdotL = max(dot(N, L), 0.0);
	if (NdotL > 0.0)
	{
		vec3 H = normalize(V + L);
		float NDF = distribution_ggx(max(dot(Ng, H), 0.0), max(mat_b.r, 0.035));
		float G = geometry_smith(NdotV, NdotL, max(mat_b.r, 0.035));
		vec3 F = fresnel_schlick(max(dot(H, V), 0.0), vec3(0.02));
		vec3 specular = NDF * G * F / (4.0 * NdotV * NdotL + 1e-6);
		color += specular * sun_color * sun_power * NdotL;
	}

	// moonlight : the same lobe, cool and much weaker
	vec3 Lm = -moon_direction;
	float NdotLm = max(dot(N, Lm), 0.0);
	if (NdotLm > 0.0 && moon_power > 0.0)
	{
		vec3 Hm = normalize(V + Lm);
		float NDFm = distribution_ggx(max(dot(Ng, Hm), 0.0), max(mat_b.r, 0.035));
		float Gm = geometry_smith(NdotV, NdotLm, max(mat_b.r, 0.035));
		vec3 Fm = fresnel_schlick(max(dot(Hm, V), 0.0), vec3(0.02));
		vec3 specularm = NDFm * Gm * Fm / (4.0 * NdotV * NdotLm + 1e-6);
		color += specularm * moon_color * moon_power * NdotLm;
	}

	// subsurface scattering : light entering a crest scatters forward through the
	// thin water. the peak mask follows the crest top and its steep sides, and the
	// phase term peaks when the sun is behind the wave from this view
	float crest_height = vs_out.world_position.y - water_level;
	float crest = smoothstep(0.02, 0.40, crest_height);
	float steep = smoothstep(0.02, 0.45, 1.0 - N.y);
	float peak = crest * (0.35 + 0.65 * steep);
	// wrapped phase : strongest when the sun is behind the crest, but the volume
	// still carries a little light from the side and above
	float backlight = pow(clamp(dot(V, sun_direction) * 0.5 + 0.5, 0.0, 1.0), 2.0);
	float sss = peak * backlight * (1.0 - fresnel) * water_sss;
	vec3 sss_tint = vec3(0.012, 0.18, 0.17);
	color += sss_tint * (sun_color * sun_power * 0.8 + ambient_up * 1.5) * sss;

	// foam : simulated crests, a breaking band where the water is shallow, and a
	// public-domain lace mask so the whitewater has structure instead of a flat
	// blob. two scales drift at different speeds to break up the tiling
	vec2 foam_uv = vs_out.world_position.xz / 7.0;
	float lace_a = texture(foam_pattern, foam_uv + vec2(water_time * 0.008, water_time * 0.003)).r;
	float lace_b = texture(foam_pattern, foam_uv * 1.73 + vec2(-water_time * 0.012, water_time * 0.005)).r;
	float lace = smoothstep(0.15, 0.75, lace_a * 0.65 + lace_b * 0.35);

	float foam = 0.0;
	if (mat_b.a > 0.5)
		foam = smoothstep(0.15, 0.85, texture(foam_map, vs_out.uv).r);

	// breaking : the band widens with the crest, a slow run-up pulse washes it up
	// and back, and a steep bottom (rock, wall, reef) pulls more whitewater
	vec3 bottom_n = texture(scene_normal, screen_uv).rgb;
	float bottom_slope = 1.0 - clamp(normalize(bottom_n + vec3(0.0, 1e-4, 0.0)).y, 0.0, 1.0);
	float shore_width = 0.35 + 1.8 * crest;
	float shore = 1.0 - smoothstep(0.0, shore_width, thickness);
	float runup = 0.72 + 0.28 * sin(water_time * 1.15 + vs_out.world_position.x * 0.17
	                              + vs_out.world_position.z * 0.11);
	shore *= runup;
	float breakers = shore * (0.25 + 0.75 * crest) * (0.55 + 0.90 * bottom_slope);
	foam = max(foam, breakers);

	foam *= mix(0.35, 1.15, lace);
	foam = clamp(foam, 0.0, 1.0);

	vec3 foam_linear = pow(foam_color, vec3(2.2));
	vec3 foam_lit = foam_linear * (ambient_irradiance(vec3(0.0, 1.0, 0.0)) * 1.5
	                             + sun_color * sun_power * 0.28);
	foam_lit *= 1.0 + (1.0 - foam_roughness) * 0.6; // gloss : brighter foam
	color = mix(color, foam_lit, clamp(foam, 0.0, 1.0));

	// a little glisten on the foam itself : the lace mask doubles as a bump
	float foam_spec = pow(max(dot(reflect(-V, Ng), -sun_direction), 0.0), 24.0) * foam;
	color += foam_spec * sun_color * sun_power * 0.22;

	// fog : the same distance fade as the opaque pass
	vec3 view_dir = normalize(vs_out.world_position - view_pos);
	color = mix(color, environment(view_dir), 1.0 - exp(-dist * fog_density));

	pixel_color = vec4(color, 1.0);
}
