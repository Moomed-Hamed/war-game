#version 420 core

struct VS_OUT
{
	vec2 tex_coords;
	vec3 view_pos;
};

in VS_OUT vs_out;

// these come from the g_buffer
layout (binding = 0) uniform sampler2D positions;
layout (binding = 1) uniform sampler2D normals;
layout (binding = 2) uniform sampler2D albedo;

layout (location = 0) out vec4 frag_color;

// lights
uniform vec3 light_positions[4];
uniform vec3 light_colors[4];

const float PI = 3.14159265359;
const float MINF = 0.00001;

float D(float alpha, float n_dot_h)
{
	float a2  = alpha * alpha;
	float nh2 = n_dot_h * n_dot_h;

	return a2 / max(PI * pow(nh2 * (a2 - 1) + 1, 2), MINF);
}

float G1(float alpha, vec3 N, vec3 X) // Schlick-Beckmann geometry shadowing function
{
	float ndx = max(dot(N,X), MINF);
	float k = alpha * .5;

	return ndx / max(ndx * (1 - k) + k, MINF);
}

float G(float alpha, vec3 N, vec3 V, vec3 L) // Smith model
{
	return G1(alpha, N, V) * G1(alpha, N, L);
}

vec3 F(vec3 F0, float h_dot_v)
{
	return vec3(F0 + (1 - F0) * pow(1 - h_dot_v, 5));
}

void main()
{
	float metallic  = texture(positions, vs_out.tex_coords).w;
	float roughness = texture(normals  , vs_out.tex_coords).w;
	float ao        = texture(albedo   , vs_out.tex_coords).w;

	vec3 position = texture(positions, vs_out.tex_coords).rgb;
	vec3 normal   = texture(normals  , vs_out.tex_coords).rgb;
	vec3 albedo   = texture(albedo   , vs_out.tex_coords).rgb;

	vec3 N = normalize(normal);
	vec3 V = normalize(vs_out.view_pos - position); // world_pos -> camera

	// if dia-electric (eg. plastic) use 0.04. if metal, use albedo (metallic workflow)    
	vec3 F0 = mix(vec3(0.02), albedo, metallic); // base reflectivity

	vec3 Lo = vec3(0); // Lo = outgoing radiance (final color - ambient)

	vec3 lambert = vec3(ao / PI);
	float alpha  = roughness * roughness;

	// directional light
	{
		vec3 light_dir = -1 * normalize(vec3(1, -.1, 1)); // idk why this is flipped
		vec3 light_color = vec3(.31, .41, .53) * .5;
		
		vec3 L = normalize(light_dir);
		vec3 H = normalize(V + L);
		
		float n_dot_v = max(dot(N, V), MINF);
		float n_dot_l = max(dot(N, L), MINF);
		float h_dot_v = max(dot(H, V), 0.0);
		float n_dot_h = max(dot(N, H), 0.0);
		
		vec3 Ks = F(F0, h_dot_v);
		vec3 Kd = (vec3(1) - Ks) * (1 - metallic);
		
		// Cook-Torrance BRDF
		vec3 n = D(alpha, n_dot_h) * G(alpha, N, V, L) * F(F0, h_dot_v);
		vec3 d = 4.0 * vec3(max(n_dot_v * n_dot_l, MINF));
		vec3 cook_torrance = n / d;
		vec3 BRDF = Kd * lambert + cook_torrance;
		
		Lo += 2 * BRDF * light_color * n_dot_l;
	}

	for(int i = 0; i < 4; ++i) // point lights
	{
		vec3 L = normalize(light_positions[i] - position); // world_pos -> light source
		vec3 H = normalize(V + L);
		float distance = length(light_positions[i] - position);
		float attenuation = 5 * ( (1 / distance) + 1.0 / (distance * distance) );
		vec3 radiance = light_colors[i] * attenuation;
	
		float n_dot_v = max(dot(N, V), MINF);
		float n_dot_l = max(dot(N, L), MINF);
		float h_dot_v = max(dot(H, V), 0.0);
		float n_dot_h = max(dot(N, H), 0.0);

		vec3 Ks = F(F0, h_dot_v);
		vec3 Kd = (vec3(1) - Ks) * (1 - metallic);

		// Cook-Torrance BRDF
		vec3 n = D(alpha, n_dot_h) * G(alpha, N, V, L) * F(F0, h_dot_v);
		vec3 d = 4.0 * vec3(max(n_dot_v * n_dot_l, MINF));
		vec3 cook_torrance = n / d;
		vec3 BRDF = Kd * lambert + cook_torrance;

		Lo += BRDF * radiance * n_dot_l; // + emissivity;
	}

	vec3 color = Lo + (albedo * ao); // albedo * ao = ambient (replace with env. lighting)
	color = color / (color + vec3(1.0)); // HDR tonemapping
	color = pow(color, vec3(.8)); // gamma correct 1/2.2

	frag_color = vec4(color, 1.0);
}