#version 420 core

struct VS_OUT {
    vec2 uv;
    vec3 view_position;
};

in VS_OUT vs_out;

// Sampled from the G-buffer
layout (binding = 0) uniform sampler2D positions; // RGB = world pos   , A = metallic
layout (binding = 1) uniform sampler2D normals;   // RGB = world normal, A = roughness
layout (binding = 2) uniform sampler2D albedo;    // RGB = base color  , A = AO

layout (location = 0) out vec4 pixel_color;

const float PI = 3.14159265359;

// Hardcoded sun-light parameters (do not normalize here in a const)
const vec3 light_direction_raw = vec3(-0.2, -1.0, -0.3);
const vec3 light_color         = vec3(1.0, 0.85, 0.7);

// Hardcoded camera position for now (replace with uniform later)
const vec3 camera_position = vs_out.view_position;

// === PBR helper functions ===
vec3 fresnel_schlick(float cosTheta, vec3 F0)
{
    return F0 + (1.0 - F0) * pow(1.0 - cosTheta, 5.0);
}

float distribution_ggx(vec3 N, vec3 H, float roughness)
{
    float a      = roughness * roughness;
    float a2     = a * a;
    float NdotH  = max(dot(N, H), 0.0);
    float NdotH2 = NdotH * NdotH;

    float num   = a2;
    float denom = (NdotH2 * (a2 - 1.0) + 1.0);
    denom       = PI * denom * denom;

    return num / max(denom, 1e-6);
}

float geometry_schlick_ggx(float NdotV, float roughness)
{
    float r = roughness + 1.0;
    float k = (r * r) / 8.0;
    return NdotV / (NdotV * (1.0 - k) + k);
}

float geometry_smith(vec3 N, vec3 V, vec3 L, float roughness)
{
    float NdotV = max(dot(N, V), 0.0);
    float NdotL = max(dot(N, L), 0.0);
    float ggx1  = geometry_schlick_ggx(NdotV, roughness);
    float ggx2  = geometry_schlick_ggx(NdotL, roughness);
    return ggx1 * ggx2;
}

void main()
{
    // G-buffer fetch
    vec4 pos_tex    = texture(positions, vs_out.uv);
    vec4 norm_tex   = texture(normals,   vs_out.uv);
    vec4 albedo_tex = texture(albedo,    vs_out.uv);

    vec3 world_pos  = pos_tex.rgb;
    vec3 N_raw      = norm_tex.rgb;
    vec3 baseColor_srgb = albedo_tex.rgb;

    // Safety for normals
    if(length(N_raw) < 1e-4) {
        // fallback normal if texture is invalid
        N_raw = vec3(0.0, 1.0, 0.0);
    }
    vec3 N = normalize(N_raw);

    // Convert albedo sRGB -> linear (approx)
    vec3 baseColor = pow(baseColor_srgb, vec3(2.2));

    float metallic  = clamp(pos_tex.a   , 0.0 , .95);
    float roughness = clamp(norm_tex.a  , 0.05, .95); // avoid 0
    float ao        = clamp(albedo_tex.a, 0.0 , .95);

    // View + light
    vec3 V = normalize(camera_position - world_pos);

    // normalize light direction at runtime (not in a const initializer)
    vec3 L = normalize(-light_direction_raw); // light points FROM light -> so use -dir
    vec3 H = normalize(V + L);

    // Radiance (directional light: no attenuation here)
    vec3 radiance = light_color;

    // Cook–Torrance BRDF
    float NDF = distribution_ggx(N, H, roughness);
    float G   = geometry_smith(N, V, L, roughness);
    vec3  F0  = mix(vec3(0.04), baseColor, metallic);
    vec3  F   = fresnel_schlick(max(dot(H, V), 0.0), F0);

    vec3 numerator    = NDF * G * F;
    float denominator = 4.0 * max(dot(N, V), 0.0) * max(dot(N, L), 0.0) + 1e-6;
    vec3 specular     = numerator / denominator;

    vec3 kS = F;
    vec3 kD = (vec3(1.0) - kS) * (1.0 - metallic);

    float NdotL = max(dot(N, L), 0.0);

    vec3 Lo = (kD * baseColor / PI + specular) * radiance * NdotL;

    // Ambient lighting
    vec3 ambient = vec3(0.03) * baseColor * ao;

    // Combine
    vec3 color = ambient + Lo;

    // Tone map + gamma correct
    color = color / (color + vec3(1.0)); // simple reinhard
    color = pow(color, vec3(1.0/2.2));

    pixel_color = vec4(color, 1.0);
}
