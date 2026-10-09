#version 420 core

// deferred lighting pass : one fullscreen quad shades the frame from the g-buffer
// (world pos + metallic, normal + roughness, albedo + AO). one directional sun and
// an analytic sky stand in for the whole environment. window.h binds the samplers
// and uploads view_pos (it arrives as vs_out.view_position).

struct VS_OUT {
    vec2 uv;
    vec3 view_position;
};

in VS_OUT vs_out;

layout (binding = 0) uniform sampler2D positions; // rgb world pos, a metallic
layout (binding = 1) uniform sampler2D normals;   // rgb world normal, a roughness
layout (binding = 2) uniform sampler2D albedo;    // rgb base color, a = ao

layout (location = 0) out vec4 pixel_color;

const float PI = 3.14159265359;

// directional sun : direction points from the sun into the scene
const vec3  sun_direction = vec3(-0.2, -1.0, -0.3);
const vec3  sun_color     = vec3(1.0, 0.85, 0.7);
const float sun_power     = 4.0; // HDR intensity, the tonemapper rolls it off

// analytic sky : ambient source, what reflective surfaces mirror
const vec3 sky_zenith  = vec3(0.16, 0.32, 0.62);
const vec3 sky_horizon = vec3(0.62, 0.70, 0.85);
const vec3 sky_ground  = vec3(0.11, 0.10, 0.09);
const vec3 sky_irr     = vec3(0.55, 0.62, 0.80); // hemisphere irradiance, surfaces facing up
const vec3 ground_irr  = vec3(0.10, 0.09, 0.08); // dim bounce from below

const float exposure = 1.0;

// flat backdrop, linear (about rgb 23, 25, 31 once tonemapped and encoded)
const vec3 background = vec3(0.012, 0.014, 0.019);

// sky radiance along a world direction, sun disk included
vec3 sky(vec3 dir, vec3 L)
{
    vec3 c = dir.y >= 0.0
        ? mix(sky_horizon, sky_zenith, pow(dir.y, 0.65))
        : mix(sky_horizon, sky_ground, pow(-dir.y, 0.65));
    float sun_d = max(dot(dir, L), 0.0);
    return c + sun_color * (pow(sun_d, 256.0) * 6.0 + pow(sun_d, 8.0) * 0.10);
}

// fresnel-weighted environment response (Karis analytic split-sum fit) :
// returns (scale, bias), ambient_spec = F0 * ab.x + ab.y
vec2 env_brdf(float roughness, float NdotV)
{
    const vec4 c0 = vec4(-1.0, -0.0275, -0.572, 0.022);
    const vec4 c1 = vec4( 1.0,  0.0425,  1.04, -0.04);
    vec4 r = roughness * c0 + c1;
    float a004 = min(r.x * r.x, exp2(-9.28 * NdotV)) * r.x + r.y;
    return vec2(-1.04, 1.04) * a004 + r.zw;
}

float distribution_ggx(float NdotH, float roughness)
{
    float a2 = roughness * roughness;
    a2 *= a2;
    float d = NdotH * NdotH * (a2 - 1.0) + 1.0;
    return a2 / max(PI * d * d, 1e-6);
}

float geometry_smith(float NdotV, float NdotL, float roughness)
{
    float k = pow(roughness + 1.0, 2.0) / 8.0; // direct-lighting form
    float gv = NdotV / (NdotV * (1.0 - k) + k);
    float gl = NdotL / (NdotL * (1.0 - k) + k);
    return gv * gl;
}

vec3 fresnel_schlick(float cosTheta, vec3 F0)
{
    float c = clamp(cosTheta, 0.0, 1.0); // pow() is undefined for a negative base
    return F0 + (1.0 - F0) * pow(1.0 - c, 5.0);
}

vec3 aces(vec3 x) // Narkowicz fit
{
    return clamp((x * (2.51 * x + 0.03)) / (x * (2.43 * x + 0.59) + 0.14), 0.0, 1.0);
}

// interleaved gradient noise, +/- half an 8-bit step after encode : kills banding
float screen_dither()
{
    return (fract(52.9829189 * fract(dot(gl_FragCoord.xy, vec2(0.06711056, 0.00583715)))) - 0.5) / 255.0;
}

void main()
{
    vec4 pos_tex  = texture(positions, vs_out.uv);
    vec4 norm_tex = texture(normals,   vs_out.uv);
    vec4 alb_tex  = texture(albedo,    vs_out.uv);

    // per-pixel normal change, taken before the background branch (derivatives are
    // only defined in uniform flow). it becomes roughness below : edges and grazing
    // silhouettes go soft instead of sparkling
    vec3 dn = fwidth(norm_tex.rgb);

    // nothing rasterized here (cleared buffer) : flat dark backdrop
    if (dot(norm_tex.rgb, norm_tex.rgb) < 1e-8)
    {
        pixel_color = vec4(pow(aces(exposure * background), vec3(1.0 / 2.2)), 1.0);
        return;
    }

    // material : albedo is stored as sRGB bytes, decode before any lighting
    vec3  base      = pow(alb_tex.rgb, vec3(2.2));
    float metallic  = clamp(pos_tex.a,  0.0, 1.0);
    float roughness = clamp(norm_tex.a, 0.045, 1.0);
    float ao        = clamp(alb_tex.a,  0.0, 1.0);

    vec3 N = normalize(norm_tex.rgb);
    vec3 P = pos_tex.rgb;
    vec3 V = normalize(vs_out.view_position - P);
    vec3 L = normalize(-sun_direction);
    vec3 H = normalize(V + L);

    float NdotV = max(dot(N, V), 1e-4);
    float NdotL = max(dot(N, L), 0.0);

    // specular anti-aliasing : fold the normal variance into the direct lobe's roughness
    float spec_rough = min(sqrt(roughness * roughness + dot(dn, dn)), 1.0);

    // --- direct sun : cook-torrance -------------------------------------------
    float NDF = distribution_ggx(max(dot(N, H), 0.0), spec_rough);
    float G   = geometry_smith(NdotV, NdotL, spec_rough);
    vec3  F0  = mix(vec3(0.04), base, metallic);
    vec3  F   = fresnel_schlick(max(dot(H, V), 0.0), F0);

    vec3 specular = NDF * G * F / (4.0 * NdotV * NdotL + 1e-6);
    vec3 kD = (1.0 - F) * (1.0 - metallic); // metals keep no diffuse

    vec3 direct = (kD * base / PI + specular) * sun_color * sun_power * NdotL;

    // --- ambient : the sky, split-sum ------------------------------------------
    // ao is an indirect-bounce estimate, so it gates the ambient and leaves the sun alone
    vec3 diffuse_amb = mix(ground_irr, sky_irr, N.y * 0.5 + 0.5) * base * (1.0 - metallic) * ao / PI;

    vec3  sky_rad = sky(reflect(-V, N), L);
    vec2  ab      = env_brdf(spec_rough, NdotV);
    vec3 specular_amb = (F0 * ab.x + ab.y) * sky_rad * ao;

    // --- output : exposure -> ACES -> display encode, 1-lsb dither -------------
    vec3 color = aces(exposure * (direct + diffuse_amb + specular_amb));
    color = pow(color, vec3(1.0 / 2.2)); // pair of the albedo decode above

    //pixel_color = vec4(color + screen_dither(), 1.0);
    pixel_color = vec4(color, 1.0);
}
