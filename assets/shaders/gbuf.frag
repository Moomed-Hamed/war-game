#version 420 core

// deferred lighting pass : one fullscreen quad shades the frame from the g-buffer
// (world pos + metallic, normal + roughness, albedo + ao).
//
// the rig is a warm key sun and a cool rim from behind the camera. the environment
// is a procedural hazy sky that serves as the background, the ambient and the
// reflections at once, so what a mirror shows is what the camera sees. occlusion is
// screen space and applies to the ambient only - there are no shadow maps yet, so
// the key light does not cast. the lighting pass binds the samplers and uploads
// view_pos (it arrives as vs_out.view_position) and proj_view.

struct VS_OUT {
    vec2 uv;
    vec3 view_position;
};

in VS_OUT vs_out;

layout (binding = 0) uniform sampler2D positions; // rgb world pos, a metallic
layout (binding = 1) uniform sampler2D normals;   // rgb world normal, a roughness
layout (binding = 2) uniform sampler2D albedo;    // rgb base color, a = ao

layout (location = 0) out vec4 pixel_color;

uniform mat4 proj_view; // the occlusion taps land with it

const float PI = 3.14159265359;

// ---- the lights -----------------------------------------------------------

// key : warm, high, off to the right, so the material grid is lit from the front
const vec3  sun_direction = normalize(vec3(-0.36, -0.84, -0.30)); // travel direction
const vec3  sun_color     = vec3(1.00, 0.86, 0.68);
const float sun_power     = 4.0;

// rim : cool, low, out of the camera's side of the world. a single light leaves a
// silhouette flat against whatever is behind it, this is what separates it
const vec3  rim_direction = normalize(vec3(0.58, -0.22, 0.60));
const vec3  rim_color     = vec3(0.50, 0.64, 0.92);
const float rim_power     = 0.60;

const vec3 L_sun = -sun_direction;
const vec3 L_rim = -rim_direction;

const float exposure     = 1.0;
const float fog_density  = 0.0070; // the far end of the floor reads far
const float multiscatter = 0.35;   // how much lost rough-metal light comes back
const float ao_strength  = 1.1;    // screen space occlusion, 0 turns it off

// ---- the environment ------------------------------------------------------

// the sky, by direction : dark overhead, hazy at the horizon and warm toward the
// sun, dark underfoot. the background branch, the ambient and every reflection call
// this, so the world and what it reflects are the same place
vec3 environment(vec3 dir)
{
    dir = normalize(dir);
    float h = dir.y;

    // how much of the sun's compass direction this ray carries : the horizon warms
    // around it whether the sun is high or low
    vec2 flat_dir = normalize(vec2(dir.x, dir.z) + vec2(1e-4, 0.0));
    vec2 flat_sun = normalize(vec2(L_sun.x, L_sun.z) + vec2(1e-4, 0.0));
    float azimuth = max(dot(flat_dir, flat_sun), 0.0);

    const vec3 zenith = vec3(0.100, 0.155, 0.290);
    const vec3 haze   = vec3(0.300, 0.315, 0.345);
    const vec3 glow   = vec3(0.720, 0.420, 0.220);
    const vec3 ground = vec3(0.045, 0.042, 0.038);

    vec3 horizon = mix(haze, glow, pow(azimuth, 3.0) * 0.85);
    vec3 col = (h > 0.0) ? mix(horizon, zenith, pow(h, 0.55))
                         : mix(horizon, ground, pow(-h, 0.45));

    // the sun : an antialiased disk plus a bloom wide enough to smear across a rough
    // sphere instead of sitting on it as a dot
    float d = max(dot(dir, L_sun), 0.0);
    float disk = smoothstep(0.99930, 0.99975, d);
    return col + sun_color * (disk * 26.0 + pow(d, 24.0) * 0.35 + pow(d, 4.0) * 0.05);
}

// hemisphere irradiance : three zones is enough for the eye to read the sky's shape
vec3 ambient_irradiance(vec3 N)
{
    const vec3 up   = vec3(0.500, 0.575, 0.730);
    const vec3 side = vec3(0.255, 0.270, 0.310);
    const vec3 down = vec3(0.075, 0.070, 0.062);

    return (N.y > 0.0) ? mix(side, up,   smoothstep(0.0, 1.0,  N.y))
                       : mix(side, down, smoothstep(0.0, 1.0, -N.y));
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

vec3 aces(vec3 x) // Narkowicz fit
{
    return clamp((x * (2.51 * x + 0.03)) / (x * (2.43 * x + 0.59) + 0.14), 0.0, 1.0);
}

// one analytic light : cook-torrance, and nothing occludes it. the occlusion below
// is an ambient term, so a lit fold stays lit until shadows exist
vec3 direct_light(vec3 N, vec3 V, vec3 L, vec3 radiance,
                  vec3 base, vec3 F0, float metallic, float spec_rough, float NdotV)
{
    float NdotL = max(dot(N, L), 0.0);
    if (NdotL <= 0.0) return vec3(0.0);

    vec3  H  = normalize(V + L);
    float NDF = distribution_ggx(max(dot(N, H), 0.0), spec_rough);
    float G   = geometry_smith(NdotV, NdotL, spec_rough);
    vec3  F   = fresnel_schlick(max(dot(H, V), 0.0), F0);

    vec3 specular = NDF * G * F / (4.0 * NdotV * NdotL + 1e-6);
    vec3 kD = (1.0 - F) * (1.0 - metallic); // metals keep no diffuse

    return (kD * base / PI + specular) * radiance * NdotL;
}

// ---- occlusion ------------------------------------------------------------

// interleaved gradient noise : a stable pseudo-random per pixel, used to rotate the
// taps so the sampling pattern does not read as rings
float hash(vec2 p)
{
    return fract(52.9829189 * fract(dot(p, vec2(0.06711056, 0.00583715))));
}

// screen space occlusion, read back out of the g-buffer : how much of the hemisphere
// over this pixel is taken up by geometry close enough to matter. this is what stops
// a sphere on the floor reading as if it were floating
float ssao(vec3 P, vec3 N)
{
    const int   taps   = 12;
    const float radius = 0.40; // metres : reaches the floor under a resting object

    // a basis around the normal, turned per pixel
    float turn = hash(gl_FragCoord.xy) * 6.2831853;
    vec3  t = normalize(abs(N.z) < 0.9 ? cross(N, vec3(0.0, 0.0, 1.0))
                                       : cross(N, vec3(1.0, 0.0, 0.0)));
    vec3  x = t * cos(turn) + cross(N, t) * sin(turn);
    vec3  y = cross(N, x);

    float occ = 0.0;
    for (int i = 0; i < taps; i++)
    {
        // spiral : even angular coverage, growing radius
        float fi  = float(i);
        float ang = fi * 2.399963; // golden angle
        float rad = radius * sqrt((fi + 0.5) / float(taps));
        vec3  tap = P + (x * cos(ang) + y * sin(ang)) * rad;

        vec4 clip = proj_view * vec4(tap, 1.0);
        if (clip.w <= 0.0) continue;
        vec2 suv = clip.xy / clip.w * 0.5 + 0.5;
        if (suv.x < 0.0 || suv.x > 1.0 || suv.y < 0.0 || suv.y > 1.0) continue; // off screen

        vec4 s_norm = texture(normals, suv);
        if (dot(s_norm.rgb, s_norm.rgb) < 1e-8) continue; // sky : nothing to hit there

        // the same surface, one pixel over : on a convex edge the taps land back on
        // the object and it occludes itself, ringing every sphere with its own band
        if (dot(s_norm.rgb, N) > 0.9) continue;

        vec3 sp = texture(positions, suv).rgb;

        // and it has to sit properly between the eye and the tap, not a hair in front
        if (dot(sp - tap, normalize(tap - vs_out.view_position)) > -0.02) continue;

        occ += 1.0 - clamp(length(sp - P) / radius, 0.0, 1.0);
    }

    return clamp(1.0 - occ / float(taps) * ao_strength, 0.0, 1.0);
}

// interleaved gradient noise, +/- half an 8-bit step after encode : kills banding
float screen_dither()
{
    return (fract(52.9829189 * fract(dot(gl_FragCoord.xy, vec2(0.06711056, 0.00583715)))) - 0.5) / 255.0;
}

// ---- main -----------------------------------------------------------------

void main()
{
    vec4 pos_tex  = texture(positions, vs_out.uv);
    vec4 norm_tex = texture(normals,   vs_out.uv);
    vec4 alb_tex  = texture(albedo,    vs_out.uv);

    // per-pixel normal change, taken before the background branch : derivatives are
    // only defined in uniform flow. it becomes a roughness floor below, so edges and
    // grazing silhouettes go soft instead of sparkling
    vec3 dn = fwidth(norm_tex.rgb);

    vec3 view_dir = normalize(pos_tex.rgb - vs_out.view_position); // eye -> surface

    // nothing rasterized here : the open sky
    if (dot(norm_tex.rgb, norm_tex.rgb) < 1e-8)
    {
        vec3 col = aces(exposure * environment(view_dir));
        pixel_color = vec4(pow(col, vec3(1.0 / 2.2)) + screen_dither(), 1.0);
        return;
    }

    // material : albedo is stored display-referred, and lighting runs linear
    vec3  base      = pow(alb_tex.rgb, vec3(2.2));
    float metallic  = clamp(pos_tex.a,  0.0, 1.0);
    float roughness = clamp(norm_tex.a, 0.045, 1.0);
    float ao        = clamp(alb_tex.a,  0.0, 1.0);

    vec3 N = normalize(norm_tex.rgb);
    vec3 P = pos_tex.rgb;
    vec3 V = -view_dir; // surface -> eye

    float NdotV = max(dot(N, V), 1e-4);

    // specular anti-aliasing : fold a little of the surface's own normal variance
    // into the lobe's roughness. the clamp is the important half - at a silhouette
    // the derivative jumps to the background's normal, and an unbounded fold makes
    // every edge fully rough, which on a metal reads as a black rim around it
    float variance   = min(dot(dn, dn), 0.09);
    float spec_rough = min(sqrt(roughness * roughness + variance), 1.0);

    vec3 F0 = mix(vec3(0.04), base, metallic);

    // occlusion : screen space, darkening the diffuse ambient only. a mirror is left
    // alone - the sky it reflects is not blocked by what stands in front of it, and
    // dimming the reflections is most of what made this read as flat
    float vis        = ssao(P, N);
    float diffuse_ao = ao * vis;

    // --- the lights ----------------------------------------------------------
    vec3 direct = direct_light(N, V, L_sun, sun_color * sun_power,
                               base, F0, metallic, spec_rough, NdotV)
                + direct_light(N, V, L_rim, rim_color * rim_power,
                               base, F0, metallic, spec_rough, NdotV);

    // --- ambient : the environment, split-sum --------------------------------
    vec3 diffuse_amb = ambient_irradiance(N) * base * (1.0 - metallic) / PI * diffuse_ao;

    // reflection : a rough surface sees a blur of its surroundings, which one sample
    // cannot give. bending the reflection toward the normal at least makes it read
    // the right part of the sky, and that is most of what roughness looks like
    vec3 R       = reflect(-V, N);
    vec3 env_dir = normalize(mix(R, N, spec_rough * spec_rough));
    vec3 env_spec = environment(env_dir);

    vec2 ab     = env_brdf(spec_rough, NdotV);
    vec3 FssEss = F0 * ab.x + ab.y;

    // energy : a rough metal loses light between its own microfacets and never gets
    // it back. this puts a share of the multiple scattering back (Fdez-Aguera's fit)
    vec3  F_avg  = F0 + (1.0 - F0) / 21.0;
    float Ems    = 1.0 - (ab.x + ab.y);
    vec3  FmsEms = min(Ems * F_avg / max(1.0 - F_avg * Ems, 1e-3), vec3(1.0));

    vec3 specular_amb = (FssEss + FmsEms * multiscatter) * env_spec * ao;

    vec3 color = direct + diffuse_amb + specular_amb;

    // --- distance : the world fades into the sky it stands in ----------------
    float dist = length(P - vs_out.view_position);
    color = mix(color, environment(view_dir), 1.0 - exp(-dist * fog_density));

    // --- output : exposure -> ACES -> display encode, 1 lsb of dither ---------
    color = aces(exposure * color);
    color = pow(color, vec3(1.0 / 2.2));
    pixel_color = vec4(color + screen_dither(), 1.0);
}
