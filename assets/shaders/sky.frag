#version 420 core

// sky disks : a linear additive pass over the deferred frame, before water. the
// opaque depth mask keeps them off terrain; forward water then covers them

struct VS_OUT {
    vec2 uv;
    vec3 view_position;
};

in VS_OUT vs_out;

layout (binding = 0) uniform sampler2D scene_depth;

layout (location = 0) out vec4 pixel_color;

uniform vec2 sun_screen;
uniform vec2 moon_screen;
uniform vec3 sun_color;
uniform vec3 moon_color;
uniform float sun_strength;
uniform float moon_strength;
uniform float aspect;

vec3 sky_disk(vec2 uv, vec2 center, vec3 color, float strength, float radius)
{
    vec2 d = (uv - center) * vec2(aspect, 1.0);
    float r = length(d);
    float core = smoothstep(radius, radius * 0.72, r);
    float halo = pow(max(0.0, 1.0 - r / (radius * 5.0)), 3.0);
    return color * (core * 34.0 + halo * 1.4) * strength;
}

void main()
{
    if (texture(scene_depth, vs_out.uv).r < 0.99999)
        discard;

    vec3 col = sky_disk(vs_out.uv, sun_screen, sun_color, sun_strength, 0.011);
    col += sky_disk(vs_out.uv, moon_screen, moon_color, moon_strength, 0.009);
    pixel_color = vec4(col, 1.0);
}
