#version 420 core

// the last pass : exposure, ACES and the display encode. everything before this
// point is linear HDR, so a local light and the sun add on the same scale

struct VS_OUT {
    vec2 uv;
    vec3 view_position;
};

in VS_OUT vs_out;

layout (binding = 0) uniform sampler2D lit_color;

layout (location = 0) out vec4 pixel_color;

uniform float exposure;
uniform float contrast;
uniform float saturation;

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
    vec3 color = texture(lit_color, vs_out.uv).rgb;
    color = aces(exposure * color);

    // a small grade : ACES rolls colour off in the highlights, and the tropical
    // presets read better with the turquoise kept alive
    color = (color - 0.5) * contrast + 0.5;
    float luma = dot(color, vec3(0.2126, 0.7152, 0.0722));
    color = mix(vec3(luma), color, saturation);

    color = clamp(color, 0.0, 1.0);
    color = pow(color, vec3(1.0 / 2.2));
    pixel_color = vec4(color + screen_dither(), 1.0);
}
