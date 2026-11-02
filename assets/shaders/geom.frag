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

uniform float night; // 0 = day, 1 = night : emitters only glow at night

// material : comes in flat from the instance (geom.vert / anim.vert)
flat in vec4 mat_a; // albedo rgb + metallic
flat in vec4 mat_b; // roughness, ao, texture-albedo flag

float hash21(vec2 p)
{
	p = fract(p * vec2(123.34, 456.21));
	p += dot(p, p + 45.32);
	return fract(p.x * p.y);
}

float vnoise(vec2 p)
{
	vec2 i = floor(p);
	vec2 f = fract(p);
	f = f * f * (3.0 - 2.0 * f);
	float a = hash21(i);
	float b = hash21(i + vec2(1.0, 0.0));
	float c = hash21(i + vec2(0.0, 1.0));
	float d = hash21(i + vec2(1.0, 1.0));
	return mix(mix(a, b, f.x), mix(c, d, f.x), f.y);
}

void main()
{
	// albedo is display space, the lighting pass decodes the g-buffer. the flag picks
	// between the texture tinted by the material and the material colour alone.
	// mat_b.b > 1.5 marks a procedural fabric surface : soft bodies use their
	// grid uv for a woven thread pattern instead of a texture.
	float ao = mat_b.g;
	vec3 base;

	if (mat_b.b > 1.5)
	{
		vec2 uv = vs_out.uv;
		vec2 q = uv * vec2(26.0, 34.0);
		float aa = fwidth(q.x) + fwidth(q.y);
		float detail = 1.0 - smoothstep(.55, 1.6, aa);

		// warp/weft plus a twill diagonal, faded with distance to stop aliasing
		float warp = .5 + .5 * sin(q.x * 6.28318 + .35 * sin(q.y * 2.1));
		float weft = .5 + .5 * sin(q.y * 6.28318 + .35 * sin(q.x * 2.1));
		float twill = .5 + .5 * sin((q.x + q.y) * 3.14159);
		float weave = mix(.5, .35 * warp + .35 * weft + .30 * twill, detail);

		float fibre = vnoise(uv * vec2(55.0, 75.0));
		float slub = vnoise(uv * vec2(9.0, 13.0));

		vec3 cloth = mat_a.rgb;
		cloth *= .84 + .16 * weave;
		cloth *= .93 + .07 * fibre;
		cloth *= .94 + .06 * slub;

		// subtle herringbone tonal stripe
		float herring = step(.5, fract((uv.x + uv.y) * 18.0 +
		                              .5 * sin(uv.y * 12.0)));
		cloth = mix(cloth, cloth * 1.10, herring * detail * .22);

		// folded hem and a fine stitched line just inside it
		float edge = min(min(uv.x, 1.0 - uv.x), min(uv.y, 1.0 - uv.y));
		float hem = 1.0 - smoothstep(0.0, .032, edge);
		cloth = mix(cloth, cloth * .44, hem);

		float stitch = smoothstep(.040, .044, edge) *
		               (1.0 - smoothstep(.050, .054, edge));
		cloth = mix(cloth, vec3(.66, .58, .42), stitch * .55);

		// lining on the back side : soft bodies are rendered double-sided
		if (!gl_FrontFacing) cloth *= .66;

		base = cloth;
		ao *= .88 + .12 * weave;
		ao *= mix(.76, 1.0, smoothstep(0.0, .05, edge));
	}
	else
	{
		vec3 tex = texture(texture_data, vs_out.uv * 4.f).rgb;
		base = mix(mat_a.rgb, tex * mat_a.rgb, mat_b.b);
	}

	// mat_b.w flags an emitter : the excess above 1 in the ao channel becomes
	// the emissive strength in the lighting pass (lamp heads, light panels)
	if (mat_b.w > .5) ao = 1.0 + mat_b.w * night;

	pixel_position = vec4(vs_out.world_position, mat_a.a); // metalness
	pixel_normal   = vec4(normalize(vs_out.normal), mat_b.r);
	pixel_albedo   = vec4(base, ao);                       // ambient occlusion
}
