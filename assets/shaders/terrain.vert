#version 420 core

struct VS_OUT
{
	vec3 frag_pos;
	vec2 tex_coord;
	vec3 color;
};

layout (location = 0) in vec3 position;
layout (location = 1) in vec3 normal;

uniform mat4 proj_view;

layout (binding = 2) uniform sampler2D heightmap;

out VS_OUT vs_out;

void main()
{
	vec2 tex_coord = position.xz / 50;
	float h = 1.5 * texture(heightmap, tex_coord).r;

	if(h > 0.0001)
	{
	  vs_out.frag_pos  = position + vec3(0, h - .4, 0);
	} else vs_out.frag_pos  = position + vec3(0, -9, 0);

	
	vs_out.tex_coord = tex_coord;
	vs_out.color     = vec3(.8, .58, .28);

	gl_Position = proj_view * vec4(vs_out.frag_pos, 1.0);
}