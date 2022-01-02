#version 330 core

layout (location = 0) in vec2 vertex;
layout (location = 1) in vec2 position;
layout (location = 2) in vec2 scale;
layout (location = 3) in vec2 tex_coords;

struct VS_OUT
{
	vec2 frag_pos;
	vec2 tex_coords;
};

out VS_OUT vs_out;

void main()
{
	vs_out.frag_pos   = (vertex * scale) + position;
	vs_out.tex_coords = tex_coords;

	gl_Position = vec4(vs_out.frag_pos, 0, 1.0);
}