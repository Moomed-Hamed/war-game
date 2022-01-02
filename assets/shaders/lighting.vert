#version 330 core

struct VS_OUT {
	vec2 tex_coords;
	vec3 view_pos;
};

layout (location = 0) in vec2 position;
layout (location = 1) in vec2 tex_coords;

uniform vec3 view_pos;

out VS_OUT vs_out;

void main()
{
	vs_out.tex_coords = tex_coords;
	vs_out.view_pos   = view_pos;

	gl_Position = vec4(position, 0.0, 1.0);
}