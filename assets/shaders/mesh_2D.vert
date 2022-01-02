#version 330 core

layout (location = 0) in vec2 vertex;
layout (location = 1) in vec2 position;
layout (location = 2) in vec2 scale;
layout (location = 3) in vec3 color;

struct VS_OUT
{
	vec2 frag_pos;
	vec3 color;
};

out VS_OUT vs_out;

void main()
{
	vs_out.color = color;
	vs_out.frag_pos = (vertex * scale) + position;
	gl_Position = vec4((vertex * scale) + position, 0, 1.0);
}