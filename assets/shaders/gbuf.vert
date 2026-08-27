#version 330 core

struct VS_OUT {
	vec2 uv;
	vec3 view_position;
};

 // screen quad vertices
layout (location = 0) in vec3 vertex_position;
layout (location = 1) in vec2 vertex_uv;

uniform vec3 view_pos; // camera information for lighting

out VS_OUT vs_out;

void main()
{
	vs_out.uv   = vertex_uv;
	vs_out.view_position = view_pos;

	gl_Position = vec4(vertex_position.xz, 0.0, 1.0);
}