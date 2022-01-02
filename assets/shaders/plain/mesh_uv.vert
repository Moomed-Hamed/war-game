#version 330 core

struct VS_OUT
{
	vec3 normal;   // normal vector
	vec3 frag_pos; // position of this pixel in world space
	vec2 tex_coord;
};

layout (location = 0) in vec3 position;
layout (location = 1) in vec3 normal;
layout (location = 2) in vec2 tex_coords;

uniform mat4 proj_view;

out VS_OUT vs_out;

void main()
{
	vs_out.normal     = normal;
	vs_out.frag_pos   = position;
	vs_out.tex_coord  = tex_coords;

	gl_Position = proj_view * vec4(vs_out.frag_pos, 1.0);
}
