#version 330 core

struct VS_OUT
{
	vec3 color;
};

in VS_OUT vs_out;

layout (location = 0) out vec4 frag_color;


void main()
{
	frag_color = vec4(vs_out.color , 0);
}