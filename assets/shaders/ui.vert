#version 330 core

layout (location = 0) in vec3 position;
layout (location = 1) in vec3 normal;
layout (location = 2) in vec3 color;
layout (location = 3) in vec3 scale;
layout (location = 4) in vec3 world_position;
layout (location = 5) in mat3 rotation;

struct VS_OUT
{
	vec3 color;
};

out VS_OUT vs_out;

void main()
{
	vs_out.color = color;

	vec3 final_position = (rotation * (position * scale)) + world_position;
	gl_Position = vec4(final_position, 1.0);
}