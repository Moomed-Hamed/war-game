#version 420 core

// depth-only sun pass : position through the instance model

layout (location = 0) in vec3 vertex_position;
layout (location = 3) in mat4 instance_model;

uniform mat4 light_proj_view;

void main()
{
	gl_Position = light_proj_view * instance_model * vec4(vertex_position, 1.0);
}
