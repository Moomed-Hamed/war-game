#version 330 core

layout (location = 0) in vec3 vertex_position;
layout (location = 3) in mat4 instance_model; // same vao layout as the static pass

uniform mat4 proj_view;

void main()
{
   gl_Position = proj_view * instance_model * vec4(vertex_position, 1.0);
}
