#version 330 core

layout (location = 0) in vec3 vertex_position;
layout (location = 3) in mat4 instance_model; // same vao layout as the static pass
layout (location = 7) in vec4 instance_mat_a; // albedo : tints the wire, white = no tint

uniform mat4 proj_view;

flat out vec3 tint;

void main()
{
   tint = instance_mat_a.rgb;
   gl_Position = proj_view * instance_model * vec4(vertex_position, 1.0);
}
