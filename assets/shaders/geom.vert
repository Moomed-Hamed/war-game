#version 330 core

layout (location = 0) in vec3 vertex_position;
layout (location = 1) in vec3 vertex_normal;
layout (location = 2) in vec2 vertex_uv;

layout (location = 3) in mat4 instance_model; // model matrix for this instance

struct VS_OUT
{
	vec3 normal;         // normal vector
	vec3 world_position; // position of this vertex in world space
	vec2 uv;
};

out VS_OUT vs_out;

uniform mat4 proj_view;

void main()
{
   vec4 world_pos = instance_model * vec4(vertex_position, 1.0);
   vs_out.world_position = world_pos.xyz;

   vec4 world_normal = instance_model * vec4(vertex_normal, 0.0);
   vs_out.normal = world_normal.xyz;

   vs_out.uv = vertex_uv;

   gl_Position = proj_view * world_pos;
}