#version 330 core

layout (location = 0) in vec3 vertex_position;
layout (location = 1) in vec3 vertex_normal;
layout (location = 2) in vec2 vertex_uv;

layout (location = 3) in mat4 instance_model; // model matrix for this instance
layout (location = 7) in vec4 instance_mat_a; // albedo rgb + metallic
layout (location = 8) in vec4 instance_mat_b; // roughness, ao, texture-albedo flag

struct VS_OUT
{
	vec3 normal;         // normal vector
	vec3 world_position; // position of this vertex in world space
	vec2 uv;
};

out VS_OUT vs_out;

// material : flat, the same for every vertex of the instance
flat out vec4 mat_a;
flat out vec4 mat_b;

uniform mat4 proj_view;

void main()
{
   vec4 world_pos = instance_model * vec4(vertex_position, 1.0);
   vs_out.world_position = world_pos.xyz;

   // inverse transpose : instance scale is not always uniform (the car chassis), and
   // a plain multiply would skew the normal along with the geometry
   vs_out.normal = transpose(inverse(mat3(instance_model))) * vertex_normal;

   vs_out.uv = vertex_uv;

   mat_a = instance_mat_a;
   mat_b = instance_mat_b;

   gl_Position = proj_view * world_pos;
}