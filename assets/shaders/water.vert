#version 420 core

// forward water : the same instance layout as geom.vert, but this pass runs after
// the opaque frame so the shader can sample what is behind the surface

layout (location = 0) in vec3 vertex_position;
layout (location = 1) in vec3 vertex_normal;
layout (location = 2) in vec2 vertex_uv;

layout (location = 3) in mat4 instance_model;
layout (location = 7) in vec4 instance_mat_a;
layout (location = 8) in vec4 instance_mat_b;

struct VS_OUT
{
	vec3 normal;
	vec3 world_position;
	vec2 uv;
};

out VS_OUT vs_out;
flat out vec4 mat_a; // water colour rgb + mean surface height
flat out vec4 mat_b; // roughness, ao, open-sea flag, 1

uniform mat4 proj_view;

void main()
{
	vec4 world_pos = instance_model * vec4(vertex_position, 1.0);
	vs_out.world_position = world_pos.xyz;
	vs_out.normal = transpose(inverse(mat3(instance_model))) * vertex_normal;
	vs_out.uv = vertex_uv;

	mat_a = instance_mat_a;
	mat_b = instance_mat_b;

	gl_Position = proj_view * world_pos;
}
