#version 420 core

struct VS_OUT
{
	vec3 normal;   // normal vector
	vec3 frag_pos; // position of this pixel in world space
	vec2 tex_coord;
};

const int MAX_JOINTS = 16;

layout (location = 0) in vec3 position;
layout (location = 1) in vec3 normal;
layout (location = 2) in vec3 weight;
layout (location = 3) in ivec3 bones;
layout (location = 4) in vec2 tex_coord;
layout (location = 5) in vec3 world_position;
layout (location = 6) in mat3 rotation;
layout (location = 9) in mat3 local_rotation;

layout (binding = 1, std140) uniform skeleton
{
	uniform mat4 joint_transforms[MAX_JOINTS];
};

uniform mat4 proj_view;

out VS_OUT vs_out;

void main()
{
	vec4 final_pos    = vec4(0, 0, 0, 0);
	vec4 final_normal = vec4(0, 0, 0, 0);

	vec3 pos = local_rotation * position;
	vec3 nor = local_rotation * normal;

	for(int i = 0; i < 3; i++) // 3 weights per vertex
	{
		mat4 joint_transform = joint_transforms[bones[i]];

		vec4 pose_position = vec4(pos, 1.0) * joint_transform;
		vec4 pose_normal   = vec4(nor, 0.0) * joint_transform;

		final_pos    += pose_position * weight[i];
		final_normal += pose_normal   * weight[i];
	}
	
	vs_out.frag_pos   = (rotation * final_pos.xyz) + world_position;
	vs_out.normal     = (rotation * final_normal.xyz);
	vs_out.tex_coord = tex_coord;

	gl_Position = proj_view * vec4(vs_out.frag_pos, 1.0);
}
