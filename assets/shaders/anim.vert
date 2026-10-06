#version 420 core

// the animated twin of geom.vert : identical output (feeds the same geom.frag),
// but vertices are skinned through the skeleton UBO before the instance transform.
// inputs MUST match DrawBufferAnim::init() in src/anim.h

struct VS_OUT
{
	vec3 normal;         // normal vector
	vec3 world_position; // position of this vertex in world space
	vec2 uv;
};

const int MAX_JOINTS = 16;

layout (location = 0) in vec3 vertex_position;
layout (location = 1) in vec3 vertex_normal;
layout (location = 2) in vec2 vertex_uv;
layout (location = 3) in vec3 vertex_weights;
layout (location = 4) in ivec3 vertex_bones;
layout (location = 5) in mat4 instance_model; // model matrix for this instance

layout (binding = 1, std140) uniform skeleton
{
	mat4 joint_transforms[MAX_JOINTS];
};

uniform mat4 proj_view;

out VS_OUT vs_out;

void main()
{
	// column skinning : add_anim() transposes the row-major file at load, so joint * v
	// is the right order here
	vec4 skinned_pos    = vec4(0.0);
	vec4 skinned_normal = vec4(0.0);

	for (int i = 0; i < 3; i++) // 3 weights per vertex
	{
		mat4 joint = joint_transforms[vertex_bones[i]];

		skinned_pos    += (joint * vec4(vertex_position, 1.0)) * vertex_weights[i];
		skinned_normal += (joint * vec4(vertex_normal,   0.0)) * vertex_weights[i];
	}

	vec4 world_pos = instance_model * skinned_pos;

	vs_out.world_position = world_pos.xyz;
	vs_out.normal         = (instance_model * skinned_normal).xyz;
	vs_out.uv             = vertex_uv;

	gl_Position = proj_view * world_pos;
}
