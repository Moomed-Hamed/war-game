#include "props.h"

// (FABRIK) inverse kinematics + ai = active ragdolls (probably need an animation update too)
struct Peer
{
	vec3 feet_position;
	vec3 look_direction;
	Cylinder_Collider hitbox;

	float health = 100;
};

void init(Peer* peer)
{
	init_collider(&peer->hitbox, vec3(0, 1, 0), vec3(0, 0, 0), vec3(0, 0, 0), 1, 1, .5);
}
void update(Peer* peer, float dtime)
{
	peer->hitbox.position = peer->feet_position;
}

// rendering

struct Peer_Drawable
{
	vec3 position;
	mat3 rotation;
};

struct Peer_Renderer
{
	Drawable_Mesh_Anim_UV mesh;
	Shader shader;
	Animation animation;
	mat4 current_pose[MAX_ANIMATED_BONES];
};

void init(Peer_Renderer* renderer)
{
	load(&renderer->mesh, "assets/meshes/peer.mesh_anim", sizeof(Peer_Drawable));
	mesh_add_attrib_vec3(5, sizeof(Peer_Drawable), 0); // world pos
	mesh_add_attrib_mat3(6, sizeof(Peer_Drawable), sizeof(vec3)); // rotation

	renderer->mesh.texture_id  = load_texture("assets/textures/palette2.bmp");
	renderer->mesh.material_id = load_texture("assets/textures/materials.bmp");

	load(&(renderer->shader), "assets/shaders/transform/mesh_anim_uv.vert", "assets/shaders/mesh_uv.frag");
	load(&renderer->animation, "assets/animations/peer.anim"); // animaiton keyframes
}
void update_renderer(Peer_Renderer* renderer, float dtime, Peer peer)
{
	update_animation(&renderer->animation, renderer->current_pose, dtime * 2);

	Peer_Drawable drawable = {};
	drawable.position = peer.feet_position + vec3(7, 0, -5);
	drawable.rotation = point_at(vec3(peer.look_direction.x, 0, peer.look_direction.z), vec3(0, 1, 0));

	update(renderer->mesh, renderer->animation.num_bones, renderer->current_pose, sizeof(Peer_Drawable), (byte*)(&drawable));
}
void draw(Peer_Renderer* renderer, mat4 proj_view)
{
	bind(renderer->shader);
	set_mat4(renderer->shader, "proj_view", proj_view);
	bind_texture(renderer->mesh);
	draw(renderer->mesh);
}