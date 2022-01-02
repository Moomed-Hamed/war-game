#include "peer.h"

#define FOV ToRadians(45.0f)

#define ITEM_GUN	0
#define ITEM_SWORD	1
#define ITEM_POTION	2

struct Player
{
	uint item_id; // id of item player is holding
	Camera eyes;
	Sphere_Collider feet;
};

void init(Player* player)
{
	init_collider(&player->feet, vec3(0, 1, 0), vec3(0, 0, 0), vec3(0, 0, 0), 1, .25);
}
void update(Player* player, float dtime, Keyboard keys, Mouse mouse)
{
	// i don't really have any idea why this works but it does so it stays 4 now

	Sphere_Collider* feet = &player->feet;

	camera_update_dir(&(player->eyes), mouse.dx, mouse.dy, dtime);

	// decoupling movement controls from physics
	float side_velocity    = 0;
	float forward_velocity = 0;

	if (keys.W.is_pressed) forward_velocity =  6.f;
	if (keys.S.is_pressed) forward_velocity = -6.f;
	if (keys.A.is_pressed) side_velocity    = -6.f;
	if (keys.D.is_pressed) side_velocity    =  6.f;

	vec3 movement_velocity = (player->eyes.front * forward_velocity) + (player->eyes.right * side_velocity);
	movement_velocity.y = 0;

	feet->position += dtime * movement_velocity;

	Plane_Collider ground = {};
	init_collider(&ground, vec3(0, 0, 0), vec3(0, 0, 0), vec3(0, 0, 0), vec3(0, 1, 0), vec2(100, 100));

	vec3 force = vec3(feet->force.x, feet->force.y + GRAVITY, feet->force.z);

	float bounce_impulse = 0;
	if (sphere_plane_intersect(*feet, ground) && dot(feet->velocity, ground.normal) < 0)
	{
		if (dot(feet->velocity, ground.normal) < 0) bounce_impulse = feet->velocity.y * -1.f;
	
		float penetration_depth = abs(feet->position.y - feet->radius); // distance from contact point to plane
		feet->velocity = penetration_depth * ground.normal * 10.f;
	}

	feet->velocity += dtime * ((force / feet->mass) + bounce_impulse);
	feet->position += dtime * feet->velocity;

	static float jump_timer  = -1;
	static float jump_offset = 0;

	if (jump_timer < 0 && keys.SPACE.is_pressed && !keys.SPACE.was_pressed)
	{
		jump_timer = 1 / 1.5f;
	}

	if (jump_timer > 0)
	{
		jump_offset = 1.5 * sin(jump_timer * 1.5 * PI);
		jump_timer -= dtime;
	}

	player->eyes.position   = player->feet.position;
	player->eyes.position.y = player->feet.position.y + 1.3f + jump_offset;

	// item selection
	if (keys.I.is_pressed) player->item_id = ITEM_GUN;
	if (keys.O.is_pressed) player->item_id = ITEM_SWORD;
	if (keys.P.is_pressed) player->item_id = ITEM_POTION;
}

// rendering

struct Player_Renderer
{
	uint item_id;
	Drawable_Mesh_UV sword;
	Drawable_Mesh_UV potion;

	Shader mesh_uv, mesh_anim;
};

void init(Player_Renderer* renderer)
{
	load(&(renderer->mesh_uv), "assets/shaders/transform/mesh_uv.vert", "assets/shaders/mesh_uv.frag");

	load(&renderer->sword, "assets/meshes/sword.mesh_uv", sizeof(Prop_Drawable));
	mesh_add_attrib_vec3(3, sizeof(Prop_Drawable), 0); // world pos
	mesh_add_attrib_mat3(4, sizeof(Prop_Drawable), sizeof(vec3)); // transform
	renderer->sword.texture_id  = load_texture("assets/textures/palette2.bmp");
	renderer->sword.material_id = load_texture("assets/textures/materials.bmp");

	load(&renderer->potion, "assets/meshes/potion.mesh_uv", sizeof(Prop_Drawable));
	mesh_add_attrib_vec3(3, sizeof(Prop_Drawable), 0); // world pos
	mesh_add_attrib_mat3(4, sizeof(Prop_Drawable), sizeof(vec3)); // transform
	renderer->potion.texture_id  = load_texture("assets/textures/palette2.bmp");
	renderer->potion.material_id = load_texture("assets/textures/materials.bmp");

}
void update_renderer(Player_Renderer* renderer, float dtime, Player player, Mouse mouse)
{
	renderer->item_id = player.item_id;

	vec3 position = player.eyes.position;
	vec3 front    = player.eyes.front;
	vec3 right    = player.eyes.right;
	vec3 up       = player.eyes.up;

	Prop_Drawable drawable = {};
	drawable.position  = position + (front * 0.9f) + (up * -.0f) + (right * .0f);
	drawable.transform = point_at(front, up);

	static float turn_amount = 0; turn_amount += mouse.norm_dx;
	if (turn_amount >  .1) turn_amount = .1;
	if (turn_amount < -.1) turn_amount = -.1;

	vec3 look = lerp(front * -1.f, right, -1 * (-.05 + turn_amount)); turn_amount *= dtime;

	switch (player.item_id)
	{
		case ITEM_GUN: {
			return;
		} break;

		case ITEM_SWORD: {
			drawable.position = position + front + (up * -.4f) + (right * .4f);
			drawable.transform = point_at(look, up) * mat3(rotate(PI / 2, vec3(0, 1, 0)));
			update(renderer->sword, sizeof(Prop_Drawable), (byte*)(&drawable));
		}

		default: {
			drawable.position  = position + (front * .9f) + (up * -.4f) + (right * .4f) + vec3(turn_amount);
			drawable.transform = point_at(front, up);
			update(renderer->potion, sizeof(Prop_Drawable), (byte*)(&drawable));
		} break;
	}

	
}
void draw(Player_Renderer* renderer, mat4 proj_view)
{
	switch (renderer->item_id)
	{
		case ITEM_GUN: {
		} break;

		case ITEM_POTION: {
			bind(renderer->mesh_uv);
			bind_texture(renderer->potion);
			set_mat4(renderer->mesh_uv, "proj_view", proj_view);
			draw(renderer->potion);
		} break;

		default: {
			bind(renderer->mesh_uv);
			bind_texture(renderer->sword);
			set_mat4(renderer->mesh_uv, "proj_view", proj_view);
			draw(renderer->sword);
		} break;
	}
}