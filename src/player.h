#include "props.h"

#define FOV ToRadians(45.0f)

struct Player
{
	Camera eyes;
	Sphere_Collider feet;
};

void init(Player* player)
{
	player->feet = { vec3(1), vec3(0), vec3(0), 1, .25 };
}
void update(Player* player, float dtime, Heightmap* map, Keyboard keys, Mouse mouse)
{
	camera_update_dir(&player->eyes, mouse.dx, mouse.dy, dtime);

	float side_velocity = 0, front_velocity = 0;
	if (keys.W.is_pressed) front_velocity =  6.f;
	if (keys.S.is_pressed) front_velocity = -6.f;
	if (keys.A.is_pressed) side_velocity  = -6.f;
	if (keys.D.is_pressed) side_velocity  =  6.f;

	vec3 movement_velocity  = (player->eyes.front * front_velocity) + (player->eyes.right * side_velocity);
	player->feet.position  += dtime * movement_velocity;
	player->feet.position.y = height(map, player->feet.position);

	static float jump_timer  = -1;
	static float jump_offset = 0;

	if (jump_timer < 0 && keys.SPACE.is_pressed && !keys.SPACE.was_pressed)
	{
		jump_timer = 1 / 1.5f;
		player->eyes.trauma = .3;
	}

	if (jump_timer > 0)
	{
		jump_offset = 1.5 * sin(jump_timer * 1.5 * PI);
		jump_timer -= dtime;
		if (jump_timer < 0) player->eyes.trauma = .3;
	}

	player->eyes.position   = player->feet.position;
	player->eyes.position.y = player->feet.position.y + jump_offset + 1.8f;
}