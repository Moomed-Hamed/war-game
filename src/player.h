#include "props.h"

#define FOV ToRadians(45.0f)

struct Player
{
	Camera eyes;
	RigidBody body;
};

void init(Player* player)
{
	player->eyes.position = { vec3{0,6,0} };
}
void update(Player* player, float dtime, Heightmap* map, Keyboard keys, Mouse mouse)
{
	camera_update_dir(&player->eyes, mouse.dx, mouse.dy, dtime);

	if (keys.W.is_pressed) player->eyes.position += player->eyes.front * 15.f * dtime;
	if (keys.S.is_pressed) player->eyes.position -= player->eyes.front * 15.f * dtime;
	if (keys.A.is_pressed) player->eyes.position -= player->eyes.right * 15.f * dtime;
	if (keys.D.is_pressed) player->eyes.position += player->eyes.right * 15.f * dtime;
}