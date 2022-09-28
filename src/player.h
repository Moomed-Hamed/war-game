#include "props.h"

#define FOV ToRadians(45.0f)

struct Player
{
	Camera eyes;
	Phys_Capsule* capsule;
};

void init(Player* player, Physics* phys)
{
	uint index = add_phys_capsule(phys, { 1, 4, 6 });
	player->capsule = &phys->capsules[index];
	player->capsule->body->setAngularFactor(0); // disable rotation
}
void update(Player* player, float dtime, Heightmap* map, Keyboard keys, Mouse mouse)
{
	static float counter = 0; counter += dtime * 2.f;

	// player height = 1.8 units
	vec3 pos; get_transform(player->capsule->body, &pos);
	player->eyes.position = pos + vec3(0, .8, 0) + vec3(0, sin(counter) * .01f, 0);

	{
		bool sprint = keys.SHIFT.is_pressed;
		vec3 player_front = normalize(vec3(1, 0, 1) * player->eyes.front);
		vec3 player_right = normalize(vec3(1, 0, 1) * player->eyes.right);

		Vec3 vel = player->capsule->body->getLinearVelocity();

		vec3 v = {vel.x(), 0, vel.z()};
		if (keys.W.is_pressed) v = player_front *  1.f * (sprint ? 8.f : 3.f);
		if (keys.S.is_pressed) v = player_front * -1.f * (sprint ? 8.f : 3.f);
		if (keys.A.is_pressed) v = player_right * -1.f * (sprint ? 8.f : 3.f);
		if (keys.D.is_pressed) v = player_right *  1.f * (sprint ? 8.f : 3.f);

		v.y = vel.y();

		if (keys.SPACE.is_pressed && !keys.SPACE.was_pressed) // jump
			v += vec3(0, 5, 0);

		set_linear_velocity(player->capsule->body, v);
	}

	camera_update_dir(&player->eyes, mouse.dx, mouse.dy, dtime); player->capsule->position = vec3(-111);
}