#include "props.h"

#define FOV ToRadians(45.0f)

struct Player
{
	Camera eyes;
	Phys_Capsule* capsule;
};

void apply_central_impulse(btRigidBody* body, vec3 impulse)
{
	body->setActivationState(1);
	body->applyCentralImpulse(Vec3(impulse.x, impulse.y, impulse.z));
}
void set_linear_velocity(btRigidBody* body, vec3 velocity)
{
	body->setActivationState(1);
	body->applyCentralImpulse(Vec3(velocity.x, velocity.y, velocity.z));
}

void init(Player* player, Physics* phys)
{
	uint index = add_phys_capsule(phys, { 1, 6, 1 });
	player->capsule = &phys->capsules[index];

	player->capsule->body->setAngularFactor(0); // disable rotation
}
void update(Player* player, float dtime, Heightmap* map, Keyboard keys, Mouse mouse)
{
	static float counter = 0; counter += dtime * 2.f;

	// player height = 1.8 units
	vec3 pos; get_transform(player->capsule->body, &pos);
	player->eyes.position = pos + vec3(0, .8, 0) + vec3(0, sin(counter) * .01f, 0);

	player->capsule->body->setDamping(0,0);

	if (keys.SHIFT.is_pressed) // sprint
	{
		if (keys.W.is_pressed) set_linear_velocity(player->capsule->body, player->eyes.front *  10.f * dtime);
		if (keys.S.is_pressed) set_linear_velocity(player->capsule->body, player->eyes.front * -10.f * dtime);
		if (keys.A.is_pressed) set_linear_velocity(player->capsule->body, player->eyes.right * -10.f * dtime);
		if (keys.D.is_pressed) set_linear_velocity(player->capsule->body, player->eyes.right *  10.f * dtime);
	}
	else if(keys.W.is_pressed || keys.A.is_pressed || keys.S.is_pressed || keys.D.is_pressed) // move
	{
		if (keys.W.is_pressed) set_linear_velocity(player->capsule->body, player->eyes.front *  5.f * dtime);
		if (keys.S.is_pressed) set_linear_velocity(player->capsule->body, player->eyes.front * -5.f * dtime);
		if (keys.A.is_pressed) set_linear_velocity(player->capsule->body, player->eyes.right * -5.f * dtime);
		if (keys.D.is_pressed) set_linear_velocity(player->capsule->body, player->eyes.right *  5.f * dtime);
	} // else if (keys.SPACE.is_pressed) set_linear_velocity(player->capsule->body, vec3(0, 10, 0));
	else
	{
		player->capsule->body->setDamping(.8, .8);
	}

	camera_update_dir(&player->eyes, mouse.dx, mouse.dy, dtime);

}