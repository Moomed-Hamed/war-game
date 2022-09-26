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
	static float counter = 0; counter += dtime * 3.f;

	// player height = 1.8 units here
	vec3 pos; get_transform(player->capsule->body, &pos);
	player->eyes.position = pos + vec3(0, .8, 0) + vec3(0, sin(counter) * .05f, 0);

	camera_update_dir(&player->eyes, mouse.dx, mouse.dy, dtime);
	if (keys.W.is_pressed) set_linear_velocity(player->capsule->body, player->eyes.front *  5.f * dtime);
	if (keys.S.is_pressed) set_linear_velocity(player->capsule->body, player->eyes.front * -5.f * dtime);
	if (keys.A.is_pressed) set_linear_velocity(player->capsule->body, player->eyes.right * -5.f * dtime);
	if (keys.D.is_pressed) set_linear_velocity(player->capsule->body, player->eyes.right *  5.f * dtime);

	//if (keys.W.is_pressed) player->eyes.position += player->eyes.front * 15.f * dtime;
	//if (keys.S.is_pressed) player->eyes.position -= player->eyes.front * 15.f * dtime;
	//if (keys.A.is_pressed) player->eyes.position -= player->eyes.right * 15.f * dtime;
	//if (keys.D.is_pressed) player->eyes.position += player->eyes.right * 15.f * dtime;
}