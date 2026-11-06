#include "physics.h"

/* ------------------------- *
   -        Player         -
 * ------------------------- */

// first person player : a capsule that sets its own velocity (rotation locked)
// and jumps off a ground raycast, with a free fly camera on F.

const float PLAYER_RADIUS  = .4f;
const float PLAYER_HEIGHT  = 1.f; // cylinder part : 1.8 total
const float PLAYER_MASS    = 75.f;
const float WALK_SPEED     = 4.5f;
const float SPRINT_SCALE   = 1.9f;
const float JUMP_SPEED     = 5.5f;
const float EYE_HEIGHT     = .75f; // above the capsule center

struct Player
{
	Camera camera; // eyes
	uint   body;   // capsule phys_id (0 = none)
	bool   flycam; // F : free camera instead of the capsule
	float  bob;    // walk bob phase
	bool   grounded;

	void init(Physics* phys, vec3 spawn)
	{
		body = phys->add_capsule(spawn, PLAYER_RADIUS, PLAYER_HEIGHT, PLAYER_MASS);
		if (!body) return;

		btRigidBody* b = phys->bodies[body].body;
		b->setAngularFactor(0); // never tips over
		b->setFriction(.9f);

		camera.position = spawn + vec3(0, EYE_HEIGHT, 0);
		camera.yaw      = -PI / 2; // facing -z, into the scene
		camera.pitch    = 0;
		camera.update_dir(0, 0, 0);

		console->add_entry("Player spawned : F swaps to the fly camera", SUCCESS, PHYS);
	}

	// one frame : input -> movement -> camera
	void update(GameWindow* window, Physics* phys)
	{
		ImGuiIO& io = ImGui::GetIO();
		float dt = window->dtime;

		bool ours = window->focus_status == GAME;

		// F : swap between the capsule and the free camera
		if (ours && !io.WantCaptureKeyboard &&
			window->keys.F.is_pressed && !window->keys.F.was_pressed)
			flycam = !flycam;

		if (flycam) return;

		vec2 look = vec2(0);
		if (ours && !io.WantCaptureMouse)
		{
			look = vec2((float)window->mouse.dx, (float)window->mouse.dy);
			camera.update_dir(look.x, look.y, dt);
		}

		move(window, phys); // every frame : the eyes ride the capsule
	}

	// capsule movement : velocity is set directly, gravity stays bullet's
	void move(GameWindow* window, Physics* phys)
	{
		if (!body) return;

		btRigidBody* b = phys->bodies[body].body;
		vec3 pos = phys->get_position(body);

		float half = PLAYER_HEIGHT * .5f + PLAYER_RADIUS;
		grounded = phys->raycast(pos, pos - vec3(0, half + .15f, 0)).hit;

		vec3 vel  = phys->get_velocity(body);
		vec3 flat = vec3(vel.x, 0, vel.z);

		// wish direction from the camera's yaw only : looking up never slows you
		vec3 fwd   = normalize(vec3(camera.front.x, 0, camera.front.z));
		vec3 right = normalize(vec3(camera.right.x, 0, camera.right.z));

		vec3 wish = vec3(0);
		if (window->keys.W.is_pressed) wish += fwd;
		if (window->keys.S.is_pressed) wish -= fwd;
		if (window->keys.D.is_pressed) wish += right;
		if (window->keys.A.is_pressed) wish -= right;
		if (length(wish) > 0) wish = normalize(wish);

		float speed = WALK_SPEED * (window->keys.SHIFT.is_pressed ? SPRINT_SCALE : 1.f);
		float accel = grounded ? 12.f : 2.5f; // weak air control, on purpose
		flat += (wish * speed - flat) * (1.f - expf(-accel * window->dtime));

		if (grounded && window->keys.SPACE.is_pressed && !window->keys.SPACE.was_pressed)
			vel.y = JUMP_SPEED;

		b->setLinearVelocity(btVector3(flat.x, vel.y, flat.z));
		b->activate(true); // the player is always in play

		// eyes : capsule center, an eye height up, bobbing with the walk
		float walk = length(flat);
		bob += window->dtime * (1.f + walk);
		float amp = grounded ? walk / WALK_SPEED : 0;
		camera.position = pos + vec3(cosf(bob * 2.f) * .02f * amp,
		                             EYE_HEIGHT + sinf(bob * 4.f) * .022f * amp, 0);
	}
};
