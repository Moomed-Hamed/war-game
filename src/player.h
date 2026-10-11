#include "physics.h"

/* ------------------------- *
   -        Player         -
 * ------------------------- */

// first person player + view model : a capsule that sets its own velocity
// (rotation locked) and jumps off a ground raycast, a fly camera on F, and a gun
// whose clip slices carry the action frames (order set in scene init).
// the clip is authored with the camera at the origin, so it places the gun itself ;
// sway, bob and recoil ride on top ; firing spawns projectiles into the world

const uint  MAX_BULLETS    = 32;
const float PLAYER_RADIUS  = .4f;
const float PLAYER_HEIGHT  = 1.f; // cylinder part : 1.8 total
const float PLAYER_MASS    = 75.f;
const float WALK_SPEED     = 4.5f;
const float SPRINT_SCALE   = 1.9f;
const float JUMP_SPEED     = 5.5f;
const float EYE_HEIGHT     = .75f; // above the capsule center

const float GUN_SHOT_TIME    = .60f;
const float GUN_RELOAD_TIME  = 3.58f; // the reload take is 86 frames at 24 fps : play it as authored
const float GUN_INSPECT_TIME = 1.70f;
const uint  GUN_MAG          = 17;
const float BULLET_SPEED     = 110.f;
const float BULLET_PUSH      = 5.f; // impulse per hit

// view model : the clip is authored with the camera at the origin, so it sits the gun in front
// of the eye on its own - there is no placement offset to apply. these are the feel, not placement
const float WEAPON_SCALE  = 1.f;
const float WEAPON_KICK   = .5f;    // recoil kick -> metres
const float WEAPON_SWAY   = .0035f; // mouse delta -> sway radians
const float WEAPON_ROLL   = 1.4f;   // shot : how much the kick rolls the gun

// the action *is* the clip's segment index : the action time drives the
// segment with the same number
enum { GUN_RELOAD = 0, GUN_SHOOT, GUN_INSPECT, GUN_IDLE };

struct Player
{
	Camera camera; // eyes
	uint   body;   // capsule phys_id (0 = none)
	bool   flycam; // F : free camera instead of the capsule

	// weapon : its own mesh + player so the view model animates on its own
	uint  gun_mesh, gun_player;
	uint  bullet_mesh; // small sphere for the projectiles
	uint  sfx_fire[3], sfx_reload, sfx_inspect, sfx_hit; // mixer slots
	uint  shot_variant; // which shot take plays next
	uint  action;      // GUN_* (also the segment index)
	float action_time; // seconds left in the action
	uint  ammo;

	// juice : every one of these springs back to nothing
	vec3  kick, kick_vel; // recoil offset on the gun, view space (right, up, back)
	vec2  sway, sway_vel; // look sway, lags the mouse
	float fov_punch;      // degrees added on fire, decays
	float bob;            // walk bob phase
	bool  grounded;

	// projectiles : stepped and raycast every frame
	struct Bullet { vec3 pos, vel; float life; } bullets[MAX_BULLETS];
	uint fired, hits;

	void init(GameRenderer* renderer, Physics* phys, uint mesh)
	{
		body = phys->add_capsule(vec3(0, 5, 8), PLAYER_RADIUS, PLAYER_HEIGHT, PLAYER_MASS);
		if (!body) return;

		btRigidBody* b = phys->bodies[body].body;
		b->setAngularFactor(0); // never tips over
		b->setFriction(.9f);

		camera.position = vec3(0, 5 + EYE_HEIGHT, 8);
		camera.yaw      = -PI / 2; // facing -z, into the scene
		camera.pitch    = 0;
		camera.update_dir(0, 0, 0);

		// the view gun rides the display row's mesh : its instance is appended
		// just before the row each frame, so the mesh keeps one contiguous run
		gun_mesh = mesh;
		uint clip = 0;
		renderer->animloader.load("assets/anim/testpistol.anim", &clip);
		gun_player = renderer->animloader.play(clip, GUN_IDLE, .5f);
		if (gun_player == (uint)-1 || !gun_mesh) // no weapon : the capsule still works
		{
			gun_mesh = 0;
			console->add_entry("Player : view model failed to load", FIXME, PHYS);
		}
		else renderer->bind_player(gun_mesh, gun_player);

		bullet_mesh = renderer->add_mesh(MeshFactory::generate_uv_sphere(.04f, 8, 6));

		sfx_fire[0] = audio.load("shot 1",  "assets/audio/pistol_shot_1.audio");
		sfx_fire[1] = audio.load("shot 2",  "assets/audio/pistol_shot_2.audio");
		sfx_fire[2] = audio.load("shot 3",  "assets/audio/pistol_shot_3.audio");
		sfx_reload  = audio.load("reload",  "assets/audio/arm.audio");
		sfx_inspect = audio.load("inspect", "assets/audio/lmg_tail.audio");
		sfx_hit     = audio.load("hit",     "assets/audio/hitmarker.audio");

		ammo   = GUN_MAG;
		action = GUN_IDLE;

		console->add_entry("Player spawned : F swaps to the fly camera", SUCCESS, PHYS);
	}

	// one frame : input -> movement -> camera -> weapon -> projectiles
	void update(GameWindow* window, GameRenderer* renderer, Physics* phys)
	{
		ImGuiIO& io = ImGui::GetIO();
		float dt = window->dtime;

		bool ours = window->focus_status == GAME;

		// F : swap between the capsule and the free camera
		if (ours && !io.WantCaptureKeyboard &&
			window->keys.F.is_pressed && !window->keys.F.was_pressed)
			flycam = !flycam;

		if (flycam) // the free camera drives itself, but rounds keep flying
		{
			update_bullets(phys, dt);
			return;
		}

		vec2 look = vec2(0);
		if (ours && !io.WantCaptureMouse)
		{
			look = vec2((float)window->mouse.dx, (float)window->mouse.dy);
			camera.update_dir(look.x, look.y, dt);
		}

		move(window, phys); // every frame : the eyes and the gun ride the capsule
		weapon(window, renderer, phys, io, look, dt);
		update_bullets(phys, dt);
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

	// action clock, input, and the juice layers
	void weapon(GameWindow* window, GameRenderer* renderer, Physics* phys, ImGuiIO& io, vec2 look, float dt)
	{
		// an action holds the gun until its time runs out
		if (action_time > 0)
		{
			action_time -= dt;
			if (action_time <= 0)
			{
				if (action == GUN_RELOAD) ammo = GUN_MAG;
				action = GUN_IDLE;
				action_time = 0;
				to_idle(renderer);
			}
		}

		bool ours = !flycam && window->focus_status == GAME &&
			!io.WantCaptureMouse && !io.WantCaptureKeyboard;

		if (ours && action == GUN_IDLE)
		{
			if (window->mouse.left_button.is_pressed) fire(renderer, phys);
			else if (window->keys.R.is_pressed && ammo < GUN_MAG) start_action(renderer, GUN_RELOAD, GUN_RELOAD_TIME);
			else if (window->keys.N.is_pressed) start_action(renderer, GUN_INSPECT, GUN_INSPECT_TIME);
		}

		// sway : the mouse kicks it, a spring drags it home
		sway_vel -= sway * 90.f * dt;   // spring home
		sway_vel -= sway_vel * 12.f * dt; // damping
		sway_vel += look * WEAPON_SWAY;   // mouse kick
		sway += sway_vel * dt;

		// recoil : the same spring in 3d
		kick_vel -= kick * 120.f * dt;
		kick_vel -= kick_vel * 14.f * dt;
		kick += kick_vel * dt;

		fov_punch *= expf(-9.f * dt);
		if (fov_punch < .001f) fov_punch = 0;
	}

	// cut to an action : the anim player crossfades from the pose on screen, and
	// the action's sound plays here so the tab buttons sound like the keys
	void start_action(GameRenderer* renderer, uint a, float time)
	{
		action = a;
		action_time = time;

		if (a == GUN_RELOAD) audio.play(sfx_reload);
		else if (a == GUN_INSPECT) audio.play(sfx_inspect);

		if (gun_player == (uint)-1) return;

		renderer->animloader.players[gun_player].control.loop = false; // play once
		renderer->animloader.blend_to(gun_player, a, time);
	}

	void to_idle(GameRenderer* renderer)
	{
		if (gun_player == (uint)-1) return;

		renderer->animloader.players[gun_player].control.loop = true;
		renderer->animloader.blend_to(gun_player, GUN_IDLE, .5f);
	}

	void fire(GameRenderer* renderer, Physics* phys)
	{
		start_action(renderer, GUN_SHOOT, GUN_SHOT_TIME);

		if (ammo == 0) return; // dry click, nothing leaves

		ammo--;
		fired++;
		audio.play(sfx_fire[shot_variant++ % 3]); // round robin : takes alternate

		// weight : muzzle jump + a harder kick that springs back
		fov_punch += 2.0f;
		kick_vel += vec3(-.03f, .8f, 2.4f);
		// muzzle : just past the capsule, low and to the right like a hand
		vec3 muzzle = camera.position + camera.front * .6f + camera.right * .12f - camera.up * .12f;
		spawn_bullet(muzzle, camera.front);
	}

	void spawn_bullet(vec3 pos, vec3 dir)
	{
		Bullet& b = bullets[fired % MAX_BULLETS]; // ring : the oldest slot gives way
		b.pos  = pos;
		b.vel  = dir * BULLET_SPEED;
		b.life = 2.f;
	}

	void update_bullets(Physics* phys, float dt)
	{
		for (uint i = 0; i < MAX_BULLETS; i++)
		{
			Bullet& b = bullets[i];
			if (b.life <= 0) continue;

			b.life -= dt;
			vec3 to = b.pos + b.vel * dt;

			Physics::Phys_Ray ray = phys->raycast(b.pos, to);
			if (ray.hit && ray.body_id != body) // never shoot yourself
			{
				b.life = 0;
				hits++;
				phys->apply_impulse(ray.body_id, normalize(b.vel) * BULLET_PUSH);
				if (phys->is_dynamic(ray.body_id)) audio.play(sfx_hit); // mark a solid hit
				continue;
			}

			b.pos = to;
		}
	}

	uint bullets_alive()
	{
		uint n = 0;
		for (uint i = 0; i < MAX_BULLETS; i++)
			if (bullets[i].life > 0) n++;
		return n;
	}
};
