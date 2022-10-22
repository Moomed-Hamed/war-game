#include "enemies.h"

#define MAX_BULLETS 16

struct Bullet
{
	uint type;
	vec3 position, velocity;
	float time_elapsed;
};

void spawn(Bullet* bullets, vec3 position, vec3 velocity, uint type = 1)
{
	for (uint i = 0; i < MAX_BULLETS; i++)
	{
		if (bullets[i].type == NULL)
		{
			bullets[i] = { type, position, velocity };
			return;
		}
	}
}
void update(Bullet* bullets, float dtime, Physics* phys, Particle_Emitter* emitter, Enemy* enemies, uint* new_event)
{
	for (uint i = 0; i < MAX_BULLETS; i++)
	{
		if (bullets[i].type == NULL) continue;

		bullets[i].time_elapsed += dtime;
		if (bullets[i].time_elapsed > 5)
		{
			bullets[i] = {};
			continue;
		}

		bullets[i].position += bullets[i].velocity * dtime;

		vec3 dir = normalize(bullets[i].velocity) * 3.f;
		Raycast_Result res = phys_raycast_closest(phys, bullets[i].position, dir);

		if (res.hit_body == NULL) continue;

		bool handled = false;

		if (res.hit_body_type == PHYS_TERRAIN)
		{
			emit_sparks(emitter, res.hit_position, res.hit_normal, PARTICLE_DEBRIS);
			handled = true;
		}
		else if (res.hit_body_type == PHYS_SPHERE)
		{
			for (uint j = 0; j < MAX_ENEMIES; j++)
			{
				if (enemies[j].body == res.hit_body)
				{
					enemies[j].health -= 10;// bullets[i].damage;
					emit_sparks(emitter, res.hit_position, res.hit_normal, PARTICLE_BLOOD);
					*new_event = 1;
					handled = true;
				}
			}
		}

		if(!handled) emit_sparks(emitter, res.hit_position, res.hit_normal);
		apply_central_impulse(res.hit_body, normalize(bullets[i].velocity));
		bullets[i] = {};
	}
}

// rendering

struct Bullet_Renderer
{
	Mesh_Drawable bullets[MAX_BULLETS];
	Mesh_Renderer mesh;
	Shader shader;
};

void init(Bullet_Renderer* renderer)
{
	const char* meshes[] = { "assets/meshes/basic/ico.mesh" };
	load(&renderer->shader, "assets/shaders/transform/mesh.vert", "assets/shaders/mesh.frag");
	init_mesh_drawable(&renderer->mesh, meshes, MAX_BULLETS * sizeof(Mesh_Drawable));
}
void update(Bullet_Renderer* renderer, Bullet* bullets)
{
	for (uint i = 0; i < MAX_BULLETS; i++)
		renderer->bullets[i] = { vec3(1,1,0), vec3(.01), bullets[i].position, mat3(1) };

	update(renderer->mesh, MAX_BULLETS * sizeof(Mesh_Drawable), (byte*)(&renderer->bullets));
}
void draw(Bullet_Renderer* renderer, mat4 proj_view)
{
	bind(renderer->shader);
	set_mat4(renderer->shader, "proj_view", proj_view);
	renderer->mesh.meshes[0].num_instances = MAX_BULLETS;
	draw(renderer->mesh);
}

// -- weapons -- //

enum {
	ACTION_IDLE = 0, // also ACTION_EQUIP
	ACTION_SHOOT,
	ACTION_RELOAD,
	ACTION_INSPECT,
	ACTION_ADS,
	ACTION_ADS_SHOOT,
	ACTION_ADS_RELOAD
};
enum {
	GUN_GLOCK = 0,
	GUN_SHOTGUN,
	GUN_M4A1,
	GUN_AK47,
	
	NUM_GUNS
};

struct Gun_Meta
{
	struct Gun_Info {
		uint mag_size;
		float reload_time;
		float fire_time; // fire_rate = 1 / fire_time
		float fire_trauma;
		float inspect_time; // weapon inspect duration
		float damage;
		float equip_time;
	} info[NUM_GUNS];

	struct Gun_Audio {
		Audio shoot[6], empty, reload, foley;
	} audio[NUM_GUNS];
};

void init(Gun_Meta* meta)
{
	for (uint i = 0; i < NUM_GUNS; i++) // bugs++ if all guns have the same stats! (?)
	{
		meta->info[i].mag_size = 5;
		meta->info[i].equip_time = 1;
		meta->info[i].reload_time = 2;
		meta->info[i].fire_time = .25;
		meta->info[i].fire_trauma = .1;
		meta->info[i].inspect_time = 1.2;
		meta->info[i].damage = 10;
	}

	meta->info[GUN_GLOCK].fire_time = .5;
	meta->audio[GUN_GLOCK].shoot[0] = load_audio("assets/audio/pistol_shot_1.audio");

	meta->info[GUN_SHOTGUN].fire_time = .9f;

	meta->info[GUN_M4A1].equip_time = .85;
	meta->info[GUN_M4A1].fire_time = 1.f / 13;
	meta->info[GUN_M4A1].fire_trauma = .05;
	meta->info[GUN_M4A1].mag_size = 30;
	meta->info[GUN_M4A1].reload_time = 2.2;
	meta->audio[GUN_M4A1].shoot[0] = load_audio("assets/audio/pistol_shot_1.audio");
}

struct Gun
{
	uint type, action;

	float action_total_time;
	float action_time; // time left to complete current action

	uint ammo_count;
	vec3 look_direction;
};

void switch_gun(Gun* gun, Gun_Meta* meta, uint gun_id)
{
	if (gun_id >= NUM_GUNS) { out("SWITCH GUN ID ERROR"); stop; };

	gun->type = gun_id;
	gun->action = ACTION_IDLE;
	gun->action_total_time = gun->action_time = meta->info[gun_id].equip_time;
	gun->ammo_count = meta->info[gun_id].mag_size;
}
void update(Gun* gun, float dtime, Gun_Meta* meta, Bullet* bullets, Camera* cam, Mouse mouse, Keyboard keys)
{
	uint id = gun->type;
	gun->action_time -= dtime;

	if (gun->action_time > 0) // action is not finished
	{
		return; // do nothing if an action is ongoing (for now)
	} else {} // action complete

	switch (gun->action)
	{
	case ACTION_RELOAD: gun->ammo_count = meta->info[id].mag_size; break;
	}

	gun->action = NULL;
	gun->action_time = -1; // idle

	// these if statements all end in gotos
	if (mouse.left_button.is_pressed) goto shoot;
	if (keys.R.is_pressed && gun->ammo_count < meta->info[gun->type].mag_size) goto reload;
	if (keys.N.is_pressed) goto inspect;

	return;

shoot:
	{
		if (gun->ammo_count <= 0) goto reload;

		play_audio(meta->audio[id].shoot[0]);

		gun->action = ACTION_SHOOT;
		gun->action_time = gun->action_total_time = meta->info[gun->type].fire_time;
		cam->trauma += meta->info[gun->type].fire_trauma;
		gun->ammo_count--;

		spawn(bullets, cam->position + cam->front, cam->front * 100.f);

		return;
	}
reload:
	{
		gun->action = ACTION_RELOAD;
		gun->action_time = gun->action_total_time = meta->info[gun->type].reload_time;
		return;
	}
inspect:
	{
		gun->action = ACTION_INSPECT;
		gun->action_time = gun->action_total_time = meta->info[gun->type].inspect_time;
		return;
	}
}

// rendering

void update_pistol_anim(Gun gun, Animation* anim, mat4* current_pose)
{
	float duration = gun.action_total_time;
	float elapsed  = gun.action_total_time - gun.action_time;

	switch (gun.action)
	{
	case ACTION_SHOOT:
	{
		play_animation(anim, current_pose, elapsed, duration, 65, 72);
	} break;
	case ACTION_RELOAD:
	{
		play_animation(anim, current_pose, elapsed, duration, 0, 63);
	} break;
	case ACTION_INSPECT:
	{
		play_animation(anim, current_pose, elapsed, duration, 74, 104);
	} break;

	default: // ACTION_IDLE
	{
		if (gun.action_time > 0)
			play_animation(anim, current_pose, elapsed, duration, 74, 104);
		else
			play_animation(anim, current_pose, 1, 1, 0, 104);
	} break;
	}
}
void update_shotgun_anim(Gun gun, Animation* anim, mat4* current_pose)
{
	float duration = gun.action_total_time;
	float elapsed = gun.action_total_time - gun.action_time;

	switch (gun.action)
	{
	case ACTION_SHOOT:
	{
		play_animation(anim, current_pose, elapsed, duration, 0, 16);
	} break;
	case ACTION_RELOAD:
	{
		play_animation(anim, current_pose, elapsed, duration, 17, 77);
	} break;
	case ACTION_INSPECT:
	{
		play_animation(anim, current_pose, elapsed, duration, 18, 78);
	} break;

	default: // ACTION_IDLE
	{
		//if (gun.action_time > 0)
		//	play_animation(anim, current_pose, elapsed, duration, 74, 104);
		//else
			play_animation(anim, current_pose, 0, 1, 77, 104);
	} break;
	}
}
void update_ak_anim(Gun gun, Animation* anim, mat4* current_pose)
{
	float duration = gun.action_total_time;
	float elapsed = gun.action_total_time - gun.action_time;

	switch (gun.action)
	{
	case ACTION_SHOOT:
	{
		play_animation(anim, current_pose, elapsed, duration, 0, 88);
	} break;
	case ACTION_RELOAD:
	{
		play_animation(anim, current_pose, elapsed, duration, 0, 88);
	} break;
	case ACTION_INSPECT:
	{
		play_animation(anim, current_pose, elapsed, duration, 0, 88);
	} break;

	default: // ACTION_IDLE
	{
		if (gun.action_time > 0)
			play_animation(anim, current_pose, elapsed, duration, 55, 88);
		else
			play_animation(anim, current_pose, 0, 1, 88, 88);
	} break;
	}
}
void update_m4_anim(Gun gun, Animation* anim, mat4* current_pose)
{
	float duration = gun.action_total_time;
	float elapsed = gun.action_total_time - gun.action_time;

	switch (gun.action)
	{
	case ACTION_SHOOT:
	{
		play_animation(anim, current_pose, elapsed, duration, 100, 107);
	} break;
	case ACTION_RELOAD:
	{
		play_animation(anim, current_pose, elapsed, duration, 31, 100);
	} break;
	case ACTION_INSPECT:
	{
		play_animation(anim, current_pose, elapsed, duration, 0, 50);
	} break;

	default: // ACTION_IDLE
	{
		if (gun.action_time > 0)
			play_animation(anim, current_pose, elapsed, duration, 0, 31);
		else
			play_animation(anim, current_pose, 1, 1, 0, 103);
	} break;
	}
}

struct Gun_Drawable
{
	vec3 position;
	mat3 rotation;
	mat3 local_rotation;
};

struct Gun_Renderer
{
	uint id; // this prolly shouldn't be here

	Drawable_Mesh_Anim_UV meshes[NUM_GUNS];
	Animation animations[NUM_GUNS];
	mat4 current_pose[MAX_ANIM_BONES];

	Shader shader;
	GLuint texture, material;
};

void init(Gun_Renderer* renderer)
{
	const char names[NUM_GUNS][16] = { "glock"  , "shotgun", "m4" , "ak" };

	load(&renderer->shader, "assets/shaders/transform/mesh_anim_uv.vert", "assets/shaders/mesh_uv.frag");
	renderer->texture  = load_texture_bmp("assets/textures/heist.bmp");
	renderer->material = load_texture_bmp("assets/textures/materials.bmp");
	glUniformBlockBinding(renderer->shader.id, 1, 0); // uniform 1 reads from UBO binding 0

	for (uint i = 0; i < NUM_GUNS; i++)
	{
		char mesh_path[128] = {}, anim_path[128] = {};
		snprintf(mesh_path, 128, "assets/meshes/weps/%s.mesh_anim", names[i]);
		snprintf(anim_path, 128, "assets/meshes/weps/%s.anim"     , names[i]);

		load(renderer->meshes + i, mesh_path, sizeof(Gun_Drawable));
		mesh_add_attrib_vec3(5, sizeof(Gun_Drawable), 0); // world pos
		mesh_add_attrib_mat3(6, sizeof(Gun_Drawable), sizeof(vec3)); // rotation
		mesh_add_attrib_mat3(9, sizeof(Gun_Drawable), sizeof(vec3) + sizeof(mat3)); // local rotation

		load(renderer->animations + i, anim_path);
	}
}
void update(Gun_Renderer* renderer, Gun gun, float dtime, Camera cam, float turn)
{
	vec3 f = cam.front;
	vec3 r = cam.right;
	vec3 u = cam.up;

	float max_turn =  25;
	float min_turn = -25;

	// weapon sway
	static float turn_amount = 0;
	turn_amount *= 1.f - dtime;
	if (turn_amount < 0) turn_amount = 0;
	turn_amount += turn * max_turn;
	turn_amount = turn_amount > max_turn ? max_turn : turn_amount;
	turn_amount = turn_amount < min_turn ? min_turn : turn_amount;
	vec3 look = lerp(f, r, turn_amount);

	Animation* anim = renderer->animations + gun.type;
	mat4* pose = renderer->current_pose;

	switch (gun.type)
	{
	case GUN_GLOCK   : update_pistol_anim(gun, anim, pose); break;
	case GUN_SHOTGUN : update_shotgun_anim(gun, anim, pose); break;
	case GUN_M4A1    : update_m4_anim(gun, anim, pose); break;
	case GUN_AK47    : update_ak_anim(gun, anim, pose); break;
	default: out("ERROR : not an animated gun type"); stop;
	}

	//mat3 shake = glm::rotate(ToRadians(0), f);
	Gun_Drawable drawable = { cam.position, point_at(f, u), mat3(1)}; //shake };
	//drawable = { vec3(2), mat3(1), mat3(1) };

	static float time = 0; time += dtime * (float)TWOPI; // should we reset this?
	
	if (gun.action == ACTION_SHOOT)
	{
		float progress = gun.action_time / gun.action_total_time;
		drawable.position -= f * (.04f * sin(progress * (float)TWOPI));
	}
	else if (gun.action != ACTION_ADS) // weapon bob (for walking / running / breathing)
	{
		drawable.position += u * (.01f  * sin(time * 2.6f / (float)TWOPI));
		drawable.position += r * (.005f * sin(time * 1.6f / (float)TWOPI));
	}

	renderer->id = gun.type;
	update(renderer->meshes[gun.type], renderer->animations[gun.type].num_bones, renderer->current_pose, sizeof(Gun_Drawable), (byte*)(&drawable));
}
void draw(Gun_Renderer* renderer, mat4 proj_view)
{
	bind(renderer->shader);
	bind_texture(renderer->texture , 0);
	bind_texture(renderer->material, 1);
	set_mat4(renderer->shader, "proj_view", proj_view);
	draw(renderer->meshes[renderer->id]);
}

// crosshair rendering

struct C_Renderer
{
	enum {
		DOT_MESH,
		HITMARKER_MESH,

		NUM_MESHES
	};

	Mesh_Renderer mesh_renderer;
	Shader shader;
};

void init(C_Renderer* renderer)
{
	load(&renderer->shader, "assets/shaders/ui.vert", "assets/shaders/ui.frag");

	const char* paths[] = {
		"assets/meshes/ui/dot.mesh",
		"assets/meshes/ui/hitmarker.mesh"
	};

	init_mesh_drawable(&renderer->mesh_renderer, paths, sizeof(Mesh_Drawable), 2);
}
void update(C_Renderer* renderer, Window window, float dtime, uint new_event)
{
	for (uint i = 0; i < Prop_Renderer::NUM_MESHES; i++)
		renderer->mesh_renderer.meshes[i].num_instances = 0;

	quat rotation = quaternion(0, vec3(0, 1, 0));
	vec3 scale = vec3(.4);

	float event_duration = .05;
	vec3 neutral_color = vec3(.2);
	vec3 kill_color = vec3(.7, 0, 0);

	static uint current_event = 0;
	static float event_timer = 0; event_timer -= dtime;
	if (new_event && event_timer < 0)
	{
		event_timer = event_duration;
		current_event = new_event;
		//play_audio(renderer->hitmarker);
	} else if (event_timer < 0) current_event = 0;

	vec3 color = neutral_color;

	switch (current_event)
	{
	case 2: {
		renderer->mesh_renderer.meshes[C_Renderer::HITMARKER_MESH].num_instances = 1;
		color = kill_color;
	} break;
	case 1: {
		renderer->mesh_renderer.meshes[C_Renderer::HITMARKER_MESH].num_instances = 1;
		color = kill_color;
		//scale = vec3(.6);
		rotation = quaternion(ToRadians(randfns() * 10), vec3(1, 1, 1));
	} break;
	default: {
		renderer->mesh_renderer.meshes[C_Renderer::DOT_MESH].num_instances = 1;
		color = neutral_color;
	} break;
	}

	// finalizing

	float vertical_scale = (float)window.screen_width / window.screen_height;
	float horizontal_scale = 1;

	Mesh_Drawable drawable = {};
	drawable.color = color;
	drawable.position = vec3(0);
	drawable.scale    = scale * vec3(horizontal_scale, vertical_scale, 1);
	drawable.rotation = mat3(rotation);

	update(renderer->mesh_renderer, sizeof(Mesh_Drawable), (byte*)&drawable);
}
void draw(C_Renderer* renderer)
{
	bind(renderer->shader);
	//glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	glDisable(GL_CULL_FACE);
	draw(renderer->mesh_renderer);
	glEnable(GL_CULL_FACE);
	//glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
}