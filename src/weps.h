#include "player.h"

#define MAX_BULLETS 16

struct Bullet
{
	uint type;
	vec3 position, velocity;
	float damage, age; // age is in seconds
};

void spawn(Bullet* bullets, vec3 position, vec3 velocity, uint type = 1)
{
	for (uint i = 0; i < MAX_BULLETS; i++)
	{
		if (bullets[i].type == NULL)
		{
			bullets[i] = { type, position, velocity, 10 };
			return;
		}
	}
}
void update(Bullet* bullets, float dtime)
{
	for (uint i = 0; i < MAX_BULLETS; i++)
	{
		if (bullets[i].type)
		{
			if (bullets[i].age > 5) { bullets[i] = {}; continue; }
			bullets[i].age += dtime;
			bullets[i].position += bullets[i].velocity * dtime;
		}
	}
}

// rendering

struct Bullet_Drawable
{
	vec3 position;
	vec3 scale, color;
	mat3 transform;
};

struct Bullet_Renderer
{
	Bullet_Drawable bullets[MAX_BULLETS];
	Drawable_Mesh mesh;
	Shader shader;
};

void init(Bullet_Renderer* renderer)
{
	load(&renderer->shader, "assets/shaders/transform/mesh.vert", "assets/shaders/mesh.frag");
	load(&renderer->mesh, "assets/meshes/basic/sphere.mesh", MAX_BULLETS * sizeof(Bullet_Drawable));
	mesh_add_attrib_vec3(2, sizeof(Bullet_Drawable), 0 * sizeof(vec3)); // position
	mesh_add_attrib_vec3(3, sizeof(Bullet_Drawable), 1 * sizeof(vec3)); // scale
	mesh_add_attrib_vec3(4, sizeof(Bullet_Drawable), 2 * sizeof(vec3)); // color
	mesh_add_attrib_mat3(5, sizeof(Bullet_Drawable), 3 * sizeof(vec3)); // rotation
}
void update(Bullet_Renderer* renderer, Bullet* bullets)
{
	for (uint i = 0; i < MAX_BULLETS; i++)
	{
		if (bullets[i].type > 0)
		{
			renderer->bullets[i] = { bullets[i].position , vec3(.01) , vec3(1), mat3(1) };
		}
		else renderer->bullets[i] = {};
	}

	update(renderer->mesh, MAX_BULLETS * sizeof(Bullet_Drawable), (byte*)(&renderer->bullets));
}
void draw(Bullet_Renderer* renderer, mat4 proj_view)
{
	bind(renderer->shader);
	set_mat4(renderer->shader, "proj_view", proj_view);
	draw(renderer->mesh, MAX_BULLETS);
}

// -- weapons -- //

#define ACTION_SHOOT	1
#define ACTION_RELOAD	2
#define ACTION_INSPECT	3
#define ACTION_SWITCH	4

#define NUM_GUNS	15

#define GUN_US_RIFLE	0 // M1 Garand
#define GUN_GE_RIFLE	1 // bolt action
#define GUN_RU_RIFLE	2 // bolt action
#define GUN_UK_RIFLE	3 // bolt action

#define GUN_US_SMG	4
#define GUN_GE_SMG	5
#define GUN_RU_SMG	6

#define GUN_GE_AR	7

#define GUN_US_MG	8 // Bar
#define GUN_GE_MG	9

#define GUN_US_PISTOL	10 // M1911

#define GUN_SHOTGUN	11
#define GUN_MORTAR	12
#define GUN_RPG	13
#define GUN_FLAMETHROWER	14

struct Gun_Meta
{
	struct Gun_Info {
		uint mag_size;
		float reload_time;
		float fire_time;
		float damage;
	} info[NUM_GUNS];

	struct Gun_Audio {
		Audio shoot[6], empty, reload, foley;
	} audio[NUM_GUNS];
};

void init(Gun_Meta* meta)
{
	meta->info[GUN_US_RIFLE] = { 5, 1, .25, 10 }; // M1 Garand
	meta->audio[GUN_US_RIFLE].shoot[0] = load_audio("assets/audio/garand_shot.audio");
	meta->audio[GUN_US_RIFLE].shoot[1] = load_audio("assets/audio/garand_ping.audio");

	meta->info[GUN_UK_RIFLE] = { 5, 1, .25, 10 };
	meta->info[GUN_RU_RIFLE] = { 5, 1, .25, 10 };
	meta->info[GUN_GE_RIFLE] = { 5, 1, .25, 10 };

	meta->info[GUN_US_SMG] = { 5, 1, .25, 10 };
	meta->info[GUN_GE_SMG] = { 5, 1, .25, 10 };
	meta->info[GUN_RU_SMG] = { 5, 1, .25, 10 };

	meta->info[GUN_US_MG] = { 5, 1, .25, 10 };
	meta->info[GUN_GE_MG] = { 5, 1, .25, 10 };

	meta->info[GUN_US_PISTOL] = { 5, 1, .25, 10 };

	meta->info[GUN_RPG] = { 5, 1, .25, 10 };
	meta->info[GUN_MORTAR] = { 5, 1, .25, 10 };
	meta->info[GUN_FLAMETHROWER] = { 5, 1, .25, 10 };

	meta->info[GUN_US_PISTOL] = { 10, 1, .25, 5 };
	meta->audio[GUN_US_PISTOL].shoot[0] = load_audio("assets/audio/pistol_shot_1.audio");

	meta->audio[GUN_GE_RIFLE].shoot[0] = load_audio("assets/audio/gewehr_shot_1.audio");
	meta->audio[GUN_RU_RIFLE].shoot[0] = load_audio("assets/audio/gewehr_shot_2.audio");
	meta->audio[GUN_UK_RIFLE].shoot[0] = load_audio("assets/audio/gewehr_shot_3.audio");
}

struct Gun
{
	uint type, ammo_count;
	vec3 look_direction;

	uint action;
	float action_time;
};

void update(Gun* gun, Gun_Meta* meta, Bullet* bullets, Camera* cam, Mouse mouse, Keyboard keys, float dtime)
{
	uint id = gun->type;

	if (gun->action_time > 0)
	{
		gun->action_time -= dtime;
		return;
	} // else action has been completed

	switch (gun->action)
	{
	case ACTION_RELOAD: gun->ammo_count = meta->info[id].mag_size; break;
	}

	gun->action = NULL; // idle
	gun->action_time = -1;

	if (mouse.left_button.is_pressed && !mouse.left_button.was_pressed) goto shoot;
	if (keys.R.is_pressed && !keys.R.was_pressed) goto reload;
	if (keys.N.is_pressed && !keys.R.was_pressed) goto inspect;

	return;

shoot:
	{
		if (gun->ammo_count <= 0) goto reload;
		if (gun->ammo_count == 1 && gun->type == GUN_US_RIFLE) play_audio(meta->audio[id].shoot[1]);
		else play_audio(meta->audio[id].shoot[0]);

		gun->action = ACTION_SHOOT;
		gun->action_time = .1;
		cam->trauma += .4;
		gun->ammo_count--;

		spawn(bullets, cam->position, cam->front * 10.f);
		return;
	}
reload:
	{
		gun->action = ACTION_RELOAD;
		gun->action_time = 2;
		return;
	}
inspect:
	{
		gun->action = ACTION_INSPECT;
		gun->action_time = 3;
		return;
	}
}

// rendering

void update_pistol_anim(Gun gun, Animation* anim, mat4* current_pose);
void update_bolt_action_anim(Gun gun, Animation* anim, mat4* current_pose);
void update_smg_anim(Gun gun, Animation* anim, mat4* current_pose);
void update_us_rifle_anim(Gun gun, Animation* anim, mat4* current_pose);

struct Gun_Drawable
{
	vec3 position;
	mat3 rotation;
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
	const char names[NUM_GUNS][16] = {
		"us_rifle"  , "ge_rifle", "ru_rifle" , "uk_rifle",
		"ak"	      , "ge_smg"  , "ak" ,
		"ak"	      ,
		"us_mg"	   , "ak"	   ,
		"us_pistol" ,
		"shotgun"   ,
		"ak"	      , "m4"	   , "sniper"
	};

	load(&renderer->shader, "assets/shaders/transform/mesh_anim_uv.vert", "assets/shaders/mesh_uv.frag");
	renderer->texture  = load_texture("assets/textures/war.bmp");
	renderer->material = load_texture("assets/textures/materials.bmp");
	glUniformBlockBinding(renderer->shader.id, 1, 0); // uniform 1 reads from UBO binding 0

	for (uint i = 0; i < NUM_GUNS; i++)
	{
		char mesh_path[128] = {}, anim_path[128] = {};
		snprintf(mesh_path, 128, "assets/meshes/weps/%s.mesh_anim", names[i]);
		snprintf(anim_path, 128, "assets/animations/%s.anim", names[i]);

		load(renderer->meshes + i, mesh_path, sizeof(Gun_Drawable));
		mesh_add_attrib_vec3(5, sizeof(Gun_Drawable), 0); // world pos
		mesh_add_attrib_mat3(6, sizeof(Gun_Drawable), sizeof(vec3)); // rotation

		load(renderer->animations + i, anim_path);
	}
}
void update(Gun_Renderer* renderer, Gun gun, float dtime, Camera cam, float turn)
{
	Animation* anim = renderer->animations;
	mat4* pose = renderer->current_pose;

	switch (gun.type)
	{
	case GUN_US_RIFLE: update_us_rifle_anim(gun, anim + gun.type, pose); break;
	case GUN_UK_RIFLE:
	case GUN_RU_RIFLE:
	case GUN_GE_RIFLE: update_bolt_action_anim(gun, anim + gun.type, pose); break;

	case GUN_US_SMG:
	case GUN_GE_SMG:
	case GUN_RU_SMG: update_smg_anim(gun, anim + gun.type, pose); break;

	case GUN_US_MG:
	case GUN_GE_MG: update_pistol_anim(gun, anim + gun.type, pose); break;

	case GUN_US_PISTOL: update_pistol_anim(gun, anim + gun.type, pose); break;

	default: update_smg_anim(gun, anim + gun.type, pose); out("ERROR : not an animated gun type");
	}

	vec3 f = cam.front;
	vec3 r = cam.right;
	vec3 u = cam.up;

	// weapon sway
	static float turn_amount = 0;
	turn_amount = turn * 8 * dtime;
	turn_amount = turn_amount >  .1 ?  .1 : turn_amount;
	turn_amount = turn_amount < -.1 ? -.1 : turn_amount;
	vec3 look = lerp(-f, r, -1 * (-.05 + turn_amount)); // f is negative

	// offset & scaling (so first person perspective looks good)
	vec3 o = { 1, 1, 1 };
	float s = 1;

	switch (gun.type)
	{
	case GUN_US_RIFLE : o = { .65, -.45, .4 }; s = 1.0; break;
	case GUN_UK_RIFLE : o = { .65, -.45, .4 }; s = 1.0; break;
	case GUN_RU_RIFLE : o = { .65, -.25, .4 }; s = 1.0; break;
	case GUN_GE_RIFLE : o = { .65, -.40, .4 }; s = 1.0; break;

	case GUN_US_SMG : o = { .85, -.45, .4 }; s = 1.0; break;
	case GUN_GE_SMG : o = { .45, -.35, .3 }; s = 1.2; break;
	case GUN_RU_SMG : o = { .85, -.45, .4 }; s = 1.0; break;

	case GUN_US_MG : o = { .85, -.45, .4 }; s = 1.0; break;
	case GUN_GE_MG : o = { .85, -.45, .4 }; s = 1.0; break;

	case GUN_US_PISTOL : o = { 1.5, -.25, .4 }; s = .25; break;

	default : o = { .85, -.45, .4 }; s = 1.0; break;
	}

	Gun_Drawable drawable = { cam.position + (f * o.x) + (u * o.y) + (r * o.z) , mat3(s) * point_at(look, u) };

	// weapon bob (for walking / running / breathing)
	static float time = 0; time += dtime;
	if (gun.action == NULL)
	{
		drawable.position += u * (.01f  * sin(time * 2.6f));
		drawable.position += r * (.005f * sin(time * 1.6f));
	} else time = 0;

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

// weapon animations
uint sub_anim(vec4 t, float at, float* c)
{
	float x = 0;
	for (uint i = 0; i < 4; i++)
	{
		x += t[i];

		if (at < x)
		{
			*c = (at - (x - t[i])) / t[i];
			return i;
		}
	}

	*c = 1;
	return 4;
}
void update_pistol_anim(Gun gun, Animation* anim, mat4* current_pose)
{
	uvec2 frames = {}; // index of frames
	float mix = 0, c = 0; // completeness

	switch (gun.action)
	{
	case ACTION_SHOOT:
	{
		switch (sub_anim({ .05, .05, 0, 0}, .1 - gun.action_time, &c))
		{
		case 0: frames = { 0 , 1 }; mix = c; break; // slide back
		case 1: frames = { 1 , 0 }; mix = c; break; // slide fwd
		}
	} break;
	case ACTION_RELOAD:
	{
		switch (sub_anim({ .4, .3, .1, .4 }, 2 - gun.action_time, &c))
		{
		case 0: frames = { 0 , 2 }; mix = bounce(sqrt(c)); break;
		case 1: frames = { 2 , 2 }; mix = bounce(sqrt(c)); break;
		case 2: frames = { 2 , 3 }; mix = bounce(c); break;
		case 3: frames = { 3 , 0 }; mix = lerp_spring(c); break;
		}
	} break;
	case ACTION_INSPECT:
	{
		switch (sub_anim({ 1, 1, 1, 0}, 3 - gun.action_time, &c))
		{
		case 0: frames = { 0 , 6 }; mix = bounce(sqrt(c)); // inspect 1
		case 1: frames = { 6 , 7 }; mix = bounce(sqrt(c)); // inspect 2
		case 2: frames = { 7 , 0 }; mix = bounce(c); break; // idle
		}
	} break;
	}

	update_animation_pose(anim, current_pose, frames.x, frames.y, mix);
}
void update_bolt_action_anim(Gun gun, Animation* anim, mat4* current_pose)
{
	uvec2 frames = {}; // index of frames
	float mix = 0, c = 0; // completeness

	switch (gun.action)
	{
	case ACTION_SHOOT:
	{
		switch (sub_anim({ .05, .05, 5, 0}, .1 - gun.action_time, &c))
		{
		case 0: frames = { 0 , 1 }; mix = c; break; // shoot
		case 1: frames = { 1 , 0 }; mix = c; break; // idle
		}
	} break;
	case ACTION_RELOAD:
	{
		switch (sub_anim({ .25, .5, .5, .1 }, 2 - gun.action_time, &c))
		{
		case 0: frames = { 0 , 1 }; mix = bounce(c); break; // bolt up
		case 1: frames = { 1 , 2 }; mix = bounce(c); break; // bolt back
		case 2: frames = { 2 , 1 }; mix = bounce(c); break; // bolt up
		case 3: frames = { 1 , 0 }; mix = lerp_spring(c); break; // idle
		}
	} break;
	case ACTION_INSPECT:
	{
		switch (sub_anim({ 1, 1, 1, 0}, 3 - gun.action_time, &c))
		{
		case 0: frames = { 0 , 6 }; mix = bounce(c); // inspect 1
		case 1: frames = { 6 , 7 }; mix = bounce(c); // inspect 2
		case 2: frames = { 7 , 0 }; mix = bounce(c); break; // idle
		}
	} break;
	}

	update_animation_pose(anim, current_pose, frames.x, frames.y, mix);
}
void update_smg_anim(Gun gun, Animation* anim, mat4* current_pose)
{
	uvec2 frames = {}; // index of frames
	float mix = 0, c = 0; // completeness

	switch (gun.action)
	{
	case ACTION_SHOOT:
	{
		switch (sub_anim({ .05, .05, 0, 0 }, .1 - gun.action_time, &c))
		{
		case 0: frames = { 0 , 1 }; mix = c; break; // slide back
		case 1: frames = { 1 , 0 }; mix = c; break; // slide fwd
		}
	} break;
	case ACTION_RELOAD:
	{
		switch (sub_anim({ .6, .6, .5, .3 }, 2 - gun.action_time, &c))
		{
		case 0: frames = { 0 , 2 }; mix = lerp_spring(c, 5); break;
		case 1: frames = { 2 , 3 }; mix = c * c; break;
		case 2: frames = { 3 , 4 }; mix = c * c; break;
		case 3: frames = { 4 , 0 }; mix = bounce(c); break;
		}
	} break;
	case ACTION_INSPECT:
	{
		switch (sub_anim({ 1, 1, 1, 0 }, 3 - gun.action_time, &c))
		{
		case 0: frames = { 0 , 6 }; mix = c; // inspect 1
		case 1: frames = { 6 , 7 }; mix = c; // inspect 2
		case 2: frames = { 7 , 0 }; mix = bounce(c); break; // idle
		}
	} break;
	}

	update_animation_pose(anim, current_pose, frames.x, frames.y, mix);
}
void update_us_rifle_anim(Gun gun, Animation* anim, mat4* current_pose)
{
	uvec2 frames = { 1, 1 }; // index of frames
	float mix = 0, c = 0; // completeness

	switch (gun.action)
	{
	case ACTION_SHOOT:
	{
		switch (sub_anim({ .05, .05, 0, 0 }, .1 - gun.action_time, &c))
		{
		case 0: frames = { 1 , 2 }; mix = c; break; // slide back
		case 1: frames = { 2 , 1 }; mix = c; break; // slide fwd
		}
	} break;
	case ACTION_RELOAD:
	{
		switch (sub_anim({ .6, .6, .5, .3 }, 2 - gun.action_time, &c))
		{
		case 0: frames = { 1 , 2 }; mix = bounce(c); break;
		case 1: frames = { 2 , 3 }; mix = c * c; break;
		case 2: frames = { 3 , 2 }; mix = c * c; break;
		case 3: frames = { 2 , 1 }; mix = bounce(c); break;
		}
	} break;
	case ACTION_INSPECT:
	{
		switch (sub_anim({ 1, 1, 1, 0 }, 3 - gun.action_time, &c))
		{
		case 0: frames = { 0 , 0 }; mix = c; // inspect 1
		case 1: frames = { 0 , 0 }; mix = c; // inspect 2
		case 2: frames = { 0 , 0 }; mix = bounce(c); break; // idle
		}
	} break;
	}

	update_animation_pose(anim, current_pose, frames.x, frames.y, mix);
}

// crosshair rendering

struct Quad_Drawable
{
	vec2 position;
	vec2 scale;
	vec3 color;
};

struct Crosshair_Renderer
{
	Quad_Drawable quads[4];
	Drawable_Mesh_2D mesh;
	Shader shader;
};

void init(Crosshair_Renderer* renderer)
{
	init(&renderer->mesh, 4 * sizeof(Quad_Drawable));
	mesh_add_attrib_vec2(1, sizeof(Quad_Drawable), 0); // position
	mesh_add_attrib_vec2(2, sizeof(Quad_Drawable), sizeof(vec2)); // scale
	mesh_add_attrib_vec3(3, sizeof(Quad_Drawable), sizeof(vec2) + sizeof(vec2)); // color

	load(&renderer->shader, "assets/shaders/mesh_2D.vert", "assets/shaders/mesh_2D.frag");

	vec2 position = vec2(0);
	vec2 scale    = vec2(.003, .005) / 2.f;
	vec3 color    = vec3(1);

	renderer->quads[0] = { position + vec2(.006f, 0), scale, color };
	renderer->quads[1] = { position - vec2(.006f, 0), scale, color };
	renderer->quads[2] = { position + vec2(.0, .01f), scale, color };
	renderer->quads[3] = { position - vec2(.0, .01f), scale, color };
}
void update(Crosshair_Renderer* renderer)
{
	update(renderer->mesh, 4 * sizeof(Quad_Drawable), (byte*)renderer->quads);
}
void draw(Crosshair_Renderer* renderer)
{
	bind(renderer->shader);
	draw(renderer->mesh, 4);
}