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

#define NUM_GUNS	14

#define GUN_RIFLE	1
#define GUN_SMG	4
#define GUN_AR	5
#define GUN_LMG	3
#define GUN_PISTOL	4
#define GUN_RPG	5

#define GUN_US_RIFLE	1 // M1 Garand
#define GUN_GE_RIFLE	2
#define GUN_RU_RIFLE	3

#define GUN_US_SMG	4
#define GUN_GE_SMG	5
#define GUN_RU_SMG	6

#define GUN_US_AR	7
#define GUN_GE_AR	8
#define GUN_RU_AR	9

#define GUN_US_MG	10 // Bar
#define GUN_GE_MG	11

#define GUN_US_PISTOL	12 // M1911
#define GUN_GE_PISTOL	13
#define GUN_RU_PISTOL	14

struct Gun_Meta
{
	struct Gun_Audio {
		Audio shoot[2], empty, reload, foley;
	} audio[NUM_GUNS];

	struct Gun_Info {
		uint mag_size;
		float reload_time;
		float fire_time;
		float damage;
	} info[NUM_GUNS];
};

void init(Gun_Meta* meta)
{
	meta->audio[GUN_US_RIFLE].shoot[0] = load_audio("assets/audio/garand_shot.audio");
	meta->audio[GUN_US_RIFLE].shoot[1] = load_audio("assets/audio/garand_ping.audio");
	meta->info[GUN_US_RIFLE] = { 5, 1, .25, 10 };

	meta->audio[GUN_US_PISTOL].shoot[0] = load_audio("assets/audio/pistol_shot.audio");
	meta->audio[GUN_US_PISTOL].shoot[1] = load_audio("assets/audio/pistol_shot.audio"); // duplicate
	meta->info[GUN_US_PISTOL] = { 10, 1, .25, 5 };
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

	if (gun->action_time < 0)
	{
		switch (gun->action)
		{
		case ACTION_RELOAD: gun->ammo_count = meta->info[id].mag_size; break;
		}

		gun->action = NULL; // idle
		gun->action_time = -1;
	} else gun->action_time -= dtime;

	if (mouse.left_button.is_pressed && !mouse.left_button.was_pressed)
	{
		if (gun->ammo_count > 0) // shoot
		{
			gun->action = ACTION_SHOOT;
			gun->action_time = .1;
			cam->trauma = .4;

			if (gun->ammo_count == 1) play_audio(meta->audio[id].shoot[1]);
			else play_audio(meta->audio[id].shoot[0]);

			spawn(bullets, cam->position, cam->front * 10.f);
			gun->ammo_count -= 1;
		}
	}
	if (keys.R.is_pressed && !keys.R.was_pressed)
	{
		gun->action = ACTION_RELOAD;
		gun->action_time = 2;
	}
	if (keys.N.is_pressed && !keys.R.was_pressed)
	{
		gun->action = ACTION_INSPECT;
		gun->action_time = 3;
	}
}

// rendering

void update_us_pistol_anim(Gun gun, Animation* anim, mat4* current_pose, float dtime);

struct Gun_Drawable
{
	vec3 position;
	mat3 rotation;
};

struct Gun_Renderer
{
	Drawable_Mesh_Anim_UV meshes[NUM_GUNS];
	Animation animations[NUM_GUNS];

	Shader shader;
	GLuint texture, material;

	uint id; // this prolly shouldn't be here

	mat4 current_pose[MAX_ANIM_BONES];
};

void init(Gun_Renderer* renderer)
{
	load(&renderer->shader, "assets/shaders/transform/mesh_anim_uv.vert", "assets/shaders/mesh_uv.frag");
	renderer->texture = load_texture("assets/textures/palette.bmp");
	renderer->material = load_texture("assets/textures/materials.bmp");
	glUniformBlockBinding(renderer->shader.id, 1, 0); // uniform 1 reads from UBO binding 0

	{
		load(renderer->meshes + GUN_US_MG, "assets/meshes/weps/us_mg.mesh_anim", sizeof(Gun_Drawable));
		mesh_add_attrib_vec3(5, sizeof(Gun_Drawable), 0); // world pos
		mesh_add_attrib_mat3(6, sizeof(Gun_Drawable), sizeof(vec3)); // rotation

		load(renderer->meshes + GUN_US_PISTOL, "assets/meshes/weps/pistol.mesh_anim", sizeof(Gun_Drawable));
		mesh_add_attrib_vec3(5, sizeof(Gun_Drawable), 0); // world pos
		mesh_add_attrib_mat3(6, sizeof(Gun_Drawable), sizeof(vec3)); // rotation

		load(renderer->meshes + GUN_US_RIFLE, "assets/meshes/weps/us_rifle.mesh_anim", sizeof(Gun_Drawable));
		mesh_add_attrib_vec3(5, sizeof(Gun_Drawable), 0); // world pos
		mesh_add_attrib_mat3(6, sizeof(Gun_Drawable), sizeof(vec3)); // rotation

		 // animaiton keyframes
		load(renderer->animations + GUN_US_MG, "assets/animations/us_mg.anim");
		load(renderer->animations + GUN_US_PISTOL, "assets/animations/pistol.anim");
		load(renderer->animations + GUN_US_RIFLE, "assets/animations/us_rifle.anim");
	}
}
void update(Gun_Renderer* renderer, Gun gun, float dtime, Camera cam, float turn)
{
	update_us_pistol_anim(gun, renderer->animations + gun.type, renderer->current_pose, dtime);

	vec3 f = cam.front;
	vec3 r = cam.right;
	vec3 u = cam.up;

	// weapon sway
	static float turn_amount = 0;
	turn_amount += turn;
	turn_amount = dtime * ( turn_amount > .1 ? .1 : -.1 );
	vec3 look = lerp(-f, r, -1 * (-.05 + turn_amount)); // f is negative

	// offset & scaling (so first person perspective looks good)
	vec3 o = { 1, 1, 1 };
	float s = 1;

	switch (gun.type)
	{
	case GUN_US_PISTOL : o = { 1.5, -.25, .4 }; s = .25; break;
	case GUN_US_MG    : o = { .85, -.45, .4 }; s = 1.0; break;
	case GUN_US_RIFLE: o = { .65, -.45, .4 }; s = 1.0; break;
	}

	Gun_Drawable drawable = { cam.position + (f * o.x) + (u * o.y) + (r * o.z) , mat3(s) * point_at(look, u) };

	// weapon bob (for walking / running / breathing)
	static float time = 0; time += dtime;
	if (gun.action == NULL)
	{
		drawable.position += u * (.01f  * sin(time * 2.6f));
		drawable.position += r * (.005f * sin(time * 1.6f));
	} else time = 0;

	// new
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
void update_us_pistol_anim(Gun gun, Animation* anim, mat4* current_pose, float dtime)
{
	switch (gun.action)
	{
	case ACTION_SHOOT:
	{
		float completeness = (.1 - gun.action_time) / .1;

		if (completeness < .5)
		{
			float mix = completeness * 2;
			update_animation_pose(anim, current_pose, 0, 1, mix); // slide back
		}
		else
		{
			float mix = (completeness - .5) * 2;
			update_animation_pose(anim, current_pose, 1, 0, mix); // slide fwd
		}
	} break;
	case ACTION_RELOAD:
	{
		float completeness = (2 - gun.action_time) / 2;

		if (completeness < .4) // mag out
		{
			float mix = bounce(sqrt(completeness * (1 / .4)), -.2);
			update_animation_pose(anim, current_pose, 0, 2, mix);
		}
		else if (completeness < .7) // pause
		{
			float mix = bounce(sqrt((completeness - .4) * (1 / .3)));
			update_animation_pose(anim, current_pose, 2, 2, 1);
		}
		else if (completeness < .8) // mag in
		{
			float mix = bounce((completeness - .7) * (1 / .1));
			update_animation_pose(anim, current_pose, 2, 3, mix);
		}
		else // rest
		{
			float mix = lerp_spring(0, 1, (completeness - .8) * (1 / .2));
			update_animation_pose(anim, current_pose, 3, 0, mix);
		}
	} break;
	case ACTION_INSPECT:
	{
		float completeness = (3 - gun.action_time) / 3;

		if (completeness < .5) //inspect 1
		{
			float mix = bounce(sqrt(completeness * 2));
			update_animation_pose(anim, current_pose, 0, 6, mix);
		}
		else if (completeness < .9) //inspect 1
		{
			float mix = bounce(sqrt((completeness - .5) * (1 / .4)));
			update_animation_pose(anim, current_pose, 6, 7, mix);
		}
		else // rest
		{
			float mix = bounce((completeness - .9) * (1 / .1));
			update_animation_pose(anim, current_pose, 7, 0, mix);
		}
	} break;
	default: // ACTION_IDLE
	{
		update_animation_pose(anim, current_pose, 0, 0, 1);
	} break;
	}
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