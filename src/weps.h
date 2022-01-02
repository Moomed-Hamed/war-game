#include "player.h"

#define MAX_BULLETS 16

struct Bullet
{
	uint type;
	vec3 position, velocity;
	float damage, time_alive;
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
			if (bullets[i].time_alive > 5) { bullets[i] = {}; continue; }
			bullets[i].time_alive += dtime;
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
		} else renderer->bullets[i] = {};
	}

	update(renderer->mesh, MAX_BULLETS * sizeof(Bullet_Drawable), (byte*)(&renderer->bullets));
}
void draw(Bullet_Renderer* renderer, mat4 proj_view)
{
	bind(renderer->shader);
	set_mat4(renderer->shader, "proj_view", proj_view);
	draw(renderer->mesh, MAX_BULLETS);
}

#define ACTION_FIRE	1
#define ACTION_RELOAD	2
#define ACTION_INSPECT	3

struct Gun
{
	vec3 position;
	vec3 look_direction;

	uint action;
	float action_time;

	Audio shoot;
};

void update(Gun* gun, Bullet* bullets, Camera* cam, Mouse mouse, Keyboard keys, float dtime)
{
	// gun
	gun->position = cam->position;

	if (gun->action_time < 0)
	{
		gun->action = NULL; // idle
		gun->action_time = -1;
	}
	else gun->action_time -= dtime;

	if (mouse.left_button.is_pressed && !mouse.left_button.was_pressed)
	{
		gun->action = ACTION_FIRE;
		gun->action_time = .1;
		cam->trauma = .25;

		spawn(bullets, cam->position, cam->front * 10.f);
		play_audio(gun->shoot);
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

void update_pistol_anim(Gun gun, Animation* anim, mat4* current_pose, float dtime);

struct Gun_Drawable
{
	vec3 position;
	mat3 rotation;
};

struct Gun_Renderer
{
	Drawable_Mesh_Anim_UV mesh;
	Shader shader;
	GLuint texture, material;
	Animation animation;
	mat4 current_pose[MAX_ANIM_BONES];
};

void init(Gun_Renderer* renderer)
{
	load(&(renderer->shader), "assets/shaders/transform/mesh_anim_uv.vert", "assets/shaders/mesh_uv.frag");
	renderer->texture  = load_texture("assets/textures/palette.bmp");
	renderer->material = load_texture("assets/textures/materials.bmp");

	load(&renderer->mesh, "assets/meshes/weps/pistol.mesh_anim", sizeof(Gun_Drawable));
	mesh_add_attrib_vec3(5, sizeof(Gun_Drawable), 0); // world pos
	mesh_add_attrib_mat3(6, sizeof(Gun_Drawable), sizeof(vec3)); // rotation

	load(&renderer->animation, "assets/animations/pistol.anim"); // animaiton keyframes
	GLint skeleton_id = glGetUniformBlockIndex(renderer->shader.id, "skeleton");
	glUniformBlockBinding(renderer->shader.id, skeleton_id, 0);
}
void update(Gun_Renderer* renderer, Gun& gun, float dtime, Camera cam, float turn)
{
	vec3 front = cam.front;
	vec3 right = cam.right;
	vec3 up    = cam.up;

	update_pistol_anim(gun, &renderer->animation, renderer->current_pose, dtime);

	static float turn_amount = 0; turn_amount += turn;
	if (turn_amount >  .1) turn_amount =  .1;
	if (turn_amount < -.1) turn_amount = -.1;
	vec3 look = lerp(front * -1.f, right, -1 * (-.05 + turn_amount));
	turn_amount *= dtime;

	Gun_Drawable drawable = {};
	drawable.position = gun.position + (front * 1.5f) + (up * -.25f) + (right * .4f);
	drawable.rotation = mat3(.25f) * point_at(look, up);

	static float time = 0; time += dtime;
	if (gun.action == NULL)
	{
		drawable.position += up    * (.01f  * sin(time * 2.6f));
		drawable.position += right * (.005f * sin(time * 1.6f));
	} else time = 0;

	update(renderer->mesh, renderer->animation.num_bones, renderer->current_pose, sizeof(Gun_Drawable), (byte*)(&drawable));
}
void draw(Gun_Renderer* renderer, mat4 proj_view)
{
	bind(renderer->shader);
	bind_texture(renderer->texture , 0);
	bind_texture(renderer->material, 1);
	set_mat4(renderer->shader, "proj_view", proj_view);
	draw(renderer->mesh);
}

// weapon animations
void update_pistol_anim(Gun gun, Animation* anim, mat4* current_pose, float dtime)
{
	switch (gun.action)
	{
	case ACTION_FIRE:
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