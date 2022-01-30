#include "weps.h"

#define MAX_ENEMIES 16

struct Enemy
{
	uint type;
	float health, trauma;
	Sphere_Collider collider;
};

void init(Enemy* enemies)
{
	for (uint i = 0; i < MAX_ENEMIES; i++)
	{
		vec3 position = vec3(randfn() * 15.f, 3, randfn() * 15.f);
		enemies[i] = { 1, 100, 0, { position, {}, {}, 1, .5 } };
	}
}
void update(Enemy* enemies, float dtime, Bullet* bullets, Particle_Emitter* emitter, Camera* cam)
{
	for (uint i = 0; i < MAX_ENEMIES; i++)
	{
		if (enemies[i].type == 0) continue;

		if (enemies[i].health > 0)
		{
			enemies[i].trauma -= dtime;
			if (enemies[i].trauma < 0) enemies[i].trauma = 0;

			for (uint j = 0; j < MAX_BULLETS; j++)
			{
				if (point_in_sphere(bullets[j].position, enemies[i].collider))
				{
					emit_blood(emitter, enemies[i].collider.position);
					enemies[i].health -= 10;
					enemies[i].trauma += .25;
					bullets[j] = {};
					break;
				}
			}
		}
		else
		{
			emit_explosion(emitter, enemies[i].collider.position);
			cam->trauma += .5;
			enemies[i] = {};
		}
	}
}

// rendering

struct Enemy_Drawable
{
	vec3 position;
	vec3 scale;
	vec3 color;
	mat3 transform;
};

struct Enemy_Renderer
{
	Enemy_Drawable enemies[MAX_ENEMIES];
	Drawable_Mesh mesh;
	GLuint texture, material;
	Shader shader;
};

void init(Enemy_Renderer* renderer)
{
	load(&renderer->mesh, "assets/meshes/basic/sphere.mesh", MAX_ENEMIES * sizeof(Enemy_Drawable));
	mesh_add_attrib_vec3(2, sizeof(Enemy_Drawable), 0 * sizeof(vec3)); // position
	mesh_add_attrib_vec3(3, sizeof(Enemy_Drawable), 1 * sizeof(vec3)); // scale
	mesh_add_attrib_vec3(4, sizeof(Enemy_Drawable), 2 * sizeof(vec3)); // color
	mesh_add_attrib_mat3(5, sizeof(Enemy_Drawable), 3 * sizeof(vec3)); // rotation

	load(&(renderer->shader), "assets/shaders/transform/mesh.vert", "assets/shaders/mesh.frag");
	renderer->texture  = load_texture("assets/textures/palette.bmp");
	renderer->material = load_texture("assets/textures/materials.bmp");
}
void update(Enemy_Renderer* renderer, Enemy* enemies)
{
	for (uint i = 0; i < MAX_ENEMIES; i++)
	{
		if (enemies[i].type > 0)
		{
			float trauma = enemies[i].trauma;
			uint offset = random_uint() % 16;
			float o1 = ((perlin((trauma + offset + 0) * 1000) * 2) - 1) * trauma;
			float o2 = ((perlin((trauma + offset + 1) * 2000) * 2) - 1) * trauma;
			float o3 = ((perlin((trauma + offset + 2) * 3000) * 2) - 1) * trauma;
			vec3 pos_offset = vec3(o1, o2, o3) * .05f;

			renderer->enemies[i].position  = enemies[i].collider.position + pos_offset;
			renderer->enemies[i].color     = lerp(vec3(0, 1, 0), vec3(1, 0, 0), 1.f - (enemies[i].health / 100));
			renderer->enemies[i].scale     = vec3(.75);
			renderer->enemies[i].transform = mat3(1);
		}
		else { renderer->enemies[i] = {}; }
	}

	update(renderer->mesh, sizeof(Enemy_Drawable) * MAX_ENEMIES, (byte*)(&renderer->enemies));
}
void draw(Enemy_Renderer* renderer, mat4 proj_view)
{
	bind(renderer->shader);
	set_mat4(renderer->shader, "proj_view", proj_view);
	draw(renderer->mesh, MAX_ENEMIES);
}