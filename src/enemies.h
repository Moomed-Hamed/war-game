#include "player.h"

#define MAX_ENEMIES 10

struct Enemy
{
	uint type;
	float health, trauma;
	btRigidBody* body;
};

void init(Enemy* enemies, Physics* phys)
{
	for (uint i = 0; i < MAX_ENEMIES; i++)
	{
		enemies[i] = { 1, 100 };
		enemies[i].body = add_phys_sphere(phys, vec3(randfn() * 15.f, 3, randfn() * 15.f), .5).body;
	}
}
void update(Enemy* enemies, float dtime, Particle_Emitter* emitter, Camera* cam, Physics* phys)
{
	for (uint i = 0; i < MAX_ENEMIES; i++)
	{
		if (enemies[i].type == NULL) continue;

		enemies[i].trauma -= dtime;
		if (enemies[i].trauma < 0) enemies[i].trauma = 0;

		if (enemies[i].health < 0)
		{
			// spawn some xp or something
			vec3 pos;
			get_transform(enemies[i].body, &pos);
			emit_explosion(emitter, pos);
			phys->world->removeRigidBody(enemies[i].body);
			enemies[i] = {};
		}
	}
}

// rendering

struct Enemy_Renderer
{
	Mesh_Drawable enemies[MAX_ENEMIES];
	Mesh_Renderer mesh;
	Shader shader;
};

void init(Enemy_Renderer* renderer)
{
	const char* meshes[] = { "assets/meshes/basic/sphere.mesh" };
	init_mesh_drawable(&renderer->mesh, meshes, MAX_ENEMIES * sizeof(Mesh_Drawable));
	load(&(renderer->shader), "assets/shaders/transform/mesh.vert", "assets/shaders/mesh.frag");
}
void update(Enemy_Renderer* renderer, Enemy* enemies, Physics* phys)
{
	for (uint i = 0; i < MAX_ENEMIES; i++)
	{
		if (enemies[i].type > 0)
		{
			vec3 p; mat3 r;
			get_transform(enemies[i].body, &p, &r);
			renderer->enemies[i].position = p + (shake(enemies[i].trauma / 5.f) * .1f);
			renderer->enemies[i].color    = lerp(vec3(0, 1, 0), vec3(1, 0, 0), 1.f - (enemies[i].health / 100));
			renderer->enemies[i].scale    = vec3(1);
			renderer->enemies[i].rotation = r;
		} else { renderer->enemies[i] = {}; }
	}

	update(renderer->mesh, sizeof(renderer->enemies), (byte*)(&renderer->enemies));
}
void draw(Enemy_Renderer* renderer, mat4 proj_view)
{
	bind(renderer->shader);
	set_mat4(renderer->shader, "proj_view", proj_view);
	renderer->mesh.meshes[0].num_instances = MAX_ENEMIES;
	draw(renderer->mesh);
}