#include "terrain.h"

#define MAX_PARTICLES 512

#define PARTICLE_DEBRIS	1
#define PARTICLE_FIRE	2
#define PARTICLE_SMOKE	3
#define PARTICLE_BLOOD	4
#define PARTICLE_WATER	5
#define PARTICLE_SPARK	6
#define PARTICLE_STEAM	7
#define PARTICLE_BONES	8

struct Particle
{
	uint type;
	vec3 position, velocity;
	float time_alive, max_age;
};

struct Particle_Emitter
{
	Particle particles[MAX_PARTICLES];
};

void emit_cone(Particle_Emitter* emitter, vec3 pos, vec3 dir, uint type, float radius = 1, float speed = 1)
{
	Particle* particles = emitter->particles;
	dir = normalize(dir);

	for (uint i = 0; i < MAX_PARTICLES; i++) // this is broken
	{
		if (particles[i].type == NULL)
		{
			dir.x += radius * dot(dir, {1, 0, 0}) * randfns();
			dir.y += radius * dot(dir, {0, 1, 0}) * randfns();
			dir.z += radius * dot(dir, {0, 0, 1}) * randfns();

			particles[i] = { type, pos, normalize(dir) * speed , 0, 4 };
			return;
		}
	}
}
void emit_circle(Particle_Emitter* emitter, vec3 pos, uint type, float radius = .5, float speed = 1)
{
	Particle* particles = emitter->particles;

	for (uint i = 0; i < MAX_PARTICLES; i++)
	{
		if (particles[i].type == NULL)
		{
			particles[i] = { type, pos + (radius * vec3(randfn(), 0, randfn())), vec3(0, speed, 0) , 0, 4 };
			return;
		}
	}
}
void emit_sphere(Particle_Emitter* emitter, vec3 pos, uint type, float speed = 1)
{
	Particle* particles = emitter->particles;

	for (uint i = 0; i < MAX_PARTICLES; i++)
	{
		if (particles[i].type == NULL)
		{
			particles[i] = { type, pos, speed * randf3ns() , 0, 3 };
			return;
		}
	}
}

void emit_blood(Particle_Emitter* emitter, vec3 position)
{
	for (uint i = 0; i < 12; i++) emit_sphere(emitter, position, PARTICLE_BLOOD, 1);
}
void emit_explosion(Particle_Emitter* emitter, vec3 position)
{
	for (uint i = 0; i < 12; i++) emit_sphere(emitter, position, PARTICLE_FIRE  , 3);
	for (uint i = 0; i < 16; i++) emit_sphere(emitter, position, PARTICLE_SPARK , 6);
	for (uint i = 0; i < 10; i++) emit_sphere(emitter, position, PARTICLE_SMOKE , 1);
	for (uint i = 0; i <  8; i++) emit_sphere(emitter, position, PARTICLE_DEBRIS, 1);
}
void emit_fire(Particle_Emitter* emitter, vec3 pos)
{
	uint num = (random_uint() % 3) + 1;
	for (uint i = 0; i < num; i++) { emit_circle(emitter, pos, PARTICLE_FIRE, .3); }
}

void update(Particle_Emitter* emitter, Heightmap* map, float dtime, vec3 wind = vec3(0))
{
	Particle* particles = emitter->particles;

	for (uint i = 0; i < MAX_PARTICLES; i++)
	{
		if (particles[i].type != NULL && particles[i].time_alive < particles[i].max_age)
		{
			switch (particles[i].type)
			{
			case PARTICLE_SMOKE:
			{
				particles[i].velocity.y -= GRAVITY * .1 * dtime;
			} break;
			case PARTICLE_SPARK :
			{
				particles[i].velocity.y += GRAVITY * .3 * dtime;
			} break;
			case PARTICLE_DEBRIS:
			case PARTICLE_BLOOD :
			{
				particles[i].velocity.y += GRAVITY * .1 * dtime;
			} break;
			}

			// terrain collision
			if (particles[i].position.y < height(map, particles[i].position))
				particles[i].velocity.y = abs(particles[i].velocity.y) * .8;

			particles[i].position += (particles[i].velocity + wind) * dtime;
			particles[i].time_alive += dtime;

			if (particles[i].position.x < 0)particles[i].position.x = 0;
			if (particles[i].position.z < 0)particles[i].position.z = 0;
		}
		else particles[i] = {};
	}
}

// rendering

struct Particle_Drawable // don't forget about billboards
{
	vec3 position;
	vec3 scale;
	vec3 color;
	mat3 transform;
};

struct Particle_Renderer
{
	Particle_Drawable particles[MAX_PARTICLES];
	Drawable_Mesh cube, plane;
	Shader shader;
};

void init(Particle_Renderer* renderer)
{
	load(&renderer->cube, "assets/meshes/basic/ico.mesh", MAX_PARTICLES * sizeof(Particle_Drawable));
	mesh_add_attrib_vec3(2, sizeof(Particle_Drawable), 0 * sizeof(vec3)); // position
	mesh_add_attrib_vec3(3, sizeof(Particle_Drawable), 1 * sizeof(vec3)); // scale
	mesh_add_attrib_vec3(4, sizeof(Particle_Drawable), 2 * sizeof(vec3)); // color
	mesh_add_attrib_mat3(5, sizeof(Particle_Drawable), 3 * sizeof(vec3)); // rotation

	load(&(renderer->shader), "assets/shaders/transform/mesh.vert", "assets/shaders/mesh.frag");
}
void update(Particle_Renderer* renderer, Particle_Emitter* emitter)
{
	for (uint i = 0; i < MAX_PARTICLES; i++)
	{
		Particle particle = emitter->particles[i];
		Particle_Drawable* drawable = renderer->particles + i;

		float completeness = particle.time_alive / particle.max_age;

		drawable->position = particle.position;

		switch (particle.type)
		{
		case PARTICLE_DEBRIS:
		{
			mat3 rotation = rotate(completeness * 2 * TWOPI, randf3ns(noise(i), randfns(i), noise(i)));
			drawable->scale = vec3(.08f);
			drawable->color = vec3(.2);
			drawable->transform = rotation;
		} break;
		case PARTICLE_FIRE:
		{
			mat3 rotation = rotate(completeness * 2 * TWOPI, randf3ns(noise(i), randfns(i), noise(i)));
			drawable->scale = vec3(lerp(.08, .02, completeness));
			drawable->color = lerp(vec3(1,1,0), vec3(1,0,0), completeness * 1.5);
			drawable->transform = rotation;
		} break;
		case PARTICLE_SPARK:
		{
			mat3 rotation = rotate(completeness * 2 * TWOPI, randf3ns(noise(i), randfns(i), noise(i)));
			drawable->scale = vec3(lerp(.02, .02, completeness));
			drawable->color = lerp(vec3(1), vec3(1, 1, 0), completeness * .5);
			drawable->transform = rotation;
		} break;
		case PARTICLE_SMOKE:
		{
			mat3 rotation = rotate(completeness * TWOPI, randf3ns(noise(i), randfns(i), noise(i)));
			drawable->scale = vec3(lerp(.3, .02, completeness));
			drawable->color = lerp(vec3(1), vec3(0), completeness);
			drawable->transform = rotation;
		} break;
		case PARTICLE_BLOOD:
		{
			mat3 rotation = rotate(completeness * 2 * TWOPI, randf3ns(noise(i), randfns(i), noise(i)));
			drawable->scale = normalize(emitter->particles[i].velocity) * .05f;
			drawable->color = lerp(vec3(0.843, 0.015, 0.015), vec3(.1), completeness);
			drawable->transform = rotation;
		} break;
		default:
		{
			drawable->scale = vec3(0);
			drawable->color = vec3(1);
			drawable->transform = mat3(1);
		} break;
		}
	}

	update(renderer->cube, sizeof(Particle_Drawable) * MAX_PARTICLES, (byte*)(&renderer->particles));
}
void draw(Particle_Renderer* renderer, mat4 proj_view)
{
	bind(renderer->shader);
	set_mat4(renderer->shader, "proj_view", proj_view);
	draw(renderer->cube, MAX_PARTICLES);
}
