#include "renderer.h"

#define GRAVITY -9.80665f

#define MAX_CUBE_COLLIDERS      8
#define MAX_PLANE_COLLIDERS     8
#define MAX_SPHERE_COLLIDERS    8
#define MAX_CYLINDER_COLLIDERS  8

struct Cube_Collider // rigid body position = center of mass
{
	vec3 position, velocity, force;
	float mass;
	vec3 scale;
};

struct Sphere_Collider
{
	vec3 position, velocity, force;
	float mass;
	float radius;
};

struct Cylinder_Collider
{
	vec3 position, velocity, force;
	float mass;
	float radius;
	float height;
};

// AA = axis-aligned
struct Cube_Collider_AA
{
	vec3 position, velocity, force;
	vec3 min, max;
	float mass;
};

struct Cylinder_Collider_AA
{
	vec3 position, velocity, force;
	float radius;
	float height;
};

struct Plane_Collider // _AA? _Static?
{
	vec3 position, velocity, force;
	vec3 normal;
	vec2 scale;
};

// --- collisions --- //

bool point_in_cube_aa(vec3 point, Cube_Collider_AA cube)
{
	if (point.x < cube.min.x || point.x > cube.max.x) return false;
	if (point.z < cube.min.z || point.z > cube.max.z) return false;
	if (point.y < cube.min.y || point.y > cube.max.y) return false;
	return true;
}
bool point_in_sphere(vec3 point, Sphere_Collider sphere)
{
	float distance = (point.x - sphere.position.x) * (point.x - sphere.position.x) +
		              (point.y - sphere.position.y) * (point.y - sphere.position.y) +
		              (point.z - sphere.position.z) * (point.z - sphere.position.z);

	return (distance < (sphere.radius * sphere.radius));
}
bool point_in_cylinder(vec3 point, Cylinder_Collider cylinder)
{
	if (point.y > cylinder.position.y + (cylinder.height / 2.f)) return false;
	if (point.y < cylinder.position.y - (cylinder.height / 2.f)) return false;

	float distance = (point.x - cylinder.position.x) * (point.x - cylinder.position.x) +
						  (point.z - cylinder.position.z) * (point.z - cylinder.position.z);

	return (distance < (cylinder.radius * cylinder.radius));
}

bool sphere_cube_aa_intersect(Sphere_Collider sphere, Cube_Collider_AA cube)
{
	float x_pos, y_pos, z_pos;

	if (sphere.position.x < cube.min.x) x_pos = cube.min.x;
	else if (sphere.position.x > cube.max.x) x_pos = cube.max.x;
	else x_pos = sphere.position.x;

	if (sphere.position.y < cube.min.y) y_pos = cube.min.y;
	else if (sphere.position.y > cube.max.y) y_pos = cube.max.y;
	else y_pos = sphere.position.y;

	if (sphere.position.z < cube.min.z) z_pos = cube.min.z;
	else if (sphere.position.z > cube.max.z) z_pos = cube.max.z;
	else z_pos = sphere.position.z;

	return point_in_sphere(vec3(x_pos, y_pos, z_pos), sphere);
}
bool sphere_sphere_intersect(Sphere_Collider sphere_1, Sphere_Collider sphere_2)
{
	float distance = (sphere_1.position.x - sphere_2.position.x) * (sphere_1.position.x - sphere_2.position.x) +
		(sphere_1.position.y - sphere_2.position.y) * (sphere_1.position.y - sphere_2.position.y) +
		(sphere_1.position.z - sphere_2.position.z) * (sphere_1.position.z - sphere_2.position.z);

	float radius_sum = (sphere_1.radius * sphere_1.radius) + (sphere_2.radius * sphere_2.radius);

	return (distance < radius_sum);
}
bool sphere_plane_intersect(Sphere_Collider sphere, Plane_Collider plane)
{
	vec3 n = plane.normal;
	vec3 p = plane.position;
	vec3 s = sphere.position;

	float epsilon = (n.x * (s.x - p.x)) + (n.y * (s.y - p.y)) + (n.z * (s.z - p.z));
	if (epsilon > sphere.radius) return false;
	
	if (abs(dot(n, vec3(0, 1, 0))) - 1> 0.01)
	{
		mat3 undo_rotation = inverse(point_at(n, vec3(0, 1, 0))); // only works on vertical walls (fix 'up')

		// assert(n * undo_rotation == vec3(0, 1, 0);
		s = undo_rotation * s;
		p = undo_rotation * p;
	}

	if (s.x > (plane.scale.x /  2.f) + p.x) return false; // these don't take radius into account
	if (s.z > (plane.scale.y /  2.f) + p.z) return false; // these don't take radius into account
	if (s.x < (plane.scale.x / -2.f) + p.x) return false; // these don't take radius into account
	if (s.z < (plane.scale.y / -2.f) + p.z) return false; // these don't take radius into account

	return true;
}

// --- core --- //

void init_colldier(Cube_Collider* cube, vec3 position, vec3 velocity, vec3 force, float mass, vec3 scale)
{
	cube->position = position;
	cube->velocity = velocity;
	cube->force    = force;
	cube->mass     = mass;
	cube->scale    = scale;
}
void init_collider(Plane_Collider* plane, vec3 position, vec3 velocity, vec3 force, vec3 normal, vec2 scale)
{
	plane->position = position;
	plane->velocity = velocity;
	plane->force    = force;
	plane->normal   = normal;
	plane->scale    = scale;
}
void init_collider(Sphere_Collider* sphere, vec3 position, vec3 velocity, vec3 force, float mass, float radius)
{
	sphere->position = position;
	sphere->velocity = velocity;
	sphere->force    = force;
	sphere->mass     = mass;
	sphere->radius   = radius;
}
void init_collider(Cylinder_Collider* cylinder, vec3 position, vec3 velocity, vec3 force, float mass, float height, float radius)
{
	cylinder->position = position;
	cylinder->velocity = velocity;
	cylinder->force    = force;
	cylinder->mass     = mass;
	cylinder->height   = height;
	cylinder->radius   = radius;
}

struct Dynamic_Colliders
{
	Cube_Collider     cubes[MAX_CUBE_COLLIDERS];
	Sphere_Collider   spheres[MAX_SPHERE_COLLIDERS];
	Cylinder_Collider cylinders[MAX_CYLINDER_COLLIDERS];
};

struct Fixed_Colliders // static is a reserved keyword lol
{
	Cube_Collider  cubes[MAX_CUBE_COLLIDERS];
	Plane_Collider planes[MAX_PLANE_COLLIDERS];
};

struct Physics_Colliders
{
	Dynamic_Colliders dynamic;
	Fixed_Colliders   fixed;
};

// rendering

struct Collider_Drawable
{
	vec3 position;
	mat3 transform; // rotation & scale
};

struct Physics_Renderer
{
	uint num_planes, num_cubes, num_spheres, num_cylinders;

	Collider_Drawable planes   [MAX_PLANE_COLLIDERS];
	Collider_Drawable cubes    [MAX_CUBE_COLLIDERS     * 2]; // for dynamic & fixed
	Collider_Drawable spheres  [MAX_SPHERE_COLLIDERS   * 2]; // for dynamic & fixed
	Collider_Drawable cylinders[MAX_CYLINDER_COLLIDERS * 2]; // for dynamic & fixed

	Drawable_Mesh_UV cube_mesh, sphere_mesh, cylinder_mesh, plane_mesh;
	Shader shader;
};

void init(Physics_Renderer* renderer)
{
	uint reserved_size = sizeof(Collider_Drawable) * MAX_PLANE_COLLIDERS;

	load(&renderer->cube_mesh, "assets/meshes/basic/cube.mesh_uv", reserved_size);
	mesh_add_attrib_vec3(3, sizeof(Collider_Drawable), 0); // world pos
	mesh_add_attrib_mat3(4, sizeof(Collider_Drawable), sizeof(vec3)); // transform

	load(&renderer->sphere_mesh, "assets/meshes/basic/sphere.mesh_uv", reserved_size);
	mesh_add_attrib_vec3(3, sizeof(Collider_Drawable), 0); // world pos
	mesh_add_attrib_mat3(4, sizeof(Collider_Drawable), sizeof(vec3)); // transform

	load(&renderer->cylinder_mesh, "assets/meshes/basic/cylinder.mesh_uv", reserved_size);
	mesh_add_attrib_vec3(3, sizeof(Collider_Drawable), 0); // world pos
	mesh_add_attrib_mat3(4, sizeof(Collider_Drawable), sizeof(vec3)); // transform

	load(&renderer->plane_mesh, "assets/meshes/basic/plane.mesh_uv", reserved_size);
	mesh_add_attrib_vec3(3, sizeof(Collider_Drawable), 0); // world pos
	mesh_add_attrib_mat3(4, sizeof(Collider_Drawable), sizeof(vec3)); // transform

	GLuint texture  = load_texture("assets/textures/palette.bmp");
	GLuint material = load_texture("assets/textures/materials.bmp");

	renderer->cube_mesh.texture_id     = texture;
	renderer->sphere_mesh.texture_id   = texture;
	renderer->cylinder_mesh.texture_id = texture;
	renderer->plane_mesh.texture_id    = texture;

	renderer->cube_mesh.material_id     = material;
	renderer->sphere_mesh.material_id   = material;
	renderer->cylinder_mesh.material_id = material;
	renderer->plane_mesh.material_id    = material;

	load(&(renderer->shader), "assets/shaders/transform/mesh_uv.vert", "assets/shaders/mesh_uv.frag");
}
void update_renderer(Physics_Renderer* renderer, Physics_Colliders* colliders)
{
	renderer->num_cubes     = 0;
	renderer->num_planes    = 0;
	renderer->num_spheres   = 0;
	renderer->num_cylinders = 0;

	for (uint i = 0; i < MAX_CUBE_COLLIDERS; i++)
	{
		if (colliders->dynamic.cubes[i].scale.x > 0) // is there a better way to check?
		{
			renderer->num_cubes += 1;
			renderer->cubes[i].position = colliders->dynamic.cubes[i].position;
			renderer->cubes[i].transform = mat3(1.f);
		}
	}

	for (uint i = 0; i < MAX_PLANE_COLLIDERS; i++)
	{
		if (colliders->fixed.planes[i].scale.x > 0)
		{
			mat3 scale = mat3(1);
			scale[0][0] = colliders->fixed.planes[i].scale.x;
			scale[1][1] = colliders->fixed.planes[i].scale.y;

			renderer->num_planes += 1;
			renderer->planes[i].position = colliders->fixed.planes[i].position;
			renderer->planes[i].transform = point_at(colliders->fixed.planes[i].normal, vec3(1, 0, 0)) * scale; // ???
		}
	}

	for (uint i = 0; i < MAX_SPHERE_COLLIDERS; i++)
	{
		if (colliders->dynamic.spheres[i].radius > 0)
		{
			renderer->num_spheres += 1;
			renderer->spheres[i].position = colliders->dynamic.spheres[i].position;
			renderer->spheres[i].transform = mat3(1);
		}
	}

	for (uint i = 0; i < MAX_CYLINDER_COLLIDERS; i++)
	{
		if (colliders->dynamic.cylinders[i].radius > 0)
		{
			renderer->num_cylinders += 1;
			renderer->cylinders[i].position = colliders->dynamic.cylinders[i].position;
			renderer->cylinders[i].transform = mat3(1);
		}
	}

	update(renderer->cube_mesh, sizeof(Collider_Drawable) * renderer->num_cubes, (byte*)(&renderer->cubes));
	update(renderer->plane_mesh, sizeof(Collider_Drawable) * renderer->num_planes, (byte*)(&renderer->planes));
	update(renderer->sphere_mesh, sizeof(Collider_Drawable) * renderer->num_spheres, (byte*)(&renderer->spheres));
	update(renderer->cylinder_mesh, sizeof(Collider_Drawable) * renderer->num_cylinders, (byte*)(&renderer->cylinders));
}
void draw(Physics_Renderer* renderer, mat4 proj_view)
{
	bind(renderer->shader);
	set_mat4(renderer->shader, "proj_view", proj_view);

	bind_texture(renderer->cube_mesh    );
	bind_texture(renderer->plane_mesh   );
	bind_texture(renderer->sphere_mesh  );
	bind_texture(renderer->cylinder_mesh);

	//glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	//glDisable(GL_CULL_FACE);
	draw(renderer->cube_mesh    , renderer->num_cubes);
	draw(renderer->plane_mesh   , renderer->num_planes);
	draw(renderer->sphere_mesh  , renderer->num_spheres);
	draw(renderer->cylinder_mesh, renderer->num_cylinders);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	glEnable(GL_CULL_FACE);
}

// particle system

#define MAX_PARTICLES 128
#define MAX_PARTICLE_AGE 2

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
	vec3 position;
	vec3 velocity;
	float time_alive;
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
			particles[i].type = type;
			particles[i].position = pos;

			dir.x += radius * random_chance_signed() * dot(dir, vec3(1, 0, 0));
			dir.y += radius * random_chance_signed() * dot(dir, vec3(0, 1, 0));
			dir.z += radius * random_chance_signed() * dot(dir, vec3(0, 0, 1));

			particles[i].velocity = normalize(dir) * speed;
			particles[i].time_alive = 0;
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
			particles[i].type = type;
			particles[i].position = pos + (radius * vec3(random_chance_signed(), 0, random_chance_signed()));
			particles[i].velocity = vec3(0, speed, 0);
			particles[i].time_alive = 0;
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
			particles[i].type = type;
			particles[i].position = pos;
			particles[i].velocity = speed * vec3(random_chance_signed(), random_chance_signed(), random_chance_signed());
			particles[i].time_alive = 0;
			return;
		}
	}
}

void emit_explosion(Particle_Emitter* emitter, vec3 position)
{
	for (uint i = 0; i < 16; i++) emit_sphere(emitter, position, PARTICLE_FIRE  , 2);
	for (uint i = 0; i < 16; i++) emit_sphere(emitter, position, PARTICLE_SPARK , 4);
	for (uint i = 0; i < 16; i++) emit_sphere(emitter, position, PARTICLE_SMOKE , 1);
	for (uint i = 0; i <  8; i++) emit_sphere(emitter, position, PARTICLE_DEBRIS, 1);
}
void emit_fire(Particle_Emitter* emitter)
{
	uint num = (random_uint() % 3) + 1;
	for (uint i = 0; i < num; i++) { emit_circle(emitter, vec3(9.3, -.2, -6.5), PARTICLE_FIRE, .2); }
	//emit_circle(emitter, vec3(5, 1, 0), PARTICLE_SMOKE, .2);
}

void update(Particle_Emitter* emitter, float dtime, vec3 wind = vec3(0))
{
	Particle* particles = emitter->particles;

	for (uint i = 0; i < MAX_PARTICLES; i++)
	{
		if (particles[i].type != NULL && particles[i].time_alive < MAX_PARTICLE_AGE)
		{
			switch (particles[i].type)
			{
			case PARTICLE_SMOKE: // things that rise
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

			particles[i].position += particles[i].velocity * dtime;
			particles[i].position += wind * dtime;
			particles[i].time_alive += dtime;
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
	//debris
	load(&renderer->cube, "assets/meshes/basic/ico.mesh", MAX_PARTICLES * sizeof(Particle_Drawable));
	mesh_add_attrib_vec3(2, sizeof(Particle_Drawable), 0 * sizeof(vec3)); // position
	mesh_add_attrib_vec3(3, sizeof(Particle_Drawable), 1 * sizeof(vec3)); // scale
	mesh_add_attrib_vec3(4, sizeof(Particle_Drawable), 2 * sizeof(vec3)); // color
	mesh_add_attrib_mat3(5, sizeof(Particle_Drawable), 3 * sizeof(vec3)); // rotation

	load(&(renderer->shader), "assets/shaders/transform/mesh.vert", "assets/shaders/mesh.frag");
}
void update_renderer(Particle_Renderer* renderer, Particle_Emitter* emitter)
{
	for (uint i = 0; i < MAX_PARTICLES; i++)
	{
		float time = emitter->particles[i].time_alive;
		float completeness = time / MAX_PARTICLE_AGE;

		renderer->particles[i].position = emitter->particles[i].position;

		switch (emitter->particles[i].type)
		{
		case PARTICLE_DEBRIS:
		{
			renderer->particles[i].scale = vec3(.08f);
			renderer->particles[i].color = vec3(.2);
			mat3 rotation = rotate(noise_chance(.2 * (i + (completeness * 100))), vec3(noise_chance(i), noise_chance(i), noise_chance(i)));
			renderer->particles[i].transform = rotation;
		} break;
		case PARTICLE_FIRE:
		{
			renderer->particles[i].scale = vec3(lerp(.08, .02, completeness));
			renderer->particles[i].color = lerp(vec3(1,1,0), vec3(1,0,0), completeness * 1.5);
			mat3 rotation = rotate(noise_chance(.2 * (i + (completeness * 100))), vec3(noise_chance(i), noise_chance(i), noise_chance(i)));
			renderer->particles[i].transform = rotation;
		} break;
		case PARTICLE_SPARK:
		{
			renderer->particles[i].scale = vec3(lerp(.02, .02, completeness));
			renderer->particles[i].color = lerp(vec3(1), vec3(1, 1, 0), completeness * .5);
			mat3 rotation = rotate(noise_chance(.2 * (i + (completeness * 100))), vec3(noise_chance(i), noise_chance(i), noise_chance(i)));
			renderer->particles[i].transform = rotation;
		} break;
		case PARTICLE_SMOKE:
		{
			renderer->particles[i].scale = vec3(lerp(.2, .02, completeness));
			renderer->particles[i].color = lerp(vec3(1), vec3(0), completeness);
			mat3 rotation = rotate(noise_chance(.02 * (i + (completeness * 1000))), vec3(noise_chance(i), noise_chance(i), noise_chance(i)));
			renderer->particles[i].transform = rotation;
		} break;
		case PARTICLE_BLOOD:
		{
			renderer->particles[i].scale = normalize(emitter->particles[i].velocity) * .05f;
			renderer->particles[i].color = lerp(vec3(0.843, 0.015, 0.015), vec3(.1), completeness);
			mat3 rotation = rotate(noise_chance(.2 * (i + (completeness * 100))), vec3(noise_chance(i), noise_chance(i), noise_chance(i)));
			renderer->particles[i].transform = rotation;
		} break;
		default:
		{
			renderer->particles[i].scale = vec3(0);
			renderer->particles[i].color = vec3(1);
			renderer->particles[i].transform = mat3(1);
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