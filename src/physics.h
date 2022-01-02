#include "renderer.h"

#define GRAVITY -9.80665f
#define MAX_COLLIDERS 8 // max colliders of a single type

struct Cube_Collider // rigid body position = center of mass
{
	vec3 position, velocity, force;
	float mass;
	vec3 scale;
};

struct Sphere_Collider
{
	vec3 position, velocity, force;
	float mass, radius;
};

struct Cylinder_Collider
{
	vec3 position, velocity, force;
	float mass, radius, height;
};

struct Cube_Collider_AA // AA = axis-aligned
{
	vec3 position, velocity, force;
	vec3 min, max;
	float mass;
};

struct Cylinder_Collider_AA
{
	vec3 position, velocity, force;
	float radius, height;
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

struct Dynamic_Colliders
{
	Cube_Collider     cubes[MAX_COLLIDERS];
	Sphere_Collider   spheres[MAX_COLLIDERS];
	Cylinder_Collider cylinders[MAX_COLLIDERS];
};

struct Fixed_Colliders // static is a reserved keyword lol
{
	Cube_Collider  cubes[MAX_COLLIDERS];
	Plane_Collider planes[MAX_COLLIDERS];
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
	mat3 transform;
};

struct Physics_Renderer
{
	uint num_planes, num_cubes, num_spheres, num_cylinders;

	Collider_Drawable planes   [MAX_COLLIDERS];
	Collider_Drawable cubes    [MAX_COLLIDERS * 2]; // for dynamic & fixed
	Collider_Drawable spheres  [MAX_COLLIDERS * 2]; // for dynamic & fixed
	Collider_Drawable cylinders[MAX_COLLIDERS * 2]; // for dynamic & fixed

	Drawable_Mesh_UV cube_mesh, sphere_mesh, cylinder_mesh, plane_mesh;
	GLuint texture, material;
	Shader shader;
};

void init(Physics_Renderer* renderer)
{
	uint reserved_size = sizeof(Collider_Drawable) * MAX_COLLIDERS;

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

	load(&renderer->shader, "assets/shaders/transform/mesh_uv.vert", "assets/shaders/mesh_uv.frag");
	renderer->texture  = load_texture("assets/textures/palette.bmp");
	renderer->material = load_texture("assets/textures/materials.bmp");
}
void update(Physics_Renderer* renderer, Physics_Colliders* colliders)
{
	renderer->num_cubes     = 0;
	renderer->num_planes    = 0;
	renderer->num_spheres   = 0;
	renderer->num_cylinders = 0;

	for (uint i = 0; i < MAX_COLLIDERS; i++)
	{
		if (colliders->dynamic.cubes[i].scale.x > 0) // is there a better way to check?
		{
			renderer->num_cubes += 1;
			renderer->cubes[i].position = colliders->dynamic.cubes[i].position;
			renderer->cubes[i].transform = mat3(1.f);
		}

		if (colliders->fixed.planes[i].scale.x > 0)
		{
			mat3 scale = mat3(1);
			scale[0][0] = colliders->fixed.planes[i].scale.x;
			scale[1][1] = colliders->fixed.planes[i].scale.y;

			renderer->num_planes += 1;
			renderer->planes[i].position = colliders->fixed.planes[i].position;
			renderer->planes[i].transform = point_at(colliders->fixed.planes[i].normal, vec3(1, 0, 0)) * scale; // ???
		}

		if (colliders->dynamic.spheres[i].radius > 0)
		{
			renderer->num_spheres += 1;
			renderer->spheres[i].position = colliders->dynamic.spheres[i].position;
			renderer->spheres[i].transform = mat3(1);
		}

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

	glActiveTexture(GL_TEXTURE0); glBindTexture(GL_TEXTURE_2D, renderer->texture);
	glActiveTexture(GL_TEXTURE1); glBindTexture(GL_TEXTURE_2D, renderer->material);

	//glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	//glDisable(GL_CULL_FACE);
	draw(renderer->cube_mesh    , renderer->num_cubes);
	draw(renderer->plane_mesh   , renderer->num_planes);
	draw(renderer->sphere_mesh  , renderer->num_spheres);
	draw(renderer->cylinder_mesh, renderer->num_cylinders);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	glEnable(GL_CULL_FACE);
}