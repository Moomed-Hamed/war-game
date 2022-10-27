#include "particles.h"

#define MAX_PROPS 5 // max number of props of a given type
#define NUM_PROP_TYPES 6
#define MAX_PROPS_TOTAL (MAX_PROPS * NUM_PROP_TYPES)

struct Prop
{
	vec3 position;
	mat3 rotation;
	uint body_index; // index of this prop's rigid_body
};

struct Props
{
	Prop props[MAX_PROPS * NUM_PROP_TYPES];
};

void init(Props* props, Physics* phys, Heightmap* map)
{
	props->props[0].body_index = add_phys_cube(phys, vec3(7,8,7), vec3(.134, .052, .334), 1);
	props->props[1].body_index = add_phys_cube(phys, vec3(7), vec3(1), 10);
	props->props[2] = { terrain(map, vec2(9 , 6)) , mat3(1) };
}
void update(Props* props, Physics* phys)
{
	uint i = props->props[0].body_index;
	props->props[0].position = phys->cubes[i].position;
	props->props[0].rotation = phys->cubes[i].rotation;

	i = props->props[1].body_index;
	props->props[1].position = phys->cubes[i].position;
	props->props[1].rotation = phys->cubes[i].rotation;
}

// rendering

struct Prop_Drawable
{
	vec3 position;
	mat3 transform;
};

struct Prop_Renderer
{
	enum {
		GOLD_BAR_MESH,
		CRATE_MESH   ,
		CAMPFIRE_MESH,

		NUM_MESHES
	};

	Prop_Drawable props[MAX_PROPS_TOTAL];
	Mesh_Renderer_UV meshes;
	GLuint texture, material;
	Shader shader;
};

void init_prop_drawable(Mesh_Renderer_UV* mesh, const char** paths, uint size = 0, uint num_meshes = 1)
{
	load(mesh, paths, size, num_meshes);
	mesh_add_attrib_vec3(3, sizeof(Prop_Drawable), 0 * sizeof(vec3)); // position
	mesh_add_attrib_mat3(4, sizeof(Prop_Drawable), 1 * sizeof(vec3)); // rotation
}

void init(Prop_Renderer* renderer)
{
	load(&renderer->shader, "assets/shaders/transform/mesh_uv.vert", "assets/shaders/mesh_uv.frag");
	renderer->texture  = load_texture_bmp("assets/textures/war.bmp");
	renderer->material = load_texture_bmp("assets/textures/materials.bmp");

	const char* meshes[] = {
		"assets/meshes/props/gold_bar.mesh_uv",
		"assets/meshes/props/crate.mesh_uv"   ,
		"assets/meshes/props/campfire.mesh_uv"
	};

	init_prop_drawable(&renderer->meshes, meshes, MAX_PROPS_TOTAL * sizeof(Prop_Drawable), Prop_Renderer::NUM_MESHES);
}
void update(Prop_Renderer* renderer, Props* props)
{
	for (uint i = 0; i < MAX_PROPS; i++)
	{
		Prop prop = props->props[i];
		renderer->props[i] = Prop_Drawable{ prop.position, prop.rotation };
		renderer->meshes.meshes[i].instance_offset = i;
		renderer->meshes.meshes[i].num_instances = 1;
	}

	update(renderer->meshes, sizeof(renderer->props), (byte*)renderer->props);
}
void draw(Prop_Renderer* renderer, mat4 proj_view)
{
	bind(renderer->shader);
	set_mat4(renderer->shader, "proj_view", proj_view);
	bind_texture(renderer->texture , 0);
	bind_texture(renderer->material, 1);

	draw(renderer->meshes);
}

// buildings

enum {
	AIR = 0,

	WALL_EXTERIOR,
	WALL_EXTERIOR_WINDOW,
	WALL_EXTERIOR_WINDOW_HUGE,
	WALL_EXTERIOR_SINGLE_DOOR,
	WALL_EXTERIOR_DOUBLE_DOOR,

	WALL_INTERIOR,
	WALL_INTERIOR_WINDOW,
	WALL_INTERIOR_WINDOW_HUGE,
	WALL_INTERIOR_SINGLE_DOOR,
	WALL_INTERIOR_DOUBLE_DOOR,

	ROOF_EDGE,
	ROOF_MIDDLE,
	ROOF_CORNER,
	ROOF_WINDOW_HUGE,

	CEILING_EDGE,
	CEILING_MIDDLE,
	CEILING_CORNER,

	FLOOR_CHECKERED,
	FLOOR_RED_CARPET,

	NUM_BLOCKS
};

#define BUILDING_SIZE_X 8
#define BUILDING_SIZE_Z 7
#define NUM_BUILDING_CELLS (BUILDING_SIZE_X * BUILDING_SIZE_Z)

struct Building_Cell
{
	struct Walls {
		uint front, back, left, right;
		uint top, bottom;
	} walls;

	struct Pillars {
		uint fl, fr, br, bl;
	} pillars;
};

struct Building_Data {
	vec3 position;
	Building_Cell cells[NUM_BUILDING_CELLS];
};

void init(Building_Data* data)
{
	*data = {};
	uint i = 0;

	data->cells[i++] = { WALL_EXTERIOR_WINDOW, 0, WALL_EXTERIOR };
	data->cells[i++] = { WALL_EXTERIOR_WINDOW, 0, 0, WALL_EXTERIOR };
	data->cells[i++] = { WALL_EXTERIOR_WINDOW };
	data->cells[i++] = { WALL_EXTERIOR_DOUBLE_DOOR };
	data->cells[i++] = { WALL_EXTERIOR_WINDOW };
	data->cells[i++] = { WALL_EXTERIOR_WINDOW, 0, WALL_INTERIOR };
	data->cells[i++] = { WALL_EXTERIOR_WINDOW, 0, 0, WALL_EXTERIOR};

	data->cells[i++].walls.left  = WALL_EXTERIOR_WINDOW;
	data->cells[i++].walls.right = WALL_INTERIOR_WINDOW;
	data->cells[i++] = {};
	data->cells[i++] = {};
	data->cells[i++] = {};
	data->cells[i++].walls.left  = WALL_INTERIOR_WINDOW;
	data->cells[i++].walls.right = WALL_EXTERIOR_WINDOW;
	
	data->cells[i++].walls.left  = WALL_EXTERIOR;
	data->cells[i++].walls.right = WALL_INTERIOR_DOUBLE_DOOR;
	data->cells[i++] = {};
	data->cells[i++] = {};
	data->cells[i++] = {};
	data->cells[i++].walls.left  = WALL_INTERIOR_WINDOW;
	data->cells[i++].walls.right = WALL_EXTERIOR;
}

#define MAX_BUILDING_WALLS 64

struct Building_Wall
{
	Phys_Cube* cube;
	uint id;
};

struct Building
{
	vec3 position; // mat3 rotation
	Building_Wall walls[MAX_BUILDING_WALLS];
};

void init(Building* building, Physics* phys)
{
	Building_Data* data = Alloc(Building_Data, 1);
	init(data);

	*building = {};
	float width = 5, height = 3, thickness = .3; // walls

	vec3 offset = building->position + (vec3(width, height, width) / 2.f);
	vec3 dimensions = vec3(thickness, height, width);

	uint num_walls = 0, i = 0;
	for (uint x = 0; x < BUILDING_SIZE_X; x++) {
	for (uint z = 0; z < BUILDING_SIZE_Z; z++)
	{
		Building_Cell cell = data->cells[i++];

		if (cell.walls.front)
		{
			Quat r = Quaternion(ToRadians(180), vec3(0, 1, 0));
			vec3 p = offset + vec3(width * x, 0, width * z) + vec3(width / -2.f, 0, 0);
			uint cube_index = add_phys_cube(phys, p, dimensions, 0, r);

			uint i = num_walls++;
			building->walls[i].cube = &phys->cubes[cube_index];
			building->walls[i].id = cell.walls.front;
		}

		if (cell.walls.left)
		{
			Quat r = Quaternion(ToRadians(90), vec3(0, 1, 0));
			vec3 p = offset + vec3(width * x, 0, width * z) + vec3(0, 0, width / -2.f);
			uint cube_index = add_phys_cube(phys, p, dimensions, 0, r);

			uint i = num_walls++;
			building->walls[i].cube = &phys->cubes[cube_index];
			building->walls[i].id = cell.walls.left;
		}

		if (cell.walls.right)
		{
			Quat r = Quaternion(ToRadians(-90), vec3(0, 1, 0));
			vec3 p = offset + vec3(width * x, 0, width * z) + vec3(0, 0, width / 2.f);
			uint cube_index = add_phys_cube(phys, p, dimensions, 0, r);

			uint i = num_walls++;
			building->walls[i].cube = &phys->cubes[cube_index];
			building->walls[i].id = cell.walls.right;
		}
	} }

	free(data);
}

// rendering

struct Building_Renderer
{
	uint num_walls;
	Prop_Drawable walls[MAX_BUILDING_WALLS];
	uint wall_mesh_index[MAX_BUILDING_WALLS];

	Shader shader;
	GLuint texture, material;
	Mesh_Renderer_UV meshes;
};

void init(Building_Renderer* renderer)
{
	load(&renderer->shader, "assets/shaders/transform/mesh_uv.vert", "assets/shaders/mesh_uv.frag");
	renderer->texture  = load_texture_bmp("assets/textures/heist.bmp"    );
	renderer->material = load_texture_bmp("assets/textures/materials.bmp");

	const char* paths[] = {
		"assets/meshes/build/wall_exterior.mesh_uv"
	};

	uint size = MAX_BUILDING_WALLS * sizeof(Prop_Drawable);
	init_prop_drawable(&renderer->meshes, paths, size, 1);
}
void update(Building_Renderer* renderer, Building* buildings, Physics* phys)
{
	uint num_walls = 0;
	for (uint i = 0; i < MAX_BUILDING_WALLS; i++)
	{
		if (buildings->walls[i].cube == NULL) continue;

		vec3 pos; mat3 rot;
		get_transform(buildings->walls[i].cube->body, &pos, &rot);

		uint j = num_walls++;
		renderer->walls[j] = Prop_Drawable{ pos, rot };
		renderer->wall_mesh_index[j] = buildings->walls[i].id - 1;
	}

	update(renderer->meshes, sizeof(Prop_Drawable) * num_walls, (byte*)renderer->walls);
}
void draw(Building_Renderer* renderer, mat4 proj_view)
{
	bind(renderer->shader);
	set_mat4(renderer->shader, "proj_view", proj_view);
	bind_texture(renderer->texture , 0);
	bind_texture(renderer->material, 1);

	renderer->meshes.meshes[0].num_instances = MAX_BUILDING_WALLS;
	draw(renderer->meshes);
}