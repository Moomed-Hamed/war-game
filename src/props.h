#include "particles.h"

#define MAX_PROPS 8 // max number of props of a given type
#define NUM_PROP_TYPES 6

#define PROP_STATIC 1
#define PROP_DESTRUCTIBLE 2

struct Prop
{
	uint type;
	vec3 position;
	mat3 transform;
};

struct Props
{
	union
	{
		Prop props[MAX_PROPS * NUM_PROP_TYPES];

		struct // don't rearrange these pl0x
		{
			Prop tanktraps [MAX_PROPS];
			Prop crates    [MAX_PROPS];
			Prop sandbags  [MAX_PROPS];
			Prop campfires [MAX_PROPS];
			Prop trees     [MAX_PROPS];
			Prop barrels   [MAX_PROPS];
		};
	};
};

void init(Props* props, Heightmap* map)
{
	props->crates    [0] = { PROP_STATIC , terrain(map, vec2(15, 6)) , mat3(1) };
	props->tanktraps [0] = { PROP_STATIC , terrain(map, vec2(8 , 7)) , mat3(1) };
	props->sandbags  [0] = { PROP_STATIC , terrain(map, vec2(15, 6)) , mat3(1) };
	props->trees     [0] = { PROP_STATIC , terrain(map, vec2(32, 4)) , mat3(1) };
	props->campfires [0] = { PROP_STATIC , terrain(map, vec2(9 , 6)) , mat3(1) };
	props->barrels   [0] = { PROP_STATIC , terrain(map, vec2(13, 6)) , mat3(1) };
}
void update(Props* props)
{

}

// rendering

struct Prop_Drawable
{
	vec3 position;
	mat3 transform;
};

struct Prop_Renderer
{
	Prop_Drawable props[MAX_PROPS];
	Drawable_Mesh_UV meshes[NUM_PROP_TYPES];
	GLuint texture, material;
	Shader shader;
};

void init_prop_drawable(Drawable_Mesh_UV* mesh, const char* path)
{
	load(mesh, path, MAX_PROPS * sizeof(Prop_Drawable));
	mesh_add_attrib_vec3(3, sizeof(Prop_Drawable), 0 * sizeof(vec3)); // position
	mesh_add_attrib_mat3(4, sizeof(Prop_Drawable), 1 * sizeof(vec3)); // color
}

void init(Prop_Renderer* renderer)
{
	load(&(renderer->shader), "assets/shaders/transform/mesh_uv.vert", "assets/shaders/mesh_uv.frag");
	renderer->texture  = load_texture("assets/textures/palette2.bmp");
	renderer->material = load_texture("assets/textures/materials.bmp");

	init_prop_drawable(renderer->meshes + 0, "assets/meshes/env/palm_tree.mesh_uv" );
	init_prop_drawable(renderer->meshes + 1, "assets/meshes/props/crate.mesh_uv"   );
	init_prop_drawable(renderer->meshes + 2, "assets/meshes/props/barrel.mesh_uv"  );
	init_prop_drawable(renderer->meshes + 3, "assets/meshes/props/campfire.mesh_uv");
	init_prop_drawable(renderer->meshes + 4, "assets/meshes/env/grass_1.mesh_uv"   );
	init_prop_drawable(renderer->meshes + 5, "assets/meshes/env/grass_2.mesh_uv"   );
}
void update(Prop_Renderer* renderer, Props* props)
{
	for (uint i = 0; i < NUM_PROP_TYPES; i++) {
	for (uint j = 0; j < MAX_PROPS; j++)
	{
		Prop prop = props->props[(i * MAX_PROPS) + j];
		renderer->props[j] = prop.type ? Prop_Drawable{ prop.position, prop.transform } : Prop_Drawable{};
	}
		update(renderer->meshes[i], sizeof(renderer->props), (byte*)renderer->props);
	}
}
void draw(Prop_Renderer* renderer, mat4 proj_view)
{
	bind(renderer->shader);
	set_mat4(renderer->shader, "proj_view", proj_view);
	bind_texture(renderer->texture , 0);
	bind_texture(renderer->material, 1);

	for (uint i = 0; i < NUM_PROP_TYPES; i++)
		draw(renderer->meshes[i], MAX_PROPS);
}