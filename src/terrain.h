#include "renderer.h"

#define HEIGHTMAP_N	1024 // num data points per row | resolution
#define HEIGHTMAP_L	256  // side length of terrain in meters
#define HEIGHTMAP_S	30   // terrain vertical scale in meters

uint height_index(float pos_x, float pos_z)
{
	uint x = pos_x / (HEIGHTMAP_N / (float)HEIGHTMAP_L);
	uint z = pos_z / (HEIGHTMAP_N / (float)HEIGHTMAP_L);
	return (x * HEIGHTMAP_N) + z;
}

struct Heightmap
{
	float height[HEIGHTMAP_N * HEIGHTMAP_N];
	vec3 normals[HEIGHTMAP_N * HEIGHTMAP_N];
};

float height(Heightmap* map, vec3 pos)
{
	return map->height[height_index(pos.x, pos.z)];
}
float height(Heightmap* map, vec2 pos)
{
	return map->height[height_index(pos.x, pos.y)];
}
vec3 terrain(Heightmap* map, vec2 pos)
{
	return vec3(pos.x, map->height[height_index(pos.x, pos.y)], pos.y);
}

void explode(Heightmap* map, vec3 position, float radius = 15)
{
	float scale = HEIGHTMAP_N / (float)HEIGHTMAP_L;
	uint nx = (uint)position.x * scale;
	uint nz = (uint)position.z * scale;

	uint n_length = radius * scale; // radius of the circle

	for (uint x = nx - n_length; x < nx + n_length; x++) {
	for (uint z = nz - n_length; z < nz + n_length; z++)
	{
		float k = length(vec2{ nx, nz } - vec2{ x, z }) / n_length;
		map->height[z * HEIGHTMAP_N + x] *= k < 1.f ? k : 1.f;
	}}
}
void extrude(Heightmap* map, vec3 position, float radius = 5)
{
	float scale = HEIGHTMAP_N / (float)HEIGHTMAP_L;
	uint nx = (uint)position.x * scale;
	uint nz = (uint)position.z * scale;

	uint n_length = radius * scale; // radius of the circle

	for (uint x = nx - n_length; x < nx + n_length; x++) {
	for (uint z = nz - n_length; z < nz + n_length; z++)
	{
		float k = length(vec2{ nx, nz } - vec2{ x, z }) / n_length;
		map->height[z * HEIGHTMAP_N + x] += k < 1.f ? 1 - k : 0.f;
	} }
}

// is this a good idea?
struct PBR_Texture { GLuint normal, albedo, material; };

struct Heightmap_Renderer
{
	Drawable_Mesh mesh;
	Shader shader;
	GLuint heights;
	PBR_Texture grass, dirt;
};

void init(Heightmap_Renderer* renderer, Heightmap* heightmap, const char* path)
{
	load(&renderer->mesh, "assets/meshes/env/terrain.mesh");
	load(&renderer->shader, "assets/shaders/terrain.vert", "assets/shaders/terrain.frag");

	renderer->grass.normal   = load_texture_png("assets/textures/ground/grass_normal.png");
	renderer->grass.albedo   = load_texture_png("assets/textures/ground/grass_albedo.png");
	renderer->grass.material = load_texture_png("assets/textures/ground/grass_mat.png");

	renderer->dirt.normal   = load_texture_png("assets/textures/ground/dirt_normal.png");
	renderer->dirt.albedo   = load_texture_png("assets/textures/ground/unit_albedo.png");
	renderer->dirt.material = load_texture_png("assets/textures/ground/dirt_mat.png");

	// load the heightmap data + scale it + raise it
	load_file_r32(path, heightmap->height, HEIGHTMAP_N);
	for (uint i = 0; i < HEIGHTMAP_N * HEIGHTMAP_N; i++) heightmap->height[i] *= HEIGHTMAP_S;
	//for (uint i = 0; i < HEIGHTMAP_N * HEIGHTMAP_N; i++) heightmap->height[i] += 1.f;
	for (uint i = 0; i < HEIGHTMAP_N * HEIGHTMAP_N; i++) heightmap->height[i] += perlin((float)i / 100);

	glGenTextures(1, &renderer->heights);
	glBindTexture(GL_TEXTURE_2D, renderer->heights);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_R32F, HEIGHTMAP_N, HEIGHTMAP_N, 0, GL_RED, GL_FLOAT, heightmap->height);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
}
void update(Heightmap_Renderer* renderer, Heightmap* map)
{
	glBindTexture(GL_TEXTURE_2D, renderer->heights);
	glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, HEIGHTMAP_N, HEIGHTMAP_N, GL_RED, GL_FLOAT, map->height);
}
void draw(Heightmap_Renderer* renderer, mat4 proj_view)
{
	bind_texture(renderer->heights, 2);
	bind_texture(renderer->grass.normal  , 3);
	bind_texture(renderer->grass.albedo  , 4);
	bind_texture(renderer->grass.material, 5);
	bind_texture(renderer->dirt.normal   , 6);
	bind_texture(renderer->dirt.albedo   , 7);
	bind_texture(renderer->dirt.material , 8);

	bind(renderer->shader);
	set_mat4(renderer->shader, "proj_view", proj_view);
	draw(renderer->mesh);
}