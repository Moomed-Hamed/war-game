#include "physics.h"

#define HEIGHTMAP_N	1024 // num data points per row
#define HEIGHTMAP_L	256  // side length of terrain
#define HEIGHTMAP_S	5    // terrain vertical scale

uint height_index(float pos_x, float pos_z)
{
	uint x = (pos_x / HEIGHTMAP_L) * HEIGHTMAP_N;
	uint z = (pos_z / HEIGHTMAP_L) * HEIGHTMAP_N;
	return (x * HEIGHTMAP_N) + z;
}

struct Heightmap
{
	float height[HEIGHTMAP_N * HEIGHTMAP_N];
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

void explode(Heightmap* map, vec3 position, GLuint tex, float dtime)
{
	uint x = (position.x / HEIGHTMAP_L) * HEIGHTMAP_N;
	uint z = (position.z / HEIGHTMAP_L) * HEIGHTMAP_N;

	for (uint x = 56; x < 100; x++) {
	for (uint z = 56; z < 100; z++) {
		float a = map->height[x * HEIGHTMAP_N + z];
		a *= a < 0 ? .9 : -.9;
		map->height[x * HEIGHTMAP_N + z] = a;
	}}

	glBindTexture(GL_TEXTURE_2D, tex);
	glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, HEIGHTMAP_N, HEIGHTMAP_N, GL_RED, GL_FLOAT, map->height);
}

struct Heightmap_Renderer
{
	Drawable_Mesh mesh;
	Shader shader;
	GLuint heights;
	GLuint grass[3], dirt[3];
};

void init(Heightmap_Renderer* renderer, Heightmap* heightmap, const char* path)
{
	load(&renderer->mesh, "assets/meshes/env/terrain.mesh");
	load(&renderer->shader, "assets/shaders/terrain.vert", "assets/shaders/terrain.frag");

	renderer->grass[0] = load_texture_png("assets/textures/ground/grass_normal.png");
	renderer->grass[1] = load_texture_png("assets/textures/ground/grass_albedo.png");
	renderer->grass[2] = load_texture_png("assets/textures/ground/grass_mat.png");

	renderer->dirt [0] = load_texture_png("assets/textures/ground/dirt_normal.png");
	renderer->dirt [1] = load_texture_png("assets/textures/ground/dirt_albedo.png");
	renderer->dirt [2] = load_texture_png("assets/textures/ground/dirt_mat.png");

	load_file_r32(path, heightmap->height, HEIGHTMAP_N);
	for (uint i = 0; i < HEIGHTMAP_N * HEIGHTMAP_N; i++) heightmap->height[i] *= HEIGHTMAP_S;
	for (uint i = 0; i < HEIGHTMAP_N * HEIGHTMAP_N; i++) heightmap->height[i] += 1;

	glGenTextures(1, &renderer->heights);
	glBindTexture(GL_TEXTURE_2D, renderer->heights);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_R32F, HEIGHTMAP_N, HEIGHTMAP_N, 0, GL_RED, GL_FLOAT, heightmap->height);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
}
void draw(Heightmap_Renderer* renderer, mat4 proj_view)
{
	bind_texture(renderer->heights, 2);
	bind_texture(renderer->grass[0], 3);
	bind_texture(renderer->grass[1], 4);
	bind_texture(renderer->grass[2], 5);
	bind_texture(renderer->dirt [0], 6);
	bind_texture(renderer->dirt [1], 7);
	bind_texture(renderer->dirt [2], 8);
	bind(renderer->shader);
	set_mat4(renderer->shader, "proj_view", proj_view);
	draw(renderer->mesh);
}