#include "physics.h"

#define HEIGHTMAP_N	1024
#define HEIGHTMAP_L	64
#define HEIGHTMAP_S	2

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

void explode(Heightmap* map, vec3 position, GLuint tex) // not working yet
{
	uint x = (position.x / HEIGHTMAP_L) * HEIGHTMAP_N;
	uint z = (position.z / HEIGHTMAP_L) * HEIGHTMAP_N;
	
	map->height[(x * HEIGHTMAP_N) + z] *= 1.25;

	for (uint x = 256; x < 400; x++) {
	for (uint z = 256; z < 400; z++) {
		map->height[x * HEIGHTMAP_N + z] *= .9;
	}}

	glBindTexture(GL_TEXTURE_2D, tex);
	glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, HEIGHTMAP_N, HEIGHTMAP_N, GL_RED, GL_FLOAT, map->height);
}

struct Heightmap_Renderer
{
	Drawable_Mesh mesh;
	Shader shader;
	GLuint heights, normals, albedo;
};

void init(Heightmap_Renderer* renderer, Heightmap* heightmap, const char* path)
{
	load(&renderer->mesh, "assets/meshes/env/terrain.mesh", 0);
	load(&(renderer->shader), "assets/shaders/terrain.vert", "assets/shaders/terrain.frag");
	renderer->normals = load_texture_png("assets/textures/normals.png");
	renderer->albedo = load_texture_png("assets/textures/albedo.png");

	load_file_r32(path, heightmap->height, HEIGHTMAP_N);
	for (uint i = 0; i < HEIGHTMAP_N * HEIGHTMAP_N; i++) heightmap->height[i] *= HEIGHTMAP_S;

	glGenTextures(1, &renderer->heights);
	glBindTexture(GL_TEXTURE_2D, renderer->heights);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_R32F, HEIGHTMAP_N, HEIGHTMAP_N, 0, GL_RED, GL_FLOAT, heightmap->height);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
}
void draw(Heightmap_Renderer* renderer, mat4 proj_view)
{
	bind_texture(renderer->heights, 2);
	bind_texture(renderer->normals, 3);
	bind_texture(renderer->albedo , 4);
	bind(renderer->shader);
	set_mat4(renderer->shader, "proj_view", proj_view);
	draw(renderer->mesh);
}