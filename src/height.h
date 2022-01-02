#include "physics.h"

#define HEIGHTMAP_N	1024
#define HEIGHTMAP_L	50
#define MAX_HEIGHTMAP_POINTS (HEIGHTMAP_N * HEIGHTMAP_N)

struct Heightmap
{
	float height[MAX_HEIGHTMAP_POINTS];
};

float get_height(Heightmap* map, vec3 pos)
{
	uint x = pos.x / (float)HEIGHTMAP_L;
	uint z = pos.x / (float)HEIGHTMAP_L;

	uint index = x * HEIGHTMAP_N + z;
	return map->height[index];
}

struct Heightmap_Renderer
{
	Drawable_Mesh mesh;
	Shader shader;
	GLuint heights, normals;
};

void init(Heightmap_Renderer* renderer, Heightmap* heightmap, const char* path)
{
	load(&renderer->mesh, "assets/meshes/env/sea.mesh", 0);
	load(&(renderer->shader), "assets/shaders/terrain.vert", "assets/shaders/terrain.frag");

	uint n = HEIGHTMAP_N;
	load_file_r32(path, heightmap->height, n);

	glGenTextures(1, &renderer->heights); // terrain heightmap
	glBindTexture(GL_TEXTURE_2D, renderer->heights);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_R32F, n, n, 0, GL_RED, GL_FLOAT, heightmap->height);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

	renderer->normals = load_texture_png("assets/textures/normals.png");
}
void draw(Heightmap_Renderer* renderer, mat4 proj_view)
{
	bind(renderer->shader);
	set_mat4(renderer->shader, "proj_view", proj_view);

	glActiveTexture(GL_TEXTURE2);
	glBindTexture(GL_TEXTURE_2D, renderer->heights);

	glActiveTexture(GL_TEXTURE3);
	glBindTexture(GL_TEXTURE_2D, renderer->normals);

	draw(renderer->mesh);
}