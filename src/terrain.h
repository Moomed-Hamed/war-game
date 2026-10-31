#include "player.h"

/* ------------------------- *
	-       Terrain         -
 * ------------------------- */

// heightfield generation : noise, a height grid, and a renderer mesh. the static
// pass has no terrain shader, so the grid is baked into band meshes while the
// collider stays a bullet heightfield.

const uint TERRAIN_MAX_N = 257; // points per side : 257^2 is the mesh limit
const uint TERRAIN_UV_TILES = 8; // the shader multiplies uv by 4 again

// a band is one mesh and one material, so the renderer never needs a per-vertex
// palette. water is flattened to its level, the rest follow the height grid
enum Terrain_Band
{
	TERRAIN_WATER = 0,
	TERRAIN_SAND,
	TERRAIN_GRASS,
	TERRAIN_ROCK,
	TERRAIN_SNOW,
	TERRAIN_BAND_COUNT
};

struct Terrain_Config
{
	int   resolution = 161;
	float size       = 128.f; // metres, full side
	float height     = 16.f;  // metres, full relief
	uint  seed       = 1337;

	int   octaves    = 5;
	float frequency  = .018f;
	float lacunarity = 2.f;
	float gain       = .5f;

	// strata boundaries as a fraction of the generated relief
	float water_at = .14f;
	float sand_at  = .22f;
	float grass_at = .62f;
	float rock_at  = .84f; // above this : snow

	vec3 water_albedo = vec3(.05f, .14f, .18f);
	vec3 sand_albedo  = vec3(.58f, .51f, .34f);
	vec3 grass_albedo = vec3(.16f, .26f, .11f);
	vec3 rock_albedo  = vec3(.26f, .25f, .23f);
	vec3 snow_albedo  = vec3(.82f, .85f, .88f);

	float water_roughness = .10f;
	float sand_roughness  = .85f;
	float grass_roughness = .90f;
	float rock_roughness  = .80f;
	float snow_roughness  = .60f;
};

struct Terrain
{
	int   resolution;
	float size;
	float height;
	float min_height, max_height;
	float water_level, sand_level, grass_level, rock_level;
	uint  seed;

	float heights[TERRAIN_MAX_N * TERRAIN_MAX_N];
};

/* ------------------------- *
	-        Noise          -
 * ------------------------- */

uint terrain_hash(uint x)
{
	x ^= x >> 16;
	x *= 0x7feb352du;
	x ^= x >> 15;
	x *= 0x846ca68bu;
	x ^= x >> 16;
	return x;
}

uint terrain_hash2(int x, int y, uint seed)
{
	uint h = (uint)x * 374761393u + (uint)y * 668265263u + seed * 1442695041u;
	return terrain_hash(h);
}

// one of eight unit gradients, picked by the hash
float terrain_gradient(int x, int y, uint seed, float dx, float dy)
{
	static const vec2 grads[8] = {
		{ 1, 0 }, { -1, 0 }, { 0, 1 }, { 0, -1 },
		{ .70710678f, .70710678f }, { -.70710678f, .70710678f },
		{ .70710678f, -.70710678f }, { -.70710678f, -.70710678f },
	};
	vec2 g = grads[terrain_hash2(x, y, seed) & 7];
	return g.x * dx + g.y * dy;
}

// classic gradient noise, smoothstep interpolated, roughly -1..1
float terrain_gradient_noise(float x, float y, uint seed)
{
	int ix = (int)floorf(x);
	int iy = (int)floorf(y);

	float fx = x - ix;
	float fy = y - iy;

	float u = fx * fx * fx * (fx * (fx * 6.f - 15.f) + 10.f);
	float v = fy * fy * fy * (fy * (fy * 6.f - 15.f) + 10.f);

	float n00 = terrain_gradient(ix,     iy,     seed, fx,      fy);
	float n10 = terrain_gradient(ix + 1, iy,     seed, fx - 1.f, fy);
	float n01 = terrain_gradient(ix,     iy + 1, seed, fx,      fy - 1.f);
	float n11 = terrain_gradient(ix + 1, iy + 1, seed, fx - 1.f, fy - 1.f);

	float a = n00 + (n10 - n00) * u;
	float b = n01 + (n11 - n01) * u;
	return (a + (b - a) * v) * 1.4f;
}

float terrain_fbm(float x, float y, uint seed, int octaves,
                  float lacunarity, float gain)
{
	float sum = 0.f, amp = 1.f, norm = 0.f;

	for (int i = 0; i < octaves; i++)
	{
		sum  += terrain_gradient_noise(x, y, seed + (uint)i * 1013u) * amp;
		norm += amp;
		amp  *= gain;
		x    *= lacunarity;
		y    *= lacunarity;
	}

	return norm > 0.f ? sum / norm : 0.f;
}

/* ------------------------- *
	-      Height grid       -
 * ------------------------- */

// a raised island : fbm inside, falling to the surrounding floor at the border
float terrain_sample(const Terrain_Config& c, float x, float z)
{
	float n = terrain_fbm(x * c.frequency, z * c.frequency,
	                      c.seed, c.octaves, c.lacunarity, c.gain);
	float h = (n * .5f + .5f) * c.height;

	float nx = x / (c.size * .5f);
	float nz = z / (c.size * .5f);
	float d  = fmaxf(fabsf(nx), fabsf(nz));

	float t = (d - .78f) / .22f;
	if (t < 0.f) t = 0.f;
	if (t > 1.f) t = 1.f;
	t = t * t * (3.f - 2.f * t);

	return .05f + h * (1.f - t);
}

// bounds and strata from the current heights : generation and any later edit
// share this, so a flattened plateau keeps sensible bands
void terrain_update_levels(Terrain* t, const Terrain_Config& c)
{
	int n = t->resolution;
	float min_h = 1e30f, max_h = -1e30f;

	for (int i = 0; i < n * n; i++)
	{
		float h = t->heights[i];
		if (h < min_h) min_h = h;
		if (h > max_h) max_h = h;
	}

	t->min_height = min_h;
	t->max_height = max_h;

	// bands are fractions of the actual relief, so a seed always shows every
	// stratum instead of a map that happens to miss the configured range
	float span = max_h - min_h;
	if (span < .001f) span = .001f;

	float water = c.water_at, sand = c.sand_at, grass = c.grass_at, rock = c.rock_at;
	if (sand  < water) sand  = water;
	if (grass < sand)  grass = sand;
	if (rock  < grass) rock  = grass;

	t->water_level = min_h + span * water;
	t->sand_level  = min_h + span * sand;
	t->grass_level = min_h + span * grass;
	t->rock_level  = min_h + span * rock;
}

void terrain_generate(Terrain* t, const Terrain_Config& c)
{
	int n = c.resolution;
	if (n < 2) n = 2;
	if (n > (int)TERRAIN_MAX_N) n = TERRAIN_MAX_N;

	t->resolution = n;
	t->size       = c.size;
	t->height     = c.height;
	t->seed       = c.seed;

	for (int j = 0; j < n; j++)
	{
		float z = ((float)j / (float)(n - 1) - .5f) * c.size;

		for (int i = 0; i < n; i++)
		{
			float x = ((float)i / (float)(n - 1) - .5f) * c.size;
			t->heights[j * n + i] = terrain_sample(c, x, z);
		}
	}

	terrain_update_levels(t, c);
}

float terrain_height(const Terrain* t, float x, float z);

// mean of the grid inside a rectangle : a plateau settles near its surroundings
float terrain_average_height(const Terrain* t, float cx, float cz, float half_x, float half_z)
{
	if (!t || t->resolution < 2) return 0.f;

	int n = t->resolution;
	float sum = 0.f;
	uint count = 0;

	for (int j = 0; j < n; j++)
	{
		float z = ((float)j / (float)(n - 1) - .5f) * t->size;
		if (fabsf(z - cz) > half_z) continue;

		for (int i = 0; i < n; i++)
		{
			float x = ((float)i / (float)(n - 1) - .5f) * t->size;
			if (fabsf(x - cx) > half_x) continue;

			sum += t->heights[j * n + i];
			count++;
		}
	}

	return count ? sum / (float)count : terrain_height(t, cx, cz);
}

// blend a corridor toward a straight profile : z0 is the high end, z1 the low
// end, and the width fades out so the cut reads as a valley rather than a trench
void terrain_ramp(Terrain* t, float cx, float z0, float z1, float half_x,
                  float falloff, float h0, float h1)
{
	if (!t || t->resolution < 2) return;
	if (z0 <= z1) return;
	if (half_x < .01f) half_x = .01f;
	if (falloff < .01f) falloff = .01f;

	int n = t->resolution;

	for (int j = 0; j < n; j++)
	{
		float z = ((float)j / (float)(n - 1) - .5f) * t->size;
		if (z > z0 || z < z1) continue;

		float along = (z0 - z) / (z0 - z1);
		float target = h0 + (h1 - h0) * along;

		for (int i = 0; i < n; i++)
		{
			float x = ((float)i / (float)(n - 1) - .5f) * t->size;
			float d = fabsf(x - cx) / half_x;

			float blend = 0.f;
			if (d <= 1.f) blend = 1.f;
			else if (d < 1.f + falloff)
			{
				float u = (d - 1.f) / falloff;
				blend = 1.f - u * u * (3.f - 2.f * u);
			}

			if (blend <= 0.f) continue;

			int at = j * n + i;
			t->heights[at] += (target - t->heights[at]) * blend;
		}
	}
}

// blend a rectangle toward one height : the core is flat, the falloff keeps a
// built yard from ending in a vertical cut through the island
void terrain_flatten(Terrain* t, float cx, float cz, float half_x, float half_z,
                     float height, float falloff)
{
	if (!t || t->resolution < 2) return;
	if (half_x < .01f) half_x = .01f;
	if (half_z < .01f) half_z = .01f;
	if (falloff < .01f) falloff = .01f;

	int n = t->resolution;

	for (int j = 0; j < n; j++)
	{
		float z = ((float)j / (float)(n - 1) - .5f) * t->size;

		for (int i = 0; i < n; i++)
		{
			float x = ((float)i / (float)(n - 1) - .5f) * t->size;

			float dx = fabsf(x - cx) / half_x;
			float dz = fabsf(z - cz) / half_z;
			float d  = dx > dz ? dx : dz;

			float blend = 0.f;
			if (d <= 1.f) blend = 1.f;
			else if (d < 1.f + falloff)
			{
				float u = (d - 1.f) / falloff;
				blend = 1.f - u * u * (3.f - 2.f * u);
			}

			if (blend <= 0.f) continue;

			int at = j * n + i;
			t->heights[at] += (height - t->heights[at]) * blend;
		}
	}
}

// bilinear sample in world space, clamped to the grid
float terrain_height(const Terrain* t, float x, float z)
{
	if (!t || t->resolution < 2) return 0.f;

	float u = (x / t->size + .5f) * (float)(t->resolution - 1);
	float v = (z / t->size + .5f) * (float)(t->resolution - 1);

	if (u < 0.f) u = 0.f;
	if (v < 0.f) v = 0.f;
	if (u > (float)(t->resolution - 1)) u = (float)(t->resolution - 1);
	if (v > (float)(t->resolution - 1)) v = (float)(t->resolution - 1);

	int i0 = (int)u, j0 = (int)v;
	int i1 = i0 + 1 < t->resolution ? i0 + 1 : i0;
	int j1 = j0 + 1 < t->resolution ? j0 + 1 : j0;

	float fu = u - i0, fv = v - j0;

	float a = t->heights[j0 * t->resolution + i0];
	float b = t->heights[j0 * t->resolution + i1];
	float c = t->heights[j1 * t->resolution + i0];
	float d = t->heights[j1 * t->resolution + i1];

	float top = a + (b - a) * fu;
	float bot = c + (d - c) * fu;
	return top + (bot - top) * fv;
}

// which stratum a height belongs to
uint terrain_band_at(const Terrain* t, float height)
{
	if (height < t->water_level) return TERRAIN_WATER;
	if (height < t->sand_level)  return TERRAIN_SAND;
	if (height < t->grass_level) return TERRAIN_GRASS;
	if (height < t->rock_level)  return TERRAIN_ROCK;
	return TERRAIN_SNOW;
}

// one band's triangles, using only the grid vertices that band touches. water
// is flattened to its level so it reads as a surface, not a low hillside
Mesh_Data terrain_build_band(const Terrain* t, uint band)
{
	Mesh_Data mesh = {};
	int n = t->resolution;
	if (n < 2 || band >= TERRAIN_BAND_COUNT) return mesh;

	uint max_vertices = (uint)(n * n);
	uint max_indices  = (uint)((n - 1) * (n - 1) * 6);

	mesh.positions = Alloc(vec3, max_vertices);
	mesh.normals   = Alloc(vec3, max_vertices);
	mesh.uvs       = Alloc(vec2, max_vertices);
	mesh.indices   = Alloc(uint, max_indices);

	int* remap = Alloc(int, max_vertices);
	for (uint i = 0; i < max_vertices; i++) remap[i] = -1;

	float step = t->size / (float)(n - 1);
	uint num_vertices = 0, num_indices = 0;

	for (int j = 0; j < n - 1; j++)
	{
		for (int i = 0; i < n - 1; i++)
		{
			uint a = (uint)(j * n + i);
			uint b = a + 1;
			uint c = a + n;
			uint d = c + 1;

			const uint tris[2][3] = { { a, c, b }, { b, c, d } };

			for (uint k = 0; k < 2; k++)
			{
				float center = (t->heights[tris[k][0]] + t->heights[tris[k][1]] +
				                t->heights[tris[k][2]]) / 3.f;
				if (terrain_band_at(t, center) != band) continue;

				for (uint q = 0; q < 3; q++)
				{
					uint v = tris[k][q];
					if (remap[v] < 0)
					{
						int gx = (int)(v % (uint)n);
						int gz = (int)(v / (uint)n);

						float x = ((float)gx / (float)(n - 1) - .5f) * t->size;
						float z = ((float)gz / (float)(n - 1) - .5f) * t->size;
						float y = band == TERRAIN_WATER ? t->water_level : t->heights[v];
						mesh.positions[num_vertices] = vec3(x, y, z);

						if (band == TERRAIN_WATER)
							mesh.normals[num_vertices] = vec3(0, 1, 0);
						else
						{
							int i0 = gx > 0 ? gx - 1 : gx;
							int i1 = gx < n - 1 ? gx + 1 : gx;
							int j0 = gz > 0 ? gz - 1 : gz;
							int j1 = gz < n - 1 ? gz + 1 : gz;

							float dx = (t->heights[gz * n + i1] - t->heights[gz * n + i0]) / ((i1 - i0) * step);
							float dz = (t->heights[j1 * n + gx] - t->heights[j0 * n + gx]) / ((j1 - j0) * step);

							mesh.normals[num_vertices] = normalize(vec3(-dx, 1.f, -dz));
						}

						mesh.uvs[num_vertices] = vec2((float)gx / (float)(n - 1),
						                              (float)gz / (float)(n - 1)) * (float)TERRAIN_UV_TILES;
						remap[v] = (int)num_vertices++;
					}

					mesh.indices[num_indices++] = (uint)remap[v];
				}
			}
		}
	}

	mesh.num_vertices = num_vertices;
	mesh.num_indices  = num_indices;
	free(remap);
	return mesh;
}

// the material one band hands to the instance stream
void terrain_band_material(const Terrain_Config& c, uint band, vec4* mat_a, vec4* mat_b)
{
	vec3  albedo    = c.grass_albedo;
	float roughness = c.grass_roughness;

	switch (band)
	{
		case TERRAIN_WATER: albedo = c.water_albedo; roughness = c.water_roughness; break;
		case TERRAIN_SAND:  albedo = c.sand_albedo;  roughness = c.sand_roughness;  break;
		case TERRAIN_GRASS: albedo = c.grass_albedo; roughness = c.grass_roughness; break;
		case TERRAIN_ROCK:  albedo = c.rock_albedo;  roughness = c.rock_roughness;  break;
		case TERRAIN_SNOW:  albedo = c.snow_albedo;  roughness = c.snow_roughness;  break;
	}

	*mat_a = vec4(albedo, 0.f);
	*mat_b = vec4(roughness, 1.f, 0.f, 0.f);
}