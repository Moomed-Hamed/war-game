#include "water.h"

/* ------------------------- *
	-    Shallow Water      -
 * ------------------------- */

// small fd wave patch : a height/velocity scheme on one cell per point,
// reflecting edges, gaussian drops, and a fixed step that stays stable.

const uint SHALLOW_MAX_N = 128;

struct Shallow_Config
{
	int   n = 128;
	float size = 8.f;          // metres, full patch
	float speed = 1.6f;         // m/s
	float damping = .998f;
	float drop_strength = .65f; // metres
	float drop_interval = 4.f;  // seconds
	float level = 1.6f;         // mean water height in the scene
	vec3  albedo = vec3(.02f, .14f, .15f);
	float metallic = 0.f;
	float roughness = .05f;
};

struct Shallow_Water
{
	int   n;
	float size;
	float time;
	float drop_timer;
	float accumulator; // fixed-step remainder
	bool  rain;
	bool  paused;
	bool  dirty;
	uint  rng;

	float a[SHALLOW_MAX_N * SHALLOW_MAX_N];
	float b[SHALLOW_MAX_N * SHALLOW_MAX_N];
	float velocity[SHALLOW_MAX_N * SHALLOW_MAX_N];

	float* height;
	float* next;
};

uint shallow_rand(uint* state)
{
	uint x = *state;
	x ^= x << 13;
	x ^= x >> 17;
	x ^= x << 5;
	*state = x;
	return x;
}

float shallow_rand01(uint* state)
{
	return (shallow_rand(state) & 0xFFFFFFu) / 16777215.f;
}

void shallow_reset(Shallow_Water* w, const Shallow_Config& c)
{
	int n = c.n;
	if (n < 8) n = 8;
	if (n > (int)SHALLOW_MAX_N) n = SHALLOW_MAX_N;

	w->n = n;
	w->size = c.size;
	w->time = 0.f;
	w->drop_timer = 1.f;
	w->accumulator = 0.f;
	w->rain = true;
	w->paused = false;
	w->dirty = true;
	w->rng = 0x9e3779b9u;
	w->height = w->a;
	w->next   = w->b;

	int count = n * n;
	for (int i = 0; i < count; i++)
	{
		w->a[i] = 0.f;
		w->b[i] = 0.f;
		w->velocity[i] = 0.f;
	}
}

// x, z are patch-local metres, centred on the patch
void shallow_drop(Shallow_Water* w, float x, float z, float strength)
{
	int n = w->n;
	if (n < 2) return;

	float spacing = w->size / (float)(n - 1);
	float sigma = .28f;
	int cx = (int)((x / w->size + .5f) * (float)(n - 1));
	int cz = (int)((z / w->size + .5f) * (float)(n - 1));

	int reach = (int)(sigma * 3.f / spacing) + 1;

	for (int dz = -reach; dz <= reach; dz++)
	{
		for (int dx = -reach; dx <= reach; dx++)
		{
			int ix = cx + dx;
			int iz = cz + dz;
			if (ix < 1 || ix >= n - 1 || iz < 1 || iz >= n - 1) continue;

			float rx = (float)dx * spacing;
			float rz = (float)dz * spacing;
			float bump = strength * expf(-(rx * rx + rz * rz) / (2.f * sigma * sigma));
			w->height[iz * n + ix] += bump;
		}
	}

	w->dirty = true;
}

// returns false when paused / no time passed
// one fixed step : the caller never hands this a frame-sized delta
void shallow_step(Shallow_Water* w, const Shallow_Config& c, float dt)
{
	int n = w->n;
	float spacing = w->size / (float)(n - 1);
	float c2 = c.speed * c.speed;

	memcpy(w->next, w->height, sizeof(float) * n * n);

	for (int z = 1; z < n - 1; z++)
	{
		for (int x = 1; x < n - 1; x++)
		{
			int i = z * n + x;
			float h = w->height[i];

			float lap = (w->height[i - 1] + w->height[i + 1] +
			             w->height[i - n] + w->height[i + n] - 4.f * h) / (spacing * spacing);

			float v = w->velocity[i] + c2 * lap * dt;
			v *= c.damping;
			w->velocity[i] = v;
			w->next[i] = h + v * dt;
		}
	}

	float* swap = w->height;
	w->height = w->next;
	w->next = swap;
	w->time += dt;

	// numeric guard : a clamp keeps a bad frame from taking the surface with it
	bool bad = false;
	for (int i = 0; i < n * n; i++)
	{
		float h = w->height[i];
		float v = w->velocity[i];
		if (!isfinite(h) || !isfinite(v)) { bad = true; break; }

		if (h >  1.f) h =  1.f;
		if (h < -1.f) h = -1.f;
		if (v >  10.f) v =  10.f;
		if (v < -10.f) v = -10.f;

		w->height[i] = h;
		w->velocity[i] = v;
	}

	if (bad)
	{
		for (int i = 0; i < n * n; i++)
		{
			w->a[i] = 0.f;
			w->b[i] = 0.f;
			w->velocity[i] = 0.f;
		}
		w->height = w->a;
		w->next = w->b;
		w->accumulator = 0.f;
		return;
	}

	if (w->rain)
	{
		w->drop_timer -= dt;
		if (w->drop_timer <= 0.f)
		{
			float x = (shallow_rand01(&w->rng) * 2.f - 1.f) * w->size * .36f;
			float z = (shallow_rand01(&w->rng) * 2.f - 1.f) * w->size * .36f;
			shallow_drop(w, x, z, c.drop_strength);
			w->drop_timer = c.drop_interval;
		}
	}
}

bool shallow_update(Shallow_Water* w, const Shallow_Config& c, float dt, bool paused)
{
	if (w->n < 2) return false;
	if (paused) return false;

	// fixed 120 hz : stable for every speed the slider allows, independent of fps
	const float fixed_dt = 1.f / 120.f;
	float incoming = dt;
	if (incoming < 0.f) incoming = 0.f;
	if (incoming > .10f) incoming = .10f;
	w->accumulator += incoming;

	int steps = 0;
	bool stepped = false;

	while (w->accumulator >= fixed_dt && steps < 12)
	{
		shallow_step(w, c, fixed_dt);
		w->accumulator -= fixed_dt;
		steps++;
		stepped = true;
	}

	if (w->accumulator > .10f) w->accumulator = .10f;
	return stepped;
}

Mesh_Data shallow_build_mesh(const Shallow_Water* w)
{
	Mesh_Data mesh = {};
	int n = w->n;
	if (n < 2) return mesh;

	uint num_vertices = (uint)(n * n);
	uint num_indices  = (uint)((n - 1) * (n - 1) * 6);

	mesh.positions = Alloc(vec3, num_vertices);
	mesh.normals   = Alloc(vec3, num_vertices);
	mesh.uvs       = Alloc(vec2, num_vertices);
	mesh.indices   = Alloc(uint, num_indices);

	float spacing = w->size / (float)(n - 1);

	for (int z = 0; z < n; z++)
	{
		for (int x = 0; x < n; x++)
		{
			int i = z * n + x;
			int x0 = x > 0 ? x - 1 : x;
			int x1 = x < n - 1 ? x + 1 : x;
			int z0 = z > 0 ? z - 1 : z;
			int z1 = z < n - 1 ? z + 1 : z;

			float dx = (w->height[z * n + x1] - w->height[z * n + x0]) / ((x1 - x0) * spacing);
			float dz = (w->height[z1 * n + x] - w->height[z0 * n + x]) / ((z1 - z0) * spacing);

			mesh.positions[i] = vec3(((float)x / (float)(n - 1) - .5f) * w->size,
			                         w->height[i],
			                         ((float)z / (float)(n - 1) - .5f) * w->size);
			mesh.normals[i] = normalize(vec3(-dx, 1.f, -dz));
			mesh.uvs[i]     = vec2((float)x / (float)(n - 1), (float)z / (float)(n - 1));
		}
	}

	uint e = 0;
	for (int z = 0; z < n - 1; z++)
	{
		for (int x = 0; x < n - 1; x++)
		{
			uint a = (uint)(z * n + x);
			uint b = a + 1;
			uint c = a + (uint)n;
			uint d = c + 1;

			mesh.indices[e++] = a;  mesh.indices[e++] = c; mesh.indices[e++] = b;
			mesh.indices[e++] = b;  mesh.indices[e++] = c; mesh.indices[e++] = d;
		}
	}

	mesh.num_vertices = num_vertices;
	mesh.num_indices  = num_indices;
	return mesh;
}
