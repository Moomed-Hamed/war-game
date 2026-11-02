#include "player.h"

/* ------------------------- *
	-        Fluids         -
 * ------------------------- */

// ---- FFT ocean ----

// cpu fft ocean : a phillips spectrum, evolved in time, then five inverse
// transforms (height, two slopes, two horizontal displacements). the patch is
// periodic and `size` metres wide, so the scene can tile one mesh.
//
// h0(-k) is taken from the mirrored spectrum entry and every inverse pass
// divides by n, so the height field is real and its rms matches the amplitude
// the controls expose.

const uint WATER_MAX_N = 256;
const uint WATER_DETAIL_N = 128;

struct Water_Complex
{
	float re, im;
};

inline Water_Complex operator+(Water_Complex a, Water_Complex b) { return { a.re + b.re, a.im + b.im }; }
inline Water_Complex operator-(Water_Complex a, Water_Complex b) { return { a.re - b.re, a.im - b.im }; }
inline Water_Complex operator*(Water_Complex a, Water_Complex b) { return { a.re * b.re - a.im * b.im, a.re * b.im + a.im * b.re }; }
inline Water_Complex operator*(Water_Complex a, float s)         { return { a.re * s, a.im * s }; }
inline Water_Complex water_conj(Water_Complex a)                 { return { a.re, -a.im }; }

struct Water_Config
{
	int   resolution = 128; // power of two : spectrum + render grid
	float size       = 160.f; // metres for one patch
	float wind_speed = 9.f;  // m/s
	float wind_angle = .60f; // radians
	float amplitude  = .55f; // rms height, metres
	float choppiness = .95f; // horizontal displacement scale
	float time_scale = .88f;
	float foam       = .35f; // foam source amount, 0..1
	float foam_persistence = 1.6f; // seconds a trail survives
	float detail     = .68f; // short-chop normal strength
	uint  seed       = 7;

	// material
	vec3  albedo        = vec3(.020f, .160f, .185f);
	float roughness     = .055f;
	float sss           = .72f; // backlit crest glow
	vec3  foam_albedo   = vec3(.80f, .87f, .89f);
	float foam_roughness = .50f;
};

// second cascade : short wind chop, only slopes. it becomes a tiled normal map
struct Water_Detail
{
	int   n;
	float size;

	Water_Complex h0    [WATER_DETAIL_N * WATER_DETAIL_N];
	Water_Complex h0_conj[WATER_DETAIL_N * WATER_DETAIL_N];
	Water_Complex field [WATER_DETAIL_N * WATER_DETAIL_N];
	Water_Complex work  [WATER_DETAIL_N * WATER_DETAIL_N];

	float slope_x[WATER_DETAIL_N * WATER_DETAIL_N];
	float slope_z[WATER_DETAIL_N * WATER_DETAIL_N];

	unsigned char normals[WATER_DETAIL_N * WATER_DETAIL_N * 2];
};

struct Water
{
	int   n;
	float size;
	float time;
	bool  paused;

	// spectrum : h0(k) and conj(h0(-k)), then one evolved field at a time
	Water_Complex h0    [WATER_MAX_N * WATER_MAX_N];
	Water_Complex h0_conj[WATER_MAX_N * WATER_MAX_N];
	Water_Complex field [WATER_MAX_N * WATER_MAX_N];
	Water_Complex work  [WATER_MAX_N * WATER_MAX_N];

	// displacements already in metres
	float height  [WATER_MAX_N * WATER_MAX_N];
	float slope_x [WATER_MAX_N * WATER_MAX_N];
	float slope_z [WATER_MAX_N * WATER_MAX_N];
	float disp_x  [WATER_MAX_N * WATER_MAX_N];
	float disp_z  [WATER_MAX_N * WATER_MAX_N];
	float jacobian[WATER_MAX_N * WATER_MAX_N];

	// persistent foam : source adds, trails decay, the blurred field is uploaded
	float foam[WATER_MAX_N * WATER_MAX_N];
	float foam_blur[WATER_MAX_N * WATER_MAX_N];
	unsigned char foam_pixels[WATER_MAX_N * WATER_MAX_N];

	Water_Detail detail;
};

// fully developed sea presets : wind in m/s, rms from Hs = 0.21 U^2 / g
// (significant height divided by four), patch sizes cover the wavelength
// where the phillips peak sits (2 pi U^2 / g)
struct Water_Preset
{
	const char* name;
	float size, wind_speed, wind_angle;
	float amplitude, choppiness, time_scale;
	float foam, persistence, detail, roughness;
	vec3  albedo;
};

// name, patch, wind, direction, rms, chop, time, foam amount, persistence, detail, roughness, albedo
const Water_Preset WATER_PRESETS[] = {
	{ "glassy",  96.f,  3.f, 0.20f, 0.05f, 0.25f, 0.55f, 0.00f, 0.50f, 0.25f, 0.025f, vec3(.012f, .075f, .100f) },
	{ "calm",   128.f,  6.f, 0.45f, 0.19f, 0.50f, 0.80f, 0.10f, 1.00f, 0.45f, 0.040f, vec3(.018f, .095f, .125f) },
	{ "breeze", 160.f,  9.f, 0.60f, 0.43f, 0.80f, 1.00f, 0.45f, 1.80f, 0.65f, 0.060f, vec3(.022f, .120f, .155f) },
	{ "caribbean", 160.f, 9.f, 0.60f, 0.55f, 0.95f, 0.88f, 0.35f, 1.60f, 0.68f, 0.055f, vec3(.020f, .160f, .185f) },
	{ "lagoon", 128.f, 6.f, 0.55f, 0.18f, 0.45f, 0.80f, 0.08f, 1.00f, 0.55f, 0.035f, vec3(.030f, .220f, .230f) },
	{ "swell",  192.f, 13.f, 0.95f, 0.90f, 0.90f, 0.90f, 0.55f, 2.20f, 0.70f, 0.080f, vec3(.020f, .105f, .140f) },
	{ "chop",   256.f, 16.f, 1.35f, 1.37f, 1.15f, 1.10f, 0.65f, 2.50f, 0.85f, 0.100f, vec3(.018f, .090f, .125f) },
	{ "storm",  256.f, 20.f, 2.20f, 2.14f, 1.45f, 1.20f, 0.80f, 3.00f, 1.00f, 0.130f, vec3(.012f, .055f, .075f) },
};

const uint WATER_PRESET_COUNT = sizeof(WATER_PRESETS) / sizeof(WATER_PRESETS[0]);

void water_apply_preset(Water_Config* c, uint index)
{
	if (index >= WATER_PRESET_COUNT) return;
	const Water_Preset& p = WATER_PRESETS[index];

	c->size       = p.size;
	c->wind_speed = p.wind_speed;
	c->wind_angle = p.wind_angle;
	c->amplitude  = p.amplitude;
	c->choppiness = p.choppiness;
	c->time_scale = p.time_scale;
	c->foam       = p.foam;
	c->foam_persistence = p.persistence;
	c->detail     = p.detail;
	c->roughness  = p.roughness;
	c->albedo     = p.albedo;
}

// ---- FFT ----

// in-place radix-2 inverse transform, scaled by 1/n
void water_ifft(Water_Complex* a, uint n)
{
	for (uint i = 1, j = 0; i < n; i++)
	{
		uint bit = n >> 1;
		for (; j & bit; bit >>= 1) j ^= bit;
		j ^= bit;
		if (i < j) { Water_Complex t = a[i]; a[i] = a[j]; a[j] = t; }
	}

	for (uint len = 2; len <= n; len <<= 1)
	{
		float angle = 2.f * PI / (float)len;
		Water_Complex wlen = { cosf(angle), sinf(angle) };

		for (uint i = 0; i < n; i += len)
		{
			Water_Complex w = { 1.f, 0.f };
			for (uint j = 0; j < len / 2; j++)
			{
				Water_Complex u = a[i + j];
				Water_Complex v = a[i + j + len / 2] * w;
				a[i + j]           = u + v;
				a[i + j + len / 2] = u - v;
				w = w * wlen;
			}
		}
	}

	float inv = 1.f / (float)n;
	for (uint i = 0; i < n; i++) a[i] = a[i] * inv;
}

// rows then columns : the 1/n from each pass makes the 2D scale 1/n^2
void water_ifft2(Water_Complex* data, uint n)
{
	Water_Complex line[WATER_MAX_N];

	for (uint x = 0; x < n; x++)
	{
		for (uint z = 0; z < n; z++) line[z] = data[z * n + x];
		water_ifft(line, n);
		for (uint z = 0; z < n; z++) data[z * n + x] = line[z];
	}

	for (uint z = 0; z < n; z++)
	{
		for (uint x = 0; x < n; x++) line[x] = data[z * n + x];
		water_ifft(line, n);
		for (uint x = 0; x < n; x++) data[z * n + x] = line[x];
	}
}

// ---- spectrum ----

uint water_rng(uint* state)
{
	uint x = *state;
	x ^= x << 13;
	x ^= x >> 17;
	x ^= x << 5;
	*state = x;
	return x;
}

float water_rand01(uint* state)
{
	return (water_rng(state) & 0x00FFFFFFu) / 16777215.f;
}

// two independent standard normals, box-muller
void water_gaussian2(uint* state, float* a, float* b)
{
	float u1 = water_rand01(state);
	float u2 = water_rand01(state);
	if (u1 < 1e-6f) u1 = 1e-6f;

	float r = sqrtf(-2.f * logf(u1));
	*a = r * cosf(2.f * PI * u2);
	*b = r * sinf(2.f * PI * u2);
}

// phillips spectrum shape : wind drives the peak, damping kills the smallest ripples
float water_phillips(vec2 k, vec2 wind, float damping)
{
	float k2 = dot(k, k);
	if (k2 < 1e-10f || dot(wind, wind) < 1e-6f) return 0.f;

	float kw  = dot(normalize(k), normalize(wind));
	float l   = dot(wind, wind) / 9.81f;
	float l2  = l * l;
	float dl2 = l2 * damping * damping;

	return expf(-1.f / (k2 * l2)) / (k2 * k2) * kw * kw * expf(-k2 * dl2);
}

void water_build_spectrum(Water* w, const Water_Config& c)
{
	int n = c.resolution;
	if (n < 16) n = 16;
	if (n > (int)WATER_MAX_N) n = WATER_MAX_N;
	n = 1 << (int)floorf(log2f((float)n) + .5f); // power of two

	w->n    = n;
	w->size = c.size;
	for (int i = 0; i < n * n; i++) w->foam[i] = 0.f;

	vec2  wind     = vec2(cosf(c.wind_angle), sinf(c.wind_angle)) * c.wind_speed;
	float k_min     = -PI * (float)n / c.size;
	float dk        = TWOPI / c.size;
	float energy    = 0.f;
	uint  rng       = c.seed ? c.seed : 1;

	for (int z = 0; z < n; z++)
	{
		for (int x = 0; x < n; x++)
		{
			vec2 k = vec2(k_min + x * dk, k_min + z * dk);
			float p = water_phillips(k, wind, .001f);

			float g1, g2;
			water_gaussian2(&rng, &g1, &g2);

			float amp = sqrtf(p * .5f);
			w->h0[z * n + x] = { g1 * amp, g2 * amp };
			energy += p;
		}
	}

	// make the height slider mean rms metres instead of hiding a spectrum scale.
	// the 2 is the complex gaussian's two unit-variance components
	float scale = (energy > 1e-12f) ? c.amplitude * (float)n * (float)n / sqrtf(2.f * energy) : 0.f;
	for (int i = 0; i < n * n; i++) w->h0[i] = w->h0[i] * scale;

	for (int z = 0; z < n; z++)
	{
		for (int x = 0; x < n; x++)
		{
			int mx = (n - x) % n;
			int mz = (n - z) % n;
			w->h0_conj[z * n + x] = water_conj(w->h0[mz * n + mx]);
		}
	}
}

// ---- simulation ----

// the centered spectrum leaves a (-1)^(x+z) modulation on the output
void water_extract(Water* w, float* out)
{
	int n = w->n;
	for (int z = 0; z < n; z++)
		for (int x = 0; x < n; x++)
		{
			float sign = ((x + z) & 1) ? -1.f : 1.f;
			out[z * n + x] = w->work[z * n + x].re * sign;
		}
}

// returns false when paused and nothing forced a recompute
bool water_update(Water* w, const Water_Config& c, float dt, bool force)
{
	if (w->n < 2) force = true;
	if (w->n < 2) water_build_spectrum(w, c);

	if (w->paused && !force) return false;

	float step = w->paused ? 0.f : dt * c.time_scale;
	w->time += step;

	int n = w->n;
	float k_min = -PI * (float)n / w->size;
	float dk    = TWOPI / w->size;

	// h(k,t) = h0 e^(iwt) + conj(h0(-k)) e^(-iwt)
	for (int z = 0; z < n; z++)
	{
		for (int x = 0; x < n; x++)
		{
			vec2 k = vec2(k_min + x * dk, k_min + z * dk);
			float omega = sqrtf(9.81f * length(k));
			float wt    = omega * w->time;

			Water_Complex e1 = { cosf(wt),  sinf(wt) };
			Water_Complex e2 = { cosf(wt), -sinf(wt) };

			int i = z * n + x;
			w->field[i] = w->h0[i] * e1 + w->h0_conj[i] * e2;
		}
	}

	// height
	for (int i = 0; i < n * n; i++) w->work[i] = w->field[i];
	water_ifft2(w->work, n);
	water_extract(w, w->height);

	// slopes and horizontal displacement, all from the same h(k,t)
	for (int pass = 0; pass < 4; pass++)
	{
		for (int z = 0; z < n; z++)
		{
			for (int x = 0; x < n; x++)
			{
				vec2 k = vec2(k_min + x * dk, k_min + z * dk);
				float len = length(k);
				int i = z * n + x;

				Water_Complex h = w->field[i];
				Water_Complex v = { 0, 0 };

				if (pass == 0)      v = h * Water_Complex{ 0.f, k.x };       // dh/dx
				else if (pass == 1) v = h * Water_Complex{ 0.f, k.y };       // dh/dz
				// +i, not -i : with this inverse transform's positive exponent
				// that pushes points toward the crest, so the jacobian compresses
				// there and the foam field forms on crests instead of troughs
				else if (len > 1e-6f && pass == 2)
					v = h * Water_Complex{ 0.f, k.x / len };                 // +i kx/|k| h
				else if (len > 1e-6f)
					v = h * Water_Complex{ 0.f, k.y / len };                 // +i kz/|k| h

				w->work[i] = v;
			}
		}

		water_ifft2(w->work, n);

		if (pass == 0)      water_extract(w, w->slope_x);
		else if (pass == 1) water_extract(w, w->slope_z);
		else if (pass == 2) water_extract(w, w->disp_x);
		else                water_extract(w, w->disp_z);
	}

	for (int i = 0; i < n * n; i++)
	{
		w->disp_x[i] *= c.choppiness;
		w->disp_z[i] *= c.choppiness;
	}

	// jacobian of the displaced surface : crests that fold get foam
	float spacing = w->size / (float)n;
	for (int z = 0; z < n; z++)
	{
		for (int x = 0; x < n; x++)
		{
			int xp = (x + 1) % n, xm = (x - 1 + n) % n;
			int zp = (z + 1) % n, zm = (z - 1 + n) % n;

			float dx_dx = (w->disp_x[z * n + xp] - w->disp_x[z * n + xm]) / (2.f * spacing);
			float dz_dz = (w->disp_z[zp * n + x] - w->disp_z[zm * n + x]) / (2.f * spacing);
			float dx_dz = (w->disp_x[zp * n + x] - w->disp_x[zm * n + x]) / (2.f * spacing);
			float dz_dx = (w->disp_z[z * n + xp] - w->disp_z[z * n + xm]) / (2.f * spacing);

			w->jacobian[z * n + x] = (1.f + dx_dx) * (1.f + dz_dz) - dx_dz * dz_dx;
		}
	}

	// foam : crests add, the field fades, the blur makes a soft field edge
	float amount = fminf(fmaxf(c.foam, 0.f), 1.f);
	float threshold = .60f + .35f * amount;
	float soft = .06f;
	float decay = (step > 0.f) ? expf(-step / fmaxf(c.foam_persistence, .05f)) : 1.f;

	for (int i = 0; i < n * n; i++)
	{
		float source = 0.f;
		if (amount > .001f)
		{
			float t = (w->jacobian[i] - (threshold - soft)) / (2.f * soft);
			t = fminf(fmaxf(t, 0.f), 1.f);
			t = t * t * (3.f - 2.f * t);
			source = 1.f - t;
		}

		w->foam[i] = fmaxf(w->foam[i] * decay, source);
	}

	for (int z = 0; z < n; z++)
	{
		for (int x = 0; x < n; x++)
		{
			float sum = 0.f;
			for (int dz = -1; dz <= 1; dz++)
				for (int dx = -1; dx <= 1; dx++)
				{
					int sx = (x + dx + n) % n;
					int sz = (z + dz + n) % n;
					sum += w->foam[sz * n + sx];
				}

			float value = fminf(fmaxf(sum / 9.f, 0.f), 1.f);
			w->foam_blur[z * n + x] = value;
			w->foam_pixels[z * n + x] = (unsigned char)(value * 255.f);
		}
	}

	return true;
}

// short-chop cascade : same phillips shape, a 16 m patch, normalized by slope
// energy so the normal map has the same bite in every sea state
void water_build_detail(Water_Detail* d, const Water_Config& c)
{
	int n = WATER_DETAIL_N;
	d->n = n;
	d->size = 16.f;

	float wind_speed = fminf(fmaxf(c.wind_speed * .35f, 3.f), 6.f);
	vec2  wind = vec2(cosf(c.wind_angle + .35f), sinf(c.wind_angle + .35f)) * wind_speed;

	float k_min = -PI * (float)n / d->size;
	float dk    = TWOPI / d->size;
	float slope_energy = 0.f;
	uint  rng = c.seed + 7919u;

	for (int z = 0; z < n; z++)
	{
		for (int x = 0; x < n; x++)
		{
			vec2 k = vec2(k_min + x * dk, k_min + z * dk);
			float p = water_phillips(k, wind, .001f);

			float g1, g2;
			water_gaussian2(&rng, &g1, &g2);

			float amp = sqrtf(p * .5f);
			d->h0[z * n + x] = { g1 * amp, g2 * amp };
			slope_energy += dot(k, k) * p;
		}
	}

	float scale = (slope_energy > 1e-12f)
		? .50f * (float)n * (float)n / sqrtf(2.f * slope_energy) : 0.f;
	for (int i = 0; i < n * n; i++) d->h0[i] = d->h0[i] * scale;

	for (int z = 0; z < n; z++)
	{
		for (int x = 0; x < n; x++)
		{
			int mx = (n - x) % n;
			int mz = (n - z) % n;
			d->h0_conj[z * n + x] = water_conj(d->h0[mz * n + mx]);
		}
	}
}

void water_update_detail(Water_Detail* d, float time)
{
	int n = d->n;
	if (n < 2) return;

	float k_min = -PI * (float)n / d->size;
	float dk    = TWOPI / d->size;

	for (int z = 0; z < n; z++)
	{
		for (int x = 0; x < n; x++)
		{
			vec2 k = vec2(k_min + x * dk, k_min + z * dk);
			float omega = sqrtf(9.81f * length(k));
			float wt = omega * time;

			Water_Complex e1 = { cosf(wt),  sinf(wt) };
			Water_Complex e2 = { cosf(wt), -sinf(wt) };

			int i = z * n + x;
			d->field[i] = d->h0[i] * e1 + d->h0_conj[i] * e2;
		}
	}

	for (int pass = 0; pass < 2; pass++)
	{
		for (int z = 0; z < n; z++)
		{
			for (int x = 0; x < n; x++)
			{
				vec2 k = vec2(k_min + x * dk, k_min + z * dk);
				int i = z * n + x;
				float kk = (pass == 0) ? k.x : k.y;
				d->work[i] = d->field[i] * Water_Complex{ 0.f, kk };
			}
		}

		water_ifft2(d->work, n);

		for (int z = 0; z < n; z++)
		{
			for (int x = 0; x < n; x++)
			{
				float sign = ((x + z) & 1) ? -1.f : 1.f;
				int i = z * n + x;
				float value = d->work[i].re * sign;
				if (pass == 0) d->slope_x[i] = value;
				else           d->slope_z[i] = value;
			}
		}
	}

	for (int i = 0; i < n * n; i++)
	{
		float sx = d->slope_x[i];
		float sz = d->slope_z[i];
		float inv = 1.f / sqrtf(sx * sx + 1.f + sz * sz);

		float nx = -sx * inv;
		float nz = -sz * inv;

		d->normals[i * 2 + 0] = (unsigned char)((nx * .5f + .5f) * 255.f);
		d->normals[i * 2 + 1] = (unsigned char)((nz * .5f + .5f) * 255.f);
	}
}

// ---- mesh ----

// one water vertex : the grid is periodic, so i == n samples the seam at i == 0
void water_mesh_vertex(const Water* w, int i, int j, Mesh_Data* mesh, uint v)
{
	int n = w->n;
	int k = (j % n) * n + (i % n);

	float x = ((float)i / (float)n - .5f) * w->size + w->disp_x[k];
	float z = ((float)j / (float)n - .5f) * w->size + w->disp_z[k];

	mesh->positions[v] = vec3(x, w->height[k], z);
	mesh->normals[v]   = normalize(vec3(-w->slope_x[k], 1.f, -w->slope_z[k]));
	mesh->uvs[v]       = vec2((float)i / (float)n, (float)j / (float)n);
}

// one periodic patch : (n+1)^2 vertices so the seam is a real edge
Mesh_Data water_build_mesh(const Water* w)
{
	Mesh_Data mesh = {};
	int n = w->n;
	if (n < 2) return mesh;

	int vn = n + 1;
	uint num_vertices = (uint)(vn * vn);
	uint num_indices  = (uint)(n * n * 6);

	mesh.positions = Alloc(vec3, num_vertices);
	mesh.normals   = Alloc(vec3, num_vertices);
	mesh.uvs       = Alloc(vec2, num_vertices);
	mesh.indices   = Alloc(uint, num_indices);

	for (int j = 0; j < vn; j++)
		for (int i = 0; i < vn; i++)
			water_mesh_vertex(w, i, j, &mesh, (uint)(j * vn + i));

	uint e = 0;
	for (int j = 0; j < n; j++)
	{
		for (int i = 0; i < n; i++)
		{
			uint a = (uint)(j * vn + i);
			uint b = a + 1;
			uint c = a + vn;
			uint d = c + 1;

			mesh.indices[e++] = a;  mesh.indices[e++] = c; mesh.indices[e++] = b;
			mesh.indices[e++] = b;  mesh.indices[e++] = c; mesh.indices[e++] = d;
		}
	}

	mesh.num_vertices = num_vertices;
	mesh.num_indices  = num_indices;
	return mesh;
}


// ---- shallow water (finite difference) ----

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
	vec3  albedo = vec3(.030f, .200f, .215f);
	float roughness = .045f;
	float sss = .75f; // backlit crest glow
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
