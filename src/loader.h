#include "logger.h"

/* ------------------------- *
	-    Asset Loading      -
 * ------------------------- */

const uint MAX_MESHES   = 128;
const uint MAX_TEXTURES = 16;

struct Mesh_Data
{
	uint num_vertices, num_indices;

	vec3* positions, *normals;
	vec2* uvs;
	uint* indices;

	void release()
	{
		free(positions);
		free(normals);
		free(uvs);
		free(indices);
	}
};

// the same mesh without uvs : for geometry that is only ever shaded as a shape
// (the light volumes), so there is no texture column to fill
struct Mesh_Data_NoUV
{
	uint num_vertices, num_indices;

	vec3* positions;
	vec3* normals;
	uint* indices;

	void release()
	{
		free(positions);
		free(normals);
		free(indices);
	}
};

struct Texture_Data // alignas(64)
{
	uint id;
	GLuint handle;
	int width, height; // filled by TextureLoader::load, shown in the console
	char path[56];
};

struct TextureLoader
{
	uint num_textures;
	Texture_Data textures[MAX_TEXTURES];

	// load a texture into the next slot : same path twice = same id back (cached).
	// returns the texture id (0 = default texture, error already logged)
	uint load(const char* path)
	{
		for (uint i = 0; i < num_textures; i++)
		{
			if (strcmp(textures[i].path, path) == 0) // 0 = strings match!
				return textures[i].id;
		}

		if (num_textures >= MAX_TEXTURES)
		{
			console->add_entry("TextureLoader : MAX_TEXTURE limit reached", FIXME, RNDR);
			return 0;
		}

		uint i = num_textures; // for convenience

		glActiveTexture(GL_TEXTURE0);
		glGenTextures(1, &textures[i].handle);
		glBindTexture(GL_TEXTURE_2D, textures[i].handle);

		// Texture wrapping/filtering options (on currently bound texture object)
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

		// load & generate texture
		int width, height, num_channels;
		unsigned char* data = stbi_load(path, &width, &height, &num_channels, 0);
		if (data)
		{
			// match the upload to the file's channel count : a grayscale mask is
			// one byte per texel, not three
			GLint internal = GL_RGB;
			GLenum format = GL_RGB;
			if (num_channels == 1)      { internal = GL_R8;   format = GL_RED;  }
			else if (num_channels == 2) { internal = GL_RG8;  format = GL_RG;   }
			else if (num_channels == 4) { internal = GL_RGBA; format = GL_RGBA; }

			glTexImage2D(GL_TEXTURE_2D, 0, internal, width, height, 0, format, GL_UNSIGNED_BYTE, data);
			glGenerateMipmap(GL_TEXTURE_2D);
		}
		else
		{
			char msg[MAX_LOG_MSG_LENGTH];
			snprintf(msg, MAX_LOG_MSG_LENGTH, "failed to load texture : %s", path);
			console->add_entry(msg, FIXME, RNDR);
			stbi_image_free(data);
			glDeleteTextures(1, &textures[i].handle);
			return 0; // don't register a texture that never loaded
		}

		stbi_image_free(data);

		textures[i].id = num_textures;
		textures[i].width  = width;
		textures[i].height = height;
		snprintf(textures[i].path, sizeof(textures[i].path), "%s", path);
		num_textures   = num_textures + 1;

		char msg[MAX_LOG_MSG_LENGTH] = {};
		snprintf(msg, MAX_LOG_MSG_LENGTH, "Loaded Texture : [%s], Size : [%d] bytes", path, width * height * num_channels);
		console->add_entry(msg, SUCCESS, RNDR);
		return textures[i].id;
	}
	// empty R8 texture, filled by upload_dynamic : the water foam field
	uint create_dynamic(const char* path, int width, int height)
	{
		for (uint i = 0; i < num_textures; i++)
			if (strcmp(textures[i].path, path) == 0)
				return textures[i].id;

		if (num_textures >= MAX_TEXTURES)
		{
			console->add_entry("TextureLoader : MAX_TEXTURE limit reached", FIXME, RNDR);
			return 0;
		}

		uint i = num_textures;
		glActiveTexture(GL_TEXTURE0);
		glGenTextures(1, &textures[i].handle);
		glBindTexture(GL_TEXTURE_2D, textures[i].handle);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_R8, width, height, 0, GL_RED, GL_UNSIGNED_BYTE, NULL);

		textures[i].id = num_textures;
		textures[i].width  = width;
		textures[i].height = height;
		snprintf(textures[i].path, sizeof(textures[i].path), "%s", path);
		num_textures++;

		char msg[MAX_LOG_MSG_LENGTH];
		snprintf(msg, MAX_LOG_MSG_LENGTH, "Dynamic Texture : [%s], %d x %d", path, width, height);
		console->add_entry(msg, SUCCESS, RNDR);
		return textures[i].id;
	}

	void upload_dynamic(uint id, const unsigned char* pixels, int width, int height)
	{
		for (uint i = 0; i < num_textures; i++)
		{
			if (textures[i].id == id)
			{
				glBindTexture(GL_TEXTURE_2D, textures[i].handle);
				glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
				glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, width, height,
				                GL_RED, GL_UNSIGNED_BYTE, pixels);
				glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
				return;
			}
		}
	}

	// two channel dynamic texture : the short-chop normal map
	uint create_dynamic_rg(const char* path, int width, int height)
	{
		for (uint i = 0; i < num_textures; i++)
			if (strcmp(textures[i].path, path) == 0)
				return textures[i].id;

		if (num_textures >= MAX_TEXTURES)
		{
			console->add_entry("TextureLoader : MAX_TEXTURE limit reached", FIXME, RNDR);
			return 0;
		}

		uint i = num_textures;
		glActiveTexture(GL_TEXTURE0);
		glGenTextures(1, &textures[i].handle);
		glBindTexture(GL_TEXTURE_2D, textures[i].handle);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RG8, width, height, 0, GL_RG, GL_UNSIGNED_BYTE, NULL);

		textures[i].id = num_textures;
		textures[i].width  = width;
		textures[i].height = height;
		snprintf(textures[i].path, sizeof(textures[i].path), "%s", path);
		num_textures++;

		char msg[MAX_LOG_MSG_LENGTH];
		snprintf(msg, MAX_LOG_MSG_LENGTH, "Dynamic Texture : [%s], %d x %d", path, width, height);
		console->add_entry(msg, SUCCESS, RNDR);
		return textures[i].id;
	}

	void upload_dynamic_rg(uint id, const unsigned char* pixels, int width, int height)
	{
		for (uint i = 0; i < num_textures; i++)
		{
			if (textures[i].id == id)
			{
				glBindTexture(GL_TEXTURE_2D, textures[i].handle);
				glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
				glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, width, height,
				                GL_RG, GL_UNSIGNED_BYTE, pixels);
				glGenerateMipmap(GL_TEXTURE_2D);
				glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
				return;
			}
		}
	}

	// delete every texture and forget the slots : init() reloads the default
	void clear()
	{
		for (uint i = 0; i < num_textures; i++)
			glDeleteTextures(1, &textures[i].handle);
		num_textures = 0;
		memset(textures, 0, sizeof(textures));
	}
	void bind(uint id, GLuint active_texture = 0)
	{
		glActiveTexture(GL_TEXTURE0 + active_texture);
		for (uint i = 0; i < MAX_TEXTURES; i++)
		{
			if (textures[i].id == id)
			{
				glBindTexture(GL_TEXTURE_2D, textures[i].handle);
				return;
			}
		}

		// not found : log once (this runs every frame, don't spam the ring buffer)
		static uint last_missing_id = (uint)-1;
		if (id != last_missing_id)
		{
			char msg[MAX_LOG_MSG_LENGTH];
			snprintf(msg, MAX_LOG_MSG_LENGTH, "Texture with id [%d] not found!", id);
			console->add_entry(msg, FIXME, RNDR);
			last_missing_id = id;
		}
	}
	void init(const char* default_texture_path = "assets/textures/default.jpg")
	{
		load(default_texture_path); // load default texture so we always have one to revert to
	}
};

// Shaders

struct ShaderProgram
{
	GLuint id;

	// query results, filled by create() and shown in the console (Assets tab)
	GLint link_status;
	GLint num_uniforms;
	GLint num_attributes;

	void create(const char* vert_path, const char* frag_path)
	{
		char* vert_source = (char*)read_text_file_into_memory(vert_path);
		char* frag_source = (char*)read_text_file_into_memory(frag_path);

		GLuint vert_shader = glCreateShader(GL_VERTEX_SHADER);
		glShaderSource(vert_shader, 1, &vert_source, NULL);
		glCompileShader(vert_shader);

		GLuint frag_shader = glCreateShader(GL_FRAGMENT_SHADER);
		glShaderSource(frag_shader, 1, &frag_source, NULL);
		glCompileShader(frag_shader);

		free(vert_source);
		free(frag_source);

		// verify successful shader compilation
		{
			GLint compiled = 0;
			glGetShaderiv(vert_shader, GL_COMPILE_STATUS, &compiled);
			if (!compiled)
				log_shader_error(vert_shader, "VERTEX SHADER", vert_path);

			compiled = 0;
			glGetShaderiv(frag_shader, GL_COMPILE_STATUS, &compiled);
			if (!compiled)
				log_shader_error(frag_shader, "FRAGMENT SHADER", frag_path);
		}

		id = glCreateProgram();
		glAttachShader(id, vert_shader);
		glAttachShader(id, frag_shader);
		glLinkProgram(id);

		glGetProgramiv(id, GL_LINK_STATUS, &link_status);
		glGetProgramiv(id, GL_ACTIVE_UNIFORMS,  &num_uniforms);
		glGetProgramiv(id, GL_ACTIVE_ATTRIBUTES, &num_attributes);

		if (!link_status)
		{
			GLsizei length = 0;
			char error[256] = {};
			glGetProgramInfoLog(id, 256, &length, error);

			char msg[MAX_LOG_MSG_LENGTH];
			snprintf(msg, MAX_LOG_MSG_LENGTH, "LINK FAILED [%s]: %.40s", vert_path, error);
			console->add_entry(msg, FIXME, RNDR);
		}

		glDeleteShader(vert_shader);
		glDeleteShader(frag_shader);
	}

	// pull the compiler log for a failed shader into the in-app console
	static void log_shader_error(GLuint shader, const char* label, const char* path)
	{
		GLint log_size = 0;
		glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &log_size);

		char msg[MAX_LOG_MSG_LENGTH] = {};
		if (log_size > 0)
		{
			char* error_log = (char*)calloc(log_size + 1, sizeof(char));
			glGetShaderInfoLog(shader, log_size, NULL, error_log);

			// first line only : the console entry is 62 chars
			for (int i = 0; error_log[i]; i++)
				if (error_log[i] == '\n') { error_log[i] = 0; break; }

			snprintf(msg, MAX_LOG_MSG_LENGTH, "%.8s FAILED [%.20s]: %.28s", label, path, error_log);
			free(error_log);
		}
		else
		{
			snprintf(msg, MAX_LOG_MSG_LENGTH, "%.8s FAILED [%.40s]", label, path);
		}

		console->add_entry(msg, FIXME, RNDR);
	}
	void bind() { glUseProgram(id); }
	void destroy() { glDeleteProgram(id); }

	// WARNING : bind shader *before* calling these!
	void set_int  (const char* name, int   value) { glUniform1i(glGetUniformLocation(id, name), value); }
	void set_float(const char* name, float value) { glUniform1f(glGetUniformLocation(id, name), value); }
	void set_vec2 (const char* name, vec2  value) { glUniform2f(glGetUniformLocation(id, name), value.x, value.y); }
	void set_vec3 (const char* name, vec3  value) { glUniform3f(glGetUniformLocation(id, name), value.x, value.y, value.z); }
	void set_mat4 (const char* name, mat4  value) { glUniformMatrix4fv(glGetUniformLocation(id, name), 1, GL_FALSE, (float*)&value); }
};

// Mesh Generators

namespace MeshFactory {

	// cube : 24 verts (per-face normals + uvs), edge length = size, centered
	Mesh_Data generate_cube(float size)
	{
		Mesh_Data data = {};
		data.num_vertices = 24;
		data.num_indices  = 36;

		data.positions = Alloc(vec3, 24);
		data.normals   = Alloc(vec3, 24);
		data.uvs       = Alloc(vec2, 24);
		data.indices   = Alloc(uint, 36);

		uint v = 0, idx = 0;
		for (uint face = 0; face < 6; face++)
		{
			uint axis = face / 2;                       // 0 = x, 1 = y, 2 = z
			float sign = (face & 1) ? -1.f : 1.f;
			uint  b = (axis + 1) % 3, c = (axis + 2) % 3; // cyclic in-plane axes

			vec3 n = vec3(0);
			n[axis] = sign;

			// quad corners, ccw seen from outside : (-b,-c) (+b,-c) (+b,+c) (-b,+c)
			for (uint k = 0; k < 4; k++)
			{
				vec3 p = vec3(0);
				p[axis] = sign;
				p[b] = (k == 0 || k == 3) ? -1.f : 1.f;
				p[c] = (k < 2) ? -1.f : 1.f;

				data.positions[v + k] = p * (size * 0.5f);
				data.normals[v + k]   = n;
				data.uvs[v + k]       = vec2((k == 0 || k == 3) ? 0.f : 1.f, (k < 2) ? 0.f : 1.f);
			}

			// negative faces flip the quad, so flip the triangles back to ccw-outward
			uint tri[6] = { 0, 1, 2, 0, 2, 3 };
			if (sign < 0) { tri[1] = 2; tri[2] = 1; tri[4] = 3; tri[5] = 2; }

			for (uint k = 0; k < 6; k++)
				data.indices[idx++] = v + tri[k];

			v += 4;
		}

		return data;
	}

	// lat-long sphere : the light volume. no uvs, the light shader only needs shape
	Mesh_Data_NoUV generate_sphere(float radius, uint rings, uint segments)
	{
		Mesh_Data_NoUV data = {};
		uint vn = (rings + 1) * (segments + 1);

		data.num_vertices = vn;
		data.num_indices  = rings * segments * 6;
		data.positions = Alloc(vec3, vn);
		data.normals   = Alloc(vec3, vn);
		data.indices   = Alloc(uint, data.num_indices);

		for (uint iy = 0; iy <= rings; iy++)
		{
			float theta = (float)iy / (float)rings * PI;
			float y = cosf(theta), r = sinf(theta);

			for (uint ix = 0; ix <= segments; ix++)
			{
				float phi = (float)ix / (float)segments * 2.f * PI;
				vec3 n = vec3(r * cosf(phi), y, r * sinf(phi));
				data.positions[iy * (segments + 1) + ix] = n * radius;
				data.normals  [iy * (segments + 1) + ix] = n;
			}
		}

		uint e = 0;
		for (uint iy = 0; iy < rings; iy++)
			for (uint ix = 0; ix < segments; ix++)
			{
				uint a = iy * (segments + 1) + ix;
				uint b = a + 1;
				uint c = a + segments + 1;
				uint d = c + 1;

				data.indices[e++] = a; data.indices[e++] = b; data.indices[e++] = c;
				data.indices[e++] = b; data.indices[e++] = d; data.indices[e++] = c;
			}

		return data;
	}

	// unit quad for full-screen passes (gbuf lighting), lying in the xz plane
	Mesh_Data generate_screenquad()
	{
		Mesh_Data data = {};
	
		vec3 positions[] =
		{
			{ -1.f, 0, -1.f }, // 0  1-------3
			{ -1.f, 0,  1.f }, // 1  |       |
			{  1.f, 0, -1.f }, // 2  |       |
			{  1.f, 0,  1.f }  // 3  0-------2
		};
	
		vec3 normals[] =
		{
			{ 0.f, 0.f, 1.f },
			{ 0.f, 0.f, 1.f },
			{ 0.f, 0.f, 1.f },
			{ 0.f, 0.f, 1.f }
		};
	
		vec2 uvs[] =
		{
			{ 0.f, 0.f }, // 0  1-------3
			{ 0.f, 1.f }, // 1  |       |
			{ 1.f, 0.f }, // 2  |       |
			{ 1.f, 1.f }  // 3  0-------2
		};
	
		uint indices[] =
		{
			0,2,3,
			3,1,0
		};
	
		data.num_indices  = sizeof(indices)   / sizeof(uint);
		data.num_vertices = sizeof(positions) / sizeof(vec3);
	
		data.positions = Alloc(vec3, data.num_vertices);
		data.normals   = Alloc(vec3, data.num_vertices);
		data.uvs       = Alloc(vec2, data.num_vertices);
		data.indices   = Alloc(uint, data.num_indices);
	
		memcpy(data.positions, positions, sizeof(positions));
		memcpy(data.normals  , normals  , sizeof(normals)  );
		memcpy(data.uvs      , uvs      , sizeof(uvs)      );
		memcpy(data.indices  , indices  , sizeof(indices)  );
	
		return data;
	}
	// flat grid plane, centered : resolution = quads per side, ccw winding facing the normal
	enum { PLANE_XZ, PLANE_XY, PLANE_YZ }; // which world plane it lies in
	Mesh_Data generate_plane(float size, uint resolution, uint orientation)
	{
		Mesh_Data data = {};
		if (resolution < 1) resolution = 1;

		uint vert_count = (resolution + 1) * (resolution + 1);
		data.num_vertices = vert_count;
		data.num_indices  = resolution * resolution * 6;

		data.positions = Alloc(vec3, vert_count);
		data.normals   = Alloc(vec3, vert_count);
		data.uvs       = Alloc(vec2, vert_count);
		data.indices   = Alloc(uint, data.num_indices);

		// the two in-plane axes + the normal the orientation faces
		vec3 axis_u, axis_v, normal;
		switch (orientation)
		{
			case PLANE_XY: axis_u = { 1, 0, 0 }; axis_v = { 0, 1, 0 }; normal = { 0, 0, 1 }; break;
			case PLANE_YZ: axis_u = { 0, 1, 0 }; axis_v = { 0, 0, 1 }; normal = { 1, 0, 0 }; break;
			default:       axis_u = { 1, 0, 0 }; axis_v = { 0, 0, 1 }; normal = { 0, 1, 0 }; break; // ground
		}

		float step  = size / resolution;
		float from  = -size * 0.5f; // first vertex offset : centered on the origin
		uint v = 0;
		for (uint z = 0; z <= resolution; z++)
		{
			for (uint x = 0; x <= resolution; x++)
			{
				data.positions[v] = axis_u * (from + x * step) + axis_v * (from + z * step);
				data.normals[v]   = normal;
				data.uvs[v]       = vec2(x / (float)resolution, z / (float)resolution);
				v++;
			}
		}

		uint idx = 0;
		for (uint z = 0; z < resolution; z++)
		{
			for (uint x = 0; x < resolution; x++)
			{
				uint a = z * (resolution + 1) + x;
				uint b = a + 1;
				uint c = a + resolution + 1;
				uint d = c + 1;
				// ccw winding facing the plane's normal
				data.indices[idx++] = a; data.indices[idx++] = c; data.indices[idx++] = b;
				data.indices[idx++] = b; data.indices[idx++] = c; data.indices[idx++] = d;
			}
		}

		return data;
	}

	// uv sphere : rings around y, seam column duplicated for clean uvs. radius
	// sets the size, so the caller scales what it needs
	Mesh_Data generate_uv_sphere(float radius, uint slices, uint stacks)
	{
		Mesh_Data data = {};
		if (slices < 3) slices = 3;
		if (stacks < 2) stacks = 2;

		uint vert_count = (stacks + 1) * (slices + 1);
		data.num_vertices = vert_count;
		data.num_indices  = stacks * slices * 6;

		data.positions = Alloc(vec3, vert_count);
		data.normals   = Alloc(vec3, vert_count);
		data.uvs       = Alloc(vec2, vert_count);
		data.indices   = Alloc(uint, data.num_indices);

		// rings from the top pole down, columns wrap around y
		uint v = 0;
		for (uint i = 0; i <= stacks; i++)
		{
			float phi    = PI * i / stacks;
			float ring   = sinf(phi);
			float height = cosf(phi);

			for (uint j = 0; j <= slices; j++)
			{
				float theta = 2.f * PI * j / slices;

				vec3 normal = vec3(ring * cosf(theta), height, ring * sinf(theta));
				data.positions[v] = normal * radius;
				data.normals[v]   = normal;
				data.uvs[v]       = vec2(j / (float)slices, 1.f - i / (float)stacks);
				v++;
			}
		}

		// two ccw triangles per quad, wound to face outward
		uint idx = 0;
		for (uint i = 0; i < stacks; i++)
		{
			for (uint j = 0; j < slices; j++)
			{
				uint upper = i * (slices + 1) + j;
				uint lower = upper + slices + 1;

				data.indices[idx++] = upper;
				data.indices[idx++] = upper + 1;
				data.indices[idx++] = lower;

				data.indices[idx++] = upper + 1;
				data.indices[idx++] = lower + 1;
				data.indices[idx++] = lower;
			}
		}

		return data;
	}

	// capsule : sphere caps on a cylinder, rings top to bottom, seam duplicated.
	// height = the cylinder part only : bullet's convention, colliders match
	Mesh_Data generate_capsule(float radius, float height, uint slices, uint stacks)
	{
		Mesh_Data data = {};
		if (slices < 3) slices = 3;
		if (stacks < 1) stacks = 1;
		if (height < 0) height = 0; // degenerate height = a sphere

		uint rings = stacks * 2 + 2; // top cap + cylinder + bottom cap
		uint verts = rings * (slices + 1);
		data.num_vertices = verts;
		data.num_indices  = (rings - 1) * slices * 6;

		data.positions = Alloc(vec3, verts);
		data.normals   = Alloc(vec3, verts);
		data.uvs       = Alloc(vec2, verts);
		data.indices   = Alloc(uint, data.num_indices);

		float half = height * 0.5f;
		uint v = 0;
		for (uint ring = 0; ring < rings; ring++)
		{
			// profile latitude + cap center : 0..stacks = top cap, stacks+1 =
			// cylinder, the rest = bottom cap
			float phi    = (PI * 0.5f) * (ring / (float)stacks);
			float center = half;

			if (ring == stacks + 1)
			{
				phi    = PI * 0.5f;
				center = -half;
			}
			else if (ring > stacks + 1)
			{
				phi    = (PI * 0.5f) * (1.f + (ring - stacks - 1) / (float)stacks);
				center = -half;
			}

			float cap_r = sinf(phi) * radius;          // this ring's radius
			float y     = center + cosf(phi) * radius; // and its height

			for (uint j = 0; j <= slices; j++)
			{
				float theta = 2.f * PI * j / slices;

				data.positions[v] = vec3(cap_r * cosf(theta), y, cap_r * sinf(theta));
				data.normals[v]   = vec3(sinf(phi) * cosf(theta), cosf(phi), sinf(phi) * sinf(theta));
				data.uvs[v]       = vec2(j / (float)slices, ring / (float)(rings - 1));
				v++;
			}
		}

		// two ccw triangles per quad, same outward winding as the sphere
		uint idx = 0;
		for (uint ring = 0; ring < rings - 1; ring++)
		{
			for (uint j = 0; j < slices; j++)
			{
				uint upper = ring * (slices + 1) + j;
				uint lower = upper + slices + 1;

				data.indices[idx++] = upper;
				data.indices[idx++] = upper + 1;
				data.indices[idx++] = lower;

				data.indices[idx++] = upper + 1;
				data.indices[idx++] = lower + 1;
				data.indices[idx++] = lower;
			}
		}

		return data;
	}

	// cone : radius + full height, y aligned, centered like bullet's btConeShape
	Mesh_Data generate_cone(float radius, float height, uint slices)
	{
		Mesh_Data data = {};
		if (slices < 3) slices = 3;

		float half = height * 0.5f;
		data.num_vertices = 2 * (slices + 2); // side : ring + apex | base : center + rim
		data.num_indices  = slices * 6;

		data.positions = Alloc(vec3, data.num_vertices);
		data.normals   = Alloc(vec3, data.num_vertices);
		data.uvs       = Alloc(vec2, data.num_vertices);
		data.indices   = Alloc(uint, data.num_indices);

		uint v = 0, idx = 0;

		// side : base ring (seam column duplicated) then the apex
		for (uint j = 0; j <= slices; j++)
		{
			float theta = 2.f * PI * j / slices;
			vec2 dir = vec2(cosf(theta), sinf(theta));

			data.positions[v] = vec3(dir.x * radius, -half, dir.y * radius);
			// slanted normal : radial by height, vertical by radius
			data.normals[v]   = normalize(vec3(dir.x * height, radius, dir.y * height));
			data.uvs[v]       = vec2(j / (float)slices, 1.f);
			v++;
		}
		uint apex = v++;
		data.positions[apex] = vec3(0, half, 0);
		data.normals[apex]   = vec3(0, 1, 0);
		data.uvs[apex]       = vec2(0.5f, 0.f);

		for (uint j = 0; j < slices; j++) // apex first, ring winds back (ccw out)
		{
			data.indices[idx++] = apex;
			data.indices[idx++] = j + 1;
			data.indices[idx++] = j;
		}

		// base : center fan, wound to face down like the cylinder's bottom cap
		uint center = v++;
		data.positions[center] = vec3(0, -half, 0);
		data.normals[center]   = vec3(0, -1, 0);
		data.uvs[center]       = vec2(0.5f, 0.5f);

		uint rim = v;
		for (uint j = 0; j <= slices; j++)
		{
			float theta = 2.f * PI * j / slices;

			data.positions[v] = vec3(cosf(theta) * radius, -half, sinf(theta) * radius);
			data.normals[v]   = vec3(0, -1, 0);
			data.uvs[v]       = vec2(0.5f + 0.5f * cosf(theta), 0.5f + 0.5f * sinf(theta));
			v++;
		}

		for (uint j = 0; j < slices; j++)
		{
			data.indices[idx++] = center;
			data.indices[idx++] = rim + j;
			data.indices[idx++] = rim + j + 1;
		}

		return data;
	}

	// cylinder : smooth-walled, flat disc caps. height is the full height, centered
	Mesh_Data generate_cylinder(float radius, float height, uint slices)
	{
		Mesh_Data data = {};
		if (slices < 3) slices = 3;

		uint verts = 2 * (slices + 1)   // wall, seam column duplicated
		           + 2 * (slices + 2); // both caps : rim + one center vert
		data.num_vertices = verts;
		data.num_indices  = slices * 12; // wall quads + two cap fans

		data.positions = Alloc(vec3, verts);
		data.normals   = Alloc(vec3, verts);
		data.uvs       = Alloc(vec2, verts);
		data.indices   = Alloc(uint, data.num_indices);

		float half = height * 0.5f;
		uint v = 0, idx = 0;

		// wall : two rings, top then bottom (same winding as the sphere's quads)
		for (uint ring = 0; ring < 2; ring++)
		{
			float y = ring ? -half : half;

			for (uint j = 0; j <= slices; j++)
			{
				float theta = 2.f * PI * j / slices;
				vec2 dir = vec2(cosf(theta), sinf(theta)); // radial direction

				data.positions[v] = vec3(dir.x * radius, y, dir.y * radius);
				data.normals[v]   = vec3(dir.x, 0, dir.y);
				data.uvs[v]       = vec2(j / (float)slices, (float)ring);
				v++;
			}
		}

		for (uint j = 0; j < slices; j++)
		{
			uint upper = j, lower = slices + 1 + j;

			data.indices[idx++] = upper;
			data.indices[idx++] = upper + 1;
			data.indices[idx++] = lower;

			data.indices[idx++] = upper + 1;
			data.indices[idx++] = lower + 1;
			data.indices[idx++] = lower;
		}

		// caps : a center fan each, wound to face away from the cylinder
		for (uint cap = 0; cap < 2; cap++)
		{
			float  y     = cap ? -half : half;
			float  sign  = cap ? -1.f : 1.f; // outward along y
			uint   center = v++;

			data.positions[center] = vec3(0, y, 0);
			data.normals[center]   = vec3(0, sign, 0);
			data.uvs[center]       = vec2(0.5f, 0.5f);

			uint rim = v;
			for (uint j = 0; j <= slices; j++)
			{
				float theta = 2.f * PI * j / slices;

				data.positions[v] = vec3(cosf(theta) * radius, y, sinf(theta) * radius);
				data.normals[v]   = vec3(0, sign, 0);
				data.uvs[v]       = vec2(0.5f + 0.5f * cosf(theta), 0.5f + 0.5f * sinf(theta));
				v++;
			}

			// top fan winds rim j+1 first (ccw from above) ; the bottom flips
			for (uint j = 0; j < slices; j++)
			{
				data.indices[idx++] = center;
				data.indices[idx++] = cap ? rim + j : rim + j + 1;
				data.indices[idx++] = cap ? rim + j + 1 : rim + j;
			}
		}

		return data;
	}
}
