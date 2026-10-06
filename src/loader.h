#include "logger.h"

/* ------------------------- *
	-    Asset Loading      -
 * ------------------------- */

const uint MAX_FILEPATH_LENGTH = 64;
const uint MAX_MESHES   = 16;
const uint MAX_SHADERS  = 16;
const uint MAX_TEXTURES = 16;

struct Mesh_Data
{
	uint num_vertices, num_indices;

	vec3* positions, *normals;
	vec2* uvs;
	uint* indices;

	void load(const char* path)
	{
		FILE* mesh_file = fopen(path, "rb"); // rb = read binary
		if (!mesh_file)
		{
			char msg[MAX_LOG_MSG_LENGTH];
			snprintf(msg, MAX_LOG_MSG_LENGTH, "could not open model file : %s", path);
			console->add_entry(msg, FIXME, RNDR);
			return;
		}

		fread(&num_vertices, sizeof(uint), 1, mesh_file);
		fread(&num_indices , sizeof(uint), 1, mesh_file);

		positions = (vec3*)calloc(num_vertices, sizeof(vec3));
		normals   = (vec3*)calloc(num_vertices, sizeof(vec3));
		uvs       = (vec2*)calloc(num_vertices, sizeof(vec2));
		indices   = (uint*)calloc(num_indices , sizeof(uint));

		fread(positions, sizeof(vec3), num_vertices, mesh_file);
		fread(normals  , sizeof(vec3), num_vertices, mesh_file);
		fread(uvs      , sizeof(vec2), num_vertices, mesh_file);
		fread(indices  , sizeof(uint), num_indices , mesh_file);

		fclose(mesh_file);
	}
	void release()
	{
		free(positions);
		free(normals);
		free(uvs);
		free(indices);
	}
};

struct MeshLoader
{
	uint num_cached; // number of meshes currently stored in MeshLoader
	uint num_loaded; // total number of meshes loaded since program was launched; used for id generation

	struct MeshInfo
	{
		uint id, size, is_cached;
		char filepath[MAX_FILEPATH_LENGTH];
	} meshes[MAX_MESHES];

	void load_mesh(const char* filepath, uint* id) // returns mesh_id
	{
		// check if previously cached
		for (uint i = 0; i < MAX_MESHES; i++)
		{
			if (strcmp(meshes[i].filepath, filepath) == 0) // 0 = strings match!
			{
				out(filepath << " already cached!");
				if (id) *id = meshes[i].id;
				return;
			}
		}

		// if not already cached, then load from disk & add to cache
		FILE* mesh_file = fopen(filepath, "rb");
		if (!mesh_file)
		{
			char msg[MAX_LOG_MSG_LENGTH];
			snprintf(msg, MAX_LOG_MSG_LENGTH, "could not open model file : %s", filepath);
			console->add_entry(msg, FIXME, RNDR);
			return; // *id is left untouched (0) : caller checks for this
		}

		uint num_vertices = 0, num_indices = 0;
		fread(&num_vertices, sizeof(uint), 1, mesh_file);
		fread(&num_indices , sizeof(uint), 1, mesh_file);

		fclose(mesh_file);

		// look for empty spot
		for (uint i = 0; i < MAX_MESHES; i++)
		{
			if (meshes[i].id == 0)
			{
				uint filepath_length = strlen(filepath);
				if (filepath_length >= MAX_FILEPATH_LENGTH)
				{
					char msg[MAX_LOG_MSG_LENGTH];
					snprintf(msg, MAX_LOG_MSG_LENGTH, "mesh filepath too long (%d > %d) : %s",
						filepath_length, MAX_FILEPATH_LENGTH, filepath);
					console->add_entry(msg, FIXME, RNDR);
					return;
				}

				strcpy(meshes[i].filepath, filepath);

				meshes[i].id = num_loaded + 1; // avoid NULL value
				meshes[i].is_cached = true; // TODO : is this redundant if we use the filepath to check for caching?
				meshes[i].size = get_file_size(filepath); // disk bytes, shown in the console

				// update meta-data
				num_cached++;
				num_loaded++;

				if (id) *id = meshes[i].id;
				return;
			}
		}

		console->add_entry("mesh cache full (MAX_MESHES)", FIXME, RNDR);
	}
	void load_mesh_data(uint mesh_id, Mesh_Data* data)
	{
		if (mesh_id == 0) { console->add_entry("load_mesh_data : invalid mesh id 0", FIXME, RNDR); return; }

		for (uint i = 0; i < MAX_MESHES; i++)
		{
			if (meshes[i].id == mesh_id && data) // data cannot be nullptr
			{
				data->load(meshes[i].filepath);
				return;
			}
		}

		char msg[MAX_LOG_MSG_LENGTH];
		snprintf(msg, MAX_LOG_MSG_LENGTH, "load_mesh_data : mesh id [%d] not found", mesh_id);
		console->add_entry(msg, FIXME, RNDR);
	}
};

// Animated Meshes

struct Mesh_Data_Anim
{
	uint num_vertices, num_indices;

	vec3*  positions, *normals, *weights;
	ivec3* bones;
	vec2*  uvs;
	uint*  indices;

	void load(const char* path)
	{
		FILE* mesh_file = fopen(path, "rb"); // rb = read binary
		if (!mesh_file)
		{
			char msg[MAX_LOG_MSG_LENGTH];
			snprintf(msg, MAX_LOG_MSG_LENGTH, "could not open animated model file : %s", path);
			console->add_entry(msg, FIXME, RNDR);
			return;
		}

		fread(&num_vertices, sizeof(uint), 1, mesh_file);
		fread(&num_indices , sizeof(uint), 1, mesh_file);

		positions = (vec3*) calloc(num_vertices, sizeof(vec3) );
		normals   = (vec3*) calloc(num_vertices, sizeof(vec3) );
		weights   = (vec3*) calloc(num_vertices, sizeof(vec3) );
		bones     = (ivec3*)calloc(num_vertices, sizeof(ivec3));
		uvs       = (vec2*) calloc(num_vertices, sizeof(vec2) );
		indices   = (uint*) calloc(num_indices , sizeof(uint) );

		fread(positions, sizeof(vec3) , num_vertices, mesh_file);
		fread(normals  , sizeof(vec3) , num_vertices, mesh_file);
		fread(weights  , sizeof(vec3) , num_vertices, mesh_file);
		fread(bones    , sizeof(ivec3), num_vertices, mesh_file);
		fread(uvs      , sizeof(vec2) , num_vertices, mesh_file);
		fread(indices  , sizeof(uint) , num_indices , mesh_file);

		fclose(mesh_file);
	}
	void release()
	{
		free(positions);
		free(normals);
		free(weights); // hehe
		free(bones);
		free(uvs);
		free(indices);
	}
};

struct MeshLoaderAnim
{
	uint num_cached; // number of meshes currently stored in MeshLoader
	uint num_loaded; // total number of meshes loaded since program was launched; used for id generation

	struct MeshInfo
	{
		uint id, size, is_cached;
		char filepath[MAX_FILEPATH_LENGTH];
	} meshes[MAX_MESHES];

	void load_mesh(const char* filepath, uint* id) // returns mesh_id
	{
		// check if previously cached
		for (uint i = 0; i < MAX_MESHES; i++)
		{
			if (strcmp(meshes[i].filepath, filepath) == 0) // 0 = strings match!
			{
				out(filepath << " already cached!");
				if (id) *id = meshes[i].id;
				return;
			}
		}

		// if not already cached, then load from disk & add to cache
		FILE* mesh_file = fopen(filepath, "rb");
		if (!mesh_file)
		{
			char msg[MAX_LOG_MSG_LENGTH];
			snprintf(msg, MAX_LOG_MSG_LENGTH, "could not open animated model file : %s", filepath);
			console->add_entry(msg, FIXME, RNDR);
			return; // *id is left untouched (0) : caller checks for this
		}

		uint num_vertices = 0, num_indices = 0;
		fread(&num_vertices, sizeof(uint), 1, mesh_file);
		fread(&num_indices , sizeof(uint), 1, mesh_file);

		fclose(mesh_file);

		// look for empty spot
		for (uint i = 0; i < MAX_MESHES; i++)
		{
			if (meshes[i].id == 0)
			{
				uint filepath_length = strlen(filepath);
				if (filepath_length >= MAX_FILEPATH_LENGTH)
				{
					char msg[MAX_LOG_MSG_LENGTH];
					snprintf(msg, MAX_LOG_MSG_LENGTH, "mesh filepath too long (%d > %d) : %s",
						filepath_length, MAX_FILEPATH_LENGTH, filepath);
					console->add_entry(msg, FIXME, RNDR);
					return;
				}

				strcpy(meshes[i].filepath, filepath);

				meshes[i].id = num_loaded + 1; // avoid NULL value
				meshes[i].is_cached = true; // TODO : is this redundant if we use the filepath to check for caching?
				meshes[i].size = get_file_size(filepath); // disk bytes, shown in the console

				// update meta-data
				num_cached++;
				num_loaded++;

				if (id) *id = meshes[i].id;
				return;
			}
		}

		console->add_entry("animated mesh cache full (MAX_MESHES)", FIXME, RNDR);
	}
	void load_mesh_data(uint mesh_id, Mesh_Data_Anim* data)
	{
		if (mesh_id == 0) { console->add_entry("[ANIM] load_mesh_data : invalid mesh id 0", FIXME, RNDR); return; }

		for (uint i = 0; i < MAX_MESHES; i++)
		{
			if (meshes[i].id == mesh_id && data) // data cannot be nullptr
			{
				data->load(meshes[i].filepath);
				return;
			}
		}

		char msg[MAX_LOG_MSG_LENGTH];
		snprintf(msg, MAX_LOG_MSG_LENGTH, "[ANIM] load_mesh_data : mesh id [%d] not found", mesh_id);
		console->add_entry(msg, FIXME, RNDR);
	}
};

// Animation Keyframes & Skeleton

#define MAX_ANIM_BONES 16

struct Animation
{
	uint num_bones, num_frames;

	mat4  ibm[MAX_ANIM_BONES]; // inverse-bind matrices
	mat4* keyframes[MAX_ANIM_BONES]; // animation keyframes
	int   parents[MAX_ANIM_BONES]; // indices of parent bones
};

struct AnimLoader
{
	Animation animations[1] = {};

	void load_animation(Animation* anim, const char* path)
	{
		*anim = {};

		FILE* read = fopen(path, "rb");
		if (!read) { print("could not open animation file: %s\n", path); stop; return; }

		// skeleton
		fread(&anim->num_bones, sizeof(uint), 1, read);
		fread(anim->parents   , sizeof(uint), anim->num_bones, read);
		fread(anim->ibm       , sizeof(mat4), anim->num_bones, read);

		// animation keyframes
		fread(&anim->num_frames, sizeof(uint), 1, read);
		for (int i = 0; i < anim->num_bones; i++)
		{
			anim->keyframes[i] = Alloc(mat4, anim->num_frames);
			fread(anim->keyframes[i], sizeof(mat4), anim->num_frames, read);
		}

		fclose(read);
	}
};

// Textures

struct Texture_Data // alignas(64)
{
	uint id;
	GLuint handle;
	int width, height; // filled by TextureLoader::load, shown in the console
	char path[56];
};

struct TextureLoader // TODO : support caching & unloading textures (like MeshLoader)
{
	uint num_textures;
	Texture_Data textures[MAX_TEXTURES];

	void load(const char* path)
	{
		if (num_textures >= MAX_TEXTURES)
		{
			console->add_entry("TextureLoader : MAX_TEXTURE limit reached", FIXME, RNDR);
			return;
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
			glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, data);
			glGenerateMipmap(GL_TEXTURE_2D);
		}
		else
		{
			char msg[MAX_LOG_MSG_LENGTH];
			snprintf(msg, MAX_LOG_MSG_LENGTH, "failed to load texture : %s", path);
			console->add_entry(msg, FIXME, RNDR);
			stbi_image_free(data);
			glDeleteTextures(1, &textures[i].handle);
			return; // don't register a texture that never loaded
		}

		stbi_image_free(data);

		// update texture metadata
		textures[i].id = num_textures;
		textures[i].width  = width;
		textures[i].height = height;
		snprintf(textures[i].path, sizeof(textures[i].path), "%s", path);
		num_textures   = num_textures + 1;

		// log
		char msg[MAX_LOG_MSG_LENGTH] = {};
		snprintf(msg, MAX_LOG_MSG_LENGTH, "Loaded Texture : [%s], Size : [%d] bytes", path, width * height * num_channels);
		console->add_entry(msg, SUCCESS, RNDR);
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
	void set_vec3 (const char* name, vec3  value) { glUniform3f(glGetUniformLocation(id, name), value.x, value.y, value.z); }
	void set_mat4 (const char* name, mat4  value) { glUniformMatrix4fv(glGetUniformLocation(id, name), 1, GL_FALSE, (float*)&value); }
};

// Mesh Generators

namespace MeshFactory {

	enum MESH_GEN {
		CUBE, QUAD
	};

	Mesh_Data generate_cube(vec3 scale = vec3(1))
	{
		float positions[] = {
			// front face
			-0.5f, -0.5f,  0.5f, // 0
			 0.5f, -0.5f,  0.5f, // 1
			 0.5f,  0.5f,  0.5f, // 2
			-0.5f,  0.5f,  0.5f, // 3

			// back face
			 0.5f, -0.5f, -0.5f, // 4
			-0.5f, -0.5f, -0.5f, // 5
			-0.5f,  0.5f, -0.5f, // 6
			 0.5f,  0.5f, -0.5f, // 7

			// left face
			-0.5f, -0.5f, -0.5f, // 8
			-0.5f, -0.5f,  0.5f, // 9
			-0.5f,  0.5f,  0.5f, //10
			-0.5f,  0.5f, -0.5f, //11

			// right face
			 0.5f, -0.5f,  0.5f, //12
			 0.5f, -0.5f, -0.5f, //13
			 0.5f,  0.5f, -0.5f, //14
			 0.5f,  0.5f,  0.5f, //15

			// top face
			-0.5f,  0.5f,  0.5f, //16
			 0.5f,  0.5f,  0.5f, //17
			 0.5f,  0.5f, -0.5f, //18
			-0.5f,  0.5f, -0.5f, //19

			// bottom face
			-0.5f, -0.5f, -0.5f, //20
			 0.5f, -0.5f, -0.5f, //21
			 0.5f, -0.5f,  0.5f, //22
			-0.5f, -0.5f,  0.5f  //23
		};

		float normals[] = {
			// front (z+)
			0.0f,  0.0f,  1.0f,
			0.0f,  0.0f,  1.0f,
			0.0f,  0.0f,  1.0f,
			0.0f,  0.0f,  1.0f,

			// back (z-)
			0.0f,  0.0f, -1.0f,
			0.0f,  0.0f, -1.0f,
			0.0f,  0.0f, -1.0f,
			0.0f,  0.0f, -1.0f,

			// left (x-)
		  -1.0f,  0.0f,  0.0f,
		  -1.0f,  0.0f,  0.0f,
		  -1.0f,  0.0f,  0.0f,
		  -1.0f,  0.0f,  0.0f,

		  // right (x+)
		  1.0f,  0.0f,  0.0f,
		  1.0f,  0.0f,  0.0f,
		  1.0f,  0.0f,  0.0f,
		  1.0f,  0.0f,  0.0f,

		  // top (y+)
		  0.0f,  1.0f,  0.0f,
		  0.0f,  1.0f,  0.0f,
		  0.0f,  1.0f,  0.0f,
		  0.0f,  1.0f,  0.0f,

		  // bottom (y-)
		  0.0f, -1.0f,  0.0f,
		  0.0f, -1.0f,  0.0f,
		  0.0f, -1.0f,  0.0f,
		  0.0f, -1.0f,  0.0f
		};

		float uvs[] = {
			// front
			0, 0,  1, 0,  1, 1,  0, 1,
			// back
			0, 0,  1, 0,  1, 1,  0, 1,
			// left
			0, 0,  1, 0,  1, 1,  0, 1,
			// right
			0, 0,  1, 0,  1, 1,  0, 1,
			// top
			0, 0,  1, 0,  1, 1,  0, 1,
			// bottom
			0, 0,  1, 0,  1, 1,  0, 1
		};

		uint indices[] = {
			// front
			0, 1, 2,  2, 3, 0,
			// back
			4, 5, 6,  6, 7, 4,
			// left
			8, 9,10, 10,11, 8,
			// right
			12,13,14, 14,15,12,
			// top
			16,17,18, 18,19,16,
			// bottom
			20,21,22, 22,23,20
		};

		return Mesh_Data{}; // TODO : finish this function
	}
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
	
		vec2 uvs[]
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
}