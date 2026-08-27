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
		if (!mesh_file) { print("could not open model file: %s\n", path); stop; return; }

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
		if (!mesh_file) { print("could not open model file: %s\n", filepath); stop; }

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
				if (filepath_length < MAX_FILEPATH_LENGTH)
				{
					strcpy(meshes[i].filepath, filepath);
				}
				else
				{
					out("ERROR : MeshLoader : max filepath length exceeded!\n FILENAME : " << filepath);
					out("\n SIZE : " << filepath_length << " | MAX : " << MAX_FILEPATH_LENGTH);
					stop;
				}

				meshes[i].id = num_loaded + 1; // avoid NULL value
				meshes[i].is_cached = true; // TODO : is this redundant if we use the filepath to check for caching?

				// update meta-data
				num_cached++;
				num_loaded++;

				if (id) *id = meshes[i].id;
				return;
			}
		}

		out("ERROR : Cannot Cache : [" << filepath << "]!");
	}
	void load_mesh_data(uint mesh_id, Mesh_Data* data)
	{
		for (uint i = 0; i < MAX_MESHES; i++)
		{
			if (meshes[i].id == mesh_id && data) // data cannot be nullptr
			{
				data->load(meshes[i].filepath);
				return;
			}
		}

		out("ERROR : load_mesh_data() : mesh not found!"); stop;
	}
};

struct Texture_Data // alignas(64)
{
	uint id;
	GLuint handle;
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
			out("ERROR : TextureLoader : MAX_TEXTURE limit reached!"); stop;
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
		} else { out("ERROR : Failed to load texture"); stop; }

		stbi_image_free(data);

		// update texture count
		textures[i].id = num_textures;
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

		out("ERROR : TextureLoader : Texture with id [" << id << "] not found!"); stop;
	}
	void init(const char* default_texture_path = "assets/textures/default.jpg")
	{
		load(default_texture_path); // load default texture so we always have one to revert to
	}
};

struct ShaderProgram
{
	GLuint id;

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

		{
			GLint log_size = 0;
			glGetShaderiv(vert_shader, GL_INFO_LOG_LENGTH, &log_size);
			if (log_size)
			{
				char* error_log = (char*)calloc(log_size, sizeof(char));
				glGetShaderInfoLog(vert_shader, log_size, NULL, error_log);
				out("VERTEX SHADER ERROR:\n" << error_log);
				free(error_log);
			}

			log_size = 0;
			glGetShaderiv(frag_shader, GL_INFO_LOG_LENGTH, &log_size);
			if (log_size)
			{
				char* error_log = (char*)calloc(log_size, sizeof(char));
				glGetShaderInfoLog(frag_shader, log_size, NULL, error_log);
				out("FRAGMENT SHADER ERROR:\n" << error_log);
				free(error_log);
			}
		}

		id = glCreateProgram();
		glAttachShader(id, vert_shader);
		glAttachShader(id, frag_shader);
		glLinkProgram(id);

		GLsizei length = 0;
		char error[256] = {};
		glGetProgramInfoLog(id, 256, &length, error);
		if (length > 0) { out("SHADER PROGRAM ERROR:\n" << error); }

		glDeleteShader(vert_shader);
		glDeleteShader(frag_shader);
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