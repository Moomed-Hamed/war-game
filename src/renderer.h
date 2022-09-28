#include "window.h"

#define DRAW_DISTANCE 1024.0f

#define LOG_RNDR(msg) out("[renderer] "  << msg)
#define ERR_RNDR(msg) LOG_RNDR("ERROR: " << msg)

#define KiloByte(n) (n * 1024)
#define MegaByte(n) (KiloByte(n) * 1024)

// -------------------- Shaders -------------------- //

struct Shader { GLuint id; };

void load(Shader* shader, const char* vert_path, const char* frag_path)
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

	shader->id = glCreateProgram();
	glAttachShader(shader->id, vert_shader);
	glAttachShader(shader->id, frag_shader);
	glLinkProgram (shader->id);

	GLsizei length = 0;
	char error[256] = {};
	glGetProgramInfoLog(shader->id, 256, &length, error);
	if(length > 0) out(error);

	glDeleteShader(vert_shader);
	glDeleteShader(frag_shader);
}
void bind(Shader shader)
{
	glUseProgram(shader.id);
}
void free(Shader shader)
{
	glDeleteShader(shader.id);
}

struct Compute_Shader { GLuint id; };

void load(Compute_Shader* shader, const char* path)
{
	char* source = (char*)read_text_file_into_memory(path);

	GLuint comp_shader = glCreateShader(GL_COMPUTE_SHADER);
	glShaderSource(comp_shader, 1, &source, NULL);
	glCompileShader(comp_shader);

	free(source);

	{
		GLint log_size = 0;
		glGetShaderiv(comp_shader, GL_INFO_LOG_LENGTH, &log_size);
		if (log_size)
		{
			char* error_log = (char*)calloc(log_size, sizeof(char));
			glGetShaderInfoLog(comp_shader, log_size, NULL, error_log);
			out(path);
			out("COMPUTE SHADER ERROR:\n" << error_log);
			free(error_log);
		}
	}

	shader->id = glCreateProgram();
	glAttachShader(shader->id, comp_shader);
	glLinkProgram(shader->id);

	GLsizei length = 0;
	char error[256] = {};
	glGetProgramInfoLog(shader->id, 256, &length, error);
	if (length) out(error);

	glDeleteShader(comp_shader);
}
void bind(Compute_Shader shader)
{
	glUseProgram(shader.id);
}
void dispatch(Compute_Shader shader, uint num_x, uint num_y, uint num_z)
{
	glUseProgram(shader.id);
	glDispatchCompute(num_x, num_y, num_z);
}

struct Shader_Compute_Render { GLuint id; };

void load(Shader_Compute_Render* shader, const char* vert, const char* frag, const char* comp)
{
	char* vert_source = (char*)read_text_file_into_memory(vert);
	char* frag_source = (char*)read_text_file_into_memory(frag);
	char* comp_source = (char*)read_text_file_into_memory(comp);

	GLuint vert_shader = glCreateShader(GL_VERTEX_SHADER);
	glShaderSource(vert_shader, 1, &vert_source, NULL);
	glCompileShader(vert_shader);

	GLuint frag_shader = glCreateShader(GL_FRAGMENT_SHADER);
	glShaderSource(frag_shader, 1, &frag_source, NULL);
	glCompileShader(frag_shader);

	GLuint comp_shader = glCreateShader(GL_COMPUTE_SHADER);
	glShaderSource(comp_shader, 1, &comp_source, NULL);
	glCompileShader(comp_shader);

	free(vert_source);
	free(frag_source);
	free(comp_source);

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

		log_size = 0;
		glGetShaderiv(comp_shader, GL_INFO_LOG_LENGTH, &log_size);
		if (log_size)
		{
			char* error_log = (char*)calloc(log_size, sizeof(char));
			glGetShaderInfoLog(comp_shader, log_size, NULL, error_log);
			out("FRAGMENT SHADER ERROR:\n" << error_log);
			free(error_log);
		}
	}

	shader->id = glCreateProgram();
	glAttachShader(shader->id, vert_shader);
	glAttachShader(shader->id, comp_shader);
	glAttachShader(shader->id, frag_shader);
	glLinkProgram(shader->id);

	GLsizei length = 0;
	char error[256] = {};
	glGetProgramInfoLog(shader->id, 256, &length, error);
	if (length > 0) out(error);

	glDeleteShader(vert_shader);
	glDeleteShader(frag_shader);
	glDeleteShader(comp_shader);
}
void bind(Shader_Compute_Render shader)
{
	glUseProgram(shader.id);
}
void free(Shader_Compute_Render shader)
{
	glDeleteShader(shader.id);
}

struct Shader_Geometry_Render { GLuint id; };

void load(Shader_Geometry_Render* shader, const char* vert, const char* frag, const char* geom)
{
	char* vert_source = (char*)read_text_file_into_memory(vert);
	char* frag_source = (char*)read_text_file_into_memory(frag);
	char* geom_source = (char*)read_text_file_into_memory(geom);

	GLuint vert_shader = glCreateShader(GL_VERTEX_SHADER);
	glShaderSource(vert_shader, 1, &vert_source, NULL);
	glCompileShader(vert_shader);

	GLuint frag_shader = glCreateShader(GL_FRAGMENT_SHADER);
	glShaderSource(frag_shader, 1, &frag_source, NULL);
	glCompileShader(frag_shader);

	GLuint geom_shader = glCreateShader(GL_GEOMETRY_SHADER);
	glShaderSource(geom_shader, 1, &geom_source, NULL);
	glCompileShader(geom_shader);

	free(vert_source);
	free(frag_source);
	free(geom_source);

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

		log_size = 0;
		glGetShaderiv(geom_shader, GL_INFO_LOG_LENGTH, &log_size);
		if (log_size)
		{
			char* error_log = (char*)calloc(log_size, sizeof(char));
			glGetShaderInfoLog(geom_shader, log_size, NULL, error_log);
			out("FRAGMENT SHADER ERROR:\n" << error_log);
			free(error_log);
		}
	}

	shader->id = glCreateProgram();
	glAttachShader(shader->id, vert_shader);
	glAttachShader(shader->id, geom_shader);
	glAttachShader(shader->id, frag_shader);
	glLinkProgram(shader->id);

	GLsizei length = 0;
	char error[256] = {};
	glGetProgramInfoLog(shader->id, 256, &length, error);
	if (length > 0) out(error);

	glDeleteShader(vert_shader);
	glDeleteShader(frag_shader);
	glDeleteShader(geom_shader);
}
void bind(Shader_Geometry_Render shader)
{
	glUseProgram(shader.id);
}
void free(Shader_Geometry_Render shader)
{
	glDeleteShader(shader.id);
}

// remember : bind the shader *before* calling these
void set_int  (Shader shader, const char* name, int   value)
{
	glUniform1i(glGetUniformLocation(shader.id, name), value);
}
void set_float(Shader shader, const char* name, float value)
{
	glUniform1f(glGetUniformLocation(shader.id, name), value);
}
void set_vec3 (Shader shader, const char* name, vec3  value)
{
	glUniform3f(glGetUniformLocation(shader.id, name), value.x, value.y, value.z);
}
void set_mat4 (Shader shader, const char* name, mat4  value)
{
	glUniformMatrix4fv(glGetUniformLocation(shader.id, name), 1, GL_FALSE, (float*)&value);
}

// -------------------- Textures ------------------- //

GLuint load_texture_bmp(const char* path)
{
	GLuint id = {};
	int width, height, num_channels;
	byte* image;

	stbi_set_flip_vertically_on_load(true);

	image = stbi_load(path, &width, &height, &num_channels, 0);
	if (image == NULL) out("ERROR : '" << path << "' NOT FOUND!");

	glGenTextures(1, &id);
	glBindTexture(GL_TEXTURE_2D, id);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB32F, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, image);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glGenerateMipmap(GL_TEXTURE_2D);

	stbi_image_free(image);

	return id;
}
GLuint load_texture_png(const char* path)
{
	GLuint id = {};
	int width, height, num_channels;
	byte* image;

	stbi_set_flip_vertically_on_load(false);

	image = stbi_load(path, &width, &height, &num_channels, 0);
	if (image == NULL) ERR_RNDR("'" << path << "' NOT FOUND!");
	if (num_channels == 3) ERR_RNDR("'" << path << "' must be 32 bits deep!");

	glGenTextures(1, &id);
	glBindTexture(GL_TEXTURE_2D, id);
	if(num_channels == 3)
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB32F, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, image);
	else
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB32F, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, image);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glGenerateMipmap(GL_TEXTURE_2D);

	stbi_image_free(image);

	return id;
}
void bind_texture(GLuint texture, uint texture_unit = 0)
{
	glActiveTexture(GL_TEXTURE0 + texture_unit);
	glBindTexture(GL_TEXTURE_2D, texture);
}

// ------------------ Mesh Loading ----------------- //

struct Mesh_Data
{
	uint num_vertices, num_indices;

	vec3* positions;
	vec3* normals;
	uint* indices;
};

struct Mesh_Data_UV
{
	uint num_vertices, num_indices;

	vec3* positions;
	vec3* normals;
	vec2* textures;
	uint* indices;
};

struct Mesh_Data_Anim
{
	uint num_vertices, num_indices;

	vec3*  positions;
	vec3*  normals;
	vec3*  weights;
	ivec3* bones;
	uint*  indices;
};

struct Mesh_Data_Anim_UV
{
	uint num_vertices, num_indices;

	vec3*  positions;
	vec3*  normals;
	vec2*  textures;
	vec3*  weights;
	ivec3* bones;
	uint*  indices;
};

void load(Mesh_Data* data, const char* path)
{
	FILE* mesh_file = fopen(path, "rb");
	if (!mesh_file) { print("could not open mesh file : %s\n", path); stop; return; }

	fread(&data->num_vertices, sizeof(uint), 1, mesh_file);
	fread(&data->num_indices , sizeof(uint), 1, mesh_file);

	data->positions = (vec3*)calloc(data->num_vertices, sizeof(vec3));
	data->normals   = (vec3*)calloc(data->num_vertices, sizeof(vec3));
	data->indices   = (uint*)calloc(data->num_indices , sizeof(uint));

	fread(data->positions, sizeof(vec3), data->num_vertices, mesh_file);
	fread(data->normals  , sizeof(vec3), data->num_vertices, mesh_file);
	fread(data->indices  , sizeof(uint), data->num_indices , mesh_file);

	fclose(mesh_file);
}
void load(Mesh_Data_UV* data, const char* path)
{
	FILE* mesh_file = fopen(path, "rb");
	if (!mesh_file) { print("could not open model file: %s\n", path); stop; return; }

	fread(&data->num_vertices, sizeof(uint), 1, mesh_file);
	fread(&data->num_indices , sizeof(uint), 1, mesh_file);

	data->positions = (vec3*)calloc(data->num_vertices, sizeof(vec3));
	data->normals   = (vec3*)calloc(data->num_vertices, sizeof(vec3));
	data->textures  = (vec2*)calloc(data->num_vertices, sizeof(vec2));
	data->indices   = (uint*)calloc(data->num_indices , sizeof(uint));

	fread(data->positions, sizeof(vec3), data->num_vertices, mesh_file);
	fread(data->normals  , sizeof(vec3), data->num_vertices, mesh_file);
	fread(data->textures , sizeof(vec2), data->num_vertices, mesh_file);
	fread(data->indices  , sizeof(uint), data->num_indices , mesh_file);

	fclose(mesh_file);
}
void load(Mesh_Data_Anim* data, const char* path)
{
	FILE* mesh_file = fopen(path, "rb");
	if (!mesh_file) { print("could not open mesh file: %s\n", path); stop; return; }

	fread(&data->num_vertices, sizeof(uint), 1, mesh_file);
	fread(&data->num_indices , sizeof(uint), 1, mesh_file);

	data->positions = (vec3*) calloc(data->num_vertices, sizeof(vec3));
	data->normals   = (vec3*) calloc(data->num_vertices, sizeof(vec3));
	data->weights   = (vec3*) calloc(data->num_vertices, sizeof(vec3));
	data->bones     = (ivec3*)calloc(data->num_vertices, sizeof(ivec3));
	data->indices   = (uint*) calloc(data->num_indices , sizeof(uint));

	fread(data->positions, sizeof(vec3) , data->num_vertices, mesh_file);
	fread(data->normals  , sizeof(vec3) , data->num_vertices, mesh_file);
	fread(data->weights  , sizeof(vec3) , data->num_vertices, mesh_file);
	fread(data->bones    , sizeof(ivec3), data->num_vertices, mesh_file);
	fread(data->indices  , sizeof(uint) , data->num_indices , mesh_file);

	fclose(mesh_file);
}
void load(Mesh_Data_Anim_UV* data, const char* path)
{
	FILE* mesh_file = fopen(path, "rb");
	if (!mesh_file) { print("could not open mesh file: %s\n", path); stop; return; }

	fread(&data->num_vertices, sizeof(uint), 1, mesh_file);
	fread(&data->num_indices , sizeof(uint), 1, mesh_file);

	data->positions = (vec3*) calloc(data->num_vertices, sizeof(vec3));
	data->normals   = (vec3*) calloc(data->num_vertices, sizeof(vec3));
	data->weights   = (vec3*) calloc(data->num_vertices, sizeof(vec3));
	data->bones     = (ivec3*)calloc(data->num_vertices, sizeof(ivec3));
	data->textures  = (vec2*) calloc(data->num_vertices, sizeof(vec2));
	data->indices   = (uint*) calloc(data->num_indices , sizeof(uint));

	fread(data->positions, sizeof(vec3) , data->num_vertices, mesh_file);
	fread(data->normals  , sizeof(vec3) , data->num_vertices, mesh_file);
	fread(data->weights  , sizeof(vec3) , data->num_vertices, mesh_file);
	fread(data->bones    , sizeof(ivec3), data->num_vertices, mesh_file);
	fread(data->textures , sizeof(vec2) , data->num_vertices, mesh_file);
	fread(data->indices  , sizeof(uint) , data->num_indices , mesh_file);

	//for (int i = 0; i < data->num_vertices; i++)
	//{
	//	printvec(data->positions[i]);
	//	printvec(data->normals[i]);
	//	printvec(data->weights[i]);
	//	printvec(data->bones[i]);
	//	out(data->textures[i].x << ',' << data->textures[i].y);
	//	//stop;
	//}

	fclose(mesh_file);
}

// ----------------- Mesh Rendering ---------------- //

#define MAX_MESHES 16 // track GPU memory usage?

struct Mesh_Renderer
{
	GLuint VAO, VBO, EBO;
	uint num_meshes;

	struct {
		uint num_indices;
		uint index_offset;
		uint vertex_offset;
		uint num_instances;
		uint instance_offset;
	} meshes[MAX_MESHES];
};
struct Drawable_Mesh_UV
{
	GLuint VAO, VBO, EBO;
	uint num_meshes;

	struct {
		uint num_indices;
		uint index_offset;
		uint vertex_offset;
		uint num_instances;
		uint instance_offset;
	} meshes[MAX_MESHES];
};
struct Drawable_Mesh_Anim
{
	GLuint VAO, VBO, EBO, UBO;
	uint num_indices;
};
struct Drawable_Mesh_Anim_UV
{
	GLuint VAO, VBO, EBO, UBO;
	uint num_indices;
};

void load(Mesh_Renderer* renderer, const char** paths, uint reserved_mem_size = 0, uint num_meshes = 1)
{
	renderer->num_meshes = num_meshes;

	// load meshes from files

	Mesh_Data* data = Alloc(Mesh_Data, MAX_MESHES); // this should probably be passed in as a pointer

	uint num_indices  = 0;
	uint num_vertices = 0;

	for (uint i = 0; i < num_meshes; i++)
	{
		load(data + i, paths[i]);

		renderer->meshes[i].num_indices   = data[i].num_indices;
		renderer->meshes[i].index_offset  = num_indices * sizeof(uint);
		renderer->meshes[i].vertex_offset = num_vertices;

		num_indices  += data[i].num_indices;
		num_vertices += data[i].num_vertices;
	}

	// opengl stuff

	uint vertex_size = sizeof(vec3) + sizeof(vec3);
	uint vertex_buffer_size = reserved_mem_size + (num_vertices * vertex_size);
	uint index_buffer_size  = num_indices * sizeof(uint);

	glGenVertexArrays(1, &renderer->VAO);
	glBindVertexArray(renderer->VAO);

	glGenBuffers(1, &renderer->VBO);
	glBindBuffer(GL_ARRAY_BUFFER, renderer->VBO);
	glBufferData(GL_ARRAY_BUFFER, vertex_buffer_size, NULL, GL_STATIC_DRAW);

	glGenBuffers(1, &renderer->EBO);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, renderer->EBO);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, index_buffer_size, NULL, GL_STATIC_DRAW);

	// loading mesh data into OpenGL buffers

	uint p_offset = reserved_mem_size;
	uint n_offset = p_offset + (num_vertices * sizeof(vec3));

	for (uint i = 0; i < num_meshes; i++)
	{
		uint p_size = data[i].num_vertices * sizeof(vec3);
		uint n_size = p_size;
		uint i_size = data[i].num_indices * sizeof(uint);

		glBufferSubData(GL_ARRAY_BUFFER, p_offset, p_size, data[i].positions);
		glBufferSubData(GL_ARRAY_BUFFER, n_offset, n_size, data[i].normals  );
		glBufferSubData(GL_ELEMENT_ARRAY_BUFFER, renderer->meshes[i].index_offset, i_size, data[i].indices);

		p_offset += p_size;
		n_offset += n_size;

		free(data[i].positions);
		free(data[i].normals);
		free(data[i].indices);
	} free(data);

	// vertex specification / layout

	uint position_offset = reserved_mem_size;
	uint normal_offset = position_offset + (num_vertices * sizeof(vec3));

	GLint p_attrib = 0;
	glVertexAttribPointer(p_attrib, 3, GL_FLOAT, GL_FALSE, sizeof(vec3), (void*)position_offset);
	glEnableVertexAttribArray(p_attrib);

	GLint n_attrib = 1;
	glVertexAttribPointer(n_attrib, 3, GL_FLOAT, GL_FALSE, sizeof(vec3), (void*)normal_offset);
	glEnableVertexAttribArray(n_attrib);
}
void load(Drawable_Mesh_UV* renderer, const char** paths, uint reserved_mem_size = 0, uint num_meshes = 1)
{
	renderer->num_meshes = num_meshes;

	// load meshes from files

	Mesh_Data_UV* data = Alloc(Mesh_Data_UV, num_meshes);

	uint num_indices  = 0;
	uint num_vertices = 0;

	for (uint i = 0; i < num_meshes; i++)
	{
		load(data + i, paths[i]);

		renderer->meshes[i].num_indices   = data[i].num_indices;
		renderer->meshes[i].index_offset  = num_indices * sizeof(uint);
		renderer->meshes[i].vertex_offset = num_vertices;

		num_indices  += data[i].num_indices;
		num_vertices += data[i].num_vertices;
	}

	// opengl stuff

	uint vertex_size = sizeof(vec3) + sizeof(vec3) + sizeof(vec2);
	uint vertex_buffer_size = reserved_mem_size + (num_vertices * vertex_size);
	uint index_buffer_size = num_indices * sizeof(uint);

	glGenVertexArrays(1, &renderer->VAO);
	glBindVertexArray(renderer->VAO);

	glGenBuffers(1, &renderer->VBO);
	glBindBuffer(GL_ARRAY_BUFFER, renderer->VBO);
	glBufferData(GL_ARRAY_BUFFER, vertex_buffer_size, NULL, GL_STATIC_DRAW);

	glGenBuffers(1, &renderer->EBO);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, renderer->EBO);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, index_buffer_size, NULL, GL_STATIC_DRAW);

	// loading mesh data into OpenGL buffers

	uint p_offset = reserved_mem_size;
	uint n_offset = p_offset + (num_vertices * sizeof(vec3));
	uint t_offset = n_offset + (num_vertices * sizeof(vec3));

	for (uint i = 0; i < num_meshes; i++)
	{
		uint p_size = data[i].num_vertices * sizeof(vec3);
		uint n_size = p_size;
		uint t_size = data[i].num_vertices * sizeof(vec2);
		uint i_size = data[i].num_indices  * sizeof(uint);

		glBufferSubData(GL_ARRAY_BUFFER, p_offset, p_size, data[i].positions);
		glBufferSubData(GL_ARRAY_BUFFER, n_offset, n_size, data[i].normals  );
		glBufferSubData(GL_ARRAY_BUFFER, t_offset, t_size, data[i].textures );
		glBufferSubData(GL_ELEMENT_ARRAY_BUFFER, renderer->meshes[i].index_offset, i_size, data[i].indices);

		p_offset += p_size;
		n_offset += n_size;
		t_offset += t_size;

		free(data[i].positions);
		free(data[i].normals  );
		free(data[i].textures );
		free(data[i].indices  );
	} free(data);

	// vertex specification / layout

	uint position_offset = reserved_mem_size;
	uint normal_offset   = position_offset + (num_vertices * sizeof(vec3));
	uint texture_offset  = normal_offset   + (num_vertices * sizeof(vec3));

	GLint p_attrib = 0;
	glVertexAttribPointer(p_attrib, 3, GL_FLOAT, GL_FALSE, sizeof(vec3), (void*)position_offset);
	glEnableVertexAttribArray(p_attrib);

	GLint n_attrib = 1;
	glVertexAttribPointer(n_attrib, 3, GL_FLOAT, GL_FALSE, sizeof(vec3), (void*)normal_offset);
	glEnableVertexAttribArray(n_attrib);

	GLint t_attrib = 2;
	glVertexAttribPointer(t_attrib, 2, GL_FLOAT, GL_FALSE, sizeof(vec2), (void*)texture_offset);
	glEnableVertexAttribArray(t_attrib);
}
void load(Drawable_Mesh_Anim* mesh, const char* path, uint reserved_mem_size = 0)
{
	Mesh_Data_Anim mesh_data;
	load(&mesh_data, path);
	mesh->num_indices = mesh_data.num_indices;

	glGenVertexArrays(1, &(mesh->VAO));
	glBindVertexArray(mesh->VAO);

	uint vertmemsize = mesh_data.num_vertices * sizeof(vec3);
	uint bonememsize = mesh_data.num_vertices * sizeof(ivec3);
	uint offset = reserved_mem_size;

#define RENDER_MEM_SIZE (reserved_mem_size + (vertmemsize + vertmemsize + vertmemsize + bonememsize)) // positions, normals, weights, and bones
	glGenBuffers(1, &(mesh->VBO));
	glBindBuffer(GL_ARRAY_BUFFER, mesh->VBO);
	glBufferData(GL_ARRAY_BUFFER, RENDER_MEM_SIZE, NULL, GL_STATIC_DRAW);
	glBufferSubData(GL_ARRAY_BUFFER, offset + (vertmemsize * 0), vertmemsize, mesh_data.positions);
	glBufferSubData(GL_ARRAY_BUFFER, offset + (vertmemsize * 1), vertmemsize, mesh_data.normals  );
	glBufferSubData(GL_ARRAY_BUFFER, offset + (vertmemsize * 2), vertmemsize, mesh_data.weights  );
	glBufferSubData(GL_ARRAY_BUFFER, offset + (vertmemsize * 3), bonememsize, mesh_data.bones    );
#undef RENDER_MEM_SIZE

	glGenBuffers(1, &(mesh->EBO));
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mesh->EBO);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, mesh_data.num_indices * sizeof(uint), mesh_data.indices, GL_STATIC_DRAW);

	free(mesh_data.positions);
	free(mesh_data.normals);
	free(mesh_data.weights);
	free(mesh_data.bones);
	free(mesh_data.indices);

	offset = reserved_mem_size;
	{
		GLint pos_attrib = 0; // position of a vertex
		glVertexAttribPointer(pos_attrib, 3, GL_FLOAT, GL_FALSE, sizeof(vec3), (void*)(offset + (vertmemsize * 0)));
		glEnableVertexAttribArray(pos_attrib);

		GLint norm_attrib = 1; // normal of a vertex
		glVertexAttribPointer(norm_attrib, 3, GL_FLOAT, GL_FALSE, sizeof(vec3), (void*)(offset + (vertmemsize * 1)));
		glEnableVertexAttribArray(norm_attrib);

		GLint weight_attrib = 2; // weights of bone influence
		glVertexAttribPointer(weight_attrib, 3, GL_FLOAT, GL_FALSE, sizeof(vec3), (void*)(offset + (vertmemsize * 2)));
		glEnableVertexAttribArray(weight_attrib);

		GLint bone_attrib = 3; // id's of 3 bones that influence this vertex
		glVertexAttribIPointer(bone_attrib, 3, GL_INT, sizeof(ivec3), (void*)(offset + (vertmemsize * 3)));
		glEnableVertexAttribArray(bone_attrib);
	}

	glGenBuffers(1, &mesh->UBO);
	glBindBuffer(GL_UNIFORM_BUFFER, mesh->UBO);
	glBufferData(GL_UNIFORM_BUFFER, 16 * sizeof(mat4), NULL, GL_DYNAMIC_DRAW); // WARNING MAX JOINTS HARDCODED IN AS 16

	//glBindBufferRange(GL_UNIFORM_BUFFER, 0, renderdata->UBO, 0, model_data.num_joints * sizeof(glm::mat4));
	glBindBufferBase(GL_UNIFORM_BUFFER, 0, mesh->UBO);
}
void load(Drawable_Mesh_Anim_UV* mesh, const char* path, uint reserved_mem_size = 0)
{
	Mesh_Data_Anim_UV mesh_data;
	load(&mesh_data, path);
	mesh->num_indices = mesh_data.num_indices;

	glGenVertexArrays(1, &(mesh->VAO));
	glBindVertexArray(mesh->VAO);

	uint vertmemsize = mesh_data.num_vertices * sizeof(vec3);
	uint bonememsize = mesh_data.num_vertices * sizeof(ivec3);
	uint texmemsize  = mesh_data.num_vertices * sizeof(vec2);
	uint offset = reserved_mem_size;

	// positions, normals, weights, bones, textures
	uint render_mem_size = reserved_mem_size + vertmemsize + vertmemsize + vertmemsize + bonememsize + texmemsize;
	glGenBuffers(1, &(mesh->VBO));
	glBindBuffer(GL_ARRAY_BUFFER, mesh->VBO);
	glBufferData(GL_ARRAY_BUFFER, render_mem_size, NULL, GL_STATIC_DRAW);
	glBufferSubData(GL_ARRAY_BUFFER, offset + (vertmemsize * 0), vertmemsize, mesh_data.positions);
	glBufferSubData(GL_ARRAY_BUFFER, offset + (vertmemsize * 1), vertmemsize, mesh_data.normals  );
	glBufferSubData(GL_ARRAY_BUFFER, offset + (vertmemsize * 2), vertmemsize, mesh_data.weights  );
	glBufferSubData(GL_ARRAY_BUFFER, offset + (vertmemsize * 3), bonememsize, mesh_data.bones    );
	glBufferSubData(GL_ARRAY_BUFFER, offset + (vertmemsize * 3) + bonememsize, texmemsize , mesh_data.textures );

	glGenBuffers(1, &(mesh->EBO));
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mesh->EBO);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, mesh_data.num_indices * sizeof(uint), mesh_data.indices, GL_STATIC_DRAW);

	free(mesh_data.positions);
	free(mesh_data.normals);
	free(mesh_data.textures);
	free(mesh_data.weights);
	free(mesh_data.bones);
	free(mesh_data.indices);

	offset = reserved_mem_size;
	{
		GLint pos_attrib = 0; // position of a vertex
		glVertexAttribPointer(pos_attrib, 3, GL_FLOAT, GL_FALSE, sizeof(vec3), (void*)(offset + (vertmemsize * 0)));
		glEnableVertexAttribArray(pos_attrib);

		GLint norm_attrib = 1; // normal of a vertex
		glVertexAttribPointer(norm_attrib, 3, GL_FLOAT, GL_FALSE, sizeof(vec3), (void*)(offset + (vertmemsize * 1)));
		glEnableVertexAttribArray(norm_attrib);

		GLint weight_attrib = 2; // weights of bone influence
		glVertexAttribPointer(weight_attrib, 3, GL_FLOAT, GL_FALSE, sizeof(vec3), (void*)(offset + (vertmemsize * 2)));
		glEnableVertexAttribArray(weight_attrib);

		GLint bone_attrib = 3; // id's of 3 bones that influence this vertex
		glVertexAttribIPointer(bone_attrib, 3, GL_INT, sizeof(ivec3), (void*)(offset + (vertmemsize * 3)));
		glEnableVertexAttribArray(bone_attrib);

		GLint texture_attrib = 4; // texture coordinates of a vertex
		glVertexAttribPointer(texture_attrib, 2, GL_FLOAT, GL_FALSE, sizeof(vec2), (void*)(offset + (vertmemsize * 3) + bonememsize));
		glEnableVertexAttribArray(texture_attrib);
	}

	glGenBuffers(1, &mesh->UBO);
	glBindBuffer(GL_UNIFORM_BUFFER, mesh->UBO);
	glBufferData(GL_UNIFORM_BUFFER, 16 * sizeof(mat4), NULL, GL_DYNAMIC_DRAW); // WARNING MAX JOINTS HARDCODED IN AS 16

	//glBindBufferRange(GL_UNIFORM_BUFFER, 0, renderdata->UBO, 0, model_data.num_joints * sizeof(glm::mat4));
	glBindBufferBase(GL_UNIFORM_BUFFER, 0, mesh->UBO);
}

void update(Mesh_Renderer mesh, uint vb_size, byte* vb_data)
{
	glBindBuffer(GL_ARRAY_BUFFER, mesh.VBO);
	glBufferSubData(GL_ARRAY_BUFFER, 0, vb_size, vb_data);
}
void update(Drawable_Mesh_UV mesh, uint vb_size, byte* vb_data)
{
	glBindBuffer(GL_ARRAY_BUFFER, mesh.VBO);
	glBufferSubData(GL_ARRAY_BUFFER, 0, vb_size, vb_data);
}
void update(Drawable_Mesh_Anim mesh, uint num_bones, mat4* pose, uint vb_size, byte* vb_data)
{
	glBindBuffer(GL_ARRAY_BUFFER, mesh.VBO);
	glBufferSubData(GL_ARRAY_BUFFER, 0, vb_size, vb_data);

	glBindBuffer(GL_UNIFORM_BUFFER, mesh.UBO);
	glBufferSubData(GL_UNIFORM_BUFFER, 0, num_bones * sizeof(mat4), pose);
}
void update(Drawable_Mesh_Anim_UV mesh, uint num_bones, mat4* pose, uint vb_size, byte* vb_data)
{
	glBindBuffer(GL_ARRAY_BUFFER, mesh.VBO);
	glBufferSubData(GL_ARRAY_BUFFER, 0, vb_size, vb_data);

	glBindBuffer(GL_UNIFORM_BUFFER, mesh.UBO);
	glBufferSubData(GL_UNIFORM_BUFFER, 0, num_bones * sizeof(mat4), pose);
}

void draw(Mesh_Renderer renderer)
{
	// consider switching to glMultiDrawElementsIndirect() for 1 draw call
	glBindVertexArray(renderer.VAO);
	for (uint i = 0; i < renderer.num_meshes; i++)
	{
		uint byte_offset     = renderer.meshes[i].index_offset;
		uint num_indices     = renderer.meshes[i].num_indices;
		uint vertex_offset   = renderer.meshes[i].vertex_offset;
		uint num_instances   = renderer.meshes[i].num_instances;
		uint instance_offset = renderer.meshes[i].instance_offset;

		glDrawElementsInstancedBaseVertexBaseInstance(GL_TRIANGLES, num_indices, GL_UNSIGNED_INT,
			(void*)byte_offset, num_instances, vertex_offset, instance_offset);
	}
}
void draw(Drawable_Mesh_UV renderer)
{
	// consider switching to glMultiDrawElementsIndirect() for 1 draw call
	glBindVertexArray(renderer.VAO);
	for (uint i = 0; i < renderer.num_meshes; i++)
	{
		uint byte_offset     = renderer.meshes[i].index_offset;
		uint num_indices     = renderer.meshes[i].num_indices;
		uint vertex_offset   = renderer.meshes[i].vertex_offset;
		uint num_instances   = renderer.meshes[i].num_instances;
		uint instance_offset = renderer.meshes[i].instance_offset;

		glDrawElementsInstancedBaseVertexBaseInstance(GL_TRIANGLES, num_indices, GL_UNSIGNED_INT,
			(void*)byte_offset, num_instances, vertex_offset, instance_offset);
	}
}
void draw(Drawable_Mesh_Anim mesh, uint num_instances = 1)
{
	glBindVertexArray(mesh.VAO);
	glDrawElementsInstanced(GL_TRIANGLES, mesh.num_indices, GL_UNSIGNED_INT, 0, num_instances);
}
void draw(Drawable_Mesh_Anim_UV mesh, uint num_instances = 1)
{
	glBindVertexArray(mesh.VAO);
	glBindBufferBase(GL_UNIFORM_BUFFER, 1, mesh.UBO); // animations use slot 1
	glDrawElementsInstanced(GL_TRIANGLES, mesh.num_indices, GL_UNSIGNED_INT, 0, num_instances);
}

// TODO : rename these to add_vert_attrib or vert_add_attrib?
void mesh_add_attrib_float(GLuint attrib_id, uint stride, uint offset)
{
	glVertexAttribPointer(attrib_id, 1, GL_FLOAT, GL_FALSE, stride, (void*)offset);
	glVertexAttribDivisor(attrib_id, 1);
	glEnableVertexAttribArray(attrib_id);
}
void mesh_add_attrib_vec2 (GLuint attrib_id, uint stride, uint offset)
{
	glVertexAttribPointer(attrib_id, 2, GL_FLOAT, GL_FALSE, stride, (void*)offset);
	glVertexAttribDivisor(attrib_id, 1);
	glEnableVertexAttribArray(attrib_id);
}
void mesh_add_attrib_vec3 (GLuint attrib_id, uint stride, uint offset)
{
	glVertexAttribPointer(attrib_id, 3, GL_FLOAT, GL_FALSE, stride, (void*)offset);
	glVertexAttribDivisor(attrib_id, 1);
	glEnableVertexAttribArray(attrib_id);
}
void mesh_add_attrib_mat3 (GLuint attrib_id, uint stride, uint offset)
{
	glVertexAttribPointer(attrib_id, 3, GL_FLOAT, GL_FALSE, stride, (void*)offset);
	glVertexAttribDivisor(attrib_id, 1);
	glEnableVertexAttribArray(attrib_id);

	++attrib_id;
	offset += sizeof(vec3);
	glVertexAttribPointer(attrib_id, 3, GL_FLOAT, GL_FALSE, stride, (void*)offset);
	glVertexAttribDivisor(attrib_id, 1);
	glEnableVertexAttribArray(attrib_id);

	++attrib_id;
	offset += sizeof(vec3);
	glVertexAttribPointer(attrib_id, 3, GL_FLOAT, GL_FALSE, stride, (void*)offset);
	glVertexAttribDivisor(attrib_id, 1);
	glEnableVertexAttribArray(attrib_id);
}

// ---------- Deferred Rendering Pipeline ---------- //

/* -- deferred rendering theory --

	glBindFramebuffer(GL_FRAMEBUFFER, g_buffer.FBO);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	bind(mesh_shader);
	draw(mesh);

	glBindFramebuffer(GL_FRAMEBUFFER, 0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	bind(lighting_shader);
	draw(g_buffer);
*/

struct G_Buffer
{
	GLuint FBO; // frame buffer object
	GLuint positions, normals, albedo; // textures
	GLuint VAO, VBO, EBO; // for drawing the quad
};

G_Buffer make_g_buffer(Window window)
{
	G_Buffer buf = {};

	glGenFramebuffers(1, &buf.FBO);
	glBindFramebuffer(GL_FRAMEBUFFER, buf.FBO);

	GLuint g_positions, g_normals, g_albedo;

	// position color buffer
	glGenTextures(1, &g_positions);
	glBindTexture(GL_TEXTURE_2D, g_positions);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, window.screen_width, window.screen_height, 0, GL_RGBA, GL_FLOAT, NULL);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

	// normal color buffer
	glGenTextures(1, &g_normals);
	glBindTexture(GL_TEXTURE_2D, g_normals);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA16F, window.screen_width, window.screen_height, 0, GL_RGBA, GL_FLOAT, NULL);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

	// albedo color buffer
	glGenTextures(1, &g_albedo);
	glBindTexture(GL_TEXTURE_2D, g_albedo);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, window.screen_width, window.screen_height, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

	// tell OpenGL which color attachments we'll use (of this framebuffer) for rendering 
	uint attachments[3] = { GL_COLOR_ATTACHMENT0, GL_COLOR_ATTACHMENT1, GL_COLOR_ATTACHMENT2 };
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, g_positions, 0);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT1, GL_TEXTURE_2D, g_normals  , 0);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT2, GL_TEXTURE_2D, g_albedo   , 0);
	glDrawBuffers(3, attachments);

	// then also add render buffer object as depth buffer and check for completeness.
	uint depth_render_buffer;
	glGenRenderbuffers(1, &depth_render_buffer);
	glBindRenderbuffer(GL_RENDERBUFFER, depth_render_buffer);
	glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT, window.screen_width, window.screen_height);
	glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, depth_render_buffer);

	if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) { out("FRAMEBUFFER ERROR : INCOMPLETE"); stop; }

	buf.positions = g_positions;
	buf.normals   = g_normals;
	buf.albedo    = g_albedo;

	// make a screen quad

	float verts[] = {
		// X     Y
		-1.f, -1.f, // 0  1-------3
		-1.f,  1.f, // 1  |       |
		 1.f, -1.f, // 2  |       |
		 1.f,  1.f  // 3  0-------2
	};

	float tex_coords[]{
		// X   Y
		0.f, 0.f, // 0  1-------3
		0.f, 1.f, // 1  |       |
		1.f, 0.f, // 2  |       |
		1.f, 1.f  // 3  0-------2
	};

	uint indicies[] = {
		0,2,3,
		3,1,0
	};

	glGenVertexArrays(1, &buf.VAO);
	glBindVertexArray(buf.VAO);

	glGenBuffers(1, &buf.VBO);
	glBindBuffer(GL_ARRAY_BUFFER, buf.VBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(verts) + sizeof(tex_coords), NULL, GL_STATIC_DRAW);
	glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(verts), verts);
	glBufferSubData(GL_ARRAY_BUFFER, sizeof(verts), sizeof(tex_coords), tex_coords);

	glGenBuffers(1, &buf.EBO);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, buf.EBO);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indicies), indicies, GL_STATIC_DRAW);

	uint offset = 0;
	GLint vert_attrib = 0; // vertex position
	glVertexAttribPointer(vert_attrib, 2, GL_FLOAT, GL_FALSE, sizeof(vec2), (void*)offset);
	glEnableVertexAttribArray(vert_attrib);

	offset += sizeof(verts);
	GLint tex_attrib = 1; // texture coordinates
	glVertexAttribPointer(tex_attrib, 2, GL_FLOAT, GL_FALSE, sizeof(vec2), (void*)offset);
	glEnableVertexAttribArray(tex_attrib);

	return buf;
}
void draw(G_Buffer g_buffer)
{
	glActiveTexture(GL_TEXTURE0); glBindTexture(GL_TEXTURE_2D, g_buffer.positions);
	glActiveTexture(GL_TEXTURE1); glBindTexture(GL_TEXTURE_2D, g_buffer.normals);
	glActiveTexture(GL_TEXTURE2); glBindTexture(GL_TEXTURE_2D, g_buffer.albedo);

	glBindVertexArray(g_buffer.VAO);
	glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
}

/* -- deferred rendering cheat sheet --

	position = texture_unit 0
	normal   = texture_unit 1
	albedo   = texture_unit 2

	GL_FRAMEBUFFER = 0 for default framebuffer
*/

// -------------------- Lighting ------------------- //

Shader make_lighting_shader()
{
	Shader lighting_shader = {};
	load(&lighting_shader, "assets/shaders/lighting.vert", "assets/shaders/lighting.frag");
	bind(lighting_shader);

	return lighting_shader;
}

#define NUM_LIGHTS 4

struct Point_Light
{
	vec3 position, color;
	float radiance;
};

struct Light_Renderer
{
	Point_Light lights[NUM_LIGHTS];

	Mesh_Renderer sphere_mesh;
	Shader mesh_shader, light_shader;
};

void init(Light_Renderer* renderer)
{
	renderer->lights[0] = { vec3(9, 1, 3) , vec3(1, 0, 0), 1 };
	renderer->lights[1] = { vec3(4, 1, 9) , vec3(0, 1, 0), 1 };
	renderer->lights[2] = { vec3(2, 1, 2) , vec3(0, 0, 1), 1 };
	renderer->lights[3] = { vec3(0, 1, 0) , vec3(1, 1, 1), 1 };

	renderer->lights[0] = { vec3(9, 1, 3) , vec3(1), 2 };
	renderer->lights[1] = { vec3(4, 1, 9) , vec3(1), 4 };
	renderer->lights[2] = { vec3(2, 1, 2) , vec3(1), 6 };
	renderer->lights[3] = { vec3(0, 1, 0) , vec3(1), 3 };

	const char* meshes[] = { "assets/meshes/basic/sphere.mesh" };

	load(&renderer->sphere_mesh, meshes, NUM_LIGHTS * sizeof(Point_Light));
	load(&renderer->mesh_shader, "assets/shaders/transform/light.vert", "assets/shaders/mesh.frag");

	mesh_add_attrib_vec3 (2, sizeof(Point_Light), 0 * sizeof(vec3)); // position
	mesh_add_attrib_mat3 (3, sizeof(Point_Light), 1 * sizeof(vec3)); // color
	mesh_add_attrib_float(4, sizeof(Point_Light), 2 * sizeof(vec3)); // radiance

	update(renderer->sphere_mesh, sizeof(Point_Light), (byte*)&renderer->lights);
}
void update(Light_Renderer* renderer, Shader lighting_shader)
{
	set_vec3(lighting_shader, "light_positions[0]", renderer->lights[0].position);
	set_vec3(lighting_shader, "light_positions[1]", renderer->lights[1].position);
	set_vec3(lighting_shader, "light_positions[2]", renderer->lights[2].position);
	set_vec3(lighting_shader, "light_positions[3]", renderer->lights[3].position);

	set_vec3(lighting_shader, "light_colors[0]", renderer->lights[0].color);
	set_vec3(lighting_shader, "light_colors[1]", renderer->lights[1].color);
	set_vec3(lighting_shader, "light_colors[2]", renderer->lights[2].color);
	set_vec3(lighting_shader, "light_colors[3]", renderer->lights[3].color);

	set_float(lighting_shader, "radiances[0]", renderer->lights[0].radiance);
	set_float(lighting_shader, "radiances[1]", renderer->lights[1].radiance);
	set_float(lighting_shader, "radiances[2]", renderer->lights[2].radiance);
	set_float(lighting_shader, "radiances[3]", renderer->lights[3].radiance);

	update(renderer->sphere_mesh, NUM_LIGHTS * sizeof(Point_Light), (byte*)renderer->lights);
}
void draw(Light_Renderer* renderer, mat4 proj_view)
{
	bind(renderer->mesh_shader);
	set_mat4(renderer->mesh_shader, "proj_view", proj_view);

	glCullFace(GL_FRONT);
	renderer->sphere_mesh.meshes[0].num_instances = NUM_LIGHTS;
	draw(renderer->sphere_mesh);
	glCullFace(GL_BACK);
}

// ------------------ 2D Rendering ----------------- //

// maybe just make one master renderer with all the quads in it, no need to overengineer
struct Drawable_Mesh_2D
{
	GLuint VAO, VBO, EBO;
};

struct Drawable_Mesh_2D_UV
{
	GLuint VAO, VBO, EBO;
};

void init(Drawable_Mesh_2D* mesh, uint reserved_mem_size = 0)
{
	float verts[] = {
		// X     Y
		-1.f, -1.f, // 0  1-------3
		-1.f,  1.f, // 1  |       |
		 1.f, -1.f, // 2  |       |
		 1.f,  1.f  // 3  0-------2
	};

	uint indicies[] = {
		0,2,3,
		3,1,0
	};

	uint offset = reserved_mem_size;

	glGenVertexArrays(1, &mesh->VAO);
	glBindVertexArray(mesh->VAO);

#define RENDER_MEM_SIZE (reserved_mem_size + sizeof(verts))
	glGenBuffers(1, &mesh->VBO);
	glBindBuffer(GL_ARRAY_BUFFER, mesh->VBO);
	glBufferData(GL_ARRAY_BUFFER, RENDER_MEM_SIZE, NULL, GL_STATIC_DRAW);
	glBufferSubData(GL_ARRAY_BUFFER, offset, sizeof(verts), verts);
#undef RENDER_MEM_SIZE

	glGenBuffers(1, &mesh->EBO);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mesh->EBO);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indicies), indicies, GL_STATIC_DRAW);

	offset = reserved_mem_size;
	{
		GLint vert_attrib = 0; // position of a vertex
		glVertexAttribPointer(vert_attrib, 2, GL_FLOAT, GL_FALSE, sizeof(vec2), (void*)offset);
		glEnableVertexAttribArray(vert_attrib);
	}
}
void update(Drawable_Mesh_2D mesh, uint vb_size = NULL, byte* vb_data = NULL)
{
	if (vb_size > 0)
	{
		glBindBuffer(GL_ARRAY_BUFFER, mesh.VBO);
		glBufferSubData(GL_ARRAY_BUFFER, 0, vb_size, vb_data);
	}
}
void draw(Drawable_Mesh_2D mesh, uint num_instances = 1)
{
	glBindVertexArray(mesh.VAO);
	glDrawElementsInstanced(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0, num_instances);
}

void init(Drawable_Mesh_2D_UV* mesh, uint reserved_mem_size = 0)
{
	float verts[] = {
		// X     Y
		-1.f, -1.f, // 0  1-------3
		-1.f,  1.f, // 1  |       |
		 1.f, -1.f, // 2  |       |
		 1.f,  1.f  // 3  0-------2
	};

	uint indicies[] = {
		0,2,3,
		3,1,0
	};

	uint offset = reserved_mem_size;

	glGenVertexArrays(1, &mesh->VAO);
	glBindVertexArray(mesh->VAO);

#define RENDER_MEM_SIZE (reserved_mem_size + sizeof(verts))
	glGenBuffers(1, &mesh->VBO);
	glBindBuffer(GL_ARRAY_BUFFER, mesh->VBO);
	glBufferData(GL_ARRAY_BUFFER, RENDER_MEM_SIZE, NULL, GL_STATIC_DRAW);
	glBufferSubData(GL_ARRAY_BUFFER, offset, sizeof(verts), verts);
#undef RENDER_MEM_SIZE

	glGenBuffers(1, &mesh->EBO);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mesh->EBO);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indicies), indicies, GL_STATIC_DRAW);

	offset = reserved_mem_size;
	{
		GLint vert_attrib = 0; // position of a vertex
		glVertexAttribPointer(vert_attrib, 2, GL_FLOAT, GL_FALSE, sizeof(vec2), (void*)offset);
		glEnableVertexAttribArray(vert_attrib);
	}
}
void update(Drawable_Mesh_2D_UV mesh, uint vb_size = NULL, byte* vb_data = NULL)
{
	if (vb_size > 0)
	{
		glBindBuffer(GL_ARRAY_BUFFER, mesh.VBO);
		glBufferSubData(GL_ARRAY_BUFFER, 0, vb_size, vb_data);
	}
}
void draw(Drawable_Mesh_2D_UV mesh, uint num_instances = 1)
{
	glBindVertexArray(mesh.VAO);
	glDrawElementsInstanced(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0, num_instances);
}

// -------------------- Animation ------------------ //

#define MAX_ANIM_BONES 16

struct Animation
{
	uint num_bones, num_frames;

	mat4  ibm       [MAX_ANIM_BONES]; // inverse-bind matrices
	mat4* keyframes [MAX_ANIM_BONES]; // animation keyframes
	int   parents   [MAX_ANIM_BONES]; // indices of parent bones
};

void load(Animation* anim, const char* path)
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
void update_animation_pose(Animation* anim, mat4* pose, uint frame_1, uint frame_2, float mix)
{
	// animation frames
	mat4* keyframes = Alloc(mat4, anim->num_bones);

	for (uint i = 0; i < anim->num_bones; i++)
	{
		keyframes[i] = lerp(anim->keyframes[i][frame_1], anim->keyframes[i][frame_2], mix);
	}

	// pose = final pose of the skeleton

	pose[0] = keyframes[0]; //glm::rotate(ToRadians(20), vec3(0,1,0));
	for (uint i = 1; i < anim->num_bones; ++i)
	{
		pose[i] = keyframes[i] * pose[anim->parents[i]];
	}

	for (uint i = 0; i < anim->num_bones; ++i)
	{
		pose[i] = anim->ibm[i] * pose[i];
	}

	free(keyframes);
}
void play_animation(Animation* anim, mat4* pose, float elapsed_time, float duration, uint start_frame = 0, uint end_frame = 0)
{
	// assert(a bunch of stuff)
	//if (end_frame <= 0) end_frame = anim->num_frames;
	uint num_frames = end_frame - start_frame;

	// animation progress
	float progress = elapsed_time / duration;
	float frame_number = progress * num_frames;
	float frame_progress = fract(frame_number);

	// animation frames
	uint this_frame = (uint)frame_number + start_frame;
	uint next_frame = this_frame + 1;

	// this exists because the above math is broken
	// for some reason(probably floating point error)
	// we end up with off-by-one(+1) frame errors & the animation glitches
	// i.e. frame == anim->num_frames
	if (next_frame >= anim->num_frames)
	{
		this_frame = next_frame = anim->num_frames - 1;
		frame_progress = 1;
	}

	//out(elapsed_time << '/' << duration << '=' << frame_number);
	//out(this_frame << ':' << next_frame << ':' << frame_progress << '\n');
	update_animation_pose(anim, pose, this_frame, next_frame, frame_progress);
}

// -------------------- 3D Camera ------------------ //

#define DIR_FORWARD	0
#define DIR_BACKWARD	1
#define DIR_LEFT	2
#define DIR_RIGHT	3

struct Camera
{
	vec3 position;
	vec3 front, right, up;
	float yaw, pitch;
	float trauma;
};

void camera_update_dir(Camera* camera, float dx, float dy, float dtime, float sensitivity = 0.003)
{
	if (camera->trauma > 1) camera->trauma = 1;
	if (camera->trauma > 0) camera->trauma -= dtime;
	else camera->trauma = 0;

	// camera shake
	float trauma = camera->trauma;

	vec3 p = shake(trauma);
	float shake_yaw   = ToRadians(p.x);
	float shake_pitch = ToRadians(p.y);
	float shake_roll  = ToRadians(p.z);

	camera->yaw   += (dx * sensitivity) / TWOPI;
	camera->pitch += (dy * sensitivity) / TWOPI;

	float yaw   = camera->yaw + shake_yaw;
	float pitch = camera->pitch + shake_pitch;

	// it feels a little different (better?) if we let the shake actually move the camera a little
	//camera->yaw   += shake_yaw;
	//camera->pitch += shake_pitch;

	// updating camera direction
	if (camera->pitch >  PI / 2.01) camera->pitch =  PI / 2.01;
	if (camera->pitch < -PI / 2.01) camera->pitch = -PI / 2.01;

	camera->front.y = sin(pitch);
	camera->front.x = cos(pitch) * cos(yaw);
	camera->front.z = cos(pitch) * sin(yaw);

	camera->front = normalize(camera->front);
	camera->right = normalize(cross(camera->front, vec3(0, 1, 0)));
	camera->up    = normalize(cross(camera->right, camera->front));

	mat3 roll = glm::rotate(shake_roll, camera->front);
	camera->up = roll * camera->up;
}
void camera_update_pos(Camera* camera, int direction, float distance)
{
	if (direction == DIR_FORWARD ) camera->position += camera->front * distance;
	if (direction == DIR_LEFT    ) camera->position -= camera->right * distance;
	if (direction == DIR_RIGHT   ) camera->position += camera->right * distance;
	if (direction == DIR_BACKWARD) camera->position -= camera->front * distance;
}

// -------------------- Utilities ------------------ //

#define MAX_WIREFRAMES 25

struct Mesh_Drawable
{
	vec3 color;
	vec3 scale;
	vec3 position;
	mat3 rotation;
};

void init_mesh_drawable(Mesh_Renderer* mesh, const char** paths, uint vb_size, uint num_meshes = 1)
{
	load(mesh, paths, vb_size, num_meshes);
	
	mesh_add_attrib_vec3(2, sizeof(Mesh_Drawable), 0 * sizeof(vec3)); // color
	mesh_add_attrib_vec3(3, sizeof(Mesh_Drawable), 1 * sizeof(vec3)); // scale
	mesh_add_attrib_vec3(4, sizeof(Mesh_Drawable), 2 * sizeof(vec3)); // position
	mesh_add_attrib_mat3(5, sizeof(Mesh_Drawable), 3 * sizeof(vec3)); // rotation
}

struct Wireframe_Renderer
{
	Mesh_Renderer mesh;
	Shader shader;

	uint num_wireframes;
	Mesh_Drawable wireframes[MAX_WIREFRAMES];
};

void init(Wireframe_Renderer* renderer)
{
	const char* meshes[] = {
		{"assets/meshes/basic/cube.mesh"},
		{"assets/meshes/basic/cone.mesh"},
		{"assets/meshes/basic/sphere.mesh"},
		{"assets/meshes/basic/capsule.mesh"},
		{"assets/meshes/basic/cylinder.mesh"}
	};

	load(&renderer->shader, "assets/shaders/transform/mesh.vert", "assets/shaders/mesh.frag");
	init_mesh_drawable(&renderer->mesh, meshes, sizeof(Mesh_Drawable) * MAX_WIREFRAMES, 5);
}
void update(Wireframe_Renderer* renderer)
{
	renderer->num_wireframes = 0;

	for (uint i = 0; i < MAX_WIREFRAMES; i++)
	{
		renderer->wireframes[i].color    = vec3(1, i / (float)MAX_WIREFRAMES, 1);
		renderer->wireframes[i].scale    = vec3(1);
		renderer->wireframes[i].position = vec3(i + 1, 1.5, 1);
		renderer->wireframes[i].rotation = mat3(1);
		renderer->num_wireframes++;
	}
}
void draw(Wireframe_Renderer* renderer, mat4 proj_view)
{
	bind(renderer->shader);
	set_mat4(renderer->shader, "proj_view", proj_view);

	for (uint i = 0; i < 5; i++)
	{
		renderer->mesh.meshes[i].num_instances = 5;
		renderer->mesh.meshes[i].instance_offset = 5 * i;
	}

	update(renderer->mesh, sizeof(Mesh_Drawable) * renderer->num_wireframes, (byte*)renderer->wireframes);

	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	glDisable(GL_CULL_FACE);

	draw(renderer->mesh);

	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	glEnable(GL_CULL_FACE);
}