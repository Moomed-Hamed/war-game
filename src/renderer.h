#include "window.h"

#define DRAW_DISTANCE 1024.0f

// ------------------------------------------------- //
// -------------------- Shaders -------------------- //
// ------------------------------------------------- //

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

// make sure you bind a shader *before* calling these!
void set_int  (Shader shader, const char* name, int value  )
{
	glUniform1i(glGetUniformLocation(shader.id, name), value);
}
void set_float(Shader shader, const char* name, float value)
{
	glUniform1f(glGetUniformLocation(shader.id, name), value);
}
void set_vec3 (Shader shader, const char* name, vec3 value )
{
	glUniform3f(glGetUniformLocation(shader.id, name), value.x, value.y, value.z);
}
void set_mat4 (Shader shader, const char* name, mat4 value )
{
	glUniformMatrix4fv(glGetUniformLocation(shader.id, name), 1, GL_FALSE, (float*)&value);
}

// ------------------------------------------------- //
// -------------------- Textures ------------------- //
// ------------------------------------------------- //

GLuint load_texture(const char* path)
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
	if (image == NULL) out("ERROR : '" << path << "' NOT FOUND!");

	glGenTextures(1, &id);
	glBindTexture(GL_TEXTURE_2D, id);
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

// ------------------------------------------------- //
// ------------------ Mesh Loading ----------------- //
// ------------------------------------------------- //

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

// ------------------------------------------------- //
// ----------------- Mesh Rendering ---------------- //
// ------------------------------------------------- //

struct Drawable_Mesh
{
	GLuint VAO, VBO, EBO;
	uint num_indices;
};

struct Drawable_Mesh_UV
{
	GLuint VAO, VBO, EBO;
	uint num_indices;
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

void load(Drawable_Mesh* mesh, const char* path, uint reserved_mem_size = 0)
{
	Mesh_Data mesh_data;
	load(&mesh_data, path);
	mesh->num_indices = mesh_data.num_indices;

	glGenVertexArrays(1, &(mesh->VAO));
	glBindVertexArray(mesh->VAO);

	uint vertmemsize = mesh_data.num_vertices * sizeof(vec3);
	uint offset = reserved_mem_size;

#define RENDER_MEM_SIZE (reserved_mem_size + (mesh_data.num_vertices * sizeof(vec3) * 2)) // '2' for positions & normals
	glGenBuffers(1, &(mesh->VBO));
	glBindBuffer(GL_ARRAY_BUFFER, mesh->VBO);
	glBufferData(GL_ARRAY_BUFFER, RENDER_MEM_SIZE, NULL, GL_STATIC_DRAW);
	glBufferSubData(GL_ARRAY_BUFFER, offset, vertmemsize, mesh_data.positions);
	glBufferSubData(GL_ARRAY_BUFFER, offset + vertmemsize, vertmemsize, mesh_data.normals);
#undef RENDER_MEM_SIZE

	glGenBuffers(1, &(mesh->EBO));
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mesh->EBO);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, mesh_data.num_indices * sizeof(uint), mesh_data.indices, GL_STATIC_DRAW);

	free(mesh_data.positions);
	free(mesh_data.normals);
	free(mesh_data.indices);

	offset = reserved_mem_size;
	{
		GLint pos_attrib = 0; // local position of a vertex
		glVertexAttribPointer(pos_attrib, 3, GL_FLOAT, GL_FALSE, sizeof(vec3), (void*)offset);
		glEnableVertexAttribArray(pos_attrib);

		GLint norm_attrib = 1; offset += vertmemsize; // normal of a vertex
		glVertexAttribPointer(norm_attrib, 3, GL_FLOAT, GL_FALSE, sizeof(vec3), (void*)offset);
		glEnableVertexAttribArray(norm_attrib);
	}
}
void load(Drawable_Mesh_UV* mesh, const char* path, uint reserved_mem_size = 0)
{
	Mesh_Data_UV mesh_data = {};
	load(&mesh_data, path);
	mesh->num_indices = mesh_data.num_indices;

	glGenVertexArrays(1, &(mesh->VAO));
	glBindVertexArray(mesh->VAO);

	uint vertmemsize = mesh_data.num_vertices * sizeof(vec3);
	uint texmemsize  = mesh_data.num_vertices * sizeof(vec2);
	uint offset = reserved_mem_size;

#define RENDER_MEM_SIZE (reserved_mem_size + (vertmemsize * 2) + texmemsize)
	glGenBuffers(1, &(mesh->VBO));
	glBindBuffer(GL_ARRAY_BUFFER, mesh->VBO);
	glBufferData(GL_ARRAY_BUFFER, RENDER_MEM_SIZE, NULL, GL_STATIC_DRAW);
	glBufferSubData(GL_ARRAY_BUFFER, offset, vertmemsize, mesh_data.positions);
	glBufferSubData(GL_ARRAY_BUFFER, offset + vertmemsize, vertmemsize, mesh_data.normals);
	glBufferSubData(GL_ARRAY_BUFFER, offset + vertmemsize + vertmemsize, texmemsize, mesh_data.textures);
#undef RENDER_MEM_SIZE

	glGenBuffers(1, &(mesh->EBO));
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mesh->EBO);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, mesh_data.num_indices * sizeof(uint), mesh_data.indices, GL_STATIC_DRAW);

	free(mesh_data.positions);
	free(mesh_data.normals);
	free(mesh_data.textures);
	free(mesh_data.indices);

	offset = reserved_mem_size;
	{
		GLint pos_attrib = 0; // position
		glVertexAttribPointer(pos_attrib, 3, GL_FLOAT, GL_FALSE, sizeof(vec3), (void*)offset);
		glEnableVertexAttribArray(pos_attrib);

		GLint norm_attrib = 1; offset += vertmemsize; // normal
		glVertexAttribPointer(norm_attrib, 3, GL_FLOAT, GL_FALSE, sizeof(vec3), (void*)offset);
		glEnableVertexAttribArray(norm_attrib);

		GLint tex_attrib = 2; offset += vertmemsize; // texture coordinates
		glVertexAttribPointer(tex_attrib, 2, GL_FLOAT, GL_FALSE, sizeof(vec2), (void*)offset);
		glEnableVertexAttribArray(tex_attrib);
	}
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

#define RENDER_MEM_SIZE (reserved_mem_size + (vertmemsize + vertmemsize + vertmemsize + bonememsize + texmemsize)) // positions, normals, weights, bones, and textures
	glGenBuffers(1, &(mesh->VBO));
	glBindBuffer(GL_ARRAY_BUFFER, mesh->VBO);
	glBufferData(GL_ARRAY_BUFFER, RENDER_MEM_SIZE, NULL, GL_STATIC_DRAW);
	glBufferSubData(GL_ARRAY_BUFFER, offset + (vertmemsize * 0), vertmemsize, mesh_data.positions);
	glBufferSubData(GL_ARRAY_BUFFER, offset + (vertmemsize * 1), vertmemsize, mesh_data.normals  );
	glBufferSubData(GL_ARRAY_BUFFER, offset + (vertmemsize * 2), vertmemsize, mesh_data.weights  );
	glBufferSubData(GL_ARRAY_BUFFER, offset + (vertmemsize * 3), bonememsize, mesh_data.bones    );
	glBufferSubData(GL_ARRAY_BUFFER, offset + (vertmemsize * 3) + bonememsize, texmemsize , mesh_data.textures );
#undef RENDER_MEM_SIZE

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

void update(Drawable_Mesh mesh, uint vb_size, byte* vb_data)
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

void draw(Drawable_Mesh mesh, uint num_instances = 1)
{
	glBindVertexArray(mesh.VAO);
	glDrawElementsInstanced(GL_TRIANGLES, mesh.num_indices, GL_UNSIGNED_INT, 0, num_instances);
}
void draw(Drawable_Mesh_UV mesh, uint num_instances = 1)
{
	glBindVertexArray(mesh.VAO);
	glDrawElementsInstanced(GL_TRIANGLES, mesh.num_indices, GL_UNSIGNED_INT, 0, num_instances);
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

// ------------------------------------------------- //
// ---------- Deferred Rendering Pipeline ---------- //
// ------------------------------------------------- //

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
		// X     Y
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
	glDrawElementsInstanced(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0, 1);
}

/* -- deferred rendering cheat sheet --

	position = texture_unit 0
	normal   = texture_unit 1
	albedo   = texture_unit 2

	GL_FRAMEBUFFER = 0 for default framebuffer
*/

// ------------------------------------------------- //
// -------------------- Lighting ------------------- //
// ------------------------------------------------- //

Shader make_lighting_shader()
{
	Shader lighting_shader = {};
	load(&lighting_shader, "assets/shaders/lighting.vert", "assets/shaders/lighting.frag");
	bind(lighting_shader);

	vec3 light_positions[4] = { vec3(9, 1.5, 6), vec3(50), vec3(50), vec3(50) };
	vec3 light_colors   [4] = { vec3(.88, .34, .13), vec3(1), vec3(1), vec3(1) };

	set_vec3(lighting_shader, "light_positions[0]", light_positions[0]);
	set_vec3(lighting_shader, "light_positions[1]", light_positions[1]);
	set_vec3(lighting_shader, "light_positions[2]", light_positions[2]);
	set_vec3(lighting_shader, "light_positions[3]", light_positions[3]);

	set_vec3(lighting_shader, "light_colors[0]", light_colors[0]);
	set_vec3(lighting_shader, "light_colors[1]", light_colors[1]);
	set_vec3(lighting_shader, "light_colors[2]", light_colors[2]);
	set_vec3(lighting_shader, "light_colors[3]", light_colors[3]);

	return lighting_shader;
}

// ------------------------------------------------- //
// ------------------ 2D Rendering ----------------- //
// ------------------------------------------------- //

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

// ------------------------------------------------- //
// -------------------- Animation ------------------ //
// ------------------------------------------------- //

#define MAX_ANIM_BONES 16

struct Animation
{
	uint num_bones, num_frames;

	mat4  ibm       [MAX_ANIM_BONES]; // inverse-bind matrices
	mat4* keyframes [MAX_ANIM_BONES];
	int   parents   [MAX_ANIM_BONES]; // indices of parent bones

	// playback
	uint current_frame;
	float timer;
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

	anim->current_frame = 0;
	anim->timer = 1.f / anim->num_frames;
}
void update_animation_pose(Animation* anim, mat4* poses, uint frame_1, uint frame_2, float mix)
{
	mat4* keyframes = Alloc(mat4, anim->num_bones);

	for (uint i = 0; i < anim->num_bones; i++)
	{
		keyframes[i] = lerp(anim->keyframes[i][frame_1], anim->keyframes[i][frame_2], mix);
	}

	poses[0] = keyframes[0];
	for (uint i = 1; i < anim->num_bones; ++i)
	{
		poses[i] = keyframes[i] * poses[anim->parents[i]];
	}

	for (uint i = 0; i < anim->num_bones; ++i)
	{
		poses[i] = anim->ibm[i] * poses[i];
	}

	free(keyframes);
}
void update_animation(Animation* anim, mat4* poses, float dtime)
{
	anim->timer += dtime;

	if (anim->timer > 1)
	{
		anim->timer = 0;
		if (++anim->current_frame > anim->num_frames - 1) anim->current_frame = 0;
		//out(anim->current_frame);
	}

	mat4* keyframes = Alloc(mat4, anim->num_bones);

	uint frame = anim->current_frame;
	uint next_frame = frame + 1;

	if (frame >= anim->num_frames)
	{
		frame = 0;
		next_frame = 1;
	}
	if (next_frame >= anim->num_frames)
	{
		next_frame = 0;
	} //out(frame << ',' << next_frame);

	//float mix = lerp_spring(anim->timer / 1.f, 10, 15);
	//float mix = lerp(0, 1, anim->timer / 1.f);
	//float mix = lerp(0, 1, sin(PI - (anim->timer * PI * .5)));
	float mix = lerp(0, 1, sin(anim->timer * PI * .5));
	//float mix = bounce(anim->timer / 1.f);
	update_animation_pose(anim, poses, frame, next_frame, mix);

	free(keyframes);
}

// ------------------------------------------------- //
// -------------------- 3D Camera ------------------ //
// ------------------------------------------------- //

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
	// camera shake
	float trauma = camera->trauma;

	static uint offset = random_uint() % 16;

	if (camera->trauma > 1) camera->trauma = 1;
	if (camera->trauma > 0) camera->trauma -= dtime;
	else
	{
		camera->trauma = 0;
		offset = random_uint() % 16;
	}

	float p1 = ((perlin((trauma + offset + 0) * 1000) * 2) - 1) * trauma;
	float p2 = ((perlin((trauma + offset + 1) * 2000) * 2) - 1) * trauma;
	float p3 = ((perlin((trauma + offset + 2) * 3000) * 2) - 1) * trauma;

	float shake_yaw   = ToRadians(p1);
	float shake_pitch = ToRadians(p2);
	float shake_roll  = ToRadians(p3);

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