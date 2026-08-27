#include "window.h"

/* ------------------------- *
	-       Rendering       -
 * ------------------------- */

// GeometryRenderer : drawing geometry to the G-Buffer
// 
// instance data might not be in the same order as geometry data.
// the number of instances of each mesh might vary every frame.
// 
// Therefore :
// Each mesh id can only be updated once per frame because
// while geometry data is static, we get instance data
// out of order every frame and we don't know how big each
// block will be. so for now, for each mesh must only call
// update_mesh(mesh_id, instances) a max of once per frame
// TODO : should we check this & throw an error if called twice?

// Meshes share instance-date format
// When instances of a mesh are to be drawn, we need a base_instance
// which is an offset into the master instance to where this mesh's
// instance data begins
struct DrawBuffer
{	
	struct {
		uint mesh_id;

		uint num_indices;
		uint num_instances; // can change every frame

		// These are relative to the global vertex/index count!
		uint base_index ;   // IN BYTES     : num_indices * sizeof(uint)
		uint base_vertex;   // IN VERTS     : mesh_offset / sizeof(Vertex) == vertex offset
		uint base_instance; // IN INSTANCES : num_instances * sizeof(mat4)
	} mesh_info[MAX_MESHES];

	// OpenGL gpu buffers
	GLuint geom_buffer; // handle to vertex buffer
	GLuint indx_buffer; // handle to index 
	GLuint inst_buffer; // handle to instance buffer

	// The VAO is just a recorded set of glVertexAttribPointer calls
	// that tell OpenGL how to read from specific buffers
	GLuint vao;

	// runtime buffer info
	uint max_buffer_size;

	uint num_meshes;
	uint total_instances; // total number of instances currently stored

	// tracks space in use; used as offset when reading/writing
	uint geom_size;
	uint indx_size;
	uint inst_size;

	// Generate VAO : This is where OpenGL stores data layout information
	// Generate GeometryBuffer : Mesh positions, normals, uvs
	// Generate InstanceBuffer : Mat4 for instance transforms
	// Generate IndexBuffer    : uints for drawing meshes from verts
	void init(uint buffer_size = KiloByte(256)) {

		glGenVertexArrays(1, &vao);

		// this size is for : vertex buffer, index buffer, instance-data buffer
		max_buffer_size = buffer_size;

		// generate gpu buffers : mesh vertices, mesh indices, instance data
		glGenBuffers(1, &geom_buffer); // vertex buffer
		glGenBuffers(1, &indx_buffer); // index  buffer
		glGenBuffers(1, &inst_buffer); // instance-data buffer

		// IMPORTANT : bind the vao *before* binding anything else!
		glBindVertexArray(vao);

		// ------------------------------------------------------------------------------------

		// init gpu buffers for mesh vertices(positions, normals, uvs)
		glBindBuffer(GL_ARRAY_BUFFER, geom_buffer);
		glBufferData(GL_ARRAY_BUFFER, max_buffer_size, NULL, GL_STATIC_DRAW);

		// define mesh vertex data layout : vec3 position, vec3 normal, vec2 uv
		struct MeshVertexData {
			vec3 p, n;
			vec2 uv;
		};

		// give opengl the layout of mesh vertex data
		uint stride = sizeof(MeshVertexData);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, stride, (void*)0); // vec3 position
		glEnableVertexAttribArray(0);
		glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, stride, (void*)(sizeof(vec3))); // vec3 normal
		glEnableVertexAttribArray(1);
		glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, stride, (void*)(sizeof(vec3) * 2)); // vec2 uv
		glEnableVertexAttribArray(2);

		// ------------------------------------------------------------------------------------

		// init gpu buffer for per-mesh instance data
		glBindBuffer(GL_ARRAY_BUFFER, inst_buffer);
		glBufferData(GL_ARRAY_BUFFER, max_buffer_size, NULL, GL_DYNAMIC_DRAW);

		// define per-mesh instance-data layout (mat4 instance_data)
		uint num_vertex_attribs = 3; // we have already defined attribuetes 0,1,2 above so we start at 3
		for (uint i = num_vertex_attribs; i < 4 + num_vertex_attribs; i++) // for each row of the mat4
		{
			// there are 4 floats, the next 4 are sizeof(mat4) ahead, offset by sizeof(vec4) * loop_number
			glVertexAttribPointer(i, 4, GL_FLOAT, GL_FALSE, sizeof(mat4), (void*)(sizeof(vec4) * (i - num_vertex_attribs)));
			glEnableVertexAttribArray(i);
			glVertexAttribDivisor(i, 1); // advance this attribute once per instance
		}

		// ATTRIBUTE SUMMARY
		// 0 -> vec3 position
		// 1 -> vec3 normal
		// 2 -> vec2 uv
		
		// (is it columns or rows?)
		// 3 -> mat4 column/row 1
		// 4 -> mat4 column/row 2
		// 5 -> mat4 column/row 3
		// 6 -> mat4 column/row 4

		// ------------------------------------------------------------------------------------

		// gpu index buffer : ordered uints that reference vertices for building meshes
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indx_buffer);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, max_buffer_size, NULL, GL_STATIC_DRAW);

		// ------------------------------------------------------------------------------------

		// log
		char msg[MAX_LOG_MSG_LENGTH] = {};
		snprintf(msg, MAX_LOG_MSG_LENGTH, "Init DrawBuffer | Size : [%d]", max_buffer_size);
		console->add_entry(msg, SUCCESS, RNDR);
	}

	// Check for available space, append geometry & index data from mesh data
	void append_geometry(uint mesh_id, Mesh_Data mesh_data) {

		// for storing vertex data in the correct format
		// so it can go into a gpu buffer to be drawn
		struct MeshVertexData {
			vec3 position, normal;
			vec2 uv;
		};

		uint num_vertices = mesh_data.num_vertices;
		uint num_indices  = mesh_data.num_indices;

		uint vert_data_size  = num_vertices * sizeof(MeshVertexData);
		uint index_data_size = num_indices  * sizeof(uint);

		// Ensure new mesh data will fit in gpu buffers!

		if (geom_size + vert_data_size > max_buffer_size) {
			out("Geometry buffer size exceeded! - " << geom_size + vert_data_size);
			stop;
		}

		if (indx_size + index_data_size > max_buffer_size) {
			out("Index buffer size exceeded! - " << indx_size + index_data_size);
			stop;
		}

		// update geometry buffer

		MeshVertexData* mesh_verts = Alloc(MeshVertexData, num_vertices);

		// assemble vertex buffer to be copied to gpu buffer
		for (uint i = 0; i < num_vertices; i++)
			mesh_verts[i] = MeshVertexData{ mesh_data.positions[i], mesh_data.normals[i], mesh_data.uvs[i] };

		// upload mesh data to gpu buffer
		glBindBuffer(GL_ARRAY_BUFFER, geom_buffer);
		glBufferSubData(GL_ARRAY_BUFFER, geom_size, vert_data_size, mesh_verts);

		free(mesh_verts);

		// update index buffer
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indx_buffer);
		glBufferSubData(GL_ELEMENT_ARRAY_BUFFER, indx_size, index_data_size, mesh_data.indices);

		// update mesh info
		uint mesh_index = num_meshes++;
		mesh_info[mesh_index].mesh_id     = mesh_id;
		mesh_info[mesh_index].base_vertex = geom_size / sizeof(MeshVertexData);
		mesh_info[mesh_index].num_indices = num_indices;
		mesh_info[mesh_index].base_index  = indx_size;

		// update buffer sizes, these are used later as offsets into these buffers
		geom_size += vert_data_size;
		indx_size += index_data_size;
	}

	// Append instance data for one mesh
	void append_instances(uint mesh_id, uint num_instances, mat4* instance_data)
	{
		glBindBuffer(GL_ARRAY_BUFFER, inst_buffer);

		uint instance_data_size = num_instances   * sizeof(mat4); // UNIT : BYTES
		uint instance_offset    = total_instances * sizeof(mat4); // UNIT : BYTES

		glBufferSubData(GL_ARRAY_BUFFER, instance_offset, instance_data_size, instance_data);

		// update corresponding mesh info
		for (uint i = 0; i < MAX_MESHES; i++)
		{
			if (mesh_info[i].mesh_id == mesh_id)
			{
				mesh_info[i].num_instances = num_instances;
				mesh_info[i].base_instance = total_instances; // UNIT : INSTANCES
				
				// update instance count & buffer size
				total_instances += num_instances;
				inst_size += instance_data_size;

				return;
			}
		}

		out("DRAWBUFFER ERROR : Instances do not match a stored mesh!");
		stop;
	}

	// Does not free memory, just resets counts
	void clear_instances()
	{
		total_instances = 0;
		inst_size = 0;
	}
};

// stores mesh ids + generates draw-params for opengl
struct DrawList
{
	uint meshlist[MAX_MESHES]; // mesh ids in the list

	// params for issuing a draw call on an instanced mesh, in mesh VBO order
	struct {
		uint num_indices;
		uint base_index;    // IN BYTES : num_indices * sizeof(uint)
		uint base_vertex;   // IN VERTS : mesh_offset / sizeof(Vertex)
		uint num_instances;
		uint base_instance; // IN INSTANCES : instance offset
	} mesh_params[MAX_MESHES];

	void update(DrawBuffer* db)
	{
		*this = {}; // zero memory

		for (uint i = 0; i < MAX_MESHES; i++)
		{
			if (db->mesh_info[i].mesh_id == 0)
				continue;

			meshlist[i] = db->mesh_info[i].mesh_id;

			mesh_params[i].num_indices   = db->mesh_info[i].num_indices;
			mesh_params[i].base_index    = db->mesh_info[i].base_index;
			mesh_params[i].base_vertex   = db->mesh_info[i].base_vertex;
			mesh_params[i].num_instances = db->mesh_info[i].num_instances;
			mesh_params[i].base_instance = db->mesh_info[i].base_instance;
		}
	}
};

struct GameRenderer
{
	Camera camera; // 3d camera

	// Asset Loading
	MeshLoader meshloader;
	TextureLoader txloader;
	ShaderProgram shaders[MAX_SHADERS]; // geometry shader

	// Drawing
	DrawBuffer drawbuffer; // stores geometry, indices, instance data
	DrawList   drawlist;   // per-frame draw call information

	void init()
	{
		shaders[0].create("assets/shaders/geom.vert", "assets/shaders/geom.frag");

		drawbuffer.init();
		txloader.init();

		console->add_entry((char*)"Init GameRenderer", SUCCESS, RNDR);
	}
	void add_mesh(const char* filepath)
	{
		uint mesh_id = {};
		Mesh_Data mesh_data = {};
		meshloader.load_mesh(filepath, &mesh_id);
		meshloader.load_mesh_data(mesh_id, &mesh_data);

		drawbuffer.append_geometry(mesh_id, mesh_data);

		mesh_data.release(); // no longer needed since it's now in the gpu buffer

		// log
		char msg[MAX_LOG_MSG_LENGTH] = {};
		snprintf(msg, MAX_LOG_MSG_LENGTH, "Add Mesh, id[%d], path[%s]", mesh_id, filepath);
		console->add_entry(msg, SUCCESS, RNDR);
	}
	void draw(GameWindow* window)
	{
		// if (window->focus_status) // for inventory / debug
		{
			camera.update_dir(window->mouse.dx, window->mouse.dy, 1.f / 600);
			if (window->keys.W.is_pressed) camera.position += camera.front * (1.f / 60);
			if (window->keys.S.is_pressed) camera.position -= camera.front * (1.f / 60);
			if (window->keys.D.is_pressed) camera.position += camera.right * (1.f / 60);
			if (window->keys.A.is_pressed) camera.position -= camera.right * (1.f / 60);
		}

		// Create projection-view matrix for drawing
		float fov = 45, draw_distance = 256;
		mat4 proj = perspective(fov, (float)window->screen_width / window->screen_height, 0.1f, draw_distance);
		mat4 proj_view = proj * glm::lookAt(camera.position, camera.position + camera.front, camera.up);

		// bind & clear deferred framebuffer
		glBindFramebuffer(GL_FRAMEBUFFER, window->gbuf.rendertarget.fbo);
		glClearColor(0, 0, 0, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		glBindVertexArray(drawbuffer.vao); // uv_mesh vertex layout (no animations)
		shaders[0].bind(); // uv_mesh shader (no animations)

		txloader.bind(0); // 0 = default texture

		// scene camera + (perspective) projection
		uint pv = glGetUniformLocation(shaders[0].id, "proj_view");
		glUniformMatrix4fv(pv, 1, GL_FALSE, (float*)&proj_view);

		drawlist.update(&drawbuffer); // generate opengl draw params

		for (uint i = 0; i < MAX_MESHES; i++) // draw instanced meshes from draw params
		{
			if (drawlist.meshlist[i] == 0)
				continue;

			uint num_indices     = drawlist.mesh_params[i].num_indices;
			uint index_offset    = drawlist.mesh_params[i].base_index;
			uint vertex_offset   = drawlist.mesh_params[i].base_vertex;
			uint num_instances   = drawlist.mesh_params[i].num_instances;
			uint instance_offset = drawlist.mesh_params[i].base_instance;

			glDrawElementsInstancedBaseVertexBaseInstance(GL_TRIANGLES, num_indices, GL_UNSIGNED_INT,
				(void*)index_offset, num_instances, vertex_offset, instance_offset);
		}
	}
};
