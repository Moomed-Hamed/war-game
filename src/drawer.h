#include "window.h"

/* ------------------------- *
	-       Rendering       -
 * ------------------------- */

// GameRenderer : draws meshes into the G-Buffer (deferred), instanced.
// instance data arrives out of order and block sizes vary, so each mesh id
// may only be appended to the draw buffer once per frame.

// vertex data as it lives in the gpu buffer : position, normal, uv.
// MUST match the glVertexAttribPointer layout set up in DrawBuffer::init()
struct MeshVertexData {
	vec3 position, normal;
	vec2 uv;
};

// meshes share the instance-data format : one mat4 per instance. each
// mesh's block starts at base_instance, an offset into the instance buffer
struct DrawBuffer
{
	struct {
		uint mesh_id;
		uint tx_id; // texture this mesh draws with (0 = default)

		uint num_vertices;
		uint num_indices;
		uint num_instances; // can change every frame

		// offsets into the shared gpu buffers
		uint base_index ;   // IN BYTES     : offset into the index buffer
		uint base_vertex;   // IN VERTS     : byte offset / sizeof(MeshVertexData)
		uint base_instance; // IN INSTANCES : offset into the instance buffer
	} mesh_info[MAX_MESHES];

	GLuint geom_buffer; // vertex buffer
	GLuint indx_buffer; // index buffer
	GLuint inst_buffer; // instance-data buffer
	GLuint vao;         // recorded set of glVertexAttribPointer calls

	// runtime buffer info
	uint max_buffer_size;
	uint num_meshes;
	uint total_instances; // instances currently stored

	uint geom_size, indx_size, inst_size; // space in use, doubles as the write offset

	// creates the vao, the three gpu buffers and the vertex/instance layouts
	void init(uint buffer_size = MegaByte(1)) { // the floor alone eats 119KB : 256 runs out fast

		glGenVertexArrays(1, &vao);
		max_buffer_size = buffer_size;

		// bind the vao *before* binding anything else!
		glBindVertexArray(vao);

		// vertex buffer + its layout : vec3 position, vec3 normal, vec2 uv
		glGenBuffers(1, &geom_buffer);
		glBindBuffer(GL_ARRAY_BUFFER, geom_buffer);
		glBufferData(GL_ARRAY_BUFFER, max_buffer_size, NULL, GL_STATIC_DRAW);

		uint stride = sizeof(MeshVertexData);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, stride, (void*)0); // position
		glEnableVertexAttribArray(0);
		glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, stride, (void*)sizeof(vec3)); // normal
		glEnableVertexAttribArray(1);
		glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, stride, (void*)(sizeof(vec3) * 2)); // uv
		glEnableVertexAttribArray(2);

		// instance buffer : one mat4 per instance, attributes 3..6, one vec4 per row
		glGenBuffers(1, &inst_buffer);
		glBindBuffer(GL_ARRAY_BUFFER, inst_buffer);
		glBufferData(GL_ARRAY_BUFFER, max_buffer_size, NULL, GL_DYNAMIC_DRAW);
		for (uint i = 3; i < 7; i++)
		{
			glVertexAttribPointer(i, 4, GL_FLOAT, GL_FALSE, sizeof(mat4), (void*)(sizeof(vec4) * (i - 3)));
			glEnableVertexAttribArray(i);
			glVertexAttribDivisor(i, 1); // advance once per instance
		}

		// index buffer : ordered uints referencing the vertices
		glGenBuffers(1, &indx_buffer);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indx_buffer);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, max_buffer_size, NULL, GL_STATIC_DRAW);

		char msg[MAX_LOG_MSG_LENGTH] = {};
		snprintf(msg, MAX_LOG_MSG_LENGTH, "Init DrawBuffer | Size : [%d]", max_buffer_size);
		console->add_entry(msg, SUCCESS, RNDR);
	}

	// append a mesh's vertex + index data to the gpu buffers.
	// returns false (after logging) if they don't have room
	bool append_geometry(uint mesh_id, Mesh_Data mesh_data, uint tx_id = 0) {
		if (num_meshes >= MAX_MESHES) {
			char msg[MAX_LOG_MSG_LENGTH];
			snprintf(msg, MAX_LOG_MSG_LENGTH, "Too many meshes! [MAX_MESHES = %d]", MAX_MESHES);
			console->add_entry(msg, FIXME, RNDR);
			return false; // don't write past mesh_info
		}

		uint num_vertices = mesh_data.num_vertices;
		uint num_indices  = mesh_data.num_indices;

		uint vert_data_size  = num_vertices * sizeof(MeshVertexData);
		uint index_data_size = num_indices  * sizeof(uint);

		if (geom_size + vert_data_size > max_buffer_size) {
			char msg[MAX_LOG_MSG_LENGTH];
			snprintf(msg, MAX_LOG_MSG_LENGTH, "Geometry buffer full! [%d + %d > %d bytes]",
				geom_size, vert_data_size, max_buffer_size);
			console->add_entry(msg, FIXME, RNDR);
			return false; // don't write past the gpu buffer
		}
		if (indx_size + index_data_size > max_buffer_size) {
			char msg[MAX_LOG_MSG_LENGTH];
			snprintf(msg, MAX_LOG_MSG_LENGTH, "Index buffer full! [%d + %d > %d bytes]",
				indx_size, index_data_size, max_buffer_size);
			console->add_entry(msg, FIXME, RNDR);
			return false;
		}

		// interleave the vertex rows, upload, then the indices
		MeshVertexData* mesh_verts = Alloc(MeshVertexData, num_vertices);
		for (uint i = 0; i < num_vertices; i++)
			mesh_verts[i] = MeshVertexData{ mesh_data.positions[i], mesh_data.normals[i], mesh_data.uvs[i] };

		glBindBuffer(GL_ARRAY_BUFFER, geom_buffer);
		glBufferSubData(GL_ARRAY_BUFFER, geom_size, vert_data_size, mesh_verts);
		free(mesh_verts);

		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indx_buffer);
		glBufferSubData(GL_ELEMENT_ARRAY_BUFFER, indx_size, index_data_size, mesh_data.indices);

		// record where this mesh landed, then move the write offsets
		uint mesh_index = num_meshes++;
		mesh_info[mesh_index].mesh_id      = mesh_id;
		mesh_info[mesh_index].tx_id        = tx_id;
		mesh_info[mesh_index].num_vertices = num_vertices;
		mesh_info[mesh_index].base_vertex  = geom_size / sizeof(MeshVertexData);
		mesh_info[mesh_index].num_indices  = num_indices;
		mesh_info[mesh_index].base_index   = indx_size;

		geom_size += vert_data_size;
		indx_size += index_data_size;

		return true;
	}

	// append instance data for one mesh
	void append_instances(uint mesh_id, uint num_instances, mat4* instance_data)
	{
		glBindBuffer(GL_ARRAY_BUFFER, inst_buffer);

		uint instance_data_size = num_instances   * sizeof(mat4);
		uint instance_offset    = total_instances * sizeof(mat4);

		glBufferSubData(GL_ARRAY_BUFFER, instance_offset, instance_data_size, instance_data);

		for (uint i = 0; i < MAX_MESHES; i++)
		{
			if (mesh_info[i].mesh_id == mesh_id)
			{
				mesh_info[i].num_instances = num_instances;
				mesh_info[i].base_instance = total_instances;

				total_instances += num_instances;
				inst_size += instance_data_size;
				return;
			}
		}

		// unknown mesh id : log once, don't spam (this runs every frame)
		static uint last_unknown_id = (uint)-1;
		if (mesh_id != last_unknown_id)
		{
			char msg[MAX_LOG_MSG_LENGTH];
			snprintf(msg, MAX_LOG_MSG_LENGTH, "Instances for unknown mesh id [%d]!", mesh_id);
			console->add_entry(msg, FIXME, RNDR);
			last_unknown_id = mesh_id;
		}
	}

	// does not free memory, just resets counts
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

	// draw params copied from the buffer's mesh_info, in buffer order
	struct {
		uint num_indices;
		uint base_index;    // IN BYTES : offset into the index buffer
		uint base_vertex;   // IN VERTS : offset into the vertex buffer
		uint num_instances;
		uint base_instance; // IN INSTANCES : offset into the instance buffer
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

// ---- skinned path -------------------------------------------------------------
// same buffer/list pattern, but vertices carry skinning data (3 weights + 3 bone
// ids) and the skeleton pose uploads to UBO slot 1. AnimLoader hands the renderer
// a finished pose; the row-major file layout already got transposed at load.

// vertex data as it lives in the gpu buffer : position, normal, uv, weights, bones.
// MUST match the glVertexAttribPointer layout set up in DrawBufferAnim::init()
// and the inputs of assets/shaders/anim.vert
struct MeshVertexDataAnim {
	vec3 position, normal;
	vec2 uv;
	vec3 weights; // top-3 skinning weights (sum to 1)
	ivec3 bones;  // bone id for each weight
}; // 56 bytes

// the same once-per-frame instancing rule as DrawBuffer applies here
struct DrawBufferAnim
{
	struct {
		uint mesh_id;
		uint tx_id;        // texture this mesh draws with (0 = default)
		uint anim_player;  // AnimLoader player that poses it ((uint)-1 = identity)

		uint num_vertices;
		uint num_indices;
		uint num_instances; // can change every frame

		// offsets into the shared gpu buffers
		uint base_index ;   // IN BYTES     : offset into the index buffer
		uint base_vertex;   // IN VERTS     : byte offset / sizeof(MeshVertexDataAnim)
		uint base_instance; // IN INSTANCES : offset into the instance buffer
	} mesh_info[MAX_MESHES];

	GLuint geom_buffer; // vertex buffer
	GLuint indx_buffer; // index buffer
	GLuint inst_buffer; // instance-data buffer
	GLuint pose_buffer; // skeleton UBO : mat4 joint_transforms[16] at binding slot 1

	mat4 identity_pose[MAX_ANIM_BONES]; // uploaded for meshes with no player bound

	GLuint vao; // recorded set of glVertexAttribPointer calls

	// runtime buffer info
	uint max_buffer_size;
	uint num_meshes;
	uint total_instances; // instances currently stored

	uint geom_size, indx_size, inst_size; // space in use, doubles as the write offset

	// creates the vao, the four gpu buffers and the vertex/instance layouts
	void init(uint buffer_size = MegaByte(1)) { // skinned vertices are 56 bytes : 256KB runs out fast

		glGenVertexArrays(1, &vao);
		max_buffer_size = buffer_size;

		// bind the vao *before* binding anything else!
		glBindVertexArray(vao);

		// vertex buffer + its layout : vec3 position, vec3 normal, vec2 uv, vec3 weights, ivec3 bones
		glGenBuffers(1, &geom_buffer);
		glBindBuffer(GL_ARRAY_BUFFER, geom_buffer);
		glBufferData(GL_ARRAY_BUFFER, max_buffer_size, NULL, GL_STATIC_DRAW);

		uint stride = sizeof(MeshVertexDataAnim);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, stride, (void*)0); // position
		glEnableVertexAttribArray(0);
		glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, stride, (void*)sizeof(vec3)); // normal
		glEnableVertexAttribArray(1);
		glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, stride, (void*)(sizeof(vec3) * 2)); // uv
		glEnableVertexAttribArray(2);
		glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, stride, (void*)(sizeof(vec3) * 2 + sizeof(vec2))); // weights
		glEnableVertexAttribArray(3);
		glVertexAttribIPointer(4, 3, GL_INT, stride, (void*)(sizeof(vec3) * 2 + sizeof(vec2) + sizeof(vec3))); // bone ids are indices, not floats
		glEnableVertexAttribArray(4);

		// instance buffer : one mat4 per instance, attributes 5..8, one vec4 per row
		glGenBuffers(1, &inst_buffer);
		glBindBuffer(GL_ARRAY_BUFFER, inst_buffer);
		glBufferData(GL_ARRAY_BUFFER, max_buffer_size, NULL, GL_DYNAMIC_DRAW);
		for (uint i = 5; i < 9; i++)
		{
			glVertexAttribPointer(i, 4, GL_FLOAT, GL_FALSE, sizeof(mat4), (void*)(sizeof(vec4) * (i - 5)));
			glEnableVertexAttribArray(i);
			glVertexAttribDivisor(i, 1); // advance once per instance
		}

		// index buffer : ordered uints referencing the vertices
		glGenBuffers(1, &indx_buffer);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indx_buffer);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, max_buffer_size, NULL, GL_STATIC_DRAW);

		// skeleton UBO : one mat4 per bone, std140, binding slot 1
		glGenBuffers(1, &pose_buffer);
		glBindBuffer(GL_UNIFORM_BUFFER, pose_buffer);
		glBufferData(GL_UNIFORM_BUFFER, MAX_ANIM_BONES * sizeof(mat4), NULL, GL_DYNAMIC_DRAW);
		glBindBufferBase(GL_UNIFORM_BUFFER, 1, pose_buffer);

		// identity pose : with no player bound, the mesh draws unskinned instead of collapsed
		for (uint i = 0; i < MAX_ANIM_BONES; i++) identity_pose[i] = mat4(1);
		glBufferSubData(GL_UNIFORM_BUFFER, 0, sizeof(identity_pose), identity_pose);

		char msg[MAX_LOG_MSG_LENGTH] = {};
		snprintf(msg, MAX_LOG_MSG_LENGTH, "Init DrawBufferAnim | Size : [%d]", max_buffer_size);
		console->add_entry(msg, SUCCESS, RNDR);
	}

	// upload the skeleton pose : mat4 joint_transforms[num_bones] at binding slot 1
	void update_pose(mat4* pose, uint num_bones) {
		glBindBuffer(GL_UNIFORM_BUFFER, pose_buffer);
		glBufferSubData(GL_UNIFORM_BUFFER, 0, num_bones * sizeof(mat4), pose);
	}

	// append a mesh's vertex + index data to the gpu buffers.
	// returns false (after logging) if they don't have room
	bool append_geometry(uint mesh_id, Mesh_Data_Anim mesh_data, uint tx_id = 0) {
		if (num_meshes >= MAX_MESHES) {
			char msg[MAX_LOG_MSG_LENGTH];
			snprintf(msg, MAX_LOG_MSG_LENGTH, "Too many meshes! [MAX_MESHES = %d]", MAX_MESHES);
			console->add_entry(msg, FIXME, RNDR);
			return false; // don't write past mesh_info
		}

		uint num_vertices = mesh_data.num_vertices;
		uint num_indices  = mesh_data.num_indices;

		uint vert_data_size  = num_vertices * sizeof(MeshVertexDataAnim);
		uint index_data_size = num_indices  * sizeof(uint);

		if (geom_size + vert_data_size > max_buffer_size) {
			char msg[MAX_LOG_MSG_LENGTH];
			snprintf(msg, MAX_LOG_MSG_LENGTH, "Geometry buffer full! [%d + %d > %d bytes]",
				geom_size, vert_data_size, max_buffer_size);
			console->add_entry(msg, FIXME, RNDR);
			return false; // don't write past the gpu buffer
		}
		if (indx_size + index_data_size > max_buffer_size) {
			char msg[MAX_LOG_MSG_LENGTH];
			snprintf(msg, MAX_LOG_MSG_LENGTH, "Index buffer full! [%d + %d > %d bytes]",
				indx_size, index_data_size, max_buffer_size);
			console->add_entry(msg, FIXME, RNDR);
			return false;
		}

		// interleave the vertex rows, upload, then the indices
		MeshVertexDataAnim* mesh_verts = Alloc(MeshVertexDataAnim, num_vertices);
		for (uint i = 0; i < num_vertices; i++)
			mesh_verts[i] = MeshVertexDataAnim{ mesh_data.positions[i], mesh_data.normals[i], mesh_data.uvs[i],
				mesh_data.weights[i], mesh_data.bones[i] };

		glBindBuffer(GL_ARRAY_BUFFER, geom_buffer);
		glBufferSubData(GL_ARRAY_BUFFER, geom_size, vert_data_size, mesh_verts);
		free(mesh_verts);

		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indx_buffer);
		glBufferSubData(GL_ELEMENT_ARRAY_BUFFER, indx_size, index_data_size, mesh_data.indices);

		// record where this mesh landed, then move the write offsets
		uint mesh_index = num_meshes++;
		mesh_info[mesh_index].mesh_id      = mesh_id;
		mesh_info[mesh_index].tx_id        = tx_id;
		mesh_info[mesh_index].anim_player  = (uint)-1; // identity pose until bound
		mesh_info[mesh_index].num_vertices = num_vertices;
		mesh_info[mesh_index].base_vertex  = geom_size / sizeof(MeshVertexDataAnim);
		mesh_info[mesh_index].num_indices  = num_indices;
		mesh_info[mesh_index].base_index   = indx_size;

		geom_size += vert_data_size;
		indx_size += index_data_size;

		return true;
	}

	// append instance data for one mesh
	void append_instances(uint mesh_id, uint num_instances, mat4* instance_data)
	{
		glBindBuffer(GL_ARRAY_BUFFER, inst_buffer);

		uint instance_data_size = num_instances   * sizeof(mat4);
		uint instance_offset    = total_instances * sizeof(mat4);

		glBufferSubData(GL_ARRAY_BUFFER, instance_offset, instance_data_size, instance_data);

		for (uint i = 0; i < MAX_MESHES; i++)
		{
			if (mesh_info[i].mesh_id == mesh_id)
			{
				mesh_info[i].num_instances = num_instances;
				mesh_info[i].base_instance = total_instances;

				total_instances += num_instances;
				inst_size += instance_data_size;
				return;
			}
		}

		// unknown mesh id : log once, don't spam (this runs every frame)
		static uint last_unknown_id = (uint)-1;
		if (mesh_id != last_unknown_id)
		{
			char msg[MAX_LOG_MSG_LENGTH];
			snprintf(msg, MAX_LOG_MSG_LENGTH, "Instances for unknown mesh id [%d]!", mesh_id);
			console->add_entry(msg, FIXME, RNDR);
			last_unknown_id = mesh_id;
		}
	}

	// does not free memory, just resets counts
	void clear_instances()
	{
		total_instances = 0;
		inst_size = 0;
	}
};

// stores mesh ids + generates draw-params for opengl (skinned pass)
struct DrawListAnim
{
	uint meshlist[MAX_MESHES]; // mesh ids in the list

	// draw params copied from the buffer's mesh_info, in buffer order
	struct {
		uint num_indices;
		uint base_index;    // IN BYTES : offset into the index buffer
		uint base_vertex;   // IN VERTS : offset into the vertex buffer
		uint num_instances;
		uint base_instance; // IN INSTANCES : offset into the instance buffer
	} mesh_params[MAX_MESHES];

	void update(DrawBufferAnim* db)
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
	// per-frame draw statistics, filled by draw() and shown in the console (Render tab)
	struct {
		uint draw_calls;
		uint meshes_drawn;
		uint instances;
		uint triangles;
		uint vertices;
	} stats;

	// asset loading
	MeshLoader     meshloader;
	MeshLoaderAnim meshloader_anim;
	AnimLoader     animloader; // clips + players : the only animation state in the engine
	TextureLoader  txloader;
	ShaderProgram  shaders[MAX_SHADERS]; // 0 = static meshes, 1 = skinned meshes

	// drawing : static vertex layout
	DrawBuffer drawbuffer; // stores geometry, indices, instance data
	DrawList   drawlist;   // per-frame draw call information

	// drawing : skinned vertex layout (pose UBO + skin weights)
	DrawBufferAnim drawbuffer_anim;
	DrawListAnim   drawlist_anim;

	void init()
	{
		shaders[0].create("assets/shaders/geom.vert", "assets/shaders/geom.frag");
		shaders[1].create("assets/shaders/anim.vert", "assets/shaders/geom.frag"); // same frag as the static pass

		drawbuffer.init();
		drawbuffer_anim.init();
		txloader.init();
		animloader.init(); // playback defaults (calloc skips them)

		console->add_entry((char*)"Init GameRenderer", SUCCESS, RNDR);
	}

	// returns the mesh id (0 = failed, error already logged)
	uint add_mesh(const char* filepath, const char* texture = NULL)
	{
		uint mesh_id = {};
		Mesh_Data mesh_data = {};
		meshloader.load_mesh(filepath, &mesh_id);
		if (mesh_id == 0) return 0; // load failed, error already logged

		meshloader.load_mesh_data(mesh_id, &mesh_data);
		if (mesh_data.num_vertices == 0) return 0; // no data, error already logged

		return add_mesh(mesh_data, texture); // mesh_data is freed by the overload below
	}

	// mesh built in code (MeshFactory) : same trip into the draw buffer.
	// its id is borrowed from the mesh loader's counter so mesh ids never collide
	uint add_mesh(Mesh_Data mesh_data, const char* texture = NULL)
	{
		uint mesh_id = ++meshloader.num_loaded;
		uint tx_id   = texture ? txloader.load(texture) : 0; // 0 = default texture

		bool appended = drawbuffer.append_geometry(mesh_id, mesh_data, tx_id);
		mesh_data.release(); // consumed : freed whether it made it in or not
		if (!appended) return 0; // buffer full, error already logged

		char msg[MAX_LOG_MSG_LENGTH] = {};
		snprintf(msg, MAX_LOG_MSG_LENGTH, "Add Mesh, id[%d], generated, texture[%s]",
			mesh_id, texture ? texture : "default");
		console->add_entry(msg, SUCCESS, RNDR);

		return mesh_id;
	}

	// skinned mesh : load, then append into the animated draw buffer
	// returns the mesh id (0 = failed, error already logged)
	uint add_anim_mesh(const char* filepath, const char* texture = NULL)
	{
		uint mesh_id = {};
		Mesh_Data_Anim mesh_data = {};
		meshloader_anim.load_mesh(filepath, &mesh_id);
		if (mesh_id == 0) return 0; // load failed, error already logged

		meshloader_anim.load_mesh_data(mesh_id, &mesh_data);
		if (mesh_data.num_vertices == 0) return 0; // no data, error already logged

		uint tx_id = texture ? txloader.load(texture) : 0; // 0 = default texture

		bool appended = drawbuffer_anim.append_geometry(mesh_id, mesh_data, tx_id);
		mesh_data.release(); // no longer needed since it's now in the gpu buffer

		if (!appended) return 0; // buffer full, error already logged

		char msg[MAX_LOG_MSG_LENGTH] = {};
		snprintf(msg, MAX_LOG_MSG_LENGTH, "Add Anim Mesh, id[%d], %s", mesh_id, filepath);
		console->add_entry(msg, SUCCESS, RNDR);

		return mesh_id;
	}

	// which animation player poses this skinned mesh (unbound meshes draw the identity pose)
	void bind_player(uint mesh_id, uint player_id)
	{
		for (uint i = 0; i < MAX_MESHES; i++)
		{
			if (drawbuffer_anim.mesh_info[i].mesh_id == mesh_id)
			{
				drawbuffer_anim.mesh_info[i].anim_player = player_id;
				return;
			}
		}
		console->add_entry("bind_player : no such mesh id", FIXME, RNDR);
	}

	// the scene owns the camera : it passes this frame's matrices in
	void draw(GameWindow* window, mat4 proj_view)
	{
		// animation playback : the loader runs every player, we only ship finished poses
		animloader.update(window->dtime);

		// bind & clear deferred framebuffer (both passes draw into it)
		glBindFramebuffer(GL_FRAMEBUFFER, window->gbuf.rendertarget.fbo);
		glClearColor(0, 0, 0, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		stats = {}; // reset per-frame draw statistics (read by the console)

		// pass 1 : static meshes (uv vertex layout)
		if (drawbuffer.num_meshes)
		{
			glBindVertexArray(drawbuffer.vao);
			shaders[0].bind();
			glUniformMatrix4fv(glGetUniformLocation(shaders[0].id, "proj_view"), 1, GL_FALSE, (float*)&proj_view);

			drawlist.update(&drawbuffer);
			draw_pass();
		}

		// pass 2 : skinned meshes (pose UBO + skin vertex layout)
		if (drawbuffer_anim.num_meshes)
		{
			glBindVertexArray(drawbuffer_anim.vao);
			shaders[1].bind();
			glUniformMatrix4fv(glGetUniformLocation(shaders[1].id, "proj_view"), 1, GL_FALSE, (float*)&proj_view);

			drawlist_anim.update(&drawbuffer_anim);
			draw_pass_anim();
		}
	}

	// issue the instanced draw calls for the static pass, tallying stats
	void draw_pass()
	{
		uint bound_tx = (uint)-1; // rebind only when the mesh draws with a different texture

		for (uint i = 0; i < MAX_MESHES; i++)
		{
			if (drawlist.meshlist[i] == 0)
				continue;

			if (drawbuffer.mesh_info[i].tx_id != bound_tx)
			{
				bound_tx = drawbuffer.mesh_info[i].tx_id;
				txloader.bind(bound_tx);
			}

			auto& mp = drawlist.mesh_params[i];
			glDrawElementsInstancedBaseVertexBaseInstance(GL_TRIANGLES, mp.num_indices, GL_UNSIGNED_INT,
				(void*)mp.base_index, mp.num_instances, mp.base_vertex, mp.base_instance);

			// record what we just submitted (mesh_info shares this index with mesh_params)
			stats.draw_calls++;
			stats.meshes_drawn++;
			stats.instances  += mp.num_instances;
			stats.triangles  += (mp.num_indices / 3) * mp.num_instances;
			stats.vertices   += drawbuffer.mesh_info[i].num_vertices * mp.num_instances;
		}
	}

	// same, for the skinned pass : each mesh also gets its player's pose
	void draw_pass_anim()
	{
		uint bound_tx = (uint)-1; // rebind only when the mesh draws with a different texture

		for (uint i = 0; i < MAX_MESHES; i++)
		{
			if (drawlist_anim.meshlist[i] == 0)
				continue;

			// pose : the player bound to this mesh (identity pose when unbound)
			uint player = drawbuffer_anim.mesh_info[i].anim_player;
			if (player < animloader.num_players)
			{
				Anim_Player& pl = animloader.players[player];
				drawbuffer_anim.update_pose(pl.pose, animloader.clips[pl.clip_id].num_bones);
			}
			else
			{
				drawbuffer_anim.update_pose(drawbuffer_anim.identity_pose, MAX_ANIM_BONES);
			}

			if (drawbuffer_anim.mesh_info[i].tx_id != bound_tx)
			{
				bound_tx = drawbuffer_anim.mesh_info[i].tx_id;
				txloader.bind(bound_tx);
			}

			auto& mp = drawlist_anim.mesh_params[i];
			glDrawElementsInstancedBaseVertexBaseInstance(GL_TRIANGLES, mp.num_indices, GL_UNSIGNED_INT,
				(void*)mp.base_index, mp.num_instances, mp.base_vertex, mp.base_instance);

			// record what we just submitted (mesh_info shares this index with mesh_params)
			stats.draw_calls++;
			stats.meshes_drawn++;
			stats.instances  += mp.num_instances;
			stats.triangles  += (mp.num_indices / 3) * mp.num_instances;
			stats.vertices   += drawbuffer_anim.mesh_info[i].num_vertices * mp.num_instances;
		}
	}
};
