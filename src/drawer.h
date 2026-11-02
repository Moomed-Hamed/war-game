#include "window.h"

/* ------------------------- *
	-       Rendering       -
 * ------------------------- */

// GameRenderer : draws meshes into the G-Buffer (deferred), instanced.
// instance data arrives in blocks : a mesh id's appends must form ONE run per
// frame (base_instance is taken at the first append, later appends only extend
// the count), so group a mesh's instances together or give it its own mesh

// vertex data as it lives in the gpu buffer : position, normal, uv.
// MUST match the glVertexAttribPointer layout set up in DrawBuffer::init()
struct MeshVertexData {
	vec3 position, normal;
	vec2 uv;
};

// mode 3 : static, no uvs. an export that carried none, so the texture drops out of the
// pipeline and the instance material is the whole colour
struct MeshVertexDataNoUV {
	vec3 position, normal;
};

// the skinned layouts : position, normal, uv, then the top-3 weights and the bone ids
// feeding them. MUST match the glVertexAttribPointer layout set up in DrawBuffer::init()
struct MeshVertexDataAnim {
	vec3 position, normal;
	vec2 uv;
	vec3 weights; // top-3 skinning weights (sum to 1)
	ivec3 bones;  // bone id for each weight
}; // 56 bytes

// mode 4 : skinned, no uvs : the same rows minus the uv column
struct MeshVertexDataAnimNoUV {
	vec3 position, normal;
	vec3 weights;
	ivec3 bones;
}; // 48 bytes

// ---- the draw modes -------------------------------------------------------------------
// a mode is a vertex layout, a stride and a shader pair. the no-uv modes leave attribute 2
// disabled, so the sampler reads its constant (0,0) and the instance material's texture
// flag decides whether that counts : no shader branch, and no second shader
enum DrawMode : uint {
	MODE_STATIC_UV = 0, // 1 : the floor, props, generated shapes
	MODE_STATIC_NOUV,   // 3 : the same meshes, built without a uv column
	MODE_ANIM_UV,       // 2 : weapons and rigs, skinned
	MODE_ANIM_NOUV,     // 4 : the same, built without a uv column
	MODE_WATER,         // 5 : forward water, drawn after the opaque frame
	DRAW_MODE_COUNT
};

// one row per mode : what the buffer init, the draw pass and the console all read.
// the table is the only list of modes in the engine
struct DrawMode_Info {
	const char* name;  // as the console lists it
	uint shader;       // index into GameRenderer::shaders
	uint vertex_size;  // bytes per vertex : the gpu stride, and the layout arithmetic
	bool skinned;      // needs the pose ubo, and a player bound to be posed
	bool has_uv;       // enables attribute 2
	bool deferred;     // writes the g-buffer : otherwise a forward pass owns it
};

const DrawMode_Info DRAW_MODE_INFO[DRAW_MODE_COUNT] = {
	{ "1  static + uv",    0, sizeof(MeshVertexData),         false, true,  true  },
	{ "3  static, no uv",  0, sizeof(MeshVertexDataNoUV),     false, false, true  },
	{ "2  skinned + uv",   1, sizeof(MeshVertexDataAnim),     true,  true,  true  },
	{ "4  skinned, no uv", 1, sizeof(MeshVertexDataAnimNoUV), true,  false, true  },
	{ "5  water (forward)",2, sizeof(MeshVertexData),         false, true,  false },
};

// one instance : the model matrix plus the material the deferred lighting reads.
// mat_a = albedo rgb + metallic, mat_b = roughness, ao, "texture provides albedo"
// flag. the g-buffer carries sRGB, so authored albedo is in display space
struct Instance {
	mat4 model;
	vec4 mat_a;
	vec4 mat_b;
};

// what a mesh with no material of its own draws with : white albedo, dielectric,
// mid roughness, full ao, texture colour kept
const Instance INSTANCE_DEFAULT = { mat4(1.f), vec4(1.f, 1.f, 1.f, 0.f), vec4(0.6f, 1.f, 1.f, 0.f) };

// an instance carrying a material but no pose : set .model afterwards.
// albedo is display space, texture_albedo = 1 keeps the mesh's texture colour
Instance material(vec3 albedo, float metallic, float roughness, float ao = 1.f, float texture_albedo = 0.f)
{
	Instance m = INSTANCE_DEFAULT;
	m.mat_a = vec4(albedo, metallic);
	m.mat_b = vec4(roughness, ao, texture_albedo, 0.f);
	return m;
}

// one gpu buffer set per mode : geometry, indices, instance data, and the vao that
// records the vertex layout. a mesh's instance block starts at base_instance, an offset
// into the instance buffer
struct DrawBuffer
{
	uint mode; // a DrawMode : fixes the stride, the attribute layout and the shader

	struct {
		uint mesh_id;
		uint tx_id;        // texture this mesh draws with (0 = default)
		uint anim_player;  // skinned : the AnimLoader player posing it ((uint)-1 = identity)

		uint num_vertices;
		uint num_indices;
		uint num_instances; // can change every frame
		mat4 last_model;    // transform the last instance was submitted with

		// offsets into the shared gpu buffers
		uint base_index ;   // IN BYTES     : offset into the index buffer
		uint base_vertex;   // IN VERTS     : byte offset / vertex size
		uint base_instance; // IN INSTANCES : offset into the instance buffer
	} mesh_info[MAX_MESHES];

	GLuint geom_buffer; // vertex buffer
	GLuint indx_buffer; // index buffer
	GLuint inst_buffer; // instance-data buffer
	GLuint pose_buffer; // skinned only : the skeleton ubo at binding slot 1 (0 = none)
	GLuint vao;         // recorded set of glVertexAttribPointer calls

	mat4 identity_pose[MAX_ANIM_BONES]; // uploaded for skinned meshes with no player bound

	// runtime buffer info
	uint max_buffer_size;
	uint num_meshes;
	uint total_instances; // instances currently stored

	uint geom_size, indx_size, inst_size; // space in use, doubles as the write offset

	// creates the vao, the gpu buffers and this mode's vertex/instance layouts
	void init(uint buffer_size = MegaByte(1), uint draw_mode = MODE_STATIC_UV) {

		mode = draw_mode;
		const DrawMode_Info& info = DRAW_MODE_INFO[mode];

		glGenVertexArrays(1, &vao);
		max_buffer_size = buffer_size;

		// bind the vao *before* binding anything else!
		glBindVertexArray(vao);

		// vertex buffer : position and normal are always attributes 0 and 1. the uv
		// column is 2 when the mode has one ; a skinned mode then adds weights and bone
		// ids, which sit where the uv column was once it is gone
		glGenBuffers(1, &geom_buffer);
		glBindBuffer(GL_ARRAY_BUFFER, geom_buffer);
		glBufferData(GL_ARRAY_BUFFER, max_buffer_size, NULL, GL_STATIC_DRAW);

		uint stride = info.vertex_size;
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, stride, (void*)0); // position
		glEnableVertexAttribArray(0);
		glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, stride, (void*)sizeof(vec3)); // normal
		glEnableVertexAttribArray(1);

		if (info.has_uv)
		{
			glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, stride, (void*)(sizeof(vec3) * 2)); // uv
			glEnableVertexAttribArray(2);
		}
		else
			glDisableVertexAttribArray(2); // the shader reads the constant (0,0) instead

		uint first_instance_attrib = 3;
		if (info.skinned)
		{
			uint weights_at = sizeof(vec3) * 2 + (info.has_uv ? sizeof(vec2) : 0);
			glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, stride, (void*)weights_at); // weights
			glEnableVertexAttribArray(3);
			glVertexAttribIPointer(4, 3, GL_INT, stride, (void*)(weights_at + sizeof(vec3))); // bone ids are indices, not floats
			glEnableVertexAttribArray(4);
			first_instance_attrib = 5;
		}

		// instance buffer : the model mat4 (one vec4 per row) plus the material (7 and 8,
		// or 9 and 10 when skinned), all advancing once per instance
		glGenBuffers(1, &inst_buffer);
		glBindBuffer(GL_ARRAY_BUFFER, inst_buffer);
		glBufferData(GL_ARRAY_BUFFER, max_buffer_size, NULL, GL_DYNAMIC_DRAW);
		for (uint i = first_instance_attrib; i < first_instance_attrib + 4; i++)
		{
			glVertexAttribPointer(i, 4, GL_FLOAT, GL_FALSE, sizeof(Instance), (void*)(sizeof(vec4) * (i - first_instance_attrib)));
			glEnableVertexAttribArray(i);
			glVertexAttribDivisor(i, 1); // advance once per instance
		}
		glVertexAttribPointer(first_instance_attrib + 4, 4, GL_FLOAT, GL_FALSE, sizeof(Instance), (void*)sizeof(mat4)); // mat_a
		glEnableVertexAttribArray(first_instance_attrib + 4);
		glVertexAttribDivisor(first_instance_attrib + 4, 1);
		glVertexAttribPointer(first_instance_attrib + 5, 4, GL_FLOAT, GL_FALSE, sizeof(Instance), (void*)(sizeof(mat4) + sizeof(vec4))); // mat_b
		glEnableVertexAttribArray(first_instance_attrib + 5);
		glVertexAttribDivisor(first_instance_attrib + 5, 1);

		// index buffer : ordered uints referencing the vertices
		glGenBuffers(1, &indx_buffer);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indx_buffer);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, max_buffer_size, NULL, GL_STATIC_DRAW);

		// skeleton ubo : one mat4 per bone, std140, binding slot 1. skinned modes only
		pose_buffer = 0;
		if (info.skinned)
		{
			glGenBuffers(1, &pose_buffer);
			glBindBuffer(GL_UNIFORM_BUFFER, pose_buffer);
			glBufferData(GL_UNIFORM_BUFFER, MAX_ANIM_BONES * sizeof(mat4), NULL, GL_DYNAMIC_DRAW);
			glBindBufferBase(GL_UNIFORM_BUFFER, 1, pose_buffer);

			// identity pose : with no player bound, the mesh draws unskinned instead of collapsed
			for (uint i = 0; i < MAX_ANIM_BONES; i++) identity_pose[i] = mat4(1);
			glBufferSubData(GL_UNIFORM_BUFFER, 0, sizeof(identity_pose), identity_pose);
		}

		char msg[MAX_LOG_MSG_LENGTH] = {};
		snprintf(msg, MAX_LOG_MSG_LENGTH, "Init DrawBuffer [%s] | Size : [%d]", info.name, max_buffer_size);
		console->add_entry(msg, SUCCESS, RNDR);
	}

	// upload the skeleton pose : mat4 joint_transforms[num_bones] at binding slot 1
	void update_pose(mat4* pose, uint num_bones) {
		glBindBuffer(GL_UNIFORM_BUFFER, pose_buffer);
		glBufferSubData(GL_UNIFORM_BUFFER, 0, num_bones * sizeof(mat4), pose);
	}

	// the tail every append_geometry shares : upload one interleaved vertex block and its
	// indices, then record where they landed. the overloads below only differ in how they
	// interleave a row of the four mesh formats
	bool append_mesh(uint mesh_id, uint num_vertices, uint num_indices, const void* verts, const uint* indices, uint tx_id)
	{
		if (num_meshes >= MAX_MESHES) {
			char msg[MAX_LOG_MSG_LENGTH];
			snprintf(msg, MAX_LOG_MSG_LENGTH, "Too many meshes! [MAX_MESHES = %d]", MAX_MESHES);
			console->add_entry(msg, FIXME, RNDR);
			return false; // don't write past mesh_info
		}

		uint stride      = DRAW_MODE_INFO[mode].vertex_size;
		uint vert_bytes  = num_vertices * stride;
		uint index_bytes = num_indices  * sizeof(uint);

		if (geom_size + vert_bytes > max_buffer_size) {
			char msg[MAX_LOG_MSG_LENGTH];
			snprintf(msg, MAX_LOG_MSG_LENGTH, "Geometry buffer full [%s]! [%d + %d > %d bytes]",
				DRAW_MODE_INFO[mode].name, geom_size, vert_bytes, max_buffer_size);
			console->add_entry(msg, FIXME, RNDR);
			return false; // don't write past the gpu buffer
		}
		if (indx_size + index_bytes > max_buffer_size) {
			char msg[MAX_LOG_MSG_LENGTH];
			snprintf(msg, MAX_LOG_MSG_LENGTH, "Index buffer full [%s]! [%d + %d > %d bytes]",
				DRAW_MODE_INFO[mode].name, indx_size, index_bytes, max_buffer_size);
			console->add_entry(msg, FIXME, RNDR);
			return false;
		}

		glBindBuffer(GL_ARRAY_BUFFER, geom_buffer);
		glBufferSubData(GL_ARRAY_BUFFER, geom_size, vert_bytes, verts);

		// the element-array binding is vao state : bind ours first, or the upload
		// re-points whichever vao happens to be bound (silent cross-buffer theft)
		glBindVertexArray(vao);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indx_buffer);
		glBufferSubData(GL_ELEMENT_ARRAY_BUFFER, indx_size, index_bytes, indices);

		// record where this mesh landed, then move the write offsets
		uint mesh = num_meshes++;
		mesh_info[mesh].mesh_id       = mesh_id;
		mesh_info[mesh].tx_id         = tx_id;
		mesh_info[mesh].anim_player   = (uint)-1; // identity pose until bound
		mesh_info[mesh].num_vertices  = num_vertices;
		mesh_info[mesh].num_indices   = num_indices;
		mesh_info[mesh].num_instances = 0;
		mesh_info[mesh].base_vertex   = geom_size / stride;
		mesh_info[mesh].base_index    = indx_size;
		mesh_info[mesh].base_instance = 0;

		geom_size += vert_bytes;
		indx_size += index_bytes;

		return true;
	}

	// modes 1 and 3 : positions, normals, uvs
	bool append_geometry(uint mesh_id, Mesh_Data mesh_data, uint tx_id = 0) {
		MeshVertexData* verts = Alloc(MeshVertexData, mesh_data.num_vertices);
		for (uint i = 0; i < mesh_data.num_vertices; i++)
			verts[i] = MeshVertexData{ mesh_data.positions[i], mesh_data.normals[i], mesh_data.uvs[i] };

		bool ok = append_mesh(mesh_id, mesh_data.num_vertices, mesh_data.num_indices, verts, mesh_data.indices, tx_id);
		free(verts);
		return ok;
	}

	// mode 3 : 24 byte rows with no uv column, fed by its own file format so nothing has
	// to pretend a uv exists
	bool append_geometry(uint mesh_id, Mesh_Data_NoUV mesh_data, uint tx_id = 0)
	{
		MeshVertexDataNoUV* verts = Alloc(MeshVertexDataNoUV, mesh_data.num_vertices);
		for (uint i = 0; i < mesh_data.num_vertices; i++)
			verts[i] = MeshVertexDataNoUV{ mesh_data.positions[i], mesh_data.normals[i] };

		bool ok = append_mesh(mesh_id, mesh_data.num_vertices, mesh_data.num_indices, verts, mesh_data.indices, tx_id);
		free(verts);
		return ok;
	}

	// modes 2 and 4 : the skinned rows add weights and bone ids
	bool append_geometry(uint mesh_id, Mesh_Data_Anim mesh_data, uint tx_id = 0) {
		MeshVertexDataAnim* verts = Alloc(MeshVertexDataAnim, mesh_data.num_vertices);
		for (uint i = 0; i < mesh_data.num_vertices; i++)
			verts[i] = MeshVertexDataAnim{ mesh_data.positions[i], mesh_data.normals[i], mesh_data.uvs[i],
			                               mesh_data.weights[i], mesh_data.bones[i] };

		bool ok = append_mesh(mesh_id, mesh_data.num_vertices, mesh_data.num_indices, verts, mesh_data.indices, tx_id);
		free(verts);
		return ok;
	}

	// mode 4 : 48 byte rows, weights and bone ids where the uv column used to be
	bool append_geometry(uint mesh_id, Mesh_Data_Anim_NoUV mesh_data, uint tx_id = 0)
	{
		MeshVertexDataAnimNoUV* verts = Alloc(MeshVertexDataAnimNoUV, mesh_data.num_vertices);
		for (uint i = 0; i < mesh_data.num_vertices; i++)
			verts[i] = MeshVertexDataAnimNoUV{ mesh_data.positions[i], mesh_data.normals[i],
			                                   mesh_data.weights[i], mesh_data.bones[i] };

		bool ok = append_mesh(mesh_id, mesh_data.num_vertices, mesh_data.num_indices, verts, mesh_data.indices, tx_id);
		free(verts);
		return ok;
	}

	// append instance data for one mesh
	void append_instances(uint mesh_id, uint num_instances, Instance* instance_data)
	{
		glBindBuffer(GL_ARRAY_BUFFER, inst_buffer);

		uint instance_data_size = num_instances   * sizeof(Instance);
		uint instance_offset    = total_instances * sizeof(Instance);

		glBufferSubData(GL_ARRAY_BUFFER, instance_offset, instance_data_size, instance_data);

		for (uint i = 0; i < MAX_MESHES; i++)
		{
			if (mesh_info[i].mesh_id == mesh_id)
			{
				if (mesh_info[i].num_instances == 0)
					mesh_info[i].base_instance = total_instances;

				// remember where it landed : the skeleton overlay draws a rig in
				// world space and has no other way to know where the mesh went
				if (num_instances)
					mesh_info[i].last_model = instance_data[num_instances - 1].model;

				mesh_info[i].num_instances += num_instances;

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

		// appends build up over the frame : zero every count so they start clean
		for (uint i = 0; i < MAX_MESHES; i++)
			mesh_info[i].num_instances = 0;
	}

	// drop every mesh appended after a mark : a live edit re-adds its meshes at the
	// same slots without leaving the old ones in the buffer
	void rewind(uint mesh_count, uint geom_bytes, uint indx_bytes)
	{
		if (mesh_count > num_meshes) return;
		for (uint i = mesh_count; i < num_meshes; i++) mesh_info[i] = {};
		num_meshes = mesh_count;
		geom_size  = geom_bytes;
		indx_size  = indx_bytes;
	}

	// release the gpu objects : the buffer must be init()ed again before use
	void shutdown()
	{
		glDeleteBuffers(1, &geom_buffer);
		glDeleteBuffers(1, &indx_buffer);
		glDeleteBuffers(1, &inst_buffer);
		if (pose_buffer) glDeleteBuffers(1, &pose_buffer);
		glDeleteVertexArrays(1, &vao);
		*this = {};
	}

	// drop every mesh and refill with empty buffers of the same size
	void reset()
	{
		uint size = max_buffer_size, m = mode;
		shutdown();
		init(size, m);
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

// skeleton overlay : the animator's own little renderer, the twin of Phys_Debug.
// the selected mesh's bones draw as octahedra (a joint, a tip and a diamond ring
// between them - the shape every rig tool uses), each one linked to its parent, plus
// a cross per joint on the bone's own axes so rotations read in flight. own shader,
// own draw buffer : the scene buffers never hear about it

// the tier ramp : a bone is tinted by how deep in the chain it sits, so a rig reads
// as a hierarchy. the last tier is everything deeper than the ramp
const uint SKELETON_TIERS = 5;
const vec3 SKELETON_TIER_COLOR[SKELETON_TIERS] = {
	vec3(0.98f, 0.74f, 0.34f), // amber  : the root
	vec3(0.90f, 0.87f, 0.38f), // yellow
	vec3(0.55f, 0.88f, 0.44f), // green
	vec3(0.34f, 0.84f, 0.82f), // teal
	vec3(0.64f, 0.62f, 0.98f), // violet : the deep end of the chain
};

// the shared wire overlay : physics, animation, light volumes and scene guides
// all append into one buffer and one shader. line colour rides the instance tint,
// so a category can be read at a glance without another draw state
struct Debug_Draw
{
	DrawBuffer    lines;
	ShaderProgram shader;

	uint box_mesh, sphere_mesh, cylinder_mesh, cone_mesh;
	uint light_cone_mesh, segment_mesh;
	uint next_mesh; // free id for generated wires (capsules, etc)

	uint static_mesh_mark, static_geom_mark, static_indx_mark;

	// presentation toggles, edited in the Debug tab
	bool lights;
	bool light_directions;
	bool camera;
	bool axes;

	void init()
	{
		light_directions = true;

		shader.create("assets/shaders/wire.vert", "assets/shaders/wire.frag");
		lines.init();

		// unit box : 8 corners, 12 edges (an edge joins corners differing by one bit)
		Mesh_Data box = {};
		box.num_vertices = 8;
		box.num_indices  = 24;
		box.positions = Alloc(vec3, 8);
		box.normals   = Alloc(vec3, 8);
		box.uvs       = Alloc(vec2, 8);
		box.indices   = Alloc(uint, 24);

		for (uint i = 0; i < 8; i++)
		{
			box.positions[i] = vec3((i & 1) ? 1.f : -1.f, (i & 2) ? 1.f : -1.f, (i & 4) ? 1.f : -1.f);
			box.normals[i]   = vec3(0, 1, 0);
		}

		uint e = 0;
		for (uint i = 0; i < 8; i++)
			for (uint bit = 1; bit < 8; bit *= 2)
			{
				uint j = i ^ bit;
				if (i < j) { box.indices[e++] = i; box.indices[e++] = j; }
			}

		box_mesh = 1;
		lines.append_geometry(box_mesh, box);
		box.release();

		// unit sphere : ring + column edges of a coarse uv sphere
		uint slices = 16, stacks = 12;
		Mesh_Data src = MeshFactory::generate_uv_sphere(1.f, slices, stacks);
		Mesh_Data ball = {};
		ball.num_vertices = src.num_vertices;
		ball.num_indices  = stacks * slices * 8;
		ball.positions = Alloc(vec3, src.num_vertices);
		ball.normals   = Alloc(vec3, src.num_vertices);
		ball.uvs       = Alloc(vec2, src.num_vertices);
		ball.indices   = Alloc(uint, ball.num_indices);

		for (uint i = 0; i < src.num_vertices; i++)
		{
			ball.positions[i] = src.positions[i];
			ball.normals[i]   = src.normals[i];
			ball.uvs[i]       = src.uvs[i];
		}

		e = 0;
		for (uint i = 0; i < stacks; i++)
			for (uint j = 0; j < slices; j++)
			{
				uint upper = i * (slices + 1) + j;
				uint lower = upper + slices + 1;

				ball.indices[e++] = upper;     ball.indices[e++] = upper + 1;
				ball.indices[e++] = lower;     ball.indices[e++] = lower + 1;
				ball.indices[e++] = upper;     ball.indices[e++] = lower;
				ball.indices[e++] = upper + 1; ball.indices[e++] = lower + 1;
			}

		src.release();
		sphere_mesh = 2;
		lines.append_geometry(sphere_mesh, ball);
		ball.release();

		// unit cylinder : radius 1, half-height 1, rings only in the xz plane
		uint cyl_slices = 16;
		Mesh_Data cyl = {};
		cyl.num_vertices = 2 * (cyl_slices + 1);
		cyl.num_indices  = cyl_slices * 6;
		cyl.positions = Alloc(vec3, cyl.num_vertices);
		cyl.normals   = Alloc(vec3, cyl.num_vertices);
		cyl.uvs       = Alloc(vec2, cyl.num_vertices);
		cyl.indices   = Alloc(uint, cyl.num_indices);

		for (uint ring = 0; ring < 2; ring++)
			for (uint j = 0; j <= cyl_slices; j++)
			{
				float theta = 2.f * PI * j / cyl_slices;
				cyl.positions[ring * (cyl_slices + 1) + j] =
					vec3(cosf(theta), ring ? -1.f : 1.f, sinf(theta));
			}

		e = 0;
		for (uint j = 0; j < cyl_slices; j++)
		{
			uint top = j, bottom = cyl_slices + 1 + j;
			cyl.indices[e++] = top;    cyl.indices[e++] = top + 1;
			cyl.indices[e++] = bottom; cyl.indices[e++] = bottom + 1;
			cyl.indices[e++] = top;    cyl.indices[e++] = bottom;
		}

		cylinder_mesh = 3;
		lines.append_geometry(cylinder_mesh, cyl);
		cyl.release();

		// unit cone : base ring at y -0.5, apex at y +0.5
		uint cone_slices = 16;
		Mesh_Data cone = {};
		cone.num_vertices = cone_slices + 1;
		cone.num_indices  = cone_slices * 4;
		cone.positions = Alloc(vec3, cone.num_vertices);
		cone.normals   = Alloc(vec3, cone.num_vertices);
		cone.uvs       = Alloc(vec2, cone.num_vertices);
		cone.indices   = Alloc(uint, cone.num_indices);

		for (uint j = 0; j < cone_slices; j++)
		{
			float theta = 2.f * PI * j / cone_slices;
			cone.positions[j] = vec3(cosf(theta), -.5f, sinf(theta));
		}
		cone.positions[cone_slices] = vec3(0, .5f, 0);

		e = 0;
		for (uint j = 0; j < cone_slices; j++)
		{
			uint next = (j + 1) % cone_slices;
			cone.indices[e++] = j; cone.indices[e++] = next;
			cone.indices[e++] = j; cone.indices[e++] = cone_slices;
		}

		cone_mesh = 4;
		lines.append_geometry(cone_mesh, cone);
		cone.release();

		// light cone : apex at the origin, base at y = 1, radius 1
		Mesh_Data light_cone = {};
		light_cone.num_vertices = cone_slices + 1;
		light_cone.num_indices  = cone_slices * 6; // two side edges + the base ring
		light_cone.positions = Alloc(vec3, light_cone.num_vertices);
		light_cone.normals   = Alloc(vec3, light_cone.num_vertices);
		light_cone.uvs       = Alloc(vec2, light_cone.num_vertices);
		light_cone.indices   = Alloc(uint, light_cone.num_indices);

		light_cone.positions[0] = vec3(0.f);
		for (uint j = 0; j < cone_slices; j++)
		{
			float theta = 2.f * PI * j / cone_slices;
			light_cone.positions[1 + j] = vec3(cosf(theta), 1.f, sinf(theta));
		}

		e = 0;
		for (uint j = 0; j < cone_slices; j++)
		{
			uint next = 1 + (j + 1) % cone_slices;
			light_cone.indices[e++] = 0; light_cone.indices[e++] = 1 + j;
			light_cone.indices[e++] = 0; light_cone.indices[e++] = next;
			light_cone.indices[e++] = 1 + j; light_cone.indices[e++] = next;
		}

		light_cone_mesh = 5;
		lines.append_geometry(light_cone_mesh, light_cone);
		light_cone.release();

		// one +x segment, stretched between any two points
		Mesh_Data seg = {};
		seg.num_vertices = 2;
		seg.num_indices  = 2;
		seg.positions = Alloc(vec3, 2);
		seg.normals   = Alloc(vec3, 2);
		seg.uvs       = Alloc(vec2, 2);
		seg.indices   = Alloc(uint, 2);

		seg.positions[0] = vec3(0.f, 0.f, 0.f);
		seg.positions[1] = vec3(1.f, 0.f, 0.f);
		seg.normals[0] = seg.normals[1] = vec3(0.f, 1.f, 0.f);
		seg.uvs[0] = seg.uvs[1] = vec2(0.f);
		seg.indices[0] = 0;
		seg.indices[1] = 1;

		segment_mesh = 6;
		lines.append_geometry(segment_mesh, seg);
		seg.release();

		next_mesh = 20;
	}

	// call once the built-in unit meshes and the anim rig meshes exist : everything
	// appended after this mark is per-scene and can be dropped on a scene switch
	void mark_static()
	{
		static_mesh_mark = lines.num_meshes;
		static_geom_mark = lines.geom_size;
		static_indx_mark = lines.indx_size;
		next_mesh = 20;
	}

	void reset_dynamic()
	{
		if (lines.num_meshes >= static_mesh_mark)
			lines.rewind(static_mesh_mark, static_geom_mark, static_indx_mark);
		next_mesh = 20;
	}

	void begin() { lines.clear_instances(); }

	void add_instance(uint mesh, const mat4& model, vec4 color)
	{
		Instance inst = INSTANCE_DEFAULT;
		inst.model = model;
		inst.mat_a = color;
		lines.append_instances(mesh, 1, &inst);
	}

	void append_shape(uint mesh, const mat4& model, vec4 color)
	{
		add_instance(mesh, model, color);
	}

	void append_sphere(vec3 center, float radius, vec4 color)
	{
		add_instance(sphere_mesh, glm::translate(mat4(1), center) * glm::scale(mat4(1), vec3(radius)), color);
	}

	void append_box(vec3 center, vec3 half, vec4 color)
	{
		add_instance(box_mesh, glm::translate(mat4(1), center) * glm::scale(mat4(1), half), color);
	}

	void append_cylinder(vec3 center, vec3 half, vec4 color)
	{
		add_instance(cylinder_mesh, glm::translate(mat4(1), center) * glm::scale(mat4(1), half), color);
	}

	void append_cone(vec3 center, vec3 half, mat3 basis, vec4 color)
	{
		add_instance(cone_mesh, glm::translate(mat4(1), center) * mat4(basis) * glm::scale(mat4(1), half), color);
	}

	// a spot volume : apex at the light, base ring at the reach
	void append_light_cone(vec3 apex, vec3 dir, float radius, float outer_angle, vec4 color)
	{
		vec3 helper = fabsf(dir.y) < .95f ? vec3(0, 1, 0) : vec3(1, 0, 0);
		vec3 right = normalize(cross(helper, dir));
		vec3 up = cross(dir, right);
		mat3 basis(right, dir, up);

		float tan_outer = tanf(outer_angle);
		mat4 model = glm::translate(mat4(1), apex) * mat4(basis)
		           * glm::scale(mat4(1), vec3(tan_outer * radius, radius, tan_outer * radius));
		add_instance(light_cone_mesh, model, color);
	}

	void append_segment(vec3 a, vec3 b, vec4 color)
	{
		vec3 d = b - a;
		float len = length(d);
		if (len < 1e-5f) return;

		vec3 x = d / len;
		vec3 helper = fabsf(x.y) < .95f ? vec3(0, 1, 0) : vec3(1, 0, 0);
		vec3 z = normalize(cross(x, helper));
		vec3 y = cross(z, x);

		mat3 rot;
		rot[0] = x;
		rot[1] = y;
		rot[2] = z;

		add_instance(segment_mesh, glm::translate(mat4(1), a) * mat4(rot) *
		             glm::scale(mat4(1), vec3(len, 1.f, 1.f)), color);
	}

	void append_cross(vec3 p, float size, vec4 color)
	{
		append_segment(p - vec3(size, 0, 0), p + vec3(size, 0, 0), color);
		append_segment(p - vec3(0, size, 0), p + vec3(0, size, 0), color);
		append_segment(p - vec3(0, 0, size), p + vec3(0, 0, size), color);
	}

	void append_axes(float size)
	{
		if (!axes) return;
		append_segment(vec3(0), vec3(size, 0, 0), vec4(1.f, .25f, .25f, 1.f));
		append_segment(vec3(0), vec3(0, size, 0), vec4(.25f, 1.f, .25f, 1.f));
		append_segment(vec3(0), vec3(0, 0, size), vec4(.25f, .45f, 1.f, 1.f));
	}

	void append_frustum(mat4 proj_view, vec4 color)
	{
		if (!camera) return;
		mat4 inv = glm::inverse(proj_view);
		vec3 c[8];
		uint k = 0;
		for (uint z = 0; z < 2; z++)
			for (uint y = 0; y < 2; y++)
				for (uint x = 0; x < 2; x++)
				{
					vec4 p = inv * vec4(x ? 1.f : -1.f, y ? 1.f : -1.f, z ? .999f : .02f, 1.f);
					c[k++] = vec3(p) / p.w;
				}

		// near / far rectangles, then the four side edges
		uint edges[12][2] = {
			{0,1},{1,3},{3,2},{2,0}, {4,5},{5,7},{7,6},{6,4},
			{0,4},{1,5},{2,6},{3,7},
		};
		for (uint i = 0; i < 12; i++) append_segment(c[edges[i][0]], c[edges[i][1]], color);
	}

	void draw(mat4 proj_view)
	{
		glBindFramebuffer(GL_FRAMEBUFFER, 0);
		glClear(GL_DEPTH_BUFFER_BIT);
		glDisable(GL_DEPTH_TEST);
		glDepthMask(GL_FALSE);

		glBindVertexArray(lines.vao);
		shader.bind();
		glUniformMatrix4fv(glGetUniformLocation(shader.id, "proj_view"), 1, GL_FALSE, (float*)&proj_view);
		glUniform4f(glGetUniformLocation(shader.id, "color"), 1.f, 1.f, 1.f, 1.f);

		for (uint i = 0; i < MAX_MESHES; i++)
		{
			auto& mi = lines.mesh_info[i];
			if (mi.mesh_id == 0 || mi.num_instances == 0) continue;
			glDrawElementsInstancedBaseVertexBaseInstance(GL_LINES, mi.num_indices, GL_UNSIGNED_INT,
				(void*)mi.base_index, mi.num_instances, mi.base_vertex, mi.base_instance);
		}

		glDepthMask(GL_TRUE);
		glEnable(GL_DEPTH_TEST);
	}
};

// the one overlay every system appends to
static Debug_Draw debug_draw;

struct Anim_Debug
{
	bool enabled; // the Anim tab's master toggle
	bool bones;   // the octahedra themselves
	bool links;   // a line from every bone to its parent
	bool axes;    // a cross per joint, on the bone's own frame
	bool no_uv;   // which skinned buffer the slot belongs to (mode 2 or mode 4)
	uint slot;    // the mesh_info slot to draw : what the dropdown picked

	uint bone_mesh, link_mesh, joint_mesh;

	void init()
	{
		// calloc leaves these at zero : the overlay starts off, and when it goes on the
		// bones and the links between them are what you want first
		bones = true;
		links = true;

		// a bone : joint at the origin, tip at +1, a diamond ring at 0.15. sized per
		// bone when it is instanced, so one unit mesh covers every length
		vec3 ring[4] = { vec3( 0.15f, 0.15f, 0.f), vec3(0.f, 0.15f,  0.15f),
		                 vec3(-0.15f, 0.15f, 0.f), vec3(0.f, 0.15f, -0.15f) };

		Mesh_Data bone = {};
		bone.num_vertices = 6;
		bone.num_indices  = 24; // joint -> ring, ring -> tip, and the ring itself
		bone.positions = Alloc(vec3, 6);
		bone.normals   = Alloc(vec3, 6);
		bone.uvs       = Alloc(vec2, 6);
		bone.indices   = Alloc(uint, 24);

		bone.positions[0] = vec3(0.f);              // joint
		bone.positions[1] = vec3(0.f, 1.f, 0.f);    // tip
		for (uint i = 0; i < 4; i++) bone.positions[2 + i] = ring[i];
		for (uint i = 0; i < 6; i++) bone.normals[i] = vec3(0.f); // the wire shader ignores them

		uint e = 0;
		for (uint i = 0; i < 4; i++)
		{
			bone.indices[e++] = 0;     bone.indices[e++] = 2 + i;            // joint -> ring
			bone.indices[e++] = 2 + i; bone.indices[e++] = 1;                // ring  -> tip
			bone.indices[e++] = 2 + i; bone.indices[e++] = 2 + ((i + 1) % 4); // the ring
		}

		bone_mesh = 7;
		debug_draw.lines.append_geometry(bone_mesh, bone);
		bone.release();

		// a joint : the bone's own x / y / z, unit length
		Mesh_Data joint = {};
		joint.num_vertices = 4;
		joint.num_indices  = 6;
		joint.positions = Alloc(vec3, 4);
		joint.normals   = Alloc(vec3, 4);
		joint.uvs       = Alloc(vec2, 4);
		joint.indices   = Alloc(uint, 6);

		joint.positions[0] = vec3(0.f);
		joint.positions[1] = vec3(1.f, 0.f, 0.f);
		joint.positions[2] = vec3(0.f, 1.f, 0.f);
		joint.positions[3] = vec3(0.f, 0.f, 1.f);
		for (uint i = 0; i < 4; i++) joint.normals[i] = vec3(0.f);
		joint.indices[0] = 0; joint.indices[1] = 1;
		joint.indices[2] = 0; joint.indices[3] = 2;
		joint.indices[4] = 0; joint.indices[5] = 3;

		joint_mesh = 8;
		debug_draw.lines.append_geometry(joint_mesh, joint);
		joint.release();

		// a link : one line along +y, unit length. scaled to the joint gap, so a bone
		// can be drawn tied to the joint it hangs off - that is the hierarchy made
		// visible, and the only thing that shows a branch
		Mesh_Data link = {};
		link.num_vertices = 2;
		link.num_indices  = 2;
		link.positions = Alloc(vec3, 2);
		link.normals   = Alloc(vec3, 2);
		link.uvs       = Alloc(vec2, 2);
		link.indices   = Alloc(uint, 2);

		link.positions[0] = vec3(0.f);
		link.positions[1] = vec3(0.f, 1.f, 0.f);
		link.indices[0] = 0; link.indices[1] = 1;

		link_mesh = 9;
		debug_draw.lines.append_geometry(link_mesh, link);
		link.release();
	}

	// a unit bone laid along dir with the given length, in the mesh's own space, then
	// placed by the instance matrix. +y of the unit mesh is the bone's direction
	mat4 bone_at(mat4 place, vec3 head, vec3 dir, float len)
	{
		vec3 up    = (glm::abs(dir.y) < 0.99f) ? vec3(0.f, 1.f, 0.f) : vec3(1.f, 0.f, 0.f);
		vec3 right = normalize(cross(up, dir));
		mat3 basis(right, dir, cross(right, dir)); // columns : local x -> right, y -> dir

		return place * glm::translate(mat4(1.f), head) * mat4(basis)
		             * glm::scale(mat4(1.f), vec3(len));
	}

	// the selected mesh's rig, as the player posing it has it right now
	void append(DrawBuffer* meshes, AnimLoader* anim)
	{
		if (!enabled || slot >= MAX_MESHES)
			return;

		auto& mi = meshes->mesh_info[slot];
		uint player = mi.anim_player;
		if (mi.mesh_id == 0 || mi.num_instances == 0 || player >= anim->num_players)
			return;

		Anim_Player* pl = &anim->players[player];
		Animation* clip = &anim->clips[pl->clip_id];
		uint bones = clip->num_bones;
		if (bones == 0 || bones > MAX_ANIM_BONES)
			return;

		// what the loader hands around is the skinning matrix (world * inverse bind), not
		// the bone's own transform, so recover the joint from it : world = pose * inverse(ibm)
		mat4 bone_world[MAX_ANIM_BONES];
		vec3 head[MAX_ANIM_BONES], tail[MAX_ANIM_BONES];
		for (uint b = 0; b < bones; b++)
		{
			bone_world[b] = pl->pose[b] * glm::inverse(clip->ibm[b]);
			head[b]       = vec3(bone_world[b][3]);
			tail[b]       = head[b];
		}

		// a bone's length comes off the bind pose, never from where its child happens to
		// be at this instant : a bone that translates - the ak magazine sliding out of
		// the receiver - would otherwise stretch its parent and shrink it back on the
		// way in, which is exactly what a rig does not do
		for (uint b = 0; b < bones; b++)
		{
			vec3 bind_b = vec3(glm::inverse(clip->ibm[b])[3]);

			// these files point a root at itself (parents[0] == 0), so a bone is only a
			// child when some *other* bone names it, and a bone is a root when it names
			// itself (or has no parent at all)
			uint child = bones; // none found
			for (uint c = 0; c < bones; c++)
				if (c != b && clip->parents[c] == (int)b) { child = c; break; }

			int  parent = clip->parents[b];
			bool rooted = (parent < 0 || parent == (int)b);

			vec3 segment = vec3(0.f);
			if (child < bones)   segment = vec3(glm::inverse(clip->ibm[child])[3])  - bind_b;
			else if (!rooted)    segment = bind_b - vec3(glm::inverse(clip->ibm[parent])[3]); // leaf : back toward the parent

			// a bone with a child runs the whole way to it ; a leaf draws half of the way
			// back to its parent, so the parent link stays readable underneath it ; a root
			// with no children at all has no segment to draw
			float len = length(segment) * ((child < bones) ? 1.f : 0.5f);
			if (len < 1e-4f) continue;

			// the same segment in the bone's own frame, then turned by the bone as it is
			// posed right now. the length is the bind length, so the bone never scales
			vec3  local = mat3(clip->ibm[b]) * segment;
			vec3  dir   = mat3(bone_world[b]) * local;
			float dl    = length(dir);
			tail[b] = head[b] + ((dl > 1e-6f) ? (dir / dl) : vec3(0.f, 1.f, 0.f)) * len;
		}

		// tint : these rigs are flat, one root with every part hanging straight off it,
		// so a chain depth would paint them all the same colour. the bone's own index is
		// what tells the parts apart, and the parent links carry the hierarchy
		uint tier[MAX_ANIM_BONES];
		for (uint b = 0; b < bones; b++)
			tier[b] = (b < SKELETON_TIERS) ? b : SKELETON_TIERS - 1;

		// three runs that must not interleave : a draw reads its instances from one
		// block. the instance albedo carries each bone's tier colour and wire.frag
		// multiplies it in, so a single mesh draws every tier
		mat4 place = mi.last_model;

		if (bones)
			for (uint b = 0; b < bones; b++)
			{
				vec3  axis = tail[b] - head[b];
				float len  = length(axis);
				if (len < 1e-4f) continue;

				Instance inst = INSTANCE_DEFAULT;
				inst.mat_a = vec4(SKELETON_TIER_COLOR[tier[b]], 0.f);
				inst.model = bone_at(place, head[b], axis / len, len);
				debug_draw.lines.append_instances(bone_mesh, 1, &inst);
			}

		if (links)
			for (uint b = 0; b < bones; b++)
			{
				int parent = clip->parents[b];
				if (parent < 0 || parent >= (int)bones || parent == (int)b) continue; // a root hangs off nothing

				vec3  axis = head[parent] - head[b]; // this joint to the joint it hangs off
				float len  = length(axis);
				if (len < 1e-4f) continue;

				Instance inst = INSTANCE_DEFAULT;
				inst.mat_a = vec4(SKELETON_TIER_COLOR[tier[b]] * 0.62f, 0.f); // dimmer than the bones
				inst.model = bone_at(place, head[b], axis / len, len);
				debug_draw.lines.append_instances(link_mesh, 1, &inst);
			}

		if (axes)
			for (uint b = 0; b < bones; b++)
			{
				vec3  axis = tail[b] - head[b];
				float len  = length(axis);
				if (len < 1e-4f) continue;

				Instance inst = INSTANCE_DEFAULT;
				inst.mat_a = vec4(0.58f, 0.62f, 0.68f, 0.f);
				inst.model = bone_at(place, head[b], axis / len, len * 0.20f);
				debug_draw.lines.append_instances(joint_mesh, 1, &inst);
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
	uint           foam_pattern_tx; // CC0 lace mask, shared by every water surface
	ShaderProgram  shaders[MAX_SHADERS]; // 0 = static meshes, 1 = skinned, 2 = forward water

	// drawing : one buffer and one draw list per mode. the table is the only list of
	// them in the engine, so a fifth layout is a row in DRAW_MODE_INFO and nothing else
	DrawBuffer drawbuffers[DRAW_MODE_COUNT];
	DrawList   drawlists[DRAW_MODE_COUNT];

	// one directional shadow map : depth-only render of the static world, sampled
	// by the deferred sun light
	struct {
		GLuint fbo, depth_tx;
		ShaderProgram shader;
		mat4 proj_view;
		uint size;
		bool enabled;
	} shadow;

	// light volumes : one unit sphere drawn once per light through an instance
	// buffer. the sphere is the light's reach, so the fragment shader only runs
	// on the pixels the light can actually touch
	struct LightVolumeMesh {
		GLuint vao, geom_buffer, idx_buffer;
		uint num_indices;
	};

	struct {
		ShaderProgram shader;
		GLuint inst_buffer;
		LightVolumeMesh sphere, cone;
	} lightvol;

	// one light instance : centre/radius, colour/power, axis/shape and cone angles
	struct LightInstance {
		vec4 pos_radius;
		vec4 color_power;
		vec4 direction_shape;
		vec4 params; // outer radians, outer cos, inner cos
	};

	// which buffer an asset belongs to, from the only two things that decide it. the
	// mapping lives here so no caller has to open-code it
	static uint mode_of(bool skinned, bool has_uv)
	{
		return skinned ? (has_uv ? MODE_ANIM_UV : MODE_ANIM_NOUV)
		               : (has_uv ? MODE_STATIC_UV : MODE_STATIC_NOUV);
	}
	DrawBuffer& buffer(uint mode) { return drawbuffers[mode]; }
	DrawBuffer& buffer_for(bool skinned, bool has_uv) { return drawbuffers[mode_of(skinned, has_uv)]; }

	Anim_Debug animdebug; // skeleton overlay, toggled in the Anim tab

	void init()
	{
		shaders[0].create("assets/shaders/geom.vert", "assets/shaders/geom.frag");
		shaders[1].create("assets/shaders/anim.vert", "assets/shaders/geom.frag"); // same frag as the static pass
		shaders[2].create("assets/shaders/water.vert", "assets/shaders/water.frag");

		for (uint mode = 0; mode < DRAW_MODE_COUNT; mode++)
			drawbuffers[mode].init(MegaByte(8), mode);

		init_light_volumes();
		init_shadow();

		txloader.init();
		foam_pattern_tx = txloader.load("assets/textures/foam_pattern.jpg");
		animloader.init(); // playback defaults (calloc skips them)
		debug_draw.init();
		animdebug.init();
		debug_draw.mark_static();

		console->add_entry((char*)"Init GameRenderer", SUCCESS, RNDR);
	}

	// one unit sphere and one unit cone, both drawn through the same instance
	// buffer. only the sphere or the cone group is submitted for each light
	void init_light_volumes()
	{
		lightvol.shader.create("assets/shaders/light.vert", "assets/shaders/light.frag");

		glGenBuffers(1, &lightvol.inst_buffer);
		glBindBuffer(GL_ARRAY_BUFFER, lightvol.inst_buffer);
		glBufferData(GL_ARRAY_BUFFER, MAX_LIGHTS * sizeof(LightInstance), NULL, GL_DYNAMIC_DRAW);

		Mesh_Data_NoUV sphere = MeshFactory::generate_sphere(1.f, 12, 20);
		init_light_volume_mesh(&lightvol.sphere, sphere);
		sphere.release();

		// centre cone : base at -Y, apex at +Y. the vertex shader maps the apex
		// to the light centre and the base ring to the reach at the outer angle
		Mesh_Data_NoUV cone = MeshFactory::without_uv(MeshFactory::generate_cone(1.f, 1.f, 24));
		init_light_volume_mesh(&lightvol.cone, cone);
		cone.release();
	}

	void init_light_volume_mesh(LightVolumeMesh* mesh, Mesh_Data_NoUV& data)
	{
		mesh->num_indices = data.num_indices;

		glGenVertexArrays(1, &mesh->vao);
		glBindVertexArray(mesh->vao);

		glGenBuffers(1, &mesh->geom_buffer);
		glBindBuffer(GL_ARRAY_BUFFER, mesh->geom_buffer);
		glBufferData(GL_ARRAY_BUFFER, data.num_vertices * sizeof(vec3), data.positions, GL_STATIC_DRAW);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(vec3), (void*)0);
		glEnableVertexAttribArray(0);

		glGenBuffers(1, &mesh->idx_buffer);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mesh->idx_buffer);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, data.num_indices * sizeof(uint), data.indices, GL_STATIC_DRAW);

		glBindBuffer(GL_ARRAY_BUFFER, lightvol.inst_buffer);
		uint stride = sizeof(LightInstance);
		glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, stride, (void*)0);
		glEnableVertexAttribArray(1);
		glVertexAttribDivisor(1, 1);
		glVertexAttribPointer(2, 4, GL_FLOAT, GL_FALSE, stride, (void*)sizeof(vec4));
		glEnableVertexAttribArray(2);
		glVertexAttribDivisor(2, 1);
		glVertexAttribPointer(3, 4, GL_FLOAT, GL_FALSE, stride, (void*)(sizeof(vec4) * 2));
		glEnableVertexAttribArray(3);
		glVertexAttribDivisor(3, 1);
		glVertexAttribPointer(4, 4, GL_FLOAT, GL_FALSE, stride, (void*)(sizeof(vec4) * 3));
		glEnableVertexAttribArray(4);
		glVertexAttribDivisor(4, 1);
	}

	// the animator's own overlay pass : drawn straight to the backbuffer, over the
	// 2048 depth-only target for the sun. the pass is rebuilt every frame around
	// the camera, so it follows the scene instead of matching one fixed map
	void init_shadow()
	{
		shadow.size = 2048;
		shadow.enabled = true;
		shadow.shader.create("assets/shaders/shadow.vert", "assets/shaders/shadow.frag");

		glGenTextures(1, &shadow.depth_tx);
		glBindTexture(GL_TEXTURE_2D, shadow.depth_tx);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT24, shadow.size, shadow.size,
		             0, GL_DEPTH_COMPONENT, GL_FLOAT, NULL);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_COMPARE_REF_TO_TEXTURE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_FUNC, GL_LEQUAL);
		float border[4] = { 1, 1, 1, 1 };
		glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, border);

		glGenFramebuffers(1, &shadow.fbo);
		glBindFramebuffer(GL_FRAMEBUFFER, shadow.fbo);
		glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, shadow.depth_tx, 0);
		glDrawBuffer(GL_NONE);
		glReadBuffer(GL_NONE);

		if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
			console->add_entry("shadow framebuffer incomplete", FIXME, RNDR);

		glBindFramebuffer(GL_FRAMEBUFFER, 0);
	}

	// depth-only pass : static meshes from the sun's point of view. skinned meshes
	// are skipped (they would need a second pose-aware depth shader)
	void draw_shadows(GameWindow* window, const Lighting& l, vec3 camera_pos, vec3 camera_front,
	                  float extent)
	{
		if (!shadow.enabled)
		{
			window->shadow_on = false;
			return;
		}

		// at night the moon takes over the same depth pass
		bool use_moon = l.sun_power < .05f && l.moon_power > .02f
		             && l.moon_direction.y < -.02f;
		vec3 dir = use_moon ? l.moon_direction : l.sun_direction;
		if (dot(dir, dir) < 1e-6f) dir = vec3(-.36f, -.84f, -.30f);
		dir = normalize(dir);

		// the active light is below the horizon : nothing useful to render
		if (!use_moon && (l.sun_power < .05f || dir.y > -.02f))
		{
			window->shadow_on = false;
			return;
		}
		if (use_moon && dir.y > -.02f)
		{
			window->shadow_on = false;
			return;
		}

		vec3 center = camera_pos + camera_front * (extent * .20f);
		vec3 up = fabsf(dir.y) > .95f ? vec3(0, 0, 1) : vec3(0, 1, 0);
		mat4 view = glm::lookAt(center - dir * extent, center, up);
		mat4 proj = glm::ortho(-extent * .5f, extent * .5f, -extent * .5f, extent * .5f,
		                       .1f, extent * 2.f);
		shadow.proj_view = proj * view;

		glBindFramebuffer(GL_FRAMEBUFFER, shadow.fbo);
		glViewport(0, 0, shadow.size, shadow.size);
		glClear(GL_DEPTH_BUFFER_BIT);

		glEnable(GL_DEPTH_TEST);
		glDepthMask(GL_TRUE);
		glDisable(GL_BLEND);
		glDisable(GL_CULL_FACE); // the floor and staged meshes are single-sided

		shadow.shader.bind();
		shadow.shader.set_mat4("light_proj_view", shadow.proj_view);

		for (uint mode = 0; mode < DRAW_MODE_COUNT; mode++)
		{
			DrawBuffer& db = drawbuffers[mode];
			if (db.num_meshes == 0) continue;
			if (!DRAW_MODE_INFO[mode].deferred) continue;
			if (DRAW_MODE_INFO[mode].skinned) continue; // no pose-aware depth shader yet


			glBindVertexArray(db.vao);
			drawlists[mode].update(&db);

			for (uint i = 0; i < MAX_MESHES; i++)
			{
				auto& mp = drawlists[mode].mesh_params[i];
				if (mp.num_instances == 0) continue;

				glDrawElementsInstancedBaseVertexBaseInstance(GL_TRIANGLES, mp.num_indices,
					GL_UNSIGNED_INT, (void*)mp.base_index, mp.num_instances,
					mp.base_vertex, mp.base_instance);
			}
		}

		glEnable(GL_CULL_FACE);
		glCullFace(GL_BACK);
		glBindFramebuffer(GL_FRAMEBUFFER, 0);

		window->shadow_tx = shadow.depth_tx;
		window->shadow_proj_view = shadow.proj_view;
		window->shadow_direction = dir;
		window->shadow_texel = 1.f / (float)shadow.size;
		window->shadow_world_texel = extent / (float)shadow.size;
		window->shadow_is_moon = use_moon;
		window->shadow_on = true;
	}

	// append the selected rig to the shared debug overlay
	void append_skeleton()
	{
		// a no-uv rig lives in the other skinned buffer, and the slot number alone cannot
		// say which : the tab sets no_uv alongside it
		DrawBuffer* meshes = &buffer(animdebug.no_uv ? MODE_ANIM_NOUV : MODE_ANIM_UV);
		animdebug.append(meshes, &animloader);
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

		bool appended = buffer(MODE_STATIC_UV).append_geometry(mesh_id, mesh_data, tx_id);
		mesh_data.release(); // consumed : freed whether it made it in or not
		if (!appended) return 0; // buffer full, error already logged

		char msg[MAX_LOG_MSG_LENGTH] = {};
		snprintf(msg, MAX_LOG_MSG_LENGTH, "Add Mesh, id[%d], generated, texture[%s]",
			mesh_id, texture ? texture : "default");
		console->add_entry(msg, SUCCESS, RNDR);

		return mesh_id;
	}

	// forward water : same vertex layout as the static buffer, different pass
	uint add_mesh_water(Mesh_Data mesh_data, const char* texture = NULL)
	{
		uint mesh_id = ++meshloader.num_loaded;
		uint tx_id   = texture ? txloader.load(texture) : 0;

		bool appended = buffer(MODE_WATER).append_geometry(mesh_id, mesh_data, tx_id);
		mesh_data.release();
		if (!appended) return 0;

		char msg[MAX_LOG_MSG_LENGTH] = {};
		snprintf(msg, MAX_LOG_MSG_LENGTH, "Add Mesh (water), id[%d], texture[%s]",
			mesh_id, texture ? texture : "default");
		console->add_entry(msg, SUCCESS, RNDR);
		return mesh_id;
	}

	// mode 3 for geometry built in code : the same trip as add_mesh(Mesh_Data), into the
	// no-uv buffer. ids come from the same counter, so a static mesh id is unique across
	// both layouts
	uint add_mesh_no_uv(Mesh_Data_NoUV mesh_data)
	{
		uint mesh_id = ++meshloader.num_loaded;
		bool appended = buffer(MODE_STATIC_NOUV).append_geometry(mesh_id, mesh_data);
		mesh_data.release();
		if (!appended) return 0;

		char msg[MAX_LOG_MSG_LENGTH] = {};
		snprintf(msg, MAX_LOG_MSG_LENGTH, "Add Mesh (no uv), id[%d], generated", mesh_id);
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

		bool appended = buffer(MODE_ANIM_UV).append_geometry(mesh_id, mesh_data, tx_id);
		mesh_data.release(); // no longer needed since it's now in the gpu buffer

		if (!appended) return 0; // buffer full, error already logged

		char msg[MAX_LOG_MSG_LENGTH] = {};
		snprintf(msg, MAX_LOG_MSG_LENGTH, "Add Anim Mesh, id[%d], %s", mesh_id, filepath);
		console->add_entry(msg, SUCCESS, RNDR);

		return mesh_id;
	}

	// which animation player poses a skinned mesh (unbound meshes draw the identity pose)
	void bind_player(uint mesh_id, uint player_id, bool no_uv = false)
	{
		DrawBuffer& db = buffer(no_uv ? MODE_ANIM_NOUV : MODE_ANIM_UV);

		for (uint i = 0; i < MAX_MESHES; i++)
		{
			if (db.mesh_info[i].mesh_id == mesh_id)
			{
				db.mesh_info[i].anim_player = player_id;
				return;
			}
		}
		console->add_entry("bind_player : no such mesh id", FIXME, RNDR);
	}

	// ---- modes 3 and 4 : the same two trips, for assets with no uvs --------------

	uint add_mesh_no_uv(const char* filepath)
	{
		uint mesh_id = 0;
		Mesh_Data_NoUV mesh_data = {};

		meshloader.load_mesh(filepath, &mesh_id);
		if (mesh_id == 0) return 0; // load failed, error already logged

		meshloader.load_mesh_data_no_uv(mesh_id, &mesh_data);
		if (mesh_data.num_vertices == 0) return 0; // no data, error already logged

		bool appended = buffer(MODE_STATIC_NOUV).append_geometry(mesh_id, mesh_data);
		mesh_data.release();
		if (!appended) return 0; // buffer full, error already logged

		char msg[MAX_LOG_MSG_LENGTH] = {};
		snprintf(msg, MAX_LOG_MSG_LENGTH, "Add Mesh (no uv), id[%d], %s", mesh_id, filepath);
		console->add_entry(msg, SUCCESS, RNDR);
		return mesh_id;
	}

	uint add_anim_mesh_no_uv(const char* filepath)
	{
		uint mesh_id = 0;
		Mesh_Data_Anim_NoUV mesh_data = {};

		meshloader_anim.load_mesh(filepath, &mesh_id);
		if (mesh_id == 0) return 0;

		meshloader_anim.load_mesh_data_no_uv(mesh_id, &mesh_data);
		if (mesh_data.num_vertices == 0) return 0;

		bool appended = buffer(MODE_ANIM_NOUV).append_geometry(mesh_id, mesh_data);
		mesh_data.release();
		if (!appended) return 0;

		char msg[MAX_LOG_MSG_LENGTH] = {};
		snprintf(msg, MAX_LOG_MSG_LENGTH, "Add Anim Mesh (no uv), id[%d], %s", mesh_id, filepath);
		console->add_entry(msg, SUCCESS, RNDR);
		return mesh_id;
	}

	// every buffer's instance stream is rebuilt from zero each frame
	void clear_instances()
	{
		for (uint mode = 0; mode < DRAW_MODE_COUNT; mode++)
			drawbuffers[mode].clear_instances();
	}

	// drop the current scene's content : gpu buffers, mesh caches, clips and
	// textures. the shaders and the debug overlays are renderer state and stay
	void unload_scene()
	{
		for (uint mode = 0; mode < DRAW_MODE_COUNT; mode++)
			drawbuffers[mode].reset();

		meshloader.clear();
		meshloader_anim.clear();
		animloader.clear();
		debug_draw.reset_dynamic();
		txloader.clear();
		txloader.init();
		foam_pattern_tx = txloader.load("assets/textures/foam_pattern.jpg");

		animdebug.slot = (uint)-1;
		stats = {};
	}

	// the scene owns the camera : it passes this frame's matrices in
	void draw(GameWindow* window, mat4 proj_view)
	{
		// animation playback : the loader runs every player, we only ship finished poses
		animloader.update(window->dtime);

		// bind & clear deferred framebuffer (both passes draw into it)
		glBindFramebuffer(GL_FRAMEBUFFER, window->gbuf.rendertarget.fbo);
		glViewport(0, 0, window->screen_width, window->screen_height);
		glClearColor(0, 0, 0, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		stats = {}; // reset per-frame draw statistics (read by the console)

		// soft bodies are genuine two-sided sheets, and the studio floor/backdrop
		// are single-sided planes : the deferred pass draws everything double-sided
		glDisable(GL_CULL_FACE);

		// every mode, one loop : the table supplies the shader, the buffer supplies its
		// meshes. an empty mode skips the bind entirely, and the no-uv modes need nothing
		// new here - a mesh with no uv column samples the texture at (0,0), and with the
		// instance material's texture flag off the flat material colour is what lands
		for (uint mode = 0; mode < DRAW_MODE_COUNT; mode++)
		{
			DrawBuffer& db = drawbuffers[mode];
			if (db.num_meshes == 0) continue;
			if (!DRAW_MODE_INFO[mode].deferred) continue; // forward water owns a later pass

			ShaderProgram& shader = shaders[DRAW_MODE_INFO[mode].shader];

			glBindVertexArray(db.vao);
			shader.bind();
			glUniformMatrix4fv(glGetUniformLocation(shader.id, "proj_view"), 1, GL_FALSE, (float*)&proj_view);
			shader.set_float("night", 0.f);

			drawlists[mode].update(&db);
			draw_pass(&db, &drawlists[mode]);
		}
	}

	// one instanced pass over the light volumes, additively into the linear frame.
	// the fragment shader reads the g-buffer, so a volume that covers a pixel but
	// reaches no surface in it costs one texture fetch and a discard
	void draw_light_volumes(GameWindow* window, mat4 proj_view, vec3 view_pos,
	                        const Point_Light* lights, uint count)
	{
		if (count == 0) return;
		if (count > MAX_LIGHTS) count = MAX_LIGHTS;

		static LightInstance spheres[MAX_LIGHTS];
		static LightInstance cones[MAX_LIGHTS];
		uint num_spheres = 0, num_cones = 0;

		for (uint i = 0; i < count; i++)
		{
			const Point_Light& l = lights[i];
			vec3 dir = l.direction;
			if (dot(dir, dir) < 1e-6f) dir = vec3(0, -1, 0);
			else                       dir = normalize(dir);

			LightInstance inst = {};
			inst.pos_radius      = vec4(l.position, l.radius);
			inst.color_power     = vec4(l.color, l.intensity);
			inst.direction_shape = vec4(dir, (float)l.shape);
			inst.params          = vec4(l.outer_angle, cosf(l.outer_angle), cosf(l.inner_angle), 0.f);

			if (l.shape == LIGHT_CONE) cones[num_cones++] = inst;
			else                       spheres[num_spheres++] = inst;
		}

		glBindFramebuffer(GL_FRAMEBUFFER, window->hdr.lit.fbo);
		glViewport(0, 0, window->screen_width, window->screen_height);

		glDisable(GL_DEPTH_TEST);
		glDepthMask(GL_FALSE);
		glEnable(GL_CULL_FACE);
		glCullFace(GL_FRONT); // one layer of the closed volume, camera inside or out
		glEnable(GL_BLEND);
		glBlendFunc(GL_ONE, GL_ONE);

		lightvol.shader.bind();
		lightvol.shader.set_mat4("proj_view", proj_view);
		lightvol.shader.set_vec3("view_pos", view_pos);
		glUniform2i(glGetUniformLocation(lightvol.shader.id, "screen_size"),
		            (int)window->screen_width, (int)window->screen_height);

		glActiveTexture(GL_TEXTURE0); glBindTexture(GL_TEXTURE_2D, window->gbuf.rendertarget.position_tx);
		glActiveTexture(GL_TEXTURE1); glBindTexture(GL_TEXTURE_2D, window->gbuf.rendertarget.normal_tx);
		glActiveTexture(GL_TEXTURE2); glBindTexture(GL_TEXTURE_2D, window->gbuf.rendertarget.albedo_tx);

		glBindBuffer(GL_ARRAY_BUFFER, lightvol.inst_buffer);

		if (num_spheres)
		{
			glBindVertexArray(lightvol.sphere.vao);
			glBufferSubData(GL_ARRAY_BUFFER, 0, num_spheres * sizeof(LightInstance), spheres);
			glDrawElementsInstanced(GL_TRIANGLES, lightvol.sphere.num_indices, GL_UNSIGNED_INT, 0, num_spheres);
			stats.draw_calls++;
			stats.instances += num_spheres;
		}

		if (num_cones)
		{
			glBindVertexArray(lightvol.cone.vao);
			glBufferSubData(GL_ARRAY_BUFFER, 0, num_cones * sizeof(LightInstance), cones);
			glDrawElementsInstanced(GL_TRIANGLES, lightvol.cone.num_indices, GL_UNSIGNED_INT, 0, num_cones);
			stats.draw_calls++;
			stats.instances += num_cones;
		}

		glCullFace(GL_BACK);
		glDisable(GL_BLEND);
		glEnable(GL_CULL_FACE);
		glDepthMask(GL_TRUE);
		glEnable(GL_DEPTH_TEST);
	}

	// append the light volumes as coloured wire shapes : a sphere for a point
	// light, a cone for a spot, with a short axis line for the spot direction
	void append_light_debug(const Point_Light* lights, uint count)
	{
		if (!debug_draw.lights) return;

		for (uint i = 0; i < count; i++)
		{
			const Point_Light& l = lights[i];
			vec4 color = vec4(glm::mix(l.color, vec3(1.f), .25f), .85f);

			if (l.shape == LIGHT_CONE)
			{
				vec3 dir = l.direction;
				if (dot(dir, dir) < 1e-6f) dir = vec3(0, -1, 0);
				else                       dir = normalize(dir);

				debug_draw.append_light_cone(l.position, dir, l.radius, l.outer_angle, color);
				if (debug_draw.light_directions)
					debug_draw.append_segment(l.position, l.position + dir * fminf(l.radius, 5.f),
					                          vec4(1.f, .92f, .55f, 1.f));
			}
			else
			{
				debug_draw.append_sphere(l.position, l.radius, color);
				debug_draw.append_cross(l.position, .25f, vec4(1.f, .92f, .55f, 1.f));
			}
		}
	}

	// forward water : copy the lit frame, then draw the water meshes over it. the
	// copy is what the surface refracts; the g-buffer depth says how much water is
	// between the surface and the floor
	void draw_water(GameWindow* window, mat4 proj_view, vec3 view_pos, const Lighting& l)
	{
		DrawBuffer& db = drawbuffers[MODE_WATER];
		if (db.num_meshes == 0) return;

		// copy the g-buffer depth into the lit target's own depth renderbuffer : the
		// water shader samples the g-buffer depth, and a texture cannot be both
		// attached to the draw target and sampled in the same pass
		glBindFramebuffer(GL_READ_FRAMEBUFFER, window->gbuf.rendertarget.fbo);
		glBindFramebuffer(GL_DRAW_FRAMEBUFFER, window->hdr.lit.fbo);
		glBlitFramebuffer(0, 0, window->screen_width, window->screen_height,
		                  0, 0, window->screen_width, window->screen_height,
		                  GL_DEPTH_BUFFER_BIT, GL_NEAREST);

		glBindFramebuffer(GL_READ_FRAMEBUFFER, window->hdr.lit.fbo);
		glBindFramebuffer(GL_DRAW_FRAMEBUFFER, window->hdr.scene_copy.fbo);
		glBlitFramebuffer(0, 0, window->screen_width, window->screen_height,
		                  0, 0, window->screen_width, window->screen_height,
		                  GL_COLOR_BUFFER_BIT, GL_NEAREST);

		glBindFramebuffer(GL_FRAMEBUFFER, window->hdr.lit.fbo);
		glViewport(0, 0, window->screen_width, window->screen_height);

		glEnable(GL_DEPTH_TEST);
		glDepthFunc(GL_LEQUAL);
		glDepthMask(GL_TRUE);
		glDisable(GL_BLEND);
		glEnable(GL_CULL_FACE);

		ShaderProgram& shader = shaders[DRAW_MODE_INFO[MODE_WATER].shader];
		shader.bind();
		shader.set_mat4("proj_view", proj_view);
		shader.set_vec3("view_pos", view_pos);
		shader.set_float("near_plane", 0.1f); // matches Scene::update's projection
		shader.set_float("far_plane", window->far_plane);
		glUniform2f(glGetUniformLocation(shader.id, "screen_size"),
		            (float)window->screen_width, (float)window->screen_height);

		vec3 sun_dir = l.sun_direction;
		if (dot(sun_dir, sun_dir) < 1e-6f) sun_dir = vec3(-0.36f, -0.84f, -0.30f);
		sun_dir = normalize(sun_dir);

		shader.set_vec3 ("sun_direction", sun_dir);
		shader.set_vec3 ("sun_color", l.sun_color);
		shader.set_float("sun_power", l.sun_power);
		shader.set_vec3 ("moon_direction", l.moon_direction);
		shader.set_vec3 ("moon_color", l.moon_color);
		shader.set_float("moon_power", l.moon_power);
		shader.set_float("moon_disk", l.moon_disk);
		shader.set_vec3("zenith", l.zenith);
		shader.set_vec3("haze",   l.haze);
		shader.set_vec3("glow",   l.glow);
		shader.set_vec3("ground", l.ground);
		shader.set_vec3("ambient_up",   l.ambient_up);
		shader.set_vec3("ambient_side", l.ambient_side);
		shader.set_vec3("ambient_down", l.ambient_down);
		shader.set_float("fog_density", l.fog_density);

		shader.set_vec3 ("foam_color", window->water_foam_color);
		shader.set_float("foam_roughness", window->water_foam_roughness);
		shader.set_float("water_detail", window->water_detail);
		shader.set_float("water_sss", window->water_sss);
		shader.set_float("water_time", window->water_time);

		glActiveTexture(GL_TEXTURE1); glBindTexture(GL_TEXTURE_2D, window->hdr.scene_copy.color_tx);
		glActiveTexture(GL_TEXTURE2); glBindTexture(GL_TEXTURE_2D, window->gbuf.rendertarget.depth_tx);
		glActiveTexture(GL_TEXTURE3); txloader.bind(window->water_normal_tx, 3);
		glActiveTexture(GL_TEXTURE4); txloader.bind(foam_pattern_tx, 4);
		glActiveTexture(GL_TEXTURE5); glBindTexture(GL_TEXTURE_2D, window->gbuf.rendertarget.normal_tx);
		glActiveTexture(GL_TEXTURE0);

		glBindVertexArray(db.vao);
		drawlists[MODE_WATER].update(&db);
		draw_pass(&db, &drawlists[MODE_WATER]);

		glDepthFunc(GL_LESS);
	}

	// issue the instanced draw calls for one buffer, tallying stats. the pose upload is
	// the only thing a skinned buffer adds, and pose_buffer == 0 is what says which kind
	// this buffer is
	void draw_pass(DrawBuffer* db, DrawList* dl)
	{
		// one binding slot, several skinned buffers : claim it before uploading this one's
		// pose, or the shader reads a different buffer's
		if (db->pose_buffer)
			glBindBufferBase(GL_UNIFORM_BUFFER, 1, db->pose_buffer);

		uint bound_tx = (uint)-1; // rebind only when the mesh draws with a different texture

		for (uint i = 0; i < MAX_MESHES; i++)
		{
			if (dl->meshlist[i] == 0)
				continue;

			// pose : the player bound to this mesh (identity pose when unbound)
			if (db->pose_buffer)
			{
				uint player = db->mesh_info[i].anim_player;
				if (player < animloader.num_players)
				{
					Anim_Player& pl = animloader.players[player];
					db->update_pose(pl.pose, animloader.clips[pl.clip_id].num_bones);
				}
				else
					db->update_pose(db->identity_pose, MAX_ANIM_BONES);
			}

			if (db->mesh_info[i].tx_id != bound_tx)
			{
				bound_tx = db->mesh_info[i].tx_id;
				txloader.bind(bound_tx);
			}

			auto& mp = dl->mesh_params[i];
			glDrawElementsInstancedBaseVertexBaseInstance(GL_TRIANGLES, mp.num_indices, GL_UNSIGNED_INT,
				(void*)mp.base_index, mp.num_instances, mp.base_vertex, mp.base_instance);

			// record what we just submitted (mesh_info shares this index with mesh_params)
			stats.draw_calls++;
			stats.meshes_drawn++;
			stats.instances  += mp.num_instances;
			stats.triangles  += (mp.num_indices / 3) * mp.num_instances;
			stats.vertices   += db->mesh_info[i].num_vertices * mp.num_instances;
		}
	}

};
