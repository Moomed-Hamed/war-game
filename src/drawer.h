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

// ---- the four draw modes -------------------------------------------------------------
// a mode is a vertex layout, a stride and a shader pair. the no-uv modes leave attribute 2
// disabled, so the sampler reads its constant (0,0) and the instance material's texture
// flag decides whether that counts : no shader branch, and no second shader
enum DrawMode : uint {
	MODE_STATIC_UV = 0, // 1 : the floor, props, generated shapes
	MODE_STATIC_NOUV,   // 3 : the same meshes, built without a uv column
	MODE_ANIM_UV,       // 2 : weapons and rigs, skinned
	MODE_ANIM_NOUV,     // 4 : the same, built without a uv column
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
};

const DrawMode_Info DRAW_MODE_INFO[DRAW_MODE_COUNT] = {
	{ "1  static + uv",    0, sizeof(MeshVertexData),         false, true  },
	{ "3  static, no uv",  0, sizeof(MeshVertexDataNoUV),     false, false },
	{ "2  skinned + uv",   1, sizeof(MeshVertexDataAnim),     true,  true  },
	{ "4  skinned, no uv", 1, sizeof(MeshVertexDataAnimNoUV), true,  false },
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

struct Anim_Debug
{
	bool enabled; // the Anim tab's master toggle
	bool bones;   // the octahedra themselves
	bool links;   // a line from every bone to its parent
	bool axes;    // a cross per joint, on the bone's own frame
	bool no_uv;   // which skinned buffer the slot belongs to (mode 2 or mode 4)
	uint slot;    // the mesh_info slot to draw : what the dropdown picked

	DrawBuffer    lines;
	ShaderProgram shader;
	uint bone_mesh, link_mesh, joint_mesh;

	void init()
	{
		// calloc leaves these at zero : the overlay starts off, and when it goes on the
		// bones and the links between them are what you want first
		bones = true;
		links = true;

		shader.create("assets/shaders/wire.vert", "assets/shaders/wire.frag");
		lines.init(); // 1MB default : a rig barely uses it

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

		bone_mesh = 1;
		lines.append_geometry(bone_mesh, bone);
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

		joint_mesh = 2;
		lines.append_geometry(joint_mesh, joint);
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

		link_mesh = 3;
		lines.append_geometry(link_mesh, link);
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

	// one shape's line draw, straight from the buffer's own bookkeeping
	void draw_shape(uint mesh)
	{
		for (uint i = 0; i < MAX_MESHES; i++)
		{
			if (lines.mesh_info[i].mesh_id != mesh)
				continue;

			auto& mi = lines.mesh_info[i];
			if (mi.num_instances)
				glDrawElementsInstancedBaseVertexBaseInstance(GL_LINES, mi.num_indices, GL_UNSIGNED_INT,
					(void*)mi.base_index, mi.num_instances, mi.base_vertex, mi.base_instance);
			return;
		}
	}

	// the selected mesh's rig, as the player posing it has it right now
	void draw(mat4 proj_view, DrawBuffer* meshes, AnimLoader* anim)
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

		// after lighting the backbuffer depth is nobody's : start clean
		glBindFramebuffer(GL_FRAMEBUFFER, 0);
		glClear(GL_DEPTH_BUFFER_BIT);

		// three runs that must not interleave : a draw reads its instances from one
		// block. the instance albedo carries each bone's tier colour and wire.frag
		// multiplies it in, so a single mesh draws every tier
		mat4 place = mi.last_model;
		lines.clear_instances();

		if (bones)
			for (uint b = 0; b < bones; b++)
			{
				vec3  axis = tail[b] - head[b];
				float len  = length(axis);
				if (len < 1e-4f) continue;

				Instance inst = INSTANCE_DEFAULT;
				inst.mat_a = vec4(SKELETON_TIER_COLOR[tier[b]], 0.f);
				inst.model = bone_at(place, head[b], axis / len, len);
				lines.append_instances(bone_mesh, 1, &inst);
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
				lines.append_instances(link_mesh, 1, &inst);
			}

		if (axes)
			for (uint b = 0; b < bones; b++)
			{
				vec3  axis = tail[b] - head[b];
				float len  = length(axis);
				if (len < 1e-4f) continue;

				Instance inst = INSTANCE_DEFAULT;
				inst.model = bone_at(place, head[b], axis / len, len * 0.20f);
				lines.append_instances(joint_mesh, 1, &inst);
			}

		glBindVertexArray(lines.vao);
		shader.bind();
		glUniformMatrix4fv(glGetUniformLocation(shader.id, "proj_view"), 1, GL_FALSE, (float*)&proj_view);

		// white : the instances carry the colour now. the crosses stay neutral, so the
		// tier ramp is the only thing tinting
		glUniform4f(glGetUniformLocation(shader.id, "color"), 1.f, 1.f, 1.f, 1.f);
		draw_shape(bone_mesh);
		draw_shape(link_mesh);
		glUniform4f(glGetUniformLocation(shader.id, "color"), 0.58f, 0.62f, 0.68f, 1.f);
		draw_shape(joint_mesh);
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

	// drawing : one buffer and one draw list per mode. the table is the only list of
	// them in the engine, so a fifth layout is a row in DRAW_MODE_INFO and nothing else
	DrawBuffer drawbuffers[DRAW_MODE_COUNT];
	DrawList   drawlists[DRAW_MODE_COUNT];

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

		// 8MB apiece : the scene needs a few hundred KB, and the buffers only grow
		for (uint mode = 0; mode < DRAW_MODE_COUNT; mode++)
			drawbuffers[mode].init(MegaByte(8), mode);

		txloader.init();
		animloader.init(); // playback defaults (calloc skips them)
		animdebug.init();

		console->add_entry((char*)"Init GameRenderer", SUCCESS, RNDR);
	}

	// the animator's own overlay pass : drawn straight to the backbuffer, over the
	// lit frame, after the scene. mirrors Phys::draw_debug
	void draw_skeleton(mat4 proj_view)
	{
		// a no-uv rig lives in the other skinned buffer, and the slot number alone cannot
		// say which : the tab sets no_uv alongside it
		DrawBuffer* meshes = &buffer(animdebug.no_uv ? MODE_ANIM_NOUV : MODE_ANIM_UV);
		animdebug.draw(proj_view, meshes, &animloader);
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

		// every mode, one loop : the table supplies the shader, the buffer supplies its
		// meshes. an empty mode skips the bind entirely, and the no-uv modes need nothing
		// new here - a mesh with no uv column samples the texture at (0,0), and with the
		// instance material's texture flag off the flat material colour is what lands
		for (uint mode = 0; mode < DRAW_MODE_COUNT; mode++)
		{
			DrawBuffer& db = drawbuffers[mode];
			if (db.num_meshes == 0) continue;

			ShaderProgram& shader = shaders[DRAW_MODE_INFO[mode].shader];

			glBindVertexArray(db.vao);
			shader.bind();
			glUniformMatrix4fv(glGetUniformLocation(shader.id, "proj_view"), 1, GL_FALSE, (float*)&proj_view);

			drawlists[mode].update(&db);
			draw_pass(&db, &drawlists[mode]);
		}
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
