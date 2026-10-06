#include "drawer.h"

/* ------------------------- *
	-  Animation Rendering   -
 * ------------------------- */

// drawer.h for animated meshes : same buffer/list/renderer pattern, but vertices
// carry skinning data (3 weights + 3 bone ids) and the pose uploads to UBO slot 1.
// the .anim file is row-major : add_anim() transposes it once at load, so
// everything downstream is plain column-major skinning.

// rotation hold : [first,last] = boundaries where no bone turns
struct Anim_Hold { uint first, last; };

// playback state : written by draw(), read by the Anim tab in ux.h.
// control : written by the tab, read by draw() - each side only writes its fields
struct Anim_State {
	float elapsed, duration; // seconds (duration 0 = never played -> static frame 0)
	float frame_f;           // fractional frame head - interpolation below the frame
	uint  this_frame;
	uint  num_frames, num_bones;
	Anim_Hold holds[8];      // rotation holds from load, banded on the tab's timeline
	uint  num_holds;
	bool  playing;
} anim_state;

struct Anim_Control {
	float speed     = 1.f;  // playback rate
	int   time_ease = 0;    // global time warp, see GameRendererAnim::time_warp()
	float weight    = 0.4f; // juice : spring stiffness/settle, 0 = exact clip
	bool  loop      = true;
	bool  paused    = false;
	bool  scrubbing = false; // the tab is dragging the timeline
	float scrub_t   = 0.f;   // 0..1 while scrubbing
	bool  restart   = false; // one-shot command from the tab
} anim_control;

// Vertex data as it lives in the gpu buffer : position, normal, uv, weights, bones.
// MUST match the glVertexAttribPointer layout set up in DrawBufferAnim::init()
// and the inputs of assets/shaders/anim.vert
struct MeshVertexDataAnim {
	vec3 position, normal;
	vec2 uv;
	vec3 weights; // top-3 skinning weights (sum to 1)
	ivec3 bones;  // bone id for each weight
}; // 56 bytes

// the same instancing rules as drawer.h apply : each mesh id's instances can only
// be appended once per frame, since instance data arrives out of order every frame
struct DrawBufferAnim
{
	struct {
		uint mesh_id;

		uint num_vertices;
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
	GLuint pose_buffer; // skeleton UBO : mat4 joint_transforms[16] at binding slot 1

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
	// Generate GeometryBuffer : Mesh positions, normals, uvs, weights, bones
	// Generate InstanceBuffer : Mat4 for instance transforms
	// Generate IndexBuffer    : uints for drawing meshes from verts
	// Generate PoseBuffer     : the skeleton UBO the vertex shader reads
	void init(uint buffer_size = MegaByte(1)) { // m4's mesh_anim alone is 421KB, 256KB isn't enough

		glGenVertexArrays(1, &vao);

		// this size is for : vertex buffer, index buffer, instance-data buffer
		max_buffer_size = buffer_size;

		// generate gpu buffers : mesh vertices, mesh indices, instance data, pose
		glGenBuffers(1, &geom_buffer); // vertex buffer
		glGenBuffers(1, &indx_buffer); // index  buffer
		glGenBuffers(1, &inst_buffer); // instance-data buffer
		glGenBuffers(1, &pose_buffer); // skeleton UBO

		// IMPORTANT : bind the vao *before* binding anything else!
		glBindVertexArray(vao);

		// ------------------------------------------------------------------------------------

		// init gpu buffers for mesh vertices(positions, normals, uvs, weights, bones)
		glBindBuffer(GL_ARRAY_BUFFER, geom_buffer);
		glBufferData(GL_ARRAY_BUFFER, max_buffer_size, NULL, GL_STATIC_DRAW);

		// define mesh vertex data layout : vec3 position, vec3 normal, vec2 uv, vec3 weights, ivec3 bones
		uint stride = sizeof(MeshVertexDataAnim);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, stride, (void*)0); // vec3 position
		glEnableVertexAttribArray(0);
		glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, stride, (void*)(sizeof(vec3))); // vec3 normal
		glEnableVertexAttribArray(1);
		glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, stride, (void*)(sizeof(vec3) * 2)); // vec2 uv
		glEnableVertexAttribArray(2);
		glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, stride, (void*)(sizeof(vec3) * 2 + sizeof(vec2))); // vec3 weights
		glEnableVertexAttribArray(3);
		// bones are indices, not floats : glVertexAttribPointer would read garbage here
		glVertexAttribIPointer(4, 3, GL_INT, stride, (void*)(sizeof(vec3) * 2 + sizeof(vec2) + sizeof(vec3))); // ivec3 bones
		glEnableVertexAttribArray(4);

		// ------------------------------------------------------------------------------------

		// init gpu buffer for per-mesh instance data
		glBindBuffer(GL_ARRAY_BUFFER, inst_buffer);
		glBufferData(GL_ARRAY_BUFFER, max_buffer_size, NULL, GL_DYNAMIC_DRAW);

		// define per-mesh instance-data layout (mat4 instance_data)
		uint num_vertex_attribs = 5; // we have already defined attribuetes 0,1,2,3,4 above so we start at 5
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
		// 3 -> vec3 weights
		// 4 -> ivec3 bones (integer attribute!)
		// 5..8 -> mat4 instance_model (one vec4 per instance)

		// ------------------------------------------------------------------------------------

		// gpu index buffer : ordered uints that reference vertices for building meshes
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indx_buffer);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, max_buffer_size, NULL, GL_STATIC_DRAW);

		// ------------------------------------------------------------------------------------

		// skeleton UBO : one mat4 per bone, std140, binding slot 1
		glBindBuffer(GL_UNIFORM_BUFFER, pose_buffer);
		glBufferData(GL_UNIFORM_BUFFER, MAX_ANIM_BONES * sizeof(mat4), NULL, GL_DYNAMIC_DRAW);
		glBindBufferBase(GL_UNIFORM_BUFFER, 1, pose_buffer);

		// identity pose : with no clip loaded, the mesh draws unskinned instead of collapsed
		mat4 identity[MAX_ANIM_BONES];
		for (uint i = 0; i < MAX_ANIM_BONES; i++) identity[i] = mat4(1);
		glBufferSubData(GL_UNIFORM_BUFFER, 0, sizeof(identity), identity);

		// ------------------------------------------------------------------------------------

		// log
		char msg[MAX_LOG_MSG_LENGTH] = {};
		snprintf(msg, MAX_LOG_MSG_LENGTH, "Init DrawBufferAnim | Size : [%d]", max_buffer_size);
		console->add_entry(msg, SUCCESS, RNDR);
	}

	// Upload the skeleton pose : mat4 joint_transforms[num_bones] at binding slot 1
	void update_pose(mat4* pose, uint num_bones) {
		glBindBuffer(GL_UNIFORM_BUFFER, pose_buffer);
		glBufferSubData(GL_UNIFORM_BUFFER, 0, num_bones * sizeof(mat4), pose);
	}

	// Check for available space, append geometry & index data from mesh data
	// returns false (after logging) if the gpu buffers don't have room
	bool append_geometry(uint mesh_id, Mesh_Data_Anim mesh_data) {

		uint num_vertices = mesh_data.num_vertices;
		uint num_indices  = mesh_data.num_indices;

		uint vert_data_size  = num_vertices * sizeof(MeshVertexDataAnim);
		uint index_data_size = num_indices  * sizeof(uint);

		// Ensure new mesh data will fit in gpu buffers!

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
			return false; // don't write past the gpu buffer
		}

		// update geometry buffer

		MeshVertexDataAnim* mesh_verts = Alloc(MeshVertexDataAnim, num_vertices);

		// assemble vertex buffer to be copied to gpu buffer
		for (uint i = 0; i < num_vertices; i++)
			mesh_verts[i] = MeshVertexDataAnim{ mesh_data.positions[i], mesh_data.normals[i], mesh_data.uvs[i],
				mesh_data.weights[i], mesh_data.bones[i] };

		// upload mesh data to gpu buffer
		glBindBuffer(GL_ARRAY_BUFFER, geom_buffer);
		glBufferSubData(GL_ARRAY_BUFFER, geom_size, vert_data_size, mesh_verts);

		free(mesh_verts);

		// update index buffer
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indx_buffer);
		glBufferSubData(GL_ELEMENT_ARRAY_BUFFER, indx_size, index_data_size, mesh_data.indices);

		// update mesh info
		uint mesh_index = num_meshes++;
		mesh_info[mesh_index].mesh_id       = mesh_id;
		mesh_info[mesh_index].num_vertices  = num_vertices;
		mesh_info[mesh_index].base_vertex   = geom_size / sizeof(MeshVertexDataAnim);
		mesh_info[mesh_index].num_indices   = num_indices;
		mesh_info[mesh_index].base_index    = indx_size;

		// update buffer sizes, these are used later as offsets into these buffers
		geom_size += vert_data_size;
		indx_size += index_data_size;

		return true;
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

		// unknown mesh id : log once, don't spam (this can run every frame)
		static uint last_unknown_id = (uint)-1;
		if (mesh_id != last_unknown_id)
		{
			char msg[MAX_LOG_MSG_LENGTH];
			snprintf(msg, MAX_LOG_MSG_LENGTH, "Instances for unknown mesh id [%d]!", mesh_id);
			console->add_entry(msg, FIXME, RNDR);
			last_unknown_id = mesh_id;
		}
	}

	// Does not free memory, just resets counts
	void clear_instances()
	{
		total_instances = 0;
		inst_size = 0;
	}
};

// stores mesh ids + generates draw-params for opengl (same job as drawer's DrawList)
struct DrawListAnim
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

struct GameRendererAnim
{
	Camera camera; // 3d camera

	// per-frame draw statistics, filled by draw() (a future Anim tab can show these)
	struct {
		uint draw_calls;
		uint meshes_drawn;
		uint instances;
		uint triangles;
		uint vertices;
	} stats;

	// Asset Loading
	MeshLoaderAnim meshloader;
	AnimLoader     animloader; // reads .anim files (skeleton + keyframes)
	TextureLoader  txloader;
	ShaderProgram  shaders[MAX_SHADERS]; // anim.vert + geom.frag (same frag as drawer)

	// Animation state
	Animation clip;                 // current clip (loaded by add_anim)
	mat4 pose[MAX_ANIM_BONES];      // composed skeleton pose, uploaded every frame
	float elapsed;                  // playback clock in seconds
	float duration;                 // seconds per loop ; 0 = static frame 0

	// loop seam : blend the fresh start toward the pose that was on screen at the wrap
	mat4  seam_pose[MAX_ANIM_BONES];
	float seam_t;        // seconds into the crossfade (>= seam_duration = idle)
	float seam_duration; // set in init() - calloc'd struct skips default member values
	bool  wrapped;       // raised by advance_time() when the clock looped this frame

	// juice : per-bone spring chasing the sampled pose (weight 0 = pass-through)
	vec3  spring_pos[MAX_ANIM_BONES], spring_vel[MAX_ANIM_BONES];
	quat  spring_rot[MAX_ANIM_BONES];
	vec3  spring_ang_vel[MAX_ANIM_BONES];
	bool  spring_armed;

	// Drawing
	DrawBufferAnim drawbuffer; // stores geometry, indices, instance data
	DrawListAnim   drawlist;   // per-frame draw call information

	void init()
	{
		seam_duration = 0.15f; // loop crossfade seconds (struct itself comes from calloc)

		shaders[0].create("assets/shaders/anim.vert", "assets/shaders/geom.frag");

		drawbuffer.init();
		txloader.init();

		console->add_entry("Init GameRendererAnim", SUCCESS, RNDR);
	}
	void add_mesh(const char* filepath)
	{
		uint mesh_id = {};
		Mesh_Data_Anim mesh_data = {};
		meshloader.load_mesh(filepath, &mesh_id);
		if (mesh_id == 0) return; // load failed, error already logged

		meshloader.load_mesh_data(mesh_id, &mesh_data);
		if (mesh_data.num_vertices == 0) return; // no data, error already logged

		bool appended = drawbuffer.append_geometry(mesh_id, mesh_data);
		mesh_data.release(); // no longer needed since it's now in the gpu buffer

		if (!appended) return; // buffer full, error already logged

		// log
		char msg[MAX_LOG_MSG_LENGTH] = {};
		snprintf(msg, MAX_LOG_MSG_LENGTH, "Add Anim Mesh, id[%d], %s", mesh_id, filepath);
		console->add_entry(msg, SUCCESS, RNDR);
	}
	// load the clip : skeleton (parents + ibm) + keyframes
	void add_anim(const char* filepath)
	{
		animloader.load_animation(&clip, filepath);
		if (clip.num_bones == 0)
		{
			console->add_entry("Add Anim : no bones loaded", FIXME, RNDR);
			return;
		}

		// row-major file -> column-major glm, once here
		for (uint b = 0; b < clip.num_bones; b++)
		{
			for (uint f = 0; f < clip.num_frames; f++)
				transpose_in_place(clip.keyframes[b][f]);
			transpose_in_place(clip.ibm[b]);
		}

		// rotation holds : spans where no bone turns - bands on the tab's timeline.
		// rotation-only : translation drifts through the whole clip, never rests
		anim_state.num_holds = 0;
		int run_first = -1;
		for (uint f = 0; f + 1 < clip.num_frames; f++)
		{
			bool still = true;
			for (uint b = 0; b < clip.num_bones && still; b++)
			{
				quat qa = glm::quat_cast(mat3(clip.keyframes[b][f]));
				quat qb = glm::quat_cast(mat3(clip.keyframes[b][f + 1]));
				float d = fabsf(glm::dot(qa, qb));
				if (d > 1.f) d = 1.f;
				float deg = 2.f * acosf(d) * (180.f / PI);
				if (deg >= 0.05f) still = false;
			}
			if (still)
			{
				if (run_first < 0) run_first = (int)f;
			}
			else if (run_first >= 0)
			{
				if (anim_state.num_holds < 8)
					anim_state.holds[anim_state.num_holds++] = { (uint)run_first, f - 1 };
				run_first = -1;
			}
		}
		if (run_first >= 0 && anim_state.num_holds < 8)
			anim_state.holds[anim_state.num_holds++] = { (uint)run_first, clip.num_frames - 2 };

		spring_armed = false;
		anim_state.num_frames = clip.num_frames;
		anim_state.num_bones  = clip.num_bones;

		char msg[MAX_LOG_MSG_LENGTH] = {};
		snprintf(msg, MAX_LOG_MSG_LENGTH, "Add Anim, bones[%d], frames[%d], holds[%d]",
			clip.num_bones, clip.num_frames, anim_state.num_holds);
		console->add_entry(msg, SUCCESS, RNDR);
	}

	// ---- playback --------------------------------------------------------------

	// sample the clip at a fractional frame head into pose[] : slerp rotations +
	// lerp offsets per segment (matrix lerp pinches on fast frames), then
	// world = parent * local, pose = world * ibm
	void sample_pose(float head)
	{
		uint n = clip.num_frames;
		if (n == 0 || clip.num_bones == 0) return;
		if (head >= (float)n) head = (float)n - 0.001f; // one-shot end hold

		uint i0 = (uint)head;
		uint i1 = (i0 + 1 < n) ? i0 + 1 : i0;
		float t = head - (float)i0;

		mat4 local[MAX_ANIM_BONES];
		for (uint b = 0; b < clip.num_bones; b++)
		{
			const mat4& A = clip.keyframes[b][i0];
			const mat4& B = clip.keyframes[b][i1];

			quat qa = glm::quat_cast(mat3(A));
			quat qb = glm::quat_cast(mat3(B));
			if (glm::dot(qa, qb) < 0.f) qb = -qb; // shortest arc
			quat q = glm::slerp(qa, qb, t);

			vec3 p = vec3(A[3]) + (vec3(B[3]) - vec3(A[3])) * t;

			mat4 M = glm::mat4_cast(q);
			M[3] = vec4(p, 1.f);
			local[b] = M;
		}

		pose[0] = local[0]; // root (parents[0] == 0 == itself)
		for (uint i = 1; i < clip.num_bones; i++)
			pose[i] = pose[clip.parents[i]] * local[i];
		for (uint i = 0; i < clip.num_bones; i++)
			pose[i] = pose[i] * clip.ibm[i];
	}

	// global rhythm warp for the whole clip : linear by default (timing is baked
	// into the file), presets are for feel experiments from the tab
	float time_warp(float u)
	{
		switch (anim_control.time_ease)
		{
			case 1: return u * u * (3.f - 2.f * u);                            // smooth
			case 2: return u < .5f ? 4.f * u * u * u
				: 1.f - powf(-2.f * u + 2.f, 3.f) / 2.f;                       // in-out cubic
			case 3: return 1.f - powf(1.f - u, 3.f);                           // out cubic
			case 4: return u < .5f ? 16.f * u * u * u * u * u
				: 1.f - powf(-2.f * u + 2.f, 5.f) / 2.f;                       // in-out quint
			default: return u;
		}
	}

	// the clock : speed, pause, scrub, restart, loop (raises wrapped on the seam)
	void advance_time(float dt)
	{
		wrapped = false;

		if (anim_control.restart)
		{
			anim_control.restart = false;
			elapsed = 0;
			seam_t = seam_duration; // no seam on a manual jump
			spring_armed = false;   // snap instead of swinging from the old spot
			return;
		}
		if (anim_control.scrubbing)
		{
			elapsed = anim_control.scrub_t * duration;
			seam_t = seam_duration;
			spring_armed = false;   // follow the dragged pose exactly, no jelly
			return;
		}
		if (duration <= 0.f || anim_control.paused) return;

		elapsed += dt * anim_control.speed;
		if (elapsed >= duration)
		{
			if (anim_control.loop)
			{
				while (elapsed >= duration) elapsed -= duration;
				wrapped = true; // draw() freezes the on-screen pose for the seam blend
			}
			else
			{
				elapsed = duration; // finished : hold the last frame
			}
		}
	}

	// the juice : a spring per bone chasing the sampled pose - overshoot and settle
	// out of the sparse keys. weight 0 = the clip passes through untouched
	void apply_spring(float dt)
	{
		if (clip.num_bones == 0) return;

		float w = anim_control.weight;
		if (!spring_armed || w < 0.01f)
		{
			// (re)sync to the target : bypass, or a clean base when weight comes up
			for (uint i = 0; i < clip.num_bones; i++)
			{
				spring_pos[i]     = vec3(pose[i][3]);
				spring_rot[i]     = glm::quat_cast(mat3(pose[i]));
				spring_vel[i]     = vec3(0.f);
				spring_ang_vel[i] = vec3(0.f);
			}
			spring_armed = true;
			return;
		}

		float k    = 60.f + 740.f * w * w; // stiffness
		float zeta = 0.85f - 0.35f * w;    // damping ratio : below 1 = bounces
		float c    = 2.f * zeta * sqrtf(k);

		for (uint i = 0; i < clip.num_bones; i++)
		{
			vec3 target_p(pose[i][3]);
			quat target_q = glm::quat_cast(mat3(pose[i]));

			// position
			vec3 dp = target_p - spring_pos[i];
			spring_vel[i] += (dp * k - spring_vel[i] * c) * dt;
			spring_pos[i] += spring_vel[i] * dt;

			// rotation : error as axis-angle -> torque -> angular velocity -> integrate
			quat qe = target_q * glm::conjugate(spring_rot[i]);
			if (qe.w < 0.f) qe = -qe; // shortest arc
			vec3 axis(qe.x, qe.y, qe.z);
			float alen = glm::length(axis);
			vec3 torque = (alen > 1e-5f)
				? axis * ((2.f * atan2f(alen, qe.w) / alen) * k)
				: vec3(0.f);
			spring_ang_vel[i] += (torque - spring_ang_vel[i] * c) * dt;
			quat wq(0.f, spring_ang_vel[i].x, spring_ang_vel[i].y, spring_ang_vel[i].z);
			spring_rot[i] = glm::normalize(spring_rot[i] + 0.5f * (wq * spring_rot[i]) * dt);

			pose[i] = glm::mat4_cast(spring_rot[i]);
			pose[i][3] = vec4(spring_pos[i], 1.f);
		}
	}

	// start playback : loop_duration in seconds (one-shot is the tab's loop checkbox)
	void play(float loop_duration)
	{
		duration   = loop_duration;
		elapsed    = 0;
		seam_t     = seam_duration;
		spring_armed = false;

		char msg[MAX_LOG_MSG_LENGTH];
		snprintf(msg, MAX_LOG_MSG_LENGTH, "Play Anim | %.2fs per loop", duration);
		console->add_entry(msg, SUCCESS, RNDR);
	}

	// clear_framebuffer : pass false to join a frame the static renderer already cleared
	void draw(GameWindow* window, bool clear_framebuffer = true)
	{
		// skeleton pose : time -> warp -> sample -> seam blend -> spring
		if (clip.num_bones)
		{
			float dt = window->dtime;
			if (dt > 0.05f) dt = 0.05f; // hitch guard

			advance_time(dt);
			if (wrapped)
			{
				for (uint i = 0; i < clip.num_bones; i++)
					seam_pose[i] = pose[i]; // what was actually on screen at the wrap
				seam_t = 0;
			}

			float head = 0;
			if (duration > 0)
				head = time_warp(elapsed / duration) * clip.num_frames;

			sample_pose(head);

			if (seam_t < seam_duration)
			{
				float k = 1.f - seam_t / seam_duration; // 1 = frozen old pose, 0 = fresh clip
				for (uint i = 0; i < clip.num_bones; i++)
					pose[i] = pose[i] * (1.f - k) + seam_pose[i] * k;
				seam_t += dt;
			}

			apply_spring(dt);
			drawbuffer.update_pose(pose, clip.num_bones);

			// mirror state for the Anim tab
			anim_state.elapsed    = elapsed;
			anim_state.duration   = duration;
			anim_state.frame_f    = head;
			anim_state.this_frame = (head >= (float)clip.num_frames) ? clip.num_frames - 1 : (uint)head;
			anim_state.num_frames = clip.num_frames;
			anim_state.num_bones  = clip.num_bones;
			anim_state.playing    = (duration > 0 && !anim_control.paused &&
				(anim_control.loop || elapsed < duration));
		}
		// no clip loaded : the identity pose from DrawBufferAnim::init draws the mesh unskinned

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

		// bind the deferred framebuffer ; only the first renderer of the frame clears it
		glBindFramebuffer(GL_FRAMEBUFFER, window->gbuf.rendertarget.fbo);
		if (clear_framebuffer)
		{
			glClearColor(0, 0, 0, 1.0f);
			glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		}

		glBindVertexArray(drawbuffer.vao); // animated vertex layout
		shaders[0].bind();                 // anim.vert + geom.frag

		txloader.bind(0); // 0 = default texture

		// scene camera + (perspective) projection
		uint pv = glGetUniformLocation(shaders[0].id, "proj_view");
		glUniformMatrix4fv(pv, 1, GL_FALSE, (float*)&proj_view);

		drawlist.update(&drawbuffer); // generate opengl draw params

		stats = {}; // reset per-frame draw statistics

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

			// record what we just submitted (mesh_info shares this index with mesh_params)
			stats.draw_calls++;
			stats.meshes_drawn++;
			stats.instances  += num_instances;
			stats.triangles  += (num_indices / 3) * num_instances;
			stats.vertices   += drawbuffer.mesh_info[i].num_vertices * num_instances;
		}
	}
};
