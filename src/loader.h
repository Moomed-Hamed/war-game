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
	uint num_loaded; // total loaded since launch : source for mesh ids

	struct MeshInfo
	{
		uint id, size;
		char filepath[MAX_FILEPATH_LENGTH];
	} meshes[MAX_MESHES];

	void load_mesh(const char* filepath, uint* id) // returns mesh_id
	{
		for (uint i = 0; i < MAX_MESHES; i++)
		{
			if (strcmp(meshes[i].filepath, filepath) == 0) // 0 = strings match!
			{
				out(filepath << " already cached!");
				if (id) *id = meshes[i].id;
				return;
			}
		}

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

		// look for an empty slot
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
				meshes[i].size = get_file_size(filepath); // disk bytes, shown in the console

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
	uint num_loaded; // total loaded since launch : source for mesh ids

	struct MeshInfo
	{
		uint id, size;
		char filepath[MAX_FILEPATH_LENGTH];
	} meshes[MAX_MESHES];

	void load_mesh(const char* filepath, uint* id) // returns mesh_id
	{
		for (uint i = 0; i < MAX_MESHES; i++)
		{
			if (strcmp(meshes[i].filepath, filepath) == 0) // 0 = strings match!
			{
				out(filepath << " already cached!");
				if (id) *id = meshes[i].id;
				return;
			}
		}

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

		// look for an empty slot
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
				meshes[i].size = get_file_size(filepath); // disk bytes, shown in the console

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
const uint MAX_ANIMATIONS = 8; // clip slots in AnimLoader

// rotation hold : [first,last] = boundaries where no bone turns
struct Anim_Hold { uint first, last; };

const uint MAX_ANIM_SEGMENTS = 8; // animations sliced out of one file

// one animation inside a loaded file : a frame range with its own feel.
// files carry several back-to-back animations, the tab slices them up
struct Anim_Segment { uint first, last; int ease; float juice; };

struct Animation
{
	uint num_bones, num_frames;

	mat4  ibm[MAX_ANIM_BONES]; // inverse-bind matrices
	mat4* keyframes[MAX_ANIM_BONES]; // animation keyframes
	int   parents[MAX_ANIM_BONES]; // indices of parent bones

	Anim_Hold holds[8]; // rotation holds, found while loading
	uint num_holds;

	Anim_Segment segs[MAX_ANIM_SEGMENTS]; // frame ranges = separate animations
	uint num_segs;                        // always >= 1
};

const uint MAX_ANIM_PLAYERS = 8; // clips playing at the same time

// one clip in flight : its own clock, loop seam and springs, so several meshes
// can animate at once. state = written by update(), control = written by the tab
struct Anim_Player
{
	uint clip_id; // which clip plays
	uint seg;     // which segment of that clip

	float elapsed;  // playback clock in seconds
	float duration; // seconds per loop ; 0 = static frame 0

	// loop seam : blend the fresh start toward the pose that was on screen at the wrap
	mat4  seam_pose[MAX_ANIM_BONES];
	float seam_t;  // seconds into the crossfade (>= seam_duration = idle)
	bool  wrapped; // raised by advance_time() when the clock looped this frame

	// juice : per-bone spring chasing the sampled pose (weight 0 = pass-through)
	vec3  spring_pos[MAX_ANIM_BONES], spring_vel[MAX_ANIM_BONES];
	quat  spring_rot[MAX_ANIM_BONES];
	vec3  spring_ang_vel[MAX_ANIM_BONES];
	bool  spring_armed;

	mat4 pose[MAX_ANIM_BONES]; // composed skeleton pose, uploaded when this mesh draws

	// playback state : written by update(), read by the Anim tab
	struct {
		float elapsed, duration; // seconds (duration 0 = never played -> static frame 0)
		float frame_f;           // fractional frame head
		uint  this_frame;
		uint  seg;               // segment being played (index into clip->segs)
		bool  playing;
	} state;

	// control : written by the Anim tab, read by update() - each side only writes its fields.
	// per-segment feel (ease/juice) lives in the clip's segments, edited by the tab directly
	struct {
		float speed     = 1.f;  // playback rate
		bool  loop      = true;
		bool  paused    = false;
		bool  scrubbing = false; // the tab is dragging the timeline
		float scrub_t   = 0.f;   // 0..1 while scrubbing
		bool  restart   = false; // one-shot command from the tab
	} control;
};

// loads clips into slots (filepath cached, like MeshLoader) and runs playback :
// the clock, pose sampling, loop seam and spring juice all live here, so the
// renderer only asks for the finished pose and hands it to the gpu
struct AnimLoader
{
	Animation clips[MAX_ANIMATIONS]; // slot 0 unused : id 0 = no clip
	char clip_paths[MAX_ANIMATIONS][MAX_FILEPATH_LENGTH];

	// one player per clip in flight (play() hands out the slots)
	Anim_Player players[MAX_ANIM_PLAYERS];
	uint num_players; // slots handed out so far
	uint focus;       // which player the Anim tab edits

	float seam_duration; // loop crossfade seconds, shared by every player

	void init()
	{
		// calloc skips default member values : seed the non-zero ones here
		seam_duration = 0.15f;
		for (uint i = 0; i < MAX_ANIM_PLAYERS; i++)
		{
			players[i].control.speed = 1.f;
			players[i].control.loop  = true; // NSDMI default, skipped by calloc just like speed
		}
	}

	// load a clip into a free slot : skeleton (parents + ibm) + keyframes.
	// same path twice = same id back (cached) ; returns 0 on failure
	void load(const char* filepath, uint* id) // returns clip_id
	{
		for (uint i = 1; i < MAX_ANIMATIONS; i++) // slot 0 unused : id 0 = no clip
		{
			if (clip_paths[i][0] && strcmp(clip_paths[i], filepath) == 0) // 0 = strings match!
			{
				if (id) *id = i;
				return;
			}
		}

		// free slot + disk read
		for (uint i = 1; i < MAX_ANIMATIONS; i++) // slot 0 unused : id 0 = no clip
		{
			if (clip_paths[i][0]) continue;

			FILE* file = fopen(filepath, "rb");
			if (!file)
			{
				char msg[MAX_LOG_MSG_LENGTH];
				snprintf(msg, MAX_LOG_MSG_LENGTH, "could not open animation file : %s", filepath);
				console->add_entry(msg, FIXME, RNDR);
				return; // *id is left untouched (0) : caller checks for this
			}

			Animation& anim = clips[i];
			anim = {};

			// skeleton
			fread(&anim.num_bones, sizeof(uint), 1, file);
			fread(anim.parents   , sizeof(uint), anim.num_bones, file);
			fread(anim.ibm       , sizeof(mat4), anim.num_bones, file);

			if (anim.num_bones == 0 || anim.num_bones > MAX_ANIM_BONES)
			{
				fclose(file);
				char msg[MAX_LOG_MSG_LENGTH];
				snprintf(msg, MAX_LOG_MSG_LENGTH, "Add Anim : bad bone count [%d]", anim.num_bones);
				console->add_entry(msg, FIXME, RNDR);
				return; // slot stays free
			}

			// animation keyframes
			fread(&anim.num_frames, sizeof(uint), 1, file);
			for (uint b = 0; b < anim.num_bones; b++)
			{
				anim.keyframes[b] = Alloc(mat4, anim.num_frames);
				fread(anim.keyframes[b], sizeof(mat4), anim.num_frames, file);
			}
			fclose(file);

			// row-major file -> column-major glm, once here
			for (uint b = 0; b < anim.num_bones; b++)
			{
				for (uint f = 0; f < anim.num_frames; f++)
					transpose_in_place(anim.keyframes[b][f]);
				transpose_in_place(anim.ibm[b]);
			}

			// rotation holds : spans where no bone turns - bands on the tab's timeline.
			// rotation-only : translation drifts through the whole clip, never rests
			int run_first = -1;
			for (uint f = 0; f + 1 < anim.num_frames; f++)
			{
				bool still = true;
				for (uint b = 0; b < anim.num_bones && still; b++)
				{
					quat qa = glm::quat_cast(mat3(anim.keyframes[b][f]));
					quat qb = glm::quat_cast(mat3(anim.keyframes[b][f + 1]));
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
					if (anim.num_holds < 8)
						anim.holds[anim.num_holds++] = { (uint)run_first, f - 1 };
					run_first = -1;
				}
			}
			if (run_first >= 0 && anim.num_holds < 8)
				anim.holds[anim.num_holds++] = { (uint)run_first, anim.num_frames - 2 };

			strcpy(clip_paths[i], filepath);

			// one segment over the whole clip until the tab slices it up
			anim.segs[0] = { 0, anim.num_frames ? anim.num_frames - 1 : 0, 0, 0.4f };
			anim.num_segs = 1;

			if (id) *id = i;

			char msg[MAX_LOG_MSG_LENGTH] = {};
			snprintf(msg, MAX_LOG_MSG_LENGTH, "Add Anim, bones[%d], frames[%d], holds[%d]",
				anim.num_bones, anim.num_frames, anim.num_holds);
			console->add_entry(msg, SUCCESS, RNDR);
			return;
		}

		console->add_entry("animation clip cache full (MAX_ANIMATIONS)", FIXME, RNDR);
	}

	// start (or restart) a clip's segment : loop_duration in seconds.
	// reuses the player already on that clip, otherwise takes a free slot.
	// returns the player id ((uint)-1 = failed, error already logged)
	uint play(uint clip_id, uint seg_id, float loop_duration)
	{
		if (clip_id == 0 || clip_id >= MAX_ANIMATIONS || clip_paths[clip_id][0] == 0)
		{
			console->add_entry("Play Anim : no such clip id", FIXME, RNDR);
			return (uint)-1;
		}
		if (seg_id >= clips[clip_id].num_segs)
		{
			console->add_entry("Play Anim : no such segment", FIXME, RNDR);
			return (uint)-1;
		}

		// reuse the player already on this clip, otherwise take a free slot
		uint p = (uint)-1;
		for (uint i = 0; i < num_players; i++)
		{
			if (players[i].clip_id == clip_id) { p = i; break; }
		}
		if (p == (uint)-1)
		{
			if (num_players >= MAX_ANIM_PLAYERS)
			{
				console->add_entry("Play Anim : no free players (MAX_ANIM_PLAYERS)", FIXME, RNDR);
				return (uint)-1;
			}
			p = num_players++;
		}

		Anim_Player& pl = players[p];
		pl.clip_id      = clip_id;
		pl.seg          = seg_id;
		pl.duration     = loop_duration;
		pl.elapsed      = 0;
		pl.seam_t       = seam_duration; // no phantom blend on a fresh start
		pl.spring_armed = false;

		char msg[MAX_LOG_MSG_LENGTH];
		snprintf(msg, MAX_LOG_MSG_LENGTH, "Play Anim | player %d | seg %d | %.2fs per loop",
			p, seg_id, loop_duration);
		console->add_entry(msg, SUCCESS, RNDR);
		return p;
	}

	// per-frame playback for every player : time -> warp -> sample -> seam -> spring.
	// the renderer passes window->dtime ; dt in seconds
	void update(float dt)
	{
		if (dt > 0.05f) dt = 0.05f; // hitch guard
		for (uint i = 0; i < num_players; i++)
			update_player(&players[i], dt);
	}

	// one player's full pipeline (called by update() for each player in flight)
	void update_player(Anim_Player* pl, float dt)
	{
		Animation* clip = &clips[pl->clip_id];

		if (pl->seg >= clip->num_segs) pl->seg = clip->num_segs - 1; // the tab may have removed it
		const Anim_Segment& s = clip->segs[pl->seg];

		advance_time(pl, dt);
		if (pl->wrapped)
		{
			for (uint i = 0; i < clip->num_bones; i++)
				pl->seam_pose[i] = pl->pose[i]; // what was actually on screen at the wrap
			pl->seam_t = 0;
		}

		// clock -> warp -> this segment's frame range (holds the last frame at 1.0)
		float head = 0;
		if (pl->duration > 0)
		{
			float span = (float)(s.last - s.first + 1);
			head = s.first + time_warp(pl->elapsed / pl->duration, s.ease) * span;
			if (head > (float)s.last) head = (float)s.last;
		}

		sample_pose(clip, pl, head);

		if (pl->seam_t < seam_duration)
		{
			float k = 1.f - pl->seam_t / seam_duration; // 1 = frozen old pose, 0 = fresh clip
			for (uint i = 0; i < clip->num_bones; i++)
				pl->pose[i] = pl->pose[i] * (1.f - k) + pl->seam_pose[i] * k;
			pl->seam_t += dt;
		}

		apply_spring(clip, pl, dt);

		// mirror state for the Anim tab
		pl->state.elapsed    = pl->elapsed;
		pl->state.duration   = pl->duration;
		pl->state.frame_f    = head;
		pl->state.this_frame = (head >= (float)clip->num_frames) ? clip->num_frames - 1 : (uint)head;
		pl->state.seg        = pl->seg;
		pl->state.playing    = (pl->duration > 0 && !pl->control.paused &&
			(pl->control.loop || pl->elapsed < pl->duration));
	}

	// ---- playback internals ---------------------------------------------------

	// sample the clip at a fractional frame head into the player's pose : slerp
	// rotations + lerp offsets per bone (matrix lerp pinches on fast frames),
	// then world = parent * local, pose = world * ibm
	void sample_pose(Animation* clip, Anim_Player* pl, float head)
	{
		uint n = clip->num_frames;
		if (n == 0) return;
		if (head >= (float)n) head = (float)n - 0.001f; // one-shot end hold

		uint i0 = (uint)head;
		uint i1 = (i0 + 1 < n) ? i0 + 1 : i0;
		float t = head - (float)i0;

		mat4 local[MAX_ANIM_BONES];
		for (uint b = 0; b < clip->num_bones; b++)
		{
			const mat4& A = clip->keyframes[b][i0];
			const mat4& B = clip->keyframes[b][i1];

			quat qa = glm::quat_cast(mat3(A));
			quat qb = glm::quat_cast(mat3(B));
			if (glm::dot(qa, qb) < 0.f) qb = -qb; // shortest arc
			quat q = glm::slerp(qa, qb, t);

			vec3 p = vec3(A[3]) + (vec3(B[3]) - vec3(A[3])) * t;

			mat4 M = glm::mat4_cast(q);
			M[3] = vec4(p, 1.f);
			local[b] = M;
		}

		mat4* pose = pl->pose;
		pose[0] = local[0]; // root (parents[0] == 0 == itself)
		for (uint i = 1; i < clip->num_bones; i++)
			pose[i] = pose[clip->parents[i]] * local[i];
		for (uint i = 0; i < clip->num_bones; i++)
			pose[i] = pose[i] * clip->ibm[i];
	}

	// global rhythm warp for the whole clip : linear by default (timing is baked
	// into the file). each segment picks its own method; the tab also samples
	// every method directly to draw the graph
	float time_warp(float u, int method)
	{
		switch (method)
		{
			case 1: return u * u * (3.f - 2.f * u);                            // smooth
			case 2: return u < .5f ? 4.f * u * u * u
				: 1.f - powf(-2.f * u + 2, 3.f) / 2.f;                       // in-out cubic
			case 3: return 1.f - powf(1.f - u, 3.f);                           // out cubic
			case 4: return u < .5f ? 16.f * u * u * u * u * u
				: 1.f - powf(-2.f * u + 2, 5.f) / 2.f;                       // in-out quint
			default: return u;
		}
	}

	// the clock : speed, pause, scrub, restart, loop (raises wrapped on the seam)
	void advance_time(Anim_Player* pl, float dt)
	{
		pl->wrapped = false;

		if (pl->control.restart)
		{
			pl->control.restart = false;
			pl->elapsed = 0;
			pl->seam_t = seam_duration; // no seam on a manual jump
			pl->spring_armed = false;   // snap instead of swinging from the old spot
			return;
		}
		if (pl->control.scrubbing)
		{
			pl->elapsed = pl->control.scrub_t * pl->duration;
			pl->seam_t = seam_duration;
			pl->spring_armed = false;   // follow the dragged pose exactly, no jelly
			return;
		}
		if (pl->duration <= 0.f || pl->control.paused) return;

		pl->elapsed += dt * pl->control.speed;
		if (pl->elapsed >= pl->duration)
		{
			if (pl->control.loop)
			{
				while (pl->elapsed >= pl->duration) pl->elapsed -= pl->duration;
				pl->wrapped = true; // update_player() freezes the on-screen pose for the seam blend
			}
			else
			{
				pl->elapsed = pl->duration; // finished : hold the last frame
			}
		}
	}

	// the juice : a spring per bone chasing the sampled pose - overshoot and settle
	// out of the sparse keys. weight 0 = the clip passes through untouched
	void apply_spring(Animation* clip, Anim_Player* pl, float dt)
	{
		mat4* pose = pl->pose;
		float w = clip->segs[pl->seg].juice; // the playing segment's own juice
		if (!pl->spring_armed || w < 0.01f)
		{
			// (re)sync to the target : bypass, or a clean base when weight comes up
			for (uint i = 0; i < clip->num_bones; i++)
			{
				pl->spring_pos[i]     = vec3(pose[i][3]);
				pl->spring_rot[i]     = glm::quat_cast(mat3(pose[i]));
				pl->spring_vel[i]     = vec3(0.f);
				pl->spring_ang_vel[i] = vec3(0.f);
			}
			pl->spring_armed = true;
			return;
		}

		float k    = 60.f + 740.f * w * w; // stiffness
		float zeta = 0.85f - 0.35f * w;    // damping ratio : below 1 = bounces
		float c    = 2.f * zeta * sqrtf(k);

		for (uint i = 0; i < clip->num_bones; i++)
		{
			vec3 target_p(pose[i][3]);
			quat target_q = glm::quat_cast(mat3(pose[i]));

			// position
			vec3 dp = target_p - pl->spring_pos[i];
			pl->spring_vel[i] += (dp * k - pl->spring_vel[i] * c) * dt;
			pl->spring_pos[i] += pl->spring_vel[i] * dt;

			// rotation : error as axis-angle -> torque -> angular velocity -> integrate
			quat qe = target_q * glm::conjugate(pl->spring_rot[i]);
			if (qe.w < 0.f) qe = -qe; // shortest arc
			vec3 axis(qe.x, qe.y, qe.z);
			float alen = glm::length(axis);
			vec3 torque = (alen > 1e-5f)
				? axis * ((2.f * atan2f(alen, qe.w) / alen) * k)
				: vec3(0.f);
			pl->spring_ang_vel[i] += (torque - pl->spring_ang_vel[i] * c) * dt;
			quat wq(0.f, pl->spring_ang_vel[i].x, pl->spring_ang_vel[i].y, pl->spring_ang_vel[i].z);
			pl->spring_rot[i] = glm::normalize(pl->spring_rot[i] + 0.5f * (wq * pl->spring_rot[i]) * dt);

			pose[i] = glm::mat4_cast(pl->spring_rot[i]);
			pose[i][3] = vec4(pl->spring_pos[i], 1.f);
		}
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

	// uv sphere : rings around y, seam column duplicated for clean uvs. unit
	// sized (radius 1) so the scene derives world size from its scale alone
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

	// capsule : uv sphere caps on a cylinder. height = the cylinder part only,
	// same convention as bullet's capsule shape, so colliders match by construction.
	// rings run top to bottom, seam column duplicated like the sphere
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
			// profile latitude + the cap center it swings around : 0..stacks is the
			// top cap (phi 0..90 deg), stacks+1 is the cylinder, the rest the bottom cap
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