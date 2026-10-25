#include "player.h"

/* ------------------------- *
	-        Scenes         -
 * ------------------------- */

// what the world's guns are made of : dull steel, the mesh's texture keeps the detail
const vec3 GUN_ALBEDO = vec3(0.55f, 0.56f, 0.58f);

const uint NUM_WEAPONS = 7;  // the weapon row
const uint MAX_ACTORS  = 16; // skinned meshes animating in place

// one scene. no state outlives a switch : the manager tears this down and the
// next builder fills it from scratch. systems are passed in, not inherited
struct Scene
{
	Camera camera;    // the view the world is rendered from
	mat4   proj_view; // built from the camera every frame in update()

	Physics phys; // owns motion, the props read their pose from it

	Player player; // the capsule, the view gun and its projectiles

	// world content : a mesh plus an optional body. collider sizes are in
	// world units (mesh scale already applied) ; dynamic props get pos/rot
	// overwritten from bullet every frame
	struct Prop {
		uint mesh_id;
		uint phys_id; // 0 = static decoration
		bool no_uv;   // which static buffer this mesh lives in (mode 1 or mode 3)
		vec3 pos;
		mat3 rot;
		vec3 scale;
		vec4 mat_a;     // albedo rgb + metallic
		vec4 mat_b;     // roughness, ao, texture-albedo flag
		char label[24]; // drawn over it in the world (empty = none)
	} props[192];
	uint num_props;

	// skinned meshes that animate in place : the scene submits one instance each
	struct Actor {
		uint  mesh, player;
		vec3  pos;
		float yaw, scale;
	} actors[MAX_ACTORS];
	uint num_actors;

	bool show_labels; // debug toggle : world labels on/off

	Lighting lighting;      // the deferred pass's per-scene rig
	uint     lighting_kind; // LightingPreset this scene started from

	// one more instance in the world, posed at (pos, yaw, scale)
	Prop& add_prop(uint mesh_id, vec3 pos, float yaw, float scale)
	{
		return add_prop(mesh_id, pos, yaw, vec3(scale));
	}

	// non-uniform sizes (chassis, wheels) set every axis directly
	Prop& add_prop(uint mesh_id, vec3 pos, float yaw, vec3 scale)
	{
		// no room : say so and drop the prop, rather than writing past the array
		if (num_props >= 192)
		{
			static Prop overflow = {};
			console->add_entry("props full : raise the props[] size", FIXME, RNDR);
			overflow = {};
			return overflow;
		}

		Prop& p = props[num_props++];
		p.mesh_id = mesh_id;
		p.phys_id = 0;
		p.no_uv   = false;
		p.pos     = pos;
		p.rot     = mat3(glm::rotate(mat4(1), yaw, vec3(0, 1, 0)));
		p.scale   = scale;
		p.mat_a   = INSTANCE_DEFAULT.mat_a; // sane dielectric until a caller says otherwise
		p.mat_b   = INSTANCE_DEFAULT.mat_b;
		p.label[0] = 0;
		return p;
	}

	void add_actor(uint mesh, uint player, vec3 pos, float yaw, float scale)
	{
		if (num_actors >= MAX_ACTORS)
		{
			console->add_entry("scene actor slots full", FIXME, RNDR);
			return;
		}

		Actor& a = actors[num_actors++];
		a.mesh   = mesh;
		a.player = player;
		a.pos    = pos;
		a.yaw    = yaw;
		a.scale  = scale;
	}

	void use_lighting(uint kind)
	{
		lighting_kind = kind;
		lighting = lighting_preset(kind);
	}

	// ---- shared start : wipe the scene, place the camera and the ground
	void begin(GameRenderer* renderer, vec3 cam_pos, float yaw, float pitch,
	           float ground_size, float ground_half)
	{
		*this = {}; // no world, no props : the caller has already shut the old one down
		show_labels = true;
		use_lighting(LIGHT_DAY);

		camera.position = cam_pos;
		camera.yaw      = yaw;
		camera.pitch    = pitch;
		camera.update_dir(0, 0, 0); // seeds front/right/up from yaw/pitch

		phys.init();

		// floor : generated grid plane, the shader tiles its uvs (uv * 4)
		uint resolution = (uint)(ground_size > 1.f ? ground_size : 1.f);
		uint floor_mesh = renderer->add_mesh(
			MeshFactory::generate_plane(ground_size, resolution, MeshFactory::PLANE_XZ),
			"assets/textures/grid.bmp");
		add_prop(floor_mesh, vec3(0), 0, 1.f).phys_id = phys.add_ground(ground_half);
	}

	// ---- shared finish : keep every mesh's instances in one run, and log the load
	void finish(GameRenderer* renderer, const char* name)
	{
		// a scene without a player still gets eyes to look through
		if (!player.body) player.init(renderer, &phys, vec3(0, 5, 8), 0, 0);

		group_props();
		group_actors();

		char msg[MAX_LOG_MSG_LENGTH];
		snprintf(msg, sizeof(msg), "Scene loaded : %s", name);
		console->add_entry(msg, SUCCESS, RNDR);
	}

	// props are submitted in array order, and the draw buffer takes one run per
	// mesh : stable-sort by mesh so a shared prop can be added anywhere
	void group_props()
	{
		for (uint i = 1; i < num_props; i++)
		{
			Prop key = props[i];
			int j = (int)i - 1;
			while (j >= 0 && props[j].mesh_id > key.mesh_id)
			{
				props[j + 1] = props[j];
				j--;
			}
			props[j + 1] = key;
		}
	}

	// the gun's mesh is the view model's : its actor goes immediately after it,
	// so that mesh still has one contiguous run of instances
	void group_actors()
	{
		for (uint i = 1; i < num_actors; i++)
		{
			Actor key = actors[i];
			int j = (int)i - 1;
			while (j >= 0 && actors[j].mesh > key.mesh)
			{
				actors[j + 1] = actors[j];
				j--;
			}
			actors[j + 1] = key;
		}

		if (!player.gun_mesh) return;

		uint at = 0;
		for (uint i = 0; i < num_actors; i++)
			if (actors[i].mesh == player.gun_mesh)
			{
				Actor gun = actors[i];
				for (uint j = i; j > at; j--) actors[j] = actors[j - 1];
				actors[at++] = gun;
			}
	}

	// ---- material study : the range the deferred lighting can show, as a wall of
	// spheres. the bottom rows name the materials worth having, above them metallic
	// sweeps upward and roughness across. all sharing one mesh per layout
	void build_material_grid(GameRenderer* renderer, vec3 at)
	{
		struct Material {
			const char* name;
			vec3  albedo; // display space : the g-buffer holds sRGB
			float metallic, roughness;
		};

		const Material named[] = {
			{ "gold",      vec3(1.000f, 0.766f, 0.336f), 1.00f, 0.10f },
			{ "copper",    vec3(0.955f, 0.638f, 0.538f), 1.00f, 0.26f },
			{ "brass",     vec3(0.887f, 0.789f, 0.434f), 1.00f, 0.22f },
			{ "chrome",    vec3(0.550f, 0.556f, 0.554f), 1.00f, 0.03f },
			{ "aluminium", vec3(0.913f, 0.921f, 0.925f), 1.00f, 0.16f },
			{ "iron",      vec3(0.560f, 0.570f, 0.580f), 1.00f, 0.42f },
			{ "ceramic",   vec3(0.930f, 0.930f, 0.900f), 0.00f, 0.08f },
			{ "plastic",   vec3(0.900f, 0.100f, 0.100f), 0.00f, 0.42f },
			{ "wood",      vec3(0.580f, 0.400f, 0.220f), 0.00f, 0.62f },
			{ "rubber",    vec3(0.050f, 0.050f, 0.060f), 0.00f, 0.92f },
		};
		const uint num_named = (uint)(sizeof(named) / sizeof(named[0]));

		uint sphere_uv   = renderer->add_mesh(MeshFactory::generate_uv_sphere(1.f, 32, 20));
		uint sphere_nouv = renderer->add_mesh_no_uv(
			MeshFactory::without_uv(MeshFactory::generate_uv_sphere(1.f, 32, 20)));

		const uint  columns = 5;
		const float radius  = 0.22f;
		const float spacing = 0.75f;
		const float base_y  = at.y;
		const float wall_z  = at.z;

		for (uint wall = 0; wall < 2; wall++)
		{
			uint  mesh   = wall ? sphere_nouv : sphere_uv;
			bool  no_uv  = (wall != 0);
			float wall_x = at.x + (wall ? 2.2f : -2.2f);

			for (uint i = 0; i < num_named; i++)
			{
				uint col = i % columns, row = i / columns;

				Prop& p = add_prop(mesh, vec3(wall_x + ((float)col - (columns - 1) * 0.5f) * spacing,
				                              base_y + (float)row * spacing, wall_z), 0, radius);
				p.no_uv = no_uv;
				p.mat_a = vec4(named[i].albedo, named[i].metallic);
				p.mat_b = vec4(named[i].roughness, 1.f, 0.f, 0.f);
				snprintf(p.label, sizeof(p.label), "%s", named[i].name);
			}

			const uint named_rows = (num_named + columns - 1) / columns;
			for (uint row = 0; row < 5; row++)
				for (uint col = 0; col < columns; col++)
				{
					float metal = row / 4.f;
					float rough = 0.05f + col * 0.2375f;

					Prop& p = add_prop(mesh, vec3(wall_x + ((float)col - (columns - 1) * 0.5f) * spacing,
					                              base_y + (float)(row + named_rows) * spacing, wall_z),
					                   0, radius);
					p.no_uv = no_uv;
					p.mat_a = vec4(0.72f, 0.70f, 0.68f, metal);
					p.mat_b = vec4(rough, 1.f, 0.f, 0.f);
					snprintf(p.label, sizeof(p.label), "m%.2f r%.2f", metal, rough);
				}
		}
	}

	// teardown : the world and its collider wires. the renderer clears its own
	// buffers and caches from SceneManager
	void shutdown(GameRenderer*)
	{
		phys.shutdown();
		num_props = 0;
		num_actors = 0;
	}

	// called once per frame : input -> camera -> matrices, then submit this frame's instances
	void update(GameWindow* window, GameRenderer* renderer)
	{
		if (!phys.world) return;

		if (player.flycam) update_fly_camera(&camera, window, window->dtime);
		player.update(window, renderer, &phys);

		// the player's camera while walking, the free camera while flying
		Camera& view = player.flycam ? camera : player.camera;

		float fov = (60.f + player.fov_punch) * PI / 180.f;
		mat4 proj = glm::perspective(fov, (float)window->screen_width / window->screen_height, 0.1f, 256.f);
		proj_view = proj * glm::lookAt(view.position, view.position + view.front, view.up);

		phys.update_vehicle(window);
		phys.step(window->dtime);

		// fresh instance streams : each mesh id may only be appended once per frame
		renderer->clear_instances();

		// props : the body owns the pose of anything dynamic
		for (uint i = 0; i < num_props; i++)
		{
			Prop& p = props[i];

			// a prop with no mesh has nothing to submit
			if (p.mesh_id == 0) continue;

			if (p.phys_id)
				phys.get_transform(p.phys_id, &p.pos, &p.rot);

			Instance inst = { glm::translate(mat4(1), p.pos) * mat4(p.rot) * glm::scale(mat4(1), p.scale),
			                  p.mat_a, p.mat_b };

			// a prop says which static layout its mesh was built for (mode 1 or mode 3)
			renderer->buffer(p.no_uv ? MODE_STATIC_NOUV : MODE_STATIC_UV).append_instances(p.mesh_id, 1, &inst);
		}

		// view gun : appended just before the actors, so the shared gun mesh
		// keeps one contiguous run of instances
		if (!player.flycam && player.gun_mesh)
		{
			Camera& c = player.camera;

			// view -> world : the columns are the camera's own axes
			mat3 aim;
			aim[0] = c.right;
			aim[1] = c.up;
			aim[2] = -c.front;

			// the clip is authored with the camera at the origin, so it does the placing : the
			// eye's frame plus the juice is the whole view model transform
			vec3 place = player.kick * WEAPON_KICK;
			mat4 model = glm::translate(mat4(1), c.position + aim * place)
			           * mat4(aim)
			           * glm::scale(mat4(1), vec3(WEAPON_SCALE))
			           * glm::rotate(mat4(1), player.sway.x, vec3(0, 1, 0))
			           * glm::rotate(mat4(1), player.sway.y, vec3(1, 0, 0))
			           * glm::rotate(mat4(1), player.kick.x * WEAPON_ROLL, vec3(0, 0, 1)); // shot roll
			Instance inst = material(GUN_ALBEDO, 0.85f, 0.30f, 1.f, 1.f); // gunmetal, texture keeps the detail
			inst.model = model;
			renderer->buffer(MODE_ANIM_UV).append_instances(player.gun_mesh, 1, &inst);
		}

		// actors : parked where the builder put them, still animating
		for (uint i = 0; i < num_actors; i++)
		{
			Actor& a = actors[i];
			mat4 model = glm::translate(mat4(1), a.pos)
			      * glm::rotate(mat4(1), a.yaw, vec3(0, 1, 0))
			      * glm::scale(mat4(1), vec3(a.scale));
			Instance inst = material(GUN_ALBEDO, 0.85f, 0.30f, 1.f, 1.f); // same steel as the viewmodel
			inst.model = model;
			renderer->buffer(MODE_ANIM_UV).append_instances(a.mesh, 1, &inst);
		}

		// bullets : one contiguous block of instances
		Instance bullet_inst[MAX_BULLETS];
		uint num_bullets = 0;
		for (uint i = 0; i < MAX_BULLETS; i++)
		{
			if (player.bullets[i].life <= 0) continue;
			bullet_inst[num_bullets] = material(vec3(1.f, 0.86f, 0.42f), 0.f, 0.4f); // hot tracer, flat colour
			bullet_inst[num_bullets].model = glm::translate(mat4(1), player.bullets[i].pos);
			num_bullets++;
		}
		if (num_bullets)
			renderer->buffer(MODE_STATIC_UV).append_instances(player.bullet_mesh, num_bullets, bullet_inst);
	}
};

// the test pistol's four action frames : reload, shoot, inspect, idle
void slice_testpistol(Animation* clip)
{
	clip->segs[0] = {  0, 85, 0, 0.40f }; // reload
	clip->segs[1] = {  0,  0, 0, 0.40f }; // shoot : no take yet
	clip->segs[2] = {  0,  0, 0, 0.40f }; // inspect : no take yet
	clip->segs[3] = {  0,  0, 0, 0.40f }; // idle : no take yet
	clip->num_segs = 4;
}

// ---- hub : the quiet default. a place to stand, not a demo of everything
void scene_hub(Scene* s, GameRenderer* renderer)
{
	s->begin(renderer, vec3(0, 2.6f, 10), -PI / 2, -0.13f, 60.f, 30.f);

	// two palms, a thin trunk slab so nothing shoves them
	uint palm_mesh = renderer->add_mesh("assets/meshes/palm_tree.mesh_uv");
	vec3 palm_spots[2] = { vec3(-13, 0, -2), vec3(-10, 0, -5) };
	for (uint i = 0; i < 2; i++)
	{
		Scene::Prop& palm = s->add_prop(palm_mesh, palm_spots[i], (float)i * 2.1f, 1.6f);
		palm.phys_id = s->phys.add_box(palm.pos + vec3(0, 3, 0), vec3(.2f, 3.f, .2f), 0);
	}

	// a small crate stack and a loose ammo box : enough to bump into
	uint cube_mesh = renderer->add_mesh(MeshFactory::generate_cube(1.f), "assets/textures/checker.bmp");
	vec3 crates[3] = { vec3(5, .5f, 2), vec3(6, .5f, 2), vec3(5.5f, 1.5f, 2) };
	for (uint i = 0; i < 3; i++)
	{
		Scene::Prop& box = s->add_prop(cube_mesh, crates[i], 0, 1.f);
		box.phys_id = s->phys.add_box(box.pos, vec3(.5f), 1.f);
	}

	uint ammo_mesh = renderer->add_mesh("assets/meshes/ammo.mesh_uv");
	Scene::Prop& ammo = s->add_prop(ammo_mesh, vec3(2, 2.5f, 5), 0.3f, 2.f);
	ammo.phys_id = s->phys.add_box(ammo.pos, vec3(.15f, .1f, .15f), .5f);

	s->player.init(renderer, &s->phys, vec3(0, 5, 8), 0, 0);
	s->finish(renderer, "hub");
}

// ---- weapons : the row, a range, and knockable targets
void scene_weapons(Scene* s, GameRenderer* renderer)
{
	s->begin(renderer, vec3(0, 2.6f, 10), -PI / 2, -0.13f, 60.f, 30.f);
	s->use_lighting(LIGHT_OVERCAST);

	// the row sits along the left wall, facing into the range
	struct { const char* mesh, *clip; uint seg; float dur; } setup[NUM_WEAPONS] = {
		{ "assets/anim/glock.mesh_anim",       "assets/anim/glock.anim",       0, 2.0f },
		{ "assets/anim/ak.mesh_anim",          "assets/anim/ak.anim",          0, 2.6f },
		{ "assets/anim/m4.mesh_anim",          "assets/anim/m4.anim",          1, 2.3f },
		{ "assets/anim/shotgun.mesh_anim",     "assets/anim/shotgun.anim",     1, 3.2f },
		{ "assets/anim/pistol_colt.mesh_anim", "assets/anim/pistol_colt.anim", 1, 1.6f },
		{ "assets/anim/bolt_action.mesh_anim", "assets/anim/bolt_action.anim", 0, 1.4f },
		{ "assets/anim/testpistol.mesh_anim",  "assets/anim/testpistol.anim",  0, 3.6f },
	};

	struct Weapon { uint mesh, clip, player; vec3 pos; float scale, yaw; } weapons[NUM_WEAPONS] = {};

	for (uint i = 0; i < NUM_WEAPONS; i++)
	{
		Weapon& w = weapons[i];
		w.mesh = renderer->add_anim_mesh(setup[i].mesh);
		renderer->animloader.load(setup[i].clip, &w.clip);
		if (!w.mesh || !w.clip) continue;

		// frame ranges per weapon + a different feel on every segment
		Animation* clip = &renderer->animloader.clips[w.clip];
		switch (i)
		{
			case 0: // glock : the classic frame map, felt rather than hurried
				clip->segs[0] = {   0,  63, 2, 0.85f }; // reload  : slow pull, hard slam, spring
				clip->segs[1] = {  65,  72, 3, 1.00f }; // shoot   : snaps out, big overshoot
				clip->segs[2] = {  74, 104, 3, 0.60f }; // inspect : settles with jiggle
				clip->segs[3] = { 104, 104, 0, 0.40f }; // idle pose
				clip->num_segs = 4;
				break;
			case 1: // ak : one continuous action cycle
				clip->segs[0] = { 0, 88, 2, 0.35f }; // full cycle
				clip->segs[1] = { 88, 88, 0, 0.40f }; // idle pose
				clip->num_segs = 2;
				break;
			case 2: // m4
				clip->segs[0] = {   0,  50, 1, 0.30f }; // inspect
				clip->segs[1] = {  31, 100, 3, 0.40f }; // reload
				clip->segs[2] = { 100, 107, 2, 0.60f }; // shoot
				clip->segs[3] = { 103, 103, 0, 0.40f }; // idle pose
				clip->num_segs = 4;
				break;
			case 3: // shotgun
				clip->segs[0] = {  0,  16, 2, 0.60f }; // shoot
				clip->segs[1] = { 17,  77, 3, 0.40f }; // reload
				clip->segs[2] = { 18,  78, 1, 0.30f }; // inspect
				clip->segs[3] = { 77,  77, 0, 0.40f }; // idle pose
				clip->num_segs = 4;
				break;
			case 4: // colt : cut at its rotation rests
				clip->segs[0] = {  0,  2, 0, 0.40f };
				clip->segs[1] = {  3, 28, 3, 0.45f };
				clip->segs[2] = { 29, 33, 2, 0.35f };
				clip->segs[3] = { 34, 46, 1, 0.30f };
				clip->num_segs = 4;
				break;
			case 5: // bolt action : one working cycle, no rotation rest to cut at,
			        // so linear time and let the motion's own ease do the work
				clip->segs[0] = {  0, 33, 0, 0.35f }; // full cycle : settle, work the action, settle
				clip->segs[1] = {  8, 20, 0, 0.60f }; // the action : unlock, back, eject, home, lock
				clip->segs[2] = { 33, 33, 0, 0.40f }; // idle pose (translation-identical to frame 0)
				clip->num_segs = 3;
				break;
			case 6: // testpistol : a reload take and nothing else yet
				slice_testpistol(clip);
				break;
		}

		w.player = renderer->animloader.play(w.clip, setup[i].seg, setup[i].dur);
		if (w.mesh) renderer->bind_player(w.mesh, w.player);

		// row runs along z on the left wall, raised near eye level
		w.pos = vec3(-14, 1.6f, (i - (NUM_WEAPONS - 1) * 0.5f) * 4.f);
		w.scale = 4.f;
		w.yaw = PI / 2 + ((i % 2) ? 0.17f : -0.17f);

		s->add_actor(w.mesh, w.player, w.pos, w.yaw, w.scale);
	}

	// range : three targets at 10 / 20 / 30 m from the spawn, marked, plus a backstop
	uint target_mesh = renderer->add_mesh(MeshFactory::generate_cube(1.f), "assets/textures/checker.bmp");
	float target_x[3] = { -5, 0, 5 };
	float target_z[3] = { -2, -12, -22 };
	for (uint i = 0; i < 3; i++)
	{
		Scene::Prop& target = s->add_prop(target_mesh, vec3(target_x[i], .75f, target_z[i]), 0,
		                                  vec3(1.4f, 1.5f, .3f));
		target.phys_id = s->phys.add_box(target.pos, vec3(.7f, .75f, .15f), 1.f);
	}

	uint marker_mesh = renderer->add_mesh(MeshFactory::generate_cube(1.f));
	for (uint i = 0; i < 3; i++)
	{
		Scene::Prop& marker = s->add_prop(marker_mesh, vec3(-9, 1.5f, target_z[i]), 0, vec3(.15f, 3.f, .15f));
		marker.phys_id = s->phys.add_box(marker.pos, vec3(.075f, 1.5f, .075f), 0);
		snprintf(marker.label, sizeof(marker.label), "%d m", 10 * (int)(i + 1));
	}

	Scene::Prop& backstop = s->add_prop(target_mesh, vec3(0, 5, -29), 0, vec3(36.f, 10.f, .5f));
	backstop.phys_id = s->phys.add_box(backstop.pos, vec3(18.f, 5.f, .25f), 0);

	// ammo crate near the firing line
	uint ammo_mesh = renderer->add_mesh("assets/meshes/ammo.mesh_uv");
	Scene::Prop& ammo = s->add_prop(ammo_mesh, vec3(2, 2.5f, 5), 0.3f, 2.f);
	ammo.phys_id = s->phys.add_box(ammo.pos, vec3(.15f, .1f, .15f), .5f);

	// player : the row's test pistol is also the view model
	s->player.init(renderer, &s->phys, vec3(0, 5, 8), weapons[6].mesh, weapons[6].clip);

	s->finish(renderer, "weapons");
}

// ---- materials : the study walls and a few large reference spheres
void scene_materials(Scene* s, GameRenderer* renderer)
{
	s->begin(renderer, vec3(0, 2.6f, 10), -PI / 2, -0.13f, 40.f, 20.f);
	s->use_lighting(LIGHT_STUDIO);

	s->build_material_grid(renderer, vec3(0, 0.9f, -9.f));

	// bigger spheres in front of the wall, so a material can be read at a glance
	struct Ref { const char* name; vec3 albedo; float metallic, roughness; };
	const Ref refs[] = {
		{ "gold",   vec3(1.000f, 0.766f, 0.336f), 1.f, 0.10f },
		{ "chrome", vec3(0.550f, 0.556f, 0.554f), 1.f, 0.03f },
		{ "wood",   vec3(0.580f, 0.400f, 0.220f), 0.f, 0.62f },
		{ "rubber", vec3(0.050f, 0.050f, 0.060f), 0.f, 0.92f },
		{ "plastic", vec3(0.900f, 0.100f, 0.100f), 0.f, 0.42f },
	};

	uint sphere_mesh = renderer->add_mesh(MeshFactory::generate_uv_sphere(1.f, 32, 20));
	for (uint i = 0; i < 5; i++)
	{
		Scene::Prop& p = s->add_prop(sphere_mesh, vec3(-4.f + i * 2.f, 1.4f, -4.f), 0, .6f);
		p.mat_a = vec4(refs[i].albedo, refs[i].metallic);
		p.mat_b = vec4(refs[i].roughness, 1.f, 0.f, 0.f);
		snprintf(p.label, sizeof(p.label), "%s", refs[i].name);
	}

	s->player.init(renderer, &s->phys, vec3(0, 5, 8), 0, 0);
	s->finish(renderer, "materials");
}

// ---- physics : colliders, a stack, dominoes and the ragdoll
void scene_physics(Scene* s, GameRenderer* renderer)
{
	s->begin(renderer, vec3(0, 11, 24), -PI / 2, -0.28f, 80.f, 40.f);
	s->use_lighting(LIGHT_NOON);

	// one cube mesh carries the platform, dominoes and stack
	uint cube_mesh = renderer->add_mesh(MeshFactory::generate_cube(1.f), "assets/textures/checker.bmp");

	// platform : the ragdoll starts up here and falls off the edge
	Scene::Prop& platform = s->add_prop(cube_mesh, vec3(-11, .5f, 0), 0, vec3(7.f, 1.f, 7.f));
	platform.phys_id = s->phys.add_box(platform.pos, vec3(3.5f, .5f, 3.5f), 0);

	// dominoes : a body knocks the row over
	for (uint i = 0; i < 6; i++)
	{
		Scene::Prop& domino = s->add_prop(cube_mesh, vec3(-3.f + i * .8f, .9f, 6.f), 0,
		                                 vec3(.12f, 1.8f, 1.f));
		domino.phys_id = s->phys.add_box(domino.pos, vec3(.06f, .9f, .5f), .4f);
	}

	// cube stack : 3-2-1, rows at resting heights
	vec3 stack[6] = {
		vec3(-1.5f, .5f, 0), vec3(0, .5f, 0), vec3(1.5f, .5f, 0),
		vec3(-.75f, 1.5f, 0), vec3(.75f, 1.5f, 0),
		vec3(0, 2.5f, 0),
	};
	for (uint i = 0; i < 6; i++)
	{
		Scene::Prop& box = s->add_prop(cube_mesh, stack[i], 0, 1.f);
		box.phys_id = s->phys.add_box(box.pos, vec3(.5f), 1.f);
	}

	// spheres : drop in from staggered heights and roll away
	uint sphere_mesh = renderer->add_mesh(MeshFactory::generate_uv_sphere(1.f, 24, 16));
	vec3 sphere_spots[3] = { vec3(4, 8, -2), vec3(6, 11, -4), vec3(8, 14, -6) };
	for (uint i = 0; i < 3; i++)
	{
		Scene::Prop& sphere = s->add_prop(sphere_mesh, sphere_spots[i], 0, 1.f);
		sphere.phys_id = s->phys.add_sphere(sphere.pos, 1.f, 1.f);
	}

	// capsules : the leaning pair topples into each other
	uint capsule_mesh = renderer->add_mesh(MeshFactory::generate_capsule(.5f, 1.4f, 20, 10));
	vec3 capsule_spots[2] = { vec3(4, 4.5f, -8), vec3(5.2f, 7.f, -8.8f) };
	for (uint i = 0; i < 2; i++)
	{
		Scene::Prop& capsule = s->add_prop(capsule_mesh, capsule_spots[i], 0, 1.f);
		capsule.phys_id = s->phys.add_capsule(capsule.pos, .5f, 1.4f, 2.f);
	}

	// cylinders : stand on their base, matched to the collider's half extents
	uint cylinder_mesh = renderer->add_mesh(MeshFactory::generate_cylinder(.6f, 2.f, 24));
	vec3 cylinder_spots[3] = { vec3(-6, 1.f, -7), vec3(-7.5f, 1.f, -8), vec3(-9, 1.f, -9) };
	for (uint i = 0; i < 3; i++)
	{
		Scene::Prop& cylinder = s->add_prop(cylinder_mesh, cylinder_spots[i], 0, 1.f);
		cylinder.phys_id = s->phys.add_cylinder(cylinder.pos, vec3(.6f, 1.f, .6f), 1.f);
	}

	// cones : base down, y aligned like the mesh
	uint cone_mesh = renderer->add_mesh(MeshFactory::generate_cone(.6f, 1.6f, 24));
	vec3 cone_spots[3] = { vec3(2, 3.f, -6), vec3(3.2f, 5.f, -7), vec3(1, 7.f, -7.5f) };
	for (uint i = 0; i < 3; i++)
	{
		Scene::Prop& cone = s->add_prop(cone_mesh, cone_spots[i], 0, 1.f);
		cone.phys_id = s->phys.add_cone(cone.pos, .6f, 1.6f, 1.f);
	}

	// ragdoll : linked capsules on the platform
	uint ragdoll = s->phys.add_ragdoll(vec3(-11, 1.02f, 0), 1.6f);
	for (uint i = 0; ragdoll && i < 11; i++)
	{
		uint id = ragdoll + i;
		vec3 d = s->phys.bodies[id].dims; // (radius, cylinder height)
		Scene::Prop& part = s->add_prop(renderer->add_mesh(MeshFactory::generate_capsule(d.x, d.y, 20, 10)),
		                                vec3(0), 0, 1.f);
		part.phys_id = id;
	}

	s->player.init(renderer, &s->phys, vec3(0, 5, 8), 0, 0);
	s->finish(renderer, "physics");
}

// ---- vehicles : the car, two ramps and things to hit
void scene_vehicles(Scene* s, GameRenderer* renderer)
{
	s->begin(renderer, vec3(0, 12, 34), -PI / 2, -0.30f, 120.f, 60.f);
	s->use_lighting(LIGHT_SUNSET);

	uint cube_mesh = renderer->add_mesh(MeshFactory::generate_cube(1.f), "assets/textures/checker.bmp");

	// two ramps, facing opposite ways, on a long straight
	const float tilt = 0.18f;
	Scene::Prop& ramp = s->add_prop(cube_mesh, vec3(0, .9f, 12), 0, vec3(10.f, .4f, 12.f));
	ramp.rot = mat3(glm::rotate(mat4(1), tilt, vec3(1, 0, 0)));
	ramp.phys_id = s->phys.add_box(ramp.pos, vec3(5.f, .2f, 6.f), 0, vec3(tilt, 0, 0));

	Scene::Prop& return_ramp = s->add_prop(cube_mesh, vec3(-20, .9f, -15), 0, vec3(8.f, .4f, 10.f));
	return_ramp.rot = mat3(glm::rotate(mat4(1), -tilt, vec3(1, 0, 0)));
	return_ramp.phys_id = s->phys.add_box(return_ramp.pos, vec3(4.f, .2f, 5.f), 0, vec3(-tilt, 0, 0));

	// obstacles : crates to push, cones to scatter
	vec3 crates[3] = { vec3(-6, .5f, -18), vec3(0, .5f, -20), vec3(6, .5f, -18) };
	for (uint i = 0; i < 3; i++)
	{
		Scene::Prop& crate = s->add_prop(cube_mesh, crates[i], 0, 1.f);
		crate.phys_id = s->phys.add_box(crate.pos, vec3(.5f), 1.f);
	}

	uint cone_mesh = renderer->add_mesh(MeshFactory::generate_cone(.5f, 1.5f, 24));
	for (uint i = 0; i < 5; i++)
	{
		Scene::Prop& cone = s->add_prop(cone_mesh, vec3(-4.f + i * 2.f, .75f, -26), 0, 1.f);
		cone.phys_id = s->phys.add_cone(cone.pos, .5f, 1.5f, .5f);
	}

	// the car
	if (s->phys.add_vehicle(vec3(0, 1.5f, 0)))
	{
		uint chassis_mesh = renderer->add_mesh(MeshFactory::generate_cube(1.f));
		Scene::Prop& chassis = s->add_prop(chassis_mesh, vec3(0), 0, vec3(2.f, 1.f, 4.f));
		chassis.phys_id = s->phys.vehicle.chassis_id;

		uint wheel_mesh = renderer->add_mesh(MeshFactory::generate_cylinder(.5f, .4f, 24));
		for (uint i = 0; i < 4; i++)
		{
			Scene::Prop& wheel = s->add_prop(wheel_mesh, vec3(0), 0, 1.f);
			wheel.phys_id = s->phys.vehicle.wheel_ids[i];
		}
	}

	s->player.init(renderer, &s->phys, vec3(0, 5, 24), 0, 0);
	s->finish(renderer, "vehicles");
}

// ---------------------------------------------------------------------------
// HUD helpers : scene UX draws on the background list, over the world and
// under the console panel. all sizes scale from the 1080p design base.
// ---------------------------------------------------------------------------

ImFont* hud_font_of(float size)
{
	if (size >= 20.f && hud_font_big) return hud_font_big;
	if (hud_font_small) return hud_font_small;
	return ImGui::GetFont();
}

void hud_text(ImDrawList* dl, float size, ImVec2 at, ImU32 color, const char* text)
{
	ImFont* font = hud_font_of(size);
	dl->AddText(font, size, ImVec2(at.x + 1.5f, at.y + 1.5f), IM_COL32(0, 0, 0, 170), text);
	dl->AddText(font, size, at, color, text);
}

void hud_text_right(ImDrawList* dl, float size, float right, float y, ImU32 color, const char* text)
{
	ImVec2 extent = hud_font_of(size)->CalcTextSizeA(size, 9999.f, 0.f, text);
	hud_text(dl, size, ImVec2(right - extent.x, y), color, text);
}

void hud_text_center(ImDrawList* dl, float size, float center_x, float y, ImU32 color, const char* text)
{
	ImVec2 extent = hud_font_of(size)->CalcTextSizeA(size, 9999.f, 0.f, text);
	hud_text(dl, size, ImVec2(center_x - extent.x * .5f, y), color, text);
}

void hud_crosshair(ImDrawList* dl, float cx, float cy, float scale, ImU32 color)
{
	const float gap = 6.f * scale, len = 9.f * scale;
	const float thick = 1.5f * scale;
	const ImU32 shadow = IM_COL32(0, 0, 0, 170);

	ImVec2 ticks[4][2] = {
		{ ImVec2(cx - gap - len, cy), ImVec2(cx - gap, cy) },
		{ ImVec2(cx + gap, cy),       ImVec2(cx + gap + len, cy) },
		{ ImVec2(cx, cy - gap - len), ImVec2(cx, cy - gap) },
		{ ImVec2(cx, cy + gap),       ImVec2(cx, cy + gap + len) },
	};

	for (uint i = 0; i < 4; i++) dl->AddLine(ticks[i][0], ticks[i][1], shadow, thick + 1.5f * scale);
	for (uint i = 0; i < 4; i++) dl->AddLine(ticks[i][0], ticks[i][1], color, thick);
	dl->AddCircleFilled(ImVec2(cx, cy), 1.5f * scale, color);
}

// ---- hub : title and the one control worth knowing
void scene_hub_ux(Scene*, GameWindow* window)
{
	ImDrawList* dl = ImGui::GetBackgroundDrawList();
	float scale = window->screen_height / 1080.f;
	float cx = window->screen_width * .5f;

	ImU32 title = IM_COL32(235, 238, 243, 225);
	ImU32 dim   = IM_COL32(170, 178, 190, 200);

	hud_text_center(dl, 26.f * scale, cx, 36.f * scale, title, "WAR EFFORT");
	hud_text_center(dl, 14.f * scale, cx, 70.f * scale, dim, "hub");
	hud_text_center(dl, 14.f * scale, cx, window->screen_height - 52.f * scale, dim,
	                "T console   |   F fly");
}

// ---- weapons : crosshair, ammo, reload and hit feedback
void scene_weapons_ux(Scene* s, GameWindow* window)
{
	ImDrawList* dl = ImGui::GetBackgroundDrawList();
	ImGuiIO& io = ImGui::GetIO();

	float scale = window->screen_height / 1080.f;
	float w = (float)window->screen_width;
	float h = (float)window->screen_height;
	float cx = w * .5f, cy = h * .5f;

	ImU32 white = IM_COL32(235, 238, 243, 235);
	ImU32 dim   = IM_COL32(170, 178, 190, 200);
	ImU32 amber = IM_COL32(230, 184, 112, 235);
	ImU32 red   = IM_COL32(220, 90, 80, 235);

	bool reloading = (s->player.action == GUN_RELOAD);
	hud_crosshair(dl, cx, cy, scale, s->player.ammo == 0 ? red : (reloading ? amber : white));

	// ammo : bottom right
	char ammo[32];
	snprintf(ammo, sizeof(ammo), "%u / %u", s->player.ammo, GUN_MAG);

	float size = 34.f * scale;
	ImVec2 extent = hud_font_of(size)->CalcTextSizeA(size, 9999.f, 0.f, ammo);
	float right = w - 48.f * scale;
	float y = h - extent.y - 38.f * scale;

	hud_text_right(dl, size, right, y, s->player.ammo ? white : red, ammo);
	hud_text_right(dl, 15.f * scale, right, y - 24.f * scale, dim, "TEST PISTOL");

	char stats[64];
	snprintf(stats, sizeof(stats), "HITS %u   FIRED %u", s->player.hits, s->player.fired);
	hud_text_right(dl, 14.f * scale, right, y + extent.y + 5.f * scale, dim, stats);

	// reload : a thin bar under the crosshair
	if (reloading)
	{
		float t = 1.f - s->player.action_time / GUN_RELOAD_TIME;
		if (t < 0.f) t = 0.f;
		if (t > 1.f) t = 1.f;

		float bar_w = 220.f * scale, bar_h = 4.f * scale;
		float x = cx - bar_w * .5f;
		float bar_y = cy + 40.f * scale;

		dl->AddRectFilled(ImVec2(x, bar_y), ImVec2(x + bar_w, bar_y + bar_h), IM_COL32(0, 0, 0, 150));
		dl->AddRectFilled(ImVec2(x, bar_y), ImVec2(x + bar_w * t, bar_y + bar_h), amber);
		hud_text_center(dl, 14.f * scale, cx, bar_y + 8.f * scale, amber, "RELOADING");
	}

	// hit marker : a short cross when a shot finds a solid body
	static uint  last_hits = 0;
	static float hit_time = 0.f;

	if (s->player.hits > last_hits) hit_time = .18f;
	last_hits = s->player.hits;
	hit_time -= io.DeltaTime;

	if (hit_time > 0.f)
	{
		float gap = 5.f * scale, len = 9.f * scale;
		ImU32 hit = IM_COL32(255, 236, 190, 220);

		dl->AddLine(ImVec2(cx - gap - len, cy - gap - len), ImVec2(cx - gap, cy - gap), hit, 2.f * scale);
		dl->AddLine(ImVec2(cx + gap, cy - gap), ImVec2(cx + gap + len, cy - gap - len), hit, 2.f * scale);
		dl->AddLine(ImVec2(cx - gap - len, cy + gap + len), ImVec2(cx - gap, cy + gap), hit, 2.f * scale);
		dl->AddLine(ImVec2(cx + gap, cy + gap), ImVec2(cx + gap + len, cy + gap + len), hit, 2.f * scale);
	}

	hud_text_center(dl, 14.f * scale, cx, h - 52.f * scale, dim,
	                "LMB fire   |   R reload   |   N inspect   |   T console");
}

// ---- materials : title, current rig, and the walls do the talking
void scene_materials_ux(Scene* s, GameWindow* window)
{
	ImDrawList* dl = ImGui::GetBackgroundDrawList();
	float scale = window->screen_height / 1080.f;
	float cx = window->screen_width * .5f;

	ImU32 title = IM_COL32(235, 238, 243, 225);
	ImU32 dim   = IM_COL32(170, 178, 190, 200);

	const char* preset = (s->lighting_kind < LIGHT_PRESET_COUNT)
		? LIGHTING_PRESET_NAMES[s->lighting_kind] : "-";

	hud_text_center(dl, 24.f * scale, cx, 36.f * scale, title, "MATERIALS");
	hud_text_center(dl, 14.f * scale, cx, 70.f * scale, dim, preset);
	hud_text_center(dl, 14.f * scale, cx, window->screen_height - 52.f * scale, dim,
	                "F fly to inspect   |   T console");
}

// ---- physics : title and the body count
void scene_physics_ux(Scene* s, GameWindow* window)
{
	ImDrawList* dl = ImGui::GetBackgroundDrawList();
	float scale = window->screen_height / 1080.f;
	float cx = window->screen_width * .5f;

	ImU32 title = IM_COL32(235, 238, 243, 225);
	ImU32 dim   = IM_COL32(170, 178, 190, 200);

	char bodies[48];
	snprintf(bodies, sizeof(bodies), "%u bodies", s->phys.num_bodies ? s->phys.num_bodies - 1 : 0);

	hud_text_center(dl, 24.f * scale, cx, 36.f * scale, title, "PHYSICS");
	hud_text_center(dl, 14.f * scale, cx, 70.f * scale, dim, bodies);
}

// ---- vehicles : speed readout
void scene_vehicles_ux(Scene* s, GameWindow* window)
{
	ImDrawList* dl = ImGui::GetBackgroundDrawList();
	float scale = window->screen_height / 1080.f;
	float cx = window->screen_width * .5f;
	float w = (float)window->screen_width;
	float h = (float)window->screen_height;

	ImU32 white = IM_COL32(235, 238, 243, 235);
	ImU32 dim   = IM_COL32(170, 178, 190, 200);

	char speed[32];
	snprintf(speed, sizeof(speed), "%+.0f km/h", s->phys.vehicle_speed() * 3.6f);

	hud_text_center(dl, 24.f * scale, cx, 36.f * scale, white, "VEHICLES");
	hud_text_right(dl, 30.f * scale, w - 48.f * scale, h - 96.f * scale, white, speed);
	hud_text_right(dl, 14.f * scale, w - 48.f * scale, h - 52.f * scale, dim,
	               "arrows drive   |   F fly");
}

// ---------------------------------------------------------------------------
// Scene debug : drawn inside the Scenes tab, for the active scene
// ---------------------------------------------------------------------------

void scene_labels_checkbox(Scene* s)
{
	ImGui::Checkbox("world labels", &s->show_labels);
}

void scene_hub_debug(Scene* s)
{
	if (ImGui::Button("reset props")) s->phys.reset_all();
	ImGui::SameLine();
	scene_labels_checkbox(s);
}

void scene_weapons_debug(Scene* s)
{
	ImGui::Text("%u / %u rounds   %u fired   %u hits",
	            s->player.ammo, GUN_MAG, s->player.fired, s->player.hits);

	if (ImGui::Button("reset range")) s->phys.reset_all();
	ImGui::SameLine();
	if (ImGui::Button("clear bullets"))
		for (uint i = 0; i < MAX_BULLETS; i++) s->player.bullets[i].life = 0;
	ImGui::SameLine();
	scene_labels_checkbox(s);
}

void scene_materials_debug(Scene* s)
{
	int preset = (int)s->lighting_kind;
	if (ImGui::Combo("lighting preset", &preset, LIGHTING_PRESET_NAMES, LIGHT_PRESET_COUNT))
		s->use_lighting((uint)preset);

	if (ImGui::Button("reset lighting")) s->lighting = lighting_preset(s->lighting_kind);
	ImGui::SameLine();
	scene_labels_checkbox(s);

	if (ImGui::CollapsingHeader("lighting", ImGuiTreeNodeFlags_DefaultOpen))
	{
		ImGui::ColorEdit3("sun color", &s->lighting.sun_color.x);
		ImGui::SliderFloat("sun power", &s->lighting.sun_power, 0.f, 8.f);
		ImGui::SliderFloat("sun disk", &s->lighting.sun_disk, 0.f, 2.f);
		ImGui::DragFloat3("sun direction", &s->lighting.sun_direction.x, .01f, -1.f, 1.f);

		ImGui::ColorEdit3("rim color", &s->lighting.rim_color.x);
		ImGui::SliderFloat("rim power", &s->lighting.rim_power, 0.f, 3.f);
		ImGui::DragFloat3("rim direction", &s->lighting.rim_direction.x, .01f, -1.f, 1.f);

		ImGui::SliderFloat("exposure", &s->lighting.exposure, .2f, 3.f);
		ImGui::SliderFloat("fog", &s->lighting.fog_density, 0.f, .03f, "%.4f");
		ImGui::SliderFloat("ambient occlusion", &s->lighting.ao_strength, 0.f, 3.f);
		ImGui::SliderFloat("multiscatter", &s->lighting.multiscatter, 0.f, 1.f);

		if (ImGui::CollapsingHeader("sky"))
		{
			ImGui::ColorEdit3("zenith", &s->lighting.zenith.x);
			ImGui::ColorEdit3("haze", &s->lighting.haze.x);
			ImGui::ColorEdit3("glow", &s->lighting.glow.x);
			ImGui::ColorEdit3("ground", &s->lighting.ground.x);
			ImGui::ColorEdit3("ambient up", &s->lighting.ambient_up.x);
			ImGui::ColorEdit3("ambient side", &s->lighting.ambient_side.x);
			ImGui::ColorEdit3("ambient down", &s->lighting.ambient_down.x);
		}
	}
}

void scene_physics_debug(Scene* s)
{
	if (ImGui::Button("wake all")) s->phys.wake_all();
	ImGui::SameLine();
	if (ImGui::Button("reset all")) s->phys.reset_all();
	ImGui::SameLine();
	scene_labels_checkbox(s);
}

void scene_vehicles_debug(Scene* s)
{
	ImGui::Text("%.0f km/h   steer %+.0f deg",
	            s->phys.vehicle_speed() * 3.6f, s->phys.vehicle_steer() * 180.f / PI);

	if (ImGui::Button("reset car"))
	{
		if (s->phys.vehicle.chassis_id) s->phys.reset(s->phys.vehicle.chassis_id);
		for (uint i = 0; i < 4; i++)
			if (s->phys.vehicle.wheel_ids[i]) s->phys.reset(s->phys.vehicle.wheel_ids[i]);
	}
	ImGui::SameLine();
	scene_labels_checkbox(s);
}

enum SceneKind {
	SCENE_HUB = 0,
	SCENE_WEAPONS,
	SCENE_MATERIALS,
	SCENE_PHYSICS,
	SCENE_VEHICLES,
	SCENE_KIND_COUNT
};

// one row per scene : build it, draw its HUD, draw its debug controls
struct Scene_Def
{
	const char* name;
	void (*build)(Scene*, GameRenderer*);
	void (*ux)(Scene*, GameWindow*);
	void (*debug)(Scene*);
};

static const Scene_Def SCENE_DEFS[SCENE_KIND_COUNT] = {
	{ "hub",       scene_hub,       scene_hub_ux,       scene_hub_debug },
	{ "weapons",   scene_weapons,   scene_weapons_ux,   scene_weapons_debug },
	{ "materials", scene_materials, scene_materials_ux, scene_materials_debug },
	{ "physics",   scene_physics,   scene_physics_ux,   scene_physics_debug },
	{ "vehicles",  scene_vehicles,  scene_vehicles_ux,  scene_vehicles_debug },
};

// owns exactly one scene. a request is applied at the top of the next frame, so
// the UI can never pull a scene out from under the pass it is drawing
struct SceneManager
{
	Scene scene;
	uint  current_kind;
	uint  requested_kind;
	bool  loaded;

	static const char* label(uint kind)
	{
		return (kind < SCENE_KIND_COUNT) ? SCENE_DEFS[kind].name : "-";
	}

	void init(GameRenderer* renderer)
	{
		loaded = false;
		current_kind = requested_kind = SCENE_HUB;
		load(renderer, SCENE_HUB);
	}

	// ask for a scene : switching happens in update(), never mid-frame
	void request(uint kind)
	{
		if (kind < SCENE_KIND_COUNT) requested_kind = kind;
	}

	void update(GameWindow* window, GameRenderer* renderer)
	{
		if (!loaded || requested_kind != current_kind)
			load(renderer, requested_kind);

		scene.update(window, renderer);
	}

	// unload the current scene, then build the next one from its table entry
	void load(GameRenderer* renderer, uint kind)
	{
		if (loaded)
		{
			scene.shutdown(renderer);
			renderer->unload_scene();
		}

		if (kind >= SCENE_KIND_COUNT || !SCENE_DEFS[kind].build) kind = SCENE_HUB;
		SCENE_DEFS[kind].build(&scene, renderer);

		current_kind = requested_kind = kind;
		loaded = true;
	}

	// scene HUD : drawn every frame, even with the console hidden
	void draw_ux(GameWindow* window)
	{
		if (current_kind < SCENE_KIND_COUNT && SCENE_DEFS[current_kind].ux)
			SCENE_DEFS[current_kind].ux(&scene, window);
	}

	// scene debug : drawn inside the Scenes tab
	void draw_debug()
	{
		if (current_kind < SCENE_KIND_COUNT && SCENE_DEFS[current_kind].debug)
			SCENE_DEFS[current_kind].debug(&scene);
	}
};
