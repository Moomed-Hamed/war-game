#include "shallow.h"

/* ------------------------- *
	-        Scenes         -
 * ------------------------- */

// the console slab : the scene aims at the strip the panel leaves visible.
// draw_console owns the same width rule and the F1 toggle
static bool ux_panel_visible = true;

static float ux_panel_width(GameWindow* window)
{
	float width = window->screen_height * (620.f / 1080.f);
	float max_width = window->screen_width * .45f;
	return width > max_width ? max_width : width;
}

// what the world's guns are made of : dull steel, the mesh's texture keeps the detail
const vec3 GUN_ALBEDO = vec3(0.55f, 0.56f, 0.58f);

const uint NUM_WEAPONS = 7;      // the weapon row
const uint WEAPON_VIEW_MODEL = 6; // testpistol : the on-foot view model
const uint MAX_ACTORS  = 16;     // skinned meshes animating in place

// one scene. no state outlives a switch : the manager tears this down and the
// next builder fills it from scratch. systems are passed in, not inherited
struct Scene
{
	Camera camera;    // the view the world is rendered from
	mat4   proj_view; // built from the camera every frame in update()

	// vehicles : a smoothed chase camera read from the car, not the player
	bool   car_camera;
	Camera vehicle_view;
	float  chase_distance, chase_height;

	Physics phys; // owns motion, the props read their pose from it
	Terrain terrain; // generated height grid for the terrain scene
	Water   water;   // fft ocean grid for the ocean scene
	Shallow_Water shallow; // small fd patch for the shallow scene

	Player player; // the capsule, the view gun and its projectiles

	// world content : a mesh plus an optional body. collider sizes are in
	// world units (mesh scale already applied) ; dynamic props get pos/rot
	// overwritten from bullet every frame
	struct Prop {
		uint mesh_id;
		uint phys_id;    // 0 = static decoration
		uint wheel_index; // vehicles : raycast wheel visual, (uint)-1 = none
		bool no_uv;      // which static buffer this mesh lives in (mode 1 or mode 3)
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
	bool reload_requested; // scene asks the manager to rebuild it next frame
	bool terrain_rebuild; // terrain generation changed : refresh grid, collider and meshes
	bool water_dirty;     // water settings changed : refresh the spectrum or the mesh
	bool water_spectrum;  // wind, seed or size changed : rebuild h0 as well

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
		p.wheel_index = (uint)-1;
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

	// ---- shared start : wipe the scene, place the camera and the ground.
	// floor_resolution 0 = one quad per metre
	void begin(GameRenderer* renderer, vec3 cam_pos, float yaw, float pitch,
	           float ground_size, float ground_half, uint floor_resolution = 0,
	           float floor_y = 0.f)
	{
		// zero in place : the scene owns multi-megabyte grids, and `= {}` would
		// build a temporary of the whole struct on the stack
		memset(this, 0, sizeof(*this));
		show_labels = true;
		use_lighting(LIGHT_DAY);

		camera.position = cam_pos;
		camera.yaw      = yaw;
		camera.pitch    = pitch;
		camera.update_dir(0, 0, 0); // seeds front/right/up from yaw/pitch

		phys.init();

		// floor : generated grid plane, the shader tiles its uvs (uv * 4)
		uint resolution = floor_resolution ? floor_resolution
		                                    : (uint)(ground_size > 1.f ? ground_size : 1.f);
		uint floor_mesh = renderer->add_mesh(
			MeshFactory::generate_plane(ground_size, resolution, MeshFactory::PLANE_XZ),
			"assets/textures/grid.bmp");
		add_prop(floor_mesh, vec3(0, floor_y, 0), 0, 1.f).phys_id = phys.add_ground(ground_half, floor_y);
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
		player.update(window, renderer, &phys, !car_camera);

		// only the chase view drives : walking or flying leaves the car parked
		phys.update_vehicle(window, car_camera && !player.flycam);
		phys.step(window->dtime);

		// vehicles : the chase camera reads the car, not the player capsule
		if (car_camera && !player.flycam && phys.vehicle.chassis_id)
		{
			vec3 car_pos;
			mat3 car_rot;
			if (phys.get_transform(phys.vehicle.chassis_id, &car_pos, &car_rot))
			{
				vec3 forward = car_rot[2];
				forward.y = 0.f;
				if (length(forward) < .01f) forward = vec3(0, 0, 1);
				else forward = normalize(forward);

				vec3 target  = car_pos + forward * 3.f + vec3(0, 1.f, 0);
				vec3 desired = car_pos - forward * chase_distance + vec3(0, chase_height, 0);

				float k = 1.f - expf(-6.f * window->dtime);
				vehicle_view.position += (desired - vehicle_view.position) * k;
				vehicle_view.front = normalize(target - vehicle_view.position);
				vehicle_view.up = vec3(0, 1, 0);

				camera.position = vehicle_view.position; // fog and sky read the active view
			}
		}

		// the player's camera while walking, the car camera, or the free camera
		Camera& view = player.flycam ? camera
		             : (car_camera && phys.vehicle.chassis_id ? vehicle_view : player.camera);

		// the g-buffer reads this for fog and sky : keep it on the active view
		if (!player.flycam && !car_camera) camera.position = view.position;

		// a little FOV stretch with speed, only in the chase view
		float speed_fov = 0.f;
		if (car_camera && !player.flycam)
		{
			float kmh = fabsf(phys.vehicle_speed() * 3.6f);
			speed_fov = kmh / 160.f * 8.f;
			if (speed_fov > 8.f) speed_fov = 8.f;
		}

		float fov = (60.f + player.fov_punch + speed_fov) * PI / 180.f;
		mat4 proj = glm::perspective(fov, (float)window->screen_width / window->screen_height, 0.1f, 256.f);

		// off-axis projection : the camera keeps looking down the world axis, but the
		// visible strip is centred, so the panel does not hide the subject
		if (ux_panel_visible)
			proj[2][0] = -ux_panel_width(window) / (float)window->screen_width;

		proj_view = proj * glm::lookAt(view.position, view.position + view.front, view.up);

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
			else if (p.wheel_index < 4)
			{
				if (phys.get_wheel_transform(p.wheel_index, &p.pos, &p.rot))
				{
					// the wheel mesh is y-up, the raycast wheel's axle is its local x
					static const mat3 wheel_mesh_rot =
						mat3(glm::rotate(mat4(1), -PI / 2, vec3(0, 0, 1)));
					p.rot = p.rot * wheel_mesh_rot;
				}
			}

			Instance inst = { glm::translate(mat4(1), p.pos) * mat4(p.rot) * glm::scale(mat4(1), p.scale),
			                  p.mat_a, p.mat_b };

			// a prop says which static layout its mesh was built for (mode 1 or mode 3)
			renderer->buffer(p.no_uv ? MODE_STATIC_NOUV : MODE_STATIC_UV).append_instances(p.mesh_id, 1, &inst);
		}

		// view gun : appended just before the actors, so the shared gun mesh
		// keeps one contiguous run of instances
		if (!player.flycam && !car_camera && player.gun_mesh)
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

// the display row and the hub's armory read the same table : mesh, clip, which
// take starts it, and how long one cycle runs
struct Weapon_Setup
{
	const char* mesh;
	const char* clip;
	uint seg;
	float dur;
};

static const Weapon_Setup WEAPON_SETUP[NUM_WEAPONS] = {
	{ "assets/anim/glock.mesh_anim",       "assets/anim/glock.anim",       0, 2.0f },
	{ "assets/anim/ak.mesh_anim",          "assets/anim/ak.anim",          0, 2.6f },
	{ "assets/anim/m4.mesh_anim",          "assets/anim/m4.anim",          1, 2.3f },
	{ "assets/anim/shotgun.mesh_anim",     "assets/anim/shotgun.anim",     1, 3.2f },
	{ "assets/anim/pistol_colt.mesh_anim", "assets/anim/pistol_colt.anim", 1, 1.6f },
	{ "assets/anim/bolt_action.mesh_anim", "assets/anim/bolt_action.anim", 0, 1.4f },
	{ "assets/anim/testpistol.mesh_anim",  "assets/anim/testpistol.anim",  0, 3.6f },
};

// frame ranges per weapon + a different feel on every segment
void weapon_slice(uint index, Animation* clip)
{
	switch (index)
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
}

// load one weapon actor into the animated buffer and start its cycle
void weapon_add_actor(Scene* s, GameRenderer* renderer, uint index, vec3 pos,
                      float yaw, float scale)
{
	if (index >= NUM_WEAPONS) return;

	uint mesh = renderer->add_anim_mesh(WEAPON_SETUP[index].mesh);
	uint clip = 0;
	renderer->animloader.load(WEAPON_SETUP[index].clip, &clip);
	if (!mesh || !clip) return;

	weapon_slice(index, &renderer->animloader.clips[clip]);

	uint player = renderer->animloader.play(clip, WEAPON_SETUP[index].seg,
	                                        WEAPON_SETUP[index].dur);
	renderer->bind_player(mesh, player);
	s->add_actor(mesh, player, pos, yaw, scale);
}

// ---- weapons : the row, a range, and knockable targets
void scene_weapons(Scene* s, GameRenderer* renderer)
{
	s->begin(renderer, vec3(0, 2.6f, 10), -PI / 2, -0.13f, 60.f, 30.f);
	s->use_lighting(LIGHT_OVERCAST);

	struct Weapon { uint mesh, clip, player; vec3 pos; float scale, yaw; } weapons[NUM_WEAPONS] = {};

	// the row sits along the left wall, facing into the range
	for (uint i = 0; i < NUM_WEAPONS; i++)
	{
		Weapon& w = weapons[i];
		w.mesh = renderer->add_anim_mesh(WEAPON_SETUP[i].mesh);
		renderer->animloader.load(WEAPON_SETUP[i].clip, &w.clip);
		if (!w.mesh || !w.clip) continue;

		weapon_slice(i, &renderer->animloader.clips[w.clip]);

		w.player = renderer->animloader.play(w.clip, WEAPON_SETUP[i].seg, WEAPON_SETUP[i].dur);
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
	s->player.init(renderer, &s->phys, vec3(0, 5, 8),
	               weapons[WEAPON_VIEW_MODEL].mesh, weapons[WEAPON_VIEW_MODEL].clip);

	s->finish(renderer, "weapons");
}

// the hero sphere's live material : the builder fills the handle, the debug
// tab edits the values and applies them to the prop
struct Render_Controls
{
	vec3  hero_albedo;
	float hero_metallic, hero_roughness, hero_ao;
	uint  hero_prop;
};

static Render_Controls render_controls;

void render_default_material()
{
	render_controls.hero_albedo   = vec3(1.000f, 0.766f, 0.336f); // gold
	render_controls.hero_metallic = 1.f;
	render_controls.hero_roughness = .12f;
	render_controls.hero_ao        = 1.f;
}

// ---- render : a studio lab for materials, the metal/roughness matrix and texture
void scene_render(Scene* s, GameRenderer* renderer)
{
	render_controls = {};
	render_controls.hero_prop = (uint)-1;
	render_default_material();

	s->begin(renderer, vec3(0, 2.4f, 10.f), -PI / 2, -0.10f, 30.f, 15.f);
	s->use_lighting(LIGHT_STUDIO);

	// neutral floor : the grid is a developer texture, not part of the lab
	Scene::Prop& floor = s->props[0];
	floor.mat_a = vec4(0.045f, 0.048f, 0.055f, 0.f);
	floor.mat_b = vec4(0.55f, 1.f, 0.f, 0.f);

	// stage and backdrop : a dark, matte frame for the tests
	uint cube_mesh  = renderer->add_mesh(MeshFactory::generate_cube(1.f), "assets/textures/checker.bmp");
	uint stage_mesh = renderer->add_mesh(MeshFactory::generate_cube(1.f));

	Scene::Prop& stage = s->add_prop(stage_mesh, vec3(0.f, .15f, -4.f), 0, vec3(17.f, .3f, 10.f));
	stage.phys_id = s->phys.add_box(stage.pos, vec3(8.5f, .15f, 5.f), 0);
	stage.mat_a = vec4(0.060f, 0.064f, 0.072f, 0.f);
	stage.mat_b = vec4(0.34f, 1.f, 0.f, 0.f);

	Scene::Prop& backdrop = s->add_prop(stage_mesh, vec3(0.f, 3.f, -9.f), 0, vec3(20.f, 6.f, .2f));
	backdrop.mat_a = vec4(0.020f, 0.022f, 0.026f, 0.f);
	backdrop.mat_b = vec4(0.72f, 1.f, 0.f, 0.f);

	// material library : ten named spheres, metals first
	struct Material { const char* name; vec3 albedo; float metallic, roughness; };
	const Material materials[] = {
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
	const uint num_materials = (uint)(sizeof(materials) / sizeof(materials[0]));

	uint sphere_mesh = renderer->add_mesh(MeshFactory::generate_uv_sphere(1.f, 32, 20));
	for (uint i = 0; i < num_materials; i++)
	{
		float x = ((float)i - (num_materials - 1) * .5f) * 1.2f;
		Scene::Prop& p = s->add_prop(sphere_mesh, vec3(x, .72f, -2.5f), 0, .42f);
		p.mat_a = vec4(materials[i].albedo, materials[i].metallic);
		p.mat_b = vec4(materials[i].roughness, 1.f, 0.f, 0.f);
		snprintf(p.label, sizeof(p.label), "%s", materials[i].name);
	}

	// metal/roughness matrix : five steps each way on a matte panel
	Scene::Prop& panel = s->add_prop(stage_mesh, vec3(0.f, 2.1f, -8.75f), 0, vec3(3.6f, 3.6f, .12f));
	panel.mat_a = vec4(0.030f, 0.032f, 0.038f, 0.f);
	panel.mat_b = vec4(0.62f, 1.f, 0.f, 0.f);

	const uint  columns = 5;
	const float spacing = .55f;
	const float radius  = .20f;
	for (uint row = 0; row < columns; row++)
		for (uint col = 0; col < columns; col++)
		{
			float metal = row / (float)(columns - 1);
			float rough = .05f + col * .2375f;
			float x = ((float)col - (columns - 1) * .5f) * spacing;
			float y = 1.f + (float)row * spacing;

			Scene::Prop& p = s->add_prop(sphere_mesh, vec3(x, y, -8.45f), 0, radius);
			p.mat_a = vec4(0.72f, 0.70f, 0.68f, metal);
			p.mat_b = vec4(rough, 1.f, 0.f, 0.f);
		}

	Scene::Prop& caption = s->add_prop(0, vec3(0.f, 4.2f, -8.45f), 0, 1.f);
	snprintf(caption.label, sizeof(caption.label), "metal up / rough right");

	// texture vs flat : the same cube, one reads the checker, one ignores it
	Scene::Prop& textured = s->add_prop(cube_mesh, vec3(-6.f, .8f, -5.f), .35f, vec3(1.6f, 1.f, 1.6f));
	textured.mat_a = vec4(1.f, 1.f, 1.f, 0.f);
	textured.mat_b = vec4(.42f, 1.f, 1.f, 0.f);
	snprintf(textured.label, sizeof(textured.label), "textured");

	Scene::Prop& flat = s->add_prop(cube_mesh, vec3(6.f, .8f, -5.f), -.35f, vec3(1.6f, 1.f, 1.6f));
	flat.mat_a = vec4(.72f, .18f, .14f, 0.f);
	flat.mat_b = vec4(.38f, 1.f, 0.f, 0.f);
	snprintf(flat.label, sizeof(flat.label), "flat");

	// hero sphere : the material the UX edits live
	Scene::Prop& plinth = s->add_prop(stage_mesh, vec3(7.5f, .6f, -2.5f), 0, vec3(1.6f, .6f, 1.6f));
	plinth.mat_a = vec4(0.060f, 0.064f, 0.072f, 0.f);
	plinth.mat_b = vec4(0.34f, 1.f, 0.f, 0.f);

	Scene::Prop& hero = s->add_prop(sphere_mesh, vec3(7.5f, 1.7f, -2.5f), 0, .8f);
	hero.mat_a = vec4(render_controls.hero_albedo, render_controls.hero_metallic);
	hero.mat_b = vec4(render_controls.hero_roughness, render_controls.hero_ao, 0.f, 0.f);
	snprintf(hero.label, sizeof(hero.label), "hero");

	s->player.init(renderer, &s->phys, vec3(0.f, 2.2f, 10.f), 0, 0);
	s->finish(renderer, "render");

	// props are sorted by mesh in finish(), so find the hero afterwards
	for (uint i = 0; i < s->num_props; i++)
		if (strcmp(s->props[i].label, "hero") == 0)
			render_controls.hero_prop = i;
}

// terrain : the generation and strata settings persist across a reload, the band
// handles let the debug tab retint a mesh without rebuilding it
static Terrain_Config terrain_config;

struct Terrain_View
{
	uint mesh[TERRAIN_BAND_COUNT]; // band mesh ids : stable across a live rebuild
	uint prop[TERRAIN_BAND_COUNT]; // prop indices, for live material edits
	uint body;                     // heightfield body, replaced in place

	// where the band meshes start in the static buffer, so a rebuild can drop
	// them and upload the new triangles into the same space
	uint mesh_mark, geom_mark, indx_mark;
};

static Terrain_View terrain_view = {};

// build the generated island + collider into the current scene. a caller that
// edited the heights first passes generate = false and its own band colours
void terrain_build_content(Scene* s, GameRenderer* renderer, const Terrain_Config& config,
                           bool generate = true)
{
	if (generate) terrain_generate(&s->terrain, config);

	DrawBuffer& buffer = renderer->buffer(MODE_STATIC_UV);
	terrain_view.mesh_mark = buffer.num_meshes;
	terrain_view.geom_mark = buffer.geom_size;
	terrain_view.indx_mark = buffer.indx_size;

	// one mesh and one material per stratum : water, sand, grass, rock, snow.
	// every band gets an id even when empty, so a rebuild never has to move props
	for (uint band = 0; band < TERRAIN_BAND_COUNT; band++)
	{
		uint mesh_id = renderer->add_mesh(terrain_build_band(&s->terrain, band));
		Scene::Prop& p = s->add_prop(mesh_id, vec3(0), 0, 1.f);
		terrain_band_material(config, band, &p.mat_a, &p.mat_b);
		terrain_view.mesh[band] = mesh_id;
		terrain_view.prop[band] = (uint)-1;
	}

	terrain_view.body = s->phys.add_heightfield(s->terrain.heights, s->terrain.resolution,
	                                            s->terrain.size, s->terrain.min_height,
	                                            s->terrain.max_height);
}

void terrain_resolve_props(Scene* s)
{
	for (uint band = 0; band < TERRAIN_BAND_COUNT; band++)
	{
		if (!terrain_view.mesh[band]) continue;
		for (uint i = 0; i < s->num_props; i++)
			if (s->props[i].mesh_id == terrain_view.mesh[band])
				terrain_view.prop[band] = i;
	}
}

// ---- terrain : a generated island with a matching bullet heightfield
void scene_terrain(Scene* s, GameRenderer* renderer)
{
	// far enough back that the island fits the strip beside the console, not the
	// whole window; F drops to the player for a ground-level look
	s->begin(renderer, vec3(0, 88.f, 162.f), -PI / 2, -0.50f, 200.f, 100.f);
	s->use_lighting(LIGHT_STUDIO);

	// the flat ground under the island
	Scene::Prop& floor = s->props[0];
	floor.mat_a = vec4(.10f, .12f, .08f, 0.f);
	floor.mat_b = vec4(.95f, 1.f, 0.f, 0.f);

	terrain_build_content(s, renderer, terrain_config);

	float spawn_h = terrain_height(&s->terrain, 0.f, 20.f);
	s->player.init(renderer, &s->phys, vec3(0.f, spawn_h + 2.f, 20.f), 0, 0);
	s->player.flycam = true; // inspect the terrain first; F switches to the capsule
	s->finish(renderer, "terrain");
	terrain_resolve_props(s);
}

// live terrain edit : regenerate the grid, replace the collider and refill the
// band meshes at their original slots. mesh ids stay put, so props need no work
void terrain_rebuild(Scene* s, GameRenderer* renderer)
{
	terrain_generate(&s->terrain, terrain_config);

	terrain_view.body = s->phys.set_heightfield(terrain_view.body,
		s->terrain.heights, s->terrain.resolution, s->terrain.size,
		s->terrain.min_height, s->terrain.max_height);

	DrawBuffer& buffer = renderer->buffer(MODE_STATIC_UV);
	buffer.rewind(terrain_view.mesh_mark, terrain_view.geom_mark, terrain_view.indx_mark);

	for (uint band = 0; band < TERRAIN_BAND_COUNT; band++)
	{
		if (!terrain_view.mesh[band]) continue;
		Mesh_Data mesh = terrain_build_band(&s->terrain, band);
		buffer.append_geometry(terrain_view.mesh[band], mesh, 0);
		mesh.release();
	}

	s->terrain_rebuild = false;
}

/* ------------------------- *
	-         Water         -
 * ------------------------- */

const float WATER_LEVEL = 1.6f;  // mean surface height, metres
const uint  WATER_TILES = 5;     // one patch repeated into a sea

static Water_Config water_config;

struct Water_View
{
	uint water_mesh;
	uint water_tx;  // dynamic foam field texture
	uint detail_tx; // dynamic short-chop normal map
	uint mesh_mark, geom_mark, indx_mark;

	uint water_props[WATER_TILES * WATER_TILES]; // tile order, after finish()

	float tile_size; // size the tile transforms were built for
	float level;     // sea level the tiles sit at
};

static Water_View water_view = {};

// ---- shallow : a small finite-difference patch in a stone basin
static Shallow_Config shallow_config;

struct Shallow_View
{
	uint water_mesh;
	uint prop;
	uint mesh_mark, geom_mark, indx_mark;
	float level; // surface height the prop sits at
};

static Shallow_View shallow_view = {};

void water_apply_materials(Scene* s)
{
	vec4 water_a = vec4(water_config.albedo, water_config.metallic);
	vec4 water_b = vec4(water_config.roughness, 1.f, 0.f, 1.f); // mode 1 : foam field

	for (uint i = 1; i < s->num_props; i++)
	{
		if (s->props[i].mesh_id == water_view.water_mesh)
		{
			s->props[i].mat_a = water_a;
			s->props[i].mat_b = water_b;
		}
	}
}

// the patch size is live : the tiles have to follow the mesh's own spacing
void water_place_tiles(Scene* s)
{
	float tile = water_config.size;
	float origin = -((float)(WATER_TILES - 1) * .5f) * tile;

	for (uint j = 0; j < WATER_TILES; j++)
	{
		for (uint i = 0; i < WATER_TILES; i++)
		{
			uint k = j * WATER_TILES + i;
			vec3 pos = vec3(origin + i * tile, water_view.level, origin + j * tile);

			if (water_view.water_props[k] < s->num_props)
				s->props[water_view.water_props[k]].pos = pos;
		}
	}

	water_view.tile_size = tile;
}

// build the fft sea into the current scene : water mesh + tile props. the
// caller owns finish()/prop resolution so the hub can share this too
void water_build_content(Scene* s, GameRenderer* renderer, float level)
{
	water_build_spectrum(&s->water, water_config);
	water_build_detail(&s->water.detail, water_config);
	water_update(&s->water, water_config, 0.f, true);
	water_update_detail(&s->water.detail, s->water.time);

	DrawBuffer& buffer = renderer->buffer(MODE_STATIC_UV);
	water_view.mesh_mark = buffer.num_meshes;
	water_view.geom_mark = buffer.geom_size;
	water_view.indx_mark = buffer.indx_size;
	water_view.level = level;

	// one dynamic texture holds the persistent foam field. the water shader samples
	// it per pixel : no foam geometry, so no cell edges and no flicker
	water_view.water_tx = renderer->txloader.create_dynamic("foam_field", s->water.n, s->water.n);
	renderer->txloader.upload_dynamic(water_view.water_tx, s->water.foam_pixels,
	                                  s->water.n, s->water.n);

	water_view.detail_tx = renderer->txloader.create_dynamic_rg("water_detail",
	                                                            s->water.detail.n, s->water.detail.n);
	renderer->txloader.upload_dynamic_rg(water_view.detail_tx, s->water.detail.normals,
	                                     s->water.detail.n, s->water.detail.n);

	water_view.water_mesh = renderer->add_mesh(water_build_mesh(&s->water), "foam_field");

	float tile = water_config.size;
	float origin = -((float)(WATER_TILES - 1) * .5f) * tile;
	for (uint j = 0; j < WATER_TILES; j++)
	{
		for (uint i = 0; i < WATER_TILES; i++)
		{
			vec3 pos = vec3(origin + i * tile, level, origin + j * tile);
			s->add_prop(water_view.water_mesh, pos, 0, 1.f);
		}
	}

	water_view.tile_size = tile;
}

void water_resolve_props(Scene* s)
{
	for (uint i = 0; i < WATER_TILES * WATER_TILES; i++)
		water_view.water_props[i] = (uint)-1;

	uint nw = 0;
	for (uint i = 0; i < s->num_props; i++)
		if (s->props[i].mesh_id == water_view.water_mesh && nw < WATER_TILES * WATER_TILES)
			water_view.water_props[nw++] = i;

	water_apply_materials(s);
}

// evolve the sim and refresh its two dynamic textures. no geometry, so the hub
// can call this before it appends every dynamic mesh at one shared mark
bool water_simulate(Scene* s, GameRenderer* renderer, float dt)
{
	if (s->water_spectrum)
	{
		water_build_spectrum(&s->water, water_config);
		water_build_detail(&s->water.detail, water_config);
		s->water_spectrum = false;
	}

	bool changed = water_update(&s->water, water_config, dt, s->water_dirty);
	s->water_dirty = false;

	if (changed)
	{
		water_update_detail(&s->water.detail, s->water.time);

		renderer->txloader.upload_dynamic(water_view.water_tx, s->water.foam_pixels,
		                                  s->water.n, s->water.n);
		renderer->txloader.upload_dynamic_rg(water_view.detail_tx, s->water.detail.normals,
		                                     s->water.detail.n, s->water.detail.n);
	}

	return changed;
}

void water_append_geometry(Scene* s, GameRenderer* renderer)
{
	DrawBuffer& buffer = renderer->buffer(MODE_STATIC_UV);
	Mesh_Data mesh = water_build_mesh(&s->water);
	buffer.append_geometry(water_view.water_mesh, mesh, water_view.water_tx);
	mesh.release();
}

// per frame water uniforms : the deferred pass reads the foam tint and the
// short-chop normal map from the window, whichever scene owns the sea
void water_bind_window(GameWindow* window)
{
	window->water_foam_color     = water_config.foam_albedo;
	window->water_foam_roughness = water_config.foam_roughness;
	window->water_normal_tx      = water_view.detail_tx;
	window->water_detail         = water_config.detail;
	window->water_detail_tiles   = water_config.size / 16.f;
}

// ---- water : one fft patch tiled into open sea
void scene_water(Scene* s, GameRenderer* renderer)
{
	s->begin(renderer, vec3(0, 8.f, 30.f), -PI / 2, -0.12f, 240.f, 120.f, 8, -5.f);
	s->use_lighting(LIGHT_SUNSET);

	// seabed : dark and deep enough that the troughs never reach it
	Scene::Prop& floor = s->props[0];
	floor.mat_a = vec4(.025f, .035f, .032f, 0.f);
	floor.mat_b = vec4(.95f, 1.f, 0.f, 0.f);

	s->player.init(renderer, &s->phys, vec3(0, WATER_LEVEL + 3.f, 22.f), 0, 0);
	s->player.flycam = true; // inspect the surface first

	water_build_content(s, renderer, WATER_LEVEL);

	s->finish(renderer, "water");
	water_resolve_props(s);
	s->water_dirty = false;
	s->water_spectrum = false;
}

void water_scene_update(Scene* s, GameRenderer* renderer, float dt)
{
	bool changed = water_simulate(s, renderer, dt);

	if (changed)
	{
		DrawBuffer& buffer = renderer->buffer(MODE_STATIC_UV);
		buffer.rewind(water_view.mesh_mark, water_view.geom_mark, water_view.indx_mark);
		water_append_geometry(s, renderer);
	}

	if (water_config.size != water_view.tile_size) water_place_tiles(s);

	water_apply_materials(s);
}

void shallow_apply_material(Scene* s)
{
	if (shallow_view.prop >= s->num_props) return;
	Scene::Prop& p = s->props[shallow_view.prop];
	p.pos.y = shallow_view.level;
	p.mat_a = vec4(shallow_config.albedo, shallow_config.metallic);
	p.mat_b = vec4(shallow_config.roughness, 1.f, 0.f, 0.f);
}

// add the simulated patch at one spot : the caller builds the rim/platform
void shallow_build_content(Scene* s, GameRenderer* renderer, vec3 center, float level)
{
	shallow_reset(&s->shallow, shallow_config);
	shallow_drop(&s->shallow, 0.f, 0.f, shallow_config.drop_strength);
	shallow_view.level = level;

	DrawBuffer& buffer = renderer->buffer(MODE_STATIC_UV);
	shallow_view.mesh_mark = buffer.num_meshes;
	shallow_view.geom_mark = buffer.geom_size;
	shallow_view.indx_mark = buffer.indx_size;

	shallow_view.water_mesh = renderer->add_mesh(shallow_build_mesh(&s->shallow));
	Scene::Prop& water = s->add_prop(shallow_view.water_mesh,
	                                 vec3(center.x, level, center.z), 0, 1.f);
	water.mat_a = vec4(shallow_config.albedo, shallow_config.metallic);
	water.mat_b = vec4(shallow_config.roughness, 1.f, 0.f, 0.f);
}

void shallow_resolve_props(Scene* s)
{
	shallow_view.prop = (uint)-1;
	for (uint i = 0; i < s->num_props; i++)
		if (s->props[i].mesh_id == shallow_view.water_mesh)
			shallow_view.prop = i;

	shallow_apply_material(s);
}

bool shallow_simulate(Scene* s, float dt)
{
	bool stepped = shallow_update(&s->shallow, shallow_config, dt, s->shallow.paused);
	if (!stepped && !s->shallow.dirty) return false;
	s->shallow.dirty = false;
	return true;
}

void shallow_append_geometry(Scene* s, GameRenderer* renderer)
{
	DrawBuffer& buffer = renderer->buffer(MODE_STATIC_UV);
	Mesh_Data mesh = shallow_build_mesh(&s->shallow);
	buffer.append_geometry(shallow_view.water_mesh, mesh, 0);
	mesh.release();
}

// ---- shallow : the finite-difference patch in a stone basin
void scene_shallow(Scene* s, GameRenderer* renderer)
{
	s->begin(renderer, vec3(0, 5.5f, 9.f), -PI / 2, -0.35f, 20.f, 10.f, 4, .5f);
	s->use_lighting(LIGHT_DAY);

	Scene::Prop& floor = s->props[0];
	floor.mat_a = vec4(.34f, .30f, .23f, 0.f);
	floor.mat_b = vec4(.92f, 1.f, 0.f, 0.f);

	uint cube = renderer->add_mesh(MeshFactory::generate_cube(1.f));

	// four stone walls : inner opening is the 8 x 8 water patch
	struct Wall { vec3 pos, scale; };
	const Wall walls[4] = {
		{ vec3(0, 1.35f,  4.3f), vec3(9.2f, 1.7f, .6f) },
		{ vec3(0, 1.35f, -4.3f), vec3(9.2f, 1.7f, .6f) },
		{ vec3( 4.3f, 1.35f, 0), vec3(.6f, 1.7f, 9.2f) },
		{ vec3(-4.3f, 1.35f, 0), vec3(.6f, 1.7f, 9.2f) },
	};

	for (uint i = 0; i < 4; i++)
	{
		Scene::Prop& wall = s->add_prop(cube, walls[i].pos, 0, walls[i].scale);
		wall.mat_a = vec4(.27f, .26f, .24f, 0.f);
		wall.mat_b = vec4(.86f, 1.f, 0.f, 0.f);
		s->phys.add_box(wall.pos, walls[i].scale * .5f, 0);
	}

	// the player is only a camera here; the basin is walkable if F is pressed
	s->player.init(renderer, &s->phys, vec3(0, 2.4f, 7.f), 0, 0);
	s->player.flycam = true;

	shallow_build_content(s, renderer, vec3(0), shallow_config.level);

	s->finish(renderer, "shallow");
	shallow_resolve_props(s);
}

void shallow_scene_update(Scene* s, GameRenderer* renderer, float dt)
{
	if (!shallow_simulate(s, dt))
	{
		shallow_apply_material(s);
		return;
	}

	DrawBuffer& buffer = renderer->buffer(MODE_STATIC_UV);
	buffer.rewind(shallow_view.mesh_mark, shallow_view.geom_mark, shallow_view.indx_mark);
	shallow_append_geometry(s, renderer);
	shallow_apply_material(s);
}

/* ------------------------- *
	-          Hub          -
 * ------------------------- */

// the hub's island is its own generation : the terrain scene's settings stay
// untouched when either scene reloads
static Terrain_Config hub_terrain_config;
static bool hub_config_ready = false;

const float HUB_CISTERN_Z = -24.f;  // basin centre
const float HUB_CISTERN_LIFT = .55f; // water above the yard

const float HUB_VALLEY_Z0 = -30.f; // yard end of the sea valley
const float HUB_VALLEY_Z1 = -63.f; // waterline end
const float HUB_VALLEY_H1 = .40f;  // valley floor at the water
const float HUB_SHORE_Z   = -56.f; // where the road meets the sea

struct Hub_View
{
	uint mesh_mark, geom_mark, indx_mark; // where the two simulated surfaces start
	float plateau;                        // the flat yard height
};

static Hub_View hub_view = {};

void hub_prepare_config()
{
	if (hub_config_ready) return;
	hub_config_ready = true;

	hub_terrain_config.resolution = 177;
	hub_terrain_config.size       = 128.f;
	hub_terrain_config.height     = 15.f;
	hub_terrain_config.seed       = 19440606u;
	hub_terrain_config.octaves    = 6;
	hub_terrain_config.frequency  = .017f;
	hub_terrain_config.lacunarity = 2.05f;
	hub_terrain_config.gain       = .52f;

	hub_terrain_config.water_at = .06f;
	hub_terrain_config.sand_at  = .20f;
	hub_terrain_config.grass_at = .75f; // the yard sits on grass, high ground on rock
	hub_terrain_config.rock_at  = 1.10f; // rock to the top : no snow on this coast

	hub_terrain_config.sand_albedo  = vec3(.50f, .44f, .30f);
	hub_terrain_config.grass_albedo = vec3(.14f, .22f, .09f);
	hub_terrain_config.rock_albedo  = vec3(.25f, .24f, .22f);
}

void hub_apply_lighting(Scene* s)
{
	s->use_lighting(LIGHT_SUNSET);

	// the sun sits low over the water, ahead of the yard : warm tops, long
	// silhouettes and a specular path back to the camera
	s->lighting.sun_direction = normalize(vec3(.10f, -.42f, .90f));

	// a cool fill from behind the camera keeps the yard readable against that sun
	s->lighting.rim_direction = normalize(vec3(-.15f, -.45f, -.88f));
	s->lighting.rim_color     = vec3(.42f, .50f, .62f);
	s->lighting.rim_power     = 1.05f;
}

void hub_build_terrain(Scene* s, GameRenderer* renderer)
{
	hub_prepare_config();
	terrain_generate(&s->terrain, hub_terrain_config);

	const float cx = 0.f, cz = -8.f;
	const float half_x = 30.f, half_z = 24.f;

	float plateau = terrain_average_height(&s->terrain, cx, cz, half_x, half_z);
	if (plateau < WATER_LEVEL + 3.2f) plateau = WATER_LEVEL + 3.2f;

	terrain_flatten(&s->terrain, cx, cz, half_x, half_z, plateau, .35f);

	// a valley from the yard down to the waterline : the sea stays in view
	// instead of sitting behind whatever hill the seed placed to the south
	terrain_ramp(&s->terrain, 0.f, HUB_VALLEY_Z0, HUB_VALLEY_Z1, 8.f, .55f,
	             plateau, HUB_VALLEY_H1);

	terrain_update_levels(&s->terrain, hub_terrain_config);

	// the fft sea owns the water surface : a flat terrain-water shelf would sit
	// above it, so the water band is left empty and the sand runs into the sea
	s->terrain.water_level = -4.f;

	hub_view.plateau = plateau;
	terrain_build_content(s, renderer, hub_terrain_config, false);
}

void hub_add_road(Scene* s, uint path_mesh, float p)
{
	const vec4 road_a = vec4(.125f, .110f, .085f, 0.f);
	const vec4 road_b = vec4(.94f, 1.f, 0.f, 0.f);

	// a gravel lane from the gate to the yard, then down the valley to the water
	Scene::Prop& path = s->add_prop(path_mesh, vec3(0.f, p + .03f, -8.f), 0,
	                                vec3(6.f, 1.f, 44.f));
	path.mat_a = road_a;
	path.mat_b = road_b;

	float shore_h = p + (HUB_VALLEY_H1 - p) *
	                ((HUB_VALLEY_Z0 - HUB_SHORE_Z) / (HUB_VALLEY_Z0 - HUB_VALLEY_Z1));
	float drop_z = HUB_VALLEY_Z0 - HUB_SHORE_Z;
	float drop_h = p - shore_h;
	float slope_len = sqrtf(drop_z * drop_z + drop_h * drop_h);
	float slope_angle = -atanf(drop_h / drop_z);

	Scene::Prop& slope = s->add_prop(path_mesh,
	                                 vec3(0.f, (p + shore_h) * .5f + .04f,
	                                      (HUB_VALLEY_Z0 + HUB_SHORE_Z) * .5f), 0,
	                                 vec3(5.5f, 1.f, slope_len));
	slope.rot = mat3(glm::rotate(mat4(1), slope_angle, vec3(1, 0, 0)));
	slope.mat_a = road_a;
	slope.mat_b = road_b;
}

void hub_add_armory(Scene* s, GameRenderer* renderer, uint cube_mesh, float p)
{
	const vec3 wall_a = vec3(.185f, .190f, .185f);
	const vec4 wall_b = vec4(.84f, 1.f, 0.f, 0.f);

	// back wall and overhang : a covered rack along the west edge
	Scene::Prop& wall = s->add_prop(cube_mesh, vec3(-18.6f, p + 2.2f, -14.f), 0,
	                                vec3(.5f, 4.4f, 23.f));
	wall.mat_a = vec4(wall_a, 0.f);
	wall.mat_b = wall_b;
	wall.phys_id = s->phys.add_box(wall.pos, vec3(.25f, 2.2f, 11.5f), 0);

	Scene::Prop& roof = s->add_prop(cube_mesh, vec3(-16.4f, p + 4.35f, -14.f), 0,
	                                vec3(5.4f, .25f, 23.f));
	roof.mat_a = vec4(.105f, .110f, .105f, 0.f);
	roof.mat_b = vec4(.78f, 1.f, 0.f, 0.f);
	roof.phys_id = s->phys.add_box(roof.pos, vec3(2.7f, .125f, 11.5f), 0);

	Scene::Prop& shelf = s->add_prop(cube_mesh, vec3(-16.4f, p + .65f, -14.f), 0,
	                                 vec3(2.4f, .18f, 22.f));
	shelf.mat_a = vec4(.085f, .088f, .085f, 0.f);
	shelf.mat_b = vec4(.72f, 1.f, 0.f, 0.f);
	shelf.phys_id = s->phys.add_box(shelf.pos, vec3(1.2f, .09f, 11.f), 0);

	for (uint i = 0; i < WEAPON_VIEW_MODEL; i++)
	{
		float yaw = PI / 2 + ((i % 2) ? .12f : -.12f);
		weapon_add_actor(s, renderer, i, vec3(-16.f, p + 2.05f, -4.f - (float)i * 3.f),
		                 yaw, 3.4f);
	}
}

void hub_add_supply(Scene* s, uint cube_mesh, uint cylinder_mesh, uint ammo_mesh, float p)
{
	const vec4 crate_a = vec4(.155f, .165f, .120f, 0.f);
	const vec4 crate_b = vec4(.86f, 1.f, 0.f, 0.f);

	// supply : a dynamic stack the player and the car can knock over
	struct Crate { vec3 pos; float yaw; };
	const Crate crates[5] = {
		{ vec3(12.0f, p + .5f, -8.0f),  .10f },
		{ vec3(13.2f, p + .5f, -8.2f), -.08f },
		{ vec3(12.6f, p + 1.5f, -8.1f), .20f },
		{ vec3(12.1f, p + .5f, -9.4f), -.14f },
		{ vec3(13.3f, p + .5f, -9.5f),  .06f },
	};

	for (uint i = 0; i < 5; i++)
	{
		Scene::Prop& crate = s->add_prop(cube_mesh, crates[i].pos, crates[i].yaw, 1.f);
		crate.mat_a = crate_a;
		crate.mat_b = crate_b;
		crate.phys_id = s->phys.add_box(crate.pos, vec3(.5f), 8.f,
		                                vec3(0.f, crates[i].yaw, 0.f));
	}

	// ammo crates : small, loose, and light enough to shove
	vec3 ammo_spots[2] = { vec3(12.6f, p + 2.12f, -8.1f), vec3(15.4f, p + .45f, -12.0f) };
	for (uint i = 0; i < 2; i++)
	{
		Scene::Prop& ammo = s->add_prop(ammo_mesh, ammo_spots[i], .3f * (float)i, 1.4f);
		ammo.mat_a = vec4(.30f, .27f, .18f, .10f);
		ammo.mat_b = vec4(.70f, 1.f, 0.f, 0.f);
		ammo.phys_id = s->phys.add_box(ammo.pos, vec3(.15f, .10f, .15f), .6f);
	}

	// fuel drums
	vec3 drums[3] = {
		vec3(16.0f, p + .65f, -10.1f),
		vec3(17.3f, p + .65f, -10.6f),
		vec3(16.7f, p + .65f, -11.9f),
	};

	for (uint i = 0; i < 3; i++)
	{
		Scene::Prop& drum = s->add_prop(cylinder_mesh, drums[i], .2f * (float)i,
		                                vec3(1.2f, 1.2f, 1.2f));
		drum.mat_a = vec4(.22f, .13f, .075f, .45f);
		drum.mat_b = vec4(.64f, 1.f, 0.f, 0.f);
		drum.phys_id = s->phys.add_cylinder(drum.pos, vec3(.6f, .6f, .6f), 6.f);
	}
}

void hub_add_signal(Scene* s, uint cube_mesh, uint sphere_mesh, float p)
{
	// one polished sphere : the material work shown in the world
	Scene::Prop& plinth = s->add_prop(cube_mesh, vec3(18.f, p + 1.0f, -18.f), 0,
	                                  vec3(2.2f, 2.0f, 2.2f));
	plinth.mat_a = vec4(.175f, .180f, .175f, .55f);
	plinth.mat_b = vec4(.58f, 1.f, 0.f, 0.f);
	plinth.phys_id = s->phys.add_box(plinth.pos, vec3(1.1f, 1.f, 1.1f), 0);

	Scene::Prop& signal = s->add_prop(sphere_mesh, vec3(18.f, p + 3.0f, -18.f), 0, .95f);
	signal.mat_a = vec4(.72f, .73f, .75f, 1.f);
	signal.mat_b = vec4(.06f, 1.f, 0.f, 0.f);
}

void hub_add_palms(Scene* s, uint palm_mesh, float p)
{
	vec3 palms[4] = {
		vec3(-26.f, 0.f,   6.f), vec3(26.f, 0.f,   6.f),
		vec3(-26.f, 0.f, -29.f), vec3(26.f, 0.f, -29.f),
	};

	for (uint i = 0; i < 4; i++)
	{
		vec3 at = vec3(palms[i].x, p, palms[i].z);
		Scene::Prop& palm = s->add_prop(palm_mesh, at, (float)i * 1.7f, 1.8f);
		palm.phys_id = s->phys.add_box(at + vec3(0, 3.2f, 0), vec3(.22f, 3.2f, .22f), 0);
	}
}

void hub_add_cistern(Scene* s, uint cube_mesh, float p)
{
	const float cz = HUB_CISTERN_Z;
	const float level = p + HUB_CISTERN_LIFT;

	Scene::Prop& floor = s->add_prop(cube_mesh, vec3(0.f, p + .05f, cz), 0,
	                                 vec3(8.4f, .10f, 8.4f));
	floor.mat_a = vec4(.16f, .16f, .15f, 0.f);
	floor.mat_b = vec4(.90f, 1.f, 0.f, 0.f);

	struct Wall { vec3 pos, scale; };
	const Wall walls[4] = {
		{ vec3(0.f, level, cz + 4.3f), vec3(9.2f, 1.3f, .6f) },
		{ vec3(0.f, level, cz - 4.3f), vec3(9.2f, 1.3f, .6f) },
		{ vec3( 4.3f, level, cz), vec3(.6f, 1.3f, 9.2f) },
		{ vec3(-4.3f, level, cz), vec3(.6f, 1.3f, 9.2f) },
	};

	for (uint i = 0; i < 4; i++)
	{
		Scene::Prop& wall = s->add_prop(cube_mesh, walls[i].pos, 0, walls[i].scale);
		wall.mat_a = vec4(.255f, .245f, .225f, 0.f);
		wall.mat_b = vec4(.86f, 1.f, 0.f, 0.f);
		wall.phys_id = s->phys.add_box(wall.pos, walls[i].scale * .5f, 0);
	}
}

void hub_add_vehicle(Scene* s, uint cube_mesh, uint wheel_mesh, float p)
{
	// parked at the yard, pointed at the sea. C switches to the chase view
	vec3 car_spawn = vec3(7.f, p + 1.2f, 3.f);
	if (!s->phys.add_vehicle(car_spawn, PI)) return;

	Scene::Prop& chassis = s->add_prop(cube_mesh, car_spawn, PI, vec3(2.f, 1.f, 4.f));
	chassis.phys_id = s->phys.vehicle.chassis_id;
	chassis.mat_a = vec4(.24f, .08f, .055f, .18f);
	chassis.mat_b = vec4(.34f, 1.f, 0.f, 0.f);

	for (uint i = 0; i < 4; i++)
	{
		Scene::Prop& wheel = s->add_prop(wheel_mesh, car_spawn, 0, 1.f);
		wheel.wheel_index = i;
		wheel.mat_a = vec4(.028f, .028f, .032f, 0.f);
		wheel.mat_b = vec4(.88f, 1.f, 0.f, 0.f);
	}
}

void hub_add_player(Scene* s, GameRenderer* renderer, float p)
{
	// on foot with the test pistol. the bullet mesh is added before the
	// simulated surfaces, so the shared dynamic mark stays behind them
	uint gun_mesh = renderer->add_anim_mesh(WEAPON_SETUP[WEAPON_VIEW_MODEL].mesh);
	uint gun_clip = 0;
	renderer->animloader.load(WEAPON_SETUP[WEAPON_VIEW_MODEL].clip, &gun_clip);
	if (gun_mesh && gun_clip)
		weapon_slice(WEAPON_VIEW_MODEL, &renderer->animloader.clips[gun_clip]);

	s->player.init(renderer, &s->phys, vec3(0.f, p + 1.3f, 10.f), gun_mesh, gun_clip);
	s->chase_distance = 8.f;
	s->chase_height   = 3.4f;
}

void hub_add_signs(Scene* s, float p)
{
	struct Sign { vec3 pos; const char* text; };
	const Sign signs[4] = {
		{ vec3(-16.f, p + 5.2f, -14.f), "ARMORY" },
		{ vec3(  0.f, p + 2.3f, HUB_CISTERN_Z), "CISTERN" },
		{ vec3( 14.f, p + 3.1f,  -9.f), "SUPPLY" },
		{ vec3( 18.f, p + 4.4f, -18.f), "SIGNAL" },
	};

	for (uint i = 0; i < 4; i++)
	{
		Scene::Prop& sign = s->add_prop(0, signs[i].pos, 0, 1.f);
		snprintf(sign.label, sizeof(sign.label), "%s", signs[i].text);
	}
}

void scene_hub(Scene* s, GameRenderer* renderer)
{
	// a catch floor far below : the island's heightfield is the real ground
	s->begin(renderer, vec3(0, 12.f, 20.f), -PI / 2, -.12f, 512.f, 256.f, 4, -30.f);
	s->props[0].mesh_id = 0;
	hub_apply_lighting(s);

	hub_build_terrain(s, renderer);
	float p = hub_view.plateau;

	// shared base meshes
	uint cube_mesh     = renderer->add_mesh(MeshFactory::generate_cube(1.f));
	uint cylinder_mesh = renderer->add_mesh(MeshFactory::generate_cylinder(.5f, 1.f, 20));
	uint wheel_mesh    = renderer->add_mesh(MeshFactory::generate_cylinder(.5f, .4f, 24));
	uint sphere_mesh   = renderer->add_mesh("assets/meshes/sphere.mesh_uv");
	uint ammo_mesh     = renderer->add_mesh("assets/meshes/ammo.mesh_uv");
	uint palm_mesh     = renderer->add_mesh("assets/meshes/palm_tree.mesh_uv");
	uint path_mesh     = renderer->add_mesh(MeshFactory::generate_plane(1.f, 1,
	                                                                    MeshFactory::PLANE_XZ));

	hub_add_road(s, path_mesh, p);
	hub_add_armory(s, renderer, cube_mesh, p);
	hub_add_supply(s, cube_mesh, cylinder_mesh, ammo_mesh, p);
	hub_add_signal(s, cube_mesh, sphere_mesh, p);
	hub_add_palms(s, palm_mesh, p);
	hub_add_cistern(s, cube_mesh, p);

	hub_add_vehicle(s, cube_mesh, wheel_mesh, p);
	hub_add_player(s, renderer, p);

	// dynamic surfaces share one mark : a rewind drops both, then they are
	// appended in the same order, so their mesh slots never move
	DrawBuffer& buffer = renderer->buffer(MODE_STATIC_UV);
	hub_view.mesh_mark = buffer.num_meshes;
	hub_view.geom_mark = buffer.geom_size;
	hub_view.indx_mark = buffer.indx_size;

	water_build_content(s, renderer, WATER_LEVEL);
	shallow_build_content(s, renderer, vec3(0.f, 0.f, HUB_CISTERN_Z),
	                      p + HUB_CISTERN_LIFT);

	water_view.mesh_mark = shallow_view.mesh_mark = hub_view.mesh_mark;
	water_view.geom_mark = shallow_view.geom_mark = hub_view.geom_mark;
	water_view.indx_mark = shallow_view.indx_mark = hub_view.indx_mark;

	hub_add_signs(s, p);

	s->finish(renderer, "hub");
	terrain_resolve_props(s);
	water_resolve_props(s);
	shallow_resolve_props(s);

	s->water_dirty = false;
	s->water_spectrum = false;
	s->shallow.dirty = false;
}

void hub_seat_camera(Scene* s)
{
	vec3 pos;
	mat3 rot;
	if (!s->phys.vehicle.chassis_id ||
	    !s->phys.get_transform(s->phys.vehicle.chassis_id, &pos, &rot)) return;

	vec3 forward = rot[2];
	forward.y = 0.f;
	if (length(forward) < .01f) forward = vec3(0, 0, 1);
	else forward = normalize(forward);

	vec3 target = pos + forward * 3.f + vec3(0, 1.f, 0);
	s->vehicle_view.position = pos - forward * s->chase_distance +
	                           vec3(0, s->chase_height, 0);
	s->vehicle_view.front = normalize(target - s->vehicle_view.position);
	s->vehicle_view.up = vec3(0, 1, 0);
}

void scene_hub_key(Scene* s, GameWindow* window)
{
	if (window->focus_status != GAME) return;
	if (ImGui::GetIO().WantCaptureKeyboard) return;

	if (window->keys.C.is_pressed && !window->keys.C.was_pressed)
	{
		s->car_camera = !s->car_camera;
		if (s->car_camera)
		{
			s->player.flycam = false;
			hub_seat_camera(s);
		}
	}
}

// the sea and the cistern both live in the static buffer : one rewind covers
// whichever of them changed, then both are appended again at their own ids
void hub_scene_update(Scene* s, GameRenderer* renderer, float dt)
{
	bool water_changed   = water_simulate(s, renderer, dt);
	bool shallow_changed = shallow_simulate(s, dt);

	if (water_changed || shallow_changed)
	{
		DrawBuffer& buffer = renderer->buffer(MODE_STATIC_UV);
		buffer.rewind(hub_view.mesh_mark, hub_view.geom_mark, hub_view.indx_mark);
		water_append_geometry(s, renderer);
		shallow_append_geometry(s, renderer);
	}

	if (water_config.size != water_view.tile_size) water_place_tiles(s);

	water_apply_materials(s);
	shallow_apply_material(s);
}

void hud_text_center(ImDrawList* dl, float size, float center_x, float y, ImU32 color, const char* text);

// ---- water : title, sea state and the tiles in view
void scene_water_ux(Scene* s, GameWindow* window)
{
	ImDrawList* dl = ImGui::GetBackgroundDrawList();
	float scale = window->screen_height / 1080.f;

	float panel = ux_panel_visible ? ux_panel_width(window) : 0.f;
	float cx = (window->screen_width + panel) * .5f;

	ImU32 title = IM_COL32(235, 238, 243, 225);
	ImU32 dim   = IM_COL32(170, 178, 190, 200);

	char sub[160];
	snprintf(sub, sizeof(sub),
	         "fft %d x %d  |  patch %.0f m  |  wind %.1f m/s  |  Hs %.1f m  |  %u x %u tiles",
	         s->water.n, s->water.n, water_config.size, water_config.wind_speed,
	         water_config.amplitude * 4.f, WATER_TILES, WATER_TILES);

	hud_text_center(dl, 24.f * scale, cx, 36.f * scale, title, "OCEAN");
	hud_text_center(dl, 14.f * scale, cx, 70.f * scale, dim, sub);
	hud_text_center(dl, 13.f * scale, cx, window->screen_height - 42.f * scale, dim,
	                "WASD fly  |  mouse look  |  T console  |  ocean controls under Scenes");
}

// ---- shallow : title, patch and drop state
void scene_shallow_ux(Scene* s, GameWindow* window)
{
	ImDrawList* dl = ImGui::GetBackgroundDrawList();
	float scale = window->screen_height / 1080.f;

	float panel = ux_panel_visible ? ux_panel_width(window) : 0.f;
	float cx = (window->screen_width + panel) * .5f;

	ImU32 title = IM_COL32(235, 238, 243, 225);
	ImU32 dim   = IM_COL32(170, 178, 190, 200);

	float spacing = shallow_config.size / (float)(s->shallow.n - 1) * 100.f;

	char sub[160];
	snprintf(sub, sizeof(sub),
	         "fd %d x %d  |  %.0f m patch  |  %.1f m/s  |  %.0f cm grid  |  drops %.0f s",
	         s->shallow.n, s->shallow.n, shallow_config.size, shallow_config.speed,
	         spacing, shallow_config.drop_interval);

	hud_text_center(dl, 24.f * scale, cx, 36.f * scale, title, "SHALLOW");
	hud_text_center(dl, 14.f * scale, cx, 70.f * scale, dim, sub);
	hud_text_center(dl, 13.f * scale, cx, window->screen_height - 42.f * scale, dim,
	                "WASD fly  |  mouse look  |  T console  |  drops + damping under Scenes");
}

// live hinge and motor controls : the builder fills the handles, the debug
// tab edits the values and pushes them into the constraints
struct Physics_Controls
{
	btHingeConstraint* arm;
	btHingeConstraint* seesaw;
	btHingeConstraint* rotor;
	btHingeConstraint* door;

	uint arm_body, rotor_body, door_body;
	uint chain_end, cradle_first, ccd_on, ccd_off;

	float arm_limit, arm_push, arm_damping;
	float seesaw_limit;
	float rotor_speed, rotor_impulse;
	float door_speed, door_impulse, door_open_angle, door_damping, door_target;
	float gravity;
	int   solver_iters;
	bool  rotor_on, door_on;
};

static Physics_Controls physics_controls;

// a hair of clearance : the motor should not be born against its closed stop
const float DOOR_CLOSED_LIMIT = -.01f;

void physics_default_settings()
{
	physics_controls.arm_limit = 60.f * PI / 180.f;
	physics_controls.arm_push = 3.f;
	physics_controls.arm_damping = .05f;
	physics_controls.seesaw_limit = .3f;

	physics_controls.rotor_speed = 3.f;
	physics_controls.rotor_impulse = 8.f;
	physics_controls.rotor_on = true;

	physics_controls.door_speed = 2.f;
	physics_controls.door_impulse = 30.f;
	physics_controls.door_open_angle = 90.f;
	physics_controls.door_damping = .2f;
	physics_controls.door_target = 0.f;
	physics_controls.door_on = false;

	physics_controls.gravity = 9.8f;
	physics_controls.solver_iters = 10;
}

void physics_apply_settings(Scene* s)
{
	if (!s->phys.world) return;

	if (physics_controls.arm)
		physics_controls.arm->setLimit(-physics_controls.arm_limit, physics_controls.arm_limit);
	if (physics_controls.seesaw)
		physics_controls.seesaw->setLimit(-physics_controls.seesaw_limit, physics_controls.seesaw_limit);
	if (physics_controls.door)
		physics_controls.door->setLimit(DOOR_CLOSED_LIMIT, physics_controls.door_open_angle * PI / 180.f);

	if (physics_controls.rotor)
	{
		physics_controls.rotor->enableMotor(physics_controls.rotor_on);
		physics_controls.rotor->setMotorTargetVelocity(physics_controls.rotor_speed);
		physics_controls.rotor->setMaxMotorImpulse(physics_controls.rotor_impulse);
	}

	if (physics_controls.door)
	{
		physics_controls.door->enableMotor(physics_controls.door_on);
		physics_controls.door->setMotorTargetVelocity(physics_controls.door_target * physics_controls.door_speed);
		physics_controls.door->setMaxMotorImpulse(physics_controls.door_impulse);
	}

	// a motor does not wake a sleeping body : do it while the motor is driving
	if (physics_controls.rotor_body && physics_controls.rotor_body < s->phys.num_bodies &&
	    physics_controls.rotor_on && physics_controls.rotor_speed != 0.f)
		s->phys.bodies[physics_controls.rotor_body].body->activate(true);

	if (physics_controls.door_body && physics_controls.door_body < s->phys.num_bodies &&
	    physics_controls.door_on && physics_controls.door_target != 0.f)
		s->phys.bodies[physics_controls.door_body].body->activate(true);

	if (physics_controls.arm_body && physics_controls.arm_body < s->phys.num_bodies)
		s->phys.bodies[physics_controls.arm_body].body->setDamping(0.f, physics_controls.arm_damping);

	if (physics_controls.door_body && physics_controls.door_body < s->phys.num_bodies)
		s->phys.bodies[physics_controls.door_body].body->setDamping(0.f, physics_controls.door_damping);

	s->phys.world->setGravity(btVector3(0, -physics_controls.gravity, 0));
	s->phys.world->getSolverInfo().m_numIterations = physics_controls.solver_iters;
}

// station order : the number in the world sign is the number in the panel
enum Physics_Station {
	PST_SHAPES = 0, PST_STACK, PST_DOMINOES, PST_RAGDOLL,
	PST_HINGE, PST_CONE, PST_CHAIN, PST_SEESAW,
	PST_MOTOR, PST_CRADLE, PST_RAMP, PST_CCD, PST_DOOR, PST_COUNT
};

static const char* PHYSICS_STATIONS[PST_COUNT] = {
	"shapes", "stack", "dominoes", "ragdoll",
	"hinge", "cone twist", "chain", "see-saw",
	"motor", "newton's cradle", "ramp", "ccd", "door",
};

static const vec3 PHYSICS_SIGN_AT[PST_COUNT] = {
	vec3(-12.f, 4.f, 4.f),   vec3(-4.f, 4.f, 4.f),    vec3(4.4f, 4.f, 4.f),
	vec3(12.f, 5.5f, 4.f),   vec3(-14.f, 4.f, -6.f),  vec3(-7.f, 4.f, -6.f),
	vec3(0.f, 6.f, -6.f),    vec3(7.f, 3.f, -6.f),    vec3(14.f, 4.f, -6.f),
	vec3(-9.f, 5.8f, -16.f), vec3(-1.f, 4.f, -16.f),  vec3(9.f, 5.f, -16.f),
	vec3(15.85f, 4.2f, -16.f),
};

void station_label(uint station)
{
	ImGui::TextDisabled("%u  %s", station + 1, PHYSICS_STATIONS[station]);
}

// ---- physics : one labelled station per rigid-body test
void scene_physics(Scene* s, GameRenderer* renderer)
{
	physics_controls = {};
	physics_default_settings();

	s->begin(renderer, vec3(0, 3.f, 14.f), -PI / 2, -0.16f, 80.f, 40.f);
	s->use_lighting(LIGHT_NOON);

	// shared meshes : every station draws from this small set
	uint cube_mesh     = renderer->add_mesh(MeshFactory::generate_cube(1.f), "assets/textures/checker.bmp");
	uint sphere_mesh   = renderer->add_mesh(MeshFactory::generate_uv_sphere(1.f, 24, 16));
	uint capsule_mesh  = renderer->add_mesh(MeshFactory::generate_capsule(.5f, 1.4f, 20, 10));
	uint cylinder_mesh = renderer->add_mesh(MeshFactory::generate_cylinder(.6f, 2.f, 24));
	uint cone_mesh     = renderer->add_mesh(MeshFactory::generate_cone(.6f, 1.6f, 24));
	uint bob_mesh      = renderer->add_mesh(MeshFactory::generate_capsule(.4f, 1.6f, 20, 10));

	// numbered station signs : world text with no mesh behind it
	for (uint i = 0; i < PST_COUNT; i++)
	{
		Scene::Prop& sign = s->add_prop(0, PHYSICS_SIGN_AT[i], 0, 1.f);
		snprintf(sign.label, sizeof(sign.label), "%u  %s", i + 1, PHYSICS_STATIONS[i]);
	}

	// shapes : one of every primitive, dropped from the same height
	{
		Scene::Prop& box = s->add_prop(cube_mesh, vec3(-15.f, 3.f, 4.f), 0, 1.f);
		box.phys_id = s->phys.add_box(box.pos, vec3(.5f), 1.f);

		Scene::Prop& sphere = s->add_prop(sphere_mesh, vec3(-13.4f, 3.f, 4.f), 0, .5f);
		sphere.phys_id = s->phys.add_sphere(sphere.pos, .5f, 1.f);

		Scene::Prop& capsule = s->add_prop(capsule_mesh, vec3(-11.8f, 3.f, 4.f), 0, 1.f);
		capsule.phys_id = s->phys.add_capsule(capsule.pos, .5f, 1.4f, 2.f);

		Scene::Prop& cylinder = s->add_prop(cylinder_mesh, vec3(-10.2f, 3.f, 4.f), 0, 1.f);
		cylinder.phys_id = s->phys.add_cylinder(cylinder.pos, vec3(.6f, 1.f, .6f), 1.f);

		Scene::Prop& cone = s->add_prop(cone_mesh, vec3(-8.6f, 3.f, 4.f), 0, 1.f);
		cone.phys_id = s->phys.add_cone(cone.pos, .6f, 1.6f, 1.f);
	}

	// stack : 3-2-1, rows at resting heights
	vec3 stack[6] = {
		vec3(-5.5f, .5f, 4), vec3(-4.f, .5f, 4), vec3(-2.5f, .5f, 4),
		vec3(-4.75f, 1.5f, 4), vec3(-3.25f, 1.5f, 4),
		vec3(-4.f, 2.5f, 4),
	};
	for (uint i = 0; i < 6; i++)
	{
		Scene::Prop& box = s->add_prop(cube_mesh, stack[i], 0, 1.f);
		box.phys_id = s->phys.add_box(box.pos, vec3(.5f), 1.f);
	}

	// dominoes : the starter ball rolls into the first plate
	{
		Scene::Prop& ball = s->add_prop(sphere_mesh, vec3(1.2f, .25f, 4.f), 0, .25f);
		ball.phys_id = s->phys.add_sphere(ball.pos, .25f, .5f);
		s->phys.apply_impulse(ball.phys_id, vec3(2.f, 0, 0));
	}
	for (uint i = 0; i < 6; i++)
	{
		Scene::Prop& domino = s->add_prop(cube_mesh, vec3(2.4f + i * .8f, .9f, 4.f), 0,
		                                 vec3(.12f, 1.8f, 1.f));
		domino.phys_id = s->phys.add_box(domino.pos, vec3(.06f, .9f, .5f), .4f);
	}

	// ragdoll : linked capsules on a platform
	Scene::Prop& platform = s->add_prop(cube_mesh, vec3(12.f, .5f, 4.f), 0, vec3(7.f, 1.f, 7.f));
	platform.phys_id = s->phys.add_box(platform.pos, vec3(3.5f, .5f, 3.5f), 0);

	uint ragdoll = s->phys.add_ragdoll(vec3(12.f, 1.02f, 4.f), 1.6f);
	for (uint i = 0; ragdoll && i < 11; i++)
	{
		uint id = ragdoll + i;
		vec3 d = s->phys.bodies[id].dims; // (radius, cylinder height)
		Scene::Prop& part = s->add_prop(renderer->add_mesh(MeshFactory::generate_capsule(d.x, d.y, 20, 10)),
		                                vec3(0), 0, 1.f);
		part.phys_id = id;
	}

	// hinge : a plank hinged at the top of a post, swinging under gravity
	{
		Scene::Prop& post = s->add_prop(cube_mesh, vec3(-14.f, 1.5f, -6.6f), 0, vec3(.3f, 3.f, .3f));
		post.phys_id = s->phys.add_box(post.pos, vec3(.15f, 1.5f, .15f), 0);
		s->add_prop(cube_mesh, vec3(-14.f, 3.f, -6.3f), 0, vec3(.15f, .15f, .6f)); // mount

		Scene::Prop& arm = s->add_prop(cube_mesh, vec3(-12.6f, 3.f, -6.f), 0, vec3(2.8f, .18f, .18f));
		arm.phys_id = s->phys.add_box(arm.pos, vec3(1.4f, .09f, .09f), 2.f);
		physics_controls.arm_body = arm.phys_id;

		physics_controls.arm = s->phys.add_hinge(
			*s->phys.bodies[post.phys_id].body, *s->phys.bodies[arm.phys_id].body,
			joint_frame(1.f, vec3(0), vec3(0, 1.5f, .6f)),
			joint_frame(1.f, vec3(0), vec3(-1.4f, 0, 0)),
			-physics_controls.arm_limit, physics_controls.arm_limit);
	}

	// cone twist : a heavy bag hangs from the post top on a cone joint
	{
		Scene::Prop& post = s->add_prop(cube_mesh, vec3(-7.f, 1.5f, -6.6f), 0, vec3(.3f, 3.f, .3f));
		post.phys_id = s->phys.add_box(post.pos, vec3(.15f, 1.5f, .15f), 0);
		s->add_prop(cube_mesh, vec3(-7.f, 3.f, -6.3f), 0, vec3(.15f, .15f, .6f)); // mount

		Scene::Prop& bag = s->add_prop(cube_mesh, vec3(-7.f, 2.3f, -6.f), 0, vec3(.8f, 1.4f, .8f));
		bag.phys_id = s->phys.add_box(bag.pos, vec3(.4f, .7f, .4f), 5.f);

		s->phys.add_cone(*s->phys.bodies[post.phys_id].body, *s->phys.bodies[bag.phys_id].body,
		                 joint_frame(1.f, vec3(0, 0, -PI * .5f), vec3(0, 1.5f, .6f)),
		                 joint_frame(1.f, vec3(0, 0, -PI * .5f), vec3(0, .7f, 0)),
		                 .6f, .4f);
	}

	// chain : five links, hinged end to end from a post
	{
		Scene::Prop& post = s->add_prop(cube_mesh, vec3(0.f, 2.5f, -6.6f), 0, vec3(.3f, 5.f, .3f));
		post.phys_id = s->phys.add_box(post.pos, vec3(.15f, 2.5f, .15f), 0);
		s->add_prop(cube_mesh, vec3(0.f, 5.f, -6.3f), 0, vec3(.15f, .15f, .6f)); // mount

		uint prev = post.phys_id;
		btTransform prev_frame = joint_frame(1.f, vec3(0), vec3(0, 2.5f, .6f)); // post top
		float y = 5.f - .4f;

		for (uint i = 0; i < 5; i++)
		{
			Scene::Prop& link = s->add_prop(cube_mesh, vec3(0.f, y, -6.f), 0, vec3(.24f, .8f, .24f));
			link.phys_id = s->phys.add_box(link.pos, vec3(.12f, .4f, .12f), .8f);
			if (i == 4) physics_controls.chain_end = link.phys_id;

			s->phys.add_hinge(*s->phys.bodies[prev].body, *s->phys.bodies[link.phys_id].body,
			                  prev_frame, joint_frame(1.f, vec3(0), vec3(0, .4f, 0)),
			                  -2.8f, 2.8f);

			prev = link.phys_id;
			prev_frame = joint_frame(1.f, vec3(0), vec3(0, -.4f, 0));
			y -= .8f;
		}
	}

	// see-saw : a loaded plank pivots on a low post
	{
		Scene::Prop& pivot = s->add_prop(cube_mesh, vec3(7.f, .5f, -6.f), 0, vec3(.6f, 1.f, .6f));
		pivot.phys_id = s->phys.add_box(pivot.pos, vec3(.3f, .5f, .3f), 0);

		Scene::Prop& plank = s->add_prop(cube_mesh, vec3(7.f, 1.06f, -6.f), 0, vec3(6.f, .12f, 1.4f));
		plank.phys_id = s->phys.add_box(plank.pos, vec3(3.f, .06f, .7f), 4.f);

		physics_controls.seesaw = s->phys.add_hinge(
			*s->phys.bodies[pivot.phys_id].body, *s->phys.bodies[plank.phys_id].body,
			joint_frame(1.f, vec3(0), vec3(0, .5f, 0)),
			joint_frame(1.f, vec3(0), vec3(0, -.06f, 0)),
			-physics_controls.seesaw_limit, physics_controls.seesaw_limit);

		vec3 loads[2] = { vec3(4.6f, 1.62f, -6.f), vec3(5.8f, 1.62f, -6.f) };
		for (uint i = 0; i < 2; i++)
		{
			Scene::Prop& crate = s->add_prop(cube_mesh, loads[i], 0, 1.f);
			crate.phys_id = s->phys.add_box(crate.pos, vec3(.5f), 2.f);
		}
	}

	// motor : a hinge with a target velocity, running continuously
	{
		Scene::Prop& post = s->add_prop(cube_mesh, vec3(14.f, 1.5f, -6.f), 0, vec3(.3f, 3.f, .3f));
		post.phys_id = s->phys.add_box(post.pos, vec3(.15f, 1.5f, .15f), 0);

		Scene::Prop& rotor = s->add_prop(cube_mesh, vec3(15.2f, 3.1f, -6.f), 0, vec3(2.4f, .16f, .16f));
		rotor.phys_id = s->phys.add_box(rotor.pos, vec3(1.2f, .08f, .08f), 2.f);
		physics_controls.rotor_body = rotor.phys_id;

		physics_controls.rotor = s->phys.add_hinge(
			*s->phys.bodies[post.phys_id].body, *s->phys.bodies[rotor.phys_id].body,
			joint_frame(1.f, vec3(-PI * .5f, 0, 0), vec3(0, 1.6f, 0)),
			joint_frame(1.f, vec3(-PI * .5f, 0, 0), vec3(-1.2f, 0, 0)),
			-1000.f, 1000.f, physics_controls.rotor_speed, physics_controls.rotor_impulse);
	}

	// door : a motorised hinge with a real travel limit
	{
		Scene::Prop& post = s->add_prop(cube_mesh, vec3(14.5f, 2.f, -16.f), 0, vec3(.3f, 4.f, .3f));
		post.phys_id = s->phys.add_box(post.pos, vec3(.15f, 2.f, .15f), 0);

		Scene::Prop& door = s->add_prop(cube_mesh, vec3(15.85f, 1.6f, -16.f), 0, vec3(2.4f, 3.f, .16f));
		door.phys_id = s->phys.add_box(door.pos, vec3(1.2f, 1.5f, .08f), 8.f);
		physics_controls.door_body = door.phys_id;

		physics_controls.door = s->phys.add_hinge(
			*s->phys.bodies[post.phys_id].body, *s->phys.bodies[door.phys_id].body,
			joint_frame(1.f, vec3(-PI * .5f, 0, 0), vec3(.15f, 0, 0)),
			joint_frame(1.f, vec3(-PI * .5f, 0, 0), vec3(-1.2f, .4f, 0)),
			DOOR_CLOSED_LIMIT, physics_controls.door_open_angle * PI / 180.f);
	}

	// ccd : two fast spheres at a thin wall, one swept, one discrete
	{
		Scene::Prop& wall = s->add_prop(cube_mesh, vec3(9.f, 2.f, -16.f), 0, vec3(.1f, 4.f, 2.4f));
		wall.phys_id = s->phys.add_box(wall.pos, vec3(.05f, 2.f, 1.2f), 0);

		const char* names[2] = { "ccd on", "ccd off" };
		for (uint i = 0; i < 2; i++)
		{
			Scene::Prop& shot = s->add_prop(sphere_mesh, vec3(3.f, 2.f, -16.6f + i * 1.2f), 0, .18f);
			shot.phys_id = s->phys.add_sphere(shot.pos, .18f, 1.f);
			snprintf(shot.label, sizeof(shot.label), "%s", names[i]);
			if (i == 0)
			{
				physics_controls.ccd_on = shot.phys_id;
				s->phys.enable_ccd(shot.phys_id);
			}
			else physics_controls.ccd_off = shot.phys_id;
			s->phys.apply_impulse(shot.phys_id, vec3(150.f, 0, 0));
		}
	}

	// newton's cradle : five hanging capsules transfer a kick along the row
	{
		Scene::Prop& beam = s->add_prop(cube_mesh, vec3(-9.f, 4.6f, -16.f), 0, vec3(4.4f, .2f, .3f));
		beam.phys_id = s->phys.add_box(beam.pos, vec3(2.2f, .1f, .15f), 0);

		uint first = 0;
		for (uint i = 0; i < 5; i++)
		{
			float x = -10.4f + i * .8f;
			Scene::Prop& bob = s->add_prop(bob_mesh, vec3(x, 3.3f, -16.f), 0, 1.f);
			bob.phys_id = s->phys.add_capsule(bob.pos, .4f, 1.6f, 1.f);
			if (i == 0)
			{
				first = bob.phys_id;
				physics_controls.cradle_first = bob.phys_id;
			}

			s->phys.add_hinge(*s->phys.bodies[beam.phys_id].body, *s->phys.bodies[bob.phys_id].body,
			                  joint_frame(1.f, vec3(0), vec3(x + 9.f, -.1f, 0)),
			                  joint_frame(1.f, vec3(0), vec3(0, 1.2f, 0)),
			                  -PI * .99f, PI * .99f);
		}
		if (first) s->phys.apply_impulse(first, vec3(6.f, 0, 0));
	}

	// ramp : a sphere and a cylinder roll down the inclined plane
	{
		Scene::Prop& ramp = s->add_prop(cube_mesh, vec3(-1.f, .72f, -16.f), 0, vec3(8.f, .4f, 4.f));
		ramp.rot = mat3(glm::rotate(mat4(1), -.18f, vec3(0, 0, 1)));
		ramp.phys_id = s->phys.add_box(ramp.pos, vec3(4.f, .2f, 2.f), 0, vec3(0, 0, -.18f));

		Scene::Prop& ball = s->add_prop(sphere_mesh, vec3(-4.4f, 2.2f, -16.f), 0, .5f);
		ball.phys_id = s->phys.add_sphere(ball.pos, .5f, 1.f);

		Scene::Prop& roller = s->add_prop(cylinder_mesh, vec3(-4.4f, 2.6f, -14.8f), 0, .5f);
		roller.phys_id = s->phys.add_cylinder(roller.pos, vec3(.3f, .5f, .3f), 1.f);
	}

	physics_apply_settings(s);

	s->player.init(renderer, &s->phys, vec3(0.f, 2.5f, 14.f), 0, 0);
	s->finish(renderer, "physics");
}

// ---- vehicles : the car, two ramps and things to hit
void scene_vehicles(Scene* s, GameRenderer* renderer)
{
	s->begin(renderer, vec3(0, 10, -18), PI / 2, -0.22f, 240.f, 120.f);
	s->use_lighting(LIGHT_SUNSET);

	// warm dirt floor : the grid is a developer texture
	Scene::Prop& floor = s->props[0];
	floor.mat_a = vec4(.16f, .13f, .10f, 0.f);
	floor.mat_b = vec4(.92f, 1.f, 0.f, 0.f);

	uint plain_mesh   = renderer->add_mesh(MeshFactory::generate_cube(1.f));
	uint checker_mesh = renderer->add_mesh(MeshFactory::generate_cube(1.f), "assets/textures/checker.bmp");
	uint road_mesh    = renderer->add_mesh(MeshFactory::generate_plane(1.f, 1, MeshFactory::PLANE_XZ));
	uint wheel_mesh   = renderer->add_mesh(MeshFactory::generate_cylinder(.5f, .4f, 24));
	uint cone_mesh    = renderer->add_mesh(MeshFactory::generate_cone(.5f, 1.5f, 24));

	// road, edge lines and start line
	Scene::Prop& road = s->add_prop(road_mesh, vec3(0, .01f, 54.f), 0, vec3(12.f, 1.f, 128.f));
	road.mat_a = vec4(.035f, .038f, .045f, 0.f);
	road.mat_b = vec4(.48f, 1.f, 0.f, 0.f);
	road.phys_id = s->phys.add_box(road.pos, vec3(6.f, .01f, 64.f), 0);
	s->phys.bodies[road.phys_id].body->setFriction(1.f);

	for (uint i = 0; i < 2; i++)
	{
		float x = i ? 6.1f : -6.1f;
		Scene::Prop& edge = s->add_prop(plain_mesh, vec3(x, .02f, 54.f), 0, vec3(.12f, .02f, 128.f));
		edge.mat_a = vec4(.82f, .78f, .66f, 0.f);
		edge.mat_b = vec4(.55f, 1.f, 0.f, 0.f);
	}

	Scene::Prop& start_line = s->add_prop(plain_mesh, vec3(0, .02f, 0.f), 0, vec3(12.f, .02f, .25f));
	start_line.mat_a = vec4(.90f, .86f, .72f, 0.f);
	start_line.mat_b = vec4(.55f, 1.f, 0.f, 0.f);

	// jump ramp : low edge at the start side, high edge toward the landing
	const float tilt = .18f;
	Scene::Prop& ramp = s->add_prop(plain_mesh, vec3(0, .38f, 40.f), 0, vec3(8.f, .4f, 6.f));
	ramp.rot = mat3(glm::rotate(mat4(1), -tilt, vec3(1, 0, 0)));
	ramp.phys_id = s->phys.add_box(ramp.pos, vec3(4.f, .2f, 3.f), 0, vec3(-tilt, 0, 0));
	s->phys.bodies[ramp.phys_id].body->setFriction(1.f);
	ramp.mat_a = vec4(.22f, .20f, .16f, 0.f);
	ramp.mat_b = vec4(.62f, 1.f, 0.f, 0.f);

	Scene::Prop& support = s->add_prop(plain_mesh, vec3(0, .36f, 42.4f), 0, vec3(8.f, .72f, .8f));
	support.mat_a = vec4(.10f, .095f, .08f, 0.f);
	support.mat_b = vec4(.8f, 1.f, 0.f, 0.f);

	// slalom : alternate sides after the jump
	vec3 cone_spots[5] = {
		vec3(-2.5f, .75f, 70.f), vec3(2.5f, .75f, 80.f), vec3(-2.5f, .75f, 90.f),
		vec3(2.5f, .75f, 100.f), vec3(-2.5f, .75f, 108.f),
	};
	for (uint i = 0; i < 5; i++)
	{
		Scene::Prop& cone = s->add_prop(cone_mesh, cone_spots[i], 0, 1.f);
		cone.phys_id = s->phys.add_cone(cone.pos, .5f, 1.5f, .5f);
		cone.mat_a = vec4(.90f, .32f, .04f, 0.f);
		cone.mat_b = vec4(.52f, 1.f, 0.f, 0.f);
	}

	// a crate stack off the left edge
	vec3 crates[3] = { vec3(-8.5f, .5f, 15.f), vec3(-9.5f, .5f, 15.8f), vec3(-9.f, 1.5f, 15.4f) };
	for (uint i = 0; i < 3; i++)
	{
		Scene::Prop& crate = s->add_prop(checker_mesh, crates[i], 0, 1.f);
		crate.phys_id = s->phys.add_box(crate.pos, vec3(.5f), 1.f);
		crate.mat_b = vec4(.6f, 1.f, 1.f, 0.f);
	}

	// finish gate
	for (uint i = 0; i < 2; i++)
	{
		float x = i ? 5.5f : -5.5f;
		Scene::Prop& post = s->add_prop(checker_mesh, vec3(x, 2.f, 112.f), 0, vec3(.4f, 4.f, .4f));
		post.phys_id = s->phys.add_box(post.pos, vec3(.2f, 2.f, .2f), 0);
		post.mat_b = vec4(.6f, 1.f, 1.f, 0.f);
	}

	Scene::Prop& beam = s->add_prop(checker_mesh, vec3(0, 4.f, 112.f), 0, vec3(11.4f, .4f, .4f));
	beam.mat_b = vec4(.6f, 1.f, 1.f, 0.f);

	// course signs
	struct Sign { vec3 at; const char* text; };
	const Sign signs[] = {
		{ vec3(0, 4.f, 0.f),    "START" },
		{ vec3(0, 5.f, 40.f),   "JUMP" },
		{ vec3(0, 5.f, 88.f),   "SLALOM" },
		{ vec3(0, 5.4f, 112.f), "FINISH" },
	};
	for (uint i = 0; i < (uint)(sizeof(signs) / sizeof(signs[0])); i++)
	{
		Scene::Prop& sign = s->add_prop(0, signs[i].at, 0, 1.f);
		snprintf(sign.label, sizeof(sign.label), "%s", signs[i].text);
	}

	// the car
	if (s->phys.add_vehicle(vec3(0, 1.f, 0)))
	{
		Scene::Prop& chassis = s->add_prop(plain_mesh, vec3(0), 0, vec3(2.f, 1.f, 4.f));
		chassis.phys_id = s->phys.vehicle.chassis_id;
		chassis.mat_a = vec4(.62f, .12f, .08f, .18f);
		chassis.mat_b = vec4(.30f, 1.f, 0.f, 0.f);

		for (uint i = 0; i < 4; i++)
		{
			Scene::Prop& wheel = s->add_prop(wheel_mesh, vec3(0), 0, 1.f);
			wheel.wheel_index = i;
			wheel.mat_a = vec4(.03f, .03f, .035f, 0.f);
			wheel.mat_b = vec4(.88f, 1.f, 0.f, 0.f);
		}
	}

	s->player.init(renderer, &s->phys, vec3(-16.f, 2.f, -8.f), 0, 0);

	// chase camera : behind the car, looking down the course
	s->car_camera = true;
	s->chase_distance = 6.f;
	s->chase_height = 2.4f;

	vec3 car_pos;
	mat3 car_rot;
	if (s->phys.vehicle.chassis_id &&
	    s->phys.get_transform(s->phys.vehicle.chassis_id, &car_pos, &car_rot))
	{
		vec3 forward = car_rot[2];
		forward.y = 0.f;
		if (length(forward) < .01f) forward = vec3(0, 0, 1);
		else forward = normalize(forward);

		s->vehicle_view.position = car_pos - forward * s->chase_distance + vec3(0, s->chase_height, 0);
		s->vehicle_view.front = normalize(car_pos + vec3(0, 1.f, 0) - s->vehicle_view.position);
		s->vehicle_view.up = vec3(0, 1, 0);
	}

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

// ---- hub : title, the walk/drive hint and the live vehicle or ammo readout
void scene_hub_ux(Scene* s, GameWindow* window)
{
	ImDrawList* dl = ImGui::GetBackgroundDrawList();
	float scale = window->screen_height / 1080.f;

	float panel = ux_panel_visible ? ux_panel_width(window) : 0.f;
	float cx = (window->screen_width + panel) * .5f;
	float right = (float)window->screen_width - 48.f * scale;

	ImU32 title = IM_COL32(235, 238, 243, 225);
	ImU32 dim   = IM_COL32(170, 178, 190, 200);
	ImU32 amber = IM_COL32(230, 184, 112, 235);

	hud_text_center(dl, 26.f * scale, cx, 36.f * scale, title, "WAR EFFORT");
	hud_text_center(dl, 14.f * scale, cx, 70.f * scale, dim, "coastal forward base");

	const char* controls = s->player.flycam
		? "WASD fly  |  mouse look  |  C drive  |  T console"
		: s->car_camera
		? "arrows drive  |  SPACE brake  |  C walk  |  F fly  |  T console"
		: "WASD walk  |  C drive  |  F fly  |  T console";

	hud_text_center(dl, 13.f * scale, cx, window->screen_height - 42.f * scale,
	                dim, controls);

	if (s->car_camera && !s->player.flycam)
	{
		float kmh = s->phys.vehicle_speed() * 3.6f;
		const char* gear = kmh < -1.f ? "R" : fabsf(kmh) < 1.f ? "N" : "D";

		char speed[32];
		snprintf(speed, sizeof(speed), "%s  %+.0f km/h", gear, kmh);

		hud_text_right(dl, 14.f * scale, right, window->screen_height - 136.f * scale,
		               dim, "MOTOR POOL");
		hud_text_right(dl, 30.f * scale, right, window->screen_height - 112.f * scale,
		               amber, speed);
	}
	else if (!s->player.flycam && s->player.gun_mesh)
	{
		char ammo[32];
		snprintf(ammo, sizeof(ammo), "%u / %u", s->player.ammo, GUN_MAG);

		hud_text_right(dl, 14.f * scale, right, window->screen_height - 96.f * scale,
		               dim, "TEST PISTOL");
		hud_text_right(dl, 24.f * scale, right, window->screen_height - 74.f * scale,
		               s->player.ammo ? title : IM_COL32(220, 90, 80, 235), ammo);

		hud_crosshair(dl, window->screen_width * .5f, window->screen_height * .5f,
		              scale, IM_COL32(235, 238, 243, 170));
	}
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

// ---- render : title, current rig, and the lab legend
void scene_render_ux(Scene* s, GameWindow* window)
{
	ImDrawList* dl = ImGui::GetBackgroundDrawList();
	float scale = window->screen_height / 1080.f;
	float cx = window->screen_width * .5f;

	ImU32 title = IM_COL32(235, 238, 243, 225);
	ImU32 dim   = IM_COL32(170, 178, 190, 200);

	const char* preset = (s->lighting_kind < LIGHT_PRESET_COUNT)
		? LIGHTING_PRESET_NAMES[s->lighting_kind] : "-";

	char sub[96];
	snprintf(sub, sizeof(sub), "%s  |  10 materials  |  5x5 metal / roughness  |  texture vs flat", preset);

	hud_text_center(dl, 24.f * scale, cx, 36.f * scale, title, "RENDER");
	hud_text_center(dl, 14.f * scale, cx, 70.f * scale, dim, sub);
	hud_text_center(dl, 13.f * scale, cx, window->screen_height - 42.f * scale, dim,
	                "F fly to inspect  |  T console  |  hero material + lighting under Scenes");
}

// ---- terrain : title, seed and the generated extent
void scene_terrain_ux(Scene* s, GameWindow* window)
{
	ImDrawList* dl = ImGui::GetBackgroundDrawList();
	float scale = window->screen_height / 1080.f;

	// centre on the strip the console leaves visible, not the whole window
	float panel = ux_panel_visible ? ux_panel_width(window) : 0.f;
	float cx = (window->screen_width + panel) * .5f;

	ImU32 title = IM_COL32(235, 238, 243, 225);
	ImU32 dim   = IM_COL32(170, 178, 190, 200);

	char sub[128];
	snprintf(sub, sizeof(sub), "seed %u  |  %d x %d  |  %.0f m  |  %.1f m relief  |  water %.1f m",
	         s->terrain.seed, s->terrain.resolution, s->terrain.resolution, s->terrain.size,
	         s->terrain.max_height - s->terrain.min_height, s->terrain.water_level);

	hud_text_center(dl, 24.f * scale, cx, 36.f * scale, title, "TERRAIN");
	hud_text_center(dl, 14.f * scale, cx, 70.f * scale, dim, sub);
	hud_text_center(dl, 13.f * scale, cx, window->screen_height - 42.f * scale, dim,
	                "WASD fly  |  mouse look  |  F walk  |  T console  |  generation under Scenes");
}

// ---- physics : title, counts and the controls
void scene_physics_ux(Scene* s, GameWindow* window)
{
	ImDrawList* dl = ImGui::GetBackgroundDrawList();
	float scale = window->screen_height / 1080.f;
	float cx = window->screen_width * .5f;
	float h  = (float)window->screen_height;

	ImU32 title = IM_COL32(235, 238, 243, 225);
	ImU32 dim   = IM_COL32(170, 178, 190, 200);

	char bodies[64];
	snprintf(bodies, sizeof(bodies), "%u bodies  |  %u joints",
	         s->phys.num_bodies ? s->phys.num_bodies - 1 : 0, s->phys.num_joints);

	hud_text_center(dl, 24.f * scale, cx, 36.f * scale, title, "PHYSICS");
	hud_text_center(dl, 14.f * scale, cx, 70.f * scale, dim, bodies);
	hud_text_center(dl, 13.f * scale, cx, h - 42.f * scale, dim,
	                "WASD move  |  SHIFT sprint  |  SPACE jump  |  F flycam  |  T console  |  walk into a station");
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
	ImU32 amber = IM_COL32(230, 184, 112, 235);
	ImU32 red   = IM_COL32(220, 90, 80, 235);

	float kmh = s->phys.vehicle_speed() * 3.6f;
	float abs_kmh = kmh < 0.f ? -kmh : kmh;
	const char* gear = kmh < -1.f ? "R" : abs_kmh < 1.f ? "N" : "D";

	char speed[32];
	snprintf(speed, sizeof(speed), "%s  %+.0f km/h", gear, kmh);

	float z = s->phys.get_position(s->phys.vehicle.chassis_id).z;
	const char* section = z < 30.f ? "START" : z < 55.f ? "JUMP" : z < 108.f ? "SLALOM" : "FINISH";

	char info[80];
	snprintf(info, sizeof(info), "%s   |   steer %+.0f deg",
	         s->player.flycam ? "FREE" : "CHASE", s->phys.vehicle_steer() * 180.f / PI);

	hud_text_center(dl, 24.f * scale, cx, 36.f * scale, white, "VEHICLES");
	hud_text_center(dl, 14.f * scale, cx, 70.f * scale, dim, section);

	float right = w - 48.f * scale;
	float bar_w = 240.f * scale;
	float bar_h = 4.f * scale;
	float bar_x = right - bar_w;
	float bar_y = h - 92.f * scale;
	float t = abs_kmh / 160.f;
	if (t > 1.f) t = 1.f;

	dl->AddRectFilled(ImVec2(bar_x, bar_y), ImVec2(bar_x + bar_w, bar_y + bar_h),
	                  IM_COL32(30, 34, 40, 180));
	dl->AddRectFilled(ImVec2(bar_x, bar_y), ImVec2(bar_x + bar_w * t, bar_y + bar_h),
	                  abs_kmh > 120.f ? red : amber);

	hud_text_right(dl, 34.f * scale, right, h - 136.f * scale, white, speed);
	hud_text_right(dl, 14.f * scale, right, h - 80.f * scale, dim, info);
	hud_text_right(dl, 13.f * scale, right, h - 40.f * scale, dim,
	               "up/down drive  |  left/right steer  |  SPACE handbrake  |  F camera  |  T console");
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
	ImGui::Text("island %d x %d   yard %.1f m   sea %u x %u tiles",
	            s->terrain.resolution, s->terrain.resolution, hub_view.plateau,
	            WATER_TILES, WATER_TILES);

	if (ImGui::Button("reset props")) s->phys.reset_all();
	ImGui::SameLine();
	scene_labels_checkbox(s);

	bool drive = s->car_camera;
	if (ImGui::Checkbox("drive the car", &drive))
	{
		s->car_camera = drive;
		if (drive)
		{
			s->player.flycam = false;
			hub_seat_camera(s);
		}
	}
	ImGui::TextDisabled("C does the same from the world");

	ImGui::Separator();
	ImGui::TextDisabled("sea (live)");
	if (ImGui::BeginTable("hub_sea_presets", 3, ImGuiTableFlags_SizingStretchSame))
	{
		for (uint i = 0; i < WATER_PRESET_COUNT; i++)
		{
			ImGui::TableNextColumn();
			if (ImGui::Button(WATER_PRESETS[i].name, ImVec2(-1, 0)))
			{
				water_apply_preset(&water_config, i);
				s->water_spectrum = true;
				s->water_dirty = true;
			}
		}
		ImGui::EndTable();
	}

	if (ImGui::SliderFloat("wind", &water_config.wind_speed, 1.f, 20.f, "%.1f m/s"))
	{
		s->water_spectrum = true;
		s->water_dirty = true;
	}
	if (ImGui::SliderFloat("wave height", &water_config.amplitude, .05f, 2.5f, "%.2f m"))
	{
		s->water_spectrum = true;
		s->water_dirty = true;
	}
	if (ImGui::SliderFloat("foam", &water_config.foam, 0.f, 1.f, "%.2f"))
		s->water_dirty = true;

	ImGui::Separator();
	ImGui::TextDisabled("cistern");
	if (ImGui::Button("drop"))
		shallow_drop(&s->shallow, 0.f, 0.f, shallow_config.drop_strength);
	ImGui::SameLine();
	ImGui::Checkbox("rain", &s->shallow.rain);
	ImGui::SameLine();
	ImGui::Checkbox("pause", &s->shallow.paused);

	ImGui::Separator();
	ImGui::TextDisabled("lighting");
	int preset = (int)s->lighting_kind;
	if (ImGui::Combo("preset", &preset, LIGHTING_PRESET_NAMES, LIGHT_PRESET_COUNT))
		s->use_lighting((uint)preset);
	ImGui::SameLine();
	if (ImGui::Button("reset")) hub_apply_lighting(s);
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

void scene_render_debug(Scene* s)
{
	int preset = (int)s->lighting_kind;
	if (ImGui::Combo("preset", &preset, LIGHTING_PRESET_NAMES, LIGHT_PRESET_COUNT))
		s->use_lighting((uint)preset);

	if (ImGui::Button("reset lighting")) s->lighting = lighting_preset(s->lighting_kind);
	ImGui::SameLine();
	scene_labels_checkbox(s);

	ImGui::Separator();
	ImGui::TextDisabled("hero sphere (right)");
	ImGui::ColorEdit3("albedo", &render_controls.hero_albedo.x);
	ImGui::SliderFloat("metallic", &render_controls.hero_metallic, 0.f, 1.f);
	ImGui::SliderFloat("roughness", &render_controls.hero_roughness, 0.f, 1.f);
	ImGui::SliderFloat("material ao", &render_controls.hero_ao, 0.f, 1.f);

	if (ImGui::SmallButton("chrome"))
	{
		render_controls.hero_albedo = vec3(.550f, .556f, .554f);
		render_controls.hero_metallic = 1.f;
		render_controls.hero_roughness = .03f;
	}
	ImGui::SameLine();
	if (ImGui::SmallButton("gold"))
	{
		render_controls.hero_albedo = vec3(1.000f, .766f, .336f);
		render_controls.hero_metallic = 1.f;
		render_controls.hero_roughness = .10f;
	}
	ImGui::SameLine();
	if (ImGui::SmallButton("wood"))
	{
		render_controls.hero_albedo = vec3(.580f, .400f, .220f);
		render_controls.hero_metallic = 0.f;
		render_controls.hero_roughness = .62f;
	}
	ImGui::SameLine();
	if (ImGui::SmallButton("rubber"))
	{
		render_controls.hero_albedo = vec3(.050f, .050f, .060f);
		render_controls.hero_metallic = 0.f;
		render_controls.hero_roughness = .92f;
	}
	ImGui::SameLine();
	if (ImGui::SmallButton("reset material")) render_default_material();

	if (render_controls.hero_prop < s->num_props)
	{
		Scene::Prop& p = s->props[render_controls.hero_prop];
		p.mat_a = vec4(render_controls.hero_albedo, render_controls.hero_metallic);
		p.mat_b = vec4(render_controls.hero_roughness, render_controls.hero_ao, 0.f, 0.f);
	}

	ImGui::Separator();
	ImGui::TextDisabled("sun");
	ImGui::ColorEdit3("sun color", &s->lighting.sun_color.x);
	ImGui::SliderFloat("sun power", &s->lighting.sun_power, 0.f, 8.f);
	ImGui::SliderFloat("sun disk", &s->lighting.sun_disk, 0.f, 2.f);
	ImGui::DragFloat3("sun direction", &s->lighting.sun_direction.x, .01f, -1.f, 1.f);

	ImGui::Separator();
	ImGui::TextDisabled("rim");
	ImGui::ColorEdit3("rim color", &s->lighting.rim_color.x);
	ImGui::SliderFloat("rim power", &s->lighting.rim_power, 0.f, 3.f);
	ImGui::DragFloat3("rim direction", &s->lighting.rim_direction.x, .01f, -1.f, 1.f);

	ImGui::Separator();
	ImGui::TextDisabled("image");
	ImGui::SliderFloat("exposure", &s->lighting.exposure, .2f, 3.f);
	ImGui::SliderFloat("fog", &s->lighting.fog_density, 0.f, .03f, "%.4f");
	ImGui::SliderFloat("ambient occlusion", &s->lighting.ao_strength, 0.f, 3.f);
	ImGui::SliderFloat("multiscatter", &s->lighting.multiscatter, 0.f, 1.f);

	ImGui::Separator();
	if (ImGui::CollapsingHeader("sky and ambient"))
	{
		ImGui::ColorEdit3("zenith", &s->lighting.zenith.x);
		ImGui::ColorEdit3("haze", &s->lighting.haze.x);
		ImGui::ColorEdit3("glow", &s->lighting.glow.x);
		ImGui::ColorEdit3("ground", &s->lighting.ground.x);
		ImGui::ColorEdit3("ambient up", &s->lighting.ambient_up.x);
		ImGui::ColorEdit3("ambient side", &s->lighting.ambient_side.x);
		ImGui::ColorEdit3("ambient down", &s->lighting.ambient_down.x);
	}

	ImGui::Separator();
	ImGui::TextDisabled("studio is the neutral reference");
	ImGui::TextDisabled("metals, roughness and texture should separate here");
}

void scene_water_debug(Scene* s)
{
	ImGui::Text("fft %d x %d   patch %.0f m   Hs %.1f m   t %.1f s   %u tiles",
	            s->water.n, s->water.n, water_config.size, water_config.amplitude * 4.f,
	            s->water.time, WATER_TILES);

	ImGui::Checkbox("pause", &s->water.paused);
	ImGui::SameLine();
	if (ImGui::Button("reset time"))
	{
		s->water.time = 0.f;
		s->water_dirty = true;
	}
	ImGui::SameLine();
	scene_labels_checkbox(s);

	ImGui::Separator();
	ImGui::TextDisabled("presets");
	if (ImGui::BeginTable("water_presets", 3, ImGuiTableFlags_SizingStretchSame))
	{
		for (uint i = 0; i < WATER_PRESET_COUNT; i++)
		{
			ImGui::TableNextColumn();
			if (ImGui::Button(WATER_PRESETS[i].name, ImVec2(-1, 0)))
			{
				water_apply_preset(&water_config, i);
				s->water_spectrum = true;
				s->water_dirty = true;
			}
		}
		ImGui::EndTable();
	}

	ImGui::Separator();
	ImGui::TextDisabled("spectrum (live)");

	const int resolutions[3] = { 64, 128, 256 };
	const char* resolution_names[3] = { "64", "128", "256" };
	int res = 1;
	for (uint i = 0; i < 3; i++) if (water_config.resolution == resolutions[i]) res = (int)i;
	if (ImGui::Combo("fft / mesh", &res, resolution_names, 3))
	{
		water_config.resolution = resolutions[res];
		s->water_spectrum = true;
		s->water_dirty = true;
	}

	int seed = (int)water_config.seed;
	if (ImGui::DragInt("seed", &seed, 1, 0, 1000000))
	{
		water_config.seed = (uint)seed;
		s->water_spectrum = true;
		s->water_dirty = true;
	}
	if (ImGui::SliderFloat("patch", &water_config.size, 48.f, 256.f, "%.0f m"))
	{
		s->water_spectrum = true;
		s->water_dirty = true;
	}
	if (ImGui::SliderFloat("wind speed", &water_config.wind_speed, 1.f, 20.f, "%.1f m/s"))
	{
		s->water_spectrum = true;
		s->water_dirty = true;
	}
	float wind_deg = water_config.wind_angle * 180.f / PI;
	if (ImGui::SliderFloat("wind dir", &wind_deg, 0.f, 360.f, "%.0f deg"))
	{
		water_config.wind_angle = wind_deg * PI / 180.f;
		s->water_spectrum = true;
		s->water_dirty = true;
	}
	if (ImGui::SliderFloat("wave height (rms)", &water_config.amplitude, .05f, 2.5f, "%.2f m"))
	{
		s->water_spectrum = true;
		s->water_dirty = true;
	}

	ImGui::Separator();
	ImGui::TextDisabled("surface (live)");
	ImGui::SliderFloat("time scale", &water_config.time_scale, 0.f, 3.f, "%.2f");
	if (ImGui::SliderFloat("chop", &water_config.choppiness, 0.f, 2.f, "%.2f"))
		s->water_dirty = true;

	float foam_pct = water_config.foam * 100.f;
	if (ImGui::SliderFloat("foam amount", &foam_pct, 0.f, 100.f, "%.0f%%"))
	{
		water_config.foam = foam_pct / 100.f;
		s->water_dirty = true;
	}

	ImGui::SliderFloat("foam persistence", &water_config.foam_persistence, .2f, 5.f, "%.1f s");
	ImGui::TextDisabled("crests add foam, trails fade over the persistence");

	ImGui::SliderFloat("ripple detail", &water_config.detail, 0.f, 1.5f, "%.2f");
	ImGui::TextDisabled("short-chop normals from a second simulated cascade");

	ImGui::Separator();
	ImGui::TextDisabled("material (live)");
	ImGui::ColorEdit3("water colour", &water_config.albedo.x);
	float gloss = 1.f - water_config.roughness;
	if (ImGui::SliderFloat("water gloss", &gloss, 0.f, 1.f, "%.2f"))
		water_config.roughness = 1.f - gloss;
	ImGui::SliderFloat("metallic", &water_config.metallic, 0.f, 1.f);
	ImGui::ColorEdit3("foam colour", &water_config.foam_albedo.x);
	float foam_gloss = 1.f - water_config.foam_roughness;
	if (ImGui::SliderFloat("foam gloss", &foam_gloss, 0.f, 1.f, "%.2f"))
		water_config.foam_roughness = 1.f - foam_gloss;
	water_apply_materials(s);

	ImGui::Separator();
	ImGui::TextDisabled("lighting");
	int preset = (int)s->lighting_kind;
	if (ImGui::Combo("preset", &preset, LIGHTING_PRESET_NAMES, LIGHT_PRESET_COUNT))
		s->use_lighting((uint)preset);
	ImGui::SameLine();
	if (ImGui::Button("reset lighting")) s->lighting = lighting_preset(s->lighting_kind);
}

void scene_shallow_debug(Scene* s)
{
	int n = s->shallow.n;
	float spacing = n > 1 ? shallow_config.size / (float)(n - 1) * 100.f : 0.f;

	ImGui::Text("fd %d x %d   %.0f m patch   %.0f cm grid   t %.1f s",
	            n, n, shallow_config.size, spacing, s->shallow.time);

	if (ImGui::Button("drop now"))
		shallow_drop(&s->shallow, 0.f, 0.f, shallow_config.drop_strength);
	ImGui::SameLine();
	if (ImGui::Button("reset"))
	{
		shallow_reset(&s->shallow, shallow_config);
		shallow_drop(&s->shallow, 0.f, 0.f, shallow_config.drop_strength);
	}
	ImGui::SameLine();
	scene_labels_checkbox(s);

	ImGui::Checkbox("rain", &s->shallow.rain);
	ImGui::SameLine();
	ImGui::Checkbox("pause", &s->shallow.paused);

	ImGui::Separator();
	ImGui::TextDisabled("simulation (live)");
	ImGui::SliderFloat("wave speed", &shallow_config.speed, .5f, 2.2f, "%.1f m/s");
	ImGui::SliderFloat("damping", &shallow_config.damping, .98f, 1.f, "%.3f");
	ImGui::SliderFloat("drop strength", &shallow_config.drop_strength, .05f, 1.f, "%.2f m");
	ImGui::SliderFloat("drop interval", &shallow_config.drop_interval, 1.f, 20.f, "%.1f s");
	ImGui::TextDisabled("clamped edges reflect the waves at the stone walls");

	ImGui::Separator();
	ImGui::TextDisabled("material (live)");
	ImGui::ColorEdit3("water colour", &shallow_config.albedo.x);
	float gloss = 1.f - shallow_config.roughness;
	if (ImGui::SliderFloat("water gloss", &gloss, 0.f, 1.f, "%.2f"))
		shallow_config.roughness = 1.f - gloss;
	ImGui::SliderFloat("metallic", &shallow_config.metallic, 0.f, 1.f);
	shallow_apply_material(s);

	ImGui::Separator();
	ImGui::TextDisabled("lighting");
	int preset = (int)s->lighting_kind;
	if (ImGui::Combo("preset", &preset, LIGHTING_PRESET_NAMES, LIGHT_PRESET_COUNT))
		s->use_lighting((uint)preset);
	ImGui::SameLine();
	if (ImGui::Button("reset lighting")) s->lighting = lighting_preset(s->lighting_kind);
}

// live band materials : the meshes are baked, but colour and roughness are per prop
void terrain_apply_materials(Scene* s)
{
	for (uint band = 0; band < TERRAIN_BAND_COUNT; band++)
	{
		uint at = terrain_view.prop[band];
		if (at >= s->num_props) continue;
		terrain_band_material(terrain_config, band, &s->props[at].mat_a, &s->props[at].mat_b);
	}
}

void scene_terrain_debug(Scene* s)
{
	ImGui::Text("seed %u   %d x %d   %.0f m   %.1f..%.1f m   water %.1f m",
	            s->terrain.seed, s->terrain.resolution, s->terrain.resolution,
	            s->terrain.size, s->terrain.min_height, s->terrain.max_height,
	            s->terrain.water_level);

	if (ImGui::Button("rebuild now")) s->terrain_rebuild = true;
	ImGui::SameLine();
	if (ImGui::Button("random seed"))
	{
		terrain_config.seed = (uint)(ImGui::GetTime() * 100000.0);
		s->terrain_rebuild = true;
	}
	ImGui::SameLine();
	scene_labels_checkbox(s);

	ImGui::Separator();
	ImGui::TextDisabled("generation (live)");
	int seed = (int)terrain_config.seed;
	if (ImGui::DragInt("seed", &seed, 1, 0, 1000000))
	{
		terrain_config.seed = (uint)seed;
		s->terrain_rebuild = true;
	}
	int res = terrain_config.resolution;
	if (ImGui::SliderInt("resolution", &res, 33, (int)TERRAIN_MAX_N))
	{
		terrain_config.resolution = res;
		s->terrain_rebuild = true;
	}
	if (ImGui::SliderFloat("size", &terrain_config.size, 64.f, 128.f, "%.0f m"))
		s->terrain_rebuild = true;
	if (ImGui::SliderFloat("relief", &terrain_config.height, 2.f, 30.f, "%.0f m"))
		s->terrain_rebuild = true;
	if (ImGui::SliderInt("octaves", &terrain_config.octaves, 1, 8))
		s->terrain_rebuild = true;
	if (ImGui::SliderFloat("frequency", &terrain_config.frequency, .002f, .06f, "%.3f"))
		s->terrain_rebuild = true;
	if (ImGui::SliderFloat("lacunarity", &terrain_config.lacunarity, 1.5f, 3.f, "%.1f"))
		s->terrain_rebuild = true;
	if (ImGui::SliderFloat("gain", &terrain_config.gain, .3f, .7f, "%.2f"))
		s->terrain_rebuild = true;

	ImGui::Separator();
	ImGui::TextDisabled("strata (live)");
	float water_pct = terrain_config.water_at * 100.f;
	if (ImGui::SliderFloat("water", &water_pct, 0.f, 95.f, "%.0f%%"))
	{
		terrain_config.water_at = water_pct / 100.f;
		s->terrain_rebuild = true;
	}
	float sand_pct = terrain_config.sand_at * 100.f;
	if (ImGui::SliderFloat("sand", &sand_pct, 0.f, 95.f, "%.0f%%"))
	{
		terrain_config.sand_at = sand_pct / 100.f;
		s->terrain_rebuild = true;
	}
	float grass_pct = terrain_config.grass_at * 100.f;
	if (ImGui::SliderFloat("grass", &grass_pct, 0.f, 100.f, "%.0f%%"))
	{
		terrain_config.grass_at = grass_pct / 100.f;
		s->terrain_rebuild = true;
	}
	float rock_pct = terrain_config.rock_at * 100.f;
	if (ImGui::SliderFloat("rock", &rock_pct, 0.f, 100.f, "%.0f%%"))
	{
		terrain_config.rock_at = rock_pct / 100.f;
		s->terrain_rebuild = true;
	}
	ImGui::TextDisabled("above rock : snow");

	ImGui::Separator();
	ImGui::TextDisabled("strata material (live)");

	const char* band_names[TERRAIN_BAND_COUNT] = { "water", "sand", "grass", "rock", "snow" };
	vec3*  band_colors[TERRAIN_BAND_COUNT] = {
		&terrain_config.water_albedo, &terrain_config.sand_albedo,
		&terrain_config.grass_albedo, &terrain_config.rock_albedo,
		&terrain_config.snow_albedo };
	float* band_roughs[TERRAIN_BAND_COUNT] = {
		&terrain_config.water_roughness, &terrain_config.sand_roughness,
		&terrain_config.grass_roughness, &terrain_config.rock_roughness,
		&terrain_config.snow_roughness };

	if (ImGui::BeginTable("terrain_strata", 3, ImGuiTableFlags_SizingFixedFit))
	{
		ImGui::TableSetupColumn("band");
		ImGui::TableSetupColumn("albedo");
		ImGui::TableSetupColumn("rough");
		ImGui::TableHeadersRow();

		for (uint i = 0; i < TERRAIN_BAND_COUNT; i++)
		{
			ImGui::PushID((int)i);
			ImGui::TableNextRow();
			ImGui::TableNextColumn();
			ImGui::TextUnformatted(band_names[i]);
			ImGui::TableNextColumn();
			ImGui::SetNextItemWidth(-1);
			ImGui::ColorEdit3("##albedo", &band_colors[i]->x,
			                  ImGuiColorEditFlags_NoInputs | ImGuiColorEditFlags_NoLabel);
			ImGui::TableNextColumn();
			ImGui::SetNextItemWidth(-1);
			ImGui::SliderFloat("##roughness", band_roughs[i], 0.f, 1.f, "%.2f");
			ImGui::PopID();
		}
		ImGui::EndTable();
	}

	terrain_apply_materials(s);

	ImGui::Separator();
	ImGui::TextDisabled("lighting");
	int preset = (int)s->lighting_kind;
	if (ImGui::Combo("preset", &preset, LIGHTING_PRESET_NAMES, LIGHT_PRESET_COUNT))
		s->use_lighting((uint)preset);
	ImGui::SameLine();
	if (ImGui::Button("reset lighting")) s->lighting = lighting_preset(s->lighting_kind);
}

void scene_physics_debug(Scene* s)
{
	if (ImGui::Button("wake all")) s->phys.wake_all();
	ImGui::SameLine();
	if (ImGui::Button("reset all")) s->phys.reset_all();
	ImGui::SameLine();
	scene_labels_checkbox(s);

	ImGui::Separator();
	if (ImGui::Button("kick everything"))
	{
		for (uint i = 1; i < s->phys.num_bodies; i++)
			if (i != s->player.body && s->phys.is_dynamic(i))
				s->phys.apply_impulse(i, vec3(1.5f, 0, 0));
	}
	ImGui::SameLine();
	if (ImGui::Button("raycast"))
	{
		Camera& view = s->player.flycam ? s->camera : s->player.camera;
		vec3 from = view.position + view.front * .5f; // clear of the player capsule
		Physics::Phys_Ray hit = s->phys.raycast(from, from + view.front * 50.f);

		char msg[96];
		if (hit.hit)
			snprintf(msg, sizeof(msg), "Ray hit body %u at %.1f, %.1f, %.1f",
			         hit.body_id, hit.point.x, hit.point.y, hit.point.z);
		else
			snprintf(msg, sizeof(msg), "Ray missed");
		console->add_entry(msg, hit.hit ? SUCCESS : DEBUG, PHYS);
	}

	ImGui::Separator();
	ImGui::TextDisabled("stations");
	if (ImGui::BeginTable("physics_stations", 3, ImGuiTableFlags_SizingStretchSame))
	{
		for (uint i = 0; i < PST_COUNT; i++)
		{
			ImGui::TableNextColumn();
			station_label(i);
		}
		ImGui::EndTable();
	}

	ImGui::Separator();
	station_label(PST_HINGE);
	float arm_limit_deg = physics_controls.arm_limit * 180.f / PI;
	if (ImGui::SliderFloat("arm limit", &arm_limit_deg, 10.f, 178.f, "%.0f deg"))
	{
		physics_controls.arm_limit = arm_limit_deg * PI / 180.f;
		s->phys.wake_all();
	}
	if (ImGui::SliderFloat("arm damping", &physics_controls.arm_damping, 0.f, 1.f, "%.2f"))
		s->phys.wake_all();
	ImGui::SliderFloat("push impulse", &physics_controls.arm_push, .5f, 12.f, "%.1f");

	if (ImGui::Button("push hinge") && physics_controls.arm_body)
	{
		vec3 at;
		mat3 rot;
		if (s->phys.get_transform(physics_controls.arm_body, &at, &rot))
		{
			// push across the arm, not along it : a hanging arm has no torque from a vertical kick
			vec3 along = rot[0];
			vec3 tangent = normalize(vec3(-along.y, along.x, 0.f));
			s->phys.apply_impulse(physics_controls.arm_body, tangent * physics_controls.arm_push);
		}
	}
	ImGui::SameLine();
	if (ImGui::Button("release arm") && physics_controls.arm_body)
	{
		s->phys.reset(physics_controls.arm_body);
		s->phys.wake_all();
	}

	if (physics_controls.arm)
		ImGui::Text("angle %.0f deg", physics_controls.arm->getHingeAngle() * 180.f / PI);
	ImGui::TextDisabled("limit clamps the swing | push kicks the free end");
	ImGui::TextDisabled("release drops it from horizontal");

	ImGui::Separator();
	station_label(PST_CHAIN);
	if (ImGui::Button("swing chain") && physics_controls.chain_end)
		s->phys.apply_impulse(physics_controls.chain_end, vec3(3.f, 0, 0));

	ImGui::Separator();
	station_label(PST_SEESAW);
	float seesaw_limit_deg = physics_controls.seesaw_limit * 180.f / PI;
	if (ImGui::SliderFloat("see-saw limit", &seesaw_limit_deg, 3.f, 20.f, "%.0f deg"))
	{
		physics_controls.seesaw_limit = seesaw_limit_deg * PI / 180.f;
		s->phys.wake_all();
	}
	if (physics_controls.seesaw)
		ImGui::Text("angle %.0f deg", physics_controls.seesaw->getHingeAngle() * 180.f / PI);
	ImGui::TextDisabled("the crates on the left end load the plank");

	ImGui::Separator();
	station_label(PST_MOTOR);
	if (physics_controls.rotor)
	{
		ImGui::Checkbox("rotor motor", &physics_controls.rotor_on);
		ImGui::SameLine();
		ImGui::Text("%.0f deg | target %.1f", physics_controls.rotor->getHingeAngle() * 180.f / PI,
		            physics_controls.rotor->getMotorTargetVelocity());
		ImGui::SliderFloat("rotor speed", &physics_controls.rotor_speed, -8.f, 8.f, "%.1f rad/s");
		ImGui::SliderFloat("rotor impulse", &physics_controls.rotor_impulse, 0.f, 40.f, "%.1f");
	}

	ImGui::Separator();
	station_label(PST_CRADLE);
	if (ImGui::Button("swing cradle") && physics_controls.cradle_first)
		s->phys.apply_impulse(physics_controls.cradle_first, vec3(6.f, 0, 0));

	ImGui::Separator();
	station_label(PST_CCD);
	if (ImGui::Button("fire ccd"))
	{
		if (physics_controls.ccd_on)
		{
			s->phys.reset(physics_controls.ccd_on);
			s->phys.apply_impulse(physics_controls.ccd_on, vec3(150.f, 0, 0));
		}
		if (physics_controls.ccd_off)
		{
			s->phys.reset(physics_controls.ccd_off);
			s->phys.apply_impulse(physics_controls.ccd_off, vec3(150.f, 0, 0));
		}
	}

	ImGui::Separator();
	station_label(PST_DOOR);
	if (physics_controls.door)
	{
		ImGui::Checkbox("door motor", &physics_controls.door_on);
		ImGui::SameLine();

		bool door_awake = physics_controls.door_body && physics_controls.door_body < s->phys.num_bodies &&
		                  s->phys.bodies[physics_controls.door_body].body->isActive();
		ImGui::Text("%.0f deg | %s | target %.1f", physics_controls.door->getHingeAngle() * 180.f / PI,
		            door_awake ? "awake" : "asleep",
		            physics_controls.door->getMotorTargetVelocity());
		ImGui::SliderFloat("door speed", &physics_controls.door_speed, .5f, 6.f, "%.1f rad/s");
		ImGui::SliderFloat("door impulse", &physics_controls.door_impulse, 1.f, 80.f, "%.0f");
		if (ImGui::SliderFloat("door travel", &physics_controls.door_open_angle, 30.f, 120.f, "%.0f deg"))
			s->phys.wake_all();
		ImGui::SliderFloat("door damping", &physics_controls.door_damping, 0.f, 1.f, "%.2f");

		if (ImGui::Button("open door"))
		{
			physics_controls.door_target = 1.f;
			physics_controls.door_on = true;
		}
		ImGui::SameLine();
		if (ImGui::Button("close door"))
		{
			physics_controls.door_target = -1.f;
			physics_controls.door_on = true;
		}
		ImGui::SameLine();
		if (ImGui::Button("stop door")) physics_controls.door_target = 0.f;
	}

	ImGui::Separator();
	ImGui::TextDisabled("world");
	if (ImGui::SliderFloat("gravity", &physics_controls.gravity, 0.f, 20.f, "%.1f m/s2"))
		s->phys.wake_all();
	if (ImGui::SliderInt("solver iterations", &physics_controls.solver_iters, 1, 30))
		s->phys.wake_all();
	if (ImGui::Button("reset settings"))
	{
		physics_default_settings();
		physics_apply_settings(s);
		s->phys.wake_all();
	}

	physics_apply_settings(s);

	ImGui::Separator();
	ImGui::TextDisabled("world signs use the same station numbers");
}

void scene_vehicles_debug(Scene* s)
{
	ImGui::Text("%.0f km/h   steer %+.0f deg   %s",
	            s->phys.vehicle_speed() * 3.6f,
	            s->phys.vehicle_steer() * 180.f / PI,
	            s->player.flycam ? "free camera" : "chase camera");

	if (ImGui::Button("reset car"))
	{
		if (s->phys.vehicle.chassis_id) s->phys.reset(s->phys.vehicle.chassis_id);
	}
	ImGui::SameLine();
	if (ImGui::Button("reset course")) s->phys.reset_all();
	ImGui::SameLine();
	scene_labels_checkbox(s);

	ImGui::Separator();
	ImGui::TextDisabled("camera");
	bool chase = !s->player.flycam;
	if (ImGui::Checkbox("chase camera", &chase)) s->player.flycam = !chase;
	ImGui::SliderFloat("chase distance", &s->chase_distance, 4.f, 14.f, "%.1f m");
	ImGui::SliderFloat("chase height", &s->chase_height, 1.5f, 8.f, "%.1f m");

	ImGui::Separator();
	ImGui::TextDisabled("engine");
	ImGui::SliderFloat("engine force", &s->phys.vehicle.engine_force, 100.f, 2000.f, "%.0f / wheel");
	ImGui::SliderFloat("brake force", &s->phys.vehicle.brake_force, 5.f, 100.f, "%.0f");
	ImGui::SliderFloat("coast brake", &s->phys.vehicle.coast_brake, 0.f, 20.f, "%.0f");
	ImGui::SliderFloat("top speed", &s->phys.vehicle.top_speed, 40.f, 200.f, "%.0f km/h");

	ImGui::Separator();
	ImGui::TextDisabled("steering");
	float steer_deg = s->phys.vehicle.steer_angle * 180.f / PI;
	if (ImGui::SliderFloat("steer angle", &steer_deg, 10.f, 45.f, "%.0f deg"))
		s->phys.vehicle.steer_angle = steer_deg * PI / 180.f;
	ImGui::SliderFloat("steer rate", &s->phys.vehicle.steer_rate, 1.f, 10.f, "%.1f rad/s");

	ImGui::Separator();
	ImGui::TextDisabled("suspension and tyres");
	ImGui::SliderFloat("wheel grip", &s->phys.vehicle.wheel_grip, .5f, 3.f, "%.2f");
	ImGui::SliderFloat("suspension stiffness", &s->phys.vehicle.suspension_stiffness, 10.f, 100.f, "%.0f");
	ImGui::SliderFloat("suspension damping", &s->phys.vehicle.suspension_damping, .5f, 5.f, "%.2f");
	ImGui::SliderFloat("suspension compression", &s->phys.vehicle.suspension_compression, .5f, 8.f, "%.2f");
	ImGui::SliderFloat("suspension travel", &s->phys.vehicle.suspension_travel, 10.f, 60.f, "%.0f cm");
	ImGui::SliderFloat("roll influence", &s->phys.vehicle.roll_influence, 0.f, 1.f, "%.2f");

	if (ImGui::Button("boost") && s->phys.vehicle.chassis_id)
	{
		vec3 pos;
		mat3 rot;
		if (s->phys.get_transform(s->phys.vehicle.chassis_id, &pos, &rot))
		{
			vec3 forward = rot[2];
			forward.y = 0.f;
			if (length(forward) > .01f)
				s->phys.apply_impulse(s->phys.vehicle.chassis_id, normalize(forward) * 8000.f);
		}
	}

	ImGui::Separator();
	ImGui::TextDisabled("lighting");
	int preset = (int)s->lighting_kind;
	if (ImGui::Combo("preset", &preset, LIGHTING_PRESET_NAMES, LIGHT_PRESET_COUNT))
		s->use_lighting((uint)preset);
	ImGui::SameLine();
	if (ImGui::Button("reset lighting")) s->lighting = lighting_preset(s->lighting_kind);
}

enum SceneKind {
	SCENE_HUB = 0,
	SCENE_WEAPONS,
	SCENE_RENDER,
	SCENE_WATER,
	SCENE_SHALLOW,
	SCENE_TERRAIN,
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
	{ "render",    scene_render,    scene_render_ux,    scene_render_debug },
	{ "ocean",     scene_water,     scene_water_ux,     scene_water_debug },
	{ "shallow",   scene_shallow,   scene_shallow_ux,   scene_shallow_debug },
	{ "terrain",   scene_terrain,   scene_terrain_ux,   scene_terrain_debug },
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
		if (scene.reload_requested)
			load(renderer, current_kind);
		else if (!loaded || requested_kind != current_kind)
			load(renderer, requested_kind);

		// terrain edits rebuild the grid and collider in place, before physics steps
		if (loaded && current_kind == SCENE_TERRAIN && scene.terrain_rebuild)
			terrain_rebuild(&scene, renderer);

		// water evolves every frame and refills its geometry before the draw pass
		if (loaded && current_kind == SCENE_WATER)
		{
			water_scene_update(&scene, renderer, window->dtime);
			water_bind_window(window);
		}

		// the shallow patch steps and rebuilds its small surface every frame
		if (loaded && current_kind == SCENE_SHALLOW)
			shallow_scene_update(&scene, renderer, window->dtime);

		// the hub runs both simulations : C swaps between walking and the car
		if (loaded && current_kind == SCENE_HUB)
		{
			scene_hub_key(&scene, window);
			hub_scene_update(&scene, renderer, window->dtime);
			water_bind_window(window);
		}

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
