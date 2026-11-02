#include "fluids.h"

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
const uint MAX_PROPS   = 65536;  // scene instances are uploaded once per mesh run
static float game_fov = 60.f;    // options menu : vertical field of view

// one scene. no state outlives a switch : the manager tears this down and the
// next builder fills it from scratch. systems are passed in, not inherited
struct Scene
{
	Camera camera;    // the view the world is rendered from
	mat4   proj_view; // built from the camera every frame in update()

	// studio / preview orbit : independent of the player and preserved across
	// scene reloads by the preview scene
	bool   orbit_cam;
	vec3   orbit_target;
	float  orbit_yaw, orbit_pitch, orbit_distance;

	// vehicles : a smoothed chase camera read from the car, not the player
	bool   car_camera;
	Camera vehicle_view;
	float  chase_distance, chase_height;

	Physics phys; // owns motion, the props read their pose from it
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
		bool water;      // forward water pass instead of the static buffers
		uint draw_mode;  // explicit DrawMode, (uint)-1 = derive from no_uv/water
		vec3 pos;
		mat3 rot;
		vec3 scale;
		vec4 mat_a;     // albedo rgb + metallic
		vec4 mat_b;     // roughness, ao, texture-albedo flag
		char label[24]; // drawn over it in the world (empty = none)
	} props[MAX_PROPS];
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
	bool water_dirty;     // water settings changed : refresh the spectrum or the mesh
	bool water_spectrum;  // wind, seed or size changed : rebuild h0 as well

	Lighting lighting;      // the deferred pass's per-scene rig
	uint     lighting_kind; // LightingPreset this scene started from
	float    shadow_extent; // metres the sun shadow map covers around the camera
	float    far_plane;     // draw distance for this scene, scaled by its ground size
	float    fly_speed_scale; // free-camera speed for this scene's world size

	// day/night : slot 0..24, optionally advancing while cycle_playing
	bool  cycle;
	bool  cycle_playing;
	float time_of_day;
	float time_speed; // hours per second

	// local lights : each one is drawn as a sphere over the g-buffer
	bool        local_lights;
	Point_Light lights[MAX_LIGHTS];
	uint        num_lights;

	void add_light(vec3 position, vec3 color, float radius, float intensity)
	{
		if (num_lights >= MAX_LIGHTS)
		{
			console->add_entry("light slots full : raise MAX_LIGHTS", FIXME, RNDR);
			return;
		}

		Point_Light& l = lights[num_lights++];
		l.position    = position;
		l.color       = color;
		l.radius      = radius;
		l.intensity   = intensity;
		l.direction   = vec3(0, -1, 0);
		l.shape       = LIGHT_SPHERE;
		l.outer_angle = 0.f;
		l.inner_angle = 0.f;
	}

	// a spot light : the volume is a cone from the light head along dir
	void add_spot_light(vec3 position, vec3 dir, vec3 color, float radius, float intensity,
	                    float outer_degrees, float inner_degrees)
	{
		if (num_lights >= MAX_LIGHTS)
		{
			console->add_entry("light slots full : raise MAX_LIGHTS", FIXME, RNDR);
			return;
		}

		Point_Light& l = lights[num_lights++];
		l.position    = position;
		l.color       = color;
		l.radius      = radius;
		l.intensity   = intensity;
		l.direction   = normalize(dir);
		l.shape       = LIGHT_CONE;
		l.outer_angle = outer_degrees * PI / 180.f;
		l.inner_angle = inner_degrees * PI / 180.f;
	}

	// two headlights that follow a car body every frame
	uint headlight_body;
	int  headlight_light[2];

	void add_vehicle_headlights(uint phys_id)
	{
		headlight_body = phys_id;
		for (uint i = 0; i < 2; i++)
		{
			add_spot_light(vec3(0), vec3(0, 0, 1), vec3(1.f, .92f, .76f),
			               26.f, 8.f, 24.f, 10.f);
			headlight_light[i] = (int)num_lights - 1;
		}
	}

	void update_vehicle_headlights()
	{
		if (!headlight_body) return;

		vec3 pos;
		mat3 rot;
		if (!phys.get_transform(headlight_body, &pos, &rot)) return;

		vec3 forward = normalize(rot[2]);
		vec3 right   = normalize(rot[0]);
		for (uint i = 0; i < 2; i++)
		{
			if (headlight_light[i] < 0 || headlight_light[i] >= (int)num_lights) continue;

			Point_Light& l = lights[headlight_light[i]];
			l.position  = pos + forward * 2.2f + right * (i ? .70f : -.70f) + vec3(0, .55f, 0);
			l.direction = normalize(forward - vec3(0, .08f, 0));
		}
	}

	// one more instance in the world, posed at (pos, yaw, scale)
	Prop& add_prop(uint mesh_id, vec3 pos, float yaw, float scale)
	{
		return add_prop(mesh_id, pos, yaw, vec3(scale));
	}

	// non-uniform sizes (chassis, wheels) set every axis directly
	Prop& add_prop(uint mesh_id, vec3 pos, float yaw, vec3 scale)
	{
		// no room : say so and drop the prop, rather than writing past the array
		if (num_props >= MAX_PROPS)
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
		p.water   = false;
		p.draw_mode = (uint)-1;
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
		cycle = false;
		cycle_playing = false;
	}

	// move the clock and rebuild the rig from the sun/moon path
	void use_time(float hours, bool play = false)
	{
		time_of_day = hours;
		cycle = true;
		cycle_playing = play;
		lighting = lighting_from_time(hours);
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
		local_lights = true;
		shadow_extent = 40.f;
		far_plane = fmaxf(256.f, ground_size * 2.5f);
		fly_speed_scale = 1.f;
		time_of_day = 15.f;
		time_speed = .5f;
		use_lighting(LIGHT_DAY);

		camera.position = cam_pos;
		camera.yaw      = yaw;
		camera.pitch    = pitch;
		camera.update_dir(0, 0, 0); // seeds front/right/up from yaw/pitch

		phys.init();

		// floor : one generated plane with material-only shading
		uint resolution = floor_resolution ? floor_resolution
		                                    : (uint)(ground_size > 1.f ? ground_size : 1.f);
		uint floor_mesh = renderer->add_mesh(
			MeshFactory::generate_plane(ground_size, resolution, MeshFactory::PLANE_XZ));
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

	// teardown : the world and its debug wires. the renderer clears its own
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

		if (cycle && cycle_playing)
		{
			time_of_day += window->dtime * time_speed;
			if (time_of_day >= 24.f) time_of_day -= 24.f;
			lighting = lighting_from_time(time_of_day);
		}

		if (orbit_cam)
		{
			float cp = cosf(orbit_pitch);
			camera.position = orbit_target + vec3(cp * cosf(orbit_yaw),
			                                      sinf(orbit_pitch),
			                                      cp * sinf(orbit_yaw)) * orbit_distance;
			camera.yaw = orbit_yaw + PI;
			camera.pitch = -orbit_pitch;
			camera.update_dir(0, 0, 0);
		}
		else if (player.flycam)
		{
			update_fly_camera(&camera, window, window->dtime, fly_speed_scale);
		}

		player.update(window, renderer, &phys, !car_camera);

		// only the chase view drives : walking or flying leaves the car parked
		phys.update_vehicle(window, car_camera && !player.flycam);
		phys.step(window->dtime);
		update_vehicle_headlights();

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

		// the player's camera while walking, the car camera, the free camera or
		// the studio orbit camera
		Camera& view = orbit_cam ? camera
		             : (player.flycam ? camera
		             : (car_camera && phys.vehicle.chassis_id ? vehicle_view : player.camera));

		// the g-buffer reads this for fog and sky : keep it on the active view
		if (!orbit_cam && !player.flycam && !car_camera) camera.position = view.position;

		// a little FOV stretch with speed, only in the chase view
		float speed_fov = 0.f;
		if (car_camera && !player.flycam)
		{
			float kmh = fabsf(phys.vehicle_speed() * 3.6f);
			speed_fov = kmh / 160.f * 8.f;
			if (speed_fov > 8.f) speed_fov = 8.f;
		}

		float fov = (game_fov + player.fov_punch + speed_fov) * PI / 180.f;
		window->far_plane = far_plane;
		mat4 proj = glm::perspective(fov, (float)window->screen_width / window->screen_height, 0.1f, window->far_plane);

		// off-axis projection : the camera keeps looking down the world axis, but the
		// visible strip is centred, so the panel does not hide the subject
		if (ux_panel_visible)
			proj[2][0] = -ux_panel_width(window) / (float)window->screen_width;

		proj_view = proj * glm::lookAt(view.position, view.position + view.front, view.up);

		// fresh instance streams : each mesh id may only be appended once per frame
		renderer->clear_instances();

		// props : the body owns the pose of anything dynamic. submissions are batched
		// by mesh so many instances cost one upload per mesh run
		static Instance prop_batch[512];
		uint batch_count = 0;
		uint batch_mesh  = 0;
		uint batch_mode  = 0;

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

			// water leaves the opaque pass; everything else says which layout its mesh
			// was built for. a prop can also name its draw mode outright
			uint mode = p.water ? MODE_WATER : (p.no_uv ? MODE_STATIC_NOUV : MODE_STATIC_UV);
			if (p.draw_mode != (uint)-1) mode = p.draw_mode;

			if (batch_count && (batch_mesh != p.mesh_id || batch_mode != mode))
			{
				renderer->buffer(batch_mode).append_instances(batch_mesh, batch_count, prop_batch);
				batch_count = 0;
			}

			batch_mesh = p.mesh_id;
			batch_mode = mode;
			prop_batch[batch_count++] = inst;

			if (batch_count == 512)
			{
				renderer->buffer(batch_mode).append_instances(batch_mesh, batch_count, prop_batch);
				batch_count = 0;
			}
		}

		if (batch_count)
			renderer->buffer(batch_mode).append_instances(batch_mesh, batch_count, prop_batch);

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

// the display row and the player's armory read the same table : mesh, clip,
// which take starts it, and how long one cycle runs
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

// magazine size and the four action slices for each weapon. some takes are
// shared (the AK cycle, the bolt action) so the map is not always 0/1/2/3
struct Weapon_Action
{
	uint mag;
	uint reload, shoot, inspect, idle;
};

static const Weapon_Action WEAPON_ACTIONS[NUM_WEAPONS] = {
	{ 17, 0, 1, 2, 3 }, // glock
	{ 30, 0, 0, 0, 1 }, // ak
	{ 30, 1, 2, 0, 3 }, // m4
	{  8, 1, 0, 2, 3 }, // shotgun
	{  6, 3, 1, 2, 0 }, // colt
	{  5, 1, 0, 1, 2 }, // bolt action
	{ 17, 0, 1, 2, 3 }, // test pistol
};

struct Weapon_Actor_Ref
{
	uint mesh, player;
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
Weapon_Actor_Ref weapon_add_actor(Scene* s, GameRenderer* renderer, uint index, vec3 pos,
                                  float yaw, float scale)
{
	Weapon_Actor_Ref ref = {};
	if (index >= NUM_WEAPONS) return ref;

	uint mesh = renderer->add_anim_mesh(WEAPON_SETUP[index].mesh);
	uint clip = 0;
	renderer->animloader.load(WEAPON_SETUP[index].clip, &clip);
	if (!mesh || !clip) return ref;

	weapon_slice(index, &renderer->animloader.clips[clip]);

	uint player = renderer->animloader.play(clip, WEAPON_SETUP[index].seg,
	                                        WEAPON_SETUP[index].dur);
	if (player == (uint)-1) return ref;

	renderer->bind_player(mesh, player);
	s->add_actor(mesh, player, pos, yaw, scale);

	ref.mesh = mesh;
	ref.player = player;
	return ref;
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

// main menu backdrop : a quiet voxel stage behind the overlay
void scene_menu(Scene* s, GameRenderer* renderer)
{
	s->begin(renderer, vec3(0.f, 2.6f, 9.f), -PI / 2.f, -.10f, 32.f, 16.f, 8, 0.f);
	s->show_labels = false;
	s->shadow_extent = 32.f;
	s->far_plane = 220.f;
	s->use_lighting(LIGHT_STUDIO);

	Scene::Prop& floor = s->props[0];
	floor.mat_a = vec4(.045f, .050f, .058f, 0.f);
	floor.mat_b = vec4(.82f, 1.f, 0.f, 0.f);

	uint cube = renderer->add_mesh(MeshFactory::generate_cube(1.f));
	static const vec3 colours[6] = {
		vec3(.34f, .48f, .38f), vec3(.46f, .38f, .28f), vec3(.28f, .34f, .44f),
		vec3(.52f, .34f, .30f), vec3(.42f, .40f, .34f), vec3(.30f, .42f, .44f)
	};

	for (int i = 0; i < 6; i++)
	{
		float x = ((float)i - 2.5f) * 1.30f;
		float h = .55f + (float)((i * 7) % 5) * .28f;
		Scene::Prop& block = s->add_prop(cube, vec3(x, h * .5f, -1.5f), 0.f,
		                                 vec3(1.05f, h, 1.05f));
		block.mat_a = vec4(colours[i], 0.f);
		block.mat_b = vec4(.72f, 1.f, 0.f, 0.f);
	}

	s->finish(renderer, "menu");
}

void scene_menu_ux(Scene* s, GameWindow* window)
{
	(void)s; (void)window;
}

void scene_menu_debug(Scene* s)
{
	(void)s;
	ImGui::TextDisabled("main menu backdrop");
}

// ---- render : a studio lab for materials, the metal/roughness matrix and texture
void scene_render(Scene* s, GameRenderer* renderer)
{
	render_controls = {};
	render_controls.hero_prop = (uint)-1;
	render_default_material();

	s->begin(renderer, vec3(0, 2.4f, 10.f), -PI / 2, -0.10f, 30.f, 15.f);
	s->shadow_extent = 30.f;
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

	// local-light stress row : eight volumes over the material line. each sphere
	// is the light's reach, so the cost rises with screen coverage, not count
	for (uint i = 0; i < 8; i++)
	{
		float t = (float)i / 7.f;
		vec3 color = vec3(1.f - t, .25f + t * .55f, t);
		vec3 at = vec3(-7.f + t * 14.f, 2.6f, -3.2f);

		s->add_light(at, color, 5.5f, 3.2f);

		Scene::Prop& bulb = s->add_prop(sphere_mesh, at, 0, .11f);
		bulb.mat_a = vec4(color, 0.f);
		bulb.mat_b = vec4(.25f, 1.f, 0.f, 0.f);
	}

	s->player.init(renderer, &s->phys, vec3(0.f, 2.2f, 10.f), 0, 0);
	s->finish(renderer, "render");

	// props are sorted by mesh in finish(), so find the hero afterwards
	for (uint i = 0; i < s->num_props; i++)
		if (strcmp(s->props[i].label, "hero") == 0)
			render_controls.hero_prop = i;
}
/* ------------------------- *
	-         Water         -
 * ------------------------- */

const float WATER_LEVEL = 1.6f;  // mean surface height, metres
const uint  WATER_TILES = 9;     // one patch repeated into a sea

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
	vec4 water_a = vec4(water_config.albedo, water_view.level); // a = mean surface height
	vec4 water_b = vec4(water_config.roughness, 1.f, 0.f, 1.f); // open sea : foam field

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
// caller owns finish()/prop resolution so other water scenes can share this
void water_build_content(Scene* s, GameRenderer* renderer, float level)
{
	water_build_spectrum(&s->water, water_config);
	water_build_detail(&s->water.detail, water_config);
	water_update(&s->water, water_config, 0.f, true);
	water_update_detail(&s->water.detail, s->water.time);

	DrawBuffer& buffer = renderer->buffer(MODE_WATER);
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

	water_view.water_mesh = renderer->add_mesh_water(water_build_mesh(&s->water), "foam_field");

	float tile = water_config.size;
	float origin = -((float)(WATER_TILES - 1) * .5f) * tile;
	for (uint j = 0; j < WATER_TILES; j++)
	{
		for (uint i = 0; i < WATER_TILES; i++)
		{
			vec3 pos = vec3(origin + i * tile, level, origin + j * tile);
			s->add_prop(water_view.water_mesh, pos, 0, 1.f).water = true;
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

// evolve the sim and refresh its two dynamic textures. no geometry, so water
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
	DrawBuffer& buffer = renderer->buffer(MODE_WATER);
	Mesh_Data mesh = water_build_mesh(&s->water);
	buffer.append_geometry(water_view.water_mesh, mesh, water_view.water_tx);
	mesh.release();
}

// per frame water uniforms : the deferred pass reads the foam tint and the
// short-chop normal map from the window, whichever scene owns the sea
void water_bind_window(GameWindow* window, float time)
{
	window->water_foam_color     = water_config.foam_albedo;
	window->water_foam_roughness = water_config.foam_roughness;
	window->water_normal_tx      = water_view.detail_tx;
	window->water_detail         = water_config.detail;
	window->water_sss            = water_config.sss;
	window->water_time           = time;
}

// the shallow patch has no second cascade : its mesh normals are the whole wave
void shallow_bind_window(GameWindow* window, float time)
{
	window->water_foam_color     = vec3(.82f, .88f, .90f);
	window->water_foam_roughness = .55f;
	window->water_normal_tx      = 0;
	window->water_detail         = 0.f;
	window->water_sss            = shallow_config.sss;
	window->water_time           = time;
}

// ---- water : one fft patch tiled into open sea
void scene_water(Scene* s, GameRenderer* renderer)
{
	s->begin(renderer, vec3(0, 8.f, 30.f), -PI / 2, -0.12f, 240.f, 120.f, 8, -5.f);
	s->shadow_extent = 60.f;
	s->use_lighting(LIGHT_CARIBBEAN);

	// seabed : a sandy shelf, far enough down that the troughs never reach it
	Scene::Prop& floor = s->props[0];
	floor.mat_a = vec4(.12f, .14f, .10f, 0.f);
	floor.mat_b = vec4(.92f, 1.f, 0.f, 0.f);

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
		DrawBuffer& buffer = renderer->buffer(MODE_WATER);
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
	p.mat_a = vec4(shallow_config.albedo, shallow_view.level);
	p.mat_b = vec4(shallow_config.roughness, 1.f, 0.f, 0.f);
}

// add the simulated patch at one spot : the caller builds the rim/platform
void shallow_build_content(Scene* s, GameRenderer* renderer, vec3 center, float level)
{
	shallow_reset(&s->shallow, shallow_config);
	shallow_drop(&s->shallow, 0.f, 0.f, shallow_config.drop_strength);
	shallow_view.level = level;

	DrawBuffer& buffer = renderer->buffer(MODE_WATER);
	shallow_view.mesh_mark = buffer.num_meshes;
	shallow_view.geom_mark = buffer.geom_size;
	shallow_view.indx_mark = buffer.indx_size;

	shallow_view.water_mesh = renderer->add_mesh_water(shallow_build_mesh(&s->shallow));
	Scene::Prop& water = s->add_prop(shallow_view.water_mesh,
	                                 vec3(center.x, level, center.z), 0, 1.f);
	water.water = true;
	water.mat_a = vec4(shallow_config.albedo, level);
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
	DrawBuffer& buffer = renderer->buffer(MODE_WATER);
	Mesh_Data mesh = shallow_build_mesh(&s->shallow);
	buffer.append_geometry(shallow_view.water_mesh, mesh, 0);
	mesh.release();
}

// ---- shallow : the finite-difference patch in a stone basin
void scene_shallow(Scene* s, GameRenderer* renderer)
{
	s->begin(renderer, vec3(0, 5.5f, 9.f), -PI / 2, -0.35f, 20.f, 10.f, 4, .5f);
	s->shadow_extent = 20.f;
	s->use_lighting(LIGHT_CARIBBEAN);

	Scene::Prop& floor = s->props[0];
	floor.mat_a = vec4(.46f, .40f, .30f, 0.f);
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
		wall.mat_a = vec4(.38f, .36f, .30f, 0.f);
		wall.mat_b = vec4(.84f, 1.f, 0.f, 0.f);
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

	DrawBuffer& buffer = renderer->buffer(MODE_WATER);
	buffer.rewind(shallow_view.mesh_mark, shallow_view.geom_mark, shallow_view.indx_mark);
	shallow_append_geometry(s, renderer);
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
const uint PHYSICS_TRIGGER_COUNT = 3;
const int  PHYS_FLAG_COLS = 16;
const int  PHYS_FLAG_ROWS = 10;
const int  PHYS_CLOTH_N   = 20;
const int  PHYS_CURTAIN_N = 14;

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
	uint cradle_last, stack_top, domino_last, bag_body, seesaw_body;
	uint ramp_ball, cloth_sphere, shape_cone;
	uint slider_body, fixed_anchor, fixed_body;

	// real Bullet soft bodies : patch nodes are pinned with setMass(...,0)
	btSoftBody* flag_body;
	btSoftBody* cloth_body;
	btSoftBody* curtain_body;
	int flag_tip_index;
	int cloth_center_index;
	float cloth_min_y;
	vec3 flag_tip_start, flag_base_start, cloth_center_start, ramp_ball_start;
	vec3 slider_start, fixed_start_a, fixed_start_b, cradle_last_start;

	// interaction test : a ray hits the cap, E latches it in
	uint button_body, button_prop;
	vec3 button_base;
	bool button_on, button_target;
	float button_press;

	// player trigger exhibits
	uint trigger_prop[PHYSICS_TRIGGER_COUNT];
	bool trigger_on[PHYSICS_TRIGGER_COUNT];

	float arm_limit, arm_push, arm_damping;
	float seesaw_limit;
	float rotor_speed, rotor_impulse;
	float door_speed, door_impulse, door_open_angle, door_damping, door_target;
	float gravity;
	float flag_wind;
	int   solver_iters;
	bool  rotor_on, door_on;
};

static Physics_Controls physics_controls;

// unit-test status : -1 pending, 0 fail, 1 pass
enum Physics_Test {
	PT_SHAPES = 0, PT_STACK, PT_DOMINOES, PT_HINGE, PT_CONE,
	PT_CHAIN, PT_SEESAW, PT_MOTOR, PT_CCD, PT_CRADLE,
	PT_RAMP, PT_FLAGPOLE, PT_CLOTH, PT_SLIDER, PT_FIXED,
	PT_COUNT
};

static const char* PHYSICS_TEST_NAMES[PT_COUNT] = {
	"shapes settle", "stack stable", "dominoes topple", "hinge swings",
	"cone hang", "chain links", "see-saw tilts", "motor turns",
	"ccd stops", "cradle transfers", "ramp rolls", "flag pole",
	"soft cloth", "slider travel", "fixed joint",
};

static int   physics_test_state[PT_COUNT];
static float physics_test_clock;
static bool  physics_tests_evaluated;

struct Physics_Soft_View
{
	uint flag_mesh, cloth_mesh, curtain_mesh;
	uint mesh_mark, geom_mark, indx_mark;
};

static Physics_Soft_View physics_soft_view;

// a hair of clearance : the motor should not be born against its closed stop
const float DOOR_CLOSED_LIMIT = -.01f;

void physics_default_settings()
{
	physics_controls.arm_limit = 60.f * PI / 180.f;
	physics_controls.arm_push = 3.f;
	physics_controls.arm_damping = .05f;
	physics_controls.seesaw_limit = .3f;

	physics_controls.rotor_speed = 3.f;
	physics_controls.rotor_impulse = 60.f;
	physics_controls.rotor_on = true;

	physics_controls.door_speed = 2.f;
	physics_controls.door_impulse = 30.f;
	physics_controls.door_open_angle = 90.f;
	physics_controls.door_damping = .2f;
	physics_controls.door_target = 0.f;
	physics_controls.door_on = false;

	physics_controls.gravity = 9.8f;
	physics_controls.flag_wind = 4.0f;
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
		physics_controls.rotor->enableAngularMotor(physics_controls.rotor_on,
		                                           physics_controls.rotor_speed,
		                                           physics_controls.rotor_impulse);
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
	{
		btRigidBody* b = s->phys.bodies[physics_controls.rotor_body].body;
		b->setActivationState(DISABLE_DEACTIVATION);
		b->activate(true);
	}

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
	PST_MOTOR, PST_CRADLE, PST_RAMP, PST_CCD, PST_DOOR,
	PST_SLIDER, PST_FIXED, PST_FLAGPOLE, PST_CLOTH, PST_CURTAIN,
	PST_COUNT
};

static const char* PHYSICS_STATIONS[PST_COUNT] = {
	"shapes", "stack", "dominoes", "ragdoll",
	"hinge", "cone twist", "chain", "see-saw",
	"motor", "newton's cradle", "ramp", "ccd", "door",
	"slider", "fixed", "flagpole", "soft cloth", "walk curtain",
};

static const vec3 PHYSICS_SIGN_AT[PST_COUNT] = {
	vec3(-12.f, 4.f, 4.f),   vec3(-4.f, 4.f, 4.f),    vec3(4.4f, 4.f, 4.f),
	vec3(12.f, 5.5f, 4.f),   vec3(-14.f, 4.f, -6.f),  vec3(-7.f, 4.f, -6.f),
	vec3(0.f, 6.f, -6.f),    vec3(7.f, 3.f, -6.f),    vec3(14.f, 4.f, -6.f),
	vec3(-9.f, 5.8f, -16.f), vec3(-1.f, 4.f, -16.f),  vec3(9.f, 5.f, -16.f),
	vec3(15.85f, 4.2f, -16.f),
	vec3(-14.f, 3.2f, -27.f), vec3(-5.f, 3.2f, -27.f),
	vec3(4.5f, 8.4f, -27.f),  vec3(12.f, 4.8f, -27.f),
	vec3(19.f, 4.8f, -33.f),
};

struct Physics_Trigger
{
	vec3  pos;    // floor centre
	float radius;
};

static const Physics_Trigger PHYSICS_TRIGGERS[PHYSICS_TRIGGER_COUNT] = {
	{ vec3(-9.75f, 0.f, 10.f), 1.75f },
	{ vec3(-6.00f, 0.f, 10.f), 1.75f },
	{ vec3(-2.25f, 0.f, 10.f), 1.75f },
};

const vec4 ACTIVE_OFF = vec4(.55f, .045f, .030f, 0.f); // red
const vec4 ACTIVE_ON  = vec4(.035f, .52f, .095f, 0.f); // green

void station_label(uint station)
{
	ImGui::TextDisabled("%u  %s", station + 1, PHYSICS_STATIONS[station]);
}

void physics_add_button(Scene* s, uint cube_mesh, uint cylinder_mesh)
{
	// console : base, panel, then a cap that sits just proud of the panel
	Scene::Prop& base = s->add_prop(cube_mesh, vec3(4.5f, .35f, 9.f), 0,
	                                vec3(1.2f, .7f, 1.2f));
	base.mat_a = vec4(.11f, .115f, .12f, 0.f);
	base.mat_b = vec4(.55f, 1.f, 0.f, 0.f);
	base.phys_id = s->phys.add_box(base.pos, vec3(.6f, .35f, .6f), 0);

	Scene::Prop& panel = s->add_prop(cube_mesh, vec3(4.5f, 1.55f, 9.f), 0,
	                                 vec3(1.4f, 1.7f, .2f));
	panel.mat_a = vec4(.06f, .065f, .07f, 0.f);
	panel.mat_b = vec4(.50f, 1.f, 0.f, 0.f);
	panel.phys_id = s->phys.add_box(panel.pos, vec3(.7f, .85f, .1f), 0);

	vec3 cap_at = vec3(4.5f, 1.55f, 9.28f);
	Scene::Prop& cap = s->add_prop(cylinder_mesh, cap_at, 0, vec3(.45f, .15f, .45f));
	cap.rot = mat3(glm::rotate(mat4(1), PI / 2, vec3(1, 0, 0)));
	cap.mat_a = ACTIVE_OFF;
	cap.mat_b = vec4(.25f, 1.f, 0.f, 0.f);
	snprintf(cap.label, sizeof(cap.label), "BTN");

	physics_controls.button_body = s->phys.add_box(cap_at, vec3(.3f, .3f, .16f), 0);
	physics_controls.button_base = cap_at;
	physics_controls.button_prop = (uint)-1;

	Scene::Prop& sign = s->add_prop(0, vec3(4.5f, 2.9f, 9.f), 0, 1.f);
	snprintf(sign.label, sizeof(sign.label), "INTERACT");
}

void physics_add_triggers(Scene* s, uint cube_mesh, uint sphere_mesh, uint cone_mesh)
{
	uint meshes[PHYSICS_TRIGGER_COUNT] = { cube_mesh, sphere_mesh, cone_mesh };
	float scales[PHYSICS_TRIGGER_COUNT] = { 1.1f, .6f, .8f };
	float heights[PHYSICS_TRIGGER_COUNT] = { 1.45f, 1.50f, 1.55f };

	for (uint i = 0; i < PHYSICS_TRIGGER_COUNT; i++)
	{
		vec3 at = PHYSICS_TRIGGERS[i].pos;

		Scene::Prop& plinth = s->add_prop(cube_mesh, at + vec3(0, .45f, 0), 0,
		                                  vec3(1.2f, .9f, 1.2f));
		plinth.mat_a = vec4(.08f, .085f, .09f, 0.f);
		plinth.mat_b = vec4(.62f, 1.f, 0.f, 0.f);
		plinth.phys_id = s->phys.add_box(plinth.pos, vec3(.6f, .45f, .6f), 0);

		Scene::Prop& shape = s->add_prop(meshes[i], vec3(at.x, heights[i], at.z), 0,
		                                 scales[i]);
		shape.mat_a = ACTIVE_OFF;
		shape.mat_b = vec4(.28f, 1.f, 0.f, 0.f);
		snprintf(shape.label, sizeof(shape.label), "T%u", i + 1);

		physics_controls.trigger_prop[i] = (uint)-1;
		physics_controls.trigger_on[i] = false;
	}

	Scene::Prop& sign = s->add_prop(0, vec3(-6.f, 2.75f, 10.f), 0, 1.f);
	snprintf(sign.label, sizeof(sign.label), "ACTIVATION");
}

// tag props back to their handles after finish() has sorted them by mesh
void physics_resolve_interactive(Scene* s)
{
	for (uint i = 0; i < PHYSICS_TRIGGER_COUNT; i++)
	{
		char tag[8];
		snprintf(tag, sizeof(tag), "T%u", i + 1);

		for (uint j = 0; j < s->num_props; j++)
		{
			if (strcmp(s->props[j].label, tag) != 0) continue;
			physics_controls.trigger_prop[i] = j;
			s->props[j].label[0] = 0;
			break;
		}
	}

	for (uint j = 0; j < s->num_props; j++)
	{
		if (strcmp(s->props[j].label, "BTN") != 0) continue;
		physics_controls.button_prop = j;
		physics_controls.button_base = s->props[j].pos;
		physics_controls.button_press = 0.f;
		s->props[j].label[0] = 0;
		break;
	}
}

bool physics_body_pos(Scene* s, uint id, vec3* out)
{
	if (!id || id >= s->phys.num_bodies) return false;
	btRigidBody* b = s->phys.bodies[id].body;
	if (!b) return false;
	btVector3 p = b->getWorldTransform().getOrigin();
	*out = vec3(p.x(), p.y(), p.z());
	return true;
}

bool physics_body_up(Scene* s, uint id, vec3* out)
{
	if (!id || id >= s->phys.num_bodies) return false;
	btRigidBody* b = s->phys.bodies[id].body;
	if (!b) return false;
	btVector3 u = b->getWorldTransform().getBasis().getColumn(1);
	*out = vec3(u.x(), u.y(), u.z());
	return true;
}

// a real Bullet soft-body flag : one pinned hoist column, the rest free.
void physics_add_flag(Scene* s, uint cube_mesh, uint cylinder_mesh)
{
	vec3 base = vec3(4.6f, 0.f, -27.f);

	Scene::Prop& pole = s->add_prop(cylinder_mesh, base + vec3(0, 3.6f, 0), 0,
	                                vec3(.18f, 3.6f, .18f));
	pole.mat_a = vec4(.42f, .43f, .44f, .55f);
	pole.mat_b = vec4(.45f, 1.f, 0.f, 0.f);
	pole.phys_id = s->phys.add_cylinder(base + vec3(0, 3.6f, 0),
	                                    vec3(.09f, 3.6f, .09f), 0);

	int cols = PHYS_FLAG_COLS, rows = PHYS_FLAG_ROWS;
	float sx = .22f, sy = .21f;
	btVector3 c00(base.x + .10f, base.y + 6.8f, base.z + .02f);
	btVector3 c10(c00.x() + (cols - 1) * sx, c00.y(), c00.z());
	btVector3 c01(c00.x(), c00.y() - (rows - 1) * sy, c00.z());
	btVector3 c11(c10.x(), c01.y(), c00.z());

	btSoftBody* sb = s->phys.add_soft_patch(c00, c10, c01, c11, cols, rows, 1.6f);
	physics_controls.flag_body = sb;
	if (!sb) return;

	// pin only the vertices attached to the pole : the rest of the flag is free
	// to billow and flutter in the wind
	for (int y = 0; y < rows; y++)
		sb->setMass(y * cols, 0.f);

	btVector3 base_node = sb->m_nodes[0].m_x;
	physics_controls.flag_base_start = vec3(base_node.x(), base_node.y(), base_node.z());

	physics_controls.flag_tip_index = (rows - 1) * cols + (cols - 1);
	btVector3 tip = sb->m_nodes[physics_controls.flag_tip_index].m_x;
	physics_controls.flag_tip_start = vec3(tip.x(), tip.y(), tip.z());
}

// a suspended soft-body cloth : pinned perimeter, free interior and a heavy
// rigid sphere dropped onto the centre.
void physics_add_soft_cloth(Scene* s, uint cube_mesh, uint sphere_mesh)
{
	vec3 centre = vec3(11.5f, 2.0f, -27.f);
	int n = PHYS_CLOTH_N;
	float step = .14f;
	float half = (n - 1) * .5f * step;

	btVector3 c00(centre.x - half, centre.y, centre.z - half);
	btVector3 c10(centre.x + half, centre.y, centre.z - half);
	btVector3 c01(centre.x - half, centre.y, centre.z + half);
	btVector3 c11(centre.x + half, centre.y, centre.z + half);

	btSoftBody* sb = s->phys.add_soft_patch(c00, c10, c01, c11, n, n, 3.0f);
	physics_controls.cloth_body = sb;
	if (!sb) return;

	for (int y = 0; y < n; y++)
		for (int x = 0; x < n; x++)
			if (x == 0 || y == 0 || x == n - 1 || y == n - 1)
				sb->setMass(y * n + x, 0.f);

	physics_controls.cloth_center_index = (n / 2) * n + n / 2;
	btVector3 cp = sb->m_nodes[physics_controls.cloth_center_index].m_x;
	physics_controls.cloth_center_start = vec3(cp.x(), cp.y(), cp.z());
	physics_controls.cloth_min_y = cp.y();

	Scene::Prop& ball = s->add_prop(sphere_mesh, centre + vec3(0, .70f, 0), 0, .50f);
	ball.mat_a = vec4(.24f, .26f, .28f, .15f);
	ball.mat_b = vec4(.36f, 1.f, 0.f, 0.f);
	ball.phys_id = s->phys.add_sphere(ball.pos, .50f, 2.6f);
	s->phys.enable_ccd(ball.phys_id);
	physics_controls.cloth_sphere = ball.phys_id;

	// a thin frame around the suspended perimeter
	for (int i = 0; i < 4; i++)
	{
		bool along_x = (i < 2);
		float sign = (i & 1) ? 1.f : -1.f;
		vec3 at = centre + (along_x ? vec3(0, -.12f, sign * half)
		                            : vec3(sign * half, -.12f, 0));
		vec3 dims = along_x ? vec3(half + .35f, .12f, .12f)
		                    : vec3(.12f, .12f, half + .35f);
		Scene::Prop& beam = s->add_prop(cube_mesh, at, 0, dims * 2.f);
		beam.mat_a = vec4(.18f, .19f, .20f, .35f);
		beam.mat_b = vec4(.55f, 1.f, 0.f, 0.f);
	}
}

Mesh_Data physics_soft_mesh(btSoftBody* sb, int cols, int rows)
{
	Mesh_Data data = {};
	if (!sb || sb->m_nodes.size() == 0 || sb->m_faces.size() == 0) return data;

	uint nv = (uint)sb->m_nodes.size();
	uint nf = (uint)sb->m_faces.size();

	data.num_vertices = nv;
	data.num_indices = nf * 3;
	data.positions = Alloc(vec3, nv);
	data.normals = Alloc(vec3, nv);
	data.uvs = Alloc(vec2, nv);
	data.indices = Alloc(uint, nf * 3);

	for (uint i = 0; i < nv; i++)
	{
		const btVector3& p = sb->m_nodes[i].m_x;
		data.positions[i] = vec3(p.x(), p.y(), p.z());
		data.normals[i] = vec3(0.f);
		float u = cols > 1 ? (float)(i % (uint)cols) / (float)(cols - 1) : 0.f;
		float v = (cols > 0 && rows > 1) ? (float)(i / (uint)cols) / (float)(rows - 1) : 0.f;
		data.uvs[i] = vec2(u, v);
	}

	for (uint i = 0; i < nf; i++)
	{
		const btSoftBody::Face& f = sb->m_faces[i];
		const btSoftBody::Node* base = &sb->m_nodes[0];
		uint a = (uint)(f.m_n[0] - base);
		uint b = (uint)(f.m_n[1] - base);
		uint c = (uint)(f.m_n[2] - base);
		data.indices[i * 3 + 0] = a;
		data.indices[i * 3 + 1] = b;
		data.indices[i * 3 + 2] = c;

		vec3 n = cross(data.positions[b] - data.positions[a],
		               data.positions[c] - data.positions[a]);
		data.normals[a] += n;
		data.normals[b] += n;
		data.normals[c] += n;
	}

	for (uint i = 0; i < nv; i++)
	{
		float l = length(data.normals[i]);
		data.normals[i] = l > .0001f ? data.normals[i] / l : vec3(0, 1, 0);
	}

	return data;
}

// built after finish() so the soft meshes are the tail of the static buffer and
// can be replaced in place each frame
void physics_build_soft_meshes(Scene* s, GameRenderer* renderer)
{
	physics_soft_view = {};
	DrawBuffer& buffer = renderer->buffer(MODE_STATIC_UV);
	physics_soft_view.mesh_mark = buffer.num_meshes;
	physics_soft_view.geom_mark = buffer.geom_size;
	physics_soft_view.indx_mark = buffer.indx_size;

	if (physics_controls.flag_body)
	{
		Mesh_Data mesh = physics_soft_mesh(physics_controls.flag_body, PHYS_FLAG_COLS, PHYS_FLAG_ROWS);
		physics_soft_view.flag_mesh = renderer->add_mesh(mesh);
		Scene::Prop& p = s->add_prop(physics_soft_view.flag_mesh, vec3(0), 0, 1.f);
		p.mat_a = vec4(.55f, .15f, .13f, 0.f);
		p.mat_b = vec4(.86f, 1.f, 2.f, 0.f);
	}

	if (physics_controls.cloth_body)
	{
		Mesh_Data mesh = physics_soft_mesh(physics_controls.cloth_body, PHYS_CLOTH_N, PHYS_CLOTH_N);
		physics_soft_view.cloth_mesh = renderer->add_mesh(mesh);
		Scene::Prop& p = s->add_prop(physics_soft_view.cloth_mesh, vec3(0), 0, 1.f);
		p.mat_a = vec4(.42f, .37f, .29f, 0.f);
		p.mat_b = vec4(.94f, 1.f, 2.f, 0.f);
	}

	if (physics_controls.curtain_body)
	{
		Mesh_Data mesh = physics_soft_mesh(physics_controls.curtain_body, PHYS_CURTAIN_N, PHYS_CURTAIN_N);
		physics_soft_view.curtain_mesh = renderer->add_mesh(mesh);
		Scene::Prop& p = s->add_prop(physics_soft_view.curtain_mesh, vec3(0), 0, 1.f);
		p.mat_a = vec4(.24f, .34f, .30f, 0.f);
		p.mat_b = vec4(.96f, 1.f, 2.f, 0.f);
	}
}

void physics_update_soft_meshes(GameRenderer* renderer)
{
	if (!physics_soft_view.flag_mesh && !physics_soft_view.cloth_mesh &&
	    !physics_soft_view.curtain_mesh) return;

	DrawBuffer& buffer = renderer->buffer(MODE_STATIC_UV);
	buffer.rewind(physics_soft_view.mesh_mark, physics_soft_view.geom_mark,
	              physics_soft_view.indx_mark);

	if (physics_soft_view.flag_mesh)
	{
		Mesh_Data mesh = physics_soft_mesh(physics_controls.flag_body, PHYS_FLAG_COLS, PHYS_FLAG_ROWS);
		buffer.append_geometry(physics_soft_view.flag_mesh, mesh, 0);
	}
	if (physics_soft_view.cloth_mesh)
	{
		Mesh_Data mesh = physics_soft_mesh(physics_controls.cloth_body, PHYS_CLOTH_N, PHYS_CLOTH_N);
		buffer.append_geometry(physics_soft_view.cloth_mesh, mesh, 0);
	}
	if (physics_soft_view.curtain_mesh)
	{
		Mesh_Data mesh = physics_soft_mesh(physics_controls.curtain_body, PHYS_CURTAIN_N, PHYS_CURTAIN_N);
		buffer.append_geometry(physics_soft_view.curtain_mesh, mesh, 0);
	}
}

// a walk-through soft curtain hanging in a doorway
void physics_add_curtain(Scene* s, uint cube_mesh)
{
	vec3 centre = vec3(18.1f, 0.f, -33.f);
	float half_w = 1.1f;
	float top = 3.15f;
	float bottom = .35f;

	for (int i = -1; i <= 1; i += 2)
	{
		vec3 at = centre + vec3((float)i * (half_w + .18f), 1.55f, 0.f);
		Scene::Prop& post = s->add_prop(cube_mesh, at, 0, vec3(.18f, 3.1f, .18f));
		post.mat_a = vec4(.20f, .21f, .22f, .40f);
		post.mat_b = vec4(.58f, 1.f, 0.f, 0.f);
		post.phys_id = s->phys.add_box(at, vec3(.09f, 1.55f, .09f), 0);
	}

	vec3 bar_at = centre + vec3(0.f, 3.18f, 0.f);
	Scene::Prop& bar = s->add_prop(cube_mesh, bar_at, 0, vec3(2.6f, .14f, .14f));
	bar.mat_a = vec4(.20f, .21f, .22f, .40f);
	bar.mat_b = vec4(.58f, 1.f, 0.f, 0.f);
	bar.phys_id = s->phys.add_box(bar_at, vec3(1.3f, .07f, .07f), 0);

	btVector3 c00(centre.x - half_w, top, centre.z);
	btVector3 c10(centre.x + half_w, top, centre.z);
	btVector3 c01(centre.x - half_w, bottom, centre.z);
	btVector3 c11(centre.x + half_w, bottom, centre.z);

	btSoftBody* sb = s->phys.add_soft_patch(c00, c10, c01, c11,
	                                        PHYS_CURTAIN_N, PHYS_CURTAIN_N, 1.2f);
	physics_controls.curtain_body = sb;
	if (!sb) return;

	for (int x = 0; x < PHYS_CURTAIN_N; x++)
		sb->setMass(x, 0.f); // top edge, hanging from the bar
}

// ---- physics : one labelled station per rigid-body test
void scene_physics(Scene* s, GameRenderer* renderer)
{
	physics_controls = {};
	physics_default_settings();

	s->begin(renderer, vec3(0, 3.f, 14.f), -PI / 2, -0.16f, 80.f, 40.f);
	s->shadow_extent = 35.f;
	s->use_lighting(LIGHT_NOON);

	// shared meshes : every station draws from this small set
	uint cube_mesh     = renderer->add_mesh(MeshFactory::generate_cube(1.f));
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

	physics_add_button(s, cube_mesh, cylinder_mesh);
	physics_add_triggers(s, cube_mesh, sphere_mesh, cone_mesh);
	physics_add_flag(s, cube_mesh, cylinder_mesh);
	physics_add_soft_cloth(s, cube_mesh, sphere_mesh);
	physics_add_curtain(s, cube_mesh);

	for (uint i = 0; i < PT_COUNT; i++) physics_test_state[i] = -1;
	physics_test_clock = 0.f;
	physics_tests_evaluated = false;

	// constraints and the interaction ray are the scene's debug subject
	s->phys.debug.constraints = true;
	s->phys.debug.rays = true;

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
		physics_controls.shape_cone = cone.phys_id;
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
		if (i == 5) physics_controls.stack_top = box.phys_id;
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
		if (i == 5) physics_controls.domino_last = domino.phys_id;
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
		physics_controls.bag_body = bag.phys_id;

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
		physics_controls.seesaw_body = plank.phys_id;

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

		Scene::Prop& rotor = s->add_prop(cube_mesh, vec3(15.2f, 3.5f, -6.f), 0, vec3(2.4f, .16f, .16f));
		rotor.phys_id = s->phys.add_box(rotor.pos, vec3(1.2f, .08f, .08f), 2.f);
		physics_controls.rotor_body = rotor.phys_id;

		physics_controls.rotor = s->phys.add_hinge(
			*s->phys.bodies[post.phys_id].body, *s->phys.bodies[rotor.phys_id].body,
			joint_frame(1.f, vec3(-PI * .5f, 0, 0), vec3(0, 2.0f, 0)),
			joint_frame(1.f, vec3(-PI * .5f, 0, 0), vec3(-1.2f, 0, 0)),
			-PI, PI, physics_controls.rotor_speed, physics_controls.rotor_impulse, false);
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
			if (i == 4)
			{
				physics_controls.cradle_last = bob.phys_id;
				physics_body_pos(s, bob.phys_id, &physics_controls.cradle_last_start);
			}
		}
		if (first) s->phys.apply_impulse(first, vec3(25.f, 0, 0));
	}

	// ramp : a sphere and a cylinder roll down the inclined plane
	{
		Scene::Prop& ramp = s->add_prop(cube_mesh, vec3(-1.f, .72f, -16.f), 0, vec3(8.f, .4f, 4.f));
		ramp.rot = mat3(glm::rotate(mat4(1), -.18f, vec3(0, 0, 1)));
		ramp.phys_id = s->phys.add_box(ramp.pos, vec3(4.f, .2f, 2.f), 0, vec3(0, 0, -.18f));

		Scene::Prop& ball = s->add_prop(sphere_mesh, vec3(-4.4f, 2.2f, -16.f), 0, .5f);
		ball.phys_id = s->phys.add_sphere(ball.pos, .5f, 1.f);
		physics_controls.ramp_ball = ball.phys_id;
		physics_body_pos(s, ball.phys_id, &physics_controls.ramp_ball_start);

		Scene::Prop& roller = s->add_prop(cylinder_mesh, vec3(-4.4f, 2.6f, -14.8f), 0, .5f);
		roller.phys_id = s->phys.add_cylinder(roller.pos, vec3(.3f, .5f, .3f), 1.f);
	}

	// slider : a crate travels along a linear rail
	{
		Scene::Prop& rail = s->add_prop(cube_mesh, vec3(-13.5f, .55f, -27.f), 0,
		                                vec3(5.4f, .22f, .46f));
		rail.mat_a = vec4(.18f, .19f, .20f, .30f);
		rail.mat_b = vec4(.58f, 1.f, 0.f, 0.f);
		rail.phys_id = s->phys.add_box(rail.pos, vec3(2.7f, .11f, .23f), 0);

		Scene::Prop& crate = s->add_prop(cube_mesh, vec3(-13.5f, 1.25f, -27.f), 0,
		                                 vec3(.9f, .9f, .9f));
		crate.mat_a = vec4(.52f, .34f, .20f, .05f);
		crate.mat_b = vec4(.72f, 1.f, 0.f, 0.f);
		crate.phys_id = s->phys.add_box(crate.pos, vec3(.45f), 1.4f);
		physics_controls.slider_body = crate.phys_id;
		physics_body_pos(s, crate.phys_id, &physics_controls.slider_start);

		s->phys.add_slider(*s->phys.bodies[rail.phys_id].body,
		                   *s->phys.bodies[crate.phys_id].body,
		                   joint_frame(1.f, vec3(0), vec3(0, .70f, 0)),
		                   joint_frame(1.f, vec3(0), vec3(0)),
		                   -2.2f, 2.2f);
		s->phys.apply_impulse(crate.phys_id, vec3(2.2f, 0, 0));
	}

	// fixed : a crate welded to a static anchor
	{
		Scene::Prop& anchor = s->add_prop(cube_mesh, vec3(-5.f, .65f, -27.f), 0,
		                                  vec3(.9f, 1.3f, .9f));
		anchor.mat_a = vec4(.18f, .20f, .22f, .20f);
		anchor.mat_b = vec4(.68f, 1.f, 0.f, 0.f);
		anchor.phys_id = s->phys.add_box(anchor.pos, vec3(.45f, .65f, .45f), 0);

		Scene::Prop& crate = s->add_prop(cube_mesh, vec3(-5.f, 1.80f, -27.f), 0,
		                                 vec3(1.1f, 1.0f, 1.1f));
		crate.mat_a = vec4(.34f, .38f, .44f, .08f);
		crate.mat_b = vec4(.72f, 1.f, 0.f, 0.f);
		crate.phys_id = s->phys.add_box(crate.pos, vec3(.55f, .5f, .55f), 1.8f);
		physics_controls.fixed_anchor = anchor.phys_id;
		physics_controls.fixed_body = crate.phys_id;
		physics_body_pos(s, anchor.phys_id, &physics_controls.fixed_start_a);
		physics_body_pos(s, crate.phys_id, &physics_controls.fixed_start_b);

		s->phys.add_fixed(*s->phys.bodies[anchor.phys_id].body,
		                  *s->phys.bodies[crate.phys_id].body,
		                  joint_frame(1.f, vec3(0), vec3(0, .65f, 0)),
		                  joint_frame(1.f, vec3(0), vec3(0, -.5f, 0)));
		s->phys.apply_impulse(crate.phys_id, vec3(1.2f, 0, 0));
	}

	physics_apply_settings(s);

	s->player.init(renderer, &s->phys, vec3(0.f, 2.5f, 14.f), 0, 0);
	s->finish(renderer, "physics");
	physics_build_soft_meshes(s, renderer);
	physics_resolve_interactive(s);
}

// automated checks : run after the simulation has had time to settle. this is
// the unit-test half of the physics scene; the stations remain inspectable.
void physics_eval_tests(Scene* s)
{
	Physics_Controls& c = physics_controls;
	vec3 p, u;

	physics_test_state[PT_SHAPES] =
		physics_body_pos(s, c.shape_cone, &p) && p.y < 2.0f && p.y > .15f ? 1 : 0;

	physics_test_state[PT_STACK] =
		physics_body_pos(s, c.stack_top, &p) && p.y > 1.9f ? 1 : 0;

	bool domino_down = physics_body_up(s, c.domino_last, &u) && u.y < .88f;
	physics_test_state[PT_DOMINOES] = domino_down ? 1 : 0;

	physics_test_state[PT_HINGE] =
		physics_body_up(s, c.arm_body, &u) && u.y < .96f ? 1 : 0;

	physics_test_state[PT_CONE] =
		physics_body_pos(s, c.bag_body, &p) && p.y > 1.1f && p.y < 2.5f ? 1 : 0;

	physics_test_state[PT_CHAIN] =
		physics_body_pos(s, c.chain_end, &p) && p.y < 3.6f ? 1 : 0;

	physics_test_state[PT_SEESAW] =
		physics_body_up(s, c.seesaw_body, &u) && fabsf(u.x) > .03f ? 1 : 0;

	bool rotor_moving = false;
	if (c.rotor_body && c.rotor_body < s->phys.num_bodies &&
	    s->phys.bodies[c.rotor_body].body)
	{
		btRigidBody* b = s->phys.bodies[c.rotor_body].body;
		rotor_moving = b->getAngularVelocity().length() > .5f;
	}
	physics_test_state[PT_MOTOR] = rotor_moving ? 1 : 0;

	physics_test_state[PT_CCD] =
		physics_body_pos(s, c.ccd_on, &p) && p.x < 8.85f ? 1 : 0;

	bool cradle_moved = false;
	if (physics_body_pos(s, c.cradle_last, &p))
		cradle_moved = length(p - c.cradle_last_start) > .03f;
	physics_test_state[PT_CRADLE] = cradle_moved ? 1 : 0;

	physics_test_state[PT_RAMP] =
		physics_body_pos(s, c.ramp_ball, &p) &&
		length(p - c.ramp_ball_start) > .4f ? 1 : 0;

	bool flag_moved = false, flag_pinned = false;
	if (c.flag_body && c.flag_tip_index < c.flag_body->m_nodes.size())
	{
		const btVector3& tip = c.flag_body->m_nodes[c.flag_tip_index].m_x;
		flag_moved = length(vec3(tip.x(), tip.y(), tip.z()) - c.flag_tip_start) > .04f;

		const btVector3& root = c.flag_body->m_nodes[0].m_x;
		flag_pinned = length(vec3(root.x(), root.y(), root.z()) - c.flag_base_start) < .04f;
	}
	physics_test_state[PT_FLAGPOLE] = (flag_moved && flag_pinned) ? 1 : 0;

	bool ball_caught = false;
	vec3 ball_at;
	if (physics_body_pos(s, c.cloth_sphere, &ball_at))
		ball_caught = ball_at.y > c.cloth_center_start.y - .60f;
	physics_test_state[PT_CLOTH] =
		((c.cloth_center_start.y - c.cloth_min_y) > .08f && ball_caught) ? 1 : 0;

	physics_test_state[PT_SLIDER] =
		physics_body_pos(s, c.slider_body, &p) &&
		fabsf(p.x - c.slider_start.x) > .20f ? 1 : 0;

	bool fixed_ok = false;
	vec3 a, b;
	if (physics_body_pos(s, c.fixed_anchor, &a) &&
	    physics_body_pos(s, c.fixed_body, &b))
		fixed_ok = fabsf(length(b - a) - length(c.fixed_start_b - c.fixed_start_a)) < .12f;
	physics_test_state[PT_FIXED] = fixed_ok ? 1 : 0;

	uint passed = 0;
	for (uint i = 0; i < PT_COUNT; i++) if (physics_test_state[i] == 1) passed++;

	char msg[96];
	snprintf(msg, sizeof(msg), "Physics tests : %u/%u passed", passed, PT_COUNT);
	console->add_entry(msg, passed == PT_COUNT ? SUCCESS : FIXME, PHYS);

	physics_tests_evaluated = true;
}

// per frame : player triggers, the interaction ray, and the button cap
void scene_physics_update(Scene* s, GameWindow* window)
{
	Physics_Controls& c = physics_controls;
	vec3 eye = s->player.camera.position;

	physics_test_clock += window->dtime;

	if (c.cloth_body && c.cloth_center_index < c.cloth_body->m_nodes.size())
	{
		const btVector3& cp = c.cloth_body->m_nodes[c.cloth_center_index].m_x;
		if (cp.y() < c.cloth_min_y) c.cloth_min_y = cp.y();
	}

	if (!physics_tests_evaluated && physics_test_clock > 2.6f)
		physics_eval_tests(s);

	// motors do not wake their own bodies in Bullet, so a long-running motor
	// would otherwise fall asleep after the default deactivation time
	if (c.rotor_body && c.rotor_on && c.rotor_body < s->phys.num_bodies &&
	    s->phys.bodies[c.rotor_body].body)
		s->phys.bodies[c.rotor_body].body->activate(true);

	if (c.door_body && c.door_on && c.door_body < s->phys.num_bodies &&
	    s->phys.bodies[c.door_body].body)
		s->phys.bodies[c.door_body].body->activate(true);

	// flag wind : a gust along +z so the soft flag ripples without leaving
	// the camera's viewing plane
	if (c.flag_body)
	{
		float gust = c.flag_wind * (1.f + .18f * sinf(physics_test_clock * 2.2f));
		c.flag_body->setWindVelocity(btVector3(0.f, c.flag_wind * .22f, gust));
	}

	for (uint i = 0; i < PHYSICS_TRIGGER_COUNT; i++)
	{
		const Physics_Trigger& t = PHYSICS_TRIGGERS[i];
		float dx = eye.x - t.pos.x;
		float dz = eye.z - t.pos.z;
		bool on = dx * dx + dz * dz <= t.radius * t.radius &&
		          fabsf(eye.y - 1.35f) < 2.2f;

		if (on == c.trigger_on[i]) continue;
		c.trigger_on[i] = on;

		if (c.trigger_prop[i] < s->num_props)
			s->props[c.trigger_prop[i]].mat_a = on ? ACTIVE_ON : ACTIVE_OFF;
	}

	c.button_target = false;
	s->phys.debug.ray_active = false;
	s->phys.debug.ray_hit = false;
	s->phys.debug.ray_target = false;

	if (!s->player.flycam && s->player.body)
	{
		vec3 from = s->player.camera.position + s->player.camera.front * .5f;
		vec3 to = from + s->player.camera.front * 4.5f;
		Physics::Phys_Ray ray = s->phys.raycast(from, to);

		bool hit = ray.hit && ray.body_id != s->player.body;
		bool target = hit && ray.body_id == c.button_body;

		s->phys.debug.ray_active = true;
		s->phys.debug.ray_hit = hit;
		s->phys.debug.ray_target = target;
		s->phys.debug.ray_a = from;
		s->phys.debug.ray_b = to;
		s->phys.debug.ray_point = ray.point;
		c.button_target = target;

		ImGuiIO& io = ImGui::GetIO();
		bool use = window->focus_status == GAME && !io.WantCaptureKeyboard &&
		           window->keys.E.is_pressed && !window->keys.E.was_pressed;

		if (target && use) c.button_on = !c.button_on;
	}

	float press_target = c.button_on ? 1.f : 0.f;
	c.button_press += (press_target - c.button_press) * (1.f - expf(-14.f * window->dtime));

	if (c.button_prop < s->num_props)
	{
		Scene::Prop& cap = s->props[c.button_prop];
		cap.pos = c.button_base + vec3(0.f, 0.f, -.055f * c.button_press);
		cap.mat_a = c.button_on ? ACTIVE_ON : ACTIVE_OFF;
	}
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
	snprintf(sub, sizeof(sub), "%s  |  10 materials  |  8 local lights  |  5x5 metal / roughness", preset);

	hud_text_center(dl, 24.f * scale, cx, 36.f * scale, title, "RENDER");
	hud_text_center(dl, 14.f * scale, cx, 70.f * scale, dim, sub);
	hud_text_center(dl, 13.f * scale, cx, window->screen_height - 42.f * scale, dim,
	                "F fly to inspect  |  T console  |  hero material + lighting under Scenes");
}
// ---- physics : title, counts and the controls
// HUD helpers : scene UX draws on the background list, over the world and
// under the console panel. all sizes scale from the 1080p design base.
ImFont* hud_font_of(float size)
{
	float big_at = 20.f * ImGui::GetIO().FontGlobalScale;
	if (size >= big_at && hud_font_big) return hud_font_big;
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

void scene_physics_ux(Scene* s, GameWindow* window)
{
	ImDrawList* dl = ImGui::GetBackgroundDrawList();
	float scale = window->screen_height / 1080.f;
	float h  = (float)window->screen_height;

	float panel = ux_panel_visible ? ux_panel_width(window) : 0.f;
	float cx = (window->screen_width + panel) * .5f;

	ImU32 title = IM_COL32(235, 238, 243, 225);
	ImU32 dim   = IM_COL32(170, 178, 190, 200);
	ImU32 amber = IM_COL32(230, 184, 112, 235);

	uint active = 0;
	for (uint i = 0; i < PHYSICS_TRIGGER_COUNT; i++)
		if (physics_controls.trigger_on[i]) active++;

	char bodies[96];
	snprintf(bodies, sizeof(bodies), "%u bodies  |  %u joints  |  %u/%u triggers  |  button %s",
	         s->phys.num_bodies ? s->phys.num_bodies - 1 : 0, s->phys.num_joints,
	         active, PHYSICS_TRIGGER_COUNT, physics_controls.button_on ? "ON" : "OFF");

	uint passed = 0, failed = 0;
	for (uint i = 0; i < PT_COUNT; i++)
	{
		if (physics_test_state[i] == 1) passed++;
		else if (physics_test_state[i] == 0) failed++;
	}

	char tests[96];
	if (!physics_tests_evaluated)
		snprintf(tests, sizeof(tests), "unit tests running ...");
	else
		snprintf(tests, sizeof(tests), "unit tests %u/%u passed%s",
		         passed, PT_COUNT, failed ? "  |  some failed" : "  |  all green");

	hud_text_center(dl, 24.f * scale, cx, 36.f * scale, title, "PHYSICS");
	hud_text_center(dl, 14.f * scale, cx, 70.f * scale, dim, bodies);
	hud_text_center(dl, 13.f * scale, cx, 94.f * scale,
	                failed ? IM_COL32(235, 145, 120, 230) : dim, tests);

	if (!s->player.flycam)
	{
		hud_crosshair(dl, cx, h * .5f, scale,
		              physics_controls.button_target ? amber : IM_COL32(180, 188, 200, 95));

		if (physics_controls.button_target)
			hud_text_center(dl, 16.f * scale, cx, h * .5f + 42.f * scale, amber,
			                physics_controls.button_on ? "E  RELEASE" : "E  PRESS");
	}

	hud_text_center(dl, 13.f * scale, cx, h - 42.f * scale, dim,
	                "WASD move  |  SHIFT sprint  |  SPACE jump  |  E use  |  F flycam  |  T console  |  touch red shapes");
}
// global day/night control : presets set the clock, the slider moves it, and
// animate advances it. a static lighting preset turns the cycle back off
void scene_day_cycle_ui(Scene* s)
{
	if (ImGui::Checkbox("day cycle", &s->cycle))
	{
		if (s->cycle) s->lighting = lighting_from_time(s->time_of_day);
		else          s->cycle_playing = false;
	}

	float time = s->time_of_day;
	if (ImGui::SliderFloat("time of day", &time, 0.f, 24.f, "%.2f h"))
		s->use_time(time);

	if (ImGui::Checkbox("animate", &s->cycle_playing) && s->cycle_playing)
		s->cycle = true;

	if (s->cycle_playing)
	{
		ImGui::SameLine();
		ImGui::SetNextItemWidth(120.f);
		ImGui::DragFloat("speed", &s->time_speed, .01f, .01f, 2.f, "%.2f h/s");
	}

	struct Time_Preset { const char* name; float hours; };
	const Time_Preset presets[] = {
		{ "dawn", 6.f }, { "morning", 9.f }, { "noon", 12.f },
		{ "afternoon", 15.f }, { "sunset", 18.2f }, { "dusk", 20.f }, { "night", 23.f },
	};

	for (uint i = 0; i < 7; i++)
	{
		if (i) ImGui::SameLine();
		if (ImGui::SmallButton(presets[i].name)) s->use_time(presets[i].hours);
	}

	if (s->cycle)
	{
		float sun_elev = asinf(fminf(fmaxf(-s->lighting.sun_direction.y, -1.f), 1.f)) * 180.f / PI;
		float moon_elev = asinf(fminf(fmaxf(-s->lighting.moon_direction.y, -1.f), 1.f)) * 180.f / PI;
		ImGui::Text("%s  |  sun %+.0f deg  |  moon %+.0f deg",
		            s->cycle_playing ? "advancing" : "held", sun_elev, moon_elev);
		ImGui::TextDisabled("the cycle overrides the static lighting preset");
	}
	else
		ImGui::TextDisabled("static preset : move the slider or pick a time to start the cycle");
}

void scene_labels_checkbox(Scene* s)
{
	ImGui::Checkbox("world labels", &s->show_labels);
}

void scene_render_debug(Scene* s)
{
	int preset = (int)s->lighting_kind;
	if (ImGui::Combo("preset", &preset, LIGHTING_PRESET_NAMES, LIGHT_PRESET_COUNT))
		s->use_lighting((uint)preset);

	if (ImGui::Button("reset lighting")) s->lighting = lighting_preset(s->lighting_kind);
	ImGui::SameLine();
	scene_labels_checkbox(s);

	ImGui::Checkbox("local light volumes", &s->local_lights);
	ImGui::SameLine();
	ImGui::TextDisabled("%u lights", s->num_lights);

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
	ImGui::SliderFloat("contrast", &s->lighting.contrast, .80f, 1.30f, "%.3f");
	ImGui::SliderFloat("saturation", &s->lighting.saturation, .70f, 1.40f, "%.3f");
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
	ImGui::SliderFloat("subsurface", &water_config.sss, 0.f, 2.f, "%.2f");
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
	ImGui::SliderFloat("subsurface", &shallow_config.sss, 0.f, 2.f, "%.2f");
	shallow_apply_material(s);

	ImGui::Separator();
	ImGui::TextDisabled("lighting");
	int preset = (int)s->lighting_kind;
	if (ImGui::Combo("preset", &preset, LIGHTING_PRESET_NAMES, LIGHT_PRESET_COUNT))
		s->use_lighting((uint)preset);
	ImGui::SameLine();
	if (ImGui::Button("reset lighting")) s->lighting = lighting_preset(s->lighting_kind);
}

void physics_view_station(Scene* s, uint station)
{
	if (station >= PST_COUNT) return;

	vec3 at = PHYSICS_SIGN_AT[station];
	vec3 pos = at + vec3(0.f, 2.2f, 7.f);
	vec3 dir = at - pos;
	if (length(dir) < .01f) dir = vec3(0, 0, -1);
	dir = normalize(dir);

	s->player.flycam = true;
	s->camera.position = pos;
	s->camera.yaw = atan2f(dir.x, dir.z);
	s->camera.pitch = asinf(fmaxf(-1.f, fminf(1.f, dir.y)));
	s->camera.update_dir(0.f, 0.f, 0.f);
}

void scene_physics_debug(Scene* s)
{
	uint passed = 0, failed = 0;
	for (uint i = 0; i < PT_COUNT; i++)
	{
		if (physics_test_state[i] == 1) passed++;
		else if (physics_test_state[i] == 0) failed++;
	}

	if (!physics_tests_evaluated)
		ImGui::TextColored(ImVec4(.62f, .64f, .68f, 1.f), "tests running ...");
	else
		ImGui::TextColored(failed ? ImVec4(.95f, .45f, .35f, 1.f)
		                          : ImVec4(.45f, .85f, .55f, 1.f),
		                   "tests %u/%u passed", passed, PT_COUNT);

	ImGui::SameLine();
	if (ImGui::Button("rerun tests")) s->reload_requested = true;

	if (ImGui::CollapsingHeader("actions", ImGuiTreeNodeFlags_DefaultOpen))
	{
		if (ImGui::Button("wake all")) s->phys.wake_all();
		ImGui::SameLine();
		if (ImGui::Button("reset all")) s->phys.reset_all();
		ImGui::SameLine();
		if (ImGui::Button("kick everything"))
		{
			for (uint i = 1; i < s->phys.num_bodies; i++)
				if (i != s->player.body && s->phys.is_dynamic(i))
					s->phys.apply_impulse(i, vec3(1.5f, 0, 0));
		}
		ImGui::SameLine();
		scene_labels_checkbox(s);

		if (ImGui::Button("raycast"))
		{
			Camera& view = s->player.flycam ? s->camera : s->player.camera;
			vec3 from = view.position + view.front * .5f;
			Physics::Phys_Ray hit = s->phys.raycast(from, from + view.front * 50.f);

			char msg[96];
			if (hit.hit)
				snprintf(msg, sizeof(msg), "Ray hit body %u at %.1f, %.1f, %.1f",
				         hit.body_id, hit.point.x, hit.point.y, hit.point.z);
			else
				snprintf(msg, sizeof(msg), "Ray missed");
			console->add_entry(msg, hit.hit ? SUCCESS : DEBUG, PHYS);
		}
	}

	if (ImGui::CollapsingHeader("unit tests", ImGuiTreeNodeFlags_DefaultOpen))
	{
		if (ImGui::BeginTable("physics_tests", 2, ImGuiTableFlags_SizingStretchSame))
		{
			for (uint i = 0; i < PT_COUNT; i++)
			{
				int state = physics_test_state[i];
				ImVec4 col = state == 1 ? ImVec4(.45f, .85f, .55f, 1.f)
				           : state == 0 ? ImVec4(.95f, .45f, .35f, 1.f)
				                        : ImVec4(.62f, .64f, .68f, 1.f);
				ImGui::TableNextColumn();
				ImGui::TextColored(col, "%s  %s", state == 1 ? "PASS" :
				                   state == 0 ? "FAIL" : "....",
				                   PHYSICS_TEST_NAMES[i]);
			}
			ImGui::EndTable();
		}
	}

	if (ImGui::CollapsingHeader("stations / jump"))
	{
		if (ImGui::BeginTable("physics_stations", 4, ImGuiTableFlags_SizingStretchSame))
		{
			for (uint i = 0; i < PST_COUNT; i++)
			{
				ImGui::TableNextColumn();
				station_label(i);
				ImGui::SameLine();
				char go[16];
				snprintf(go, sizeof(go), "go##%u", i);
				if (ImGui::SmallButton(go))
					physics_view_station(s, i);
			}
			ImGui::EndTable();
		}
	}

	if (ImGui::CollapsingHeader("soft bodies", ImGuiTreeNodeFlags_DefaultOpen))
	{
		if (physics_controls.flag_body)
		{
			int nodes = physics_controls.flag_body->m_nodes.size();
			int faces = physics_controls.flag_body->m_faces.size();
			ImGui::Text("flag  %d nodes  %d faces  pinned hoist", nodes, faces);
		}
		if (physics_controls.cloth_body)
		{
			int nodes = physics_controls.cloth_body->m_nodes.size();
			int faces = physics_controls.cloth_body->m_faces.size();
			ImGui::Text("cloth  %d nodes  %d faces  pinned perimeter", nodes, faces);
		}
		if (physics_controls.curtain_body)
		{
			int nodes = physics_controls.curtain_body->m_nodes.size();
			int faces = physics_controls.curtain_body->m_faces.size();
			ImGui::Text("curtain  %d nodes  %d faces  walk-through", nodes, faces);
		}
		ImGui::SliderFloat("flag wind", &physics_controls.flag_wind, 0.f, 6.f, "%.1f m/s");
		if (ImGui::Button("rebuild soft bodies")) s->reload_requested = true;
		ImGui::SameLine();
		if (ImGui::Button("drop cloth ball"))
		{
			if (physics_controls.cloth_sphere)
			{
				s->phys.reset(physics_controls.cloth_sphere);
				s->phys.wake_all();
			}
		}
	}

	if (ImGui::CollapsingHeader("world"))
	{
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
	}

	if (!ImGui::CollapsingHeader("joint controls", ImGuiTreeNodeFlags_DefaultOpen))
	{
		physics_apply_settings(s);
		return;
	}

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
		ImGui::SliderFloat("rotor impulse", &physics_controls.rotor_impulse, 0.f, 120.f, "%.1f");
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

	physics_apply_settings(s);
	ImGui::TextDisabled("world signs use the same station numbers");
}
enum SceneKind {
	SCENE_MENU = 0,
	SCENE_PHYSICS,
	SCENE_RENDER,
	SCENE_WATER,
	SCENE_SHALLOW,
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
	{ "menu",      scene_menu,      scene_menu_ux,      scene_menu_debug },
	{ "physics",   scene_physics,   scene_physics_ux,   scene_physics_debug },
	{ "render",    scene_render,    scene_render_ux,    scene_render_debug },
	{ "ocean",     scene_water,     scene_water_ux,     scene_water_debug },
	{ "shallow",   scene_shallow,   scene_shallow_ux,   scene_shallow_debug },
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
		current_kind = requested_kind = SCENE_MENU;
		load(renderer, SCENE_MENU);
	}

	// ask for a scene : switching happens in update(), never mid-frame
	void request(uint kind)
	{
		if (kind < SCENE_KIND_COUNT) requested_kind = kind;
	}

	void update(GameWindow* window, GameRenderer* renderer)
	{
		bool reloading = scene.reload_requested || !loaded || requested_kind != current_kind;
		if (reloading) window->ui_lock = false;

		if (scene.reload_requested)
			load(renderer, current_kind);
		else if (!loaded || requested_kind != current_kind)
			load(renderer, requested_kind);

		// water evolves every frame and refills its geometry before the draw pass
		if (loaded && current_kind == SCENE_WATER)
		{
			water_scene_update(&scene, renderer, window->dtime);
			water_bind_window(window, scene.water.time);
		}

		// the shallow patch steps and rebuilds its small surface every frame
		if (loaded && current_kind == SCENE_SHALLOW)
		{
			shallow_scene_update(&scene, renderer, window->dtime);
			shallow_bind_window(window, scene.shallow.time);
		}

		// replace the soft-body geometry before instance submission : a rewind
		// after submission would clear the instances that point at those meshes
		if (loaded && current_kind == SCENE_PHYSICS)
			physics_update_soft_meshes(renderer);

		scene.update(window, renderer);

		// the physics scene reads the finished camera for its interaction ray
		if (loaded && current_kind == SCENE_PHYSICS)
			scene_physics_update(&scene, window);
	}

	// unload the current scene, then build the next one from its table entry
	void load(GameRenderer* renderer, uint kind)
	{
		if (loaded)
		{
			scene.shutdown(renderer);
			renderer->unload_scene();
		}

		if (kind >= SCENE_KIND_COUNT || !SCENE_DEFS[kind].build) kind = SCENE_MENU;
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