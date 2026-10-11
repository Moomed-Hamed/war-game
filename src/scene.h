#include "player.h"

/* ------------------------- *
	-        Scene          -
 * ------------------------- */

// what the world's guns are made of : dull steel, the mesh's texture keeps the detail
const vec3 GUN_ALBEDO = vec3(0.55f, 0.56f, 0.58f);

// the weapon row is a fixed set : one mesh + clip each, drawn abreast
const uint NUM_WEAPONS = 7;

// the scene owns the content AND the camera it's viewed from : the renderer
// only receives this frame's matrices and draws whatever the scene submits
struct GameScene
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

	// skinned weapons : each on its own animation player so they animate at once
	struct Weapon {
		uint mesh, clip, player;
		vec3 pos;    // spot on the weapon row
		float scale; // world size of the model
		float yaw;   // slight turn so the row reads with some depth
	} weapons[NUM_WEAPONS];

	// load everything this scene needs : the renderer owns the loaders & gpu buffers
	void init(GameRenderer* renderer)
	{
		// camera : stands back from the weapon row, looking slightly down
		camera.position = vec3(0, 2.6f, 10);
		camera.yaw   = -PI / 2; // front = (cos yaw, .., sin yaw) : face -z
		camera.pitch = -0.13f;
		camera.update_dir(0, 0, 0); // seeds front/right/up from yaw/pitch

		// floor : generated grid plane, the shader tiles its uvs (uv * 4)
		uint floor_mesh = renderer->add_mesh(
			MeshFactory::generate_plane(60.f, 60, MeshFactory::PLANE_XZ), "assets/textures/grid.bmp");

		// world content : every body starts at its authored spot
		phys.init();

		// floor : ground slab under it, top face exactly at y 0
		add_prop(floor_mesh, vec3(0), 0, 1.f).phys_id = phys.add_ground(30.f);

		// props section (left) : file-loaded art, static scenery
		uint palm_mesh = renderer->add_mesh("assets/meshes/palm_tree.mesh_uv");

		// palms : a thin trunk slab so nothing shoves them
		vec3 palm_spots[2] = { vec3(-13, 0, -2), vec3(-10, 0, -5) };
		for (uint i = 0; i < 2; i++)
		{
			Prop& palm = add_prop(palm_mesh, palm_spots[i], (float)i * 2.1f, 1.6f);
			palm.phys_id = phys.add_box(palm.pos + vec3(0, 3, 0), vec3(.2f, 3.f, .2f), 0);
		}

		// physics primitives section (right) : generated meshes, mostly dynamic
		uint sphere_mesh   = renderer->add_mesh(MeshFactory::generate_uv_sphere(1.f, 24, 16));
		uint cube_mesh     = renderer->add_mesh(MeshFactory::generate_cube(1.f), "assets/textures/checker.bmp");
		uint cylinder_mesh = renderer->add_mesh(MeshFactory::generate_cylinder(0.75f, 2.5f, 24));
		uint capsule_mesh  = renderer->add_mesh(MeshFactory::generate_capsule(0.6f, 1.2f, 20, 10));
		uint ammo_mesh     = renderer->add_mesh("assets/meshes/ammo.mesh_uv");

		// cylinder : stands on its base, y = where the centered mesh rests on the floor
		Prop& cylinder = add_prop(cylinder_mesh, vec3(8, 1.25f, 2), 0, 1.f);
		cylinder.phys_id = phys.add_cylinder(cylinder.pos, vec3(0.75f, 1.25f, 0.75f), 1.f);

		// cone : drops in beside the cylinder, for inspection
		uint cone_mesh = renderer->add_mesh(MeshFactory::generate_cone(0.5f, 1.5f, 24));
		Prop& cone = add_prop(cone_mesh, vec3(9.5f, 2.5f, 4), 0, 1.f);
		cone.phys_id = phys.add_cone(cone.pos, 0.5f, 1.5f, 1.f);

		// sphere : drops in from above and lands back on its spot
		Prop& sphere = add_prop(sphere_mesh, vec3(1.5f, 4.5f, 14.5f), 0, 1.5f);
		sphere.phys_id = phys.add_sphere(sphere.pos, 1.5f, 1.f);

		// capsule : leans toward the pyramid, so its topple sweeps the base row
		// and the stack comes down. radius + height match the mesh exactly
		Prop& capsule = add_prop(capsule_mesh, vec3(12, 4.5f, 3.2f), 0, 1.f);
		capsule.phys_id = phys.add_capsule(capsule.pos, 0.6f, 1.2f, 15.f, vec3(-0.65f, 0, 0.18f));

		// ammo : one little box dropping in beside the rest
		Prop& ammo = add_prop(ammo_mesh, vec3(8.5f, 2.5f, 6.5f), 0.3f, 2.f);
		ammo.phys_id = phys.add_box(ammo.pos, vec3(.15f, .1f, .15f), .5f);

		// cube pyramid : 3-2-1, rows at resting heights. the capsule topples it,
		// the sleep-respawn loop builds it back
		vec3 pyramid_spots[6] = {
			vec3(11, .5f, 2.5f), vec3(12, .5f, 2.5f), vec3(13, .5f, 2.5f),
			vec3(11.5f, 1.5f, 2.5f), vec3(12.5f, 1.5f, 2.5f),
			vec3(12, 2.5f, 2.5f),
		};
		for (uint i = 0; i < 6; i++)
		{
			Prop& box = add_prop(cube_mesh, pyramid_spots[i], 0, 1.f);
			box.phys_id = phys.add_box(box.pos, vec3(.5f), 1.f);
		}

		// ragdoll (front right) : linked capsules that crumple where they stand
		uint ragdoll = phys.add_ragdoll(vec3(5, 0.05f, 4), 1.4f);
		for (uint i = 0; ragdoll && i < 11; i++)
		{
			uint id = ragdoll + i;
			vec3 d = phys.bodies[id].dims; // (radius, cylinder height)
			Prop& part = add_prop(renderer->add_mesh(MeshFactory::generate_capsule(d.x, d.y, 20, 10)),
			                      vec3(0), 0, 1.f);
			part.phys_id = id;
		}

		// vehicle (front left) : chassis and wheels drawn from their colliders.
		// own meshes : a mesh id's instances must form one contiguous run in the
		// draw buffer, and the pyramid already uses cube_mesh / the cylinder
		// prop uses cylinder_mesh, both appended long before the car
		if (phys.add_vehicle(vec3(-6, 1.5f, 4)))
		{
			// unit cube : scale = full extents (2x the half extents)
			uint chassis_mesh = renderer->add_mesh(MeshFactory::generate_cube(1.f));
			Prop& chassis = add_prop(chassis_mesh, vec3(0), 0, vec3(2.f, 1.f, 4.f));
			chassis.phys_id = phys.vehicle.chassis_id;

			// wheel mesh built at true size (radius .5, height .4) : scale 1
			uint wheel_mesh = renderer->add_mesh(MeshFactory::generate_cylinder(.5f, .4f, 24));
			for (uint i = 0; i < 4; i++)
			{
				Prop& wheel = add_prop(wheel_mesh, vec3(0), 0, 1.f);
				wheel.phys_id = phys.vehicle.wheel_ids[i];
			}
		}

		// weapons row : seven skinned meshes, each with its own clip sliced into
		// segments as it plays
		struct { const char* mesh, *clip; uint seg; float dur; } setup[NUM_WEAPONS] = {
			{ "assets/anim/glock.mesh_anim",       "assets/anim/glock.anim",       0, 2.0f },
			{ "assets/anim/ak.mesh_anim",          "assets/anim/ak.anim",          0, 2.6f },
			{ "assets/anim/m4.mesh_anim",          "assets/anim/m4.anim",          1, 2.3f },
			{ "assets/anim/shotgun.mesh_anim",     "assets/anim/shotgun.anim",     1, 3.2f },
			{ "assets/anim/pistol_colt.mesh_anim", "assets/anim/pistol_colt.anim", 1, 1.6f },
			{ "assets/anim/bolt_action.mesh_anim", "assets/anim/bolt_action.anim", 0, 1.4f },
			{ "assets/anim/testpistol.mesh_anim",  "assets/anim/testpistol.anim",  0, 3.6f },
		};

		for (uint i = 0; i < NUM_WEAPONS; i++)
		{
			Weapon& w = weapons[i];
			w.mesh = renderer->add_anim_mesh(setup[i].mesh);
			renderer->animloader.load(setup[i].clip, &w.clip);

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
				case 6: // testpistol : a reload take and nothing else yet. the row plays that
				        // whole ; the view model holds frame 0 for the actions with no take
					clip->segs[0] = {  0, 85, 0, 0.40f }; // reload
					clip->segs[1] = {  0,  0, 0, 0.40f }; // shoot : no take yet
					clip->segs[2] = {  0,  0, 0, 0.40f }; // inspect : no take yet
					clip->segs[3] = {  0,  0, 0, 0.40f }; // idle : no take yet
					clip->num_segs = 4;
					break;
			}

			w.player = renderer->animloader.play(w.clip, setup[i].seg, setup[i].dur);
			if (w.mesh) renderer->bind_player(w.mesh, w.player);

			// row layout : animations section (center back), raised near eye level
			// and spaced wide so each one reads on its own
			w.pos = vec3((i - (NUM_WEAPONS - 1) * 0.5f) * 4.f, 1.6f, -3); // centred on the spawn
			w.scale = 4.f;
			w.yaw = (i % 2) ? 0.17f : -0.17f; // +-10 degrees
		}

		// player last : its view model shares the tool pistol's clip, sliced above
		player.init(renderer, &phys, weapons[6].mesh);

		add_material_grid(renderer);
	}

	// ---- material study : the range the deferred lighting can show, as a wall of
	// spheres facing the spawn. the bottom two rows name the materials worth having,
	// above them metallic sweeps upward and roughness across. all static, and all
	// sharing one mesh : the whole wall is a single instanced draw call
	void add_material_grid(GameRenderer* renderer)
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

		// own mesh, not the physics sphere's : a mesh id's instances have to sit in
		// one contiguous run, and the props in between would split it. two walls : the
		// same materials once through the uv layout (mode 1) and once through the no-uv
		// one (mode 3), side by side, so the two can be compared. the second wall is the
		// same sphere with its uv block dropped
		uint sphere_uv   = renderer->add_mesh(MeshFactory::generate_uv_sphere(1.f, 32, 20));
		uint sphere_nouv = renderer->add_mesh_no_uv(
			MeshFactory::without_uv(MeshFactory::generate_uv_sphere(1.f, 32, 20)));

		const uint  columns = 5;     // five to a row, the rows stack up each wall
		const float radius  = 0.22f; // small : a wall of beads, not boulders
		const float spacing = 0.75f; // still leaves the gap the labels sit in (2r + 0.25)
		const float wall_z  = -9.f;  // clear of the weapon row, facing the spawn
		const float base_y  = 0.90f; // off the floor : these hang rather than rest

		for (uint wall = 0; wall < 2; wall++)
		{
			uint  mesh   = wall ? sphere_nouv : sphere_uv;
			bool  no_uv  = (wall != 0);
			float wall_x = wall ? 2.2f : -2.2f; // a gap between the two, easy to read

			// the bottom rows : the named materials, five per row
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

			// above them : the sweep, metallic up the rows, roughness across the columns
			const uint named_rows = (num_named + columns - 1) / columns;
			for (uint row = 0; row < 5; row++)
				for (uint col = 0; col < columns; col++)
				{
					float metal = row / 4.f;
					float rough = 0.05f + col * 0.2375f; // 0.05 .. 1.00

					Prop& p = add_prop(mesh, vec3(wall_x + ((float)col - (columns - 1) * 0.5f) * spacing,
					                              base_y + (float)(row + named_rows) * spacing, wall_z),
					                   0, radius);
					p.no_uv = no_uv;
					p.mat_a = vec4(0.72f, 0.70f, 0.68f, metal);
					p.mat_b = vec4(rough, 1.f, 0.f, 0.f); // flat colour : the texture stays out of it
					snprintf(p.label, sizeof(p.label), "m%.2f r%.2f", metal, rough);
				}
		}
	}

	// called once per frame : input -> camera -> matrices, then submit this frame's instances
	void update(GameWindow* window, GameRenderer* renderer)
	{
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

		// view gun : appended just before the row, so the shared gun mesh
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

		// weapons : parked in their row, still animating
		for (uint i = 0; i < NUM_WEAPONS; i++)
		{
			Weapon& w = weapons[i];
			mat4 model = glm::translate(mat4(1), w.pos)
			      * glm::rotate(mat4(1), w.yaw, vec3(0, 1, 0))
			      * glm::scale(mat4(1), vec3(w.scale));
			Instance inst = material(GUN_ALBEDO, 0.85f, 0.30f, 1.f, 1.f); // same steel as the viewmodel
			inst.model = model;
			renderer->buffer(MODE_ANIM_UV).append_instances(w.mesh, 1, &inst);
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
