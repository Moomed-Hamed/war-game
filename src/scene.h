#include "physics.h"

/* ------------------------- *
	-        Scene          -
 * ------------------------- */

// the scene owns the content AND the camera it's viewed from : the renderer
// only receives this frame's matrices and draws whatever the scene submits
struct GameScene
{
	Camera camera;    // the view the world is rendered from
	mat4   proj_view; // built from the camera every frame in update()

	Physics phys; // owns motion, the props read their pose from it

	// world content : a mesh plus an optional body. collider sizes are in
	// world units (mesh scale already applied) ; dynamic props get pos/rot
	// overwritten from bullet every frame
	struct Prop {
		uint mesh_id;
		uint phys_id; // 0 = static decoration
		vec3 pos;
		mat3 rot;
		float scale;
	} props[16];
	uint num_props;

	// one more instance in the world, posed at (pos, yaw, scale)
	Prop& add_prop(uint mesh_id, vec3 pos, float yaw, float scale)
	{
		Prop& p = props[num_props++];
		p.mesh_id = mesh_id;
		p.phys_id = 0;
		p.pos     = pos;
		p.rot     = mat3(glm::rotate(mat4(1), yaw, vec3(0, 1, 0)));
		p.scale   = scale;
		return p;
	}

	// skinned weapons : each on its own animation player so they animate at once
	struct Weapon {
		uint mesh, clip, player;
		vec3 pos;    // spot on the weapon row
		float scale; // world size of the model
		float yaw;   // slight turn so the row reads with some depth
	} weapons[5];

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

		uint sphere_mesh = renderer->add_mesh("assets/meshes/sphere.mesh_uv");
		uint cube_mesh   = renderer->add_mesh("assets/meshes/cube.mesh_uv", "assets/textures/checker.bmp");
		uint ammo_mesh   = renderer->add_mesh("assets/meshes/ammo.mesh_uv");
		uint palm_mesh   = renderer->add_mesh("assets/meshes/palm_tree.mesh_uv");

		// world content : every body starts at its authored spot
		phys.init();

		// floor : ground slab under it, top face exactly at y 0
		add_prop(floor_mesh, vec3(0), 0, 1.f).phys_id = phys.add_ground(30.f);

		// sphere : dynamic, radius 1.5 at this scale
		Prop& sphere = add_prop(sphere_mesh, vec3(-11, 1.5f, 3), 0, 1.5f);
		sphere.phys_id = phys.add_sphere(sphere.pos, 1.5f, 1.f);

		// cube : drops in from above and lands back on its spot
		Prop& cube = add_prop(cube_mesh, vec3(11, 4.f, 3), 0, 1.f);
		cube.phys_id = phys.add_box(cube.pos, vec3(.5f), 1.f);

		// ammo diagonal : three small dynamic boxes
		vec3 ammo_spots[3] = { vec3(7.8f, 0.1f, 1.6f), vec3(8.6f, 0.1f, 2.3f), vec3(9.4f, 0.1f, 1.6f) };
		for (uint i = 0; i < 3; i++)
		{
			Prop& ammo = add_prop(ammo_mesh, ammo_spots[i], 0.7f * i, 2.f);
			ammo.phys_id = phys.add_box(ammo.pos, vec3(.15f, .1f, .15f), .5f);
		}

		// palms : static scenery, a thin trunk slab so nothing shoves them
		vec3 palm_spots[2] = { vec3(-13, 0, -4), vec3(13, 0, -4) };
		for (uint i = 0; i < 2; i++)
		{
			Prop& palm = add_prop(palm_mesh, palm_spots[i], (float)i * 2.1f, 1.6f);
			palm.phys_id = phys.add_box(palm.pos + vec3(0, 3, 0), vec3(.2f, 3.f, .2f), 0);
		}

		// weapons row : same files the old game shipped, sliced the way it played them
		struct { const char* mesh, *clip; uint seg; float dur; } setup[5] = {
			{ "assets/anim/glock.mesh_anim",       "assets/anim/glock.anim",       0, 2.0f },
			{ "assets/anim/ak.mesh_anim",          "assets/anim/ak.anim",          0, 2.6f },
			{ "assets/anim/m4.mesh_anim",          "assets/anim/m4.anim",          1, 2.3f },
			{ "assets/anim/shotgun.mesh_anim",     "assets/anim/shotgun.anim",     1, 3.2f },
			{ "assets/anim/pistol_colt.mesh_anim", "assets/anim/pistol_colt.anim", 1, 1.6f },
		};

		for (uint i = 0; i < 5; i++)
		{
			Weapon& w = weapons[i];
			w.mesh = renderer->add_anim_mesh(setup[i].mesh);
			renderer->animloader.load(setup[i].clip, &w.clip);

			// frame ranges per weapon + a different feel on every segment
			Animation* clip = &renderer->animloader.clips[w.clip];
			switch (i)
			{
				case 0: // glock : the classic frame map
					clip->segs[0] = {   0,  63, 3, 0.40f }; // reload
					clip->segs[1] = {  65,  72, 2, 0.60f }; // shoot
					clip->segs[2] = {  74, 104, 1, 0.30f }; // inspect / equip
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
			}

			w.player = renderer->animloader.play(w.clip, setup[i].seg, setup[i].dur);
			if (w.mesh) renderer->bind_player(w.mesh, w.player);

			// row layout : even spacing, a slight alternating turn for depth
			w.pos = vec3(i * 3.2f - 6.4f, 1.2f, 0);
			w.scale = 4.f;
			w.yaw = (i % 2) ? 0.17f : -0.17f; // +-10 degrees
		}
	}

	// called once per frame : input -> camera -> matrices, then submit this frame's instances
	void update(GameWindow* window, GameRenderer* renderer)
	{
		update_fly_camera(&camera, window, window->dtime);

		float fov = 55.f * PI / 180.f;
		mat4 proj = glm::perspective(fov, (float)window->screen_width / window->screen_height, 0.1f, 256.f);
		proj_view = proj * glm::lookAt(camera.position, camera.position + camera.front, camera.up);

		phys.step(window->dtime);

		// fresh instance streams (each mesh id may only be appended once per frame)
		renderer->drawbuffer.clear_instances();
		renderer->drawbuffer_anim.clear_instances();

		// props : the body owns the pose of anything dynamic
		for (uint i = 0; i < num_props; i++)
		{
			Prop& p = props[i];
			if (p.phys_id) phys.get_transform(p.phys_id, &p.pos, &p.rot);

			mat4 model = glm::translate(mat4(1), p.pos)
			           * mat4(p.rot)
			           * glm::scale(mat4(1), vec3(p.scale));
			renderer->drawbuffer.append_instances(p.mesh_id, 1, &model);
		}

		// weapons : parked in their row, still animating
		for (uint i = 0; i < 5; i++)
		{
			Weapon& w = weapons[i];
			mat4 model = glm::translate(mat4(1), w.pos)
			      * glm::rotate(mat4(1), w.yaw, vec3(0, 1, 0))
			      * glm::scale(mat4(1), vec3(w.scale));
			renderer->drawbuffer_anim.append_instances(w.mesh, 1, &model);
		}
	}
};
