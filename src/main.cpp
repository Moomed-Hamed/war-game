#include "weps.h"

int main()
{
	Window   window = {};
	Mouse    mouse = {};
	Keyboard keys = {};

	//init_window(&window, 1280, 720, "war game");
	init_window(&window, 1920, 1080, "war game");
	init_keyboard(&keys);
	init_imgui(window);

	Wireframe_Renderer* wireframe_renderer = Alloc(Wireframe_Renderer, 1);
	init(wireframe_renderer);

	Heightmap* heightmap = Alloc(Heightmap, 1);
	init(heightmap, "assets/textures/heightmap.r32");

	Heightmap_Renderer* heightmap_renderer = Alloc(Heightmap_Renderer, 1);
	init(heightmap_renderer, heightmap);

	Physics* phys = Alloc(Physics, 1);
	init(phys);

	Physics_Renderer* physics_renderer = Alloc(Physics_Renderer, 1);
	init(physics_renderer);

	// add phys stuff
	add_phys_terrain(phys, heightmap);
	add_phys_vehicle(phys); phys->num_vehicles = 1;
	add_phys_ragdoll(phys, 2, Vec3(5, 5, 5));

	Player* player = Alloc(Player, 1); init(player, phys);

	Building* building = Alloc(Building, 1);
	init(building, phys);

	Building_Renderer* building_renderer = Alloc(Building_Renderer, 1);
	init(building_renderer);

	Props* props = Alloc(Props, 1); init(props, phys, heightmap);
	Prop_Renderer* prop_renderer = Alloc(Prop_Renderer, 1);
	init(prop_renderer);

	Particle_Emitter* emitter = Alloc(Particle_Emitter, 1);
	Particle_Renderer* particle_renderer = Alloc(Particle_Renderer, 1);
	init(particle_renderer);

	Bullet* bullets = Alloc(Bullet, MAX_BULLETS);
	Bullet_Renderer* bullet_renderer = Alloc(Bullet_Renderer, 1);
	init(bullet_renderer);

	Gun_Meta* gun_meta = Alloc(Gun_Meta, 1);
	init(gun_meta);
	Gun gun = { GUN_GLOCK, 0, gun_meta->info[GUN_GLOCK].equip_time, gun_meta->info[GUN_GLOCK].equip_time };

	Gun_Renderer* gun_renderer = Alloc(Gun_Renderer, 1);
	init(gun_renderer);

	Enemy* enemies = Alloc(Enemy, MAX_ENEMIES);
	init(enemies, phys);

	Enemy_Renderer* enemy_renderer = Alloc(Enemy_Renderer, 1);
	init(enemy_renderer);

	Audio headshot = load_audio("assets/audio/headshot.audio");

	C_Renderer* crosshair_renderer = Alloc(C_Renderer, 1);
	init(crosshair_renderer);

	Light_Renderer* light_renderer = Alloc(Light_Renderer, 1);
	init(light_renderer);

	G_Buffer g_buffer = make_g_buffer(window);
	Shader lighting_shader = make_lighting_shader();
	mat4 proj = glm::perspective(FOV, (float)window.screen_width / window.screen_height, 0.1f, DRAW_DISTANCE);

	// frame timer
	float frame_time = 1.f / 60;
	int64 target_frame_milliseconds = frame_time * 1000.f; // seconds * 1000 = milliseconds
	Timestamp frame_start = get_timestamp(), frame_end;

	while (1)
	{
		update_window(window);
		update_mouse(&mouse, window);
		update_keyboard(&keys, window);

		if (keys.ESC.is_pressed) break;

		uint new_event = 0;
		if (keys.DOWN.is_pressed && !keys.DOWN.was_pressed) switch_gun(&gun, gun_meta, --gun.type);
		if (keys.UP.is_pressed   && !keys.UP.was_pressed  ) switch_gun(&gun, gun_meta, ++gun.type);

		// game updates
		update(phys   , frame_time, keys);
		update(props  , phys);
		update(player , frame_time, heightmap, keys, mouse);
		update(emitter, frame_time, heightmap, vec3(0));
		update(bullets, frame_time, phys, emitter, enemies, &new_event);
		update(enemies, frame_time, emitter, &player->eyes, phys);
		update(&gun   , frame_time, gun_meta, bullets, &player->eyes, mouse, keys);

		// renderer updates
		update(physics_renderer  , phys);
		update(crosshair_renderer, window, frame_time, new_event);
		update(particle_renderer , emitter);
		update(building_renderer , building, phys);
		update(prop_renderer     , props);
		update(enemy_renderer    , enemies, phys);
		update(gun_renderer      , gun, frame_time, player->eyes, mouse.norm_dx);
		update(heightmap_renderer, heightmap);
		update(wireframe_renderer);
		update(bullet_renderer   , bullets);
		update(light_renderer    , lighting_shader);
		//gui_test();

		// geometry pass
		glBindFramebuffer(GL_FRAMEBUFFER, g_buffer.FBO);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		mat4 proj_view = proj * lookAt(player->eyes.position, player->eyes.position + player->eyes.front, player->eyes.up);
		draw(building_renderer , proj_view);
		draw(prop_renderer     , proj_view);
		draw(particle_renderer , proj_view);
		draw(enemy_renderer    , proj_view);
		draw(gun_renderer      , proj_view);
		draw(heightmap_renderer, proj_view);
		draw(physics_renderer  , proj_view);

		// debug wireframes
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
		//draw(wireframe_renderer, proj_view);
		//draw(bullet_renderer   , proj_view);
		//draw(light_renderer    , proj_view);
		draw_gui();

		// lighting pass
		glBindFramebuffer(GL_FRAMEBUFFER, 0);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		bind(lighting_shader);
		set_vec3(lighting_shader, "view_pos", player->eyes.position);
		draw(g_buffer);

		// user interface
		glDisable(GL_DEPTH_TEST);
		draw(crosshair_renderer);
		glEnable(GL_DEPTH_TEST);

		//frame time
		frame_end = get_timestamp();
		int64 milliseconds_elapsed = calculate_milliseconds_elapsed(frame_start, frame_end);

		//print("frame time: %02d ms | fps: %06f\n", milliseconds_elapsed, 1000.f / milliseconds_elapsed);
		if (target_frame_milliseconds > milliseconds_elapsed) // frame finished early
		{
			os_sleep(target_frame_milliseconds - milliseconds_elapsed);
		}
		
		frame_start = frame_end;
	}

	return 0;
}

//#define MAX_ANIMATIONS 1
//
//struct Animation_Skeleton
//{
//	uint num_bones;
//
//	mat4  ibm[MAX_ANIM_BONES]; // inverse-bind matrices
//	int   parents[MAX_ANIM_BONES]; // indices of parent bones
//
//	Animation_Data animations[MAX_ANIMATIONS];
//};
//
//struct Animation_Data
//{
//	float time;
//	uint num_keyframes;
//	mat4* keyframes[MAX_ANIM_BONES]; // LOCAL-SPACE poses
//};
//
//struct Animated_Mesh
//{
//
//};
// 
// Tatenen
//#define NUM_LAYER_NODES 16
//
//struct Hidden_Layer
//{
//	float activations[NUM_LAYER_NODES];
//};