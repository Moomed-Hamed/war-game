#include "weps.h"

int main()
{
	Window   window = {};
	Mouse    mouse  = {};
	Keyboard keys   = {};

	init_window(&window, 1920, 1080, "war game");
	init_keyboard(&keys);

	Heightmap* heightmap = Alloc(Heightmap, 1);
	Heightmap_Renderer* heightmap_renderer = Alloc(Heightmap_Renderer, 1);
	init(heightmap_renderer, heightmap, "assets/textures/heightmap.r32");

	Physics* phys = Alloc(Physics, 1);
	init(phys);

	Physics_Renderer* physics_renderer = Alloc(Physics_Renderer, 1);
	init(physics_renderer);

	add_phys_terrain (phys, heightmap);
	add_phys_cube    (phys, { 10 , height(heightmap, vec2(10, 7)) + 5.f , 7 }, { 1, 1, 1 });
	add_phys_cone    (phys, { 16 , height(heightmap, vec2(16, 3)) + 5.f , 3 }, .5, 1);
	add_phys_sphere  (phys, { 7  , height(heightmap, vec2(7 , 6)) + 5.f , 6 }, .5);
	add_phys_cylinder(phys, { 7  , height(heightmap, vec2(7 , 6)) + 5.f , 6 }, .5, 1);
	add_phys_capsule (phys, { 10 , height(heightmap, vec2(10, 7)) + 5.f , 7 }, .5, 1);

	add_phys_vehicle(phys); phys->num_vehicles = 1;
	add_phys_ragdoll(phys, 2, Vec3(5,5,5));

	print_phys(phys->world);

	Player* player = Alloc(Player, 1); init(player);

	Props* props = Alloc(Props, 1); init(props, heightmap);
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
	Gun gun = { GUN_US_RIFLE};
	Gun_Renderer* gun_renderer = Alloc(Gun_Renderer, 1);
	init(gun_renderer);

	Crosshair_Renderer* crosshair_renderer = Alloc(Crosshair_Renderer, 1);
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

	Audio headshot = load_audio("assets/audio/headshot.audio");

	while (1)
	{
		update_window(window);
		update_mouse(&mouse, window);
		update_keyboard(&keys, window);

		if (keys.ESC.is_pressed) break;

		if (keys.G.is_pressed && !keys.G.was_pressed) play_audio(headshot);
		if (keys.G.is_pressed) player->eyes.trauma = 1;
		if (keys.G.is_pressed) emit_explosion(emitter, player->eyes.position + 14.f * player->eyes.front);

		if (keys.H.is_pressed)
		{
			phys->capsules[0].body->applyCentralImpulse(Vec3(5,5,5) * frame_time);
		}

		if (keys.M.is_pressed)
		{
			explode(heightmap, player->eyes.position, 5);
			update_phys_terrain(phys, heightmap);
		}

		if (keys.N.is_pressed)
		{
			extrude(heightmap, player->eyes.position, 5);
			update_phys_terrain(phys, heightmap);
		}

		if (keys.J.is_pressed) gun.type = GUN_US_PISTOL;
		if (keys.K.is_pressed) gun.type = GUN_RU_RIFLE;
		if (keys.L.is_pressed) gun.type = GUN_US_RIFLE;

		//if (keys.LEFT.is_pressed  && !keys.LEFT.was_pressed ) gun.type--;
		//if (keys.RIGHT.is_pressed && !keys.RIGHT.was_pressed) gun.type++;

		if (keys.O.is_pressed)
			set_vec3(lighting_shader, "light_positions[0]", player->eyes.position);

		static float a = 0; a += frame_time;
		if (a > .4) { a = 0; emit_fire(emitter, terrain(heightmap, vec2(9, 6))); }

		if (keys.L.is_pressed && !keys.L.was_pressed)
			phys->world->removeConstraint(phys->wheel_hinges[0]);

		if (keys.U.is_pressed)
			phys->ragdoll.bodies[2]->applyCentralImpulse({ 0, 2, 0 });

		// game updates
		update(phys, frame_time, keys); // physics update
		update(player , frame_time, heightmap, keys, mouse);
		update(emitter, heightmap, frame_time, vec3(0));
		update(bullets, frame_time);
		update(&gun, gun_meta, bullets, &player->eyes, mouse, keys, frame_time);

		// renderer updates
		update(crosshair_renderer);
		update(particle_renderer , emitter);
		update(prop_renderer     , props);
		update(bullet_renderer   , bullets);
		update(gun_renderer      , gun, frame_time, player->eyes, mouse.norm_dx * 2);
		update(heightmap_renderer, heightmap);
		update(physics_renderer  , phys);
		update(light_renderer, lighting_shader); out(phys->num_cylinders);

		// geometry pass
		glBindFramebuffer(GL_FRAMEBUFFER, g_buffer.FBO);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		mat4 proj_view = proj * lookAt(player->eyes.position, player->eyes.position + player->eyes.front, player->eyes.up);
		draw(crosshair_renderer);
		draw(prop_renderer     , proj_view);
		draw(particle_renderer , proj_view);
		draw(bullet_renderer   , proj_view);
		draw(gun_renderer      , proj_view);
		draw(heightmap_renderer, proj_view);
		draw(physics_renderer  , proj_view);

		// debug wireframes
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
		//draw(light_renderer , proj_view);

		// lighting pass
		glBindFramebuffer(GL_FRAMEBUFFER, 0);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		bind(lighting_shader);
		set_vec3(lighting_shader, "view_pos", player->eyes.position);
		draw(g_buffer);

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