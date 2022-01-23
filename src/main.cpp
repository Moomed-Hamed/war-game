#include "enemies.h"

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
	Gun gun = {GUN_US_RIFLE};
	Gun_Renderer* gun_renderer = Alloc(Gun_Renderer, 1);
	init(gun_renderer);

	Enemy* enemies = Alloc(Enemy, MAX_ENEMIES); init(enemies);
	Enemy_Renderer* enemy_renderer = Alloc(Enemy_Renderer, 1);
	init(enemy_renderer);

	Physics_Colliders* colliders = Alloc(Physics_Colliders, 1);
	colliders->dynamic.cubes     [0] = { {1, 2, 3}, {}, {}, 1, vec3(1) };
	colliders->dynamic.cylinders [0] = { {5, 2, 3}, {}, {}, 1, 1, .5 };
	colliders->dynamic.spheres   [0] = { {3, 2, 3}, {}, {}, 1, .5 };

	Physics_Renderer* physics_renderer = Alloc(Physics_Renderer, 1);
	init(physics_renderer);

	Crosshair_Renderer* crosshair_renderer = Alloc(Crosshair_Renderer, 1);
	init(crosshair_renderer);

	G_Buffer g_buffer = make_g_buffer(window);
	Shader lighting_shader = make_lighting_shader();
	mat4 proj = glm::perspective(FOV, (float)window.screen_width / window.screen_height, 0.1f, DRAW_DISTANCE);

	// frame timer
	float frame_time = 1.f / 45;
	int64 target_frame_milliseconds = frame_time * 1000.f; // seconds * 1000 = milliseconds
	Timestamp frame_start = get_timestamp(), frame_end;

	Audio orb = load_audio("assets/audio/orb.audio");
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

		if (keys.M.is_pressed) explode(heightmap, player->feet.position, heightmap_renderer->heights);

		//if (mouse.left_button.is_pressed && !mouse.left_button.was_pressed) play_audio(orb);

		static float a = 0; a += frame_time;
		if (a > .4) { a = 0; emit_fire(emitter, terrain(heightmap, vec2(9, 6))); }

		// game updates
		update(player , frame_time, heightmap, keys, mouse);
		update(emitter, heightmap, frame_time, vec3(0));
		update(bullets, frame_time);
		update(&gun, gun_meta, bullets, &player->eyes, mouse, keys, frame_time);
		update(enemies, frame_time, bullets, emitter, &player->eyes);

		// renderer updates
		update(crosshair_renderer);
		update(physics_renderer  , colliders);
		update(particle_renderer , emitter);
		update(prop_renderer     , props);
		update(bullet_renderer   , bullets);
		update(gun_renderer      , gun, frame_time, player->eyes, 0);
		update(enemy_renderer    , enemies);

		// geometry pass
		glBindFramebuffer(GL_FRAMEBUFFER, g_buffer.FBO);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		mat4 proj_view = proj * lookAt(player->eyes.position, player->eyes.position + player->eyes.front, player->eyes.up);
		
		draw(crosshair_renderer);
		draw(prop_renderer     , proj_view);
		draw(physics_renderer  , proj_view);
		draw(particle_renderer , proj_view);
		draw(heightmap_renderer, proj_view);
		draw(bullet_renderer   , proj_view);
		draw(gun_renderer      , proj_view);
		draw(enemy_renderer    , proj_view);

		// lighting pass
		glBindFramebuffer(GL_FRAMEBUFFER, 0);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

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

	shutdown_window();
	return 0;
}