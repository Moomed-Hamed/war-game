#include "ux.h"

int main()
{
	console = Alloc(GameConsole, 1);

	GameWindow* window = Alloc(GameWindow, 1);
	window->init(1920, 1080);

	audio.init(); // mixer up : the device + context came up in window->init

	// extra shot takes, registered by name : the Audio tab lists whatever
	// ended up in the mixer
	struct { const char* name, * file; } takes[] = {
		{ "shot 4", "pistol_shot_4" }, { "shot 5", "pistol_shot_5" },
		{ "shot 6", "pistol_shot_6" }, { "shot 7", "pistol_shot_7" },
		{ "gewehr 1", "gewehr_shot_1" }, { "gewehr 2", "gewehr_shot_2" },
		{ "gewehr 3", "gewehr_shot_3" }, { "gewehr 4", "gewehr_shot_4" },
		{ "gewehr 5", "gewehr_shot_5" }, { "gewehr 6", "gewehr_shot_6" },
		{ "garand", "garand_shot" }, { "garand ping", "garand_ping" },
	};

	for (uint i = 0; i < 12; i++)
	{
		char path[64] = {};
		snprintf(path, sizeof(path), "assets/audio/%s.audio", takes[i].file);
		audio.load(takes[i].name, path);
	}

	// the renderer draws, the scene holds the content
	GameRenderer* renderer = Alloc(GameRenderer, 1);
	renderer->init();

	Scene* scene = NULL;
	SceneManager* scenes = Alloc(SceneManager, 1);
	scenes->init(renderer);

	window->set_focus(GUI);
	ux_panel_visible = false;
	game_apply_options();

	window->timer.start();
	while (window->instance && !glfwWindowShouldClose(window->instance))
	{
		window->begin_frame(); // poll input, swap GL buffers
		if (window->instance == NULL) break;

		game_handle_keys(window);
		if (game_exit_requested) break;

		if (game_screen != GAME_SCREEN_PAUSED)
			scenes->update(window, renderer); // builds a requested scene before updating it
		scene = &scenes->scene;

		renderer->draw_shadows(window, scene->lighting, scene->camera.position,
		                       scene->camera.front, scene->shadow_extent); // sun depth pass
		renderer->draw(window, scene->proj_view);  // static + skinned pass into the gbuf
		window->draw_lighting(scene->camera.position, scene->proj_view, &scene->lighting); // sun + ambient
		renderer->draw_light_volumes(window, scene->proj_view, scene->camera.position,
		                             scene->lights,
		                             scene->local_lights ? scene->num_lights : 0); // local lights, additive
		window->draw_sky_disks(&scene->lighting, scene->proj_view,
		                       scene->camera.position); // disks over opaque sky only
		renderer->draw_water(window, scene->proj_view, scene->camera.position,
		                     scene->lighting); // forward water covers the disks
		window->post_process(&scene->lighting); // exposure, ACES, display encode

		// one shared wire pass : physics, animation, light volumes and scene guides
		debug_draw.begin();
		scene->phys.append_debug();
		renderer->append_skeleton();
		renderer->append_light_debug(scene->lights, scene->num_lights);
		debug_draw.append_axes(5.f);
		debug_draw.append_frustum(scene->proj_view, vec4(.20f, .85f, 1.f, 1.f));
		debug_draw.draw(scene->proj_view);

		draw_console(console, renderer, window, scenes);
		draw_game_menus(window, scenes);
		ImGui::Render();
		ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

		window->end_frame(); // calculate frame time, sleep
		if (game_exit_requested) break;
	}

	if (window->instance) window->shutdown();

	// the console goes to disk on the way out, so a session can be read back instead of
	// described. the path is relative to the working directory
	console->dump("console_log.txt");

	return 0;
}
