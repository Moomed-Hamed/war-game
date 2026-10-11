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

	GameScene* scene = Alloc(GameScene, 1);
	scene->init(renderer);

	window->timer.start();
	while (window->instance)
	{
		window->begin_frame(); // poll input, swap GL buffers
		if (window->instance == NULL) break; // out of samsara

		scene->update(window, renderer);           // input -> camera -> instances
		renderer->draw(window, scene->proj_view);  // static + skinned pass into the gbuf
		window->draw_gbuf(scene->camera.position, scene->proj_view); // lighting
		scene->phys.draw_debug(scene->proj_view);  // collider wireframes, toggled in the Phys tab
		renderer->draw_skeleton(scene->proj_view); // rig overlay, toggled in the Anim tab

		draw_console(console, renderer, window, scene);
		ImGui::Render();
		ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

		window->end_frame(); // calculate frame time, sleep
	}

	// the console goes to disk on the way out, so a session can be read back instead of
	// described. the path is relative to the working directory
	console->dump("console_log.txt");

	return 0;
}
