#include "ux.h"

int main()
{
	console = Alloc(GameConsole, 1);

	GameWindow* window = Alloc(GameWindow, 1);
	window->init(1920, 1080);

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
		window->draw_gbuf(scene->camera.position); // lighting
		scene->phys.draw_debug(scene->proj_view);  // collider wireframes, toggled in the Phys tab

		draw_console(console, renderer, window, scene);
		ImGui::Render();
		ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

		window->end_frame(); // calculate frame time, sleep
	}

	return 0;
}
