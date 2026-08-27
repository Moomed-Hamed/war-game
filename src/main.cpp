#include "ux.h"

void demo_rotate(mat4 model[4])
{
	static float runningTime = 0;
	runningTime += 1.f / 60000;

	model[0] = glm::rotate(mat4(1), 360.f * runningTime, vec3(0, 1, 0));

	model[1] = glm::translate(mat4(1), vec3(2, 0, 0));
	model[1] = glm::rotate(model[1], 360.f * runningTime, vec3(1, 1, 0));

	model[2] = glm::translate(mat4(1), vec3(4, 0, 0));
	model[2] = glm::rotate(model[2], 360.f * runningTime, vec3(0, 1, 1));
}

int main()
{
	console = Alloc(GameConsole, 1);

	GameWindow* window = Alloc(GameWindow, 1);
	window->init(1920, 1080);

	GameRenderer* renderer = Alloc(GameRenderer, 1);
	renderer->init();
	renderer->add_mesh("assets/meshes/SM/UV/sphere.mesh_uv");
	renderer->add_mesh("assets/meshes/SM/UV/cube.mesh_uv");
	renderer->add_mesh("assets/meshes/SM/UV/ammo.mesh_uv");

	window->timer.start();
	while (window->instance)
	{
		window->begin_frame(); // poll input, swap GL buffers

		// where should this go?
		renderer->drawbuffer.clear_instances();

		// Quit the game?
		if (window->instance == NULL)
		{
			console->add_entry((char*)"window.instance == NULL"); // | Window Shutdown!");
			break; // out of samsara
		}

		// temporary demo for rotating meshes
		mat4 model[4] = { mat4(1), mat4(1) };
		demo_rotate(model);

		renderer->drawbuffer.append_instances(1, 1, &model[0]);
		renderer->drawbuffer.append_instances(2, 1, &model[1]);
		renderer->drawbuffer.append_instances(3, 1, &model[2]);

		// geometry
		renderer->draw(window);

		// gbuffer (direct lighting)
		window->draw_gbuf(renderer->camera.position);
		//ImGui::ShowDemoWindow();

		// draw console
		draw_console(console, renderer, window);
		ImGui::Render();
		ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

		window->end_frame(); // calculate frame time, sleep
	}

	return 0;
}