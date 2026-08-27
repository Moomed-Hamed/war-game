#include "loader.h"

/* ------------------------- *
	-   Window & Input      -
 * ------------------------- */

struct Button
{
	char is_pressed;
	char was_pressed;
	u16  id;
};

struct Mouse
{
	double raw_x, raw_y;   // pixel coordinates
	double norm_x, norm_y; // normalized screen coordinates
	double dx, dy;  // pos change since last frame in pixels
	double norm_dx, norm_dy;

	Button right_button, left_button;
};

void update_mouse(Mouse* mouse, GLFWwindow* instance, uint screen_x, uint screen_y)
{
	static Mouse prev_mouse = {};

	glfwGetCursorPos(instance, &mouse->raw_x, &mouse->raw_y);

	mouse->dx = mouse->raw_x - prev_mouse.raw_x; // what do these mean again?
	mouse->dy = prev_mouse.raw_y - mouse->raw_y;

	mouse->norm_dx = mouse->dx / screen_x;
	mouse->norm_dy = mouse->dy / screen_y;

	prev_mouse.raw_x = mouse->raw_x;
	prev_mouse.raw_y = mouse->raw_y;

	mouse->norm_x = ((uint)mouse->raw_x % screen_x) / (double)screen_x;
	mouse->norm_y = ((uint)mouse->raw_y % screen_y) / (double)screen_y;

	mouse->norm_y = 1 - mouse->norm_y;
	mouse->norm_x = (mouse->norm_x * 2) - 1;
	mouse->norm_y = (mouse->norm_y * 2) - 1;

	bool key_down = (glfwGetMouseButton(instance, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);

	if (key_down)
	{
		mouse->right_button.was_pressed = (mouse->right_button.is_pressed == true);
		mouse->right_button.is_pressed = true;
	}
	else
	{
		mouse->right_button.was_pressed = (mouse->right_button.is_pressed == true);
		mouse->right_button.is_pressed = false;
	}

	key_down = (glfwGetMouseButton(instance, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);

	if (key_down)
	{
		mouse->left_button.was_pressed = (mouse->left_button.is_pressed == true);
		mouse->left_button.is_pressed = true;
	}
	else
	{
		mouse->left_button.was_pressed = (mouse->left_button.is_pressed == true);
		mouse->left_button.is_pressed = false;
	}
}

// for selecting game objecs
vec3 get_mouse_world_dir(Mouse mouse, mat4 proj_view)
{
	proj_view = glm::inverse(proj_view); // what is an unproject matrix?

	vec4 ray_near = vec4(mouse.norm_x, mouse.norm_y, -1, 1); // near plane is z = -1
	vec4 ray_far  = vec4(mouse.norm_x, mouse.norm_y,  0, 1);

	// these are actually using inverse(proj_view)
	ray_near = proj_view * ray_near; ray_near /= ray_near.w;
	ray_far  = proj_view * ray_far;  ray_far /= ray_far.w;

	return glm::normalize(ray_far - ray_near);
}

#define NUM_KEYBOARD_BUTTONS 34 // update when adding keyboard buttons
struct Keyboard
{
	union
	{
		Button buttons[NUM_KEYBOARD_BUTTONS];

		struct
		{
			Button A, B, C, D, E, F, G, H;
			Button I, J, K, L, M, N, O, P;
			Button Q, R, S, T, U, V, W, X;
			Button Y, Z;
			
			Button ESC, SPACE;
			Button SHIFT, CTRL;
			Button UP, DOWN, LEFT, RIGHT;
		};
	};
};

void init_keyboard(Keyboard* keyboard)
{
	keyboard->A = { false, false, GLFW_KEY_A };
	keyboard->B = { false, false, GLFW_KEY_B };
	keyboard->C = { false, false, GLFW_KEY_C };
	keyboard->D = { false, false, GLFW_KEY_D };
	keyboard->E = { false, false, GLFW_KEY_E };
	keyboard->F = { false, false, GLFW_KEY_F };
	keyboard->G = { false, false, GLFW_KEY_G };
	keyboard->H = { false, false, GLFW_KEY_H };
	keyboard->I = { false, false, GLFW_KEY_I };
	keyboard->J = { false, false, GLFW_KEY_J };
	keyboard->K = { false, false, GLFW_KEY_K };
	keyboard->L = { false, false, GLFW_KEY_L };
	keyboard->M = { false, false, GLFW_KEY_M };
	keyboard->N = { false, false, GLFW_KEY_N };
	keyboard->O = { false, false, GLFW_KEY_O };
	keyboard->P = { false, false, GLFW_KEY_P };
	keyboard->Q = { false, false, GLFW_KEY_Q };
	keyboard->R = { false, false, GLFW_KEY_R };
	keyboard->S = { false, false, GLFW_KEY_S };
	keyboard->T = { false, false, GLFW_KEY_T };
	keyboard->U = { false, false, GLFW_KEY_U };
	keyboard->V = { false, false, GLFW_KEY_V };
	keyboard->W = { false, false, GLFW_KEY_W };
	keyboard->X = { false, false, GLFW_KEY_X };
	keyboard->Y = { false, false, GLFW_KEY_Y };
	keyboard->Z = { false, false, GLFW_KEY_Z };

	keyboard->ESC   = { false, false, GLFW_KEY_ESCAPE       };
	keyboard->SPACE = { false, false, GLFW_KEY_SPACE        };
	keyboard->SHIFT = { false, false, GLFW_KEY_LEFT_SHIFT   };
	keyboard->CTRL  = { false, false, GLFW_KEY_LEFT_CONTROL };

	keyboard->UP    = { false, false, GLFW_KEY_UP    };
	keyboard->DOWN  = { false, false, GLFW_KEY_DOWN  };
	keyboard->LEFT  = { false, false, GLFW_KEY_LEFT  };
	keyboard->RIGHT = { false, false, GLFW_KEY_RIGHT };
}
void update_keyboard(Keyboard* keyboard, GLFWwindow* instance)
{
	for (uint i = 0; i < NUM_KEYBOARD_BUTTONS; ++i)
	{
		bool key_down = (glfwGetKey(instance, keyboard->buttons[i].id) == GLFW_PRESS);

		if (key_down)
		{
			keyboard->buttons[i].was_pressed = (keyboard->buttons[i].is_pressed == true);
			keyboard->buttons[i].is_pressed = true;
		}
		else
		{
			keyboard->buttons[i].was_pressed = (keyboard->buttons[i].is_pressed == true);
			keyboard->buttons[i].is_pressed = false;
		}
	}
}

// RenderTarget allows us to *write* to textures
// as opposed to just sampling them!
struct RenderTarget
{
	GLuint fbo;

	GLuint position_tx; // tx = texture
	GLuint normal_tx;
	GLuint albedo_tx;
	GLuint depth_tx; // could also use a render buffer (write only)

	void init(uint height, uint width)
	{
		glGenFramebuffers(1, &fbo);
		glBindFramebuffer(GL_FRAMEBUFFER, fbo);

		// position color buffer
		glGenTextures(1, &position_tx);
		glBindTexture(GL_TEXTURE_2D, position_tx);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, width, height, 0, GL_RGBA, GL_FLOAT, NULL);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

		// normal color buffer
		glGenTextures(1, &normal_tx);
		glBindTexture(GL_TEXTURE_2D, normal_tx);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA16F, width, height, 0, GL_RGBA, GL_FLOAT, NULL);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

		// albedo color buffer
		glGenTextures(1, &albedo_tx);
		glBindTexture(GL_TEXTURE_2D, albedo_tx);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

		// without glDrawBuffers(), only GL_COLOR_ATTACHMENT0 receives output
		uint attachments[3] = { GL_COLOR_ATTACHMENT0, GL_COLOR_ATTACHMENT1, GL_COLOR_ATTACHMENT2 };
		glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, position_tx, 0);
		glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT1, GL_TEXTURE_2D, normal_tx  , 0);
		glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT2, GL_TEXTURE_2D, albedo_tx  , 0);
		glDrawBuffers(3, attachments);

		// depth texture
		glGenTextures(1, &depth_tx);
		glBindTexture(GL_TEXTURE_2D, depth_tx);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT24, width, height, 0, GL_DEPTH_COMPONENT, GL_FLOAT, NULL);
		glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, depth_tx, 0);

		if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
		{
					out("FRAMEBUFFER ERROR : INCOMPLETE"); stop;
		}

		console->add_entry((char*)"Init RenderBuffer", SUCCESS, RNDR);
	}
	void bind()
	{
		glBindFramebuffer(GL_FRAMEBUFFER, fbo);
	}
};

enum WindowFocus
{
	GUI = 0,
	GAME
};

struct GameWindow
{
	uint focus_status;

	Mouse mouse;
	Keyboard keys;

	GLFWwindow* instance;
	uint screen_width, screen_height;

	// Frame Timing
	Timer timer;
	uint frame_milliseconds_target;
	float dtime;

	// deferred rendering
	struct {
		RenderTarget rendertarget;
		GLuint VAO, VBO, EBO; // for drawing the quad
		ShaderProgram shader;
	} gbuf; // G-Buffer

	void init(uint, uint);
	uint begin_frame();
	void end_frame();

	void draw_gbuf(vec3 view_position);

	void shutdown();
};

void GameWindow::init(uint screen_width, uint screen_height)
{
	timer.init();
	init_keyboard(&keys);

	this->screen_width  = screen_width;
	this->screen_height = screen_height;

	if (!glfwInit()) {
		console->add_entry((char*)"Init GLFW", SEVERITY::FIXME, LOGSOURCE::WNDW);
		stop; return;
	} console->add_entry((char*)"Init GLFW", SEVERITY::SUCCESS, LOGSOURCE::WNDW);

	// OpenGL Window Hints (This is not renderer code!)
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
	glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);

	this->instance = glfwCreateWindow(screen_width, screen_height, "GameWindow", NULL, NULL);

	if (!this->instance) {
		glfwTerminate();
		console->add_entry((char*)"no window instance", SEVERITY::FIXME);
		stop; return;
	}

	glfwMakeContextCurrent(this->instance);
	glfwSwapInterval(1); // Disable v-sync

	// There needs to already be a glfw context *before* you call glew
	glewExperimental = GL_TRUE;
	glewInit(); // TODO : CHECK FOR ERRORS
	console->add_entry((char*)"Init GLEW", SEVERITY::SUCCESS, LOGSOURCE::WNDW);

	glClearColor(.1, .2, .3, 1);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_CULL_FACE);
	//glEnable(GL_FRAMEBUFFER_SRGB); // gamma correction
	//glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

	// Capture cursor
	glfwSetInputMode(this->instance, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

	// Audio Code | Todo : This should be moved to it's own place

	ALCdevice* audio_device = alcOpenDevice(NULL);
	if (audio_device == NULL)
		console->add_entry((char*)"cannot open sound card", FIXME);

	ALCcontext* audio_context = alcCreateContext(audio_device, NULL);
	if (audio_context == NULL) { out("cannot open context"); }
	alcMakeContextCurrent(audio_context);

	console->add_entry((char*)"Init OpenAL", SUCCESS, WNDW);
	console->add_entry((char*)"Init Window", SUCCESS, WNDW);

	// Setup ImGUI Context
	IMGUI_CHECKVERSION();
	ImGui::CreateContext();
	ImGuiIO& io = ImGui::GetIO(); (void)io;
	//io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard; // Enable Keyboard Controls
	//io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;  // Enable Gamepad Controls

	// Colors & Style
	apply_imgui_style(io);

	// Setup Platform/Renderer backends
	ImGui_ImplGlfw_InitForOpenGL(instance, true);
	ImGui_ImplOpenGL3_Init("#version 130");

	// G-Buffer
	gbuf.shader.create("assets/shaders/gbuf.vert", "assets/shaders/gbuf.frag");
	gbuf.shader.bind();

	gbuf.rendertarget.init(screen_height, screen_width);

	// Screen Quad Setup ---------------------------------------------------------

	Mesh_Data screenquad = MeshFactory::generate_screenquad();

	uint pos_data_size   = screenquad.num_vertices * sizeof(vec3);
	uint uv_data_size    = screenquad.num_vertices * sizeof(vec2);
	uint index_data_size = screenquad.num_indices  * sizeof(uint);

	glGenVertexArrays(1, &gbuf.VAO);
	glBindVertexArray(gbuf.VAO);

	glGenBuffers(1, &gbuf.VBO);
	glBindBuffer(GL_ARRAY_BUFFER, gbuf.VBO);
	glBufferData(GL_ARRAY_BUFFER, pos_data_size + uv_data_size, NULL, GL_STATIC_DRAW);
	glBufferSubData(GL_ARRAY_BUFFER, 0, pos_data_size, screenquad.positions);
	glBufferSubData(GL_ARRAY_BUFFER, pos_data_size, uv_data_size, screenquad.uvs);

	glGenBuffers(1, &gbuf.EBO);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, gbuf.EBO);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, index_data_size, screenquad.indices, GL_STATIC_DRAW);

	screenquad.release(); // no longer needed since data is now in gpu buffers

	{
		uint offset = 0;
		GLint pos_attrib = 0; // vertex position
		glVertexAttribPointer(pos_attrib, 3, GL_FLOAT, GL_FALSE, 0, (void*)offset);
		glEnableVertexAttribArray(pos_attrib);

		offset += pos_data_size;
		GLint tex_attrib = 1; // vertex uv coordinates
		glVertexAttribPointer(tex_attrib, 2, GL_FLOAT, GL_FALSE, 0, (void*)offset);
		glEnableVertexAttribArray(tex_attrib);
	}

	console->add_entry((char*)"Init ImGUI", SEVERITY::SUCCESS);
}

// Updates frame timestamp
// Polls for glfw events
// Swaps OpenGL buffers
// Sets window focus
// Updates ImGui stuff
uint GameWindow::begin_frame()
{
	update_keyboard(&keys, instance);
	update_mouse(&mouse, instance, screen_width, screen_height);
	
	//console->add_entry((char*)"Beginning new frame...");
	//console->add_entry((char*)"Poll glfw events & swap buffers...");

	glfwPollEvents();
	glfwSwapBuffers(instance);

	//console->add_entry((char*)"Handle key input...");

	if (keys.ESC.is_pressed)
	{
		this->shutdown();
		return 1;
	}

	if (keys.T.is_pressed)
		glfwSetInputMode(instance, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
	if (keys.Y.is_pressed)
		glfwSetInputMode(instance, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

	//console->add_entry((char*)"Update ImGui");

	ImGui_ImplOpenGL3_NewFrame();
	ImGui_ImplGlfw_NewFrame();
	ImGui::NewFrame();
}
void GameWindow::end_frame()
{
	//console->add_entry((char*)"Calculate frame time...");
	
	//calculate frame time
	int64 microseconds_elapsed = timer.microseconds_elapsed();
	int64 milliseconds_elapsed = microseconds_elapsed / 1000;

	//print("frame time: %d microseconds | fps: %06f\n", microseconds_elapsed, 1000.f / milliseconds_elapsed);
	
	frame_milliseconds_target = 1000.f / 120;

	string32 msg = {};
	snprintf(msg, 32, "Frame : [%dms][%dfps]", milliseconds_elapsed, (int)(1000.f / milliseconds_elapsed));
	//out(msg);
	//console->add_entry(msg);

	// if frame finished early, wait
	if (milliseconds_elapsed < frame_milliseconds_target)
	{
		out("Frame done; spare ms: [" << frame_milliseconds_target - milliseconds_elapsed << ']');
		os_sleep(frame_milliseconds_target - milliseconds_elapsed);
	}

	timer.start(); // begin timing next frame
}
void GameWindow::draw_gbuf(vec3 view_position)
{
	// G Buffer
	glBindFramebuffer(GL_FRAMEBUFFER, 0); // Framebuffer 0 = the screen
	glBindVertexArray(gbuf.VAO);
	glClearColor(0.1f, 0.2f, 0.3f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	gbuf.shader.bind();
	uint vp = glGetUniformLocation(gbuf.shader.id, "view_pos");
	glUniform3f(vp, view_position.x, view_position.y, view_position.z);

	glActiveTexture(GL_TEXTURE0); glBindTexture(GL_TEXTURE_2D, gbuf.rendertarget.position_tx);
	glActiveTexture(GL_TEXTURE1); glBindTexture(GL_TEXTURE_2D, gbuf.rendertarget.normal_tx  );
	glActiveTexture(GL_TEXTURE2); glBindTexture(GL_TEXTURE_2D, gbuf.rendertarget.albedo_tx  );

	glDrawElementsInstanced(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0, 1); // Draw ScreenQuad
}

// Terminates GLFW
// Sets instance to NULL (important)
void GameWindow::shutdown()
{
	glfwTerminate();
	this->instance = NULL;
}

// audio

/* -- how 2 play a sound --

	Audio sound = load_audio("sound.audio");
	play_sudio(sound);
*/

Audio load_audio(const char* path)
{
	uint format, size, sample_rate;
	byte* audio_data = NULL;

	FILE* file = fopen(path, "rb"); // rb = read binary
	if (file == NULL) { print("ERROR : %s not found\n"); stop;  return 0; }

	fread(&format     , sizeof(uint), 1, file);
	fread(&sample_rate, sizeof(uint), 1, file);
	fread(&size       , sizeof(uint), 1, file);

	audio_data = Alloc(byte, size);
	fread(audio_data, sizeof(byte), size, file);

	fclose(file);

	ALuint buffer_id = NULL;
	alGenBuffers(1, &buffer_id);
	alBufferData(buffer_id, format, audio_data, size, sample_rate);

	ALuint source_id = NULL;
	alGenSources(1, &source_id);
	alSourcei(source_id, AL_BUFFER, buffer_id);

	free(audio_data);
	return source_id;
}
void play_audio(Audio source_id)
{
	//alSourcePlay(source_id);
}