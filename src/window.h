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
	static bool seeded = false; // first frame has no previous position to diff against

	glfwGetCursorPos(instance, &mouse->raw_x, &mouse->raw_y);

	if (!seeded)
	{
		prev_mouse.raw_x = mouse->raw_x;
		prev_mouse.raw_y = mouse->raw_y;
		seeded = true; // no phantom swing from the launch cursor position
	}

	mouse->dx = mouse->raw_x - prev_mouse.raw_x;
	mouse->dy = prev_mouse.raw_y - mouse->raw_y; // flipped : screen y grows downward, world y grows up

	mouse->norm_dx = mouse->dx / screen_x;
	mouse->norm_dy = mouse->dy / screen_y;

	prev_mouse.raw_x = mouse->raw_x;
	prev_mouse.raw_y = mouse->raw_y;

	mouse->norm_x = ((uint)mouse->raw_x % screen_x) / (double)screen_x;
	mouse->norm_y = ((uint)mouse->raw_y % screen_y) / (double)screen_y;

	mouse->norm_y = 1 - mouse->norm_y;
	mouse->norm_x = (mouse->norm_x * 2) - 1;
	mouse->norm_y = (mouse->norm_y * 2) - 1;

	// shift each button's edge state, then store the new one
	mouse->right_button.was_pressed = mouse->right_button.is_pressed;
	mouse->right_button.is_pressed  = glfwGetMouseButton(instance, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;

	mouse->left_button.was_pressed = mouse->left_button.is_pressed;
	mouse->left_button.is_pressed  = glfwGetMouseButton(instance, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
}

// for selecting game objects
vec3 get_mouse_world_dir(Mouse mouse, mat4 proj_view)
{
	proj_view = glm::inverse(proj_view); // unproject : maps screen space back to world space

	vec4 ray_near = vec4(mouse.norm_x, mouse.norm_y, -1, 1); // near plane is z = -1
	vec4 ray_far  = vec4(mouse.norm_x, mouse.norm_y,  0, 1);

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
	// A..Z sit contiguous in glfw : GLFW_KEY_A + i
	for (uint i = 0; i < 26; i++)
		keyboard->buttons[i] = { false, false, (u16)(GLFW_KEY_A + i) };

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
		Button& b = keyboard->buttons[i];
		b.was_pressed = b.is_pressed;
		b.is_pressed  = glfwGetKey(instance, b.id) == GLFW_PRESS;
	}
}

// RenderTarget allows us to *write* to textures
// as opposed to just sampling them!
struct RenderTarget
{
	GLuint fbo;
	bool complete; // framebuffer passed the completeness check

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
			complete = false;
			console->add_entry("FRAMEBUFFER ERROR : INCOMPLETE", FIXME, WNDW);
		}
		else
		{
			complete = true;
			console->add_entry((char*)"Init RenderBuffer", SUCCESS, RNDR);
		}
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
	float frame_milliseconds_target; // ms per frame at the software cap (1000/120)
	float dtime; // frame time in seconds (set at end of every frame)

	// Diagnostics : filled at init, read by the console UI (System tab)
	struct {
		char vendor[64];
		char renderer[128];
		char version[64];
		char glsl_version[64];
		int  context_major, context_minor; // context actually created
		int  max_texture_size;
		int  max_vertex_attribs;
		int  max_uniform_components;
		bool glew_ok;
	} gl;

	// GL errors drained at the end of every frame (see end_frame)
	struct {
		uint total;            // total errors since launch
		uint this_frame;       // errors drained from the last frame
		uint last_code;        // most recent error code (0 = none)
		uint last_logged_code; // so each new error type is logged once
	} gl_errors;

	// OpenAL device status (filled at init)
	struct {
		bool ok;
		char device[128];
	} audio;

	// frame time history : avg/min/max + sparkline in the console
	float frame_ms[120];
	uint  frame_ms_count; // samples recorded so far (caps at 120)
	uint  frame_ms_idx;   // next slot to write
	float frame_ms_last, frame_ms_avg, frame_ms_min, frame_ms_max;
	float frame_ms_work; // cpu/gpu cost of this frame, display waits excluded
	float swap_wait_ms;  // how long glfwSwapBuffers blocked this frame (vsync)

	// deferred rendering
	struct {
		RenderTarget rendertarget;
		GLuint VAO, VBO, EBO; // for drawing the quad
		ShaderProgram shader;
	} gbuf; // G-Buffer

	void init(uint, uint);
	void begin_frame();
	void end_frame();

	void draw_gbuf(vec3 view_position);

	void shutdown();
};

// fly camera : WASD + mouse look. only moves while the game has focus, and
// imgui capture wins outright - navigating the menus never touches the camera
void update_fly_camera(Camera* cam, GameWindow* window, float dt)
{
	if (window->focus_status != GAME) return; // menus own the input (T frees the cursor)

	ImGuiIO& io = ImGui::GetIO();

	if (!io.WantCaptureMouse) // hovering a widget : don't spin the view
		cam->update_dir((float)window->mouse.dx, (float)window->mouse.dy, dt);

	if (!io.WantCaptureKeyboard) // typing into a widget : don't fly
	{
		float speed = (window->keys.SHIFT.is_pressed ? 18.f : 6.f) * dt; // m/s
		if (window->keys.W.is_pressed) cam->position += cam->front * speed;
		if (window->keys.S.is_pressed) cam->position -= cam->front * speed;
		if (window->keys.D.is_pressed) cam->position += cam->right * speed;
		if (window->keys.A.is_pressed) cam->position -= cam->right * speed;
	}
}

/* ------------------------- *
	-     Diagnostics       -
 * ------------------------- */

// safe copy of a glGetString() result (returns NULL if there is no context)
void copy_gl_string(char* dst, uint dst_size, GLenum name)
{
	const char* str = (const char*)glGetString(name);
	snprintf(dst, dst_size, "%s", str ? str : "?");
}

// human readable name for a glGetError() code (drained in end_frame)
const char* gl_error_name(uint code)
{
	switch (code)
	{
	case GL_INVALID_ENUM:                  return "GL_INVALID_ENUM";
	case GL_INVALID_VALUE:                 return "GL_INVALID_VALUE";
	case GL_INVALID_OPERATION:             return "GL_INVALID_OPERATION";
	case GL_OUT_OF_MEMORY:                 return "GL_OUT_OF_MEMORY";
	case GL_INVALID_FRAMEBUFFER_OPERATION: return "GL_INVALID_FRAMEBUFFER_OPERATION";
	default:                               return "UNKNOWN";
	}
}

void GameWindow::init(uint screen_width, uint screen_height)
{
	timer.init();
	frame_milliseconds_target = 1000.f / 120.f; // software cap : ms per frame
	timeBeginPeriod(1); // 1ms scheduler granularity so the cap sleep is accurate
	init_keyboard(&keys);

	this->screen_width  = screen_width;
	this->screen_height = screen_height;

	if (!glfwInit()) {
		console->add_entry((char*)"Init GLFW", SEVERITY::FIXME, LOGSOURCE::WNDW);
		return; // fatal : instance stays NULL, main loop never starts
	} console->add_entry((char*)"Init GLFW", SEVERITY::SUCCESS, LOGSOURCE::WNDW);

	// OpenGL window hints
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
	glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);

	this->instance = glfwCreateWindow(screen_width, screen_height, "GameWindow", NULL, NULL);

	if (!this->instance) {
		glfwTerminate();
		console->add_entry((char*)"no window instance", SEVERITY::FIXME);
		return; // fatal : main loop never starts
	}

	glfwMakeContextCurrent(this->instance);
	glfwSwapInterval(1); // vsync : 1 = swap on every monitor refresh, 0 = off

	// There needs to already be a glfw context *before* you call glew
	glewExperimental = GL_TRUE;
	GLenum glew_result = glewInit();
	gl.glew_ok = (glew_result == GLEW_OK);
	glGetError(); // GLEW generates a spurious GL_INVALID_ENUM on core profiles; drain it
	console->add_entry(gl.glew_ok ? "Init GLEW" : "glewInit FAILED", gl.glew_ok ? SUCCESS : FIXME, WNDW);

	// Gather GPU / context info for the console (System tab)
	copy_gl_string(gl.vendor      ,  sizeof(gl.vendor)  ,      GL_VENDOR);
	copy_gl_string(gl.renderer    ,  sizeof(gl.renderer),      GL_RENDERER);
	copy_gl_string(gl.version     ,  sizeof(gl.version) ,      GL_VERSION);
	copy_gl_string(gl.glsl_version,  sizeof(gl.glsl_version),  GL_SHADING_LANGUAGE_VERSION);
	gl.context_major = glfwGetWindowAttrib(instance, GLFW_CONTEXT_VERSION_MAJOR);
	gl.context_minor = glfwGetWindowAttrib(instance, GLFW_CONTEXT_VERSION_MINOR);
	glGetIntegerv(GL_MAX_TEXTURE_SIZE  ,            &gl.max_texture_size);
	glGetIntegerv(GL_MAX_VERTEX_ATTRIBS,            &gl.max_vertex_attribs);
	glGetIntegerv(GL_MAX_VERTEX_UNIFORM_COMPONENTS, &gl.max_uniform_components);

	glClearColor(.1, .2, .3, 1);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_CULL_FACE);
	//glPolygonMode(GL_FRONT_AND_BACK, GL_LINE); // wireframe

	// Capture cursor : start in game focus (Y recaptures after T frees it)
	focus_status = GAME;
	glfwSetInputMode(this->instance, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

	// audio init : TODO : move out of the window into its own place
	// status is recorded honestly so the console (System tab) can show it

	audio.ok = false;
	audio.device[0] = 0;

	ALCdevice* audio_device = alcOpenDevice(NULL);
	if (audio_device == NULL)
	{
		console->add_entry((char*)"cannot open sound card", FIXME, WNDW);
	}
	else
	{
		ALCcontext* audio_context = alcCreateContext(audio_device, NULL);
		if (audio_context == NULL)
		{
			console->add_entry((char*)"cannot create OpenAL context", FIXME, WNDW);
		}
		else
		{
			alcMakeContextCurrent(audio_context);
			audio.ok = true;
			snprintf(audio.device, sizeof(audio.device), "%s",
				(const char*)alcGetString(audio_device, ALC_DEVICE_SPECIFIER));
			console->add_entry((char*)"Init OpenAL", SUCCESS, WNDW);
		}
	}
	console->add_entry((char*)"Init Window", SUCCESS, WNDW);

	// Setup ImGUI Context
	IMGUI_CHECKVERSION();
	ImGui::CreateContext();
	ImGuiIO& io = ImGui::GetIO();

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

// poll input & events, swap buffers (the vsync block is measured as wait), imgui frame
void GameWindow::begin_frame()
{
	update_keyboard(&keys, instance);
	update_mouse(&mouse, instance, screen_width, screen_height);

	glfwPollEvents();
	int64 swap_start = timer.microseconds_elapsed();
	glfwSwapBuffers(instance);
	swap_wait_ms = (timer.microseconds_elapsed() - swap_start) / 1000.f; // display wait, not cost

	if (keys.ESC.is_pressed)
	{
		this->shutdown();
		return;
	}

	// T hands input to the menus (free cursor), Y gives it back to the game.
	// typing into a widget? the menus keep it (same rule as the camera)
	if (!ImGui::GetIO().WantCaptureKeyboard)
	{
		if (keys.T.is_pressed)
		{
			focus_status = GUI;
			glfwSetInputMode(instance, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
		}
		if (keys.Y.is_pressed)
		{
			focus_status = GAME;
			glfwSetInputMode(instance, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
		}
	}

	ImGui_ImplOpenGL3_NewFrame();
	ImGui_ImplGlfw_NewFrame();
	ImGui::NewFrame();
}
void GameWindow::end_frame()
{
	// ---- pacing : the cap sleep runs on total frame time (vsync composes with it)
	int64 work_us = timer.microseconds_elapsed();
	frame_ms_work = (work_us / 1000.f) - swap_wait_ms; // swap block split out as wait

	if (work_us / 1000.f < frame_milliseconds_target)
		os_sleep((uint)glm::ceil(frame_milliseconds_target - work_us / 1000.f));

	// ---- drain GL errors so the console sees them instead of them vanishing
	gl_errors.this_frame = 0;
	uint code;
	while ((code = glGetError()) != (uint)GL_NO_ERROR)
	{
		gl_errors.total++;
		gl_errors.this_frame++;
		gl_errors.last_code = code;

		// log each error type once, not 120 times a second
		if (code != gl_errors.last_logged_code)
		{
			char msg[MAX_LOG_MSG_LENGTH];
			snprintf(msg, MAX_LOG_MSG_LENGTH, "GL error 0x%04X (%s)", code, gl_error_name(code));
			console->add_entry(msg, FIXME, RNDR);
			gl_errors.last_logged_code = code;
		}
	}
	if (gl_errors.this_frame == 0)
		gl_errors.last_logged_code = 0; // clean frame -> log it again if it comes back

	// ---- frame timing : work + sleep + drain since the last frame = one true period
	int64 period_us = timer.microseconds_elapsed();

	dtime = period_us / 1000000.f;
	frame_ms_last = period_us / 1000.f;

	// record history for avg/min/max + the console's sparkline
	frame_ms[frame_ms_idx] = frame_ms_last;
	frame_ms_idx = (frame_ms_idx + 1) % 120;
	if (frame_ms_count < 120) frame_ms_count++;

	float ms_total = 0;
	frame_ms_min = 1000000.f;
	frame_ms_max = 0.f;
	for (uint i = 0; i < frame_ms_count; i++)
	{
		ms_total += frame_ms[i];
		if (frame_ms[i] < frame_ms_min) frame_ms_min = frame_ms[i];
		if (frame_ms[i] > frame_ms_max) frame_ms_max = frame_ms[i];
	}
	frame_ms_avg = frame_ms_count ? (ms_total / frame_ms_count) : 0.f;

	timer.start(); // begin timing next frame
}
void GameWindow::draw_gbuf(vec3 view_position)
{
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
	timeEndPeriod(1); // pairs the timeBeginPeriod from init
	this->instance = NULL;
}

Audio load_audio(const char* path)
{
	uint format, size, sample_rate;
	byte* audio_data = NULL;

	FILE* file = fopen(path, "rb"); // rb = read binary
	if (file == NULL)
	{
		char msg[MAX_LOG_MSG_LENGTH];
		snprintf(msg, MAX_LOG_MSG_LENGTH, "audio not found : %s", path);
		console->add_entry(msg, FIXME, WNDW);
		return 0;
	}

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