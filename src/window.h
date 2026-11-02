#include "audio.h"

/* ------------------------- *
	-   Window & Input      -
 * ------------------------- */

// fonts : the UI face and the two HUD faces. the default 13px atlas cannot be
// upscaled cleanly, so every face is rasterized at the size it draws at
ImFont* ui_font         = NULL;
ImFont* hud_font_small  = NULL;
ImFont* hud_font_big    = NULL;

// ---- lighting ------------------------------------------------------------
// everything the deferred pass reads from the scene. directions are the
// direction light travels; the shader flips them to get surface -> light.
struct Lighting
{
	vec3  sun_direction;
	vec3  sun_color;
	float sun_power;
	float sun_disk;      // the bright spot in the procedural sky

	vec3  rim_direction;
	vec3  rim_color;
	float rim_power;

	vec3  moon_direction; // light travel direction
	vec3  moon_color;
	float moon_power;
	float moon_disk;

	float exposure;
	float contrast;
	float saturation;
	float fog_density;
	float multiscatter;
	float ao_strength;

	vec3  zenith, haze, glow, ground;
	vec3  ambient_up, ambient_side, ambient_down;

	// 0 = day, 1 = night. flagged emitters use it
	float night;
};

// one local light : a point with a hard radius. the renderer draws a sphere of
// this radius over the g-buffer, so only the pixels it covers run the light math
const uint MAX_LIGHTS = 64;

enum Light_Shape : uint
{
	LIGHT_SPHERE = 0,
	LIGHT_CONE,
};

struct Point_Light
{
	vec3  position;
	float radius;
	vec3  color;
	float intensity;

	vec3  direction;   // cone axis, normalised
	float outer_angle; // radians, cone half-angle

	Light_Shape shape;
	float inner_angle; // radians, cone hot core
};

enum LightingPreset
{
	LIGHT_DAY = 0,
	LIGHT_OVERCAST,
	LIGHT_NOON,
	LIGHT_SUNSET,
	LIGHT_NIGHT,
	LIGHT_STUDIO,
	LIGHT_CARIBBEAN,
	LIGHT_TRADE_WINDS,
	LIGHT_TROPICAL_DUSK,
	LIGHT_PRESET_COUNT
};

const char* LIGHTING_PRESET_NAMES[LIGHT_PRESET_COUNT] = {
	"day", "overcast", "noon", "sunset", "night", "studio",
	"caribbean", "trade winds", "tropical dusk",
};

Lighting lighting_day()
{
	Lighting l = {};

	l.sun_direction = normalize(vec3(-0.36f, -0.84f, -0.30f));
	l.sun_color     = vec3(1.00f, 0.88f, 0.72f);
	l.sun_power     = 4.2f;
	l.sun_disk      = 1.0f;

	l.rim_direction = normalize(vec3(0.58f, -0.22f, 0.60f));
	l.rim_color     = vec3(0.48f, 0.66f, 0.95f);
	l.rim_power     = 0.70f;

	l.moon_direction = normalize(vec3(-0.25f, -0.65f, 0.72f));
	l.moon_color     = vec3(0.58f, 0.70f, 1.00f);
	l.moon_power     = 0.0f;
	l.moon_disk      = 0.0f;

	l.exposure      = 1.0f;
	l.contrast      = 1.03f;
	l.saturation    = 1.06f;
	l.fog_density   = 0.0032f; // clear enough to read the horizon
	l.multiscatter  = 0.40f;
	l.ao_strength   = 1.1f;

	l.zenith = vec3(0.080f, 0.160f, 0.340f);
	l.haze   = vec3(0.320f, 0.400f, 0.480f);
	l.glow   = vec3(0.820f, 0.550f, 0.300f);
	l.ground = vec3(0.040f, 0.040f, 0.035f);

	l.ambient_up   = vec3(0.460f, 0.550f, 0.720f);
	l.ambient_side = vec3(0.230f, 0.270f, 0.330f);
	l.ambient_down = vec3(0.070f, 0.065f, 0.060f);

	return l;
}

Lighting lighting_overcast()
{
	Lighting l = lighting_day();

	l.sun_direction = normalize(vec3(-0.18f, -0.94f, -0.28f));
	l.sun_color     = vec3(0.72f, 0.78f, 0.88f);
	l.sun_power     = 1.8f;
	l.sun_disk      = 0.06f;

	l.rim_color     = vec3(0.48f, 0.56f, 0.68f);
	l.rim_power     = 0.25f;

	l.exposure      = 1.08f;
	l.fog_density   = 0.0042f;

	l.zenith = vec3(0.280f, 0.330f, 0.420f);
	l.haze   = vec3(0.480f, 0.530f, 0.600f);
	l.glow   = vec3(0.550f, 0.580f, 0.620f);
	l.ground = vec3(0.110f, 0.110f, 0.110f);

	l.ambient_up   = vec3(0.550f, 0.600f, 0.680f);
	l.ambient_side = vec3(0.310f, 0.340f, 0.400f);
	l.ambient_down = vec3(0.100f, 0.100f, 0.100f);

	return l;
}

Lighting lighting_noon()
{
	Lighting l = lighting_day();

	l.sun_direction = normalize(vec3(-0.15f, -0.98f, -0.10f));
	l.sun_color     = vec3(1.00f, 0.97f, 0.92f);
	l.sun_power     = 4.8f;
	l.sun_disk      = 0.80f;

	l.rim_power     = 0.35f;
	l.exposure      = 0.98f;
	l.fog_density   = 0.0016f;

	l.zenith = vec3(0.040f, 0.200f, 0.550f);
	l.haze   = vec3(0.300f, 0.500f, 0.750f);
	l.glow   = vec3(0.620f, 0.720f, 0.880f);
	l.ground = vec3(0.040f, 0.040f, 0.038f);

	l.ambient_up   = vec3(0.420f, 0.600f, 0.900f);
	l.ambient_side = vec3(0.200f, 0.290f, 0.420f);
	l.ambient_down = vec3(0.070f, 0.080f, 0.090f);

	return l;
}

Lighting lighting_sunset()
{
	Lighting l = lighting_day();

	l.sun_direction = normalize(vec3(-0.82f, -0.20f, -0.34f));
	l.sun_color     = vec3(1.00f, 0.55f, 0.26f);
	l.sun_power     = 3.5f;
	l.sun_disk      = 1.3f;

	l.rim_color     = vec3(0.30f, 0.42f, 0.85f);
	l.rim_power     = 0.60f;

	l.exposure      = 1.12f;
	l.fog_density   = 0.0045f;

	l.zenith = vec3(0.050f, 0.080f, 0.190f);
	l.haze   = vec3(0.340f, 0.220f, 0.260f);
	l.glow   = vec3(1.000f, 0.450f, 0.180f);
	l.ground = vec3(0.050f, 0.035f, 0.030f);

	l.ambient_up   = vec3(0.380f, 0.310f, 0.460f);
	l.ambient_side = vec3(0.240f, 0.180f, 0.220f);
	l.ambient_down = vec3(0.060f, 0.045f, 0.040f);

	return l;
}

Lighting lighting_night()
{
	Lighting l = lighting_day();
	l.night = 1.f;

	l.sun_direction = normalize(vec3(0.35f, -0.72f, 0.42f));
	l.sun_color     = vec3(0.45f, 0.58f, 0.92f);
	l.sun_power     = 0.0f;
	l.sun_disk      = 0.0f;

	l.rim_color     = vec3(0.20f, 0.30f, 0.60f);
	l.rim_power     = 0.35f;

	l.moon_direction = normalize(vec3(0.35f, -0.72f, 0.42f));
	l.moon_color     = vec3(0.58f, 0.70f, 1.00f);
	l.moon_power     = 0.48f;
	l.moon_disk      = 1.0f;

	l.exposure      = 1.5f;
	l.fog_density   = 0.0032f;

	l.zenith = vec3(0.010f, 0.018f, 0.052f);
	l.haze   = vec3(0.030f, 0.040f, 0.075f);
	l.glow   = vec3(0.080f, 0.110f, 0.200f);
	l.ground = vec3(0.006f, 0.006f, 0.010f);

	l.ambient_up   = vec3(0.100f, 0.140f, 0.260f);
	l.ambient_side = vec3(0.050f, 0.060f, 0.100f);
	l.ambient_down = vec3(0.012f, 0.012f, 0.018f);

	return l;
}

Lighting lighting_studio()
{
	Lighting l = lighting_day();

	l.sun_direction = normalize(vec3(-0.40f, -0.85f, -0.35f));
	l.sun_color     = vec3(1.00f, 0.97f, 0.92f);
	l.sun_power     = 3.6f;
	l.sun_disk      = 0.35f;

	l.rim_direction = normalize(vec3(0.45f, -0.35f, 0.62f));
	l.rim_color     = vec3(0.55f, 0.65f, 0.95f);
	l.rim_power     = 0.85f;

	l.exposure      = 1.0f;
	l.fog_density   = 0.0f;
	l.ao_strength   = 1.2f;

	l.zenith = vec3(0.018f, 0.020f, 0.026f);
	l.haze   = vec3(0.045f, 0.048f, 0.055f);
	l.glow   = vec3(0.060f, 0.060f, 0.070f);
	l.ground = vec3(0.010f, 0.010f, 0.012f);

	l.ambient_up   = vec3(0.220f, 0.230f, 0.260f);
	l.ambient_side = vec3(0.110f, 0.115f, 0.130f);
	l.ambient_down = vec3(0.030f, 0.030f, 0.035f);

	return l;
}

// bright turquoise water, high sun slightly ahead : the specular path and the
// crest scatter both read from the default water camera
Lighting lighting_caribbean()
{
	Lighting l = lighting_day();

	l.sun_direction = normalize(vec3(-0.30f, -0.86f, 0.41f));
	l.sun_color     = vec3(1.00f, 0.96f, 0.86f);
	l.sun_power     = 4.9f;
	l.sun_disk      = 1.4f;

	l.rim_color     = vec3(0.38f, 0.74f, 1.00f);
	l.rim_power     = 0.55f;

	l.exposure      = 1.0f;
	l.contrast      = 1.05f;
	l.saturation    = 1.12f;
	l.fog_density   = 0.0012f;
	l.multiscatter  = 0.50f;

	l.zenith = vec3(0.025f, 0.260f, 0.620f);
	l.haze   = vec3(0.180f, 0.520f, 0.720f);
	l.glow   = vec3(0.900f, 0.780f, 0.550f);
	l.ground = vec3(0.030f, 0.090f, 0.100f);

	l.ambient_up   = vec3(0.400f, 0.720f, 0.950f);
	l.ambient_side = vec3(0.180f, 0.420f, 0.500f);
	l.ambient_down = vec3(0.050f, 0.110f, 0.100f);

	return l;
}

// late-morning trade winds : warm sun, bright water, a little atmospheric depth
Lighting lighting_trade_winds()
{
	Lighting l = lighting_day();

	l.sun_direction = normalize(vec3(-0.42f, -0.68f, 0.60f));
	l.sun_color     = vec3(1.00f, 0.88f, 0.68f);
	l.sun_power     = 4.3f;
	l.sun_disk      = 1.0f;

	l.rim_color     = vec3(0.48f, 0.72f, 0.98f);
	l.rim_power     = 0.65f;

	l.exposure      = 1.04f;
	l.contrast      = 1.04f;
	l.saturation    = 1.08f;
	l.fog_density   = 0.0024f;

	l.zenith = vec3(0.055f, 0.220f, 0.500f);
	l.haze   = vec3(0.300f, 0.480f, 0.620f);
	l.glow   = vec3(0.980f, 0.680f, 0.380f);
	l.ground = vec3(0.045f, 0.065f, 0.055f);

	l.ambient_up   = vec3(0.460f, 0.620f, 0.820f);
	l.ambient_side = vec3(0.240f, 0.340f, 0.420f);
	l.ambient_down = vec3(0.070f, 0.080f, 0.070f);

	return l;
}

// warm pink/orange dusk with a cool teal fill : the lamps and crest scatter carry
Lighting lighting_tropical_dusk()
{
	Lighting l = lighting_day();

	l.sun_direction = normalize(vec3(-0.55f, -0.22f, 0.81f));
	l.sun_color     = vec3(1.00f, 0.50f, 0.30f);
	l.sun_power     = 3.2f;
	l.sun_disk      = 1.5f;

	l.rim_color     = vec3(0.28f, 0.52f, 0.88f);
	l.rim_power     = 0.75f;

	l.exposure      = 1.10f;
	l.contrast      = 1.05f;
	l.saturation    = 1.10f;
	l.fog_density   = 0.0032f;

	l.zenith = vec3(0.045f, 0.070f, 0.220f);
	l.haze   = vec3(0.380f, 0.180f, 0.260f);
	l.glow   = vec3(1.000f, 0.380f, 0.200f);
	l.ground = vec3(0.050f, 0.025f, 0.035f);

	l.ambient_up   = vec3(0.360f, 0.280f, 0.480f);
	l.ambient_side = vec3(0.260f, 0.160f, 0.240f);
	l.ambient_down = vec3(0.060f, 0.030f, 0.040f);

	return l;
}

Lighting lighting_preset(uint kind)
{
	switch (kind)
	{
		case LIGHT_OVERCAST:      return lighting_overcast();
		case LIGHT_NOON:          return lighting_noon();
		case LIGHT_SUNSET:        return lighting_sunset();
		case LIGHT_NIGHT:         return lighting_night();
		case LIGHT_STUDIO:        return lighting_studio();
		case LIGHT_CARIBBEAN:     return lighting_caribbean();
		case LIGHT_TRADE_WINDS:   return lighting_trade_winds();
		case LIGHT_TROPICAL_DUSK: return lighting_tropical_dusk();
		default:                  return lighting_day();
	}
}

// a full solar day : the sun climbs from +X over the sea, the moon is opposite,
// and the sky/ambient/fog blend from midnight through dawn, noon and dusk
Lighting lighting_from_time(float hours)
{
	Lighting l = lighting_day();

	float a = (hours - 6.f) / 24.f * TWOPI;
	vec3 sun_pos = normalize(vec3(cosf(a) * .95f, sinf(a), -.55f)); // surface -> sun
	l.sun_direction = -sun_pos;

	float day      = glm::smoothstep(-.08f, .18f, sun_pos.y);
	float dusk     = expf(-fabsf(sun_pos.y) * 5.f);
	float sun_high = glm::smoothstep(.02f, .45f, sun_pos.y);

	l.sun_color = glm::mix(vec3(1.00f, .38f, .14f), vec3(1.00f, .95f, .86f), sun_high);
	l.sun_power = 4.8f * glm::smoothstep(-.02f, .16f, sun_pos.y);
	l.sun_disk  = glm::smoothstep(-.04f, .02f, sun_pos.y) * glm::mix(1.6f, 1.0f, sun_high);

	// moon : opposite the sun, visible only once the sun is below the horizon
	l.moon_direction = sun_pos;
	l.moon_color     = vec3(.55f, .68f, 1.00f);
	l.moon_disk      = glm::smoothstep(.02f, .18f, -sun_pos.y);
	l.moon_power     = .38f * l.moon_disk;

	vec3 day_zenith   = vec3(.030f, .200f, .550f);
	vec3 night_zenith = vec3(.010f, .018f, .055f);
	vec3 dusk_zenith  = vec3(.060f, .060f, .200f);
	l.zenith = glm::mix(night_zenith, day_zenith, day);
	l.zenith = glm::mix(l.zenith, dusk_zenith, dusk * .60f);

	vec3 day_haze   = vec3(.28f, .48f, .72f);
	vec3 night_haze = vec3(.030f, .040f, .080f);
	vec3 dusk_haze  = vec3(.60f, .26f, .20f);
	l.haze = glm::mix(night_haze, day_haze, day);
	l.haze = glm::mix(l.haze, dusk_haze, dusk * .80f);

	vec3 day_glow   = vec3(.70f, .72f, .80f);
	vec3 night_glow = vec3(.06f, .09f, .18f);
	vec3 dusk_glow  = vec3(1.00f, .36f, .14f);
	l.glow = glm::mix(night_glow, day_glow, day);
	l.glow = glm::mix(l.glow, dusk_glow, dusk * .90f);

	l.ground = glm::mix(vec3(.006f, .006f, .010f), vec3(.040f, .040f, .035f), day);

	l.ambient_up   = glm::mix(vec3(.10f, .14f, .26f), vec3(.42f, .60f, .90f), day);
	l.ambient_side = glm::mix(vec3(.05f, .06f, .10f), vec3(.20f, .29f, .42f), day);
	l.ambient_down = glm::mix(vec3(.012f, .012f, .018f), vec3(.07f, .08f, .09f), day);

	l.rim_direction = normalize(vec3(-sun_pos.x, -.25f, -sun_pos.z));
	l.rim_color     = vec3(.30f, .44f, .72f);
	l.rim_power     = glm::mix(.25f, .65f, day);

	l.exposure   = glm::mix(1.50f, 1.00f, day);
	l.contrast   = glm::mix(1.04f, 1.05f, day);
	l.saturation = glm::mix(1.04f, 1.10f, day) + dusk * .02f;
	l.fog_density = glm::mix(.0022f, .0028f, day);
	l.night = 1.f - day;

	return l;
}

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

#define NUM_KEYBOARD_BUTTONS 35 // update when adding keyboard buttons
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

			Button F1;
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

	keyboard->F1    = { false, false, GLFW_KEY_F1    };
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

		// depth texture : the water shader samples it, so it needs a complete
		// non-mipmapped sampler state (the default min filter expects mipmaps)
		glGenTextures(1, &depth_tx);
		glBindTexture(GL_TEXTURE_2D, depth_tx);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT24, width, height, 0, GL_DEPTH_COMPONENT, GL_FLOAT, NULL);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
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

// one linear HDR colour target. the lit target can carry a depth renderbuffer,
// filled from the g-buffer before forward water so the surface tests correctly
struct HdrTarget
{
	GLuint fbo;
	GLuint color_tx;
	GLuint depth_tx; // optional : lit depth, sampled by the sky-disk post pass
	bool complete;

	void init(uint height, uint width)
	{
		depth_tx = 0;
		glGenFramebuffers(1, &fbo);
		glBindFramebuffer(GL_FRAMEBUFFER, fbo);

		glGenTextures(1, &color_tx);
		glBindTexture(GL_TEXTURE_2D, color_tx);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA16F, width, height, 0, GL_RGBA, GL_FLOAT, NULL);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
		glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, color_tx, 0);

		complete = glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE;
		if (!complete) console->add_entry("HDR framebuffer incomplete", FIXME, WNDW);
	}

	void add_depth_buffer(uint height, uint width)
	{
		glGenTextures(1, &depth_tx);
		glBindTexture(GL_TEXTURE_2D, depth_tx);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT24, width, height, 0,
		             GL_DEPTH_COMPONENT, GL_FLOAT, NULL);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
		glBindFramebuffer(GL_FRAMEBUFFER, fbo);
		glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, depth_tx, 0);

		complete = glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE;
		if (!complete) console->add_entry("HDR depth attach failed", FIXME, WNDW);
	}

	void bind() { glBindFramebuffer(GL_FRAMEBUFFER, fbo); }
};

enum WindowFocus
{
	GUI = 0,
	GAME
};

struct GameWindow
{
	uint focus_status;
	bool ui_lock; // a scene overlay owns the cursor : T/Y must not steal it

	Mouse mouse;
	Keyboard keys;

	GLFWwindow* instance;
	uint screen_width, screen_height;
	float far_plane; // per-scene draw distance, set by the active scene


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

	// foam shading + short-chop normal map : filled by whichever scene owns the sea
	vec3  water_foam_color;
	float water_foam_roughness;
	uint  water_normal_tx;
	float water_detail;
	float water_sss;
	float water_time;

	// deferred rendering
	struct {
		RenderTarget rendertarget;
		GLuint VAO, VBO, EBO; // for drawing the quad
		ShaderProgram shader;
	} gbuf; // geometry pass : position, normal, albedo, depth

	// linear frame : deferred lighting and light volumes write here, water draws
	// over it, then the tonemap pass puts it on the screen. scene_copy is the
	// same frame without water, which is what water refracts
	struct {
		HdrTarget lit;
		HdrTarget scene_copy;
		ShaderProgram tonemap;
		ShaderProgram sky;
	} hdr;

	// filled by the renderer's shadow pass, read by the lighting shader
	GLuint shadow_tx;
	mat4   shadow_proj_view;
	vec3   shadow_direction;
	float  shadow_texel;
	float  shadow_world_texel;
	bool   shadow_on;
	bool   shadow_is_moon;

	// depth-only copy of the lit target : the sky-disk pass samples this instead
	// of a texture attached to the framebuffer it draws into
	GLuint sky_fbo, sky_depth_tx;

	void init(uint, uint);
	void set_focus(uint status);
	void begin_frame();
	void end_frame();

	void draw_lighting(vec3 view_position, mat4 proj_view, const Lighting* lighting);
	void draw_sky_disks(const Lighting* lighting, mat4 proj_view, vec3 camera_pos);
	void post_process(const Lighting* lighting);

	void shutdown();
};

// fly camera : WASD + mouse look. only moves while the game has focus, and
// imgui capture wins outright - navigating the menus never touches the camera
void update_fly_camera(Camera* cam, GameWindow* window, float dt, float speed_scale = 1.f)
{
	if (window->focus_status != GAME) return; // menus own the input (T frees the cursor)

	ImGuiIO& io = ImGui::GetIO();

	if (!io.WantCaptureMouse) // hovering a widget : don't spin the view
		cam->update_dir((float)window->mouse.dx, (float)window->mouse.dy, dt);

	if (!io.WantCaptureKeyboard) // typing into a widget : don't fly
	{
		// a large scene needs speed at altitude, but street-level flight should
		// still be controllable, so the free camera slows down near the ground
		float altitude = cam->position.y > 0.f ? cam->position.y : 0.f;
		float altitude_scale = .15f + .85f * fminf(altitude / 300.f, 1.f);
		float speed = (window->keys.SHIFT.is_pressed ? 18.f : 6.f) * dt *
		              speed_scale * altitude_scale; // m/s
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
	frame_milliseconds_target = 1000.f / 120.f; // frame cap : 120 fps
	far_plane = 256.f;

	water_foam_color = vec3(.82f, .88f, .90f);
	water_foam_roughness = .55f;
	water_detail = .70f;
	water_sss = .60f;
	water_time = 0.f;
	shadow_tx = 0;
	shadow_on = false;
	shadow_is_moon = false;
	shadow_direction = vec3(0, -1, 0);
	shadow_proj_view = mat4(1.f);
	shadow_texel = 1.f / 2048.f;
	shadow_world_texel = .04f;

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
	glfwSwapInterval(0); // vsync off : the 120 fps cap is owned by end_frame

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
	ui_lock = false;
	set_focus(GAME);

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

	// scale fonts + style sizes to the window : hand-placed pixels assume a 1080p design base
	float ui_scale = screen_height / 1080.f;
	io.FontGlobalScale = ui_scale;
	ImGui::GetStyle().ScaleAllSizes(ui_scale);

	// fonts : one TTF at UI size, plus the two HUD sizes the overlay draws at.
	// the console and the widgets use ui_font; the HUD picks its own face.
	const char* font_path = "assets/fonts/Roboto-Medium.ttf";
	FILE* font_file = fopen(font_path, "rb");
	if (font_file)
	{
		fclose(font_file);
		ui_font        = io.Fonts->AddFontFromFileTTF(font_path, 14.f);
		hud_font_small = io.Fonts->AddFontFromFileTTF(font_path, 16.f * ui_scale);
		hud_font_big   = io.Fonts->AddFontFromFileTTF(font_path, 40.f * ui_scale);
		io.FontDefault = ui_font;
	}
	else
	{
		io.Fonts->AddFontDefault();
		console->add_entry("font missing : assets/fonts/Roboto-Medium.ttf", FIXME, WNDW);
	}

	// Setup Platform/Renderer backends
	ImGui_ImplGlfw_InitForOpenGL(instance, true);
	ImGui_ImplOpenGL3_Init("#version 130");

	// G-Buffer
	gbuf.shader.create("assets/shaders/gbuf.vert", "assets/shaders/gbuf.frag");
	gbuf.shader.bind();

	gbuf.rendertarget.init(screen_height, screen_width);

	// linear HDR targets : the lit one shares the g-buffer depth for forward water
	hdr.lit.init(screen_height, screen_width);
	hdr.lit.add_depth_buffer(screen_height, screen_width);
	hdr.scene_copy.init(screen_height, screen_width);
	hdr.tonemap.create("assets/shaders/gbuf.vert", "assets/shaders/post.frag");
	hdr.sky.create("assets/shaders/gbuf.vert", "assets/shaders/sky.frag");

	glGenTextures(1, &sky_depth_tx);
	glBindTexture(GL_TEXTURE_2D, sky_depth_tx);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT24, screen_width, screen_height, 0,
	             GL_DEPTH_COMPONENT, GL_FLOAT, NULL);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glGenFramebuffers(1, &sky_fbo);
	glBindFramebuffer(GL_FRAMEBUFFER, sky_fbo);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, sky_depth_tx, 0);
	glDrawBuffer(GL_NONE);
	glReadBuffer(GL_NONE);
	glBindFramebuffer(GL_FRAMEBUFFER, 0);

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

void GameWindow::set_focus(uint status)
{
	focus_status = status;
	glfwSetInputMode(instance, GLFW_CURSOR, status == GAME ? GLFW_CURSOR_DISABLED
	                                                     : GLFW_CURSOR_NORMAL);
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

	// T hands input to the menus (free cursor), Y gives it back to the game.
	// typing into a widget? the menus keep it. a scene overlay locks this out
	if (!ImGui::GetIO().WantCaptureKeyboard && !ui_lock)
	{
		if (keys.T.is_pressed) set_focus(GUI);
		if (keys.Y.is_pressed) set_focus(GAME);
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
void GameWindow::draw_lighting(vec3 view_position, mat4 proj_view, const Lighting* lighting)
{
	// linear HDR target : no exposure or display encode yet, local lights come next
	hdr.lit.bind();
	glViewport(0, 0, screen_width, screen_height);
	glDisable(GL_DEPTH_TEST);
	glDisable(GL_BLEND);
	glDepthMask(GL_FALSE);
	glBindVertexArray(gbuf.VAO);
	glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT);

	gbuf.shader.bind();
	uint vp = glGetUniformLocation(gbuf.shader.id, "view_pos");
	glUniform3f(vp, view_position.x, view_position.y, view_position.z);

	// the occlusion taps project world points back onto this screen
	glUniformMatrix4fv(glGetUniformLocation(gbuf.shader.id, "proj_view"), 1, GL_FALSE, (float*)&proj_view);

	// per-scene lighting
	static Lighting fallback = lighting_day();
	const Lighting& l = lighting ? *lighting : fallback;

	vec3 sun_dir = l.sun_direction;
	vec3 rim_dir = l.rim_direction;
	if (dot(sun_dir, sun_dir) < 1e-6f) sun_dir = vec3(-0.36f, -0.84f, -0.30f);
	if (dot(rim_dir, rim_dir) < 1e-6f) rim_dir = vec3(0.58f, -0.22f, 0.60f);
	sun_dir = normalize(sun_dir);
	rim_dir = normalize(rim_dir);

	gbuf.shader.set_vec3 ("sun_direction", sun_dir);
	gbuf.shader.set_vec3 ("sun_color",     l.sun_color);
	gbuf.shader.set_float("sun_power",     l.sun_power);
	gbuf.shader.set_float("sun_disk",      0.f); // the post pass draws the real disk

	gbuf.shader.set_vec3 ("rim_direction", rim_dir);
	gbuf.shader.set_vec3 ("rim_color",     l.rim_color);
	gbuf.shader.set_float("rim_power",     l.rim_power);

	gbuf.shader.set_vec3 ("moon_direction", l.moon_direction);
	gbuf.shader.set_vec3 ("moon_color",     l.moon_color);
	gbuf.shader.set_float("moon_power",     l.moon_power);
	gbuf.shader.set_float("moon_disk",      0.f); // the post pass draws the real disk

	gbuf.shader.set_float("fog_density",   l.fog_density);
	gbuf.shader.set_float("multiscatter",  l.multiscatter);
	gbuf.shader.set_float("ao_strength",   l.ao_strength);
	gbuf.shader.set_float("night",         l.night);

	gbuf.shader.set_vec3("zenith", l.zenith);
	gbuf.shader.set_vec3("haze",   l.haze);
	gbuf.shader.set_vec3("glow",   l.glow);
	gbuf.shader.set_vec3("ground", l.ground);

	gbuf.shader.set_vec3("ambient_up",   l.ambient_up);
	gbuf.shader.set_vec3("ambient_side", l.ambient_side);
	gbuf.shader.set_vec3("ambient_down", l.ambient_down);

	glActiveTexture(GL_TEXTURE0); glBindTexture(GL_TEXTURE_2D, gbuf.rendertarget.position_tx);
	glActiveTexture(GL_TEXTURE1); glBindTexture(GL_TEXTURE_2D, gbuf.rendertarget.normal_tx  );
	glActiveTexture(GL_TEXTURE2); glBindTexture(GL_TEXTURE_2D, gbuf.rendertarget.albedo_tx  );

	gbuf.shader.set_mat4 ("light_proj_view", shadow_proj_view);
	gbuf.shader.set_vec3 ("shadow_direction", shadow_direction);
	gbuf.shader.set_float("shadow_texel", shadow_texel);
	gbuf.shader.set_float("shadow_world_texel", shadow_world_texel);
	gbuf.shader.set_float("shadow_on", shadow_on ? 1.f : 0.f);
	gbuf.shader.set_float("shadow_moon", shadow_is_moon ? 1.f : 0.f);
	gbuf.shader.set_float("shadow_bias", .0009f);
	if (shadow_on && shadow_tx)
	{
		glActiveTexture(GL_TEXTURE3);
		glBindTexture(GL_TEXTURE_2D, shadow_tx);
	}

	glDrawElementsInstanced(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0, 1); // Draw ScreenQuad

	glDepthMask(GL_TRUE);
	glEnable(GL_DEPTH_TEST);
}

// additive sky disks over the opaque frame, before water : anything that draws
// after this pass (water in particular) correctly covers the sun and moon
void GameWindow::draw_sky_disks(const Lighting* lighting, mat4 proj_view, vec3 camera_pos)
{
	static Lighting fallback = lighting_day();
	const Lighting& l = lighting ? *lighting : fallback;

	vec4 sun_clip  = proj_view * vec4(camera_pos - l.sun_direction * 200.f, 1.f);
	vec4 moon_clip = proj_view * vec4(camera_pos - l.moon_direction * 200.f, 1.f);
	vec2 sun_uv  = (sun_clip.w  > 0.f)
	             ? vec2(sun_clip.x  / sun_clip.w  * .5f + .5f, sun_clip.y  / sun_clip.w  * .5f + .5f)
	             : vec2(-1.f);
	vec2 moon_uv = (moon_clip.w > 0.f)
	             ? vec2(moon_clip.x / moon_clip.w * .5f + .5f, moon_clip.y / moon_clip.w * .5f + .5f)
	             : vec2(-1.f);

	// copy the opaque g-buffer depth so the disk shader can sample the mask
	// without reading the depth texture attached to the target it draws into
	glBindFramebuffer(GL_READ_FRAMEBUFFER, gbuf.rendertarget.fbo);
	glBindFramebuffer(GL_DRAW_FRAMEBUFFER, sky_fbo);
	glBlitFramebuffer(0, 0, screen_width, screen_height, 0, 0, screen_width, screen_height,
	                  GL_DEPTH_BUFFER_BIT, GL_NEAREST);

	hdr.lit.bind();
	glViewport(0, 0, screen_width, screen_height);
	glDisable(GL_DEPTH_TEST);
	glDepthMask(GL_FALSE);
	glDisable(GL_CULL_FACE);
	glEnable(GL_BLEND);
	glBlendFunc(GL_ONE, GL_ONE);

	hdr.sky.bind();
	hdr.sky.set_vec3("view_pos", vec3(0.f));
	hdr.sky.set_vec2("sun_screen", sun_uv);
	hdr.sky.set_vec2("moon_screen", moon_uv);
	hdr.sky.set_vec3("sun_color", l.sun_color);
	hdr.sky.set_vec3("moon_color", l.moon_color);
	hdr.sky.set_float("sun_strength", l.sun_disk);
	hdr.sky.set_float("moon_strength", l.moon_disk);
	hdr.sky.set_float("aspect", (float)screen_width / (float)screen_height);

	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, sky_depth_tx);

	glBindVertexArray(gbuf.VAO);
	glDrawElementsInstanced(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0, 1);

	glDisable(GL_BLEND);
	glEnable(GL_CULL_FACE);
	glDepthMask(GL_TRUE);
	glEnable(GL_DEPTH_TEST);
}

// exposure -> ACES -> display encode, from the linear lit target to the screen
void GameWindow::post_process(const Lighting* lighting)
{
	Lighting fallback = lighting_day();
	const Lighting& l = lighting ? *lighting : fallback;

	glBindFramebuffer(GL_FRAMEBUFFER, 0);
	glViewport(0, 0, screen_width, screen_height);
	glDisable(GL_DEPTH_TEST);
	glDisable(GL_BLEND);
	glDepthMask(GL_TRUE);
	glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glDepthMask(GL_FALSE);

	hdr.tonemap.bind();
	hdr.tonemap.set_float("exposure", l.exposure);
	hdr.tonemap.set_float("contrast", l.contrast);
	hdr.tonemap.set_float("saturation", l.saturation);
	hdr.tonemap.set_vec3("view_pos", vec3(0.f));

	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, hdr.lit.color_tx);

	glBindVertexArray(gbuf.VAO);
	glDrawElementsInstanced(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0, 1);

	glDepthMask(GL_TRUE);
	glEnable(GL_DEPTH_TEST);
}

// Terminates GLFW
// Sets instance to NULL (important)
void GameWindow::shutdown()
{
	glfwTerminate();
	timeEndPeriod(1); // pairs the timeBeginPeriod from init
	this->instance = NULL;
}