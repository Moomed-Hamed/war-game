#include "scene.h"

/* ------------------------- *
   ---  User Xperience   ---
 * ------------------------- */

enum Game_Screen
{
	GAME_SCREEN_MENU = 0,
	GAME_SCREEN_PLAYING,
	GAME_SCREEN_PAUSED
};

static int   game_screen = GAME_SCREEN_MENU;
static bool  game_options_open;
static bool  game_scene_select_open;
static bool  game_exit_requested;
static float game_master_volume = 1.f;
static bool  game_vsync;

namespace CNSL { // console

   const char* severity_messages[5] = {
    "T", // trace
    "W", // warning
    "F", // fixme
    "D", // debug
    "S"  // success
   };

   const char* source_messages[5] = {
       "WNDW", // Window
       "RNDR", // Renderer
       "PHYS", // Physics
       "NETW", // Networking
       "AUDI"  // Audio
   };

   // Source colors : one hue per subsystem, earthy pastels
   const ImVec4 source_colors[5] = {
       ImVec4(0.60f, 0.70f, 0.82f, 1.0f), // WNDW : Dusty Blue
       ImVec4(0.72f, 0.65f, 0.82f, 1.0f), // RNDR : Muted Mauve
       ImVec4(0.85f, 0.60f, 0.50f, 1.0f), // PHYS : Terracotta
       ImVec4(0.88f, 0.76f, 0.55f, 1.0f), // NETW : Sandy Tan
       ImVec4(0.72f, 0.78f, 0.56f, 1.0f)  // AUDI : Olive
   };

   // Severity levels : the badge dot - red/green keep their meaning, tuned earthy
   const ImVec4 severity_colors[5] = {
       ImVec4(0.70f, 0.62f, 0.80f, 1.0f), // TRACE   : Mauve (Insight)
       ImVec4(0.92f, 0.78f, 0.42f, 1.0f), // WARNING : Ochre (Caution)
       ImVec4(0.90f, 0.52f, 0.42f, 1.0f), // FIXME   : Terracotta Red (Problem)
       ImVec4(0.55f, 0.70f, 0.85f, 1.0f), // DEBUG   : Slate Blue (Info)
       ImVec4(0.55f, 0.80f, 0.55f, 1.0f)  // SUCCESS : Sage Green (OK)
   };

   // Message text : gentle pastels on the obsidian base
   const ImVec4 message_colors[5] = {
       ImVec4(0.82f, 0.76f, 0.88f, 1.0f), // TRACE   : Lavender Gray
       ImVec4(0.95f, 0.88f, 0.70f, 1.0f), // WARNING : Soft Ochre
       ImVec4(0.93f, 0.72f, 0.64f, 1.0f), // FIXME   : Clay
       ImVec4(0.72f, 0.82f, 0.90f, 1.0f), // DEBUG   : Dusty Blue
       ImVec4(0.72f, 0.88f, 0.74f, 1.0f)  // SUCCESS : Sage Mint
   };

   // data : neutral highlighted values (kv grid lines, table totals)
   const ImVec4 data_color = ImVec4(0.64f, 0.78f, 0.85f, 1.00f); // dusty blue
}

// ---------------- shared helpers ----------------

// design pixels -> screen pixels : fonts + style sizes scale once at startup
// (1080p design base); every hand-placed number goes through here
static float ui(float design_px)
{
   return design_px * ImGui::GetIO().FontGlobalScale;
}

// one table convention : fixed columns, row banding, borders. every table in the console
// is set up this way, so widths and flags do not have to be re-decided per tab
const ImGuiTableFlags TABLE_FLAGS = ImGuiTableFlags_SizingFixedFit
                                  | ImGuiTableFlags_RowBg
                                  | ImGuiTableFlags_Borders;

// title over a hairline separator
static void section(const char* title)
{
   ImGui::Spacing();
   ImGui::TextColored(ImVec4(0.90f, 0.72f, 0.44f, 1.f), "%s", title);
   ImGui::Separator();
}

// key/value grid line : dim key, data-colored value on a fixed column
static void kv(const char* key, const char* fmt, ...)
{
   ImGui::TextDisabled("%s", key);
   ImGui::SameLine(ui(90));

   char buf[128];
   va_list args;
   va_start(args, fmt);
   vsnprintf(buf, sizeof(buf), fmt, args);
   va_end(args);

   ImGui::TextColored(CNSL::data_color, "%s", buf);
}

// case-insensitive substring search for the Console tab filter
static bool log_contains(const char* haystack, const char* needle)
{
   if (needle[0] == 0) return true;

   for (const char* h = haystack; *h; h++)
   {
      const char* a = h;
      const char* b = needle;
      while (*a && *b)
      {
         char ca = *a, cb = *b;
         if (ca >= 'A' && ca <= 'Z') ca += 32;
         if (cb >= 'A' && cb <= 'Z') cb += 32;
         if (ca != cb) break;
         a++; b++;
      }
      if (*b == 0) return true;
   }
   return false;
}

// one colored log line : [SOURCE] [S] message
static void draw_log_entry(const Log_Entry& entry)
{
   using namespace CNSL;

   uint severity = entry.severity;
   uint source   = entry.source;
   if (severity > 4) severity = 4; // defensive : never index past the color arrays
   if (source > 4)   source   = 4;

   ImGui::TextColored(source_colors[source], "[%-4s]", source_messages[source]);
   ImGui::SameLine();
   ImGui::TextColored(severity_colors[severity], "[%s]", severity_messages[severity]);
   ImGui::SameLine();
   ImGui::TextColored(message_colors[severity], "[%s]", entry.text);
}

// ---------------- tab 1 : System ----------------
void draw_SYSTEM_console(GameWindow* window)
{
   using namespace CNSL;

   if (ImGui::BeginTabItem("System"))
   {
      // only failures speak : a healthy launch is one line, not three badges
      section("Status");
      if (window->gl.glew_ok && window->audio.ok && window->gl_errors.total == 0)
         ImGui::TextColored(severity_colors[SUCCESS], "all good");
      else
      {
         if (!window->gl.glew_ok)
            ImGui::TextColored(severity_colors[FIXME], "glew failed - extensions unavailable");
         if (!window->audio.ok)
            ImGui::TextColored(severity_colors[FIXME], "no audio device - sounds will not play");
         if (window->gl_errors.total)
            ImGui::TextColored(window->gl_errors.this_frame ? severity_colors[WARNING] : severity_colors[FIXME],
               "%u gl errors (%u last frame)", window->gl_errors.total, window->gl_errors.this_frame);
      }

      if (window->gl_errors.total) // only worth naming when there is one
         kv("last error", "0x%04X (%s)", window->gl_errors.last_code,
            gl_error_name(window->gl_errors.last_code));

      section("Context");
      kv("renderer", "%s", window->gl.renderer);
      kv("vendor", "%s", window->gl.vendor);
      kv("version", "%s (GLSL %s)", window->gl.version, window->gl.glsl_version);
      kv("window", "%d x %d, not resizable", window->screen_width, window->screen_height);
      kv("frame cap", "120 fps, vsync on");
      kv("audio", "%s", window->audio.ok ? window->audio.device : "none");

      section("Limits");
      kv("texture size", "%d", window->gl.max_texture_size);
      kv("attribs", "%d", window->gl.max_vertex_attribs);
      kv("uniform floats", "%d", window->gl.max_uniform_components);

      ImGui::EndTabItem();
   }
}

// ---------------- tab : Scenes ----------------
void draw_SCENES_console(SceneManager* scenes)
{
   using namespace CNSL;

   if (ImGui::BeginTabItem("Scenes"))
   {
      section("Scene");
      kv("active", "%s", SceneManager::label(scenes->current_kind));
      uint bodies = scenes->scene.phys.num_bodies ? scenes->scene.phys.num_bodies - 1 : 0;
      kv("contents", "%u props | %u bodies | %u joints",
         scenes->scene.num_props, bodies, scenes->scene.phys.num_joints);

      const char* names[SCENE_KIND_COUNT];
      for (uint k = 0; k < SCENE_KIND_COUNT; k++) names[k] = SceneManager::label(k);

      int pick = (int)scenes->requested_kind;
      ImGui::SetNextItemWidth(ui(220));
      if (ImGui::Combo("scene", &pick, names, SCENE_KIND_COUNT))
         scenes->request((uint)pick);
      ImGui::TextDisabled("switching reloads the scene");

      section("Day / night");
      scene_day_cycle_ui(&scenes->scene);

      section("Debug");
      ImGui::TextDisabled("%s", SceneManager::label(scenes->current_kind));
      scenes->draw_debug();

      ImGui::EndTabItem();
   }
}

// ---------------- tab 2 : Render ----------------
// the g-buffer inspector : deferred pipeline targets, one at a time, picked from a
// dropdown. given its own function so render concerns stay separable
static void draw_gbuffer_panel(GameWindow* window)
{
   using namespace CNSL;

   section("G-Buffer");

   if (window->gbuf.rendertarget.complete)
      ImGui::TextColored(severity_colors[SUCCESS], "framebuffer : COMPLETE");
   else
      ImGui::TextColored(severity_colors[FIXME], "framebuffer : INCOMPLETE - nothing will render");
   ImGui::SameLine();
   ImGui::Text("| %u x %u", window->screen_width, window->screen_height);

   static const char* gbuf_labels[6] = {
      "position [RGBA32F]  pos+metallic",
      "normal   [RGBA16F]  nrm+roughness",
      "albedo   [RGBA8]    color+AO",
      "depth    [DEPTH24]",
      "lit      [RGBA16F]  linear HDR",
      "copy     [RGBA16F]  under the water",
   };
   GLuint gbuf_tex[6] = {
      window->gbuf.rendertarget.position_tx,
      window->gbuf.rendertarget.normal_tx,
      window->gbuf.rendertarget.albedo_tx,
      window->gbuf.rendertarget.depth_tx,
      window->hdr.lit.color_tx,
      window->hdr.scene_copy.color_tx,
   };

   static int gbuf_pick = 0;
   ImGui::SetNextItemWidth(ui(300));
   ImGui::Combo("target", &gbuf_pick, gbuf_labels, 6);

   // one image at full panel width, true window aspect. uv flipped because
   // render-to-texture is upside down versus imgui
   float img_w = ImGui::GetContentRegionAvail().x;
   ImGui::Image((ImTextureID)(intptr_t)gbuf_tex[gbuf_pick],
      ImVec2(img_w, img_w * window->screen_height / (float)window->screen_width),
      ImVec2(0, 1), ImVec2(1, 0));
}

// the texture loader's contents : one texture, picked from a dropdown, with its size
static void draw_texture_panel(GameRenderer* rndr)
{
   using namespace CNSL;

   section("Textures");

   if (rndr->txloader.num_textures == 0)
   {
      ImGui::TextColored(severity_colors[WARNING], "no textures loaded");
      return;
   }

   static int tx_pick = 0;
   if (tx_pick >= (int)rndr->txloader.num_textures) tx_pick = 0;

   ImGui::SetNextItemWidth(ui(300));
   if (ImGui::BeginCombo("texture", rndr->txloader.textures[tx_pick].path))
   {
      for (uint i = 0; i < rndr->txloader.num_textures; i++)
      {
         char label[80];
         snprintf(label, sizeof(label), "%u : %s##tx%u", i, rndr->txloader.textures[i].path, i);
         if (ImGui::Selectable(label, (int)i == tx_pick)) tx_pick = (int)i;
         if ((int)i == tx_pick) ImGui::SetItemDefaultFocus();
      }
      ImGui::EndCombo();
   }

   Texture_Data* tx = &rndr->txloader.textures[tx_pick];
   kv("size", "%d x %d  |  handle %u", tx->width, tx->height, tx->handle);

   // 128 square, uv flipped : same convention as the g-buffer
   ImGui::Image((ImTextureID)(intptr_t)tx->handle, ImVec2(ui(128), ui(128)), ImVec2(0, 1), ImVec2(1, 0));
}

// ---- render tab panels : one function per concern, so the tab reads as a list --------

// frame timing : the headline number, the history plot and the work budget
static void draw_frame_panel(GameWindow* window)
{
   using namespace CNSL;

   section("Frame");

   if (window->frame_ms_count == 0) return; // not very first frame

   float fps = (window->frame_ms_last > 0) ? (1000.f / window->frame_ms_last) : 0;
   ImVec4 fps_color = (fps >= 60) ? severity_colors[SUCCESS]
      : (fps >= 30) ? severity_colors[WARNING]
      : severity_colors[FIXME];

   // the headline number, detail beside it
   ImGui::SetWindowFontScale(1.7f);
   ImGui::TextColored(fps_color, "%.0f fps", fps);
   ImGui::SetWindowFontScale(1.0f);
   ImGui::SameLine(0, ui(12));
   ImGui::Text("| %.2f ms   target 8.33 ms", window->frame_ms_last);

   if (window->frame_ms_count > 1)
      ImGui::PlotLines("##framems", window->frame_ms, (int)window->frame_ms_count, 0,
         NULL, 0.f, 33.f, ImVec2(ImGui::GetContentRegionAvail().x, ui(64)));
         // ^ explicit width : PlotLines does not accept -1

   kv("history", "avg %.1f   min %.1f   max %.1f ms",
      window->frame_ms_avg, window->frame_ms_min, window->frame_ms_max);

   // how much of the work budget this frame used (clamped: ImGui asserts fraction <= 1)
   float target = window->frame_milliseconds_target;
   float used = (target > 0) ? (window->frame_ms_work / target) : 0;
   if (used > 1.f) used = 1.f;
   char budget[48];
   snprintf(budget, 48, "budget : %.1f / %.1f ms", window->frame_ms_work, target);
   ImGui::PushStyleColor(ImGuiCol_PlotHistogram,
      (used >= 1.f) ? ImVec4(0.82f, 0.50f, 0.44f, 1.f) : ImVec4(0.58f, 0.72f, 0.82f, 1.f));
   ImGui::ProgressBar(used, ImVec2(-1, 0), budget);
   ImGui::PopStyleColor();

   kv("work / wait", "%.1f ms  /  %.1f ms",
      window->frame_ms_work, window->frame_ms_last - window->frame_ms_work);
}

// what the renderer submitted this frame, as GameRenderer::draw tallies it
static void draw_drawstats_panel(GameRenderer* rndr)
{
   using namespace CNSL;

   // instances/vertices-per-mesh live in the table's total row : no repeat here,
   // but the *instanced* vertex count is its own number and stays
   section("Draw Stats");
   kv("calls", "%u  (%u geometry + lighting)", rndr->stats.draw_calls + 1, rndr->stats.draw_calls);
   kv("meshes", "%u / %d", rndr->stats.meshes_drawn, MAX_MESHES);
   kv("tris", "%u  |  %u verts drawn", rndr->stats.triangles, rndr->stats.vertices);
}

// per-mesh draw list : proof each append_instances() landed. one row per mesh, the four
// modes filterable, so a busy scene can be read one layout at a time
static void draw_meshdrawlist_panel(GameRenderer* rndr)
{
   using namespace CNSL;

   static const ImVec4 mode_colors[DRAW_MODE_COUNT] = {
      ImVec4(0.65f, 0.76f, 0.87f, 1), // static + uv : dusty blue
      ImVec4(0.56f, 0.66f, 0.78f, 1), // water       : deeper blue
   };

   section("Mesh Draw List");

   // 0 = every mode at once, otherwise a DrawMode + 1
   static int pass_pick = 0;
   ImGui::SetNextItemWidth(ui(200));
   if (ImGui::BeginCombo("pass", (pass_pick == 0) ? "all modes" : DRAW_MODE_INFO[pass_pick - 1].name))
   {
      if (ImGui::Selectable("all modes", pass_pick == 0)) pass_pick = 0;
      for (uint m = 0; m < DRAW_MODE_COUNT; m++)
         if (ImGui::Selectable(DRAW_MODE_INFO[m].name, pass_pick == (int)m + 1)) pass_pick = (int)m + 1;
      ImGui::EndCombo();
   }

   if (ImGui::BeginTable("meshdraw", 7, TABLE_FLAGS))
   {
      ImGui::TableSetupColumn("pass"    , ImGuiTableColumnFlags_WidthFixed, ui(104));
      ImGui::TableSetupColumn("id"      , ImGuiTableColumnFlags_WidthFixed, ui(30));
      ImGui::TableSetupColumn("indices" , ImGuiTableColumnFlags_WidthFixed, ui(58));
      ImGui::TableSetupColumn("insts"   , ImGuiTableColumnFlags_WidthFixed, ui(44));
      ImGui::TableSetupColumn("verts"   , ImGuiTableColumnFlags_WidthFixed, ui(58));
      ImGui::TableSetupColumn("base vtx", ImGuiTableColumnFlags_WidthFixed, ui(58));
      ImGui::TableSetupColumn("base idx", ImGuiTableColumnFlags_WidthFixed, ui(62));
      ImGui::TableHeadersRow();

      // every mode writes the same fields, so one walk covers them all; the capture
      // accumulates the totals row
      uint tot_idx = 0, tot_inst = 0, tot_vert = 0;

      for (uint m = 0; m < DRAW_MODE_COUNT; m++)
      {
         if (pass_pick != 0 && pass_pick != (int)m + 1) continue;

         DrawBuffer& db = rndr->buffer(m);
         bool named = false; // the mode says its name once : its tint carries the rows

         for (uint i = 0; i < MAX_MESHES; i++)
         {
            if (db.mesh_info[i].mesh_id == 0) continue;

            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            if (!named) { ImGui::TextColored(mode_colors[m], "%s", DRAW_MODE_INFO[m].name); named = true; }
            ImGui::TableNextColumn(); ImGui::Text("%u", db.mesh_info[i].mesh_id);
            ImGui::TableNextColumn(); ImGui::Text("%u", db.mesh_info[i].num_indices);
            ImGui::TableNextColumn();
            if (db.mesh_info[i].num_instances == 0)
               ImGui::TextColored(severity_colors[WARNING], "%u", 0u); // never submitted this frame
            else
               ImGui::Text("%u", db.mesh_info[i].num_instances);
            ImGui::TableNextColumn(); ImGui::Text("%u", db.mesh_info[i].num_vertices);
            ImGui::TableNextColumn(); ImGui::Text("%u", db.mesh_info[i].base_vertex);
            ImGui::TableNextColumn(); ImGui::Text("%u", db.mesh_info[i].base_index);

            tot_idx  += db.mesh_info[i].num_indices;
            tot_inst += db.mesh_info[i].num_instances;
            tot_vert += db.mesh_info[i].num_vertices;
         }
      }

      // totals : what every mode submitted this frame
      ImGui::TableNextRow();
      ImGui::TableNextColumn(); ImGui::TextColored(ImVec4(0.63f, 0.61f, 0.58f, 1), "total");
      ImGui::TableNextColumn();
      ImGui::TableNextColumn(); ImGui::TextColored(data_color, "%u", tot_idx);
      ImGui::TableNextColumn(); ImGui::TextColored(data_color, "%u", tot_inst);
      ImGui::TableNextColumn(); ImGui::TextColored(data_color, "%u", tot_vert);
      ImGui::TableNextColumn();
      ImGui::TableNextColumn();
      ImGui::EndTable();
   }
}

// gpu buffers : one row per mode plus the whole lot, then the cpu side of the same story
static void draw_gpubuffers_panel(GameRenderer* rndr)
{
   using namespace CNSL;

   section("GPU Buffers");

   uint used = 0, reserved = 0;

   if (ImGui::BeginTable("gpubuffers", 5, TABLE_FLAGS))
   {
      ImGui::TableSetupColumn("mode"    , ImGuiTableColumnFlags_WidthFixed, ui(140));
      ImGui::TableSetupColumn("geometry", ImGuiTableColumnFlags_WidthFixed, ui(72));
      ImGui::TableSetupColumn("index"   , ImGuiTableColumnFlags_WidthFixed, ui(72));
      ImGui::TableSetupColumn("instance", ImGuiTableColumnFlags_WidthFixed, ui(72));
      ImGui::TableSetupColumn("used / limit (KB)", ImGuiTableColumnFlags_WidthFixed, ui(132));
      ImGui::TableHeadersRow();

      for (uint m = 0; m < DRAW_MODE_COUNT; m++)
      {
         DrawBuffer& db = rndr->buffer(m);

         ImGui::TableNextRow();
         ImGui::TableNextColumn(); ImGui::Text("%s", DRAW_MODE_INFO[m].name);
         ImGui::TableNextColumn(); ImGui::Text("%.1f", db.geom_size / 1024.f);
         ImGui::TableNextColumn(); ImGui::Text("%.1f", db.indx_size / 1024.f);
         ImGui::TableNextColumn(); ImGui::Text("%.1f", db.inst_size / 1024.f);
         ImGui::TableNextColumn(); ImGui::Text("%.1f / %.0f",
            (db.geom_size + db.indx_size + db.inst_size) / 1024.f, db.max_buffer_size / 1024.f);

         used     += db.geom_size + db.indx_size + db.inst_size;
         reserved += db.max_buffer_size * 3;
      }

      // every mode reserves geometry, index and instance space, so the floor is three
      // buffers' worth whether or not anything was loaded into them
      ImGui::TableNextRow();
      ImGui::TableNextColumn(); ImGui::TextColored(severity_colors[TRACE], "all modes");
      ImGui::TableNextColumn(); ImGui::TableNextColumn(); ImGui::TableNextColumn();
      ImGui::TableNextColumn(); ImGui::TextColored(severity_colors[TRACE], "%.1f / %.0f", used / 1024.f, reserved / 1024.f);

      ImGui::EndTable();
   }

   // and the cpu side : what the loaders hold besides the buffers
   uint tex_ram = 0;

   for (uint i = 0; i < rndr->txloader.num_textures; i++)
      tex_ram += rndr->txloader.textures[i].width * rndr->txloader.textures[i].height * 3; // decoded to rgb

   kv("textures", "~%.1f KB decoded", tex_ram / 1024.f);
}

// the scene owns the camera and the input : this only reads them
static void draw_camera_panel(GameWindow* window, Camera* camera)
{
   using namespace CNSL;

   section("Camera / Input");

   vec3 pos = camera->position;
   kv("position", "%6.2f  %6.2f  %6.2f", pos.x, pos.y, pos.z);
   kv("yaw / pitch", "%.1f / %.1f deg", ToDegrees(camera->yaw), ToDegrees(camera->pitch));
   kv("focus", "%s   (T frees the cursor, Y recaptures)",
      window->focus_status == GAME ? "GAME" : "GUI");
   kv("mouse", "%.0f, %.0f   delta %.1f, %.1f",
      window->mouse.raw_x, window->mouse.raw_y, window->mouse.dx, window->mouse.dy);
   kv("buttons", "L[%s] R[%s]   keys W[%d] A[%d] S[%d] D[%d]",
      window->mouse.left_button.is_pressed ? "down" : "up  ",
      window->mouse.right_button.is_pressed ? "down" : "up  ",
      window->keys.W.is_pressed, window->keys.A.is_pressed,
      window->keys.S.is_pressed, window->keys.D.is_pressed);
}

void draw_RENDER_console(GameRenderer* rndr, GameWindow* window, Camera* camera)
{
   if (ImGui::BeginTabItem("Render"))
   {
      draw_frame_panel(window);
      draw_drawstats_panel(rndr);
      draw_meshdrawlist_panel(rndr);
      draw_gpubuffers_panel(rndr);
      draw_gbuffer_panel(window);
      draw_camera_panel(window, camera);

      ImGui::EndTabItem();
   }
}

// ---------------- tab : Debug ----------------
void draw_DEBUG_console(GameRenderer* rndr, Scene* scene)
{
   using namespace CNSL;

   if (ImGui::BeginTabItem("Debug"))
   {
      section("Physics");
      ImGui::Checkbox("collider wires", &scene->phys.debug.enabled);
      ImGui::SameLine();
      ImGui::Checkbox("constraint wires", &scene->phys.debug.constraints);
      ImGui::SameLine();
      ImGui::Checkbox("interaction ray", &scene->phys.debug.rays);
      ImGui::TextDisabled("green awake | amber asleep | steel static | magenta constraints");

      section("Lighting");
      ImGui::Checkbox("light wires", &debug_draw.lights);
      ImGui::SameLine();
      ImGui::Checkbox("local lighting", &scene->local_lights);
      ImGui::Text("%u point lights", scene->num_lights);
      ImGui::TextDisabled("wire tint follows each light's colour");

      section("Rendering");
      ImGui::Checkbox("sun shadows", &rndr->shadow.enabled);
      ImGui::SameLine();
      ImGui::Checkbox("camera frustum", &debug_draw.camera);
      ImGui::SameLine();
      ImGui::Checkbox("world axes", &debug_draw.axes);
      ImGui::TextDisabled("axes : X red | Y green | Z blue | frustum cyan");

      section("Scene");
      ImGui::Text("%u props  |  %u lights", scene->num_props, scene->num_lights);

      if (ImGui::Button("all debug off"))
      {
         scene->phys.debug.enabled = false;
         scene->phys.debug.constraints = false;
         scene->phys.debug.rays = false;
         debug_draw.lights = false;
         debug_draw.camera = false;
         debug_draw.axes = false;
      }

      ImGui::EndTabItem();
   }
}

// ---------------- tab : Phys ----------------
void draw_PHYS_console(Physics* phys)
{
   using namespace CNSL;

   if (ImGui::BeginTabItem("Phys"))
   {
      // per-shape identity : chips and table rows share it
      static const char* shape_names[] = { "none", "box", "sphere", "capsule", "cylinder", "cone" };
      static const ImVec4 shape_colors[] = {
         ImVec4(1, 1, 1, 1),
         ImVec4(0.78f, 0.82f, 0.88f, 1), // box      : pale steel
         ImVec4(0.64f, 0.80f, 0.84f, 1), // sphere   : dusty aqua
         ImVec4(0.68f, 0.85f, 0.70f, 1), // capsule  : sage
         ImVec4(0.90f, 0.78f, 0.58f, 1), // cylinder : sand
         ImVec4(0.86f, 0.64f, 0.58f, 1), // cone     : clay
      };

      // joint kinds : hinge, cone twist, slider, fixed
      static const char* joint_names[] = { "none", "hinge", "cone", "slider", "fixed" };
      static const ImVec4 joint_colors[] = {
         ImVec4(1, 1, 1, 1),
         ImVec4(0.70f, 0.76f, 0.88f, 1), // hinge  : pale slate
         ImVec4(0.80f, 0.70f, 0.86f, 1), // cone   : soft mauve
         ImVec4(0.90f, 0.66f, 0.54f, 1), // slider : clay
         ImVec4(0.82f, 0.80f, 0.62f, 1), // fixed  : sand
      };

      // one tally feeds every readout on this tab
      uint shape_counts[6] = {};
      uint awake = 0, asleep = 0, total = 0;
      for (uint i = 1; i < phys->num_bodies; i++)
      {
         if (!phys->bodies[i].body) continue;
         total++;
         shape_counts[phys->bodies[i].shape]++;
         if (phys->bodies[i].body->getMass() == 0) continue; // static
         if (phys->is_awake(i)) awake++; else asleep++;
      }

      section("World");
      kv("gravity", "%.2f m/s2", phys->world->getGravity().y());
      kv("material", "friction %.2f | restitution %.2f | ground %.2f",
         PHYS_FRICTION, PHYS_RESTITUTION, GROUND_FRICTION);
      kv("world", "%u / %d bodies | %u / %d joints | solver %d iters",
         total, MAX_BODIES, phys->num_joints, MAX_JOINTS, phys->world->getSolverInfo().m_numIterations);

      ImGui::TextDisabled("shapes"); // one tinted tally per shape tag
      for (uint s = 1; s < 6; s++)
      {
         ImGui::SameLine();
         ImGui::TextColored(shape_colors[s], "%u %s", shape_counts[s], shape_names[s]);
      }

      section("Constraints");
      if (phys->num_joints == 0)
         ImGui::TextDisabled("none built yet");
      else if (ImGui::BeginTable("physjoints", 4,
         TABLE_FLAGS))
      {
         ImGui::TableSetupColumn("kind",       ImGuiTableColumnFlags_WidthFixed, ui(62));
         ImGui::TableSetupColumn("a",          ImGuiTableColumnFlags_WidthFixed, ui(30));
         ImGui::TableSetupColumn("b",          ImGuiTableColumnFlags_WidthFixed, ui(30));
         ImGui::TableSetupColumn("limits deg", ImGuiTableColumnFlags_WidthFixed, ui(158));
         ImGui::TableHeadersRow();

         const float to_deg = 180.f / PI;
         for (uint j = 0; j < phys->num_joints; j++)
         {
            Phys_Joint& jt = phys->joints[j];
            uint kind = (jt.kind <= 3) ? jt.kind : 0; // defensive : never index past the names

            ImGui::TableNextRow();
            ImGui::TableNextColumn(); ImGui::TextColored(joint_colors[kind], "%s", joint_names[kind]);
            ImGui::TableNextColumn(); ImGui::TextColored(shape_colors[phys->bodies[jt.a].shape], "%u", jt.a);
            ImGui::TableNextColumn(); ImGui::TextColored(shape_colors[phys->bodies[jt.b].shape], "%u", jt.b);
            ImGui::TableNextColumn();
            if (jt.kind == PHYS_CONETWIST) ImGui::Text("swing %.0f | twist %.0f", jt.lo * to_deg, jt.hi * to_deg);
            else                           ImGui::Text("%.0f .. %.0f", jt.lo * to_deg, jt.hi * to_deg);
         }
         ImGui::EndTable();
      }

      section("Bodies");
      // state chips : what the solver is actually doing right now
      ImGui::TextColored(severity_colors[WARNING], "%u awake", awake);
      ImGui::SameLine();
      ImGui::TextColored(ImVec4(0.56f, 0.68f, 0.79f, 1), "%u asleep", asleep);
      ImGui::SameLine();
      ImGui::TextColored(ImVec4(0.63f, 0.61f, 0.58f, 1), "%u static", total - awake - asleep);
      ImGui::SameLine(0, ui(12));
      if (ImGui::SmallButton("wake all")) phys->wake_all();
      ImGui::SameLine();
      if (ImGui::SmallButton("reset all")) phys->reset_all();

      ImGui::Checkbox("collider wires", &phys->debug.enabled);
      ImGui::SameLine();
      ImGui::Checkbox("constraint wires", &phys->debug.constraints);
      ImGui::SameLine();
      ImGui::Checkbox("ray debug", &phys->debug.rays);

      if (ImGui::BeginTable("physbodies", 6,
         TABLE_FLAGS))
      {
         ImGui::TableSetupColumn("id"   , ImGuiTableColumnFlags_WidthFixed, ui(28));
         ImGui::TableSetupColumn("shape", ImGuiTableColumnFlags_WidthFixed, ui(62));
         ImGui::TableSetupColumn("joints", ImGuiTableColumnFlags_WidthFixed, ui(46));
         ImGui::TableSetupColumn("pos", ImGuiTableColumnFlags_WidthFixed, ui(122));
         ImGui::TableSetupColumn("vel", ImGuiTableColumnFlags_WidthFixed, ui(122));
         ImGui::TableSetupColumn("state", ImGuiTableColumnFlags_WidthFixed, ui(52));
         ImGui::TableHeadersRow();

         for (uint i = 1; i < phys->num_bodies; i++)
         {
            Phys_Body& b = phys->bodies[i];
            if (!b.body) continue;

            vec3 pos = phys->get_position(i);
            vec3 vel = phys->get_velocity(i);
            bool moving = b.mass > 0 && phys->is_awake(i);

            ImGui::TableNextRow();
            if (moving) // live rows glow faint amber : motion at a glance
               ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg0, IM_COL32(255, 188, 80, 24));

            ImGui::TableNextColumn(); ImGui::Text("%u", i);
            ImGui::TableNextColumn();
            ImGui::TextColored(shape_colors[b.shape], "%s", shape_names[b.shape]);
            ImGui::TableNextColumn();
            uint linked = phys->joints_of(i);
            if (linked) ImGui::TextColored(data_color, "%u", linked);
            else        ImGui::TextDisabled("-");
            ImGui::TableNextColumn(); ImGui::Text("%6.2f %6.2f %6.2f", pos.x, pos.y, pos.z);
            ImGui::TableNextColumn(); ImGui::Text("%6.2f %6.2f %6.2f", vel.x, vel.y, vel.z);
            ImGui::TableNextColumn();
            if (b.mass == 0)
               ImGui::TextColored(ImVec4(0.63f, 0.61f, 0.58f, 1), "static");
            else if (moving)
               ImGui::TextColored(severity_colors[WARNING], "awake");
            else
               ImGui::TextColored(ImVec4(0.56f, 0.68f, 0.79f, 1), "sleep");
         }
         ImGui::EndTable();
      }

      ImGui::EndTabItem();
   }
}

// ---------------- tab 3 : Assets ----------------
void draw_ASST_console(GameWindow* window, GameRenderer* rndr)
{
   using namespace CNSL;

   if (ImGui::BeginTabItem("Assets"))
   {
      // ---- textures : the loader's contents, one at a time
      draw_texture_panel(rndr);

      // ---- shaders : compile/link results queried from GL at startup
      section("Shaders");
      {
         auto shader_row = [](const char* label, const char* files, ShaderProgram* s)
         {
            if (!s->id) return;
            ImGui::Text("%s (%s) : ", label, files);
            ImGui::SameLine();
            if (s->link_status)
               ImGui::TextColored(severity_colors[SUCCESS], "linked, %d uniforms, %d attribs",
                  s->num_uniforms, s->num_attributes);
            else
               ImGui::TextColored(severity_colors[FIXME], "LINK FAILED - see Console tab");
         };

         shader_row("geometry ", "geom.vert/frag", &rndr->shaders[0]);
         shader_row("water    ", "water.vert/frag", &rndr->shaders[1]);
         shader_row("lighting ", "gbuf.vert/frag", &window->gbuf.shader);
      }

      ImGui::EndTabItem();
   }
}

// ---------------- tab : Player ----------------
void draw_PLAYER_console(Scene* scene)
{
   using namespace CNSL;

   Player* player = &scene->player;

   if (ImGui::BeginTabItem("Player"))
   {
      section("View");
      kv("mode", player->flycam ? "fly camera (F)" : "capsule (F to fly)");
      kv("position", "%.2f %.2f %.2f", player->camera.position.x, player->camera.position.y,
         player->camera.position.z);
      kv("grounded / bob", "%s / %.2f", player->grounded ? "yes" : "no", player->bob);

      ImGui::EndTabItem();
   }
}

// ---------------- tab : Audio ----------------
void draw_AUDIO_console()
{
   using namespace CNSL;

   if (ImGui::BeginTabItem("Audio"))
   {
      section("Master");
      float master = audio.master;
      ImGui::SetNextItemWidth(ui(180));
      if (ImGui::SliderFloat("gain", &master, 0.f, 1.f, "%.2f")) audio.set_master(master);
      ImGui::SameLine();
      if (ImGui::SmallButton("halt all")) audio.halt_all();
      ImGui::SameLine();
      ImGui::TextDisabled("%u sounds / %u slots", audio.num_sounds, MAX_SOUNDS);

      section("Sounds");

      // mixer queries take the backend lock : sample them a few times a second
      static bool  live[MAX_SOUNDS] = {};
      static float head[MAX_SOUNDS] = {};
      static uint  tick = 0;
      if ((tick++ % 8) == 0)
         for (uint i = 1; i <= audio.num_sounds; i++)
         {
            live[i - 1] = audio.playing(i);
            head[i - 1] = audio.head(i);
         }

      if (audio.num_sounds == 0)
         ImGui::TextDisabled("nothing loaded yet");
      else if (ImGui::BeginTable("audiosounds", 6,
         TABLE_FLAGS))
      {
         ImGui::TableSetupColumn("name", ImGuiTableColumnFlags_WidthFixed, ui(72));
         ImGui::TableSetupColumn("gain", ImGuiTableColumnFlags_WidthFixed, ui(102));
         ImGui::TableSetupColumn("pitch", ImGuiTableColumnFlags_WidthFixed, ui(98));
         ImGui::TableSetupColumn("wave / playhead", ImGuiTableColumnFlags_WidthFixed, ui(176));
         ImGui::TableSetupColumn("stop", ImGuiTableColumnFlags_WidthFixed, ui(52));
         ImGui::TableSetupColumn("loop", ImGuiTableColumnFlags_WidthFixed, ui(46));
         ImGui::TableHeadersRow();

         for (uint i = 1; i <= audio.num_sounds; i++)
         {
            uint s = i - 1;
            ImGui::PushID((int)s);
            ImGui::TableNextRow();
            if (live[s]) // live rows glow faint amber
               ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg0, IM_COL32(230, 184, 112, 20));

            // the name is a button as well : two hit targets per row
            ImGui::TableNextColumn();
            if (ImGui::SmallButton(audio.sounds[s].name)) audio.play(i);
            ImGui::TextDisabled("%.2fs", audio.sounds[s].seconds);

            ImGui::TableNextColumn();
            float gain = audio.sounds[s].gain;
            ImGui::SetNextItemWidth(ui(90));
            if (ImGui::SliderFloat("##gain", &gain, 0.f, 1.f, "%.2f")) audio.set_gain(i, gain);

            ImGui::TableNextColumn();
            float pitch = audio.sounds[s].pitch;
            ImGui::SetNextItemWidth(ui(86));
            if (ImGui::SliderFloat("##pitch", &pitch, .25f, 2.5f, "%.2f")) audio.set_pitch(i, pitch);

            // envelope with the playhead riding it
            ImGui::TableNextColumn();
            {
               ImVec2 p = ImGui::GetCursorScreenPos();
               float w = ui(144), h = ui(20), mid = p.y + h * .5f;
               ImDrawList* dl = ImGui::GetWindowDrawList();

               dl->AddRectFilled(p, ImVec2(p.x + w, p.y + h), IM_COL32(10, 12, 18, 255));
               dl->AddRect(p, ImVec2(p.x + w, p.y + h), IM_COL32(70, 78, 92, 255));

               for (uint k = 0; k < WAVE_POINTS; k++)
               {
                  float x = p.x + w * (k / (float)WAVE_POINTS);
                  float amp = audio.sounds[s].wave[k] * h * .42f;
                  dl->AddRectFilled(ImVec2(x, mid - amp), ImVec2(x + ui(1.2f), mid + amp), IM_COL32(170, 190, 205, 200));
               }
               if (live[s])
               {
                  float x = p.x + w * head[s];
                  dl->AddLine(ImVec2(x, p.y), ImVec2(x, p.y + h), IM_COL32(230, 184, 112, 255), ui(1.5f));
               }
               ImGui::Dummy(ImVec2(w, h));
            }

            // play lives on the name above ; this column just silences the voices
            ImGui::TableNextColumn();
            if (ImGui::SmallButton("stop")) audio.halt(i);

            ImGui::TableNextColumn();
            bool loop = audio.sounds[s].loop;
            if (ImGui::Checkbox("##loop", &loop)) audio.set_loop(i, loop);

            ImGui::PopID();
         }
         ImGui::EndTable();
      }

      ImGui::EndTabItem();
   }
}

// ---------------- tab 5 : Console ----------------
void draw_MAIN_console(GameConsole* console)
{
   using namespace CNSL;

   LogQueue& logs = console->logs;

   // filter state (persists while the app runs)
   static bool severity_visible[5] = { true, true, true, true, true };
   static bool autoscroll = true;
   static char search[64] = {};

   if (ImGui::BeginTabItem("Console"))
   {
      // ---- row 1 : severity chips with live counts
      for (uint s = 0; s < 5; s++)
      {
         ImGui::PushStyleColor(ImGuiCol_Text, severity_visible[s]
            ? severity_colors[s] : ImVec4(0.48f, 0.46f, 0.43f, 0.6f));
         char label[20];
         snprintf(label, sizeof(label), "%s %u##sev%u", severity_messages[s], console->count(s), s);
         if (ImGui::SmallButton(label)) severity_visible[s] = !severity_visible[s];
         ImGui::PopStyleColor();
         if (s < 4) ImGui::SameLine();
      }

      ImGui::SameLine();
      ImGui::TextColored(severity_colors[WARNING], "%u warnings", console->count(WARNING));
      ImGui::SameLine();
      ImGui::TextColored(severity_colors[FIXME], "%u errors", console->count(FIXME));

      // ---- row 2 : search, autoscroll, clear
      ImGui::SetNextItemWidth(-ui(190));
      ImGui::InputText("##logsearch", search, sizeof(search));
      ImGui::SameLine();
      ImGui::Checkbox("autoscroll", &autoscroll);
      ImGui::SameLine();
      if (ImGui::Button("Clear")) console->clear();

      ImGui::Separator();

      // ---- entries, oldest -> newest (rolling buffer wraps, so 2 passes)
      auto draw_range = [&](uint begin, uint end)
      {
         for (uint i = begin; i < end; i++)
         {
            const Log_Entry& entry = logs.entries[i];
            if (entry.text[0] == 0) continue;

            uint severity = entry.severity;
            if (severity > 4) continue;
            if (!severity_visible[severity]) continue;

            if (search[0])
            {
               bool match = log_contains(entry.text, search);
               if (!match && entry.source <= 3)
                  match = log_contains(source_messages[entry.source], search);
               if (!match) continue;
            }

            draw_log_entry(entry);
         }
      };
      // the log scrolls inside its own child, so autoscroll can never drag the
      // tab bar out of reach : the tabs stay put, the lines scroll beneath them
      ImGui::BeginChild("##log", ImVec2(0, 0), false);
      {
         draw_range(logs.write_idx, MAX_LOGQUEUE_ENTRIES);
         draw_range(0, logs.write_idx);

         if (autoscroll) ImGui::SetScrollHereY(1.0f); // keep newest line visible

         // reaching back up lets go of autoscroll instead of fighting it
         if (ImGui::IsWindowHovered() && ImGui::GetIO().MouseWheel > 0) autoscroll = false;
      }
      ImGui::EndChild();

      ImGui::EndTabItem();
   }
}

// material names, projected onto their spheres : the only text the scene gets.
// props without a label (everything except the material grid) are skipped
void draw_world_labels(Scene* scene, GameWindow* window)
{
   if (!scene->show_labels) return;

   ImDrawList* dl = ImGui::GetBackgroundDrawList(); // over the frame, under the console

   for (uint i = 0; i < scene->num_props; i++)
   {
      Scene::Prop& p = scene->props[i];
      if (!p.label[0]) continue;

      // just above the prop, so the sphere itself stays readable
      vec4 clip = scene->proj_view * vec4(p.pos + vec3(0, p.scale.y + 0.25f, 0), 1.f);
      if (clip.w < 0.001f) continue; // behind the camera

      // a touch under the ui font : world text should not shout
      const ImFont* font = ImGui::GetFont();
      const float label_size = 11.f;

      vec2 ndc = vec2(clip) / clip.w;
      if (ndc.x < -1.f || ndc.x > 1.f || ndc.y < -1.f || ndc.y > 1.f) continue; // off screen

      ImVec2 size = font->CalcTextSizeA(label_size, 9999.f, 0.f, p.label);
      ImVec2 at((ndc.x * 0.5f + 0.5f) * window->screen_width - size.x * 0.5f,
                (0.5f - ndc.y * 0.5f) * window->screen_height);

      dl->AddText(font, label_size, ImVec2(at.x + 1.f, at.y + 1.f), IM_COL32(0, 0, 0, 200), p.label); // shadow : reads over sky
      dl->AddText(font, label_size, at, IM_COL32(235, 238, 243, 255), p.label);
   }
}

static void game_apply_options()
{
	audio.set_master(game_master_volume);
	glfwSwapInterval(game_vsync ? 1 : 0);
}

static void game_start_scene(GameWindow* window, SceneManager* scenes, uint kind)
{
	scenes->request(kind);
	game_screen = GAME_SCREEN_PLAYING;
	game_options_open = false;
	game_scene_select_open = false;
	ux_panel_visible = false;
	window->ui_lock = false;
	window->set_focus(GAME);
}

static void game_go_menu(GameWindow* window, SceneManager* scenes)
{
	scenes->request(SCENE_MENU);
	game_screen = GAME_SCREEN_MENU;
	game_options_open = false;
	game_scene_select_open = false;
	ux_panel_visible = false;
	window->ui_lock = true;
	window->set_focus(GUI);
}

static void game_pause(GameWindow* window)
{
	game_screen = GAME_SCREEN_PAUSED;
	game_options_open = false;
	game_scene_select_open = false;
	window->ui_lock = true;
	window->set_focus(GUI);
}

static void game_resume(GameWindow* window)
{
	game_screen = GAME_SCREEN_PLAYING;
	game_options_open = false;
	game_scene_select_open = false;
	window->ui_lock = false;
	window->set_focus(GAME);
}

void game_handle_keys(GameWindow* window)
{
	if (!(window->keys.ESC.is_pressed && !window->keys.ESC.was_pressed)) return;

	if (game_options_open)
	{
		game_options_open = false;
		return;
	}
	if (game_scene_select_open)
	{
		game_scene_select_open = false;
		return;
	}

	if (game_screen == GAME_SCREEN_PLAYING) game_pause(window);
	else if (game_screen == GAME_SCREEN_PAUSED) game_resume(window);
}

static bool menu_button(const char* label, float scale)
{
	return ImGui::Button(label, ImVec2(-1.f, 38.f * scale));
}

static void draw_options_menu(GameWindow* window)
{
	float scale = window->screen_height / 1080.f;
	ImGuiIO& io = ImGui::GetIO();
	ImGui::GetBackgroundDrawList()->AddRectFilled(
		ImVec2(0, 0), io.DisplaySize, IM_COL32(6, 8, 12, 190));

	ImGui::SetNextWindowPos(ImVec2(io.DisplaySize.x * .5f, io.DisplaySize.y * .5f),
	                        ImGuiCond_Always, ImVec2(.5f, .5f));
	ImGui::SetNextWindowSize(ImVec2(420.f * scale, 0.f));
	if (!ImGui::Begin("##options", NULL, ImGuiWindowFlags_NoTitleBar |
	                  ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
	                  ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_AlwaysAutoResize))
	{
		ImGui::End();
		return;
	}

	ImGui::SetWindowFontScale(scale);
	ImGui::TextColored(ImVec4(.90f, .72f, .44f, 1.f), "OPTIONS");
	ImGui::Separator();

	ImGui::TextDisabled("display");
	ImGui::SliderFloat("field of view", &game_fov, 50.f, 110.f, "%.0f deg");
	if (ImGui::Checkbox("vsync", &game_vsync)) game_apply_options();
	ImGui::TextDisabled("vsync overrides the 120 fps software cap when the display is faster");

	ImGui::Separator();
	ImGui::TextDisabled("audio");
	if (ImGui::SliderFloat("master volume", &game_master_volume, 0.f, 1.f, "%.2f"))
		game_apply_options();

	ImGui::Separator();
	if (menu_button("back", scale)) game_options_open = false;

	ImGui::End();
}

static void draw_scene_select(GameWindow* window, SceneManager* scenes)
{
	float scale = window->screen_height / 1080.f;
	ImGuiIO& io = ImGui::GetIO();
	ImGui::GetBackgroundDrawList()->AddRectFilled(
		ImVec2(0, 0), io.DisplaySize, IM_COL32(6, 8, 12, 190));

	ImGui::SetNextWindowPos(ImVec2(io.DisplaySize.x * .5f, io.DisplaySize.y * .5f),
	                        ImGuiCond_Always, ImVec2(.5f, .5f));
	ImGui::SetNextWindowSize(ImVec2(420.f * scale, 0.f));
	if (!ImGui::Begin("##scenes", NULL, ImGuiWindowFlags_NoTitleBar |
	                  ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
	                  ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_AlwaysAutoResize))
	{
		ImGui::End();
		return;
	}

	ImGui::SetWindowFontScale(scale);
	ImGui::TextColored(ImVec4(.90f, .72f, .44f, 1.f), "SCENES");
	ImGui::Separator();

	for (uint i = 0; i < SCENE_KIND_COUNT; i++)
	{
		if (i == SCENE_MENU) continue;
		if (ImGui::Button(SceneManager::label(i), ImVec2(-1.f, 34.f * scale)))
			game_start_scene(window, scenes, i);
	}

	ImGui::Separator();
	if (menu_button("back", scale)) game_scene_select_open = false;
	ImGui::End();
}

static void draw_main_menu(GameWindow* window)
{
	float scale = window->screen_height / 1080.f;
	ImGuiIO& io = ImGui::GetIO();
	ImGui::GetBackgroundDrawList()->AddRectFilled(
		ImVec2(0, 0), io.DisplaySize, IM_COL32(5, 7, 10, 205));

	hud_text_center(ImGui::GetBackgroundDrawList(), 54.f * scale,
	                io.DisplaySize.x * .5f, io.DisplaySize.y * .22f,
	                IM_COL32(235, 238, 243, 245), "PROJECT ONE");
	hud_text_center(ImGui::GetBackgroundDrawList(), 15.f * scale,
	                io.DisplaySize.x * .5f, io.DisplaySize.y * .22f + 58.f * scale,
	                IM_COL32(170, 178, 190, 220), "engine / physics / water");

	ImGui::SetNextWindowPos(ImVec2(io.DisplaySize.x * .5f, io.DisplaySize.y * .52f),
	                        ImGuiCond_Always, ImVec2(.5f, .5f));
	ImGui::SetNextWindowSize(ImVec2(400.f * scale, 0.f));
	if (!ImGui::Begin("##mainmenu", NULL, ImGuiWindowFlags_NoTitleBar |
	                  ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
	                  ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_AlwaysAutoResize))
	{
		ImGui::End();
		return;
	}

	ImGui::SetWindowFontScale(scale);
	if (menu_button("scenes", scale))
	{
		game_scene_select_open = true;
		game_options_open = false;
	}
	if (menu_button("options", scale))
	{
		game_options_open = true;
		game_scene_select_open = false;
	}
	ImGui::Separator();
	if (menu_button("exit", scale)) game_exit_requested = true;

	ImGui::End();
}

static void draw_pause_menu(GameWindow* window, SceneManager* scenes)
{
	float scale = window->screen_height / 1080.f;
	ImGuiIO& io = ImGui::GetIO();
	ImGui::GetBackgroundDrawList()->AddRectFilled(
		ImVec2(0, 0), io.DisplaySize, IM_COL32(5, 7, 10, 185));

	ImGui::SetNextWindowPos(ImVec2(io.DisplaySize.x * .5f, io.DisplaySize.y * .5f),
	                        ImGuiCond_Always, ImVec2(.5f, .5f));
	ImGui::SetNextWindowSize(ImVec2(360.f * scale, 0.f));
	if (!ImGui::Begin("##pause", NULL, ImGuiWindowFlags_NoTitleBar |
	                  ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
	                  ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_AlwaysAutoResize))
	{
		ImGui::End();
		return;
	}

	ImGui::SetWindowFontScale(scale);
	ImGui::TextColored(ImVec4(.90f, .72f, .44f, 1.f), "PAUSED");
	ImGui::Separator();
	if (menu_button("resume", scale)) game_resume(window);
	if (menu_button("options", scale)) game_options_open = true;
	if (menu_button("scenes", scale)) game_scene_select_open = true;
	if (menu_button("main menu", scale)) game_go_menu(window, scenes);
	ImGui::Separator();
	if (menu_button("exit", scale)) game_exit_requested = true;

	ImGui::End();
}

void draw_game_menus(GameWindow* window, SceneManager* scenes)
{
	if (game_options_open)
	{
		draw_options_menu(window);
		return;
	}
	if (game_scene_select_open)
	{
		draw_scene_select(window, scenes);
		return;
	}

	if (game_screen == GAME_SCREEN_MENU) draw_main_menu(window);
	else if (game_screen == GAME_SCREEN_PAUSED) draw_pause_menu(window, scenes);
}

void draw_console(GameConsole* console, GameRenderer* rndr, GameWindow* window, SceneManager* scenes)
{
   using namespace CNSL;

   Scene* scene = &scenes->scene;

   // the scene's annotations : only while actually playing
   if (game_screen == GAME_SCREEN_PLAYING)
   {
      draw_world_labels(scene, window);
      scenes->draw_ux(window);
   }

   // F1 hides the whole panel : the scene plays unobstructed
   if (window->keys.F1.is_pressed && !window->keys.F1.was_pressed) ux_panel_visible = !ux_panel_visible;
   if (!ux_panel_visible) return;

   // full-height side slab : top to bottom of the window, width scaled to it,
   // scrollable (the Assets tab overflows a short window)
   ImGuiWindowFlags flags = ImGuiWindowFlags_NoCollapse
      | ImGuiWindowFlags_NoResize
      | ImGuiWindowFlags_NoMove
      | ImGuiWindowFlags_NoTitleBar;

   float panel_w = ux_panel_width(window); // shared with the scene's projection:
                                           // never eats the whole scene on narrow windows

   ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.f); // edge to edge slab
   ImGui::SetNextWindowPos(ImVec2(0, 0));
   ImGui::SetNextWindowSize(ImVec2(panel_w, (float)window->screen_height));
   ImGui::Begin("GameConsole", NULL, flags);

   // header band : identity + the number everyone watches + the hide hint
   ImGui::TextColored(ImVec4(0.90f, 0.72f, 0.44f, 1.f), "ENGINE CONSOLE");
   if (window->frame_ms_last > 0)
   {
      ImGui::SameLine();
      ImGui::TextColored(severity_colors[TRACE], "| %.0f fps  %.1f ms  |  F1 hide",
         1000.f / window->frame_ms_last, window->frame_ms_last);
   }
   ImGui::Separator();

   // general -> specific -> the log
   // nothing runs past the panel edge : overlong lines wrap instead of clipping
   ImGui::PushTextWrapPos(ImGui::GetWindowWidth() - ui(16));

   ImGui::BeginTabBar("Console Tabs", ImGuiTabBarFlags_None);
   draw_SYSTEM_console(window);
   draw_SCENES_console(scenes);
   draw_RENDER_console(rndr, window, &scene->camera);
   draw_DEBUG_console(rndr, scene);
   draw_PHYS_console(&scene->phys);
   draw_PLAYER_console(scene);
   draw_ASST_console(window, rndr);
   draw_AUDIO_console();
   draw_MAIN_console(console);
   ImGui::EndTabBar();
   ImGui::PopTextWrapPos();

   ImGui::End();
   ImGui::PopStyleVar(); // window rounding
}