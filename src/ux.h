#include "scene.h"

/* ------------------------- *
   ---  User Xperience   ---
 * ------------------------- */

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

   static const char* gbuf_labels[4] = {
      "position [RGBA32F]  pos+metallic",
      "normal   [RGBA16F]  nrm+roughness",
      "albedo   [RGBA8]    color+AO",
      "depth    [DEPTH24]",
   };
   GLuint gbuf_tex[4] = {
      window->gbuf.rendertarget.position_tx,
      window->gbuf.rendertarget.normal_tx,
      window->gbuf.rendertarget.albedo_tx,
      window->gbuf.rendertarget.depth_tx,
   };

   static int gbuf_pick = 0;
   ImGui::SetNextItemWidth(ui(300));
   ImGui::Combo("target", &gbuf_pick, gbuf_labels, 4);

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
      ImVec4(0.65f, 0.76f, 0.87f, 1), // 1  static + uv    : dusty blue
      ImVec4(0.56f, 0.66f, 0.78f, 1), // 3  static, no uv  : deeper blue
      ImVec4(0.75f, 0.68f, 0.85f, 1), // 2  skinned + uv   : muted lilac
      ImVec4(0.68f, 0.60f, 0.80f, 1), // 4  skinned, no uv : deeper lilac
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
   uint disk = 0, clip_ram = 0, tex_ram = 0;

   for (uint i = 0; i < MAX_MESHES; i++)
   {
      if (rndr->meshloader.meshes[i].size      != (uint)-1) disk += rndr->meshloader.meshes[i].size;
      if (rndr->meshloader_anim.meshes[i].size != (uint)-1) disk += rndr->meshloader_anim.meshes[i].size;
   }

   for (uint i = 1; i < MAX_ANIMATIONS; i++) // slot 0 = no clip
      if (rndr->animloader.clip_paths[i][0])
         clip_ram += rndr->animloader.clips[i].num_bones * rndr->animloader.clips[i].num_frames * (uint)sizeof(mat4);

   for (uint i = 0; i < rndr->txloader.num_textures; i++)
      tex_ram += rndr->txloader.textures[i].width * rndr->txloader.textures[i].height * 3; // decoded to rgb

   kv("mesh files"    , "%.1f KB on disk", disk / 1024.f);
   kv("clip keyframes", "%.1f KB in ram", clip_ram / 1024.f);
   kv("textures"      , "~%.1f KB decoded", tex_ram / 1024.f);
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

      // joint kinds : hinge, cone twist, hinge2 (the car's wheels)
      static const char* joint_names[] = { "none", "hinge", "cone", "hinge2" };
      static const ImVec4 joint_colors[] = {
         ImVec4(1, 1, 1, 1),
         ImVec4(0.70f, 0.76f, 0.88f, 1), // hinge  : pale slate
         ImVec4(0.80f, 0.70f, 0.86f, 1), // cone   : soft mauve
         ImVec4(0.86f, 0.78f, 0.56f, 1), // hinge2 : soft ochre
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

      // vehicle : live numbers read straight off the hinges (only when built)
      if (phys->vehicle.chassis_id)
      {
         section("Vehicle");
         float speed = phys->vehicle_speed();
         kv("speed", "%.2f m/s  |  %.1f km/h", speed, speed * 3.6f);
         kv("steer", "%.1f deg", phys->vehicle_steer() * 180.f / PI);
         kv("wheel", "%.2f rev", phys->vehicle_spin() / (2.f * PI));
         kv("arrows", "spin %.0f | steer %.0f", phys->vehicle.spin_cmd, phys->vehicle.steer_cmd);

         if (ImGui::BeginTable("physwheels", 4,
            TABLE_FLAGS))
         {
            ImGui::TableSetupColumn("axle",      ImGuiTableColumnFlags_WidthFixed, ui(56));
            ImGui::TableSetupColumn("id",        ImGuiTableColumnFlags_WidthFixed, ui(28));
            ImGui::TableSetupColumn("spin rev",  ImGuiTableColumnFlags_WidthFixed, ui(66));
            ImGui::TableSetupColumn("steer deg", ImGuiTableColumnFlags_WidthFixed, ui(66));
            ImGui::TableHeadersRow();

            for (uint i = 0; i < 4; i++)
            {
               btHinge2Constraint* h = phys->vehicle.hinges[i];
               ImGui::TableNextRow();
               ImGui::TableNextColumn(); ImGui::Text("%s %u", i < 2 ? "front" : "rear", i % 2 + 1);
               ImGui::TableNextColumn(); ImGui::Text("%u", phys->vehicle.wheel_ids[i]);
               ImGui::TableNextColumn(); ImGui::Text("%.2f", h->getAngle2() / (2.f * PI));
               ImGui::TableNextColumn();
               if (i < 2) ImGui::Text("%.1f", h->getAngle1() * 180.f / PI);
               else       ImGui::TextDisabled("-"); // rear axle : no steering motor
            }
            ImGui::EndTable();
         }
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
            if (jt.kind == PHYS_CONETWIST)   ImGui::Text("swing %.0f | twist %.0f", jt.lo * to_deg, jt.hi * to_deg);
            else if (jt.kind == PHYS_HINGE2) ImGui::Text("steer +-%.0f", jt.hi * to_deg);
            else                             ImGui::Text("%.0f .. %.0f", jt.lo * to_deg, jt.hi * to_deg);
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
      ImGui::SameLine();
      ImGui::Checkbox("collider wireframes", &phys->debug.enabled);

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
      // both mesh loaders and draw buffers share field names, so one table covers both
      auto mesh_table = [](const char* table_id, auto* meshes, auto* db, uint vert_bytes, TextureLoader* tx)
      {
         if (!ImGui::BeginTable(table_id, 7,
            TABLE_FLAGS))
            return;

         ImGui::TableSetupColumn("id"  , ImGuiTableColumnFlags_WidthFixed, ui(30));
         ImGui::TableSetupColumn("file", ImGuiTableColumnFlags_WidthFixed, ui(190));
         ImGui::TableSetupColumn("disk", ImGuiTableColumnFlags_WidthFixed, ui(54));
         ImGui::TableSetupColumn("verts", ImGuiTableColumnFlags_WidthFixed, ui(58));
         ImGui::TableSetupColumn("tris",  ImGuiTableColumnFlags_WidthFixed, ui(54));
         ImGui::TableSetupColumn("gpu" , ImGuiTableColumnFlags_WidthFixed, ui(54));
         ImGui::TableSetupColumn("texture", ImGuiTableColumnFlags_WidthFixed, ui(100));
         ImGui::TableHeadersRow();

         // texture id -> file name only (the loader holds the full path)
         auto tx_name = [](const char* path)
         {
            const char* slash = strrchr(path, '/');
            return slash ? slash + 1 : path;
         };

         for (uint m = 0; m < MAX_MESHES; m++)
         {
            if (meshes[m].id == 0) continue;

            // disk size (get_file_size returns (uint)-1 on failure)
            char disk[24];
            if (meshes[m].size == (uint)-1)
               snprintf(disk, sizeof(disk), "?");
            else
               snprintf(disk, sizeof(disk), "%.1f KB", meshes[m].size / 1024.f);

            // find this mesh's slot in the draw buffer (holds the vertex counts)
            bool in_gpu_buffer = false;

            for (uint i = 0; i < MAX_MESHES; i++)
            {
               if (db->mesh_info[i].mesh_id != meshes[m].id) continue;
               in_gpu_buffer = true;

               uint gpu_bytes = (db->mesh_info[i].num_vertices * vert_bytes)
                              + (db->mesh_info[i].num_indices  * sizeof(uint));

               uint tx_id = db->mesh_info[i].tx_id;

               ImGui::TableNextRow();
               ImGui::TableNextColumn(); ImGui::Text("%u", meshes[m].id);
               ImGui::TableNextColumn(); ImGui::Text("%s", meshes[m].filepath);
               ImGui::TableNextColumn(); ImGui::Text("%s", disk);
               ImGui::TableNextColumn(); ImGui::Text("%u", db->mesh_info[i].num_vertices);
               ImGui::TableNextColumn(); ImGui::Text("%u", db->mesh_info[i].num_indices / 3);
               ImGui::TableNextColumn(); ImGui::Text("%.1f KB", gpu_bytes / 1024.f);
               ImGui::TableNextColumn();
               if (tx_id < tx->num_textures)
                  ImGui::Text("%u : %s", tx_id, tx_name(tx->textures[tx_id].path));
               else
                  ImGui::TextColored(severity_colors[WARNING], "%u : missing", tx_id);
               break;
            }

            // id reserved but nothing in the buffer = load failed (reason is in Console tab)
            if (!in_gpu_buffer)
            {
               ImGui::TableNextRow();
               ImGui::TableNextColumn(); ImGui::Text("%u", meshes[m].id);
               ImGui::TableNextColumn();
               ImGui::TextColored(severity_colors[FIXME], "%s  (NOT IN GPU BUFFER)", meshes[m].filepath);
               ImGui::TableNextColumn(); ImGui::Text("%s", disk);
               ImGui::TableNextColumn(); ImGui::TextColored(severity_colors[FIXME], "-");
               ImGui::TableNextColumn(); ImGui::TextColored(severity_colors[FIXME], "-");
               ImGui::TableNextColumn(); ImGui::TextColored(severity_colors[FIXME], "-");
               ImGui::TableNextColumn(); ImGui::TextColored(severity_colors[FIXME], "-");
            }
         }
         ImGui::EndTable();
      };

      // ---- meshes : one layout at a time. the picker is the drawer mode, which also
      // makes the no-uv buffers visible here instead of nowhere
      section("Meshes");

      // the table id has to differ per layout : imgui keys its state on that string
      static const char* mesh_table_ids[DRAW_MODE_COUNT] = {
         "meshtable_1", "meshtable_3", "meshtable_2", "meshtable_4",
      };

      static int mesh_layout = MODE_STATIC_UV;
      const char* layouts[DRAW_MODE_COUNT];
      for (uint m = 0; m < DRAW_MODE_COUNT; m++) layouts[m] = DRAW_MODE_INFO[m].name;

      ImGui::SetNextItemWidth(ui(300));
      ImGui::Combo("layout", &mesh_layout, layouts, DRAW_MODE_COUNT);

      // the skinned modes read the anim loader's meshes, the static ones the mesh loader's.
      // the two loaders' rows are different types, so this stays a branch, not a ternary
      DrawBuffer& db          = rndr->buffer((uint)mesh_layout);
      const char* table_id    = mesh_table_ids[mesh_layout];
      uint        vertex_size = DRAW_MODE_INFO[mesh_layout].vertex_size;

      if (DRAW_MODE_INFO[mesh_layout].skinned)
         mesh_table(table_id, rndr->meshloader_anim.meshes, &db, vertex_size, &rndr->txloader);
      else
         mesh_table(table_id, rndr->meshloader.meshes,      &db, vertex_size, &rndr->txloader);

      // ---- animation clips : skeleton (parents + inverse-bind) + keyframes per loaded slot
      section("Animations");
      {
         AnimLoader* anim = &rndr->animloader;

         // widest skeleton any loaded clip uses : that is what the pose ubo has to hold
         uint bones = 0;
         for (uint i = 1; i < MAX_ANIMATIONS; i++) // slot 0 = no clip
            if (anim->clip_paths[i][0] && anim->clips[i].num_bones > bones)
               bones = anim->clips[i].num_bones;

         ImGui::Text("widest skeleton : %u bones / %u max | pose ubo : %u bytes (binding 1)",
            bones, MAX_ANIM_BONES, (uint)(MAX_ANIM_BONES * sizeof(mat4)));

         if (ImGui::BeginTable("cliptable", 6,
            TABLE_FLAGS))
         {
            ImGui::TableSetupColumn("slot"  , ImGuiTableColumnFlags_WidthFixed, ui(36));
            ImGui::TableSetupColumn("file"  , ImGuiTableColumnFlags_WidthFixed, ui(190));
            ImGui::TableSetupColumn("bones" , ImGuiTableColumnFlags_WidthFixed, ui(48));
            ImGui::TableSetupColumn("frames", ImGuiTableColumnFlags_WidthFixed, ui(52));
            ImGui::TableSetupColumn("holds" , ImGuiTableColumnFlags_WidthFixed, ui(48));
            ImGui::TableSetupColumn("ram"   , ImGuiTableColumnFlags_WidthFixed, ui(64));
            ImGui::TableHeadersRow();

            for (uint i = 1; i < MAX_ANIMATIONS; i++) // slot 0 = no clip
            {
               Animation* clip = &anim->clips[i];
               if (anim->clip_paths[i][0] == 0) continue; // free slot

               // keyframes + inverse-bind matrices + parent list
               uint ram_bytes = clip->num_bones * clip->num_frames * (uint)sizeof(mat4)
                              + MAX_ANIM_BONES * (uint)sizeof(mat4)
                              + MAX_ANIM_BONES * (uint)sizeof(int);

               ImGui::TableNextRow();
               ImGui::TableNextColumn(); ImGui::Text("%u", i);
               ImGui::TableNextColumn(); ImGui::Text("%s", anim->clip_paths[i]);
               ImGui::TableNextColumn(); ImGui::Text("%u", clip->num_bones);
               ImGui::TableNextColumn(); ImGui::Text("%u", clip->num_frames);
               ImGui::TableNextColumn(); ImGui::Text("%u", clip->num_holds);
               ImGui::TableNextColumn(); ImGui::Text("%.1f KB", ram_bytes / 1024.f);
            }
            ImGui::EndTable();
         }
      }

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
         shader_row("skinned  ", "anim.vert/geom.frag", &rndr->shaders[1]);
         shader_row("lighting ", "gbuf.vert/frag", &window->gbuf.shader);
      }

      ImGui::EndTabItem();
   }
}

// ---------------- tab 4 : Anim ----------------
// an inspector for the skinned meshes the scene draws : pick one from the dropdown
// and everything under it belongs to that mesh - the player posing it, the clip that
// player runs (skeleton, segments and per-segment feel) and the playback controls.
// one player per clip in flight, so several meshes animate at once
void draw_ANIM_console(GameRenderer* rndr)
{
   using namespace CNSL;

   if (ImGui::BeginTabItem("Anim"))
   {
      AnimLoader* anim = &rndr->animloader;

      static const char* eases[] = { "linear", "smooth", "in-out cubic", "out cubic", "in-out quint" };
      static int  selected   = 0; // segment the editor below acts on
      static bool show_juice = false;
      static bool combine    = false;

      // the mesh loader keeps the path every id was loaded from. id 0 is every unused
      // loader slot, so it is never looked up, and a name is never empty : an empty
      // label in a combo is how garbage ends up on screen
      static char fallback[32];

      auto path_of = [&](uint mesh_id) -> const char*
      {
         if (mesh_id == 0) return "";
         for (uint i = 0; i < MAX_MESHES; i++)
            if (rndr->meshloader_anim.meshes[i].id == mesh_id)
               return rndr->meshloader_anim.meshes[i].filepath;
         return "";
      };
      auto name_of = [&](uint mesh_id) -> const char*
      {
         const char* path  = path_of(mesh_id);
         const char* slash = strrchr(path, '/');
         const char* name  = (slash ? slash + 1 : path);

         if (name[0]) return name;
         snprintf(fallback, sizeof(fallback), "mesh %u", mesh_id);
         return fallback;
      };
      auto clip_of = [&](uint clip_id) -> const char*
      {
         const char* path  = (clip_id < MAX_ANIMATIONS) ? anim->clip_paths[clip_id] : "";
         const char* slash = strrchr(path, '/');
         const char* name  = (slash ? slash + 1 : path);

         if (name[0]) return name;
         snprintf(fallback, sizeof(fallback), "clip %u", clip_id);
         return fallback;
      };

      // ---- the mesh : the dropdown the whole tab hangs off
      section("Mesh");

      // both skinned buffers : an entry past MAX_MESHES is the no-uv one, so a single
      // pick stays unambiguous without a second dropdown
      auto buf_of = [&](uint entry) -> DrawBuffer*
      {
         return &rndr->buffer((entry >= MAX_MESHES) ? MODE_ANIM_NOUV : MODE_ANIM_UV);
      };
      auto slot_of    = [](uint entry) { return (entry >= MAX_MESHES) ? entry - MAX_MESHES : entry; };
      auto mesh_id_at = [&](uint entry) { return buf_of(entry)->mesh_info[slot_of(entry)].mesh_id; };

      uint slots[MAX_MESHES * 2];
      uint num_slots = 0;

      DrawBuffer& anim_uv   = rndr->buffer(MODE_ANIM_UV);
      DrawBuffer& anim_nouv = rndr->buffer(MODE_ANIM_NOUV);

      for (uint i = 0; i < MAX_MESHES; i++)
         if (anim_uv.mesh_info[i].mesh_id)
            slots[num_slots++] = i;

      for (uint i = 0; i < MAX_MESHES; i++)
         if (anim_nouv.mesh_info[i].mesh_id)
            slots[num_slots++] = i + MAX_MESHES;

      if (num_slots == 0)
      {
         ImGui::TextColored(severity_colors[FIXME], "no animated mesh in the scene");
         ImGui::EndTabItem();
         return;
      }

      static uint pick = (uint)-1; // an entry : it never moves, unlike a list index
      bool valid = false;
      for (uint i = 0; i < num_slots; i++)
         if (slots[i] == pick) valid = true;
      if (!valid) pick = slots[0];

      auto* mi = &buf_of(pick)->mesh_info[slot_of(pick)];

      ImGui::SetNextItemWidth(ui(300));
      if (ImGui::BeginCombo("mesh", name_of(mi->mesh_id)))
      {
         for (uint i = 0; i < num_slots; i++)
         {
            uint mesh_id = mesh_id_at(slots[i]);
            bool nouv    = (slots[i] >= MAX_MESHES);

            char label[96];
            snprintf(label, sizeof(label), "%u : %.40s%s##amesh%u",
               mesh_id, name_of(mesh_id), nouv ? "  (no uv)" : "", slots[i]);
            if (ImGui::Selectable(label, slots[i] == pick)) pick = slots[i];
            if (slots[i] == pick) ImGui::SetItemDefaultFocus();
         }
         ImGui::EndCombo();
      }

      kv("source", "%.44s", name_of(mi->mesh_id));
      kv("geometry", "%u verts | %u tris | %u instances",
         mi->num_vertices, mi->num_indices / 3, mi->num_instances);
      kv("texture", "id %u", mi->tx_id);

      // ---- the player posing it : writing here is what update() reads next frame
      uint player = mi->anim_player;
      bool posed  = player < anim->num_players;

      ImGui::SetNextItemWidth(ui(300));
      if (ImGui::BeginCombo("posed by", posed ? clip_of(anim->players[player].clip_id)
                                             : "none - identity pose"))
      {
         if (ImGui::Selectable("none - identity pose", !posed))
            mi->anim_player = (uint)-1;

         for (uint i = 0; i < anim->num_players; i++)
         {
            char label[80];
            snprintf(label, sizeof(label), "%u : %.40s##aply%u",
               i, clip_of(anim->players[i].clip_id), i);
            if (ImGui::Selectable(label, posed && i == player))
               mi->anim_player = i;
            if (posed && i == player) ImGui::SetItemDefaultFocus();
         }
         ImGui::EndCombo();
      }

      // ---- the rig on screen : the animator's own overlay pass. it follows whatever
      // the dropdown is showing, so it is always the mesh being inspected
      section("Skeleton");
      ImGui::Checkbox("skeleton", &rndr->animdebug.enabled);
      ImGui::SameLine();
      ImGui::Checkbox("bones", &rndr->animdebug.bones);
      ImGui::SameLine();
      ImGui::Checkbox("parent links", &rndr->animdebug.links);
      ImGui::SameLine();
      ImGui::Checkbox("joint axes", &rndr->animdebug.axes);

      if (rndr->animdebug.enabled)
      {
         rndr->animdebug.slot  = slot_of(pick); // the overlay takes a slot plus its buffer
         rndr->animdebug.no_uv = (pick >= MAX_MESHES);
         kv("overlay", "mesh %u, in world space", mi->mesh_id);

         // what the tint means : one colour per bone, in index order
         ImGui::TextDisabled("tint");
         for (uint t = 0; t < SKELETON_TIERS; t++)
         {
            ImGui::SameLine();
            vec3 c = SKELETON_TIER_COLOR[t];
            ImGui::TextColored(ImVec4(c.r, c.g, c.b, 1.f),
               (t + 1 == SKELETON_TIERS) ? "%u+" : "%u", t);
         }

         // the wiring in words. these rigs are flat - one root, every part hanging off
         // it - so this, not a chain depth, is what says which bone moves which
         if (posed)
         {
            Animation* bones_of = &anim->clips[anim->players[player].clip_id];
            ImGui::TextDisabled("bones");
            for (uint b = 0; b < bones_of->num_bones; b++)
            {
               int  p = bones_of->parents[b];
               vec3 c = SKELETON_TIER_COLOR[(b < SKELETON_TIERS) ? b : SKELETON_TIERS - 1];

               ImGui::SameLine();
               ImGui::TextColored(ImVec4(c.r, c.g, c.b, 1.f),
                  (p < 0 || p == (int)b) ? "%u root" : "%u<-%d", b, p);
            }
         }
      }

      if (!posed)
      {
         ImGui::TextDisabled("nothing bound : this mesh draws its identity pose");
      }
      else
      {
         Anim_Player* pl = &anim->players[player];
         Animation* clip = &anim->clips[pl->clip_id];
         if (selected >= (int)clip->num_segs) selected = (int)clip->num_segs - 1; // the pick can swap to a shorter clip

         // ---- playback : the timeline, the transport and the clock
         section("Playback");

         const Anim_Segment& pseg = clip->segs[pl->state.seg]; // the playing one

         // timeline + scrub : drag to inspect the playing segment
         if (!pl->control.scrubbing)
            pl->control.scrub_t = (pl->state.duration > 0) ? pl->state.elapsed / pl->state.duration : 0.f;
         ImGui::SliderFloat("##scrub", &pl->control.scrub_t, 0.f, 1.f, "%.3f");
         pl->control.scrubbing = ImGui::IsItemActive();

         // green bands = rotation holds of this segment found at load
         ImDrawList* draw_list = ImGui::GetWindowDrawList();
         ImVec2 r0 = ImGui::GetItemRectMin(), r1 = ImGui::GetItemRectMax();
         float span = (float)(pseg.last - pseg.first + 1);
         for (uint h = 0; h < clip->num_holds; h++)
         {
            const Anim_Hold& hold = clip->holds[h];
            if (hold.last < pseg.first || hold.first > pseg.last) continue; // another segment's rest
            uint b0 = (hold.first > pseg.first) ? hold.first : pseg.first; // clamp into the range
            uint b1 = (hold.last < pseg.last) ? hold.last : pseg.last;
            float x0 = r0.x + ((b0 - pseg.first) / span) * (r1.x - r0.x);
            float x1 = r0.x + (((b1 + 1) - pseg.first) / span) * (r1.x - r0.x);
            draw_list->AddRectFilled(ImVec2(x0, r0.y), ImVec2(x1, r1.y), IM_COL32(118, 172, 124, 78));
            draw_list->AddLine(ImVec2(x0, r0.y), ImVec2(x0, r1.y), IM_COL32(150, 198, 150, 215), ui(1.f));
         }

         // transport : one dropdown, and its own label is the playback state
         const char* play_state = pl->control.paused ? "paused"
            : pl->state.playing ? "playing"
            : (pl->state.duration > 0 ? "finished" : "static frame 0");

         ImGui::SetNextItemWidth(ui(170));
         if (ImGui::BeginCombo("playback", play_state))
         {
            if (ImGui::Selectable("play", !pl->control.paused))
            {
               pl->control.paused = false;
               if (!pl->state.playing && pl->state.duration > 0)
                  pl->control.restart = true; // finished : play means start over
            }
            if (ImGui::Selectable("pause", pl->control.paused))
               pl->control.paused = true;
            if (ImGui::Selectable("restart"))
               pl->control.restart = true;
            ImGui::EndCombo();
         }

         kv("clock", "%.2f / %.2f s", pl->state.elapsed, pl->state.duration);
         kv("frame", "%.2f / %u", pl->state.frame_f, clip->num_frames);
         kv("segment", "%u / %u", pl->state.seg, clip->num_segs);
         kv("seam", "%.2f s blend where the loop wraps", anim->seam_duration);

         ImGui::SliderFloat("speed", &pl->control.speed, 0.1f, 2.f, "%.2fx");
         ImGui::Checkbox("loop", &pl->control.loop);

         // ---- the clip : several animations live in one file as frame ranges
         section("Clip");
         kv("file", "%.44s", clip_of(pl->clip_id));
         kv("skeleton", "%u bones | %u frames", clip->num_bones, clip->num_frames);
         kv("holds", "%u rotation rests | %u segments", clip->num_holds, clip->num_segs);
         kv("length", "%.2f s : set by play(), scaled by speed", pl->state.duration);

         for (uint s = 0; s < clip->num_segs; s++)
         {
            Anim_Segment& sg = clip->segs[s];
            char label[80];
            snprintf(label, sizeof(label), "%s%u : frames %u-%u | %s | juice %.2f",
               (s == pl->state.seg) ? "> " : "  ", s, sg.first, sg.last, eases[sg.ease], sg.juice);
            if (ImGui::Selectable(label, selected == (int)s)) selected = s;
         }

         Anim_Segment& seg = clip->segs[selected];
         int range[2] = { (int)seg.first, (int)seg.last };
         if (ImGui::SliderInt2("frames", range, 0, (int)clip->num_frames - 1))
         {
            if (range[0] > range[1]) range[1] = range[0]; // keep first <= last
            seg.first = range[0]; seg.last = range[1];
         }

         if (ImGui::Button("play"))
            anim->play(pl->clip_id, (uint)selected, pl->duration);
         ImGui::SameLine();
         if (ImGui::Button("add") && clip->num_segs < MAX_ANIM_SEGMENTS)
         {
            uint mid = (seg.first + seg.last) / 2;
            if (mid > seg.first && mid < seg.last)
            {
               for (uint i = clip->num_segs; i > (uint)selected + 1; i--)
                  clip->segs[i] = clip->segs[i - 1];
               Anim_Segment tail = seg;
               seg.last = mid;
               tail.first = mid + 1;
               clip->segs[selected + 1] = tail;
               clip->num_segs++;
            }
         }
         ImGui::SameLine();
         if (ImGui::Button("remove") && clip->num_segs > 1)
         {
            for (uint i = selected; i + 1 < clip->num_segs; i++)
               clip->segs[i] = clip->segs[i + 1];
            clip->num_segs--;
            if (selected >= (int)clip->num_segs) selected--;
         }
         ImGui::SameLine();
         if (ImGui::Button("split at holds"))
         {
            uint n = 0, start = 0;
            for (uint h = 0; h < clip->num_holds && n < MAX_ANIM_SEGMENTS; h++)
            {
               if (clip->holds[h].last < start) continue;
               clip->segs[n].first = start;               // each hold's rest tail stays
               clip->segs[n].last  = clip->holds[h].last; // with the animation before it
               clip->segs[n].ease  = 0;
               clip->segs[n].juice = 0.4f;
               start = clip->holds[h].last + 1;
               n++;
            }
            if (start < clip->num_frames && n < MAX_ANIM_SEGMENTS)
            {
               clip->segs[n].first = start;
               clip->segs[n].last  = clip->num_frames - 1;
               clip->segs[n].ease  = 0;
               clip->segs[n].juice = 0.4f;
               n++;
            }
            if (n) clip->num_segs = n; // 0 = the whole clip was one hold : keep as-is
            if (selected >= (int)clip->num_segs) selected = clip->num_segs - 1;
         }

         // ---- the selected segment : frame range, feel and what both do to the clock
         section("Segment");
         ImGui::Text("segment %d of %u", selected, clip->num_segs);
         ImGui::Combo("ease", &clip->segs[selected].ease, eases, 5);
         ImGui::SliderFloat("juice", &clip->segs[selected].juice, 0.f, 1.f, "%.2f");

         // time warp graph : clock (x) -> clip position (y), all methods at once.
         // bright = selected segment's ease, dot = live playback head.
         // juice layer = the spring chasing a ramp, same constants as apply_spring ;
         // combine = the spring chases the selected ease instead, so you see both together
         {
            const int N = 48;
            int ease = clip->segs[selected].ease;
            ImVec2 p0 = ImGui::GetCursorScreenPos();
            ImVec2 size(ui(240), ui(140)), p1(p0.x + size.x, p0.y + size.y);
            ImDrawList* dl = ImGui::GetWindowDrawList();

            dl->AddRectFilled(p0, p1, IM_COL32(8, 10, 15, 255)); // plot bg
            for (int q = 1; q < 4; q++) // quarter grid
            {
               dl->AddLine(ImVec2(p0.x + size.x * q / 4, p0.y), ImVec2(p0.x + size.x * q / 4, p1.y),
                  IM_COL32(255, 255, 255, 14));
               dl->AddLine(ImVec2(p0.x, p0.y + size.y * q / 4), ImVec2(p1.x, p0.y + size.y * q / 4),
                  IM_COL32(255, 255, 255, 14));
            }
            dl->AddLine(p0, p1, IM_COL32(255, 255, 255, 40)); // linear reference
            dl->AddRect(p0, p1, IM_COL32(255, 255, 255, 70));

            for (int m = 0; m < 5; m++)
            {
               ImVec2 pts[51];
               for (int s = 0; s <= N; s++)
               {
                  float u = s / (float)N;
                  float w = anim->time_warp(u, m);
                  pts[s] = ImVec2(p0.x + u * size.x, p1.y - w * size.y);
               }
               bool active = (m == ease);
               dl->AddPolyline(pts, N + 1,
                  active ? IM_COL32(150, 172, 205, 255) : IM_COL32(158, 152, 142, 90),
                  active ? ui(2.f) : ui(1.f), true);
            }

            // juice layer : spring response over a 2s window (ramps 0..1s, then settles)
            if (show_juice)
            {
               float w = clip->segs[selected].juice;
               if (w > 0.01f)
               {
                  float k    = 60.f + 740.f * w * w; // same constants as apply_spring
                  float zeta = 0.85f - 0.35f * w;
                  float c    = 2.f * zeta * sqrtf(k);
                  float pos = 0, vel = 0, sdt = 1.f / 60;

                  ImVec2 pts[121];
                  for (int i = 0; i <= 120; i++)
                  {
                     float t = i * sdt;                    // 0..2s
                     float ramp = (t < 1.f) ? t : 1.f;     // ramps over 1s, then holds
                     float target = combine ? anim->time_warp(ramp, ease) : ramp;
                     vel += ((target - pos) * k - vel * c) * sdt;
                     pos += vel * sdt;
                     pts[i] = ImVec2(p0.x + (t / 2.f) * size.x, p1.y - pos * size.y);
                  }
                  dl->AddPolyline(pts, 121, IM_COL32(218, 158, 190, 205), ui(1.5f), false);
               }
            }

            // live playback dot on the selected ease
            float raw = (pl->state.duration > 0) ? pl->state.elapsed / pl->state.duration : 0.f;
            ImVec2 dot(p0.x + raw * size.x, p1.y - anim->time_warp(raw, ease) * size.y);
            dl->AddLine(ImVec2(dot.x, p0.y), ImVec2(dot.x, p1.y), IM_COL32(225, 200, 130, 75), ui(1.f));
            dl->AddCircleFilled(dot, ui(3.f), IM_COL32(225, 200, 130, 255));

            ImGui::Dummy(size);
            ImGui::Checkbox("juice layer", &show_juice);
            ImGui::SameLine();
            ImGui::Checkbox("combine with ease", &combine);
            ImGui::Text("x : clock    y : clip position    dot = playback head");
         }

      }

      ImGui::EndTabItem();
   }
}

// ---------------- tab : Player ----------------
void draw_PLAYER_console(GameScene* scene, GameRenderer* rndr)
{
   using namespace CNSL;

   Player* player = &scene->player;
   static const char* action_names[] = { "reload", "shoot", "inspect", "idle" };

   if (ImGui::BeginTabItem("Player"))
   {
      section("View");
      kv("mode", player->flycam ? "fly camera (F)" : "capsule (F to fly)");
      kv("position", "%.2f %.2f %.2f", player->camera.position.x, player->camera.position.y,
         player->camera.position.z);
      kv("grounded / bob", "%s / %.2f", player->grounded ? "yes" : "no", player->bob);
      kv("fov punch", "%.2f deg", player->fov_punch);

      section("View model");
      uint action = (player->action <= 3) ? player->action : 3; // defensive index
      kv("action", "%s", action_names[action]);
      kv("time left", "%.2f s", player->action_time);
      kv("ammo", "%u / %u", player->ammo, GUN_MAG);
      kv("shots", "%u fired, %u hit", player->fired, player->hits);
      kv("mesh / player", "%u / %u", player->gun_mesh, player->gun_player);
      kv("sway", "%.3f %.3f", player->sway.x, player->sway.y);
      kv("kick", "%.2f %.2f %.2f", player->kick.x, player->kick.y, player->kick.z);

      // drive the gun from here too : handy without grabbing the mouse
      if (ImGui::SmallButton("fire"))    player->fire(rndr, &scene->phys);
      ImGui::SameLine();
      if (ImGui::SmallButton("reload"))  player->start_action(rndr, GUN_RELOAD, GUN_RELOAD_TIME);
      ImGui::SameLine();
      if (ImGui::SmallButton("inspect")) player->start_action(rndr, GUN_INSPECT, GUN_INSPECT_TIME);

      section("Bullets");
      uint live_rounds = player->bullets_alive();
      kv("alive", "%u / %u", live_rounds, MAX_BULLETS);

      if (live_rounds == 0)
         ImGui::TextDisabled("none in flight");
      else if (ImGui::BeginTable("playerbullets", 7,
         TABLE_FLAGS))
      {
         ImGui::TableSetupColumn("#",     ImGuiTableColumnFlags_WidthFixed, ui(24));
         ImGui::TableSetupColumn("pos x", ImGuiTableColumnFlags_WidthFixed, ui(52));
         ImGui::TableSetupColumn("pos y", ImGuiTableColumnFlags_WidthFixed, ui(52));
         ImGui::TableSetupColumn("pos z", ImGuiTableColumnFlags_WidthFixed, ui(52));
         ImGui::TableSetupColumn("speed", ImGuiTableColumnFlags_WidthFixed, ui(56));
         ImGui::TableSetupColumn("life",  ImGuiTableColumnFlags_WidthFixed, ui(44));
         ImGui::TableSetupColumn("state", ImGuiTableColumnFlags_WidthFixed, ui(50));
         ImGui::TableHeadersRow();

         for (uint i = 0; i < MAX_BULLETS; i++)
         {
            Player::Bullet& b = player->bullets[i];
            if (b.life <= 0) continue;

            ImGui::TableNextRow();
            ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg0, IM_COL32(230, 184, 112, 18)); // live rounds glow

            ImGui::TableNextColumn(); ImGui::Text("%u", i);
            ImGui::TableNextColumn(); ImGui::Text("%.2f", b.pos.x);
            ImGui::TableNextColumn(); ImGui::Text("%.2f", b.pos.y);
            ImGui::TableNextColumn(); ImGui::Text("%.2f", b.pos.z);
            ImGui::TableNextColumn(); ImGui::Text("%.1f", length(b.vel));
            ImGui::TableNextColumn(); ImGui::Text("%.2f", b.life);
            ImGui::TableNextColumn(); ImGui::TextColored(severity_colors[SUCCESS], "flying");
         }
         ImGui::EndTable();
      }

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
void draw_world_labels(GameScene* scene, GameWindow* window)
{
   ImDrawList* dl = ImGui::GetBackgroundDrawList(); // over the frame, under the console

   for (uint i = 0; i < scene->num_props; i++)
   {
      GameScene::Prop& p = scene->props[i];
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

void draw_console(GameConsole* console, GameRenderer* rndr, GameWindow* window, GameScene* scene)
{
   using namespace CNSL;

   // the scene's annotations : drawn whether or not the panel is up
   draw_world_labels(scene, window);

   // F1 hides the whole panel : the scene plays unobstructed
   static bool visible = true;
   if (window->keys.F1.is_pressed && !window->keys.F1.was_pressed) visible = !visible;
   if (!visible) return;

   // full-height side slab : top to bottom of the window, width scaled to it,
   // scrollable (the Assets tab overflows a short window)
   ImGuiWindowFlags flags = ImGuiWindowFlags_NoCollapse
      | ImGuiWindowFlags_NoResize
      | ImGuiWindowFlags_NoMove
      | ImGuiWindowFlags_NoTitleBar;

   float panel_w = ui(620.f);
   float max_w = window->screen_width * 0.45f; // never eats the whole scene on narrow windows
   if (panel_w > max_w) panel_w = max_w;

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
   draw_RENDER_console(rndr, window, &scene->camera);
   draw_PHYS_console(&scene->phys);
   draw_PLAYER_console(scene, rndr);
   draw_ANIM_console(rndr);
   draw_ASST_console(window, rndr);
   draw_AUDIO_console();
   draw_MAIN_console(console);
   ImGui::EndTabBar();
   ImGui::PopTextWrapPos();

   ImGui::End();
   ImGui::PopStyleVar(); // window rounding
}