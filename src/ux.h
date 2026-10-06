#include "anim.h"

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

   const char* source_messages[4] = {
       "WNDW", // Window
       "RNDR", // Renderer
       "PHYS", // Physics
       "NETW"  // Networking
   };

   // Source colors  modern, balanced, and consistent brightness
   const ImVec4 source_colors[4] = {
       ImVec4(0.45f, 0.65f, 1.00f, 1.0f), // WNDW : Azure Blue
       ImVec4(0.75f, 0.60f, 1.00f, 1.0f), // RNDR : Vivid Violet
       ImVec4(1.00f, 0.55f, 0.45f, 1.0f), // PHYS : Coral Rust
       ImVec4(1.00f, 0.80f, 0.55f, 1.0f)  // NETW : Signal Amber
   };

   // Severity levels  distinct, modern color language
   const ImVec4 severity_colors[5] = {
       ImVec4(0.75f, 0.55f, 1.00f, 1.0f), // TRACE   : Purple (Insight)
       ImVec4(1.00f, 0.90f, 0.45f, 1.0f), // WARNING : Yellow (Caution)
       ImVec4(1.00f, 0.55f, 0.20f, 1.0f), // FIXME   : Orange (Problem)
       ImVec4(0.45f, 0.70f, 1.00f, 1.0f), // DEBUG   : Blue (Info)
       ImVec4(0.45f, 1.00f, 0.55f, 1.0f)  // SUCCESS : Green (OK)
   };

   // Message text  softened tints of above for legibility on dark backgrounds
   const ImVec4 message_colors[5] = {
       ImVec4(0.85f, 0.70f, 1.00f, 1.0f), // TRACE   : Soft Lavender
       ImVec4(1.00f, 0.95f, 0.70f, 1.0f), // WARNING : Pale Yellow
       ImVec4(1.00f, 0.75f, 0.50f, 1.0f), // FIXME   : Soft Orange
       ImVec4(0.70f, 0.85f, 1.00f, 1.0f), // DEBUG   : Light Blue
       ImVec4(0.70f, 1.00f, 0.75f, 1.0f)  // SUCCESS : Soft Green
   };
}

// ---------------- shared helpers ----------------

// orange section header (matches the accent color in apply_imgui_style)
static void section(const char* title)
{
   ImGui::Spacing();
   ImGui::TextColored(ImVec4(0.98f, 0.66f, 0.2f, 1.0f), "%s", title);
   ImGui::Separator();
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
   if (source > 3)   source   = 3;

   ImGui::TextColored(source_colors[source], "[%-4s]", source_messages[source]);
   ImGui::SameLine();
   ImGui::TextColored(severity_colors[severity], "[%s]", severity_messages[severity]);
   ImGui::SameLine();
   ImGui::TextColored(message_colors[severity], "[%s]", entry.text);
}

// ---------------- tab 1 : System ----------------
void draw_SYSTEM_console(GameConsole* console, GameWindow* window)
{
   using namespace CNSL;

   if (ImGui::BeginTabItem("System"))
   {
      section("GPU / Context");
      ImGui::Text("Renderer : %s", window->gl.renderer);
      ImGui::Text("Vendor   : %s", window->gl.vendor);
      ImGui::Text("Version  : %s", window->gl.version);
      ImGui::Text("GLSL     : %s", window->gl.glsl_version);
      ImGui::Text("Context  : OpenGL %d.%d core profile",
         window->gl.context_major, window->gl.context_minor);
      ImGui::Text("Window   : %d x %d (not resizable)", window->screen_width, window->screen_height);
      ImGui::Text("Frame cap: 120 fps software cap, vsync ON (swap interval 1)");

      ImGui::Text("GLEW     : ");
      ImGui::SameLine();
      if (window->gl.glew_ok)
         ImGui::TextColored(severity_colors[SUCCESS], "OK");
      else
         ImGui::TextColored(severity_colors[FIXME], "FAILED - extensions unavailable");

      section("GL Limits");
      ImGui::Text("Max texture size       : %d", window->gl.max_texture_size);
      ImGui::Text("Max vertex attributes  : %d", window->gl.max_vertex_attribs);
      ImGui::Text("Max uniform floats     : %d", window->gl.max_uniform_components);

      section("GL Errors");
      if (window->gl_errors.total == 0)
      {
         ImGui::TextColored(severity_colors[SUCCESS], "none - GL is happy");
      }
      else
      {
         ImGui::TextColored(severity_colors[FIXME], "%u total since launch", window->gl_errors.total);
         ImGui::SameLine();
         ImGui::TextColored(severity_colors[window->gl_errors.this_frame ? WARNING : TRACE],
            "(%u last frame)", window->gl_errors.this_frame);
         ImGui::Text("Last : 0x%04X (%s)", window->gl_errors.last_code,
            gl_error_name(window->gl_errors.last_code));
      }

      section("Audio");
      if (window->audio.ok)
         ImGui::Text("Device : %s", window->audio.device);
      else
         ImGui::TextColored(severity_colors[FIXME], "no audio device - sounds will not play");

      ImGui::EndTabItem();
   }
}

// ---------------- tab 2 : Render ----------------
void draw_RENDER_console(GameConsole* console, GameRenderer* rndr, GameWindow* window)
{
   using namespace CNSL;

   if (ImGui::BeginTabItem("Render"))
   {
      // ---- frame timing
      section("Frame");

      if (window->frame_ms_count > 0) // not very first frame
      {
         float fps = (window->frame_ms_last > 0) ? (1000.f / window->frame_ms_last) : 0;
         ImVec4 fps_color = (fps >= 60) ? severity_colors[SUCCESS]
            : (fps >= 30) ? severity_colors[WARNING]
            : severity_colors[FIXME];
         ImGui::TextColored(fps_color, "%.0f FPS", fps);
         ImGui::SameLine();
         ImGui::Text("| %.1f ms", window->frame_ms_last);
         ImGui::SameLine();
         ImGui::TextColored(severity_colors[TRACE], "(target 8.3 ms / 120 fps)");

         if (window->frame_ms_count > 1)
            ImGui::PlotLines("##framems", window->frame_ms, (int)window->frame_ms_count, 0,
               NULL, 0.f, 33.f, ImVec2(600, 60)); // WARNING : THIS ImVec2 HARDCODES SIZE IN PIXELS

         ImGui::Text("avg %.1f ms  |  min %.1f  |  max %.1f",
            window->frame_ms_avg, window->frame_ms_min, window->frame_ms_max);

         // how much of the frame budget was used (clamped: ImGui asserts fraction <= 1)
         float target = (float)window->frame_milliseconds_target;
         float used = (target > 0) ? (window->frame_ms_last / target) : 0;
         if (used > 1.f) used = 1.f;
         char budget[48];
         snprintf(budget, 48, "budget : %.1f / %.0f ms", window->frame_ms_last, target);
         ImGui::PushStyleColor(ImGuiCol_PlotHistogram,
            (used >= 1.f) ? ImVec4(1.f, 0.4f, 0.3f, 1.f) : ImVec4(0.4f, 0.75f, 1.f, 1.f));
         ImGui::ProgressBar(used, ImVec2(-1, 0), budget);
         ImGui::PopStyleColor();
         ImGui::Text("slept %.1f ms waiting for frame cap", window->spare_milliseconds);
      } // end of frame-stats block (needs at least 1 recorded frame)

      // ---- draw statistics (filled by GameRenderer::draw)
      section("Draw Stats (this frame)");

      ImGui::Text("draw calls : %u total", rndr->stats.draw_calls + 1); // +1 = lighting fullscreen quad
      ImGui::SameLine();
      ImGui::Text("(%u geometry + 1 lighting quad)", rndr->stats.draw_calls);

      ImGui::Text("meshes     : %u / %d", rndr->stats.meshes_drawn, MAX_MESHES);
      ImGui::SameLine();
      ImGui::Text("| instances : %u", rndr->stats.instances);

      ImGui::Text("triangles  : %u", rndr->stats.triangles);
      ImGui::SameLine();
      ImGui::Text("| vertices : %u", rndr->stats.vertices);

      // ---- per-mesh draw list : proof each append_instances() worked
      section("Mesh Draw List");
      if (ImGui::BeginTable("meshdraw", 6,
         ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_RowBg | ImGuiTableFlags_Borders))
      {
         ImGui::TableSetupColumn("id"      , ImGuiTableColumnFlags_WidthFixed, 30);
         ImGui::TableSetupColumn("indices" , ImGuiTableColumnFlags_WidthFixed, 58);
         ImGui::TableSetupColumn("insts"   , ImGuiTableColumnFlags_WidthFixed, 44);
         ImGui::TableSetupColumn("verts"   , ImGuiTableColumnFlags_WidthFixed, 58);
         ImGui::TableSetupColumn("base vtx", ImGuiTableColumnFlags_WidthFixed, 58);
         ImGui::TableSetupColumn("base idx", ImGuiTableColumnFlags_WidthFixed, 62);
         ImGui::TableHeadersRow();

         for (uint i = 0; i < MAX_MESHES; i++)
         {
            if (rndr->drawbuffer.mesh_info[i].mesh_id == 0) continue;

            ImGui::TableNextRow();
            ImGui::TableNextColumn(); ImGui::Text("%u", rndr->drawbuffer.mesh_info[i].mesh_id);
            ImGui::TableNextColumn(); ImGui::Text("%u", rndr->drawbuffer.mesh_info[i].num_indices);
            ImGui::TableNextColumn();
            if (rndr->drawbuffer.mesh_info[i].num_instances == 0)
               ImGui::TextColored(severity_colors[WARNING], "%u", 0u); // never submitted this frame
            else
               ImGui::Text("%u", rndr->drawbuffer.mesh_info[i].num_instances);
            ImGui::TableNextColumn(); ImGui::Text("%u", rndr->drawbuffer.mesh_info[i].num_vertices);
            ImGui::TableNextColumn(); ImGui::Text("%u", rndr->drawbuffer.mesh_info[i].base_vertex);
            ImGui::TableNextColumn(); ImGui::Text("%u", rndr->drawbuffer.mesh_info[i].base_index);
         }
         ImGui::EndTable();
      }

      // ---- gpu buffer health
      section("GPU Buffers");

      auto buffer_bar = [](const char* label, uint used, uint total)
      {
         float pct = total ? ((float)used / total) : 0;
         char overlay[64];
         snprintf(overlay, 64, "%s : %u / %u bytes (%.0f%%)", label, used, total, pct * 100);

         ImVec4 color = (pct > .8f) ? ImVec4(1.f, .4f, .3f, 1.f)
            : (pct > .5f) ? ImVec4(1.f, .8f, .3f, 1.f)
            : ImVec4(.4f, .75f, 1.f, 1.f);
         ImGui::PushStyleColor(ImGuiCol_PlotHistogram, color);
         ImGui::ProgressBar(pct, ImVec2(-1, 0), overlay);
         ImGui::PopStyleColor();
      };

      buffer_bar("geometry" , rndr->drawbuffer.geom_size, rndr->drawbuffer.max_buffer_size);
      buffer_bar("indices"  , rndr->drawbuffer.indx_size, rndr->drawbuffer.max_buffer_size);
      buffer_bar("instances", rndr->drawbuffer.inst_size, rndr->drawbuffer.max_buffer_size);

      // ---- g-buffer : deferred pipeline inspector
      section("G-Buffer");

      if (window->gbuf.rendertarget.complete)
         ImGui::TextColored(severity_colors[SUCCESS], "framebuffer : COMPLETE");
      else
         ImGui::TextColored(severity_colors[FIXME], "framebuffer : INCOMPLETE - nothing will render");
      ImGui::SameLine();
      ImGui::Text("| %u x %u", window->screen_width, window->screen_height);

      auto gbuf_image = [&](const char* label, GLuint texture)
      {
         ImGui::Text("%s", label);
         //ImGui::SameLine();
         // uv flipped because render-to-texture is upside down vs imgui
         ImGui::Image((ImTextureID)(intptr_t)texture, ImVec2(176, 99), ImVec2(0, 1), ImVec2(1, 0));
      };

      gbuf_image("positions  RGB = world pos, A = metallic  [RGBA32F] ", window->gbuf.rendertarget.position_tx);
      gbuf_image("normals    RGB = world nrm, A = roughness [RGBA16F] ", window->gbuf.rendertarget.normal_tx);
      gbuf_image("albedo     RGB = base col, A = AO         [RGBA8]   ", window->gbuf.rendertarget.albedo_tx);
      gbuf_image("depth                                   [DEPTH24] ", window->gbuf.rendertarget.depth_tx);

      // ---- camera & input
      section("Camera / Input");
      vec3 pos = rndr->camera.position;
      ImGui::Text("position : %6.2f %6.2f %6.2f", pos.x, pos.y, pos.z);
      ImGui::Text("yaw/pitch: %6.1f / %5.1f deg",
         ToDegrees(rndr->camera.yaw), ToDegrees(rndr->camera.pitch));
      ImGui::Text("mouse    : %.0f, %.0f   delta: %.1f, %.1f",
         window->mouse.raw_x, window->mouse.raw_y, window->mouse.dx, window->mouse.dy);
      ImGui::Text("buttons  : L[%s] R[%s]   keys: W[%d] A[%d] S[%d] D[%d]",
         window->mouse.left_button.is_pressed ? "down" : "up  ",
         window->mouse.right_button.is_pressed ? "down" : "up  ",
         window->keys.W.is_pressed, window->keys.A.is_pressed,
         window->keys.S.is_pressed, window->keys.D.is_pressed);

      ImGui::EndTabItem();
   }
}

// ---------------- tab 3 : Assets ----------------
void draw_ASST_console(GameConsole* console, GameWindow* window, GameRenderer* rndr)
{
   using namespace CNSL;

   if (ImGui::BeginTabItem("Assets"))
   {
      // ---- meshes : file info joined with what actually landed in the gpu buffer
      section("Meshes");
      if (ImGui::BeginTable("meshtable", 6,
         ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_RowBg | ImGuiTableFlags_Borders))
      {
         ImGui::TableSetupColumn("id",   ImGuiTableColumnFlags_WidthFixed, 30);
         ImGui::TableSetupColumn("file", ImGuiTableColumnFlags_WidthFixed, 190);
         ImGui::TableSetupColumn("disk", ImGuiTableColumnFlags_WidthFixed, 54);
         ImGui::TableSetupColumn("verts", ImGuiTableColumnFlags_WidthFixed, 58);
         ImGui::TableSetupColumn("tris",  ImGuiTableColumnFlags_WidthFixed, 54);
         ImGui::TableSetupColumn("gpu",   ImGuiTableColumnFlags_WidthFixed, 54);
         ImGui::TableHeadersRow();

         for (uint m = 0; m < MAX_MESHES; m++)
         {
            MeshLoader::MeshInfo* mesh = &rndr->meshloader.meshes[m];
            if (mesh->id == 0) continue;

            // disk size (get_file_size returns (uint)-1 on failure)
            char disk[24];
            if (mesh->size == (uint)-1)
               snprintf(disk, sizeof(disk), "?");
            else
               snprintf(disk, sizeof(disk), "%.1f KB", mesh->size / 1024.f);

            // find this mesh's slot in the draw buffer (holds the vertex counts)
            DrawBuffer* db = &rndr->drawbuffer;
            bool in_gpu_buffer = false;

            for (uint i = 0; i < MAX_MESHES; i++)
            {
               if (db->mesh_info[i].mesh_id != mesh->id) continue;
               in_gpu_buffer = true;

               uint gpu_bytes = (db->mesh_info[i].num_vertices * sizeof(MeshVertexData))
                              + (db->mesh_info[i].num_indices  * sizeof(uint));

               ImGui::TableNextRow();
               ImGui::TableNextColumn(); ImGui::Text("%u", mesh->id);
               ImGui::TableNextColumn(); ImGui::Text("%s", mesh->filepath);
               ImGui::TableNextColumn(); ImGui::Text("%s", disk);
               ImGui::TableNextColumn(); ImGui::Text("%u", db->mesh_info[i].num_vertices);
               ImGui::TableNextColumn(); ImGui::Text("%u", db->mesh_info[i].num_indices / 3);
               ImGui::TableNextColumn(); ImGui::Text("%.1f KB", gpu_bytes / 1024.f);
               break;
            }

            // id reserved but nothing in the buffer = load failed (reason is in Console tab)
            if (!in_gpu_buffer)
            {
               ImGui::TableNextRow();
               ImGui::TableNextColumn(); ImGui::Text("%u", mesh->id);
               ImGui::TableNextColumn();
               ImGui::TextColored(severity_colors[FIXME], "%s  (NOT IN GPU BUFFER)", mesh->filepath);
               ImGui::TableNextColumn(); ImGui::Text("%s", disk);
               ImGui::TableNextColumn(); ImGui::TextColored(severity_colors[FIXME], "-");
               ImGui::TableNextColumn(); ImGui::TextColored(severity_colors[FIXME], "-");
               ImGui::TableNextColumn(); ImGui::TextColored(severity_colors[FIXME], "-");
            }
         }
         ImGui::EndTable();
      }

      // ---- textures : TextureLoader storage, shown for the first time
      section("Textures");
      if (rndr->txloader.num_textures == 0)
      {
         ImGui::TextColored(severity_colors[WARNING], "no textures loaded");
      }
      for (uint i = 0; i < rndr->txloader.num_textures; i++)
      {
         Texture_Data* tx = &rndr->txloader.textures[i];
         ImGui::Image((ImTextureID)(intptr_t)tx->handle, ImVec2(48, 48), ImVec2(0, 1), ImVec2(1, 0));
         ImGui::SameLine();
         ImGui::Text("id %u : %s\n%dx%d, handle %u", tx->id, tx->path, tx->width, tx->height, tx->handle);
      }

      // ---- shaders : compile/link results queried from GL at startup
      section("Shaders");
      {
         ShaderProgram* geom = &rndr->shaders[0];
         if (geom->id)
         {
            ImGui::Text("geometry  (geom.vert/frag) : ");
            ImGui::SameLine();
            if (geom->link_status)
               ImGui::TextColored(severity_colors[SUCCESS], "linked, %d uniforms, %d attribs",
                  geom->num_uniforms, geom->num_attributes);
            else
               ImGui::TextColored(severity_colors[FIXME], "LINK FAILED - see Console tab");
         }

         ShaderProgram* gbuf = &window->gbuf.shader;
         if (gbuf->id)
         {
            ImGui::Text("lighting  (gbuf.vert/frag) : ");
            ImGui::SameLine();
            if (gbuf->link_status)
               ImGui::TextColored(severity_colors[SUCCESS], "linked, %d uniforms, %d attribs",
                  gbuf->num_uniforms, gbuf->num_attributes);
            else
               ImGui::TextColored(severity_colors[FIXME], "LINK FAILED - see Console tab");
         }
      }

      ImGui::EndTabItem();
   }
}

// ---------------- tab 4 : Anim ----------------
void draw_ANIM_console(GameConsole* console)
{
   using namespace CNSL;

   if (ImGui::BeginTabItem("Anim"))
   {
      section("Playback");

      if (anim_state.num_bones == 0)
      {
         ImGui::TextColored(severity_colors[FIXME], "no clip loaded");
      }
      else
      {
         // timeline + scrub : drag to inspect any moment
         if (!anim_control.scrubbing)
            anim_control.scrub_t = (anim_state.duration > 0) ? anim_state.elapsed / anim_state.duration : 0.f;
         ImGui::SliderFloat("##scrub", &anim_control.scrub_t, 0.f, 1.f, "%.3f");
         anim_control.scrubbing = ImGui::IsItemActive();

         // green bands = rotation holds found at load
         ImDrawList* draw_list = ImGui::GetWindowDrawList();
         ImVec2 r0 = ImGui::GetItemRectMin(), r1 = ImGui::GetItemRectMax();
         for (uint h = 0; h < anim_state.num_holds; h++)
         {
            float x0 = r0.x + (anim_state.holds[h].first / (float)anim_state.num_frames) * (r1.x - r0.x);
            float x1 = r0.x + ((anim_state.holds[h].last + 1) / (float)anim_state.num_frames) * (r1.x - r0.x);
            draw_list->AddRectFilled(ImVec2(x0, r0.y), ImVec2(x1, r1.y), IM_COL32(90, 170, 100, 90));
            draw_list->AddLine(ImVec2(x0, r0.y), ImVec2(x0, r1.y), IM_COL32(130, 230, 140, 200));
         }

         // transport
         if (ImGui::Button(anim_control.paused ? ">" : "||"))
            anim_control.paused = !anim_control.paused;
         ImGui::SameLine();
         if (ImGui::Button("restart"))
            anim_control.restart = true;
         ImGui::SameLine();
         const char* state = anim_control.paused ? "paused"
            : anim_state.playing ? "playing"
            : (anim_state.duration > 0 ? "finished" : "static frame 0");
         ImGui::Text("%s | %.2f / %.2f s | frame %.2f / %u",
            state, anim_state.elapsed, anim_state.duration, anim_state.frame_f, anim_state.num_frames);

         section("Feel");
         static const char* eases[] = { "linear", "smooth", "in-out cubic", "out cubic", "in-out quint" };
         ImGui::Combo("time ease", &anim_control.time_ease, eases, 5);
         ImGui::SliderFloat("juice", &anim_control.weight, 0.f, 1.f, "%.2f");
         ImGui::SliderFloat("speed", &anim_control.speed, 0.1f, 2.f, "%.2fx");
         ImGui::Checkbox("loop", &anim_control.loop);

         section("Clip");
         ImGui::Text("bones : %u | frames : %u | holds : %u",
            anim_state.num_bones, anim_state.num_frames, anim_state.num_holds);
         ImGui::Text("length: %.2fs (set by play(), scaled by speed)", anim_state.duration);
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
      // ---- row 1 : severity toggles + badge counts
      for (uint s = 0; s < 5; s++)
      {
         ImGui::PushStyleColor(ImGuiCol_Text,
            severity_visible[s] ? severity_colors[s] : ImVec4(0.35f, 0.35f, 0.35f, 0.7f));
         char label[12];
         snprintf(label, 12, "%s##sev%u", severity_messages[s], s);
         if (ImGui::SmallButton(label)) severity_visible[s] = !severity_visible[s];
         ImGui::PopStyleColor();
         if (s < 4) ImGui::SameLine();
      }

      ImGui::SameLine();
      ImGui::TextColored(severity_colors[WARNING], "%u warnings", console->count(WARNING));
      ImGui::SameLine();
      ImGui::TextColored(severity_colors[FIXME], "%u errors", console->count(FIXME));

      // ---- row 2 : search, autoscroll, clear
      ImGui::SetNextItemWidth(-190);
      ImGui::InputText("##logsearch", search, sizeof(search));
      ImGui::SameLine();
      ImGui::Checkbox("Autoscroll", &autoscroll);
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
      draw_range(logs.write_idx, MAX_LOGQUEUE_ENTRIES);
      draw_range(0, logs.write_idx);

      if (autoscroll) ImGui::SetScrollHereY(1.0f); // keep newest line visible

      ImGui::EndTabItem();
   }
}

// This draws an imgui window for the console
void draw_console(GameConsole* console, GameRenderer* rndr, GameWindow* window)
{
   using namespace CNSL;

   // imgui window settings
   ImGuiWindowFlags flags = ImGuiWindowFlags_NoCollapse
      | ImGuiWindowFlags_NoResize
      | ImGuiWindowFlags_NoMove
      | ImGuiWindowFlags_NoScrollbar
      | ImGuiWindowFlags_NoTitleBar;

   // imgui window -> background color
   ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.1f, 0.1f, 0.1f, 0.5f)); // RGBA

   // create imgui window
   ImGui::Begin("GameConsole", (bool*)1, flags);
   ImGui::SetWindowPos(ImVec2(0, 0));
   ImGui::SetWindowSize(ImVec2(620, 1080)); // wide enough for the data tables

   // draw tabs
   ImGui::BeginTabBar("Console Tabs", ImGuiTabBarFlags_None);
   draw_ANIM_console(console);
   draw_SYSTEM_console(console, window);
   draw_RENDER_console(console, rndr, window);
   draw_ASST_console(console, window, rndr);
   draw_MAIN_console(console);
   ImGui::EndTabBar();

   ImGui::End();
   ImGui::PopStyleColor();
}
