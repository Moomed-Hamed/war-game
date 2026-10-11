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
void draw_SYSTEM_console(GameWindow* window)
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
void draw_RENDER_console(GameRenderer* rndr, GameWindow* window, Camera* camera)
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
               NULL, 0.f, 33.f, ImVec2(600, 60)); // hardcoded pixel size (window is fixed-size)

         ImGui::Text("avg %.1f ms  |  min %.1f  |  max %.1f",
            window->frame_ms_avg, window->frame_ms_min, window->frame_ms_max);

         // how much of the work budget this frame used (clamped: ImGui asserts fraction <= 1)
         float target = window->frame_milliseconds_target;
         float used = (target > 0) ? (window->frame_ms_work / target) : 0;
         if (used > 1.f) used = 1.f;
         char budget[48];
         snprintf(budget, 48, "budget : %.1f / %.1f ms", window->frame_ms_work, target);
         ImGui::PushStyleColor(ImGuiCol_PlotHistogram,
            (used >= 1.f) ? ImVec4(1.f, 0.4f, 0.3f, 1.f) : ImVec4(0.4f, 0.75f, 1.f, 1.f));
         ImGui::ProgressBar(used, ImVec2(-1, 0), budget);
         ImGui::PopStyleColor();
         ImGui::Text("work %.1f ms  |  wait %.1f ms",
            window->frame_ms_work, window->frame_ms_last - window->frame_ms_work);
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

      // ---- per-mesh draw list, both passes : proof each append_instances() worked
      section("Mesh Draw List");
      if (ImGui::BeginTable("meshdraw", 7,
         ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_RowBg | ImGuiTableFlags_Borders))
      {
         ImGui::TableSetupColumn("pass"  , ImGuiTableColumnFlags_WidthFixed, 52);
         ImGui::TableSetupColumn("id"    , ImGuiTableColumnFlags_WidthFixed, 30);
         ImGui::TableSetupColumn("indices", ImGuiTableColumnFlags_WidthFixed, 58);
         ImGui::TableSetupColumn("insts" , ImGuiTableColumnFlags_WidthFixed, 44);
         ImGui::TableSetupColumn("verts" , ImGuiTableColumnFlags_WidthFixed, 58);
         ImGui::TableSetupColumn("base vtx", ImGuiTableColumnFlags_WidthFixed, 58);
         ImGui::TableSetupColumn("base idx", ImGuiTableColumnFlags_WidthFixed, 62);
         ImGui::TableHeadersRow();

         // both draw buffers share the mesh_info field names, so one writer covers them
         auto pass_rows = [](const char* pass, auto* mesh_info, uint count)
         {
            for (uint i = 0; i < count; i++)
            {
               if (mesh_info[i].mesh_id == 0) continue;

               ImGui::TableNextRow();
               ImGui::TableNextColumn(); ImGui::Text("%s", pass);
               ImGui::TableNextColumn(); ImGui::Text("%u", mesh_info[i].mesh_id);
               ImGui::TableNextColumn(); ImGui::Text("%u", mesh_info[i].num_indices);
               ImGui::TableNextColumn();
               if (mesh_info[i].num_instances == 0)
                  ImGui::TextColored(severity_colors[WARNING], "%u", 0u); // never submitted this frame
               else
                  ImGui::Text("%u", mesh_info[i].num_instances);
               ImGui::TableNextColumn(); ImGui::Text("%u", mesh_info[i].num_vertices);
               ImGui::TableNextColumn(); ImGui::Text("%u", mesh_info[i].base_vertex);
               ImGui::TableNextColumn(); ImGui::Text("%u", mesh_info[i].base_index);
            }
         };

         pass_rows("static" , rndr->drawbuffer.mesh_info     , MAX_MESHES);
         pass_rows("skinned", rndr->drawbuffer_anim.mesh_info, MAX_MESHES);
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
         // uv flipped because render-to-texture is upside down vs imgui
         ImGui::Image((ImTextureID)(intptr_t)texture, ImVec2(176, 99), ImVec2(0, 1), ImVec2(1, 0));
      };

      gbuf_image("positions  RGB = world pos, A = metallic  [RGBA32F] ", window->gbuf.rendertarget.position_tx);
      gbuf_image("normals    RGB = world nrm, A = roughness [RGBA16F] ", window->gbuf.rendertarget.normal_tx);
      gbuf_image("albedo     RGB = base col, A = AO         [RGBA8]   ", window->gbuf.rendertarget.albedo_tx);
      gbuf_image("depth                                   [DEPTH24] ", window->gbuf.rendertarget.depth_tx);

      // ---- camera & input (the scene owns the camera, this just reads it)
      section("Camera / Input");
      vec3 pos = camera->position;
      ImGui::Text("position : %6.2f %6.2f %6.2f", pos.x, pos.y, pos.z);
      ImGui::Text("yaw/pitch: %6.1f / %5.1f deg",
         ToDegrees(camera->yaw), ToDegrees(camera->pitch));
      ImGui::Text("focus    : %s (T frees the cursor, Y recaptures)",
         window->focus_status == GAME ? "GAME" : "GUI");
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

// ---------------- tab : Phys ----------------
void draw_PHYS_console(Physics* phys)
{
   using namespace CNSL;

   if (ImGui::BeginTabItem("Phys"))
   {
      section("Bodies");
      ImGui::Text("in world : %u / %u slots", phys->num_bodies - 1, MAX_BODIES);
      ImGui::Checkbox("collider wireframes", &phys->debug.enabled);

      static const char* shape_names[] = { "none", "box", "sphere", "capsule", "cylinder" };

      if (ImGui::BeginTable("physbodies", 9,
         ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_RowBg | ImGuiTableFlags_Borders))
      {
         ImGui::TableSetupColumn("id"   , ImGuiTableColumnFlags_WidthFixed, 30);
         ImGui::TableSetupColumn("shape", ImGuiTableColumnFlags_WidthFixed, 52);
         ImGui::TableSetupColumn("pos x", ImGuiTableColumnFlags_WidthFixed, 50);
         ImGui::TableSetupColumn("pos y", ImGuiTableColumnFlags_WidthFixed, 50);
         ImGui::TableSetupColumn("pos z", ImGuiTableColumnFlags_WidthFixed, 50);
         ImGui::TableSetupColumn("vel x", ImGuiTableColumnFlags_WidthFixed, 50);
         ImGui::TableSetupColumn("vel y", ImGuiTableColumnFlags_WidthFixed, 50);
         ImGui::TableSetupColumn("vel z", ImGuiTableColumnFlags_WidthFixed, 50);
         ImGui::TableSetupColumn("state", ImGuiTableColumnFlags_WidthFixed, 54);
         ImGui::TableHeadersRow();

         for (uint i = 1; i < phys->num_bodies; i++)
         {
            Phys_Body& b = phys->bodies[i];
            if (!b.body) continue;

            vec3 pos = phys->get_position(i);
            vec3 vel = phys->get_velocity(i);

            ImGui::TableNextRow();
            ImGui::TableNextColumn(); ImGui::Text("%u", i);
            ImGui::TableNextColumn(); ImGui::Text("%s", shape_names[b.shape]);
            ImGui::TableNextColumn(); ImGui::Text("%.2f", pos.x);
            ImGui::TableNextColumn(); ImGui::Text("%.2f", pos.y);
            ImGui::TableNextColumn(); ImGui::Text("%.2f", pos.z);
            ImGui::TableNextColumn(); ImGui::Text("%.2f", vel.x);
            ImGui::TableNextColumn(); ImGui::Text("%.2f", vel.y);
            ImGui::TableNextColumn(); ImGui::Text("%.2f", vel.z);
            ImGui::TableNextColumn();
            if (b.mass == 0)
               ImGui::Text("static");
            else if (phys->is_awake(i))
               ImGui::TextColored(severity_colors[WARNING], "awake");
            else
               ImGui::TextColored(severity_colors[TRACE], "sleep");
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
            ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_RowBg | ImGuiTableFlags_Borders))
            return;

         ImGui::TableSetupColumn("id"  , ImGuiTableColumnFlags_WidthFixed, 30);
         ImGui::TableSetupColumn("file", ImGuiTableColumnFlags_WidthFixed, 190);
         ImGui::TableSetupColumn("disk", ImGuiTableColumnFlags_WidthFixed, 54);
         ImGui::TableSetupColumn("verts", ImGuiTableColumnFlags_WidthFixed, 58);
         ImGui::TableSetupColumn("tris",  ImGuiTableColumnFlags_WidthFixed, 54);
         ImGui::TableSetupColumn("gpu" , ImGuiTableColumnFlags_WidthFixed, 54);
         ImGui::TableSetupColumn("texture", ImGuiTableColumnFlags_WidthFixed, 100);
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

      // ---- static meshes : file info joined with what actually landed in the gpu buffer
      section("Static Meshes");
      mesh_table("staticmeshtable", rndr->meshloader.meshes, &rndr->drawbuffer, sizeof(MeshVertexData), &rndr->txloader);

      // ---- skinned meshes : weights + bone ids vertex layout, posed by the active clip
      section("Skinned Meshes");
      mesh_table("skinnedmeshtable", rndr->meshloader_anim.meshes, &rndr->drawbuffer_anim, sizeof(MeshVertexDataAnim), &rndr->txloader);

      // ---- animation clips : skeleton (parents + inverse-bind) + keyframes per loaded slot
      section("Animations");
      {
         AnimLoader* anim = &rndr->animloader;
         uint bones = anim->num_players ? anim->clips[anim->players[anim->focus].clip_id].num_bones : 0u;
         ImGui::Text("skeleton : %u bones in use / %u max | pose ubo : %u bytes (binding 1)",
            bones, MAX_ANIM_BONES, (uint)(MAX_ANIM_BONES * sizeof(mat4)));

         if (ImGui::BeginTable("cliptable", 6,
            ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_RowBg | ImGuiTableFlags_Borders))
         {
            ImGui::TableSetupColumn("slot"  , ImGuiTableColumnFlags_WidthFixed, 36);
            ImGui::TableSetupColumn("file"  , ImGuiTableColumnFlags_WidthFixed, 190);
            ImGui::TableSetupColumn("bones" , ImGuiTableColumnFlags_WidthFixed, 48);
            ImGui::TableSetupColumn("frames", ImGuiTableColumnFlags_WidthFixed, 52);
            ImGui::TableSetupColumn("holds" , ImGuiTableColumnFlags_WidthFixed, 48);
            ImGui::TableSetupColumn("ram"   , ImGuiTableColumnFlags_WidthFixed, 64);
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

      // ---- textures : TextureLoader storage
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
// reads/writes the renderer's AnimLoader : the focus player's state (what update()
// wrote), its control (what the tab sets) and the clip's segments (frame ranges +
// per-segment feel). one player per clip in flight - the picker picks who edits
void draw_ANIM_console(AnimLoader* anim)
{
   using namespace CNSL;

   if (ImGui::BeginTabItem("Anim"))
   {
      section("Playback");

      if (anim->num_players == 0)
      {
         ImGui::TextColored(severity_colors[FIXME], "no clip playing");
      }
      else
      {
         static int selected = 0; // tab-local : which segment the Feel section edits
         static const char* eases[] = { "linear", "smooth", "in-out cubic", "out cubic", "in-out quint" };

         Anim_Player* pl = &anim->players[anim->focus];
         Animation* clip = &anim->clips[pl->clip_id];
         if (selected >= (int)clip->num_segs) selected = (int)clip->num_segs - 1; // the picker can swap to a shorter clip

         // pick which player this tab edits
         ImGui::Text("player :");
         for (uint i = 0; i < anim->num_players; i++)
         {
            ImGui::SameLine();
            const char* file = anim->clip_paths[anim->players[i].clip_id];
            const char* name = strrchr(file, '/'); name = name ? name + 1 : file;
            char label[48];
            snprintf(label, sizeof(label), "%s%u : %.14s##plr%u",
               (i == anim->focus) ? " >" : " ", i, name, i);
            if (ImGui::SmallButton(label)) anim->focus = i;
         }

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
            draw_list->AddRectFilled(ImVec2(x0, r0.y), ImVec2(x1, r1.y), IM_COL32(90, 170, 100, 90));
            draw_list->AddLine(ImVec2(x0, r0.y), ImVec2(x0, r1.y), IM_COL32(130, 230, 140, 200));
         }

         // transport
         if (ImGui::Button(pl->control.paused ? ">" : "||"))
            pl->control.paused = !pl->control.paused;
         ImGui::SameLine();
         if (ImGui::Button("restart"))
            pl->control.restart = true;
         ImGui::SameLine();
         const char* play_state = pl->control.paused ? "paused"
            : pl->state.playing ? "playing"
            : (pl->state.duration > 0 ? "finished" : "static frame 0");
         ImGui::Text("%s | %.2f / %.2f s | frame %.2f / %u | seg %u / %u",
            play_state, pl->state.elapsed, pl->state.duration, pl->state.frame_f,
            clip->num_frames, pl->state.seg, clip->num_segs);

         // ---- the segmented clip : several animations live in one file as frame ranges
         section("Segments");
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

         // ---- feel : the selected segment's easing + juice, previewed on the graph
         section("Feel");
         ImGui::Text("segment %d of %u", selected, clip->num_segs);
         ImGui::Combo("time ease", &clip->segs[selected].ease, eases, 5);
         ImGui::SliderFloat("juice", &clip->segs[selected].juice, 0.f, 1.f, "%.2f");

         // time warp graph : clock (x) -> clip position (y), all methods at once.
         // bright = selected segment's ease, dot = live playback head.
         // juice layer = the spring chasing a ramp, same constants as apply_spring ;
         // combine = the spring chases the selected ease instead, so you see both together
         {
            static bool show_juice = false;
            static bool combine = false; // ease shapes the target the spring follows
            const int N = 48;
            int ease = clip->segs[selected].ease;
            ImVec2 p0 = ImGui::GetCursorScreenPos();
            ImVec2 size(240, 140), p1(p0.x + size.x, p0.y + size.y);
            ImDrawList* dl = ImGui::GetWindowDrawList();

            dl->AddRectFilled(p0, p1, IM_COL32(16, 18, 24, 255)); // plot bg
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
                  active ? IM_COL32(120, 180, 255, 255) : IM_COL32(150, 160, 180, 90),
                  active ? 2.f : 1.f, true);
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
                  dl->AddPolyline(pts, 121, IM_COL32(235, 130, 210, 210), 1.5f, false);
               }
            }

            // live playback dot on the selected ease
            float raw = (pl->state.duration > 0) ? pl->state.elapsed / pl->state.duration : 0.f;
            ImVec2 dot(p0.x + raw * size.x, p1.y - anim->time_warp(raw, ease) * size.y);
            dl->AddLine(ImVec2(dot.x, p0.y), ImVec2(dot.x, p1.y), IM_COL32(255, 220, 90, 70));
            dl->AddCircleFilled(dot, 3.f, IM_COL32(255, 220, 90, 255));

            ImGui::Dummy(size);
            ImGui::Checkbox("juice layer", &show_juice);
            ImGui::SameLine();
            ImGui::Checkbox("combine with ease", &combine);
            ImGui::Text("x : clock    y : clip position    dot = playback head");
         }

         ImGui::SliderFloat("speed", &pl->control.speed, 0.1f, 2.f, "%.2fx");
         ImGui::Checkbox("loop", &pl->control.loop);

         section("Clip");
         ImGui::Text("bones : %u | frames : %u | holds : %u | segments : %u",
            clip->num_bones, clip->num_frames, clip->num_holds, clip->num_segs);
         ImGui::Text("length: %.2fs (set by play(), scaled by speed)", pl->state.duration);
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

void draw_console(GameConsole* console, GameRenderer* rndr, GameWindow* window, GameScene* scene)
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
   ImGui::Begin("GameConsole", NULL, flags);
   ImGui::SetWindowPos(ImVec2(0, 0));
   ImGui::SetWindowSize(ImVec2(620, 1080)); // wide enough for the data tables

   // draw tabs
   ImGui::BeginTabBar("Console Tabs", ImGuiTabBarFlags_None);
   draw_ANIM_console(&rndr->animloader);
   draw_SYSTEM_console(window);
   draw_RENDER_console(rndr, window, &scene->camera);
   draw_PHYS_console(&scene->phys);
   draw_ASST_console(window, rndr);
   draw_MAIN_console(console);
   ImGui::EndTabBar();

   ImGui::End();
   ImGui::PopStyleColor();
}
