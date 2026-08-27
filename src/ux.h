#include "drawer.h"

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

   // Source colors — modern, balanced, and consistent brightness
   const ImVec4 source_colors[4] = {
       ImVec4(0.45f, 0.65f, 1.00f, 1.0f), // WNDW : Azure Blue
       ImVec4(0.75f, 0.60f, 1.00f, 1.0f), // RNDR : Vivid Violet
       ImVec4(1.00f, 0.55f, 0.45f, 1.0f), // PHYS : Coral Rust
       ImVec4(1.00f, 0.80f, 0.55f, 1.0f)  // NETW : Signal Amber
   };

   // Severity levels — distinct, modern color language
   const ImVec4 severity_colors[5] = {
       ImVec4(0.75f, 0.55f, 1.00f, 1.0f), // TRACE   : Purple (Insight)
       ImVec4(1.00f, 0.90f, 0.45f, 1.0f), // WARNING : Yellow (Caution)
       ImVec4(1.00f, 0.55f, 0.20f, 1.0f), // FIXME   : Orange (Problem)
       ImVec4(0.45f, 0.70f, 1.00f, 1.0f), // DEBUG   : Blue (Info)
       ImVec4(0.45f, 1.00f, 0.55f, 1.0f)  // SUCCESS : Green (OK)
   };

   // Message text — softened tints of above for legibility on dark backgrounds
   const ImVec4 message_colors[5] = {
       ImVec4(0.85f, 0.70f, 1.00f, 1.0f), // TRACE   : Soft Lavender
       ImVec4(1.00f, 0.95f, 0.70f, 1.0f), // WARNING : Pale Yellow
       ImVec4(1.00f, 0.75f, 0.50f, 1.0f), // FIXME   : Soft Orange
       ImVec4(0.70f, 0.85f, 1.00f, 1.0f), // DEBUG   : Light Blue
       ImVec4(0.70f, 1.00f, 0.75f, 1.0f)  // SUCCESS : Soft Green
   };
}

void draw_MAIN_console(GameConsole* console)
{
   using namespace CNSL;

   LogQueue logs = console->logs;

   if (ImGui::BeginTabItem("Console"))
   {
      // loop to print newest->oldest, we do 2 loops because we're using a rolling buffer
      for (uint i = logs.write_idx; i < MAX_LOGQUEUE_ENTRIES; i++)
      {
         if (logs.entries[i].text[0] == 0) continue; // don't draw empty strings

         uint severity = logs.entries[i].severity;
         uint source = logs.entries[i].source;

         // Print the source, severity, and message with appropriate colors
         ImGui::TextColored(source_colors[source], "[%-4s]", source_messages[source]);
         ImGui::SameLine();
         ImGui::TextColored(severity_colors[severity], "[%s]", severity_messages[severity]);
         ImGui::SameLine();
         ImGui::TextColored(message_colors[severity], "[%s]", logs.entries[i].text);
      }

      for (uint i = 0; i < logs.write_idx; i++)
      {
         if (logs.entries[i].text[0] == 0) continue; // don't draw empty strings

         uint severity = logs.entries[i].severity;
         uint source = logs.entries[i].source;

         ImGui::TextColored(source_colors[source], "[%-4s]", source_messages[source]);
         ImGui::SameLine();
         ImGui::TextColored(severity_colors[severity], "[%s]", severity_messages[severity]);
         ImGui::SameLine();
         ImGui::TextColored(message_colors[severity], "[%s]", logs.entries[i].text);
      }

      ImGui::Separator();
      ImGui::SetScrollHereY(1.0f); // Scroll to bottom
      ImGui::EndTabItem();
   }
}
void draw_RNDR_console(GameConsole* console, GameRenderer* rndr)
{
   using namespace CNSL;

   LogQueue logs = console->logs;

   if (ImGui::BeginTabItem("Render"))
   {
      ImGui::Text("Num Meshes: %d", rndr->drawbuffer.num_meshes); ImGui::SameLine();
      ImGui::Text("Buffer Size: %d"    , rndr->drawbuffer.max_buffer_size);
      ImGui::Text("Vertex Memory: %d"  , rndr->drawbuffer.geom_size); ImGui::SameLine();
      ImGui::Text("Index Memory: %d"   , rndr->drawbuffer.indx_size); ImGui::SameLine();
      ImGui::Text("Instance Memory: %d", rndr->drawbuffer.inst_size);

      ImGui::Separator();

      // loop to print newest->oldest, we do 2 loops because we're using a rolling buffer
      for (uint i = logs.write_idx; i < MAX_LOGQUEUE_ENTRIES; i++)
      {
         if (logs.entries[i].text[0] == 0 || logs.entries[i].source != RNDR)
            continue; // don't draw empty strings

         uint severity = logs.entries[i].severity;
         uint source = logs.entries[i].source;

         // Print the source, severity, and message with appropriate colors
         ImGui::TextColored(source_colors[source], "[%-4s]", source_messages[source]);
         ImGui::SameLine();
         ImGui::TextColored(severity_colors[severity], "[%s]", severity_messages[severity]);
         ImGui::SameLine();
         ImGui::TextColored(message_colors[severity], "[%s]", logs.entries[i].text);
      }

      for (uint i = 0; i < logs.write_idx; i++)
      {
         if (logs.entries[i].text[0] == 0 || logs.entries[i].source != RNDR)
            continue; // don't draw empty strings

         uint severity = logs.entries[i].severity;
         uint source = logs.entries[i].source;

         ImGui::TextColored(source_colors[source], "[%-4s]", source_messages[source]);
         ImGui::SameLine();
         ImGui::TextColored(severity_colors[severity], "[%s]", severity_messages[severity]);
         ImGui::SameLine();
         ImGui::TextColored(message_colors[severity], "[%s]", logs.entries[i].text);
      }

      ImGui::Separator();
      ImGui::SetScrollHereY(1.0f); // Scroll to bottom
      ImGui::EndTabItem();
   }
}
void draw_ASST_console(GameConsole* console, GameWindow* window, GameRenderer* rndr)
{
   using namespace CNSL;

   LogQueue logs = console->logs;

   if (ImGui::BeginTabItem("Assets"))
   {
      // Texture has to be flipped for ImGui
      float scale = .27;
      ImGui::Image((ImTextureID)(intptr_t)window->gbuf.rendertarget.normal_tx, 
         ImVec2(1920 * scale, 1080 * scale), ImVec2(0, 1), ImVec2(1, 0));

      ImGui::Separator();

      // list of loaded meshes
      if (ImGui::CollapsingHeader("Meshes"), ImGuiTreeNodeFlags_DefaultOpen)
      {
         for (int i = 0; i < MAX_MESHES; i++)
         {
            if (rndr->meshloader.meshes[i].id == 0)
               continue;

            ImGui::Text("%s", rndr->meshloader.meshes[i].filepath);
         }
      }

      ImGui::Separator();
      ImGui::EndTabItem();
   }
}

// This draws an imgui window for the console
void draw_console(GameConsole* console, GameRenderer* rndr, GameWindow* window)
{
   using namespace CNSL;

   LogQueue logs = console->logs;

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
   ImGui::SetWindowSize(ImVec2(540, 1080));

   // draw tabs
   ImGui::BeginTabBar("Console Tabs", ImGuiTabBarFlags_None);
   draw_ASST_console(console, window, rndr);
   draw_RNDR_console(console, rndr);
   draw_MAIN_console(console);
   ImGui::EndTabBar();

   ImGui::End();
   ImGui::PopStyleColor();
}