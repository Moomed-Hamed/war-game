#include "../dependencies/proprietary/boilerplate.h"

/* ------------------------- *
   -    Simple Console     -
 * ------------------------- */

enum SEVERITY {
   TRACE = 0,
   WARNING ,
   FIXME   ,
   DEBUG   ,
   SUCCESS
};

enum LOGSOURCE {
   WNDW = 0, // Window
   RNDR,     // Renderer
   PHYS,     // Physics
   NETW      // Networking
};

const uint MAX_LOGQUEUE_ENTRIES = 128;
const uint MAX_LOG_MSG_LENGTH = 62;

// Each log entry must be exactly 64 bytes
struct alignas(64) Log_Entry {
   uint8 source;
   uint8 severity;
   char  text[MAX_LOG_MSG_LENGTH];
};

struct LogQueue {
   Log_Entry entries[MAX_LOGQUEUE_ENTRIES];
   uint write_idx; // write index
};

struct GameConsole {
   LogQueue logs;

   // Add a message to the rolling log. Safe to call from anywhere with any
   // string : always null-terminated, never reads past the source string.
   void add_entry(const char* text, uint8 severity = TRACE, uint8 source = WNDW)
   {
      uint idx = logs.write_idx;

      // create a new entry
      Log_Entry& entry = logs.entries[idx];
      entry.source   = source;
      entry.severity = severity;
      snprintf(entry.text, MAX_LOG_MSG_LENGTH, "%s", text);

      // increment + wrap around if needed
      logs.write_idx = (logs.write_idx + 1) % MAX_LOGQUEUE_ENTRIES;
   }

   // wipe all entries (Console tab -> Clear button)
   void clear()
   {
      memset(&logs, 0, sizeof(LogQueue));
   }

   // count entries matching a severity, for the warning/error badges
   uint count(uint8 severity)
   {
      uint total = 0;
      for (uint i = 0; i < MAX_LOGQUEUE_ENTRIES; i++)
         if (logs.entries[i].text[0] != 0 && logs.entries[i].severity == severity)
            total++;
      return total;
   }
} *console;
