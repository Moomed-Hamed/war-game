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

const uint MAX_LOGQUEUE_ENTRIES = 64;
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

   void add_entry(char* text, uint8 severity = TRACE, uint8 source = WNDW)
   {
      uint idx = logs.write_idx;

      // create a new entry
      logs.entries[idx].source   = source;
      logs.entries[idx].severity = severity;
      memcpy(logs.entries[idx].text, text, 62);

      // increment + wrap around if needed
      logs.write_idx = (logs.write_idx + 1) % MAX_LOGQUEUE_ENTRIES;
   }
} *console;
