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
   NETW,     // Networking
   AUDI      // Audio
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

   // write the whole log to a text file, oldest first, on the way out : a session can
   // then be read back rather than described. returns false if the file would not open
   bool dump(const char* path = "console_log.txt")
   {
      FILE* out = fopen(path, "w");
      if (!out) return false;

      static const char* sev[5] = { "TRACE", "WARN ", "FIXME", "DEBUG", "OK   " };
      static const char* src[5] = { "WNDW", "RNDR", "PHYS", "NETW", "AUDI" };

      // the ring's write index is the oldest entry still held, so start there and walk
      // forwards : that is chronological order
      for (uint n = 0; n < MAX_LOGQUEUE_ENTRIES; n++)
      {
         Log_Entry& e = logs.entries[(logs.write_idx + n) % MAX_LOGQUEUE_ENTRIES];
         if (e.text[0] == 0) continue; // never written

         fprintf(out, "[%s][%s] %s\n",
            sev[e.severity < 5 ? e.severity : 0],
            src[e.source   < 5 ? e.source   : 0],
            e.text);
      }

      fclose(out);
      return true;
   }
} *console;
