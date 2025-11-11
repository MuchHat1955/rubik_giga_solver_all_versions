#pragma once
#include <Arduino.h>

// ============================================================================
// CONFIGURATION
// ============================================================================
#define MAX_NESTED_SECTIONS 24
#define LOG_BUFFER_SIZE 1024  // ring buffer for deferred logging from ISRs

// CATEGORY ENABLES (comment out to disable compile-time)

// #define LOG_MENU
#define LOG_RB_COMMANDS
#define LOG_PARAM
#define LOG_POSE
#define LOG_SEQUENCES
#define LOG_CUBE

// Forward declaration to avoid circular include
class RBInterface;
extern RBInterface rb;

// ============================================================================
// GLOBAL FLAGS / STATE
// ============================================================================
extern bool logging_on;
extern bool log_menu_enabled;
extern bool log_rb_enabled;
extern bool log_param_enabled;
extern bool log_pose_enabled;
extern bool log_seq_enabled;
extern bool log_cube_enabled;

// section context
extern bool current_section_enabled;
extern const char* current_log_prefix;

// bee logging (auto-off after 6 min)
extern bool bee_logging_enabled;
extern unsigned long bee_logging_start_ms;
#define BEE_LOGGING_DURATION_MS (6UL * 60UL * 1000UL)

inline void init_logging() {
  bee_logging_enabled = true;
  bee_logging_start_ms = millis();
}
inline void update_bee_logging() {
  if (bee_logging_enabled && millis() - bee_logging_start_ms > BEE_LOGGING_DURATION_MS)
    bee_logging_enabled = false;
}
inline void on_system_pose_opened() {
  bee_logging_enabled = true;
  bee_logging_start_ms = millis();
}

// ---- error retrieval -----------------------------------------------------
void addErrorLine(const String& line);
const char* getLastErrorLine();
String getAllErrorLines();
void clearErrorBuffer();

#define SHOULD_LOG(flag) (bee_logging_enabled && (flag))

// ============================================================================
// CORE LOGGING INTERFACE
// ============================================================================
void log_indent();
void log_section_start(const String& section_name);
void log_section_start_var(const String& title, const String& var_name, const String& var_val);
void log_section_end();
void log_indent_reset();
void log_flush_buffer();            // periodic flush
void log_enqueue(const char* msg);  // ISR-safe enqueue

// ============================================================================
// LOW-LEVEL PRINT HELPERS
// ============================================================================
template<typename... Args>
inline void serial_printf(const char* fmt, Args... args) {
  char buf[200];
  snprintf(buf, sizeof(buf), fmt, args...);
  for (char* p = buf; *p; ++p) *p = tolower(*p);  // convert to lowercase
  Serial.print(buf);
}

// ============================================================================
// BASE MACROS
// ============================================================================
#define LOG_SECTION_START(title) \
  do { \
    if (logging_on) log_section_start(title); \
  } while (0)
#define LOG_SECTION_END() \
  do { \
    if (logging_on) log_section_end(); \
  } while (0)
#define LOG_RESET() \
  do { log_indent_reset(); } while (0)
#define LOG_FLUSH() \
  do { log_flush_buffer(); } while (0)

// ============================================================================
// FORMATTED LOGGING
// ============================================================================
#define LOG_PRINTF(fmt, ...) \
  do { \
    if (logging_on) { \
      log_indent(); \
      serial_printf(fmt, ##__VA_ARGS__); \
    } \
  } while (0)

#define LOG_SECTION_START(title, fmt, ...) \
  do { \
    if (logging_on) { \
      log_indent(); \
      serial_printf("---- {{%s}} start ", title); \
      serial_printf(fmt, ##__VA_ARGS__); \
      Serial.println(" ----"); \
    } \
  } while (0)

// Error helper (ties to RBInterface)
#define LOG_ERROR(fmt, ...) \
  do { \
    char _buf[200]; \
    snprintf(_buf, sizeof(_buf), fmt, ##__VA_ARGS__); \
    LOG_PRINTF("[!] %s", _buf); \
    addErrorLine(_buf); \
  } while (0)

// ============================================================================
// AUTO-PREFIXED LOGGING
// ============================================================================
#define LOG_PRINTF_AUTO(fmt, ...) \
  do { \
    if (current_section_enabled && bee_logging_enabled && logging_on) { \
      if (current_log_prefix && current_log_prefix[0]) \
        LOG_PRINTF("[%s] " fmt, current_log_prefix, ##__VA_ARGS__); \
      else \
        LOG_PRINTF(fmt, ##__VA_ARGS__); \
    } \
  } while (0)

// ============================================================================
// SECTION MACROS
// ============================================================================
#define LOG_SECTION_START_TAG(tag, fmt, ...) \
  do { \
    current_section_enabled = true; \
    current_log_prefix = tag; \
    if (bee_logging_enabled && logging_on) { \
      if (tag && tag[0]) \
        LOG_SECTION_START(tag, "| [%s] " fmt, tag, ##__VA_ARGS__); \
      else \
        LOG_SECTION_START(tag, fmt, ##__VA_ARGS__); \
    } \
  } while (0)

#define LOG_SECTION_START_IF(flag, tag, fmt, ...) \
  do { \
    current_section_enabled = SHOULD_LOG(flag) && logging_on; \
    current_log_prefix = (current_section_enabled) ? tag : ""; \
    if (current_section_enabled) { \
      if (tag && tag[0]) \
        LOG_SECTION_START(tag, "| [%s] " fmt, tag, ##__VA_ARGS__); \
      else \
        LOG_SECTION_START(tag, fmt, ##__VA_ARGS__); \
    } \
  } while (0)

#define LOG_SECTION_END_TAG() \
  do { \
    current_section_enabled = true; \
    current_log_prefix = ""; \
  } while (0)

// ============================================================================
// CATEGORY SECTION SHORTCUTS
// ============================================================================
#ifdef LOG_MENU
#define LOG_SECTION_START_MENU(fmt, ...) LOG_SECTION_START_IF(log_menu_enabled, "MENU", fmt, ##__VA_ARGS__)
#else
#define LOG_SECTION_START_MENU(fmt, ...) ((void)0)
#endif

#ifdef LOG_RB_COMMANDS
#define LOG_SECTION_START_RB(fmt, ...) LOG_SECTION_START_IF(log_rb_enabled, "RB", fmt, ##__VA_ARGS__)
#else
#define LOG_SECTION_START_RB(fmt, ...) ((void)0)
#endif

#ifdef LOG_PARAM
#define LOG_SECTION_START_PARAM(fmt, ...) LOG_SECTION_START_IF(log_param_enabled, "PARAM", fmt, ##__VA_ARGS__)
#else
#define LOG_SECTION_START_PARAM(fmt, ...) ((void)0)
#endif

#ifdef LOG_POSE
#define LOG_SECTION_START_POSE(fmt, ...) LOG_SECTION_START_IF(log_pose_enabled, "POSE", fmt, ##__VA_ARGS__)
#else
#define LOG_SECTION_START_POSE(fmt, ...) ((void)0)
#endif

#ifdef LOG_SEQUENCES
#define LOG_SECTION_START_SEQ(fmt, ...) LOG_SECTION_START_IF(log_seq_enabled, "SEQ", fmt, ##__VA_ARGS__)
#else
#define LOG_SECTION_START_SEQ(fmt, ...) ((void)0)
#endif

#ifdef LOG_CUBE
#define LOG_SECTION_START_CUBE(fmt, ...) LOG_SECTION_START_IF(log_cube_enabled, "CUBE", fmt, ##__VA_ARGS__)
#else
#define LOG_SECTION_START_CUBE(fmt, ...) ((void)0)
#endif

// ============================================================================
// CATEGORY PRINTF MACROS (independent or inside a section)
// ============================================================================
// Each one checks if (a) its own runtime flag is on, or (b) the current section
// prefix matches the same tag (e.g. inside a REFLECT section).
// This way, standalone logs obey the per-category flag, while
// nested ones follow the section’s active state automatically.

#define LOG_SHOULD_PRINT(tag, enabled_flag) \
  (logging_on && bee_logging_enabled && (enabled_flag || (current_log_prefix && strcmp(current_log_prefix, tag) == 0)))

// ---------------------------------------------------------------- MENU --
#ifdef LOG_MENU
#define LOG_PRINTF_MENU(fmt, ...) \
  do { \
    if (LOG_SHOULD_PRINT("MENU", log_menu_enabled)) { \
      LOG_PRINTF("[REFLECT] " fmt, ##__VA_ARGS__); \
    } \
  } while (0)
#else
#define LOG_PRINTF_MENU(fmt, ...) ((void)0)
#endif

// ------------------------------------------------------------------ RB -----
#ifdef LOG_RB_COMMANDS
#define LOG_PRINTF_RB(fmt, ...) \
  do { \
    if (LOG_SHOULD_PRINT("RB", log_rb_enabled)) { \
      LOG_PRINTF("[RB] " fmt, ##__VA_ARGS__); \
    } \
  } while (0)
#else
#define LOG_PRINTF_RB(fmt, ...) ((void)0)
#endif

// ---------------------------------------------------------------- PARAM --
#ifdef LOG_PARAM
#define LOG_PRINTF_PARAM(fmt, ...) \
  do { \
    if (LOG_SHOULD_PRINT("PARAM", log_param_enabled)) { \
      LOG_PRINTF("[PARAM] " fmt, ##__VA_ARGS__); \
    } \
  } while (0)
#else
#define LOG_PRINTF_PARAM(fmt, ...) ((void)0)
#endif

// ------------------------------------------------------------------ MENU ---
#ifdef LOG_MENU
#define LOG_PRINTF_MENU(fmt, ...) \
  do { \
    if (LOG_SHOULD_PRINT("MENU", log_pose_enabled)) { \
      LOG_PRINTF("[MENU] " fmt, ##__VA_ARGS__); \
    } \
  } while (0)
#else
#define LOG_PRINTF_MENU(fmt, ...) ((void)0)
#endif

// --------------------------------------------------------------- SEQUENCES --
#ifdef LOG_SEQUENCES
#define LOG_PRINTF_SEQ(fmt, ...) \
  do { \
    if (LOG_SHOULD_PRINT("SEQ", log_seq_enabled)) { \
      LOG_PRINTF("[SEQ] " fmt, ##__VA_ARGS__); \
    } \
  } while (0)
#else
#define LOG_PRINTF_SEQ(fmt, ...) ((void)0)
#endif

// ------------------------------------------------------------------ CUBE ---
#ifdef LOG_CUBE
#define LOG_PRINTF_CUBE(fmt, ...) \
  do { \
    if (LOG_SHOULD_PRINT("CUBE", log_cube_enabled)) { \
      LOG_PRINTF("[CUBE] " fmt, ##__VA_ARGS__); \
    } \
  } while (0)
#else
#define LOG_PRINTF_CUBE(fmt, ...) ((void)0)
#endif
