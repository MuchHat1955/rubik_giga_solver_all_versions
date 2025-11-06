#include "logging.h"

static String log_section_name[MAX_NESTED_SECTIONS];
static unsigned long log_section_start_time[MAX_NESTED_SECTIONS];
static int log_section_index = 0;

// Ring buffer for deferred logging (ISR safe)
static char log_buffer[LOG_BUFFER_SIZE];
static volatile int log_buffer_head = 0;
static volatile int log_buffer_tail = 0;

bool logging_on = true;

// ----- Internal Helpers -----
static String format_time(unsigned long ms) {
  unsigned long minutes = ms / 60000;
  ms %= 60000;
  unsigned long seconds = ms / 1000;
  unsigned long millis_part = ms % 1000;

  if (minutes > 0)
    return String(minutes) + "m" + String(seconds) + "s" + String(millis_part) + "ms";
  else if (seconds > 0)
    return String(seconds) + "s" + String(millis_part) + "ms";
  else
    return String(millis_part) + "ms";
}

// --- Portable safe print helpers (no xPortInIsrContext on GIGA) ---
static bool in_isr_context() {
#ifdef ARDUINO_ARCH_ESP32
  return xPortInIsrContext();
#else
  return false;
#endif
}

static void safe_print(const char* msg) {
  if (in_isr_context()) {
    log_enqueue(msg);
  } else {
    Serial.print(msg);
  }
}

static void safe_println(const char* msg) {
  if (in_isr_context()) {
    log_enqueue(msg);
    log_enqueue("\n");
  } else {
    Serial.println(msg);
  }
}

// ----- Ring buffer logging -----
void log_enqueue(const char* msg) {
  if (!msg) return;
  while (*msg) {
    int next_head = (log_buffer_head + 1) % LOG_BUFFER_SIZE;
    if (next_head == log_buffer_tail) break;  // full
    log_buffer[log_buffer_head] = *msg++;
    log_buffer_head = next_head;
  }
}

void log_flush_buffer() {
  while (log_buffer_tail != log_buffer_head) {
    Serial.write(log_buffer[log_buffer_tail]);
    log_buffer_tail = (log_buffer_tail + 1) % LOG_BUFFER_SIZE;
  }
}

// ----- Indentation -----
void log_indent() {
  if (!logging_on) return;
  // Serial.println();
  for (int i = 0; i < log_section_index && i < MAX_NESTED_SECTIONS; i++) {
    Serial.print("   ");
  }
}

// ----- Section Start -----
void log_section_start(const String& section_name) {
  if (!logging_on) return;

  unsigned long now = millis();
  String t = format_time(now);

  if (log_section_index < MAX_NESTED_SECTIONS) {
    log_section_name[log_section_index] = section_name;
    log_section_start_time[log_section_index] = now;
  }

  log_indent();
  String msg = "---- {" + section_name + "} start <" + t + "> ----";
  safe_println(msg.c_str());

  log_section_index++;
  if (log_section_index > MAX_NESTED_SECTIONS)
    log_section_index = MAX_NESTED_SECTIONS;
}

// ----- Section Start with Variable -----
void log_section_start_var(const String& title, const String& var_name, const String& var_val) {
  String section = title + " | " + var_name + " {" + var_val + "}";
  log_section_start(section);
}

// ----- Section End -----
void log_section_end() {
  if (!logging_on) return;

  if (log_section_index < 1) {
    safe_println("");
    return;
  }

  log_section_index--;
  unsigned long now = millis();
  unsigned long start_time = log_section_start_time[log_section_index];
  unsigned long delta = now - start_time;

  String section_name = log_section_name[log_section_index];
  String t_now = format_time(now);
  String t_delta = format_time(delta);

  log_indent();
  String msg = "---- {" + section_name + "} end <" + t_now + ", Δ" + t_delta + "> ----";
  safe_println(msg.c_str());
}

// ----- Reset -----
void log_indent_reset() {
  log_section_index = 0;
}
