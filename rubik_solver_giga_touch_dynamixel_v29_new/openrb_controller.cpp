// -------------------------------------------------------------------
//                      COMMAND TABLE
// ------------------ -------------------------------------------------

struct CommandEntry {
  const char* name;
  const char* fmt;
  bool (*handler)(int argc, double* argv);
  const char* desc;
};

static CommandEntry command_table[] = {
  { "VERBOSEON", "", cmd_verbose_on, "VERBOSEON - enable verbose output" },
  { "VERBOSEOFF", "", cmd_verbose_off, "VERBOSEOFF - disable verbose output" },

  { "SETMIN", "%d", cmd_set_min, "SETMIN <id> - set current pos as min" },
  { "SETMAX", "%d", cmd_set_max, "SETMAX <id> - set current pos as max" },
  { "SETZERO", "%d", cmd_set_zero, "SETLIMITMAX <id> - set current pos as zero" },
  { "SETDIRPLUS", "%d", cmd_set_dir_plus, "SETDIRPLUS <id> - set dir using the current pos > zero" },
  { "SETDIRMINUS", "%d", cmd_set_dir_minus, "SETDIRMINUS <id> - set dir using the current pos < zero" },

  { "POSZERO", "", cmd_pos_zero, "POSZERO - move to pos zero" },
  { "MOVETICKS", "%d %d", cmd_move_ticks, "MOVEDEG <id> <ticks goal> - move one servo to ticks (not smooth)" },
  { "MOVEDEG", "%d %f", cmd_move_deg, "MOVEDEG <id> <deg goal> - move one servo to degree (smooth)" },
  { "MOVEPER", "%d %f", cmd_move_per, "MOVEPER <id> <per goal> - move one servo to percent (smooth)" },
  { "MOVEYMM", "%f", cmd_move_y, "MOVEYMM <float mm> - vertical move" },
  { "MOVEXMM", "%f", cmd_move_x, "MOVEXMM <float mm> - lateral move" },
  { "MOVEXYMM", "%f %f", cmd_move_xy, "MOVEXYMM <float mm> <float mm> - lateral then vertical move" },
  { "MOVEGRIPPER", "%f", cmd_move_gripper, "MOVEGRIPPER <percentage> - move both grips to percentage" },
  { "MOVEWRISTVERTDEG", "%f", cmd_move_wrist_vert, "MOVEWRISTVERTDEG <deg> - move wrist relative to vertical" },

  { "READ", "%d", cmd_read, "READ <id> - show servo summary status" },
  { "INFO", "%d", cmd_info, "INFO <id> - show servo full status" },

  { "LEDON", "%d", cmd_ledon, "LEDON <id> - turn servo LED on" },
  { "LEDOFF", "%d", cmd_ledoff, "LEDOFF <id> - turn servo LED off" },
};

/*
VERBOSEOFF START x_mm=3.10 y_mm=45.29 a1_deg=68.55 a2_deg=-39.29 g_vert_deg=0.79 g1_per=30.12 g2_per=30.12 base_deg=-39.29
Verbose OFF
VERBOSEOFF END completed=1 x_mm=3.10 y_mm=45.29 a1_deg=68.55 a2_deg=-39.29 g_vert_deg=0.79 g1_per=30.12 g2_per=30.12 base_deg=-39.29
*/

bool begin();

bool runCommand(char* name, float* arguments);
bool servoInfoDeg(int id, int* mn, int* mx, int* pos);

bool xyInfoMm(double* x, double* y);
bool gipperInfoPer(double* g);
bool baseInfoDeg(double* b);
bool wristVertInfoDeg(double* v);

bool moveXmm(double x);
bool moveXmm(double y);
bool moveBaseDeg(double deg);
bool moveWristVertDeg(double deg);
bool moveGrippersPer(double per);

char* getLastError();
char* getAllErrors();
