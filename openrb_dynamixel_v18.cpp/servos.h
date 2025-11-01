#pragma once
#include <Arduino.h>
#include <Dynamixel2Arduino.h>
#include "utils.h"
#include <initializer_list>
#include <vector>

// -------------------------------------------------------------------
//                           SERVO CONFIG
// -------------------------------------------------------------------

// ---------------- Constants ----------------
extern const int STALL_CURRENT_mA;
extern const int TEMP_LIMIT_C;

extern const float TICKS_PER_REV;
extern const float PV_UNIT_RPM;
extern const float TIME_SAFETY_FACTOR;
extern const uint32_t MIN_WAIT_MS;
extern const uint32_t EXTRA_SETTLE_MS;

extern const double TICKS_PER_DEG;
extern const double DEG_PER_TICK;
extern const double MM_PER_TICK;

extern const uint8_t SERVO_COUNT;

// -------------------------------------------------------------------
//                     ServoConfig class
// -------------------------------------------------------------------

class ServoConfig {
public:
  ServoConfig(const char *key,
              uint8_t id,
              uint16_t zero_ticks,
              double dir,
              uint16_t limit_min,
              uint16_t limit_max);

  uint8_t get_id() const;
  const char *get_key() const;
  uint16_t zero_ticks() const;
  uint16_t min_ticks() const;
  uint16_t max_ticks() const;
  double dir() const;

private:
  const char *key_;
  uint8_t id_;
  uint16_t zero_ticks_;
  double dir_;
  uint16_t limit_min_;
  uint16_t limit_max_;
};

// -------------------------------------------------------------------
//                     Servo global variables
// -------------------------------------------------------------------

extern Dynamixel2Arduino dxl;
extern ServoConfig *all_servos[];

// IDs
#define ID_ARM1 11
#define ID_ARM2 12
#define ID_GRIP 13
#define ID_GRIP1 14
#define ID_GRIP2 15
#define ID_BASE 16
#define ID_XM 17

// -------------------------------------------------------------------
//                     Servo helper functions
// -------------------------------------------------------------------

void enforce_servo_limits();

ServoConfig *find_servo(uint8_t id);
ServoConfig *find_servo(const char *name);

double ticks2deg(uint8_t id, int ticks);
int deg2ticks(uint8_t id, double deg);

int per2ticks(uint8_t id, double per);
double per2deg(uint8_t id, double per);
double ticks2per(uint8_t id, double ticks);

uint8_t name2id(const char *name);
const char *id2name(uint8_t id);

void lOn(uint8_t id);
void lOff(uint8_t id);

// ---------------- Group helpers ----------------
void torqueOnGroup(const std::vector<uint8_t> &ids);
void torqueOffGroup(const std::vector<uint8_t> &ids);
void adjustPwmGroup(const std::vector<uint8_t> &ids, int pwmLimit);
void ledOnGroup(const std::vector<uint8_t> &ids);
void ledOffGroup(const std::vector<uint8_t> &ids);
bool checkStallGroup(const std::vector<uint8_t> &ids);

// ---------------- Position helpers ----------------
double getPos_deg(int id);
void setGoal_deg(int id, double goal_deg);

// ---------------- Status helpers ----------------
bool isInPosition(uint8_t id);
bool isMoving(uint8_t id);
bool checkStall(uint8_t id);
void print_servo_status(uint8_t id);
