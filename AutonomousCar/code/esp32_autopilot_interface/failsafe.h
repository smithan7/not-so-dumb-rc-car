// Control arbitration and failsafe for the ESP32 autopilot interface.
//
// Pure C++ (no Arduino headers) so it is unit-tested on a PC: test/test_failsafe.cpp.
// The sketch feeds it timestamps and pulse widths each loop and writes out what it returns.
//
// Modes
//   MANUAL  (0)  receiver ch3 > ch3_threshold_us: receiver steering/throttle passed through
//   AUTO    (1)  ch3 low, Jetson commands fresh, and armed: Jetson commands, clamped
//   failsafe (< 0): steering centred, throttle braked or neutral (below)
//     NO_RC      (-1)  no valid receiver pulses for rc_timeout_ms (or never)
//     NO_JETSON  (-2)  AUTO selected but no valid Jetson command for jetson_timeout_ms
//     NOT_ARMED  (-3)  AUTO selected, Jetson fine, but not armed
// Arming: AUTO is only allowed after the receiver has been seen in MANUAL since the last
// failsafe. So a Jetson that comes back, or a car powered up with the switch in AUTO, does not
// drive off on its own: the operator must flip to MANUAL and back.
//
// Failsafe throttle: brake (brake_us) only if the car is moving (hall edge within
// hall_moving_ms) AND the last throttle output was not reverse when the failsafe started; hold
// it for at least brake_min_ms, until the car stops, and at most brake_max_ms. Then neutral for
// the rest of the failsafe - never a second brake, since brake-from-standstill after neutral is
// reverse on this kind of ESC. If the hall sensor is dead the car is treated as stopped and
// simply coasts (neutral).
#pragma once
#include <stdint.h>

namespace car {

enum Mode {
  MODE_MANUAL = 0,
  MODE_AUTO = 1,
  MODE_FAILSAFE_NO_RC = -1,
  MODE_FAILSAFE_NO_JETSON = -2,
  MODE_NOT_ARMED = -3,
};

struct Config {
  int neutral_us = 1500;
  int steer_center_us = 1500;
  int ch3_threshold_us = 1500;  // ch3 above this = human has control
  int brake_us = 1400;          // failsafe brake (below neutral)
  // SYSID STAGE LIMITS - Jetson commands are clamped to these. Start low on the 6S car and
  // raise one stage at a time (reflash) as the fitted model says the next level is safe.
  int auto_speed_min_us = 1350;
  int auto_speed_max_us = 1600;
  int auto_steer_min_us = 1300;
  int auto_steer_max_us = 1700;
  uint32_t rc_timeout_ms = 100;      // receiver frames are ~5.5 ms
  uint32_t jetson_timeout_ms = 200;  // Jetson commands at 50 Hz (>= 5 Hz required)
  uint32_t hall_moving_ms = 300;     // a driveshaft edge within this = moving
  uint32_t brake_min_ms = 1000;
  uint32_t brake_max_ms = 3000;
};

struct Inputs {
  uint32_t now_ms = 0;
  bool rc_seen = false;  // a valid pulse set has arrived at least once
  uint32_t rc_last_ms = 0;
  int rc_ch3_us = 0, rc_speed_us = 1500, rc_steer_us = 1500;
  bool jetson_seen = false;
  uint32_t jetson_last_ms = 0;
  int jetson_speed_us = 1500, jetson_steer_us = 1500;
  bool hall_seen = false;
  uint32_t hall_last_edge_ms = 0;
};

struct Output {
  int mode;
  int speed_us;
  int steer_us;
  bool armed;
};

struct Command {
  uint32_t seq;
  int steer_us;
  int speed_us;
};

inline int clamp_int(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); }

// fresh = seen, and less than timeout old. Unsigned subtraction is millis()-wraparound safe.
inline bool fresh(bool seen, uint32_t last_ms, uint32_t now_ms, uint32_t timeout_ms) {
  return seen && (uint32_t)(now_ms - last_ms) < timeout_ms;
}

class Arbiter {
 public:
  explicit Arbiter(const Config& cfg) : cfg_(cfg) {}

  Output update(const Inputs& in) {
    const bool rc_ok = fresh(in.rc_seen, in.rc_last_ms, in.now_ms, cfg_.rc_timeout_ms);
    const bool jetson_ok = fresh(in.jetson_seen, in.jetson_last_ms, in.now_ms, cfg_.jetson_timeout_ms);
    const bool moving = fresh(in.hall_seen, in.hall_last_edge_ms, in.now_ms, cfg_.hall_moving_ms);

    int mode;
    if (!rc_ok) {
      mode = MODE_FAILSAFE_NO_RC;
    } else if (in.rc_ch3_us > cfg_.ch3_threshold_us) {
      mode = MODE_MANUAL;
    } else if (!jetson_ok) {
      mode = MODE_FAILSAFE_NO_JETSON;
    } else if (!armed_) {
      mode = MODE_NOT_ARMED;
    } else {
      mode = MODE_AUTO;
    }

    Output out;
    out.mode = mode;
    if (mode == MODE_MANUAL) {
      armed_ = true;
      in_failsafe_ = false;
      out.speed_us = in.rc_speed_us;
      out.steer_us = in.rc_steer_us;
    } else if (mode == MODE_AUTO) {
      in_failsafe_ = false;
      out.speed_us = clamp_int(in.jetson_speed_us, cfg_.auto_speed_min_us, cfg_.auto_speed_max_us);
      out.steer_us = clamp_int(in.jetson_steer_us, cfg_.auto_steer_min_us, cfg_.auto_steer_max_us);
    } else {
      armed_ = false;
      if (!in_failsafe_) {  // a new failsafe episode
        in_failsafe_ = true;
        failsafe_start_ms_ = in.now_ms;
        brake_allowed_ = moving && last_speed_us_ >= cfg_.neutral_us;
      }
      const uint32_t elapsed = in.now_ms - failsafe_start_ms_;
      if (brake_allowed_ && elapsed < cfg_.brake_max_ms && (elapsed < cfg_.brake_min_ms || moving)) {
        out.speed_us = cfg_.brake_us;
      } else {
        brake_allowed_ = false;  // latched: no second brake in this episode
        out.speed_us = cfg_.neutral_us;
      }
      out.steer_us = cfg_.steer_center_us;
    }
    out.armed = armed_;
    last_speed_us_ = out.speed_us;
    return out;
  }

 private:
  Config cfg_;
  bool armed_ = false;
  bool in_failsafe_ = false;
  bool brake_allowed_ = false;
  uint32_t failsafe_start_ms_ = 0;
  int last_speed_us_ = 1500;
};

// Parse one unsigned decimal field ending at ',' or end of string; advances *p.
inline bool parse_uint_field(const char** p, uint32_t* out) {
  const char* s = *p;
  uint32_t v = 0;
  int digits = 0;
  while (*s >= '0' && *s <= '9') {
    if (++digits > 9) return false;
    v = v * 10 + (uint32_t)(*s - '0');
    s++;
  }
  if (digits == 0) return false;
  *out = v;
  *p = s;
  return true;
}

// Jetson -> ESP32 command: "C,<seq>,<steer_us>,<speed_us>,<(seq+steer+speed)%1000>" with an
// optional trailing CR/LF. Strict: any malformed or out-of-range field rejects the whole line.
inline bool parse_command(const char* line, Command* cmd) {
  if (line[0] != 'C' || line[1] != ',') return false;
  const char* p = line + 2;
  uint32_t f[4];
  for (int i = 0; i < 4; i++) {
    if (!parse_uint_field(&p, &f[i])) return false;
    if (i < 3) {
      if (*p != ',') return false;
      p++;
    }
  }
  while (*p == '\r' || *p == '\n') p++;
  if (*p != '\0') return false;
  const uint32_t seq = f[0], steer = f[1], speed = f[2], checksum = f[3];
  if (steer < 900 || steer > 2100 || speed < 900 || speed > 2100) return false;
  if ((seq + steer + speed) % 1000 != checksum) return false;
  cmd->seq = seq;
  cmd->steer_us = (int)steer;
  cmd->speed_us = (int)speed;
  return true;
}

}  // namespace car
