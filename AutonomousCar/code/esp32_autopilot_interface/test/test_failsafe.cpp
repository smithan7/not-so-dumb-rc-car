// Host-side unit tests for failsafe.h (no Arduino needed):
//   c++ -std=c++11 -Wall -Wextra -I.. test_failsafe.cpp -o /tmp/t && /tmp/t
// Run automatically by AutonomousCar/code/tests/test_esp32_failsafe.py.
#include <cstdio>
#include <cstdint>

#include "../failsafe.h"

using namespace car;

static int failures = 0;
#define CHECK(cond)                                                         \
  do {                                                                      \
    if (!(cond)) {                                                          \
      std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);           \
      failures++;                                                           \
    }                                                                       \
  } while (0)
#define CHECK_EQ(a, b)                                                      \
  do {                                                                      \
    long _a = (long)(a), _b = (long)(b);                                    \
    if (_a != _b) {                                                         \
      std::printf("FAIL %s:%d: %s == %ld, expected %ld\n", __FILE__,        \
                  __LINE__, #a, _a, _b);                                    \
      failures++;                                                           \
    }                                                                       \
  } while (0)

// A test bench: one Arbiter plus the inputs the sketch would feed it.
struct Bench {
  Config cfg;
  Arbiter arb;
  Inputs in;
  Bench() : arb(cfg) {}

  void rc(uint32_t t, int ch3, int speed = 1500, int steer = 1500) {
    in.rc_seen = true;
    in.rc_last_ms = t;
    in.rc_ch3_us = ch3;
    in.rc_speed_us = speed;
    in.rc_steer_us = steer;
  }
  void jetson(uint32_t t, int speed, int steer = 1500) {
    in.jetson_seen = true;
    in.jetson_last_ms = t;
    in.jetson_speed_us = speed;
    in.jetson_steer_us = steer;
  }
  void hall(uint32_t t) {
    in.hall_seen = true;
    in.hall_last_edge_ms = t;
  }
  Output at(uint32_t t) {
    in.now_ms = t;
    return arb.update(in);
  }
  // manual (ch3 high) then hand over to the Jetson (ch3 low) -> armed AUTO
  void arm_and_go_auto(uint32_t t, int speed = 1580) {
    rc(t, 1900);
    at(t);
    rc(t + 10, 1100);
    jetson(t + 10, speed);
    at(t + 10);
  }
};

static void test_boot_without_rc_is_failsafe_neutral() {
  Bench b;
  Output o = b.at(50);  // timestamps start at 0: must not look "fresh"
  CHECK_EQ(o.mode, MODE_FAILSAFE_NO_RC);
  CHECK_EQ(o.speed_us, 1500);
  CHECK_EQ(o.steer_us, 1500);
  CHECK(!o.armed);
}

static void test_manual_passes_rc_through_unclamped() {
  Bench b;
  b.rc(1000, 1900, 1850, 1200);
  Output o = b.at(1005);
  CHECK_EQ(o.mode, MODE_MANUAL);
  CHECK_EQ(o.speed_us, 1850);
  CHECK_EQ(o.steer_us, 1200);
}

static void test_rc_timeout() {
  Bench b;
  b.rc(1000, 1900, 1600);
  CHECK_EQ(b.at(1099).mode, MODE_MANUAL);
  CHECK_EQ(b.at(1101).mode, MODE_FAILSAFE_NO_RC);
}

static void test_auto_requires_arming_in_manual_first() {
  Bench b;
  b.rc(1000, 1100);  // switch already in Jetson position at power-up
  b.jetson(1000, 1580);
  Output o = b.at(1000);
  CHECK_EQ(o.mode, MODE_NOT_ARMED);
  CHECK_EQ(o.speed_us, 1500);
}

static void test_auto_clamps_jetson_commands() {
  Bench b;
  b.arm_and_go_auto(1000);
  b.jetson(1020, 1900, 1000);
  b.rc(1020, 1100);
  Output o = b.at(1020);
  CHECK_EQ(o.mode, MODE_AUTO);
  CHECK(o.armed);
  CHECK_EQ(o.speed_us, b.cfg.auto_speed_max_us);
  CHECK_EQ(o.steer_us, b.cfg.auto_steer_min_us);
  b.jetson(1030, 1000, 2000);
  o = b.at(1030);
  CHECK_EQ(o.speed_us, b.cfg.auto_speed_min_us);
  CHECK_EQ(o.steer_us, b.cfg.auto_steer_max_us);
}

static void test_jetson_timeout_boundary() {
  Bench b;
  b.arm_and_go_auto(1000);  // last Jetson command at 1010
  b.rc(1200, 1100);
  CHECK_EQ(b.at(1209).mode, MODE_AUTO);
  CHECK_EQ(b.at(1211).mode, MODE_FAILSAFE_NO_JETSON);
}

static void test_jetson_loss_while_moving_brakes_then_neutral() {
  Bench b;
  b.arm_and_go_auto(1000, 1580);
  // car moving: hall edges every 20 ms, Jetson commands keep coming until 2000
  for (uint32_t t = 1020; t <= 2000; t += 20) {
    b.hall(t);
    b.rc(t, 1100);
    b.jetson(t, 1580);
    CHECK_EQ(b.at(t).mode, MODE_AUTO);
  }
  // Jetson goes silent at 2000; failsafe starts at 2201
  uint32_t t0 = 2201;
  for (uint32_t t = 2020; t < t0; t += 20) {
    b.hall(t);
    b.rc(t, 1100);
    b.at(t);
  }
  b.rc(t0, 1100);
  b.hall(t0);
  Output o = b.at(t0);
  CHECK_EQ(o.mode, MODE_FAILSAFE_NO_JETSON);
  CHECK_EQ(o.speed_us, b.cfg.brake_us);
  CHECK_EQ(o.steer_us, b.cfg.steer_center_us);
  // still moving 1.5 s later: still braking
  b.hall(t0 + 1500);
  b.rc(t0 + 1500, 1100);
  CHECK_EQ(b.at(t0 + 1500).speed_us, b.cfg.brake_us);
  // stopped (no edge for > hall_moving_ms) and past brake_min_ms -> neutral
  b.rc(t0 + 1900, 1100);
  CHECK_EQ(b.at(t0 + 1900).speed_us, 1500);
  // and it stays neutral even if the hall sensor twitches (no second brake = no reverse)
  b.hall(t0 + 1950);
  b.rc(t0 + 1950, 1100);
  CHECK_EQ(b.at(t0 + 1950).speed_us, 1500);
}

static void test_brake_holds_for_min_time_even_if_stopped_quickly() {
  Bench b;
  b.arm_and_go_auto(1000);
  b.hall(1010);
  b.rc(1211, 1100);
  Output o = b.at(1211);  // Jetson lost; hall edge 201 ms ago -> still "moving"
  CHECK_EQ(o.speed_us, b.cfg.brake_us);
  b.rc(1600, 1100);
  CHECK_EQ(b.at(1600).speed_us, b.cfg.brake_us);  // stopped, but < brake_min_ms
  b.rc(1211 + b.cfg.brake_min_ms + 1, 1100);
  CHECK_EQ(b.at(1211 + b.cfg.brake_min_ms + 1).speed_us, 1500);
}

static void test_brake_max_time_even_if_still_moving() {
  Bench b;
  b.arm_and_go_auto(1000);
  uint32_t t0 = 1211;
  for (uint32_t t = 1020; t < t0 + b.cfg.brake_max_ms + 100; t += 20) {
    b.hall(t);
    b.rc(t, 1100);
    Output o = b.at(t);
    if (t >= t0 && t < t0 + b.cfg.brake_max_ms) CHECK_EQ(o.speed_us, b.cfg.brake_us);
    if (t > t0 + b.cfg.brake_max_ms) CHECK_EQ(o.speed_us, 1500);
  }
}

static void test_failsafe_when_stationary_never_brakes() {
  // a brake command from standstill can be taken as reverse by the ESC
  Bench b;
  b.arm_and_go_auto(1000, 1500);  // armed, Jetson holding neutral, car never moved
  b.rc(1300, 1100);
  Output o = b.at(1300);
  CHECK_EQ(o.mode, MODE_FAILSAFE_NO_JETSON);
  CHECK_EQ(o.speed_us, 1500);
}

static void test_failsafe_after_reverse_command_does_not_brake() {
  // if the car was reversing, "brake" (< neutral) would drive it backwards harder
  Bench b;
  b.rc(1000, 1900, 1300);  // manual, reversing
  b.hall(1000);
  CHECK_EQ(b.at(1000).speed_us, 1300);
  b.hall(1150);
  Output o = b.at(1150);  // RC lost at 1101
  CHECK_EQ(o.mode, MODE_FAILSAFE_NO_RC);
  CHECK_EQ(o.speed_us, 1500);
}

static void test_rc_loss_in_manual_brakes_when_moving_forward() {
  Bench b;
  b.rc(1000, 1900, 1700);
  b.hall(1000);
  CHECK_EQ(b.at(1000).speed_us, 1700);
  b.hall(1150);
  CHECK_EQ(b.at(1150).speed_us, b.cfg.brake_us);
}

static void test_jetson_return_after_failsafe_needs_rearm() {
  Bench b;
  b.arm_and_go_auto(1000);
  b.rc(1300, 1100);
  CHECK_EQ(b.at(1300).mode, MODE_FAILSAFE_NO_JETSON);
  b.rc(1310, 1100);
  b.jetson(1310, 1580);  // Jetson back
  Output o = b.at(1310);
  CHECK_EQ(o.mode, MODE_NOT_ARMED);
  CHECK_EQ(o.speed_us, 1500);
  // operator flips to manual and back -> auto again
  b.rc(1320, 1900);
  CHECK_EQ(b.at(1320).mode, MODE_MANUAL);
  b.rc(1330, 1100);
  b.jetson(1330, 1580);
  CHECK_EQ(b.at(1330).mode, MODE_AUTO);
}

static void test_millis_wraparound() {
  Bench b;
  uint32_t t = 0xFFFFFFF0u;
  b.rc(t, 1900, 1600);
  CHECK_EQ(b.at(t + 40).mode, MODE_MANUAL);  // now has wrapped past 0
  CHECK_EQ(b.at(t + 40).speed_us, 1600);
}

static void test_parse_command() {
  Command c;
  CHECK(parse_command("C,7,1520,1580,107", &c));  // (7+1520+1580)%1000 = 107
  CHECK_EQ(c.seq, 7);
  CHECK_EQ(c.steer_us, 1520);
  CHECK_EQ(c.speed_us, 1580);
  CHECK(parse_command("C,7,1520,1580,107\r", &c));    // CR left over from CRLF
  CHECK(!parse_command("C,7,1520,1580,108", &c));     // bad checksum
  CHECK(!parse_command("C,7,1520,1580", &c));         // missing field
  CHECK(!parse_command("C,7,1520,1580,107,9", &c));   // extra field
  CHECK(!parse_command("C,7,15x0,1580,107", &c));     // junk in a number
  CHECK(!parse_command("C,7,800,1580,387", &c));      // steer out of range
  CHECK(!parse_command("C,7,1520,2200,727", &c));     // speed out of range
  CHECK(!parse_command("X,7,1520,1580,107", &c));     // wrong header
  CHECK(!parse_command("C,-1,1520,1580,99", &c));     // negative seq
  CHECK(!parse_command("", &c));
  CHECK(!parse_command("C,,1520,1580,100", &c));      // empty field
}

int main() {
  test_boot_without_rc_is_failsafe_neutral();
  test_manual_passes_rc_through_unclamped();
  test_rc_timeout();
  test_auto_requires_arming_in_manual_first();
  test_auto_clamps_jetson_commands();
  test_jetson_timeout_boundary();
  test_jetson_loss_while_moving_brakes_then_neutral();
  test_brake_holds_for_min_time_even_if_stopped_quickly();
  test_brake_max_time_even_if_still_moving();
  test_failsafe_when_stationary_never_brakes();
  test_failsafe_after_reverse_command_does_not_brake();
  test_rc_loss_in_manual_brakes_when_moving_forward();
  test_jetson_return_after_failsafe_needs_rearm();
  test_millis_wraparound();
  test_parse_command();
  if (failures) {
    std::printf("%d check(s) failed\n", failures);
    return 1;
  }
  std::printf("all failsafe tests passed\n");
  return 0;
}
