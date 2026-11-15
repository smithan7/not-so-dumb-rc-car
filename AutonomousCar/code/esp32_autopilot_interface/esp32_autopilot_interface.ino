/*
ESP32 autopilot interface: car-side firmware between the RC receiver ("Steve"), the Jetson
Xavier NX ("Xavier"), the steering servo / ESC, and the driveshaft hall sensor.
Host side: smithan7/ROS_nsd_rc_car catkin_ws/src/esp32_interface/scripts/esp32_node.py
(protocol v2 - needs the matching esp32_node / esp32_protocol.py).

Who drives, and every failsafe decision, is in failsafe.h (unit-tested on a PC:
AutonomousCar/code/tests/test_esp32_failsafe.py). This file is only the hardware around it.

Protocol v2, SERIAL_BAUD
  Jetson -> ESP32   C,<seq>,<steer_us>,<speed_us>,<(seq+steer+speed)%1000>\n
  ESP32 -> Jetson, immediately after each accepted command (timestamp echo for latency):
                    A,<seq>,<rx_us>,<mode>,<out_speed_us>,<out_steer_us>,$
  ESP32 -> Jetson, every STATUS_PERIOD_MS:
                    S,<now_us>,<mode>,<armed>,<rc_age_ms>,<jetson_age_ms>,<rc_ch3_us>,
                      <rc_speed_us>,<rc_steer_us>,<jetson_seq>,<jetson_speed_us>,
                      <jetson_steer_us>,<out_speed_us>,<out_steer_us>,<hall_count>,
                      <hall_last_edge_us>,<hall_dt_us>,<bad_cmds>,$
  *_us times are this ESP32's micros() (wraps every ~71.6 min); ages are -1 if never seen.
  out_* is what is written to the servo/ESC (after clamping/failsafe); it reaches the
  actuators at their next PWM frame (0-20 ms at SERVO_HZ = 50).
  Modes: 0 manual, 1 auto, -1 no receiver, -2 no Jetson, -3 auto not armed (see failsafe.h).

History: imported 2026-11 from the local Arduino folder (esp32_autopilot_interace.ino, never in
git), then rewritten to fix: the Jetson timeout never firing; a 20 s receiver-loss timeout;
"brake" that was neutral; no device timestamps; 10-bit PWM output (~20 us throttle steps);
ISR data shared without critical sections; unframed serial input.

BENCH CHECKS before driving (wheels off the ground):
  - receiver failsafe: switch the transmitter off in MANUAL and in AUTO; mode must go to -1 (or
    -2/-3) and the throttle to neutral/brake. If the receiver keeps outputting pulses with ch3
    low on signal loss, the ESP32 cannot tell - set the receiver failsafe so it does not.
  - pins / channels below, and that brake_us (failsafe.h) brakes rather than reverses.
*/

#include <ESP32Servo.h>

#include "failsafe.h"

// ---- wiring (unchanged from the original sketch) ----
#define HE_Sensor_Pin 21
#define SpeedPinIn 19
#define SteerPinIn 18
#define CmdPinIn 17  // receiver ch3: > 1500 us = human has control
#define SpeedPinOut 16
#define SteerPinOut 4

const unsigned long SERIAL_BAUD = 460800;   // 115200 was ~65% busy at these line rates
const uint32_t STATUS_PERIOD_MS = 20;       // 50 Hz status
const int SERVO_HZ = 50;                    // output frame rate (receiver itself runs ~180 Hz)
const uint32_t PWM_FRAME_US = 5550;         // receiver frame period (new receiver)
const uint32_t PWM_FRAME_TOL_US = 100;
const uint32_t PULSE_MIN_US = 900, PULSE_MAX_US = 2100;
const uint32_t HALL_MIN_DT_US = 300;        // debounce: faster than ~3300 driveshaft rev/s is noise

car::Config cfg;
car::Arbiter arbiter(cfg);
Servo steer_servo;
Servo speed_servo;

// ---- ISR state (guarded by mux) ----
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

struct PulseChannel {
  uint8_t pin;
  uint32_t rise_us;
  bool good_rise;
  int width_us;
  uint32_t last_valid_ms;
  bool seen;
};
volatile PulseChannel ch_speed = {SpeedPinIn, 0, false, 1500, 0, false};
volatile PulseChannel ch_steer = {SteerPinIn, 0, false, 1500, 0, false};
volatile PulseChannel ch_cmd = {CmdPinIn, 0, false, 0, 0, false};

volatile uint32_t hall_count = 0;
volatile uint32_t hall_last_us = 0;
volatile uint32_t hall_last_ms = 0;
volatile uint32_t hall_dt_us = 0;

static inline bool IRAM_ATTR pin_high(uint8_t pin) { return (REG_READ(GPIO_IN_REG) >> pin) & 1; }

static void IRAM_ATTR pulse_edge(volatile PulseChannel& ch) {
  const uint32_t now = micros();
  portENTER_CRITICAL_ISR(&mux);
  if (pin_high(ch.pin)) {
    const uint32_t period = now - ch.rise_us;
    ch.good_rise = (period > PWM_FRAME_US - PWM_FRAME_TOL_US) && (period < PWM_FRAME_US + PWM_FRAME_TOL_US);
    ch.rise_us = now;
  } else {
    const uint32_t w = now - ch.rise_us;
    if (ch.good_rise && w >= PULSE_MIN_US && w <= PULSE_MAX_US) {
      ch.width_us = (int)w;
      ch.last_valid_ms = millis();
      ch.seen = true;
    }
  }
  portEXIT_CRITICAL_ISR(&mux);
}
void IRAM_ATTR speed_isr() { pulse_edge(ch_speed); }
void IRAM_ATTR steer_isr() { pulse_edge(ch_steer); }
void IRAM_ATTR cmd_isr() { pulse_edge(ch_cmd); }

void IRAM_ATTR hall_isr() {
  const uint32_t now = micros();
  portENTER_CRITICAL_ISR(&mux);
  const uint32_t dt = now - hall_last_us;
  if (hall_count == 0 || dt >= HALL_MIN_DT_US) {
    hall_dt_us = hall_count == 0 ? 0 : dt;
    hall_last_us = now;
    hall_last_ms = millis();
    hall_count = hall_count + 1;
  }
  portEXIT_CRITICAL_ISR(&mux);
}

// ---- Jetson command state (loop only) ----
bool jetson_seen = false;
uint32_t jetson_last_ms = 0;
uint32_t jetson_seq = 0;
int jetson_speed_us = 1500, jetson_steer_us = 1500;
uint32_t bad_cmds = 0;

char line_buf[64];
size_t line_len = 0;
bool line_overflow = false;

// Read whatever has arrived; returns true (and fills ack_*) when a valid command was accepted.
bool read_commands(uint32_t* ack_seq, uint32_t* ack_rx_us) {
  bool accepted = false;
  while (Serial.available()) {
    const char c = (char)Serial.read();
    if (c != '\n') {
      if (line_len < sizeof(line_buf) - 1) {
        line_buf[line_len++] = c;
      } else {
        line_overflow = true;
      }
      continue;
    }
    line_buf[line_len] = '\0';
    car::Command cmd;
    if (!line_overflow && car::parse_command(line_buf, &cmd)) {
      jetson_seen = true;
      jetson_last_ms = millis();
      jetson_seq = cmd.seq;
      jetson_speed_us = cmd.speed_us;
      jetson_steer_us = cmd.steer_us;
      *ack_seq = cmd.seq;
      *ack_rx_us = micros();
      accepted = true;
    } else if (line_len > 0) {
      bad_cmds++;
    }
    line_len = 0;
    line_overflow = false;
  }
  return accepted;
}

static long age_ms(bool seen, uint32_t last_ms, uint32_t now_ms) {
  return seen ? (long)(uint32_t)(now_ms - last_ms) : -1L;
}

void setup() {
  Serial.setTxBufferSize(1024);  // a status line never blocks the control loop
  Serial.begin(SERIAL_BAUD);
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  speed_servo.setPeriodHertz(SERVO_HZ);
  steer_servo.setPeriodHertz(SERVO_HZ);
  speed_servo.attach(SpeedPinOut, 900, 2100);
  steer_servo.attach(SteerPinOut, 900, 2100);
  // ESP32Servo's default 10-bit timer = ~20 us steps at 50 Hz; 16 bits = ~0.3 us. Must come
  // after attach() (which resets the width), and must be followed by writeMicroseconds():
  // the library rescales the current pulse wrongly on a width change (ESP32Servo 3.2.1).
  speed_servo.setTimerWidth(16);
  steer_servo.setTimerWidth(16);
  speed_servo.writeMicroseconds(cfg.neutral_us);
  steer_servo.writeMicroseconds(cfg.steer_center_us);

  pinMode(SpeedPinIn, INPUT);
  pinMode(SteerPinIn, INPUT);
  pinMode(CmdPinIn, INPUT);
  pinMode(HE_Sensor_Pin, INPUT);
  attachInterrupt(digitalPinToInterrupt(SpeedPinIn), speed_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(SteerPinIn), steer_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CmdPinIn), cmd_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(HE_Sensor_Pin), hall_isr, RISING);
}

void loop() {
  static uint32_t last_status_ms = 0;
  static int written_speed = -1, written_steer = -1;

  uint32_t ack_seq = 0, ack_rx_us = 0;
  const bool ack = read_commands(&ack_seq, &ack_rx_us);

  // snapshot the ISR state
  car::Inputs in;
  uint32_t h_count, h_last_us, h_dt;
  int ch3;
  portENTER_CRITICAL(&mux);
  const uint32_t now_ms = millis();
  in.now_ms = now_ms;
  in.rc_seen = ch_speed.seen && ch_steer.seen && ch_cmd.seen;
  // the receiver is only as fresh as its stalest channel
  uint32_t oldest = ch_speed.last_valid_ms;
  if ((uint32_t)(now_ms - ch_steer.last_valid_ms) > (uint32_t)(now_ms - oldest)) oldest = ch_steer.last_valid_ms;
  if ((uint32_t)(now_ms - ch_cmd.last_valid_ms) > (uint32_t)(now_ms - oldest)) oldest = ch_cmd.last_valid_ms;
  in.rc_last_ms = oldest;
  in.rc_speed_us = ch_speed.width_us;
  in.rc_steer_us = ch_steer.width_us;
  ch3 = ch_cmd.width_us;
  in.hall_seen = hall_count > 0;
  in.hall_last_edge_ms = hall_last_ms;
  h_count = hall_count;
  h_last_us = hall_last_us;
  h_dt = hall_dt_us;
  portEXIT_CRITICAL(&mux);
  in.rc_ch3_us = ch3;
  in.jetson_seen = jetson_seen;
  in.jetson_last_ms = jetson_last_ms;
  in.jetson_speed_us = jetson_speed_us;
  in.jetson_steer_us = jetson_steer_us;

  const car::Output out = arbiter.update(in);
  if (out.speed_us != written_speed) {
    speed_servo.writeMicroseconds(out.speed_us);
    written_speed = out.speed_us;
  }
  if (out.steer_us != written_steer) {
    steer_servo.writeMicroseconds(out.steer_us);
    written_steer = out.steer_us;
  }

  if (ack) {
    Serial.printf("A,%lu,%lu,%d,%d,%d,$\n", (unsigned long)ack_seq, (unsigned long)ack_rx_us, out.mode,
                  out.speed_us, out.steer_us);
  }
  if ((uint32_t)(now_ms - last_status_ms) >= STATUS_PERIOD_MS) {
    last_status_ms = now_ms;
    Serial.printf("S,%lu,%d,%d,%ld,%ld,%d,%d,%d,%lu,%d,%d,%d,%d,%lu,%lu,%lu,%lu,$\n",
                  (unsigned long)micros(), out.mode, out.armed ? 1 : 0,
                  age_ms(in.rc_seen, in.rc_last_ms, now_ms), age_ms(jetson_seen, jetson_last_ms, now_ms),
                  ch3, in.rc_speed_us, in.rc_steer_us, (unsigned long)jetson_seq, jetson_speed_us,
                  jetson_steer_us, out.speed_us, out.steer_us, (unsigned long)h_count,
                  (unsigned long)h_last_us, (unsigned long)h_dt, (unsigned long)bad_cmds);
  }
  delay(1);
}
