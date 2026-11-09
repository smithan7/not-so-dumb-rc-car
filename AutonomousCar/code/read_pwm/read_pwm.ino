/*
Reading Values from an RC Receiver using Arduino
Based on Brandon Tsuge's example:
https://www.theboredrobot.com/post/reading-values-from-an-rc-receiver-using-arduino

Measures 50 Hz RC PWM pulse widths with pin-change interrupts and reports them over serial:
  "PWM, 1, <throttle_us>, <steer_us>"  - a fresh pair of pulses
  "PWM, 0, -1, -1"                     - no valid pulse on a channel for SIGNAL_TIMEOUT_MS
*/

#define ThrottlePin 2
#define SteerPin 3

const unsigned long SIGNAL_TIMEOUT_MS = 100;   // ~5 missed frames at 50 Hz
const unsigned long REPORT_PERIOD_MS = 50;     // 20 Hz output, same rate as before
const unsigned long MIN_PULSE_US = 900;
const unsigned long MAX_PULSE_US = 2100;

// Written only inside the ISRs
unsigned long ThrottleRiseTime = 0;
unsigned long SteerRiseTime = 0;

// Shared between ISRs and loop(): must be volatile
volatile unsigned int ThrottlePulseWidth = 0;
volatile unsigned int SteerPulseWidth = 0;
volatile bool newThrottle = false;
volatile bool newSteer = false;
volatile unsigned long ThrottleLastValidMs = 0;
volatile unsigned long SteerLastValidMs = 0;

unsigned long lastReportMs = 0;

void setup() {
  Serial.begin(115200);
  pinMode(ThrottlePin, INPUT_PULLUP);
  pinMode(SteerPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ThrottlePin), ThrottlePulseTimer, CHANGE);
  attachInterrupt(digitalPinToInterrupt(SteerPin), SteerPulseTimer, CHANGE);
}

void loop() {
  unsigned long now = millis();
  if (now - lastReportMs < REPORT_PERIOD_MS) {
    return;
  }
  lastReportMs = now;

  // Copy shared values atomically (multi-byte reads can tear on 8-bit AVR boards)
  noInterrupts();
  unsigned int throttle = ThrottlePulseWidth;
  unsigned int steer = SteerPulseWidth;
  bool haveThrottle = newThrottle;
  bool haveSteer = newSteer;
  unsigned long throttleAge = now - ThrottleLastValidMs;
  unsigned long steerAge = now - SteerLastValidMs;
  interrupts();

  if (throttleAge > SIGNAL_TIMEOUT_MS || steerAge > SIGNAL_TIMEOUT_MS) {
    // real signal loss: no valid pulse recently. Keep looping (no blocking delay) so recovery is immediate.
    Serial.println("PWM, 0, -1, -1");
    return;
  }

  if (haveThrottle && haveSteer) {
    noInterrupts();
    newThrottle = false;
    newSteer = false;
    interrupts();

    Serial.print("PWM, 1, ");
    Serial.print(throttle);
    Serial.print(", ");
    Serial.println(steer);
  }
}

// Time each high pulse by reading the pin level, rather than assuming edges alternate.
void ThrottlePulseTimer() {
  unsigned long t = micros();
  if (digitalRead(ThrottlePin) == HIGH) {
    ThrottleRiseTime = t;
  } else {
    unsigned long width = t - ThrottleRiseTime;  // unsigned math is correct across micros() rollover
    if (width >= MIN_PULSE_US && width <= MAX_PULSE_US) {
      ThrottlePulseWidth = (unsigned int)width;
      newThrottle = true;
      ThrottleLastValidMs = millis();
    }
  }
}

void SteerPulseTimer() {
  unsigned long t = micros();
  if (digitalRead(SteerPin) == HIGH) {
    SteerRiseTime = t;
  } else {
    unsigned long width = t - SteerRiseTime;
    if (width >= MIN_PULSE_US && width <= MAX_PULSE_US) {
      SteerPulseWidth = (unsigned int)width;
      newSteer = true;
      SteerLastValidMs = millis();
    }
  }
}
