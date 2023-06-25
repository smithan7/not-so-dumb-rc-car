/*
Reading Values from an RC Receiver using Arduino
Brandon Tsuge
In this example, I demonstrate how to use Arduino to read RC (50Hz PWM) values using pulseIn() and external interrupts.
https://www.theboredrobot.com/post/reading-values-from-an-rc-receiver-using-arduino
 */

//define the pins and variables
#define ThrottlePin 2
#define SteerPin 3
volatile long ThrottleStartTime = 0;
volatile long SteerStartTime = 0;
volatile long ThrottleCurrentTime = 0;
volatile long SteerCurrentTime = 0;
volatile long ThrottlePulses = 0;
volatile long SteerPulses = 0;
int ThrottlePulseWidth = 0;
int SteerPulseWidth = 0;
bool newThrottle = false;
bool newSteer = false;
int err_cntr = 0;

void setup() {
  //set up the serial monitor, pin mode, and external interrupt.
  Serial.begin(115200);
  pinMode(ThrottlePin, INPUT_PULLUP);
  pinMode(SteerPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ThrottlePin),ThrottlePulseTimer,CHANGE);
  attachInterrupt(digitalPinToInterrupt(SteerPin),SteerPulseTimer,CHANGE);
}

void loop() {
  if(newSteer && newThrottle){
    Serial.print("PWM, 1, ");
    Serial.print(ThrottlePulseWidth);
    Serial.print(", ");
    Serial.println(SteerPulseWidth);
    newSteer = false;
    newThrottle = false;
    err_cntr = 0;
    delay(50);
  }
  else{
    err_cntr = err_cntr + 1;
  }

  if(err_cntr > 100){
    Serial.println("PWM, 0, -1, -1");
    err_cntr = 0;
    delay(1000);
  }
}

void ThrottlePulseTimer(){
//  Serial.println("throttle in");
  //measure the time between interrupts
  ThrottleCurrentTime = micros();
  long dt = ThrottleCurrentTime - ThrottleStartTime;
//  Serial.print("Throttle dt: ");
//  Serial.println(dt);
  if (dt > 0){
    if (dt >= 900 && dt <= 2100){
      ThrottlePulseWidth = dt;
      newThrottle = true;
    }
    ThrottleStartTime = ThrottleCurrentTime;
  }
}

void SteerPulseTimer(){
  //measure the time between interrupts
  SteerCurrentTime = micros();
  long dt = SteerCurrentTime - SteerStartTime;
//  Serial.println(SteerCurrentTime);
//  Serial.print("Steering dt: ");
//  Serial.println(dt);
  if (dt > 0){
    if (dt >= 900 && dt <= 2100){
      SteerPulseWidth = dt;
      newSteer = true;
    }
    SteerStartTime = SteerCurrentTime;
  }
}
