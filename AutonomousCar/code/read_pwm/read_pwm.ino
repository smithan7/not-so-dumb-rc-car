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

void setup() {
  //set up the serial monitor, pin mode, and external interrupt.
  Serial.begin(9600);
  pinMode(ThrottlePin, INPUT_PULLUP);
  pinMode(SteerPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ThrottlePin),ThrottlePulseTimer,CHANGE);
  attachInterrupt(digitalPinToInterrupt(SteerPin),SteerPulseTimer,CHANGE);
}


void loop() {
  //only save pulse lengths that are less than 2000 microseconds
  if (ThrottlePulses < 2000){
    ThrottlePulseWidth = ThrottlePulses;
  }  
  if (SteerPulses < 2000){
    SteerPulseWidth = SteerPulses;
  }

  Serial.print("IMU, "
  Serial.print(ThrottlePulseWidth);
  Serial.print(", ");
  
  Serial.println(SteerPulseWidth);
}

void ThrottlePulseTimer(){
  //measure the time between interrupts
  ThrottleCurrentTime = micros();
  if (ThrottleCurrentTime > ThrottleStartTime){
    ThrottlePulses = ThrottleCurrentTime - ThrottleStartTime;
    ThrottleStartTime = ThrottleCurrentTime;
  }
}

void SteerPulseTimer(){
  //measure the time between interrupts
  SteerCurrentTime = micros();
  if (SteerCurrentTime > SteerStartTime){
    SteerPulses = SteerCurrentTime - SteerStartTime;
    SteerStartTime = SteerCurrentTime;
  }
}
