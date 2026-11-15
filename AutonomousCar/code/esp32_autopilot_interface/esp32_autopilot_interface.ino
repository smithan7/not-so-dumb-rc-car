/*
ESP32 autopilot interface: car-side firmware between the RC receiver ("Steve"), the Jetson
Xavier NX ("Xavier"), the steering servo / ESC, and the driveshaft hall sensor.
Host side: smithan7/ROS_nsd_rc_car catkin_ws/src/esp32_interface/scripts/esp32_node.py

Imported unchanged (apart from this comment) from the local Arduino folder
esp32_autopilot_interace/esp32_autopilot_interace.ino; it was never in git. Two variants there,
_w_esp32pwm and _testing_led_lib_for_servo, differ only in the PWM output library.

Protocol (115200 baud)
  PC -> ESP32:  C,<steer_us>,<speed_us>,<(steer+speed)%1000>
  ESP32 -> PC (~20 Hz):
    C,<ms since Steve>,<ms since Xavier>,<cmd_status_flag>,<steve_cntrl_us>,<steve_speed_us>,
      <steve_steer_us>,<xavier_status_flag>,<xavier_speed_us>,<xavier_steer_us>,
      <hall_edge_time_us>,<hall_dt_us>,$
  Receiver channel 3 > 1500 us: human (Steve) has control; otherwise Xavier's commands are output,
  clamped to 1300..1700 us. Hall sensor: one rising edge per driveshaft revolution.

KNOWN ISSUES - fix before autonomous driving on the 6S car:
  1. Xavier timeout never triggers: `if(cmd_loop_time - last_xavier_time)` is true for any
     nonzero value, so a dead Jetson leaves its last command applied indefinitely.
  2. steve_interval = 20000 ms: losing the receiver keeps the last command for 20 s.
  3. BRAKE_CMD = 1500 is ESC neutral (coast), not brake.
  Also: no millis() stamp on the status line (sysid time alignment), hall_dt is not reset when
  the driveshaft stops, and ISR variables are shared across tasks without a critical section.
*/

#include <ESP32Servo.h>

volatile long steve_speed_rise_time = 0;
volatile bool good_speed_rise = false;
volatile long steve_steer_rise_time = 0;
volatile bool good_steer_rise = false;
volatile long steve_cntrl_rise_time = 0;
volatile bool good_cntrl_rise = false;

int cmd_status_flag = 0;
//  0 : Good, Steve has control
//  1 : Good, Xavier has control
// -1 : Bad, have not heard from Steve in a while
// -2 : Bad, have not heard from Xavier in a while

int xavier_status_flag = 0;
//  0 : Good connection/msgs
// -1 : bad serial  message

//define the pins and variables
volatile long cmdStartTime = 0;
volatile long speedStartTime = 0;
volatile long steerStartTime = 0;
volatile bool newCmd = false;
volatile bool newSpeed = false;
volatile bool newSteer = false;
//volatile long pwm_frame_length = 10957; // Old RC Receiver
volatile long pwm_frame_length = 5550; // New RC Receiver - 5450

long cmd_loop_time = -1;
long status_loop_time = -1;

volatile int steve_cntrl_cmd = 0;
volatile int steve_speed_cmd = 0;
volatile int steve_steer_cmd = 0;
long last_steve_time = -10000000;
long steve_interval = 20000;

long ir_current_time = micros();
long ir_last_time = micros();
int ir_high = -1;
long ir_dt = -1;

long he_current_time = micros();
long he_last_time = micros();
long he_dt = -1;

int xavier_steer_cmd = 1500;
int xavier_speed_cmd = 1500;
long last_xavier_time = -10000000;
long xavier_interval = 30;
int min_speed = 1300;
int max_speed = 1700;
int min_steer = 1300;
int max_steer = 1700;

int BRAKE_CMD = 1500; // cmd I sent to throttle to come to a stop - assume we should hold current steering command

Servo steer_servo;
Servo speed_servo;

// This is what we actually wired up
//#define IR_Sensor_Pin 21
#define HE_Sensor_Pin 21
#define SpeedPinIn 19
#define SteerPinIn 18
#define CmdPinIn 17
#define SpeedPinOut 16
#define SteerPinOut 4


int checksum_modulator = 1000;

void setup() {
  //Start Serial Output
  Serial.begin(115200); 
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  // initialize PWM stuff
  speed_servo.setPeriodHertz(50);
  speed_servo.setResolution(10);
  speed_servo.attach(SpeedPinOut);
  steer_servo.setPeriodHertz(50);
  steer_servo.attach(SteerPinOut);

  attachInterrupt(digitalPinToInterrupt(SpeedPinIn), speedPulseTimer,CHANGE);
  attachInterrupt(digitalPinToInterrupt(SteerPinIn), steerPulseTimer,CHANGE);
  attachInterrupt(digitalPinToInterrupt(CmdPinIn),   cmdPulseTimer,  CHANGE);
//  attachInterrupt(digitalPinToInterrupt(IR_Sensor_Pin),   irPulseTimer,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(HE_Sensor_Pin),   hallEffectPulseTimer,  RISING);

//  Serial.println("Nano PWM Interface");
//  Serial.println("P,Time (ms), Steer (pwm), Speed (pwm), (Steer+Speed)%255, $"); 

  xTaskCreatePinnedToCore(
              pwm_pub_task,       //Function to implement the task 
              "PMW_Pub_Thread", //Name of the task
              6000,       //Stack size in words 
              NULL,       //Task input parameter 
              0,          //Priority of the task - might want to bump this to 2 to give priority: https://www.freertos.org/RTOS-task-priority.html
              NULL,       //Task handle.
              1);         //Core where the task should run 

  xTaskCreatePinnedToCore(
              status_pub_task,       //Function to implement the task 
              "Status_Pub_Thread", //Name of the task
              6000,       //Stack size in words 
              NULL,       //Task input parameter 
              0,          //Priority of the task - this is good at 0.
              NULL,       //Task handle.
              1);         //Core where the task should run 

}

void loop() {
// get the commands from the xavier
  String cmds_in = "";
  char character;
  while(Serial.available()) {
     character = Serial.read();
     cmds_in.concat(character);
     delayMicroseconds(100); // 0.001 seconds to ensure there is data in the buffer
  }
  // parse the commands from the xavier
  if (cmds_in != "") {
//      Serial.print("Received: ");
//      Serial.println(cmds_in);
    int ind1 = cmds_in.indexOf(',');  //finds location of first ,
    String header = cmds_in.substring(0, ind1);   //captures first data String
//      Serial.print("Header: ");
//      Serial.println(header);
    if(header.compareTo("C") ==0){//&& tail.compareTo("$") ==0){
      int ind2 = cmds_in.indexOf(',', ind1+1 );   //finds location of second ,
      int steer_in = cmds_in.substring(ind1+1, ind2+1).toInt();   //captures second data String
//        Serial.print("Steer_in: ");
//        Serial.println(steer_in);
      int ind3 = cmds_in.indexOf(',', ind2+1 );
      int speed_in = cmds_in.substring(ind2+1, ind3+1).toInt();
//        Serial.print("Speed_in: ");
//        Serial.println(speed_in);
      int ind4 = cmds_in.indexOf(',', ind3+1 );
      int checksum = cmds_in.substring(ind3+1).toInt();
//        Serial.print("Checksum: ");
//        Serial.println(checksum);
      if((steer_in + speed_in) % checksum_modulator == checksum){
//          Serial.println("Checksum worked");
        if(steer_in > 900 && steer_in <= 2100){
          xavier_steer_cmd = steer_in;
        }
        if(speed_in >= 900 && speed_in <= 2100){
          xavier_speed_cmd = speed_in;
        }        
        last_xavier_time = millis();
        xavier_status_flag = 0;
      }
      else{
//        Serial.print("Checksum Failed: ");
//        Serial.println((steer_in + speed_in) % checksum_modulator);
        xavier_status_flag = -1;
      }
    }
  }

  if(millis() - last_xavier_time > xavier_interval){
    xavier_status_flag = -1;
  }
  
  delay(20);
}

float clampFloat(float v, const float &minV, const float &maxV){
  if(v < minV){
    return minV;
  }
  if(v > maxV){
      return maxV;
  }
  return v;
}

int clampInt(int v, const int &minV, const int &maxV){
  if(v < minV){
    return minV;
  }
  if(v > maxV){
      return maxV;
  }
  return v;
}

void hallEffectPulseTimer(){
   //measure the time between rising edge interrupts - which is one full rotation
  he_current_time = micros();
  he_dt = he_current_time - he_last_time;
  he_last_time = he_current_time;
}

void cmdPulseTimer(){
  //measure the time between interrupts
  long currentTime = micros();
  // begin new
  if (digitalRead(CmdPinIn) == HIGH){
    // this is the rising edge - so we just went up. These should happen at the same time unless it is noise
    long d_rise = currentTime - steve_cntrl_rise_time;
    if( (abs(d_rise - pwm_frame_length) > 100)){
      // bad measurement
      good_cntrl_rise = false;
    }
    else{
      // good measurement
      good_cntrl_rise = true;
    }
    steve_cntrl_rise_time = currentTime;
  }
  else{
    // this is the fall time - or the measurement
    long dt = currentTime - steve_cntrl_rise_time;
    if (dt >= 1000 && dt <= 2000 && good_cntrl_rise){
      steve_cntrl_cmd = dt;
      newCmd = true;
    }
  } 
}

void speedPulseTimer(){
  //measure the time between interrupts
  long currentTime = micros();
  // begin new
  if (digitalRead(SpeedPinIn) == HIGH){
    // this is the rising edge - so we just went up. These should happen at the same time unless it is noise
    long d_rise = currentTime - steve_speed_rise_time;
//    Serial.println(d_rise);
    if( (abs(d_rise - pwm_frame_length) > 100)){
      // bad measurement
//      Serial.print("bad measurement: ");
//      Serial.println(d_rise);
      good_speed_rise = false;
    }
    else{
      // good measurement
      good_speed_rise = true;
    }
    steve_speed_rise_time = currentTime;
  }
  else{
    // this is the fall time - or the measurement
    long dt = currentTime - steve_speed_rise_time;
    if (dt >= 1000 && dt <= 2000 && good_speed_rise){
      steve_speed_cmd = dt;
      newSpeed = true;
    }
  }    
}

void steerPulseTimer(){
  //measure the time between interrupts
  long currentTime = micros();
  // begin new
  if (digitalRead(SteerPinIn) == HIGH){
    // this is the rising edge - so we just went up. These should happen at the same time unless it is noise
    long d_rise = currentTime - steve_steer_rise_time;
    if( (abs(d_rise - pwm_frame_length) > 100)){
      // bad measurement
      good_steer_rise = false;
    }
    else{
      // good measurement
      good_steer_rise = true;
    }
    steve_steer_rise_time = currentTime;
  }
  else{
    // this is the fall time - or the measurement
    long dt = currentTime - steve_steer_rise_time;
    if (dt >= 1000 && dt <= 2000 && good_steer_rise){
      steve_steer_cmd = dt;
      newSteer = true;
    }
  }  
}

void status_pub_task( void* ){
  while(1){
    status_loop_time = millis();
    Serial.print("C,");
    Serial.print(status_loop_time - last_steve_time);
    Serial.print(",");
    Serial.print(status_loop_time - last_xavier_time);
    Serial.print(",");
    Serial.print(cmd_status_flag);
    Serial.print(",");
    Serial.print(steve_cntrl_cmd);
    Serial.print(",");
    Serial.print(steve_speed_cmd);
    Serial.print(",");
    Serial.print(steve_steer_cmd);
    Serial.print(",");
    Serial.print(xavier_status_flag);
    Serial.print(",");
    Serial.print(xavier_speed_cmd);
    Serial.print(",");
    Serial.print(xavier_steer_cmd);
    Serial.print(",");
//    Serial.print(ir_current_time);
//    Serial.print(",");
//    Serial.print(ir_dt);
//    Serial.print(",");
//    Serial.print(ir_high);
    Serial.print(he_current_time);
    Serial.print(",");
    Serial.print(he_dt);
    Serial.println(",$");
    delay(50);
  }
}

void pwm_pub_task( void* ){
  while(1){
//    Serial.println("pwm_pub_task::in pwm_loop");
    cmd_loop_time = millis();
    // Check to see if I have heard from Steve
    if(newSteer & newSpeed & newCmd){
      last_steve_time = cmd_loop_time;
      newSteer = false;
      newSpeed = false;
      newCmd = false;
    }
    // Just test servos actually work
//    speed_servo.writeMicroseconds(xavier_speed_cmd);
//    steer_servo.writeMicroseconds(xavier_steer_cmd);
//    speed_servo.writeMicroseconds(steve_speed_cmd);
//    steer_servo.writeMicroseconds(steve_steer_cmd);
    
    if(cmd_loop_time - last_steve_time < steve_interval){
//      Serial.println("pwm_pub_task::in cmd loop");
      // I have heard from Steve - continue, does Steve have cmd or do I?
      if(steve_cntrl_cmd > 1500){
//        Serial.println("pwm_pub_task::publishing steve's cmds");
        // Steve has cmd - publish out his commands
        cmd_status_flag = 0;
        speed_servo.writeMicroseconds(steve_speed_cmd);
        steer_servo.writeMicroseconds(steve_steer_cmd);
      }
      else{
        // I have cmd - check to see if I have heard from the Xavier
        if(cmd_loop_time - last_xavier_time){
          // I have heard from the Xavier - publish the cmds
//          Serial.println("pwm_pub_task::publishing Xavier's cmds");
          cmd_status_flag = 1;
          speed_servo.writeMicroseconds(clampInt(xavier_speed_cmd, min_speed, max_speed));
          steer_servo.writeMicroseconds(clampInt(xavier_steer_cmd, min_steer, max_steer)); // it is discretized into bins of 20 - I add 11 toeven it out, seams to work well
        }
        else{
          // I am asking Xavier to take control and I have not heard from the Xavier - apply brakes and set status flag to error
          Serial.println("pwm_pub_task::have not heard from Xavier - not publishing Xavier's cmds");
          cmd_status_flag = -2;
          speed_servo.writeMicroseconds(BRAKE_CMD);
        }
      }
    }
    else{
      // I am asking Steve to take control and I have not heard from Steve - apply brakes and set status flag to error
//      Serial.println("pwm_pub_task::have not heard from Steve - not publishing steve's cmds");
      cmd_status_flag = -1;
      speed_servo.writeMicroseconds(BRAKE_CMD);
    }
//  delay(5);
  }
}

//void irPulseTimer(){
//  // key here is that EVERY change - up/down represents a 1/4 turn
//  // so....
//  // time for 1/4 turn = dt since last spike in seconds
//  // estimated time for full turn = 4 * (dt since last spike) in seconds
//  // estimated radians per second = 2*pi / (estimated time for full turn) 
//  // I *think*  it will be better to do filtering on the xavier and just get the raw data out at ~100 Hz
//  //measure the time between interrupts
//  ir_current_time = micros();
//  ir_dt = ir_current_time - ir_last_time;
//  ir_last_time = ir_current_time;
//  if (digitalRead(IR_Sensor_Pin) == HIGH){
//    ir_high = 1;
//  }
//  else{
//    ir_high = 0;
//  } 
//}
