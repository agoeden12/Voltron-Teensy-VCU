#include <Arduino.h>
#include "ODriveTeensyCAN.h"
#include "SBUS.h"
#include <SimpleTimer.h>
#include "VoltronSD.h"
#include <TimeLib.h>

#define throttle A22
#define RTD_led A18
#define ODRIVE_status_led A21
#define deadman_switch_led A19
#define button 26

bool DEBUG = false;

float maxthrottle = 2700;
float controller_deadband = 0.01;


// a SBUS object, which is on hardware


// serial port 2
SBUS x8r(Serial2);
int relay_in = 23;
float max_brake = 0.6;
float max_steering = 13;
int control_state_=0;
// channel, fail safe, and lost frames data

VoltronSD voltronSD;

//ODRIVE STUFF

//on the odrive I am using axis 3 and 5

int axis_steering = 3;
int axis_braking = 5;

int deadman;
float controller_steering;
float controller_throttle;
int arm_switch;

enum odrive_satus
{
  no_error = 0,
  startup = 1,
  error = 2
};

int odrive_st = startup;
ODriveTeensyCAN odrive;

SimpleTimer check_odrive;

bool armed = false;
bool deadman_switched = false;
bool emergency = false;

time_t getTeensy3Time()
{
  return Teensy3Clock.get();
}

void checkOdrive()
{
  voltronSD.log_message("--Started Checking Odrive--");
  if (odrive.GetAxisError(axis_braking) != 0 || odrive.GetAxisError(axis_steering) != 0)
  {
    voltronSD.log_message("Odrive errors: ----------");
    String msg = "Braking Error: ";
    msg += odrive.GetAxisError(axis_braking);
    voltronSD.log_message(msg);

    msg = "Steering Error: ";
    msg += odrive.GetAxisError(axis_steering);
    voltronSD.log_message(msg);

    voltronSD.log_message("\n");
    odrive_st = error;
  }

  if(DEBUG)
  {
    Serial.println("testing the check");
    Serial.println("braking:: Axis error : Encoder error : Motor error");
    Serial.println(odrive.GetAxisError(axis_braking));
    Serial.println(odrive.GetEncoderError(axis_braking));
    Serial.println(odrive.GetMotorError(axis_braking));

    Serial.println("steering:: Axis error : Encoder error : Motor error");
    Serial.println(odrive.GetAxisError(axis_steering));
    Serial.println(odrive.GetEncoderError(axis_steering));
    Serial.println(odrive.GetMotorError(axis_steering));
  }
  voltronSD.log_message("--Finished Checking Odrive--");
  
}
void setup()
{
  control_state_=1;
  // this means that the kart is in initialization state

  x8r.begin();
  analogWriteResolution(12);
  Serial.begin(9600);
  Serial3.begin(115200);
  pinMode(relay_in, OUTPUT);
  pinMode(button, INPUT);
  pinMode(RTD_led, OUTPUT);
  //pinMode(ODRIVE_status_led,OUTPUT);
  pinMode(deadman_switch_led, OUTPUT);

  setSyncProvider(getTeensy3Time);
  voltronSD.InitializeSDcard();

  digitalWrite(relay_in, HIGH);

  //check the odrive for axis errors every 250ms
  check_odrive.setInterval(250, checkOdrive);
}
//
void brake(float b_val)
{
  float val = max_brake * b_val;
  odrive.SetTorque(axis_braking, -val);
}
void throttle_control(float t_val)
{

  int controller_throttle = maxthrottle * t_val;
  analogWrite(throttle, (int)controller_throttle);
}

void arm_odrive()
{
  voltronSD.log_message("--Started Arming Odrive--");

  int requested_state;
  requested_state = ODriveTeensyCAN::AXIS_STATE_CLOSED_LOOP_CONTROL;
  odrive.RunState(axis_braking, requested_state);
  odrive.RunState(axis_steering, requested_state);
  while ((odrive.GetCurrentState(axis_braking) != ODriveTeensyCAN::AXIS_STATE_CLOSED_LOOP_CONTROL) && (odrive.GetCurrentState(axis_steering) != ODriveTeensyCAN::AXIS_STATE_CLOSED_LOOP_CONTROL))
  {
    delay(250);
    voltronSD.log_message("Arm Odrive waiting...");
  }
  odrive_st = no_error;
  analogWrite(ODRIVE_status_led, 4096);

  voltronSD.log_message("--Finished Arming Odrive--");
}

void calibrate_odrive()
{
  voltronSD.log_message("--Started Calibrating Odrive--");

  int requested_state;

  requested_state = ODriveTeensyCAN::AXIS_STATE_FULL_CALIBRATION_SEQUENCE;
  odrive.RunState(axis_steering, requested_state);
  odrive.RunState(axis_braking, requested_state);
  delay(200);

  int timeout_ms = 0;
  while ((odrive.GetCurrentState(axis_braking) != ODriveTeensyCAN::AXIS_STATE_IDLE) && (odrive.GetCurrentState(axis_steering) != ODriveTeensyCAN::AXIS_STATE_IDLE))
  {
    delay(500);
    timeout_ms += 500;
    voltronSD.log_message("waiting to calibrate Odrive...");

    // if (timeout_ms > 2000)
    // {
    //   Serial.println("timeout...");
    //   return;
    // }
  }
  voltronSD.log_message("--Finished Calibrating Odrive--");
}

void odrive_reset()
{

  odrive.ClearErrors(axis_steering);
  delay(100);
  odrive.ClearErrors(axis_braking);
  delay(100);
  arm_odrive();
  voltronSD.log_message("resetting odrive");
}

void armOdrivefull()
{
  voltronSD.log_message("--Started Full Arming Odrive--");

  odrive_reset();
  delay(200);

  // Calibrating steering and braking axises.
  calibrate_odrive();

  // Arming Odrive
  arm_odrive();
  voltronSD.log_message("--Finished Full Arming Odrive--");
}

void emergency_state()
{
  control_state_=0;
  digitalWrite(relay_in, HIGH);
  digitalWrite(RTD_led, LOW);

  emergency = true;
  voltronSD.log_message("\n\nEmergency State------------------------");
}

void idle_state(float controller_steering, float)
{ 
  digitalWrite(relay_in, LOW);
  throttle_control(0.0003);

  ////if it has, change the position of the odrive
  //Serial.println("steering");
  //Serial.println(controller_steering);
  
  odrive.SetPosition(axis_steering, controller_steering);
  //Serial.println("odrive position");

  //Serial.println(odrive.GetPosition(axis_id));
}

void control_state(float controller_steering, float controller_throttle)
{
  control_state_=3;
  digitalWrite(relay_in, LOW);

  odrive.SetPosition(axis_steering, -controller_steering);

  // String msg = "Controller Throttle: ";
  // msg += controller_throttle;
  // voltronSD.log_message(msg);

  // msg = "Controller Steering: ";
  // msg += controller_steering;
  // voltronSD.log_message(msg);
  
  if ((controller_throttle - controller_deadband) > 0)
  {
    throttle_control(controller_throttle);
  }
  else if (controller_throttle < 0)
  {
    brake(controller_throttle);
  }
  else
  {
    throttle_control(0.0003);
  }
}

void loop()
{

  check_odrive.run();
  //Serial.println(odrive.Heartbeat());
  //Serial.println(odrive_st);

  if (digitalRead(button) && control_state_!=3)
  {
    armOdrivefull();
  }

  float channels[16];
  bool failSafe;
  bool lostFrame;

  if (x8r.readCal(&channels[0], &failSafe, &lostFrame))
  {
    //Serial.println((float)channels[0]);
    for (int i = 0; i < 16; i++)
    {
      x8r.readCal(&channels[i], &failSafe, &lostFrame);
      //Serial.println("Channel "+i);
      //Serial.println(channels[i]);
    }

    deadman = channels[4];
    controller_steering = max_steering * channels[1];
    controller_throttle = channels[0];
    arm_switch = channels[5];

    //LED status updating
    //Serial.println("odrive errors: (braking then steering)");
    //Serial.println(odrive.GetAxisError(axis_braking));
    //Serial.println(odrive.GetAxisError(axis_steering));
    if (deadman == 0)
    {

      digitalWrite(deadman_switch_led, HIGH);
      //Serial.println("deadman switch on");
    }
    else
    {
      digitalWrite(deadman_switch_led, LOW);
    }

    if (odrive_st == no_error)
    {
      analogWrite(ODRIVE_status_led, 4096);
    }
    else
    {
      analogWrite(ODRIVE_status_led, 0);
    }

    //Serial.println(odrive.Heartbeat());
    //if(emergency){
    //emergency_state();
    //}
    //else{
    //Serial.println(arm_switch);
    if ((arm_switch == 0) && (odrive_st == startup || emergency))
    {
      arm_odrive();
    }

    //state machine
    if ((odrive_st == error || emergency) || (deadman_switched && !(deadman == 0)))
    {
      emergency_state();
    }
    else if (odrive_st == startup)
    {
      // Do nothing until startup finishes
      Serial.println("in startup");
    }
    else if (odrive_st == no_error && !(deadman == 0) && !deadman_switched)
    {
      //Serial.println("in idle state");
      Serial.println("steering");
      Serial.println(controller_steering);
      Serial.println("throttle");
      Serial.println(controller_throttle);
      idle_state(controller_steering, controller_throttle);
    }
    else if (odrive_st == no_error && deadman == 0)
    {
      //control_state_=3;
    // this means that the kart is in control
      control_state(controller_steering, controller_throttle);
      deadman_switched = true;
      digitalWrite(RTD_led, HIGH);
    }
    else
    {
      emergency_state();
    }
  }
}