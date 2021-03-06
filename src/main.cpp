#include <Arduino.h>
#include "ODriveTeensyCAN.h"
#include "SBUS.h"
#include <SimpleTimer.h>

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
float max_brake = 0.7;
float max_steering = 13;
// channel, fail safe, and lost frames data

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

void checkOdrive()
{
  if (odrive.GetAxisError(axis_braking) != 0 || odrive.GetAxisError(axis_steering) != 0)
  {
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
  
}
void setup()
{
  // begin the SBUS communication

  x8r.begin();
  analogWriteResolution(12);
  Serial.begin(9600);
  Serial3.begin(115200);
  pinMode(relay_in, OUTPUT);
  pinMode(button, INPUT);
  pinMode(RTD_led, OUTPUT);
  //pinMode(ODRIVE_status_led,OUTPUT);
  pinMode(deadman_switch_led, OUTPUT);

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
  int requested_state;
  requested_state = ODriveTeensyCAN::AXIS_STATE_CLOSED_LOOP_CONTROL;
  odrive.RunState(axis_braking, requested_state);
  odrive.RunState(axis_steering, requested_state);
  while ((odrive.GetCurrentState(axis_braking) != ODriveTeensyCAN::AXIS_STATE_CLOSED_LOOP_CONTROL) && (odrive.GetCurrentState(axis_steering) != ODriveTeensyCAN::AXIS_STATE_CLOSED_LOOP_CONTROL))
  {
    delay(250);
    Serial.println("waiting...");
  }
  odrive_st = no_error;
  analogWrite(ODRIVE_status_led, 4096);
}

void calibrate_odrive()
{
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
    Serial.println("waiting...");

    // if (timeout_ms > 2000)
    // {
    //   Serial.println("timeout...");
    //   return;
    // }
  }
  Serial.println("got out");
}

void odrive_reset()
{

  odrive.ClearErrors(axis_steering);
  delay(100);
  odrive.ClearErrors(axis_braking);
  delay(100);
  arm_odrive();
  Serial.println("resetting odrive");
}

void armOdrivefull()
{
  odrive_reset();
  delay(200);

  // Calibrating steering and braking axises.
  calibrate_odrive();

  // Arming Odrive
  arm_odrive();
}

void emergency_state()
{
  digitalWrite(relay_in, HIGH);
  digitalWrite(RTD_led, LOW);

  emergency = true;
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
  digitalWrite(relay_in, LOW);

  odrive.SetPosition(axis_steering, -controller_steering);

  Serial.print("Controller Throttle: ");
  Serial.println(controller_throttle);
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

  if (digitalRead(button))
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
