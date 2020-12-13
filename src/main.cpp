#include <Arduino.h>
#include "ODriveTeensyCAN.h"
#include "SBUS.h"
#include <SimpleTimer.h>

#define throttle A22
#define RTD_led A18
#define ODRIVE_status_led A21
#define deadman_switch_led A19
#define button 26
float maxthrottle=4096;
float controller_deadband=0.01;
// a SBUS object, which is on hardware
// serial port 1
SBUS x8r(Serial2);
int relay_in = 23;
float max_brake = 0.6;
// channel, fail safe, and lost frames data

//ODRIVE STUFF
//on the teensy I am using serial port 3
//on the odrive I am using axis 0
int prev=0;

int axis_steering = 3;
int axis_braking=5;



ODriveTeensyCAN odrive;


bool armed=false;

bool emergency=false;

void setup() {
  // begin the SBUS communication

  x8r.begin();
  analogWriteResolution(12);
  Serial.begin(9600);
  Serial3.begin(115200);
  pinMode(relay_in, OUTPUT);
  pinMode(button, INPUT);
  pinMode(RTD_led,OUTPUT);
  pinMode(ODRIVE_status_led,OUTPUT);
  pinMode(deadman_switch_led,OUTPUT);
  
  digitalWrite(relay_in, HIGH);
  


  



}
//
void brake(float b_val){
    float val = max_brake * b_val;
    odrive.SetTorque(axis_braking, val);
}
void throttle_control(float t_val){


  int controller_throttle=maxthrottle*t_val;
  analogWrite(throttle, (int)controller_throttle);
}

void armOdrive() {
//
  int requested_state;
  //
  //Serial.print("testing");
  //Serial.print(odrive.GetPosition(axis_steering));
  //delay(200);
  //requested_state = ODriveTeensyCAN::AXIS_STATE_FULL_CALIBRATION_SEQUENCE;
  //odrive.RunState(axis_steering, requested_state);
  //odrive.RunState(axis_braking, requested_state);
  //delay(200);
  //while((odrive.GetCurrentState(axis_braking)!=ODriveTeensyCAN::AXIS_STATE_IDLE)&&(odrive.GetCurrentState(axis_steering)!=ODriveTeensyCAN::AXIS_STATE_IDLE)){
  //  delay(500);
  //  Serial.println("waiting...");
  //}
  //Serial.println("got out");
  requested_state = ODriveTeensyCAN::AXIS_STATE_CLOSED_LOOP_CONTROL;
  odrive.RunState(axis_braking, requested_state);
  odrive.RunState(axis_steering, requested_state);
  while((odrive.GetCurrentState(axis_braking)!=ODriveTeensyCAN::AXIS_STATE_CLOSED_LOOP_CONTROL)&&(odrive.GetCurrentState(axis_steering)!=ODriveTeensyCAN::AXIS_STATE_CLOSED_LOOP_CONTROL)){
    delay(250);
    Serial.println("waiting...");
  }
  armed=true;
  digitalWrite(ODRIVE_status_led, HIGH); 
}
void odrive_reset() {

  odrive.ClearErrors(axis_steering);
  delay(100);
  odrive.ClearErrors(axis_braking);
  delay(100);
  armOdrive();
  Serial.println("resetting odrive");
  
}
void armOdrivefull() {
//
  int requested_state;
  //
  //Serial.print("testing");
  //Serial.print(odrive.GetPosition(axis_steering));
  odrive_reset();
  delay(200);
  requested_state = ODriveTeensyCAN::AXIS_STATE_FULL_CALIBRATION_SEQUENCE;
  odrive.RunState(axis_steering, requested_state);
  odrive.RunState(axis_braking, requested_state);
  delay(200);
  while((odrive.GetCurrentState(axis_braking)!=ODriveTeensyCAN::AXIS_STATE_IDLE)&&(odrive.GetCurrentState(axis_steering)!=ODriveTeensyCAN::AXIS_STATE_IDLE)){
    delay(500);
    Serial.println("waiting...");
  }
  Serial.println("got out");
  requested_state = ODriveTeensyCAN::AXIS_STATE_CLOSED_LOOP_CONTROL;
  odrive.RunState(axis_braking, requested_state);
  odrive.RunState(axis_steering, requested_state);
  while((odrive.GetCurrentState(axis_braking)!=ODriveTeensyCAN::AXIS_STATE_CLOSED_LOOP_CONTROL)&&(odrive.GetCurrentState(axis_steering)!=ODriveTeensyCAN::AXIS_STATE_CLOSED_LOOP_CONTROL)){
    delay(250);
    Serial.println("waiting...");
  }
  armed=true;
  digitalWrite(ODRIVE_status_led, HIGH); 
}






void emergency_state(){
  digitalWrite(relay_in, HIGH);
  digitalWrite(RTD_led, LOW);
  emergency=true;
  
}

void idle_state(float controller_steering){
  digitalWrite(relay_in, LOW);
  throttle_control(0);
 
  ////if it has, change the position of the odrive
  //Serial.println("steering");
  //Serial.println(controller_steering);
  //prev=controller_steering;
  odrive.SetPosition(axis_steering,controller_steering);
  //Serial.println("odrive position");
  
  //Serial.println(odrive.GetPosition(axis_id));


}

void control_state(float controller_steering, float controller_throttle){
  digitalWrite(relay_in, LOW);
  
  odrive.SetPosition(axis_steering,controller_steering);


  if((controller_throttle-controller_deadband)>0){
    throttle_control(controller_throttle);
  }
  else if(controller_throttle<0){
    brake(controller_throttle);
  }
  else{
    throttle_control(0);
  }
}




void loop(){
  
  

  //Serial.println(odrive.Heartbeat());

  if(digitalRead(button)){
    armOdrivefull();
  }
  
  float channels[16]; bool failSafe; bool lostFrame;

  if(x8r.readCal(&channels[0],&failSafe,&lostFrame)){
    //Serial.println((float)channels[0]);
      for(int i=0;i<16;i++){
        x8r.readCal(&channels[i],&failSafe,&lostFrame);
        //Serial.println("Channel "+i);
        //Serial.println(channels[i]);
    }



    int deadman=channels[4];
    float controller_steering=10*channels[1];
    float controller_throttle=channels[0];
    int arm_switch=channels[5];





    //LED status updating
    //Serial.println("odrive errors: (braking then steering)");
    //Serial.println(odrive.GetAxisError(axis_braking));
    //Serial.println(odrive.GetAxisError(axis_steering));
    if(deadman==0){

      digitalWrite(deadman_switch_led,HIGH);
      //Serial.println("deadman switch on");
    }
    else{
      digitalWrite(deadman_switch_led,LOW);
    }
    
    if(odrive.GetAxisError(axis_braking)!=0 || odrive.GetAxisError(axis_steering)!=0){
      digitalWrite(ODRIVE_status_led,LOW);
    }
    else{
      digitalWrite(ODRIVE_status_led,HIGH);
    }

    //Serial.println(odrive.Heartbeat());
    //if(emergency){
      //emergency_state();
    //}
    //else{
    //Serial.println(arm_switch);
    if((arm_switch==0)&&!armed){
      //Serial.println("attempting to arm");
      armOdrive();
    }



    //state machine
    if((odrive.GetAxisError(axis_braking)!=0 || odrive.GetAxisError(axis_steering)!=0) && armed&&!emergency){
      emergency_state();
    }
    else if(odrive.GetAxisError(axis_braking)==0 && odrive.GetAxisError(axis_steering)==0&&!(deadman==0)){
      idle_state(controller_steering);  
    }
    else if(odrive.GetAxisError(axis_braking)==0 && odrive.GetAxisError(axis_steering)==0&&deadman==0&&armed){
      control_state(controller_steering,controller_throttle);
      digitalWrite(RTD_led, HIGH);
    }
    else{
      emergency_state();
    }
    

    





  }
  else{
    emergency_state();
  }


}
