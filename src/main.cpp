#include <Arduino.h>
#include "ODriveTeensyCAN.h"
#include "SBUS.h"
#define throttle A22




float maxthrottle=4096;
float controller_deadband=0.01;
// a SBUS object, which is on hardware
// serial port 1
SBUS x8r(Serial2);
int relay_in = 23;

// channel, fail safe, and lost frames data

//ODRIVE STUFF
//on the teensy I am using serial port 3
//on the odrive I am using axis 0
int prev=0;
int axis_id = 3;

// if this becomes true, kill system.
bool emergency = false;
String emergencyMsg = ""; 

ODriveTeensyCAN odrive;


bool armed=false;

void armOdrive() {

  int requested_state;
  //odrive.RunState(axis_id,ODriveTeensyCAN::CMD_ID_CANOPEN_NMT_MESSAGE);
  delay(1000);
  Serial.print("testing");
  Serial.print(odrive.GetPosition(axis_id));
  delay(200);
  requested_state = ODriveTeensyCAN::AXIS_STATE_FULL_CALIBRATION_SEQUENCE;
  odrive.RunState(axis_id, requested_state);
  delay(200);
  while(odrive.GetCurrentState(axis_id)!=ODriveTeensyCAN::AXIS_STATE_IDLE){
    delay(500);
    Serial.println("waiting...");
  }
  Serial.println("got out");
  requested_state = ODriveTeensyCAN::AXIS_STATE_CLOSED_LOOP_CONTROL;
  odrive.RunState(axis_id, requested_state);
  armed=true; 
}

void killSystem() {
  while(emergency) {
    Serial.println("Emergency tripped: " + emergencyMsg);
    delay(500);
  }
}

void setup() {
  // begin the SBUS communication

  x8r.begin();
  analogWriteResolution(12);
  Serial.begin(9600);
  Serial3.begin(115200);
  pinMode(relay_in, OUTPUT);
  digitalWrite(relay_in, HIGH);

}

void loop() {
  //Serial.println(odrive.Heartbeat());
  if (emergency)
  {
    killSystem();
  }

  float channels[16]; bool failSafe; bool lostFrame;

  if(x8r.readCal(&channels[0],&failSafe,&lostFrame)){
    //Serial.println((float)channels[0]);
      for(int i=0;i<16;i++){
        x8r.readCal(&channels[i],&failSafe,&lostFrame);
        //Serial.println("Channel "+i);
        Serial.println(channels[i]);
    }
    if(channels[2]>0){
      digitalWrite(relay_in, LOW);
    }
    else{
      digitalWrite(relay_in, HIGH);
    }
 

    //odrive arming with SF toggle switch on controller
    Serial.println("odrive armed: ");
    Serial.println(armed);
    Serial.println(" ");

    if(channels[3]>0&&!armed){
      armOdrive();
    }


    // deadman controls
    if(channels[7]-controller_deadband>0){
      odrive.SetTorque(1, 0);
      
      if(channels[2]>0){
        //channel 3 is deadman switchxz
        int controller_throttle = maxthrottle*channels[7];
        Serial.println("throttle value: ");
        Serial.println(controller_throttle);
        analogWrite(throttle, (int)controller_throttle);
      }
      else{
        analogWrite(throttle, 0);
        
        //go into emergency
        emergency = true;
        emergencyMsg.append("Dead man switch fault");
        return;
      }
    }
    else{ // brake
      //if the channel value is not within the deadband and more than zero, set throttle to zero
      analogWrite(throttle, 0);

      int controller_throttle = maxthrottle*channels[7] * -1;
      Serial.println("throttle value: ");
      Serial.println(controller_throttle);
      odrive.SetTorque(1, controller_throttle);
    }
    
    int controller_steering=int(10*channels[0]);
    //check to see if steering value has changed

    if(controller_steering!=prev && armed&&(abs(channels[0])<=1)){
      //if it has, change the position of the odrive
      Serial.println("steering");
      Serial.println(controller_steering);
      prev=controller_steering;
      odrive.SetPosition(axis_id,controller_steering);
      Serial.println("odrive position");
      
      Serial.println(odrive.GetPosition(axis_id));
    
    }
    

    
    

  }

  // state checks

  if (odrive.Heartbeat() == -1) {
    emergency = true;
    emergencyMsg.append("Odrive Heartbeat: " + odrive.Heartbeat());
    return;
  }
}
