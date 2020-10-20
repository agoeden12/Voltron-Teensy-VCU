#include <Arduino.h>
#include "ODriveArduino.h"
#include "ODriveTeensyCAN.h"
#include "SBUS.h"
#define throttle A22


float maxthrottle=4096;
float controller_deadband=0.01;
// a SBUS object, which is on hardware
// serial port 1
SBUS x8r(Serial2);


// channel, fail safe, and lost frames data

//ODRIVE STUFF
//on the teensy I am using serial port 3
//on the odrive I am using axis 0
int prev=0;
int axis_id = 3;


ODriveTeensyCAN odrive;


bool armed=false;

void setup() {
  // begin the SBUS communication
  x8r.begin();
  analogWriteResolution(12);
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);
  
  //odrive_serial.begin(115200);

   
  


}

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

void loop() {
  

  //Serial.println(odrive.Heartbeat());
  // look for a good SBUS packet from the receiver
  /*
  if(x8r.read(&channels[0], &failSafe, &lostFrame)){
    //const char *castb = channels[0];
    // write the SBUS packet to an SBUS compatible servo
    Serial.println((float)channels[0]);
    //Serial.println("1");
  }
  */
  //Serial.println("test");
  
  float channels[16]; bool failSafe; bool lostFrame;

  if(x8r.readCal(&channels[0],&failSafe,&lostFrame)){
    //Serial.println((float)channels[0]);
      for(int i=0;i<16;i++){
        x8r.readCal(&channels[i],&failSafe,&lostFrame);
        //Serial.println("Channel "+i);
        Serial.println(channels[i]);
    } 

    
    Serial.println("odrive armed: ");
    Serial.println(armed);
    Serial.println(" ");

    if(channels[3]>0&&!armed){
      armOdrive();
    }
    if(channels[7]-controller_deadband>0){
      
      if(channels[2]>0){
        //channel 3 is deadman switch
        int controller_throttle = maxthrottle*channels[7];
        Serial.println("throttle value: ");
        Serial.println(controller_throttle);
        analogWrite(throttle, (int)controller_throttle);
      }
      else{
        analogWrite(throttle, 0);
      }
    }
    else{
      //if the channel value is not within the deadband and more than zero, set throttle to zero
      analogWrite(throttle, 0);
    }
    
    int controller_steering=int(10*channels[0]);
    //check to see if steering value has changed

    if(controller_steering!=prev && armed){
      //if it has, change the position of the odrive
      Serial.println("steering");
      Serial.println(controller_steering);
      prev=controller_steering;
      odrive.SetPosition(axis_id,controller_steering);
      Serial.println("odrive position");
      //Serial3.write("r axis0.encoder.pos_estimate\n");
      Serial.println(odrive.GetPosition(axis_id));
    
    }
    //odrive.SetPosition(0,(int) 100*channels[0])
    //this is a value from -1 to 1 for the throttle value
  }
  //delay(10);
  //Serial.println(failSafe);
}

