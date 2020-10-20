#include <Arduino.h>

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
int relay_in = 23;
void setup() {
  // begin the SBUS communication

  x8r.begin();
  analogWriteResolution(12);
  Serial.begin(9600);
  Serial3.begin(115200);
  pinMode(relay_in, OUTPUT);
  digitalWrite(relay_in, HIGH);


    int motornum = 0;
    int requested_state;

  requested_state = ODriveArduino::AXIS_STATE_FULL_CALIBRATION_SEQUENCE;
  odrive.run_state(motornum, requested_state, true);
  requested_state = ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL;
  odrive.run_state(motornum, requested_state, false); // don't wait

}

void loop() {

  // look for a good SBUS packet from the receiver
  /*
  if(x8r.read(&channels[0], &failSafe, &lostFrame)){
    //const char *castb = channels[0];
    // write the SBUS packet to an SBUS compatible servo
    Serial.println((float)channels[0]);
    //Serial.println("1");
  }
  */

  
  float channels[16]; bool failSafe; bool lostFrame;

  if(x8r.readCal(&channels[0],&failSafe,&lostFrame)){
    //Serial.println((float)channels[0]);
      for(int i=0;i<16;i++){
        x8r.readCal(&channels[i],&failSafe,&lostFrame);
        //Serial.println("Channel "+i);
        Serial.println(channels[i]);
    } 
    Serial.println(" ");
    


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

    if(channels[2]>0){
      digitalWrite(relay_in, LOW);
    }
    else{
      digitalWrite(relay_in, HIGH);


    if(controller_steering!=prev){
      //if it has, change the position of the odrive
      Serial.println("steering");
      Serial.println(controller_steering);
      prev=controller_steering;
      odrive.SetPosition(0,controller_steering);
      Serial3.write("r axis0.encoder.pos_estimate\n");
      Serial.println(odrive.readFloat());
    
    }
    

    
    

  }
  else{
    //if there is no input from controller, write zero volt to dac
    analogWrite(throttle, 0);
  }

}