#include <Arduino.h>

#include "SBUS.h"


#define throttle A22
float maxthrottle=255;
float controller_deadband=0.01;
// a SBUS object, which is on hardware
// serial port 1
SBUS x8r(Serial2);


// channel, fail safe, and lost frames data

//ODRIVE STUFF
//on the teensy I am using serial port 3
//on the odrive I am using axis 0
int prev=0;

void setup() {
  // begin the SBUS communication
  x8r.begin();
  Serial.begin(9600);
  

   
  
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
    //output the values of the channels to serial
    
    
    //for(int i=0;i<16;i++){
      //Serial.println(channels[i]);
    //}
    //Serial.println(" ");
    //channel[0] is -1 to 1 left vertical stick position
    //channel[7] is the self-centering vertical throttle

    if(channels[7]-controller_deadband>0){
      

      int testing = maxthrottle*channels[7];
      Serial.println("throttle value: ");
      Serial.println(testing);
      analogWrite(throttle, testing);
    }
    else{
      //if the channel value is not within the deadband and more than zero, set throttle to zero
      analogWrite(throttle, 0);
    }
    

    
    

  }
  else{
    //if there is no input from controller, write zero volt to dac
    analogWrite(throttle, 0);
  }

}