#include <Arduino.h>

#include "SBUS.h"


#define throttle A22
float maxthrottle=2048;
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
  //odrive_serial.begin(115200);

   
  
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
  //Serial.println("test");
  
  float channels[16]; bool failSafe; bool lostFrame;
  if(x8r.readCal(&channels[0],&failSafe,&lostFrame)){

    Serial.println(channels[0]);
    int testing=abs(255*channels[0]);

    //check to see if throttle value has changed
    analogWrite(throttle, testing);

  }

}