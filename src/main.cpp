#include <Arduino.h>
#include "ODriveArduino.h"
#include "SBUS.h"

// a SBUS object, which is on hardware
// serial port 1
SBUS x8r(Serial2);


// channel, fail safe, and lost frames data

//ODRIVE STUFF
//on the teensy I am using serial port 3
//on the odrive I am using axis 0
int prev=0;
ODriveArduino odrive(Serial3);
void setup() {
  // begin the SBUS communication
  x8r.begin();
  Serial.begin(9600);
  Serial3.begin(115200);
  //odrive_serial.begin(115200);

   
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
  //Serial.println("test");
  
  float channels[16]; bool failSafe; bool lostFrame;
  if(x8r.readCal(&channels[0],&failSafe,&lostFrame)){
    //Serial.println((float)channels[0]);
    Serial3.write("r axis0.encoder.pos_estimate\n");
    Serial.println(odrive.readFloat());
    int testing=100*channels[0];

    //check to see if throttle value has changed
    if(testing!=prev){
      //if it has, change the position of the odrive
      prev=testing;
      odrive.SetPosition(0,testing);
    }
    //odrive.SetPosition(0,(int) 100*channels[0])
    //this is a value from -1 to 1 for the throttle value
  }
  //delay(10);
  //Serial.println(failSafe);
}