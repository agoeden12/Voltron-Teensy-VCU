#include <Arduino.h>
#include "teensy_voltron.h"

//pin defs
#define throttle A22
#define odrive_armed_led 37
#define emergency_relay 23
#define reset_switch 33
#define e_stop_state 34
// safety thread stuff
// if this becomes true, kill system.
bool emergency = false;


//actual max is 4096
float maxthrottle=500;



// receiver input stuff
float controller_deadband=0.01;
int deadman_val;
  // a SBUS object, which is on hardware
  // serial port 2
SBUS x8r(Serial2);

teensy_voltron vcu;

void killSystem(String msg) {
  while(emergency) {
    digitalWrite(emergency_relay, HIGH);
    Serial.println("Emergency tripped -> " + msg);
    delay(500);
  }
}


void safety_thread(){
  while(1){
    if(!digitalRead(e_stop_state)&&!vcu.odrive_error()){
      if(deadman_val>-0.5){
        vcu.safety_state=teensy_voltron::VOLTRON_CONTROL_READY;
      }
      else{
        vcu.safety_state=teensy_voltron::VOLTRON_IDLE;
      }

    }
    else{
      vcu.safety_state=teensy_voltron::VOLTRON_EMERGENCY;
      if(vcu.armed){
        killSystem("uh, idk");
      }
      
    }
  }
}

void setup() {
  // begin the SBUS communication
  
  x8r.begin();
  //pinMode(reset_switch, INPUT_PULLUP);
  analogWriteResolution(12);
  Serial.begin(9600);

  pinMode(e_stop_state,INPUT);

  pinMode(emergency_relay, OUTPUT);
  pinMode(odrive_armed_led,OUTPUT);
  digitalWrite(emergency_relay, HIGH);
  pinMode(LED_BUILTIN,OUTPUT);
  threads.addThread(safety_thread);
  //attachInterrupt(digitalPinToInterrupt(reset_switch),debouncereset,CHANGE);
  
}

void loop() {
  //Serial.println(odrive.Heartbeat());

  float channels[16]; bool failSafe; bool lostFrame;

  if(x8r.readCal(&channels[0],&failSafe,&lostFrame)){
    //Serial.println((float)channels[0]);
      for(int i=0;i<16;i++){
        x8r.readCal(&channels[i],&failSafe,&lostFrame);
        //Serial.println("Channel "+i);
        //Serial.println(channels[i]);
    }
    deadman_val=channels[4];
    //odrive arming with SF toggle switch on controller
    Serial.println("odrive armed: ");
    Serial.println(vcu.armed);
    Serial.println(" ");
    
    if(vcu.armed){
      digitalWrite(odrive_armed_led,HIGH);
    }
    else{
      digitalWrite(odrive_armed_led,LOW);
    }

    if((channels[5]>0)&&(!vcu.armed)&&(!vcu.armed_controller)){
      //Serial.println("uh what");
      vcu.armOdrive();

    }


    // deadman controls
    if((channels[0]-controller_deadband>0)&&(vcu.safety_state>=teensy_voltron::VOLTRON_CONTROL_READY){
      if(channels[4]>-0.5){
        //channel 3 is deadman switchxz
        digitalWrite(emergency_relay, LOW);
        int controller_throttle = maxthrottle*channels[0];
        //Serial.println("throttle value: ");
        //Serial.println(controller_throttle);
        analogWrite(throttle, (int)controller_throttle);
      }

    }
    else if((channels[0]<(0-controller_deadband)) &&(vcu.safety_state>=teensy_voltron::VOLTRON_IDLE)){ // brake
      //if the channel value is not within the deadband and more than zero, set throttle to zero
      analogWrite(throttle, 0);

      float controller_braking = channels[0];

      //Serial.println("braking value: ");
      //Serial.println(controller_braking);
      
      vcu.brake(controller_braking);
      
    }
    else{
      analogWrite(throttle, 0);
      vcu.brake(0);
    }

    if((vcu.safety_state>=teensy_voltron::VOLTRON_IDLE)&&(abs(channels[1])<=1)){
      vcu.steer(channels[1]);    
    }
    

    
    

  }

  // state checks


}
