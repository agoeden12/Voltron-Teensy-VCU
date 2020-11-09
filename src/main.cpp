#include <Arduino.h>
#include "ODriveTeensyCAN.h"
#include "SBUS.h"

#define throttle A22


//debounce stuff for reset button
long debouncing_time = 30; //Debouncing Time in Milliseconds
volatile unsigned long last_micros;

bool reset_state;
float maxthrottle=4096;
//max_brake in newton-meters
float max_brake = 0.6; 

float controller_deadband=0.01;
// a SBUS object, which is on hardware
// serial port 2
SBUS x8r(Serial2);

bool armed_controller=false;
int emergency_relay = 23;
int reset_switch = 33;

// channel, fail safe, and lost frames data

//ODRIVE STUFF
//on the teensy I am using serial port 3
//on the odrive I am using axis 0
int prev=0;
//axis0 (steering is node id 3)
//axis1    (steering is node id 5)
int axis_steering = 3;
int axis_braking=5;

// if this becomes true, kill system.
bool emergency = false;

ODriveTeensyCAN odrive;


bool armed=false;

void armOdrive() {

  int requested_state;
  //odrive.RunState(axis_steering,ODriveTeensyCAN::CMD_ID_CANOPEN_NMT_MESSAGE);
  delay(1000);
  Serial.print("testing");
  Serial.print(odrive.GetPosition(axis_steering));
  delay(200);
  requested_state = ODriveTeensyCAN::AXIS_STATE_FULL_CALIBRATION_SEQUENCE;
  odrive.RunState(axis_steering, requested_state);
  requested_state = ODriveTeensyCAN::AXIS_STATE_FULL_CALIBRATION_SEQUENCE;
  odrive.RunState(axis_braking, requested_state);
  delay(200);
  while(odrive.GetCurrentState(axis_steering)!=ODriveTeensyCAN::AXIS_STATE_IDLE && (axis_braking)!=ODriveTeensyCAN::AXIS_STATE_IDLE){
    delay(500);
    Serial.println("waiting...");
  }
  Serial.println("got out");



  requested_state = ODriveTeensyCAN::AXIS_STATE_CLOSED_LOOP_CONTROL;
  odrive.RunState(axis_steering, requested_state);
  delay(200);
  odrive.RunState(axis_braking, requested_state);
  delay(200);
  armed_controller=true; 
  armed=true;
}

void killSystem(String msg) {
  while(emergency) {
    Serial.println("Emergency tripped -> " + msg);
    delay(500);
  }
}
void reset() {

  odrive.ClearErrors(axis_steering);
  delay(100);
  odrive.ClearErrors(axis_braking);
  delay(100);
  armOdrive();
  Serial.println("resetting odrive");
}

void debouncereset(){
    if(((long)(micros() - last_micros) >= debouncing_time * 1000)&&!armed) {
    reset();
    last_micros = micros();
  }
}
void setup() {
  // begin the SBUS communication
  
  x8r.begin();
  //pinMode(reset_switch, INPUT_PULLUP);
  analogWriteResolution(12);
  Serial.begin(9600);
  Serial3.begin(115200);
  pinMode(emergency_relay, OUTPUT);
  digitalWrite(emergency_relay, HIGH);
  pinMode(LED_BUILTIN,OUTPUT);
  attachInterrupt(digitalPinToInterrupt(reset_switch),debouncereset,CHANGE);
  
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

    if(channels[4]>-0.5){
      digitalWrite(emergency_relay, LOW);
    }
    else{
      digitalWrite(emergency_relay, HIGH);
    }
    
    //odrive arming with SF toggle switch on controller
    Serial.println("odrive armed: ");
    Serial.println(armed);
    Serial.println(" ");

    if((channels[5]>0)&&(!armed)&&(!armed_controller)){
      Serial.println("uh what");
      armOdrive();
    }
    if(reset_state){
      digitalWrite(LED_BUILTIN,HIGH);
    }
    else{
      digitalWrite(LED_BUILTIN,LOW);
    }


    // deadman controls
    if((channels[0]-controller_deadband>0)&&armed){
      odrive.SetTorque(1, 0);
      
      if(channels[4]>-0.5){
        //channel 3 is deadman switchxz
        int controller_throttle = maxthrottle*channels[0];
        //Serial.println("throttle value: ");
        //Serial.println(controller_throttle);
        analogWrite(throttle, (int)controller_throttle);
      }

    }
    else if((channels[0]<(0-controller_deadband)) &&armed){ // brake
      //if the channel value is not within the deadband and more than zero, set throttle to zero
      analogWrite(throttle, 0);

      float controller_braking = max_brake*channels[0];

      Serial.println("braking value: ");
      Serial.println(controller_braking);
      
      odrive.SetTorque(axis_braking, controller_braking);
    }
    else{
      analogWrite(throttle, 0);
      odrive.SetTorque(axis_braking,0);
    }
    





    if((odrive.GetAxisError(axis_braking)!=0 || odrive.GetAxisError(axis_steering)!=0) && armed_controller){
      armed=false;
      Serial.println("odrive error: ");
      Serial.println(odrive.GetAxisError(axis_steering));
      Serial.println(odrive.GetAxisError(axis_braking));
      //emergency=true;
      //killSystem("odrive error, needs reseting");
    }


    int controller_steering=int(10*channels[1]);
    //check to see if steering value has changed

    if(controller_steering!=prev && armed&&(abs(channels[1])<=1)){
      //if it has, change the position of the odrive
      //Serial.println("steering");
      //Serial.println(controller_steering);
      prev=controller_steering;
      odrive.SetPosition(axis_steering,controller_steering);
      //Serial.println("odrive position");
      
      //Serial.println(odrive.GetPosition(axis_steering));
    
    }
    

    
    

  }

  // state checks

  if (odrive.Heartbeat() == -1) {
    int test=0;
    //emergency = true;
    // killSystem("Odrive Heartbeat: " + odrive.Heartbeat());
  }
}
