#include "teensy_voltron.h"

//odrive stuff 

teensy_voltron::teensy_voltron(){

}

void teensy_voltron::armOdrive() {

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

void teensy_voltron::odrive_reset() {

  odrive.ClearErrors(axis_steering);
  delay(100);
  odrive.ClearErrors(axis_braking);
  delay(100);
  armOdrive();
  Serial.println("resetting odrive");
  
}



void teensy_voltron::brake(float b_val){
    float val = max_brake * b_val;
    odrive.SetTorque(axis_braking, val);
}

void teensy_voltron::steer(float s_val){
    float val=max_steer * s_val;
    odrive.SetPosition(axis_steering,val);
}

bool teensy_voltron::odrive_error(){
    if((odrive.GetAxisError(axis_braking)!=0 || odrive.GetAxisError(axis_steering)!=0) && armed_controller){
      return false;
      //Serial.println("odrive error: ");
      //Serial.println(odrive.GetAxisError(axis_steering));
      //Serial.println(odrive.GetAxisError(axis_braking));
      //emergency=true;
      //killSystem("odrive error, needs reseting");
    }
    else if(odrive.Heartbeat() == -1){
        return false;
    }
    else{
        return true;
    }
}