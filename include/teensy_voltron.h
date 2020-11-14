#include <Arduino.h>
#include "ODriveTeensyCAN.h"
#include "SBUS.h"
#include "TeensyThreads.h"

class teensy_voltron {
public:
    enum VoltronState_t{
        VOLTRON_EMERGENCY = 0,
        VOLTRON_IDLE = 1,
        VOLTRON_CONTROL_READY = 2,
        VOLTRON_RACE_READY =3
    };
    
    int safety_state;
    bool armed=false;
    bool armed_controller=false;
      //max_brake in newton-meters
    float max_brake = 0.6;
    float max_steer= -10; 
    int voltron_state=0;

    teensy_voltron();
    
    void armOdrive();
    void odrive_reset();
    bool odrive_error();

    void brake(float b_val);
    void steer(float s_val);

private:
    
    ODriveTeensyCAN odrive;
      //axis0 (steering is node id 3)
      //axis1    (steering is node id 5)
    int axis_steering = 3;
    int axis_braking=5;
    
    bool reset_state;
    
};