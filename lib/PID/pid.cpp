#include "pid.h"


motor_cmd pid_duty_;

motor_cmd pid::PID_output(data_imu intput) {

    pid_duty_.motor_1_duty = -intput.pitch_deg + intput.roll_deg;
    pid_duty_.motor_2_duty = -intput.pitch_deg + -intput.roll_deg;
    pid_duty_.motor_3_duty = intput.pitch_deg + -intput.roll_deg;
    pid_duty_.motor_4_duty = intput.pitch_deg + intput.roll_deg;

    
    return pid_duty_;
}
