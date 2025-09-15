#include "pid.h"

float pid::PID_output(data_imu intput) {
    
    return coef_orientation.kp * intput.roll_deg;
}
