#pragma once
#include "imu.h"
#include "motor_types.h"

struct coef_pid {
  int kp;
  int ki;
  int kd;
};

extern motor_cmd pid_duty_;

class pid {

private:
    coef_pid coef_orientation = {4, 0, 0};
    coef_pid coef_accelero = {0, 0, 0};

public:
    motor_cmd PID_output(data_imu intput);
    
    
};