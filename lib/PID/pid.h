#pragma once
#include "imu.h"

struct coef_pid {
  int kp;
  int ki;
  int kd;
};

class pid {

private:
    coef_pid coef_orientation = {4, 0, 0};
    coef_pid coef_accelero = {0, 0, 0};

public:
    float PID_output(data_imu intput);
    
    
};