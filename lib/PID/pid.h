#pragma once
#include "imu.h"
#include "motor_types.h"

struct coef_pid {
  float kp;
  float ki;
  float kd;
};

extern motor_cmd pid_duty_;

class pid {

private:
    coef_pid coef_orientation = {4, 0, 0};
    coef_pid coef_accelero = {0, 0, 0};

public:
    motor_cmd trad_motor(data_imu intput);
    float pi_attitude_pitch(float erreur, float kp, float ki, float dt);
    float pi_attitude_roll(float erreur, float kp, float ki, float dt);
    float pid_rate_pitch(float erreur, coef_pid pid, float dt);
    float pid_rate_roll(float erreur, coef_pid pid, float dt);
    
    
};