#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <MPU9250_WE.h> 
#include <math.h>


extern const uint8_t MPU6500_ADDR;
extern MPU6500_WE myMPU6500;

struct data_imu {
  float roll_deg;     // rotation autour X
  float pitch_deg;    // rotation autour Y
  float yaw_deg;      // rotation autour Z (dérive sans mag)
};

extern data_imu orientation; 

extern float last_mes;


class imu_sensor {

private:
    int first = 1;
    float roll = 0;
    float pitch = 0;
    float yaw = 0;



public:
    bool IMU_init();
    data_imu get_orientation();

    
};