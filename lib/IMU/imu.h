#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <MPU9250_WE.h> 
#include <math.h>
#include "Wifi_com.h"

extern bool emergency;

extern const uint8_t MPU6500_ADDR;
extern MPU6500_WE myMPU6500;

extern int gain;

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
    float roll = 1;
    float pitch = 1;
    float yaw = 1;

    float roll_gyro = 0;
    float pitch_gyro = 0;
    float yaw_gyro = 0;

    data_imu gyro_fast;

    drone_connect my_connect;




public:
    bool IMU_init();
    data_imu get_orientation();
    data_imu get_gyro();

    
};