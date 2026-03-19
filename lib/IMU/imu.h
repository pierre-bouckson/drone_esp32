#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <MPU9250_WE.h> 
#include <math.h>
#include "Wifi_com.h"
#include "motor_types.h"
#include <Adafruit_LSM6DSOX.h>

#define I2C_SDA 21
#define I2C_SCL 22

extern Adafruit_LSM6DSOX sox;

extern bool emergency;

// ──────────────────────────────────────────
//  Filtre de Kalman (identique au code précédent)
// ──────────────────────────────────────────
struct KalmanFilter {
  float angle;
  float bias;
  float P[2][2];
  float Q_angle;
  float Q_bias;
  float R_meas;

  KalmanFilter() {
    angle   = 0.0f; bias    = 0.0f;
    P[0][0] = 0.0f; P[0][1] = 0.0f;
    P[1][0] = 0.0f; P[1][1] = 0.0f;
    Q_angle = 0.001f;
    Q_bias  = 0.003f;
    R_meas  = 0.03f;
  }

  float update(float newAngle, float newRate, float dt) {
    // Prédiction
    float rate = newRate - bias;
    angle += dt * rate;
    P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_bias * dt;

    // Correction
    float S  = P[0][0] + R_meas;
    float K0 = P[0][0] / S;
    float K1 = P[1][0] / S;
    float y  = newAngle - angle;
    angle += K0 * y;
    bias  += K1 * y;
    float P00_tmp = P[0][0], P01_tmp = P[0][1];
    P[0][0] -= K0 * P00_tmp;
    P[0][1] -= K0 * P01_tmp;
    P[1][0] -= K1 * P00_tmp;
    P[1][1] -= K1 * P01_tmp;

    return angle;
  }
};

extern KalmanFilter kalmanRoll;
extern KalmanFilter kalmanPitch;

extern float roll;
extern float pitch;
extern unsigned long lastMicros;

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

    float roll_gyro = 0;
    float pitch_gyro = 0;
    float yaw_gyro = 0;

    float p = 0;
    float q = 0;
    float r = 0;

    float roll_dot = 0;
    float pitch_dot = 0;


    data_imu gyro_fast;

    drone_connect my_connect;

public:
    bool IMU_init();
    data_imu get_orientation();
};