#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "pid.h"
#include "imu.h"
#include "Wifi_com.h"

struct motor_cmd {
  int motor_1_duty;
  int motor_2_duty;
  int motor_3_duty;
  int motor_4_duty;
};

extern msg_rc msg_rc_;

class motor_controller {

private:
    int PIN_motor_1 = 23;
    int PIN_motor_2 = 0;
    int PIN_motor_3 = 12;
    int PIN_motor_4 = 32;
    const int freq = 50000;
    const int resolution = 8;

    // Init 4 channel for 4 different duty cycle 
    const int ledChannel = 0;
    const int ledChanne2 = 1;
    const int ledChanne3 = 2;
    const int ledChanne4 = 3;

    int duty1 = 0;
    int duty2 = 0;
    int duty3 = 0;
    int duty4 = 0;

    imu_sensor imu_;                // ton objet IMU
    pid pid_orientation_;    // ton contrôleur PID
    motor_cmd commande_stability_;
    motor_cmd commande_rc_;
    motor_cmd commande_final;



public:
    void motor_init();
    motor_cmd cmd_vel();
    motor_cmd stability();
    void send_cmd();


    
};