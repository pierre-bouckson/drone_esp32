#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

#include "imu.h"
#include "Wifi_com.h"
#include "motor_types.h"
#include "pid.h"



extern msg_rc msg_rc_;


class motor_controller {

private:
    int PIN_motor_1 = 23;
    int PIN_motor_2 = 15;
    int PIN_motor_3 = 13;
    int PIN_motor_4 = 33;
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

    int trottle = 0;
    unsigned long last_time = 0;

    float erreur_pitch = 0;
    float erreur_roll = 0;

    float rate_sp_pitch = 0;
    float rate_sp_roll = 0;

    float erreur_rate_pitch = 0;
    float erreur_rate_roll = 0;

    float cmd_motor_pitch = 0;
    float cmd_motor_roll = 0;


    imu_sensor imu_;                // ton objet IMU
    pid pid_;    // ton contrôleur PID
    motor_cmd commande_stability_;
    motor_cmd commande_rc_;
    motor_cmd commande_final;

    motor_cmd cmd_motor_rate;
    data_imu erreur_rate;



public:
    void motor_init();
    motor_cmd cmd_vel();
    motor_cmd stability();
    void send_cmd();


    
};