#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

#include "imu.h"
#include "Wifi_com.h"
#include "motor_types.h"
#include "pid.h"


extern coef_pid coef_udp;
extern msg_rc   msg_rc_;
extern int      motor1;

class motor_controller {

private:
    const int PIN_motor_1 = 23;
    const int PIN_motor_2 = 15;
    const int PIN_motor_3 = 13;
    const int PIN_motor_4 = 33;

    const int CH_motor_1 = 0;
    const int CH_motor_2 = 1;
    const int CH_motor_3 = 2;
    const int CH_motor_4 = 3;

    // RC PWM standard : 50 Hz, impulsions 1000–2000 µs
    static constexpr int LEDC_FREQ_HZ  = 50;
    static constexpr int LEDC_RES_BITS = 16;
    static constexpr int PERIOD_US     = 1000000 / LEDC_FREQ_HZ;  // 20 000 µs
    static constexpr int PULSE_MIN_US  = 1000;   // moteur armé / arrêté
    static constexpr int PULSE_MAX_US  = 2000;   // plein gaz

    int   trottle  = 0;
    unsigned long last_time = 0;

    float erreur_pitch = 0;
    float erreur_roll  = 0;

    coef_pid coef = {1.41f, 0.011f, 0.133f};

    imu_sensor    imu_;
    pid           pid_;
    motor_cmd     commande_rc_;
    motor_cmd     commande_final;
    motor_cmd     cmd_motor_rate;
    data_imu      erreur_rate;

    drone_connect my_connect;

    // Conversion µs → valeur duty 16 bits
    uint32_t usToDuty(int pulseUs);
    // Ecriture sécurisée sur une broche moteur (clamp + conversion)
    void writeMotor(int channel, int pulseUs);

public:
    void      motor_init();
    motor_cmd cmd_vel();
    void      send_cmd();
};
