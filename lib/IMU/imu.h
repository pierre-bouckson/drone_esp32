#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <Adafruit_LSM6DSOX.h>

#include "drone_types.h"
#include "kalman.h"

// ============================================================================
//  imu_sensor : pilote du capteur inertiel LSM6DSOX.
//  Lit l'accéléromètre + le gyroscope, fusionne via deux filtres de Kalman
//  et fournit l'orientation (roll / pitch) du drone.
//  Tout l'état est encapsulé : aucune variable globale.
// ============================================================================
class imu_sensor {

private:
    // Broches du bus I2C.
    static constexpr int I2C_SDA = 21;
    static constexpr int I2C_SCL = 22;

    // Seuils d'inclinaison (degrés) au-delà desquels on déclenche l'arrêt.
    static constexpr float MAX_ROLL_DEG  = 30.0f;
    static constexpr float MAX_PITCH_DEG = 50.0f;

    Adafruit_LSM6DSOX sox;

    KalmanFilter kalmanRoll;
    KalmanFilter kalmanPitch;

    float         roll      = 0.0f;
    float         pitch     = 0.0f;
    unsigned long lastMicros = 0;

    data_imu orientation = {0.0f, 0.0f, 0.0f};
    bool     emergency   = false;

public:
    // Initialise le bus I2C et configure le capteur. Renvoie false si absent.
    bool IMU_init();

    // Lit le capteur, met à jour le filtre et renvoie l'orientation courante.
    data_imu get_orientation();

    // Vrai si l'inclinaison a dépassé les seuils de sécurité.
    bool inEmergency() const { return emergency; }
};
