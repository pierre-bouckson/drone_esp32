#include "imu.h"

bool imu_sensor::IMU_init() {
    Wire.begin(I2C_SDA, I2C_SCL);

    Serial.println("Initialisation LSM6DSOX...");

    // Adresse I2C par défaut : 0x6A (0x6B si SA0/SDO relié à 3.3V).
    if (!sox.begin_I2C(0x6A, &Wire)) {
        Serial.println("ERREUR : capteur LSM6DSOX introuvable !");
        Serial.println("Vérifie le câblage SDA/SCL et l'alimentation 3.3V.");
        while (1) delay(100);
    }

    Serial.println("LSM6DSOX détecté !");

    // Plages et fréquences de mesure.
    sox.setAccelRange(LSM6DS_ACCEL_RANGE_4_G);    // ±4g
    sox.setGyroRange(LSM6DS_GYRO_RANGE_500_DPS);  // ±500 °/s
    sox.setAccelDataRate(LSM6DS_RATE_104_HZ);
    sox.setGyroDataRate(LSM6DS_RATE_104_HZ);

    lastMicros = micros();
    delay(1000);   // laisse le capteur se stabiliser
    return true;
}

data_imu imu_sensor::get_orientation() {
    sensors_event_t accel, gyro, temp;
    sox.getEvent(&accel, &gyro, &temp);

    // Accéléromètre en m/s²  →  normalisé par g.
    float ax = accel.acceleration.x / 9.81f;
    float ay = accel.acceleration.y / 9.81f;
    float az = accel.acceleration.z / 9.81f;

    // Gyroscope en rad/s  →  conversion en °/s.
    float gx = gyro.gyro.x * 180.0f / PI;
    float gy = gyro.gyro.y * 180.0f / PI;

    // Intervalle de temps depuis la dernière mesure.
    unsigned long now = micros();
    float dt = (now - lastMicros) / 1000000.0f;
    lastMicros = now;

    // Angles bruts issus de l'accéléromètre.
    roll_acc  = atan2(ay, az)                   * 180.0f / PI;
    pitch_acc = atan2(-ax, sqrt(ay*ay + az*az)) * 180.0f / PI;

    // Fusion par filtre de Kalman.
    roll  = kalmanRoll.update(roll_acc,  gx, dt);
    pitch = kalmanPitch.update(pitch_acc, gy, dt);

    // roll et pitch sont déjà en degrés : on compare directement aux seuils.
    if (abs(roll) > MAX_ROLL_DEG || abs(pitch) > MAX_PITCH_DEG) {
        emergency = true;
    }

    orientation.roll_deg  = roll;
    orientation.pitch_deg = pitch;
    return orientation;
}
