#include "imu.h"

Adafruit_LSM6DSOX sox;
KalmanFilter kalmanRoll;
KalmanFilter kalmanPitch;
float roll = 0.0f;
float pitch = 0.0f;
unsigned long lastMicros = 0;

bool emergency = 0;

const uint8_t MPU6500_ADDR = 0x68;
MPU6500_WE myMPU6500(MPU6500_ADDR);

data_imu orientation = {0.0f, 0.0f, 0.0f};


bool imu_sensor::IMU_init() {
    
  Wire.begin(I2C_SDA, I2C_SCL);

  Serial.println("Initialisation LSM6DSOX...");

  // Adresse I2C par défaut : 0x6A
  // Si ça échoue essaie 0x6B (broche SA0/SDO reliée à 3.3V)
  if (!sox.begin_I2C(0x6A, &Wire)) {
    Serial.println("ERREUR : capteur LSM6DSOX introuvable !");
    Serial.println("Vérifie le câblage SDA/SCL et l'alimentation 3.3V.");
    while (1) delay(100);
  }

  Serial.println("LSM6DSOX détecté !");

  // Plages de mesure
  sox.setAccelRange(LSM6DS_ACCEL_RANGE_4_G);    // ±4g
  sox.setGyroRange(LSM6DS_GYRO_RANGE_500_DPS);  // ±500 °/s
  sox.setAccelDataRate(LSM6DS_RATE_104_HZ);
  sox.setGyroDataRate(LSM6DS_RATE_104_HZ);

  lastMicros = micros();
  delay(1000); // laisse le capteur stabiliser
  return true;
}


data_imu imu_sensor::get_orientation() {
  sensors_event_t accel, gyro, temp;
  sox.getEvent(&accel, &gyro, &temp);

  // Accéléromètre en m/s²  →  on divise par g pour avoir des unités normalisées
  float ax = accel.acceleration.x / 9.81f;
  float ay = accel.acceleration.y / 9.81f;
  float az = accel.acceleration.z / 9.81f;

  // Gyroscope en rad/s  →  conversion en deg/s
  float gx = gyro.gyro.x * 180.0f / PI;
  float gy = gyro.gyro.y * 180.0f / PI;

  // dt
  unsigned long now = micros();
  float dt = (now - lastMicros) / 1000000.0f;
  lastMicros = now;

  // Angles bruts accéléromètre
  float rollAcc  = atan2(ay, az)                   * 180.0f / PI;
  float pitchAcc = atan2(-ax, sqrt(ay*ay + az*az)) * 180.0f / PI;

  // Filtre de Kalman
  roll  = kalmanRoll .update(rollAcc,  gx, dt);
  pitch = kalmanPitch.update(pitchAcc, gy, dt);
  if(abs(roll*(180/PI)) > 30 || abs(pitch*(180/PI)) > 50){
    emergency = 1;
  }

  orientation.roll_deg = roll;
  orientation.pitch_deg = pitch;



  return orientation;
}
