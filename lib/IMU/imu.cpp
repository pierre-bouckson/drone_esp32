#include "imu.h"

bool emergency = 0;

const uint8_t MPU6500_ADDR = 0x68;
MPU6500_WE myMPU6500(MPU6500_ADDR);

data_imu orientation = {0.0f, 0.0f, 0.0f};


bool imu_sensor::IMU_init() {
    
  if(!myMPU6500.init()){
    Serial.println("MPU6500 does not respond");
    return false;
  }
  else{
    Serial.println("MPU6500 is connected");
  }
  myMPU6500.autoOffsets();
  myMPU6500.enableGyrDLPF();
  myMPU6500.setGyrDLPF(MPU6500_DLPF_6);
  myMPU6500.setSampleRateDivider(5);
  myMPU6500.setGyrRange(MPU6500_GYRO_RANGE_250);
  myMPU6500.setAccRange(MPU6500_ACC_RANGE_2G);
  myMPU6500.enableAccDLPF(true);
  myMPU6500.setAccDLPF(MPU6500_DLPF_6);
  return true; 
}


data_imu imu_sensor::get_orientation() {
  xyzFloat gValue = myMPU6500.getGValues();
  xyzFloat gyr = myMPU6500.getGyrValues();
  float temp = myMPU6500.getTemperature();
  float resultantG = myMPU6500.getResultantG(gValue);


  uint32_t now = micros();
  float dt = (now - last_mes) / 1e6f;
  last_mes = now;

  orientation.roll_deg = gyr.x;
  orientation.pitch_deg = gyr.y;
  orientation.yaw_deg = gyr.z;

  float roll_acc  = atan2( gValue.y, gValue.z );
  float pitch_acc = atan2( -gValue.x, sqrt(gValue.y * gValue.y + gValue.z * gValue.z) );

  if(first==1) {
    float roll = roll_acc;
    float pitch = pitch_acc;
    float yaw = 0;
    first = 0;
  }

  float roll_pred  = roll + gyr.x * dt;
  float pitch_pred = pitch + gyr.y * dt;
  float yaw_pred   = yaw  + gyr.z * dt;

  float alpha = 0.1;

  roll  = alpha * roll_pred  + (1-alpha) * (roll_acc * (180.0 / PI));
  pitch = alpha * pitch_pred + (1-alpha) * (pitch_acc * (180.0 / PI));
  yaw   = yaw_pred;    // (pas de correction sans mag)


  orientation.roll_deg = roll;
  orientation.pitch_deg = pitch;
  orientation.yaw_deg = yaw;

  if(abs(roll) > 10 || abs(pitch) > 10){
    emergency = 1;
  }



  return orientation;
}


data_imu imu_sensor::get_gyro(){

  xyzFloat gyr = myMPU6500.getGyrValues();

  gyro_fast.roll_deg = gyr.x;
  gyro_fast.pitch_deg = gyr.y;

  return gyro_fast;

}
