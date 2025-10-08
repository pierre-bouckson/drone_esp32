[1mdiff --git a/lib/IMU/imu.cpp b/lib/IMU/imu.cpp[m
[1mindex 1dda295..4e9af15 100644[m
[1m--- a/lib/IMU/imu.cpp[m
[1m+++ b/lib/IMU/imu.cpp[m
[36m@@ -1,10 +1,13 @@[m
 #include "imu.h"[m
 [m
[32m+[m[32mbool emergency = 0;[m
[32m+[m
 const uint8_t MPU6500_ADDR = 0x68;[m
 MPU6500_WE myMPU6500(MPU6500_ADDR);[m
 [m
 data_imu orientation = {0.0f, 0.0f, 0.0f};[m
 [m
[32m+[m
 bool imu_sensor::IMU_init() {[m
     [m
   if(!myMPU6500.init()){[m
[36m@@ -68,6 +71,10 @@[m [mdata_imu imu_sensor::get_orientation() {[m
   orientation.pitch_deg = pitch;[m
   orientation.yaw_deg = yaw;[m
 [m
[32m+[m[32m  if(roll > 10 || pitch > 10){[m
[32m+[m[32m    emergency = 1;[m
[32m+[m[32m  }[m
[32m+[m
 [m
 [m
   return orientation;[m
[1mdiff --git a/lib/IMU/imu.h b/lib/IMU/imu.h[m
[1mindex 5a94bcb..eae3025 100644[m
[1m--- a/lib/IMU/imu.h[m
[1m+++ b/lib/IMU/imu.h[m
[36m@@ -4,6 +4,7 @@[m
 #include <MPU9250_WE.h> [m
 #include <math.h>[m
 [m
[32m+[m[32mextern bool emergency;[m
 [m
 extern const uint8_t MPU6500_ADDR;[m
 extern MPU6500_WE myMPU6500;[m
[1mdiff --git a/lib/motor/motor_controller.cpp b/lib/motor/motor_controller.cpp[m
[1mindex b47a082..9dfc589 100644[m
[1m--- a/lib/motor/motor_controller.cpp[m
[1m+++ b/lib/motor/motor_controller.cpp[m
[36m@@ -50,6 +50,8 @@[m [mmotor_cmd motor_controller::stability() {[m
 [m
 void motor_controller::send_cmd() {[m
 [m
[32m+[m[32m    // if(emergency = 1) while(1);[m
[32m+[m
     commande_final.motor_1_duty = -stability().motor_1_duty * 5 + cmd_vel().motor_1_duty;[m
     commande_final.motor_2_duty = -stability().motor_2_duty * 5 + cmd_vel().motor_2_duty;[m
     commande_final.motor_3_duty = -stability().motor_3_duty * 5 + cmd_vel().motor_3_duty;[m
