#include <Arduino.h>
#include "AppConfig.h"
#include <WiFi.h>
#include <WiFiUdp.h>
#include "Wifi_com.h"
#include "imu.h"
#include "motor_controller.h"
#include <Wire.h>     //For I2C
#include "esp_task_wdt.h"

WiFiUDP UDP;

int LED_BUILTIN = 2;

int PIN_MOTOR1 = 4;
int PIN_MOTOR2 = 5;
int PIN_MOTOR3 = 6;
int PIN_MOTOR4 = 7;

int PIN_SDA = 21;
int PIN_SLC = 22;
float last_mes = 0;

int gain;


coef_pid coef_udp;

drone_connect drone;
imu_sensor my_imu;
motor_controller my_motors;

msg_rc msg_rc_;


// the setup function runs once when you press reset or power the board
void setup() {

  delay(1000); 
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.begin(115200);
  Serial.println("Console ready !");

  esp_task_wdt_init(30, true);
  esp_task_wdt_add(NULL);


  Wire.begin();

  drone.init_wifi(ssid, password, localPort, otaPassword);     //Init Wifi 


  if(my_imu.IMU_init()) Serial.println("IMU init");

  my_motors.motor_init();


}

// the loop function runs over and over again forever
void loop() {

  ArduinoOTA.handle(); // OTA
  esp_task_wdt_reset();   // Rassure le timer watchdog

  const char* msg = drone.read_msg();
  if(strcmp(msg, "command") == 0){
      digitalWrite(LED_BUILTIN, HIGH);
      if(drone.answer("ok", 8894) == true){
        Serial.println("answer send");
      } else {
        Serial.println("Error send answer");
      }
  }
  
  int left, forward, up, yaw;
  if (strncmp(msg, "rc", 2) == 0) {
    Serial.println("msg rc detecter");
    if(sscanf(msg, "rc %d %d %d %d", &left, &forward, &up, &yaw) == 4){
      msg_rc_.left = left;
      msg_rc_.forward = forward;
      msg_rc_.up = up;
      msg_rc_.yaw = yaw;
      Serial.print("CMD RECU : ");
      Serial.println(msg_rc_.up);
    }
  }



  if (strncmp(msg, "gain", 4) == 0) {
    Serial.println("msg gain detecter");
    if(sscanf(msg, "gain %d", &gain) == 1){
      Serial.print("GAIN RECU : ");
      Serial.println(gain);
    }
  }


  if (strncmp(msg, "pid", 3) == 0) {
    Serial.println("msg pid detecter");
    if(sscanf(msg, "pid %f %f %f", &coef_udp.kp, &coef_udp.ki, &coef_udp.kd) == 3){
      Serial.print("PID RECU : ");
      Serial.println(coef_udp.kp);
    }
  }


  // data_imu to_print =  my_imu.get_gyro();
  // Serial.print("x : ");
  // Serial.print(to_print.roll_deg);
  // Serial.print("    y : ");
  // Serial.print(to_print.pitch_deg);
  

  

  // data_imu to_print =  my_imu.get_orientation();
  // Serial.print("x : ");
  // Serial.print(to_print.roll_deg);
  // Serial.print("    y : ");
  // Serial.print(to_print.pitch_deg);


  my_motors.send_cmd();
  
 
}

 /*
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);                       // wait for a second
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);   
  // wait for a second
  //Serial.println("Nice");
  */
