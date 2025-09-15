#include <Arduino.h>
#include "AppConfig.h"
#include <WiFi.h>
#include <WiFiUdp.h>
#include "Wifi_com.h"
#include "imu.h"
#include "motor_controller.h"
#include <Wire.h>     //For I2C
WiFiUDP UDP;

int LED_BUILTIN = 2;

int PIN_MOTOR1 = 4;
int PIN_MOTOR2 = 5;
int PIN_MOTOR3 = 6;
int PIN_MOTOR4 = 7;

int PIN_SDA = 21;
int PIN_SLC = 22;
float last_mes = 0;



drone_connect drone;
imu_sensor my_imu;
motor_controller my_motors;


// the setup function runs once when you press reset or power the board
void setup() {

  delay(1000); 
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  Serial.begin(115200);
  Serial.println("Console ready !");


  Wire.begin();

  drone.init_wifi(ssid, password, localPort);     //Init Wifi 


  if(my_imu.IMU_init()) Serial.println("IMU init");

  my_motors.motor_init();


}

// the loop function runs over and over again forever
void loop() {


  const char* msg = drone.read_msg();
  if(strcmp(msg, "command") == 0){
      digitalWrite(LED_BUILTIN, HIGH);
      if(drone.answer("ok", 43127) == true){
        Serial.println("answer send");
      } else {
        Serial.println("Error send answer");
      }
  }

  

  data_imu to_print =  my_imu.get_orientation();
  Serial.print("x : ");
  Serial.print(to_print.roll_deg);
  Serial.print("    y : ");
  Serial.println(to_print.pitch_deg);

  my_motors.send_cmd(my_motors.stability());
  
  
  delay(500);
 
}

 /*
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);                       // wait for a second
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);   
  // wait for a second
  //Serial.println("Nice");
  */
