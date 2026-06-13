#include <Arduino.h>
#include <Wire.h>
#include "esp_task_wdt.h"

#include "AppConfig.h"
#include "Wifi_com.h"
#include "command_parser.h"
#include "imu.h"
#include "flight_controller.h"
#include "motor_controller.h"

// ---- Matériel ----
static constexpr int LED_BUILTIN_PIN = 2;
static constexpr int WATCHDOG_TIMEOUT_S = 30;

// ---- Modules du firmware (une responsabilité chacun) ----
drone_connect    radio;     // transport WiFi / UDP
CommandParser    parser;    // décodage du protocole
imu_sensor       imu;       // capteur d'orientation
FlightController flight;    // boucle de stabilisation
motor_controller motors;    // sorties PWM moteurs

void setup() {
    delay(1000);
    pinMode(LED_BUILTIN_PIN, OUTPUT);
    digitalWrite(LED_BUILTIN_PIN, HIGH);

    Serial.begin(115200);
    Serial.println("Console ready !");

    // Chien de garde : redémarre la carte si la boucle se bloque.
    esp_task_wdt_init(WATCHDOG_TIMEOUT_S, true);
    esp_task_wdt_add(NULL);

    Wire.begin();
    radio.init_wifi(ssid, password, localPort);

    if (imu.IMU_init()) Serial.println("IMU init");
    motors.motor_init();
}

void loop() {
    esp_task_wdt_reset();

    // 1. Réception et décodage d'un éventuel message.
    const char* msg = radio.read_msg();
    if (parser.parse(msg) == CommandParser::Kind::Ping) {
        digitalWrite(LED_BUILTIN_PIN, HIGH);
        Serial.println(radio.answer("ok", hostPort) ? "answer send" : "Error send answer");
    }

    // 2. Lecture de l'orientation.
    data_imu orientation = imu.get_orientation();

    // 3. Sécurité : inclinaison excessive → arrêt définitif des moteurs.
    if (imu.inEmergency()) {
        motors.stop();
        digitalWrite(LED_BUILTIN_PIN, LOW);
        while (1) {}
    }

    // 4. Stabilisation : calcul de la consigne et envoi aux moteurs.
    motor_cmd cmd;
    if (flight.update(parser.rc(), orientation, parser.pid(), cmd)) {
        motors.write(cmd);
    }
}
