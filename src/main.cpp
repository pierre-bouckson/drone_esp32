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

// ---- Télémétrie ----
// Cadence d'envoi des angles et des corrections moteurs vers l'outil d'analyse.
// 20 Hz : assez fluide pour tracer, assez lent pour ne pas saturer le WiFi.
static constexpr unsigned long TELEMETRY_PERIOD_MS = 50;

// ---- Modules du firmware (une responsabilité chacun) ----
drone_connect    radio;     // transport WiFi / UDP
CommandParser    parser;    // décodage du protocole
imu_sensor       imu;       // capteur d'orientation
FlightController flight;    // boucle de stabilisation
motor_controller motors;    // sorties PWM moteurs

// Arrêt d'urgence demandé par le pilote ("stop"). Verrouillé jusqu'au reboot.
static bool pilot_stop = false;

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

    if (imu.IMU_init()) Serial.println("IMU init !");
    motors.motor_init();
}

void loop() {
    esp_task_wdt_reset();

    // 1. Réception et décodage d'un éventuel message.
    const char* msg = radio.read_msg();
    switch (parser.parse(msg)) {
        case CommandParser::Kind::Ping:
            digitalWrite(LED_BUILTIN_PIN, HIGH);
            Serial.println(radio.answer("ok", hostPort) ? "answer send" : "Error send answer");
            break;
        case CommandParser::Kind::Stop:
            pilot_stop = true;
            break;
        default:
            break;
    }

    // 2. Lecture de l'orientation.
    data_imu orientation = imu.get_orientation();

    // 3. Sécurité : inclinaison excessive ou "stop" pilote → arrêt des moteurs.
    if (imu.inEmergency() || pilot_stop) {
        //motors.stop();
        digitalWrite(LED_BUILTIN_PIN, LOW);
        //while (1) {}
    }

    // 4. Stabilisation : calcul de la consigne et envoi aux moteurs.
    motor_cmd cmd;
    if (flight.update(parser.rc(), orientation, parser.pid(), cmd)) {
        motors.write(cmd);
    }

    // 5. Télémétrie : angles mesurés et corrections moteurs, poussés en UDP.
    static unsigned long last_telemetry_ms = 0;
    unsigned long now_ms = millis();
    if (now_ms - last_telemetry_ms >= TELEMETRY_PERIOD_MS) {
        last_telemetry_ms = now_ms;

        const motor_cmd& corr = flight.motor_correction();
        char line[200];
        snprintf(line, sizeof(line),
                 "ROLL: %.2f deg, PITCH: %.2f deg"
                 " | RA:%.2f PA:%.2f"
                 " | CR:%.2f CP:%.2f"
                 " | M1:%d M2:%d M3:%d M4:%d\n",
                 orientation.roll_deg, orientation.pitch_deg,
                 imu.rollAcc(), imu.pitchAcc(),
                 flight.roll_correction(), flight.pitch_correction(),
                 corr.motor_1_duty, corr.motor_2_duty,
                 corr.motor_3_duty, corr.motor_4_duty);

        radio.broadcast(line, hostPort);
    }
}
