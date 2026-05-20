#include "motor_controller.h"
#include "pid.h"
#include "imu.h"

int motor1 = 0;
int motor2 = 0;

// ─── Helpers RC PWM ───────────────────────────────────────────────────────────

uint32_t motor_controller::usToDuty(int pulseUs) {
    return (uint32_t)pulseUs * ((1UL << LEDC_RES_BITS) - 1) / PERIOD_US;
}

void motor_controller::writeMotor(int channel, int pulseUs) {
    pulseUs = constrain(pulseUs, PULSE_MIN_US, PULSE_MAX_US);
    ledcWrite(channel, usToDuty(pulseUs));
}

// ─── Init ─────────────────────────────────────────────────────────────────────

void motor_controller::motor_init() {
    ledcSetup(CH_motor_1, LEDC_FREQ_HZ, LEDC_RES_BITS);
    ledcAttachPin(PIN_motor_1, CH_motor_1);
    ledcSetup(CH_motor_2, LEDC_FREQ_HZ, LEDC_RES_BITS);
    ledcAttachPin(PIN_motor_2, CH_motor_2);
    ledcSetup(CH_motor_3, LEDC_FREQ_HZ, LEDC_RES_BITS);
    ledcAttachPin(PIN_motor_3, CH_motor_3);
    ledcSetup(CH_motor_4, LEDC_FREQ_HZ, LEDC_RES_BITS);
    ledcAttachPin(PIN_motor_4, CH_motor_4);

    // Signal à 1000 µs obligatoire avant d'alimenter les ESCs
    writeMotor(CH_motor_1, PULSE_MIN_US);
    writeMotor(CH_motor_2, PULSE_MIN_US);
    writeMotor(CH_motor_3, PULSE_MIN_US);
    writeMotor(CH_motor_4, PULSE_MIN_US);

    Serial.println("[ESC] Signal a 1000 us. Alimentez les ESCs maintenant.");
    Serial.println("[ESC] Armement dans 3 s...");
    delay(3000);
    Serial.println("[ESC] Moteurs armes. Prets.");
}

// ─── Mixing RC simple (sans PID) ─────────────────────────────────────────────

motor_cmd motor_controller::cmd_vel() {
    // up 0-100 → base 1000-2000 µs ; forward/left ±100 → ±200 µs
    int base = PULSE_MIN_US + msg_rc_.up * 10;
    commande_rc_.motor_1_duty = base - msg_rc_.forward * 2 - msg_rc_.left * 2;
    commande_rc_.motor_2_duty = base + msg_rc_.forward * 2 - msg_rc_.left * 2;
    commande_rc_.motor_3_duty = base + msg_rc_.forward * 2 + msg_rc_.left * 2;
    commande_rc_.motor_4_duty = base - msg_rc_.forward * 2 + msg_rc_.left * 2;
    return commande_rc_;
}

// ─── Boucle de contrôle (PID stabilisation) ───────────────────────────────────

void motor_controller::send_cmd() {

    data_imu orientation = imu_.get_orientation();

    if (emergency) {
        digitalWrite(2, LOW);
    }

    unsigned long now = micros();
    if (last_time == 0) {
        last_time = now;
        return;
    }

    float dt = (now - last_time) / 1000000.0f;
    last_time = now;
    if (dt <= 0 || dt > 0.05f) return;

    erreur_pitch = msg_rc_.left    / 20.0f - (orientation.pitch_deg * (180.0f / PI));
    erreur_roll  = -msg_rc_.forward / 20.0f - (orientation.roll_deg  * (180.0f / PI));

    erreur_rate.roll_deg  = pid_.pid_rate_roll (erreur_roll,  coef_udp, dt);
    erreur_rate.pitch_deg = pid_.pid_rate_pitch(erreur_pitch, coef_udp, dt);

    cmd_motor_rate = pid_.trad_motor(erreur_rate);

    // Base throttle : up 0-100 → 1000-2000 µs
    int base_us = PULSE_MIN_US + msg_rc_.up * 10;

    // Les corrections PID sont en µs (même échelle que le range 1000-2000)
    commande_final.motor_1_duty = base_us + cmd_motor_rate.motor_1_duty;
    commande_final.motor_2_duty = base_us + cmd_motor_rate.motor_2_duty;
    commande_final.motor_3_duty = base_us + cmd_motor_rate.motor_3_duty;
    commande_final.motor_4_duty = base_us + cmd_motor_rate.motor_4_duty;

    motor1 = commande_final.motor_1_duty;
    motor2 = commande_final.motor_2_duty;

    writeMotor(CH_motor_1, commande_final.motor_1_duty);
    writeMotor(CH_motor_2, commande_final.motor_2_duty);
    writeMotor(CH_motor_3, commande_final.motor_3_duty);
    writeMotor(CH_motor_4, commande_final.motor_4_duty);
}
