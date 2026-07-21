#include "motor_controller.h"

void motor_controller::motor_init() {
    ledcSetup(ledChannel, freq, resolution);
    ledcSetup(ledChanne2, freq, resolution);
    ledcSetup(ledChanne3, freq, resolution);
    ledcSetup(ledChanne4, freq, resolution);

    ledcAttachPin(PIN_motor_1, ledChannel);
    ledcAttachPin(PIN_motor_2, ledChanne2);
    ledcAttachPin(PIN_motor_3, ledChanne3);
    ledcAttachPin(PIN_motor_4, ledChanne4);

    // Les ESC refusent d'armer tant qu'ils n'ont pas vu le mini gaz : on le
    // maintient quelques secondes avant de rendre la main à la boucle de vol.
    stop();
    delay(ARM_DELAY_MS);
}

int motor_controller::clamp(int duty) {
    if (duty > DUTY_MAX) return DUTY_MAX;
    if (duty < DUTY_MIN) return DUTY_MIN;
    return duty;
}

int motor_controller::to_ticks(int duty) {
    duty = clamp(duty);
    return TICKS_MIN + (duty * (TICKS_MAX - TICKS_MIN)) / DUTY_MAX;
}

void motor_controller::write(const motor_cmd& cmd) {
    ledcWrite(ledChannel, to_ticks(cmd.motor_1_duty));
    ledcWrite(ledChanne2, to_ticks(cmd.motor_2_duty));
    ledcWrite(ledChanne3, to_ticks(cmd.motor_3_duty));
    ledcWrite(ledChanne4, to_ticks(cmd.motor_4_duty));
}

void motor_controller::stop() {
    // Mini gaz et non 0 : couper le signal remettrait les ESC en alarme.
    ledcWrite(ledChannel, TICKS_MIN);
    ledcWrite(ledChanne2, TICKS_MIN);
    ledcWrite(ledChanne3, TICKS_MIN);
    ledcWrite(ledChanne4, TICKS_MIN);
}
