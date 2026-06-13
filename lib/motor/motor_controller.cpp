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

    stop();
}

int motor_controller::clamp(int duty) {
    if (duty > DUTY_MAX) return DUTY_MAX;
    if (duty < DUTY_MIN) return DUTY_MIN;
    return duty;
}

void motor_controller::write(const motor_cmd& cmd) {
    ledcWrite(ledChannel, clamp(cmd.motor_1_duty));
    ledcWrite(ledChanne2, clamp(cmd.motor_2_duty));
    ledcWrite(ledChanne3, clamp(cmd.motor_3_duty));
    ledcWrite(ledChanne4, clamp(cmd.motor_4_duty));
}

void motor_controller::stop() {
    ledcWrite(ledChannel, 0);
    ledcWrite(ledChanne2, 0);
    ledcWrite(ledChanne3, 0);
    ledcWrite(ledChanne4, 0);
}
