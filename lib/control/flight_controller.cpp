#include "flight_controller.h"

motor_cmd FlightController::mix(float roll_corr, float pitch_corr) {
    // Disposition en X : chaque moteur reçoit une combinaison des corrections.
    motor_cmd cmd;
    cmd.motor_1_duty = -pitch_corr + roll_corr;
    cmd.motor_2_duty = -pitch_corr - roll_corr;
    cmd.motor_3_duty =  pitch_corr - roll_corr;
    cmd.motor_4_duty =  pitch_corr + roll_corr;
    return cmd;
}

bool FlightController::update(const msg_rc& rc, const data_imu& orientation,
                             const coef_pid& coef, motor_cmd& out) {
    unsigned long now = micros();

    if (last_time == 0) {
        last_time = now;
        return false;   // premier tour : on attend d'avoir un vrai dt
    }

    float dt = (now - last_time) / 1000000.0f;
    last_time = now;
    if (dt <= 0 || dt > DT_MAX) {
        return false;   // pas de temps aberrant : on saute ce cycle
    }

    // Erreur d'angle = consigne pilote − orientation mesurée (toutes deux en degrés).
    float erreur_pitch = rc.left / STICK_TO_ANGLE    - orientation.pitch_deg;
    float erreur_roll  = -rc.forward / STICK_TO_ANGLE - orientation.roll_deg;

    // Régulateurs PID de vitesse angulaire.
    float corr_roll  = pid_.pid_rate_roll(erreur_roll,  coef, dt);
    float corr_pitch = pid_.pid_rate_pitch(erreur_pitch, coef, dt);

    // Mixage des corrections puis ajout des gaz.
    motor_cmd corr = mix(corr_roll, corr_pitch);
    int throttle = rc.up;

    out.motor_1_duty = throttle * THROTTLE_GAIN + corr.motor_1_duty;
    out.motor_2_duty = throttle * THROTTLE_GAIN + corr.motor_2_duty;
    out.motor_3_duty = throttle * THROTTLE_GAIN + corr.motor_3_duty;
    out.motor_4_duty = throttle * THROTTLE_GAIN + corr.motor_4_duty;
    return true;
}
