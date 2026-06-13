#include "pid.h"

float pid::pi_attitude_pitch(float erreur, float kp, float ki, float dt) {
    att_pitch_sum += erreur * dt;          // intégrale
    return kp * erreur;                    // (+ ki * att_pitch_sum) désactivé
}

float pid::pi_attitude_roll(float erreur, float kp, float ki, float dt) {
    att_roll_sum += erreur * dt;           // intégrale
    return kp * erreur;                    // (+ ki * att_roll_sum) désactivé
}

float pid::pid_rate_pitch(float erreur, coef_pid pid, float dt) {
    rate_pitch_sum += erreur * dt;                              // intégrale
    float deriv = (erreur - rate_pitch_prev) / dt;             // dérivée
    rate_pitch_prev = erreur;
    return pid.kp * erreur + pid.ki * rate_pitch_sum + pid.kd * deriv;
}

float pid::pid_rate_roll(float erreur, coef_pid pid, float dt) {
    rate_roll_sum += erreur * dt;                              // intégrale
    float deriv = (erreur - rate_roll_prev) / dt;             // dérivée
    rate_roll_prev = erreur;
    return pid.kp * erreur + pid.ki * rate_roll_sum + pid.kd * deriv;
}
