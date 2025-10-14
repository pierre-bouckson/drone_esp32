#include "pid.h"


motor_cmd pid_duty_;

motor_cmd pid::trad_motor(data_imu intput) {

    pid_duty_.motor_1_duty = -intput.pitch_deg + intput.roll_deg;
    pid_duty_.motor_2_duty = -intput.pitch_deg + -intput.roll_deg;
    pid_duty_.motor_3_duty = intput.pitch_deg + -intput.roll_deg;
    pid_duty_.motor_4_duty = intput.pitch_deg + intput.roll_deg;

    
    return pid_duty_;
}


float pid::pi_attitude_pitch(float erreur, float kp, float ki, float dt) {
    static float erreur_prec = 0.0f;
    static float somme_erreurs = 0.0f;

    // Intégrale
    somme_erreurs += erreur * dt;


    // PID
    return kp * erreur ;//+ ki * somme_erreurs;
}

float pid::pi_attitude_roll(float erreur, float kp, float ki, float dt) {
    static float erreur_prec = 0.0f;
    static float somme_erreurs = 0.0f;

    // Intégrale
    somme_erreurs += erreur * dt;


    // PID
    return kp * erreur ;//+ ki * somme_erreurs;
}

float pid::pid_rate_pitch(float erreur, coef_pid pid, float dt) {
    static float erreur_prec = 0.0f;
    static float somme_erreurs = 0.0f;

    // Intégrale
    somme_erreurs += erreur * dt;

    // Dérivée
    float deriv = (erreur - erreur_prec) / dt;
    erreur_prec = erreur;

    // PID
    return pid.kp * erreur + pid.ki * somme_erreurs + pid.kd * deriv;
}

float pid::pid_rate_roll(float erreur, coef_pid pid, float dt) {
    static float erreur_prec = 0.0f;
    static float somme_erreurs = 0.0f;

    // Intégrale
    somme_erreurs += erreur * dt;

    // Dérivée
    float deriv = (erreur - erreur_prec) / dt;
    erreur_prec = erreur;

    // PID
    return pid.kp * erreur + pid.ki * somme_erreurs + pid.kd * deriv;
}