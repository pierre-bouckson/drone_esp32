#pragma once
#include "drone_types.h"

// ============================================================================
//  pid : régulateurs proportionnel / intégral / dérivé.
//  Classe purement calculatoire : elle ne connaît ni l'IMU ni les moteurs,
//  elle transforme une erreur en commande. Chaque régulateur conserve son
//  propre état (intégrale, erreur précédente) sous forme de membres.
// ============================================================================
class pid {

private:
    // État des boucles d'attitude (intégrale).
    float att_pitch_sum = 0.0f;
    float att_roll_sum  = 0.0f;

    // État des boucles de vitesse angulaire (intégrale + erreur précédente).
    float rate_pitch_sum  = 0.0f;
    float rate_pitch_prev = 0.0f;
    float rate_roll_sum   = 0.0f;
    float rate_roll_prev  = 0.0f;

public:
    // Boucle d'attitude (P + I) : transforme une erreur d'angle en consigne.
    float pi_attitude_pitch(float erreur, float kp, float ki, float dt);
    float pi_attitude_roll(float erreur, float kp, float ki, float dt);

    // Boucle de vitesse angulaire (P + I + D).
    float pid_rate_pitch(float erreur, coef_pid pid, float dt);
    float pid_rate_roll(float erreur, coef_pid pid, float dt);
};
