#pragma once
#include <Arduino.h>
#include "drone_types.h"

// ============================================================================
//  motor_controller : couche matérielle des moteurs.
//  Configure les 4 sorties PWM (LEDC) et applique une consigne de rapport
//  cyclique. Ne fait aucun calcul de vol : il sature simplement la commande
//  dans la plage valide avant de l'écrire.
// ============================================================================
class motor_controller {

private:
    // Broches de commande des 4 ESC / moteurs.
    static constexpr int PIN_motor_1 = 23;
    static constexpr int PIN_motor_2 = 15;
    static constexpr int PIN_motor_3 = 13;
    static constexpr int PIN_motor_4 = 33;

    static constexpr int freq       = 32000;  // fréquence PWM (Hz)
    static constexpr int resolution = 8;       // résolution PWM (bits) → 0..255

    // Un canal LEDC par moteur.
    static constexpr int ledChannel = 0;
    static constexpr int ledChanne2 = 1;
    static constexpr int ledChanne3 = 2;
    static constexpr int ledChanne4 = 3;

    static constexpr int DUTY_MIN = 0;
    static constexpr int DUTY_MAX = 255;

    static int clamp(int duty);

public:
    // Configure les canaux PWM et met tous les moteurs à l'arrêt.
    void motor_init();

    // Applique une consigne aux 4 moteurs (valeurs saturées dans 0..255).
    void write(const motor_cmd& cmd);

    // Coupe immédiatement les 4 moteurs.
    void stop();
};
