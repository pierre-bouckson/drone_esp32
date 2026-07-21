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

    // Signal type servo attendu par les ESC : 50 Hz, impulsion 1000..2000 us.
    static constexpr int freq       = 50;  // fréquence PWM (Hz)
    static constexpr int resolution = 16;  // résolution PWM (bits)

    // Un canal LEDC par moteur.
    static constexpr int ledChannel = 0;
    static constexpr int ledChanne2 = 1;
    static constexpr int ledChanne3 = 2;
    static constexpr int ledChanne4 = 3;

    // Consigne d'entrée : plage historique 0..255 produite par la stabilisation.
    static constexpr int DUTY_MIN = 0;
    static constexpr int DUTY_MAX = 255;

    // Largeurs d'impulsion converties en pas LEDC.
    // Période = 20000 us sur 65536 pas → 1 us vaut 3.2768 pas.
    static constexpr int PULSE_MIN_US = 1000;  // moteurs arrêtés, ESC armé
    static constexpr int PULSE_MAX_US = 2000;  // plein gaz
    static constexpr int TICKS_MIN = (int)((int64_t)PULSE_MIN_US * (1 << resolution) / 20000);
    static constexpr int TICKS_MAX = (int)((int64_t)PULSE_MAX_US * (1 << resolution) / 20000);

    // Durée de maintien du mini gaz au démarrage pour laisser les ESC s'armer.
    static constexpr int ARM_DELAY_MS = 3000;

    static int clamp(int duty);

    // Convertit une consigne 0..255 en pas LEDC dans la plage servo.
    static int to_ticks(int duty);

public:
    // Configure les canaux PWM et met tous les moteurs à l'arrêt.
    void motor_init();

    // Applique une consigne aux 4 moteurs (valeurs saturées dans 0..255).
    void write(const motor_cmd& cmd);

    // Coupe immédiatement les 4 moteurs.
    void stop();
};
