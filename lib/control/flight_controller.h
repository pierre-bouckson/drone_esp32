#pragma once
#include <Arduino.h>
#include "drone_types.h"
#include "pid.h"

// ============================================================================
//  FlightController : boucle de stabilisation du drone.
//  À partir de la commande pilote (msg_rc) et de l'orientation mesurée
//  (data_imu), il calcule l'erreur, applique les régulateurs PID de vitesse,
//  mixe les corrections sur les 4 moteurs et ajoute les gaz.
//
//  Il ne touche ni au capteur ni au matériel moteur : il reçoit des données
//  et renvoie une consigne. La gestion du pas de temps (dt) est interne.
// ============================================================================
class FlightController {

private:
    static constexpr float THROTTLE_GAIN = 2.5f;   // gain appliqué aux gaz
    static constexpr float STICK_TO_ANGLE = 20.0f; // sensibilité des sticks roll/pitch
    static constexpr float DT_MAX = 0.05f;         // 50 ms : au-delà on saute le tour

    pid pid_;
    unsigned long last_time = 0;

    // Convertit les corrections de roll/pitch en contributions par moteur.
    static motor_cmd mix(float roll_corr, float pitch_corr);

public:
    // Calcule la consigne moteur pour ce cycle.
    // Renvoie false (et ne remplit pas `out`) si le pas de temps est invalide
    // (premier appel ou dt hors plage) : la commande précédente reste alors active.
    bool update(const msg_rc& rc, const data_imu& orientation,
                const coef_pid& coef, motor_cmd& out);
};
