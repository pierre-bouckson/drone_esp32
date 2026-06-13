#pragma once
#include <Arduino.h>
#include "drone_types.h"

// ============================================================================
//  CommandParser : décodage du protocole texte reçu par UDP.
//  Transforme les lignes brutes en données structurées et conserve le dernier
//  état connu (commande pilote, coefficients PID, gain). Aucune dépendance au
//  transport : on lui passe la chaîne, il met à jour son état.
//
//  Messages reconnus :
//    "command"              → ping (demande d'accusé de réception)
//    "rc <l> <f> <u> <yaw>" → commande pilote
//    "gain <g>"             → gain
//    "pid <kp> <ki> <kd>"   → coefficients PID
// ============================================================================
class CommandParser {

public:
    enum class Kind { None, Ping, Rc, Gain, Pid };

    // Décode un message et met à jour l'état interne. Renvoie son type.
    Kind parse(const char* msg);

    const msg_rc&   rc()   const { return rc_; }
    const coef_pid& pid()  const { return coef_; }
    int             gain() const { return gain_; }

private:
    msg_rc   rc_   = {0, 0, 0, 0};
    coef_pid coef_ = {0.0f, 0.0f, 0.0f};
    int      gain_ = 0;
};
