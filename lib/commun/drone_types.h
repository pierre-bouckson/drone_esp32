#pragma once

// ============================================================================
//  Types de données partagés entre les modules du firmware.
//  Aucune logique ici : uniquement des structures simples (POD) afin que
//  chaque module puisse les manipuler sans dépendre des autres.
// ============================================================================

// Commande pilote reçue par radio (canaux de la télécommande).
struct msg_rc {
  int left;     // translation gauche / droite
  int forward;  // translation avant / arrière
  int up;       // gaz (poussée verticale)
  int yaw;      // rotation autour de l'axe vertical
};

// Coefficients d'un régulateur PID.
struct coef_pid {
  float kp;
  float ki;
  float kd;
};

// Consigne de rapport cyclique (0-255) envoyée aux 4 moteurs.
struct motor_cmd {
  int motor_1_duty;
  int motor_2_duty;
  int motor_3_duty;
  int motor_4_duty;
};

// Orientation du drone estimée par l'IMU, en degrés.
struct data_imu {
  float roll_deg;   // rotation autour de X
  float pitch_deg;  // rotation autour de Y
  float yaw_deg;    // rotation autour de Z (dérive sans magnétomètre)
};
