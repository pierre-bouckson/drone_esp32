#pragma once

// ============================================================================
//  Filtre de Kalman 1D pour fusionner un angle (accéléromètre) et une
//  vitesse angulaire (gyroscope) en estimant aussi le biais du gyro.
//  Helper autonome, réutilisé pour le roll et le pitch.
// ============================================================================
struct KalmanFilter {
  float angle;
  float bias;
  float P[2][2];
  float Q_angle;
  float Q_bias;
  float R_meas;

  KalmanFilter() {
    angle   = 0.0f; bias    = 0.0f;
    P[0][0] = 0.0f; P[0][1] = 0.0f;
    P[1][0] = 0.0f; P[1][1] = 0.0f;
    Q_angle = 0.001f;
    Q_bias  = 0.003f;
    R_meas  = 0.03f;
  }

  // newAngle : angle mesuré (accéléromètre), newRate : vitesse (gyro), dt en s.
  float update(float newAngle, float newRate, float dt) {
    // Prédiction
    float rate = newRate - bias;
    angle += dt * rate;
    P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_bias * dt;

    // Correction
    float S  = P[0][0] + R_meas;
    float K0 = P[0][0] / S;
    float K1 = P[1][0] / S;
    float y  = newAngle - angle;
    angle += K0 * y;
    bias  += K1 * y;
    float P00_tmp = P[0][0], P01_tmp = P[0][1];
    P[0][0] -= K0 * P00_tmp;
    P[0][1] -= K0 * P01_tmp;
    P[1][0] -= K1 * P00_tmp;
    P[1][1] -= K1 * P01_tmp;

    return angle;
  }
};
