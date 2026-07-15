/*
 * ESC Controller - RUIZHI 40A Brushless ESC
 * ESP32 DEVKIT - Arduino IDE
 *
 * Protocole standard RC PWM :
 *   - Fréquence : 50 Hz (période 20 ms)
 *   - 1000 µs → moteur arrêté (armé)
 *   - 2000 µs → plein gaz
 *
 * Câblage :
 *   ESC fil signal (blanc/jaune) → GPIO 18 (ESC_PIN)
 *   ESC fil rouge (5V BEC)       → optionnel, peut alimenter l'ESP32 via VIN
 *   ESC fil noir (GND)           → GND ESP32
 */

// ──────────────────────────────────────────────
//  Configuration
// ──────────────────────────────────────────────
#define ESC_PIN        18      // Broche signal PWM vers l'ESC
#define LEDC_CHANNEL   0       // Canal LEDC de l'ESP32 (0-15)
#define LEDC_FREQ_HZ   50      // 50 Hz standard RC
#define LEDC_RES_BITS  16      // Résolution 16 bits → 0..65535

// Largeurs d'impulsion en µs
#define PULSE_MIN_US   1000    // Moteur arrêté / armement
#define PULSE_MAX_US   2000    // Plein gaz
#define PULSE_ARM_US   1000    // Valeur d'armement (identique à MIN)

// Période en µs pour une fréquence de 50 Hz
#define PERIOD_US      (1000000 / LEDC_FREQ_HZ)   // 20 000 µs

// ──────────────────────────────────────────────
//  Conversion µs → duty cycle 16 bits
// ──────────────────────────────────────────────
uint32_t usToDuty(uint16_t pulseUs) {
  // duty = (pulseUs / périodeUs) * (2^résolution - 1)
  return (uint32_t)pulseUs * ((1UL << LEDC_RES_BITS) - 1) / PERIOD_US;
}

// ──────────────────────────────────────────────
//  Envoi d'une impulsion PWM
// ──────────────────────────────────────────────
void setESC_us(uint16_t pulseUs) {
  pulseUs = constrain(pulseUs, PULSE_MIN_US, PULSE_MAX_US);
  ledcWrite(ESC_PIN, usToDuty(pulseUs));  // v3.x : on passe la broche, pas le canal
}

// Throttle en pourcentage 0-100 %
void setESC_percent(float percent) {
  percent = constrain(percent, 0.0f, 100.0f);
  uint16_t pulse = (uint16_t)(PULSE_MIN_US + (percent / 100.0f) * (PULSE_MAX_US - PULSE_MIN_US));
  setESC_us(pulse);
}

// ──────────────────────────────────────────────
//  Séquence d'armement
//  ⚠ Effectuer sans hélice / prop-guard en place
// ──────────────────────────────────────────────
void armESC() {
  Serial.println("[ESC] Armement : envoi 1000 µs pendant 3 s...");
  setESC_us(PULSE_ARM_US);
  delay(3000);
  Serial.println("[ESC] Armement terminé. Moteur prêt.");
}

// ──────────────────────────────────────────────
//  Setup
// ──────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Serial.println("=== ESC Controller ESP32 ===");

  // Initialisation du canal LEDC (API v3.x)
  ledcAttach(ESC_PIN, LEDC_FREQ_HZ, LEDC_RES_BITS);

  // Forcer le signal à 1000 µs avant d'alimenter l'ESC
  setESC_us(PULSE_MIN_US);
  Serial.println("[ESC] Signal à 1000 µs. Alimentez l'ESC maintenant.");
  Serial.println("      Attendez les bips d'armement, puis envoyez 'a' pour armer.");

  printHelp();
}

// ──────────────────────────────────────────────
//  Loop : contrôle via port série
// ──────────────────────────────────────────────
void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd == "a") {
      // Armement
      armESC();

    } else if (cmd.startsWith("t")) {
      // Throttle en % : "t50" → 50 %
      float pct = cmd.substring(1).toFloat();
      setESC_percent(pct);
      Serial.printf("[ESC] Throttle %.1f %% → %d µs\n",
                    pct,
                    (int)(PULSE_MIN_US + (pct / 100.0f) * (PULSE_MAX_US - PULSE_MIN_US)));

    } else if (cmd.startsWith("p")) {
      // Impulsion directe en µs : "p1500"
      uint16_t us = (uint16_t)cmd.substring(1).toInt();
      setESC_us(us);
      Serial.printf("[ESC] Impulsion directe : %d µs\n", us);

    } else if (cmd == "s") {
      // Stop immédiat
      setESC_us(PULSE_MIN_US);
      Serial.println("[ESC] STOP → 1000 µs");

    } else if (cmd == "h") {
      printHelp();

    } else {
      Serial.println("Commande inconnue. Tapez 'h' pour l'aide.");
    }
  }
}

// ──────────────────────────────────────────────
//  Aide
// ──────────────────────────────────────────────
void printHelp() {
  Serial.println("-----------------------------");
  Serial.println("Commandes disponibles :");
  Serial.println("  a        → Armer l'ESC (3 s à 1000 µs)");
  Serial.println("  tXX      → Throttle XX % (ex: t30 = 30 %)");
  Serial.println("  pXXXX    → Impulsion directe en µs (ex: p1500)");
  Serial.println("  s        → Stop (retour à 1000 µs)");
  Serial.println("  h        → Cette aide");
  Serial.println("-----------------------------");
}
