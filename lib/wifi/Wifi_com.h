#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

// ============================================================================
//  drone_connect : couche transport pure.
//  Crée un point d'accès WiFi et échange des messages texte par UDP.
//  Ne connaît rien du protocole applicatif (commandes, PID, moteurs) :
//  read_msg() renvoie la chaîne brute, le décodage est fait ailleurs.
// ============================================================================
class drone_connect {

private:
    IPAddress ip_esp;            // IP du point d'accès (ESP32)
    IPAddress ip_client;         // IP du dernier client ayant émis
    char      packetBuffer[255]; // tampon de réception UDP
    WiFiUDP   UDP;

public:
    // Démarre le point d'accès et ouvre le port UDP de réception.
    void init_wifi(const char* ssid, const char* pass, uint16_t port);

    // Renvoie le dernier paquet reçu (chaîne terminée par '\0'),
    // ou "" si aucun paquet n'est disponible.
    const char* read_msg();

    // Renvoie un message texte au dernier client connu.
    bool answer(const char* msg, uint16_t port);

    // Envoie 4 valeurs flottantes (télémétrie) au client par défaut.
    bool answer_values(float v1, float v2, float v3, float v4, uint16_t port);
};
