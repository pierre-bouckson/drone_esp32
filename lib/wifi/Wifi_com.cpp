#include "Wifi_com.h"

void drone_connect::init_wifi(const char* ssid, const char* pass, uint16_t port) {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(ssid, pass);
    ip_esp = WiFi.softAPIP();
    Serial.print("AP IP address: ");
    Serial.println(ip_esp);

    UDP.begin(port);
}

const char* drone_connect::read_msg() {
    int packetSize = UDP.parsePacket();
    if (!packetSize) {
        return "";
    }

    ip_client = UDP.remoteIP();   // mémorise l'expéditeur pour pouvoir lui répondre
    Serial.print("Received packet from : ");
    Serial.print(ip_client);
    Serial.print(" of : ");
    Serial.println(packetSize);

    int len = UDP.read(packetBuffer, sizeof(packetBuffer) - 1);
    if (len > 0) packetBuffer[len] = '\0';   // termine la chaîne pour printf/sscanf
    Serial.printf("Data : %s\n\r", packetBuffer);
    return packetBuffer;
}

bool drone_connect::answer(const char* msg, uint16_t port) {
    if (UDP.beginPacket(ip_client, port) != 1) return false;
    UDP.print(msg);
    return UDP.endPacket() == 1;
}

bool drone_connect::broadcast(const char* msg, uint16_t port) {
    // Toujours en diffusion sur le sous-réseau de l'AP (192.168.4.255) plutôt
    // qu'au dernier client : plusieurs outils d'analyse peuvent ainsi écouter
    // en parallèle, sans avoir à émettre quoi que ce soit au préalable.
    IPAddress dest(ip_esp[0], ip_esp[1], ip_esp[2], 255);

    if (UDP.beginPacket(dest, port) != 1) return false;
    UDP.print(msg);
    return UDP.endPacket() == 1;
}
