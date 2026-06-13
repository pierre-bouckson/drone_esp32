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
    UDP.endPacket();
    return true;
}

bool drone_connect::answer_values(float v1, float v2, float v3, float v4, uint16_t port) {
    if (UDP.beginPacket("192.168.4.2", port) != 1) return false;

    // Format texte : "v1,v2,v3,v4\n"
    char buffer[64];
    snprintf(buffer, sizeof(buffer), "%.6f,%.6f,%.6f,%.6f\n", v1, v2, v3, v4);

    UDP.print(buffer);
    UDP.endPacket();
    return true;
}
