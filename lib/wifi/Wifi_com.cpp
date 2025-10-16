#include "Wifi_com.h"



void drone_connect::init_wifi(const char* ssid, const char* pass, uint16_t port) {
    
    WiFi.mode(WIFI_AP);                               // Mode Point d'accès
    WiFi.softAP(ssid, pass);
    ip_esp = WiFi.softAPIP();                   // Mon adresse IP
    Serial.print("AP IP address: ");
    Serial.println(ip_esp);

    UDP.begin(port);
}

const char* drone_connect::read_msg() {
    
    int packetSize = UDP.parsePacket();

    if(packetSize){
        Serial.print("Received packet from : "); 
        ip_client = UDP.remoteIP();          // De quelle IP msg recu ?
        Serial.print(ip_client);
        Serial.print(" of : ");
        Serial.println(packetSize);

        int len = UDP.read(packetBuffer, 255);          // Met le msg UDP dans le buffer
        if(len > 0) packetBuffer[len] = 0;    // Mettre au format le tableau pour printf
        Serial.printf("Data : %s\n\r", packetBuffer);
        return packetBuffer;
    }
    else{
        return "";
    }
}

bool drone_connect::answer(const char* msg, uint16_t port) {
    if (UDP.beginPacket(ip_client, port) != 1) return false;
    UDP.print(msg);                                // Repond OK
    UDP.endPacket();
    return true;
}

bool drone_connect::answer_values(float v1, float v2, float v3, float v4, uint16_t port) {
    if (UDP.beginPacket("192.168.4.2", port) != 1) return false;

    // Format texte : "123,456,789,42\n"
    char buffer[64];
    snprintf(buffer, sizeof(buffer), "%.6f,%.6f,%.6f,%.6f\n", v1, v2, v3, v4);

    UDP.print(buffer);
    UDP.endPacket();
    return true;
}