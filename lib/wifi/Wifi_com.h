#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>


class drone_connect {

private:
    IPAddress ip_esp;
    IPAddress ip_client;
    char packetBuffer[255];
    bool state_drone;
    WiFiUDP UDP;

public:
    void init_wifi(const char* ssid, const char* pass, uint16_t port);
    const char* read_msg();
    bool answer(const char* msg, uint16_t port); 
    
};