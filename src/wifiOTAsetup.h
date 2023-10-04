#ifndef wifiOTAsetup_H
#define wifiOTAsetup_H

#include <Arduino.h>
#include "LittleFS.h" // LittleFS is declared
#ifdef ESP32

#include <WiFi.h>
#endif

#ifdef ESP8266

#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#endif

#include <WiFiUdp.h>
#include <ArduinoOTA.h>
// #include <ESP8266Ping.h>


void wifiSetup();
void otaSetup();



#endif
