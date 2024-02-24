/*
This is a rework of the rework (https://github.com/bhboyle/Intelligent_Hotplate) of the Electronoobs DIY reflow hotplate project found here. https://electronoobs.com/eng_arduino_tut155.php

*/

#include <Arduino.h>
#include <Wire.h>
#include <PID_v1.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_ADS1X15.h>
#include <SPI.h>
#include <OSCMessage.h>
#include <iostream>
#include "RunningAverage.h"

#include "wifiOTAsetup.h"

Adafruit_ADS1115 ads;
const int numChannels = 4; // Channel 1 of adc is thermistor, channel 2 is reference 3V3
int adc[numChannels];
float volts[numChannels];

#define LEDPIN 4
// #define

double Setpoint, Input, Output; // variables needed for the PID loop
int temperatureSetPoint = 0;

// TODO: rewrite to use taskcheduler or something
unsigned long LEDtimer;                      // used to determine when to update the LEDs
unsigned long LEDinterval = 500;             // how often in milliseconds to update the LEDs
unsigned int millis_before, millis_before_2; // used for time tracking in the loop
unsigned int millis_now = 0;                 // used to keep track of the current time of the loop
unsigned long previousMillis = 0;
unsigned long tempTimer;

#define EVERY_N_MILLIS(identifier, interval)                     \
    static unsigned long previousMillisTimer_##identifier = 0;   \
    if (millis() - previousMillisTimer_##identifier >= interval) \
    {                                                            \
        previousMillisTimer_##identifier = millis();

WiFiUDP OscUDP;
const IPAddress outIp(192, 168, 5, 143); // TODO: set ip with a message
const unsigned int outPort = 10001;      // OpenStageControl port
const unsigned int localPort = 8888;     // local receive port

PID myPID(&Input, &Output, &Setpoint, 1.5, 0.005, 0.1, DIRECT); // create the PID object

void sendMessageString(const char *address, const char *message)
{
    OSCMessage msg(address);
    msg.add(message);

    OscUDP.beginPacket(outIp, outPort);
    msg.send(OscUDP);
    OscUDP.endPacket();
    msg.empty();
}

void sendMessage(const char *address, int value)
{
    OSCMessage msg(address);
    msg.add(value);

    OscUDP.beginPacket(outIp, outPort);
    msg.send(OscUDP);
    OscUDP.endPacket();
    msg.empty();
}

void sendMessageFloat(const char *address, float value)
{
    OSCMessage msg(address);
    msg.add(value);

    OscUDP.beginPacket(outIp, outPort);
    msg.send(OscUDP);
    OscUDP.endPacket();
    msg.empty();
}

void setup()
{
    Serial.begin(115200);

    wifiSetup();
    OscUDP.begin(localPort);

    pinMode(LEDPIN, OUTPUT);
    digitalWrite(LEDPIN, HIGH);

    otaSetup();
}

void loop()
{

    // ArduinoOTA.handle();

    OSCMessage rcvmsg;
    int size = OscUDP.parsePacket();

    if (size > 0)
    {

        while (size--)
            rcvmsg.fill(OscUDP.read());

        if (!rcvmsg.hasError())
        {

            if (rcvmsg.fullMatch("/set/dutycycle"))
            {
                analogWrite(LEDPIN, rcvmsg.getInt(0));
            }

            if (rcvmsg.fullMatch("/set/freq"))
            {
                analogWriteFreq(rcvmsg.getInt(0));


            }

        }

        else
        {
            Serial.print("error: ");
            sendMessageString("/ok", "not ok");
        }
    }
}
