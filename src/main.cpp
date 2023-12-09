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
const int numChannels = 2; // Channel 1 of adc is thermistor, channel 2 is reference 3V3
int adc[numChannels];
float volts[numChannels];

#define SSR 16
#define MOSFET 18

double Setpoint, Input, Output; // variables needed for the PID loop
int temperatureSetPoint = 0;

// TODO: rewrite to use taskcheduler or something
unsigned long LEDtimer;                      // used to determine when to update the LEDs
unsigned long LEDinterval = 500;             // how often in milliseconds to update the LEDs
unsigned int millis_before, millis_before_2; // used for time tracking in the loop
unsigned int millis_now = 0;                 // used to keep track of the current time of the loop
unsigned long previousMillis = 0;
unsigned long tempTimer;

unsigned int refresh_rate = 1000; // rate of OSC messages with info

double temperature = 0; // this is the variable that holds the current temperature reading
int period = 5;
int temp_refresh_rate = 100; // Thermistor reading rate - currently simple nonblocking read without using interrupt
int numSamples = 50;
RunningAverage last5s(numSamples);

// TODO: rewrite reflow profile mode
unsigned int seconds = 0;      // used in the display to show how long the sequence has been running
bool but_1_state = false;      // used to track of the button has been pushed. used for debouncing the button
unsigned long but_1_timer = 0; // used for checking the time for debouncing the button push
int max_temp = 260;            // ****** this is the high temperature set point for SMD mode. *******
int heatStage = 0;             // used for keeping track of where we are in the heating sequence
int stage_2_set_point = 150;   // this is the "soak" temperature set point
unsigned long stage2Timer = 0; // used to keeping track of when we went into stage two
unsigned long stage4Timer = 0; // used for keeping track of when we went into stage 4
int soakTime = 100;            // how long to soak the board for in seconds
int reflowTime = 60;           // how long to reflow the board for in seconds
int cookMode = 1;              // This is used to know what mode is selected

int n = 0; // debug variable - received message count

int temp = 0;
int percent = 0; // power in percent mode

int pwmCycleTime = 1; // Initial SLOW PWM cycle time in seconds - used for mechanical relay or ssr
int dutyCycle = 0;    // used for both slow and normal pwm outs

const int mosfetPin = MOSFET; // variables for regular pwm out
const int mosfetFreq = 100;
const int mosfetChannel = 0;
const int mosfetResolution = 8;

bool button1 = 0;
bool button2 = 0;

boolean cookStatus = 0; // this is used to know if we are actively cooking

String Names[] = // this is the text displayed in the display in the various stages for SMD cooking mode
    {
        "Off",
        "Heat",
        "Soak",
        "Blast",
        "Reflow",
};

WiFiUDP Udp;
const IPAddress outIp(192, 168, 0, 137); // TODO: set ip with a message
const unsigned int outPort = 10001;      // OpenStageControl port
const unsigned int localPort = 8888;     // local receive port

PID myPID(&Input, &Output, &Setpoint, 1.5, 0.005, 0.1, DIRECT); // create the PID object

void sendMessageString(const char *address, const char *message)
{
    OSCMessage msg(address);
    msg.add(message);

    Udp.beginPacket(outIp, outPort);
    msg.send(Udp);
    Udp.endPacket();
    msg.empty();
}

void sendMessage(const char *address, int value)
{
    OSCMessage msg(address);
    msg.add(value);

    Udp.beginPacket(outIp, outPort);
    msg.send(Udp);
    Udp.endPacket();
    msg.empty();
}

void sendMessageFloat(const char *address, float value)
{
    OSCMessage msg(address);
    msg.add(value);

    Udp.beginPacket(outIp, outPort);
    msg.send(Udp);
    Udp.endPacket();
    msg.empty();
}

void slowPWM(int pin)
{

    if (dutyCycle < 0)
    {
        dutyCycle = 0;
    }
    else if (dutyCycle > 255)
    {
        dutyCycle = 255;
    }

    unsigned long currentMillis = millis();
    unsigned long pwmCycleTimeU = pwmCycleTime;

    if (currentMillis - previousMillis <= (dutyCycle / 255.0) * (pwmCycleTime * 1000))
    {
        digitalWrite(pin, LOW);
        sendMessage("/pwm", 1);
    }
    else
    {

        digitalWrite(pin, HIGH);
        sendMessage("/pwm", 0);
    }

    if (currentMillis - previousMillis >= pwmCycleTimeU * 1000)
    {
        previousMillis = currentMillis;
    }
}

void normalPwm(int dutyCycle)
{

    int dutyCycleInverted = 255 - dutyCycle;
    ledcWrite(mosfetChannel, dutyCycleInverted);
}

double ntcResistance;
double seriesResistor = 1971;
double beta = 3950;
double T25 = 25;
double R25 = 100000;

double calculateTemperature(double R)
{
    double temperatureKelvin = 1 / ((log(R / R25) / beta) + 1 / (T25 + 273.15));

    double temperatureCelsius = temperatureKelvin - 273.15;

    return temperatureCelsius;
}

int displayTemperature()
{

    double avgTemperature;

    if (millis() - tempTimer > temp_refresh_rate)
    {

        for (int i = 0; i < numChannels; i++)
        {
            adc[i] = ads.readADC_SingleEnded(i);
            volts[i] = ads.computeVolts(adc[i]);

            double refVoltage = volts[1];
            double ntcVoltage = volts[0];

            ntcResistance = ((refVoltage * seriesResistor) / ntcVoltage) - seriesResistor;

            temperature = calculateTemperature(ntcResistance);
        }
        avgTemperature = (last5s.getAverageSubset(40, 10));

        sendMessageFloat("/info/temperature", temperature);
        last5s.addValue(temperature);
        double rateOfChange = (last5s.getAverageSubset(40, 10) - last5s.getAverageSubset(0, 10)) / 5;

        sendMessageFloat("/info/avgtemperature", avgTemperature);

        sendMessageFloat("/info/temperature/rate", rateOfChange);
        sendMessageFloat("/info/time", millis());

        tempTimer = millis();
    }
    return avgTemperature;
}

void displayMode()
{
    char modeText[20];

    switch (cookMode)
    {
    case 1:
        strcpy(modeText, "% mode");
        break;
    case 2:
        strcpy(modeText, "PID mode");
        break;
    case 3:
        strcpy(modeText, "Reflow cycle");
        break;
    default:
        strcpy(modeText, "FAULT");
        break;
    }

    sendMessageString("/info/mode", modeText);
}

void intelligentCook()
{

    millis_now = millis();

    if (millis_now - millis_before_2 > temp_refresh_rate)
    {

        millis_before_2 = millis_now;
        Input = displayTemperature();
        Setpoint = temperatureSetPoint;
        myPID.Compute();
        dutyCycle = Output;
        sendMessage("/info/pidoutput", Output);

        sendMessage("/info/setpoint", temperatureSetPoint);
    }
}

void regularCook()
{

    int mapValue = map(percent, 0, 100, 0, 255);
    dutyCycle = mapValue;

    sendMessage("/info/cooking_percent", percent);

    displayTemperature();
}

void smdCook()
{
    seconds = 0;
    heatStage = 1;

    while (button2 != 0)
    {
        displayTemperature();
        yield();

        if (millis() - millis_before_2 > temp_refresh_rate)
        {
            millis_before_2 = millis();

            sendMessage("/asdasd", n);

            Input = temperature;
        }

        // This is the first heating stage handler
        if (heatStage == 1)
        {
            Setpoint = stage_2_set_point;
            myPID.Compute();
            dutyCycle = Output;

            if (temperature >= stage_2_set_point)
            {
                heatStage++;
                stage2Timer = seconds;
            }
        }

        // This is the second heating stage handler
        if (heatStage == 2)
        {
            myPID.Compute();
            dutyCycle = Output;

            int stage2Temp = seconds - stage2Timer;
            if (stage2Temp > soakTime)
            {
                heatStage++;
            }
        }

        // This is the third heating stage handler
        if (heatStage == 3)
        {
            Setpoint = max_temp;
            myPID.Compute();
            dutyCycle = Output;

            if (temperature >= max_temp)
            {
                heatStage++;
                stage4Timer = seconds;
            }
        }

        // This is the forth heating stage handler
        if (heatStage == 4)
        {
            myPID.Compute();
            dutyCycle = Output;

            int temp = seconds - stage4Timer;
            if (temp > reflowTime)
            {
                heatStage++;
                dutyCycle = 0;
            }
        }

        // This is the fifth and last heating stage handler
        if (heatStage == 5)
        {
            // Send a message to indicate completion
            sendMessageString("/info/status", "Complete");

            // Reset timer and stage
            seconds = 0;
            heatStage = 0;
            delay(5000);
        }

        // Update timer and send temperature information via OSC
        if (millis() - millis_before > refresh_rate)
        {
            millis_before = millis();
            seconds++;

            sendMessage("/info/temperature", temperature);

            // If not in any active stage, send appropriate status message
            if (heatStage == 0)
            {
                sendMessageString("/info/status", "Not Running");
            }
            else if (heatStage > 0 && heatStage < 7)
            {
                char message[20];
                sprintf(message, "Running - Stage %d", heatStage);
                sendMessageString("/info/status", message);
            }

            // Call displayTemperature function to send temperature information via OSC
            displayTemperature();
        }
    }
}

void setPid(int pid, float value)
{

    switch (pid)
    {

    case 1:
        myPID.SetTunings(value, myPID.GetKi(), myPID.GetKd());
        break;

    case 2:

        myPID.SetTunings(myPID.GetKp(), value, myPID.GetKd());

        break;

    case 3:

        myPID.SetTunings(myPID.GetKp(), myPID.GetKi(), value);

        break;
    }
    sendMessageFloat("/info/pid/p", myPID.GetKp());
    sendMessageFloat("/info/pid/i", myPID.GetKi());
    sendMessageFloat("/info/pid/d", myPID.GetKd());
}

void setup()
{
    Serial.begin(115200);

    Serial.print("asdasda");

    if (!ads.begin())
    {
        Serial.println("Failed to initialize ADS.");
        while (1)
            ;
        ads.setGain(GAIN_ONE);
        // ads.setDataRate()
    }

    wifiSetup();
    Udp.begin(localPort);

    myPID.SetOutputLimits(0, 255);

    myPID.SetMode(AUTOMATIC);

    pinMode(SSR, OUTPUT);
    digitalWrite(SSR, HIGH);

    ledcSetup(mosfetChannel, mosfetFreq, mosfetResolution);
    ledcAttachPin(mosfetPin, mosfetChannel);

    millis_before = millis();
    millis_now = millis();
    displayTemperature();
    displayMode();
}

void handleSetMode(int newMode)
{
    cookMode = newMode;
    switch (cookMode)
    {
    case 1:
        regularCook();
        break;
    case 2:
        intelligentCook();
        break;
    // case 3:
    //     smdCook();
    //     break;
    default:
        break;
    }
    displayMode();
}

void loop()
{
    OSCMessage rcvmsg;
    int size = Udp.parsePacket();

    if (size > 0)
    {
        sendMessage("/ok", n);
        n++;

        while (size--)
            rcvmsg.fill(Udp.read());

        if (!rcvmsg.hasError())
        {

            if (rcvmsg.fullMatch("/set/mode"))
            {

                int newMode = rcvmsg.getInt(0);
                handleSetMode(newMode);
            }

            if (rcvmsg.fullMatch("/set/temp"))
            {
                temperatureSetPoint = rcvmsg.getInt(0);
            }

            if (rcvmsg.fullMatch("/set/percent"))
            {
                percent = rcvmsg.getInt(0);
            }

            if (rcvmsg.fullMatch("/button1"))
            {
                button1 = 1;
            }

            if (rcvmsg.fullMatch("/button2"))
            {
                button2 = 1;
            }

            if (rcvmsg.fullMatch("/set/pid/p"))
            {
                setPid(1, rcvmsg.getFloat(0));
            }

            if (rcvmsg.fullMatch("/set/pid/i"))
            {
                setPid(2, rcvmsg.getFloat(0));
            }

            if (rcvmsg.fullMatch("/set/pid/d"))
            {
                setPid(3, rcvmsg.getFloat(0));
            }

            sendMessage("/ok", n);
            n++;
        }

        else
        {
            Serial.print("error: ");
            sendMessageString("/ok", "not ok");
        }
    }

    slowPWM(SSR);
    normalPwm(dutyCycle);

    switch (cookMode)
    {
    case 1:
        regularCook();
        break;
    case 2:
        intelligentCook();
        break;
    // case 3:
    //     strcpy(modeText, "SMD Mode");
    //     break;
    default:
        dutyCycle = 0;
        break;
    }
}
