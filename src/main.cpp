/*
This is a rework of the Electronoobs DIY reflow hotplate project found here. https://electronoobs.com/eng_arduino_tut155.php

I did a major rewrite of the control code and added PID control of the heater. This made a big difference to the heating and
in my opinion helps a lot.

I also used a small electric hot plate rather than a cloths iron because it was cheap and this way I can make changes to that
portion of the system if I like. https://www.amazon.ca/gp/product/B08R6F5JH8/ref=ppx_yo_dt_b_search_asin_image?ie=UTF8&psc=1

Started April 20th 2022

December 2022
Major changes..

Added additional cooking mode. This means that this is no longer just for SMD soldering.

There is a "Normal mode" that is just like a regular hot plate. You set a setting number that is anywhere between 0 - 100%
that will PWM modulate the SSR on and off at that percentage. Just like a standard hotplate. This mode does not look at
the thermocouple for feedback at all.

There is "intelligent mode" that uses the thermocouple to set the hotplate at a specific temperature and keep it there until
the hotplate is turned off.

Turning off the hotplate is done by pressing button 2 at any point.

*/
// #define ESP32

// Libraries
#include <Arduino.h>
#include "max6675.h" //Download it here: http://electronoobs.com/eng_arduino_max6675.php
#include <Wire.h>
// #include "LiquidCrystal_I2C.h" //Download it here: http://electronoobs.com/eng_arduino_liq_crystal.php
#include <PID_v1.h>
// #include <RotaryEncoder.h>
#include <Adafruit_NeoPixel.h>
// #include "BluetoothSerial.h"
#include "wifiOTAsetup.h"

// #define Serial Serial0


#include <OSCMessage.h>
// #include <OSCBundle.h>
// #include <OSCData.h>
#include <iostream>

// #include <TaskScheduler.h>

// Scheduler runner;
// // Callback methods prototypes
// void t1Callback();

// Task updatePID(1000, TASK_FOREVER, &t1Callback);



// definitions - pin definitions and constants
#ifdef ESP8266
#define neopixelPIN 4       // The output pin for the Neopixels
#define SSR 2                // the output pin that the SSR is connected to
#define thermoDO 12           // Data pin for MAX6675 (thermocouple amp)
#define thermoCS 15           // CS pin for MAX6675
#define thermoCLK 14          // Clock pin for MAX6675
#elif ESP32
#define neopixelPIN 3       // The output pin for the Neopixels
#define SSR 5                // the output pin that the SSR is connected to
#define thermoDO 39           // Data pin for MAX6675 (thermocouple amp)
#define thermoCS 37           // CS pin for MAX6675
#define thermoCLK 35          // Clock pin for MAX6675
#endif


#define NUMPIXELS 8          // This how many Neopixels are connected to PIN
#define lowTempThreshold 40  // temperature threshold to start turn the LEDs from red to green
#define highTempThreshold 65 // The temperature that the LEDs are fully red
// Variables
double Setpoint, Input, Output;              // variable needed for the PID loop
unsigned long LEDtimer;                      // used to determine when to update the LEDs
unsigned long LEDinterval = 500;             // how often in milliseconds to update the LEDs
unsigned int millis_before, millis_before_2; // used for time tracking in the loop
unsigned int millis_now = 0;                 // used to keep track of the current time of the loop
unsigned int refresh_rate = 1000;                     // how often to update the display in milliseconds
unsigned int temp_refresh_rate = 100;                 // how often to check the temperature in milliseconds
unsigned int seconds = 0;                    // used in the display to show how long the sequence has been running
bool but_1_state = false;                    // used to track of the button has been pushed. used for debouncing the button
unsigned long but_1_timer = 0;               // used for checking the time for debouncing the button push
int max_temp = 260;                          // ****** this is the high temperature set point for SMD mode. *******
float temperature = 0;                       // this is the variable that holds the current temperature reading

int n = 0;

int temperatureSetPoint = 0;
int temp = 0;
int percent = 0; // used for setting the PWM percent for cooking

int pwmCycleTime = 2; // Initial PWM cycle time in seconds (e.g., 10 seconds)
unsigned long previousMillis = 0;
int dutyCycle = 0;

bool button1 = 0;
bool button2 = 0;



int heatStage = 0;                           // used for keeping track of where we are in the heating sequence
int stage_2_set_point = 150;                 // this is the "soak" temperature set point
unsigned long stage2Timer = 0;               // used to keeping track of when we went into stage two
unsigned long stage4Timer = 0;               // used for keeping track of when we went into stage 4
int soakTime = 100;                // how long to soak the board for in seconds
int reflowTime = 60;                         // how long to reflow the board for in seconds
int cookMode = 1;                            // This is used to know what mode is selected




boolean cookStatus = 0;                      // this is used to know if we are actively cooking




String Names[] =                             // this is the text displayed in the display in the various stages for SMD cooking mode
    {
        "Off",
        "Heat",
        "Soak",
        "Blast", // I am sure there is a real name for this stage but I dont know it and this seemed cool...
        "Reflow",
};




WiFiUDP Udp;
const IPAddress outIp(192,168,0,137);        // remote IP (not needed for receive)
const unsigned int outPort = 10001;          // remote port (not needed for receive)
const unsigned int localPort = 8888;      


// instantiate objects
MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);           // Start MAX6675 SPI communication
// LiquidCrystal_I2C lcd(0x27, 20, 4);                            // Address could be 0x3f or 0x27
PID myPID(&Input, &Output, &Setpoint, 1.5, 0.005, 0.1, DIRECT);      // create the PID object
// RotaryEncoder myEnc(DT, CLK, RotaryEncoder::LatchMode::TWO03); // setup the encoder
Adafruit_NeoPixel pixels(NUMPIXELS, neopixelPIN, NEO_GRB + NEO_KHZ800);


// **************** Functions begin here   **************************

// Check the temperature of the cooktop surface via the thermocouple
// and change the LEDs color based on the temperature reading


void led(OSCMessage &msg) {
  int ledState = msg.getInt(0);
  digitalWrite(LED_BUILTIN, ledState);
  Serial.print("/led: ");
  Serial.println(ledState);
}


void sendMessageString(const char* address, const char* message) {
    OSCMessage msg(address);
    msg.add(message);

    Udp.beginPacket(outIp, outPort);
    msg.send(Udp);
    Udp.endPacket();
    msg.empty();
}

void sendMessage(const char* address, int value) {
    OSCMessage msg(address);
    msg.add(value);

    Udp.beginPacket(outIp, outPort);
    msg.send(Udp);
    Udp.endPacket();
    msg.empty();
}

void sendMessageFloat(const char* address, float value) {
    OSCMessage msg(address);
    msg.add(value);

    Udp.beginPacket(outIp, outPort);
    msg.send(Udp);
    Udp.endPacket();
    msg.empty();
}


void slowPWM(int pin) {
  // Check if the duty cycle is in the valid range (0-255)
  if (dutyCycle < 0) {
    dutyCycle = 0;
  } else if (dutyCycle > 255) {
    dutyCycle = 255;
  }

  unsigned long currentMillis = millis();
  unsigned long pwmCycleTimeU = pwmCycleTime;



  if (currentMillis - previousMillis <= (dutyCycle / 255.0) * (pwmCycleTime * 1000)) {
    // Turn on the SSR pin
    digitalWrite(pin, LOW);
    sendMessage("/pwm", 1);
  } else {
    // Turn off the SSR pin
    digitalWrite(pin, HIGH);
    sendMessage("/pwm", 0);

  }

    if (currentMillis - previousMillis >= pwmCycleTimeU * 1000) {
    // It's time to change the duty cycle and reset the timer
    previousMillis = currentMillis;
  }

}




// This function is used by the displayTemperature
// function to send the colors to the LEDs
void sendColors(int red, int green, int blue)
{
  for (int i = 0; i < NUMPIXELS; i++)
  {
    pixels.setPixelColor(i, pixels.Color(red, green, blue));
    pixels.show(); // Send the updated pixel colors to the hardware.
  }
}



int displayTemperature() {
    if (millis() - LEDtimer > LEDinterval) {
        int tempReading = thermocouple.readCelsius();
        temperature = tempReading;

        if (tempReading > highTempThreshold) {
            sendColors(150, 0, 0);
        } else if (tempReading < lowTempThreshold) {
            sendColors(0, 150, 0);
        } else if (tempReading >= lowTempThreshold && tempReading <= highTempThreshold) {
            int temperatureColor = map(tempReading, lowTempThreshold, highTempThreshold + 1, 0, 150);
            sendColors(temperatureColor, 150 - temperatureColor, 0);
        }

        // Send temperature information via OSC
        sendMessage("/info/temperature", tempReading);

        LEDtimer = millis();
        // return tempReading;
    }
        return temperature;

}

void displayMode() {
    char modeText[20]; // Create a char array to hold the mode text

    switch (cookMode) {
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

    // Send the selected mode information via OSC
    sendMessageString("/info/mode", modeText);
}

void intelligentCook() {



        millis_now = millis();

        if (millis_now - millis_before_2 > temp_refresh_rate) {
            
            millis_before_2 = millis_now;
            // temperature = thermocouple.readCelsius();
            Input = displayTemperature();
            Setpoint = temperatureSetPoint;
            myPID.Compute();
            // analogWrite(SSR, Output);
           dutyCycle = Output;
           sendMessage("/info/pidoutput", Output);
            
            // Send temperature and set point information via OSC
            // sendMessage("/info/temperature", temperature);
            sendMessage("/info/setpoint", temperatureSetPoint);
        }


}

void regularCook() {

        int mapValue = map(percent, 0, 100, 0, 255);
       dutyCycle = mapValue;

        // Send cooking percentage information via OSC
        sendMessage("/info/cooking_percent", percent);

        displayTemperature();
    }


void smdCook() {
    seconds = 0;
    heatStage = 1;

    while (button2 != 0) {
        displayTemperature();
              yield();


        // Perform temperature-related tasks
        if (millis() - millis_before_2 > temp_refresh_rate) {
            millis_before_2 = millis();
            temperature = thermocouple.readCelsius();
            sendMessage("/asdasd", n);

            Input = temperature;
        }

        // This is the first heating stage handler
        if (heatStage == 1) {
            Setpoint = stage_2_set_point;
            myPID.Compute();
           dutyCycle = Output;

            if (temperature >= stage_2_set_point) {
                heatStage++;
                stage2Timer = seconds;
            }
        }

        // This is the second heating stage handler
        if (heatStage == 2) {
            myPID.Compute();
           dutyCycle = Output;

            int stage2Temp = seconds - stage2Timer;
            if (stage2Temp > soakTime) {
                heatStage++;
            }
        }

        // This is the third heating stage handler
        if (heatStage == 3) {
            Setpoint = max_temp;
            myPID.Compute();
           dutyCycle = Output;

            if (temperature >= max_temp) {
                heatStage++;
                stage4Timer = seconds;
            }
        }

        // This is the forth heating stage handler
        if (heatStage == 4) {
            myPID.Compute();
           dutyCycle = Output;

            int temp = seconds - stage4Timer;
            if (temp > reflowTime) {
                heatStage++;
               dutyCycle = 0;
            }
        }

        // This is the fifth and last heating stage handler
        if (heatStage == 5) {
            // Send a message to indicate completion
            sendMessageString("/info/status", "Complete");

            // Reset timer and stage
            seconds = 0;
            heatStage = 0;
            delay(5000);
        }

        // Update timer and send temperature information via OSC
        if (millis() - millis_before > refresh_rate) {
            millis_before = millis();
            seconds++;

            sendMessage("/info/temperature", temperature);

            // If not in any active stage, send appropriate status message
            if (heatStage == 0) {
                sendMessageString("/info/status", "Not Running");
            } else if (heatStage > 0 && heatStage < 7) {
                char message[20];
                sprintf(message, "Running - Stage %d", heatStage);
                sendMessageString("/info/status", message);
            }

            // Call displayTemperature function to send temperature information via OSC
            displayTemperature();
        }

}
}

void setPid(int pid, float value) {

    switch (pid) {

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


// Setup function is executed only once at startup
void setup()
{
    Serial.begin(115200);
     Serial.setDebugOutput(true);

    Serial.print("asdasda");


    wifiSetup();
    Udp.begin(localPort);
    

  myPID.SetOutputLimits(0, 255);

  myPID.SetMode(AUTOMATIC); // turn the PID controller on


  pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
  pixels.clear(); // Set all pixel colors to 'off'

  pinMode(SSR, OUTPUT);   // Define the OUTPUT for the Solid State Relay
  digitalWrite(SSR, HIGH); // Start with the SSR off

  // pinMode(but_1, INPUT_PULLUP); // Setup the button input
  // pinMode(but_2, INPUT_PULLUP); // Setup the end button input

  // Serial.begin(115200);
  
  
  // otaSetup();


  // lcd.init();      // Init the LCD
  // lcd.backlight(); // Activate backlight

  millis_before = millis();
  millis_now = millis();
  displayTemperature(); // display the current temperature on the display and LEDs
  displayMode();
}


// Function to handle /set/mode [number] message
void handleSetMode(int newMode) {
    // Set the selected cooking mode to the value received via the message
    cookMode = newMode;

    // Optionally, perform any additional actions needed when setting the mode
    // For example, you can call functions based on the selected mode:
    switch (cookMode) {
        case 1:
            regularCook();
            break;
        case 2:
            intelligentCook();
            break;
        case 3:
            smdCook();
            break;
        default:
            break;
    }
      displayMode();

}


// The main loop
void loop()
{
    // Check for received OSC messages
    OSCMessage rcvmsg;
    int size = Udp.parsePacket();

   if (size > 0)  {
    sendMessage("/ok", n);
    n++;
    
    while(size--)
       rcvmsg.fill(Udp.read());


        if (!rcvmsg.hasError()) {




            // Check for specific OSC addresses and handle them
            if (rcvmsg.fullMatch("/set/mode")) {
                // Handle /set/mode [number] message (set cooking mode)
               
                    int newMode = rcvmsg.getInt(0);
                    handleSetMode(newMode);                
            }

            if (rcvmsg.fullMatch("/set/temp")) {
              temperatureSetPoint = rcvmsg.getInt(0);
              
            }

            if (rcvmsg.fullMatch("/set/percent")) {
              percent = rcvmsg.getInt(0);
              
            }

            if (rcvmsg.fullMatch("/button1")) {
              button1 = 1;
              
            }
            

            if (rcvmsg.fullMatch("/button2")) {
              button2 = 1;
              
            }

            if (rcvmsg.fullMatch("/set/pid/p")) {
                setPid(1, rcvmsg.getFloat(0));
              
            }

            if (rcvmsg.fullMatch("/set/pid/i")) {
                setPid(2, rcvmsg.getFloat(0));
              
            }

            if (rcvmsg.fullMatch("/set/pid/d")) {
                setPid(3, rcvmsg.getFloat(0));
              
            }

            sendMessage("/ok", n);
            n ++;


        } 
        
        else {
            Serial.print("error: ");
            sendMessageString("/ok", "not ok");

        }
    }

    slowPWM(SSR);

        switch (cookMode) {
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
            sendMessageString("/ok", "not ok");
            dutyCycle = 0;
            break;
    }




} 
