// // slider----------------------------------------------
// #include "arduino_secrets.h"
// #include <WiFi.h>
// #include "thingProperties.h"
// #include <ArduinoIoTCloud.h>
// #include <Adafruit_NeoPixel.h>
// #define PIN_NEOPIXEL 0
// #define NUMPIXELS 1

// Adafruit_NeoPixel pixels(NUMPIXELS, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

// void setup() {
//   Serial.begin(115200);
//   delay(1500);

//   // initialize NeoPixel
//   pixels.begin();
//   pixels.setBrightness(255);
//   pixels.setPixelColor(0, pixels.Color(255, 0, 0)); // start with red color 
//   pixels.show();

//   // connect WiFi and Ardiuno Cloud
//   initProperties();
//   WiFi.begin(SSID, PASS);
//   while (WiFi.status() != WL_CONNECTED) {
//     delay(500);
//     Serial.print(".");
//   }
//   Serial.println("\nWiFi Connected");

//   ArduinoCloud.begin(ArduinoIoTPreferredConnection);
//   ArduinoCloud.printDebugInfo();
// }

// void loop() {
//   ArduinoCloud.update();

// }

// void onRedChange() {
//   Serial.print("Red LED Level Update: ");
//   Serial.println(red);
  
//   pixels.setPixelColor(0, pixels.Color(red, 0, 0));
//   pixels.show();
// }

// //status ----------------------------------------------------------------

// #include "arduino_secrets.h"
// #include <WiFi.h>
// #include "thingProperties.h"
// #include <ArduinoIoTCloud.h>
// #include <Adafruit_NeoPixel.h>
// #define PIN_NEOPIXEL 0
// #define NUMPIXELS 1

// Adafruit_NeoPixel pixels(NUMPIXELS, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

// void setup() {
//   Serial.begin(115200);
//   delay(1500);

//   // initialize NeoPixel
//   pixels.begin();
//   pixels.setBrightness(255);
//   //pixels.setPixelColor(0, pixels.Color(255, 0, 0)); // start with red color 
//   pixels.show();

//   // connect WiFi and Ardiuno Cloud
//   initProperties();
//   WiFi.begin(SSID, PASS);
//   while (WiFi.status() != WL_CONNECTED) {
//     delay(500);
//     Serial.print(".");
//   }
//   Serial.println("\nWiFi Connected");

//   ArduinoCloud.begin(ArduinoIoTPreferredConnection);
//   ArduinoCloud.printDebugInfo();
// }

// void loop() {
//   ArduinoCloud.update();
//   onConnectionStatusChange();
// }
// void onConnectionStatusChange()  {
//   if (ArduinoCloud.connected()) {
//     //LED GREEN

//     pixels.setPixelColor(0, pixels.Color(0, 255, 0));
//     connectionStatus = true;
    
//   }
//   else {
//     // LED RED
//     pixels.setPixelColor(0, pixels.Color(255, 0, 0));
    
//     connectionStatus = false;
//   }
//   pixels.show();
// }

// OLED with Battery and Feather-----------------------------------------------------------------------------------------
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include "arduino_secrets.h"
#include <WiFi.h>
#include "thingProperties.h"
#include <ArduinoIoTCloud.h>
#include <WiFi.h>
#include <Adafruit_NeoPixel.h>
#define PIN_NEOPIXEL 0
#define NUMPIXELS 1

//#define VBATPIN A13 //this one will change depend on what pin we use

Adafruit_SH1107 display = Adafruit_SH1107(64, 128, &Wire);
Adafruit_NeoPixel pixels(NUMPIXELS, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);
void setup() {
  Serial.begin(115200);
  delay(250); 
  Wire.begin(22, 20); 
  pixels.begin();
  pixels.setBrightness(255);
//   pixels.setPixelColor(0, pixels.Color(255, 0, 0)); // start with red color 
  pixels.show(); 
  //connect WiFi and Ardiuno Cloud
  initProperties();
  WiFi.begin(SSID, PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi Connected");

  
  Serial.println("128x64 OLED FeatherWing test");

  if (!display.begin(0x3C, true)) {  
    Serial.println("OLED fail");
    while (1);
  }

  Serial.println("OLED sucess");
  display.clearDisplay();
  display.setRotation(1);
  //connect Ardiuno Cloud
  ArduinoCloud.begin(ArduinoIoTPreferredConnection);
  ArduinoCloud.printDebugInfo();



}

void loop() 
{
  // example code of calculate the voltage 
  // float measuredvbat = analogReadMilliVolts(VBATPIN);
  // measuredvbat *= 2;    // we divided by 2, so multiply back
  // measuredvbat /= 1000; // convert to volts!
  // Serial.print("VBat: " ); Serial.println(measuredvbat); 


  //update arduino cloud dashboard
  ArduinoCloud.update();
  onConnectionStatusChange();
  // OLED shows battery
  displayBattery();
  // 10s update 
  delay(10000);
}

// OLED
void displayBattery() {
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(0, 0);
  display.println("Battery");
  display.setTextSize(3);
  display.setCursor(10, 30);
  //display.print(batteryPercentage);
  display.println("%");
  display.display();
}
void onConnectionStatusChange()  {
  if (ArduinoCloud.connected()) {
    //LED is GREEN, connect

    pixels.setPixelColor(0, pixels.Color(0, 255, 0));
    connectionStatus = true;
    
  }
  else {
    // LED is RED, disconnect
    pixels.setPixelColor(0, pixels.Color(255, 0, 0));
    
    connectionStatus = false;
  }
  pixels.show();
}
