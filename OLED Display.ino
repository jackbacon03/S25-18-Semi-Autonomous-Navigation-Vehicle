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
// #include <SPI.h>
// #include <Wire.h>
// #include <Adafruit_GFX.h>
// #include <Adafruit_SH110X.h>
// #include "arduino_secrets.h"
// #include <WiFi.h>
// #include "thingProperties.h"
// #include <ArduinoIoTCloud.h>
// #include <WiFi.h>
// #include <Adafruit_NeoPixel.h>
// //#include <ArduinoIoTCloudTCP.h> 

// #define PIN_NEOPIXEL 0
// #define NUMPIXELS 1

// //#define VBATPIN A13 //this one will change depend on what pin we use

// Adafruit_SH1107 display = Adafruit_SH1107(64, 128, &Wire);
// Adafruit_NeoPixel pixels(NUMPIXELS, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);
// void setup() {
//   Serial.begin(115200);
//   delay(250); 
//   Wire.begin(22, 20); 
//   pixels.begin();
//   pixels.setBrightness(255);
// //   pixels.setPixelColor(0, pixels.Color(255, 0, 0)); // start with red color 
//   pixels.show(); 
//   //connect WiFi and Ardiuno Cloud
//   initProperties();
//   WiFi.begin(SSID, PASS);
//   while (WiFi.status() != WL_CONNECTED) {
//     delay(500);
//     Serial.print(".");
//   }
//   Serial.println("\nWiFi Connected");

  
//   Serial.println("128x64 OLED FeatherWing test");

//   if (!display.begin(0x3C, true)) {  
//     Serial.println("OLED fail");
//     while (1);
//   }

//   Serial.println("OLED success");
//   display.clearDisplay();
//   display.setRotation(1);
//   //connect Ardiuno Cloud
//   ArduinoCloud.begin(ArduinoIoTPreferredConnection);
//   // 在 setup() 中注册连接和断开事件
  
//   ArduinoCloud.printDebugInfo();
  


// }

// void loop() 
// {
//   // example code of calculate the voltage 
//   // float measuredvbat = analogReadMilliVolts(VBATPIN);
//   // measuredvbat *= 2;    // we divided by 2, so multiply back
//   // measuredvbat /= 1000; // convert to volts!
//   // Serial.print("VBat: " ); Serial.println(measuredvbat); 


//   //update arduino cloud dashboard
//   ArduinoCloud.update();
//   //onConnectionStatusChange();
//   // OLED shows battery
//   displayBattery();
  
// }

// // OLED
// void displayBattery() {
//   display.clearDisplay();
//   display.setTextSize(2);
//   display.setTextColor(SH110X_WHITE);
//   display.setCursor(0, 0);
//   display.println("Battery");
//   display.setTextSize(3);
//   display.setCursor(10, 30);
//   //display.print(batteryPercentage);
//   display.println("%");
//   display.display();
// }
// void onConnectionStatusChange()  {
//   if (ArduinoCloud.connected()) {
//     //LED is GREEN, connect

//     pixels.setPixelColor(0, pixels.Color(0, 255, 0));
//     connectionStatus = true;
    
//   }
//   else {
//     // LED is RED, disconnect
//     pixels.setPixelColor(0, pixels.Color(255, 0, 0));
    
//     connectionStatus = false;
//   }
//   pixels.show();
  
// }
// OLED Button ABC 15 32 14
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>

Adafruit_SH1107 display = Adafruit_SH1107(64, 128, &Wire);

#define BUTTON_A 15
#define BUTTON_B 32
#define BUTTON_C 14


enum OLED {
  OLED_MAIN,
  OLED_MENU,
  OLED_EDIT, 
};
OLED currentOLED = OLED_MAIN;

int menuIndex = 0;
bool isEditing = false;

int batteryValue = 90;
int speedValue = 10;
int pGainValue = 2;
int iGainValue = 3;
int dGainValue = 4;

// 为了防止按住按钮一直重复触发，这里做个简单的定时来节流
unsigned long lastPressTime = 0;
const unsigned long debounceInterval = 200; // 毫秒

void setup() {
  Serial.begin(115200);
  Serial.println("128x64 OLED FeatherWing Menu Example");

  display.begin(0x3C, true); 
  display.clearDisplay();
  display.setRotation(1);
  display.display();

  pinMode(BUTTON_A, INPUT_PULLUP);
  pinMode(BUTTON_B, INPUT_PULLUP);
  pinMode(BUTTON_C, INPUT_PULLUP);

  drawMenu();
}

void loop() {
  // Button status
  bool aPressed = !digitalRead(BUTTON_A);
  bool bPressed = !digitalRead(BUTTON_B);
  bool cPressed = !digitalRead(BUTTON_C);

  unsigned long currentTime = millis();
  if (currentTime - lastPressTime > debounceInterval) {
    if (aPressed) {
      handleButtonA();
      lastPressTime = currentTime;
    } else if (bPressed) {
      handleButtonB();
      lastPressTime = currentTime;
    } else if (cPressed) {
      handleButtonC();
      lastPressTime = currentTime;
    }
  }
  delay(10);
}
// Button A
void handleButtonA() {
  switch (currentOLED) 
  {
    case OLED_MAIN: //press A, go to menu display
    currentOLED = OLED_MENU;
    menuIndex = 0;
    break;
    case OLED_MENU: // press A to select one value to edit
    currentOLED = OLED_EDIT;
    if (menuIndex >= 0 && menuIndex <= 3) {
        currentOLED = OLED_EDIT;
      }
    break;
    case OLED_EDIT: // End the edit, press A, come back the main display
    currentOLED = OLED_MAIN;
    break;

  }
  drawMenu();
}
// button B
void handleButtonB() {
  switch (currentOLED) 
  {
    case OLED_MAIN: //press B, nothing happen
    break;
    case OLED_MENU: // press B, cursor go up
      if (menuIndex > 0)
        menuIndex--;   
      break;
      
    case OLED_EDIT: 
    if (menuIndex == 0) //press B, increase speed value
    {
        speedValue++;
        Serial.print("Speed: ");
        Serial.println(speedValue); 
    }
    else if(menuIndex == 1){      ////press B, increase P Gain value
      pGainValue++;
    }
    else if(menuIndex == 2){      ////press B, increase I Gain value
      iGainValue++;
    }
    else if(menuIndex == 3){      ////press B, increase D Gain value
      dGainValue++;
    }
    break;

  }
  drawMenu();
}
// Button C
void handleButtonC() {
  switch (currentOLED) 
  {
    case OLED_MAIN: //press C, nothing happen
    break;
    case OLED_MENU: // press C, cursor go down
    menuIndex++;   
    break;
    case OLED_EDIT: 
    if (menuIndex == 0 && speedValue > 0) //press C, decrease speed value
    {
        speedValue--;      
        Serial.print("Speed: ");
        Serial.println(speedValue);  
    }
    else if(menuIndex == 1 && pGainValue > 0){ //press C, decrease P Gain value
        pGainValue--;       
    }
    else if(menuIndex == 2 && pGainValue > 0){ //press C, decrease I Gain value
        iGainValue--;       
    }
    else if(menuIndex == 3 && pGainValue > 0){ //press C, decrease D Gain value
        dGainValue--;       
    }
    break;

  }
  drawMenu();
}
void drawMenu()
{
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);

  switch (currentOLED) {
    case OLED_MAIN:
      mainScreen();
      break;
    case OLED_MENU:
      menuScreen();
      break;
    case OLED_EDIT:
      editScreen();
      break;
  }

  display.display();
}
// the main screen， display at first 
void mainScreen()
{
  // display Battery value
  display.setCursor(0, 0);
  display.print("Battery: ");
  display.print(batteryValue);
  display.println("%"); 
  // display Speed value
  display.setCursor(0, 12);
  display.print("Speed:   ");
  display.println(speedValue);
  // display p gain value
  display.setCursor(0, 24);
  display.print("P Gain:   ");
  display.println(pGainValue);
  // display i gain value
    display.setCursor(0, 36);
  display.print("I Gain:   ");
  display.println(iGainValue);
  // display d value
    display.setCursor(0, 48);
    display.print("D Gain:   ");
  display.println(dGainValue);
  // display Menu mode
  display.setCursor(90, 52);
  display.print("> Menu");  
}
//menu Screen: display the selections
void menuScreen()
{
  // selection 1: Speed
  display.setCursor(0, 0);
  if (menuIndex == 0) {
    display.print("> Speed");
  } else {
    display.print("  Speed");
  }

  // selection 2: P GAIN
  display.setCursor(0, 12);
  if (menuIndex == 1) {
    display.print("> P GAIN");
  } else {
    display.print("  P GAIN");
  }

    // selection 3: I GAIN
  display.setCursor(0, 24);
  if (menuIndex == 2) {
    display.print("> I GAIN");
  } else {
    display.print("  I GAIN");
  }
  
    // selection 2: D GAIN
  display.setCursor(0, 36);
  if (menuIndex == 3) {
    display.print("> D GAIN");
  } else {
    display.print("  D GAIN");
  }
  // tips
  display.setCursor(0, 48);
  display.print("Press B/C=Up/Down, A=Select");
}
// Editting screen: increase/decrease values
void editScreen()
{
  if (menuIndex == 0) {
    // edit Speed
    display.setCursor(0, 0);
    display.print("Editing Speed:");
    display.setCursor(0, 28);
    display.print(speedValue);
  } 
  else if(menuIndex == 1){
    // edit P GAIN
    display.setCursor(0, 16);
    display.print("Editing P GAIN:");
    display.setCursor(0, 28);
    display.print(pGainValue);
  }
    else if(menuIndex == 2){
    // edit P GAIN
    display.setCursor(0, 16);
    display.print("Editing I GAIN:");
    display.setCursor(0, 28);
    display.print(iGainValue);
  }
    else if(menuIndex == 3){
    // edit P GAIN
    display.setCursor(0, 16);
    display.print("Editing D GAIN:");
    display.setCursor(0, 28);
    display.print(dGainValue);
  }
  // tips
  display.setCursor(0, 48);
  display.print("B=+  C=-  A=Save & Return");
}
