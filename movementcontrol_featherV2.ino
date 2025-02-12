/*
  BACON / SHAIKH / MOYER
  Feb 10, 2025

  Using line position data with QTR analog sensors
  Uses an 8 channel ADC (MCP3008) to read sensors and report over SPI
  Uses an Adafruit Feather ESP32 V2 to read the ADC over SPI
  Computes line-to-sensor position
*/

// Library Includes
#include <MCP3008.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Toggle if Sensor Readings and/or Speed Values get printed to Serial Monitor
#define PRINT_SENSOR_DATA true
#define PRINT_SPEED_DATA  true

// Set Number of Sensor Inputs
#define NUM_SENSORS 5

// this is the decision point for whether a sensor 'sees' the line at all
// needs to be tuned unless confident sensor will always be on the line
#define SENSOR_CUTOFF_VALUE 100

// SPI pins for Feather <-> MCP3008
#define CS_PIN    4
#define CLOCK_PIN 5
#define MOSI_PIN  19
#define MISO_PIN  21

// Motor Control Pins (From Prototype Robot)
// FRONT
#define M1D1  52
#define M1IN1 50
#define M1D2  10
#define M1IN2 51
// RIGHT
#define M2D1  33
#define M2IN1 30
#define M2D2  8
#define M2IN2 31
// BACK
#define M3D1  40
#define M3IN1 41
#define M3D2  4
#define M3IN2 42
// LEFT
#define M4D1  22
#define M4IN1 23
#define M4D2  6
#define M4IN2 25
// ENABLE
#define EN1 32
#define EN2 24

// CONFIG TX RX PINS
#define RX_PIN 16                // Receiving
#define TX_PIN 17                // Transferring
HardwareSerial SerialRX(1);      // Use UART1 for receiving data

// PID Constants (Need to be fine tuned, Kp and Kd will be modified via UI_Feather)
float Kp = 0.80;
float Ki = 0.00;
float Kd = 0.05;

// Base Motor Speed (Will be modified via UI_Feather)
int baseSpeed = 100;

// Middle of Line
int setLinePosition = 127;

// Intregral and Error Variables
float integral = 0, previousError = 0;

// Speed Variables (For Ramp Up Implementation)
int rampStepIncrement = 5; 
int motorMIN = 0;   
int motorMAX = 127;   // 255 is Absolute Max, but keep as 127 for now
int currentFL = 0;
int currentBL = 0;
int currentFR = 0;
int currentBR = 0;

// The ADC using SPI
MCP3008 myADC(CLOCK_PIN, MOSI_PIN, MISO_PIN, CS_PIN);

// Vars for line position calculation
uint16_t linePosition = 0;
uint16_t linePosition_prev = 0;
uint16_t maxPosition = 0;
uint16_t lineCenterCutoff = 0;


/*
  SETUP
  Initializes Serial
*/
void setup() {

  // USB Serial for Debugging
  Serial.begin(115200);
  
  // UART1 for UI Feather Communication
  SerialRX.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);  

  // FRONT
  pinMode(M1D1,  OUTPUT);
  pinMode(M1IN1, OUTPUT);
  pinMode(M1D2,  OUTPUT);
  pinMode(M1IN2, OUTPUT);
  // RIGHT
  pinMode(M2D1,  OUTPUT);
  pinMode(M2IN1, OUTPUT);
  pinMode(M2D2,  OUTPUT);
  pinMode(M2IN2, OUTPUT);
  // BACK
  pinMode(M3D1,  OUTPUT);
  pinMode(M3IN1, OUTPUT);
  pinMode(M3D2,  OUTPUT);
  pinMode(M3IN2, OUTPUT);
  // LEFT
  pinMode(M4D1,  OUTPUT);
  pinMode(M4IN1, OUTPUT);
  pinMode(M4D2,  OUTPUT);
  pinMode(M4IN2, OUTPUT);
  // ENABLE
  pinMode(EN1, OUTPUT);
  pinMode(EN2, OUTPUT);

  Serial.println("Movement Control Feather Ready...");

  // vars used based on number of sensors
  maxPosition = (NUM_SENSORS - 1) * 1000;

  // Turn on ENABLE Pins to Allo Motor Movement
  digitalWrite(EN1, HIGH);
  digitalWrite(EN2, HIGH);
}


/*
  LOOP
  Reads ADC and calculates line position
*/
void loop() {
  
  // Checks if User Changed Speed, P-Constant, or D-Constant
  handleUserInput();

  uint16_t sensorValues[NUM_SENSORS];
  uint32_t avg = 0;
  uint32_t sum = 0;

  bool seeLine[NUM_SENSORS];
  for (uint8_t x = 0; x < NUM_SENSORS; x++) seeLine[x] = false;

  // get data and compute line position
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    sensorValues[i] = myADC.readADC(i);

    // determine if this sensor is on the line at all
    if (sensorValues[i] > SENSOR_CUTOFF_VALUE) seeLine[i] = true;

    avg += (uint32_t)sensorValues[i] * (i * 1000);
    sum += sensorValues[i];
  }
  linePosition = maxPosition - (avg / sum);

  // determine if on line, excluding the outermost sensors (data is unreliable at extremes)
  bool onLine = false;
  for (int j = 1; j < NUM_SENSORS - 1; j++) {
    if (seeLine[j]) onLine = true;
  }

  // save position, or if off Line then impose min/max limits
  if (onLine) linePosition_prev = linePosition;
  else linePosition = linePosition_prev < (maxPosition / 2) ? 0 : maxPosition;

  // re-map the line position range for an 8-bit value, optional
  linePosition = map(linePosition, 0, maxPosition, 0, 255);

  if (PRINT_SENSOR_DATA) {
    for (uint8_t y = 0; y < NUM_SENSORS; y++) {
      Serial.print(sensorValues[y]);
      Serial.print(" : ");
    }
    Serial.print(":::: ");
    Serial.println(linePosition);
  }

  // Call for PID control calculations
  PIDControl();

}


/*
  INPUT CONTROL: Adjust corresponding variables based on data from UI_Feather
*/
void handleUserInput() {

  // Reads from RX Serial Connected Monitor
  if (SerialRX.available() > 0) {
    String input = SerialRX.readStringUntil('\n');
    input.trim();

    if (input == "STOP") {                    // If User Stops Robot
      baseSpeed = 0;
      digitalWrite(EN1, LOW);
      digitalWrite(EN2, LOW);
      SerialRX.println("ACK: Robot Stopped. Set SPEED= to restart.");
    }
    else if (input.startsWith("SPEED=")) {    // If User Changes Speed
      baseSpeed = constrain(input.substring(6).toInt(), 0, 255);
      digitalWrite(EN1, HIGH);
      digitalWrite(EN2, HIGH);
      SerialRX.print("ACK: Speed set to ");
      SerialRX.println(baseSpeed);
    } else if (input.startsWith("P=")) {      // If User Changes P-Constant
      Kp = constrain(input.substring(2).toFloat(), 0.0, 5.0);
      SerialRX.print("ACK: P-Constant set to ");
      SerialRX.println(Kp, 2);
    } else if (input.startsWith("D=")) {      // If User Changes D-Constant
      Kd = constrain(input.substring(2).toFloat(), 0.0, 5.0);
      SerialRX.print("ACK: D-Constant set to ");
      SerialRX.println(Kd, 2);
    } else {                                  // If User Enters an Invalid Command
      SerialRX.println("ERROR: Invalid Command. Try Again.");
    }
  }

}


/*
  RAMPING: Increment the Motors to the Desired Speed
*/
int rampSpeed(int currentSpeed, int targetSpeed) {

  if (currentSpeed < targetSpeed) {         // Ramping Up Current Speed
    currentSpeed += rampStepIncrement;
    if (currentSpeed > targetSpeed) currentSpeed = targetSpeed;
  } 
  else if (currentSpeed > targetSpeed) {    // Ramping Down Current Speed
    currentSpeed -= rampStepIncrement;
    if (currentSpeed < targetSpeed) currentSpeed = targetSpeed;
  }

  // Ensure currentSpeed is Within Motor Limits
  return constrain(currentSpeed, motorMIN, motorMAX);

}


/*
  PID CONTROL: Adjust motors based on line position / Only Translational, Eventually Adding Rotational (+/- V_CR)
*/
void PIDControl() {
  
  // Calculating Error (Actual Line Value - Desired Line Value(127))
  int error = linePosition - setLinePosition;
  
  // Integral term (currently = 0 becuase Ki=0)
  integral += error;

  // Calculating Derivative by difference in Error Values
  float derivative = error - previousError;

  // Correction Value (P*Current Error + I*Error Sum + D*Error Difference)
  float correction = (Kp * error + Ki * integral + Kd * derivative);

  // Update Previous Error Value for Next Iteration
  previousError = error;

  // Compute target speeds
  int targetFL  = baseSpeed + correction;
  int targetBL  = baseSpeed - correction;
  int targetFR  = (-baseSpeed) + correction;
  int targetBR  = (-baseSpeed) - correction;

  // Apply speed ramping (gradual acceleration/deceleration)
  currentFL = rampSpeed(currentFL, targetFL);
  currentBL = rampSpeed(currentBL, targetBL);
  currentFR = rampSpeed(currentFR, targetFR);
  currentBR = rampSpeed(currentBR, targetBR);

  // Output Serial lines to read Speed Changes
  if (PRINT_SPEED_DATA) {
    Serial.print("FL Speed: "); Serial.println(currentFL);
    Serial.print("BL Speed: "); Serial.println(currentBL);
    Serial.print("FR Speed: "); Serial.println(currentFR);
    Serial.print("BR Speed: "); Serial.println(currentBR);
  }

  // Apply PWM signals to motors
  // FRONT LEFT
  analogWrite(M1D1,  0);
  analogWrite(M1IN1, currentFL);
  analogWrite(M1D2,  currentFL);
  analogWrite(M1IN2, 0);
  // FRONT RIGHT
  analogWrite(M2D1,  0);
  analogWrite(M2IN1, currentFR);
  analogWrite(M2D2,  currentFR);
  analogWrite(M2IN2, 0);
  // BACK LEFT
  analogWrite(M3D1,  0);
  analogWrite(M3IN1, currentBL);
  analogWrite(M3D2,  currentBL);
  analogWrite(M3IN2, 0);
  // BACK RIGHT
  analogWrite(M4D1,  0);
  analogWrite(M4IN1, currentBR);
  analogWrite(M4D2,  currentBR);
  analogWrite(M4IN2, 0);

}



