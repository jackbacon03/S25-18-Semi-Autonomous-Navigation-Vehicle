/*
  BACON / SHAIKH / TREVINO / MOYER
  FEB - APR, 2025

  Using line position data with QTR-8A analog sensor bars
  Uses an 8 channel ADC (MCP3008) to read sensors and report over SPI
  Uses an Adafruit Feather ESP32 V2 to read the ADC over SPI
  UART connection with the User Interface Feather (UIF)
  Accepts commands from UIF
  Uses ramp up function
*/

// Library Includes
#include <MCP3008.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Toggle if Sensor Readings and/or Speed Values get printed to Serial Monitor
#define PRINT_SENSOR_DATA true
#define PRINT_SPEED_DATA false

// Set Number of Sensor Inputs
#define NUM_SENSORS 8

// This is the decision point for whether a sensor 'sees' the line at all
// needs to be tuned unless confident sensor will always be on the line
#define SENSOR_CUTOFF_VALUE 200

// SPI pins for Feather <-> MCP3008
#define CS_PIN 25       // (A2 -> A1)
#define CS_PIN_REAR 4   // (A5)
#define CLOCK_PIN 5
#define MOSI_PIN 19
#define MISO_PIN 21

// Motor Control Pins (From Prototype Robot)
// FRONT LEFT
#define M1IN1 20  // (CounterClockwise/Forward)
#define M1IN2 14  // (Clockwise/Backward)
// FRONT RIGHT
#define M2IN1 32  // (CounterClockwise/Backward)
#define M2IN2 15  // (Clockwise/Forward)
// BACK LEFT
#define M3IN1 33  // (CounterClockwise/Forward)
#define M3IN2 27  // (Clockwise/Backward)
// BACK RIGHT
#define M4IN1 12  // (CounterClockwise/Backward)
#define M4IN2 13  // (Clockwise/Forward)          // SCL PIN

// ENABLE PINS --> Now on the User Interface Feather
// #define PWR_ENABLE 4
// #define M_ENABLE 22

// Track ENABLE State
bool prevEnableState = false;
bool stopButton;
bool sensingMethod;

// CONFIG TX RX PINS
#define RX_PIN 7              // Receiving
#define TX_PIN 8              // Transferring
HardwareSerial SerialUIF(1);  // UART1 for User Interface Feather

// PID Constants for Translational PID
float Kp = 0.00;
float Ki = 0.00;
float Kd = 0.00;
float gain_translation = 1.00;

// PID Constants for Rotational PID
float Kp_rotation = 0.00;
float Ki_rotation = 0.00;
float Kd_rotation = 0.00;
float gain_rotation = 1.00;

// Base Motor Speed
int baseSpeed = 0;

// Middle of Line
int setLinePosition = 125;

// Intregral and Error Variables
float integral = 0, previousError = 0;
float integral_rotation = 0, previousError_rotation = 0;

// Speed Variables (Ramp Up Function)
int rampStepIncrement = 100;
int motorMAX = 255;  // 255 is Absolute Max

// For Setting New Speeds
int currentFL = 0;
int currentBL = 0;
int currentFR = 0;
int currentBR = 0;
int targetFL = baseSpeed;
int targetBL = baseSpeed;
int targetFR = baseSpeed;
int targetBR = baseSpeed;

// The ADC for Front and Rear Sensors using SPI
MCP3008 myADC(CLOCK_PIN, MOSI_PIN, MISO_PIN, CS_PIN);
MCP3008 myADC_rear(CLOCK_PIN, MOSI_PIN, MISO_PIN, CS_PIN_REAR);

// Vars for line position calculation
uint16_t linePosition = 0;
uint16_t linePosition_prev = 0;
uint16_t linePosition_rear = 0;
uint16_t linePosition_prev_rear = 0;
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
  SerialUIF.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
  delay(50);

  // Verify Connection
  while (!SerialUIF) {
    Serial.println("ERROR: SerialUIF failed to start. Retrying...");
    delay(1000);  // Retry every 1 second
  }

  // FRONT LEFT
  pinMode(M1IN1, OUTPUT);
  pinMode(M1IN2, OUTPUT);
  // FRONT RIGHT
  pinMode(M2IN1, OUTPUT);
  pinMode(M2IN2, OUTPUT);
  // BACK LEFT
  pinMode(M3IN1, OUTPUT);
  pinMode(M3IN2, OUTPUT);
  // BACK RIGHT
  pinMode(M4IN1, OUTPUT);
  pinMode(M4IN2, OUTPUT);
  // ENABLES
  // pinMode(PWR_ENABLE, INPUT);  // No pull-up needed (converter handles it)
  // pinMode(M_ENABLE, OUTPUT);   // Moved to User Interface Feather

  // Writing Digital ENABLE High 
  // digitalWrite(M_ENABLE, HIGH);

  // Read and Store the Initial State of ENABLE
  // bool prevEnableState = analogRead(PWR_ENABLE);

  // vars used based on number of sensors
  maxPosition = (NUM_SENSORS - 1) * 1000;
  delay(50);

  Serial.println("Movement Control Feather Ready...");

  prevEnableState = stopButton;

}


/*
  LOOP
  Reads ADC and calculates line position
*/
void loop() {

  // Checks if User Changed Speed, P-Constant, or D-Constant
  handleUserInput();
  delay(50);

  if (prevEnableState == false && stopButton == true) {  // Detect LOW → HIGH transition
    rampUpEnable();
  }


  // Read Sensor Values
  readSensors(true);      // Front Sensor
  readSensors(false);     // Back Sensor

  // Call for PID control calculations
  if (sensingMethod == true){
    PIDControl_Both_Sensors();
  }
  else if(sensingMethod == false){
    PIDControl_Only_Front();
  }

  prevEnableState = stopButton;

}


/*
  INPUT CONTROL: Adjust corresponding variables based on data from UI_Feather
*/
void handleUserInput() {

  // Reads from RX Serial Connected Monitor
  if (SerialUIF.available() > 0) {
    String input = SerialUIF.readStringUntil('\n');
    input.trim();
    // Serial.println("RAW: " + input);     // For Testing
    delay(50);

    if(input.length() == 0) {   // Double Check that Message is Not Empty
      return;

    } else if (input == "STOP") {  // If User Stops Robot
      baseSpeed = 0;
      SerialUIF.println("ACK: Robot Stopped. Set SPEED= to restart.");
      Serial.println("STOPPING ROBOT");

    } else if (input.startsWith("SPEED=")) {  // If User Changes Speed
      baseSpeed = constrain(input.substring(6).toInt(), 0, 255);
      SerialUIF.print("ACK: Speed set to ");
      SerialUIF.println(baseSpeed);
      Serial.print("NEW SPEED: "); 
      Serial.println(baseSpeed); 

    } else if (input.startsWith("P_T=")) {  // If User Changes P-Constant (Trans)
      Kp = constrain(input.substring(4).toFloat(), 0.0, 3.0);
      SerialUIF.print("ACK: Translational P-Constant set to ");
      SerialUIF.println(Kp, 3);
      Serial.print("NEW TRANSLATIONAL P: ");
      Serial.println(Kp);

    } else if (input.startsWith("I_T=")) {  // If User Changes I-Constant (Trans)
      Ki = constrain(input.substring(4).toFloat(), 0.0, 3.0);
      SerialUIF.print("ACK: Translational I-Constant set to ");
      SerialUIF.println(Ki, 3);
      Serial.print("NEW TRANSLATIONAL I: ");
      Serial.println(Ki);

    } else if (input.startsWith("D_T=")) {  // If User Changes D-Constant (Trans)
      Kd = constrain(input.substring(4).toFloat(), 0.0, 3.0);
      SerialUIF.print("ACK: Translational D-Constant set to ");
      SerialUIF.println(Kd, 3);
      Serial.print("NEW TRANSLATIONAL D: ");
      Serial.println(Kd);

    } else if (input.startsWith("P_R=")) {  // If User Changes P-Constant (Rot)
      Kp_rotation = constrain(input.substring(4).toFloat(), 0.0, 3.0);
      SerialUIF.print("ACK: Rotational P-Constant set to ");
      SerialUIF.println(Kp_rotation, 3);
      Serial.print("NEW ROTATIONAL P: ");
      Serial.println(Kp_rotation);

    } else if (input.startsWith("I_R=")) {  // If User Changes I-Constant (Rot)
      Ki_rotation = constrain(input.substring(4).toFloat(), 0.0, 3.0);
      SerialUIF.print("ACK: Rotational I-Constant set to ");
      SerialUIF.println(Ki_rotation, 3);
      Serial.print("NEW ROTATIONAL I: ");
      Serial.println(Ki_rotation);

    } else if (input.startsWith("D_R=")) {  // If User Changes D-Constant (Rot)
      Kd_rotation = constrain(input.substring(4).toFloat(), 0.0, 3.0);
      SerialUIF.print("ACK: Rotational D-Constant set to ");
      SerialUIF.println(Kd_rotation, 3);
      Serial.print("NEW ROTATIONAL D: ");
      Serial.println(Kd_rotation);

    } else if (input.startsWith("G_T=")) {  // If User Changes Gain-Constant (Trans)
      gain_translation = constrain(input.substring(4).toFloat(), 0.0, 3.0);
      SerialUIF.print("ACK: Translational Gain set to ");
      SerialUIF.println(gain_translation, 2);
      Serial.print("NEW TRANSLATIONAL GAIN: ");
      Serial.println(gain_translation);

    } else if (input.startsWith("G_R=")) {  // If User Changes Gain-Constant (Rot)
      gain_rotation = constrain(input.substring(4).toFloat(), 0.0, 3.0);
      SerialUIF.print("ACK: Rotational Gain set to ");
      SerialUIF.println(gain_rotation, 2);
      Serial.print("NEW ROTATIONAL GAIN: ");
      Serial.println(gain_rotation);

    } else if (input.startsWith("DRV_EN=")) {  // If User Uses Stop Button
      stopButton = input.substring(7).toInt();
      SerialUIF.println("ACK: Robot Stopped. Toggle Stop Button to start again. ");
      Serial.println("STOPPING ROBOT");
    }
    else if (input.startsWith("SM=")) {  // If User Uses Stop Button
      sensingMethod = input.substring(3).toInt();
      SerialUIF.println("ACK: Switched Control Method. Toggle Sensing Method to switch again. ");
    }
  }
}


/*
  RAMPING: Increment the Motors to the Desired Speed
*/
int rampSpeed(int currentSpeed, int targetSpeed) {

  // Calculate the Difference Between the Current and Target Speeds
  int speedDifference = abs(targetSpeed - currentSpeed);

  // If the Difference is >200, Take a Half Step
  if (speedDifference > 200) {
    currentSpeed += (targetSpeed - currentSpeed) / 2;
  }
  // If Already Very Close, set it Directly
  else {
    currentSpeed = targetSpeed;
  }

  // Ensure currentSpeed is Within Motor Limits
  return constrain(currentSpeed, -motorMAX, motorMAX);
}


/*
  PID CONTROL: Adjust motors based on line position
*/
void PIDControl_Both_Sensors() {

  // Calculating Error (Actual Line Value - Desired Line Value(127))
  int error = linePosition - setLinePosition;

  // Integral term (currently = 0 becuase Ki=0)
  integral += error;
  integral = 0;

  // Calculating Derivative by difference in Error Values
  float derivative = error - previousError;

  // Correction Value (P*Current Error + I*Error Sum + D*Error Difference)
  float correction = (Kp * error + Ki * integral + Kd * derivative);

  // Update Previous Error Value for Next Iteration
  previousError = error;

  // --- ROTATIONAL ERROR ----
  int error_rotation = linePosition - linePosition_rear;

  // Integral term (currently = 0 becuase Ki=0)
  integral_rotation += error_rotation;
  integral_rotation = 0;

  // Calculating Derivative by difference in Error Values
  float derivative_rotation = error_rotation - previousError_rotation;

  // Correction Value (P*Current Error + I*Error Sum + D*Error Difference)
  float correction_rotation = (Kp_rotation * error_rotation + Ki_rotation * integral_rotation + Kd_rotation * derivative_rotation);

  // Update Previous Error Value for Next Iteration
  previousError_rotation = error_rotation;

  // Correction with the Gain Values
  correction = gain_translation * correction;
  correction_rotation = gain_rotation * correction_rotation;

  // Compute target speeds
  targetFL = baseSpeed - correction - correction_rotation;
  targetBL = baseSpeed + correction - correction_rotation;
  targetFR = (-baseSpeed) - correction - correction_rotation;
  targetBR = (-baseSpeed) + correction - correction_rotation;

  // Apply speed ramping (gradual acceleration/deceleration)
  currentFL = rampSpeed(currentFL, targetFL);
  currentBL = rampSpeed(currentBL, targetBL);
  currentFR = rampSpeed(currentFR, targetFR);
  currentBR = rampSpeed(currentBR, targetBR);

  // Output Serial lines to read Speed Changes
  if (PRINT_SPEED_DATA) {
    Serial.print("FL Speed: ");
    Serial.print(currentFL);
    Serial.print("  ||  BL Speed: ");
    Serial.print(currentBL);
    Serial.print("  ||  FR Speed: ");
    Serial.print(currentFR);
    Serial.print("  ||  BR Speed: ");
    Serial.println(currentBR);
  }

  // Apply PWM signals to motors
  // FRONT LEFT
  if (currentFL >= 0) {
    analogWrite(M1IN1, currentFL);
    analogWrite(M1IN2, 0);
  } else if (currentFL < 0) {
    analogWrite(M1IN1, 0);
    analogWrite(M1IN2, -(currentFL));
  }
  // FRONT RIGHT
  if (currentFR >= 0) {
    analogWrite(M2IN1, currentFR);
    analogWrite(M2IN2, 0);
  } else if (currentFR < 0) {
    analogWrite(M2IN1, 0);
    analogWrite(M2IN2, -(currentFR));
  }
  // BACK LEFT
  if (currentBL >= 0) {
    analogWrite(M3IN1, currentBL);
    analogWrite(M3IN2, 0);
  } else if (currentBL < 0) {
    analogWrite(M3IN1, 0);
    analogWrite(M3IN2, -(currentBL));
  }
  // BACK RIGHT
  if (currentBR >= 0) {
    analogWrite(M4IN1, currentBR);
    analogWrite(M4IN2, 0);
  } else if (currentBR < 0) {
    analogWrite(M4IN1, 0);
    analogWrite(M4IN2, -(currentBR));
  }
}





void PIDControl_Only_Front() {

  // Calculating Error (Actual Line Value - Desired Line Value(127))
  int error = linePosition - setLinePosition;

  // Integral term (currently = 0 becuase Ki=0)
  integral += error;
  integral = 0;

  // Calculating Derivative by difference in Error Values
  float derivative = error - previousError;

  // Correction Value (P*Current Error + I*Error Sum + D*Error Difference)
  float correction = (Kp * error + Ki * integral + Kd * derivative);

  // Update Previous Error Value for Next Iteration
  previousError = error;

  // --- ROTATIONAL ERROR ----
  int error_rotation = linePosition - setLinePosition;//linePosition_rear;

  // Integral term (currently = 0 becuase Ki=0)
  integral_rotation += error_rotation;
  integral_rotation = 0;

  // Calculating Derivative by difference in Error Values
  float derivative_rotation = error_rotation - previousError_rotation;

  //float EXP_Kp = 0.0001;
  //float EXP_Kd = 0.00001;

  //float EXP_P_error = error_rotation * error_rotation * error_rotation;
  //float EXP_D_error = error_rotation * error_rotation * (derivative_rotation);

  // Correction Value (P*Current Error + I*Error Sum + D*Error Difference)
  float correction_rotation = (Kp_rotation * error_rotation + Ki_rotation * integral_rotation + Kd_rotation * derivative_rotation);
  //correction_rotation += ((EXP_P_error * EXP_Kp) + EXP_D_error * EXP_Kd);

  // Update Previous Error Value for Next Iteration
  previousError_rotation = error_rotation;

  // Correction with the Gain Values
  correction = gain_translation * correction;
  correction_rotation = gain_rotation * correction_rotation;

  // Compute target speeds
  targetFL = baseSpeed - correction_rotation;
  targetBL = baseSpeed - correction_rotation;
  targetFR = (-baseSpeed) - correction_rotation;
  targetBR = (-baseSpeed) - correction_rotation;

  // Apply speed ramping (gradual acceleration/deceleration)
  currentFL = rampSpeed(currentFL, targetFL);
  currentBL = rampSpeed(currentBL, targetBL);
  currentFR = rampSpeed(currentFR, targetFR);
  currentBR = rampSpeed(currentBR, targetBR);

  // Output Serial lines to read Speed Changes
  if (PRINT_SPEED_DATA) {
    Serial.print("FL Speed: ");
    Serial.print(currentFL);
    Serial.print("  ||  BL Speed: ");
    Serial.print(currentBL);
    Serial.print("  ||  FR Speed: ");
    Serial.print(currentFR);
    Serial.print("  ||  BR Speed: ");
    Serial.println(currentBR);
  }

  // Apply PWM signals to motors
  // FRONT LEFT
  if (currentFL >= 0) {
    analogWrite(M1IN1, currentFL);
    analogWrite(M1IN2, 0);
  } else if (currentFL < 0) {
    analogWrite(M1IN1, 0);
    analogWrite(M1IN2, -(currentFL));
  }
  // FRONT RIGHT
  if (currentFR >= 0) {
    analogWrite(M2IN1, currentFR);
    analogWrite(M2IN2, 0);
  } else if (currentFR < 0) {
    analogWrite(M2IN1, 0);
    analogWrite(M2IN2, -(currentFR));
  }
  // BACK LEFT
  if (currentBL >= 0) {
    analogWrite(M3IN1, currentBL);
    analogWrite(M3IN2, 0);
  } else if (currentBL < 0) {
    analogWrite(M3IN1, 0);
    analogWrite(M3IN2, -(currentBL));
  }
  // BACK RIGHT
  if (currentBR >= 0) {
    analogWrite(M4IN1, currentBR);
    analogWrite(M4IN2, 0);
  } else if (currentBR < 0) {
    analogWrite(M4IN1, 0);
    analogWrite(M4IN2, -(currentBR));
  }
}



/*
  SENSOR READING: Get the raw sensor values from reading the environment
*/
void readSensors(bool IS_FRONT) {    
  
  uint16_t sensorValues[NUM_SENSORS];
  uint32_t avg = 0;
  uint32_t sum = 0;

  // Check if any sensors see the line
  bool seeLine[NUM_SENSORS];
  for (uint8_t x = 0; x < NUM_SENSORS; x++) seeLine[x] = false;

  // Get data and compute line position
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    if (IS_FRONT) sensorValues[i] = myADC.readADC(i);
    else sensorValues[i] = myADC_rear.readADC(i);

    // Determine if this sensor is on the line at all
    if (sensorValues[i] > SENSOR_CUTOFF_VALUE) seeLine[i] = true;

    avg += (uint32_t)sensorValues[i] * (i * 1000);
    sum += sensorValues[i];
  }

  // CAREFUL: Might cause issue if divides by 0
  if (IS_FRONT) linePosition = maxPosition - (avg / sum);
  else linePosition_rear = maxPosition - (avg / sum);

  // Determine if on line, excluding the outermost sensors (data is unreliable at extremes)
  bool onLine = false;
  for (int j = 1; j < NUM_SENSORS - 1; j++) {
    if (seeLine[j]) onLine = true;
  }

  if (IS_FRONT) {
    // Save position, or if off Line then impose min/max limits
    if (onLine) linePosition_prev = linePosition;
    else linePosition = linePosition_prev < (maxPosition / 2) ? 0 : maxPosition;

    // Re-map the line position range for an 8-bit value, optional
    linePosition = map(linePosition, 0, maxPosition, 0, 255);
  } else {
    // Save position, or if off Line then impose min/max limits
    if (onLine) linePosition_prev_rear = linePosition_rear;
    else linePosition_rear = linePosition_prev_rear < (maxPosition / 2) ? 0 : maxPosition;

    // Re-map the line position range for an 8-bit value, optional
    linePosition_rear = map(linePosition_rear, 0, maxPosition, 0, 255);
  }

  // Output Serial lines to read Sensor Readings
  if (PRINT_SENSOR_DATA) {
    for (uint8_t y = 0; y < NUM_SENSORS; y++) {
      Serial.print(sensorValues[y]);
      Serial.print(" : ");
    }
    Serial.print(":::: ");
    if (IS_FRONT) Serial.println(linePosition);
    else Serial.println(linePosition_rear);
  }

}


void rampUpEnable() {

  int speedTarget = baseSpeed;

  baseSpeed = speedTarget / 5;

  if (sensingMethod == true){
    PIDControl_Both_Sensors();
  }
  else if(sensingMethod == false){
    PIDControl_Only_Front();
  }

  delay(100);

  baseSpeed = speedTarget / 2;

  if (sensingMethod == true){
    PIDControl_Both_Sensors();
  }
  else if(sensingMethod == false){
    PIDControl_Only_Front();
  }

  delay(100);

  baseSpeed = speedTarget;
  
  /*while (baseSpeed < speedTarget) {

    PIDControl();

    baseSpeed += 2;


  }*/

}




