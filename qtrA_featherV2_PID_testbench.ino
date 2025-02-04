/*
  MOYER / BACON/ SHAIKH
  Jan 30, 2025

  Demonstrate PID motor control for line correction
  Using line position data with 5x QTR (QRE111) analog sensors
  Uses an 8 channel ADC (MCP3008) to read sensors and report over SPI
  Uses an Adafruit Feather ESP32 V2 to read the ADC over SPI
  Computes line-to-sensor position
  Displays position value on OLED
*/

// #include <MCP3008.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_MCP3008.h>

// configurables
#define PRINT_SENSOR_DATA true
#define NUM_SENSORS 5

// this is the decision point for whether a sensor 'sees' the line at all
// needs to be tuned unless confident sensor will always be on the line
#define SENSOR_CUTOFF_VALUE 100

// SPI pins for Feather <-> MCP3008
#define CS_PIN 4
#define CLOCK_PIN 5
#define MOSI_PIN 19
#define MISO_PIN 21

// I2C pins for Feather <-> OLED
#define OLED_RESET 14


// Motor Control Pins
#define MOTOR_D1  12
#define MOTOR_IN1 33
#define MOTOR_D2  27
#define MOTOR_IN2 15
#define MOTOR_EN  32

// PID Constants
float Kp = 0.10;
float Ki = 0.00;
float Kd = 0.01;

// Base Motor Speed
int baseSpeed = 0;

// Error
float integral = 0, previousError = 0;


// The ADC using SPI
Adafruit_MCP3008 myADC;

// The 128x64 OLED using I2C
Adafruit_SSD1306 display(128, 64, &Wire, OLED_RESET);

// A task handle for displaying position on OLED, using core 0
TaskHandle_t DisplayTask;

// Data shared by both threads (cores 0 and 1)
// using volatile, it works, but I think mutex or spinlock is the better approach for ESP32
volatile uint8_t display_LinePosition = 0;
volatile uint16_t display_SensorValues[NUM_SENSORS];

// Vars for line position calculation
uint16_t linePosition = 0;
uint16_t linePosition_prev = 0;
uint16_t maxPosition = 0;
uint16_t lineCenterCutoff = 0;

/*
  SETUP
  Initializes Serial, OLED, and Task to run on Core 0
*/
void setup() {

  Serial.begin(115200);

  if (!myADC.begin(CLOCK_PIN, MOSI_PIN, MISO_PIN, CS_PIN)) {
    Serial.println("Couldn't find MCP3008");
    while (1);  // Stay here forever if initialization fails
  }

  // Configure Motor Pins as Output
  pinMode(MOTOR_D1,  OUTPUT);
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_D2,  OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  pinMode(MOTOR_EN,  OUTPUT);

  // vars used based on number of sensors
  maxPosition = (NUM_SENSORS - 1) * 1000;

  // run a task on core 0 (setup() and loop() will run on core 1 by default)
  xTaskCreatePinnedToCore(
    DisplayLinePosition,        /* Task function. */
    "Task_DisplayLinePosition", /* name of task. */
    50000,                      /* Stack size of task */
    NULL,                       /* parameter of the task */
    1,                          /* priority of the task */
    &DisplayTask,               /* Task handle to keep track of created task */
    0);                         /* pin task to core 0 */

  delay(500);
}

/*
  LOOP
  Reads ADC and calculates line position
*/
void loop() {

  uint16_t sensorValues[NUM_SENSORS];
  uint32_t avg = 0;
  uint32_t sum = 0;

  bool seeLine[NUM_SENSORS];
  for (uint8_t x = 0; x < NUM_SENSORS; x++) seeLine[x] = false;

  // get data and compute line position
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    sensorValues[i] = myADC.readADC(i);
    display_SensorValues[i] = sensorValues[i];

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

  // update OLED display
  display_LinePosition = linePosition;

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
  PID CONTROL: Adjust motors based on line position
*/
void PIDControl() {
  int error = display_LinePosition - 127;
  integral += error;
  float derivative = error - previousError;

  float correction = (Kp * error + Ki * integral + Kd * derivative);
  previousError = error;

  int leftSpeed = baseSpeed + (correction*10);
  int rightSpeed = baseSpeed - (correction*10);

  leftSpeed =  constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  Serial.print("Left Speed: "); 
  Serial.println(leftSpeed);
  Serial.print("Right Speed: "); 
  Serial.println(rightSpeed);

  analogWrite(MOTOR_D1,  0);
  analogWrite(MOTOR_IN1, 0);
  analogWrite(MOTOR_D2,  leftSpeed);
  analogWrite(MOTOR_IN2, leftSpeed);
  analogWrite(MOTOR_EN,  255);
}


/*
  DISPLAY LINE POSITION: Task to run on core 0, displaying sensor data and linePosition on OLED
*/
void DisplayLinePosition(void* pvParameters) {

  // may need to use SSD1306_EXTERNALVCC if powering with 5V source
  while (!display.begin(SSD1306_SWITCHCAPVCC, 0x3D)) delay(500);

  display.display();
  delay(100);
  display.clearDisplay();

  display.setRotation(2);
  display.setTextColor(SSD1306_WHITE);

  // lock task to run continuously on core 0
  for (;;) {
    display.clearDisplay();

    // print raw sensor data
    display.setTextSize(1);
    for (int x = NUM_SENSORS - 1; x >= 0; x--) {
      display.setCursor(x * 27, 0);
      display.print(display_SensorValues[x]);
    }

    // print calculated line position
    display.setTextSize(2);
    display.setCursor(50, 40);
    display.print(display_LinePosition);

    display.display();
    delay(50);
  }
}
