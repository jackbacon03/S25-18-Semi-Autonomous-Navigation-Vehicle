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

// Set Number of Sensor Inputs
#define NUM_SENSORS 8

// this is the decision point for whether a sensor 'sees' the line at all
// needs to be tuned unless confident sensor will always be on the line
#define SENSOR_CUTOFF_VALUE 10

// SPI pins for Feather <-> MCP3008
#define CS_PIN    26  //(A1)
#define CLOCK_PIN 5
#define MOSI_PIN  19
#define MISO_PIN  21


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

  Serial.println("Movement Control Feather Ready...");

  // vars used based on number of sensors
  maxPosition = (NUM_SENSORS - 1) * 1000;

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

  delay(50);
  
}

