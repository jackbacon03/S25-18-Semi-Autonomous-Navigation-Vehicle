#include <QTRSensors.h>

QTRSensors qtr;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];


// FRONT
int M1D1  = 52;
int M1IN1 = 50;
int M1D2  = 10;
int M1IN2 = 51;

// RIGHT
int M2D1  = 33;
int M2IN1 = 30;
int M2D2  = 8;
int M2IN2 = 31;

// BACK
int M3D1  = 40;
int M3IN1 = 41;
int M3D2  = 4;
int M3IN2 = 42;

// LEFT
int M4D1  = 22;
int M4IN1 = 23;
int M4D2  = 6;
int M4IN2 = 25;

// ENABLE
int EN1 = 32;
int EN2 = 24;

// VARIABLES
String direction = "STOP";  
int speed = 60;                          // Default speed (range: 0-255)
const unsigned long changeDelay = 1000;   // 2-second delay between changes
unsigned long lastChangeTime = 0;
bool isDelayActive = false;



void setup() {
  // put your setup code here, to run once:




  pinMode(M1D1,  OUTPUT);
  pinMode(M1IN1, OUTPUT);
  pinMode(M1D2,  OUTPUT);
  pinMode(M1IN2, OUTPUT);

  pinMode(M2D1,  OUTPUT);
  pinMode(M2IN1, OUTPUT);
  pinMode(M2D2,  OUTPUT);
  pinMode(M2IN2, OUTPUT);

  pinMode(M3D1,  OUTPUT);
  pinMode(M3IN1, OUTPUT);
  pinMode(M3D2,  OUTPUT);
  pinMode(M3IN2, OUTPUT);

  pinMode(M4D1,  OUTPUT);
  pinMode(M4IN1, OUTPUT);
  pinMode(M4D2,  OUTPUT);
  pinMode(M4IN2, OUTPUT);

  pinMode(EN1, OUTPUT);
  pinMode(EN2, OUTPUT);



// configure the sensors
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){34, 35, 36, 37, 44, 45, 46, 47}, SensorCount);
  //qtr.setEmitterPin(2);

  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // turn on Arduino's LED to indicate we are in calibration mode

  // 2.5 ms RC read timeout (default) * 10 reads per calibrate() call
  // = ~25 ms per calibrate() call.
  // Call calibrate() 400 times to make calibration take about 10 seconds.
  
  for (uint16_t i = 0; i < 10; i++)
  {
    
    selfCalibrate();

  }

  stopMotor();

  digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration

  // print the calibration minimum values measured when emitters were on
  Serial.begin(9600);
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  delay(1000);


}

void loop() {
  // put your main code here, to run repeatedly:

  // read calibrated sensor values and obtain a measure of the line position
  // from 0 to 5000 (for a white line, use readLineWhite() instead)
  uint16_t position = qtr.readLineBlack(sensorValues);

  // print the sensor values as numbers from 0 to 1000, where 0 means maximum
  // reflectance and 1000 means minimum reflectance, followed by the line
  // position
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.println(position);

  delay(250);



}


// FORWARD
void moveForward() {
  digitalWrite(M1D1,  LOW);
  digitalWrite(M1IN1, LOW);
  digitalWrite(M1D2,  LOW);
  digitalWrite(M1IN2, LOW);

  digitalWrite(M2D1,  LOW);
  digitalWrite(M2IN1, LOW);
  analogWrite(M2D2, speed);
  digitalWrite(M2IN2, HIGH);

  digitalWrite(M3D1,  LOW);
  digitalWrite(M3IN1, LOW);
  digitalWrite(M3D2,  LOW);
  digitalWrite(M3IN2, LOW);

  digitalWrite(M4D1,  LOW);
  digitalWrite(M4IN1, LOW);
  analogWrite(M4D2, speed);
  digitalWrite(M4IN2, HIGH);

  digitalWrite(EN1, HIGH);
  digitalWrite(EN2, HIGH);
}

// BACKWARD
void moveBackward() {
  digitalWrite(M1D1,  LOW);
  digitalWrite(M1IN1, LOW);
  digitalWrite(M1D2,  LOW);
  digitalWrite(M1IN2, LOW);

  digitalWrite(M2D1,  LOW);
  digitalWrite(M2IN1, HIGH);
  analogWrite(M2D2, speed);
  digitalWrite(M2IN2, LOW);

  digitalWrite(M3D1,  LOW);
  digitalWrite(M3IN1, LOW);
  digitalWrite(M3D2,  LOW);
  digitalWrite(M3IN2, LOW);

  digitalWrite(M4D1,  LOW);
  digitalWrite(M4IN1, HIGH);
  analogWrite(M4D2, speed);
  digitalWrite(M4IN2, LOW);

  digitalWrite(EN1, HIGH);
  digitalWrite(EN2, HIGH);
}

// RIGHT
void moveRight() {
  digitalWrite(M1D1,  LOW);
  digitalWrite(M1IN1, HIGH);
  analogWrite(M1D2, speed);
  digitalWrite(M1IN2, LOW);
  
  digitalWrite(M2D1,  LOW);
  digitalWrite(M2IN1, LOW);
  digitalWrite(M2D2,  LOW);
  digitalWrite(M2IN2, LOW);

  digitalWrite(M3D1,  LOW);
  digitalWrite(M3IN1, LOW);
  analogWrite(M3D2, speed);
  digitalWrite(M3IN2, HIGH);

  digitalWrite(M4D1,  LOW);
  digitalWrite(M4IN1, LOW);
  digitalWrite(M4D2,  LOW);
  digitalWrite(M4IN2, LOW);

  digitalWrite(EN1, HIGH);
  digitalWrite(EN2, HIGH);
}

// LEFT
void moveLeft() {
  digitalWrite(M1D1,  LOW);
  digitalWrite(M1IN1, LOW);
  analogWrite(M1D2,   speed);
  digitalWrite(M1IN2, HIGH);
  
  digitalWrite(M2D1,  LOW);
  digitalWrite(M2IN1, LOW);
  digitalWrite(M2D2,  LOW);
  digitalWrite(M2IN2, LOW);

  digitalWrite(M3D1,  LOW);
  digitalWrite(M3IN1, HIGH);
  analogWrite(M3D2,   speed);
  digitalWrite(M3IN2, LOW);

  digitalWrite(M4D1,  LOW);
  digitalWrite(M4IN1, LOW);
  digitalWrite(M4D2,  LOW);
  digitalWrite(M4IN2, LOW);

  digitalWrite(EN1, HIGH);
  digitalWrite(EN2, HIGH);
}

// STOP
void stopMotor() {
  digitalWrite(M1D1,  LOW);
  digitalWrite(M1IN1, LOW);
  digitalWrite(M1D2,  LOW);
  digitalWrite(M1IN2, LOW);
  
  digitalWrite(M2D1,  LOW);
  digitalWrite(M2IN1, LOW);
  digitalWrite(M2D2,  LOW);
  digitalWrite(M2IN2, LOW);

  digitalWrite(M3D1,  LOW);
  digitalWrite(M3IN1, LOW);
  digitalWrite(M3D2,  LOW);
  digitalWrite(M3IN2, LOW);

  digitalWrite(M4D1,  LOW);
  digitalWrite(M4IN1, LOW);
  digitalWrite(M4D2,  LOW);
  digitalWrite(M4IN2, LOW);

  digitalWrite(EN1, LOW);
  digitalWrite(EN2, LOW);
}



void selfCalibrate(){


  moveRight();


  for(int i = 0; i < 18; i++){
    qtr.calibrate();
  }


  moveLeft();

  for(int i = 0; i < 34; i++){
    qtr.calibrate();
  }
 
 
  moveRight();

  for(int i = 0; i < 17; i++){
    qtr.calibrate();
  }



}
