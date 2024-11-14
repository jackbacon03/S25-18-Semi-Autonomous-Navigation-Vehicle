// Prototype Testing

// FRONT LEFT
int M1D1  = 52;
int M1IN1 = 50;
int M1D2  = 10;
int M1IN2 = 51;

// FRONT RIGHT
int M2D1  = 33;
int M2IN1 = 30;
int M2D2  = 8;
int M2IN2 = 31;

// BACK LEFT
int M3D1  = 40;
int M3IN1 = 41;
int M3D2  = 4;
int M3IN2 = 42;

// BACK RIGHT
int M4D1  = 22;
int M4IN1 = 23;
int M4D2  = 6;
int M4IN2 = 25;

// ENABLE
int EN1 = 32;
int EN2 = 24;

// DIRECTION
String direction = "LEFT"; // Change this to "FORWARD", "RIGHT", "BACKWARD", or "LEFT"

// TIMER
unsigned long startTime = 0;
const unsigned long duration = 30000; // 60 Seconds
const unsigned long startDelay = 5000; // 5-second delay before starting

// PIN MAPPING
void setup() {
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

  Serial.begin(9600);
}

void loop() {
  delay(startDelay);
  
  startTime = millis();
  
  while (millis() - startTime < duration) {
    if (direction == "FORWARD") {
      moveForward();
    } else if (direction == "BACKWARD") {
      moveBackward();
    } else if (direction == "RIGHT") {
      moveRight();
    } else if (direction == "LEFT") {
      moveLeft();
    }
  }
  
  stopMotor();
  delay(1000);
}

// FORWARD
void moveForward() {
  digitalWrite(M1D1,  LOW);
  digitalWrite(M1IN1, LOW);
  digitalWrite(M1D2,  LOW);
  digitalWrite(M1IN2, LOW);

  digitalWrite(M2D1,  LOW);
  digitalWrite(M2IN1, HIGH);
  digitalWrite(M2D2,  HIGH);
  digitalWrite(M2IN2, LOW);

  digitalWrite(M3D1,  LOW);
  digitalWrite(M3IN1, LOW);
  digitalWrite(M3D2,  LOW);
  digitalWrite(M3IN2, LOW);

  digitalWrite(M4D1,  LOW);
  digitalWrite(M4IN1, LOW);
  digitalWrite(M4D2,  HIGH);
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
  digitalWrite(M2D2,  LOW);
  digitalWrite(M2IN2, HIGH);

  digitalWrite(M3D1,  LOW);
  digitalWrite(M3IN1, LOW);
  digitalWrite(M3D2,  LOW);
  digitalWrite(M3IN2, LOW);

  digitalWrite(M4D1,  LOW);
  digitalWrite(M4IN1, HIGH);
  digitalWrite(M4D2,  HIGH);
  digitalWrite(M4IN2, LOW);

  digitalWrite(EN1, HIGH);
  digitalWrite(EN2, HIGH);
}

// RIGHT
void moveRight() {
  digitalWrite(M1D1,  LOW);
  digitalWrite(M1IN1, LOW);
  digitalWrite(M1D2,  HIGH);
  digitalWrite(M1IN2, HIGH);
  
  digitalWrite(M2D1,  LOW);
  digitalWrite(M2IN1, LOW);
  digitalWrite(M2D2,  LOW);
  digitalWrite(M2IN2, LOW);

  digitalWrite(M3D1,  LOW);
  digitalWrite(M3IN1, HIGH);
  digitalWrite(M3D2,  HIGH);
  digitalWrite(M3IN2, LOW);

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
  digitalWrite(M1IN1, HIGH);
  digitalWrite(M1D2,  HIGH);
  digitalWrite(M1IN2, LOW);
  
  digitalWrite(M2D1,  LOW);
  digitalWrite(M2IN1, LOW);
  digitalWrite(M2D2,  LOW);
  digitalWrite(M2IN2, LOW);

  digitalWrite(M3D1,  LOW);
  digitalWrite(M3IN1, LOW);
  digitalWrite(M3D2,  HIGH);
  digitalWrite(M3IN2, HIGH);

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











