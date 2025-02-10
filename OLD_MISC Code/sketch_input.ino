// Prototype Testing

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
int speed = 127;                          // Default speed (range: 0-255)
const unsigned long changeDelay = 1000;   // 2-second delay between changes
unsigned long lastChangeTime = 0;
bool isDelayActive = false;

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
  Serial.println("Robot Ready. Enter Commands:");
  Serial.println("DIRECTION: FORWARD, BACKWARD, LEFT, RIGHT, STOP");
  Serial.println("SPEED: Type SPEED=<value> (0-255)");
}

void loop() {
  handleUserInput();

  if (isDelayActive) {
    stopMotor();
    if (millis() - lastChangeTime >= changeDelay) {
      isDelayActive = false; 
    }
    return;
  }

  if (direction == "FORWARD") {
    moveForward();
  } else if (direction == "BACKWARD") {
    moveBackward();
  } else if (direction == "RIGHT") {
    moveRight();
  } else if (direction == "LEFT") {
    moveLeft();
  } else if (direction == "STOP") {
    stopMotor();
  }
}

// HANDLE USER INPUT
void handleUserInput() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.startsWith("SPEED=")) {
      speed = constrain(input.substring(6).toInt(), 0, 255);
      Serial.print("Speed set to: ");
      Serial.println(speed);
    } else if (input == "FORWARD" || input == "BACKWARD" || input == "LEFT" || input == "RIGHT" || input == "STOP") {
      direction = input;
      isDelayActive = true;
      lastChangeTime = millis();
      Serial.print("Direction set to: ");
      Serial.println(direction);
    } else {
      Serial.println("Invalid Command. Try Again.");
    }
  }
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
