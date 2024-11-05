// Prototype Testing

int M2D2 = 13;
int M2IN1 = 12;
int M2IN2 = 8;
int M2D1 = 4;

// int M2D3 = ;
// int M2D4 = ;
// int M2IN3 = ;
// int M2IN4 = ;

int EN = 7;

String direction = "FORWARD"; // Change this to "FORWARD", "RIGHT", "BACKWARD", or "LEFT"

// Test Timer
unsigned long startTime = 0;
const unsigned long duration = 30000; // 30 seconds 

void setup() {
  pinMode(M2D2, OUTPUT);
  pinMode(M2IN1, OUTPUT);
  pinMode(M2IN2, OUTPUT);
  pinMode(M2D1, OUTPUT);

  // pinMode(M2D3, OUTPUT);
  // pinMode(M2D4, OUTPUT);
  // pinMode(M2IN3, OUTPUT);
  // pinMode(M2IN4, OUTPUT);

  pinMode(EN, OUTPUT);

  startTime = millis();
}

void loop() {
  if (millis() - startTime < duration) {
    if (direction == "FORWARD") {
      moveForward();
    } else if (direction == "BACKWARD") {
      moveBackward();
    } else if (direction == "RIGHT") {
      moveRight();
    } else if (direction == "LEFT") {
      moveLeft();
    }
  } else {
    stopMotor();
  }
}

// FORWARD
void moveForward() {
  digitalWrite(M2D2, HIGH);
  digitalWrite(M2IN1, HIGH);
  digitalWrite(M2D1, LOW);
  digitalWrite(M2IN2, LOW);
  // digitalWrite(M2D3, );
  // digitalWrite(M2IN4, );
  // digitalWrite(M2D3, );
  // digitalWrite(M2IN4, );
  digitalWrite(EN, HIGH);
}

// BACKWARD
void moveBackward() {
  digitalWrite(M2D2, );
  digitalWrite(M2IN1, );
  digitalWrite(M2D1, );
  digitalWrite(M2IN2, );
  digitalWrite(M2D3, );
  digitalWrite(M2IN4, );
  digitalWrite(M2D3, );
  digitalWrite(M2IN4, );
  digitalWrite(EN, HIGH);
}

// RIGHT
void turnRight() {
  digitalWrite(M2D2, );
  digitalWrite(M2IN1, );
  digitalWrite(M2D1, );
  digitalWrite(M2IN2, );
  digitalWrite(M2D3, );
  digitalWrite(M2IN4, );
  digitalWrite(M2D3, );
  digitalWrite(M2IN4, );
  digitalWrite(EN, HIGH);
}

// LEFT
void turnLeft() {
  digitalWrite(M2D2, );
  digitalWrite(M2IN1, );
  digitalWrite(M2D1, );
  digitalWrite(M2IN2, );
  digitalWrite(M2D3, );
  digitalWrite(M2IN4, );
  digitalWrite(M2D3, );
  digitalWrite(M2IN4, );
  digitalWrite(EN, HIGH);
}

// STOP
void stopMotor() {
  digitalWrite(M2D2, LOW);
  digitalWrite(M2IN1, LOW);
  digitalWrite(M2D1, LOW);
  digitalWrite(M2IN2, LOW);
  digitalWrite(M2D3, LOW);
  digitalWrite(M2IN4, LOW);
  digitalWrite(M2D3, LOW);
  digitalWrite(M2IN4, LOW);
  digitalWrite(EN, LOW);
}

















