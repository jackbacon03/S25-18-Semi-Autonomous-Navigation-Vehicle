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
// int M3D1  = ;
// int M3IN1 = ;
// int M3D2  = ;
// int M3IN2 = ;

// BACK RIGHT
// int M4D1  = ;
// int M4IN1 = ;
// int M4D2  = ;
// int M4IN2 = ;

// ENABLE
int EN = 32;

// DIRECTION
String direction = "FORWARD"; // Change this to "FORWARD", "RIGHT", "BACKWARD", or "LEFT"

// TIMER
unsigned long startTime = 0;
const unsigned long duration = 60000; // 60 Seconds 

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

  /*
  pinMode(M3D1,  OUTPUT);
  pinMode(M3IN1, OUTPUT);
  pinMode(M3D2,  OUTPUT);
  pinMode(M3IN2, OUTPUT);

  pinMode(M4D1,  OUTPUT);
  pinMode(M4IN1, OUTPUT);
  pinMode(M4D2,  OUTPUT);
  pinMode(M4IN2, OUTPUT);
  */
  
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
  digitalWrite(M1D1,  LOW);
  digitalWrite(M1IN1, HIGH);
  digitalWrite(M1D2,  HIGH);
  digitalWrite(M1IN2, LOW);
  
  digitalWrite(M2D1,  LOW);
  digitalWrite(M2IN1, LOW);
  digitalWrite(M2D2,  HIGH);
  digitalWrite(M2IN2, HIGH);

  /*
  digitalWrite(M3D1,  );
  digitalWrite(M3IN1, );
  digitalWrite(M3D2,  );
  digitalWrite(M3IN2, );

  digitalWrite(M4D1,  );
  digitalWrite(M4IN1, );
  digitalWrite(M4D2,  );
  digitalWrite(M4IN2, );
  */

  digitalWrite(EN, HIGH);
}


// BACKWARD
void moveBackward() {
  /*
  digitalWrite(M1D1,  );
  digitalWrite(M1IN1, );
  digitalWrite(M1D2,  );
  digitalWrite(M1IN2, );
  
  digitalWrite(M2D1,  );
  digitalWrite(M2IN1, );
  digitalWrite(M2D2,  );
  digitalWrite(M2IN2, );

  digitalWrite(M3D1,  );
  digitalWrite(M3IN1, );
  digitalWrite(M3D2,  );
  digitalWrite(M3IN2, );

  digitalWrite(M4D1,  );
  digitalWrite(M4IN1, );
  digitalWrite(M4D2,  );
  digitalWrite(M4IN2, );

  digitalWrite(EN, HIGH);
  */
}

// RIGHT
void turnRight() {
  /*
  digitalWrite(M1D1,  );
  digitalWrite(M1IN1, );
  digitalWrite(M1D2,  );
  digitalWrite(M1IN2, );
  
  digitalWrite(M2D1,  );
  digitalWrite(M2IN1, );
  digitalWrite(M2D2,  );
  digitalWrite(M2IN2, );

  digitalWrite(M3D1,  );
  digitalWrite(M3IN1, );
  digitalWrite(M3D2,  );
  digitalWrite(M3IN2, );

  digitalWrite(M4D1,  );
  digitalWrite(M4IN1, );
  digitalWrite(M4D2,  );
  digitalWrite(M4IN2, );

  digitalWrite(EN, HIGH);
  */
}

// LEFT
void turnLeft() {
  /*
  digitalWrite(M1D1,  );
  digitalWrite(M1IN1, );
  digitalWrite(M1D2,  );
  digitalWrite(M1IN2, );
  
  digitalWrite(M2D1,  );
  digitalWrite(M2IN1, );
  digitalWrite(M2D2,  );
  digitalWrite(M2IN2, );

  digitalWrite(M3D1,  );
  digitalWrite(M3IN1, );
  digitalWrite(M3D2,  );
  digitalWrite(M3IN2, );

  digitalWrite(M4D1,  );
  digitalWrite(M4IN1, );
  digitalWrite(M4D2,  );
  digitalWrite(M4IN2, );

  digitalWrite(EN, HIGH);
  */
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

  digitalWrite(EN, LOW);
}

















