
int M2D2 = 13;
int M2IN1 = 12;
int M2IN2 = 8;
int M2D1 = 4;
int EN = 7;


void setup() {
  // put your setup code here, to run once:

  pinMode(M2D2, OUTPUT);
  pinMode(M2IN1, OUTPUT);
  pinMode(M2IN2, OUTPUT);
  pinMode(M2D1, OUTPUT);
  pinMode(EN, OUTPUT);

}

void loop() {
 
 digitalWrite(M2D2, HIGH);
 digitalWrite(M2IN1, HIGH);
 digitalWrite(M2D1, LOW);
 digitalWrite(M2IN2, LOW);
 digitalWrite(EN, HIGH);

}
