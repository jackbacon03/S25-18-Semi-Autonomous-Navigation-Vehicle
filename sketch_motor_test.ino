
int M2D2 = 8;
int M2IN1 = 30;
int M2IN2 = 31;
int M2D1 = 33;

int EN = 32;

int M1IN1 = 50;
int M1IN2 = 51;
int M1D1 = 52;
int M1D2 = 10;



void setup() {
  // put your setup code here, to run once:

  pinMode(M2D2, OUTPUT); 
  pinMode(M2IN1, OUTPUT);
  pinMode(M2IN2, OUTPUT);
  pinMode(M2D1, OUTPUT);

  pinMode(EN, OUTPUT);

  pinMode(M1D1, OUTPUT); 
  pinMode(M1IN1, OUTPUT);
  pinMode(M1IN2, OUTPUT);
  pinMode(M1D2, OUTPUT);
}

void loop() {

 digitalWrite(EN, HIGH);
 
 analogWrite(M2D2, 255);
 digitalWrite(M2IN1, HIGH);
 digitalWrite(M2D1, LOW);
 digitalWrite(M2IN2, LOW);

 analogWrite(M1D2, 255);
 digitalWrite(M1IN1, LOW);
 digitalWrite(M1D1, LOW);
 digitalWrite(M1IN2, HIGH);

 delay(2000);


 digitalWrite(EN, HIGH);
 
 analogWrite(M2D2, 200);
 digitalWrite(M2IN1, HIGH);
 digitalWrite(M2D1, LOW);
 digitalWrite(M2IN2, LOW);

 analogWrite(M1D2, 200);
 digitalWrite(M1IN1, LOW);
 digitalWrite(M1D1, LOW);
 digitalWrite(M1IN2, HIGH);

 delay(2000);


 digitalWrite(EN, HIGH);
 
 analogWrite(M2D2, 127);
 digitalWrite(M2IN1, HIGH);
 digitalWrite(M2D1, LOW);
 digitalWrite(M2IN2, LOW);

 analogWrite(M1D2, 127);
 digitalWrite(M1IN1, LOW);
 digitalWrite(M1D1, LOW);
 digitalWrite(M1IN2, HIGH);

 
 delay(2000);


 digitalWrite(EN, HIGH);
 
 analogWrite(M2D2, 60);
 digitalWrite(M2IN1, HIGH);
 digitalWrite(M2D1, LOW);
 digitalWrite(M2IN2, LOW);

 analogWrite(M1D2, 60);
 digitalWrite(M1IN1, LOW);
 digitalWrite(M1D1, LOW);
 digitalWrite(M1IN2, HIGH);

 delay(2000);

}
