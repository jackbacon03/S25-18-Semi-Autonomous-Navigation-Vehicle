bool prevEnableState = LOW;  // Track previous ENABLE state
int LM2IN1 = 13; //LEFT MOTOR DRIVER
int LM2IN2 = 12; //LEFT MOTOR DRIVER
 
int LM1IN1 = 15; //LEFT MOTOR DRIVER
int LM1IN2 = 32; //LEFT MOTOR DRIVER
 
int RM2IN1 = 27; //RIGHT MOTOR DRIVER
int RM2IN2 = 33; //RIGHT MOTOR DRIVER
 
int RM1IN1 = 14; //RIGHT MOTOR DRIVER
int RM1IN2 = 20; //RIGHT MOTOR DRIVER

int PWR_ENABLE = 4;

int M_ENABLE = 22;

unsigned long lastswitch = 0;

unsigned long currentTime = millis();

 
void setup() {


  pinMode(LM2IN2, OUTPUT);

  pinMode(LM2IN1, OUTPUT);

  pinMode(LM1IN1, OUTPUT);

  pinMode(LM1IN2, OUTPUT);

  pinMode(RM2IN2, OUTPUT);

  pinMode(RM2IN1, OUTPUT);

  pinMode(RM1IN1, OUTPUT);

  pinMode(RM1IN2, OUTPUT);
 
  pinMode(PWR_ENABLE, INPUT);  // No pull-up needed (converter handles it)

  pinMode(M_ENABLE, OUTPUT);
 
  // Read initial state of ENABLE

  bool initialEnableState = analogRead(PWR_ENABLE); 

  digitalWrite(M_ENABLE, HIGH);
 
  // Run ramp-up once on startup

  rampUpMotors();
 
  // Store the initial state

  prevEnableState = initialEnableState;



}
 
void loop() {

  

  bool currentEnableState = analogRead(PWR_ENABLE);
 
  if (prevEnableState == LOW && currentEnableState == HIGH) {  // Detect LOW → HIGH transition
   
    rampUpMotors();

  }
  
  currentTime = millis();

  
  if (currentTime > lastswitch + 30000) {
    if (digitalRead(M_ENABLE) == LOW){

      digitalWrite(M_ENABLE, HIGH);
      rampUpMotors();
      lastswitch = millis();

    }
    else {
      digitalWrite(M_ENABLE, LOW);
      lastswitch = millis();
    }
    
    
  }

  prevEnableState = currentEnableState;  // Update previous state



}
 
void rampUpMotors() {

  for (int x = 0; x < 255; x++) {

    analogWrite(LM2IN1, x);

    analogWrite(RM2IN1, x);

    analogWrite(LM1IN1, x);

    analogWrite(RM1IN1, x);

    analogWrite(LM2IN2, 0);

    analogWrite(RM2IN2, 0);

    analogWrite(LM1IN2, 0);

    analogWrite(RM1IN2, 0);

    delay(2);

  }

}
 
void stopMotors() {

  analogWrite(LM2IN1, 0);

  analogWrite(RM2IN1, 0);

  analogWrite(LM1IN1, 0);

  analogWrite(RM1IN1, 0);

  analogWrite(LM2IN2, 0);

  analogWrite(RM2IN2, 0);

  analogWrite(LM1IN2, 0);

  analogWrite(RM1IN2, 0);

}
 