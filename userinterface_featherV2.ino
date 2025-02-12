/*
  BACON / SHAIKH / MOYER
  Feb 10, 2025

  Handles user input from Arduino Cloud interface
  Uses an Adafruit Feather ESP32 V2
  Interacts with secondary Feather that controls movement control
*/

#define TX_PIN 17               // Transferring
#define RX_PIN 16               // Recieving
HardwareSerial SerialMCF(1);    // UART1 for Movement Control Feather


/*
  SETUP
  Initializes Serial
*/
void setup() {

  // USB Serial for User Input
  Serial.begin(9600);

  // UART1 for TX/RX Communication
  SerialMCF.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN); 

  // Start Up Prints
  Serial.println("User Interface Feather Ready..."); 
  Serial.println("Enter Commands:");
  Serial.println("STOP: Type STOP");
  Serial.println("SPEED: Type SPEED=<value> (0-255)");
  Serial.println("P-CONTROL: Type P=<value>");
  Serial.println("D-CONTROL: Type D=<value>");
}


/*
  LOOP
  Send User Input to Movement Control Feather
*/
void loop() {
  
  // Send User Input to Movement Control Feather
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');    // Read user input
    command.trim();                                   // Remove spaces & newlines
    SerialMCF.println(command);                       // Send to Movement Feather
    Serial.print("Sent: "); Serial.println(command);  // Print to Serial Monitor
  }

  // Receive Acknowledgment from Movement Control Feather
  if (SerialMCF.available() > 0) {
    String response = SerialMCF.readStringUntil('\n');         // Read response
    response.trim();                                           // Remove spaces & newlines
    Serial.print("MCF Response: "); Serial.println(response);  // Display acknowledgment
  }

}



