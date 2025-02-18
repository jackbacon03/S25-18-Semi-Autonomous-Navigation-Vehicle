/*
  BACON / SHAIKH / MOYER
  Feb 10, 2025

  Handles user input from Arduino Cloud interface
  Uses an Adafruit Feather ESP32 V2
  Interacts with secondary Feather that controls movement control
*/

#define TX_PIN 8                // Transferring
#define RX_PIN 7                // Recieving
HardwareSerial SerialMCF(1);    // UART1 for Movement Control Feather


/*
  SETUP
  Initializes Serial
*/
void setup() {

  // USB Serial for User Input
  Serial.begin(115200);

  // UART1 for TX/RX Communication
  SerialMCF.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN); 

  // Verify Connection
  while (!SerialMCF) {  
    Serial.println("ERROR: SerialMCF failed to start. Retrying...");
    delay(1000);  // Retry every 1 second
  }  

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
    
    // Validate Command Before Sending
    if (isValidCommand(command)) {
      SerialMCF.println(command);                       // Send to Movement Feather
      Serial.print("Sent: "); Serial.println(command);  // Print to Serial Monitor
    } else {
      Serial.println("ERROR: Invalid Command. Try again.");
    }    
  }

  // Receive Acknowledgment from Movement Control Feather
  if (SerialMCF.available() > 0) {
    String response = SerialMCF.readStringUntil('\n');         // Read response
    response.trim();                                           // Remove spaces & newlines
    
    // Ensure Response is Valid
    if (response.startsWith("ACK:") || response.startsWith("ERROR:")) {
      Serial.print("MCF Response: "); Serial.println(response);   // Display acknowledgment
    } else {
      Serial.println("WARNING: Unexpected Response Received.");
    }    
  }

}


/*
  COMMAND VALIDATION: Validates user commands before sending to MCF
*/
bool isValidCommand(String cmd) {
  if (cmd == "STOP") return true;
  if (cmd.startsWith("SPEED=")) {
    int speedValue = cmd.substring(6).toInt();
    return (speedValue >= -255 && speedValue <= 255);  // Check range
  }
  if (cmd.startsWith("P=") || cmd.startsWith("D=")) {
    float gainValue = cmd.substring(2).toFloat();
    return (gainValue >= 0.0 && gainValue <= 5.0);  // Check range
  }
  return false;  // Reject unknown commands
}



