// Define TX and RX pins for UART (change if needed)
#define TXD1 8
#define RXD1 7

// Use Serial1 for UART communication
HardwareSerial mySerial(1);

void setup() {
  Serial.begin(115200);
  mySerial.begin(9600, SERIAL_8N1, RXD1, TXD1);  // UART setup
  
  Serial.println("ESP32 UART Transmitter");
  Serial.println("Enter a message to send:");
}

void loop() {
  // Check if user entered input in Serial Monitor
  if (Serial.available()) {
    // Read input from Serial Monitor
    String userMessage = Serial.readStringUntil('\n');  
    userMessage.trim();  // Remove any trailing newline characters

    if (userMessage.length() > 0) {
      // Send message over UART
      mySerial.println(userMessage);
      
      // Print message to Serial Monitor
      Serial.println("Sent: " + userMessage);
    }
    
    Serial.println("Enter another message:");
  }
}
