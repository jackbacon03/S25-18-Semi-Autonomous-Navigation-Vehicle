// Define TX and RX pins for UART (change if needed)
#define TXD1 8
#define RXD1 7

// Use Serial1 for UART communication
HardwareSerial mySerial(2);

void setup() {
  Serial.begin(115200);
  mySerial.begin(9600, SERIAL_8N1, RXD1, TXD1);  // UART setup
  
  Serial.println("ESP32 UART Receiver");
}

void loop() {
  // Check if data is available to read
  if (mySerial.available()) {
    // Read received string
    String message = mySerial.readStringUntil('\n');
    message.trim();  // Remove any extra spaces or newline characters

    // Convert received string to integer
    int receivedValue = message.toInt();  // Convert string to integer
    
    // Ensure received value is within the expected range (0-256)
    if (receivedValue >= 0 && receivedValue <= 256) {
      Serial.println("Received Numeric Value: " + String(receivedValue));
    } else {
      Serial.println("Invalid Input: Received '" + message + "'");
    }
  }

  
}