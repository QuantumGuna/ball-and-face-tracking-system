#include <Servo.h>

Servo servoX;  // Horizontal (pan)
Servo servoY;  // Vertical (tilt)

int posX = 90;  // Default position
int posY = 90;  // Default position
String inputString = "";
boolean stringComplete = false;

void setup() {
  Serial.begin(9600);
  
  // Wait for serial connection to establish
  delay(1000);
  
  // Debug message
  Serial.println("Ball Tracker Arduino Starting Up");
  
  // Attach servos
  servoX.attach(9);  // Attach horizontal servo to pin 9
  servoY.attach(10); // Attach vertical servo to pin 10
  
  // Initialize servos to center position
  servoX.write(posX);
  servoY.write(posY);
  
  // Confirm servos initialized
  Serial.println("Servos initialized to center position");
  
  inputString.reserve(20);
}

void loop() {
  // Process serial commands when complete
  if (stringComplete) {
    // Parse X,Y values from string
    int commaIndex = inputString.indexOf(',');
    if (commaIndex > 0) {
      int newX = inputString.substring(0, commaIndex).toInt();
      int newY = inputString.substring(commaIndex + 1).toInt();
      
      // Validate ranges
      if (newX >= 0 && newX <= 180 && newY >= 0 && newY <= 180) {
        // Debug output
        Serial.print("Moving to: ");
        Serial.print(newX);
        Serial.print(",");
        Serial.println(newY);
        
        // Apply with rate limiting to prevent jitter
        posX = newX;
        posY = newY;
        
        // Move servos
        servoX.write(posX);
        servoY.write(posY);
      }
      else {
        Serial.println("Invalid servo angles");
      }
    }
    else {
      Serial.println("Invalid command format");
    }
    
    // Clear the string for next command
    inputString = "";
    stringComplete = false;
  }
  
  // Periodically check if still alive
  static unsigned long lastPing = 0;
  if (millis() - lastPing > 2000) {
    Serial.println("Arduino alive");
    lastPing = millis();
  }
}

// Serial event handler
void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    
    // Add character to input string
    if (inChar != '\n') {
      inputString += inChar;
    }
    // Set flag when newline received
    else {
      stringComplete = true;
    }
  }
}
