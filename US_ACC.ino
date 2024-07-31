#include <NewPing.h>

const int signalPin = 2;
const int xPin = A0; // X-axis pin
const int yPin = A1; // Y-axis pin
const int zPin = A2; // Z-axis pin

NewPing sonar(signalPin, signalPin);
int initialXValue = 0; // Initial X-axis value
int initialYValue = 0; // Initial Y-axis value
int initialZValue = 0; // Initial Z-axis value
char receivedChar =0;
void setup() {
  Serial.begin(57600);
  initialXValue = analogRead(xPin); // Store initial X-axis value
  initialYValue = analogRead(yPin); // Store initial Y-axis value
  initialZValue = analogRead(zPin); // Store initial Z-axis value
}

void loop() {
  unsigned int distance = sonar.ping_cm();
  int storedValue = 0; // Initialize the stored value as 0
   int storedValu = 0;
     //Read current accelerometer values
  int currentXValue = analogRead(xPin);
  int currentYValue = analogRead(yPin);
  int currentZValue = analogRead(zPin);
  if  (abs(currentXValue - initialXValue) > 240 || abs(currentYValue - initialYValue) > 240 || abs(currentZValue - initialZValue) > 240  ) {
    storedValue = 8;
    Serial.println(storedValue);
    delay(3000);
    
    }
  else if (distance < 3) {
    storedValue = 1; // Set storedValue to 1 if distance is less than 2 cm
    delay(1000);
    Serial.println(storedValue);
  
  } else if (distance >= 3 && distance < 6) {
   storedValue = 2; // Set storedValue to 2 if distance is between 2 and 3 cm
   delay(1000);
   Serial.println(storedValue);
  } else if (distance >= 6) {
    storedValue = 3; // Set storedValue to 3 if distance is greater than or equal to 3 cm
    delay(1000);
    Serial.println(storedValue);
    }


  }
  
