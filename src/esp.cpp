#include <Wire.h>
#include <Seeed_Arduino_SSCMA.h>  // Assuming you are using Seeed's AI library

SSCMA AI;  // Initialize the AI object

#define ARDUINO_ADDRESS 8  // I2C slave address for Arduino Mega

void setup() {
  AI.begin();  // Initialize the AI module
  Wire.begin(5, 6);  // ESP32 I2C: SDA = 5, SCL = 6 (adjust as needed)
  Serial.begin(9600);
}

void loop() {
  if (!AI.invoke(1, false, false)) {  // Run AI model, no filter, no image
    //Serial.print("invoke success");


    // Check if at least one object is detected
    if (AI.boxes().size() > 0) {
      // Extract the first detection data
      int target = AI.boxes()[0].target;
      int score = AI.boxes()[0].score;
      int x = AI.boxes()[0].x;
      int y = AI.boxes()[0].y;
      int w = AI.boxes()[0].w;
      int h = AI.boxes()[0].h;

      // Print data to Serial Monitor for debugging
      Serial.print("Target: "); Serial.print(target);
      Serial.print(", Score: "); Serial.print(score);
      Serial.print(", X: "); Serial.print(x);
      Serial.print(", Y: "); Serial.print(y);
      Serial.print(", W: "); Serial.print(w);
      Serial.print(", H: "); Serial.println(h);

      // Send data to Arduino Mega via I2C
      Wire.beginTransmission(ARDUINO_ADDRESS);
      Wire.write(target);  // Send object type (1 byte)
      Wire.write(score);   // Send confidence score (1 byte)
      Wire.write(x >> 8);  // Send high byte of x
      Wire.write(x & 0xFF); // Send low byte of x
      Wire.write(y >> 8);  // Send high byte of y
      Wire.write(y & 0xFF); // Send low byte of y
      Wire.write(w >> 8);  // Send high byte of w
      Wire.write(w & 0xFF); // Send low byte of w
      Wire.write(h >> 8);  // Send high byte of h
      Wire.write(h & 0xFF); // Send low byte of h
      Wire.endTransmission();
      
      Serial.println("Data sent to Arduino Mega.");
    }
  }
}