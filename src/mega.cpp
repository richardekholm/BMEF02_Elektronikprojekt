#include <Wire.h>
#include <Arduino.h>

#define ARDUINO_ADDRESS 8  // Must match ESP32 sender

// Forward declaration of the receiveData function
void receiveData(int byteCount);

void setup() {
  Wire.begin(ARDUINO_ADDRESS);  // Join I2C bus as slave
  Wire.onReceive(receiveData);   // Set function to handle received data
  Serial.begin(9600);
}

void loop() {
  // Nothing needed here, data is handled in receiveData()
}

void receiveData(int byteCount) {
  if (byteCount < 10) return;  // Ensure we have all 10 bytes
  
  int target = Wire.read();
  int score  = Wire.read();
  
  int x = (Wire.read() << 8) | Wire.read();
  int y = (Wire.read() << 8) | Wire.read();
  int w = (Wire.read() << 8) | Wire.read();
  int h = (Wire.read() << 8) | Wire.read();
  
  Serial.println("--------------------");
  Serial.print("Target: "); Serial.println(target);
  Serial.print("Score: "); Serial.println(score);
  Serial.print("X-position: "); Serial.println(x);
  Serial.print("Y-position: "); Serial.println(y);
  Serial.print("Width: "); Serial.println(w);
  Serial.print("Height: "); Serial.println(h);
}