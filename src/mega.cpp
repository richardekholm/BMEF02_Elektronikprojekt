#include <Wire.h>
#include <Arduino.h>

#define ARDUINO_ADDRESS 8  // Must match ESP32 sender

// Motor control pins (using Mega PWM pins)
const int PWM_A   = 3,
          DIR_A   = 12,
          BRAKE_A = 9,
          SNS_A   = A0;

const int PWM_B   = 11,
          DIR_B   = 13,
          BRAKE_B = 8,
          SNS_B   = A1;

const int ledPin = 13;

// Timing and speed constants
const int moveTime = 1000;      
const int turnTime = 500;       
const int stepTime = 20;
const int motorSpeed = 255;

// Declare robot state and timing variables
enum RobotState {IDLE, FORWARD, LEFT, RIGHT, STOP, BACKWARD, SEARCH, TEST};
RobotState robotState = IDLE;
unsigned long lastMoveTime = 0;
const unsigned long moveDuration = 300; // time per action in ms

void moveForward(int duration);
void moveBackward(int duration);
void turnRight(int duration);
void turnLeft(int duration);
void stop();
void receiveData(int byteCount);
void test(int duration);

void setup() {
  Wire.begin(ARDUINO_ADDRESS);  // Join I2C bus as slave
  Wire.onReceive(receiveData);   // Set function to handle received data
  Serial.begin(9600);
}

void lightOn() {
    digitalWrite(ledPin, HIGH);  
}

void lightOff() {
    digitalWrite(ledPin, LOW);  
}

void moveForward(int duration) {
    digitalWrite(BRAKE_A, LOW);    
    digitalWrite(BRAKE_B, LOW);
    digitalWrite(DIR_A, LOW);    
    digitalWrite(DIR_B, HIGH);    
    analogWrite(PWM_A, motorSpeed);
    analogWrite(PWM_B, motorSpeed);
    delay(duration);
    Serial.println("Moving forward");
}

void moveBackward(int duration) {
    digitalWrite(BRAKE_A, LOW);    
    digitalWrite(BRAKE_B, LOW);
    digitalWrite(DIR_A, HIGH);     
    digitalWrite(DIR_B, LOW);     
    analogWrite(PWM_A, motorSpeed);
    analogWrite(PWM_B, motorSpeed);
    delay(duration);
    Serial.println("Moving backward");
}

void turnRight(int duration) {
    digitalWrite(BRAKE_A, LOW);    
    digitalWrite(BRAKE_B, HIGH);
    digitalWrite(DIR_A, LOW);    
    // digitalWrite(DIR_B, LOW);     
    analogWrite(PWM_A, motorSpeed);
    analogWrite(PWM_B, motorSpeed);
    delay(duration);
    Serial.println("Turning right");
} 

void turnLeft(int duration) {
    digitalWrite(BRAKE_A, HIGH);    
    digitalWrite(BRAKE_B, LOW);
    // digitalWrite(DIR_A, HIGH);     
    digitalWrite(DIR_B, LOW);    
    analogWrite(PWM_A, motorSpeed);
    analogWrite(PWM_B, motorSpeed);
    delay(duration);
    Serial.println("Turning left");
}

void stop() {
    analogWrite(PWM_A, 0);  // Stop motor A
    analogWrite(PWM_B, 0);  // Stop motor B
    digitalWrite(BRAKE_A, HIGH);    
    digitalWrite(BRAKE_B, HIGH);
    Serial.println("Stopping motors");
}

void test(int duration){
  digitalWrite(BRAKE_A, LOW);    
    digitalWrite(BRAKE_B, LOW);
    // digitalWrite(DIR_A, LOW);     
    digitalWrite(DIR_B, HIGH);    
    // analogWrite(PWM_A, motorSpeed);
    analogWrite(PWM_B, motorSpeed);
    delay(duration);
    Serial.println("TEST");
}

void loop() {
 if (millis() - lastMoveTime > moveDuration) {
    stop();
    robotState = IDLE;
  // robotState = SEARCH;
  return;
}


robotState = TEST;
  switch (robotState) {
    case TEST:
      test(1000);
      break;
    case FORWARD:
      moveForward(100); 
      break;
    case BACKWARD:
      moveBackward(100); 
      break;
    case LEFT:
      turnLeft(100); 
      break;
    case RIGHT:
      turnRight(100); 
      break;
    case STOP:
      stop();
      break;
    case SEARCH:
      Serial.println("Searching");
      break;
    case IDLE:
      default:
      break;

}
}

void receiveData(int byteCount) {
  if (byteCount < 10) return;

  int target = Wire.read();
  int score  = Wire.read();

  int x = (Wire.read() << 8) | Wire.read();
  int y = (Wire.read() << 8) | Wire.read();
  int w = (Wire.read() << 8) | Wire.read();
  int h = (Wire.read() << 8) | Wire.read();

  int midX = x + (w / 2);

  if (midX <= 60) {
    // Serial.println("go left");
    robotState = LEFT;
  } else if (midX >= 170) {
    // Serial.println("go right");
    robotState = RIGHT;
  } else if(midX > 60 && midX < 170) {
    // Serial.println("go straight");
    robotState = FORWARD;
  } else {
    // Serial.println("searching");
    robotState = SEARCH;
  }
  lastMoveTime = millis();  // Update last move time
}
