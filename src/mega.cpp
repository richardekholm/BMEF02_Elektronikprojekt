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

const int trigPin = 6,  
          echoPin = 7; 




// Timing and speed constants
const int moveTime = 1000;      
const int turnTime = 500;       
const int stepTime = 20;
const int motorSpeed = 130;
float duration, distance;

// Declare robot state and timing variables
enum RobotState {IDLE, FORWARD, LEFT, RIGHT, STOP, BACKWARD, SEARCH, TEST};
RobotState robotState = IDLE;
unsigned long lastMoveTime = 0;
const unsigned long moveDuration = 500; // time per action in ms


void moveForward(int duration);
void moveBackward(int duration);
void turnRight(int duration);
void turnLeft(int duration);
void stop();
void receiveData(int byteCount);
void test(int duration);
void spinUpMotors();

void setup() {
  Wire.begin(ARDUINO_ADDRESS);  // Join I2C bus as slave
  Wire.onReceive(receiveData);   // Set function to handle received data
  pinMode(trigPin, OUTPUT);  
	pinMode(echoPin, INPUT);  

  Serial.begin(9600);
}

void lightOn() {
    digitalWrite(ledPin, HIGH);  
}

void lightOff() {
    digitalWrite(ledPin, LOW);  
}

void turnRight(int duration) {
  Serial.println("Turning right");
    digitalWrite(BRAKE_A, LOW);    
    digitalWrite(BRAKE_B, LOW);
    digitalWrite(DIR_A, HIGH);    
    digitalWrite(DIR_B, LOW);    
    analogWrite(PWM_A, motorSpeed);
    analogWrite(PWM_B, motorSpeed);
    // spinUpMotors();
    delay(duration);
}

void moveBackward(int duration) {
  Serial.println("Moving backward");
    digitalWrite(BRAKE_A, LOW);
    digitalWrite(BRAKE_B, LOW);
    digitalWrite(DIR_A, HIGH);     
    digitalWrite(DIR_B, HIGH);     
    analogWrite(PWM_A, motorSpeed);
    analogWrite(PWM_B, motorSpeed);
    // spinUpMotors();
    delay(duration);
}

void moveForward(int duration) {
  Serial.println("Moving forward");
    digitalWrite(BRAKE_A, LOW);    
    digitalWrite(BRAKE_B, LOW);
    digitalWrite(DIR_A, LOW);    
    digitalWrite(DIR_B, LOW);     
    analogWrite(PWM_A, motorSpeed);
    analogWrite(PWM_B, motorSpeed);
    // spinUpMotors();
    delay(duration);     
} 

void turnLeft(int duration) {
  Serial.println("Turning left");
    digitalWrite(BRAKE_A, LOW);    
    digitalWrite(BRAKE_B, LOW);
    digitalWrite(DIR_A, LOW);     
    digitalWrite(DIR_B, HIGH);    
    analogWrite(PWM_A, motorSpeed);
    analogWrite(PWM_B, motorSpeed);
    // spinUpMotors();
    delay(duration);
}

void stop() {
  digitalWrite(BRAKE_A, HIGH);    
  digitalWrite(BRAKE_B, HIGH);
    analogWrite(PWM_A, 0);  // Stop motor A
    analogWrite(PWM_B, 0);  // Stop motor B
    Serial.println("Stopping motors");
}

void spinUpMotors() {
  analogWrite(PWM_A, 255);
  analogWrite(PWM_B, 255);
  delay(50);
  analogWrite(PWM_A, 200);
  analogWrite(PWM_B, 200);
  delay(50);
  analogWrite(PWM_A, 150);
  analogWrite(PWM_B, 150);
  delay(50);
  analogWrite(PWM_A, 100);
  analogWrite(PWM_B, 100);
  delay(50);
  analogWrite(PWM_A, 50);
  analogWrite(PWM_B, 50);
}

void loop() {
//  if ((millis() - lastMoveTime) > moveDuration) {
    // stop();
    // Serial.println("Searching...");
    // robotState = RIGHT;
  // return;
// }
digitalWrite(trigPin, LOW);  
delayMicroseconds(2);  
digitalWrite(trigPin, HIGH);  
delayMicroseconds(10);  
digitalWrite(trigPin, LOW);  
duration = pulseIn(echoPin, HIGH);
distance = (duration*.0343)/2;  
Serial.print("Distance: ");  
Serial.println(distance);

robotState = IDLE;  // Reset state to IDLE after each loop iteration
  switch (robotState) {
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
    robotState = LEFT;
  } else if (midX >= 170) {
    robotState = RIGHT;
  } else if(midX > 60 && midX < 170) {
    robotState = FORWARD;
  }
  lastMoveTime = millis();  // Update last move time
}
