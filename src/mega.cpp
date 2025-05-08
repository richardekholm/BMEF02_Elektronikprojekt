#include <Wire.h>
#include <Arduino.h>
#include <Servo.h>

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

const int servoPinA = A2,
          servoPinB = A3;


Servo servoA;
Servo servoB;

// Timing and speed constants
const int moveTime = 100;      
const int turnTime = 100;       
const int stepTime = 100;
const int stopTime = 500; 
const int motorSpeed = 90;
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
void stop(int duration);
void receiveData(int byteCount);
void test(int duration);
void search(int duration);
//void spinUpMotors();

void setup() {
  Wire.begin(ARDUINO_ADDRESS);  // Join I2C bus as slave
  Wire.onReceive(receiveData);   // Set function to handle received data
  pinMode(trigPin, OUTPUT);  
	pinMode(echoPin, INPUT);
  
  servoA.attach(servoPinA);  // attach to digital pin 5
  servoB.attach(servoPinB);  // attach to digital pin 6

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
    delay(duration);
    digitalWrite(BRAKE_A, LOW);    
    digitalWrite(BRAKE_B, LOW);
    digitalWrite(DIR_A, HIGH);    
    digitalWrite(DIR_B, LOW);    
    analogWrite(PWM_A, motorSpeed);
    analogWrite(PWM_B, motorSpeed);
    delay(duration);
    stop(stopTime);
  }
  
  void moveBackward(int duration) {
    Serial.println("Moving backward");
    digitalWrite(BRAKE_A, LOW);
    digitalWrite(BRAKE_B, LOW);
    digitalWrite(DIR_A, HIGH);     
    digitalWrite(DIR_B, HIGH);     
    analogWrite(PWM_A, motorSpeed);
    analogWrite(PWM_B, motorSpeed);
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
    delay(duration);     
  } 
  
  void turnLeft(int duration) {
    Serial.println("Turning left");
    delay(duration);
    digitalWrite(BRAKE_A, LOW);    
    digitalWrite(BRAKE_B, LOW);
    digitalWrite(DIR_A, LOW);     
    digitalWrite(DIR_B, HIGH);    
    analogWrite(PWM_A, motorSpeed);
    analogWrite(PWM_B, motorSpeed);
    delay(duration);
    stop(stopTime);
}

void stop(int duration) {
  digitalWrite(BRAKE_A, HIGH);    
  digitalWrite(BRAKE_B, HIGH);
    analogWrite(PWM_A, 0);  // Stop motor A
    analogWrite(PWM_B, 0);  // Stop motor B
    Serial.println("Stopping motors");
    delay(duration);
}

void getdistance(){
  digitalWrite(trigPin, LOW);  
  delayMicroseconds(2);  
  digitalWrite(trigPin, HIGH);  
  delayMicroseconds(10);  
  digitalWrite(trigPin, LOW);  
  duration = pulseIn(echoPin, HIGH);
  distance = (duration*.0343)/2;  
  Serial.print("Distance: ");  
  Serial.println(distance);
}

void search(int duration) {
  Serial.println("Searching...");
  // delay(stepTime);
  turnRight(stepTime);
  // stop(duration*2);
  // delay(stepTime*2);
}

void servotest(){
  servoA.write(0);      // Move to 0 degrees
  delay(1000);
  servoA.write(90);     // Move to 90 degrees
  delay(1000);
  servoA.write(180);    // Move to 180 degrees
  delay(1000);
}

void loop() {
 if ((millis() - lastMoveTime) > moveDuration && robotState != SEARCH) {
    Serial.println("moveduration");
    // stop();
    robotState = SEARCH;
    // Serial.println("Searching...");
    // robotState = RIGHT;
  return;
}

// robotState = IDLE;  // Reset state to IDLE after each loop iteration
  switch (robotState) {
    case FORWARD:
      moveForward(moveTime); 
      break;
    case BACKWARD:
      moveBackward(moveTime); 
      break;
    case LEFT:
      turnLeft(stepTime); 
      break;
    case RIGHT:
      turnRight(stepTime); 
      break;
    case STOP:
      stop(stopTime);
      break;
    case SEARCH:
      search(stepTime);
      default:
      break;
    case IDLE:
      break;

}
}

void receiveData(int byteCount) {
  if (byteCount < 10){
    return; 
  }
    

  int target = Wire.read();
  int score  = Wire.read();

  int x = (Wire.read() << 8) | Wire.read();
  int y = (Wire.read() << 8) | Wire.read();
  int w = (Wire.read() << 8) | Wire.read();
  int h = (Wire.read() << 8) | Wire.read();

  int midX = x + (w / 2);

  Serial.println(w);
  if (midX <= 80) {
    robotState = LEFT;
  } else if (midX >= 150) {
    robotState = RIGHT;
  } else if(midX > 80 && midX < 150) {
    if (w < 100) {
      robotState = FORWARD;
    } else {
      robotState = STOP;
    }
  }
  lastMoveTime = millis();  // Update last move time
}
