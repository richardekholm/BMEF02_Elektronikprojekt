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

const int rightTrigPin = 32,  
          rightEchoPin = 33, 
          backTrigPin = 30,
          backEchoPin = 31;

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
enum RobotState {IDLE, FORWARD, LEFT, RIGHT, STOP, BACKWARD, SEARCH, TEST, TO_NET, DUMP};
RobotState robotState = IDLE;
unsigned long lastMoveTime = 0;
const unsigned long moveDuration = 500; // time per action in ms

//Function declarations
void moveForward(int duration);
void moveBackward(int duration);
void turnRight(int duration);
void turnLeft(int duration);
void stop(int duration);
void receiveData(int byteCount);
void test(int duration);
void search(int duration);
void moveToNet();
void dumpBall();



//Functions
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

float getDistance(int trigPin, int echoPin){
  distance = 0;
  int nbrOfReadings = 10;
  for (int i = 0; i < nbrOfReadings; i++) {
    digitalWrite(trigPin, LOW);  
    delayMicroseconds(2);  
    digitalWrite(trigPin, HIGH);  
    delayMicroseconds(2);  
    digitalWrite(trigPin, LOW);  
    duration = pulseIn(echoPin, HIGH);
    if(distance > 100){
      nbrOfReadings--;
      continue;
    }
    distance += (duration*.0343)/2;  
    
  }
  if (nbrOfReadings < 1){
    return 6969;
  }
  return distance/nbrOfReadings;
}

void search(int duration) {
  Serial.println("Searching...");
  // delay(stepTime);
  turnRight(stepTime);
  // stop(duration*2);
  // delay(stepTime*2);
}

void dumpBall(){
  servoA.write(110);      // Move to 0 degrees
  servoB.write(10);      // Move to 0 degrees
  Serial.println("Moving to 0 degrees");
  delay(5000);
  // servoA.write(30);     // Move to 90 degrees
  // servoB.write(90);     // Move to 90 degrees
  // Serial.println("Moving to 90 degrees");
  // delay(5000);
  servoA.write(55);    // Move to 180 degrees
  servoB.write(65);    // Move to 180 degrees
  delay(1000);
  servoA.write(0);    // Move to 180 degrees
  servoB.write(120);    // Move to 180 degrees
  Serial.println("Moving to 120 degrees");
  delay(5000);
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

  // Serial.println(w);
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


void moveToNet(){
  //Lägg till liten elevering av skopan i syfte behålla bollen i skopan
  float rightDistance = getDistance(rightTrigPin, rightEchoPin);
  float backDistance = getDistance(backTrigPin, backEchoPin);
  float shortDistance = 4.5;
  float midDistance = 8; //Beror på pinnens längd
  while(backDistance > shortDistance){
    Serial.print("rightDistance = ");
    Serial.println(rightDistance);
    Serial.print("backDistance = ");
    Serial.println(backDistance);
    
    if(backDistance > midDistance && rightDistance > midDistance){ //CASE 1: Center of arena (Find wall to begin traversing)
      Serial.println("case 1");
      moveBackward(stepTime);
    }
    else if(backDistance < midDistance && backDistance > shortDistance && rightDistance > midDistance){ //CASE 2: Back at wall, not parallell to wall (rotate until parallell)
      Serial.println("case 2");
      moveBackward(stepTime);
      while (rightDistance > midDistance){
        rightDistance = getDistance(rightTrigPin, rightEchoPin);
        moveForward(stepTime);
        turnRight(stepTime*2);
      }
    }
    else if(backDistance > midDistance && rightDistance < midDistance){ //CASE 2,5: Back at wall, parallell to wall (Move along wall)
      Serial.println("case 2.5");
      if (rightDistance < 2*shortDistance){
        turnRight(stepTime*2);
        // continue;
      }
      moveBackward(stepTime);
    }
    else if(backDistance < midDistance && backDistance > shortDistance && rightDistance < midDistance){ //CASE 3: Corner (Rotate until parallell again)
      Serial.println("case 3");
      turnRight(stepTime);
    }
    
    rightDistance = getDistance(rightTrigPin, rightEchoPin);
    backDistance = getDistance(backTrigPin, backEchoPin);
    delay(stepTime);
  }
  Serial.println("case 4");
  Serial.println("Net found.");
  robotState = DUMP;
  // robotState = SEARCH;
}

void setup() {
  Wire.begin(ARDUINO_ADDRESS);  // Join I2C bus as slave
  Wire.onReceive(receiveData);   // Set function to handle received data
  pinMode(rightTrigPin, OUTPUT);  
	pinMode(rightEchoPin, INPUT);
  pinMode(backTrigPin, OUTPUT);  
	pinMode(backEchoPin, INPUT);
  
  servoA.attach(servoPinA);  // attach to digital pin 5
  servoB.attach(servoPinB);  // attach to digital pin 6

  Serial.begin(9600);
}

void loop() {
  moveToNet();
  if ((millis() - lastMoveTime) > moveDuration && robotState != SEARCH) {
    // Serial.println("moveduration");
    // stop();
    //robotState = SEARCH;
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
    case TO_NET:
      moveToNet();
      break;
    case DUMP:
      dumpBall();
      break;
    case IDLE:
      break;

}
}


