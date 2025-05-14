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

const int backLeftTrigPin = 30,
          backLeftEchoPin = 31,
          backRightTrigPin = 32,
          backRightEchoPin = 33,
          bucketTrigPin = 34,
          bucketEchoPin = 35;

const int servoPinA = A2,
          servoPinB = A3;

Servo servoA;
Servo servoB;



// Timing and speed constants
const int moveTime = 100;      
const int turnTime = 100;       
const int stepTime = 100;
const int stopTime = 400; 
const int motorSpeedMove = 150;
const int motorSpeedTurn = 200;

const int wallBox = 150; // Threshold for wall detection
const int hitThreshold = 5; // Threshold for ball detection in bucket
const int bucketDistance = 12; // distance to detect ball in bucket
const int backTrigstreak = 5; // Number of triggers to detect wall
const int backShortDistance = 7; // Number of triggers to detect ball in bucket
const int backLongDistance = 10; // Number of triggers to detect ball in bucket

const int repeatThreshold = 20; //20 repeat actions trigger bailout
int repeatCount = 0;


int hits = 0;
unsigned long startTime = 0;
unsigned long lastMoveTime = 0;
const unsigned long moveDuration = 500; // time per action in ms
bool virgin = true; //fräckis


// Declare robot state and timing variables
enum RobotState {IDLE, FORWARD, LEFT, RIGHT, STOP, BACKWARD, SEARCH, TEST, TO_NET, DUMP};
RobotState robotState;
RobotState lastRobotState;

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
void setBucketHeight(String height);
bool isStuck();
float getInstantDistance(int trigPin, int echoPin);
void(* resetFunc) (void) = 0;
bool backUpStraight();
bool distanceTrig(int trigPin, int echoPin, int nbrOfHits, float threshold);
bool checkBackSensors();
void bailout();


//Functions
void lightOn() {
    digitalWrite(ledPin, HIGH);  
}

void lightOff() {
    digitalWrite(ledPin, LOW);  
}

void turnRight(int duration) {
  delay(duration);
  digitalWrite(BRAKE_A, LOW);    
  digitalWrite(BRAKE_B, LOW);
  digitalWrite(DIR_A, HIGH);    
  digitalWrite(DIR_B, LOW);    
  analogWrite(PWM_A, motorSpeedTurn*0.83);
  analogWrite(PWM_B, motorSpeedTurn);
  delay(duration);
  stop(stopTime);
}

void moveBackward(int duration) {
  digitalWrite(BRAKE_A, LOW);
  digitalWrite(BRAKE_B, LOW);
  digitalWrite(DIR_A, HIGH);     
  digitalWrite(DIR_B, HIGH);     
  analogWrite(PWM_A, motorSpeedMove*0.83);
  analogWrite(PWM_B, motorSpeedMove);
  delay(duration);
}

void moveForward(int duration) {
  digitalWrite(BRAKE_A, LOW);    
  digitalWrite(BRAKE_B, LOW);
  digitalWrite(DIR_A, LOW);    
  digitalWrite(DIR_B, LOW);     
  analogWrite(PWM_A, motorSpeedMove*0.83);
  analogWrite(PWM_B, motorSpeedMove);
  delay(duration);    
} 

void turnLeft(int duration) {
  delay(duration);
  digitalWrite(BRAKE_A, LOW);    
  digitalWrite(BRAKE_B, LOW);
  digitalWrite(DIR_A, LOW);     
  digitalWrite(DIR_B, HIGH);    
  analogWrite(PWM_A, motorSpeedTurn*0.83);
  analogWrite(PWM_B, motorSpeedTurn);
  delay(duration);
  stop(stopTime);
}

void stop(int duration) {
  digitalWrite(BRAKE_A, HIGH);    
  digitalWrite(BRAKE_B, HIGH);
  analogWrite(PWM_A, 0);  // Stop motor A
  analogWrite(PWM_B, 0);  // Stop motor B
  delay(duration);
}


float getInstantDistance(int trigPin, int echoPin) {
  // fire pulse
  digitalWrite(trigPin, LOW);  
  delayMicroseconds(2);  
  digitalWrite(trigPin, HIGH);  
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // measure time
  unsigned long duration = pulseIn(echoPin, HIGH);
  // convert to cm
  return (duration * 0.0343f) / 2.0f;
}

void search(int duration) {
turnRight(stepTime*1.2);
}

boolean backUpStraight(){
  boolean isLeftClose = distanceTrig(backLeftTrigPin, backLeftEchoPin, backTrigstreak, 30);
  boolean isRightClose = distanceTrig(backRightTrigPin, backRightEchoPin, backTrigstreak, 30);
  while(!isLeftClose && !isRightClose){//Långt ifrån väggen, back up
    moveBackward(moveTime);
    Serial.println("Not close to wall");
    isLeftClose = distanceTrig(backLeftTrigPin, backLeftEchoPin, backTrigstreak, 30);
    isRightClose = distanceTrig(backRightTrigPin, backRightEchoPin, backTrigstreak, 30);
  }
  isLeftClose = distanceTrig(backLeftTrigPin, backLeftEchoPin, backTrigstreak, 10);
  isRightClose = distanceTrig(backRightTrigPin, backRightEchoPin, backTrigstreak, 10);
  while(isLeftClose != isRightClose){
    if(isLeftClose && !isRightClose){
      Serial.println("rotating left");
      turnLeft(stepTime);
      isLeftClose = distanceTrig(backLeftTrigPin, backLeftEchoPin, backTrigstreak, 10);
      isRightClose = distanceTrig(backRightTrigPin, backRightEchoPin, backTrigstreak, 10);
    }
    else if(!isLeftClose && isRightClose){
      Serial.println("rotating right");
      turnRight(stepTime);
      isLeftClose = distanceTrig(backLeftTrigPin, backLeftEchoPin, backTrigstreak, 10);
      isRightClose = distanceTrig(backRightTrigPin, backRightEchoPin, backTrigstreak, 10);
    }
  // }
  Serial.println("backing up straight");
  moveBackward(moveTime);
  return true;//isLeftClose && isRightClose
}
}

void setBucketHeight(String height){
  if(height == "LOW"){
    servoA.write(112);    // Move to 0 degrees
    servoB.write(10);     // Move to 0 degrees
    Serial.println("Moving bucket to 0 degrees");
    delay(1000); //alltså sänka denna?
  }
  else if(height == "MID"){
    servoA.write(60);     // Move to 55 degrees
    servoB.write(70);     // Move to 55 degrees
    Serial.println("Moving bucket to 55 degrees");
    delay(1000);
  }
  else if(height == "HI"){
    servoA.write(0);      // Move to 120 degrees
    servoB.write(120);    // Move to 120 degrees
    Serial.println("Moving bucket to 90 degrees");
    delay(1000);
  }
  else if (height == "JUST ABOVE"){ // denna kanske är sketchy men tror ej de
    servoA.write(107);     // Move to 90 degrees
    servoB.write(13);     // Move to 90 degrees
    Serial.println("Moving bucket to 5 degrees");
    delay(1000);
  }
  else{
    Serial.println("Invalid height");
  }
}

boolean isStuck(){
  float dis = getInstantDistance(backLeftTrigPin, backLeftEchoPin);
  //Serial.println(dis);
  
  if (dis > 280){
     hits++;
   }

   unsigned long now = millis();
   if (now - startTime >= 500UL) {
     if (hits > 30) {
       Serial.println("TrAV is stuck");
       return true;
     }
     hits = 0;
     startTime = now;
   }
   return false;
}

void bailout(){
  moveBackward(8 * stepTime);
  turnRight(4 * turnTime);
  moveForward(4 * stepTime);
}


boolean distanceTrig(int trigPin, int echoPin, int nbrOfHits, float threshold){
  int hitStreak = 0;
  for(int i = 0; i < nbrOfHits; i++){
    float dist = getInstantDistance(trigPin, echoPin);
    if(dist < threshold && dist > 1){//Anta att värden < 0,1 är felaktiga
      hitStreak ++;
    }
  }
  Serial.println("");
  
  if (hitStreak == nbrOfHits){
    return true;
  }
  return false;
} 

void moveForwardWithChecks(int totalTime) {
  int stepTime = 100; // ms
  int elapsed = 0;
  while (elapsed < totalTime) {
    moveForward(stepTime);
    if (checkBackSensors()) {
      robotState = DUMP;
      return;
    }
    elapsed += stepTime;
  }
}

void moveBackwardWithChecks(int totalTime) {
  int stepTime = 100; // ms
  int elapsed = 0;
  while (elapsed < totalTime) {
    moveBackward(stepTime);
    if (checkBackSensors()) {
      robotState = DUMP;
      return;
    }
    elapsed += stepTime;
  }
}

void turnRightWithChecks(int totalTime) {
  int stepTime = 100; // ms
  int elapsed = 0;
  while (elapsed < totalTime) {
    turnRight(stepTime);
    if (checkBackSensors()) {
      robotState = DUMP;
      return;
    }
    elapsed += stepTime;
  }
}

void moveToNet() {
  setBucketHeight("MID");
  moveBackward(2000);
  while (true) {
    if (checkBackSensors()) {
      robotState = DUMP;
      return;
    }

    // Instead of large moves, break them up into short steps and check often
    moveForward(700);  // short movement

    if (checkBackSensors()) {return;}

    turnRight(270);    // shorter turn
    if (checkBackSensors()) {return;}

    moveBackward(4000); // short backward move
    if (checkBackSensors()) {return;}
  }
}

// Helper function for checking back sensors
bool checkBackSensors() {
  bool left = distanceTrig(backLeftTrigPin, backLeftEchoPin, 1, backShortDistance);
  bool right = distanceTrig(backRightTrigPin, backRightEchoPin, 1, backShortDistance);
  Serial.print("leftClose = "); Serial.println(left);
  Serial.print("rightClose = "); Serial.println(right);
  return left || right;
}

void receiveData(int byteCount) {
  if (byteCount < 10){  
    return;
  }
  if (robotState == TO_NET || robotState == DUMP) { // If already moving to net, ignore newmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm g data
    return;
  }
  
  int target = Wire.read();//att vi måste kasta dessa 2 värden kommer 
  int score  = Wire.read();
  
  int x = (Wire.read() << 8) | Wire.read();
  int y = (Wire.read() << 8) | Wire.read();
  int w = (Wire.read() << 8) | Wire.read();
  int h = (Wire.read() << 8) | Wire.read();
  
  if (w >= wallBox){
    moveBackward(stepTime);
    robotState = SEARCH;
    return;
  }
  Serial.print("Ball detected in arena @ X = ");
  Serial.println(x);
  Serial.print("Ball detected in arena with width = ");
  Serial.println(w);
  if(w > 90){
    robotState = FORWARD; // lägga in en bailout här??
      
  } else if (x <= w) {
      robotState = LEFT;
  } else if (x >= 240-w) {
      robotState = RIGHT;
  } else if(x >w && x < 240-w){
      robotState = FORWARD;
  } else {
      robotState = SEARCH;
  }
}

void checkTimeout(RobotState lastRobotState, RobotState robotState){
  // Timeout detection logic
  if (robotState == lastRobotState) {
    repeatCount++;
    Serial.print("Same state detected the last ");
    Serial.print(repeatCount);
    Serial.println(" times.");
    if (repeatCount >= repeatThreshold) {
      Serial.println("State repeated too many times. Bailing out TrAV...");
      bailout();
      repeatCount = 0; // reset after forcing SEARCH
    }
  } else {
    repeatCount = 0; // reset if state changes
  }
}



void setup() {
  if (virgin) {
    delay(1000);
    moveForward(stepTime);//stoppa in den
    virgin = false;
  }

  Serial.begin(9600);  
  Serial.println("STARTING SETUP");
  pinMode(A5, INPUT);  // Set pin A5 as input
  Wire.begin(ARDUINO_ADDRESS);  // Join I2C bus as slave
  Wire.onReceive(receiveData);   // Set function to handle received data
  pinMode(backLeftTrigPin, OUTPUT);  
	pinMode(backLeftEchoPin, INPUT);
  pinMode(backRightTrigPin, OUTPUT);  
	pinMode(backRightEchoPin, INPUT);
	pinMode(bucketEchoPin, INPUT);
  pinMode(bucketTrigPin, OUTPUT);  
  servoA.attach(servoPinA);  // attach to digital pin 5
  servoB.attach(servoPinB);  // attach to digital pin 6
  setBucketHeight("LOW");
  startTime = millis();
  robotState = SEARCH;
  lastRobotState = SEARCH;
}


void loop() {
  
  checkTimeout(lastRobotState, robotState);
  lastRobotState = robotState;
  bool ballInBucket = distanceTrig(bucketTrigPin, bucketEchoPin, 5, bucketDistance);
  if(ballInBucket && robotState != DUMP){ //Boll i skopa
    Serial.println("Ball detected in bucket.");
    setBucketHeight("MID");
    robotState = TO_NET;
  }
  
  switch (robotState) {
    case FORWARD:
      Serial.println("Moving forward");
      moveForward(moveTime); 
      break;
    case BACKWARD:
      Serial.println("Moving backward");
      moveBackward(moveTime); 
      break;
    case LEFT:
      Serial.println("Turning left");
      turnLeft(stepTime); 
      break;
    case RIGHT:
      Serial.println("Turning right");
      turnRight(stepTime); 
      break;
    case STOP:
      Serial.println("Stopping motors");
      stop(stopTime);
      break;
    case SEARCH:
      Serial.println("Searching...");
      search(stepTime);
      default:
      break;
    case TO_NET:
      Serial.println("Moving to net...");
      moveToNet(); //denna måste köras klart
      robotState = DUMP;
      break;
    case DUMP:
      Serial.println("DUMPING");
      stop(stopTime);
      setBucketHeight("MID");
      setBucketHeight("HI");
      setBucketHeight("LOW");
      delay(200);
      moveForward(moveTime/5);
      resetFunc();
      break;
    case IDLE:
      break;
}
}