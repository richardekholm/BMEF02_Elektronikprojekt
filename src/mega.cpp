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
          backEchoPin = 31,
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
const int motorSpeed = 200;



int hits = 0;
unsigned long startTime = 0;
unsigned long lastMoveTime = 0;
const unsigned long moveDuration = 500; // time per action in ms


// Declare robot state and timing variables
enum RobotState {IDLE, FORWARD, LEFT, RIGHT, STOP, BACKWARD, SEARCH, TEST, TO_NET, DUMP};
RobotState robotState = IDLE;

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
boolean ballInBucket();
float getInstantDistance(int trigPin, int echoPin);
boolean turnToDirection(int targetDir);



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
  analogWrite(PWM_A, motorSpeed);
  analogWrite(PWM_B, motorSpeed);
  delay(duration);
  stop(stopTime);
}

void moveBackward(int duration) {
  digitalWrite(BRAKE_A, LOW);
  digitalWrite(BRAKE_B, LOW);
  digitalWrite(DIR_A, HIGH);     
  digitalWrite(DIR_B, HIGH);     
  analogWrite(PWM_A, motorSpeed);
  analogWrite(PWM_B, motorSpeed);
  delay(duration);
}

void moveForward(int duration) {
  digitalWrite(BRAKE_A, LOW);    
  digitalWrite(BRAKE_B, LOW);
  digitalWrite(DIR_A, LOW);    
  digitalWrite(DIR_B, LOW);     
  analogWrite(PWM_A, motorSpeed);
  analogWrite(PWM_B, motorSpeed);
  delay(duration);
  robotState = STOP;     
} 

void turnLeft(int duration) {
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
  if (duration == 0 || duration > 25000) {
    // return -1.0f;     // signal “bad reading”    Tog bort för den kan trigga conditions
  }
  // convert to cm
  return (duration * 0.0343f) / 2.0f;
}

void search(int duration) {
  turnRight(stepTime); 
}

void setBucketHeight(String height){
  if(height == "LOW"){
    servoA.write(110);    // Move to 0 degrees
    servoB.write(10);     // Move to 0 degrees
    Serial.println("Moving bucket to 10 degrees");
    delay(1000); //alltså sänka denna?
  }
  else if(height == "MID"){
    servoA.write(55);     // Move to 55 degrees
    servoB.write(65);     // Move to 55 degrees
    Serial.println("Moving bucket to 55 degrees");
    delay(1000);
  }
  else if(height == "HI"){
    servoA.write(0);      // Move to 120 degrees
    servoB.write(120);    // Move to 120 degrees
    Serial.println("Moving bucket to 90 degrees");
    delay(1000);
  }
  else if (height == "JUST ABOVE"){
    servoA.write(107);     // Move to 90 degrees
    servoB.write(13);     // Move to 90 degrees
    Serial.println("Moving bucket to 90 degrees");
    delay(1000);
  }
  else{
    Serial.println("Invalid height");
  }
}

boolean ballInBucket(){
  float dis = getInstantDistance(bucketTrigPin, bucketEchoPin);
  //Serial.println(dis);
  
  if (dis < 14){
    hits++;
  }

  unsigned long now = millis();
  if (now - startTime >= 500UL) {
    if (hits > 30) {
      Serial.println("Ball detected in bucket.");
      return true;
    }
    hits = 0;
    startTime = now;
  }
  return false;
}


boolean distanceTrig(int trigPin, int echoPin, int nbrOfHits, float threshold){
  int hitStreak = 0;
  for(int i = 0; i < nbrOfHits; i++){
    float dist = getInstantDistance(trigPin, echoPin);
    Serial.println(dist);
    // Serial.print(", ");
    if(dist < threshold && dist > 0.1){//Anta att värden < 0,1 är felaktiga
      hitStreak ++;
    }
  }
  Serial.println("");
  
  if (hitStreak == nbrOfHits){
    return true;
  }
  return false;
} 

void moveToNet2(){
  boolean backDistance = distanceTrig(backTrigPin, backEchoPin, 10, 5);
  setBucketHeight("MID"); //Bucket in transport mode 
  
  while(!backDistance){//While not at net, randomly JUMP AROUND
    Serial.print("backdistance = " );
    Serial.println(getInstantDistance(backTrigPin, backEchoPin));
    
    moveBackward(500);
    backDistance = distanceTrig(backTrigPin, backEchoPin, 10, 5);
    if (!backDistance){
      moveForward(500);
    }
  }
  // robotState = DUMP;   // görs ändå i cases
}

void moveToNet(){
  setBucketHeight("MID"); //Bucket in transport mode 
  
  float rightDistance = getInstantDistance(rightTrigPin, rightEchoPin);
  float backDistance = getInstantDistance(backTrigPin, backEchoPin);
  Serial.print("rightDistance = " );
  Serial.println(rightDistance);
  float shortDistance = 3;
  float midDistance = 8; //Beror på pinnens längd
  while(backDistance > shortDistance){
    
    if(backDistance > midDistance && rightDistance > midDistance){ //CASE 1: Center of arena (Find wall to begin traversing)
      Serial.println("case 1");
      moveBackward(stepTime);
    }
    else if(backDistance < midDistance && backDistance > shortDistance && rightDistance > midDistance){ //CASE 2: Back at wall, not parallell to wall (rotate until parallell)
      Serial.println("case 2");
      moveBackward(stepTime/2);
      backDistance = getInstantDistance(backTrigPin, backEchoPin);
      if (backDistance < shortDistance){
        break;
      }
      moveForward(stepTime);
      while (rightDistance > midDistance){
        rightDistance = getInstantDistance(rightTrigPin, rightEchoPin);
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
    else{
      Serial.println("case 4");
      Serial.println("Net found.");
      robotState = DUMP;
    }
    
    rightDistance = getInstantDistance(rightTrigPin, rightEchoPin);
    backDistance = getInstantDistance(backTrigPin, backEchoPin);
    delay(stepTime);
  }
}

void receiveData(int byteCount) {
  if (byteCount < 10){
    return;
  }
  if (robotState == TO_NET) { // If already moving to net, ignore new data
    return;
  }
  
  int target = Wire.read();
  int score  = Wire.read();
  
  int x = (Wire.read() << 8) | Wire.read();
  int y = (Wire.read() << 8) | Wire.read();
  int w = (Wire.read() << 8) | Wire.read();
  int h = (Wire.read() << 8) | Wire.read();

  // int midX = x + (w / 2); // vi ska testa  om detta är the case eller om x redan returneras som mittpunkt från ESP32
  Serial.print("Ball detected in arena @ X = ");
  Serial.println(x);
  Serial.print("Ball detected in arena with width = ");
  Serial.println(w);
  if(w>90){
    robotState = FORWARD;
  }
  else if (x <= w) {
    robotState = LEFT;
  } else if (x >= 240-w) {
    robotState = RIGHT;
  } else if(x >w && x < 240-w){
    if (!ballInBucket()) {
      robotState = FORWARD;
    } else {
      robotState = STOP;
    }
  }
  lastMoveTime = millis();  // Update last move time
}

void setup() {
  Serial.begin(9600);  
  Serial.println("STARTING SETUP");
  pinMode(A5, INPUT);  // Set pin A5 as input
  Wire.begin(ARDUINO_ADDRESS);  // Join I2C bus as slave
  Wire.onReceive(receiveData);   // Set function to handle received data
  // pinMode(rightTrigPin, OUTPUT);  //Används inte längre
	// pinMode(rightEchoPin, INPUT);
  pinMode(backTrigPin, OUTPUT);  
	pinMode(backEchoPin, INPUT);
	pinMode(bucketEchoPin, INPUT);
  pinMode(bucketTrigPin, OUTPUT);  
  servoA.attach(servoPinA);  // attach to digital pin 5
  servoB.attach(servoPinB);  // attach to digital pin 6
  setBucketHeight("LOW");
  startTime = millis();
  robotState = SEARCH;
}


void loop() {
// if ((millis() - lastMoveTime) > moveDuration && robotState != SEARCH) {
//     // stop();
//     robotState = SEARCH;
//     return;
// }


  if(distanceTrig(bucketTrigPin, bucketEchoPin, 15, 14)){ //Boll i skopa
    Serial.println("Ball detected in bucket.");
    robotState = TO_NET;
  }
  robotState = TO_NET;

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
      moveToNet2(); //denna måste köras klart
      robotState = DUMP;
      break;
    case DUMP:
      Serial.println("DUMPING");
      setBucketHeight("MID");
      setBucketHeight("HI");
      setBucketHeight("LOW");
      delay(500);
      moveForward(1000);
      robotState = SEARCH;
      break;
    case IDLE:
      break;

}
}


