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
const int hitThreshold = 10; // Threshold for ball detection in bucket
const int bucketDistance = 12; // distance to detect ball in bucket
const int backTrigstreak = 5; // Number of triggers to detect wall
const int backShortDistance = 6.5; // Number of triggers to detect ball in bucket
const int backLongDistance = 10; // Number of triggers to detect ball in bucket



int hits = 0;
unsigned long startTime = 0;
unsigned long lastMoveTime = 0;
const unsigned long moveDuration = 500; // time per action in ms
bool virgin = true; //fräckis


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
boolean isStuck();
float getInstantDistance(int trigPin, int echoPin);
void(* resetFunc) (void) = 0; //ACSHUALLY
boolean backUpStraight();


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
  analogWrite(PWM_A, motorSpeedTurn);
  analogWrite(PWM_B, motorSpeedTurn);
  delay(duration);
  stop(stopTime);
}

void moveBackward(int duration) {
  digitalWrite(BRAKE_A, LOW);
  digitalWrite(BRAKE_B, LOW);
  digitalWrite(DIR_A, HIGH);     
  digitalWrite(DIR_B, HIGH);     
  analogWrite(PWM_A, motorSpeedMove);
  analogWrite(PWM_B, motorSpeedMove);
  delay(duration);
}

void moveForward(int duration) {
  digitalWrite(BRAKE_A, LOW);    
  digitalWrite(BRAKE_B, LOW);
  digitalWrite(DIR_A, LOW);    
  digitalWrite(DIR_B, LOW);     
  analogWrite(PWM_A, motorSpeedMove);
  analogWrite(PWM_B, motorSpeedMove);
  delay(duration);    
} 

void turnLeft(int duration) {
  delay(duration);
  digitalWrite(BRAKE_A, LOW);    
  digitalWrite(BRAKE_B, LOW);
  digitalWrite(DIR_A, LOW);     
  digitalWrite(DIR_B, HIGH);    
  analogWrite(PWM_A, motorSpeedTurn);
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
  if (duration == 0 || duration > 25000) {
    // return -1.0f;     // signal “bad reading”    Tog bort för den kan trigga conditions
  }
  // convert to cm
  return (duration * 0.0343f) / 2.0f;
}

void search(int duration) {
  turnRight(stepTime); 
}

boolean backUpStraight(){
  const int disparityAllowance = 3;
  float leftDistance = getInstantDistance(backLeftTrigPin, backLeftEchoPin);
  float rightDistance = getInstantDistance(backRightTrigPin, backRightEchoPin);
  float disparity = leftDistance - rightDistance;
  while(leftDistance > 20 || rightDistance > 20){//Långt ifrån väggen, back up
    moveBackward(moveTime);
    leftDistance = getInstantDistance(backLeftTrigPin, backLeftEchoPin);
    rightDistance = getInstantDistance(backRightTrigPin, backRightEchoPin);
    Serial.println("not close to wall");
    Serial.print("leftDistance = ");
    Serial.println(leftDistance);
    Serial.print("rightDistance = ");
    Serial.println(rightDistance);
  }
  while(abs(disparity) > disparityAllowance){
    if(disparity > 0){
      Serial.println("rotating left");
      turnLeft(stepTime/2);
      leftDistance = getInstantDistance(backLeftTrigPin, backLeftEchoPin);
      rightDistance = getInstantDistance(backRightTrigPin, backRightEchoPin);
      disparity = leftDistance - rightDistance;
    }
    else{
      Serial.println("rotating right");
      turnRight(stepTime/2);
      leftDistance = getInstantDistance(backLeftTrigPin, backLeftEchoPin);
      rightDistance = getInstantDistance(backRightTrigPin, backRightEchoPin);
      disparity = leftDistance - rightDistance;
    }
    leftDistance = getInstantDistance(backLeftTrigPin, backLeftEchoPin);
    rightDistance = getInstantDistance(backRightTrigPin, backRightEchoPin);
    Serial.print("leftDistance = ");
    Serial.println(leftDistance);
    Serial.print("rightDistance = ");
    Serial.println(rightDistance);
    disparity = leftDistance - rightDistance;  
  }
  Serial.println("backing up straight");
  moveBackward(moveTime);
  stop(1000);
  return true;

}

void setBucketHeight(String height){
  if(height == "LOW"){
    servoA.write(110);    // Move to 0 degrees
    servoB.write(10);     // Move to 0 degrees
    Serial.println("Moving bucket to 0 degrees");
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
  // float dis = getInstantDistance(backTrigPin, backEchoPin);
  // //Serial.println(dis);
  
  // if (dis > 280){
  //   hits++;
  // }

  // unsigned long now = millis();
  // if (now - startTime >= 500UL) {
  //   if (hits > 30) {
  //     Serial.println("TrAV is stuck");
  //     return true;
  //   }
  //   hits = 0;
  //   startTime = now;
  // }
  // return false;
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

void moveToNet(){
  // boolean closeToWall = distanceTrig(backTrigPin, backEchoPin, backTrigstreak, backShortDistance);
  // setBucketHeight("MID"); //Bucket in transport mode 
  
  // while(!closeToWall){//While not at net, randomly JUMP AROUND
  //   Serial.print("backdistance = " );
  //   Serial.println(getInstantDistance(backTrigPin, backEchoPin));
    
  //   moveBackward(500);
  //   stop(stopTime);
  //   if (distanceTrig(backTrigPin, backEchoPin, backTrigstreak, backLongDistance)){  //Trig vid alla väggar
  //     moveBackward(stepTime);
  //     if (distanceTrig(backTrigPin, backEchoPin, backTrigstreak, backShortDistance)){  //Vid nät
  //       break;
  //     }
  //     moveForward(stepTime);
  //     turnRight(stepTime);

  //   }
  //   closeToWall = distanceTrig(backTrigPin, backEchoPin, backTrigstreak, backShortDistance);
  // }
}

void receiveData(int byteCount) {
  if (byteCount < 10){  
    return;
  }
  if (robotState == TO_NET || robotState == DUMP) { // If already moving to net, ignore new data
    return;
  }
  
  int target = Wire.read();
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
    robotState = FORWARD;
  } else if (x <= w) {
    robotState = LEFT;
  } else if (x >= 240-w) {
    robotState = RIGHT;
  } else if(x >w && x < 240-w){
    // if (!distanceTrig(bucketTrigPin, bucketEchoPin, 20, 14)) {
      robotState = FORWARD;
      // }
    } else {
      robotState = SEARCH;
  }
}

void setup() {
  if (virgin) {
    moveForward(stepTime);//stoppa in den
    virgin = false;
  }

  Serial.begin(9600);  
  Serial.println("STARTING SETUP");
  pinMode(A5, INPUT);  // Set pin A5 as input
  Wire.begin(ARDUINO_ADDRESS);  // Join I2C bus as slave
  Wire.onReceive(receiveData);   // Set function to handle received data
  // pinMode(rightTrigPin, OUTPUT);  //Används inte längre
	// pinMode(rightEchoPin, INPUT);
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
}


void loop() {
  while(true){
    robotState = TO_NET;
  boolean bus = backUpStraight();
  while(!bus);
  }
  
  if(distanceTrig(bucketTrigPin, bucketEchoPin, hitThreshold, bucketDistance)){ //Boll i skopa
    Serial.println("Ball detected in bucket.");
    robotState = TO_NET; //Greed is good
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
      delay(500);
      moveForward(1000);
      // virgin = false;
      resetFunc();

      //robotState = SEARCH;
      break;
    case IDLE:
      break;
}
}


