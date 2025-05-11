#include <Wire.h>
#include <Arduino.h>
#include <Servo.h>
#include <LSM303D.h>


#define ARDUINO_ADDRESS 8  // Must match ESP32 sender
#define GYRO_ADRESS 0x1E  

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
// LSM303D Lsm303d;


// Timing and speed constants
const int moveTime = 100;      
const int turnTime = 100;       
const int stepTime = 100;
const int stopTime = 500; 
const int motorSpeed = 90;

int16_t accel[3];  // we'll store the raw acceleration values here
int16_t mag[3];  // raw magnetometer values stored here
float realAccel[3];  // calculated acceleration values here
float heading, titleHeading;

int hits = 0;
unsigned long startTime = 0;


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
void setBucketHeight(String height);
boolean ballInBucket();
float getInstantDistance(int trigPin, int echoPin);
float getAvgDistance(int trigPin, int echoPin);
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

// float getInstantDistance(int trigPin, int echoPin){
//   float distance = 0;
//   float duration = 0;
//   digitalWrite(trigPin, LOW);  
//     delayMicroseconds(2);  
//     digitalWrite(trigPin, HIGH);  
//     delayMicroseconds(10);
//     digitalWrite(trigPin, LOW);
//     duration = pulseIn(echoPin, HIGH);
//     distance = (duration*.0343)/2;
//     return distance;
// }

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
    return -1.0f;     // signal “bad reading”
  }
  // convert to cm
  return (duration * 0.0343f) / 2.0f;
}

void search(int duration) {
  // delay(stepTime);
  turnRight(stepTime); 
  // stop(duration*2);
  // delay(stepTime*2);
}

void setBucketHeight(String height){
  if(height == "LOW"){
    servoA.write(110);    // Move to 0 degrees
    servoB.write(10);     // Move to 0 degrees
    Serial.println("Moving to 10 degrees");
    delay(1000); //alltså sänka denna?
  }
  else if(height == "MID"){
    servoA.write(55);     // Move to 55 degrees
    servoB.write(65);     // Move to 55 degrees
    delay(1000);
    Serial.println("Moving to 55 degrees");
  }
  else if(height == "HI"){
    servoA.write(0);      // Move to 120 degrees
    servoB.write(120);    // Move to 120 degrees
    Serial.println("Moving to 90 degrees");
    delay(1000);
  }
}

boolean ballInBucket(){
  float dis = getInstantDistance(bucketTrigPin, bucketEchoPin);
  Serial.println(dis);
  
  if (dis < 14){
    hits++;
  }

  unsigned long now = millis();
  if (now - startTime >= 500UL) {
    if (hits > 30) {
      Serial.println("Ball detected.");
      return true;
    }
    hits = 0;
    startTime = now;
  }
  return false;
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

  int midX = x + (w / 2); // vi ska testa  om detta är the case eller om x redan returneras som mittpunkt från ESP32
  
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
  // setBucketHeight("MID"); //Bucket in transport mode 
  
  // float rightDistance = getAvgDistance(rightTrigPin, rightEchoPin);
  // float backDistance = getAvgDistance(backTrigPin, backEchoPin);
  // float shortDistance = 4.5;
  // float midDistance = 8; //Beror på pinnens längd
  // while(backDistance > shortDistance){
  //   Serial.print("rightDistance = ");
  //   Serial.println(rightDistance);
  //   Serial.print("backDistance = ");
  //   Serial.println(backDistance);
    
  //   if(backDistance > midDistance && rightDistance > midDistance){ //CASE 1: Center of arena (Find wall to begin traversing)
  //     Serial.println("case 1");
  //     moveBackward(stepTime);
  //   }
  //   else if(backDistance < midDistance && backDistance > shortDistance && rightDistance > midDistance){ //CASE 2: Back at wall, not parallell to wall (rotate until parallell)
  //     Serial.println("case 2");
  //     moveBackward(stepTime/2);
  //     backDistance = getAvgDistance(backTrigPin, backEchoPin);
  //     if (backDistance < shortDistance){
  //       break;
  //     }
  //     moveForward(stepTime);
  //     while (rightDistance > midDistance){
  //       rightDistance = getAvgDistance(rightTrigPin, rightEchoPin);
  //       turnRight(stepTime*2);
  //     }
  //   }
  //   else if(backDistance > midDistance && rightDistance < midDistance){ //CASE 2,5: Back at wall, parallell to wall (Move along wall)
  //     Serial.println("case 2.5");
  //     if (rightDistance < 2*shortDistance){
  //       turnRight(stepTime*2);
  //       // continue;
  //     }
  //     moveBackward(stepTime);
  //   }
  //   else if(backDistance < midDistance && backDistance > shortDistance && rightDistance < midDistance){ //CASE 3: Corner (Rotate until parallell again)
  //     Serial.println("case 3");
  //     turnRight(stepTime);
  //   }
    
  //   rightDistance = getAvgDistance(rightTrigPin, rightEchoPin);
  //   backDistance = getAvgDistance(backTrigPin, backEchoPin);
  //   delay(stepTime);
  // }
  // Serial.println("case 4");
  // Serial.println("Net found.");
  // robotState = DUMP;
}

boolean turnToDirection(int targetDir){
  while(!Lsm303d.isMagReady());// wait for the magnetometer readings to be ready
	Lsm303d.getMag(mag);  // get the magnetometer values, store them in mag
  float dir = Lsm303d.getHeading(mag);// Ska det skickas med en parameter här? https://github.com/pololu/lsm303-arduino
  int leniency = 20;
  
  while(!(dir > targetDir - leniency && dir < targetDir + leniency)){ //Turn to face net
    turnRight(100);
    while(!Lsm303d.isMagReady());
    Lsm303d.getMag(mag);
    dir = Lsm303d.getHeading(mag);
  }
}


boolean distanceTrig(int trigPin, int echoPin, int nbrOfHits, float threshold){
  int hitStreak = 0;
  for(int i = 0; i < nbrOfHits; i++){
    float dist = getInstantDistance(trigPin, echoPin);
    Serial.println(dist);
    if(dist < threshold){
      hitStreak ++;
    }
  }
  Serial.print("hitStreak: ");
  Serial.println(hitStreak);
  if (hitStreak == nbrOfHits){
    return true;
  }
  return false;
} 

void setup() {
  // Wire.begin(ARDUINO_ADDRESS);  // Join I2C bus as slave
  Wire.begin();
  // Wire.onReceive(receiveData);   // Set function to handle received data
  pinMode(rightTrigPin, OUTPUT);  
	pinMode(rightEchoPin, INPUT);
  pinMode(backTrigPin, OUTPUT);  
	pinMode(backEchoPin, INPUT);
	pinMode(bucketEchoPin, INPUT);
  pinMode(bucketTrigPin, OUTPUT);  
  // servoA.attach(servoPinA);  // attach to digital pin 5
  // servoB.attach(servoPinB);  // attach to digital pin 6
  // setBucketHeight("LOW");
  
  
  // char rtn = 0;
  
  Serial.begin(9600);  
  Serial.println("STARTING SETUP");
  // Serial.println("\r\npower on");
  // rtn = Lsm303d.initI2C();
  // //rtn = Lsm303d.initSPI(SPI_CS);
  // if(rtn != 0)  // Initialize the LSM303, using a SCALE full-scale range
	// {
	// 	Serial.println("\r\nLSM303D is not found");
	// 	while(1);
	// }
	// else
	// {
	// 	Serial.println("\r\nLSM303D is found");
	// }

  startTime = millis();
}


void loop() {
  
  // if(distanceTrig(bucketTrigPin, bucketEchoPin, 10, 14)){ //Boll i skopa
  //   Serial.println("Ball detected.");
  //   robotState = DUMP;
  // }


  
	// Lsm303d.getAccel(accel);
	// while(!Lsm303d.isMagReady());// wait for the magnetometer readings to be ready
	// Lsm303d.getMag(mag);  // get the magnetometer values, store them in mag
	
	// for (int i=0; i<3; i++)
	// {
	// 	realAccel[i] = accel[i] / pow(2, 15) * ACCELE_SCALE;  // calculate real acceleration values, in units of g
	// }
	// heading = Lsm303d.getHeading(mag);
	// titleHeading = Lsm303d.getTiltHeading(mag, realAccel);
	
  // Serial.println(titleHeading, 3);
  Wire.beginTransmission(GYRO_ADRESS);
  Wire.write(0x42); // Set the register to read from
  Wire.write(0x43); // Set the register to read from
  Wire.endTransmission();
  Wire.requestFrom(GYRO_ADRESS, 2); // Request 6 bytes of data
  Serial.println(Wire.available());
  if (Wire.available() == 2) {
    int16_t MSBX = Wire.read() << 8 | Wire.read();
    int16_t LSBX = Wire.read() << 8 | Wire.read();
    Serial.println(MSBX);
    Serial.println(LSBX);
    
  }
  delay(100);
  robotState = IDLE;

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
      moveToNet();
      robotState = DUMP;
      break;
    case DUMP:
      Serial.println("DUMPING");
      setBucketHeight("MID");
      setBucketHeight("HI");
      setBucketHeight("LOW");
      delay(500);
      robotState = SEARCH;
      break;
    case IDLE:
      break;

}
}


