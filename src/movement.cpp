#include <Arduino.h>

const int trigPin = 7;  
const int echoPin = 10; 
const int ledPin = 13;

// Motor control pins (using Mega PWM pins)
const int PWM_A   = 3,
          DIR_A   = 12,
          BRAKE_A = 9,
          SNS_A   = A0;

const int PWM_B   = 11,
          DIR_B   = 13,
          BRAKE_B = 8,
          SNS_B   = A1;

// Timing and speed constants
const int moveTime = 1000;      
const int turnTime = 500;       
const int stepTime = 20;
const int motorSpeed = 255;     


// Function prototypes
void setup();
void loop();
void moveForward(int duration);
void moveBackward(int duration);
void turnRight(int duration);
void turnLeft(int duration);
void stopMotors();

// Measurement constants
float duration, distance;

// Functions

long getDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  return pulseIn(echoPin, HIGH) * 0.034 / 2;
}

void lightOn() {
    digitalWrite(ledPin, HIGH);  
}

void lightOff() {
    digitalWrite(ledPin, LOW);  
}

void moveForward(int duration) {
    digitalWrite(DIR_A, LOW);    
    digitalWrite(DIR_B, HIGH);    
    analogWrite(PWM_A, motorSpeed);
    analogWrite(PWM_B, motorSpeed);
    delay(duration);
    Serial.println("Moving forward");
}

void moveBackward(int duration) {
    digitalWrite(DIR_A, HIGH);     
    digitalWrite(DIR_B, LOW);     
    analogWrite(PWM_A, motorSpeed);
    analogWrite(PWM_B, motorSpeed);
    delay(duration);
    Serial.println("Moving backward");
}

void stepForward() {
    digitalWrite(DIR_A, LOW);    
    digitalWrite(DIR_B, HIGH);    
    analogWrite(PWM_A, motorSpeed);
    analogWrite(PWM_B, motorSpeed);
    Serial.println("Stepping forward");
    delay(stepTime);                    
    stopMotors();                        
}

void turnRight(int duration) {
    digitalWrite(DIR_A, LOW);    
    digitalWrite(DIR_B, LOW);     
    analogWrite(PWM_A, motorSpeed);
    analogWrite(PWM_B, motorSpeed);
    delay(duration);
    Serial.println("Turning right");
} 

void turnLeft(int duration) {
    digitalWrite(DIR_A, HIGH);     
    digitalWrite(DIR_B, HIGH);    
    analogWrite(PWM_A, motorSpeed);
    analogWrite(PWM_B, motorSpeed);
    delay(duration);
    Serial.println("Turning left");
}

void stopMotors() {
    analogWrite(PWM_A, 0);
    analogWrite(PWM_B, 0);
    Serial.println("Stopped");
}

// SETUP AND LOOP

void setup() {  
    pinMode(trigPin, OUTPUT);  
    pinMode(echoPin, INPUT);

    // Set motor pins as outputs
    pinMode(PWM_A, OUTPUT);
    pinMode(DIR_A, OUTPUT);
    pinMode(BRAKE_A, OUTPUT);
    pinMode(PWM_B, OUTPUT);
    pinMode(DIR_B, OUTPUT);
    pinMode(BRAKE_B, OUTPUT);
    
    // Release motor brakes
    digitalWrite(BRAKE_A, LOW);
    digitalWrite(BRAKE_B, LOW);

    // Start with motors off
    stopMotors();

    // Initialize Serial for debugging 
    Serial.begin(115200);  
}  

void loop() {  
  digitalWrite(trigPin, LOW);  
  delayMicroseconds(2);  
  digitalWrite(trigPin, HIGH);  
  delayMicroseconds(10);  
  digitalWrite(trigPin, LOW);  

  duration = pulseIn(echoPin, HIGH);  
  distance = (duration * 0.0343) / 2;

  Serial.println(distance);

  if (distance < 30) {
      stopMotors();          // Stanna
      delay(1000);           // Vänta 1 sek
      moveBackward(200);     // Backa i 200 ms
      stopMotors();          // Stanna igen
      delay(500);            // Vänta 500 ms
      turnRight(325);        // Sväng höger i 350 ms
      stopMotors();          // Stanna efter sväng
      delay(500);            // Vänta 500 ms
      moveForward(500);      // Kör framåt för att fortsätta röra sig
  } 
  else {
      moveForward(50);       // Fortsätt köra framåt om det är fritt
  }
}

