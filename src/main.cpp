#include <Seeed_Arduino_SSCMA.h>
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

void moveForward(int duration);
void moveBackward(int duration);
void turnRight(int duration);
void turnLeft(int duration);


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


SSCMA AI;

void setup()
{

    AI.begin();
    Serial.begin(9600);
    while(!Serial);
    pinMode(LED_BUILTIN, OUTPUT);

}

void loop()
{

    if (!AI.invoke())
    {
        Serial.println("Invoke success");
        Serial.printf("perf: prepocess=%d, inference=%d, postprocess=%d\n",
                      AI.perf().prepocess, AI.perf().inference,
                      AI.perf().postprocess);
        for (int i = 0; i < AI.boxes().size(); i++)
        {
            delay(500);
            Serial.printf(
                "box %d: x=%d, y=%d, w=%d, h=%d, score=%d, target=%d\n", i,
                AI.boxes()[i].x, AI.boxes()[i].y, AI.boxes()[i].w,
                AI.boxes()[i].h, AI.boxes()[i].score, AI.boxes()[i].target);
        }
        for (int i = 0; i < AI.classes().size(); i++)
        {
            Serial.printf("class %d: target=%d, score=%d\n", i,
                          AI.classes()[i].target, AI.classes()[i].score);
        }
        for (int i = 0; i < AI.points().size(); i++)
        {
            Serial.printf("point %d: x=%d, y=%d, z=%d, score=%d, target=%d\n",
                          i, AI.points()[i].x, AI.points()[i].y,
                          AI.points()[i].z, AI.points()[i].score,
                          AI.points()[i].target);
        }

        //Navigera skutan
/*         if (AI.boxes().size() > 0)
        {
            digitalWrite(LED_BUILTIN, HIGH);
            if (AI.boxes()[0].x < 80)
            {   
                turnLeft(100);
            }
            else if (AI.boxes()[0].x > 200)
            {   
                turnRight(100);
            }
            else
            {
                moveForward(100);
            }
        }
        else
        {
            digitalWrite(LED_BUILTIN, LOW);
        } */
    }
}