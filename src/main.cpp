#include <Arduino.h>
#include <HardwareSerial.h>
#include <Servo.h> 

const int pinLeftBack = A0;
const int pinLeftForward = A1;
const int pinRightBack = A2;
const int pinRightForward = A3;
const int pinMotorSpeedLeft = 5;
const int pinMotorSpeedRight = 6;

const int motorSpeedLeft = 870;      // used to calibrate asymmetries between motors
const int motorSpeedRight = 860;     // used to calibrate asymmetries between motors

const int inputPin = 9;     // Define the ultrasound signal receiving a Pin
const int outputPin = 8;    // Define the ultrasound signal emission Pin

int forwardDistance = 0;    // go
int rightDistance = 0;      // The right to
int leftDistance = 0;       // Turn left to

Servo distanceDetectionServo;    // Set up the Servo for distance detector
int goDirection = 0;             // forward = 8, back = 2, left = 4, right = 6 
const int delayTime = 250;       // After the servo motor to the stability of the time

const int goForwardDirection = 8;    // go forward
const int goRightDirection = 6;      // go right
const int goLeftDirection = 4;       // go left
const int goBackDirection = 2;       // go back

void detection();               // Measuring three angles (0, 90 and 179)
void measureDistanceAhead();
void measureDistanceRight();
void measureDistanceLeft();
void advance(int i);
void turnRight(int i);
void turnLeft(int i);
void stop(int i);
void back(int i);


void setup() {

    Serial.begin(9600);

    pinMode(pinLeftBack, OUTPUT);
    pinMode(pinLeftForward, OUTPUT);
    pinMode(pinRightBack, OUTPUT);
    pinMode(pinRightForward, OUTPUT);
    pinMode(pinMotorSpeedLeft, OUTPUT);
    pinMode(pinMotorSpeedRight, OUTPUT);

    analogWrite(pinMotorSpeedLeft, motorSpeedLeft);
    analogWrite(pinMotorSpeedRight, motorSpeedRight);
    
    pinMode(inputPin, INPUT);
    pinMode(outputPin, OUTPUT);

    distanceDetectionServo.attach(11);

}
    
void loop() {

    distanceDetectionServo.write(90);  // Make the servo motor ready position prepared for the next measurement
    detection();        // Measuring angle and determine which direction to go to
      
    if(goDirection == 2) {
        back(8);
        turnLeft(2);
        Serial.print("Reverse ");
    }

    if(goDirection == 6) {
        back(1);
        turnRight(6);
        Serial.print("Right ");
    }

    if(goDirection == 4) {
        back(1);
        turnLeft(6);
        Serial.print("Left ");
    } 

    if(goDirection == 8) { 
        advance(1);
        Serial.print("Advance \n");
    }

}

void detection() {

    int delayTime = 200;      // After the servo motor to the stability of the time
    measureDistanceAhead();
      
    if(forwardDistance < 10) {    // If the front distance less than 10 cm
        stop(1);
        back(2);
    }
           
    if(forwardDistance < 25) {     // If the front distance less than 25 cm
      
        stop(1);
        measureDistanceLeft();
        delay(delayTime);               // Waiting for the servo motor is stable
        measureDistanceRight();        
        delay(delayTime);               // Waiting for the servo motor is stable  
        
        if(leftDistance > rightDistance) {    // If the distance is greater than the right distance on the left
            goDirection = goLeftDirection;
        }
        
        if(leftDistance <= rightDistance) {   // If the distance is less than or equal to the distance on the right
            goDirection = goRightDirection;
        } 
        
        if (leftDistance < 15 && rightDistance < 15) {  // If the left front distance and distance and the right distance is less than 15 cm
            goDirection = goBackDirection;
        }
  
    } else {   // If the front is not less than 25 cm (greater than)    
        goDirection = goForwardDirection;     
    }

}

void measureDistanceAhead() {

    distanceDetectionServo.write(90);
    digitalWrite(outputPin, LOW);     // For low voltage 2 us ultrasonic launch
    delayMicroseconds(2);
    digitalWrite(outputPin, HIGH);    // Let ultrasonic launch 10 us high voltage, there is at least 10 us
    delayMicroseconds(10);
    digitalWrite(outputPin, LOW);     // Maintaining low voltage ultrasonic launch
    
    float distance = pulseIn(inputPin, HIGH);    // Read the time difference
    distance = distance / 5.8 / 10;              // A time to distance distance (unit: cm)
    Serial.print("Forward distance: ");
    Serial.println(distance);
    forwardDistance = distance;

}

void measureDistanceLeft() {

    distanceDetectionServo.write(5);
    delay(delayTime);
    digitalWrite(outputPin, LOW);     // For low voltage 2 us ultrasonic launch
    delayMicroseconds(2);
    digitalWrite(outputPin, HIGH);    // Let ultrasonic launch 10 us high voltage, there is at least 10 us
    delayMicroseconds(10);
    digitalWrite(outputPin, LOW);     // Maintaining low voltage ultrasonic launch

    float distance = pulseIn(inputPin, HIGH);    // Read the time difference
    distance = distance / 5.8 / 10;              // Will be time to distance distance (unit: cm)
    Serial.print("Left distance:");
    Serial.println(distance);
    leftDistance = distance;

} 

void measureDistanceRight() {

    distanceDetectionServo.write(177);
    delay(delayTime);
    digitalWrite(outputPin, LOW);     // For low voltage 2 us ultrasonic launch
    delayMicroseconds(2);
    digitalWrite(outputPin, HIGH);    // Let ultrasonic launch 10 us high voltage, there is at least 10 us
    delayMicroseconds(10);
    digitalWrite(outputPin, LOW);     // Maintaining low voltage ultrasonic launch

    float distance = pulseIn(inputPin, HIGH);    // Read the time difference
    distance = distance / 5.8 / 10;              // Will be time to distance distance (unit: cm)
    Serial.print("Right distance:");
    Serial.println(distance);
    rightDistance = distance;

}

void advance(int i) { // go forward

    digitalWrite(pinRightBack, LOW);
    digitalWrite(pinRightForward, HIGH);

    digitalWrite(pinLeftBack, LOW);
    digitalWrite(pinLeftForward, HIGH);

    delay(i * 1);

}

void turnRight(int i) {

    digitalWrite(pinRightBack, HIGH);
    digitalWrite(pinRightForward, LOW);

    digitalWrite(pinLeftBack, LOW);
    digitalWrite(pinLeftForward, HIGH);

    delay(i * 50);

}

void turnLeft(int i)  {

    digitalWrite(pinRightBack, LOW);
    digitalWrite(pinRightForward, HIGH);

    digitalWrite(pinLeftBack, HIGH);
    digitalWrite(pinLeftForward, LOW);

    delay(i * 50);

} 

void stop(int i) {

    digitalWrite(pinRightBack, LOW);
    digitalWrite(pinRightForward, LOW);

    digitalWrite(pinLeftBack, LOW);
    digitalWrite(pinLeftForward, LOW);

    delay(i * 100);

}

void back(int i) {

    digitalWrite(pinRightBack, HIGH);
    digitalWrite(pinRightForward, LOW);

    digitalWrite(pinLeftBack, HIGH);
    digitalWrite(pinLeftForward, LOW);
     
    delay(i * 500);

}
