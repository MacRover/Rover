// Serial arm control firmware
// Used serial python program as input

#include "CytronMotorDriver.h"

#define stepDelay 100


// Map SM pins to stm32 pins
// #define SM0_DIR PB6 // J17
// #define SM0_PUL PB7

// #define SM1_DIR PB8 //J24 // //elbow
// #define SM1_PUL PB9

// #define BASE_LEFT_DIR PB12  //J16 //base left
// #define BASE_LEFT_PUL PB13

// #define BASE_RIGHT_PUL PB15
// #define BASE_RIGHT_DIR PB0

// #define WRIST_RIGHT_PUL PB11
// #define WRIST_RIGHT_DIR PB10

// #define WRIST_LEFT_PUL PB1
// #define WRIST_LEFT_DIR PB14

#define AUGER_DIR 6 // J17
#define AUGER_PUL 7

#define CAROUSEL_DIR 2 //J24 // //elbow
#define CAROUSEL_PUL 2

uint8_t PUL_PIN = 13;
uint8_t DIR_PIN = 13;
uint8_t DIR;

CytronMD motor(PWM_DIR, 9, 8);

void setup()
{
  Serial.begin(115200);
  
  pinMode(AUGER_DIR, OUTPUT);
  pinMode(AUGER_PUL, OUTPUT);
  
  pinMode(CAROUSEL_PUL, OUTPUT);
  pinMode(CAROUSEL_DIR, OUTPUT);
}

void loop()
{
  if (Serial.available() > 0){
    char command = Serial.read();

    switch(command){
      case 'a': // auger anti-clockwise
        PUL_PIN =  AUGER_PUL;
        DIR_PIN =  AUGER_DIR;
        DIR = LOW;
        digitalWrite(LED_BUILTIN, HIGH);
        break;
      case 'd': // auger clockwise
        PUL_PIN =  AUGER_PUL;
        DIR_PIN =  AUGER_DIR;
        digitalWrite(LED_BUILTIN, LOW);
        DIR = HIGH;
        break;
      case 's': // auger, carousel stop
        PUL_PIN =  13;
        DIR_PIN =  13;
        break;
      case 'q': //carousel clockwise
        PUL_PIN =  CAROUSEL_PUL;
        DIR_PIN =  CAROUSEL_DIR;
        DIR = LOW;
        break;
      case 'e': // carousel counterclockwise
        PUL_PIN =  CAROUSEL_PUL;
        DIR_PIN =  CAROUSEL_DIR;
        DIR = HIGH;
        break;                 
      case 'f': // drill clockwise
        motor.setSpeed(600);
        break;
      case 'g': // drill counterclockwise
        motor.setSpeed(-600);
        break;
      case 'h': // drill stop
        motor.setSpeed(0);
        break;
      default:
        break;
    }
    while (Serial.available() > 0){Serial.read();}//clear buffer
  }
  stepMotor(PUL_PIN, DIR_PIN, DIR);
  delayMicroseconds(stepDelay);
}

void stepMotor(uint8_t pulPin, uint8_t dirPin, uint8_t dir){
    digitalWrite(dirPin, dir);
    digitalWrite(pulPin, HIGH);
    delayMicroseconds(stepDelay);
    digitalWrite(pulPin, LOW);
}