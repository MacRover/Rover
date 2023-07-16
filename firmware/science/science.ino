// Serial arm control firmware
// Used serial python program as input

#define stepDelay 1000


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

#define SM0_DIR 1 // J17
#define SM0_PUL 1

#define SM1_DIR 2 //J24 // //elbow
#define SM1_PUL 2

#define BASE_LEFT_DIR 0  //J16 //base left
#define BASE_LEFT_PUL 0

#define BASE_RIGHT_PUL 0
#define BASE_RIGHT_DIR 0

#define WRIST_RIGHT_PUL 0
#define WRIST_RIGHT_DIR 0

#define WRIST_LEFT_PUL 0
#define WRIST_LEFT_DIR 0

uint8_t PUL_PIN;
uint8_t DIR_PIN;
uint8_t DIR;


void setup()
{
  Serial.begin(115200);
  
  // pinMode(SM0_PUL, OUTPUT);
  // pinMode(SM0_DIR, OUTPUT);
  
  // pinMode(SM1_PUL, OUTPUT);
  // pinMode(SM1_DIR, OUTPUT);
  
  // pinMode(BASE_LEFT_DIR, OUTPUT);
  // pinMode(BASE_LEFT_PUL, OUTPUT);
  
  // pinMode(WRIST_RIGHT_PUL, OUTPUT);
  // pinMode(WRIST_RIGHT_DIR, OUTPUT);
  
  // pinMode(WRIST_LEFT_PUL, OUTPUT);
  // pinMode(WRIST_LEFT_DIR, OUTPUT);

  // pinMode(BASE_RIGHT_PUL, OUTPUT);
  // pinMode(BASE_RIGHT_DIR, OUTPUT);
}

void loop()
{
  if (Serial.available() > 0){
    char command = Serial.read();

    switch(command){
      case 'a': // auger anti-clockwise
        PUL_PIN =  SM0_PUL;
        DIR_PIN =  SM0_DIR;
        DIR = LOW;
        break;
      case 'd': // auger clockwise
        PUL_PIN =  SM0_PUL;
        DIR_PIN =  SM0_DIR;
        DIR = HIGH;
        break;
      case 's': // auger stop
        PUL_PIN =  0;
        DIR_PIN =  0;
        break;
      case 'q':
        PUL_PIN =  SM1_PUL;
        DIR_PIN =  SM1_DIR;
        DIR = LOW;
        break;
      case 'e':
        PUL_PIN =  SM1_PUL;
        DIR_PIN =  SM1_DIR;
        DIR = HIGH;
        break;
      case 'w':
        stepDualMotors(BASE_LEFT_PUL, BASE_LEFT_DIR, HIGH, BASE_RIGHT_PUL, BASE_RIGHT_DIR, LOW);
        break;                  
      case 'f':
        stepDualMotors(BASE_LEFT_PUL, BASE_LEFT_DIR, HIGH, BASE_RIGHT_PUL, BASE_RIGHT_DIR, HIGH);
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
    // digitalWrite(pulPin, HIGH);
    Serial.print(pulPin);
    Serial.println("I am stepping High");
    delayMicroseconds(stepDelay);
    // digitalWrite(pulPin, LOW);
    Serial.println("I an stepping Low");
}

void stepDualMotors(uint8_t pulPin1, uint8_t dirPin1, uint8_t dir1, uint8_t pulPin2, uint8_t dirPin2, uint8_t dir2){
  // digitalWrite(dirPin1, HIGH - dir1);
  // digitalWrite(dirPin2, HIGH - dir2);
  // for(int i=0;i<20;i++){
  //   digitalWrite(pulPin1, HIGH);
  //   digitalWrite(pulPin2, HIGH);
  //   delayMicroseconds(stepDelay);
  //   digitalWrite(pulPin1, LOW);
  //   digitalWrite(pulPin2, LOW);
  //   delayMicroseconds(stepDelay);
  // }
}