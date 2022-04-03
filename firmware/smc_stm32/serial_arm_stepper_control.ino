// Serial arm control firmware
// Used serial python program as input

#define stepDelay 1000


// Map SM pins to stm32 pins
#define SM0_DIR PB6 // J17
#define SM0_PUL PB7

#define SM1_DIR PB8 //J24 // //elbow
#define SM1_PUL PB9

#define BASE_LEFT_DIR PB12  //J16 //base left
#define BASE_LEFT_PUL PB13

#define BASE_RIGHT_PUL PB15
#define BASE_RIGHT_DIR PB0

#define WRIST_RIGHT_PUL PB11
#define WRIST_RIGHT_DIR PB10

#define WRIST_LEFT_PUL PB1
#define WRIST_LEFT_DIR PB14


void setup()
{
  Serial.begin(115200);
  
  pinMode(SM0_PUL, OUTPUT);
  pinMode(SM0_DIR, OUTPUT);
  
  pinMode(SM1_PUL, OUTPUT);
  pinMode(SM1_DIR, OUTPUT);
  
  pinMode(BASE_LEFT_DIR, OUTPUT);
  pinMode(BASE_LEFT_PUL, OUTPUT);
  
  pinMode(WRIST_RIGHT_PUL, OUTPUT);
  pinMode(WRIST_RIGHT_DIR, OUTPUT);
  
  pinMode(WRIST_LEFT_PUL, OUTPUT);
  pinMode(WRIST_LEFT_DIR, OUTPUT);

  pinMode(BASE_RIGHT_PUL, OUTPUT);
  pinMode(BASE_RIGHT_DIR, OUTPUT);
}

void loop()
{
  while (!Serial.available());
    char command = Serial.read();

    switch(command){
      case 'w':
        stepMotor(SM0_PUL, SM0_DIR, LOW);
        break;
      case 's':
        stepMotor(SM0_PUL, SM0_DIR, HIGH);
        break;
      case 'a'://elbow
        stepMotor(SM1_PUL, SM1_DIR, LOW);
        break;
      case 'd'://elbow
        stepMotor(SM1_PUL, SM1_DIR, HIGH);
        break;
      case 't':
        stepDualMotors(BASE_LEFT_PUL, BASE_LEFT_DIR, LOW, BASE_RIGHT_PUL, BASE_RIGHT_DIR, HIGH);
        break;
      case 'g':
        stepDualMotors(BASE_LEFT_PUL, BASE_LEFT_DIR, HIGH, BASE_RIGHT_PUL, BASE_RIGHT_DIR, LOW);
        break;                  
      case 'f':
        stepDualMotors(BASE_LEFT_PUL, BASE_LEFT_DIR, HIGH, BASE_RIGHT_PUL, BASE_RIGHT_DIR, HIGH);
        break;
      case 'h':
        stepDualMotors(BASE_LEFT_PUL, BASE_LEFT_DIR, LOW, BASE_RIGHT_PUL, BASE_RIGHT_DIR, LOW);
        break;
      case 'j':
        stepDualMotors(WRIST_LEFT_PUL, WRIST_LEFT_DIR, LOW, WRIST_RIGHT_PUL, WRIST_RIGHT_DIR, HIGH);
        break;
      case 'l':
        stepDualMotors(WRIST_LEFT_PUL, WRIST_LEFT_DIR, HIGH, WRIST_RIGHT_PUL, WRIST_RIGHT_DIR, LOW);
        break;
      case 'i':
        stepDualMotors(WRIST_LEFT_PUL, WRIST_LEFT_DIR, HIGH, WRIST_RIGHT_PUL, WRIST_RIGHT_DIR, HIGH);
        break;
      case 'k':
        stepDualMotors(WRIST_LEFT_PUL, WRIST_LEFT_DIR, LOW, WRIST_RIGHT_PUL, WRIST_RIGHT_DIR, LOW);
        break;
      default:
        break;
    }
}

void stepMotor(uint8_t pulPin, uint8_t dirPin, uint8_t dir){
  digitalWrite(dirPin, HIGH - dir);
  for(int i=0;i<20;i++){
    digitalWrite(pulPin, HIGH);
    delayMicroseconds(stepDelay);
    digitalWrite(pulPin, LOW);
    delayMicroseconds(stepDelay);
  }
}

void stepMotorLow(uint8_t dirPin, uint8_t pulPin){
  digitalWrite(dirPin, LOW);
  for(int i=0;i<20;i++){
    digitalWrite(pulPin, HIGH);
    delayMicroseconds(stepDelay);
    digitalWrite(pulPin, LOW);
    delayMicroseconds(stepDelay);
  }
}

void stepDualMotors(uint8_t pulPin1, uint8_t dirPin1, uint8_t dir1, uint8_t pulPin2, uint8_t dirPin2, uint8_t dir2){
  digitalWrite(dirPin1, HIGH - dir1);
  digitalWrite(dirPin2, HIGH - dir2);
  for(int i=0;i<20;i++){
    digitalWrite(pulPin1, HIGH);
    digitalWrite(pulPin2, HIGH);
    delayMicroseconds(stepDelay);
    digitalWrite(pulPin1, LOW);
    digitalWrite(pulPin2, LOW);
    delayMicroseconds(stepDelay);
  }
}