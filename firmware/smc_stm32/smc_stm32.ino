// Serial arm control firmware
// Used serial python program as input

#define stepDelay 1000

// define pins for stm32 UART2
#define USE_STM32_HW_SERIAL
#define BAUD_RATE 115200
#define SERIAL_RX PA10
#define SERIAL_TX PA9

// Map SM pins to stm32 pins
#define SM0_DIR PB7 // Header J16 direction
#define SM0_PUL PB6 // Header J16 step

#define SM1_DIR PB9 // Header J20 direction
#define SM1_PUL PB8 // Header J20 step

#define SM2_DIR PB11 // Header J24 direction
#define SM2_PUL PB10 // Header J24 step

#define SM3_DIR PB13 // Header J17 direction
#define SM3_PUL PB12 // Header J17 step

#define SM4_DIR PB15 // Header J21 direction
#define SM4_PUL PB14 // Header J21 step

#define SM5_DIR PB1 // Header J25 direction
#define SM5_PUL PB0 // Header J25 step

// struct stepper
// {
//   uint8_t dir;
//   uint8_t pulse;
// };

// struct motors
// {
//   stepper S0;
//   stepper S1;
//   stepper S2;
//   stepper S3;
//   stepper S4;
//   stepper S5;
// };

// stepper S0, S1, S2, S3, S4, S5;

// configure stm32 to use Serial2 hardware serial ports
HardwareSerial hserial(SERIAL_RX, SERIAL_TX);

void setup()
{
  hserial.begin(BAUD_RATE);

  pinMode(SM0_PUL, OUTPUT);
  pinMode(SM0_DIR, OUTPUT);

  pinMode(SM1_PUL, OUTPUT);
  pinMode(SM1_DIR, OUTPUT);

  pinMode(SM2_PUL, OUTPUT);
  pinMode(SM2_DIR, OUTPUT);

  pinMode(SM3_PUL, OUTPUT);
  pinMode(SM3_DIR, OUTPUT);

  pinMode(SM4_PUL, OUTPUT);
  pinMode(SM4_DIR, OUTPUT);

  pinMode(SM5_PUL, OUTPUT);
  pinMode(SM5_DIR, OUTPUT);

  pinMode(LED_BUILTIN, OUTPUT);
}

bool ledState = false;

void loop()
{
  while (!hserial.available())
    ;
  char command = hserial.read();

  digitalWrite(LED_BUILTIN, ledState);
  ledState = !ledState;

  switch (command)
  {
  // shoulder/base
  case 'w':
    stepDualMotors(SM0_PUL, SM0_DIR, LOW, SM1_PUL, SM1_DIR, HIGH);
    break;
  case 's':
    stepDualMotors(SM0_PUL, SM0_DIR, HIGH, SM1_PUL, SM1_DIR, LOW);
    break;

  // shoulder/base
  case 'a':
    stepDualMotors(SM0_PUL, SM0_DIR, HIGH, SM1_PUL, SM1_DIR, HIGH);
    break;
  case 'd':
    stepDualMotors(SM0_PUL, SM0_DIR, LOW, SM1_PUL, SM1_DIR, LOW);
    break;

  // elbow
  case 't':
    stepMotor(SM2_PUL, SM2_DIR, LOW);
    break;
  case 'g':
    stepMotor(SM2_PUL, SM2_DIR, HIGH);
    break;

  // wrist
  case 'f':
    stepDualMotors(SM3_PUL, SM3_DIR, LOW, SM4_PUL, SM4_DIR, HIGH);
    break;
  case 'h':
    stepDualMotors(SM3_PUL, SM3_DIR, HIGH, SM4_PUL, SM4_DIR, LOW);
    break;

  // wrist
  case 'i':
    stepDualMotors(SM3_PUL, SM3_DIR, HIGH, SM4_PUL, SM4_DIR, HIGH);
    break;
  case 'k':
    stepDualMotors(SM3_PUL, SM3_DIR, LOW, SM4_PUL, SM4_DIR, LOW);
    break;

  // claw
  case 'j':
    stepMotor(SM5_PUL, SM5_DIR, LOW);
    break;
  case 'l':
    stepMotor(SM5_PUL, SM5_DIR, HIGH);
    break;

  default:
    break;
  }
}

void stepMotor(uint8_t pulPin, uint8_t dirPin, uint8_t dir)
{
  // digitalWriteFast(dirPin, HIGH - dir);
  digitalWrite(dirPin, HIGH - dir);
  for (int i = 0; i < 20; i++)
  {
    // digitalWriteFast(pulPin, HIGH);
    digitalWrite(pulPin, HIGH);
    delayMicroseconds(stepDelay);
    // digitalWriteFast(pulPin, LOW);
    digitalWrite(pulPin, LOW);
    delayMicroseconds(stepDelay);
  }
}

void stepDualMotors(uint8_t pulPin1, uint8_t dirPin1, uint8_t dir1, uint8_t pulPin2, uint8_t dirPin2, uint8_t dir2)
{
  // digitalWriteFast(dirPin1, HIGH - dir1);
  digitalWrite(dirPin1, HIGH - dir1);
  // digitalWriteFasttt(dirPin2, HIGH - dir2);
  digitalWrite(dirPin2, HIGH - dir2);
  for (int i = 0; i < 20; i++)
  {
    // digitalWriteFast(pulPin1, HIGH);
    digitalWrite(pulPin1, HIGH);
    // digitalWriteFast(pulPin2, HIGH);
    digitalWrite(pulPin2, HIGH);
    delayMicroseconds(stepDelay);
    // digitalWriteFast(pulPin1, LOW);
    digitalWrite(pulPin1, LOW);
    // digitalWriteFast(pulPin2, LOW);
    digitalWrite(pulPin2, LOW);
    delayMicroseconds(stepDelay);
  }
}
