// Serial arm control firmware
// Used serial python program as input
#include "CytronMotorDriver.h"

#define defaultStepDelay 1000
#define augerStepDelay 1000
#define carouselStepDelay 5000

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

typedef struct smc
{
  uint8_t dir_pin;
  uint8_t pul_pin;
} SMC;

SMC smc0 = {.dir_pin = PB7, .pul_pin = PB6};
SMC smc1 = {.dir_pin = PB9, .pul_pin = PB8};
SMC smc2 = {.dir_pin = PB11, .pul_pin = PB10};
SMC smc3 = {.dir_pin = PB13, .pul_pin = PB12};
SMC smc4 = {.dir_pin = PB15, .pul_pin = PB14};
SMC smc5 = {.dir_pin = PB1, .pul_pin = PB0};

// configure stm32 to use Serial2 hardware serial ports
HardwareSerial hserial(SERIAL_RX, SERIAL_TX);
CytronMD motor(PWM_DIR, PA6, PA7);

char command;
bool ledState = false;

void toggleLed()
{
  digitalWrite(LED_BUILTIN, ledState);
  ledState = !ledState;
}

void setup()
{
  hserial.begin(BAUD_RATE);

  pinMode(smc0.dir_pin, OUTPUT);
  pinMode(smc0.pul_pin, OUTPUT);

  pinMode(smc1.dir_pin, OUTPUT);
  pinMode(smc1.pul_pin, OUTPUT);

  pinMode(smc2.dir_pin, OUTPUT);
  pinMode(smc2.pul_pin, OUTPUT);

  pinMode(smc3.dir_pin, OUTPUT);
  pinMode(smc3.pul_pin, OUTPUT);

  pinMode(smc4.dir_pin, OUTPUT);
  pinMode(smc4.pul_pin, OUTPUT);

  pinMode(smc5.dir_pin, OUTPUT);
  pinMode(smc5.pul_pin, OUTPUT);

  pinMode(LED_BUILTIN, OUTPUT);

  toggleLed();
  delay(20);
  toggleLed();
}

void loop()
{

  if (hserial.available())
  {
    command = hserial.read();
    while (hserial.available() > 0)
    {
      hserial.read();
    } // clear buffer
  }
  switch (command)
  {
  // base up/down & carousel up/down
  case 'w':
    stepDualMotors(smc0, LOW, smc5, HIGH, defaultStepDelay);
    toggleLed();
    break;
  case 's':
    stepDualMotors(smc0, HIGH, smc5, LOW, defaultStepDelay);
    toggleLed();
    break;

  // base rotate
  case 'd':
    stepDualMotors(smc0, HIGH, smc5, HIGH, defaultStepDelay);
    toggleLed();
    break;
  case 'a':
    stepDualMotors(smc0, LOW, smc5, LOW, defaultStepDelay);
    toggleLed();
    break;

  // elbow
  case 'g':
    stepMotor(smc2, LOW, defaultStepDelay);
    toggleLed();
    break;
  case 't':
    stepMotor(smc2, HIGH, defaultStepDelay);
    toggleLed();
    break;

  // wrist
  case 'h':
    stepDualMotors(smc1, LOW, smc4, HIGH, defaultStepDelay);
    toggleLed();
    break;
  case 'f':
    stepDualMotors(smc1, HIGH, smc4, LOW, defaultStepDelay);
    toggleLed();
    break;

  // wrist
  case 'k':
    stepDualMotors(smc1, HIGH, smc4, HIGH, defaultStepDelay);
    toggleLed();
    break;
  case 'i':
    stepDualMotors(smc1, LOW, smc4, LOW, defaultStepDelay);
    toggleLed();
    break;

  // claw
  case 'l':
    stepMotor(smc3, LOW, defaultStepDelay);
    toggleLed();
    break;
  case 'j':
    stepMotor(smc3, HIGH, defaultStepDelay);
    toggleLed();
    break;

  case 'y':
    stepDualMotors(smc2, HIGH, smc4, HIGH, augerStepDelay);
    toggleLed();
    break;

  case 'u':
    stepDualMotors(smc2, LOW, smc4, LOW, augerStepDelay);
    toggleLed();
    break;

  // carousel clockwise
  case 'q':
    for (int i = 0; i < 50; i++)
    {
      stepMotor(smc1, HIGH, carouselStepDelay);
    }
    toggleLed();
    command = 'x';
    break;
  // carousel counterclockwise
  case 'e':
    for (int i = 0; i < 50; i++)
    {
      stepMotor(smc1, LOW, carouselStepDelay);
    }
    toggleLed();
    command = 'x';
    break;

  // motor clockwise
  case 'b':
    motor.setSpeed(600);
    toggleLed();
    break;
  // motor counter-clockwise
  case 'n':
    motor.setSpeed(-600);
    toggleLed();
    break;
  // motor stop
  case 'm':
    motor.setSpeed(0);
    toggleLed();
    break;

  default:
    delayMicroseconds(defaultStepDelay);
    break;
  }
}

void stepMotor(SMC stepper, uint8_t dir, unsigned int stepDelay)
{
  digitalWrite(stepper.dir_pin, dir);

  digitalWrite(stepper.pul_pin, HIGH);
  delayMicroseconds(stepDelay);
  digitalWrite(stepper.pul_pin, LOW);
  delayMicroseconds(stepDelay);
}

void stepDualMotors(SMC stepper1, uint8_t dir1, SMC stepper2, uint8_t dir2, unsigned int stepDelay)
{
  digitalWrite(stepper1.dir_pin, dir1);
  digitalWrite(stepper2.dir_pin, dir2);

  digitalWrite(stepper1.pul_pin, HIGH);
  digitalWrite(stepper2.pul_pin, HIGH);
  delayMicroseconds(stepDelay);
  digitalWrite(stepper1.pul_pin, LOW);
  digitalWrite(stepper2.pul_pin, LOW);
  delayMicroseconds(stepDelay);
}
