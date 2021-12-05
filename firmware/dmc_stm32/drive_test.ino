#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <HardwareSerial.h>

// define pins for stm32 UART2
#define USE_STM32_HW_SERIAL
#define BAUD_RATE 57600
#define SERIAL_RX PA3
#define SERIAL_TX PA2

// Map DMC pins to stm32 pins
#define P0_ PA8
#define P1_ PA9
#define P2_ PA10
#define P3_ PA6
#define P4_ PA7
#define P5_ PB0

// global motor variables for use in interrupts
uint32_t motorP0;
uint32_t motorP1;
uint32_t motorP2;
uint32_t motorP3;
uint32_t motorP4;
uint32_t motorP5;

// high signals in microseconds
uint32_t motorP0Pulse = 1500;
uint32_t motorP1Pulse = 1500;
uint32_t motorP2Pulse = 1500;

uint32_t motorP3Pulse = 1500;
uint32_t motorP4Pulse = 1500;
uint32_t motorP5Pulse = 1500;

// global timer variables
HardwareTimer *Timer1;
HardwareTimer *Timer3;

// configure stm32 to use Serial2 hardware serial ports
HardwareSerial hserial(SERIAL_RX, SERIAL_TX);
class NewHardware : public ArduinoHardware
{
public:
    NewHardware() : ArduinoHardware(&hserial, BAUD_RATE){};
};
ros::NodeHandle_<NewHardware> nh;

int velToPulse(float vel)
{
    return map(vel, -4, 4, 1000, 2000);
}

void controlMotors(const geometry_msgs::Twist &cmd_vel)
{
    float left_speed = cmd_vel.linear.x + cmd_vel.angular.z;
    float right_speed = cmd_vel.linear.x - cmd_vel.angular.z;

    int left_pulse = velToPulse(left_speed);
    // front motors are reversed
    int front_left_pulse = velToPulse(left_speed * -1);

    int right_pulse = velToPulse(right_speed);
    int front_right_pulse = velToPulse(right_speed * -1);

    Timer1->setCaptureCompare(motorP0, front_left_pulse, MICROSEC_COMPARE_FORMAT);
    Timer1->setCaptureCompare(motorP1, left_pulse, MICROSEC_COMPARE_FORMAT);
    Timer1->setCaptureCompare(motorP2, left_pulse, MICROSEC_COMPARE_FORMAT);
    Timer3->setCaptureCompare(motorP3, front_right_pulse, MICROSEC_COMPARE_FORMAT);
    Timer3->setCaptureCompare(motorP4, right_pulse, MICROSEC_COMPARE_FORMAT);
    Timer3->setCaptureCompare(motorP5, right_pulse, MICROSEC_COMPARE_FORMAT);
}

void configureHardwareTimers()
{
    // Automatically retrieve timer instance and channel associated to pin
    // This is used to be compatible with all STM32 series automatically.
    TIM_TypeDef *Instance1 = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(P0_), PinMap_PWM);
    TIM_TypeDef *Instance2 = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(P3_), PinMap_PWM);

    motorP0 = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(P0_), PinMap_PWM));
    motorP1 = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(P1_), PinMap_PWM));
    motorP2 = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(P2_), PinMap_PWM));
    motorP3 = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(P3_), PinMap_PWM));
    motorP4 = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(P4_), PinMap_PWM));
    motorP5 = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(P5_), PinMap_PWM));

    // Instantiate HardwareTimer object. Thanks to 'new' instantiation, HardwareTimer is not destructed when setup() function is finished.
    Timer1 = new HardwareTimer(Instance1);
    Timer3 = new HardwareTimer(Instance2);

    // Configure and start PWM
    Timer1->setMode(motorP0, TIMER_OUTPUT_COMPARE_PWM1, P0_);
    Timer1->setMode(motorP1, TIMER_OUTPUT_COMPARE_PWM1, P1_);
    Timer1->setMode(motorP2, TIMER_OUTPUT_COMPARE_PWM1, P2_);
    Timer3->setMode(motorP3, TIMER_OUTPUT_COMPARE_PWM1, P3_);
    Timer3->setMode(motorP4, TIMER_OUTPUT_COMPARE_PWM1, P4_);
    Timer3->setMode(motorP5, TIMER_OUTPUT_COMPARE_PWM1, P5_);

    // complete signal period set to 3us
    Timer1->setOverflow(3 * 1000, MICROSEC_FORMAT);
    Timer3->setOverflow(3 * 1000, MICROSEC_FORMAT);

    // set initial pulse of 1500us (stopped)
    Timer1->setCaptureCompare(motorP0, motorP0Pulse, MICROSEC_COMPARE_FORMAT);
    Timer1->setCaptureCompare(motorP1, motorP1Pulse, MICROSEC_COMPARE_FORMAT);
    Timer1->setCaptureCompare(motorP2, motorP2Pulse, MICROSEC_COMPARE_FORMAT);
    Timer3->setCaptureCompare(motorP3, motorP3Pulse, MICROSEC_COMPARE_FORMAT);
    Timer3->setCaptureCompare(motorP4, motorP4Pulse, MICROSEC_COMPARE_FORMAT);
    Timer3->setCaptureCompare(motorP5, motorP5Pulse, MICROSEC_COMPARE_FORMAT);

    // enable timers
    Timer1->resume();
    Timer3->resume();
}

// subscribe to cmd_vel Twist topic
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &controlMotors);

void setup()
{
    configureHardwareTimers();
    nh.initNode();
    nh.subscribe(sub);
}

void loop()
{
    // required for rosserial
    nh.spinOnce();
    delay(1);
}
