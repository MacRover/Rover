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

struct timers
{
    HardwareTimer *TimerA;
    HardwareTimer *TimerB;
};

struct motors
{
    uint32_t P0;
    uint32_t P1;
    uint32_t P2;
    uint32_t P3;
    uint32_t P4;
    uint32_t P5;
};

// global variables for use in interrupts
motors Motors;
timers Timers;

// configure stm32 to use Serial2 hardware serial ports
HardwareSerial hserial(SERIAL_RX, SERIAL_TX);
class NewHardware : public ArduinoHardware
{
public:
    NewHardware() : ArduinoHardware(&hserial, BAUD_RATE){};
};
ros::NodeHandle_<NewHardware> nh;

// subscribe to cmd_vel Twist topic
void controlMotors(const geometry_msgs::Twist &cmd_vel);
ros::Subscriber<geometry_msgs::Twist> velocity("cmd_vel", &controlMotors);

void setup()
{
    configureHardwareTimers();
    nh.initNode();
    nh.subscribe(velocity);
}

void loop()
{
    // required for rosserial
    nh.spinOnce();
    delay(1);
}

void controlMotors(const geometry_msgs::Twist &cmd_vel)
{
    const double left_speed = cmd_vel.linear.x + cmd_vel.angular.z;
    const double right_speed = cmd_vel.linear.x - cmd_vel.angular.z;

    Timers.TimerA->setCaptureCompare(Motors.P0, velToPulse(left_speed, true), MICROSEC_COMPARE_FORMAT);
    Timers.TimerA->setCaptureCompare(Motors.P1, velToPulse(left_speed), MICROSEC_COMPARE_FORMAT);
    Timers.TimerA->setCaptureCompare(Motors.P2, velToPulse(left_speed), MICROSEC_COMPARE_FORMAT);
    Timers.TimerB->setCaptureCompare(Motors.P3, velToPulse(right_speed, true), MICROSEC_COMPARE_FORMAT);
    Timers.TimerB->setCaptureCompare(Motors.P4, velToPulse(right_speed), MICROSEC_COMPARE_FORMAT);
    Timers.TimerB->setCaptureCompare(Motors.P5, velToPulse(right_speed), MICROSEC_COMPARE_FORMAT);
}

void configureHardwareTimers()
{
    // Automatically retrieve timer instance and channel associated to pin
    // This is used to be compatible with all STM32 series automatically.
    const TIM_TypeDef *Instance1 = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(P0_), PinMap_PWM);
    const TIM_TypeDef *Instance2 = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(P3_), PinMap_PWM);

    Motors.P0 = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(P0_), PinMap_PWM));
    Motors.P1 = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(P1_), PinMap_PWM));
    Motors.P2 = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(P2_), PinMap_PWM));
    Motors.P3 = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(P3_), PinMap_PWM));
    Motors.P4 = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(P4_), PinMap_PWM));
    Motors.P5 = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(P5_), PinMap_PWM));

    // Instantiate HardwareTimer object. Thanks to 'new' instantiation, HardwareTimer is not destructed when setup() function is finished.
    Timers.TimerA = new HardwareTimer(Instance1);
    Timers.TimerB = new HardwareTimer(Instance2);

    // Configure and start PWM
    Timers.TimerA->setMode(Motors.P0, TIMER_OUTPUT_COMPARE_PWM1, P0_);
    Timers.TimerA->setMode(Motors.P1, TIMER_OUTPUT_COMPARE_PWM1, P1_);
    Timers.TimerA->setMode(Motors.P2, TIMER_OUTPUT_COMPARE_PWM1, P2_);
    Timers.TimerB->setMode(Motors.P3, TIMER_OUTPUT_COMPARE_PWM1, P3_);
    Timers.TimerB->setMode(Motors.P4, TIMER_OUTPUT_COMPARE_PWM1, P4_);
    Timers.TimerB->setMode(Motors.P5, TIMER_OUTPUT_COMPARE_PWM1, P5_);

    // complete signal period set to 3us
    const uint16_t signalPeriod = 3000;
    Timers.TimerA->setOverflow(signalPeriod, MICROSEC_FORMAT);
    Timers.TimerB->setOverflow(signalPeriod, MICROSEC_FORMAT);

    // set initial pulse of 1500us (stopped)
    const uint16_t initialPulse = 1500;
    Timers.TimerA->setCaptureCompare(Motors.P0, initialPulse, MICROSEC_COMPARE_FORMAT);
    Timers.TimerA->setCaptureCompare(Motors.P1, initialPulse, MICROSEC_COMPARE_FORMAT);
    Timers.TimerA->setCaptureCompare(Motors.P2, initialPulse, MICROSEC_COMPARE_FORMAT);
    Timers.TimerB->setCaptureCompare(Motors.P3, initialPulse, MICROSEC_COMPARE_FORMAT);
    Timers.TimerB->setCaptureCompare(Motors.P4, initialPulse, MICROSEC_COMPARE_FORMAT);
    Timers.TimerB->setCaptureCompare(Motors.P5, initialPulse, MICROSEC_COMPARE_FORMAT);

    // enable timers
    Timers.TimerA->resume();
    Timers.TimerB->resume();
}

uint16_t velToPulse(const double vel, const bool reverse = false)
{
    if (reverse){
        // front motors face the opposite direction of the other motors
        // so they must spin in the opposite direction
        return map(vel, 4, -4, 1000, 2000);
    }
    return map(vel, -4, 4, 1000, 2000);
}