#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <HardwareSerial.h>
#include <std_msgs/UInt16.h>


#include "stm32f1xx_hal_gpio_ex.h"
#include "PinAF_STM32F1.h"


// define pins for stm32 UART3
#define USE_STM32_HW_SERIAL
#define BAUD_RATE 57600
#define SERIAL_RX PB11
#define SERIAL_TX PB10

// Map DMC pins to stm32 pins
#define P0_ PA_0
#define P1_ PA_3
#define P2_ PA_11_ALT1
#define P3_ PA_8
#define P4_ PA_9
#define P5_ PA_10

#define E0A_ PA_6
#define E0B_ PA_7_ALT1

#define E1A_ PA_2

#define E2A_ PB_0_ALT1
#define E2B_ PB_1_ALT1

#define E3A_ PB_6
#define E3B_ PB_7

#define E4A_ PA_1//PB_3

#define E5A_ PB_8
#define E5B_ PB_9

struct timers
{    HardwareTimer *Tim1;
     HardwareTimer *Tim2;
     HardwareTimer *Tim3;
     HardwareTimer *Tim4;
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

struct encoders
{
    uint32_t E0A;
    uint32_t E0B;
    uint32_t E1A;
    uint32_t E2A;
    uint32_t E2B;
    uint32_t E3A;
    uint32_t E3B;
    uint32_t E4A;
    uint32_t E5A;
    uint32_t E5B;
};

// global variables for use in interrupts
motors Motors;
timers Timers;
encoders Encoders;

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

std_msgs::UInt16 pushedVal;
ros::Publisher encoderPub("enc_pub", &pushedVal);


uint32_t pulseCount = 0;
void channelACallback(void){
  pulseCount++;

}


void channel0Callback(void){pulseCount++;}
void channel1Callback(void){pulseCount++;}
void channel2Callback(void){pulseCount++;}
void channel3Callback(void){pulseCount++;}
void channel4Callback(void){pulseCount++;}
void channel5Callback(void){pulseCount++;}

void channel0BCallback(void){pulseCount++;}
void channel2BCallback(void){pulseCount++;}
void channel3BCallback(void){pulseCount++;}
void channel5BCallback(void){pulseCount++;}

void rolloverCallback(void){
  pushedVal.data = pulseCount;
  encoderPub.publish(&pushedVal);
  pulseCount = 0;
  delay(1);
}


void configureHardwareTimers();
void setup()
{
    //pinF1_DisconnectDebug(PB_3);
    //pinF1_DisconnectDebug(PA_14);
    __HAL_RCC_AFIO_CLK_ENABLE();
    //__HAL_AFIO_REMAP_SWJ_NOJTAG();
    //__HAL_AFIO_REMAP_USART1_DISABLE();
    //__HAL_AFIO_REMAP_SWJ_DISABLE(); 
    //__HAL_AFIO_REMAP_SPI1_DISABLE();
 
    //__HAL_AFIO_REMAP_TIM2_PARTIAL_2();  
    configureHardwareTimers();
    //pin_SetF1AFPin(AFIO_SWJ_NOJTAG);
    nh.initNode();
    nh.subscribe(velocity);
    nh.advertise(encoderPub);
    Serial.end();
}

void loop()
{
    // required for rosserial
    nh.spinOnce();
    //controlMotors(0, 0);
    if (pulseCount > 0)
    {
      rolloverCallback();
    }
    delay(1);
}

// "map" is already defined in the arduino standard library
uint16_t map_(const double val, const double in_min, const double in_max, const double out_min, const double out_max)
{
    // mitigate overflows by capping output values if input is out of range
    if (val >= in_max)
    {
        return (uint16_t)out_max;
    }
    if (val <= in_min)
    {
        return (uint16_t)out_min;
    }
    return (uint16_t)round((val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}

uint16_t velToPulse(const double vel, const bool reverse = false)
{
    const double in_min = -4.0;
    const double in_max = 4.0;
    if (reverse)
    {
        // front motors face the opposite direction of the other motors
        // so they must spin in the opposite direction
        return map_(vel, in_min, in_max, 1000.0, 2000.0);
    }
    return map_(vel, in_min, in_max, 2000.0, 1000.0);
}

void controlMotors(const geometry_msgs::Twist &cmd_vel)
{
    const double left_speed = cmd_vel.linear.x + cmd_vel.angular.z;
    const double right_speed = cmd_vel.linear.x - cmd_vel.angular.z;



    Timers.Tim2->setCaptureCompare(Motors.P0, velToPulse(left_speed), MICROSEC_COMPARE_FORMAT);
    Timers.Tim2->setCaptureCompare(Motors.P1, velToPulse(left_speed), MICROSEC_COMPARE_FORMAT);
    Timers.Tim1->setCaptureCompare(Motors.P2, velToPulse(left_speed, true), MICROSEC_COMPARE_FORMAT);
    Timers.Tim1->setCaptureCompare(Motors.P3, velToPulse(right_speed), MICROSEC_COMPARE_FORMAT);
    Timers.Tim1->setCaptureCompare(Motors.P4, velToPulse(right_speed), MICROSEC_COMPARE_FORMAT);
    Timers.Tim1->setCaptureCompare(Motors.P5, velToPulse(right_speed, true), MICROSEC_COMPARE_FORMAT);
}

void configureHardwareTimers()
{
    // Automatically retrieve timer instance and channel associated to pin
    // This is used to be compatible with all STM32 series automatically.
    TIM_TypeDef *Instance1 = (TIM_TypeDef *)pinmap_peripheral(P3_, PinMap_PWM);
    TIM_TypeDef *Instance2 = (TIM_TypeDef *)pinmap_peripheral(P1_, PinMap_PWM);
    TIM_TypeDef *Instance3 = (TIM_TypeDef *)pinmap_peripheral(E0A_, PinMap_PWM);
    TIM_TypeDef *Instance4 = (TIM_TypeDef *)pinmap_peripheral(E3A_, PinMap_PWM);

    Motors.P0 = STM_PIN_CHANNEL(pinmap_function(P0_, PinMap_PWM));
    Motors.P1 = STM_PIN_CHANNEL(pinmap_function(P1_, PinMap_PWM));
    Motors.P2 = STM_PIN_CHANNEL(pinmap_function(P2_, PinMap_PWM));
    Motors.P3 = STM_PIN_CHANNEL(pinmap_function(P3_, PinMap_PWM));
    Motors.P4 = STM_PIN_CHANNEL(pinmap_function(P4_, PinMap_PWM));
    Motors.P5 = STM_PIN_CHANNEL(pinmap_function(P5_, PinMap_PWM));

    Encoders.E0A = STM_PIN_CHANNEL(pinmap_function(E0A_, PinMap_PWM));
    Encoders.E0B = STM_PIN_CHANNEL(pinmap_function(E0B_, PinMap_PWM));

    Encoders.E1A = STM_PIN_CHANNEL(pinmap_function(E1A_, PinMap_PWM));

    Encoders.E2A = STM_PIN_CHANNEL(pinmap_function(E2A_, PinMap_PWM));
    Encoders.E2B = STM_PIN_CHANNEL(pinmap_function(E2B_, PinMap_PWM));

    Encoders.E3A = STM_PIN_CHANNEL(pinmap_function(E3A_, PinMap_PWM));
    Encoders.E3B = STM_PIN_CHANNEL(pinmap_function(E3B_, PinMap_PWM));
    
    Encoders.E4A = STM_PIN_CHANNEL(pinmap_function(E4A_, PinMap_PWM));
    
    Encoders.E5A = STM_PIN_CHANNEL(pinmap_function(E5A_, PinMap_PWM));
    Encoders.E5B = STM_PIN_CHANNEL(pinmap_function(E5B_, PinMap_PWM));


    // Instantiate HardwareTimer object. Thanks to 'new' instantiation, HardwareTimer is not destructed when setup() function is finished.
    Timers.Tim1 = new HardwareTimer(Instance1);
    Timers.Tim2 = new HardwareTimer(Instance2);
    Timers.Tim3 = new HardwareTimer(Instance3);
    Timers.Tim4 = new HardwareTimer(Instance4);


    // Configure and start PWM
    Timers.Tim2->setMode(Motors.P0, TIMER_OUTPUT_COMPARE_PWM1, P0_);
    Timers.Tim2->setMode(Motors.P1, TIMER_OUTPUT_COMPARE_PWM1, P1_);
    Timers.Tim1->setMode(Motors.P2, TIMER_OUTPUT_COMPARE_PWM1, P2_);
    Timers.Tim1->setMode(Motors.P3, TIMER_OUTPUT_COMPARE_PWM1, P3_);
    Timers.Tim1->setMode(Motors.P4, TIMER_OUTPUT_COMPARE_PWM1, P4_);
    Timers.Tim1->setMode(Motors.P5, TIMER_OUTPUT_COMPARE_PWM1, P5_);

    // complete signal period set to 3us
    const uint16_t signalPeriod = 3000;
    Timers.Tim1->setOverflow(signalPeriod, MICROSEC_FORMAT);
    Timers.Tim2->setOverflow(signalPeriod, MICROSEC_FORMAT);

    Timers.Tim3->setOverflow(signalPeriod*10, MICROSEC_FORMAT);
    Timers.Tim4->setOverflow(signalPeriod*10, MICROSEC_FORMAT);

    // set initial pulse of 1500us (stopped)
    const uint16_t initialPulse = 1500;
    Timers.Tim2->setCaptureCompare(Motors.P0, initialPulse, MICROSEC_COMPARE_FORMAT);
    Timers.Tim2->setCaptureCompare(Motors.P1, initialPulse, MICROSEC_COMPARE_FORMAT);
    Timers.Tim1->setCaptureCompare(Motors.P2, initialPulse, MICROSEC_COMPARE_FORMAT);
    Timers.Tim1->setCaptureCompare(Motors.P3, initialPulse, MICROSEC_COMPARE_FORMAT);
    Timers.Tim1->setCaptureCompare(Motors.P4, initialPulse, MICROSEC_COMPARE_FORMAT);
    Timers.Tim1->setCaptureCompare(Motors.P5, initialPulse, MICROSEC_COMPARE_FORMAT);

    Timers.Tim2->setMode(Encoders.E4A, TIMER_INPUT_CAPTURE_RISING, E4A_);
    Timers.Tim2->setMode(Encoders.E1A, TIMER_INPUT_CAPTURE_RISING, E1A_);
    Timers.Tim3->setMode(Encoders.E0A, TIMER_INPUT_CAPTURE_RISING, E0A_);
    Timers.Tim3->setMode(Encoders.E2A, TIMER_INPUT_CAPTURE_RISING, E2A_);
    Timers.Tim4->setMode(Encoders.E3A, TIMER_INPUT_CAPTURE_RISING, E3A_);
    Timers.Tim4->setMode(Encoders.E5A, TIMER_INPUT_CAPTURE_RISING, E5A_);

 
    Timers.Tim3->setMode(Encoders.E0B, TIMER_INPUT_CAPTURE_RISING, E0B_);
    Timers.Tim3->setMode(Encoders.E2B, TIMER_INPUT_CAPTURE_RISING, E2B_);
    Timers.Tim4->setMode(Encoders.E3B, TIMER_INPUT_CAPTURE_RISING, E3B_);
    Timers.Tim4->setMode(Encoders.E5B, TIMER_INPUT_CAPTURE_RISING, E5B_);

    
    //Timers.Tim3->attachInterrupt(rolloverCallback);
    
    Timers.Tim2->attachInterrupt(Encoders.E4A, channel4Callback);
    Timers.Tim2->attachInterrupt(Encoders.E1A, channel1Callback);

    Timers.Tim3->attachInterrupt(Encoders.E0A, channel0Callback);
    Timers.Tim3->attachInterrupt(Encoders.E2A, channel2Callback);

    Timers.Tim4->attachInterrupt(Encoders.E3A, channel3Callback);
    Timers.Tim4->attachInterrupt(Encoders.E5A, channel5Callback);

    Timers.Tim3->attachInterrupt(Encoders.E0B, channel0BCallback);
    Timers.Tim3->attachInterrupt(Encoders.E2B, channel2BCallback);

    Timers.Tim4->attachInterrupt(Encoders.E3B, channel3BCallback);
    Timers.Tim4->attachInterrupt(Encoders.E5B, channel5BCallback);
        

    
    // enable timers
    Timers.Tim1->resume();
    Timers.Tim2->resume();
    Timers.Tim3->resume();
    Timers.Tim4->resume();
}
