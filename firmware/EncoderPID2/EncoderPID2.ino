#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <HardwareSerial.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include "PID_v1.h"

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

#define E4A_ PA_1 // PB_3

#define E5A_ PB_8
#define E5B_ PB_9

#define TIM_1_CH_1 P5_
#define TIM_2_CH_1 P1_
#define TIM_3_CH_1 E0A_
#define TIM_4_CH_1 E3A_

int16_t MIN_PULSE = 1000;
int16_t MAX_PULSE = 2000;

int32_t REFRESH_RATE = 50000;
int32_t TIMERS_RATE = 3000;

double factor[6] = {391.46, 388.37, 380.43, -376.12, -393.07, -384.64};
double offset[6] = {1488.6, 1502.7, 1497.1, 1499.2, 1501.5, 1502.3};

#define USING_ROS
#define VELOCITY_DEBUG
//#define DISTANCE_DEBUG

#ifdef USING_ROS
HardwareSerial hserial(SERIAL_RX, SERIAL_TX);

class NewHardware : public ArduinoHardware
{
public:
    NewHardware() : ArduinoHardware(&hserial, BAUD_RATE){};
};
ros::NodeHandle_<NewHardware> nh;

// subscribe to cmd_vel Twist topic
void controlMotorsRos(const geometry_msgs::Twist &cmd_vel);
void setParamsRos(const std_msgs::Float64MultiArray &cmd_pid);
void controlMotors(double left_speed, double right_speed);

ros::Subscriber<geometry_msgs::Twist> velocity("cmd_vel", &controlMotorsRos);
ros::Subscriber<std_msgs::Float64MultiArray> pid_params("cmd_pid", &setParamsRos);
// std_msgs::String pushedVal;
std_msgs::Float64 pushedVal;
// std_msgs::UInt16 pushedVal;
ros::Publisher encoderPub("enc_pub", &pushedVal);

#endif /* USING_ROS */

typedef struct encoder
{
    HardwareTimer *timer;
    uint32_t channel;
    PinName pin;
    volatile int16_t pulseCount;
    volatile double PIDvelocity;
} Encoder;

typedef struct Motor
{
    HardwareTimer *timer;
    uint32_t channel;
    PinName pin;
    volatile uint16_t pulseDuration;
    volatile double setpoint;
} Motor;

HardwareTimer *tim1, *tim2, *tim3, *tim4;
Encoder E0A, E0B, E1A, E2A, E2B, E3A, E3B, E4A, E5A, E5B;
Motor P0, P1, P2, P3, P4, P5;

Encoder *encoders[] = {&E0A, &E1A, &E2A, &E3A, &E4A, &E5A};
Motor *motors[] = {&P0, &P1, &P2, &P3, &P4, &P5};

PID feedback4(&(E4A.PIDvelocity), &(P4.pulseDuration), &(P4.setpoint), 0, 0, 0, P_ON_E, DIRECT);

void initializeTimer(HardwareTimer **tim, PinName pin, uint32_t overflow, void (*interrupt)(void))
{
    TIM_TypeDef *Instance = (TIM_TypeDef *)pinmap_peripheral(pin, PinMap_PWM);
    *tim = new HardwareTimer(Instance);

    (*tim)->setOverflow(overflow, MICROSEC_FORMAT);

    if (interrupt != NULL)
    {
        (*tim)->attachInterrupt(interrupt);
    }
}

void initializeEncoder(Encoder *encoder, HardwareTimer **tim, PinName pin, void (*interrupt)(void))
{
    encoder->timer = *tim;
    encoder->channel = STM_PIN_CHANNEL(pinmap_function(pin, PinMap_PWM));
    encoder->pin = pin;
    encoder->pulseCount = 0;
    encoder->PIDvelocity = 0;

    (*tim)->setMode(encoder->channel, TIMER_INPUT_CAPTURE_RISING, pin);

    if (interrupt != NULL)
    {
        (*tim)->attachInterrupt(encoder->channel, interrupt);
    }
}

void initializeMotor(Motor *motor, HardwareTimer **tim, PinName pin)
{
    motor->timer = *tim;
    motor->channel = STM_PIN_CHANNEL(pinmap_function(pin, PinMap_PWM));
    motor->pin = pin;
    motor->pulseDuration = 1500;
    motor->setpoint = 0.0;

    (motor->timer)->setMode(motor->channel, TIMER_OUTPUT_COMPARE_PWM1, pin);
    (motor->timer)->setCaptureCompare(motor->channel, motor->pulseDuration, MICROSEC_COMPARE_FORMAT);
    // add pid func here?
}

void setMotorSpeed(Motor *motor, uint16_t newPulseDuration)
{
    // char intbuf[20];
    // sprintf(intbuf, "%d", newPulseDuration);
    // nh.loginfo(intbuf);
    if (newPulseDuration >= MIN_PULSE && newPulseDuration <= MAX_PULSE)
    {
        // motor->PIDduration = newPulseDuration;
        motor->pulseDuration = (volatile uint16_t) newPulseDuration;
        feedback4.UpdateOutput();
        (motor->timer)->setCaptureCompare(motor->channel, newPulseDuration, MICROSEC_COMPARE_FORMAT);
    }
}

void updateMotorSpeed(Motor *motor)
{
    (motor->timer)->setCaptureCompare(motor->channel, motor->pulseDuration, MICROSEC_COMPARE_FORMAT);
}

void setSetpoint(Motor *motor, volatile double newSetpoint)
{
    motor->setpoint = newSetpoint;
}

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

uint16_t velocityToPulse(const double vel, const bool reverse = false)
{
    const double control_min = -4.0;
    const double control_max = 4.0;

    if (reverse)
    {
        return map_(vel, control_min, control_max, MIN_PULSE, MAX_PULSE);
    }
    return map_(vel, control_min, control_max, MAX_PULSE, MIN_PULSE);
}

uint16_t metersPerSecondToPulse(const double vel, const int encoderIndex)
{
    return (uint16_t)(vel * factor[encoderIndex] + offset[encoderIndex]);
}

void encoder4Callback(void)
{
    if (GPIOA->IDR & GPIO_IDR_IDR5)
    {
        // if pin B is high, then pinB went first
        (E4A.pulseCount)--;
    }
    else
    {
        (E4A.pulseCount)++;
    }
}

void rolloverCallback(void)
{
    E4A.PIDvelocity = ((volatile double)E4A.pulseCount / 1024.0) * (1000000.0 / (volatile double)REFRESH_RATE) * (0.20 * 3.141592653); //Rotations per Second * Pi * Diameter
    E4A.pulseCount = 0;
}

void initializeAllPins()
{
    initializeTimer(&tim1, TIM_1_CH_1, TIMERS_RATE, NULL);
    initializeTimer(&tim2, TIM_2_CH_1, TIMERS_RATE, NULL);
    initializeTimer(&tim3, TIM_3_CH_1, REFRESH_RATE, rolloverCallback);
    // initializeTimer(&tim4, TIM_4_CH_1, REFRESH_RATE, NULL);

    initializeEncoder(&E4A, &tim2, E4A_, encoder4Callback);

    initializeMotor(&P4, &tim1, P4_);

    tim1->resume();
    tim2->resume();
    tim3->resume();
    // tim4->resume();
}

void setParamsRos(const std_msgs::Float64MultiArray &cmd_pid)
{
    uint8_t feedback_to_change = (uint8_t)cmd_pid.data[0];
    double kP = cmd_pid.data[1];
    double kI = cmd_pid.data[2];
    double kD = cmd_pid.data[3];

    nh.loginfo("\n");
    nh.loginfo("--ISR--");
    char log_msg[13];
    char result[8]; // Buffer big enough for 7-character float

    // P
    dtostrf(kP, 6, 2, result); // Leave room for too large numbers!
    sprintf(log_msg, "kP=%s", result);
    nh.loginfo(log_msg);
    // I
    dtostrf(kI, 6, 2, result); // Leave room for too large numbers!
    sprintf(log_msg, "kI=%s", result);
    nh.loginfo(log_msg);
    // D
    dtostrf(kD, 6, 2, result); // Leave room for too large numbers!
    sprintf(log_msg, "kD=%s", result);
    nh.loginfo(log_msg);
    nh.loginfo("-------\n");

    if (feedback_to_change == 4)
    {
        feedback4.SetTunings(kP, kI, kD);
    }
}

void controlMotorsRos(const geometry_msgs::Twist &cmd_vel)
{
    double left_speed = cmd_vel.linear.x + cmd_vel.angular.z;
    double right_speed = cmd_vel.linear.x - cmd_vel.angular.z;

    controlMotors(left_speed, right_speed);
}

void controlMotors(double left_speed, double right_speed)
{
    setMotorSpeed(&P4, metersPerSecondToPulse(right_speed, 1));
    setSetpoint(&P4, right_speed);
}

void setup()
{
    initializeAllPins();

#ifdef USING_ROS
    nh.initNode();
    nh.subscribe(velocity);
    nh.subscribe(pid_params);
    nh.advertise(encoderPub);
    Serial.end();
#else
    Serial.begin();
#endif

    controlMotors(0, 0);
    pinMode(PA1, INPUT);
    pinMode(PA2, INPUT);
    pinMode(PA4, INPUT);
    pinMode(PA5, INPUT);
    pinMode(PA7, INPUT);
    pinMode(PA6, INPUT);
    pinMode(PB0, INPUT);
    pinMode(PB1, INPUT);
    pinMode(PB5, INPUT);
    pinMode(PB6, INPUT);
    pinMode(PB7, INPUT);
    pinMode(PB8, INPUT);
    pinMode(PB9, INPUT);

    feedback4.SetMode(AUTOMATIC);

    feedback4.SetOutputLimits(1500, MAX_PULSE);
}

int ctr = 0;
double kP_set, kI_set, kD_set;
void loop()
{
#ifdef USING_ROS
    nh.spinOnce();

    pushedVal.data = E4A.PIDvelocity;
    encoderPub.publish(&pushedVal);

    double computed[4];
    bool changed = feedback4.Compute(computed);
    if (changed)
    {
        updateMotorSpeed(&P4);
    }

    if ((ctr++) % 50000000 == 0)
    {
        ctr = 0;

        // char log_msg[50];
        // sprintf(log_msg, "b:%d a:%d", before, after);
        // nh.loginfo(log_msg);
        // pushedVal.data = E4A.PIDvelocity;
        // encoderPub.publish(&pushedVal);

        // char buffer[50];
        // char PIDvelocity[8];
        // char PIDduration[8];
        // char setpoint[8];
        // dtostrf(E4A.PIDvelocity, 2, 6, PIDvelocity);
        // dtostrf(P4.PIDduration, 2, 6, PIDduration);
        // dtostrf(P4.setpoint, 2, 6, setpoint);
        // // sprintf(buffer, "PIDvelocity:%s PIDduration:%s setpoint:%s", PIDvelocity, PIDduration, setpoint);
        // sprintf(buffer, "%s,%s,%s", PIDvelocity, PIDduration, setpoint);
        // pushedVal.data = buffer;
        // encoderPub.publish(&pushedVal);

        if (kP_set != feedback4.GetKp())
        {
            nh.loginfo("--loop--");
            kP_set = feedback4.GetKp();
            char log_msg[14];
            char result[8];                // Buffer big enough for 7-character float
            dtostrf(kP_set, 6, 2, result); // Leave room for too large numbers!
            sprintf(log_msg, "kP=%s", result);
            nh.loginfo(log_msg);
            nh.loginfo("--------");
        }

        if (kI_set != feedback4.GetKi())
        {
            nh.loginfo("--loop--");
            kI_set = feedback4.GetKi();
            char log_msg[14];
            char result[8];                // Buffer big enough for 7-character float
            dtostrf(kI_set, 6, 2, result); // Leave room for too large numbers!
            sprintf(log_msg, "kI=%s", result);
            nh.loginfo(log_msg);
            nh.loginfo("--------");
        }

        if (kD_set != feedback4.GetKd())
        {
            nh.loginfo("--loop--");
            kD_set = feedback4.GetKd();
            char log_msg[14];
            char result[8];                // Buffer big enough for 7-character float
            dtostrf(kD_set, 6, 2, result); // Leave room for too large numbers!
            sprintf(log_msg, "kD=%s", result);
            nh.loginfo(log_msg);
            nh.loginfo("--------");
        }
    }
#else
    /*
            for (int i = 0; i < 6; i++)
            {
                char debugLog[100];
    #ifdef VELOCITY_DEBUG
                //sprintf(debugLog, "Encoder %d has velocity %d at a pulse of %d", i, encoders[i]->velocity, map_(debugSpeed, -4, 4, MAX_PULSE, MIN_PULSE));

                sprintf(debugLog, "Encoder %d has velocity %d.%d, pulse %d, setpoint %d", i, (int)(encoders[i]->PIDvelocity*1000)/1000, (int)(encoders[i]->PIDvelocity * 1000)%1000, (int)(motors[i]->PIDduration), (int)motors[i]->setpoint);
    #endif

                Serial.println(debugLog);
    #ifdef USING_ROS
                nh.logdebug(debugLog);
    #endif
            }
            Serial.flush();
            */

#endif

    // char log_msg[50];
    // char input[8];                // Buffer big enough for 7-character float
    // char error[8];                // Buffer big enough for 7-character float
    // char dInput[8];                // Buffer big enough for 7-character float
    // char output[8];
    // dtostrf(computed[0], 6, 2, input); // Leave room for too large numbers!
    // dtostrf(computed[1], 6, 2, error); // Leave room for too large numbers!
    // dtostrf(computed[2], 6, 2, dInput); // Leave room for too large numbers!
    // dtostrf(computed[3], 6, 2, output); // Leave room for too large numbers!
    // sprintf(log_msg, "i:%s e:%s dI:%s o:%s", input, error, dInput, output);
    // nh.loginfo(log_msg);

    delay(1);
}
