#include <ros.h>
#include <nav_msgs/Odometry.h>
#include <HardwareSerial.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include "PID_v1.h"

#define USE_STM32_HW_SERIAL
#define BAUD_RATE 128000
#define SERIAL_RX PB11 // pins being used for ROSSerial
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

// map timer channels to stm32 pins
#define TIM_1_CH_1 P5_
#define TIM_2_CH_1 P1_
#define TIM_3_CH_1 E0A_
#define TIM_4_CH_1 E3A_

#define CMD_VEL_RATE_MS 40

#define MIN_PULSE (uint16_t)1000 // PWM Pulse Duration (microseconds)
#define MAX_PULSE (uint16_t)2000

#define REFRESH_RATE (uint32_t)50000 // Value (microseconds) at which encoder timers overflow, defines how often encoder velocity is calculated
#define TIMERS_RATE (uint32_t)3000   // Value (microseconds) at which motor timers overflow

#define TRACK_WIDTH 0.6

/*
 * Chart used from linear calibration of motor power - pulse curve
 * PWM Pulse = factor * desired motor speed + offset (y = mx + b)
 * where motor speed is in m/s, PWM pulse is a value between MIN and MAX
 * This gives us a good 'first guess' to get the motor near the desired speed.
 */
double factor[6] = {376.12, 393.07, 384.64, 380.43, 388.37, 391.46};
double offset[6] = {1499.2, 1501.5, 1502.3, 1497.1, 1502.7, 1488.6};

// covariance matrix diagonals
// x, y, z, rot x, rot y, rot z
double mat[6] = {0.01, 0.01, 1000, 1000, 1000, 0.01};
double x_pos = 0, y_pos = 0, heading = 0;
double vel_timer = 0.0;
double time_prev = 0, time_cur = 0, loop_delta_time;

#define USING_ROS
// #define VELOCITY_DEBUG
// #define DISTANCE_DEBUG

#ifdef USING_ROS
HardwareSerial hserial(SERIAL_RX, SERIAL_TX);

class NewHardware : public ArduinoHardware
{
public:
    NewHardware() : ArduinoHardware(&hserial, BAUD_RATE){};
};

// buffer size increased to 1024 bytes to publish covariance matrices
ros::NodeHandle_<NewHardware, 25, 25, 1024, 1024> nh;

// subscribe to cmd_vel Twist topic
void controlMotorsRos(const geometry_msgs::Twist &cmd_vel);
void setParamsRos(const std_msgs::Float64MultiArray &cmd_pid);
void controlMotors(double left_speed, double right_speed);
void updateOdometry(nav_msgs::Odometry *robot_odom, double lts, double lms, double lbs, double rts, double rms, double rbs, double dt);

ros::Subscriber<geometry_msgs::Twist> velocity("cmd_vel", &controlMotorsRos);
ros::Subscriber<std_msgs::Float64MultiArray> pid_params("cmd_pid", &setParamsRos);

// variables for publishing encoder values over ros for debugging
std_msgs::Float64 encoderVal0;
std_msgs::Float64 encoderVal1;
std_msgs::Float64 encoderVal2;
std_msgs::Float64 encoderVal3;
std_msgs::Float64 encoderVal4;
std_msgs::Float64 encoderVal5;
nav_msgs::Odometry odom;

ros::Publisher encoderPub0("enc_pub0", &encoderVal0);
ros::Publisher encoderPub1("enc_pub1", &encoderVal1);
ros::Publisher encoderPub2("enc_pub2", &encoderVal2);
ros::Publisher encoderPub3("enc_pub3", &encoderVal3);
ros::Publisher encoderPub4("enc_pub4", &encoderVal4);
ros::Publisher encoderPub5("enc_pub5", &encoderVal5);

ros::Publisher robotPub("robot_odom", &odom);

#endif /* USING_ROS */

typedef struct encoder
{
    HardwareTimer *timer;
    uint32_t channel;
    PinName pin;
    volatile int16_t pulseCount; // number of encoder pulses detected
    volatile double PIDvelocity; // current encoder velocity, output of PID loop
} Encoder;

typedef struct Motor
{
    HardwareTimer *timer;
    uint32_t channel;
    PinName pin;
    volatile uint16_t pulseDuration; // PWM pulse duration of motor
    volatile double setpoint;        // desired motor velocity

} Motor;

typedef struct PID_Values
{
    double P;
    double I;
    double D;
} PID_Values;

HardwareTimer *tim1, *tim2, *tim3, *tim4;
Encoder E0A, E0B, E1A, E2A, E2B, E3A, E3B, E4A, E5A, E5B;
Motor P0, P1, P2, P3, P4, P5;

// A encoders are initialized with timers - B encoders are merely used to identify leading/lagging
Encoder encoders[] = {E0A, E1A, E2A, E3A, E4A, E5A};
Motor *motors[] = {&P0, &P1, &P2, &P3, &P4, &P5};

PID_Values pid0 = {.P=270, .I=0.25, .D=0.0625};
PID_Values pid1 = {.P=255, .I=0.2, .D=0.0625};
PID_Values pid2 = {.P=246, .I=0.26, .D=0.065};   
PID_Values pid3 = {.P=240, .I=0.25, .D=0.0625};
PID_Values pid4 = {.P=252, .I=0.3, .D=0.075};
PID_Values pid5 = {.P=249, .I=0.275, .D=0.06875};

// PID struct with input variable, output variable, and desired variabe
PID feedback0(&(E0A.PIDvelocity), &(P0.pulseDuration), &(P0.setpoint), pid0.P, pid0.I, pid0.D, P_ON_E, DIRECT);
PID feedback1(&(E1A.PIDvelocity), &(P1.pulseDuration), &(P1.setpoint), pid1.P, pid1.I, pid1.D, P_ON_E, DIRECT);
PID feedback2(&(E2A.PIDvelocity), &(P2.pulseDuration), &(P2.setpoint), pid2.P, pid2.I, pid2.D, P_ON_E, DIRECT);
PID feedback3(&(E3A.PIDvelocity), &(P3.pulseDuration), &(P3.setpoint), pid3.P, pid3.I, pid3.D, P_ON_E, DIRECT);
PID feedback4(&(E4A.PIDvelocity), &(P4.pulseDuration), &(P4.setpoint), pid4.P, pid4.I, pid4.D, P_ON_E, DIRECT);
PID feedback5(&(E5A.PIDvelocity), &(P5.pulseDuration), &(P5.setpoint), pid5.P, pid5.I, pid5.D, P_ON_E, DIRECT);

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

// Manually set motor to desired speed
void setMotorSpeed(Motor *motor, uint16_t newPulseDuration, uint8_t feedbackIndex)
{
    // char intbuf[20];
    // sprintf(intbuf, "%d", newPulseDuration);
    // nh.loginfo(intbuf);
    if (newPulseDuration >= MIN_PULSE && newPulseDuration <= MAX_PULSE)
    {
        // motor->PIDduration = newPulseDuration;
        motor->pulseDuration = (volatile uint16_t)newPulseDuration;
        (motor->timer)->setCaptureCompare(motor->channel, newPulseDuration, MICROSEC_COMPARE_FORMAT);
    }
}

// Set motor speed based off value in motor struct
void updateMotorSpeed(Motor *motor)
{
    (motor->timer)->setCaptureCompare(motor->channel, motor->pulseDuration, MICROSEC_COMPARE_FORMAT);
}

// Set new desired velocity in motor struct
void setSetpoint(Motor *motor, volatile double newSetpoint)
{
    motor->setpoint = newSetpoint;
}

// use calibration graphs to convert meters per second to desired PWM pulse for specific encoder
uint16_t velocityToPulse(const double vel, const int encoderIndex, const bool reverse = false)
{
    // invert velocity to flip wheel direction
    // return (uint16_t)(vel * factor[encoderIndex] + offset[encoderIndex]);
    if (reverse)
    {
        return (uint16_t)(-vel * factor[encoderIndex] + offset[encoderIndex]);
    }
    return (uint16_t)(vel * factor[encoderIndex] + offset[encoderIndex]);
}

/*
 * CALLBACK FUNCTIONS
 * Called when the pin tied to the encoder A channel goes high
 * If both A channel and B channel are high, encoder must be going in negative direction (as B lags A)
 * Update encoder struct pulse count
 */

void encoder0Callback(void)
{
    if ((GPIOA->IDR & GPIO_IDR_IDR6) && (GPIOA->IDR & GPIO_IDR_IDR7))
    {
        // if pin B is high, then pinB went first
        (E0A.pulseCount)--;
    }
    else
    {
        (E0A.pulseCount)++;
    }
}

void encoder1Callback(void)
{
    if ((GPIOA->IDR & GPIO_IDR_IDR2) && (GPIOA->IDR & GPIO_IDR_IDR4))
    {
        // if pin B is high, then pinB went first
        (E1A.pulseCount)--;
    }
    else
    {
        (E1A.pulseCount)++;
    }
}

void encoder2Callback(void)
{
    if ((GPIOB->IDR & GPIO_IDR_IDR0) && (GPIOB->IDR & GPIO_IDR_IDR1))
    {
        // if pin B is high, then pinB went first
        (E2A.pulseCount)--;
    }
    else
    {
        (E2A.pulseCount)++;
    }
}
void encoder3Callback(void)
{
    if ((GPIOB->IDR & GPIO_IDR_IDR6) && (GPIOB->IDR & GPIO_IDR_IDR7))
    {
        // if pin B is high, then pinB went first
        (E3A.pulseCount)--;
    }
    else
    {
        (E3A.pulseCount)++;
    }
}

void encoder4Callback(void)
{
    if ((GPIOA->IDR & GPIO_IDR_IDR5) && (GPIOA->IDR & GPIO_IDR_IDR1))
    {
        // if pin B is high, then pinB went first
        (E4A.pulseCount)--;
    }
    else
    {
        (E4A.pulseCount)++;
    }
}

void encoder5Callback(void)
{
    if ((GPIOB->IDR & GPIO_IDR_IDR8) && (GPIOB->IDR & GPIO_IDR_IDR9))
    {
        // if pin B is high, then pinB went first
        (E5A.pulseCount)--;
    }
    else
    {
        (E5A.pulseCount)++;
    }
}

// Calculate encoder velocity and reset encoder pulse counts
void rolloverCallback(void)
{

    E0A.PIDvelocity = ((volatile double)E0A.pulseCount / 1024.0) * (1000000.0 / (volatile double)REFRESH_RATE) * (0.20 * 3.141592653); // Rotations per Second * Pi * Diameter
    E0A.pulseCount = 0;

    E1A.PIDvelocity = ((volatile double)E1A.pulseCount / 1024.0) * (1000000.0 / (volatile double)REFRESH_RATE) * (0.20 * 3.141592653); // Rotations per Second * Pi * Diameter
    E1A.pulseCount = 0;

    E2A.PIDvelocity = ((volatile double)E2A.pulseCount / 1024.0) * (1000000.0 / (volatile double)REFRESH_RATE) * (0.20 * 3.141592653); // Rotations per Second * Pi * Diameter
    E2A.pulseCount = 0;

    E3A.PIDvelocity = ((volatile double)E3A.pulseCount / 1024.0) * (1000000.0 / (volatile double)REFRESH_RATE) * (0.20 * 3.141592653); // Rotations per Second * Pi * Diameter
    E3A.pulseCount = 0;

    E4A.PIDvelocity = ((volatile double)E4A.pulseCount / 1024.0) * (1000000.0 / (volatile double)REFRESH_RATE) * (0.20 * 3.141592653); // Rotations per Second * Pi * Diameter
    E4A.pulseCount = 0;

    E5A.PIDvelocity = ((volatile double)E5A.pulseCount / 1024.0) * (1000000.0 / (volatile double)REFRESH_RATE) * (0.20 * 3.141592653); // Rotations per Second * Pi * Diameter
    E5A.pulseCount = 0;

    /*
    for (int i = 0; i < 6; i ++)
    {
        encoders[i].PIDvelocity = ((volatile double) encoders[i].pulseCount / 1024.0) * (1000000.0 / (volatile double)REFRESH_RATE) * (0.20 * 3.141592653); //Rotations per Second * Pi * Diameter
        encoders[i].pulseCount = 0;
    }
    */
}

void initializeAllPins()
{
    initializeTimer(&tim1, TIM_1_CH_1, TIMERS_RATE, NULL);
    initializeTimer(&tim2, TIM_2_CH_1, TIMERS_RATE, NULL);
    initializeTimer(&tim3, TIM_3_CH_1, REFRESH_RATE, rolloverCallback);
    initializeTimer(&tim4, TIM_4_CH_1, REFRESH_RATE, NULL);

    initializeEncoder(&E0A, &tim3, E0A_, encoder0Callback);
    initializeEncoder(&E0B, &tim3, E0B_, NULL);
    initializeEncoder(&E1A, &tim2, E1A_, encoder1Callback);
    initializeEncoder(&E2A, &tim3, E2A_, encoder2Callback);
    initializeEncoder(&E2B, &tim3, E2B_, NULL);

    initializeEncoder(&E3A, &tim4, E3A_, encoder3Callback);
    initializeEncoder(&E3B, &tim4, E3B_, NULL);
    initializeEncoder(&E4A, &tim2, E4A_, encoder4Callback);
    initializeEncoder(&E5A, &tim4, E5A_, encoder5Callback);
    initializeEncoder(&E5B, &tim4, E5B_, NULL);

    initializeMotor(&P0, &tim2, P0_);
    initializeMotor(&P1, &tim2, P1_);
    initializeMotor(&P2, &tim1, P2_);
    initializeMotor(&P3, &tim1, P3_);
    initializeMotor(&P4, &tim1, P4_);
    initializeMotor(&P5, &tim1, P5_);

    tim1->resume();
    tim2->resume();
    tim3->resume();
    tim4->resume();
}

// Receive message from ros serial to update PID values used
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

    if (feedback_to_change == 0)
    {
        feedback0.SetTunings(kP, kI, kD);
    }
    else if (feedback_to_change == 1)
    {
        feedback1.SetTunings(kP, kI, kD);
    }
    else if (feedback_to_change == 2)
    {
        feedback2.SetTunings(kP, kI, kD);
    }
    else if (feedback_to_change == 3)
    {
        feedback3.SetTunings(kP, kI, kD);
    }
    else if (feedback_to_change == 4)
    {
        feedback4.SetTunings(kP, kI, kD);
    }
    else if (feedback_to_change == 5)
    {
        feedback5.SetTunings(kP, kI, kD);
    }
    else if (feedback_to_change == 6)
    {
        feedback0.SetTunings(kP, kI, kD);
        feedback1.SetTunings(kP, kI, kD);
        feedback2.SetTunings(kP, kI, kD);
        feedback3.SetTunings(kP, kI, kD);
        feedback4.SetTunings(kP, kI, kD);
        feedback5.SetTunings(kP, kI, kD);
    }
}

void controlMotorsRos(const geometry_msgs::Twist &cmd_vel)
{
    double left_speed = cmd_vel.linear.x + cmd_vel.angular.z;
    double right_speed = cmd_vel.linear.x - cmd_vel.angular.z;

    vel_timer = 0.0;
    controlMotors(left_speed, right_speed);
}

// take min speed of left and right wheels and update odometry
void updateOdometry(nav_msgs::Odometry *robot_odom, double lts, double lms, double lbs, double rts, double rms, double rbs, double dt)
{
    double left_min_speed;
    double right_min_speed;

    left_min_speed = (abs(lts) < abs(lms)) ? lts : lms;
    left_min_speed = (abs(left_min_speed) < abs(lbs)) ? left_min_speed : lbs;
    right_min_speed = (abs(rts) < abs(rms)) ? rts : rms;
    right_min_speed = (abs(right_min_speed) < abs(rbs)) ? right_min_speed : rbs;

    double a_z = (-right_min_speed + left_min_speed) / TRACK_WIDTH;
    double l_x = (left_min_speed + right_min_speed) / 2.0;
    (robot_odom->twist).twist.linear.x = l_x;
    (robot_odom->twist).twist.angular.z = a_z;
    (robot_odom->header).stamp = nh.now();
    (robot_odom->header).frame_id = "ekf_odom";
    robot_odom->child_frame_id = "base_footprint";

    heading += a_z * dt;
    x_pos += l_x * cos(heading) * dt;
    y_pos += l_x * sin(heading) * dt;
    (robot_odom->pose).pose.position.x = x_pos;
    (robot_odom->pose).pose.position.y = y_pos;

    for (uint8_t i = 0; i < 36; i += 7)
    {
        // fill in diagonals of covariance matrix
        (robot_odom->pose).covariance[i] = mat[i % 6];
        (robot_odom->twist).covariance[i] = mat[i % 6];
    }
}

// set motors to target velocity and update motor setpoint
void controlMotors(double left_speed, double right_speed)
{

    setMotorSpeed(&P0, velocityToPulse(right_speed, 0, false), 0);
    setSetpoint(&P0, right_speed);

    setMotorSpeed(&P1, velocityToPulse(right_speed, 1, false), 1);
    setSetpoint(&P1, right_speed);

    setMotorSpeed(&P2, velocityToPulse(right_speed, 2, false), 2);
    setSetpoint(&P2, right_speed);

    setMotorSpeed(&P3, velocityToPulse(left_speed, 3, true), 3);
    setSetpoint(&P3, -1 * left_speed);

    setMotorSpeed(&P4, velocityToPulse(left_speed, 4, true), 4);
    setSetpoint(&P4, -1 * left_speed);

    setMotorSpeed(&P5, velocityToPulse(left_speed, 5, true), 5);
    setSetpoint(&P5, -1 * left_speed);

    // Update PID with new setpoint
    feedback0.UpdateOutput();
    feedback1.UpdateOutput();
    feedback2.UpdateOutput();
    feedback3.UpdateOutput();
    feedback4.UpdateOutput();
    feedback5.UpdateOutput();
}

void setup()
{
    initializeAllPins();

#ifdef USING_ROS
    nh.initNode();
    nh.subscribe(velocity);
    nh.subscribe(pid_params);

    // pushedVals.layout.dim = (std_msgs::MultiArrayDimension *)malloc(sizeof(std_msgs::MultiArrayDimension));

    // pushedVals.layout.dim_length = 0;
    //  pushedVals.data_length = 6;

    // pushedVals.data = (std_msgs::Float64*)malloc(sizeof(std_msgs::Float64)*6);

    nh.advertise(encoderPub0);
    nh.advertise(encoderPub1);
    nh.advertise(encoderPub2);
    nh.advertise(encoderPub3);
    nh.advertise(encoderPub4);
    nh.advertise(encoderPub5);
    nh.advertise(robotPub);

    Serial.end(); // if using ROS Serial, can't use USB Serial
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

    feedback0.SetMode(AUTOMATIC);
    feedback0.SetOutputLimits(MIN_PULSE, MAX_PULSE);

    feedback1.SetMode(AUTOMATIC);
    feedback1.SetOutputLimits(MIN_PULSE, MAX_PULSE);

    feedback2.SetMode(AUTOMATIC);
    feedback2.SetOutputLimits(MIN_PULSE, MAX_PULSE);

    feedback3.SetMode(AUTOMATIC);
    feedback3.SetOutputLimits(MIN_PULSE, MAX_PULSE);

    feedback4.SetMode(AUTOMATIC);
    feedback4.SetOutputLimits(MIN_PULSE, MAX_PULSE);

    feedback5.SetMode(AUTOMATIC);
    feedback5.SetOutputLimits(MIN_PULSE, MAX_PULSE);
}

int ctr = 0;
double kP_set, kI_set, kD_set;
void loop()
{
#ifdef USING_ROS
    nh.spinOnce();

    time_cur = (double)millis();

    loop_delta_time = (time_cur - time_prev) * 0.001;
    if (loop_delta_time < 0)
        loop_delta_time = 0.0;
    time_prev = time_cur;

    updateOdometry(&odom,
                   -1 * E5A.PIDvelocity, -1 * E4A.PIDvelocity, -1 * E3A.PIDvelocity,
                   E2A.PIDvelocity, E1A.PIDvelocity, E0A.PIDvelocity, loop_delta_time);

    // Send values over RosSerial every 100 iterations
    if (ctr % 100 == 0)
    {
        encoderVal0.data = E0A.PIDvelocity;
        encoderVal1.data = E1A.PIDvelocity;
        encoderVal2.data = E2A.PIDvelocity;
        encoderVal3.data = -1 * E3A.PIDvelocity;
        encoderVal4.data = -1 * E4A.PIDvelocity;
        encoderVal5.data = -1 * E5A.PIDvelocity;

        // encoderPub0.publish(&encoderVal0);
        // encoderPub1.publish(&encoderVal1);
        // encoderPub2.publish(&encoderVal2);
        // encoderPub3.publish(&encoderVal3);
        // encoderPub4.publish(&encoderVal4);
        // encoderPub5.publish(&encoderVal5);
        robotPub.publish(&odom);
    }

    // increase vel_timer by time  (converted to ms)
    vel_timer += loop_delta_time * 1000;
    // stop motors if timer exceeds timeout callback rate
    if (vel_timer > (CMD_VEL_RATE_MS * 3))
    {
        if (ctr % 1000 == 0)
        {
            controlMotors(0, 0);
        }
    }

    double computed[4];

    // calculate new PID velocity, and update motors if new velocity differs from previous one
    bool changed = feedback0.Compute(computed);
    if (changed)
    {
        updateMotorSpeed(&P0);
    }

    changed = feedback1.Compute(computed);
    if (changed)
    {
        updateMotorSpeed(&P1);
    }

    changed = feedback2.Compute(computed);
    if (changed)
    {
        updateMotorSpeed(&P2);
    }

    changed = feedback3.Compute(computed);
    if (changed)
    {
        updateMotorSpeed(&P3);
    }

    changed = feedback4.Compute(computed);
    if (changed)
    {
        updateMotorSpeed(&P4);
    }

    changed = feedback5.Compute(computed);
    if (changed)
    {
        updateMotorSpeed(&P5);
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
