// TODO: 
// FIX ENCODER DIRECTIONS                                       - DONE
// CONVERT SETPOINT TO RAD/S                                    - DONE
// FIX TWIST TO REQUIRED SPEED                                  - DONE
// TEST CONTROLLER                                              - DONE
// CALIBRATE MOTORS                                             - DONE
// PUBLISH CMD, SETPOINT, MEASURED AS Quaternion                - DONE
// USE TOPIC TO SWITCH BETWEEN MOTORS TO VISUALIZE IN RQT PLOT  - DONE
// FIX MOTOR DIRECTIONS                                         - DONE
// WHEEL 3 EXCEPTION - NO ENCODER                               - NOT DONE
// TUNE PID                                                     - NOT DONE

#include <ros.h>
#include <nav_msgs/Odometry.h>
#include <HardwareSerial.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Quaternion.h>
#include "PID_v1.h"
#include "defines.h"
#include "structs.h"

// linear calibration of PWM pulse to angular velocity curve
double factor[6] = {35.5, 35.5, 35.5, 35.5, 35.5, 35.5};
double offset[6] = {33, 33, 33, 33, 33, 33};

PID_Values pid0 = {.P = 0.8, .I = 0.5, .D = 0.0};
PID_Values pid1 = {.P = 0.8, .I = 0.5, .D = 0.0};
PID_Values pid2 = {.P = 0.8, .I = 0.5, .D = 0.0};
PID_Values pid3 = {.P = 0.8, .I = 0.5, .D = 0.0};
PID_Values pid4 = {.P = 0.8, .I = 0.5, .D = 0.0};
PID_Values pid5 = {.P = 0.8, .I = 0.5, .D = 0.0};

// covariance matrix diagonals
double mat[6] = {0.01, 0.01, 1000, 1000, 1000, 0.01};
double x_pos = 0, y_pos = 0, heading = 0;
double vel_timer = 0.0;
unsigned long time_prev = 0, time_prev2 = 0;
double loop_delta_time;
uint8_t PIDDebugMotor = 0;

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
void controlMotors(double left_ang_vel, double right_ang_vel);
void updateOdometry(nav_msgs::Odometry *robot_odom, double lts, double lms, double lbs, double rts, double rms, double rbs, double dt);
uint16_t velToCmd(const double vel, const int encoderIndex);
void updatePIDDebug(geometry_msgs::Quaternion *msg);

ros::Subscriber<geometry_msgs::Twist> velocity("cmd_vel", &controlMotorsRos);
ros::Subscriber<std_msgs::Float64MultiArray> pid_params("cmd_pid", &setParamsRos);

// variables for publishing encoder values over ros for debugging
std_msgs::Float64 encoderVal0, encoderVal1, encoderVal2, encoderVal3, encoderVal4, encoderVal5;
geometry_msgs::Quaternion pidDebugMsg;
nav_msgs::Odometry odom;

ros::Publisher encoderPub0("enc_pub0", &encoderVal0);
ros::Publisher encoderPub1("enc_pub1", &encoderVal1);
ros::Publisher encoderPub2("enc_pub2", &encoderVal2);
ros::Publisher encoderPub3("enc_pub3", &encoderVal3);
ros::Publisher encoderPub4("enc_pub4", &encoderVal4);
ros::Publisher encoderPub5("enc_pub5", &encoderVal5);

ros::Publisher PIDDebug("pid_debug", &pidDebugMsg);
ros::Publisher robotPub("robot_odom", &odom);

HardwareTimer *tim1, *tim2, *tim3, *tim4;
Encoder E0A, E0B, E1A, E2A, E2B, E3A, E3B, E4A, E5A, E5B;
Motor P0, P1, P2, P3, P4, P5;

// A encoders are initialized with timers - B encoders are merely used to identify leading/lagging
Encoder *encoders[] = {&E0A, &E1A, &E2A, &E3A, &E4A, &E5A};
Motor *motors[] = {&P0, &P1, &P2, &P3, &P4, &P5};

// PID struct with input variable, output variable, and desired variabe
PID feedback0(&(E0A.angularVelocity), &(P0.velCommand), &(P0.setpoint), pid0.P, pid0.I, pid0.D, P_ON_E, DIRECT);
PID feedback1(&(E1A.angularVelocity), &(P1.velCommand), &(P1.setpoint), pid1.P, pid1.I, pid1.D, P_ON_E, DIRECT);
PID feedback2(&(E2A.angularVelocity), &(P2.velCommand), &(P2.setpoint), pid2.P, pid2.I, pid2.D, P_ON_E, DIRECT);
PID feedback3(&(E3A.angularVelocity), &(P3.velCommand), &(P3.setpoint), pid3.P, pid3.I, pid3.D, P_ON_E, DIRECT);
PID feedback4(&(E4A.angularVelocity), &(P4.velCommand), &(P4.setpoint), pid4.P, pid4.I, pid4.D, P_ON_E, DIRECT);
PID feedback5(&(E5A.angularVelocity), &(P5.velCommand), &(P5.setpoint), pid5.P, pid5.I, pid5.D, P_ON_E, DIRECT);

void initializeTimer(HardwareTimer **tim, PinName pin, uint32_t overflow, void (*interrupt)(void))
{
    TIM_TypeDef *Instance = (TIM_TypeDef *)pinmap_peripheral(pin, PinMap_PWM);
    *tim = new HardwareTimer(Instance);

    (*tim)->setOverflow(overflow, MICROSEC_FORMAT);

    if (interrupt != NULL)
        (*tim)->attachInterrupt(interrupt);
}

void initializeEncoder(Encoder *encoder, HardwareTimer **tim, PinName pin, void (*interrupt)(void))
{
    encoder->timer = *tim;
    encoder->channel = STM_PIN_CHANNEL(pinmap_function(pin, PinMap_PWM));
    encoder->pin = pin;
    encoder->pulseCount = 0;
    encoder->linearVelocity = 0;
    encoder->angularVelocity = 0;

    (*tim)->setMode(encoder->channel, TIMER_INPUT_CAPTURE_RISING, pin);

    if (interrupt != NULL)
        (*tim)->attachInterrupt(encoder->channel, interrupt);
}

void initializeMotor(Motor *motor, HardwareTimer **tim, PinName pin)
{
    motor->timer = *tim;
    motor->channel = STM_PIN_CHANNEL(pinmap_function(pin, PinMap_PWM));
    motor->pin = pin;
    motor->velCommand = 0;
    motor->setpoint = 0.0;

    (motor->timer)->setMode(motor->channel, TIMER_OUTPUT_COMPARE_PWM1, pin);
    (motor->timer)->setCaptureCompare(motor->channel, 1500, MICROSEC_COMPARE_FORMAT);
}

// Set motor speed based off value
void updateMotorSpeed(Motor *motor, uint16_t CmdPulse)
{

    if (motor->id != 0) // disable all but one motor
        return;

    if (motor->id >= 3) // flip direction of right 3 motors
    {
        if (CmdPulse > 1500)
            CmdPulse = 1500 - (CmdPulse - 1500);
    }

    if (CmdPulse > 2000) // set limits
        CmdPulse = 2000;
    if (CmdPulse < 1000)
        CmdPulse = 1000;

    (motor->timer)->setCaptureCompare(motor->channel, CmdPulse, MICROSEC_COMPARE_FORMAT);
}

// Set new desired velocity in motor struct
void setSetpoint(Motor *motor, volatile double newSetpoint)
{
    motor->setpoint = newSetpoint;
}

uint16_t velToCmd(const double vel, const int encoderIndex)
{
    if (vel == 0)
        return 1500;
    else
        return (uint16_t)((vel * factor[encoderIndex] + offset[encoderIndex]) + 1500);
}

/*
 * CALLBACK FUNCTIONS
 * Called when the pin tied to the encoder A channel goes high
 * If both A channel and B channel are high, encoder must be going in negative direction (as B lags A)
 * Update encoder struct pulse count
 */

void encoder0Callback(void)
{
    // if pin B is high, then pinB went first
    if ((GPIOA->IDR & GPIO_IDR_IDR6) && (GPIOA->IDR & GPIO_IDR_IDR7))
        (E0A.pulseCount)--;
    else
        (E0A.pulseCount)++;
}

void encoder1Callback(void)
{
    if ((GPIOA->IDR & GPIO_IDR_IDR2) && (GPIOA->IDR & GPIO_IDR_IDR4))
        (E1A.pulseCount)--;
    else
        (E1A.pulseCount)++;
}

void encoder2Callback(void)
{
    if ((GPIOB->IDR & GPIO_IDR_IDR0) && (GPIOB->IDR & GPIO_IDR_IDR1))
        (E2A.pulseCount)--;
    else
        (E2A.pulseCount)++;
}

void encoder3Callback(void)
{
    // Flipped ++ and -- for encoders on the right side
    if ((GPIOB->IDR & GPIO_IDR_IDR6) && (GPIOB->IDR & GPIO_IDR_IDR7))
        (E3A.pulseCount)++; 
    else
        (E3A.pulseCount)--;
}

void encoder4Callback(void)
{
    if ((GPIOA->IDR & GPIO_IDR_IDR5) && (GPIOA->IDR & GPIO_IDR_IDR1))
        (E4A.pulseCount)++;
    else
        (E4A.pulseCount)--;
}

void encoder5Callback(void)
{
    if ((GPIOB->IDR & GPIO_IDR_IDR8) && (GPIOB->IDR & GPIO_IDR_IDR9))
        (E5A.pulseCount)++;
    else
        (E5A.pulseCount)--;
}

// Calculate encoder velocity and reset encoder pulse counts
void rolloverCallback(void)
{
    E0A.angularVelocity = ((volatile double)E0A.pulseCount / 1024.0) * (1000000.0 / (volatile double)REFRESH_RATE) * (2 * PI);
    E1A.angularVelocity = ((volatile double)E1A.pulseCount / 1024.0) * (1000000.0 / (volatile double)REFRESH_RATE) * (2 * PI);
    E2A.angularVelocity = ((volatile double)E2A.pulseCount / 1024.0) * (1000000.0 / (volatile double)REFRESH_RATE) * (2 * PI);
    E3A.angularVelocity = ((volatile double)E3A.pulseCount / 1024.0) * (1000000.0 / (volatile double)REFRESH_RATE) * (2 * PI);
    E4A.angularVelocity = ((volatile double)E4A.pulseCount / 1024.0) * (1000000.0 / (volatile double)REFRESH_RATE) * (2 * PI);
    E5A.angularVelocity = ((volatile double)E5A.pulseCount / 1024.0) * (1000000.0 / (volatile double)REFRESH_RATE) * (2 * PI);

    E0A.linearVelocity = E0A.angularVelocity * WHEEL_RADIUS;
    E1A.linearVelocity = E1A.angularVelocity * WHEEL_RADIUS;
    E2A.linearVelocity = E2A.angularVelocity * WHEEL_RADIUS;
    E3A.linearVelocity = E3A.angularVelocity * WHEEL_RADIUS;
    E4A.linearVelocity = E4A.angularVelocity * WHEEL_RADIUS;
    E5A.linearVelocity = E5A.angularVelocity * WHEEL_RADIUS;

    E0A.pulseCount = 0;
    E1A.pulseCount = 0;
    E2A.pulseCount = 0;
    E3A.pulseCount = 0;
    E4A.pulseCount = 0;
    E5A.pulseCount = 0;
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

    if (kP == -1.0 || kI == -1.0 || kD == -1.0)
    {
        PIDDebugMotor = (int)feedback_to_change;
        nh.loginfo("\nPID Debug Motor Updated\n");
        return;
    }

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
        feedback0.SetTunings(kP, kI, kD);
    else if (feedback_to_change == 1)
        feedback1.SetTunings(kP, kI, kD);
    else if (feedback_to_change == 2)
        feedback2.SetTunings(kP, kI, kD);
    else if (feedback_to_change == 3)
        feedback3.SetTunings(kP, kI, kD);
    else if (feedback_to_change == 4)
        feedback4.SetTunings(kP, kI, kD);
    else if (feedback_to_change == 5)
        feedback5.SetTunings(kP, kI, kD);
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
    double left_speed = cmd_vel.linear.x - (cmd_vel.angular.z * TRACK_WIDTH / 2);
    double right_speed = cmd_vel.linear.x + (cmd_vel.angular.z * TRACK_WIDTH / 2);

    vel_timer = 0.0;

    // convert linear speed to rad/s
    controlMotors(left_speed / WHEEL_RADIUS, right_speed / WHEEL_RADIUS);
}

// take min speed of left and right wheels and update odometry
void updateOdometry(nav_msgs::Odometry *robot_odom, double lts, double lms,
                    double lbs, double rts, double rms, double rbs, double dt)
{
    dt *= 0.001;
    double left_min_speed;
    double right_min_speed;

    left_min_speed = (abs(lts) < abs(lms)) ? lts : lms;
    left_min_speed = (abs(left_min_speed) < abs(lbs)) ? left_min_speed : lbs;
    right_min_speed = (abs(rts) < abs(rms)) ? rts : rms;
    right_min_speed = (abs(right_min_speed) < abs(rbs)) ? right_min_speed : rbs;

    double a_z = (right_min_speed - left_min_speed) / TRACK_WIDTH;
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

void updatePIDDebug(geometry_msgs::Quaternion *msg, double left_ang_vel, double right_ang_vel)
{
    msg->x = (float)PIDDebugMotor;
    msg->y = encoders[PIDDebugMotor]->angularVelocity; 

    // PID Output Command
    msg->z = velToCmd(motors[PIDDebugMotor]->velCommand, PIDDebugMotor);

    if (PIDDebugMotor < 3)
    {
        // Calibration Curve Output Command (no PID)
        // msg->z = velToCmd(right_ang_vel, PIDDebugMotor);

        msg->w = right_ang_vel; // setpoint
    }
    else
    {
        // Calibration Curve Output Command (no PID)
        // msg->z = velToCmd(left_ang_vel, PIDDebugMotor);

        msg->w = left_ang_vel; // setpoint
    }
}

// updates setpoint (rad/s)
void controlMotors(double left_ang_vel, double right_ang_vel)
{
    setSetpoint(&P0, right_ang_vel);
    setSetpoint(&P1, right_ang_vel);
    setSetpoint(&P2, right_ang_vel);
    setSetpoint(&P3, left_ang_vel);
    setSetpoint(&P4, left_ang_vel);
    setSetpoint(&P5, left_ang_vel);

    // Update PID with new setpoint
    feedback0.UpdateOutput();
    feedback1.UpdateOutput();
    feedback2.UpdateOutput();
    feedback3.UpdateOutput();
    feedback4.UpdateOutput();
    feedback5.UpdateOutput();

    updatePIDDebug(&pidDebugMsg, left_ang_vel, right_ang_vel);

    // Update Motors using calibration curves, not PID
    // updateMotorSpeed(&P0, velToCmd(right_ang_vel, 0));
    // updateMotorSpeed(&P1, velToCmd(right_ang_vel, 1));
    // updateMotorSpeed(&P2, velToCmd(right_ang_vel, 2));
    // updateMotorSpeed(&P3, velToCmd(left_ang_vel, 3));
    // updateMotorSpeed(&P4, velToCmd(left_ang_vel, 4));
    // updateMotorSpeed(&P5, velToCmd(left_ang_vel, 5));
}

// -----------------------------------------------------------------

void setup()
{
    initializeAllPins();

    nh.initNode();
    nh.subscribe(velocity);
    nh.subscribe(pid_params);
    nh.advertise(encoderPub0);
    nh.advertise(encoderPub1);
    nh.advertise(encoderPub2);
    nh.advertise(encoderPub3);
    nh.advertise(encoderPub4);
    nh.advertise(encoderPub5);
    nh.advertise(robotPub);
    nh.advertise(PIDDebug);

    Serial.end(); // if using ROS Serial, can't use USB Serial

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

    P0.id = 0;
    P1.id = 1;
    P2.id = 2;
    P3.id = 3;
    P4.id = 4;
    P5.id = 5;

    feedback0.SetMode(AUTOMATIC);
    feedback1.SetMode(AUTOMATIC);
    feedback2.SetMode(AUTOMATIC);
    feedback3.SetMode(AUTOMATIC);
    feedback4.SetMode(AUTOMATIC);
    feedback5.SetMode(AUTOMATIC);

    feedback0.SetOutputLimits(MIN_PULSE, MAX_PULSE);
    feedback1.SetOutputLimits(MIN_PULSE, MAX_PULSE);
    feedback2.SetOutputLimits(MIN_PULSE, MAX_PULSE);
    feedback3.SetOutputLimits(MIN_PULSE, MAX_PULSE);
    feedback4.SetOutputLimits(MIN_PULSE, MAX_PULSE);
    feedback5.SetOutputLimits(MIN_PULSE, MAX_PULSE);
}

int ctr = 0;
double kP_set, kI_set, kD_set;

void loop()
{
    nh.spinOnce();

    loop_delta_time = millis() - time_prev2;
    time_prev2 = millis();

    vel_timer += loop_delta_time; // increase vel_timer by time

    if (vel_timer > (CMD_VEL_RATE_MS * 5)) // stop motors if timer exceeds timeout callback rate
    {
        if (ctr % 1000 == 0)
        {
            controlMotors(0, 0);
            if (ctr % 10000 == 0)
                nh.loginfo("AUTO STOP DRIVE");
        }
    }

    // Send values over RosSerial every 100 iterations
    if (ctr % 150 == 0)
    {
        encoderVal0.data = E0A.angularVelocity;
        encoderVal1.data = E1A.angularVelocity;
        encoderVal2.data = E2A.angularVelocity;
        encoderVal3.data = E3A.angularVelocity;
        encoderVal4.data = E4A.angularVelocity;
        encoderVal5.data = E5A.angularVelocity;

        encoderPub0.publish(&encoderVal0);
        encoderPub1.publish(&encoderVal1);
        encoderPub2.publish(&encoderVal2);
        encoderPub3.publish(&encoderVal3);
        encoderPub4.publish(&encoderVal4);
        encoderPub5.publish(&encoderVal5);
        PIDDebug.publish(&pidDebugMsg);
        robotPub.publish(&odom);
    }

    // Run PID at 50hz
    if ((millis() - time_prev) >= 20)
    {
        updateOdometry(&odom,
                       E5A.linearVelocity, E4A.linearVelocity, E3A.linearVelocity,
                       E2A.linearVelocity, E1A.linearVelocity, E0A.linearVelocity, 20);

        double computed[4];
        char log_msg[100];

        feedback0.Compute(computed);
        feedback1.Compute(computed);
        feedback2.Compute(computed);
        feedback3.Compute(computed);
        feedback4.Compute(computed);
        feedback5.Compute(computed);

        // Update Motor Speed with PID output
        updateMotorSpeed(&P0, velToCmd(P0.velCommand, 0));
        updateMotorSpeed(&P1, velToCmd(P1.velCommand, 1));
        updateMotorSpeed(&P2, velToCmd(P2.velCommand, 2));
        updateMotorSpeed(&P3, velToCmd(P3.velCommand, 3));
        updateMotorSpeed(&P4, velToCmd(P4.velCommand, 4));
        updateMotorSpeed(&P5, velToCmd(P5.velCommand, 5));

        time_prev = millis();
    }
}
