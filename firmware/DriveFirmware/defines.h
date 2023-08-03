#define USE_STM32_HW_SERIAL
#define BAUD_RATE 128000
#define SERIAL_RX PB11 // pins being used for ROSSerial
#define SERIAL_TX PB10

#define PI 3.141592653

// Map DMC pins to stm32 pins
#define P0_ PA_0
#define P1_ PA_3
#define P2_ PA_11_ALT1
#define P3_ PA_8
#define P4_ PA_9
#define P5_ PA_10

#define E0A_ PA_6
#define E1A_ PA_2
#define E2A_ PB_0_ALT1
#define E3A_ PB_6
#define E4A_ PA_1
#define E5A_ PB_8

#define E0B_ PA_7_ALT1
#define E2B_ PB_1_ALT1
#define E3B_ PB_7
#define E5B_ PB_9

// map timer channels to stm32 pins
#define TIM_1_CH_1 P5_
#define TIM_2_CH_1 P1_
#define TIM_3_CH_1 E0A_
#define TIM_4_CH_1 E3A_

#define CMD_VEL_RATE_MS 200

#define MIN_PULSE (uint16_t)1000 // PWM Pulse Duration (microseconds)
#define MAX_PULSE (uint16_t)2000

#define REFRESH_RATE (uint32_t)50000 // Value (microseconds) at which encoder timers overflow, defines how often encoder velocity is calculated
#define TIMERS_RATE (uint32_t)3000   // Value (microseconds) at which motor timers overflow

#define TRACK_WIDTH 0.82    // 60cm
#define WHEEL_RADIUS 0.10   // 10cm
#define MAX_ANGULAR_VELOCITY 13.30 // Fastest wheel speed possible in rad/s (~1.3m/s)


#define USING_PID
#define USING_ROS
// #define VELOCITY_DEBUG
// #define DISTANCE_DEBUG