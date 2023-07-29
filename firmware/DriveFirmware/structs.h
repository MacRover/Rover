typedef struct encoder
{
    HardwareTimer *timer;
    uint32_t channel;
    PinName pin;
    volatile int16_t pulseCount; // number of encoder pulses detected
    volatile double linearVelocity;
    volatile double angularVelocity;
} Encoder;

typedef struct Motor
{
    int id;
    HardwareTimer *timer;
    uint32_t channel;
    PinName pin;
    volatile double velCommand; // angular motor velocity
    volatile double setpoint; // desired motor velocity

} Motor;

typedef struct PID_Values
{
    double P;
    double I;
    double D;
} PID_Values;
