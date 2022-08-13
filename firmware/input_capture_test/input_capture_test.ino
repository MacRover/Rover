// Map DMC pins to stm32 pins
#define P0_ PB_0_ALT1
#define P1_ PA_7_ALT1
#define P2_ PA_6
#define P3_ PA_8
#define P4_ PA_9
#define P5_ PA_10

volatile uint32_t FrequencyMeasured, LastCapture = 0, CurrentCapture;
uint32_t input_freq = 0;
volatile uint32_t rolloverCompareCount = 0;


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

motors Motors;
timers Timers;


void setup() {
  // put your setup code here, to run once:
  configureHardwareTimers();
}

void loop() {
  // put your main code here, to run repeatedly:

}

void configureHardwareTimers()
{
    // Automatically retrieve timer instance and channel associated to pin
    // This is used to be compatible with all STM32 series automatically.
    TIM_TypeDef *Instance1 = (TIM_TypeDef *)pinmap_peripheral(P0_, PinMap_PWM);
    TIM_TypeDef *Instance2 = (TIM_TypeDef *)pinmap_peripheral(P3_, PinMap_PWM);

    Motors.P0 = STM_PIN_CHANNEL(pinmap_function(P0_, PinMap_PWM));
    Motors.P1 = STM_PIN_CHANNEL(pinmap_function(P1_, PinMap_PWM));
    Motors.P2 = STM_PIN_CHANNEL(pinmap_function(P2_, PinMap_PWM));
    Motors.P3 = STM_PIN_CHANNEL(pinmap_function(P3_, PinMap_PWM));
    Motors.P4 = STM_PIN_CHANNEL(pinmap_function(P4_, PinMap_PWM));
    Motors.P5 = STM_PIN_CHANNEL(pinmap_function(P5_, PinMap_PWM));

    // Instantiate HardwareTimer object. Thanks to 'new' instantiation, HardwareTimer is not destructed when setup() function is finished.
    Timers.TimerA = new HardwareTimer(Instance1);
    Timers.TimerB = new HardwareTimer(Instance2);

    // Configure and start PWM
    Timers.TimerA->setMode(Motors.P0, TIMER_OUTPUT_COMPARE_PWM1, P0_);
    Timers.TimerA->setMode(Motors.P1, TIMER_OUTPUT_COMPARE_PWM1, P1_);
    Timers.TimerA->setMode(Motors.P2, TIMER_INPUT_CAPTURE_RISING, P2_); //SET TO INPUT CAPTURE
    Timers.TimerB->setMode(Motors.P3, TIMER_INPUT_CAPTURE_RISING, P3_);
    Timers.TimerB->setMode(Motors.P4, TIMER_INPUT_CAPTURE_FALLING, P4_);
    Timers.TimerB->setMode(Motors.P5, TIMER_INPUT_CAPTURE_RISING, P5_);

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

    Timers.TimerA->attachInterrupt(Motors.P2, InputCapture_IT_callback); //INPUT CAPTURE CALLS CALLBACK
    //Timers.TimerA->attachInterrupt(Rollover_IT_callback);

    
    // enable timers
    Timers.TimerA->resume();
    Timers.TimerB->resume();
}
int newPulse;
void InputCapture_IT_callback(void)
{
  CurrentCapture = Timers.TimerA->getCaptureCompare(Motors.P2);
  /* frequency computation */
  if (CurrentCapture > LastCapture) {
    newPulse = 1500 + (CurrentCapture - LastCapture);
    
  }
  else if (CurrentCapture <= LastCapture) {
    /* 0x1000 is max overflow value */
    newPulse = 1500+ (CurrentCapture + 3000 - LastCapture);  
  }

  Timers.TimerA->setCaptureCompare(Motors.P0, newPulse, MICROSEC_COMPARE_FORMAT);
  Timers.TimerA->setCaptureCompare(Motors.P1, newPulse, MICROSEC_COMPARE_FORMAT);
  //Timers.TimerA->setCaptureCompare(Motors.P2, newPulse, MICROSEC_COMPARE_FORMAT);

  //Timers.TimerA->resume()
  LastCapture = CurrentCapture;
}
