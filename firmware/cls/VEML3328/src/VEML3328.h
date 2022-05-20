#ifndef VEML3328_H
#define VEML3328_H

#include "arduino.h"

// VEML3328 I2C ADDRESS
#define I2C_ADDRESS   0x10

// REGISTER CONF (00H) SETTINGS

#define VEML3328_IT_50MS       0x00
#define VEML3328_IT_100MS      0x10
#define VEML3328_IT_200MS      0x20
#define VEML3328_IT_400MS      0x30

#define VEML3328_TRIG_DISABLE  0x00
#define VEML3328_TRIG_ENABLE   0x04

#define VEML3328_AF_AUTO       0x00
#define VEML3328_AF_FORCE      0x02

#define VEML3328_SD_ENABLE     0x00
#define VEML3328_SD_DISABLE    0x01 

// COMMAND CODES

#define COMMAND_CODE_CONFIG    0x00
#define COMMAND_CODE_CLEAR     0x04
#define COMMAND_CODE_RED       0x05
#define COMMAND_CODE_GREEN     0x06
#define COMMAND_CODE_BLUE      0x07
#define COMMAND_CODE_INFRA     0x08

class VEML3328 {
	
  private: 
    uint16_t read(uint8_t);
    uint8_t lastConfiguration;
	
  public:
    VEML3328();
    uint8_t ping();
    void writeConfiguration(uint8_t);
    uint16_t readRed(void);
    uint16_t readGreen(void);
    uint16_t readBlue(void);
    uint16_t readClear(void);
};

#endif