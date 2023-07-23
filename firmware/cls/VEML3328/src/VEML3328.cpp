#include "Wire.h"
#ifndef __MATH_H
#include <math.h>
#endif
#include "veml3328.h"

VEML3328::VEML3328() {}

uint8_t VEML3328::ping() {
    Wire.beginTransmission(I2C_ADDRESS);
    return Wire.endTransmission();
}

void VEML3328::writeConfiguration(uint8_t configuration) {
  Wire.beginTransmission(I2C_ADDRESS);  
  Wire.write(COMMAND_CODE_CONFIG); 
  Wire.write(configuration); 
  Wire.write(0);
  Wire.endTransmission(); 
  lastConfiguration = configuration;
}

uint16_t VEML3328::read(uint8_t commandCode) {
  uint16_t data = 0; 
  
  Wire.beginTransmission(I2C_ADDRESS);
  Wire.write(commandCode);
  Wire.endTransmission(false);
  Wire.requestFrom(I2C_ADDRESS, 2);
  while(Wire.available()) 
  {
    data = Wire.read(); 
    data |= Wire.read() << 8;
  }
  
  return data; 
}

uint16_t VEML3328::readRed(void) {
  return(read(COMMAND_CODE_RED));
}

uint16_t VEML3328::readGreen(void) {
  return(read(COMMAND_CODE_GREEN));
}

uint16_t VEML3328::readBlue(void) {
  return(read(COMMAND_CODE_BLUE));
}

uint16_t VEML3328::readClear(void) {
  return(read(COMMAND_CODE_CLEAR));
}