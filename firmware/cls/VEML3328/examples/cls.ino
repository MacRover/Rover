#include <Wire.h>
#include <VEML3328.h>

VEML3328 cls;

void setup() {
  Wire.begin();
  Serial.begin(9600);

  if(cls.ping()==0){  //board has no issues
    //Add the configurations you want for example cls.writeConfiguration(VEML3328_SD_ENABLE + VEML3328_IT_100MS)
    //Default integration time is 50ms
    cls.writeConfiguration(VEML3328_SD_ENABLE + VEML3328_IT_50MS);
  }else{
    Serial.println("Board is not responding");
  }
  
  delay(1500);//setup delay
}

void loop() {
  uint16_t red = cls.readRed();
  uint16_t blue = cls.readBlue();
  uint16_t green = cls.readGreen();
  uint16_t clearr = cls.readClear();
  Serial.print("Red: ");
  Serial.print(red);
  Serial.print(", Green: ");
  Serial.print(green);
  Serial.print(", Blue: ");
  Serial.print(blue);
  Serial.print(", Clear: ");
  Serial.println(clearr);
  delay(500);
}
