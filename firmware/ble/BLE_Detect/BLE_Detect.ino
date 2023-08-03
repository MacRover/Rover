/*
   Based on Neil Kolban example for IDF: https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleScan.cpp
   Ported to Arduino ESP32 by Evandro Copercini
   Advertised Device: Name: CIRC BLE Beacon, Address: 24:62:ab:f9:49:92, manufacturer data: 4c0002152d7a9f0ce0e84cc9a71ba21db2d034a100050058c5, txPower: 9, rssi: -48
   Advertised Device: Name: , Address: 24:62:ab:f9:49:92, manufacturer data: 4c0002154d6fc88bbe756698da486866a36ec78e0000001400, rssi: -52 
*/

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <BLEAddress.h>

int scanTime = 5; //In seconds
BLEScan* pBLEScan;

class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) {
//      Serial.printf("Advertised Device: %s \n", advertisedDevice.toString().c_str());
      if(advertisedDevice.getAddress() == BLEAddress("24:0a:c4:60:9b:5a")){
        double distance = pow(10, ((- 67 - advertisedDevice.getRSSI()) / 20.0));
        Serial.printf("ESP-1 --- Advertised Device: %s, Distance: %f \n", advertisedDevice.toString().c_str(), distance);
      }
    }
};

void setup() {
  Serial.begin(115200);
  Serial.println("ESP-1 --- Scanning...");

  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan(); //create new scan
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true); //active scan uses more power, but get results faster
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(99);  // less or equal setInterval value
}

void loop() {
  // put your main code here, to run repeatedly:
  BLEScanResults foundDevices = pBLEScan->start(scanTime, false);
//  Serial.print("Devices found: ");
//  Serial.println(foundDevices.getCount());
//  Serial.println("Scan done!");
  pBLEScan->clearResults();   // delete results fromBLEScan buffer to release memory
  delay(1000);
}
