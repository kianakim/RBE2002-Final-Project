// Accelerometer Distance Testing
#include <Wire.h>
#include <L3G.h>
#include <LSM303.h>
L3G gyro;
LSM303 accel;

void setup() {
  Serial.begin(115200);
  accel.init();
  accel.enableDefault();
  // might need to add line initializing device type
  
}

void loop() {
  // put your main code here, to run repeatedly:

}
