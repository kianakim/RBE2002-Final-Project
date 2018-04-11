// Turn Drive with IMU
// Team 17

#include <Wire.h>
#include <L3G.h>
#include <LSM303.h>
L3G gyro;
LSM303 accel;


void setup() {
  Serial.begin(115200);
  Wire.begin(); // start I2C

  // initialize gyro
  gyro.init();
  gyro.enableDefault(); // 250 deg/s
  delay(1000);
  gyroZero();
}

void loop() {
  // put your main code here, to run repeatedly:

}

// gyro calibration "zero"
void gyroZero() {
  for(int i = 0; i < 200; i++) {
    gyro.read();
    
  }
}
// read gyro

// print gyro


