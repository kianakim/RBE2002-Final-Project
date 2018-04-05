// Team 17
// RBE 2002 Main Final Project Code

#include "Drive.h"

// Drive Motor Pins (MC33926 Pololu Driver)
const int rightDriveFWDPin = 8;
const int rightDriveREVPin = 9;

const int leftDriveFWDPin = 7;
const int leftDriveREVPin = 6;

DriveMotors drive(8,9,7,6);

void setup() {
  Serial.begin(115200);
}

void loop() {
  
}
