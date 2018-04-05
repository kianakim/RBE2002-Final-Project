// Team 17
// RBE 2002 Main Final Project Code
#include <Encoder.h>

#define COUNTS_PER_DEG 4.535  // 1632.67 counts/rev * 1 rev/360deg
#define DIST_TO_DEG 20.834    // 360deg/(2*Pi*2.75" wheel)

double kp = 0.2248;

// Drive Motor Pins (MC33926 Pololu Driver)
const int rightDriveFWDPin = 8;
const int rightDriveREVPin = 9;

const int leftDriveFWDPin = 7;
const int leftDriveREVPin = 6;

Encoder leftDriveEnc(2, 3); // might only use 1 INT port
Encoder rightDriveEnc(4, 5);

void setup() {
  Serial.begin(115200);
  pinMode(rightDriveFWDPin, OUTPUT);
  pinMode(rightDriveREVPin, OUTPUT);

  pinMode(leftDriveFWDPin, OUTPUT);
  pinMode(leftDriveREVPin, OUTPUT);

  pinMode(flameSensorPin, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:

}

// (Manual) drive forward for "dist" inches
void driveForward(int dist) {
  // gets desired ang dist (deg) from desired lin distance (in) // get better at commenting Kiana
  double total_deg = dist * DIST_TO_DEG;

  // converts degrees to encoder counts
  double total_counts = deg * COUNTS_PER_DEG;

  // read encoders
  double left_pos = leftDriveEnc.read();
  double right_pos = rightDriveEnc.read();

  // Proportional Control setup
  double right_err = right_pos - total_counts;
  double left_err = left_pos - total_counts;

  while (right_error < 5 && left_err < 5) {
    analogWrite(rightDriveFWDPin, right_err * kp);
    analogWrite(rightDriveREVPin, 0);
    analogWrite(leftDriveFWDPin, left_err * kp);
    analogWrite(leftDriveREVPin, 0);

    // read current values, calculate error
    right_pos = rightDriveEnc.read();
    left_pos = leftDriveEnc.read();

    double right_err = right_pos - total_counts;
    double left_err = left_pos - total_counts;
  }

}

void driveStop() {
  analogWrite(rightDriveFWDPin, 0);
  analogWrite(rightDriveREVPin, 0);
  analogWrite(leftDriveFWDPin, 0);
  analogWrite(leftDriveREVPin, 0);
}

