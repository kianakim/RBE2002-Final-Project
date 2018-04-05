// "Drive.cpp" - Drive Class
// Motor: Pololu 34:1 Metal Gearmotor w/ 48 CPR Encoder
// Driver: Pololu MC33926 Motor Driver Carrier

#include "Drive.h"
#include <Encoder.h>

Encoder leftDriveEnc(2, 3); // might only use 1 INT port
Encoder rightDriveEnc(4, 5);

// Class constructor
DriveMotors::DriveMotors(int rightFWDPin, int rightREVPin, int leftFWDPin, int leftREVPin) {
  pinMode(rightFWDPin, OUTPUT);
  pinMode(rightREVPin, OUTPUT);

  pinMode(leftFWDPin, OUTPUT);
  pinMode(leftREVPin, OUTPUT);

  _rightFWDPin = rightFWDPin;
  _rightREVPin = rightREVPin;
  _leftFWDPin = leftFWDPin;
  _leftREVPin = leftREVPin;
}

// (Manual) drive forward for "dist" inches
void DriveMotors::driveForward(int dist) {
  // gets desired ang dist (deg) from desired lin distance (in) // get better at commenting Kiana
  double total_deg = dist * DIST_TO_DEG;

  // converts degrees to encoder counts
  double total_counts = total_deg * COUNTS_PER_DEG; // might need to be an int???

  // read encoders
  double left_pos = leftDriveEnc.read();
  double right_pos = rightDriveEnc.read();

  // Proportional Control setup
  double right_err = right_pos - total_counts;
  double left_err = left_pos - total_counts;

  while (right_err < 5 && left_err < 5) {
    analogWrite(_rightFWDPin, right_err * kp);
    analogWrite(_rightREVPin, 0);
    analogWrite(_leftFWDPin, left_err * kp);
    analogWrite(_leftREVPin, 0);

    // read current values, calculate error
    right_pos = rightDriveEnc.read();
    left_pos = leftDriveEnc.read();

    double right_err = right_pos - total_counts;
    double left_err = left_pos - total_counts;
  }
}

void DriveMotors::driveStop() {
  analogWrite(_rightFWDPin, 0);
  analogWrite(_rightREVPin, 0);
  analogWrite(_leftFWDPin, 0);
  analogWrite(_leftREVPin, 0);
}

