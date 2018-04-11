// Team 17
// RBE 2002 Main Final Project Code

#include <Encoder.h>

#define COUNTS_PER_DEG 4.535  // 1632.67 counts/rev * 1 rev/360deg
#define DIST_TO_DEG 20.834    // 360deg/(2*Pi*2.75" wheel)
#define kp 0.2248             // based on max value of 12"

// Drive Motor Pins (MC33926 Pololu Driver)
const int leftFWDPin = 5; // IN2
const int leftREVPin = 6;
const int rightFWDPin = 4; // IN1
const int rightREVPin = 3;

Encoder leftDriveEnc(3, 1);   // YEL - INT2, WH - D0
Encoder rightDriveEnc(2, 0);  // YEL - INT2, WH - D0

void setup() {
  Serial.begin(115200);

  // set all motor signal pins to LOW
  analogWrite(leftFWDPin, 0);
  analogWrite(leftREVPin, 0);
  analogWrite(rightFWDPin, 0);
  analogWrite(rightREVPin, 0);
}

void loop() {

}

// (Manual) drive forward for "dist" inches, straight via encoder
void driveForward(int dist) {
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
    analogWrite(rightFWDPin, right_err * kp);
    analogWrite(rightREVPin, 0);
    analogWrite(leftFWDPin, left_err * kp);
    analogWrite(leftREVPin, 0);

    // read current values, calculate error
    right_pos = rightDriveEnc.read();
    left_pos = leftDriveEnc.read();

    double right_err = right_pos - total_counts;
    double left_err = left_pos - total_counts;

  }
}

// stop drive motors
void driveStop() {
  analogWrite(rightFWDPin, 0);
  analogWrite(rightREVPin, 0);
  analogWrite(leftFWDPin, 0);
  analogWrite(leftREVPin, 0);
}

