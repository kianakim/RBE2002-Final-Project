/*
   Messages.h

    Created on: 10/1/16
        Author: joest
*/

#ifndef DRIVE_H_
#define DRIVE_H_

#include "Arduino.h"

#define COUNTS_PER_DEG 4.535  // 1632.67 counts/rev * 1 rev/360deg
#define DIST_TO_DEG 20.834    // 360deg/(2*Pi*2.75" wheel)
#define kp 0.2248             // based on max value of 12"

class DriveMotors {
  public:
    // constructor
    DriveMotors(int rightFWDPin, int rightREVPin, int leftFWDPin, int leftREVPin);

    // add variables
    left_pos;
    right_pos;
    // drive forward method (straightening w/ encoders)
    void driveForward(int dist);

    // stop drive motors
    void driveStop();

  private:
    int _rightFWDPin;
    int _rightREVPin;
    int _leftFWDPin;
    int _leftREVPin;
    
};



#endif
