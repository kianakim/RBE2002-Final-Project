/*
 * Messages.h
 *
 *  Created on: 10/1/16
 *      Author: joest
 */
#include "PID.h"
#include "Arduino.h"

//Class constructor
PID::PID(){
  
}

//Function to set PID gain values
void PID::setpid(float P, float I, float D){
  kp=P;
  ki=I;
  kd=D;
}

//Write this function to calculate a control signal from the set velocity 
//and the current velocity 
int PID::calc(double setpoint, double currVal){
    int output;
    unsigned long now = millis();
    double changeInTime = (double)(now - last_time);
    
    // calculate error
    double error = abs(abs(setpoint) - currVal);
    
    // calculate derivative of error
    double dErr = (error - last_error) / changeInTime;
    
    // calculate integral error. Running average is best but hard to implement
    sum_error += (error * changeInTime);
    
    // sum up the error value to send to the motor based off gain values. 
    output = kp * error + ki * sum_error + kd * dErr;

    // limit control value to 0-254
    if(output < 0) {
      output = 0;
    }
    else if (output > 254) {
      output = 254;
    }
    
    //return the control signal
    last_error = error;
    last_time = now;
    return output;
}
