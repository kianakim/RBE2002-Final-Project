// Turn Drive with IMU
// uses some functions from lab 4 compliment file

#include
#include <Wire.h>
#include <L3G.h>
#include <LSM303.h>
L3G gyro;
LSM303 accel;

#define LEFT 0
#define RIGHT 1

// gyro variables
float G_Dt = 0.020;  // Integration time (DCM algorithm)
double const G_gain = .00875; // gyros gain factor for 250deg/sec
double gyro_x;
double gyro_y;
double gyro_z;

double gerr_x;
double gerr_y;
double gerr_z;


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
// sample 200 readings and take the average to get calibration offset
void gyroZero() {
  for (int i = 0; i < 200; i++) {
    gyro.read();

    gerr_x += gyro.g.x;
    gerr_y += gyro.g.y;
    gerr_z += gyro.g.z;

    delay(20);
  }

  gerr_x = gerr_x / 200;
  gerr_y = gerr_y / 200;
  gerr_z = gerr_z / 200;
}

// read gyro
int gyroRead() {
  gyro.read();
  gyro_x = (double) (gyro.g.x - gerr_x) * G_gain;
  gyro_y = (double) (gyro.g.y - gerr_y) * G_gain;
  gyro_z = (double) (gyro.g.z - gerr_z) * G_gain;

  gyro_x = gyro_x * G_Dt; // Multiply the angular rate by the time interval
  gyro_y = gyro_y * G_Dt;
  gyro_z = gyro_z * G_Dt;

  // add stuff
}


// print gyro
// from lab4 file
void printGyro() {
  Serial.print(" GX: ");
  Serial.print(gyro_x);
  Serial.print(" GY: ");
  Serial.print(gyro_y);
  Serial.print(" GZ: ");
  Serial.print(gyro_z);
}

// turn left 90deg w/ gyro
// for future note, might want to turn off? interrupts when turning
// or maybe not because gyro will track position just not for too long
void gyroTurn(int dir) {
  int turn_angle = 90;
  int turn_error = 0;
  // zero gyro
  gyroZero();

  // initial gyro read
  double reading = 0; // w/e put stuff here
  
  // start turning drive motors in direction
  driveTurn(dir);

  // check gyro sensors n stuff
  // ugh but do proper pid
  while(abs(reading) < turn_angle) {
    turn_error = reading - turn_angle;
    driveTurn(dir, );
    reading = 0; // w/e put stuff here
  }
  
}

void driveTurn(int dir, double motor_speed) {
  // check desired direction, set appropriate motors
  if (dir == RIGHT) {
    analogWrite(leftFWDPin, motor_speed);
    analogWrite(leftREVPin, 0);
    analogWrite(rightFWDPin, 0);
    analogWrite(rightREVPin, motor_speed);
  }
  else {
    analogWrite(leftFWDPin, 0);
    analogWrite(leftREVPin, motor_speed);
    analogWrite(rightFWDPin, motor_speed);
    analogWrite(rightREVPin, 0);
  }
}

void driveStop() {
  analogWrite(leftFWDPin, 0);
  analogWrite(leftREVPin, 0);
  analogWrite(rightFWDPin, 0);
  analogWrite(rightREVPin, 0);
}

