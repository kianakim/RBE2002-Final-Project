// Turn Drive with IMU
// uses some functions from lab 4 compliment file

#include <Wire.h>
#include <L3G.h>
#include <LSM303.h>
#include <Encoder.h>
#include "PID.h"

L3G gyro;
LSM303 accel;

// FRONT IS THE SIDE WITH THE BREADBOARD
#define LEFT -1
#define RIGHT 1

unsigned int state = 0;
int gyro_SP = 0;
int turn_error = 0;
int first = 1;

boolean front_sensor = false;

// motor variables
<<<<<<< HEAD
const int leftFWDPin = 10; // IN2
const int leftREVPin = 11;
=======
const int leftFWDPin = 11; // IN2
const int leftREVPin = 10;
>>>>>>> 04eee2e38e7cbf21e0c38b6460a2fb193b0a53e4
const int rightFWDPin = 4; // IN1
const int rightREVPin = 5;

// encoder variables
#define COUNTS_PER_DEG 4.535  // 1632.67 counts/rev * 1 rev/360deg
#define TESTING 587716
#define DISTIN_TO_DEG 20.834  // 360deg/(2*Pi*2.75" wheel)
#define DISTCM_TO_DEG 8.203   // 360deg/(2*Pi*6.985cm wheel)

Encoder leftDriveEnc(10, 11);
Encoder rightDriveEnc(2, 3);

// gyro variables
double G_Dt = 0.035;  // Integration time (DCM algorithm)
double const G_gain = 0.00875; // gyros gain factor for 250deg/sec
double gyro_x; // gyro x value, used by gyroRead and complementaryFilter
double gyro_y;
double gyro_z;
double gyro_xold; //gyro cummulative x value
double gyro_yold;
double gyro_zold;
double gerrx; // gyro x error
double gerry;
double gerrz;

// accel variables
double A_gain = 0.00875;
double accel_x;
double accel_y;
double accel_z;

long timer = 0; //general purpose timer
long timer1 = 0;
long timer2 = 0;

// pid variables
PID pidTurn;
PID pidForward;
/* END OF VARIABLE DECLARATIONS */

/* SETUP AND LOOP */

void setup() {
  // set drive signal pins low
  analogWrite(leftFWDPin, 0);
  analogWrite(leftREVPin, 0);
  analogWrite(rightFWDPin, 0);
  analogWrite(rightREVPin, 0);
  delay(1000);

  // serial monitor setup
  Serial.begin(9600);
  Serial.println("STARTING SETUP");

  Wire.begin(); // start I2C

  // initialize gyro sensor
  if (!gyro.init()) { // gyro init
    Serial.println("Failed to autodetect gyro type!");
    while (1);
  }
  gyro.enableDefault(); // 250 deg/s
  gyroZero();
  accelInit();
  delay(1000);
  pidTurn.setpid(1, 0.0002, 1.2);
  pidForward.setpid(1.1, 0.0002, 1.2);
  timer = millis();
}

void loop() {
  //  Serial.println(state);
  switch (state) {
    case 0:
//            gyroTurn(RIGHT);
      gyroForward();
      break;
    case 1:
      driveStop();
      state = 2;
      delay(3000);
      break;
    case 2:
      gyroTurn(LEFT);
      break;
    case 3:
<<<<<<< HEAD
      gyroForward();
      break;
    case 4:
      Serial.println(gyro_z);
      driveStop();
      delay(1000);
     
=======
      driveStop();
      delay(3000);
      state = 4;
      break;
    case 4:
      if (!front_sensor) {
        gyroForward();
      }
      else {
        driveStop();
        delay(3000);
        state = 5;
      }
>>>>>>> 04eee2e38e7cbf21e0c38b6460a2fb193b0a53e4
  }

  //  if ((millis() - timer) >= 20)
  //  {
  //    complementaryFilter();
  //  }
  //  // prints the gyro value once per second
  //  if ((millis() - timer2) >= 1000)
  //  {
  //    printGyro();
  //  }

  //  delay(1000);
  //  gyroTurn();
  //  Serial.println("DONE");
  //  driveStop();
  //  delay(5000);
  //  exit(1);
}

/* START OF IMU METHODS */

// gyro calibration "zero"
// sample 200 readings and take the average to get calibration offset
void gyroZero() {
  for (int i = 0; i < 200; i++) {
    gyro.read();
    gerrx += gyro.g.x;
    gerry += gyro.g.y;
    gerrz += gyro.g.z;
    delay(20);
  }

  gerrx = gerrx / 200; // average reading to obtain an error/offset
  gerry = gerry / 200;
  gerrz = gerrz / 200;

  Serial.println(gerrx); // print error vals
  Serial.println(gerry);
  Serial.println(gerrz);
}

// add actual description pls kiana
void gyroRead() {
  gyro.read();
  timer = millis(); //reset timer

  gyro_x = (double)(gyro.g.x - gerrx) * G_gain; // offset by error then multiply by gyro gain factor
  gyro_y = (double)(gyro.g.y - gerry) * G_gain;
  gyro_z = (double)(gyro.g.z - gerrz) * G_gain;

  gyro_x = gyro_x * G_Dt; // Multiply the angular rate by the time interval
  gyro_y = gyro_y * G_Dt;
  gyro_z = gyro_z * G_Dt;

  gyro_x += gyro_xold; // add the displacment(rotation) to the cumulative displacment
  gyro_y += gyro_yold;
  gyro_z += gyro_zold;

  gyro_xold = gyro_x; // Set the old gyro angle to the current gyro angle
  gyro_yold = gyro_y;
  gyro_zold = gyro_z;
}

void gyroReset() {
  gyro_x = 0; // gyro x value, used by gyroRead and complementaryFilter
  gyro_y = 0;
  gyro_z = 0;
  gyro_xold = 0;
  gyro_yold = 0;
  gyro_zold = 0;
  gerrx = 0;
  gerry = 0;
  gerrz = 0;
}

// print gyro
// from lab4 file
void printGyro() {
  timer2 = millis();

  Serial.print(" GX: ");
  Serial.print(gyro_x);
  Serial.print(" GY: ");
  Serial.print(gyro_y);
  Serial.print(" GZ: ");
  Serial.print(gyro_z);

  Serial.print("  Ax =  ");
  Serial.print(accel_x);
  Serial.print("  Ay =  ");
  Serial.print(accel_y);
  Serial.print("  Az =  ");
  Serial.println(accel_z);
}

// initialize accelerometer - lab4 compliment file
// checks for device type, don't really need to modify
void accelInit()
{
  accel.init();
  accel.enableDefault();
  Serial.print("Accel Device ID");
  Serial.println(accel.getDeviceType());
  switch (accel.getDeviceType())
  {
    case LSM303::device_D:
      accel.writeReg(LSM303::CTRL2, 0x18); // 8 g full scale: AFS = 011
      break;
    case LSM303::device_DLHC:
      accel.writeReg(LSM303::CTRL_REG4_A, 0x28); // 8 g full scale: FS = 10; high resolution output mode
      break;
    default: // DLM, DLH
      accel.writeReg(LSM303::CTRL_REG4_A, 0x30); // 8 g full scale: FS = 11
  }
}

// Reads x,y and z accelerometer registers
void accelRead()
{
  accel.readAcc();

  accel_x = accel.a.x >> 4; // shift left 4 bits to use 12-bit representation (1 g = 256)
  accel_y = accel.a.y >> 4;
  accel_z = accel.a.z >> 4;

  // accelerations in G
  accel_x = (accel_x / 256);
  accel_y = (accel_y / 256);
  accel_z = (accel_z / 256);
}

// implements complementary filter w/ gyro and accel data
// updates gyro val variables
void complementaryFilter() {
  gyroRead();
  accelRead();
  double x_Acc, y_Acc;
  double magnitudeofAccel = (abs(accel_x) + abs(accel_y) + abs(accel_z));
  if (magnitudeofAccel > 6 && magnitudeofAccel < 1.2)
  {
    x_Acc = atan2(accel_y, accel_z) * 180 / PI;
    gyro_x = gyro_x * 0.98 + x_Acc * 0.02;

    y_Acc = atan2(accel_x, accel_z) * 180 / PI;
    gyro_y = gyro_y * 0.98 + y_Acc * 0.02;
  }
}

/* END OF IMU METHODS */

/* START OF DRIVE METHODS */

// untested gyroForward()
void gyroForward() {
  int rmotor_speed, lmotor_speed;
  int motor_const = 5; // to compensate for slower motor
  int motor_speed = 90;
  turn_error = gyro_SP - gyro_z;
  int motor_diff = pidForward.calc(gyro_SP, gyro_z);
  Serial.print("TURN ERR: ");
  Serial.println(turn_error);

  if (turn_error > 0) { // robot is facing right
    rmotor_speed = constrain(motor_speed + motor_diff + motor_const, 0, 254);
    lmotor_speed = constrain(motor_speed - motor_diff, 0, 254);
    driveStraight(rmotor_speed, lmotor_speed);
  }
  else {
    rmotor_speed = constrain(motor_speed - motor_diff - motor_const, 0, 254);
    lmotor_speed = constrain(motor_speed + motor_diff, 0, 254);
    driveStraight(rmotor_speed, lmotor_speed);
  }

  Serial.print("RSPEED: ");
  Serial.print(rmotor_speed);
  Serial.print(" LSPEED: ");
  Serial.println(lmotor_speed);

  complementaryFilter();

}

// turns robot 90 deg in given direction
// not enough power when angle error < like 3 deg
void gyroTurn(int dir) {
  // change setpoint on initial run
  if (first) {
    changeSP(dir);
    first = 0;
  }
  turn_error = gyro_SP - gyro_z;

  Serial.print("SP: ");
  Serial.println(gyro_SP);
  Serial.print("GYRO: ");
  Serial.println(gyro_z);

  int mspeed = pidTurn.calc(gyro_SP, gyro_z);

  if (abs(turn_error) > 1) {
    Serial.print("PID: ");
    Serial.println(mspeed);
    // run drive turn method w/ P-controlled speed
    driveTurn(setTurnDir(turn_error), mspeed);

    // update sensor reading
    complementaryFilter();
  }
  else {
    driveStop();
    first = 1;
    state++;
  }
}

// set drive motor speed and direction
void driveTurn(int dir, int motor_speed) {

  // constrain motor speed value
  int mspeed = constrain(motor_speed, 0, 255);

  // check desired direction, set appropriate motors
  if (dir == RIGHT) {
    analogWrite(leftFWDPin, mspeed);
    analogWrite(leftREVPin, 0);
    analogWrite(rightFWDPin, 0);
    analogWrite(rightREVPin, mspeed);
  }
  else if (dir == LEFT) {
    analogWrite(leftFWDPin, 0);
    analogWrite(leftREVPin, mspeed);
    analogWrite(rightFWDPin, mspeed);
    analogWrite(rightREVPin, 0);
  }
}

// used w/ gyro to drive along a straight path
void driveStraight(int rspeed, int lspeed) {
  analogWrite(leftFWDPin, lspeed);
  analogWrite(leftREVPin, 0);
  analogWrite(rightFWDPin, rspeed);
  analogWrite(rightREVPin, 0);
}

// stop motors
void driveStop() {
  analogWrite(leftFWDPin, 0);
  analogWrite(leftREVPin, 0);
  analogWrite(rightFWDPin, 0);
  analogWrite(rightREVPin, 0);
}

// set turn direction based on sign of error
int setTurnDir(double error) {
  if (error < 0) {
    return LEFT;
  }
  else {
    return RIGHT;
  }
}

void changeSP(int dir) {
  if (dir == LEFT) {
    gyro_SP -= 90;
  }
  else if (dir == RIGHT) {
    gyro_SP += 90;
  }
}

// LEFT ENCODER RETURNING WEIRD VALUES
// too much slack on wheels to use
//void encTurn(int dir) {
//  int total_counts = 2900; // 90 deg
//  int rightRead = rightDriveEnc.read();
//  turn_error = total_counts - abs(rightRead);
//  Serial.print("LEFT: ");
//  Serial.println(rightRead);
//
//  if (turn_error > 5) {
//    driveTurn(dir, 100);
//  }
//  else {
//    driveStop();
//    state++;
//  }
//}
