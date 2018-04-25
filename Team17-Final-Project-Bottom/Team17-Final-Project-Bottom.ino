// RBE2002 FInal Project Code
// Team 17
// Bottom Arduino Version
#include <Wire.h>
#include <L3G.h>
#include <LSM303.h>
#include <Encoder.h>
#include <Servo.h>
#include <LiquidCrystal.h>
#include "PID.h"

L3G gyro;
LSM303 accel;
Servo fanRotate;

const int rs = 18, en = 19, d4 = 17, d5 = 16, d6 = 15, d7 = 14;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

// FRONT IS THE SIDE WITH THE BREADBOARD
int turning_dir = 0;
#define LEFT -1
#define RIGHT 1

// states
int state = 0;
#define DRIVE_FORWARD 0
#define TURNING 1
#define STOP 2
#define DRIVE_CLIFF 3
#define PASS_WALL 4
#define DRIVE_CANDLE_BASE 5
#define FAN_ON 6
#define ROTATE_FAN 7
#define CHECK_FLAME_OUT 8
#define RETURN_COORD 9
#define FINISH 10
#define ESTOP 11

// branch states
int curr_branch = 0;
int branch_step = 0;
#define HUG_WALL_BRANCH 0
#define CLIFF_BRANCH 1
#define FRONT_WALL_BRANCH 2
#define NO_SIDE_WALL_BRANCH 3
#define FLAME_BRANCH 4

// variable used to run code only once
int first = 1;

// motor variables
const byte leftFWDPin = 11; // IN2
const byte leftREVPin = 10;
const byte rightFWDPin = 4; // IN1
const byte rightREVPin = 5;

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

int gyro_SP = 0;
int turn_error = 0;

// pid variables
PID pidTurn;
PID pidForward;

// coordinate variables
double candle_x = 0;
double candle_y = 0;
double candle_z = 0;
double numberTurns = 0;
//double val = 3300;
double distance;
volatile double numberofTurns = 0;

void setup() {
  // set drive signal pins low
  analogWrite(leftFWDPin, 0);
  analogWrite(leftREVPin, 0);
  analogWrite(rightFWDPin, 0);
  analogWrite(rightREVPin, 0);
  delay(1000);

  // fan setup
  fanRotate.attach(12);
  fanRotate.write(0);

  // serial monitor setup
  Serial.begin(9600);
  Serial.println("STARTING SETUP");

  Wire.begin(); // start I2Crobot
  // initialize gyro sensor
  if (!gyro.init()) { // gyro init
    Serial.println("Failed to autodetect gyro type!");
    while (1);
  }

  gyro.enableDefault(); // 250 deg/s
  gyroZero();
  accelInit();
  delay(1000);

  // lcd setup
  lcd.begin(16, 2);

  // pid setup
  pidTurn.setpid(1, 0.0002, 1.2);
  pidForward.setpid(1.1, 0.0002, 1.2);
  timer = millis();
}

void loop() {
  switch (state) {
    case HUG_WALL:
      curr_branch = HUG_WALL_BRANCH;
      branch_step = 0;
      stateManager(curr_branch, branch_step);
      break;

    case DRIVE_FORWARD: // DONE
      gyroForward();
      break;

    case TURNING:
      gyroTurn(LEFT); // dont' know if needs RIGHT
      break;

    case STOP:  // DONE
      driveStop();
      break;

    case DRIVE_CLIFF: //
      while () { // there is a wall,
        gyroForward();
      }
      break;

    case PASS_WALL: // DONE
      if (countsToCM(rightDriveEnc.read()) < 15) {
        gyroForward();
      }
      break;

    case DRIVE_CANDLE_BASE: //
      while () { // ian writing condition
        gyroForward();
      }
      break;

    case FAN_ON: //
      // fan code rishi
      digitalWrite(1, HIGH);
      break;

    case ROTATE_FAN: // DONE
      // go up and down three times
      for (int i = 0; i < 3; i++) {
        fanRotate.write(60);
        delay(3000);
        fanRotate.write(90);
        delay(1000);
      }
      break;

    case CHECK_FLAME_OUT: //
      // read flame pin, check if low/high
      if () {
        stateManager();
      }
      else {
        stateManager();
      }
      break;

    case RETURN_COORD: // DONE
      printToLCD();
      break;

    case ESTOP:
      exit(1);
      break;
  }
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

/* END OF DRIVE METHODS */

/* START OF GYRO METHODS */
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

// drives straight using IMU feedback
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

  // updates gyro
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
    stateManager(curr_branch, TURNING);
  }
}

/* END OF GYRO METHODS */
int state_step = 0;

// set branch in state machine / interrupt
int stateArr[5][6] = {
  {TURNING, DRIVE_FORWARD},                                               // hug wall
  {STOP, TURNING, DRIVE_CLIFF, DRIVE_FORWARD},                            // cliff
  {STOP, TURNING, DRIVE_FORWARD},                                         // front wall
  {DRIVE_WALL, STOP, TURNING, STOP, DRIVE_WALL, DRIVE_FORWARD},           // no side wall
  {STOP, TURNING, DRIVE_CANDLE_BASE, STOP, CHECK_FLAME_OUT, RETURN_COORD} // flame
};

void stateManager(int branch, int curr_state) {
  int hugWallMax = 2;
  int cliffMax = 4;
  int frontWallMax = 3;
  int noSideWallMax = 6;
  int flameMax = 6;
  int arrayMax = 0;

  switch (branch) {
    case HUG_WALL_BRANCH:
      arrayMax = hugWallMax;
      break;
    case CLIFF_BRANCH:
      arrayMax = cliffMax;
      break;
    case FRONT_WALL_BRANCH:
      arrayMax = frontWallMax;
      break;
    case NO_SIDE_WALL_BRANCH:
      arrayMax = noSideWallMax;
      break;
    case FLAME_BRANCH:
      arrayMax = flameMax;
      break;
  }

  state = stateArr[curr_branch][branch_step];
}

// interrupt
void frontWallInt() {
  curr_branch = FRONT_WALL_BRANCH;
  stateManager(curr_branch, 0);
}

// interrupt
void flameInt() {
  curr_branch = FLAME_BRANCH;
  stateManager(curr_branch, 0);
}

void estopInt() {
  state = ESTOP;
}

int countsToCM(int enc_counts) {
  int cm = ((enc_counts) / 1632.67) * 3.14 * 6.985;
  return cm;
}

void printToLCD() {
  lcd.setCursor(0, 0);
  lcd.print("x:");
  lcd.print(candle_x);
  lcd.print("in");
  lcd.setCursor(0, 1); //delete line if using z
  lcd.print("y:");
  lcd.print(candle_y);
  lcd.print("in");
}

// checks if the robot turned right and if the robot turned it adds 1 to turns count
void encoderPos() {
  val = rightDriveEnc.read() ;          // read position
}

void numberTurn() {
  if (turning_dir = LEFT) {
    numberofTurns = numberofTurns + 1;
  }
}

// calculates the distance covered by the robot based on the encoders count
void distanceCovered() {
  distance = (val / 1632.67) * (3.14 * 2.75) ; // inches
  delay(100);

}

void coordinateTrack() {
  if (numberofTurns == 1) {
    candle_y = candle_y + distance;
  }
  else if (numberofTurns == 2) {
    myEncoder.write(0);
    candle_x = candle_x + distance;
    // val = 0;
  }
  else if (numberofTurns == 3) {
    myEncoder.write(0);
    candle_y = candle_y - distance;
    // val = 0;
  }
  else if (numberofTurns == 4) {
    myEncoder.write(0);
    candle_x = candle_x - distance;
    // val = 0;
  }
}

