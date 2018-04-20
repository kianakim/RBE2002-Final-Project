// Turn Drive with IMU
// uses some functions from lab 4 compliment file

#include <Wire.h>
#include <L3G.h>
#include <LSM303.h>
#include <Encoder.h>
L3G gyro;
LSM303 accel;

#define LEFT 0
#define RIGHT 1
#define FORWARD 2

// motor variables
const int leftFWDPin = 13; // IN2
const int leftREVPin = 12;
const int rightFWDPin = 5; // IN1
const int rightREVPin = 4;

// encoder variables
#define COUNTS_PER_DEG 4.535  // 1632.67 counts/rev * 1 rev/360deg
#define DISTIN_TO_DEG 20.834  // 360deg/(2*Pi*2.75" wheel)
#define DISTCM_TO_DEG 8.203   // 360deg/(2*Pi*6.985cm wheel)

Encoder leftDriveEnc(9, 10);
Encoder rightDriveEnc(2, 3);

// gyro variables
double G_Dt = 0.020;  // Integration time (DCM algorithm)
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
#define kp 1.9
/* END OF VARIABLE DECLARATIONS */

void setup() {
  analogWrite(leftFWDPin, 0);
  analogWrite(leftREVPin, 0);
  analogWrite(rightFWDPin, 0);
  analogWrite(rightREVPin, 0);
  Serial.begin(9600);
  Serial.println("STARTING SETUP");
  Wire.begin(); // start I2C

  // initialize gyro
  if (!gyro.init()) { // gyro init
    Serial.println("Failed to autodetect gyro type!");
    while (1);
  }
  timer = millis();
  gyro.enableDefault(); // 250 deg/s
  delay(1000);
  gyroZero();
  accelInit();
}

void loop() {
  //  if ((millis() - timer) >= 20)
  //  {
  //    complementaryFilter();
  //  }
  //  // prints the gyro value once per second
  //  if ((millis() - timer2) >= 1000)
  //  {
  //    printGyro();
  //  }

  delay(1000);
  gyroTurn(LEFT);
  Serial.println("DONE");
  delay(5000);
  driveStop();
  exit(1);
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

// manual drive forward x cm based on encoder readings
void gyroForward(int dist_cm) {
  // gets rotation angle from dist parameters
  double total_deg = dist_cm * DISTCM_TO_DEG;

  // converts degrees to encoder counts
  double total_counts = total_deg * COUNTS_PER_DEG; // might need to be an int???

  // initial IMU and encoders readings
  int dir = FORWARD;
  gyroZero(); // calibration
  complementaryFilter(); // updates gyro values
  leftDriveEnc.write(0);
  double left_pos = leftDriveEnc.read();
  //  double right_pos = rightDriveEnc.read();

  // Proportional Control setup
  double gyro_err = gyro_z; // CHANGE DEPENDING ON IMU ORIENTATION
  double dist_err = left_pos - total_counts;  // CHOOSE W/E SIDE IS MORE RELIABLE
  if (gyro_err > 1) {
    dir = RIGHT;
  }
  else if (gyro_err < -1) {
    dir = LEFT;
  }
  else {
    dir = FORWARD;
  }

  while (dist_err > 5) {

  }
}

// turn robot given direction parameter
// drive using while loop not iterating through state machine
void gyroTurn(int dir) {
  int turn_angle = 90;
  int turn_error = 0;

  // zero gyro
  gyroZero();
  double reading = 0;

  // turning
  while (abs(reading) < turn_angle) {
    // check angle error (for proportional control)
    turn_error = turn_angle - reading;

    // run drive turn method w/ P-controlled speed
    driveMotors(dir, turn_error * kp);

    // update sensor reading
    complementaryFilter();
    reading = gyro_z; // or w/e axis needs to be read
    Serial.println(reading);
  }
}

// set motor controller based on direction and motor speed parameters
void driveMotors(int dir, int motor_speed) {

  // check desired direction, set appropriate motors
  if (dir == RIGHT) {
    analogWrite(leftFWDPin, motor_speed);
    analogWrite(leftREVPin, 0);
    analogWrite(rightFWDPin, 0);
    analogWrite(rightREVPin, motor_speed);
    //    Serial.println("RIGHT");
  }
  else if (dir == FORWARD) {
    analogWrite(leftFWDPin, motor_speed);
    analogWrite(leftREVPin, 0);
    analogWrite(rightFWDPin, motor_speed);
    analogWrite(rightREVPin, 0);
  }
  else {
    analogWrite(leftFWDPin, 0);
    analogWrite(leftREVPin, motor_speed);
    analogWrite(rightFWDPin, motor_speed);
    analogWrite(rightREVPin, 0);
    //    Serial.println("LEFT");
  }
}

// stop motors
void driveStop() {
  analogWrite(leftFWDPin, 0);
  analogWrite(leftREVPin, 0);
  analogWrite(rightFWDPin, 0);
  analogWrite(rightREVPin, 0);
}
