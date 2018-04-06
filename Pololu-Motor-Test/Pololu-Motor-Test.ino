// Motor Test
// kind of shitty way to do the one vs two motor test but w/e
#include <Servo.h>
#include <Encoder.h>

#define ONE_MOTOR_TEST 0
#define TWO_MOTOR_TEST 1

// Motor Pins ORANGE - IN1, BRWN - IN2
const int leftFWDPin = 5; // IN2
const int leftREVPin = 6;
const int rightFWDPin = 4; // IN1
const int rightREVPin = 3; 

// PATTERN LOOKING AT IT W/ DIGITAL PINS ON FAR SIDE: O - B - O - B

// Encoders
Encoder leftDrive(3, 1); // YEL - INT3, WH - D1
Encoder rightDrive(2, 0); // YEL - INT2, WH - D0


// Test Variables
int test_state = 0;
int motor_speed = 50;
double left_pos = 0;
double right_pos = 0;

void setup() {
  Serial.begin(115200);

  // set all motor signal pins to LOW
  analogWrite(leftFWDPin, 0);
  analogWrite(leftREVPin, 0);
  analogWrite(rightFWDPin, 0);
  analogWrite(rightREVPin, 0);    

  // set test state
  test_state = TWO_MOTOR_TEST;
}

void loop() {
  switch (test_state) {
    case ONE_MOTOR_TEST:
      // get current position
      left_pos = leftDrive.read();

      // set motor speed
      analogWrite(rightFWDPin, motor_speed);
      analogWrite(rightREVPin, 0);
      
      // print output
      Serial.print("Enc: ");
      Serial.println(left_pos);
      break;

    case TWO_MOTOR_TEST:
      // get current position
      left_pos = leftDrive.read();
      right_pos = rightDrive.read();

      // set motor speed
      analogWrite(leftFWDPin, motor_speed);
      analogWrite(leftREVPin, 0);
      analogWrite(rightFWDPin, motor_speed);
      analogWrite(rightREVPin, 0);
      break;
  }
}
