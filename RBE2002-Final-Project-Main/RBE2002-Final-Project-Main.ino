// Team 17 
// RBE 2002 Main Final Project Code

// Drive Motor Pins (MC33926 Pololu Driver)
const int rightDriveFWDPin = 8;
const int rightDriveREVPin = 9;
const int leftDriveFWDPin = 7;
const int leftDriveREVPin = 6;

// Sensor Pins
const int flameSensorPin = 0;

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
