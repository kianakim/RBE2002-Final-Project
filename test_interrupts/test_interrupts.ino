int rstate = 0;

const byte flameRXPin = 2;
const byte frontRXPin = 18;
const byte sideWallRXPin = 19;
const byte cliffRXPin = 3;

void setup() {
  Serial.begin(9600);
  Serial.println("STARTING SETUP");

  pinMode(flameRXPin, INPUT);
  pinMode(frontRXPin, INPUT);
  pinMode(sideWallRXPin, INPUT);
  pinMode(cliffRXPin, INPUT);
  
  attachInterrupt(digitalPinToInterrupt(flameRXPin), flameInt, RISING);
  attachInterrupt(digitalPinToInterrupt(frontRXPin), frontWallInt, RISING);
  attachInterrupt(digitalPinToInterrupt(sideWallRXPin), noSideWallInt, RISING);
  attachInterrupt(digitalPinToInterrupt(cliffRXPin), cliffInt, RISING);
}

void loop() {
  switch (rstate) {
    case 0:
      Serial.println("INITIAL");
      break;

    case 1:
      Serial.println("FLAME");
      break;

    case 2:
      Serial.println("FRONT WALL");
      break;

    case 3:
      Serial.println("NO SIDE WALL");
      break;

    case 4:
      Serial.println("CLIFF");
      break;
  }
  delay(100);
}

void flameInt() {
  rstate = 1;
}

void frontWallInt() {
  rstate = 2;
}

void noSideWallInt() {
  rstate = 3;
}

void cliffInt() {
  rstate = 4;
}

