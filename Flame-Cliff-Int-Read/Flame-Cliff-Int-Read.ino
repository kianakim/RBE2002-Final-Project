// Flame/Cliff sensor interrupt read
// Bottom Arduino board code

const int flameInt = 2;
const int cliffInt = 3;

void setup() {
  pinMode(flameInt, INPUT_PULLUP);
  pinMode(cliffInt, INPUT_PULLUP);
  attachInterrupt(flameInt, seeFlame, RISING);
  attachInterrupt(cliffInt, seeCliff, RISING);
}

void loop() {

}

void seeFlame() {
  
}

void seeCliff() {
  // stop motors

  // check l/r or just turn

  // wall follow
}

