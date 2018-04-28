const byte flameTXPin = 26;
const byte frontTXPin = 24;
const byte sideWallTXPin = 30;
const byte cliffTXPin = 32;

void setup() {
  // put your setup code here, to run once:
  pinMode(flameTXPin, OUTPUT);
  pinMode(frontTXPin, OUTPUT);
  pinMode(sideWallTXPin, OUTPUT);
  pinMode(cliffTXPin, OUTPUT);

  digitalWrite(flameTXPin, LOW);
  digitalWrite(frontTXPin, LOW);
  digitalWrite(sideWallTXPin, LOW);
  digitalWrite(cliffTXPin, LOW);
  Serial.begin(9600);
}

void loop() {
  Serial.println("5s FLAME");
  digitalWrite(flameTXPin, HIGH);
  delay(5000);
  digitalWrite(flameTXPin, LOW);
  delay(1000);
  
  Serial.println("5s FRONT");
  digitalWrite(frontTXPin, HIGH);
  delay(5000);
  digitalWrite(frontTXPin, LOW);
  delay(1000);

  Serial.println("5s SIDE WALL");
  digitalWrite(sideWallTXPin, HIGH);
  delay(5000);
  digitalWrite(sideWallTXPin, LOW);
  delay(1000);
  
  Serial.println("5s CLIFF");
  digitalWrite(cliffTXPin, HIGH);
  delay(5000);
  digitalWrite(cliffTXPin, LOW);
  delay(1000);
  

}
