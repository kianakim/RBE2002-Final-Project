#include<Servo.h>

Servo myservo;

void setup() {
  Serial.begin(9600);
  myservo.attach(12);
  myservo.write(0);
  digitalWrite(12, LOW);
}

void loop() {
  myservo.write(60);
  delay(3000);
  myservo.write(90);
  delay(1000);
}
