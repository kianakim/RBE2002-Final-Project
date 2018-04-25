// RBE2002 FInal Project Code
// Team 17
// Top Arduino Version

#include <Wire.h>
#include <NewPing.h>

// Ultrasonic Sensors
#define SONAR_NUM 5
#define MAX_DISTANCE 200
#define PING_INTERVAL 33

#define FRONT 0
#define BACK_RIGHT 1
#define FRONT_RIGHT 2
#define BACK_LEFT 3
#define FRONT_LEFT 4

NewPing sensors[SONAR_NUM] = {
  NewPing(6, 7, MAX_DISTANCE),  // front
  NewPing(4, 5, MAX_DISTANCE),  // back right
  NewPing(8, 9, MAX_DISTANCE),  // front right
  NewPing(2, 3, MAX_DISTANCE), // back left
  NewPing(11, 12, MAX_DISTANCE) // front left
};

unsigned int active_sensor = 0;
unsigned long pingTimer[SONAR_NUM];
double dist_cm[SONAR_NUM];

// Cliff Sensor
const byte cliffPin = 1; // dummy
const byte cliffSignal = 2; // dummy

// Flame Sensor
const byte flamePin = 0; // dummy
const byte flameSignal = 3; // dummy

void setup() {
  Serial.begin(115200);

  // pin initialization
  pinMode(cliffSignal, OUTPUT);
  pinMode(flameSignal, OUTPUT);
  pinMode(FRONT, OUTPUT);
  pinMode(FRONT_RIGHT, OUTPUT);
  pinMode(BACK_RIGHT, OUTPUT);
  pinMode(FRONT_LEFT, OUTPUT);
  pinMode(BACK_LEFT, OUTPUT);

  // set pins low
  digitalWrite(cliffSignal, LOW);
  digitalWrite(flameSignal, LOW);
  digitalWrite(FRONT, LOW);
  digitalWrite(FRONT_RIGHT, LOW);
  digitalWrite(BACK_RIGHT, LOW);
  digitalWrite(FRONT_LEFT, LOW);
  digitalWrite(BACK_LEFT, LOW);

  pingTimer[0] = millis() + 75; // First ping start in ms.

  // sets ping timers
  for (uint8_t i = 1; i < SONAR_NUM; i++)
    pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;
}

void loop() {

}

/* ULTRASONIC SENSOR METHODS */

// return whether the sensor sees a wall or not
boolean ultrasonicState(int sensor) {
  int dist = 10;

  if (dist_cm[sensor] > dist) { // dummy number
    return true;
  }
  else {
    return false;
  }
}

// reads ultrasonic sensor array, prints readings, updates ultrasonic
// signal transfer pin (to other Arduino)
void readAllUltrasonic() {
  for (int i = 0; i < SONAR_NUM; i++) {

    // ready to take reading
    if (millis() >= pingTimer[i]) {
      pingTimer[i] += PING_INTERVAL * SONAR_NUM;

      // end condition check
      if (active_sensor == SONAR_NUM - 1) {
        printUltrasonicReadings();

        // update ultrasonic signal transfer pins
        for (int i = 0; i < SONAR_NUM; i++)
          digitalWrite(i, ultrasonicState(i));
      }

      // update variables
      sensors[active_sensor].timer_stop();
      active_sensor = i;
      dist_cm[active_sensor] = 0;
      sensors[active_sensor].ping_timer(echoCheck);
    }
  }
}

// prints readings for testing
void printUltrasonicReadings() {
  for (uint8_t i = 0; i < SONAR_NUM; i++) {
    Serial.print(i);
    Serial.print("=");
    Serial.print(dist_cm[i]);
    Serial.print("cm ");
  }
  Serial.println();
}

// user function: reads distance from active sensor
void echoCheck() {
  if (sensors[active_sensor].check_timer())
    dist_cm[active_sensor] = sensors[active_sensor].ping_result / 58.2; // convert uS to cm
}

/* SIGNAL TRANSFER METHODS */

// these are bad names pls change
void seeCliff() {
  int distance;

  // read rangefinder  GP2D12 = read_gp2d12_range(cliffPin);
  a = GP2D12 / 10;
  b = GP2D12 % 10;
  distance = a * 10 + b;
  Serial.println(GP2D12);

  // might need to change to go HIGH-LOW depending on interrupt mode
  if (distance > 50) {
    digitalWrite(cliffSignal, HIGH);
  }
  else {
    digitalWrite(cliffSignal, LOW);
  }
}

// rangefinder read method
float read_gp2d12_range(byte pin)
{
  int tmp;
  tmp = analogRead(pin);
  if (tmp < 3)return -1;
  return (6787.0 / ((float)tmp - 3.0)) - 4.0;
}

void seeFlame() {
  // read flame sensor
  int sensorReading = analogRead(flamePin);

  if (sensorReading < 900 && sensorReading > 600) { //Distant Fire
    //turn right and drive 20 inches
    //turn right
  }

  else if (sensorReading < 600 && sensorReading > 300) { //close fire
    //turn right and drive 10 inches
    //turn right
  }

  else if (sensorReading < 300) { //point blank fire
    digitalWrite(flameSignal, HIGH);
  }
}

//boolean isFrontWall() {
//  if (ultrasonicState(FRONT)) {
//    digitalWrite(13, HIGH);
//    return true;
//  }
//  else {
//    digitalWrite(13, LOW);
//    return false;
//  }
//}
//
boolean isSideWall() {
  if (ultrasonicState(FRONT_RIGHT) && ultrasonicState(BACK_RIGHT)) {
    digitalWrite(12, HIGH);
    return true;
  }
  else {
    digitalWrite(12, LOW);
    return false;
  }
}

