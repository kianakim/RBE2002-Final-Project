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

boolean ultrasonicState(int sensor) {

  if (dist_cm[sensor] > 50) { // dummy number
    return true;
  }
  else {
    return false;
  }
}

void readAllUltrasonic() {
  for (int i = 0; i < SONAR_NUM; i++) {

    // ready to take reading
    if (millis() >= pingTimer[i]) {
      pingTimer[i] += PING_INTERVAL * SONAR_NUM;

      // end condition check
      if (active_sensor == SONAR_NUM - 1) {
        printReadings();

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
void printReadings() {
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

