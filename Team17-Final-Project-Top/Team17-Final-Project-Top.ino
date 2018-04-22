// RBE2002 FInal Project Code
// Team 17
// Top Arduino Version

#include <Wire.h>
#include <NewPing.h>

// Ultrasonic Sensors
#define SONAR_NUM 3
#define MAX_DISTANCE 200
#define PING_INTERVAL 33

#define FRONT 0
#define BACK_LEFT 1
#define FRONT_LEFT 2

NewPing sensors[SONAR_NUM] = {
  NewPing(10, 9, MAX_DISTANCE),  // front
  NewPing(8, 7, MAX_DISTANCE),  // back left
  NewPing(6, 5, MAX_DISTANCE)  // front left
};

unsigned int active_sensor = 0;
unsigned long pingTimer[SONAR_NUM];
double dist_cm[SONAR_NUM];

// Cliff Sensor
const byte cliffPin = 1;

// Flame Sensor (analog)
const byte flamePin = 0;

void setup() {
  
}

void loop() {

}
