// Ultrasonic State Testing
// Based off of NewPing 15 Sensor Example: https://playground.arduino.cc/Code/NewPing
#include <NewPing.h>

#define SONAR_NUM 3
#define MAX_DISTANCE 200
#define PING_INTERVAL 33 // just example

// sensor object array
NewPing sensors[SONAR_NUM] = {
  NewPing(12, 11, MAX_DISTANCE),  // back left
  NewPing(9, 8, MAX_DISTANCE),  // back right
  NewPing(6, 5, MAX_DISTANCE)  // front right
  //  NewPing(0, 1, MAX_DISTANCE),  // front left
  //  NewPing(2, 3, MAX_DISTANCE), // front
};

// sensor variables
unsigned int active_sensor = 0;

// array to store when each sensor pings and distances
unsigned long pingTimer[SONAR_NUM];
double dist[SONAR_NUM];

void setup() {
  Serial.begin(115200);
  pingTimer[0] = millis() + 75; // First ping start in ms.

  // sets ping timers
  for (uint8_t i = 1; i < SONAR_NUM; i++)
    pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;
}

void loop() {
  // put your main code here, to run repeatedly:
  readAllUltrasonic();
}

void readAllUltrasonic() {
  for (int i = 0; i < SONAR_NUM; i++) {

    // ready to take reading
    if (millis() >= pingTimer[i]) {
      pingTimer[i] += PING_INTERVAL * SONAR_NUM;

      // end condition check
      if (active_sensor == SONAR_NUM - 1)
        printReadings();

      // update variables
      sensors[active_sensor].timer_stop();
      active_sensor = i;
      dist[active_sensor] = 0;
      sensors[active_sensor].ping_timer(echoCheck);
    }
  }

  // rest of code
}

// user function: reads distance from active sensor
void echoCheck() {
  if (sensors[active_sensor].check_timer())
    dist[active_sensor] = sensors[active_sensor].ping_result / 58.2; // convert uS to cm
}

// prints readings for testing
void printReadings() {
  for (uint8_t i = 0; i < SONAR_NUM; i++) {
    Serial.print(i);
    Serial.print("=");
    Serial.print(dist[i]);
    Serial.print("cm ");
  }
  Serial.println();
}

// garbage test code
//  active_sensor = 0;
//  sensors[active_sensor].ping_timer(echoCheck);
//  Serial.print("sensor ");
//  Serial.print(active_sensor);
//  Serial.print(": ");
//  Serial.println(sensors[active_sensor].ping_result / 58.2);

