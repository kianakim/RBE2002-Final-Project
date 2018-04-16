// Ultrasonic State Testing
// Based off of NewPing 15 Sensor Example: https://playground.arduino.cc/Code/NewPing
#include <NewPing.h>

#define SONAR_NUM 3
#define MAX_DISTANCE 200
#define PING_INTERVAL 33 // just example

#define FRONT_RIGHT 0
#define FRONT 1
#define BACK_LEFT 2
#define FRONT_LEFT 3
#define BACK_RIGHT 4

// states
static enum ultraState {forward, frontWall, noWall, checkDir, blockedIn} ultraState;

// sensor object array
NewPing sensors[SONAR_NUM] = {
  NewPing(12, 11, MAX_DISTANCE),  // front right
  NewPing(10, 9, MAX_DISTANCE),  // front
  NewPing(8, 7, MAX_DISTANCE)  // back left
  NewPing(6, 5, MAX_DISTANCE),  // front left
  NewPing(3, 2, MAX_DISTANCE), // back right
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

  ultraState = forward;
}

void loop() {
  // put your main code here, to run repeatedly:
  readAllUltrasonic();
  
//  switch (ultraState) {
//    case forward:
//      // drive forward
//      Serial.println("forward");
//      analogWrite(1, 180); // dummy drive cmd
//      break;
//
//    case frontWall:
//      // stop
//      Serial.println("frontWall");
//      analogWrite(1, 0);
//      break; // could simplify w/ no wall
//
//    case noWall:
//      // stop
//      Serial.println("noWall");
//      analogWrite(1, 0);
//      break;
//
//    case checkDir:
//      // look at left and right sensor to pick direction
//      Serial.println("checkDir");
//      break;
//
//    case blockedIn:
//      // drive backwards or w/e do 180
//      Serial.println("blockedIn");
//      break;
//
//    default:
//      break;
//  }
}

void readAllUltrasonic() {
  for (int i = 0; i < SONAR_NUM; i++) {

    // ready to take reading
    if (millis() >= pingTimer[i]) {
      pingTimer[i] += PING_INTERVAL * SONAR_NUM;

      // end condition check
      if (active_sensor == SONAR_NUM - 1)
        //      printReadings();
        decisionMaking();

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

void decisionMaking() {
  if (dist[FRONT] < 20) {
    Serial.println("front wall");
    ultraState = frontWall;
  }
  else {
    Serial.println("forward");
    ultraState = forward;
  }
  
  if (dist[FRONT_LEFT] > 20) {
    Serial.println("no right wall");
    ultraState = noWall;
  }
  else {
    ultraState = forward;
  }
}
