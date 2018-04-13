// state machine stuff

static enum navStates {hugWall, turning, drivePastWall, stopRobot, finished} navStates;

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:
  switch (navState) {
    case hugWall:
      // turn left (or w/e wall following dir)
      // drive forward til n inches away
      // turn right
      break;
    case turning:
      // turning
      break;
    
    case drivePastWall:
      // 
      break;
    case stopRobot:
      break;
    case finished:
      break;
  }
}
