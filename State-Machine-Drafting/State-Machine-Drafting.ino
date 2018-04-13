// state machine stuff

static enum navStates {hugWall, driveForward, turning, drivePastWall, stopRobot, finished} navStates;

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:
  switch (navState) {
    case hugWall: // could just be initial state
      // turn left (or w/e wall following dir)

      // drive forward til n inches away

      // turn right

      // state changer:
      // just break after completing turn
      break;

    case checkUltrasonic:
      // check sensor readings
      
      // change state based on readings
      break;
    case noWall:
      // turn

      // check front sensor > length of robot
      
      // forward til both sensors are blocked and unblocked (passed wall)

      // if ^ then turn
      
      // or if blocked fully,

      // set to driveFoward state
      
      break;
    case driveForward:
      // set drive motors forward
    
      // state changer: checkUltrasonic
      break;
    case turning:
      // turning left/right
      // update direction array
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
